/*
 * FPC1020 Fingerprint sensor device driver
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>:
 *	Aleksej Makarov
 * 	Henrik Tillman <henrik.tillman@fingerprints.com>
 *
 * Copyright (C) 2017, Sultanxda <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This driver will control the platform resouretes that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 */

#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>

#define FPC1020_RESET_LOW_US	(1000)
#define FPC1020_RESET_HIGH1_US	(100)
#define FPC1020_RESET_HIGH2_US	(1250)
#define FPC_TTW_HOLD_TIME_MS	(1000)

struct fpc1020_data {
	struct device *dev;
	struct input_dev *input_dev;
	struct notifier_block fb_notif;
	struct completion irq_sent;
	struct work_struct pm_work;
	struct wakeup_source ttw_ws;
	spinlock_t irq_lock;

	bool irq_disabled;
	bool proximity_state; /* 0:far 1:near */
	bool screen_off;

	int irq_gpio;
	int rst_gpio;
	int vdd_en_gpio;
};

extern bool s3320_touch_active(void);
extern bool virtual_key_enable;

static void hw_reset(struct fpc1020_data *f)
{
	int i;

	gpio_set_value(f->vdd_en_gpio, 0);
	msleep(3);

	gpio_set_value(f->vdd_en_gpio, 1);
	msleep(3);

	for (i = 0; i < 2; i++) {
		gpio_set_value(f->rst_gpio, 1);
		usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US);

		gpio_set_value(f->rst_gpio, 0);
		usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US);

		gpio_set_value(f->rst_gpio, 1);
		usleep_range(FPC1020_RESET_HIGH2_US, FPC1020_RESET_HIGH2_US);

		if (gpio_get_value(f->irq_gpio))
			break;

		usleep_range(1250, 1250);
	}
}

static void set_fpc_irq(struct fpc1020_data *f, bool enable)
{
	bool irq_disabled;

	spin_lock(&f->irq_lock);
	irq_disabled = f->irq_disabled;
	f->irq_disabled = !enable;
	spin_unlock(&f->irq_lock);

	if (enable == !irq_disabled)
		return;

	if (enable)
		enable_irq(gpio_to_irq(f->irq_gpio));
	else
		disable_irq(gpio_to_irq(f->irq_gpio));
}

static int fpc1020_input_init(struct fpc1020_data *f)
{
	int ret;

	f->input_dev = input_allocate_device();
	if (!f->input_dev) {
		dev_err(f->dev, "Input_allocate_device failed.\n");
		return -ENOMEM;
	}

	f->input_dev->name = "fpc1020";

	set_bit(EV_KEY, f->input_dev->evbit);
	set_bit(KEY_HOME, f->input_dev->keybit);

	ret = input_register_device(f->input_dev);
	if (ret) {
		dev_err(f->dev, "Input_register_device failed.\n");
		input_free_device(f->input_dev);
	}

	return ret;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *f,
	const char *label, int *gpio)
{
	struct device *dev = f->dev;
	struct device_node *np = dev->of_node;
	int ret;

	ret = of_get_named_gpio(np, label, 0);
	if (ret < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return ret;
	}

	*gpio = ret;

	ret = devm_gpio_request(dev, *gpio, label);
	if (ret) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return ret;
	}

	return 0;
}

/*
 * sysfs node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fpc1020_data *f = dev_get_drvdata(dev);
	bool irq_disabled;
	int irq;
	ssize_t count;

	spin_lock(&f->irq_lock);
	irq_disabled = f->irq_disabled;
	spin_unlock(&f->irq_lock);

	irq = !irq_disabled && gpio_get_value(f->irq_gpio);
	count = scnprintf(buf, PAGE_SIZE, "%d\n", irq);

	complete(&f->irq_sent);

	return count;
}

static ssize_t screen_state_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fpc1020_data *f = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", !f->screen_off);
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *f = dev_get_drvdata(dev);

	if (!memcmp(buf, "reset", sizeof("reset")))
		hw_reset(f);

	return count;
}

static ssize_t proximity_state_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *f = dev_get_drvdata(dev);
	int ret, val;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return -EINVAL;

	f->proximity_state = !!val;

	if (f->screen_off)
		set_fpc_irq(f, !f->proximity_state);

	return count;
}

static ssize_t report_home_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *f = dev_get_drvdata(dev);

	if (!memcmp(buf, "down", sizeof("down"))) {
		if (!s3320_touch_active() && !virtual_key_enable) {
			input_report_key(f->input_dev, KEY_HOME, 1);
			input_sync(f->input_dev);
		}
	} else if (!memcmp(buf, "up", sizeof("up"))) {
		input_report_key(f->input_dev, KEY_HOME, 0);
		input_sync(f->input_dev);
	} else {
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, NULL);
static DEVICE_ATTR(proximity_state, S_IWUSR, NULL, proximity_state_set);
static DEVICE_ATTR(report_home, S_IWUSR, NULL, report_home_set);
static DEVICE_ATTR(screen_state, S_IRUSR, screen_state_get, NULL);

static struct attribute *attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_irq.attr,
	&dev_attr_proximity_state.attr,
	&dev_attr_report_home.attr,
	&dev_attr_screen_state.attr,
	NULL
};

static const struct attribute_group fpc1020_attr_group = {
	.attrs = attributes,
};

static void set_fingerprintd_nice(int nice)
{
	struct task_struct *p;

	read_lock(&tasklist_lock);
	for_each_process(p) {
		if (!memcmp(p->comm, "fingerprintd", sizeof("fingerprintd"))) {
			set_user_nice(p, nice);
			break;
		}
	}
	read_unlock(&tasklist_lock);
}

static void fpc1020_suspend_resume(struct work_struct *work)
{
	struct fpc1020_data *f = container_of(work, typeof(*f), pm_work);

	/* Escalate fingerprintd priority when screen is off */
	if (f->screen_off) {
		set_fingerprintd_nice(MIN_NICE);
	} else {
		set_fpc_irq(f, true);
		set_fingerprintd_nice(0);
	}

	sysfs_notify(&f->dev->kobj, NULL, dev_attr_screen_state.attr.name);
}

static int fb_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct fpc1020_data *f = container_of(nb, typeof(*f), fb_notif);
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	if (action != FB_EARLY_EVENT_BLANK)
		return 0;

	if (*blank == FB_BLANK_UNBLANK) {
		cancel_work_sync(&f->pm_work);
		f->screen_off = false;
		queue_work(system_highpri_wq, &f->pm_work);
	} else if (*blank == FB_BLANK_POWERDOWN) {
		cancel_work_sync(&f->pm_work);
		f->screen_off = true;
		queue_work(system_highpri_wq, &f->pm_work);
	}

	return 0;
}

static irqreturn_t fpc1020_irq_handler(int irq, void *dev_id)
{
	struct fpc1020_data *f = dev_id;

	sysfs_notify(&f->dev->kobj, NULL, dev_attr_irq.attr.name);

	reinit_completion(&f->irq_sent);
	wait_for_completion_timeout(&f->irq_sent, msecs_to_jiffies(100));

	if (f->screen_off)
		__pm_wakeup_event(&f->ttw_ws, FPC_TTW_HOLD_TIME_MS);

	return IRQ_HANDLED;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct fpc1020_data *f;
	int id_gpio, ret;

	if (!np) {
		dev_err(dev, "no of node found\n");
		return -EINVAL;
	}

	f = devm_kzalloc(dev, sizeof(*f), GFP_KERNEL);
	if (!f) {
		dev_err(dev, "devm_kzalloc failed for struct fpc1020_data\n");
		return -ENOMEM;
	}

	f->dev = dev;
	dev_set_drvdata(dev, f);

	ret = fpc1020_request_named_gpio(f, "fpc,irq-gpio", &f->irq_gpio);
	if (ret)
		goto err1;

	ret = fpc1020_request_named_gpio(f, "fpc,reset-gpio", &f->rst_gpio);
	if (ret)
		goto err1;

	ret = fpc1020_request_named_gpio(f, "fpc,gpio_1V8_EN", &f->vdd_en_gpio);
	if (ret)
		goto err1;

	ret = fpc1020_input_init(f);
	if (ret)
		goto err1;

	spin_lock_init(&f->irq_lock);
	INIT_WORK(&f->pm_work, fpc1020_suspend_resume);
	init_completion(&f->irq_sent);
	wakeup_source_init(&f->ttw_ws, "fpc_ttw_ws");

	ret = sysfs_create_group(&dev->kobj, &fpc1020_attr_group);
	if (ret) {
		dev_err(dev, "Could not create sysfs, ret: %d\n", ret);
		goto err2;
	}

	ret = devm_request_threaded_irq(dev, gpio_to_irq(f->irq_gpio),
			NULL, fpc1020_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			dev_name(dev), f);
	if (ret) {
		dev_err(dev, "Could not request irq, ret: %d\n", ret);
		goto err3;
	}

	f->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&f->fb_notif);
	if (ret) {
		dev_err(dev, "Unable to register fb_notifier, ret: %d\n", ret);
		goto err4;
	}

	ret = fpc1020_request_named_gpio(f, "fpc,gpio_id0", &id_gpio);
	if (!ret && gpio_is_valid(id_gpio))
		gpio_direction_input(id_gpio);

	ret = fpc1020_request_named_gpio(f, "fpc,gpio_id1", &id_gpio);
	if (!ret && gpio_is_valid(id_gpio))
		gpio_direction_input(id_gpio);

	gpio_direction_input(f->irq_gpio);
	gpio_direction_output(f->vdd_en_gpio, 1);

	return 0;
err4:
	devm_free_irq(dev, gpio_to_irq(f->irq_gpio), f);
err3:
	sysfs_remove_group(&dev->kobj, &fpc1020_attr_group);
err2:
	input_unregister_device(f->input_dev);
	input_free_device(f->input_dev);
err1:
	devm_kfree(dev, f);
	return ret;
}

static int fpc1020_sys_suspend(struct device *dev)
{
	struct fpc1020_data *f = dev_get_drvdata(dev);

	enable_irq_wake(gpio_to_irq(f->irq_gpio));
	return 0;
}

static int fpc1020_sys_resume(struct device *dev)
{
	struct fpc1020_data *f = dev_get_drvdata(dev);

	disable_irq_wake(gpio_to_irq(f->irq_gpio));
	return 0;
}

static const struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{ }
};

static const struct dev_pm_ops fpc1020_pm_ops = {
	.suspend = fpc1020_sys_suspend,
	.resume = fpc1020_sys_resume,
};

static struct platform_driver fpc1020_driver = {
	.probe = fpc1020_probe,
	.driver = {
		.name = "fpc1020",
		.owner = THIS_MODULE,
		.of_match_table = fpc1020_of_match,
		.pm = &fpc1020_pm_ops,
	},
};

static int __init fpc1020_init(void)
{
	return platform_driver_register(&fpc1020_driver);
}
device_initcall(fpc1020_init);
