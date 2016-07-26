/*
 * Cluster-plug CPU Hotplug Driver
 * Designed for homogeneous ARM big.LITTLE systems
 *
 * Copyright (C) 2015-2016 Sultan Qasim Khan and Christopher R. Palmer
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/cpufreq.h>
#include <linux/fb.h>
#include <linux/input.h>

#define CLUSTER_PLUG_MAJOR_VERSION	2
#define CLUSTER_PLUG_MINOR_VERSION	0

#define DEF_LOAD_THRESH_DOWN		20
#define DEF_LOAD_THRESH_UP		80
#define DEF_SAMPLING_MS			50
#define DEF_VOTE_THRESHOLD		3

#define N_BIG_CPUS			2
#define N_LITTLE_CPUS			2

#define LITTLE_CPU_ID_START		0
#define BIG_CPU_ID_START		2

#define FB_UNBLANK_BOOST_MS		1100
#define BTN_BOOST_MS			500

static DEFINE_MUTEX(cluster_plug_parameters_mutex);
static struct delayed_work cluster_plug_work;
static struct workqueue_struct *clusterplug_wq;

static unsigned int load_threshold_down = DEF_LOAD_THRESH_DOWN;
module_param(load_threshold_down, uint, 0664);

static unsigned int load_threshold_up = DEF_LOAD_THRESH_UP;
module_param(load_threshold_up, uint, 0664);

static unsigned int sampling_time = DEF_SAMPLING_MS;
module_param(sampling_time, uint, 0664);

static unsigned int vote_threshold_down = DEF_VOTE_THRESHOLD;
module_param(vote_threshold_down, uint, 0664);

static unsigned int vote_threshold_up = DEF_VOTE_THRESHOLD;
module_param(vote_threshold_up, uint, 0664);

static ktime_t last_action;

static bool active = false;
static bool whole_cluster_enabled = true;
static bool low_power_mode = false;
static bool online_all = false;

static unsigned int vote_up = 0;
static unsigned int vote_down = 0;

static struct workqueue_struct *cluster_pm_wq;
static struct work_struct resume_work;
static struct work_struct suspend_work;
static unsigned int cpu_was_online[CONFIG_NR_CPUS];
static bool suspended;

static struct work_struct boost_work;
static struct delayed_work unboost_dwork;

static int boost_refcnt;
static spinlock_t boost_lock;

struct cp_cpu_info {
	u64 prev_cpu_wall;
	u64 prev_cpu_idle;
};

static DEFINE_PER_CPU(struct cp_cpu_info, cp_info);

static void online_cpu(unsigned int cpu)
{
	lock_device_hotplug();
	if (!cpu_online(cpu))
		device_online(get_cpu_device(cpu));
	unlock_device_hotplug();
}

static void offline_cpu(unsigned int cpu)
{
	lock_device_hotplug();
	if (cpu_online(cpu))
		device_offline(get_cpu_device(cpu));
	unlock_device_hotplug();
}

static bool is_big_cpu(unsigned int cpu)
{
	return cpu == BIG_CPU_ID_START ||
		cpu == LITTLE_CPU_ID_START;
}

static bool is_little_cpu(unsigned int cpu)
{
	return !is_big_cpu(cpu);
}

static unsigned int get_delta_cpu_load_and_update(unsigned int cpu)
{
	u64 cur_wall_time, cur_idle_time;
	unsigned int wall_time, idle_time;
	struct cp_cpu_info *l_cp_info = &per_cpu(cp_info, cpu);

	/* last parameter 0 means that IO wait is considered idle */
	cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time, 0);

	wall_time = (unsigned int)
		(cur_wall_time - l_cp_info->prev_cpu_wall);
	l_cp_info->prev_cpu_wall = cur_wall_time;

	idle_time = (unsigned int)
		(cur_idle_time - l_cp_info->prev_cpu_idle);
	l_cp_info->prev_cpu_idle = cur_idle_time;

	if (unlikely(!wall_time || wall_time < idle_time))
		return 100;
	else
		return 100 * (wall_time - idle_time) / wall_time;
}

static unsigned int get_num_loaded_big_cpus(void)
{
	unsigned int cpu;
	unsigned int loaded_cpus = 0;

	for_each_possible_cpu(cpu) {
		if (is_big_cpu(cpu)) {
			unsigned int cpu_load = get_delta_cpu_load_and_update(cpu);
			/* If a cpu is offline, assume it was loaded and forced offline */
			if (!cpu_online(cpu) || cpu_load > load_threshold_up)
				loaded_cpus += 1;
		}
	}

	return loaded_cpus;
}

static unsigned int get_num_unloaded_little_cpus(void)
{
	unsigned int cpu;
	unsigned int unloaded_cpus = 0;

	for_each_online_cpu(cpu) {
		if (is_little_cpu(cpu)) {
			unsigned int cpu_load = get_delta_cpu_load_and_update(cpu);
			if (cpu_load < load_threshold_down)
				unloaded_cpus += 1;
		}
	}

	return unloaded_cpus;
}

static void __ref enable_whole_cluster(void)
{
	unsigned int cpu;
	unsigned int num_up = 0;

	for_each_present_cpu(cpu) {
		if (is_little_cpu(cpu) && !cpu_online(cpu)) {
			online_cpu(cpu);
			num_up++;
		}
	}

	if (!whole_cluster_enabled)
		pr_debug("cluster_plug: %d little cpus enabled\n", num_up);

	whole_cluster_enabled = true;
}

static void disable_half_cluster(void)
{
	unsigned int cpu;
	unsigned int num_down = 0;

	if (!whole_cluster_enabled)
		return;

	for_each_present_cpu(cpu) {
		if (is_little_cpu(cpu) && cpu_online(cpu)) {
			offline_cpu(cpu);
			num_down++;
		}
	}

	pr_debug("cluster_plug: %d little cpus disabled\n", num_down);

	whole_cluster_enabled = false;
}

static void queue_clusterplug_work(unsigned ms)
{
	queue_delayed_work(clusterplug_wq, &cluster_plug_work, msecs_to_jiffies(ms));
}

static void cluster_plug_perform(void)
{
	unsigned int loaded_cpus = get_num_loaded_big_cpus();
	unsigned int unloaded_cpus = get_num_unloaded_little_cpus();
	ktime_t now = ktime_get();

	pr_debug("cluster-plug: loaded: %u unloaded: %u votes %d / %d\n",
		loaded_cpus, unloaded_cpus, vote_up, vote_down);

	if (ktime_to_ms(ktime_sub(now, last_action)) > 5*sampling_time) {
		pr_debug("cluster_plug: ignoring old ts %lld\n",
			ktime_to_ms(ktime_sub(now, last_action)));
		vote_up = vote_down = 0;
	} else {
		if (loaded_cpus >= N_BIG_CPUS-1)
			vote_up++;
		else if (vote_up > 0)
			vote_up--;

		if (unloaded_cpus >= N_LITTLE_CPUS-1)
			vote_down++;
		else if (vote_down > 0)
			vote_down--;
	}

	if (vote_up > vote_threshold_up) {
		enable_whole_cluster();
		vote_up = vote_threshold_up;
		vote_down = 0;
	} else if (!vote_up && vote_down > vote_threshold_down) {
		disable_half_cluster();
		vote_down = vote_threshold_down;
	}

	last_action = now;
}

static void cluster_sched_boost(int enable)
{
	int old_refcnt;

	spin_lock(&boost_lock);
	old_refcnt = boost_refcnt;

	if (enable)
		boost_refcnt++;
	else
		boost_refcnt--;
	spin_unlock(&boost_lock);

	if ((old_refcnt && enable) ||
		(!old_refcnt && !enable))
		return;

	sched_set_boost(enable);
}

static void __ref cluster_plug_work_fn(struct work_struct *work)
{
	if (suspended)
		return;

	/*
	 * Just finished resuming, restore pre-resume state (CPU0 is
	 * always kept online)
	 */
	if (cpu_was_online[0]) {
		int cpu;
		for_each_possible_cpu(cpu) {
			if (!cpu)
				continue;
			if (cpu_was_online[cpu] && !cpu_online(cpu))
				online_cpu(cpu);
			else if (!cpu_was_online[cpu] && cpu_online(cpu))
				offline_cpu(cpu);
		}

		/* Clear flag used to indicate system recently resumed */
		cpu_was_online[0] = 0;

		cluster_sched_boost(0);
	}

	if (online_all) {
		int cpu;
		for_each_present_cpu(cpu) {
			if (!cpu_online(cpu))
				online_cpu(cpu);
		}
		whole_cluster_enabled = true;
		online_all = false;
	}

	if (active) {
		if (low_power_mode) {
			disable_half_cluster();
			/* Do not schedule more work */
			return;
		}

		cluster_plug_perform();
		queue_clusterplug_work(sampling_time);
	}
}

static int __ref active_show(char *buf,
			const struct kernel_param *kp __attribute__ ((unused)))
{
	return snprintf(buf, PAGE_SIZE, "%d", active);
}

static int __ref active_store(const char *buf,
			const struct kernel_param *kp __attribute__ ((unused)))
{
	int ret, value;

	ret = kstrtoint(buf, 0, &value);
	if (ret == 0) {
		mutex_lock(&cluster_plug_parameters_mutex);

		cancel_delayed_work(&cluster_plug_work);
		flush_workqueue(clusterplug_wq);

		active = (value != 0);

		/* make the internal state match the actual state */
		online_all = true;
		queue_clusterplug_work(1);

		mutex_unlock(&cluster_plug_parameters_mutex);
	}

	return ret;
}

static const struct kernel_param_ops param_ops_active = {
	.set = active_store,
	.get = active_show
};

module_param_cb(active, &param_ops_active, &active, 0664);

static int __ref low_power_mode_show(char *buf,
			const struct kernel_param *kp __attribute__ ((unused)))
{
	return snprintf(buf, PAGE_SIZE, "%d", low_power_mode);
}

static int __ref low_power_mode_store(const char *buf,
			const struct kernel_param *kp __attribute__ ((unused)))
{
	int ret, value;

	ret = kstrtoint(buf, 0, &value);
	if (ret == 0) {
		mutex_lock(&cluster_plug_parameters_mutex);

		cancel_delayed_work(&cluster_plug_work);
		flush_workqueue(clusterplug_wq);

		low_power_mode = (value != 0);
		queue_clusterplug_work(1);

		mutex_unlock(&cluster_plug_parameters_mutex);
	}

	return ret;
}

static const struct kernel_param_ops param_ops_low_power_mode = {
	.set = low_power_mode_store,
	.get = low_power_mode_show
};

module_param_cb(low_power_mode, &param_ops_low_power_mode, &low_power_mode, 0664);

static void stop_btn_boost(void)
{
	cancel_work_sync(&boost_work);
	cancel_delayed_work_sync(&unboost_dwork);
}

static void cluster_plug_resume(struct work_struct *work)
{
	unsigned int cpu;

	if (!suspended)
		return;

	stop_btn_boost();

	suspended = false;

	lock_device_hotplug();

	/* Enable all CPUs temporarily when resuming for performance */
	for_each_possible_cpu(cpu) {
		if (!cpu_online(cpu))
			device_online(get_cpu_device(cpu));
	}

	unlock_device_hotplug();

	cluster_sched_boost(1);

	queue_clusterplug_work(FB_UNBLANK_BOOST_MS);
}

static void cluster_plug_suspend(struct work_struct *work)
{
	unsigned int cpu;

	if (suspended)
		return;

	stop_btn_boost();

	suspended = true;

	cancel_delayed_work_sync(&cluster_plug_work);

	memset(&cpu_was_online[0], 0, sizeof(cpu_was_online));

	cluster_sched_boost(0);

	lock_device_hotplug();

	/* Unplug all CPUs except for CPU0 */
	for_each_online_cpu(cpu) {
		/* Save current state to restore it after resume */
		cpu_was_online[cpu] = 1;
		if (cpu)
			device_offline(get_cpu_device(cpu));
	}

	unlock_device_hotplug();
}

static int fb_notifier_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	if (*blank == FB_BLANK_UNBLANK) {
		if (val == FB_EARLY_EVENT_BLANK)
			queue_work(cluster_pm_wq, &resume_work);
	} else {
		if (val == FB_EVENT_BLANK)
			queue_work(cluster_pm_wq, &suspend_work);
	}

	return NOTIFY_OK;
}

static struct notifier_block fb_notifier = {
	.notifier_call	= fb_notifier_callback,
	.priority	= INT_MAX - 1,
};

static void cluster_plug_boost(struct work_struct *work)
{
	cancel_delayed_work_sync(&unboost_dwork);
	online_cpu(BIG_CPU_ID_START);
	cluster_sched_boost(1);
	queue_delayed_work(system_highpri_wq, &unboost_dwork,
				msecs_to_jiffies(BTN_BOOST_MS));
}

static void cluster_plug_unboost(struct work_struct *work)
{
	cluster_sched_boost(0);
	offline_cpu(BIG_CPU_ID_START);
}

static void cluster_plug_input_event(struct input_handle *handle,
		unsigned int type, unsigned int code, int value)
{
	if (!suspended)
		return;

	cancel_work_sync(&boost_work);
	queue_work(system_highpri_wq, &boost_work);
}

static int cluster_plug_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cluster_plug_handle";

	ret = input_register_handle(handle);
	if (ret)
		goto err2;

	ret = input_open_device(handle);
	if (ret)
		goto err1;

	return 0;

err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return ret;
}

static void cluster_plug_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cluster_plug_ids[] = {
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler cluster_plug_input_handler = {
	.event		= cluster_plug_input_event,
	.connect	= cluster_plug_input_connect,
	.disconnect	= cluster_plug_input_disconnect,
	.name		= "cluster_plug_handler",
	.id_table	= cluster_plug_ids,
};

int __init cluster_plug_init(void)
{
	int ret;

	pr_debug("cluster_plug: version %d.%d by sultanqasim and crpalmer\n",
		 CLUSTER_PLUG_MAJOR_VERSION,
		 CLUSTER_PLUG_MINOR_VERSION);

	spin_lock_init(&boost_lock);

	clusterplug_wq = alloc_workqueue("clusterplug",
				WQ_HIGHPRI | WQ_UNBOUND, 1);
	INIT_DELAYED_WORK(&cluster_plug_work, cluster_plug_work_fn);

	cluster_pm_wq = alloc_workqueue("clusterplug_pm", WQ_HIGHPRI, 1);

	INIT_WORK(&resume_work, cluster_plug_resume);
	INIT_WORK(&suspend_work, cluster_plug_suspend);

	fb_register_client(&fb_notifier);

	INIT_WORK(&boost_work, cluster_plug_boost);
	INIT_DELAYED_WORK(&unboost_dwork, cluster_plug_unboost);

	ret = input_register_handler(&cluster_plug_input_handler);
	if (ret)
		pr_err("%s: Failed to register input handler\n", __func__);

	return 0;
}

MODULE_AUTHOR("Sultan Qasim Khan <sultanqasim@gmail.com> and Christopher R. Palmer <crpalmer@gmail.com>");
MODULE_DESCRIPTION("'cluster_plug' - A cluster based hotplug for homogeneous"
        "ARM big.LITTLE systems where the big cluster is preferred."
);
MODULE_LICENSE("GPL");

late_initcall(cluster_plug_init);
