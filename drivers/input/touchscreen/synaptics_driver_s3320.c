/************************************************************************************
 ** File: - /android/kernel/drivers/input/touchscreen/synaptic_s3320.c
 ** Copyright (C), 2008-2012, OEM Mobile Comm Corp., Ltd
 **
 ** Description:
 **      touch panel driver for synaptics
 **      can change MAX_POINT_NUM value to support multipoint
 ** Version: 1.0
 ** Date created: 10:49:46,18/01/2012
 ** Author: Yixue.Ge@BasicDrv.TP
 **
 ** --------------------------- Revision History: --------------------------------
 ** 	<author>	<data>			<desc>
 **  chenggang.li@BSP.TP modified for oem 2014-07-30 14005 tp_driver
 ************************************************************************************/
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>

#include <linux/qdsp6v2/apr.h>

#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>

#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/time.h>

#include <linux/fb.h>

#include <linux/input/mt.h>

#include "synaptics_redremote.h"
#include "synaptics_baseline.h"

/*------------------------------------------------Global Define--------------------------------------------*/

#define TP_UNKNOWN 0
#define TP_G2Y 1
#define TP_TPK 2
#define TP_TRULY 3
#define TP_OFILM 4
#define TP_JDI_TPK 6

#define DiagonalUpperLimit  1100
#define DiagonalLowerLimit  900

#define PAGESIZE 512

#define TPD_DEVICE "synaptics,s3320"

#define SUPPORT_GESTURE
#define SUPPORT_VIRTUAL_KEY

#define TP_FW_NAME_MAX_LEN 128

#define TEST_MAGIC1 0x494D494C
#define TEST_MAGIC2 0x474D4954

struct test_header {
	unsigned int magic1;
	unsigned int magic2;
	unsigned int withCBC;
	unsigned int array_limit_offset;
	unsigned int array_limit_size;
	unsigned int array_limitcbc_offset;
	unsigned int array_limitcbc_size;
};

/*********************for gesture*******************/
#ifdef SUPPORT_GESTURE
#define ENABLE_UNICODE  0x40
#define ENABLE_VEE      0x20
#define ENABLE_CIRCLE   0x08
#define ENABLE_SWIPE    0x02
#define ENABLE_DTAP     0x01

#define UNICODE_DETECT  0x0b
#define VEE_DETECT      0x0a
#define CIRCLE_DETECT   0x08
#define SWIPE_DETECT    0x07
#define DTAP_DETECT     0x03


#define UnkownGestrue       0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W

#define KEY_DOUBLE_TAP				KEY_WAKEUP
#define KEY_GESTURE_CIRCLE			250
#define KEY_GESTURE_TWO_SWIPE		251
#define KEY_GESTURE_DOWN_ARROW		252
#define KEY_GESTURE_LEFT_V			253
#define KEY_GESTURE_RIGHT_V			254
#define KEY_GESTURE_UP_ARROW		255
#define KEY_GESTURE_SWIPE_RIGHT		KEY_F5
#define KEY_GESTURE_SWIPE_LEFT		KEY_F6
#define KEY_GESTURE_SWIPE_DOWN		KEY_F7
#define KEY_GESTURE_SWIPE_UP		KEY_F8

#define BIT0 (0x1 << 0)
#define BIT1 (0x1 << 1)
#define BIT2 (0x1 << 2)
#define BIT3 (0x1 << 3)
#define BIT4 (0x1 << 4)
#define BIT5 (0x1 << 5)
#define BIT6 (0x1 << 6)
#define BIT7 (0x1 << 7)

int left_arrow_enable = 0; //">"
int right_arrow_enable = 0; //"<"
int double_swipe_enable = 0; // "||"
int letter_o_enable = 0; // "O"
int down_arrow_enable = 0; //"V"
int up_arrow_enable = 0; //"^"
int double_tap_enable = 0; //"double tap"

int right_swipe_enable=0;//"(-->)"
int left_swipe_enable=0;//"(<--)"
int down_swipe_enable =0;//"up to down |"
int up_swipe_enable =0;//"down to up |"

int letter_w_enable =0;//"(W)"
int letter_m_enable =0;//"(M)"
static int gesture_switch = 0;
#endif

/*********************for Debug LOG switch*******************/
#define TPD_ERR(a, arg...)  pr_err(TPD_DEVICE ": " a, ##arg)
#define TPDTM_DMESG(a, arg...)  printk(TPD_DEVICE ": " a, ##arg)

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
		pr_err(TPD_DEVICE ": " a,##arg);\
	}while(0)

/*---------------------------------------------Global Variable----------------------------------------------*/
static int TP_FW;
static unsigned int tp_debug = 0;
static int button_map[3];
static int tx_rx_num[2];
static int16_t delta[8][16];
static int TX_NUM;
static int RX_NUM;
static int report_key_point_y = 0;
static int force_update = 0;
static int LCD_WIDTH ;
static int LCD_HEIGHT ;
static int get_tp_base = 0;
static int F51_CUSTOM_CTRL74;
static int limit_enable=1;
static void synaptics_tpedge_limitfunc(void);

static struct synaptics_ts_data *ts_g = NULL;
static struct proc_dir_entry *prEntry_tp = NULL;


#ifdef SUPPORT_GESTURE
static uint32_t clockwise;
static uint32_t gesture;

static uint32_t gesture_upload;

/****point position*****/
struct Coordinate {
	uint32_t x;
	uint32_t y;
};
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
#endif

/*-----------------------------------------Global Registers----------------------------------------------*/
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaF34_FlashControl;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;
static int F01_RMI_CTRL_BASE;
static int F01_RMI_DATA_BASE;

static int F12_2D_QUERY_BASE;
static int F12_2D_CMD_BASE;
static int F12_2D_CTRL_BASE;
static int F12_2D_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_QUERY11;
static int F01_RMI_DATA01;
static int F01_RMI_CMD00;
static int F01_RMI_CTRL00;
static int F01_RMI_CTRL01;

static int F12_2D_CTRL08;
static int F12_2D_CTRL32;
static int F12_2D_DATA04;
static int F12_2D_DATA38;
static int F12_2D_DATA39;
static int F12_2D_CMD00;
static int F12_2D_CTRL20;
static int F12_2D_CTRL27;

static int F34_FLASH_CTRL00;

static int F51_CUSTOM_CTRL00;
static int F51_CUSTOM_DATA04;
static int F51_CUSTOM_DATA11;
static int version_is_s3508=0;

/*------------------------------------------Fuction Declare----------------------------------------------*/
static int synaptics_i2c_suspend(struct device *dev);
static int synaptics_i2c_resume(struct device *dev);
/**************I2C resume && suspend end*********/
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force);

static int synaptics_rmi4_i2c_read_block(struct i2c_client* client,
		unsigned char addr,unsigned short length,unsigned char *data);

static int synaptics_rmi4_i2c_write_block(struct i2c_client* client,
		unsigned char addr, unsigned short length, unsigned char const *data);

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
		unsigned char addr);

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
		unsigned char addr,unsigned char data);

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
		unsigned char addr);
static int synaptics_mode_change(int mode);

static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id);

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
static int synaptics_soft_reset(struct synaptics_ts_data *ts);
static void synaptics_hard_reset(struct synaptics_ts_data *ts);
static void tp_baseline_get(struct synaptics_ts_data *ts);

static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

static struct of_device_id synaptics_match_table[] = {
	{ .compatible = TPD_DEVICE,},
	{ },
};

static const struct dev_pm_ops synaptic_pm_ops = {
#ifdef CONFIG_PM
	.suspend = synaptics_i2c_suspend,
	.resume = synaptics_i2c_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
};

static struct i2c_driver tpd_i2c_driver = {
	.probe		= synaptics_ts_probe,
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= TPD_DEVICE,
		.of_match_table =  synaptics_match_table,
		.pm = &synaptic_pm_ops,
	},
};

struct synaptics_ts_data {
	struct i2c_client *client;
	struct mutex mutex;
	int irq;
	int irq_gpio;
	bool irq_disabled;
	int id1_gpio;
	int id2_gpio;
	int id3_gpio;
	int reset_gpio;
	int v1p8_gpio;
	int support_hw_poweroff;
	int enable2v8_gpio;
	int max_num;
	int enable_remote;
	uint32_t irq_flags;
	uint32_t max_x;
	uint32_t max_y;
	uint32_t max_y_real;
	uint32_t btn_state;
	uint32_t pre_finger_state;
	uint32_t pre_btn_state;
	struct work_struct base_work;
	struct input_dev *input_dev;
	struct notifier_block fb_notif;
	/******gesture*******/
	int gesture_enable;
	int in_gesture_mode;
	int glove_enable;
	int screen_off;
    spinlock_t lock;

	/******power*******/
	struct regulator *vdd_2v8;
	struct regulator *vcc_i2c_1v8;

	/*pinctrl******/
	struct device						*dev;
	struct pinctrl 						*pinctrl;
	struct pinctrl_state 				*pinctrl_state_active;
	struct pinctrl_state 				*pinctrl_state_suspend;

	/*******for FW update*******/
	bool loading_fw;
    bool support_ft;//support force touch
	char fw_name[TP_FW_NAME_MAX_LEN];
	char test_limit_name[TP_FW_NAME_MAX_LEN];
	char fw_id[12];
	char manu_name[10];
#ifdef SUPPORT_VIRTUAL_KEY
        struct kobject *properties_kobj;
#endif

	struct work_struct pm_work;

	struct wakeup_source syna_isr_ws;
	spinlock_t isr_lock;
	bool i2c_awake;
	struct completion i2c_resume;

	bool touch_active;
};

static void touch_enable(struct synaptics_ts_data *ts)
{
	bool irq_disabled;

	spin_lock(&ts->lock);
	irq_disabled = ts->irq_disabled;
	ts->irq_disabled = false;
	spin_unlock(&ts->lock);

	if (irq_disabled)
		enable_irq(ts->irq);
}

static void touch_disable(struct synaptics_ts_data *ts)
{
	bool irq_disabled;

	spin_lock(&ts->lock);
	irq_disabled = ts->irq_disabled;
	ts->irq_disabled = true;
	spin_unlock(&ts->lock);

	if (!irq_disabled)
		disable_irq(ts->irq);
}

static void tpd_hw_pwron(struct synaptics_ts_data *ts)
{
	int rc;

	pinctrl_select_state(ts->pinctrl, ts->pinctrl_state_active);

	if (ts->v1p8_gpio > 0)
		gpio_direction_output(ts->v1p8_gpio, 1);

	if (ts->enable2v8_gpio > 0)
		gpio_direction_output(ts->enable2v8_gpio, 1);

	rc = regulator_enable(ts->vdd_2v8);
	if (rc)
		dev_err(&ts->client->dev, "vdd_2v8 enable failed rc=%d\n", rc);

	rc = regulator_enable(ts->vcc_i2c_1v8);
	if (rc)
		dev_err(&ts->client->dev, "vcc_i2c_1v8 enable failed rc=%d\n", rc);

	if (ts->reset_gpio > 0)
		gpio_direction_output(ts->reset_gpio, 1);
}

static void tpd_hw_pwroff(struct synaptics_ts_data *ts)
{
	int rc;

	if (ts->v1p8_gpio > 0)
		gpio_direction_output(ts->v1p8_gpio, 0);

	if (ts->enable2v8_gpio > 0)
		gpio_direction_output(ts->enable2v8_gpio, 0);

	rc = regulator_disable(ts->vdd_2v8);
	if (rc)
		dev_err(&ts->client->dev, "vdd_2v8 disable failed rc=%d\n", rc);

	rc = regulator_disable(ts->vcc_i2c_1v8);
	if (rc)
		dev_err(&ts->client->dev, "vcc_i2c_1v8 disable failed rc=%d\n", rc);

	if (ts->reset_gpio > 0)
		gpio_direction_output(ts->reset_gpio, 0);

	pinctrl_select_state(ts->pinctrl, ts->pinctrl_state_suspend);
}

static void tpd_power(struct synaptics_ts_data *ts, bool on)
{
	if (on)
		tpd_hw_pwron(ts);
	else
		tpd_hw_pwroff(ts);
}

static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
	uint8_t buf[4];
	int ret;
	memset(buf, 0, sizeof(buf));
	ret = synaptics_rmi4_i2c_write_byte( ts->client, 0xff, 0x0 );
	if( ret < 0 ){
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xDD, 4, &(buf[0x0]));
	if( ret < 0 ){
		TPD_ERR("failed for page select!\n");
		return -1;
	}

	F12_2D_QUERY_BASE = buf[0];
	F12_2D_CMD_BASE = buf[1];
	F12_2D_CTRL_BASE = buf[2];
	F12_2D_DATA_BASE = buf[3];

	TPD_ERR("F12_2D_QUERY_BASE = %x \n \
			F12_2D_CMD_BASE  = %x \n\
			F12_2D_CTRL_BASE	= %x \n\
			F12_2D_DATA_BASE	= %x \n\
			",F12_2D_QUERY_BASE,F12_2D_CMD_BASE,F12_2D_CTRL_BASE,F12_2D_DATA_BASE);


	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE3, 4, &(buf[0x0]));
	F01_RMI_QUERY_BASE = buf[0];
	F01_RMI_CMD_BASE = buf[1];
	F01_RMI_CTRL_BASE = buf[2];
	F01_RMI_DATA_BASE = buf[3];
	TPD_DEBUG("F01_RMI_QUERY_BASE = %x \n\
			F01_RMI_CMD_BASE  = %x \n\
			F01_RMI_CTRL_BASE	= %x \n\
			F01_RMI_DATA_BASE	= %x \n\
			", F01_RMI_QUERY_BASE, F01_RMI_CMD_BASE, F01_RMI_CTRL_BASE, F01_RMI_DATA_BASE);

	ret = synaptics_rmi4_i2c_read_block( ts->client, 0xE9, 4, &(buf[0x0]) );
	F34_FLASH_QUERY_BASE = buf[0];
	F34_FLASH_CMD_BASE = buf[1];
	F34_FLASH_CTRL_BASE = buf[2];
	F34_FLASH_DATA_BASE = buf[3];
	TPD_ERR("F34_FLASH_QUERY_BASE = %x \n\
			F34_FLASH_CMD_BASE	= %x \n\
			F34_FLASH_CTRL_BASE	= %x \n\
			F34_FLASH_DATA_BASE	= %x \n\
			", F34_FLASH_QUERY_BASE, F34_FLASH_CMD_BASE, F34_FLASH_CTRL_BASE, F34_FLASH_DATA_BASE);

	F01_RMI_QUERY11 = F01_RMI_QUERY_BASE+11;
	F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
	F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;
	F01_RMI_CMD00 = F01_RMI_CMD_BASE;
	F01_RMI_DATA01 = F01_RMI_DATA_BASE + 1;

	F12_2D_CTRL08 = F12_2D_CTRL_BASE;
	F12_2D_CTRL32 = F12_2D_CTRL_BASE + 15;
	F12_2D_DATA38 = F12_2D_DATA_BASE + 54;
	F12_2D_DATA39 = F12_2D_DATA_BASE + 55;
	F12_2D_CMD00 = F12_2D_CMD_BASE;
	F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x07;
	F12_2D_CTRL27 = F12_2D_CTRL_BASE + 0x0c;


	F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
	if( ret < 0 ){
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F51_CUSTOM_QUERY_BASE = buf[0];
	F51_CUSTOM_CMD_BASE = buf[1];
	F51_CUSTOM_CTRL_BASE = buf[2];
	F51_CUSTOM_DATA_BASE = buf[3];
	F51_CUSTOM_CTRL00 = F51_CUSTOM_CTRL_BASE;
	F51_CUSTOM_DATA04 = F51_CUSTOM_DATA_BASE;
	F51_CUSTOM_DATA11 = F51_CUSTOM_DATA_BASE;

	TPD_DEBUG("F51_CUSTOM_QUERY_BASE = %x \n\
			F51_CUSTOM_CMD_BASE  = %x \n\
			F51_CUSTOM_CTRL_BASE    = %x \n\
			F51_CUSTOM_DATA_BASE    = %x \n\
			", F51_CUSTOM_QUERY_BASE, F51_CUSTOM_CMD_BASE, F51_CUSTOM_CTRL_BASE, F51_CUSTOM_DATA_BASE);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	return 0;
}

#ifdef SUPPORT_GESTURE
static int synaptics_enable_interrupt_for_gesture(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	unsigned char reportbuf[4];
	//chenggang.li@BSP.TP modified for gesture
	TPD_DEBUG("%s is called\n", __func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPD_ERR("%s: select page failed ret = %d\n", __func__, ret);
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ) {
		TPD_DEBUG("read reg F12_2D_CTRL20[0x%x] failed\n",F12_2D_CTRL20);
		return -1;
	}

	if( enable ) {
		ts->in_gesture_mode = 1;
		reportbuf[2] |= 0x02 ;
	} else {
		ts->in_gesture_mode = 0;
		reportbuf[2] &= 0xfd ;
	}
	TPD_DEBUG("F12_2D_CTRL20:0x%x=[2]:0x%x\n", F12_2D_CTRL20, reportbuf[2]);
	ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ){
		TPD_ERR("%s :Failed to write report buffer\n", __func__);
		return -1;
	}
	gesture = UnkownGestrue;
	return 0;
}
#endif

static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[11];
	int ret ;

	memset(buf1, 0 , sizeof(buf1));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPDTM_DMESG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY11, 8, &(buf1[0x0]));
	ret = synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY_BASE+19, 2, &(buf1[0x8]));
	if( ret < 0 ){
		TPD_ERR("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;

	TPD_DEBUG("%s is called!\n",__func__);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPD_ERR("init_panel failed for page select\n");
		return -1;
	}
	/*device control: normal operation, configur=1*/

    ret = synaptics_mode_change(0x80);//change tp to doze mode
	if( ret < 0 ){
		msleep(150);
        ret = synaptics_mode_change(0x80);
		if( ret < 0 ){
			TPD_ERR("%s failed for mode select\n",__func__);
		}
	}

	return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	uint8_t abs_status_int;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPDTM_DMESG("synaptics_enable_interrupt: select page failed ret = %d\n",
		    ret);
		return -1;
	}
	if( enable ) {
		abs_status_int = 0x7f;
		/*clear interrupt bits for previous touch*/
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE+1);
		if( ret < 0 ) {
			TPDTM_DMESG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	} else {
		abs_status_int = 0x0;
	}
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL00+1, abs_status_int);
	if( ret < 0 ) {
		TPDTM_DMESG("%s: enable or disable abs \
		    interrupt failed,abs_int =%d\n", __func__, abs_status_int);
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_CTRL00+1);
	return 0;
}

static int synaptics_rmi4_i2c_read_block(struct i2c_client* client,
		unsigned char addr,unsigned short length,unsigned char *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};
	buf = addr & 0xFF;
	for( retry = 0; retry < 2; retry++ ) {
		if( i2c_transfer(client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if( retry == 2 ) {
		dev_err(&client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		//rst_flag_counter = 1;//reset tp
		retval = -5;
	} else {
		//rst_flag_counter = 0;
	}
	return retval;
}

static int synaptics_rmi4_i2c_write_block(struct i2c_client* client,
		unsigned char addr, unsigned short length, unsigned char const *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = addr & 0xff;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < 2; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if (retry == 2) {
		//rst_flag_counter = 1;//rest tp
		retval = -EIO;
	} else {
		//rst_flag_counter = 0;
	}
	return retval;
}

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
		unsigned char addr)
{
	int retval = 0;
	unsigned char buf[2] = {0};
	retval = synaptics_rmi4_i2c_read_block(client,addr,1,buf);
	if(retval >= 0)
		retval = buf[0]&0xff;
	return retval;
}

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
		unsigned char addr,unsigned char data)
{
	int retval;
	unsigned char data_send = data;
	retval = synaptics_rmi4_i2c_write_block(client,addr,1,&data_send);
	return retval;
}

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
		unsigned char addr)
{
	int retval;
	unsigned char buf[2] = {0};
	retval = synaptics_rmi4_i2c_read_block(client,addr,2,buf);
	if(retval >= 0)
		retval = buf[1]<<8|buf[0];
	return retval;
}
//chenggang.li@BSP.TP modified for oem 2014-08-05 gesture_judge
/***************start****************/
#ifdef SUPPORT_GESTURE
static void synaptics_get_coordinate_point(struct synaptics_ts_data *ts)
{
	int ret,i;
	uint8_t coordinate_buf[25] = {0};
	uint16_t trspoint = 0;
	static uint8_t coordinate_buf_last[25]= {0};

	TPD_DEBUG("%s is called!\n",__func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11, 8, &(coordinate_buf[0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 8, 8, &(coordinate_buf[8]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 16, 8, &(coordinate_buf[16]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 24, 1, &(coordinate_buf[24]));

    if(!memcmp(coordinate_buf_last,coordinate_buf,sizeof(coordinate_buf)))
    {
        TPD_ERR("%s reject the same gestrue[%d]\n",__func__,gesture);
        gesture = UnkownGestrue;
    }
	memcpy(coordinate_buf_last,coordinate_buf,sizeof(coordinate_buf));
   // strcpy(coordinate_buf_last,coordinate_buf/*,sizeof(coordinate_buf)*/);

	for(i = 0; i< 23; i += 2) {
		trspoint = coordinate_buf[i]|coordinate_buf[i+1] << 8;
		TPD_DEBUG("synaptics TP read coordinate_point[%d] = %d\n",i,trspoint);
	}

	TPD_DEBUG("synaptics TP coordinate_buf = 0x%x\n",coordinate_buf[24]);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	Point_start.x = (coordinate_buf[0] | (coordinate_buf[1] << 8)) * LCD_WIDTH/ (ts->max_x);
	Point_start.y = (coordinate_buf[2] | (coordinate_buf[3] << 8)) * LCD_HEIGHT/ (ts->max_y);
	Point_end.x   = (coordinate_buf[4] | (coordinate_buf[5] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_end.y   = (coordinate_buf[6] | (coordinate_buf[7] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_1st.x   = (coordinate_buf[8] | (coordinate_buf[9] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_1st.y   = (coordinate_buf[10] | (coordinate_buf[11] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_2nd.x   = (coordinate_buf[12] | (coordinate_buf[13] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_2nd.y   = (coordinate_buf[14] | (coordinate_buf[15] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_3rd.x   = (coordinate_buf[16] | (coordinate_buf[17] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_3rd.y   = (coordinate_buf[18] | (coordinate_buf[19] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_4th.x   = (coordinate_buf[20] | (coordinate_buf[21] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_4th.y   = (coordinate_buf[22] | (coordinate_buf[23] << 8)) * LCD_HEIGHT / (ts->max_y);
	clockwise     = (coordinate_buf[24] & 0x10) ? 1 :
		(coordinate_buf[24] & 0x20) ? 0 : 2; // 1--clockwise, 0--anticlockwise, not circle, report 2
}

bool s3320_touch_active(void)
{
	struct synaptics_ts_data *ts = ts_g;

	return ts ? ts->touch_active : false;
}

static void gesture_judge(struct synaptics_ts_data *ts)
{
	unsigned int keyCode = KEY_F4;
	int ret = 0,gesture_sign, regswipe;
	uint8_t gesture_buffer[10];
	unsigned char reportbuf[3];
        if(version_is_s3508)
		F12_2D_DATA04 = 0x0008;
	else
 		F12_2D_DATA04 = 0x000A;
	TPD_DEBUG("%s start!\n",__func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0) {
		TPDTM_DMESG("failed to transfer the data, ret = %d\n", ret);
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	ret = i2c_smbus_read_i2c_block_data(ts->client,  F12_2D_DATA04, 5, &(gesture_buffer[0]));
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4);
	if(version_is_s3508)
		regswipe = i2c_smbus_read_byte_data(ts->client, F51_CUSTOM_DATA04+0x18);
	else
		regswipe = i2c_smbus_read_byte_data(ts->client, F51_CUSTOM_DATA04+0x18);
	if(version_is_s3508)
		TPD_DEBUG("s35080Gesture Type[0x%x]=[0x%x],lpwg Swipe ID[0x4%x] = [0x%x]\n",\
		F12_2D_DATA04,gesture_buffer[0],(F51_CUSTOM_DATA04+0x18),regswipe);
	else
		TPD_DEBUG("Gesture Type[0x%x]=[0x%x],lpwg Swipe ID[0x4%x] = [0x%x]\n",\
		F12_2D_DATA04,gesture_buffer[0],(F51_CUSTOM_DATA04+0x18),regswipe);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	gesture_sign = gesture_buffer[0];
	//detect the gesture mode
	switch (gesture_sign) {
		case DTAP_DETECT:
			    gesture = DouTap;
			break;
		case SWIPE_DETECT:
			if(version_is_s3508){
				gesture =   (regswipe == 0x41) ? Left2RightSwip   :
					(regswipe == 0x42) ? Right2LeftSwip   :
					(regswipe == 0x44) ? Up2DownSwip      :
					(regswipe == 0x48) ? Down2UpSwip      :
					(regswipe == 0x80) ? DouSwip          :
					UnkownGestrue;
				break;
			}else{
				gesture = (regswipe == 0x41) ? Left2RightSwip   :
					(regswipe == 0x42) ? Right2LeftSwip   :
					(regswipe == 0x44) ? Up2DownSwip      :
					(regswipe == 0x48) ? Down2UpSwip      :
					(regswipe == 0x84) ? DouSwip          :
					UnkownGestrue;
				break;
			}
		case CIRCLE_DETECT:
			    gesture = Circle;
			break;
		case VEE_DETECT:
			gesture = (gesture_buffer[2] == 0x01) ? DownVee  :
				(gesture_buffer[2] == 0x02) ? UpVee    :
				(gesture_buffer[2] == 0x04) ? RightVee :
				(gesture_buffer[2] == 0x08) ? LeftVee  :
				UnkownGestrue;
			break;
		case UNICODE_DETECT:
			gesture = (gesture_buffer[2] == 0x77) ? Wgestrue :
				(gesture_buffer[2] == 0x6d) ? Mgestrue :
				UnkownGestrue;
	}

	keyCode = UnkownGestrue;
	// Get key code based on registered gesture.
	switch (gesture) {
		case DouTap:
			keyCode = KEY_DOUBLE_TAP;
			break;
		case UpVee:
			keyCode = KEY_GESTURE_DOWN_ARROW;
			break;
		case DownVee:
			keyCode = KEY_GESTURE_UP_ARROW;
			break;
		case LeftVee:
			keyCode = KEY_GESTURE_RIGHT_V;
			break;
		case RightVee:
			keyCode = KEY_GESTURE_LEFT_V;
			break;
		case Circle:
			keyCode = KEY_GESTURE_CIRCLE;
			break;
		case DouSwip:
			keyCode = KEY_GESTURE_TWO_SWIPE;
			break;
		case Left2RightSwip:
			keyCode = KEY_GESTURE_SWIPE_RIGHT;
			break;
		case Right2LeftSwip:
			keyCode = KEY_GESTURE_SWIPE_LEFT;
			break;
		case Up2DownSwip:
			keyCode = KEY_GESTURE_SWIPE_DOWN;
			break;
		case Down2UpSwip:
			keyCode = KEY_GESTURE_SWIPE_UP;
			break;
		default:
			break;
	}

	TPD_ERR("detect %s gesture\n",
			gesture == DouTap ? "(double tap)" :
			gesture == UpVee ? "(V)" :
			gesture == DownVee ? "(^)" :
			gesture == LeftVee ? "(>)" :
			gesture == RightVee ? "(<)" :
			gesture == Circle ? "(O)" :
			gesture == DouSwip ? "(||)" :
			gesture == Left2RightSwip ? "(-->)" :
			gesture == Right2LeftSwip ? "(<--)" :
			gesture == Up2DownSwip ? "(up to down |)" :
			gesture == Down2UpSwip ? "(down to up |)" :
			gesture == Mgestrue ? "(M)" :
			gesture == Wgestrue ? "(W)" : "[unknown]");
	synaptics_get_coordinate_point(ts);

    TPD_DEBUG("gesture suport LeftVee:%d RightVee:%d DouSwip:%d Circle:%d UpVee:%d DownVee:%d DouTap:%d \
Left2RightSwip:%d Right2LeftSwip:%d Up2DownSwip:%d Down2UpSwip:%d\n",
        left_arrow_enable,right_arrow_enable,double_swipe_enable,letter_o_enable,down_arrow_enable,up_arrow_enable,double_tap_enable,
		right_swipe_enable,left_swipe_enable,down_swipe_enable,up_swipe_enable);
	if ((gesture == DouTap && double_tap_enable) || (gesture == RightVee && right_arrow_enable)
        || (gesture == LeftVee && left_arrow_enable) || (gesture == UpVee && down_arrow_enable)
		|| (gesture == DownVee && up_arrow_enable) || (gesture == Circle && letter_o_enable)
		|| (gesture == DouSwip && double_swipe_enable) || (gesture == Left2RightSwip && right_swipe_enable)
		|| (gesture == Right2LeftSwip && left_swipe_enable) || (gesture == Up2DownSwip && down_swipe_enable)
		|| (gesture == Down2UpSwip && up_swipe_enable)) {
		gesture_upload = gesture;
		if (!q6voice_voice_call_active()) {
			input_report_key(ts->input_dev, keyCode, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, keyCode, 0);
			input_sync(ts->input_dev);
		}
	}else{

		ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
		ret = reportbuf[2] & 0x20;
		if(ret == 0)
			reportbuf[2] |= 0x02 ;
		ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) ); //enable gesture
		if( ret < 0 ){
			TPD_ERR("%s :Failed to write report buffer\n", __func__);
			return;
		}
	}
    TPD_DEBUG("%s end!\n",__func__);
}
#endif
static bool int_touch(struct synaptics_ts_data *ts, ktime_t timestamp)
{
	uint8_t buf[80];
	int i, ret;
	bool finger_present = false;

	i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	ret = synaptics_rmi4_i2c_read_block(ts->client, F12_2D_DATA_BASE, 80, buf);
	if (unlikely(ret < 0)) {
		TPD_ERR("int_touch: i2c_transfer failed\n");
		return false;
	}

	input_event(ts->input_dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(timestamp).tv_sec);
	input_event(ts->input_dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(timestamp).tv_nsec);

	for (i = 0; i < 10; i++) {
		int x, y, raw_x, raw_y;
		uint8_t status;

		status = buf[i * 8] & 0x03;

		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, status);

		if (!status)
			continue;

		x = ((buf[i * 8 + 2] & 0x0f) << 8) | buf[i * 8 + 1];
		y = ((buf[i * 8 + 4] & 0x0f) << 8) | buf[i * 8 + 3];
		raw_x = buf[i * 8 + 6] & 0x0f;
		raw_y = buf[i * 8 + 7] & 0x0f;

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, max(raw_x, raw_y));
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, min(raw_x, raw_y));

		finger_present = true;
	}

	input_sync(ts->input_dev);

	if (ts->in_gesture_mode && ts->screen_off)
		gesture_judge(ts);

	return finger_present;
}

static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;
	ktime_t now;
	int ret;

	if (ts->screen_off) {
		unsigned long flags;
		bool i2c_active;

		spin_lock_irqsave(&ts->isr_lock, flags);
		i2c_active = ts->i2c_awake;
		spin_unlock_irqrestore(&ts->isr_lock, flags);

		/* I2C bus must be active */
		if (!i2c_active) {
			__pm_stay_awake(&ts->syna_isr_ws);
			/* Wait for I2C to resume before proceeding */
			reinit_completion(&ts->i2c_resume);
			wait_for_completion_timeout(&ts->i2c_resume,
							msecs_to_jiffies(30));
		}
	}

	now = ktime_get();

	synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	ret = synaptics_rmi4_i2c_read_word(ts->client, F01_RMI_DATA_BASE);
	if (ret < 0) {
		TPDTM_DMESG("Synaptic:ret = %d\n", ret);
		synaptics_hard_reset(ts);
		goto exit;
	}

	if (ret & 0x80) {
		synaptics_init_panel(ts);

		if (ts->screen_off && ts->gesture_enable)
			synaptics_enable_interrupt_for_gesture(ts, true);
	}

	if (ret & 0x400) {
		bool finger_present = int_touch(ts, now);

		ts->touch_active = finger_present;

		/* All fingers up; do get base once */
		if (!get_tp_base && !finger_present) {
			get_tp_base = 1;
			TPD_ERR("start get base data: 1\n");
			schedule_work(&ts->base_work);
		}
	}

exit:
	if (ts->syna_isr_ws.active)
		__pm_relax(&ts->syna_isr_ws);
	return IRQ_HANDLED;
}

#ifdef SUPPORT_GESTURE
static ssize_t tp_gesture_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	TPD_DEBUG("gesture enable is: %d\n", ts->gesture_enable);
	ret = snprintf(page, sizeof(page), "%d\n", ts->gesture_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_gesture_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[10];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return count;
	if( count > 2 || ts->screen_off)
		return count;
	if( copy_from_user(buf, buffer, count) ){
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	TPD_ERR("%s write [0x%x]\n",__func__,buf[0]);

    down_arrow_enable = (buf[0] & BIT0)?1:0; //"V"
    double_swipe_enable = (buf[0] & BIT1)?1:0;//"||"
    up_arrow_enable = (buf[0] & BIT2)?1:0; //"^"
    left_arrow_enable = (buf[0] & BIT3)?1:0; //">"
    right_arrow_enable = (buf[0] & BIT4)?1:0;//"<"
    letter_o_enable = (buf[0] & BIT6)?1:0; //"O"
    double_tap_enable = (buf[0] & BIT7)?1:0; //double tap

	if(double_tap_enable||letter_o_enable||down_arrow_enable||up_arrow_enable||left_arrow_enable\
        ||right_arrow_enable||double_swipe_enable)
	{
		ts->gesture_enable = 1;
	}
	else
    {
        ts->gesture_enable = 0;
    }
	return count;
}
static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	TPD_ERR("%s:gesture_upload = %d \n",__func__,gesture_upload);
	ret = snprintf(page, sizeof(page), "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t gesture_switch_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	ret = snprintf(page, sizeof(page), "gesture_switch:%d\n", gesture_switch);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t gesture_switch_write_func(struct file *file, const char __user *page, size_t count, loff_t *ppos)
{
	int ret,write_flag=0;
	char buf[10]={0};
	struct synaptics_ts_data *ts = ts_g;

	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!!\n",__func__);
		return count;
	}
	if( copy_from_user(buf, page, count) ){
		TPD_ERR("%s: read proc input error.\n", __func__);
		return count;
	}
	ret = sscanf(buf,"%d",&write_flag);
	gesture_switch = write_flag;
	TPD_ERR("gesture_switch:%d,suspend:%d,gesture:%d\n",gesture_switch,ts->screen_off,ts->gesture_enable);
	if (1 == gesture_switch){
		if ((ts->screen_off == 1) && (ts->gesture_enable == 1)){
			i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
			synaptics_mode_change(0x80);
			//touch_enable(ts);
			synaptics_enable_interrupt_for_gesture(ts, 1);
		}
	}else if(2 == gesture_switch){
		if ((ts->screen_off == 1) && (ts->gesture_enable == 1)){
			i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
			synaptics_mode_change(0x81);
			//touch_disable(ts);
			synaptics_enable_interrupt_for_gesture(ts, 0);
		}
	}

	return count;
}

static void gesture_enable(struct synaptics_ts_data *ts)
{
	ts->gesture_enable = double_tap_enable || letter_o_enable || down_arrow_enable || up_arrow_enable
			|| left_arrow_enable || right_arrow_enable || double_swipe_enable || right_swipe_enable
			|| left_swipe_enable || down_swipe_enable || up_swipe_enable ? 1 : 0;
}

// chenggang.li@BSP.TP modified for oem 2014-08-08 create node
/******************************start****************************/
static const struct file_operations tp_gesture_proc_fops = {
	.write = tp_gesture_write_func,
	.read =  tp_gesture_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations gesture_switch_proc_fops = {
	.write = gesture_switch_write_func,
	.read =  gesture_switch_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations coordinate_proc_fops = {
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#define TS_ENABLE_FOPS(type) \
static ssize_t type##_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos) \
{ \
	char enable[3]; \
	sprintf(enable, "%d\n", !!type##_enable); \
	return simple_read_from_buffer(user_buf, sizeof(enable), ppos, enable - *ppos, sizeof(enable)); \
} \
static ssize_t type##_write_func(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos) \
{ \
	int ret; \
	char enable; \
	struct synaptics_ts_data *ts = ts_g; \
	if (ts->screen_off) \
		return count; \
	ret = copy_from_user(&enable, user_buf, sizeof(enable)); \
	if (ret) \
		return ret; \
	type##_enable = enable - '0'; \
	gesture_enable(ts); \
	return count; \
} \
static const struct file_operations type##_proc_fops = { \
	.write = type##_write_func, \
	.read =  type##_read_func, \
	.open = simple_open, \
	.owner = THIS_MODULE, \
};

TS_ENABLE_FOPS(double_swipe);
TS_ENABLE_FOPS(double_tap);
TS_ENABLE_FOPS(down_arrow);
TS_ENABLE_FOPS(down_swipe);
TS_ENABLE_FOPS(left_arrow);
TS_ENABLE_FOPS(left_swipe);
TS_ENABLE_FOPS(letter_o);
TS_ENABLE_FOPS(right_arrow);
TS_ENABLE_FOPS(right_swipe);
TS_ENABLE_FOPS(up_arrow);
TS_ENABLE_FOPS(up_swipe);
#endif

static int synaptics_input_init(struct synaptics_ts_data *ts)
{
	int ret = 0;

	TPD_DEBUG("%s is called\n",__func__);
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("synaptics_ts_probe: Failed to allocate input device\n");
		return ret;
	}
    ts->input_dev->name = TPD_DEVICE;;
    ts->input_dev->dev.parent = &ts->client->dev;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#ifdef SUPPORT_GESTURE
	set_bit(KEY_F4 , ts->input_dev->keybit);//doulbe-tap resume
	set_bit(KEY_DOUBLE_TAP, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_CIRCLE, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_UP_ARROW, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_DOWN_ARROW, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_TWO_SWIPE, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_LEFT_V, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_RIGHT_V, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_SWIPE_RIGHT, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_SWIPE_LEFT, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_SWIPE_DOWN, ts->input_dev->keybit);
	set_bit(KEY_GESTURE_SWIPE_UP, ts->input_dev->keybit);
#endif
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR, 0,255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, (ts->max_x-1), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, (ts->max_y-1), 0, 0);
	input_mt_init_slots(ts->input_dev, ts->max_num, 0);
	input_set_drvdata(ts->input_dev, ts);

	if(input_register_device(ts->input_dev)) {
		TPD_ERR("%s: Failed to register input device\n",__func__);
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);
		return -1;
	}
	return 0;
}

#include "fw_update_v7.if"
static int check_hardware_version(struct device *dev)
{
        int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
        const struct firmware *fw = NULL;
	if(!ts->client) {
		TPD_ERR("i2c client point is NULL\n");
		return 0;
	}
	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
		return ret;
	}

        ret = fwu_start_reflash_check(fw->data,ts->client);
	release_firmware(fw);
        if (ret < 0)
		return -1;
        else
            	return ret;
}
static int check_version = 0;
/*********************FW Update Func******************************************/
static int synatpitcs_fw_update(struct device *dev, bool force)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int ret;
	char fw_id_temp[12];
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0 ;

        static bool check_onetime = true;

        TPD_DEBUG("%s is called\n",__func__);
	if(!ts->client) {
		TPD_ERR("i2c client point is NULL\n");
		return 0;
	}
	if (!strncmp(ts->manu_name,"S3718",5)){
		if(check_onetime){
			check_onetime = false;
			check_version = check_hardware_version(dev);
			TPD_ERR("%s:first check hardware version %d\n",__func__,check_version);
			if(check_version < 0){
				TPD_ERR("checkversion fail....\n");
				return -1;
			}
		}

		if(1 == check_version ) {
			TPD_DEBUG("enter version 15801 update mode\n");
			strcpy(ts->fw_name,"tp/fw_synaptics_15801.img");
			ret = request_firmware(&fw, ts->fw_name, dev);
			if (ret < 0) {
				TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
				return ret;
		       }

		 }else{
		        TPD_DEBUG("enter version 15801 vb update mode\n");
			ret = request_firmware(&fw, ts->fw_name, dev);
			if (ret < 0) {
				TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
				return ret;
		       }
		}

	}else if(!strncmp(ts->manu_name,"S3508",5) || !strncmp(ts->manu_name,"15811",5) || !strncmp(ts->manu_name,"s3508",5)){
		        TPD_ERR("enter version 15811 update mode\n");
			ret = request_firmware(&fw, ts->fw_name, dev);
			if (ret < 0) {
				TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
				return ret;
		       }
	}else{
		TPD_ERR("firmware name not match\n");
		return -1;
	}

	ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
	if(ret < 0){
		TPD_ERR("FW update not success try again\n");
		ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
		if(ret < 0){
			TPD_ERR("FW update failed twice, quit updating process!\n");
			return ret;
		}
	}
	release_firmware(fw);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	ret = synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL00, 4, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	sprintf(fw_id_temp,"0x%x",CURRENT_FIRMWARE_ID);
	strcpy(ts->fw_id,fw_id_temp);
	report_key_point_y = ts->max_y*button_map[2]/LCD_HEIGHT;
	synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts,1);
	return 0;
}

static ssize_t synaptics_update_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t synaptics_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (ts->screen_off && ts->support_hw_poweroff){
		TPD_ERR("power off firmware abort!\n");
		return size;
	}
	if(version_is_s3508){
		if (strncmp(ts->manu_name,"S3508",5) && strncmp(ts->manu_name,"15811",5) && strncmp(ts->manu_name,"s3508",5)){
        		TPD_ERR("product name[%s] do not update!\n",ts->manu_name);
        		return size;
   		 }
	}else{
    		if (strncmp(ts->manu_name,"S3718",5)){
        		TPD_ERR("product name[%s] do not update!\n",ts->manu_name);
        		return size;
   		 }
	}
	TPD_ERR("start update ******* fw_name:%s,ts->manu_name:%s\n",ts->fw_name,ts->manu_name);

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if(!val)
		val = force_update;

	touch_disable(ts);
	mutex_lock(&ts->mutex);
	ts->loading_fw = true;
	synatpitcs_fw_update(dev, val);
	ts->loading_fw = false;
	mutex_unlock(&ts->mutex);
	touch_enable(ts);
	force_update = 0;
	return size;
}
/*********************FW Update Func End*************************************/

static DEVICE_ATTR(tp_fw_update, 0664, synaptics_update_fw_show, synaptics_update_fw_store);
static int synaptics_dsx_pinctrl_init(struct synaptics_ts_data *ts);

static void tp_baseline_get(struct synaptics_ts_data *ts)
{
	if (!ts)
		return;

	mutex_lock(&ts->mutex);
	touch_disable(ts);
	TPD_DEBUG("%s start!\n",__func__);

	if (ts->gesture_enable)
		synaptics_enable_interrupt_for_gesture(ts, false);

	synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);//soft reset
	msleep(50);
	touch_enable(ts);
	synaptics_tpedge_limitfunc();
	mutex_unlock(&ts->mutex);
	TPD_DEBUG("%s end! \n",__func__);
}

static void tp_baseline_get_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts = ts_g;

	tp_baseline_get(ts);
}

static ssize_t touch_press_status_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int x,y;
	int press_points = 0;
	int points_misspresee =0;
	int str_n = 0;

	char *page = kzalloc(1024*2,GFP_KERNEL);
	if (!page){
		TPD_ERR("%s malloc memery error!",__func__);
		return -ENOMEM;
	}
	TPD_ERR("%s",__func__);

	for (x = 0; x < 8; x++){
		str_n += sprintf(&page[str_n], "\n");
		for (y = 0; y < 16; y++){
			str_n += sprintf(&page[str_n],"%4d",delta[x][y]);
			if ((delta[x][y] < -30) && (delta[x][y] > -250))
				press_points++;
			if((delta[x][y] > 30) && (delta[x][y] < 200))
				points_misspresee ++;
		}

	}

	if(points_misspresee > 4)
		get_tp_base = 0;
	TPD_ERR("points_mispressee num:%d,get_tp_base:%d\n",points_misspresee,get_tp_base);
	str_n += sprintf(&page[str_n], "\n%s %d points delta > [25]\n",(press_points>4)?"near":"away", press_points);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	kfree(page);
	return ret;
}

static ssize_t touch_press_status_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	return count;
}

static const struct file_operations touch_press_status = {
	.write = touch_press_status_write,
	.read =  touch_press_status_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static int init_synaptics_proc(void)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_tmp  = NULL;
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if( prEntry_tp == NULL ){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create touchpanel\n");
	}

#ifdef SUPPORT_GESTURE
	prEntry_tmp = proc_create( "gesture_enable", 0666, prEntry_tp, &tp_gesture_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create gesture_enable\n");
	}
	prEntry_tmp = proc_create( "gesture_switch", 0666, prEntry_tp, &gesture_switch_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create gesture_enable\n");
	}
	prEntry_tmp = proc_create("coordinate", 0444, prEntry_tp, &coordinate_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create coordinate\n");
	}

	prEntry_tmp = proc_create("double_tap_enable", 0666, prEntry_tp, &double_tap_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create double_tap_enable\n");
	}

	prEntry_tmp = proc_create("double_swipe_enable", 0666, prEntry_tp, &double_swipe_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create double_swipe_enable\n");
	}

	prEntry_tmp = proc_create("letter_o_enable", 0666, prEntry_tp, &letter_o_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create letter_o_enable\n");
	}

	prEntry_tmp = proc_create("left_arrow_enable", 0666, prEntry_tp, &left_arrow_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create left_arrow_enable\n");
	}

	prEntry_tmp = proc_create("right_arrow_enable", 0666, prEntry_tp, &right_arrow_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create right_arrow_enable\n");
	}

	prEntry_tmp = proc_create("up_arrow_enable", 0666, prEntry_tp, &up_arrow_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create up_arrow_enable\n");
	}

	prEntry_tmp = proc_create("down_arrow_enable", 0666, prEntry_tp, &down_arrow_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create down_arrow_enable\n");
	}

	prEntry_tmp = proc_create("left_swipe_enable", 0666, prEntry_tp, &left_swipe_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create left_swipe\n");
	}

	prEntry_tmp = proc_create("right_swipe_enable", 0666, prEntry_tp, &right_swipe_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create right_swipe_enable\n");
	}

	prEntry_tmp = proc_create("up_swipe_enable", 0666, prEntry_tp, &up_swipe_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create up_swipe_enable\n");
	}

	prEntry_tmp = proc_create("down_swipe_enable", 0666, prEntry_tp, &down_swipe_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create down_swipe_enable\n");
	}
#endif
	prEntry_tmp = proc_create("touch_press", 0666, prEntry_tp, &touch_press_status);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create touch_press\n");
	}
	return ret;
}
/******************************end****************************/

/****************************S3203*****update**********************************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

static void re_scan_PDT(struct i2c_client *client)
{
	uint8_t buf[8];
	i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
	SynaF34DataBase = buf[3];
	SynaF34QueryBase = buf[0];
	i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
	SynaF01DataBase = buf[3];
	SynaF01CommandBase = buf[1];
	i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);

	SynaF34Reflash_BlockNum = SynaF34DataBase;
	SynaF34Reflash_BlockData = SynaF34DataBase + 1;
	SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 1;
	SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 2;
	SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +3;
	SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 3;
	i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
	SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	TPD_DEBUG("SynaFirmwareBlockSize 3310 is %d\n", SynaFirmwareBlockSize);
	SynaF34_FlashControl = SynaF34DataBase + 2;
}
struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_contain_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char reserved_20_2f[16];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char ds_info[10];
	unsigned char reserved_4a_4f[6];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

static unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
		(unsigned int)ptr[1] * 0x100 +
		(unsigned int)ptr[2] * 0x10000 +
		(unsigned int)ptr[3] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);
	TPD_DEBUG(" debug checksume is %x", header->checksum);
	header->bootloader_version = data->bootloader_version;
	TPD_DEBUG(" debug bootloader_version is %d\n", header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_DEBUG(" debug firmware_size is %x", header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_DEBUG(" debug header->config_size is %x", header->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	TPD_DEBUG(" debug header->contains_firmware_id is %x\n", header->contains_firmware_id);
	if( header->contains_firmware_id )
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}

static int checkFlashState(struct i2c_client *client)
{
	int ret ;
	int count = 0;
	ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl+1);
	while ( (ret != 0x80)&&(count < 8) ) {
		msleep(3); //wait 3ms
		ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl+1);
		count++;
	}
	if(count == 8)
		return 1;
	else
		return 0;
}

static int synaptics_fw_check(struct synaptics_ts_data *ts )
{
	int ret;
	uint8_t buf[4];
	uint32_t bootloader_mode;
	int max_y_ic = 0;
	int max_x_ic = 0;
	if(!ts){
		TPD_ERR("%s ts is NULL\n",__func__);
		return -1;
	}

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}

	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPD_ERR("failed to read product info \n");
		return -1;
	}
	/*read max_x ,max_y*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if (ret < 0) {
		ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
		if(ret < 0 ){
			TPD_ERR("synaptics_rmi4_i2c_write_byte failed for page select\n");
			return -1;
		}
	}

	i2c_smbus_read_i2c_block_data(ts->client, F12_2D_CTRL08, 14, buf);
	max_x_ic = ( (buf[1]<<8)&0xffff ) | (buf[0]&0xffff);
	max_y_ic = ( (buf[3]<<8)&0xffff ) | (buf[2]&0xffff);

	TPD_ERR("max_x = %d,max_y = %d; max_x_ic = %d,max_y_ic = %d\n",ts->max_x,ts->max_y,max_x_ic,max_y_ic);
	if((ts->max_x == 0) ||(ts->max_y ==0 )) {
		ts->max_x = max_x_ic;
		ts->max_y = max_y_ic;
	}
	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40;
	TPD_DEBUG("afte fw update,program memory self-check bootloader_mode = 0x%x\n",bootloader_mode);

	if((max_x_ic == 0)||(max_y_ic == 0)||(bootloader_mode == 0x40)) {
		TPD_ERR("Something terrible wrong \n Trying Update the Firmware again\n");
		return -1;
	}
	return 0;
}

static void re_scan_PDT_s3508(struct i2c_client *client)
{
    uint8_t buf[8];
    i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
    SynaF34DataBase = buf[3];
    SynaF34QueryBase = buf[0];
    i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
    SynaF01DataBase = buf[3];
    SynaF01CommandBase = buf[1];
    i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);

    SynaF34Reflash_BlockNum = SynaF34DataBase;
    SynaF34Reflash_BlockData = SynaF34DataBase + 1;
    SynaF34ReflashQuery_BootID = SynaF34QueryBase;
    SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 1;
    SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 2;
    SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +3;
    SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
    SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 3;
    i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
    SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
    TPD_DEBUG("SynaFirmwareBlockSize 3310 is %d\n", SynaFirmwareBlockSize);
    SynaF34_FlashControl = SynaF34DataBase + 2;
}

static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force)
{
	int ret,j;
	uint8_t buf[8];
	uint8_t bootloder_id[10];
	uint16_t block,firmware,configuration;
	uint32_t CURRENT_FIRMWARE_ID = 0 , FIRMWARE_ID = 0;
	const uint8_t *Config_Data = NULL;
	const uint8_t *Firmware_Data = NULL;
	struct image_header_data header;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);
	TPD_DEBUG("%s is called\n",__func__);
	if(!client)
		return -1;
	if (!strncmp(ts->manu_name,"S3718",5)){
		Config_Data = data + 0x8f0;
		ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);
		ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL00, 4, buf);
		CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
		FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];
		if(1 == check_version)
			TPD_ERR("15801CURRENT_FW_ID:%x----, FW_ID:%x----,FW_NAME:%s\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID,ts->fw_name);
		else
			TPD_ERR("15801CURRENT_FW_ID:%xvB----, FW_ID:%xvB----,FW_NAME:%s\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID,ts->fw_name);
		//TPD_ERR("synaptics force is %d\n", force);
		if(!force) {
			if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
				return 0;
			}
		}
		ret = fwu_start_reflash(data,client);
		if (ret){
			return -1;
		}
	}else if(!strncmp(ts->manu_name,"S3508",5) || !strncmp(ts->manu_name,"15811",5) || !strncmp(ts->manu_name,"s3508",5)){
		parse_header(&header,data);
		if((header.firmware_size + header.config_size + 0x100) > data_len) {
			TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
			return -1;
		}
		Firmware_Data = data + 0x100;
		Config_Data = Firmware_Data + header.firmware_size;
		ret = i2c_smbus_write_byte_data(client, 0xff, 0x0);

		ret = i2c_smbus_read_i2c_block_data(client, F34_FLASH_CTRL00, 4, buf);
		CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
		FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];
		TPD_ERR("15811CURRENT_FW_ID:%x----, FW_ID:%x----,FW_NAME:%s\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID,ts->fw_name);
		TPD_ERR("synaptics force is %d\n", force);
		if(!force) {
			if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
				return 0;
			}
		}
		re_scan_PDT_s3508(client);
		block = 16;
		TPD_DEBUG("block is %d \n",block);
		firmware = (header.firmware_size)/16;
		TPD_DEBUG("firmware is %d \n",firmware);
		configuration = (header.config_size)/16;
		TPD_DEBUG("configuration is %d \n",configuration);

		ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));
		TPD_DEBUG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);
		ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
		TPD_DEBUG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);

		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x0F);
		msleep(10);
		TPD_DEBUG("attn step 4\n");
		ret=checkFlashState(client);
		if(ret > 0) {
			TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
				return -1;
		}
		ret = i2c_smbus_read_byte_data(client,0x04);
		TPD_DEBUG("The status(device state) is %x\n",ret);
		ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
		TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
		ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret&0x04);
		/********************get into prog end************/
		ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
		TPD_DEBUG("ret is %d\n",ret);
		re_scan_PDT_s3508(client);
		i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
		i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
		msleep(2500);
		ret = i2c_smbus_read_byte_data(client, SynaF34_FlashControl);
		if(ret != 0x00)
			msleep(2000);
		ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl+1);
		TPDTM_DMESG("The status(erase) is %x\n",ret);
		TPD_ERR("15811update-----------------update------------------update!\n");
		TPD_DEBUG("cnt %d\n",firmware);
		for(j=0; j<firmware; j++) {
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Firmware_Data[j*16]);

			i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x02);
			ret=checkFlashState(client);
			if(ret > 0) {
				TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
				return -1;
			}
		}
		//step 7 configure data
		//TPD_ERR("going to flash configuration area\n");
		//TPD_ERR("header.firmware_size is 0x%x\n", header.firmware_size);
		//TPD_ERR("bootloader_size is 0x%x\n", bootloader_size);
		for(j=0;j<configuration;j++) {
			//a)write SynaF34Reflash_BlockNum to access
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
			//b) write data

				i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Config_Data[j*16]);

			//c) issue write
			i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x06);
			//d) wait attn
			ret = checkFlashState(client);
			if(ret > 0) {
				TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
				return -1;
			}
		}
		//step 1 issue reset
		i2c_smbus_write_byte_data(client,SynaF01CommandBase,0X01);
	}else{
		parse_header(&header,data);
		if((header.firmware_size + header.config_size + 0x100) > data_len) {
			TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
			return -1;
		}

		Firmware_Data = data + 0x100;
		Config_Data = Firmware_Data + header.firmware_size;
		ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);

		ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL00, 4, buf);
		CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
		FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];

		//TPD_ERR("synaptics force is %d\n", force);
		if(!force) {
			if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
				return 0;
			}
		}
		re_scan_PDT(client);
		block = 16;
		TPD_DEBUG("block is %d \n",block);
		firmware = (header.firmware_size)/16;
		TPD_DEBUG("firmware is %d \n",firmware);
		configuration = (header.config_size)/16;
		TPD_DEBUG("configuration is %d \n",configuration);


		ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));
		TPD_DEBUG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);
		ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
		TPDTM_DMESG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);

		synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x0F);
		msleep(10);
		TPD_DEBUG("attn step 4\n");
		ret=checkFlashState(client);
		if(ret > 0) {
			TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
			return -1;
		}
		ret = i2c_smbus_read_byte_data(client,0x04);
		TPD_DEBUG("The status(device state) is %x\n",ret);
		ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
		TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
		ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret&0x04);
		/********************get into prog end************/
		ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
		TPD_DEBUG("ret is %d\n",ret);
		re_scan_PDT(client);
		i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
		i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
		msleep(2000);
		ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
		TPDTM_DMESG("going to flash firmware area synaF34_FlashControl %d\n",ret);

		TPD_ERR("update-----------------firmware ------------------update!\n");
		TPD_DEBUG("cnt %d\n",firmware);
		for(j=0; j<firmware; j++) {
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockNum,2,buf);
			synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockData,16,&Firmware_Data[j*16]);
			synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x02);
			ret=checkFlashState(client);
			if(ret > 0) {
				TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
				return -1;
			}
		}
		//step 7 configure data
		//TPD_ERR("going to flash configuration area\n");
		//TPD_ERR("header.firmware_size is 0x%x\n", header.firmware_size);
		//TPD_ERR("bootloader_size is 0x%x\n", bootloader_size);
		TPD_ERR("update-----------------configuration ------------------update!\n");
		for(j=0;j<configuration;j++) {
			//a)write SynaF34Reflash_BlockNum to access
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockNum,2,buf);
			//b) write data
			synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockData,16,&Config_Data[j*16]);
			//c) issue write
			synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x06);
			//d) wait attn
			ret = checkFlashState(client);
			if(ret > 0) {
				TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
				return -1;
			}
		}

		//step 1 issue reset
		synaptics_rmi4_i2c_write_byte(client,SynaF01CommandBase,0x01);
	}
	//step2 wait ATTN
	//msleep(1000);
	mdelay(1500);
	synaptics_read_register_map(ts);
	//FW flash check!
	ret =synaptics_fw_check(ts);
	if(ret < 0 ) {
		TPD_ERR("Firmware self check failed\n");
		return -1;
	}
	TPD_ERR("Firmware self check Ok\n");
	return 0;
}

static void synaptics_tpedge_limitfunc(void)
{
	int limit_mode=0;
	int ret;

	if(version_is_s3508)
		F51_CUSTOM_CTRL74 = 0x0437;
	else
		F51_CUSTOM_CTRL74 = 0x044D;
	msleep(60);
        ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x4);
        limit_mode = i2c_smbus_read_byte_data(ts_g->client, F51_CUSTOM_CTRL74);
        TPD_ERR("%s limit_enable =%d,mode:0x%x !\n", __func__,limit_enable,limit_mode);
	if(limit_mode){
		i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x4);
		if(0 == limit_enable)
		{
			if(limit_mode & 0x1){
				//TPD_ERR("000 limit_enable:0x%xs  !\n",limit_mode);
				limit_mode = limit_mode & 0xFE;
				ret = i2c_smbus_write_byte_data(ts_g->client, F51_CUSTOM_CTRL74, limit_mode);
			}
		}
		else if(1 == limit_enable)
		{
			if(!(limit_mode & 0x1)){
				//TPD_ERR("111 limit_enable:x%xs  !\n",limit_mode);
				limit_mode = limit_mode | 0x1;
				ret = i2c_smbus_write_byte_data(ts_g->client, F51_CUSTOM_CTRL74, limit_mode);
			}
		}
	}
	i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x0);
}

static int synaptics_soft_reset(struct synaptics_ts_data *ts)
{
	int ret;
	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!\n",__func__);
		return -1;
	}
	touch_disable(ts);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
	if (ret < 0){
		TPD_ERR("reset error ret=%d\n",ret);
	}
	TPD_ERR("%s !!!\n",__func__);
	msleep(100);
	touch_enable(ts);
	synaptics_tpedge_limitfunc();
	return ret;
}
static void synaptics_hard_reset(struct synaptics_ts_data *ts)
{
    if(ts->reset_gpio > 0)
    {
        gpio_set_value(ts->reset_gpio,0);
        msleep(5);
        gpio_set_value(ts->reset_gpio,1);
        msleep(100);
        TPD_ERR("%s !!!\n",__func__);
    }

}
static int synaptics_parse_dts(struct device *dev, struct synaptics_ts_data *ts)
{
	int rc;
	struct device_node *np;
	int temp_array[2];


	np = dev->of_node;
	ts->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, &(ts->irq_flags));
	if( ts->irq_gpio < 0 ){
		TPD_DEBUG("ts->irq_gpio not specified\n");
	}

	ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);
	if( ts->reset_gpio < 0 ){
		TPD_DEBUG("ts->reset-gpio  not specified\n");
	}
	ts->v1p8_gpio = of_get_named_gpio(np, "synaptics,1v8-gpio", 0);
	if( ts->v1p8_gpio < 0 ){
		TPD_DEBUG("ts->1v8-gpio  not specified\n");
	}

	if(of_property_read_bool(np, "oem,support_hw_poweroff"))
		ts->support_hw_poweroff=true;
	else
		ts->support_hw_poweroff=false;

	TPD_ERR("%s ts->support_hw_poweroff =%d\n",__func__,ts->support_hw_poweroff);

	ts->enable2v8_gpio = of_get_named_gpio(np, "synaptics,enable2v8-gpio", 0);
	if( ts->enable2v8_gpio < 0 ){
		TPD_DEBUG("ts->enable2v8_gpio not specified\n");
	}

	rc = of_property_read_u32(np, "synaptics,max-num-support", &ts->max_num);
	if(rc){
		TPD_DEBUG("ts->max_num not specified\n");
		ts->max_num = 10;
	}

	rc = of_property_read_u32_array(np, "synaptics,button-map", button_map, 3);
	if(rc){
		TPD_DEBUG("button-map not specified\n");
		//button_map[0] = 180;
		//button_map[1] = 180;
		//button_map[2] = 2021;
	}
	TPD_DEBUG("synaptics:button map readed is %d %d %d\n", button_map[0], button_map[1], button_map[2]);

	rc = of_property_read_u32_array(np, "synaptics,tx-rx-num", tx_rx_num,2);
	if(rc){
		TPD_ERR("button-map not specified\n");
		TX_NUM =  30;
		RX_NUM =  17;
	}else{
		TX_NUM =  tx_rx_num[0];
		RX_NUM =  tx_rx_num[1];
	}
	TPD_ERR("synaptics,tx-rx-num is %d %d \n", TX_NUM,RX_NUM);

	rc = of_property_read_u32_array(np, "synaptics,display-coords", temp_array, 2);
	if(rc){
		TPD_ERR("lcd size not specified\n");
		LCD_WIDTH = 1080;
		LCD_HEIGHT = 1920;
	}else{
		LCD_WIDTH = temp_array[0];
		LCD_HEIGHT = temp_array[1];
	}
	rc = of_property_read_u32_array(np, "synaptics,panel-coords", temp_array, 2);
	if(rc){
		ts->max_x = 1080;
		ts->max_y = 1920;
	}else{
		ts->max_x = temp_array[0];
		ts->max_y = temp_array[1];
	}
    TPDTM_DMESG("synaptic:ts->irq_gpio:%d irq_flags:%u max_num %d\n"\
        ,ts->irq_gpio, ts->irq_flags, ts->max_num);

	/***********power regulator_get****************/
	ts->vdd_2v8 = regulator_get(&ts->client->dev, "vdd_2v8");
	if( IS_ERR(ts->vdd_2v8) ){
		rc = PTR_ERR(ts->vdd_2v8);
		TPD_DEBUG("Regulator get failed vdd rc=%d\n", rc);
	}

	ts->vcc_i2c_1v8 = regulator_get(&ts->client->dev, "vcc_i2c_1v8");
	if( IS_ERR(ts->vcc_i2c_1v8) ){
		rc = PTR_ERR(ts->vcc_i2c_1v8);
		TPD_DEBUG("Regulator get failed vcc_i2c rc=%d\n", rc);
	}

	if( ts->reset_gpio > 0){
		if( gpio_is_valid(ts->reset_gpio) ){
			rc = gpio_request(ts->reset_gpio, "tp-s3320-reset");
			if(rc){
				TPD_ERR("unable to request reset_gpio [%d]\n", ts->reset_gpio);
			}
			gpio_direction_output(ts->reset_gpio, 0);
		}
	}
	if( ts->v1p8_gpio > 0){
		if( gpio_is_valid(ts->v1p8_gpio) ){
			rc = gpio_request(ts->v1p8_gpio, "tp-s3320-1v8");
			if(rc){
				TPD_ERR("unable to request v1p8_gpio [%d]\n", ts->v1p8_gpio);
			}
		}
	}

	if( ts->enable2v8_gpio > 0){
		if( gpio_is_valid(ts->enable2v8_gpio) ){
			rc = gpio_request(ts->enable2v8_gpio, "rmi4-enable2v8-gpio");
			if(rc)
				TPD_ERR("unable to request enable2v8_gpio [%d]\n", ts->enable2v8_gpio);

		}
	}

	return rc;
}

static int synaptics_dsx_pinctrl_init(struct synaptics_ts_data *ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts->pinctrl = devm_pinctrl_get((ts->dev));
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		retval = PTR_ERR(ts->pinctrl);
        TPD_ERR("%s pinctrl error!\n",__func__);
		goto err_pinctrl_get;
	}

	ts->pinctrl_state_active
		= pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_active)) {
		retval = PTR_ERR(ts->pinctrl_state_active);
        TPD_ERR("%s pinctrl state active error!\n",__func__);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_state_suspend
		= pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_suspend)) {
		retval = PTR_ERR(ts->pinctrl_state_suspend);
        TPD_ERR("%s pinctrl state suspend error!\n",__func__);
		goto err_pinctrl_lookup;
	}
	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ts->pinctrl);
err_pinctrl_get:
	ts->pinctrl = NULL;
	return retval;
}

static void synaptics_suspend_resume(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, typeof(*ts), pm_work);

	mutex_lock(&ts->mutex);
	if (ts->screen_off) {
		int i;

		touch_disable(ts);
		ts->touch_active = false;

		for (i = 0; i < 10; i++) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
		input_sync(ts->input_dev);

		if (ts->gesture_enable) {
			synaptics_enable_interrupt_for_gesture(ts, true);
			touch_enable(ts);
		} else {
			if (ts->support_hw_poweroff)
				tpd_power(ts, 0);
		}
	} else {
		if (ts->gesture_enable) {
			touch_disable(ts);
			synaptics_enable_interrupt_for_gesture(ts, false);
		} else {
			if (ts->support_hw_poweroff)
				tpd_power(ts, 1);
		}
		touch_enable(ts);
	}
	mutex_unlock(&ts->mutex);
}

#ifdef SUPPORT_VIRTUAL_KEY
#define VK_KEY_X    180
#define VK_CENTER_Y 2020//2260
#define VK_WIDTH    170
#define VK_HIGHT    200
static ssize_t vk_syna_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
    int len ;

    len =  sprintf(buf,
            __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT)  ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":%d:%d:%d:%d" "\n",
            VK_KEY_X,   VK_CENTER_Y, VK_WIDTH, VK_HIGHT,
            VK_KEY_X*3, VK_CENTER_Y, VK_WIDTH, VK_HIGHT,
            VK_KEY_X*5, VK_CENTER_Y, VK_WIDTH, VK_HIGHT);

    return len ;
}

static struct kobj_attribute vk_syna_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &vk_syna_show,
};

static struct attribute *syna_properties_attrs[] = {
    &vk_syna_attr.attr,
    NULL
};

static struct attribute_group syna_properties_attr_group = {
    .attrs = syna_properties_attrs,
};
static int synaptics_ts_init_virtual_key(struct synaptics_ts_data *ts )
{
    int ret = 0;

    /* virtual keys */
    if(ts->properties_kobj)
        return 0 ;
    ts->properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (ts->properties_kobj)
        ret = sysfs_create_group(ts->properties_kobj, &syna_properties_attr_group);

    if (!ts->properties_kobj || ret)
        printk("%s: failed to create board_properties\n", __func__);
    /* virtual keys */
    return ret;
}
#endif

static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts = NULL;
	int ret = -1;
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0;
	uint32_t bootloader_mode;

	TPD_ERR("%s  is called\n",__func__);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->dev = &client->dev;
	ts->loading_fw = false;
	ts->support_ft = true;
	ts_g = ts;
	get_tp_base = 0;

	ret = synaptics_parse_dts(&client->dev, ts);
	if (ret < 0)
		goto err_alloc_data_failed;

	/***power_init*****/
	ret = synaptics_dsx_pinctrl_init(ts);
	if (ret < 0)
		goto err_alloc_data_failed;

	tpd_power(ts, 1);

    msleep(100);//after power on tp need sometime from bootloader to ui mode
	mutex_init(&ts->mutex);

    spin_lock_init(&ts->lock);
	/*****power_end*********/
	if( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ){
		TPD_ERR("%s [ERR]need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ret = synaptics_rmi4_i2c_read_byte(client, 0x13);
	if( ret < 0 ) {
		ret = synaptics_rmi4_i2c_read_byte(client, 0x13);
		if( ret < 0 ) {
                        TPD_ERR("tp is no exist!\n");
			goto err_check_functionality_failed;
		}
	}

	synaptics_read_register_map(ts);
	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE);

	bootloader_mode = bootloader_mode&0x40;
	TPD_ERR("before fw update bootloader_mode[0x%x]\n", bootloader_mode);

	synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL00, 4, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
	TPD_ERR("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);
	TP_FW = CURRENT_FIRMWARE_ID;
	sprintf(ts->fw_id,"0x%x",TP_FW);

	memset(ts->fw_name, 0, TP_FW_NAME_MAX_LEN);
	memset(ts->test_limit_name, 0, TP_FW_NAME_MAX_LEN);

	//sprintf(ts->manu_name, "TP_SYNAPTICS");
	synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY11,10, ts->manu_name);
	if (!strncmp(ts->manu_name,"S3718",5)){
		strcpy(ts->fw_name,"tp/fw_synaptics_15801b.img");
		version_is_s3508 = 0;
	}else{
		strcpy(ts->fw_name,"tp/fw_synaptics_15811.img");
		version_is_s3508 = 1;
	}

	strcpy(ts->test_limit_name,"tp/14049/14049_Limit_jdi.img");
	TPD_DEBUG("0synatpitcs_fw: fw_name = %s,ts->manu_name:%s \n",ts->fw_name,ts->manu_name);

	INIT_WORK(&ts->base_work, tp_baseline_get_work);

	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		TPD_ERR("synaptics_init_panel failed\n");
	}

	//Detect whether TP FW is error, max_x,max_y may be incoorect while it has been damaged!
	ret = synaptics_fw_check(ts);
	if(ret < 0 ) {
		force_update = 1;
		TPD_ERR("This FW need to be updated!\n");
	} else {
		force_update = 0;
	}
	/*disable interrupt*/
	ret = synaptics_enable_interrupt(ts, 0);
	if( ret < 0 ) {
		TPD_ERR(" synaptics_ts_probe: disable interrupt failed\n");
	}
    ret = synaptics_soft_reset(ts);
    if (ret < 0){
        TPD_ERR("%s faile to reset device\n",__func__);
    }
	ret = synaptics_input_init(ts);
	if(ret < 0) {
		TPD_ERR("synaptics_input_init failed!\n");
	}

	INIT_WORK(&ts->pm_work, synaptics_suspend_resume);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret)
		TPD_ERR("Unable to register fb_notifier: %d\n", ret);

	init_completion(&ts->i2c_resume);
	spin_lock_init(&ts->isr_lock);
	wakeup_source_init(&ts->syna_isr_ws, "synaptics-isr");
	ts->i2c_awake = true;

	/****************
	  shoud set the irq GPIO
	 *******************/
	if (gpio_is_valid(ts->irq_gpio)) {
        /* configure touchscreen irq gpio */
        ret = gpio_request(ts->irq_gpio,"tp-s3320-irq");
        if (ret) {
            TPD_ERR("unable to request gpio [%d]\n",ts->irq_gpio);
        }
        ret = gpio_direction_input(ts->irq_gpio);
        msleep(50);
        ts->irq = gpio_to_irq(ts->irq_gpio);
	}
	TPD_ERR("synaptic:ts->irq is %d\n",ts->irq);

	ret = request_threaded_irq(ts->irq, NULL,
			synaptics_irq_thread_fn,
			ts->irq_flags | IRQF_ONESHOT,
			TPD_DEVICE, ts);
	if(ret < 0)
		TPD_ERR("%s request_threaded_irq ret is %d\n",__func__,ret);
    msleep(5);
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret < 0)
		TPD_ERR("%s enable interrupt error ret=%d\n",__func__,ret);

	if (device_create_file(&client->dev, &dev_attr_tp_fw_update)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}

#ifdef SUPPORT_VIRTUAL_KEY
    synaptics_ts_init_virtual_key(ts);
#endif
	init_synaptics_proc();
	TPDTM_DMESG("synaptics_ts_probe 3203: normal end\n");
	return 0;

exit_init_failed:
	free_irq(client->irq,ts);
err_check_functionality_failed:
	tpd_power(ts, 0);
err_alloc_data_failed:
	tpd_i2c_driver.driver.pm=NULL;
	kfree(ts);
	TPD_ERR("synaptics_ts_probe: not normal end\n");
	return ret;
}

static int synaptics_i2c_suspend(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&ts->isr_lock, flags);
	ts->i2c_awake = false;
	spin_unlock_irqrestore(&ts->isr_lock, flags);

	if (ts->gesture_enable)
		enable_irq_wake(ts->irq);

	return 0;
}

static int synaptics_i2c_resume(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&ts->isr_lock, flags);
	ts->i2c_awake = true;
	spin_unlock_irqrestore(&ts->isr_lock, flags);

	complete(&ts->i2c_resume);

	if (ts->gesture_enable)
		disable_irq_wake(ts->irq);

	return 0;
}

static int synaptics_mode_change(int mode)
{
	int ret;
    int tmp_mode;
    tmp_mode = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_CTRL00);
    tmp_mode = tmp_mode & 0xF8;//bit0-bit2(mode)
    tmp_mode = tmp_mode | mode;
        tmp_mode = tmp_mode & 0xDF;//clear bit6(change status)
    TPD_DEBUG("%s: set TP to mode[0x%x]\n", __func__,tmp_mode);
	ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00, tmp_mode);
	if(ret<0)
		TPD_ERR("%s: set dose mode[0x%x] err!!\n", __func__,tmp_mode);
	return ret;
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct synaptics_ts_data *ts =
		container_of(self, struct synaptics_ts_data, fb_notif);
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	if (event != FB_EARLY_EVENT_BLANK)
		return NOTIFY_OK;

	switch (*blank) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (ts->screen_off) {
			cancel_work_sync(&ts->pm_work);
			ts->screen_off = 0;
			queue_work(system_highpri_wq, &ts->pm_work);
		}
		break;
	case FB_BLANK_POWERDOWN:
		if (!ts->screen_off) {
			cancel_work_sync(&ts->pm_work);
			ts->screen_off = 1;
			queue_work(system_highpri_wq, &ts->pm_work);
		}
		break;
	}

	return NOTIFY_OK;
}

static struct delayed_work init_work;

static void tpd_init_worker(struct work_struct *work)
{
	if (i2c_add_driver(&tpd_i2c_driver))
		TPD_ERR("unable to add i2c driver.\n");
}

static int __init tpd_driver_init(void)
{
	INIT_DELAYED_WORK(&init_work, tpd_init_worker);
	schedule_delayed_work(&init_work, msecs_to_jiffies(1000));
	return 0;
}
device_initcall(tpd_driver_init);
