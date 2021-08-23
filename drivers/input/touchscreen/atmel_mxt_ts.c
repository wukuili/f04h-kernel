/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2014 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
/*----------------------------------------------------------------------------*/

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/platform_data/atmel_mxt_ts.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/pinctrl/consumer.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include "touch_debug.h"

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE	255

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_PROCI_ACTIVE_STYLUS_T63	63
#define MXT_SPT_DYNAMICCONFIGRATIONCONTROLLER_T70	70
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71 71
#define MXT_PROCI_SYMBOLGESTUREPROCESSOR	92
#define MXT_PROCI_TOUCHSEQUENCELOGGER	93
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 100
#define MXT_PROCI_ACTIVESTYLUS_T107	107

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	BIT(7)
#define MXT_T6_STATUS_OFL	BIT(6)
#define MXT_T6_STATUS_SIGERR	BIT(5)
#define MXT_T6_STATUS_CAL	BIT(4)
#define MXT_T6_STATUS_CFGERR	BIT(3)
#define MXT_T6_STATUS_COMSERR	BIT(2)

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_CTRL		0
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		BIT(0)
#define MXT_T9_SUPPRESS		BIT(1)
#define MXT_T9_AMP		BIT(2)
#define MXT_T9_VECTOR		BIT(3)
#define MXT_T9_MOVE		BIT(4)
#define MXT_T9_RELEASE		BIT(5)
#define MXT_T9_PRESS		BIT(6)
#define MXT_T9_DETECT		BIT(7)

struct t9_range {
	u16 x;
	u16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	BIT(0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN      BIT(6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_RESTORE_STATIC_VALUE	0x33
#define MXT_BACKUP_VALUE	0x55

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	BIT(0)

/* T63 Stylus */
#define MXT_T63_STYLUS_PRESS	BIT(0)
#define MXT_T63_STYLUS_RELEASE	BIT(1)
#define MXT_T63_STYLUS_MOVE		BIT(2)
#define MXT_T63_STYLUS_SUPPRESS	BIT(3)

#define MXT_T63_STYLUS_DETECT	BIT(4)
#define MXT_T63_STYLUS_TIP		BIT(5)
#define MXT_T63_STYLUS_ERASER	BIT(6)
#define MXT_T63_STYLUS_BARREL	BIT(7)

#define MXT_T63_STYLUS_PRESSURE_MASK	0x3F

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_SCRAUX		2
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	BIT(5)

#define MXT_T100_TCHAUX_VECT	BIT(0)
#define MXT_T100_TCHAUX_AMPL	BIT(1)
#define MXT_T100_TCHAUX_AREA	BIT(2)

#define MXT_T100_SCRAUX_NUMRPT		BIT(0)
#define MXT_T100_SCRAUX_TCHAREA		BIT(1)
#define MXT_T100_SCRAUX_ATCHAREA	BIT(2)
#define MXT_T100_SCRAUX_INTTHRAREA	BIT(3)

#define MXT_T100_DETECT		BIT(7)
#define MXT_T100_TYPE_MASK	0x70

enum t100_type {
	MXT_T100_TYPE_FINGER		= 1,
	MXT_T100_TYPE_PASSIVE_STYLUS	= 2,
	MXT_T100_TYPE_ACTIVE_STYLUS	= 3,
	MXT_T100_TYPE_HOVERING_FINGER	= 4,
	MXT_T100_TYPE_GLOVE		= 5,
	MXT_T100_TYPE_LARGE_TOUCH	= 6,
};

#define MXT_T100_EVENT_MASK	0x0F

enum t100_event {
	MXT_T100_EVENT_MOVE = 1,
	MXT_T100_EVENT_DOWN = 4,
	MXT_T100_EVENT_UP = 5,
	/* Don't care about other events */
};

#define MXT_DISTANCE_ACTIVE_TOUCH	0
#define MXT_DISTANCE_HOVERING		1

#define MXT_TOUCH_MAJOR_DEFAULT		1
#define MXT_PRESSURE_DEFAULT		5

/* Gen2 Active Stylus */
#define MXT_T107_STYLUS_STYAUX		42
#define MXT_T107_STYLUS_STYAUX_PRESSURE	BIT(0)
#define MXT_T107_STYLUS_STYAUX_PEAK	BIT(4)

#define MXT_T107_STYLUS_HOVER		BIT(0)
#define MXT_T107_STYLUS_TIPSWITCH	BIT(1)
#define MXT_T107_STYLUS_BUTTON0		BIT(2)
#define MXT_T107_STYLUS_BUTTON1		BIT(3)

/* Dynamic configuration control */
#define MXT_T70_CTRL		0

#define MXT_T70_CTRL_ENABLE	BIT(0)
#define MXT_FRCCMD_FRCRISE	(0x01 << 3)
#define MXT_FRCCMD_FRCFALL	(0x02 << 3)

#define MXT_T70_INSTANCE_GLOVE	0

/* Wake up gesture */
#define MXT_T93_STATUS_DBLTAP	BIT(1)

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	3000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_CHG_DELAY	        90	/* msec */
#define MXT_POWERON_DELAY	300	/* msec */
#define MXT_BOOTLOADER_WAIT	36E5	/* 1 minute */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	BIT(5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0x40

#define MXT_PIXELS_PER_MM	20

#define DEBUG_MSG_MAX		200

#define MXT_TOUCH_MODE_ENABLE	1
#define MXT_TOUCH_MODE_DISABLE	0

/* Wake up gesture parameter */
#define MXT_GESTURE_OFF        0
#define MXT_GESTURE_DOUBLE_TAP BIT(0)
#define MXT_GESTURE_ALL        (MXT_GESTURE_DOUBLE_TAP)

enum mxt_wakeup_gesture_state {
	MXT_GESTURE_STATE_DISABLE,
	MXT_GESTURE_STATE_ENABLE,
	MXT_GESTURE_STATE_EXIT,
	MXT_GESTURE_STATE_SLEEP,
};

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

/* Firmware frame structure */
struct mxt_fw_frame {
	__be16 size;
	u8 data[];
};

/* Firmware update context */
struct mxt_flash {
	struct mxt_data *data;
	const struct firmware *fw;
	struct mxt_fw_frame *frame;
	loff_t pos;
	size_t frame_size;
	unsigned int count;
	unsigned int retry;
	u8 previous;
	struct completion flash_completion;
	struct delayed_work work;
};

struct wakeup_gesture_param {
	u8 obj_id;
	u16 obj_addr;
	u8 obj_offset;
	u8 data_size;
	u8 *active_data;
	u8 *default_data;
	struct wakeup_gesture_param *next;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 bootloader_addr;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 multitouch;
	struct t7_config t7_cfg;
	u8 num_stylusids;
	unsigned long t15_keystatus;
	u8 stylus_aux_pressure;
	u8 stylus_aux_peak;
	bool use_retrigen_workaround;
	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	char *fw_name;
	char *cfg_name;
	struct mxt_flash *flash;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T71_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u8 T48_reportid;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
	u16 T70_address;
	u8 T70_size;
	u8 T70_instances;
	u8 *T70_cfg;
	u16 T92_address;
	u8 T92_reportid;
	u16 T93_address;
	u8 T93_reportid;
	u8 T100_reportid_min;
	u8 T100_reportid_max;
	u16 T107_address;

	/* for reset handling */
	struct completion reset_completion;

	/* for config update handling */
	struct completion crc_completion;

	/* Indicates whether device is in suspend */
	bool suspended;

	/* Indicates whether device is updating configuration */
	bool updating_config;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	void *touch_debug;

	/* for async start */
	struct delayed_work async_start_work;
	struct workqueue_struct *async_start_queue;

	/* Screen status */
	u8 t100_scr_numrpt;
	u8 t100_scr_tcharea;
	u8 t100_scr_atcharea;
	u8 t100_scr_intthrarea;
	u8 scraux_data[8];

	/* Touchscreen Class */
	struct class *setting_class;
	dev_t setting_dev_num;
	struct device *touch_mode_dev_path;

	/* Dynamic configuration control */
	bool objects_configured;
	bool glove_mode;

	/* Wake up gesture */
	struct mutex touchmode_lock;
	u8 wakeup_gesture_mode;
	enum mxt_wakeup_gesture_state wg_state;
	u8 wg_param_count;
	struct wakeup_gesture_param *wg_params;
};

extern int charging_mode;

#if defined(CONFIG_FB) || defined(CONFIG_PM_SLEEP)
static int __maybe_unused mxt_suspend(struct device *dev);
static int __maybe_unused mxt_resume(struct device *dev);
#endif
static void mxt_reset_slots(struct mxt_data *data);
static int mxt_set_glove_mode(struct mxt_data *data, u8 enable);
static int mxt_set_wakeup_gesture_mode(struct mxt_data *data, bool enable);


static size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	dev_dbg(&data->client->dev, "MXT MSG: %*ph\n",
		       data->T5_msg_size, message);
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data)
		return;

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_dbg(dev, "Enabled message output\n");
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_v2_enabled)
		return;

	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_dbg(dev, "Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data +
		       data->debug_msg_count * data->T5_msg_size,
		       msg,
		       data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		dev_dbg(dev, "Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{
	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = S_IWUSR | S_IRUGO;
	data->debug_msg_attr.read = mxt_debug_msg_read;
	data->debug_msg_attr.write = mxt_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&data->client->dev.kobj,
				  &data->debug_msg_attr) < 0) {
		dev_err(&data->client->dev, "Failed to create %s\n",
			data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->client->dev.kobj,
				      &data->debug_msg_attr);
}

static int mxt_wait_for_completion(struct mxt_data *data,
				   struct completion *comp,
				   unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		return ret;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data,
			       u8 *val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&data->client->dev, "%s: i2c recv failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int mxt_bootloader_write(struct mxt_data *data,
				const u8 * const val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&data->client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, bool retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = data->info ? data->info->family_id : 0;

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if (retry || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;

	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, bool alt_address)
{
	struct device *dev = &data->client->dev;
	int error;
	u8 buf[3];
	bool crc_failure, extended_id;

	error = mxt_lookup_bootloader_address(data, alt_address);
	if (error)
		return error;

	/* Check bootloader status and version information */
	error = mxt_bootloader_read(data, buf, sizeof(buf));
	if (error)
		return error;

	crc_failure = (buf[0] & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;
	extended_id = buf[0] & MXT_BOOT_EXTENDED_ID;

	dev_info(dev, "Found bootloader addr:%02x ID:%u%s%u%s\n",
		 data->bootloader_addr,
		 extended_id ? (buf[1] & MXT_BOOT_ID_MASK) : buf[0],
		 extended_id ? " version:" : "",
		 extended_id ? buf[2] : 0,
		 crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock);

static int mxt_write_firmware_frame(struct mxt_data *data, struct mxt_flash *f)
{
	f->frame = (struct mxt_fw_frame *)(f->fw->data + f->pos);

	/* Take account of CRC bytes */
	f->frame_size = __be16_to_cpu(f->frame->size) + 2U;

	/* Write one frame to device */
	return mxt_bootloader_write(data, f->fw->data + f->pos,
				   f->frame_size);
}

static int mxt_check_bootloader(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_flash *f = data->flash;
	u8 state;
	int ret;

	/* Handle interrupt after download/flash process */
	if (f->pos >= f->fw->size) {
		TPD_LOG_INFO(data->touch_debug,
				"%s: Download/Flash Process completed\n",
				__func__);
		complete(&f->flash_completion);
		return 0;
	}

	ret = mxt_bootloader_read(data, &state, 1);
	if (ret)
		return ret;

	/* Remove don't care bits */
	if (state & ~MXT_BOOT_STATUS_MASK)
		state &= ~MXT_BOOT_STATUS_MASK;

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
		dev_info(dev, "Unlocking bootloader\n");
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret)
			return ret;

		break;

	case MXT_WAITING_FRAME_DATA:
		if ((f->previous != MXT_WAITING_BOOTLOAD_CMD)
		    && (f->previous != MXT_FRAME_CRC_PASS)
		    && (f->previous != MXT_FRAME_CRC_FAIL))
			goto unexpected;

		ret = mxt_write_firmware_frame(data, f);
		if (ret)
			return ret;

		break;

	case MXT_FRAME_CRC_CHECK:
		if (f->previous != MXT_WAITING_FRAME_DATA)
			goto unexpected;
		break;

	case MXT_FRAME_CRC_PASS:
		if (f->previous != MXT_FRAME_CRC_CHECK)
			goto unexpected;

		/* Next frame */
		f->retry = 0;
		f->pos += f->frame_size;
		f->count++;

		if (f->pos >= f->fw->size)
			dev_info(dev, "Sent %u frames, %zu bytes\n",
				f->count, f->fw->size);
		else if (f->count % 50 == 0)
			dev_dbg(dev, "Sent %u frames, %lld/%zu bytes\n",
				f->count, f->pos, f->fw->size);

		break;

	case MXT_FRAME_CRC_FAIL:
		if (f->retry > 20) {
			dev_err(dev, "Retry count exceeded\n");
			return -EIO;
		}

		/* Back off by 20ms per retry */
		f->retry++;
		msleep(f->retry * 20);
		TPD_LOG_INFO(data->touch_debug,
				"%s: Bootloader frame CRC failure(retry=%d)\n",
				__func__, f->retry);
		break;

	default:
		return -EINVAL;
	}

	f->previous = state;

	/* Poll after 0.1s if no interrupt received */
	schedule_delayed_work(&f->work, HZ / 10);

	return 0;

unexpected:
	dev_err(dev, "Unexpected state transition\n");
	return -EINVAL;
}

int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;
	bool retry = false;
	struct mxt_data *data = i2c_get_clientdata(client);

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	tpd_log_dump_message(data->touch_debug, xfer[1].buf, xfer[1].len, "READ");
	if (ret != ARRAY_SIZE(xfer)) {
		tpd_event_bus_error(data->touch_debug);
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
				__func__, ret);
			return -EIO;
		}
	}

	return 0;
}

#define MIN(a, b)       (a > b ? b : a)
static int mxt_read_blks(struct mxt_data *data, u16 start, u16 count, u8 *buf)
{
	u16 offset = 0;
	int error;
	u16 size;

	while (offset < count) {
		size = MIN(MXT_MAX_BLOCK_WRITE, count - offset);

		error = __mxt_read_reg(data->client,
				       start + offset,
				       size, buf + offset);
		if (error)
			return error;

		offset += size;
	}

	return 0;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
	int ret;
	bool retry = false;
	struct mxt_data *data = i2c_get_clientdata(client);

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

retry_write:
	ret = i2c_master_send(client, buf, count);
	tpd_log_dump_message(data->touch_debug, buf, count, "WRITE");
	if (ret != count) {
		tpd_event_bus_error(data->touch_debug);
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_write;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			ret = -EIO;
		}
	} else {
		ret = 0;
	}

	kfree(buf);
	return ret;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_warn(&data->client->dev, "Invalid object type T%u\n", type);
	return NULL;
}

static void mxt_touchmode_reconfigure(struct mxt_data *data)
{
	switch (data->wg_state) {
	case MXT_GESTURE_STATE_ENABLE:
		mxt_set_wakeup_gesture_mode(data, true);
		break;
	default:
		mxt_set_glove_mode(data, data->glove_mode);
		break;
	}
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		dev_dbg(dev, "T6 Config Checksum: 0x%06X\n", crc);
		tpd_info_config_id(data->touch_debug, crc);
	}

	complete(&data->crc_completion);

	/* Detect reset */
	if (status & MXT_T6_STATUS_RESET) {
		complete(&data->reset_completion);
		TPD_LOG_INFO(data->touch_debug, "%s: sensor reset\n", __func__);
		mxt_reset_slots(data);
		mxt_touchmode_reconfigure(data);
		tpd_event_reset(data->touch_debug);
	}
	/* Output debug if status has changed */
	if (status != data->t6_status)
		dev_dbg(dev, "T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			status == 0 ? " OK" : "",
			status & MXT_T6_STATUS_RESET ? " RESET" : "",
			status & MXT_T6_STATUS_OFL ? " OFL" : "",
			status & MXT_T6_STATUS_SIGERR ? " SIGERR" : "",
			status & MXT_T6_STATUS_CAL ? " CAL" : "",
			status & MXT_T6_STATUS_CFGERR ? " CFGERR" : "",
			status & MXT_T6_STATUS_COMSERR ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object || offset >= mxt_obj_size(object))
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	int i;

	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;

		/* Active-low switch */
		input_report_key(input, pdata->t19_keymap[i],
				 !(message[1] & BIT(i)));
	}
}

static void mxt_input_sync(struct mxt_data *data)
{
	if (data->input_dev) {
		input_mt_report_pointer_emulation(data->input_dev,
				data->pdata->t19_num_keys);
		input_sync(data->input_dev);
	}
}

static void mxt_input_report_wakeup(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;

	if (input_dev) {
		input_report_key(input_dev, KEY_WAKEUP, 1);
		input_sync(input_dev);
		input_report_key(input_dev, KEY_WAKEUP, 0);
		input_sync(input_dev);
	}
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];

	dev_dbg(dev,
		"[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
		id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);

	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/*
		 * Multiple bits may be set if the host is slow to read
		 * the status messages, indicating all the events that
		 * have happened.
		 */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev, 0, 0);
			mxt_input_sync(data);
		}

		/* A size of zero indicates touch is from a linked T47 Stylus */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_DEFAULT;
			tool = MT_TOOL_PEN;
		} else {
			tool = MT_TOOL_FINGER;
		}

		/* if active, pressure must be non-zero */
		if (!amplitude)
			amplitude = MXT_PRESSURE_DEFAULT;

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, 0, 0);
	}

	data->update_input = true;
}

static void mxt_t100_screen_status_message(struct mxt_data *data, u8 *message)
{
	if(message[0] == data->T100_reportid_min) {
		memset(data->scraux_data, 0, sizeof(data->scraux_data));
		data->scraux_data[0] = message[1];
		if(data->t100_scr_numrpt)
			data->scraux_data[1] = message[data->t100_scr_numrpt];
		if(data->t100_scr_tcharea) {
			data->scraux_data[2] = message[data->t100_scr_tcharea];
			data->scraux_data[3] = message[data->t100_scr_tcharea + 1];
		}
		if(data->t100_scr_atcharea) {
			data->scraux_data[4] = message[data->t100_scr_atcharea];
			data->scraux_data[5] = message[data->t100_scr_atcharea + 1];
		}
		if(data->t100_scr_intthrarea) {
			data->scraux_data[6] = message[data->t100_scr_intthrarea];
			data->scraux_data[7] = message[data->t100_scr_intthrarea + 1];
		}
		TPD_LOG_DEVICE(data->touch_debug,
				"%s: SCRSTAT(%x) rpt(%d) tch(%d) atch(%d) intthr(%d)\n",
				 __func__, data->scraux_data[0], data->scraux_data[1],
				(data->scraux_data[3] << 8) + data->scraux_data[2],
				(data->scraux_data[5] << 8) + data->scraux_data[4],
				(data->scraux_data[7] << 8) + data->scraux_data[6]);
	}
	return;
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	u8 type = 0;
	u16 x;
	u16 y;
	int distance = 0;
	int tool = 0;
	u8 major = 0;
	u8 pressure = 0;
	u8 orientation = 0;
	bool active = false;
	bool hover = false;

	if (data->wg_state == MXT_GESTURE_STATE_ENABLE) {
		TPD_LOG_INFO(data->touch_debug, "%s: T100 report canceled for wake up gesture mode\n",
				__func__);
		return;
	}

	id = message[0] - data->T100_reportid_min - 2;

	if (id < 0) {
		mxt_t100_screen_status_message(data, message);
		return;
	} else if (id >= data->num_touchids) {
		TPD_LOG_ERR(data->touch_debug, "%s: invalid touch id detected(%d/%d)",
				__func__, id, data->num_touchids);
		return;
	}
	tpd_event_report_start(data->touch_debug);

	status = message[1];
	x = get_unaligned_le16(&message[2]);
	y = get_unaligned_le16(&message[4]);
	tpd_detect_position_x(data->touch_debug, x);
	tpd_detect_position_y(data->touch_debug, y);

	if (status & MXT_T100_DETECT) {
		switch (status & MXT_T100_EVENT_MASK) {
		case MXT_T100_EVENT_MOVE:
			tpd_detect_touch_event(data->touch_debug, TPD_EVENT_REPORT_MOVE);
			break;
		case MXT_T100_EVENT_DOWN:
			tpd_detect_touch_event(data->touch_debug, TPD_EVENT_REPORT_DOWN);
			break;
		case MXT_T100_EVENT_UP:
			tpd_detect_touch_event(data->touch_debug, TPD_EVENT_REPORT_UP);
			break;
		default:
			break;
		}
		type = (status & MXT_T100_TYPE_MASK) >> 4;

		switch (type) {
		case MXT_T100_TYPE_HOVERING_FINGER:
			tool = MT_TOOL_FINGER;
			distance = MXT_DISTANCE_HOVERING;
			hover = true;
			active = true;
			tpd_detect_tool_type(data->touch_debug, TPD_EVENT_TOOL_HOVER);
			break;

		case MXT_T100_TYPE_FINGER:
			tool = MT_TOOL_FINGER;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			hover = false;
			active = true;

			if (data->t100_aux_area)
				major = message[data->t100_aux_area];

			if (data->t100_aux_ampl)
				pressure = message[data->t100_aux_ampl];

			if (data->t100_aux_vect)
				orientation = message[data->t100_aux_vect];

			tpd_detect_tool_type(data->touch_debug, TPD_EVENT_TOOL_FINGER);
			break;
		case MXT_T100_TYPE_GLOVE:
			tool = MT_TOOL_FINGER;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			hover = false;
			active = true;

			if (data->t100_aux_ampl) {
				pressure = message[data->t100_aux_ampl];
				if (data->t100_aux_area) {
					major = (message[data->t100_aux_ampl] >> 1);
				}
			}

			if (data->t100_aux_vect)
				orientation = message[data->t100_aux_vect];

			if (tpd_get_debug_state(data->touch_debug,
					TPD_DEBUG_ID_GLOVE_CORRECTION)) {
				major += MXT_MAX_AREA;
			}

			tpd_detect_tool_type(data->touch_debug, TPD_EVENT_TOOL_GLOVE);
			break;

		case MXT_T100_TYPE_PASSIVE_STYLUS:
			tool = MT_TOOL_FINGER;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			hover = false;
			active = true;

			/*
			 * Passive stylus is reported with size zero so
			 * hardcode.
			 */
			major = MXT_TOUCH_MAJOR_DEFAULT;

			if (data->t100_aux_ampl)
				pressure = message[data->t100_aux_ampl];

			tpd_detect_tool_type(data->touch_debug, TPD_EVENT_TOOL_STYLUS);
			break;

		case MXT_T100_TYPE_ACTIVE_STYLUS:
			/* Report input buttons */
			input_report_key(input_dev, BTN_STYLUS,
					 message[6] & MXT_T107_STYLUS_BUTTON0);
			input_report_key(input_dev, BTN_STYLUS2,
					 message[6] & MXT_T107_STYLUS_BUTTON1);

			/* stylus in range, but position unavailable */
			if (!(message[6] & MXT_T107_STYLUS_HOVER))
				break;

			tool = MT_TOOL_PEN;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			active = true;
			major = MXT_TOUCH_MAJOR_DEFAULT;

			if (!(message[6] & MXT_T107_STYLUS_TIPSWITCH)) {
				hover = true;
				distance = MXT_DISTANCE_HOVERING;
			} else if (data->stylus_aux_pressure) {
				pressure = message[data->stylus_aux_pressure];
			}

			tpd_detect_tool_type(data->touch_debug, TPD_EVENT_TOOL_STYLUS);
			break;

		case MXT_T100_TYPE_LARGE_TOUCH:
			/* Ignore suppressed touch */
			break;

		default:
			dev_dbg(dev, "Unexpected T100 type\n");
			tpd_event_report_end(data->touch_debug);
			return;
		}
	}

	/*
	 * Values reported should be non-zero if tool is touching the
	 * device
	 */
	if (!hover && pressure < MXT_PRESSURE_DEFAULT)
		pressure = MXT_PRESSURE_DEFAULT;

	input_mt_slot(input_dev, id);

	if (active) {
		dev_dbg(dev, "[%u] type:%u x:%u y:%u a:%02X p:%02X v:%02X\n",
			id, type, x, y, major, pressure, orientation);

		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, major);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
		input_report_abs(input_dev, ABS_MT_DISTANCE, distance);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, orientation);

	} else {
		dev_dbg(dev, "[%u] release\n", id);

		/* close out slot */
		input_mt_report_slot_state(input_dev, 0, 0);
		tpd_detect_touch_event(data->touch_debug, TPD_EVENT_REPORT_UP);
		tpd_detect_tool_type(data->touch_debug, TPD_EVENT_TOOL_LIFTOFF);
	}

	data->update_input = true;
	tpd_event_report_end(data->touch_debug);
}

static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	struct device *dev = &data->client->dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);

	for (key = 0; key < data->pdata->t15_num_keys; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);

		if (!curr_state && new_state) {
			dev_dbg(dev, "T15 key press: %u\n", key);
			__set_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 1);
			sync = true;
		} else if (curr_state && !new_state) {
			dev_dbg(dev, "T15 key release: %u\n", key);
			__clear_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 0);
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(dev, "T42 suppress\n");
	else
		dev_info(dev, "T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	dev_dbg(dev, "T48 state %d status %02X %s%s%s%s%s\n", state, status,
		status & 0x01 ? "FREQCHG " : "",
		status & 0x02 ? "APXCHG " : "",
		status & 0x04 ? "ALGOERR " : "",
		status & 0x10 ? "STATCHG " : "",
		status & 0x20 ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		dev_err(dev, "invalid stylus id %d, max slot is %d\n",
			id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_T63_STYLUS_PRESSURE_MASK;

	dev_dbg(dev,
		"[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
		id,
		msg[1] & MXT_T63_STYLUS_SUPPRESS ? 'S' : '.',
		msg[1] & MXT_T63_STYLUS_MOVE     ? 'M' : '.',
		msg[1] & MXT_T63_STYLUS_RELEASE  ? 'R' : '.',
		msg[1] & MXT_T63_STYLUS_PRESS    ? 'P' : '.',
		x, y, pressure,
		msg[2] & MXT_T63_STYLUS_BARREL   ? 'B' : '.',
		msg[2] & MXT_T63_STYLUS_ERASER   ? 'E' : '.',
		msg[2] & MXT_T63_STYLUS_TIP      ? 'T' : '.',
		msg[2] & MXT_T63_STYLUS_DETECT   ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_T63_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, 0, 0);
	}

	input_report_key(input_dev, BTN_STYLUS,
			 (msg[2] & MXT_T63_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2,
			 (msg[2] & MXT_T63_STYLUS_BARREL));

	mxt_input_sync(data);
}

static void mxt_proc_t92_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	dev_info(dev, "T92 long stroke LSTR=%d %d\n",
		 (status & 0x80) ? 1 : 0,
		 status & 0x0F);
}

static void mxt_proc_t93_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	dev_info(dev, "T93 report double tap %d\n", status);
	TPD_LOG_DEBUG(data->touch_debug,
			"%s: Wake up gesture report state(%d) mode(%d)\n",
			__func__, data->wg_state, data->wakeup_gesture_mode);

	if ((data->wg_state == MXT_GESTURE_STATE_ENABLE) &&
	    (status & MXT_T93_STATUS_DBLTAP)) {
		mxt_input_report_wakeup(data);
		tpd_event_wakeup_gesture(data->touch_debug, TPD_WAKEUP_DOUBLE_TAP);
	}
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (report_id == data->T93_reportid) {
		mxt_proc_t93_messages(data, message);
	} else if (!data->input_dev || data->suspended) {
		/*
		 * Do not report events if input device is not
		 * yet registered or returning from suspend
		 */
		mxt_dump_message(data, message);
	} else if (report_id >= data->T9_reportid_min &&
		   report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
	} else if (report_id >= data->T100_reportid_min &&
		   report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else if (report_id >= data->T63_reportid_min
		   && report_id <= data->T63_reportid_max) {
		mxt_proc_t63_messages(data, message);
	} else if (report_id >= data->T15_reportid_min
		   && report_id <= data->T15_reportid_max) {
		mxt_proc_t15_messages(data, message);
	} else if (report_id == data->T92_reportid) {
		mxt_proc_t92_messages(data, message);
	} else {
		dump = true;
	}

	if (dump)
		mxt_dump_message(data, message);

	if (data->debug_v2_enabled)
		mxt_debug_msg_add(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;

	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];

	/*
	 * This condition may be caused by the CHG line being configured in
	 * Mode 0. It results in unnecessary I2C operations but it is benign.
	 */
	if (count == 0)
		return IRQ_NONE;

	if (count > data->max_reportid) {
		dev_warn(dev, "T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		dev_warn(dev, "Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			dev_warn(dev, "Unexpected invalid message\n");
	}

end:
	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* keep reading two msgs until one is invalid or reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxt_check_flash_bootloader(struct mxt_data *data)
{
	if (!data->flash) {
		TPD_LOG_WARN(data->touch_debug,
				"%s: Interrupt triggered but bootloader flash disabled\n",
				__func__);
		return IRQ_HANDLED;
	}

	/* flash bootloader */
	cancel_delayed_work_sync(&data->flash->work);
	return IRQ_RETVAL(mxt_check_bootloader(data));
}

static irqreturn_t mxt_interrupt_appmode(struct mxt_data *data)
{
	/* Hard Control Reset PullUP Workaround
	 * Unexpected interrupt triggered before bootloader mode,
	 * because hard handling reset pullup at vdd enabled timing.
	 */
	if (!data->msg_buf) {
		TPD_LOG_WARN(data->touch_debug,
				"%s: Interrupt triggered but msg_buf is null\n",
				__func__);
		return IRQ_HANDLED;
	}

	if (!data->object_table) {
		return IRQ_HANDLED;
	}

	if (data->T44_address) {
		return mxt_process_messages_t44(data);
	} else {
		return mxt_process_messages(data);
	}
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	irqreturn_t iret;

	tpd_event_enter_irq(data->touch_debug);

	if (data->in_bootloader) {
		iret = mxt_check_flash_bootloader(data);
	} else {
		iret = mxt_interrupt_appmode(data);
	}

	tpd_event_exit_irq(data->touch_debug);

	return iret;
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while (command_register != 0 && timeout_counter++ <= 100);

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting device\n");

	disable_irq(data->irq);

	INIT_COMPLETION(data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	/* Ignore CHG line for 100ms after reset */
	msleep(100);

	enable_irq(data->irq);

	ret = mxt_wait_for_completion(data, &data->reset_completion,
				      MXT_RESET_TIMEOUT);
	if (ret)
		return ret;

	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/*
	 * On failure, CRC is set to 0 and config will always be
	 * downloaded.
	 */
	data->config_crc = 0;
	INIT_COMPLETION(data->crc_completion);

	mxt_t6_command(data, cmd, value, true);

	/*
	 * Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded.
	 */
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct irq_data *irqd;
	int error;
	int val;

	irqd = irq_get_irq_data(data->irq);
	if (irqd_get_trigger_type(irqd) & IRQF_TRIGGER_LOW)
		return 0;

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				       data->T18_address + MXT_COMMS_CTRL,
				       1, &val);
		if (error)
			return error;

		if (val & MXT_COMMS_RETRIGEN)
			return 0;
	}

	dev_warn(&client->dev, "Enabling RETRIGEN workaround\n");
	data->use_retrigen_workaround = true;
	return 0;
}

static int mxt_prepare_cfg_mem(struct mxt_data *data,
			       const struct firmware *cfg,
			       unsigned int data_pos,
			       unsigned int cfg_start_ofs,
			       u8 *config_mem,
			       size_t config_mem_size,
			       u8* cfg_data)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object;
	unsigned int type, instance, size, byte_offset;
	int offset;
	int ret;
	int i;
	u16 reg;
	u8 val;

	while (data_pos < cfg->size) {
		/* Read type, instance, length */
		ret = sscanf(cfg_data + data_pos, "%x %x %x%n",
			     &type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			dev_err(dev, "Bad format: failed to parse object\n");
			return -EINVAL;
		}
		data_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg_data + data_pos, "%hhx%n",
					     &val, &offset);
				if (ret != 1) {
					dev_err(dev, "Bad format in T%d at %d\n",
						type, i);
					return -EINVAL;
				}
				data_pos += offset;
			}
			continue;
		}

		if (size > mxt_obj_size(object)) {
			/*
			 * Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited.
			 */
			dev_warn(dev, "Discarding %zu byte(s) in T%u\n",
				 size - mxt_obj_size(object), type);
		} else if (mxt_obj_size(object) > size) {
			/*
			 * If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration.
			 */
			dev_warn(dev, "Zeroing %zu byte(s) in T%d\n",
				 mxt_obj_size(object) - size, type);
		}

		if (instance >= mxt_obj_instances(object)) {
			dev_err(dev, "Object instances exceeded!\n");
			return -EINVAL;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg_data + data_pos, "%hhx%n",
				     &val,
				     &offset);
			if (ret != 1) {
				dev_err(dev, "Bad format in T%d at %d\n",
					type, i);
				return -EINVAL;
			}
			data_pos += offset;

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg_start_ofs;

			if (byte_offset >= 0 && byte_offset < config_mem_size) {
				*(config_mem + byte_offset) = val;
			} else {
				dev_err(dev, "Bad object: reg:%d, T%d, ofs=%d\n",
					reg, object->type, byte_offset);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int mxt_upload_cfg_mem(struct mxt_data *data, unsigned int cfg_start,
			      u8 *config_mem, size_t config_mem_size)
{
	unsigned int byte_offset = 0;
	int error;

	/* Write configuration as blocks */
	while (byte_offset < config_mem_size) {
		unsigned int size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		error = __mxt_write_reg(data->client,
					cfg_start + byte_offset,
					size, config_mem + byte_offset);
		if (error) {
			dev_err(&data->client->dev,
				"Config write error, ret=%d\n", error);
			return error;
		}

		byte_offset += size;
	}

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

/*
 * mxt_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_update_cfg(struct mxt_data *data, const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info;
	int ret;
	int offset;
	int data_pos;
	int i;
	int cfg_start_ofs;
	u32 info_crc, config_crc, calculated_crc;
	u8 *config_mem;
	size_t config_mem_size;
	u16 crc_start = 0;
	u8 *cfg_data;

	TPD_LOG_INFO(data->touch_debug, "%s: start\n", __func__);

	/* 
	 * Because the next byte of the last cfg->data is not fixed null,
	 * sometimes sscanf() read over cfg->size and then be bad format.
	 */
	cfg_data = kzalloc(cfg->size + 1, GFP_KERNEL);
	if (!cfg_data) {
		TPD_LOG_ERR(data->touch_debug,"%s: Failed to allocate memory\n",__func__);
		return -ENOMEM;
	}
	memcpy(cfg_data, cfg->data, cfg->size);

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg_data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		dev_err(dev, "Unrecognised config file\n");
		ret = -EINVAL;
		goto release_cfg_data;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg_data + data_pos, "%hhx%n",
			     (unsigned char *)&cfg_info + i,
			     &offset);
		if (ret != 1) {
			dev_err(dev, "Bad format\n");
			ret = -EINVAL;
			goto release_cfg_data;
		}

		data_pos += offset;
	}

	if (cfg_info.family_id != data->info->family_id) {
		dev_err(dev, "Family ID mismatch!\n");
		ret = -EINVAL;
		goto release_cfg_data;
	}

	if (cfg_info.variant_id != data->info->variant_id) {
		dev_err(dev, "Variant ID mismatch!\n");
		ret = -EINVAL;
		goto release_cfg_data;
	}

	/* Read CRCs */
	ret = sscanf(cfg_data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release_cfg_data;
	}
	data_pos += offset;

	ret = sscanf(cfg_data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release_cfg_data;
	}
	data_pos += offset;

	/*
	 * The Info Block CRC is calculated over mxt_info and the object
	 * table. If it does not match then we are trying to load the
	 * configuration from a different chip or firmware version, so
	 * the configuration CRC is invalid anyway.
	 */
	if (info_crc == data->info_crc) {
		if (config_crc == 0 || data->config_crc == 0) {
			dev_info(dev, "CRC zero, attempting to apply config\n");
		} else if (config_crc == data->config_crc) {
			TPD_LOG_INFO(data->touch_debug, "%s: Config CRC 0x%06X: OK\n",
					__func__, data->config_crc);
			ret = 0;
			goto release_cfg_data;
		} else {
			dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X\n",
				 data->config_crc, config_crc);
		}
	} else {
		dev_warn(dev,
			 "Warning: Info CRC error - device=0x%06X file=0x%06X\n",
			 data->info_crc, info_crc);
	}

	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START +
			data->info->object_num * sizeof(struct mxt_object) +
			MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem) {
		ret = -ENOMEM;
		goto release_cfg_data;
	}

	ret = mxt_prepare_cfg_mem(data, cfg, data_pos, cfg_start_ofs,
				  config_mem, config_mem_size, cfg_data);
	if (ret)
		goto release_mem;

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_RESTORE_STATIC_VALUE);

	/* Calculate crc of the received configs (not the raw config file) */
	if (data->T71_address)
		crc_start = data->T71_address;
	else if (data->T7_address)
		crc_start = data->T7_address;
	else
		dev_warn(dev, "Could not find CRC start\n");

	if (crc_start > cfg_start_ofs) {
		calculated_crc = mxt_calculate_crc(config_mem,
						   crc_start - cfg_start_ofs,
						   config_mem_size);

		if (config_crc > 0 && config_crc != calculated_crc)
			dev_warn(dev, "Config CRC in file inconsistent, calculated=%06X, file=%06X\n",
				 calculated_crc, config_crc);
	}

	ret = mxt_upload_cfg_mem(data, cfg_start_ofs,
				 config_mem, config_mem_size);
	if (ret)
		goto release_mem;

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	ret = mxt_check_retrigen(data);
	if (ret)
		goto release_mem;

	ret = mxt_soft_reset(data);
	if (ret)
		goto release_mem;

	dev_info(dev, "Config successfully updated\n");

	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

release_mem:
	kfree(config_mem);
release_cfg_data:
	kfree(cfg_data);
	TPD_LOG_INFO(data->touch_debug, "%s: end (ret=%d)\n", __func__, ret);
	return ret;
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	if (!data->irq) {
		error = request_threaded_irq(data->client->irq, NULL,
				mxt_interrupt,
				data->pdata->irqflags | IRQF_ONESHOT,
				data->client->name, data);
		if (error) {
			dev_err(&data->client->dev, "Error requesting irq\n");
			return error;
		}

		/* Presence of data->irq means IRQ initialised */
		data->irq = data->client->irq;
	} else {
		enable_irq(data->irq);
	}

	if (data->object_table && data->use_retrigen_workaround) {
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	data->objects_configured = false;

	mxt_debug_msg_remove(data);

	data->object_table = NULL;
	data->info = NULL;
	kfree(data->raw_info_block);
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T71_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T48_reportid = 0;
	data->T63_reportid_min = 0;
	data->T63_reportid_max = 0;
	data->T70_address = 0;
	data->T70_size = 0;
	data->T70_instances = 0;
	if (data->T70_cfg) {
		kfree(data->T70_cfg);
		data->T70_cfg = NULL;
	}
	data->T92_reportid = 0;
	data->T92_address = 0;
	data->T93_reportid = 0;
	data->T93_address = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data,
				  struct mxt_object *object_table)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		TPD_LOG_DEVICE(data->touch_debug,
			"T%u Start:%u Size:%zu Instances:%zu Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80 &&
			    data->info->version < 0x20) {
				/*
				 * On mXT224 firmware versions prior to V2.0
				 * read and discard unused CRC byte otherwise
				 * DMA reads are misaligned.
				 */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
			data->T71_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			data->multitouch = MXT_TOUCH_MULTI_T9;
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_PROCI_ACTIVE_STYLUS_T63:
			/* Only handle messages from first T63 instance */
			data->T63_reportid_min = min_id;
			data->T63_reportid_max = min_id;
			data->num_stylusids = 1;
			break;
		case MXT_SPT_DYNAMICCONFIGRATIONCONTROLLER_T70:
			data->T70_address = object->start_address;
			data->T70_size = mxt_obj_size(object);
			data->T70_instances = mxt_obj_instances(object);
			break;
		case MXT_PROCI_SYMBOLGESTUREPROCESSOR:
			data->T92_reportid = min_id;
			data->T92_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSEQUENCELOGGER:
			data->T93_reportid = min_id;
			data->T93_address = object->start_address;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->multitouch = MXT_TOUCH_MULTITOUCHSCREEN_T100;
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			break;
		case MXT_PROCI_ACTIVESTYLUS_T107:
			data->T107_address = object->start_address;
			break;
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf)
		return -ENOMEM;

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf)
		return -ENOMEM;

	error = __mxt_read_reg(client, 0, size, id_buf);
	if (error) {
		kfree(id_buf);
		return error;
	}

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mxt_info *)id_buf)->object_num;
	size += (num_objects * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(id_buf, size, GFP_KERNEL);
	if (!buf) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = mxt_read_blks(data, MXT_OBJECT_START,
			      size - MXT_OBJECT_START,
			      buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/*
	 * CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol (eg i2c-hid)
	 */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	}

	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;

	dev_info(&client->dev,
		 "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);

	tpd_info_firm_id(data->touch_debug,
			(data->info->family_id << 24) +
			(data->info->variant_id << 16) +
			(data->info->version << 8) +
			(data->info->build));

	/* Parse object table information */
	error = mxt_parse_object_table(data, buf + MXT_OBJECT_START);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	if (!data->reg_vdd || !data->reg_avdd)
		return;

	if (data->pdata->ts_pinctrl) {
		error = pinctrl_select_state(data->pdata->ts_pinctrl,
				data->pdata->pinctrl_state_rst_active);
		if (error < 0) {
			pr_err("Can not set rst pins. retval=%d\n", error);
		}
		error = pinctrl_select_state(data->pdata->ts_pinctrl,
				data->pdata->pinctrl_state_i2c_active);
		if (error < 0) {
			pr_err("Can not set i2c pins. retval=%d\n", error);
		}
		error = pinctrl_select_state(data->pdata->ts_pinctrl,
				data->pdata->pinctrl_state_irq_active);
		if (error < 0) {
			pr_err("Can not set irq pins. retval=%d\n", error);
		}
	}
	gpio_direction_input(data->pdata->gpio_reset);

	error = regulator_set_optimum_mode(data->reg_avdd,
			data->pdata->avdd_optimum_uv[1]);
	if (error < 0) {
		dev_err(dev, "Error %d reg_avdd set_optimum_mode NPM\n", error);
	}

	error = regulator_enable(data->reg_avdd);
	if (error)
		return;

	error = regulator_enable(data->reg_vdd);
	if (error)
		return;

	msleep(MXT_CHG_DELAY);
}

static void mxt_regulator_disable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	if (!data->reg_vdd || !data->reg_avdd)
		return;

	if (data->pdata->ts_pinctrl) {
		error = pinctrl_select_state(data->pdata->ts_pinctrl,
				data->pdata->pinctrl_state_rst_suspend);
		if (error < 0) {
			pr_err("Can not set rst pins. retval=%d\n", error);
		}
	}

	regulator_disable(data->reg_vdd);
	regulator_disable(data->reg_avdd);

	error = regulator_set_optimum_mode(data->reg_avdd,
			data->pdata->avdd_optimum_uv[0]);
	if (error < 0) {
		dev_err(dev, "Error %d reg_avdd set_optimum_mode LPM\n", error);
	}

	if (data->pdata->ts_pinctrl) {
		error = pinctrl_select_state(data->pdata->ts_pinctrl,
				data->pdata->pinctrl_state_i2c_suspend);
		if (error < 0) {
			pr_err("Can not set i2c pins. retval=%d\n", error);
		}
		error = pinctrl_select_state(data->pdata->ts_pinctrl,
				data->pdata->pinctrl_state_irq_suspend);
		if (error < 0) {
			pr_err("Can not set irq pins. retval=%d\n", error);
		}
	}
}

static int mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	/* Must have reset GPIO to use regulator support */
	if (!gpio_is_valid(data->pdata->gpio_reset)) {
		error = -EINVAL;
		goto fail;
	}

	data->reg_vdd = regulator_get(dev, data->pdata->vdd_name);
	if (IS_ERR(data->reg_vdd)) {
		error = PTR_ERR(data->reg_vdd);
		dev_err(dev, "Error %d getting vdd regulator\n", error);
		goto fail;
	}

	data->reg_avdd = regulator_get(dev, data->pdata->avdd_name);
	if (IS_ERR(data->reg_avdd)) {
		error = PTR_ERR(data->reg_avdd);
		dev_err(dev, "Error %d getting avdd regulator\n", error);
		goto fail_release;
	}

	error = regulator_set_voltage(data->reg_avdd,
			data->pdata->avdd_vol[0], data->pdata->avdd_vol[1]);
	if (error < 0) {
		dev_err(dev, "Error %d regulator set_voltage to reg_avdd\n", error);
	} else {
		dev_info(dev, "Set_voltage reg_avdd to %d-%d\n",
				data->pdata->avdd_vol[0], data->pdata->avdd_vol[1]);
	}

	mxt_regulator_enable(data);

	TPD_LOG_DEBUG(data->touch_debug, "%s: Initialized regulators\n", __func__);
	dev_dbg(dev, "Initialised regulators\n");
	return 0;

fail_release:
	regulator_put(data->reg_vdd);
fail:
	data->reg_vdd = NULL;
	data->reg_avdd = NULL;
	return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(&range.x);
	le16_to_cpus(&range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 1023;

	if (range.y == 0)
		range.y = 1023;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	dev_dbg(&client->dev,
		"Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_read_t107_stylus_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u8 styaux;
	int aux;

	object = mxt_get_object(data, MXT_PROCI_ACTIVESTYLUS_T107);
	if (!object)
		return 0;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T107_STYLUS_STYAUX,
			       1, &styaux);
	if (error)
		return error;

	/* map aux bits */
	aux = 7;

	if (styaux & MXT_T107_STYLUS_STYAUX_PRESSURE)
		data->stylus_aux_pressure = aux++;

	if (styaux & MXT_T107_STYLUS_STYAUX_PEAK)
		data->stylus_aux_peak = aux++;

	dev_dbg(&client->dev,
		"Enabling T107 active stylus, aux map pressure:%u peak:%u\n",
		data->stylus_aux_pressure, data->stylus_aux_peak);

	return 0;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;
	u8 cfg, tchaux, scraux;
	u8 aux;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(&range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(&range_y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	error = __mxt_read_reg(client,
			object->start_address + MXT_T100_SCRAUX,
			1, &scraux);
	if (error)
		return error;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_y;
		data->max_y = range_x;
	} else {
		data->max_x = range_x;
		data->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	/* allocate aux bytes */
	aux = 2;

	if(scraux & MXT_T100_SCRAUX_NUMRPT)
		data->t100_scr_numrpt = aux++;

	if(scraux & MXT_T100_SCRAUX_TCHAREA) {
		data->t100_scr_tcharea = aux;
		aux += 2;
	}
	if(scraux & MXT_T100_SCRAUX_ATCHAREA) {
		data->t100_scr_atcharea = aux;
		aux += 2;
	}
	if(scraux & MXT_T100_SCRAUX_INTTHRAREA) {
		data->t100_scr_intthrarea = aux;
		aux += 2;
	}

	TPD_LOG_INFO(data->touch_debug,"%s: T100 SCRAUX(%d,%d,%d,%d) allocated\n",__func__,
		data->t100_scr_numrpt, data->t100_scr_tcharea,
		data->t100_scr_atcharea, data->t100_scr_intthrarea);

	dev_dbg(&client->dev,
		"T100 aux mappings vect:%u ampl:%u area:%u\n",
		data->t100_aux_vect, data->t100_aux_ampl, data->t100_aux_area);

	dev_info(&client->dev,
		 "T100 Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static void mxt_set_up_as_touchpad(struct input_dev *input_dev,
				   struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	int i;

	input_dev->name = "Atmel maXTouch Touchpad";

	__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

	input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_MT_POSITION_X,
			  MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
			  MXT_PIXELS_PER_MM);

	for (i = 0; i < pdata->t19_num_keys; i++)
		if (pdata->t19_keymap[i] != KEY_RESERVED)
			input_set_capability(input_dev, EV_KEY,
					     pdata->t19_keymap[i]);
}

static int mxt_initialize_input_device(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	unsigned int mt_flags = 0;
	int i;

	switch (data->multitouch) {
	case MXT_TOUCH_MULTI_T9:
		num_mt_slots = data->T9_reportid_max - data->T9_reportid_min + 1;
		error = mxt_read_t9_resolution(data);
		if (error)
			dev_warn(dev, "Failed to initialize T9 resolution\n");
		break;

	case MXT_TOUCH_MULTITOUCHSCREEN_T100:
		num_mt_slots = data->num_touchids;
		error = mxt_read_t100_config(data);
		if (error)
			dev_warn(dev, "Failed to read T100 config\n");

		if (data->T107_address) {
			error = mxt_read_t107_stylus_config(data);
			if (error)
				dev_warn(dev, "Failed to read T107 config\n");
		}

		break;

	default:
		dev_err(dev, "Invalid multitouch object\n");
		return -EINVAL;
	}

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	if (data->pdata->input_name)
		input_dev->name = data->pdata->input_name;
	else
		input_dev->name = "Atmel maXTouch Touchscreen";

	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* If device has buttons we assume it is a touchpad */
	if (pdata->t19_num_keys) {
		mxt_set_up_as_touchpad(input_dev, data);
		mt_flags |= INPUT_MT_POINTER;
	} else {
		mt_flags |= INPUT_MT_DIRECT;
	}

	/* For multi touch */
	error = input_mt_init_slots(input_dev, num_mt_slots, mt_flags);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	if (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100) {
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
				     0, MT_TOOL_FINGER, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_DISTANCE,
				     MXT_DISTANCE_ACTIVE_TOUCH,
				     MXT_DISTANCE_HOVERING,
				     0, 0);
	}

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);

	if (data->multitouch == MXT_TOUCH_MULTI_T9 ||
	    (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	     data->t100_aux_area)) {
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				     0, MXT_MAX_AREA, 0, 0);
	}

	if (data->multitouch == MXT_TOUCH_MULTI_T9 ||
	    (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	     (data->t100_aux_ampl || data->stylus_aux_pressure))) {
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
				     0, 255, 0, 0);
	}

	if (data->multitouch == MXT_TOUCH_MULTI_T9 ||
	    (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	    data->t100_aux_vect)) {
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				     0, 255, 0, 0);
	}

#if 0
	/* For active stylus */
	if (data->T63_reportid_min || data->T107_address) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
				0, MT_TOOL_MAX, 0, 0);
	}
#endif

	/* For T15 Key Array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;

		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					data->pdata->t15_keymap[i]);
	}

	/* For wakeup gesture */
	if (data->T92_address || data->T93_address) {
		input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_set_t70_configuration_control(
		struct mxt_data *data, u16 instance, u8 enable)
{
	int error;
	u8 new_config;

	if (!data->T70_cfg || !data->objects_configured ||
			data->T70_instances <= instance)
		return -ENODEV;

	new_config = data->T70_cfg[instance];

	if (enable) {
		new_config |= MXT_FRCCMD_FRCRISE;
	} else {
		new_config |= MXT_FRCCMD_FRCFALL;
	}
	error = __mxt_write_reg(data->client,
			data->T70_address + data->T70_size * instance,
			sizeof(new_config), &new_config);
	if (error)
		return error;

	return 0;
}

static int mxt_init_t70_configuration_control(struct mxt_data *data)
{
	int error, i;
	u8 T70_data[data->T70_size * data->T70_instances];

	data->T70_cfg = kzalloc(data->T70_instances * sizeof(u8), GFP_KERNEL);
	if (!data->T70_cfg)
		return -ENOMEM;

	error = __mxt_read_reg(data->client, data->T70_address,
				sizeof(T70_data), &T70_data);
	if (error)
		return error;

	for (i = 0; i < data->T70_instances; i++) {
		data->T70_cfg[i] = T70_data[data->T70_size * i] & ~(0x03 << 3);
		TPD_LOG_DEBUG(data->touch_debug, "%s: config(%d) value=0x%x\n",
				__func__, i, data->T70_cfg[i]);
	}

	return 0;
}

static int mxt_set_glove_mode(struct mxt_data *data, u8 enable)
{
	int error;

	error = mxt_set_t70_configuration_control(data,
			MXT_T70_INSTANCE_GLOVE, enable);
	if (error < 0) {
		return error;
	}

	TPD_LOG_DEBUG(data->touch_debug, "%s: Glove mode %sabled.\n",
			__func__, enable ? "En" : "Dis");

	return 0;
}

static bool mxt_wakeup_gesture_enabled(struct mxt_data *data)
{
	return (data->wakeup_gesture_mode & MXT_GESTURE_ALL);
}

static int mxt_set_wakeup_gesture_mode(struct mxt_data *data, bool enable)
{
	int ret;
	u8 *set_data;
	struct wakeup_gesture_param *param;

	for (param = data->wg_params; param != NULL; param = param->next) {
		if (!param->obj_addr) {
			continue;
		}

		if (enable) {
			set_data = param->active_data;
		} else {
			set_data = param->default_data;
		}

		ret = __mxt_write_reg(data->client,
				param->obj_addr + param->obj_offset,
				param->data_size, set_data);
		if (ret) {
			TPD_LOG_ERR(data->touch_debug,
					"%s: Failed to set %d param\n",
					__func__, param->obj_id);
			return ret;
		}
		TPD_LOG_DEBUG(data->touch_debug,
				"%s: T%d %s wake up gesture mode\n",
				__func__, param->obj_id,
				enable ? "Enter" : "Exit");
	}

	TPD_LOG_INFO(data->touch_debug,
			"%s: %s wake up gesture mode\n",
			__func__, enable ? "Enter" : "Exit");

	return 0;
}

static int mxt_init_wakeup_gesture_mode(struct mxt_data *data)
{
	int ret;
	struct wakeup_gesture_param *param;
	struct mxt_object *object;

	for (param = data->wg_params; param != NULL; param = param->next) {
		/* Get object address */
		object = mxt_get_object(data, param->obj_id);
		if (!object || param->obj_offset >=
				mxt_obj_size(object) * mxt_obj_instances(object)) {
			TPD_LOG_WARN(data->touch_debug,
					"%s: T%d object missing\n", __func__,
					param->obj_id);
			param->obj_addr = 0;
			continue;
		}
		param->obj_addr = object->start_address;

		ret = __mxt_read_reg(data->client,
				param->obj_addr + param->obj_offset,
				param->data_size, param->default_data);
		if (ret) {
			TPD_LOG_ERR(data->touch_debug,
					"%s: Failed to get initial default data\n", __func__);
			return ret;
		}

		TPD_LOG_DEBUG(data->touch_debug, "%s: T%d",
				__func__, param->obj_id);
		tpd_log_dump_buffer(data->touch_debug,
				param->active_data, param->data_size,
				"Active", TPD_DEBUG_ID_LOG_DEBUG);
		tpd_log_dump_buffer(data->touch_debug,
				param->default_data, param->data_size,
				"Default", TPD_DEBUG_ID_LOG_DEBUG);
	}
	return 0;
}

static void mxt_parse_wg_userdata(
		struct mxt_data *data, struct device_node *np,
		u8 *buf, int buf_size, int offset)
{
	u8 *userdata;
	int len, ret, data_size, i;

	ret = tpd_get_config_param(data->touch_debug, &userdata, &len);
	if (ret) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Failed to get config param\n", __func__);
		return;
	}

	if (offset >= len) {
		TPD_LOG_WARN(data->touch_debug,
				"%s: Invalid offset(%d/%d)\n",
				__func__, offset, len);
		return;
	}

	/* First byte indicates size */
	data_size = (userdata[offset] < buf_size) ?
			userdata[offset] : buf_size;

	if (data_size == 0) {
		TPD_LOG_INFO(data->touch_debug,
				"%s: Userdata disabled(%d)\n", __func__,
				userdata[offset]);
		return;
	}

	/* Second byte and followings are actual userdata */
	if (++offset + data_size > len) {
		TPD_LOG_WARN(data->touch_debug,
				"%s: Invalid data offset(%d/%d)\n",
				__func__, offset + data_size, len);
		return;
	}

	/* If userdata enabled, overwrite data */
	for (i = 0; i < data_size; i++) {
		buf[i] = userdata[offset + i];
	}
	TPD_LOG_INFO(data->touch_debug,
			"%s: Userdata enabled (size:%d)\n",
			__func__, data_size);

	return;
}

static struct wakeup_gesture_param *mxt_parse_wg_object(
		struct mxt_data *data, struct device_node *np)
{
	struct i2c_client *client = data->client;
	int proplen, offset, ret;
	u32 prop;
	struct wakeup_gesture_param *param;
	u8 *active_data, *default_data;

	param = devm_kzalloc(&client->dev,
				sizeof(struct wakeup_gesture_param), GFP_KERNEL);
	if (!param) {
		return ERR_PTR(-ENOMEM);
	}

	ret = of_property_read_u32(np, "id", &prop);
	if (ret) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Could not find id(%d)\n",
				__func__, ret);
		ret = -EINVAL;
		goto err_free_param;
	}
	param->obj_id = (u8)prop;

	ret = of_property_read_u32(np, "offset", &prop);
	if (ret) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Could not find offset(%d)\n",
				__func__, ret);
		ret = -EINVAL;
		goto err_free_param;
	}
	param->obj_offset = (u8)prop;

	if (!of_find_property(np, "active-data", &proplen)) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Could not find active-data\n",
				__func__);
		ret = -EINVAL;
		goto err_free_param;
	}
	param->data_size = proplen;
	active_data = devm_kzalloc(&client->dev,
			param->data_size, GFP_KERNEL);
	if (!active_data) {
		ret = -ENOMEM;
		goto err_free_param;
	}
	ret = of_property_read_u8_array(np, "active-data",
			active_data, param->data_size);
	if (ret) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Could not read active-data(%d)\n",
				__func__, ret);
		ret = -EINVAL;
		goto err_free_active_data;
	}

	if (of_find_property(np, "active-userdata-offset", NULL)) {
		ret = of_property_read_u32(np, "active-userdata-offset", &offset);
		if (ret) {
			TPD_LOG_ERR(data->touch_debug,
					"%s: Could not read active-userdata-offset(%d)\n",
					__func__, ret);
			ret = -EINVAL;
			goto err_free_active_data;
		}
		mxt_parse_wg_userdata(data, np,
				active_data, param->data_size, offset);
	}

	default_data = devm_kzalloc(&client->dev,
			param->data_size, GFP_KERNEL);
	if (!default_data) {
		ret = -ENOMEM;
		goto err_free_active_data;
	}

	param->active_data = active_data;
	param->default_data = default_data;

	TPD_LOG_INFO(data->touch_debug,
			"%s: T%d %d %d\n", __func__, param->obj_id,
			param->obj_offset, param->data_size);
	

	return param;

err_free_active_data:
	devm_kfree(&client->dev, active_data);
err_free_param:
	devm_kfree(&client->dev, param);
	return ERR_PTR(ret);
}

static int mxt_parse_wg_params(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device_node *np, *child;
	struct wakeup_gesture_param *param, *prev = NULL;

	np = of_get_child_by_name(client->dev.of_node, "atmel,wakeup-gesture");
	if (!np) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Could not find atmel,wakeup-gesture node\n",
				__func__);
		return -EINVAL;
	}
	for_each_child_of_node(np, child) {
		param = mxt_parse_wg_object(data, child);
		if (IS_ERR_OR_NULL(param)) {
			TPD_LOG_ERR(data->touch_debug,
					"%s: Failed to parse %s node\n",
					__func__, child->name);
			return -EINVAL;
		}

		if (prev) {
			prev->next = param;
		} else {
			data->wg_params = param;
		}
		prev = param;
	}

	return 0;
}

static int mxt_free_wg_params(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct wakeup_gesture_param *param;

	while (data->wg_params != NULL) {
		param = data->wg_params;
		data->wg_params = param->next;
		devm_kfree(&client->dev, param->default_data);
		devm_kfree(&client->dev, param->active_data);
		devm_kfree(&client->dev, param);
	}

	return 0;
}

static int mxt_sysfs_init(struct mxt_data *data);
static void mxt_sysfs_remove(struct mxt_data *data);
static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg);

static void mxt_config_cb(const struct firmware *cfg, void *ctx)
{
	mxt_configure_objects(ctx, cfg);
	release_firmware(cfg);
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int recovery_attempts = 0;
	int error;

	while (1) {
		error = mxt_read_info_block(data);
		if (!error)
			break;

		/* Check bootloader state */
		error = mxt_probe_bootloader(data, false);
		if (error) {
			dev_info(&client->dev, "Trying alternate bootloader address\n");
			error = mxt_probe_bootloader(data, true);
			if (error) {
				/* Chip is not in appmode or bootloader mode */
				return error;
			}
		}

		/* OK, we are in bootloader, see if we can recover */
		if (++recovery_attempts > 1) {
			dev_err(&client->dev, "Could not recover from bootloader mode\n");
			/*
			 * We can reflash from this state, so do not
			 * abort initialization.
			 */
			data->in_bootloader = true;
			return 0;
		}

		/* Attempt to exit bootloader into app mode */
		mxt_send_bootloader_cmd(data, false);
		msleep(MXT_FW_RESET_TIME);
	}

	error = mxt_check_retrigen(data);
	if (error)
		goto err_free_object_table;

	error = mxt_acquire_irq(data);
	if (error)
		goto err_free_object_table;

	error = mxt_sysfs_init(data);
	if (error)
		goto err_free_object_table;

	error = mxt_debug_msg_init(data);
	if (error)
		goto err_free_object_table;

	if (data->cfg_name) {
		error = request_firmware_nowait(THIS_MODULE, true,
					data->cfg_name, &data->client->dev,
					GFP_KERNEL, data, mxt_config_cb);
		if (error) {
			dev_err(&client->dev, "Failed to invoke firmware loader: %d\n",
				error);
			goto err_free_object_table;
		}
	} else {
		error = mxt_configure_objects(data, NULL);
		if (error)
			goto err_free_object_table;
	}

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else
		new_config = &data->t7_cfg;

	error = __mxt_write_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), new_config);
	if (error)
		return error;

	dev_dbg(dev, "Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			dev_dbg(dev, "T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
			dev_dbg(dev, "T7 cfg zero after reset, overriding\n");
			data->t7_cfg.active = 20;
			data->t7_cfg.idle = 100;
			return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	}

	dev_dbg(dev, "Initialized power cfg: ACTV %d, IDLE %d\n",
		data->t7_cfg.active, data->t7_cfg.idle);
	return 0;
}

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	int error;

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		dev_err(dev, "Failed to initialize power cfg\n");
		goto err_free_object_table;
	}

	if (cfg) {
		error = mxt_update_cfg(data, cfg);
		if (error)
			dev_warn(dev, "Error %d updating config\n", error);
	}

	if (data->multitouch) {
		error = mxt_initialize_input_device(data);
		if (error)
			goto err_free_object_table;
	} else {
		dev_warn(dev, "No touch object detected\n");
	}

	if (data->T70_address) {
		error = mxt_init_t70_configuration_control(data);
		if (error) {
			TPD_LOG_ERR(data->touch_debug,
					"Error %d failed to init t70 configuration control\n",
					error);
			goto err_free_object_table;
		}
		TPD_LOG_INFO(data->touch_debug,
				"%s: init t70 configuration control OK\n", __func__);
	}

	if (data->wg_params && (data->T92_address || data->T93_address)) {
		error = mxt_init_wakeup_gesture_mode(data);
		if (error) {
			TPD_LOG_ERR(data->touch_debug,
					"Error %d failed to init gwsture wakeup\n",
					error);
			goto err_free_object_table;
		}
		TPD_LOG_INFO(data->touch_debug,
				"%s: init wake up gesture OK\n", __func__);
	}

	data->objects_configured = true;

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t mxt_config_crc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->config_crc);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = mxt_read_blks(data, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/*
	 * To convert file try:
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw
	 */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -EINVAL;
}

static int mxt_enter_bootloader(struct mxt_data *data)
{
	int ret;

	if (data->suspended) {
		if (data->pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
			mxt_regulator_enable(data);
			mxt_process_messages_until_invalid(data);
			enable_irq(data->irq);
		}

		data->suspended = false;
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
				     MXT_BOOT_VALUE, false);
		if (ret)
			return ret;

		msleep(MXT_RESET_TIME);

		/* Do not need to scan since we know family ID */
		ret = mxt_probe_bootloader(data, 0);
		if (ret)
			return ret;

		data->in_bootloader = true;
		mxt_sysfs_remove(data);
		mxt_free_input_device(data);
		mxt_free_object_table(data);
	}

	dev_dbg(&data->client->dev, "Entered bootloader\n");

	return 0;
}

static void mxt_fw_work(struct work_struct *work)
{
	struct mxt_flash *f =
		container_of(work, struct mxt_flash, work.work);

	mxt_check_bootloader(f->data);
}

static int mxt_load_fw(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;

	TPD_LOG_INFO(data->touch_debug, "%s: start\n", __func__);
	data->flash = devm_kzalloc(dev, sizeof(struct mxt_flash), GFP_KERNEL);
	if (!data->flash)
		return -ENOMEM;

	data->flash->data = data;

	ret = request_firmware(&data->flash->fw, data->fw_name, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", data->fw_name);
		goto free;
	}

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, data->flash->fw);
	if (ret)
		goto release_firmware;

	init_completion(&data->flash->flash_completion);
	INIT_DELAYED_WORK(&data->flash->work, mxt_fw_work);

	if (!data->in_bootloader) {
		ret = mxt_enter_bootloader(data);
		if (ret)
			goto release_firmware;
	}

	/* Poll after 0.1s if no interrupt received */
	schedule_delayed_work(&data->flash->work, HZ / 10);

	TPD_LOG_INFO(data->touch_debug, "%s: Wait for flash start\n", __func__);
	/* Wait for flash. */
	ret = mxt_wait_for_completion(data, &data->flash->flash_completion,
				      MXT_BOOTLOADER_WAIT);
	TPD_LOG_INFO(data->touch_debug, "%s: Wait for flash end\n", __func__);

	disable_irq(data->irq);
	cancel_delayed_work_sync(&data->flash->work);
	data->in_bootloader = false;
release_firmware:
	release_firmware(data->flash->fw);
free:
	devm_kfree(dev, data->flash);
	data->flash = NULL;
	TPD_LOG_INFO(data->touch_debug, "%s: end(ret=%d)\n", __func__, ret);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp)
		return -ENOMEM;

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	error = mxt_update_file_name(dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(dev);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		msleep(MXT_RESET_TIMEOUT);

		data->suspended = false;

		error = mxt_initialize(data);
		if (error)
			return error;
	}

	return count;
}

static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct mxt_platform_data *pdata = data->pdata;
	const struct firmware *cfg;
	int ret;

	ret = mxt_update_file_name(dev, &data->cfg_name, buf, count);
	if (ret)
		return ret;

	ret = request_firmware(&cfg, data->cfg_name, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
			data->cfg_name);
		ret = -ENOENT;
		goto out;
	}

	data->updating_config = true;

	mxt_free_input_device(data);

	if (data->suspended) {
		if (pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
			mxt_regulator_enable(data);
			mxt_process_messages_until_invalid(data);
			enable_irq(data->irq);
		} else if (pdata->suspend_mode == MXT_SUSPEND_DEEP_SLEEP) {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg);
	if (ret)
		goto release;

	ret = count;

release:
	release_firmware(cfg);
out:
	data->updating_config = false;
	return ret;
}

static ssize_t mxt_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		if (i == 1)
			mxt_debug_msg_enable(data);
		else
			mxt_debug_msg_disable(data);

		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t mxt_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_suspend_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);
#if defined(CONFIG_FB) || defined(CONFIG_PM_SLEEP)
	mxt_suspend(dev);
#endif
	TPD_LOG_DEBUG(data->touch_debug, "%s: end(%zd)\n", __func__, count);
	return	count;
}

static ssize_t mxt_resume_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);
#if defined(CONFIG_FB) || defined(CONFIG_PM_SLEEP)
	mxt_resume(dev);
#endif
	TPD_LOG_DEBUG(data->touch_debug, "%s: end(%zd)\n", __func__, count);
	return	count;
}

static void mxt_hw_xrst(struct mxt_data *data)
{
	/* Reset Hi */
	gpio_direction_output(data->pdata->gpio_reset, 1);
	/* 1ms wait reset Hi->Lo */
	mdelay(1);
	/* Reset Lo */
	gpio_direction_output(data->pdata->gpio_reset, 0);
	/* 1ms wait reset Lo->Hi */
	mdelay(1);
	/* Reset Hi */
	gpio_direction_output(data->pdata->gpio_reset, 1);
}

static int mxt_reset_wait(struct mxt_data *data)
{
	INIT_COMPLETION(data->reset_completion);

	mxt_hw_xrst(data);

	/* T6 status reset wait */
	return mxt_wait_for_completion(data, &data->reset_completion,
			(MXT_RESET_TIMEOUT + MXT_CHG_DELAY));
}

static ssize_t mxt_hardreset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int				ret;
	struct mxt_data	*data = dev_get_drvdata(dev);

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	ret = mxt_reset_wait(data);
	if (ret) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Failed to reset(%d)\n",
				__func__, ret);
		return	ret;
	}

	TPD_LOG_DEBUG(data->touch_debug, "%s: end(%zd)\n", __func__, count);
	return	count;
}

static ssize_t mxt_softreset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int				ret;
	struct mxt_data	*data = dev_get_drvdata(dev);

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	ret = mxt_soft_reset(data);
	if (ret) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Failed to reset(%d)\n",
				__func__, ret);
		return	ret;
	}

	TPD_LOG_DEBUG(data->touch_debug, "%s: end(%zd)\n", __func__, count);
	return	count;
}

static ssize_t mxt_glove_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	ssize_t size;

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);
	size = snprintf(buf, PAGE_SIZE, "%d\n", data->glove_mode ? 1 : 0);
	TPD_LOG_DEBUG(data->touch_debug, "%s: end(%zd)\n", __func__, size);
	return size;
}

static ssize_t mxt_glove_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	mutex_lock(&data->touchmode_lock);

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		TPD_LOG_ERR(data->touch_debug, "%s: Invalid value\n", __func__);
		retval = -EINVAL;
		goto exit_store;
	}

	if (value != MXT_TOUCH_MODE_ENABLE && value != MXT_TOUCH_MODE_DISABLE) {
		TPD_LOG_ERR(data->touch_debug, "%s: Invalid value\n", __func__);
		retval = -EINVAL;
		goto exit_store;
	}

	data->glove_mode = (value == MXT_TOUCH_MODE_ENABLE) ? true : false;
	tpd_event_change_mode(data->touch_debug, TPD_MODE_GLOVE, value);

	if (data->suspended || data->in_bootloader || data->updating_config) {
		TPD_LOG_ERR(data->touch_debug, "%s: Invalid state\n", __func__);
		retval = size;
		goto exit_store;
	}

	retval = mxt_set_glove_mode(data, value);
	if (retval < 0) {
		TPD_LOG_ERR(data->touch_debug,
				"%s: Failed to set glove mode\n", __func__);
	}
	retval = size;
exit_store:
	mutex_unlock(&data->touchmode_lock);
	TPD_LOG_DEBUG(data->touch_debug, "%s: end(%d)\n", __func__, retval);
	return retval;
}
static ssize_t mxt_wakeup_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	ssize_t size;

	size = snprintf(buf, PAGE_SIZE, "%d\n", data->wakeup_gesture_mode);

	return size;
}

static ssize_t mxt_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	mutex_lock(&data->touchmode_lock);

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		TPD_LOG_ERR(data->touch_debug, "%s: Invalid value\n", __func__);
		retval = -EINVAL;
		goto exit_store;
	}

	(value > 0) ? (value = MXT_GESTURE_DOUBLE_TAP) : 0;

	data->wakeup_gesture_mode = value;
	tpd_event_change_mode(data->touch_debug, TPD_MODE_WAKEUP_GESTURE, value);
	TPD_LOG_DEBUG(data->touch_debug, "%s: Wake up gesture %sabled\n",
			__func__, (value != MXT_GESTURE_OFF) ? "En" : "Dis");

	retval = size;
exit_store:
	mutex_unlock(&data->touchmode_lock);
	TPD_LOG_DEBUG(data->touch_debug, "%s: end(%d)\n", __func__, retval);
	return retval;
}

static ssize_t mxt_wakeup_detect_area_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	TPD_LOG_WARN(data->touch_debug,
			"%s No support attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static ssize_t mxt_wakeup_detect_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	TPD_LOG_WARN(data->touch_debug,
			"%s No support attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static ssize_t mxt_initialize_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int				ret;
	struct mxt_data	*data = dev_get_drvdata(dev);

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	if (data->suspended) {
		if (data->pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
			mxt_regulator_enable(data);
			mxt_process_messages_until_invalid(data);
		}
		data->suspended = false;
	} else {
		disable_irq(data->irq);
	}

	data->updating_config = true;
	data->in_bootloader = false;

	mxt_sysfs_remove(data);
	mxt_free_input_device(data);
	mxt_free_object_table(data);

	ret = mxt_initialize(data);
	if (ret) {
		data->updating_config = false;
		TPD_LOG_ERR(data->touch_debug,
				"%s: Failed to init(%d)\n",
				__func__, ret);
		return	ret;
	}

	data->updating_config = false;

	TPD_LOG_DEBUG(data->touch_debug, "%s: end(%zd)\n", __func__, count);
	return	count;
}

static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static DEVICE_ATTR(initialize, S_IWUSR, NULL, mxt_initialize_store);

static struct attribute *mxt_fw_attrs[] = {
	&dev_attr_update_fw.attr,
	&dev_attr_initialize.attr,
	NULL
};

static const struct attribute_group mxt_fw_attr_group = {
	.attrs = mxt_fw_attrs,
};

static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(config_crc, S_IRUGO, mxt_config_crc_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
		   mxt_debug_enable_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, NULL,
		   mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_debug_notify_show, NULL);
static DEVICE_ATTR(sleep, S_IWUSR, NULL, mxt_suspend_store);
static DEVICE_ATTR(wake, S_IWUSR, NULL, mxt_resume_store);
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, mxt_hardreset_store);
static DEVICE_ATTR(sw_reset, S_IWUSR, NULL, mxt_softreset_store);

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_object.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_config_crc.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_sleep.attr,
	&dev_attr_wake.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_sw_reset.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static struct device_attribute attributes_touch_mode[] = {
	__ATTR(glove_mode, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			mxt_glove_mode_show, mxt_glove_mode_store),
	__ATTR(wakeup_gesture, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			mxt_wakeup_gesture_show, mxt_wakeup_gesture_store),
	__ATTR(wakeup_detect_area, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			mxt_wakeup_detect_area_show, mxt_wakeup_detect_area_store),
};

static int mxt_add_touch_interfaces(struct mxt_data *data)
{
	int rc, i;
	struct device *dev;

	data->setting_class = class_create(THIS_MODULE, "touchscreen");
	if (IS_ERR(data->setting_class)) {
		TPD_LOG_ERR(data->touch_debug, "%s: Failed to create touchpanel class.\n", __func__);
		return -ENODEV;
	}

	dev = device_create(data->setting_class, NULL, 0, data, "touch");
	if (IS_ERR(dev)) {
		TPD_LOG_ERR(data->touch_debug, "%s: Failed to create char device.\n", __func__);
		rc = -ENODEV;
		goto err_device_create;
	}

	data->touch_mode_dev_path = dev;

	for (i = 0; i < ARRAY_SIZE(attributes_touch_mode); i++) {
		rc = device_create_file(dev, attributes_touch_mode + i);
		if (rc) {
			TPD_LOG_ERR(data->touch_debug, "%s: Failed to add device interfaces(%d).\n", __func__, rc);
			rc = -ENODEV;
			goto err_add_interfaces;
		}
	}

	TPD_LOG_INFO(data->touch_debug, "%s: Success to add touchpanel interfaces.\n", __func__);
	return 0;

err_add_interfaces:
	for (i--; i >= 0; i--) {
		device_remove_file(dev, attributes_touch_mode + i);
	}
	device_destroy(data->setting_class, MKDEV(MAJOR(data->setting_dev_num), 0));
err_device_create:
	class_destroy(data->setting_class);
	return rc;
}

static int mxt_remove_touch_interfaces(struct mxt_data *data)
{
	int i;
	struct device *dev;

	dev = data->touch_mode_dev_path;
	if (!dev) {
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(attributes_touch_mode); i++) {
		device_remove_file(dev, attributes_touch_mode + i);
	}

	device_destroy(data->setting_class, MKDEV(MAJOR(data->setting_dev_num), 0));
	class_destroy(data->setting_class);

	return 0;
}

static int mxt_sysfs_init(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
			error);
		return error;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	error = sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr);
	if (error) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

	return 0;

err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	return error;
}

static void mxt_sysfs_remove(struct mxt_data *data)
{
	struct i2c_client *client = data->client;

	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
				      &data->mem_access_attr);

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
}

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	unsigned int num_mt_slots;
	int id;

	if (!input_dev)
		return;

	num_mt_slots = data->num_touchids + data->num_stylusids;

	for (id = 0; id < num_mt_slots; id++) {
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, 0, 0);
	}

	mxt_input_sync(data);
}

static int mxt_start(struct mxt_data *data)
{
	int ret;
	enum mxt_suspend_mode suspend_mode;

	if (!data->suspended || data->in_bootloader)
		return 0;

	if (data->wg_state == MXT_GESTURE_STATE_ENABLE) {
		suspend_mode = MXT_SUSPEND_WAKEUP_GESTURE;
	} else {
		suspend_mode = data->pdata->suspend_mode;
	}

	switch (suspend_mode) {
	case MXT_SUSPEND_WAKEUP_GESTURE:
		data->wg_state = MXT_GESTURE_STATE_EXIT;
		mxt_set_wakeup_gesture_mode(data, false);
		disable_irq_wake(data->irq);
		mxt_touchmode_reconfigure(data);
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
		data->wg_state = MXT_GESTURE_STATE_DISABLE;
		TPD_LOG_INFO(data->touch_debug,
				"%s: exit wakeup gesture\n", __func__);
		break;

	case MXT_SUSPEND_T9_CTRL:
		mxt_soft_reset(data);

		/* Touch enable */
		/* 0x83 = SCANEN | RPTEN | ENABLE */
		mxt_write_object(data,
				MXT_TOUCH_MULTI_T9, MXT_T9_CTRL, 0x83);
		break;

	case MXT_SUSPEND_REGULATOR:
		mxt_regulator_enable(data);
		/*
		 * Read messages for detect reset, and then reconfigure
		 * current touch mode
		 */
		mxt_process_messages_until_invalid(data);
		enable_irq(data->irq);
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
		break;

	case MXT_SUSPEND_DEEP_SLEEP:
	default:
		/*
		 * Discard any touch messages still in message buffer
		 * from before chip went to sleep
		 */
		mxt_process_messages_until_invalid(data);

		ret = mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		if (ret)
			return ret;

		/* Recalibrate since chip has been in deep sleep */
		ret = mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
		if (ret)
			return ret;

		ret = mxt_acquire_irq(data);
		if (ret)
			return ret;

		break;
	}

	data->suspended = false;
	tpd_event_resume(data->touch_debug);

	return 0;
}

static int mxt_stop(struct mxt_data *data)
{
	int ret;
	enum mxt_suspend_mode suspend_mode;


	if (data->suspended || data->in_bootloader || data->updating_config) {
		return 0;
	}

	mutex_lock(&data->touchmode_lock);

	if (mxt_wakeup_gesture_enabled(data)) {
		suspend_mode = MXT_SUSPEND_WAKEUP_GESTURE;
	} else {
		suspend_mode = data->pdata->suspend_mode;
	}

	switch (suspend_mode) {
	case MXT_SUSPEND_WAKEUP_GESTURE:
		disable_irq(data->irq);
		mxt_set_glove_mode(data, false);
		mxt_set_wakeup_gesture_mode(data, true);
		enable_irq_wake(data->irq);
		mxt_reset_slots(data);
		data->wg_state = MXT_GESTURE_STATE_ENABLE;
		enable_irq(data->irq);
		TPD_LOG_INFO(data->touch_debug,
				"%s: enter wakeup gesture\n", __func__);
		break;

	case MXT_SUSPEND_T9_CTRL:
		/* Touch disable */
		ret = mxt_write_object(data,
				MXT_TOUCH_MULTI_T9, MXT_T9_CTRL, 0);
		if (ret) {
			mutex_unlock(&data->touchmode_lock);
			return ret;
		}
		break;

	case MXT_SUSPEND_REGULATOR:
		disable_irq(data->irq);
		mxt_regulator_disable(data);
		mxt_reset_slots(data);
		break;

	case MXT_SUSPEND_DEEP_SLEEP:
	default:
		disable_irq(data->irq);

		ret = mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
		if (ret) {
			mutex_unlock(&data->touchmode_lock);
			return ret;
		}
		mxt_reset_slots(data);
		break;
	}

	data->suspended = true;
	mutex_unlock(&data->touchmode_lock);
	tpd_event_suspend(data->touch_debug);

	TPD_LOG_INFO(data->touch_debug, "%s: SCRSTAT(%x) rpt(%d) tch(%d) atch(%d) intthr(%d)\n",
			 __func__, data->scraux_data[0], data->scraux_data[1],
			(data->scraux_data[3] << 8) + data->scraux_data[2],
			(data->scraux_data[5] << 8) + data->scraux_data[4],
			(data->scraux_data[7] << 8) + data->scraux_data[6]);
	memset(data->scraux_data, 0, sizeof(data->scraux_data));

	return 0;
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int ret;

	ret = mxt_start(data);

	if (ret)
		dev_err(&data->client->dev, "%s failed rc=%d\n", __func__, ret);

	return ret;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int ret;

	ret = mxt_stop(data);

	if (ret)
		dev_err(&data->client->dev, "%s failed rc=%d\n", __func__, ret);
}

static void mxt_async_start_work(struct work_struct *work)
{
	struct mxt_data *data = container_of(work, struct mxt_data, async_start_work.work);
	struct input_dev *input_dev = data->input_dev;

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	if (!input_dev)
		return;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_start(data);

	mutex_unlock(&input_dev->mutex);
	TPD_LOG_DEBUG(data->touch_debug, "%s: end\n", __func__);
}

#if defined(CONFIG_FB)
static int mxt_fb_notifier_cb(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct mxt_data *mxt_dev_data =
		container_of(self, struct mxt_data, fb_notif);

	if (evdata && evdata->data && mxt_dev_data && mxt_dev_data->client) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) {
				mxt_resume(&mxt_dev_data->client->dev);
			} else if (*blank == FB_BLANK_POWERDOWN) {
				mxt_suspend(&mxt_dev_data->client->dev);
			}
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_OF
static const struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;
	struct device_node *np = client->dev.of_node;
	u32 *keymap;
	int proplen, ret;

	if (!np)
		return ERR_PTR(-ENOENT);

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	ret = of_get_named_gpio_flags(np, "atmel,reset-gpio", 0, NULL);
	if (ret < 0) {
		pr_err("[TPD_dt]failed to set gpio_ret:%d\n",ret);
		return NULL;
	}
	pdata->gpio_reset = (unsigned)ret;
	pr_info("[TPD_dt]gpio_reset:%d\n",pdata->gpio_reset);

	if (of_find_property(np, "atmel,irq-flags", NULL)) {
		of_property_read_u32(np, "atmel,irq-flags", &ret);
		pr_info("[TPD_dt]irqflags:%d\n", ret);
		pdata->irqflags = ret;
	}

	ret = gpio_request(pdata->gpio_reset, "mxt_reset_gpio");
	if (ret) {
		pr_err("[TPD_dbg]failed to request gpio_reset:%d\n", ret);
		return NULL;
	}

	pdata->ts_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(pdata->ts_pinctrl)) {
		pr_err("[TPD_dt]failed to use pinctrl\n");
		return NULL;
	}
	pdata->pinctrl_state_irq_active
		= pinctrl_lookup_state(pdata->ts_pinctrl, "mxt_irq_active");
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_irq_active)) {
		pr_err("[TPD_dt]Can not lookup irq active pinstate\n");
		return NULL;
	}
	pdata->pinctrl_state_irq_suspend
		= pinctrl_lookup_state(pdata->ts_pinctrl, "mxt_irq_default");
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_irq_suspend)) {
		pr_err("[TPD_dt]Can not lookup irq suspend pinstate\n");
		return NULL;
	}
	pdata->pinctrl_state_rst_active
		= pinctrl_lookup_state(pdata->ts_pinctrl, "mxt_rst_active");
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_rst_active)) {
		pr_err("[TPD_dt]Can not lookup reset active pinstate\n");
		return NULL;
	}
	pdata->pinctrl_state_rst_suspend
		= pinctrl_lookup_state(pdata->ts_pinctrl, "mxt_rst_default");
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_rst_suspend)) {
		pr_err("[TPD_dt]Can not lookup reset suspend pinstate\n");
		return NULL;
	}
	pdata->pinctrl_state_i2c_active
		= pinctrl_lookup_state(pdata->ts_pinctrl, "mxt_i2c_active");
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_i2c_active)) {
		pr_err("[TPD_dt]Can not lookup i2c active pinstate\n");
		return NULL;
	}
	pdata->pinctrl_state_i2c_suspend
		= pinctrl_lookup_state(pdata->ts_pinctrl, "mxt_i2c_default");
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_i2c_suspend)) {
		pr_err("[TPD_dt]Can not lookup i2c suspend pinstate\n");
		return NULL;
	}

	ret = of_property_read_string(np, "atmel,vdd-reg-name", &pdata->vdd_name);
	if (ret < 0) {
		pr_err("[TPD_dt]failed to read vdd-reg-name:%d\n", ret);
		return NULL;
	}
	pr_info("[TPD_dt]vdd_name:%s\n",pdata->vdd_name);

	ret = of_property_read_string(np, "atmel,avdd-reg-name", &pdata->avdd_name);
	if (ret < 0) {
		pr_err("[TPD_dt]failed to read avdd-reg-name:%d\n", ret);
		return NULL;
	}
	pr_info("[TPD_dt]avdd_name:%s\n",pdata->avdd_name);

	ret = of_property_read_u32_array(np, "atmel,avdd-reg-voltage",
			pdata->avdd_vol, 2);
	if (ret < 0) {
		pr_err("[TPD_dt]failed to read avdd-reg-voltage:%d\n", ret);
		return NULL;
	}
	pr_info("[TPD_dt]avdd_vol:(%d-%d)\n",pdata->avdd_vol[0], pdata->avdd_vol[1]);

	ret = of_property_read_u32_array(np, "atmel,avdd-reg-optimum-uv",
			pdata->avdd_optimum_uv, 2);
	if (ret < 0) {
		pr_err("[TPD_dt]failed to read atmel,avdd-reg-optimum-uv:%d\n", ret);
		return NULL;
	}
	pr_info("[TPD_dt]avdd_optimum_uv:(off:%d/on:%d)\n",
			pdata->avdd_optimum_uv[0], pdata->avdd_optimum_uv[1]);

	of_property_read_string(np, "atmel,cfg_name", &pdata->cfg_name);

	of_property_read_string(np, "atmel,input_name", &pdata->input_name);

	if (of_find_property(np, "linux,gpio-keymap", &proplen)) {
		pdata->t19_num_keys = proplen / sizeof(u32);

		keymap = devm_kzalloc(&client->dev,
				pdata->t19_num_keys * sizeof(keymap[0]),
				GFP_KERNEL);
		if (!keymap)
			return ERR_PTR(-ENOMEM);

		ret = of_property_read_u32_array(np, "linux,gpio-keymap",
						 keymap, pdata->t19_num_keys);
		if (ret)
			dev_warn(&client->dev,
				 "Couldn't read linux,gpio-keymap: %d\n", ret);

		pdata->t19_keymap = keymap;
	}

	of_property_read_u32(np, "atmel,suspend-mode", &pdata->suspend_mode);

	pdata->async_start_info = of_find_property(np, "atmel,async-start", NULL);

	pr_info("[TPD_dt]cfg_name:(%s)\n", pdata->cfg_name);
	pr_info("[TPD_dt]input_name:(%s)\n", pdata->input_name);
	pr_info("[TPD_dt]suspend-mode:(%d)\n", pdata->suspend_mode);
	pr_info("[TPD_dt]async_start:(%s)\n",
			(pdata->async_start_info ? "t" : "f"));

	return pdata;
}
#else
static const struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	return ERR_PTR(-ENOENT);
}
#endif

#ifdef CONFIG_ACPI

struct mxt_acpi_platform_data {
	const char *hid;
	struct mxt_platform_data pdata;
};

static unsigned int samus_touchpad_buttons[] = {
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	BTN_LEFT
};

static struct mxt_acpi_platform_data samus_platform_data[] = {
	{
		/* Touchpad */
		.hid	= "ATML0000",
		.pdata	= {
			.t19_num_keys	= ARRAY_SIZE(samus_touchpad_buttons),
			.t19_keymap	= samus_touchpad_buttons,
		},
	},
	{
		/* Touchscreen */
		.hid	= "ATML0001",
	},
	{ }
};

static const struct dmi_system_id mxt_dmi_table[] = {
	{
		/* 2015 Google Pixel */
		.ident = "Chromebook Pixel 2",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "GOOGLE"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Samus"),
		},
		.driver_data = samus_platform_data,
	},
	{ }
};

static const struct mxt_platform_data *mxt_parse_acpi(struct i2c_client *client)
{
	struct acpi_device *adev;
	const struct dmi_system_id *system_id;
	const struct mxt_acpi_platform_data *acpi_pdata;

	/*
	 * Ignore ACPI devices representing bootloader mode.
	 *
	 * This is a bit of a hack: Google Chromebook BIOS creates ACPI
	 * devices for both application and bootloader modes, but we are
	 * interested in application mode only (if device is in bootloader
	 * mode we'll end up switching into application anyway). So far
	 * application mode addresses were all above 0x40, so we'll use it
	 * as a threshold.
	 */
	if (client->addr < 0x40)
		return ERR_PTR(-ENXIO);

	adev = ACPI_COMPANION(&client->dev);
	if (!adev)
		return ERR_PTR(-ENOENT);

	system_id = dmi_first_match(mxt_dmi_table);
	if (!system_id)
		return ERR_PTR(-ENOENT);

	acpi_pdata = system_id->driver_data;
	if (!acpi_pdata)
		return ERR_PTR(-ENOENT);

	while (acpi_pdata->hid) {
		if (!strcmp(acpi_device_hid(adev), acpi_pdata->hid))
			return &acpi_pdata->pdata;

		acpi_pdata++;
	}

	return ERR_PTR(-ENOENT);
}
#else
static const struct mxt_platform_data *mxt_parse_acpi(struct i2c_client *client)
{
	return ERR_PTR(-ENOENT);
}
#endif

static struct mxt_platform_data *mxt_default_pdata(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	/* Set default parameters */
	pdata->irqflags = IRQF_TRIGGER_FALLING;

	return pdata;
}

static const struct mxt_platform_data *
mxt_get_platform_data(struct i2c_client *client)
{
	const struct mxt_platform_data *pdata;

	pdata = dev_get_platdata(&client->dev);
	if (pdata)
		return pdata;

	pdata = mxt_parse_dt(client);
	if (!IS_ERR(pdata) || PTR_ERR(pdata) != -ENOENT)
		return pdata;

	pdata = mxt_parse_acpi(client);
	if (!IS_ERR(pdata) || PTR_ERR(pdata) != -ENOENT)
		return pdata;

	pdata = mxt_default_pdata(client);
	if (!IS_ERR(pdata))
		return pdata;

	dev_err(&client->dev, "No platform data specified\n");
	return ERR_PTR(-EINVAL);
}

static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mxt_data *data;
	const struct mxt_platform_data *pdata;
	int error;

	pr_info("[TPD]%s: start\n", __func__);

	if (charging_mode) {
		pr_err("%s: *** off charging mode ***\n", __func__);
		return -ENOSYS;
	}

	pdata = mxt_get_platform_data(client);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	data->client = client;
	data->pdata = pdata;
	i2c_set_clientdata(client, data);

	error = tpd_init(&data->touch_debug);
	if (error) {
		pr_err("[TPD]E:%s: Failed to init TPD\n", __func__);
		goto err_free_mem;
	}

	error = mxt_parse_wg_params(data);
	if (error) {
		TPD_LOG_WARN(data->touch_debug,
				"%s: Failed to get Wake Up Gesture parameter\n", __func__);
	}

	/* CONFIG_PARAM disables updating config when probing */
	if (!tpd_get_debug_state(data->touch_debug, TPD_DEBUG_ID_CONFIG_PARAM) &&
			data->pdata->cfg_name) {
		mxt_update_file_name(&data->client->dev,
				     &data->cfg_name,
				     data->pdata->cfg_name,
				     strlen(data->pdata->cfg_name));
	}

	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	mutex_init(&data->debug_msg_lock);
	mutex_init(&data->touchmode_lock);

	if (pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
		error = mxt_acquire_irq(data);
		if (error)
			goto err_free_irq;

		disable_irq(data->irq);

		error = mxt_probe_regulators(data);
		if (error)
			goto err_free_irq;
	}

	error = mxt_initialize(data);
	if (error)
		goto err_free_reg;

	error = sysfs_create_group(&client->dev.kobj, &mxt_fw_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating fw sysfs group\n",
			error);
		return error;
	}

	error = mxt_add_touch_interfaces(data);
	if (error) {
		pr_err("[TPD]%s: Unable to add touch interfaces: %d\n",__func__, error);
		goto err_free_sysfs;
	}

	if (data->pdata->async_start_info) {
		data->async_start_queue = create_singlethread_workqueue("async_start_queue");
		INIT_DELAYED_WORK(&data->async_start_work, mxt_async_start_work);
	}
#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = mxt_fb_notifier_cb;
	error = fb_register_client(&data->fb_notif);
	if (error) {
		pr_err("[TPD]%s: Unable to register fb_notifier: %d\n",__func__, error);
		goto err_free_sysfs;
	}
#endif

	pr_info("[TPD]%s: OK\n", __func__);
	return 0;

err_free_sysfs:
	mxt_remove_touch_interfaces(data);
	sysfs_remove_group(&client->dev.kobj, &mxt_fw_attr_group);
	mxt_sysfs_remove(data);
err_free_reg:
	regulator_put(data->reg_avdd);
	regulator_put(data->reg_vdd);
err_free_irq:
	if (data->irq)
		free_irq(data->irq, data);
	mxt_free_wg_params(data);
	if (data->touch_debug)
		tpd_exit(data->touch_debug);
err_free_mem:
	kfree(data);
	pr_info("[TPD]%s: failed error=%d\n", __func__, error);
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	return 0;
}

static void mxt_shutdown(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	pr_info("[TPD]%s: start\n", __func__);
#if defined(CONFIG_FB)
	fb_unregister_client(&data->fb_notif);
#endif
	if (data->pdata->async_start_info) {
		cancel_delayed_work_sync(&data->async_start_work);
		destroy_workqueue(data->async_start_queue);
	}
	mxt_remove_touch_interfaces(data);
	sysfs_remove_group(&client->dev.kobj, &mxt_fw_attr_group);
	mxt_sysfs_remove(data);

	if (data->irq)
		free_irq(data->irq, data);

	data->wakeup_gesture_mode = MXT_GESTURE_OFF;
	mxt_free_input_device(data);
	mxt_free_object_table(data);
	regulator_put(data->reg_avdd);
	regulator_put(data->reg_vdd);
	mxt_free_wg_params(data);
	if (data->touch_debug)
		tpd_exit(data->touch_debug);
	kfree(data);

	pr_info("[TPD]%s: end\n", __func__);
	return;
}

static int mxt_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	if (data->wg_state == MXT_GESTURE_STATE_ENABLE) {
		disable_irq(data->irq);
		data->wg_state = MXT_GESTURE_STATE_SLEEP;
	}

	TPD_LOG_DEBUG(data->touch_debug, "%s: end\n", __func__);

	return 0;
}

static int mxt_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	if (data->wg_state == MXT_GESTURE_STATE_SLEEP) {
		data->wg_state = MXT_GESTURE_STATE_ENABLE;
		enable_irq(data->irq);
	}

	TPD_LOG_DEBUG(data->touch_debug, "%s: end\n", __func__);

	return 0;
}

static int __maybe_unused mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	if (data->pdata->async_start_info) {
		cancel_delayed_work_sync(&data->async_start_work);
	}

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data);

	mutex_unlock(&input_dev->mutex);

	TPD_LOG_DEBUG(data->touch_debug, "%s: end\n", __func__);
	return 0;
}

static int __maybe_unused mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	TPD_LOG_DEBUG(data->touch_debug, "%s: start\n", __func__);

	if (data->pdata->async_start_info) {
		queue_delayed_work(data->async_start_queue,
				&data->async_start_work, msecs_to_jiffies(10));
	} else {
		mutex_lock(&input_dev->mutex);

		if (input_dev->users)
			mxt_start(data);

		mutex_unlock(&input_dev->mutex);
	}

	TPD_LOG_DEBUG(data->touch_debug, "%s: end\n", __func__);
	return 0;
}

static const struct dev_pm_ops mxt_pm_ops = {
	.suspend = mxt_pm_suspend,
	.resume  = mxt_pm_resume,
};

static const struct of_device_id mxt_of_match[] = {
	{ .compatible = "atmel,maxtouch", },
	{},
};
MODULE_DEVICE_TABLE(of, mxt_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id mxt_acpi_id[] = {
	{ "ATML0000", 0 },	/* Touchpad */
	{ "ATML0001", 0 },	/* Touchscreen */
	{ }
};
MODULE_DEVICE_TABLE(acpi, mxt_acpi_id);
#endif

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mxt_of_match),
		.acpi_match_table = ACPI_PTR(mxt_acpi_id),
		.pm	= &mxt_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.shutdown = mxt_shutdown,
	.id_table	= mxt_id,
};

module_i2c_driver(mxt_driver);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
