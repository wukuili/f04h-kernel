/*----------------------------------------------------------------------------*/
/* COPYRIGHT(C) FUJITSU LIMITED 2014-2015                                     */
/* COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016                   */
/*----------------------------------------------------------------------------*/
/* TPD(Touch Panel Debug) modules */

#include <asm/unaligned.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/nonvolatile_common.h>
#include <linux/syscalls.h>

#include "touch_debug.h"

#define TPD_COUNT_MAX_16         0xFFFF
#define TPD_COUNT_MAX_8          0xFF

#define FJ_TOUCH_RECLOG(fmt, ...)		printk("REC@REC@0 " fmt, ##__VA_ARGS__)

/***************/
/* FTA Strings */
/***************/
#define TPD_FTA_MSG_LENGTH_MAX  128
#define TPD_FTA_EVENT_LOG_SIZE   64
#define TPD_FTA_DEVICE_LOG_SIZE 128
#define TPD_FTA_MODE_LOG_SIZE  (TPD_FTA_MSG_LENGTH_MAX - TPD_FTA_EVENT_LOG_SIZE)

/***************/
/* LED Control */
/***************/
/* Turn up the LED(Reset) */
#define TPD_LED_OFF     (u8)0x00
#define TPD_LED_RED     (u8)0x01
#define TPD_LED_GREEN   (u8)0x02
#define TPD_LED_BLUE    (u8)0x04
#define TPD_LED_WHITE   (u8)0x07

/* Turn up the LED(Touch Event) */
#define TPD_LED_ACTION_DOWN  TPD_LED_RED
#define TPD_LED_ACTION_MOVE  TPD_LED_BLUE
#define TPD_LED_ACTION_UP    TPD_LED_OFF

#define TPD_LED_EVENT_FINGER  TPD_LED_RED
#define TPD_LED_EVENT_GLOVE   TPD_LED_GREEN
#define TPD_LED_EVENT_HOVER   TPD_LED_BLUE
#define TPD_LED_EVENT_PEN     TPD_LED_WHITE

/* 2015-12-15 161-LED-SYSFS-SUPPORT add start */
#define TPD_LED_SYSFS_R "/sys/class/leds/red/brightness"
#define TPD_LED_SYSFS_G "/sys/class/leds/green/brightness"
#define TPD_LED_SYSFS_B "/sys/class/leds/blue/brightness"
#define TPD_LED_BRIGHTNESS_ON  (0xFF)
#define TPD_LED_BRIGHTNESS_OFF (0x00)
#define TPD_SYSFS_BUF_SIZE   40
/* 2015-12-15 161-LED-SYSFS-SUPPORT add end */

/***************/
/* Log Strings */
/***************/
#define TPD_LOG_DUMP_BUFFER_LENGTH  16    /* number of data per line (16 or 32) */
#define TPD_LOG_LINE_LENGTH         256   /* line buffer size */

/***************/
/* Touch Event */
/***************/

enum tpd_event_type{
	TPD_EVENT_ENTER_IRQ,
	TPD_EVENT_REPORT_START,
	TPD_EVENT_TOUCH,
	TPD_EVENT_POTITION_X,
	TPD_EVENT_POTITION_Y,
	TPD_EVENT_REPORT_END,
	TPD_EVENT_EXIT_IRQ,
};

/***************/
/* Nonvolatile */
/***************/

/* Data Block1 - Debug Flags */
#define		TPD_NV_BLOCK1_ID				0xA064
#define		TPD_NV_BLOCK1_SIZE				16
/* Data Block2 - reserved */
#define		TPD_NV_BLOCK2_ID				0xA065
#define		TPD_NV_BLOCK2_SIZE				8
/* Data Block3 - reserved */
#define		TPD_NV_BLOCK3_ID				0xA066
#define		TPD_NV_BLOCK3_SIZE				64

/* Block1 */
#define TPD_NV_RECORD_DRIVER      0
#define TPD_NV_RECORD_PARAM       1
#define TPD_NV_RECORD_LOG_LEVEL   2
#define TPD_NV_RECORD_TOOL        3
#define TPD_NV_RECORD_LED         4
#define TPD_NV_RECORD_CORRECTION  5

#define TPD_NV_ID_UNSUPPORT  TPD_DEBUG_ID_MAX

struct tpd_nonvolatile_info {
	u8 raw_data1[TPD_NV_BLOCK1_SIZE];
	u8 raw_data2[TPD_NV_BLOCK2_SIZE];
	u8 raw_data3[TPD_NV_BLOCK3_SIZE];
};

struct tpd_nv_block{
	u8          id;
	u8          record;
	u8          mask;
	u8          shift;
	const char *name;
};

static const struct tpd_nv_block nv_block1[TPD_DEBUG_ID_MAX] = {
	/* id                          , record                  , mask,sft, name              */
	{ TPD_DEBUG_ID_TOUCH_DISABLE   , TPD_NV_RECORD_DRIVER    , 0x01,  0, "touch_disable"   },
	{ TPD_DEBUG_ID_SUSPEND_MODE    , TPD_NV_RECORD_DRIVER    , 0x02,  1, "suspend_mode "   },
	{ TPD_DEBUG_ID_STARTUP_CALIB   , TPD_NV_RECORD_DRIVER    , 0x04,  2, "startup_calib"   },
/* FCNT LIMITED: 2016-03-23 161_WG_NAME mod start */
 	{ TPD_DEBUG_ID_WAKEUP_GESTURE  , TPD_NV_RECORD_DRIVER    , 0x10,  4, "wakeup_gesture"  },
/* FCNT LIMITED: 2016-03-23 161_WG_NAME mod end */
	{ TPD_DEBUG_ID_CONFIG_PARAM    , TPD_NV_RECORD_PARAM     , 0x01,  0, "config_parm"     },
	{ TPD_DEBUG_ID_CHARGER_SETTING , TPD_NV_RECORD_PARAM     , 0x02,  1, "charging"        },
	{ TPD_DEBUG_ID_MODE_SETTING    , TPD_NV_RECORD_PARAM     , 0x04,  2, "mode_setting"    },
	{ TPD_DEBUG_ID_LOG_DEVICE      , TPD_NV_RECORD_LOG_LEVEL , 0x01,  0, "log_device"      },
	{ TPD_DEBUG_ID_LOG_DEBUG       , TPD_NV_RECORD_LOG_LEVEL , 0x02,  1, "log_debug"       },
	{ TPD_DEBUG_ID_SYSTRACE_OUTPUT , TPD_NV_RECORD_LOG_LEVEL , 0x04,  2, "trace_output"    },
	{ TPD_DEBUG_ID_BENCHMARK_LOG   , TPD_NV_RECORD_LOG_LEVEL , 0x08,  3, "benchmark_log"   },
	{ TPD_DEBUG_ID_MESSAGE_LOG     , TPD_NV_RECORD_LOG_LEVEL , 0x10,  4, "message_log"     },
	{ TPD_DEBUG_ID_TOOL_ENABLE     , TPD_NV_RECORD_TOOL      , 0x01,  0, "tool_enable"     },
	{ TPD_DEBUG_ID_LED_ON_RESET    , TPD_NV_RECORD_LED       , 0x01,  0, "led_on_reset"    },
	{ TPD_DEBUG_ID_TOUCH_EVENT_LED , TPD_NV_RECORD_LED       , 0x02,  1, "touch_event_led" },
	{ TPD_DEBUG_ID_TOOL_TYPE_LED   , TPD_NV_RECORD_LED       , 0x04,  2, "tool_type_led"   },
	{ TPD_DEBUG_ID_GLOVE_CORRECTION, TPD_NV_RECORD_CORRECTION, 0x01,  0, "glove_correction"}
};

/***************/
/* Structs     */
/***************/

struct tpd_event_count {
	u16 count;
	u8  err;
	u8  reset;
	u8  info;	/* 2015-01-19 151_INITIAL_DEVICE_INFO add */
};

struct tpd_device_info {
	u32 firm;
	u32 config;
	u32 check;
};

struct tpd_mode_type {
	u32 type;
};

struct tpd_debug_info {
	struct tpd_nonvolatile_info nv_info;
	u8                          debug_data[TPD_DEBUG_ID_MAX];
};

/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add start */
struct tpd_led_info {
	int down_x;
	int down_y;
	int x;
	int y;
	int event;
	u8 led_on;
};
/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add end */

struct tpd_status_info {
	struct tpd_event_count event_count;
	struct tpd_device_info device_info;
	struct tpd_mode_type   mode_type;
/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add start */
	struct tpd_led_info    led_info;
/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add end */
};

struct tpd_infomation {
	struct tpd_debug_info debug_info;
	struct tpd_status_info status_info;
};

static inline struct tpd_infomation* _tpd_get_infomation(void *handle)
{
	if(!handle)
		return NULL;

	return (struct tpd_infomation *)handle;
}


static void tpd_send_buf(char *buf)
{
	FJ_TOUCH_RECLOG("[TPD] : %s", buf);
}

static void tpd_dump(u8 *buf, int size, const char *data_name)
{
	int i, line_point = 0;
	u8	tmp_buf[TPD_LOG_LINE_LENGTH];

	if(data_name){
		pr_info("[TPD_RAW] : [%s] size = %d\n", data_name, size);
	}
	memset(&tmp_buf[0], 0, sizeof(*tmp_buf));

	for (i = 0; i < size; i++, buf++) {
		if (line_point == 0){
			line_point = snprintf( &tmp_buf[0], TPD_LOG_LINE_LENGTH, "[TPD_RAW] : %08X | ", i);
		}
		line_point += snprintf( &tmp_buf[line_point], (TPD_LOG_LINE_LENGTH - line_point), "%02X ", *buf);
		if ((i % TPD_LOG_DUMP_BUFFER_LENGTH) == (TPD_LOG_DUMP_BUFFER_LENGTH - 1)) {
			pr_info("%s\n", &tmp_buf[0] );
			line_point = 0;
			memset (&tmp_buf[0], 0, sizeof(*tmp_buf));
		}
	}
	if ((i % TPD_LOG_DUMP_BUFFER_LENGTH) != 0) {
		pr_info("%s\n", &tmp_buf[0] );
	}
}

static void tpd_create_mode_str(struct tpd_mode_type *mode_type, char *buf)
{
	if(mode_type->type == TPD_MODE_NORMAL){
		scnprintf(buf, TPD_FTA_MODE_LOG_SIZE, "  ");
		return;
	}

	if(mode_type->type == TPD_MODE_UNKNOWN){
		scnprintf(buf, TPD_FTA_MODE_LOG_SIZE, "Mode([Unknown])");
	} else {
		scnprintf(buf, TPD_FTA_MODE_LOG_SIZE, "Mode(%s%s%s%s%s%s%s%s)",
			mode_type->type & TPD_MODE_DEFAULT     ? "[DEF]" : "",
			mode_type->type & TPD_MODE_CHARGE      ? "[CHG]" : "",
			mode_type->type & TPD_MODE_HIGH_CHARGE ? "[HiC]" : "",
			mode_type->type & TPD_MODE_PRESS       ? "[PRS]" : "",
			mode_type->type & TPD_MODE_GLOVE       ? "[GLV]" : "",
			mode_type->type & TPD_MODE_HOVER       ? "[HOV]" : "",
			mode_type->type & TPD_MODE_FEP         ? "[FEP]" : "",
			mode_type->type & TPD_MODE_WAKEUP_GESTURE ? "[GES]" : ""
		);
	}
}

static void tpd_create_event_str(struct tpd_status_info *status_info, char *buf)
{
	char *mode_str = NULL;

	mode_str = kzalloc(TPD_FTA_MODE_LOG_SIZE, GFP_KERNEL);
	if (!mode_str) {
		snprintf(buf, TPD_FTA_MSG_LENGTH_MAX,
			"Touch: Event(count[%04X],err[%02X],reset[%02X]) - [NoMem]\n",
			status_info->event_count.count, status_info->event_count.err, status_info->event_count.reset);
		return;
	}

	/* Create Mode Strings */
	tpd_create_mode_str(&status_info->mode_type, &mode_str[0]);

	snprintf(buf, TPD_FTA_MSG_LENGTH_MAX,
		"Touch: Event(count[%04X],err[%02X],reset[%02X])%s\n",
		status_info->event_count.count, status_info->event_count.err, status_info->event_count.reset,
		mode_str);

	kfree(mode_str);
}

static void tpd_create_device_str(struct tpd_status_info *status_info, char *buf)
{
	struct tpd_device_info *device_info = &status_info->device_info;

	snprintf(buf, TPD_FTA_MSG_LENGTH_MAX,
		"Touch: Device(firm[%08X],config[%08X],check[%08X])\n",
		device_info->firm, device_info->config, device_info->check);
}

static void tpd_output_status_info(struct tpd_status_info *status_info)
{
	char *buf = NULL;

	buf = kzalloc(TPD_FTA_MSG_LENGTH_MAX, GFP_KERNEL);
	if (!buf) {
		return;
	}

	tpd_create_event_str(status_info, buf);
	tpd_send_buf(buf);

	if(status_info->event_count.info || status_info->event_count.reset) {
				/* 2015-01-19 151_INITIAL_DEVICE_INFO mod */
		memset(buf, 0, sizeof(*buf));
		tpd_create_device_str(status_info, buf);
		tpd_send_buf(buf);
	}

	kfree(buf);
}

static void tpd_output_gesture_info(u8 wakeup_id)
{
	char *buf = NULL;
	char *wake_mode = NULL;

	buf = kzalloc(TPD_FTA_MSG_LENGTH_MAX, GFP_KERNEL);
	if (!buf) {
		return;
	}
	wake_mode = kzalloc(TPD_FTA_MODE_LOG_SIZE, GFP_KERNEL);
	if(!wake_mode) {
		kfree(buf);
		return;
	}

	switch(wakeup_id) {
		case TPD_WAKEUP_DOUBLE_TAP:
			scnprintf(wake_mode, TPD_FTA_MODE_LOG_SIZE, "DOUBLE-TAP");
			break;
		case TPD_WAKEUP_DOUBLE_SLIDE:
			scnprintf(wake_mode, TPD_FTA_MODE_LOG_SIZE, "DOUBLE-SLIDE");
			break;
		case TPD_WAKEUP_LONG_TAP:
			scnprintf(wake_mode, TPD_FTA_MODE_LOG_SIZE, "LONG-TAP");
			break;
		default:
			scnprintf(wake_mode, TPD_FTA_MODE_LOG_SIZE, "UNKNOWN");
			break;
	}

	snprintf(buf, TPD_FTA_MSG_LENGTH_MAX,
			"Touch: Gesture Wakeup(mode[%s])\n", wake_mode);
	tpd_send_buf(buf);

	kfree(wake_mode);
	kfree(buf);
}

static inline void tpd_clear_status_info(struct tpd_status_info *status_info)
{
	memset(&status_info->event_count, 0, sizeof(status_info->event_count));
}


static inline void tpd_init_status_info(struct tpd_status_info *status_info)
{
	/* Counter Reset */
	tpd_clear_status_info(status_info);
	status_info->mode_type.type = TPD_MODE_NORMAL;
}


static void _tpd_trace_event(enum tpd_event_type event, int param)
{
	switch(event){
	case TPD_EVENT_ENTER_IRQ :
		trace_printk("B|0|tpd_irq\n");
		break;
	case TPD_EVENT_REPORT_START :
		trace_printk("B|0|tpd_sendevent\n");
		break;
	case TPD_EVENT_TOUCH :
		trace_printk("C|0|TOUCH_EVENT|%d\n", param);
		break;
	case TPD_EVENT_POTITION_X :
		trace_printk("C|0|TOUCH_ABS_MT_POSITION_X|%d\n", param);
		break;
	case TPD_EVENT_POTITION_Y :
		trace_printk("C|0|TOUCH_ABS_MT_POSITION_Y|%d\n", param);
		break;
	case TPD_EVENT_REPORT_END :
	case TPD_EVENT_EXIT_IRQ :
		trace_printk("E\n");
		break;
	default :
		break;
	}
}


static void tpd_trace_event(struct tpd_infomation *tpd_info, enum tpd_event_type event, int param)
{
	if(tpd_info->debug_info.debug_data[TPD_DEBUG_ID_SYSTRACE_OUTPUT]){
		_tpd_trace_event(event, param);
	}
}


static void tpd_benchmark_log(struct tpd_infomation *tpd_info, const char *tag)
{
	struct timeval tv;

	if(tpd_info->debug_info.debug_data[TPD_DEBUG_ID_BENCHMARK_LOG]){
		do_gettimeofday(&tv);
		pr_info("[TPD_IRQ] : %s: %lu%06lu\n", tag, tv.tv_sec, tv.tv_usec);
	}
}

/* 2015-12-15 161-LED-SYSFS-SUPPORT add start */
static int tpd_write_value_to_sysfs(const char *path, unsigned int value)
{
	struct file* fp;
	char* buf;
	int ret, len;

	buf = kzalloc(TPD_SYSFS_BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("[TPD] E: %s: Memory Allocation Failed.\n", __func__);
		return -ENOMEM;
	}

	len = snprintf(buf, TPD_SYSFS_BUF_SIZE, "%u\n", value);

	fp = filp_open(path, O_RDWR, 0);
	if (IS_ERR(fp)) {
		pr_err("[TPD] E: %s: Failed to open %s", __func__, path);
		kfree(buf);
		return -EIO;
	}

	if (fp->f_op->write) {
		ret = fp->f_op->write(fp, buf, len, &fp->f_pos);
	} else {
		ret = do_sync_write(fp, buf, len, &fp->f_pos);
	}
	filp_close(fp, NULL);
	kfree(buf);
	
	return 0;
}
/* 2015-12-15 161-LED-SYSFS-SUPPORT add end */

static void tpd_set_led_on_reset(struct tpd_infomation *tpd_info)
{
	if(tpd_info->debug_info.debug_data[TPD_DEBUG_ID_LED_ON_RESET]){
/* 2015-12-15 161-LED-SYSFS-SUPPORT mod start */
		tpd_write_value_to_sysfs(TPD_LED_SYSFS_B,
				TPD_LED_BRIGHTNESS_ON);
/* 2015-12-15 161-LED-SYSFS-SUPPORT mod end */
	}
}


static void tpd_set_led_on_touch_event(struct tpd_infomation *tpd_info, int act)
{
	u8 pattern;

	if(tpd_info->debug_info.debug_data[TPD_DEBUG_ID_TOUCH_EVENT_LED]){
		if(act == TPD_EVENT_REPORT_MOVE){
			pattern = TPD_LED_ACTION_MOVE;
		} else if(act == TPD_EVENT_REPORT_DOWN){
			pattern = TPD_LED_ACTION_DOWN;
		} else{
			pattern = TPD_LED_ACTION_UP;
		}
		tpd_info->status_info.led_info.event = act; /* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add */
	}
}


static void tpd_set_led_on_detect_tool(struct tpd_infomation *tpd_info, int tool_type)
{
	u8 pattern;

	if(tpd_info->debug_info.debug_data[TPD_DEBUG_ID_TOOL_TYPE_LED]){
		switch(tool_type){
		case TPD_EVENT_TOOL_FINGER:
			pattern = TPD_LED_EVENT_FINGER;
			break;
		case TPD_EVENT_TOOL_GLOVE:
			pattern = TPD_LED_EVENT_GLOVE;
			break;
		case TPD_EVENT_TOOL_HOVER:
			pattern = TPD_LED_EVENT_HOVER;
			break;
		case TPD_EVENT_TOOL_STYLUS:
			pattern = TPD_LED_EVENT_PEN;
			break;
		default:
			pattern = TPD_LED_OFF;
			break;
		}
		/* Set LED if necessary */
	}
}

/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add start */
static void tpd_set_led_on_report_end(struct tpd_infomation *tpd_info)
{
	struct tpd_led_info* led_info = &tpd_info->status_info.led_info;

	if(tpd_info->debug_info.debug_data[TPD_DEBUG_ID_TOUCH_EVENT_LED]){
		if(led_info->event == TPD_EVENT_REPORT_MOVE){
			if (led_info->led_on) {
				if (led_info->down_x != led_info->x ||
				    led_info->down_y != led_info->y) {
					led_info->led_on = TPD_DEBUG_OFF;
					tpd_write_value_to_sysfs(TPD_LED_SYSFS_G,
							TPD_LED_BRIGHTNESS_OFF);
				}
			}
		} else if(led_info->event == TPD_EVENT_REPORT_DOWN){
			led_info->down_x = led_info->x;
			led_info->down_y = led_info->y;
			led_info->led_on = TPD_DEBUG_ON;
			tpd_write_value_to_sysfs(TPD_LED_SYSFS_G,
					TPD_LED_BRIGHTNESS_ON);
		} else{
			led_info->down_x = 0;
			led_info->down_y = 0;
			if (led_info->led_on) {
				led_info->led_on = TPD_DEBUG_OFF;
				tpd_write_value_to_sysfs(TPD_LED_SYSFS_G,
						TPD_LED_BRIGHTNESS_OFF);
			}
		}
	}
}
/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add end */

static int tpd_read_nvinfo(struct tpd_nonvolatile_info *nv_info)
{
	int retval;

	/* Read NV-Block1 */
	retval = get_nonvolatile(&nv_info->raw_data1[0], TPD_NV_BLOCK1_ID, TPD_NV_BLOCK1_SIZE);
	if (retval < 0){
		pr_err("[TPD_NV] : %s: set default value Block1(0x%X)\n", __func__, TPD_NV_BLOCK1_ID);
		memset(&nv_info->raw_data1[0], 0, TPD_NV_BLOCK1_SIZE);
	}

	/* Read NV-Block2 */
	retval = get_nonvolatile(&nv_info->raw_data2[0], TPD_NV_BLOCK2_ID, TPD_NV_BLOCK2_SIZE);
	if (retval < 0){
		pr_err("[TPD_NV] : %s: set default value Block2(0x%X)\n", __func__, TPD_NV_BLOCK2_ID);
		memset(&nv_info->raw_data2[0], 0, TPD_NV_BLOCK2_SIZE);
    }

	/* Read NV-Block3 */
	retval = get_nonvolatile(&nv_info->raw_data3[0], TPD_NV_BLOCK3_ID, TPD_NV_BLOCK3_SIZE);
	if (retval < 0){
		pr_err("[TPD_NV] : %s: set default value Block3(0x%X)\n", __func__, TPD_NV_BLOCK3_ID);
		memset(&nv_info->raw_data3[0], 0, TPD_NV_BLOCK3_SIZE);
	}

	return 0;
}


static int tpd_set_all_debug_state(struct tpd_debug_info *debug_info)
{
	struct tpd_nonvolatile_info *nv_info = &debug_info->nv_info;
	int i, id;
	u8 record, data;

	/* Debug Item1 */
	for(i = 0; i < TPD_DEBUG_ID_MAX; i++){
		id = nv_block1[i].id;
		if(id == TPD_NV_ID_UNSUPPORT)
			continue;
		record = nv_block1[i].record;
		data = nv_info->raw_data1[record] & nv_block1[i].mask;
		debug_info->debug_data[id] = (data >> nv_block1[i].shift);
		if(data){
			pr_info("[TPD_NV] : %s= %d\n", nv_block1[i].name, debug_info->debug_data[id]);
		}
	}

	return 0;
}


static int tpd_set_debug_info(struct tpd_debug_info *debug_info)
{
	tpd_read_nvinfo(&debug_info->nv_info);
	tpd_set_all_debug_state(debug_info);

	return 0;
}

static void tpd_sysfs_change_mode(struct kobject *kobj, const struct attribute *attr)
{
	int ret;
	umode_t new_mode = 0;
	umode_t mode = attr->mode;
	
	if(mode & S_IRUSR)
		new_mode |= S_IRUGO;
	
	if(mode & S_IWUSR)
		new_mode |= S_IWUGO;
	
	if(mode & S_IXUSR)
		new_mode |= S_IXUGO;

	if(mode == new_mode || new_mode == 0)
		return;

	ret = sysfs_chmod_file(kobj, attr, new_mode);
	if(ret) {
		pr_err("[TPD] E: %s: Sysfs Chmod Failed(%d).\n", __func__, ret);
	}
}

/***************/
/* Interfaces  */
/***************/

/* 2015-01-19 151_INITIAL_DEVICE_INFO add start */
void tpd_event_record_info(void *handle)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;
	
	tpd_info->status_info.event_count.info++;
}
/* 2015-01-19 151_INITIAL_DEVICE_INFO add end */

void tpd_event_suspend(void *handle)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);

	pr_info("[TPD] : Enter Sleep.\n");

	if(!tpd_info)
		return;
	
	/* Output Event Log */
	tpd_output_status_info(&tpd_info->status_info);
	/* Counter Reset */
	tpd_clear_status_info(&tpd_info->status_info);
}

void tpd_event_resume(void *handle)
{
	pr_info("[TPD] : Exit Sleep.\n");
	return;
}

void tpd_event_enter_irq(void *handle)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;
	
	tpd_trace_event(tpd_info, TPD_EVENT_ENTER_IRQ, 0);
	tpd_benchmark_log(tpd_info, "[IN] ");
}

void tpd_event_exit_irq(void *handle)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_trace_event(tpd_info, TPD_EVENT_EXIT_IRQ, 0);
	tpd_benchmark_log(tpd_info, "[OUT]");
}

void tpd_event_report_start(void *handle)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_trace_event(tpd_info, TPD_EVENT_REPORT_START, 0);
}

void tpd_event_report_end(void *handle)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_trace_event(tpd_info, TPD_EVENT_REPORT_END, 0);
	tpd_set_led_on_report_end(tpd_info); /* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add */
}

void tpd_event_change_mode(void *handle, u32 mode, u8 enable)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info){
		return;
	}

	if(enable) {
		tpd_info->status_info.mode_type.type |= mode;
	} else {
		tpd_info->status_info.mode_type.type &= ~mode;
	}
}

void tpd_event_reset(void *handle)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_info->status_info.event_count.reset++;
	tpd_set_led_on_reset(tpd_info);
}

void tpd_event_bus_error(void *handle)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_info->status_info.event_count.err++;
}

void tpd_event_wakeup_gesture(void *handle, u8 wakeup_id) /* FCNT LIMITED: 2016-03-23 161_WG_NAME mod */
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info || wakeup_id >= TPD_WAKEUP_MAX)
		return;

	tpd_output_gesture_info(wakeup_id);
}

void tpd_detect_touch_event(void *handle, int event)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_trace_event(tpd_info, TPD_EVENT_TOUCH, event);
	if(event == TPD_EVENT_REPORT_DOWN){
		tpd_info->status_info.event_count.count++;
	}
	tpd_set_led_on_touch_event(tpd_info, event);
}

void tpd_detect_position_x(void *handle, int x)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_trace_event(tpd_info, TPD_EVENT_POTITION_X, x);
/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add start */
	tpd_info->status_info.led_info.x = x;
/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add end */
}

void tpd_detect_position_y(void *handle, int y)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_trace_event(tpd_info, TPD_EVENT_POTITION_Y, y);
/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add start */
	tpd_info->status_info.led_info.y = y;
/* 2015-12-15 161-OSVM-SAKUSAKU-HW-COST add end */
}

void tpd_detect_tool_type(void *handle, int tool_type)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return;

	tpd_set_led_on_detect_tool(tpd_info, tool_type);
}

void tpd_info_firm_id(void *handle, u32 firm_id)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info){
		return;
	}

	tpd_info->status_info.device_info.firm = firm_id;
}

void tpd_info_config_id(void *handle, u32 config_id)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info){
		return;
	}

	tpd_info->status_info.device_info.config = config_id;
}

void tpd_info_device_check(void *handle, u32 check_code)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info){
		return;
	}

	tpd_info->status_info.device_info.check = check_code;
}

u8 tpd_get_debug_state(void *handle, u8 id)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info || TPD_DEBUG_ID_MAX <= id)
		return 0;

	return tpd_info->debug_info.debug_data[id];
}

void tpd_set_debug_state(void *handle, u8 id, u8 state)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info || TPD_DEBUG_ID_MAX <= id)
		return;

	tpd_info->debug_info.debug_data[id] = state;
	pr_info("[TPD] : Change debug state ID[%d] = %x\n", id, state);
}

/* FCNT LIMITED: 2016-02-19 161_GET_NV_DATA add start */
u8 tpd_get_config_param(void *handle, u8 **data, int *len)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info)
		return -EINVAL;

	*data = tpd_info->debug_info.nv_info.raw_data3;
	*len = TPD_NV_BLOCK3_SIZE;
	return 0;
}
/* FCNT LIMITED: 2016-02-19 161_GET_NV_DATA add end */

void tpd_log_dump_message(void *handle, u8 *buf, int size, const char *data_name)
{
	tpd_log_dump_buffer(handle, buf, size, data_name, TPD_DEBUG_ID_MESSAGE_LOG);
}

void tpd_log_dump_buffer(void *handle, u8 *buf, int size, const char *data_name, u8 id)
{
	struct tpd_infomation *touch_info = _tpd_get_infomation(handle);
	if(!touch_info || !buf || size <= 0 || TPD_DEBUG_ID_MAX <= id)
		return;

	if(tpd_get_debug_state(handle, id))
		tpd_dump(buf, size, data_name);
}

/* 2015-02-24 151_DUMP_BUFFER add start */
void tpd_log_dump_buffer_noid(void *handle, u8 *buf, int size, const char *data_name)
{
	struct tpd_infomation *touch_info = _tpd_get_infomation(handle);
	if(!touch_info || !buf || size <= 0)
		return;

	tpd_dump(buf, size, data_name);
}
/* 2015-02-24 151_DUMP_BUFFER add end */

void tpd_sysfs_chmod(void* handle, struct kobject *kobj, const struct attribute *attr)
{
	struct tpd_infomation *tpd_info = _tpd_get_infomation(handle);
	if(!tpd_info || !kobj || !attr){
		return;
	}
	
	if(tpd_get_debug_state(handle, TPD_DEBUG_ID_TOOL_ENABLE))
		tpd_sysfs_change_mode(kobj, attr);
}

int tpd_init(void **handle)
{
	struct tpd_infomation *tpd_info;

	tpd_info = kzalloc(sizeof(struct tpd_infomation), GFP_KERNEL);
	if (!tpd_info) {
		pr_err("[TPD] E: %s: Memory Allocation Failed.\n", __func__);
		return -ENOMEM;
	}
	
	tpd_set_debug_info(&tpd_info->debug_info);
	tpd_init_status_info(&tpd_info->status_info);
	*handle = (void *)tpd_info;
	
	return 0;
}

int tpd_exit(void *handle)
{
	if (handle) {
		kfree(handle);
	}
	return 0;
}
