/*----------------------------------------------------------------------------*/
/* COPYRIGHT(C) FUJITSU LIMITED 2014-2015                                     */
/* COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016                   */
/*----------------------------------------------------------------------------*/
/* TPD(Touch Panel Debug) modules */

#ifndef __TOUCH_PANEL_DEBUG_H__
#define __TOUCH_PANEL_DEBUG_H__

#define TPD_MODE_UNKNOWN      0xFFFFFFFF
#define TPD_MODE_DEFAULT      0x00000000
#define TPD_MODE_FEP          0x00000001
#define TPD_MODE_PRESS        0x00000002
#define TPD_MODE_GLOVE        0x00000004
#define TPD_MODE_HOVER        0x00000008
#define TPD_MODE_CHARGE       0x00000010
#define TPD_MODE_HIGH_CHARGE  0x00000020
#define TPD_MODE_WAKEUP_GESTURE 0x00000040 /* FCNT LIMITED: 2016-03-23 161_WG_NAME mod */
#define TPD_MODE_NORMAL TPD_MODE_DEFAULT

#define TPD_EVENT_REPORT_DOWN     1
#define TPD_EVENT_REPORT_MOVE     2
#define TPD_EVENT_REPORT_UP       3

#define TPD_EVENT_TOOL_FINGER     0
#define TPD_EVENT_TOOL_PROXIMITY  1
#define TPD_EVENT_TOOL_STYLUS     2
#define TPD_EVENT_TOOL_HOVER      3
#define TPD_EVENT_TOOL_GLOVE      4
#define TPD_EVENT_TOOL_LIFTOFF 0xffffffff

#define TPD_DEBUG_ON     (u8)1
#define TPD_DEBUG_OFF    (u8)0

enum tpd_debug_id {
	TPD_DEBUG_ID_TOUCH_DISABLE = 0x00,
	TPD_DEBUG_ID_SUSPEND_MODE,
	TPD_DEBUG_ID_STARTUP_CALIB,
	TPD_DEBUG_ID_WAKEUP_GESTURE, /* FCNT LIMITED: 2016-03-23 161_WG_NAME mod */
	TPD_DEBUG_ID_CONFIG_PARAM,
	TPD_DEBUG_ID_CHARGER_SETTING,
	TPD_DEBUG_ID_MODE_SETTING,
	TPD_DEBUG_ID_LOG_DEVICE,
	TPD_DEBUG_ID_LOG_DEBUG,
	TPD_DEBUG_ID_SYSTRACE_OUTPUT,
	TPD_DEBUG_ID_BENCHMARK_LOG,
	TPD_DEBUG_ID_MESSAGE_LOG,
	TPD_DEBUG_ID_TOOL_ENABLE,
	TPD_DEBUG_ID_LED_ON_RESET,
	TPD_DEBUG_ID_TOUCH_EVENT_LED,
	TPD_DEBUG_ID_TOOL_TYPE_LED,
	TPD_DEBUG_ID_GLOVE_CORRECTION,
	TPD_DEBUG_ID_MAX
};

enum tpd_wakeup_id {
	TPD_WAKEUP_DOUBLE_TAP,
	TPD_WAKEUP_DOUBLE_SLIDE,
	TPD_WAKEUP_LONG_TAP,
	TPD_WAKEUP_MAX
};

int tpd_init(void **handle);
int tpd_exit(void *handle);

void tpd_event_record_info(void *handle); /* 2015-01-19 151_INITIAL_DEVICE_INFO add start */
void tpd_event_suspend(void *handle);
void tpd_event_resume(void *handle);
void tpd_event_enter_irq(void *handle);
void tpd_event_exit_irq(void *handle);
void tpd_event_reset(void *handle);
void tpd_event_bus_error(void *handle);
void tpd_event_report_start(void *handle);
void tpd_event_report_end(void *handle);
void tpd_event_wakeup_gesture(void *handle, u8 wakeup_id); /* FCNT LIMITED: 2016-03-23 161_WG_NAME mod */
void tpd_event_change_mode(void *handle, u32 mode, u8 enable);

void tpd_detect_touch_event(void *handle, int event);
void tpd_detect_position_x(void *handle, int x);
void tpd_detect_position_y(void *handle, int y);
void tpd_detect_tool_type(void *handle, int tool_type);

void tpd_info_firm_id(void *handle, u32 firm_id);
void tpd_info_config_id(void *handle, u32 config_id);
void tpd_info_device_check(void *handle, u32 check_code);

void tpd_set_debug_state(void *handle, u8 id, u8 state);
u8 tpd_get_debug_state(void *handle, u8 id);

u8 tpd_get_config_param(void *handle, u8 **data, int *len); /* FCNT LIMITED: 2016-02-19 161_GET_NV_DATA add */

void tpd_log_dump_message(void *handle, u8 *buf, int size, const char *data_name);
void tpd_log_dump_buffer(void *handle, u8 *buf, int size, const char *data_name, u8 id);
void tpd_log_dump_buffer_noid(void *handle, u8 *buf, int size, const char *data_name); /* 2015-02-24 151_DUMP_BUFFER add */

void tpd_sysfs_chmod(void* handle, struct kobject *kobj, const struct attribute *attr);


/**
 * Kernel Logs
 */
#ifdef __KERNEL_PRINTK__

#define TPD_LOG_ERR(ph, fmt, arg...) { \
		if(ph) pr_err("[TPD]E:"fmt, ## arg); \
	}

#define TPD_LOG_WARN(ph, fmt, arg...) { \
		if(ph) pr_warn("[TPD]W:"fmt, ## arg); \
	}

#define TPD_LOG_INFO(ph, fmt, arg...) { \
		if(ph) pr_info("[TPD] :"fmt, ## arg); \
	}

#define TPD_LOG_DEVICE(ph, fmt, arg...) { \
		if(tpd_get_debug_state(ph, TPD_DEBUG_ID_LOG_DEVICE)) pr_info("[TPD] :"fmt, ## arg); \
	}

#define TPD_LOG_DEBUG(ph, fmt, arg...) { \
		if(tpd_get_debug_state(ph, TPD_DEBUG_ID_LOG_DEBUG)) pr_info("[TPD] :"fmt, ## arg); \
	}

#else
#define TPD_LOG_ERR(ph, fmt, arg...) {}
#define TPD_LOG_WARN(ph, fmt, arg...) {}
#define TPD_LOG_INFO(ph, fmt, arg...) {}
#define TPD_LOG_DEVICE(ph, fmt, arg...) {}
#define TPD_LOG_DEBUG(ph, fmt, arg...) {}
#endif /* __KERNEL_PRINTK__ */

#endif /* __TOUCH_PANEL_DEBUG_H__ */


