/*
 * COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2017
 * COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/fj_mode.h>
#include <linux/delay.h>
#include <linux/mfd/fj_charger.h>
#include "fj_charger_local.h"
#include <linux/qpnp/qpnp-adc.h>
#include <linux/nonvolatile_common.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/qpnp/pin.h>
#include <linux/regulator/consumer.h> 

extern int qpnp_charger_read(u16 cmd, u8 *val);
extern int qpnp_charger_write(u16 cmd, u8 mask, u8 val);

#define FJ_CHARGER_DBGLOG(x, y...)     				\
	if (fj_chg_charger_debug){ 						\
		printk(KERN_ERR "[fj_chg] " x, ## y);		\
	}
#define FJ_CHARGER_INFOLOG(x, y...)		printk(KERN_INFO "[fj_chg] " x, ## y)
#define FJ_CHARGER_WARNLOG(x, y...)		printk(KERN_WARNING "[fj_chg] " x, ## y)
#define FJ_CHARGER_ERRLOG(x, y...)		printk(KERN_ERR "[fj_chg] " x, ## y)
#define FJ_CHARGER_RECLOG(x, y...)		printk("REC@CHG@36[fj_chg] " x, ## y)

#define FJ_CHG_SOURCE_AC					(BIT(0) << CHG_SOURCE_AC)
#define FJ_CHG_SOURCE_USB					(BIT(0) << CHG_SOURCE_USB)
#define FJ_CHG_SOURCE_MHL					(BIT(0) << CHG_SOURCE_MHL)
#define FJ_CHG_SOURCE_HOLDER				(BIT(0) << CHG_SOURCE_HOLDER)

#define FJ_CHG_SOURCE_USB_PORT				(FJ_CHG_SOURCE_AC |FJ_CHG_SOURCE_USB |FJ_CHG_SOURCE_MHL )
#define FJ_CHG_SOURCE(x)					(BIT(0) << x)
#define FJ_CHG_SOURCE_IS_USB_PORT(x)		((FJ_CHG_SOURCE_USB_PORT) & FJ_CHG_SOURCE(x))
#define FJ_CHG_SOURCE_MASK					(FJ_CHG_SOURCE_USB_PORT | FJ_CHG_SOURCE_HOLDER)

#define FJ_CHG_IS_USB_PORT_CHARGE(chip)		((chip->connect_state == FJ_CHG_CONNECT_STATE_USB) ? true : false)
#define FJ_CHG_IS_HOLDER_CHARGE(chip)		(((chip->connect_state == FJ_CHG_CONNECT_STATE_HOLDER) ||\
											  (chip->connect_state == FJ_CHG_CONNECT_STATE_BOTH)) ? true : false)

#define CHG_TYPE_LOG(chip)					(FJ_CHG_IS_HOLDER_CHARGE(chip) ? "HOLDER" : "USB")

#define CHG_GET_SMALL(a ,b)					(a > b ? b : a)
#define CHG_GET_LARGE(a ,b)					(a > b ? a : b)

#define APNV_CHG_ADJ_VBAT_I					(41040)
#define APNV_CHG_ADJ_VMAX_I					(41041)
#define APNV_MC_CHARGE_FLAG					(49144)

#define CHARGE_MODE_INVALID					(-1)
#define CHARGE_MODE_NOT_CHARGE				(0)
#define CHARGE_MODE_MANUAL					(1)
#define CHARGE_MODE_NUM						(2)

#define FJ_CHG_VALUE_ADP_VOLTAGE_5V_UV		( 4000 * 1000)	/*  4.000V */
#define FJ_CHG_VALUE_ADP_VOLTAGE_5V_OV		( 6400 * 1000)	/*  6.400V */
#define FJ_CHG_VALUE_ADP_VOLTAGE_9V_UV		( 8500 * 1000)	/*  8.500V */
#define FJ_CHG_VALUE_ADP_VOLTAGE_9V_OV		(11000 * 1000)	/* 11.000V */
#define FJ_CHG_VALUE_ADP_VOLTAGE_9V_UV_L	( 8180 * 1000)	/*  8.180V */

#define FJ_CHG_VALUE_HOLDER_VOLTAGE_5V_UV	( 4000 * 1000)	/*  4.000V */
#define FJ_CHG_VALUE_HOLDER_VOLTAGE_5V_OV	( 6400 * 1000)	/*  6.400V */
#define FJ_CHG_VALUE_HOLDER_VOLTAGE_9V_UV	( 8500 * 1000)	/*  8.500V */
#define FJ_CHG_VALUE_HOLDER_VOLTAGE_9V_OV	(11000 * 1000)	/* 11.000V */
#define FJ_CHG_VALUE_HOLDER_VOLTAGE_9V_UV_L	( 8180 * 1000)	/*  8.180V */

#define FJ_CHG_VALUE_BATT_VOLTAGE_OV		( 4450 * 1000)	/* 4.45V  */
#define FJ_CHG_VALUE_UNSPEC_BATT_VOLTAGE	( 3400 * 1000)	/* 3.40V  */
#define FJ_CHG_VALUE_ONCHG_BATT_LOW_VOLT	( 3400 * 1000)	/* 3.40V  */
#define FJ_CHG_VALUE_OFFCHG_BATT_LOW_VOLT	( 3400 * 1000)	/* 3.40V  */

#define FJ_CHG_VALUE_FULL_SOC_LV1			(99)			/* 99% */
#define FJ_CHG_VALUE_FULL_SOC_LV2			(94)			/* 94% */
#define FJ_CHG_VALUE_RECHARGE_LV1			(98)			/* 98% */
#define FJ_CHG_VALUE_RECHARGE_LV2			(97)			/* 97% */

#define FJ_CHG_SET_TIMER_250MS				(250)
#define FJ_CHG_SET_TIMER_500MS				(500)
#define FJ_CHG_SET_TIMER_750MS				(750)
#define FJ_CHG_SET_TIMER_1S					(  1 * 1000)
#define FJ_CHG_SET_TIMER_5S					(  5 * 1000)
#define FJ_CHG_SET_TIMER_10S				( 10 * 1000)
#define FJ_CHG_SET_TIMER_3MIN				(  3 * 1000 * 60)
#define FJ_CHG_SET_TIMER_10MIN				( 10 * 1000 * 60)
#define FJ_CHG_SET_TIMER_30MIN				( 30 * 1000 * 60)
#define FJ_CHG_SET_TIMER_840MIN				(840 * 1000 * 60)
#define FJ_CHG_SET_TIMER_2H					(  2 * 1000 * 60 * 60)
#define FJ_CHG_SET_TIMER_3H					(  3 * 1000 * 60 * 60)

#define FJ_CHG_VALUE_BATT_TEMP_HOT_LIMIT	(500)
#define FJ_CHG_VALUE_BATT_TEMP_WARM_LIMIT	(450)
#define FJ_CHG_VALUE_BATT_TEMP_COOL_LIMIT	(100)
#define FJ_CHG_VALUE_BATT_TEMP_COLD_LIMIT	(0)
#define FJ_CHG_VALUE_BATT_TEMP_START_LIMIT	(450)
#define FJ_CHG_VALUE_BATT_TEMP_LIMIT_CANCEL (10)

#define FJ_CHG_NEGO_USB_DETECT_CNT			(10)
#define FJ_CHG_NEGO_USB_DETECT_MAX			(30)
#define FJ_CHG_NEGO_HOLDER_DETECT_CNT		(5)
#define FJ_CHG_NEGO_HOLDER_DETECT_MAX		(30)
#define FJ_CHG_ADP_VOLTAGE_DETECT_CNT		(5)
#define FJ_CHG_ADP_VOLTAGE_DETECT_MAX		(30)
#define	FJ_CHG_WAKELOCK_TIMER				(3)

#define FJ_CHG_OKI_DET_GPIO					(qpnp_pin_map("pmi8994-gpio", 7))

#define FJ_CHG_ILIMIT_NOT_LIMITED			(0)						/* ilimit not limited     */
#define FJ_CHG_ILIMIT_LIMITED_2000			FJ_CHG_CURRENT_2000		/* ilimit limited 2000mA  */
#define FJ_CHG_ILIMIT_LIMITED_1500			FJ_CHG_CURRENT_1500		/* ilimit limited 1500mA  */
#define FJ_CHG_ILIMIT_LIMITED_900			FJ_CHG_CURRENT_900		/* ilimit limited 900mA   */
#define FJ_CHG_ILIMIT_LIMITED_500			FJ_CHG_CURRENT_500		/* ilimit limited 500mA   */
#define FJ_CHG_ILIMIT_LIMITED_300			FJ_CHG_CURRENT_300		/* ilimit limited 300mA   */
#define FJ_CHG_ILIMIT_LIMITED_150			FJ_CHG_CURRENT_150		/* ilimit limited 150mA   */
#define FJ_CHG_ILIMIT_LIMITED_100			FJ_CHG_CURRENT_100		/* ilimit limited 100mA   */
#define FJ_CHG_ILIMIT_LIMITED_MIN			FJ_CHG_CURRENT_300		/* ilimit limited minimum */
#define FJ_CHG_ILIMIT_DISABLE				(-1)					/* charge disable         */

#define FJ_CHG_QUEUE_LOCK()		do { \
									unsigned long flags; \
									spin_lock_irqsave(&fj_chg_queue_lock, flags);

#define FJ_CHG_QUEUE_UNLOCK()		spin_unlock_irqrestore(&fj_chg_queue_lock, flags); \
								} while (0);

#define FJ_CHG_EVT_QUEUE_MAX				(30)
#define FJ_CHG_BATT_VOL_INDEX				(5)

/* charge err status bit */
#define FJ_CHG_PARAM_NO_ERROR				0x00000000
#define FJ_CHG_PARAM_BATT_TEMP_HOT			0x00000001
#define FJ_CHG_PARAM_BATT_TEMP_COLD			0x00000002
#define FJ_CHG_PARAM_BATT_VOLTAGE_OV		0x00000004
#define FJ_CHG_PARAM_ADP_VOLTAGE_OV			0x00000008
#define FJ_CHG_PARAM_ADP_VOLTAGE_UV			0x00000010
#define FJ_CHG_PARAM_OVP_DETECT_ERROR		0x00000020
#define FJ_CHG_PARAM_BATT_TEMP_WARM			0x00000040
#define FJ_CHG_PARAM_BATT_TEMP_COOL			0x00000080
#define FJ_CHG_PARAM_CHG_DISABLE			0x00000100
#define FJ_CHG_PARAM_UNSPEC_FAILURE			0x00000200
#define FJ_CHG_PARAM_USB_PORT_FAILURE		0x00000400
#define FJ_CHG_PARAM_USB_HOT_ERROR			0x00000800
#define FJ_CHG_PARAM_HOLDER_FAILURE			0x00001000
#define FJ_CHG_PARAM_WEAK_ADAPTER			0x00002000
#define FJ_CHG_PARAM_QC_UV_ERROR			0x00004000
#define FJ_CHG_PARAM_RECOVERY_ERROR			0x00008000
#define FJ_CHG_PARAM_CHG_TYPE_UNKNOWN		0x00010000
#define FJ_CHG_PARAM_FGIC_INITIALIZING		0x00020000

typedef enum {
	FJ_CHG_STATE_IDLE = 0,					/* idle						*/
	FJ_CHG_STATE_CHARGING,					/* charging/re-charging		*/
	FJ_CHG_STATE_FULL,						/* full charging			*/
	FJ_CHG_STATE_ERROR,						/* charge error occurred	*/
	FJ_CHG_STATE_NUM,

	FJ_CHG_STATE_NOCHANGE,
} fj_charger_state_type;

typedef enum {
	FJ_CHG_EVENT_MONITOR_PARAM = 0,			/* period monitor		*/
	FJ_CHG_EVENT_CHARGE_INFO_NOTICE,		/* charge info notice	*/
	FJ_CHG_EVENT_CHARGE_CTRL,				/* charge control		*/
	FJ_CHG_EVENT_CHARGE_TIMER,				/* timer expire			*/
	FJ_CHG_EVENT_PARAM_UPDATE,				/* parameter update		*/
	FJ_CHG_EVENT_NUM
} fj_charger_event_type;

typedef enum {
	FJ_CHG_CONNECT_STATE_NONE = 0,
	FJ_CHG_CONNECT_STATE_USB,
	FJ_CHG_CONNECT_STATE_HOLDER,
	FJ_CHG_CONNECT_STATE_BOTH,
	FJ_CHG_CONNECT_STATE_NUM,

	FJ_CHG_CONNECT_STATE_NOCHANGE,
} fj_charger_connect_state_type;

typedef enum {
	FJ_CHG_CONNECT_EVENT_USB_IN = 0,
	FJ_CHG_CONNECT_EVENT_USB_OUT,
	FJ_CHG_CONNECT_EVENT_HOLDER_IN,
	FJ_CHG_CONNECT_EVENT_HOLDER_OUT,
	FJ_CHG_CONNECT_EVENT_NUM,
} fj_charger_connect_event_type;

typedef enum {
	FJ_CHG_MONITOR_NORMAL = 0,
	FJ_CHG_MONITOR_VOLTAGE,
	FJ_CHG_MONITOR_NUM,
} fj_charger_monitor_type;

typedef enum {
	FJ_CHG_NOTICE_NON = 0,					/* 	*/
	FJ_CHG_NOTICE_OVP_NOTIFY_ERROR,			/* ovp notify error			*/
	FJ_CHG_NOTICE_USBHOT_NOTIFY_ERROR,		/* usb hot notify error		*/
	FJ_CHG_NOTICE_CHG_ENABLE,				/* charge enable			*/
	FJ_CHG_NOTICE_CHANGE_ADP_VOLT,			/* change adapter voltage	*/
	FJ_CHG_NOTICE_CHANGE_MODE,				/* change charge mode		*/
	FJ_CHG_NOTICE_OKI_DET,					/* oki_det interrupt		*/
	FJ_CHG_NOTICE_CHANGE_CURRENT,			/* change of the current	*/
	FJ_CHG_NOTICE_NUM
} fj_charger_notice_type;

typedef enum {
	FJ_CHG_TIMER_NON = 0,					/* events other than a timer 	*/
	FJ_CHG_TIMER_FULL_1,					/* full charge detection timer1	*/
	FJ_CHG_TIMER_FULL_2,					/* full charge detection timer2	*/
	FJ_CHG_TIMER_PROTECTION,				/* protection timer				*/
	FJ_CHG_TIMER_VOLTAGE_CHECK,				/* voltage check timer			*/
	FJ_CHG_TIMER_NEGOTIATION,				/* negotiation timer			*/
	FJ_CHG_TIMER_NUM
} fj_charger_timer_type;

typedef enum {
	FJ_CHG_CONTROL_NONE = 0,
	FJ_CHG_CONTROL_USB_CHARGE,
	FJ_CHG_CONTROL_HOLDER_CHARGE,
	FJ_CHG_CONTROL_PRECHARGE,
	FJ_CHG_CONTROL_CHARGE_START,
	FJ_CHG_CONTROL_POWERPATH,
	FJ_CHG_CONTROL_RECHARGE,
	FJ_CHG_CONTROL_STOP,
	FJ_CHG_CONTROL_NUM,
} fj_charger_ctrl_type;

typedef enum {
	FJ_CHG_DEVICE_TYPE_USB = 0,
	FJ_CHG_DEVICE_TYPE_HOLDER,
	FJ_CHG_DEVICE_TYPE_NUM,
} fj_charger_device_type;

typedef enum {
	FJ_CHG_CHARGE_TYPE_UNKNOWN = 0,
	FJ_CHG_CHARGE_TYPE_NORMAL,
	FJ_CHG_CHARGE_TYPE_QUICK,
	FJ_CHG_CHARGE_TYPE_NUM,
} fj_charger_charge_type;

typedef enum {
	FJ_CHG_OKI_DET_TYPE_IRQ = 0,
	FJ_CHG_OKI_DET_TYPE_TIMER,
	FJ_CHG_OKI_DET_TYPE_NUM,
} fj_charger_oki_det_type;

typedef enum {
	FJ_CHG_RESULT_SUCCESS = 0,
	FJ_CHG_RESULT_FAILURE,
	FJ_CHG_RESULT_RETRY,
	FJ_CHG_RESULT_OVER,
	FJ_CHG_RESULT_VOLTAGE_ERROR,
	FJ_CHG_RESULT_CONTINUE,
	FJ_CHG_RESULT_RECONFIRMATION,
} fj_charger_result_type;

typedef enum {
	FJ_CHG_AICL_TYPE_DISABLE = 0,
	FJ_CHG_AICL_TYPE_ENABLE,
	FJ_CHG_AICL_TYPE_RERUN,
} fj_charger_aicl_type;

typedef enum {
	FJ_CHG_SUB_STATE_TYPE_1 = 0,
	FJ_CHG_SUB_STATE_TYPE_2,
} fj_charger_sub_state_type;

typedef struct {
	int supply_volt;
	int batt_volt;
	int batt_temp;
	int case_temp;
	int soc;
	int dc_in_volt;
	int oki_adc_volt;
	int fgic_initial_sts;

	int temp_usb_error;
	int cradle_detect;
	int chg_ic_err;
	u8  chg_sts;
} fj_chg_monitor_param;

typedef struct {
	fj_charger_connect_state_type next_state;
	fj_charger_ctrl_type charge_ctrl;
} fj_chg_connect_ctrl;

typedef struct {
	fj_charger_monitor_type type;
} fj_chg_monitor_info;

typedef struct {
	fj_charger_connect_event_type event;
	int chg_source;
	unsigned int max_current;
} fj_chg_connect_info;

typedef struct {
	fj_charger_device_type device;
	int max_current;
} fj_chg_change_info;

typedef struct {
	fj_charger_notice_type type;
	unsigned int ovp_detect;				/* type:OVP_DETECT parameter		*/
	unsigned int usb_hot_detect;			/* type:USB_HOT_DETECT parameter	*/
	int ilimit;								/* type:CHG_ENABLE parameter		*/
	int charge_mode;						/* type:CHANGE_MODE parameter		*/
	int oki_det;							/* type:OKI_DET parameter			*/
	fj_chg_change_info change_info;			/* type:CHANGE_CURRENT parameter	*/
} fj_chg_notice_info;

typedef struct {
	fj_charger_timer_type type;
	fj_charger_device_type device;
} fj_chg_timer_info;

typedef union {
	fj_chg_monitor_info monitor_info;
	fj_chg_connect_info connect_info;
	fj_chg_notice_info notice_info;
	fj_chg_timer_info timer_info;
} fj_chg_evt_data;

typedef struct {
	struct list_head head;					/* queue head	*/
	fj_charger_event_type event;			/* event id		*/
	fj_chg_evt_data data;					/* event data	*/
} fj_chg_evt_queue;

typedef struct {
	int set_status;
	int set_health;
	fj_charger_ctrl_type charge_ctrl;
} fj_chg_err_ctrl;

typedef struct {
	int over_voltage;
	int under_voltage;
} fj_chg_voltage_threshold;

typedef struct {
	unsigned int check_current;
	u8 reg_data;
} fj_chg_convert_data;

typedef struct {
	struct delayed_work nego_timer_work;
	struct delayed_work check_timer_work;
	int charge_type;
	unsigned int max_current;
	int check_count[FJ_CHG_CHARGE_TYPE_NUM];
	int check_total;
	int nego_count[FJ_CHG_CHARGE_TYPE_NUM];
	int nego_total;
	int watch_count;
	int save_voltage_error;
	int qc_ilimit_value;
	int weak_adp_ilimit;
	fj_charger_sub_state_type weak_adp_state;
	bool voltage_check;
} fj_chg_device_info;

typedef struct {
	bool safety_timer_expire;
	bool batt_temp_restrict;
	bool oki_det_check;
	bool weak_adapter;
	bool qc_under_voltage;
	bool recovery_err;
	bool chg_type_unknown;
} fj_chg_err_info;

typedef struct {
	bool ovp_detect;
	bool usb_port_failure;
} fj_chg_safety_err_info;

struct fj_charger_info {
	unsigned int	state;
	unsigned int	chg_source;
	unsigned int	chg_current;
	unsigned int	current_err;
};

typedef struct {
	int vol_data[FJ_CHG_BATT_VOL_INDEX];
	int index;
	bool data_full;
} fj_chg_batt_vol_info;

struct charge_drv_chip {
	struct task_struct			*chg_thread;
	wait_queue_head_t			chg_wq;			/* charger thread waitqueue head	*/

	unsigned int				connect_state;
	int							max_ilimit_value;
	int							charge_type;
	int							heat_charge_mode;
	int							oki_det_count;
	int							consecutive_count;
	int							batt_status;
	int							batt_health;
	bool						battery_low_flag;

	int							batt_temp_hot_limit;
	int							batt_temp_warm_limit;
	int							batt_temp_cold_limit;
	int							batt_temp_cool_limit;

	int							recovery_count;
	bool						recovery_invalid;

	fj_chg_device_info			dev_usb;
	fj_chg_device_info			dev_holder;

	struct delayed_work			full_timer_work1;
	struct delayed_work			full_timer_work2;
	struct delayed_work			safety_timer_work;
	struct delayed_work			adp_volt_monitor_work;
	struct delayed_work			oki_det_work;
	struct delayed_work			watchdog_work;

	struct i2c_client			*client;
	struct power_supply			mains;
	struct power_supply			usb;
	struct power_supply			mhl;
	struct power_supply			holder;

	struct power_supply			*bms;
	struct power_supply			*battery;
	struct power_supply			*fj_pmi_adc;
	struct power_supply			*fj_pm_adc;
    struct power_supply			*fg_adc;

	int							charge_mode;
	struct delayed_work			charge_monitor_work;
	int							vmax_adj;
	fj_chg_err_info				err_info;
	fj_chg_safety_err_info		safety_err_info;
	struct fj_charger_info		charger_info;
	fj_chg_batt_vol_info		batt_vol_data;

	struct workqueue_struct		*fj_charger_drv_wq;
};

static int fj_chg_charger_debug = 0;
extern int charging_mode;

struct charge_drv_chip *the_chip = NULL;
static unsigned long fj_charger_src = 0;
static struct wake_lock fj_chg_timer_wake_lock;
static struct wake_lock fj_chg_charging_wake_lock;

static DEFINE_SPINLOCK(fj_chg_info_lock);
static DEFINE_SPINLOCK(fj_chg_queue_lock);

static LIST_HEAD(fj_chg_evt_list_free);
static LIST_HEAD(fj_chg_evt_list_exec);
static fj_chg_evt_queue fj_charge_evt_queue[FJ_CHG_EVT_QUEUE_MAX];

static int fj_charger_oki_det_irq = 0;

static int chg_enable = 0;
static int chg_mode = 0;

static int mc_chg_voltage = 0;
static int mc_chg_iusb_max = 0;
static int mc_chg_idc_max = 0;
static int mc_chg_current = 0;
static int mc_charger_enable = 0;

static enum power_supply_property fj_chg_mains_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static const fj_chg_voltage_threshold fj_chg_volt_chk_table[FJ_CHG_CHARGE_TYPE_NUM] = {
	{0,                              0                             },
	{FJ_CHG_VALUE_ADP_VOLTAGE_5V_OV, FJ_CHG_VALUE_ADP_VOLTAGE_5V_UV},
	{FJ_CHG_VALUE_ADP_VOLTAGE_9V_OV, FJ_CHG_VALUE_ADP_VOLTAGE_9V_UV},
};

static const fj_chg_voltage_threshold fj_chg_holder_volt_chk_table[FJ_CHG_CHARGE_TYPE_NUM] = {
	{0,                                 0                                },
	{FJ_CHG_VALUE_HOLDER_VOLTAGE_5V_OV, FJ_CHG_VALUE_HOLDER_VOLTAGE_5V_UV},
	{FJ_CHG_VALUE_HOLDER_VOLTAGE_9V_OV, FJ_CHG_VALUE_HOLDER_VOLTAGE_9V_UV},
};

typedef enum {
	FJ_CHG_ERR_NON = 0,						/* error not occurred		*/
	FJ_CHG_ERR_EXPIRE_TIME_OUT,				/* safe timer expire		*/
	FJ_CHG_ERR_USB_PORT_FAILURE,			/* usb port failure			*/
	FJ_CHG_ERR_OVP_DETECT,					/* ovp detect				*/
	FJ_CHG_ERR_CHARGE_DISABLE,				/* charge disable			*/
	FJ_CHG_ERR_BATT_TEMP_HOT_ERR,			/* batt temp hot 			*/
	FJ_CHG_ERR_BATT_TEMP_COLD_ERR,			/* batt temp cold			*/
	FJ_CHG_ERR_OVER_BATT_VOLTAGE,			/* over batt voltage		*/
	FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR,		/* over supply voltage		*/
	FJ_CHG_ERR_UNDER_SUPPLY_VOLTAGE_ERR,	/* under supply voltage		*/
	FJ_CHG_ERR_USB_HOT_ERR,					/* usb hot					*/
	FJ_CHG_ERR_HEAT_CHARGE,					/* heat charge				*/
	FJ_CHG_ERR_HOLDER_FAILURE,				/* holder failure			*/
	FJ_CHG_ERR_WEAK_ADAPTER,				/* weak adapter detect		*/
	FJ_CHG_ERR_OTHER_ERR,					/* other error				*/
	FJ_CHG_ERR_FGIC_INITIALIZING,			/* FGIC initializing		*/
	FJ_CHG_ERR_NUM
} fj_charger_err_type;

static const fj_chg_err_ctrl err_ctrl_table[FJ_CHG_ERR_NUM] = {
/*	  set_status						set_health									charge_ctrl					*/
	{ POWER_SUPPLY_STATUS_CHARGING,		POWER_SUPPLY_HEALTH_GOOD,					FJ_CHG_CONTROL_NONE		 },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_UNSPEC_FAILURE,			FJ_CHG_CONTROL_POWERPATH },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_USB_HOT,				FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_OVER_SUPPLY_VOLTAGE,	FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_GOOD,					FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_CHARGING,		POWER_SUPPLY_HEALTH_GOOD,					FJ_CHG_CONTROL_POWERPATH },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_COLD,					FJ_CHG_CONTROL_POWERPATH },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_OVERVOLTAGE,			FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_OVER_SUPPLY_VOLTAGE,	FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_DISCHARGING,	POWER_SUPPLY_HEALTH_GOOD,					FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_USB_HOT,				FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_CHARGING,		POWER_SUPPLY_HEALTH_GOOD,					FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_HOLDER_ERROR,			FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_OTHER_ERROR,			FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING,	POWER_SUPPLY_HEALTH_OTHER_ERROR,			FJ_CHG_CONTROL_STOP		 },
	{ POWER_SUPPLY_STATUS_CHARGING,		POWER_SUPPLY_HEALTH_GOOD,					FJ_CHG_CONTROL_POWERPATH },
};

static const fj_chg_convert_data convert_ibat_table[] = {
	{ FJ_CHG_CURRENT_1500, 0x10 },
	{ FJ_CHG_CURRENT_3000, 0x1F },
	{ FJ_CHG_CURRENT_OFF , 0x0D },
};

static const fj_chg_convert_data convert_ilimit_table[] = {
	{ FJ_CHG_CURRENT_300,  0x00 },
	{ FJ_CHG_CURRENT_500,  0x04 },
	{ FJ_CHG_CURRENT_900,  0x09 },
	{ FJ_CHG_CURRENT_1500, 0x10 },
	{ FJ_CHG_CURRENT_OFF , 0x04 },
};

typedef fj_charger_state_type (*fj_charge_func)(struct charge_drv_chip *chip, fj_chg_evt_data *data);

/**
 * fj_chg_queue_put
 *
 * @param   entry_p : entry queue pointer
 *          head_p  : list head pointer
 */
static void fj_chg_queue_put(fj_chg_evt_queue *entry_p, struct list_head *head_p)
{
	FJ_CHG_QUEUE_LOCK();
	list_add_tail(&entry_p->head, head_p);
	FJ_CHG_QUEUE_UNLOCK();
}

/**
 * fj_chg_queue_get
 *
 * @param   head_p : list head pointer
 *
 * @retval  queue pointer
 */
static void *fj_chg_queue_get(struct list_head *head_p)
{
	void *queue_p = NULL;
	struct list_head *p, *n;

	FJ_CHG_QUEUE_LOCK();
	list_for_each_safe(p, n, head_p) {
		queue_p = list_entry(p, fj_chg_evt_queue, head);
		if (queue_p != NULL) {
			list_del(p);
			break;
		}
		queue_p = NULL;
	}
	FJ_CHG_QUEUE_UNLOCK();

	return queue_p;
}

/**
 * fj_chg_event_req
 *
 * @param   chip     : charger device info pointer
 *          event    : event type
 *          evt_data : event data pointer
 */
static void fj_chg_event_req(struct charge_drv_chip *chip,
							 fj_charger_event_type event,
							 fj_chg_evt_data *evt_data)
{
	fj_chg_evt_queue *evt_queue_p = NULL;

	evt_queue_p = fj_chg_queue_get(&fj_chg_evt_list_free);
	if (unlikely(evt_queue_p == NULL)) {
		FJ_CHARGER_ERRLOG("[%s] charger event queue empty!!\n", __func__);
		return;
	}

	evt_queue_p->event = event;
	if (evt_data == NULL) {
		memset(&evt_queue_p->data, 0, sizeof(fj_chg_evt_data));
	} else {
		memcpy(&evt_queue_p->data, evt_data, sizeof(fj_chg_evt_data));
	}

	fj_chg_queue_put(evt_queue_p, &fj_chg_evt_list_exec);

	if (chip != NULL) {
		wake_up(&chip->chg_wq);
	}
}

/**
 * fj_chg_reg_read
 *
 * @param   cmd : command id
 *          val : read data
 *
 * @retval  Processing result
 */
static int fj_chg_reg_read(u16 cmd, u8 *val)
{
	u8 val_work = 0;
	int ret;

	if (unlikely(val == NULL)) {
		FJ_CHARGER_ERRLOG("[%s] Invalid parameter. cmd:[%d]\n", __func__, cmd);
		return -EINVAL;
	}

	ret = qpnp_charger_read(cmd, &val_work);
	if (likely(ret == 0)) {
		*val = val_work;
	} else {
		FJ_CHARGER_ERRLOG("PMIC register read failure. cmd:[%d]\n", cmd);
	}

	FJ_CHARGER_DBGLOG("[%s] cmd:[%d] val:[0x%02X] ret:[%d]\n",
												__func__, cmd, *val, ret);

	return ret;
}

/**
 * fj_chg_reg_write_byte
 *
 * @param   cmd  : command id
 *          mask : mask data
 *          val  : write data
 *
 * @retval  Processing result
 */
static int fj_chg_reg_write_byte(u16 cmd, u8 mask, u8 val)
{
	int ret;

	ret = qpnp_charger_write(cmd, mask, val);
	if (unlikely(ret != 0)) {
		FJ_CHARGER_ERRLOG("PMIC register write failure. cmd:[%d] mask:[0x%02X] val:[0x%02X]\n",
																			cmd, mask, val);
	}

	FJ_CHARGER_DBGLOG("[%s] cmd:[%d] mask:[0x%02X] val:[0x%02X] ret:[%d]\n",
												__func__, cmd, mask, val, ret);

	return ret;
}

/**
 * fj_chg_reg_write_bit
 *
 * @param   cmd  : command id
 *          mask : mask bit
 *          set  : set type
 *
 * @retval  Processing result
 */
static int fj_chg_reg_write_bit(u16 cmd, u8 mask, u8 set)
{
	int ret;

	if (set == FJ_CHG_SET_HIGH) {
		ret = qpnp_charger_write(cmd, mask, mask);
	} else {
		ret = qpnp_charger_write(cmd, mask, 0x00);
	}
	if (unlikely(ret != 0)) {
		FJ_CHARGER_ERRLOG("PMIC register write failure. cmd:[%d] mask:[0x%02X] set:[%d]\n",
																		cmd, mask, set);
	}

	FJ_CHARGER_DBGLOG("[%s] cmd:[%d] mask:[0x%02X] set:[%d] ret:[%d]\n",
												__func__, cmd, mask, set, ret);

	return ret;
}

/**
 * fj_chg_check_current
 *
 * @param   chip           : charger device info pointer
 *          ibat_curent    : 
 *          ilimit_current : 
 */
static void fj_chg_check_current(struct charge_drv_chip *chip, unsigned int *ibat_current, unsigned int *ilimit_current)
{
	unsigned int ilimit_work;
	int qc_ilimit_value;
	int weak_adp_ilimit;

	/* IBAT */
	if (!chip->err_info.batt_temp_restrict) {
		*ibat_current = FJ_CHG_CURRENT_3000;
	} else {
		*ibat_current = FJ_CHG_CURRENT_1500;
	}

	/* ILIMIT */
	if (FJ_CHG_IS_HOLDER_CHARGE(chip)) {
		qc_ilimit_value = chip->dev_holder.qc_ilimit_value;
		weak_adp_ilimit = chip->dev_holder.weak_adp_ilimit;
		ilimit_work = chip->dev_holder.max_current;
	} else {
		qc_ilimit_value = chip->dev_usb.qc_ilimit_value;
		weak_adp_ilimit = chip->dev_usb.weak_adp_ilimit;
		ilimit_work = chip->dev_usb.max_current;
	}

	if (chip->battery_low_flag) {
		ilimit_work = CHG_GET_SMALL(FJ_CHG_CURRENT_500, ilimit_work);
	}

	if (qc_ilimit_value != FJ_CHG_ILIMIT_NOT_LIMITED) {
		ilimit_work = CHG_GET_SMALL(qc_ilimit_value, ilimit_work);
	}

	if (weak_adp_ilimit != FJ_CHG_ILIMIT_NOT_LIMITED) {
		ilimit_work = CHG_GET_SMALL(weak_adp_ilimit, ilimit_work);
	}

	if (chip->max_ilimit_value != FJ_CHG_ILIMIT_NOT_LIMITED) {
		if (chip->max_ilimit_value == FJ_CHG_ILIMIT_DISABLE) {
			ilimit_work = FJ_CHG_ILIMIT_LIMITED_MIN;
		} else {
			ilimit_work = CHG_GET_SMALL(chip->max_ilimit_value, ilimit_work);
		}
	}

	*ilimit_current = ilimit_work;

	FJ_CHARGER_INFOLOG("[%s] %s ibat_current:[%d] ilimit_current:[%d]\n",
					__func__, CHG_TYPE_LOG(chip), *ibat_current, *ilimit_current);
}

/**
 * fj_chg_pmic_aicl_setting
 *
 * @param   chip   : charger device info pointer
 *          device : preferential device
 *          type   : AICL control type
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_aicl_setting(struct charge_drv_chip *chip,
									fj_charger_device_type device,
									fj_charger_aicl_type type)
{
	int ret;

	FJ_CHARGER_DBGLOG("[%s] device:[%d] type:[%d] batt:[%d]\n",
								__func__, device, type, chip->battery_low_flag);

	do {
		if ((chip->battery_low_flag) &&
			(type == FJ_CHG_AICL_TYPE_ENABLE)) {
			FJ_CHARGER_INFOLOG("[%s] AICL is invalidated. Because the battery voltage is low.\n", __func__);
			type = FJ_CHG_AICL_TYPE_DISABLE;
		}

		switch (type) {
			case FJ_CHG_AICL_TYPE_ENABLE:
				ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(7), FJ_CHG_SET_HIGH);
				if (unlikely(ret != 0)) {
					FJ_CHARGER_ERRLOG("[%s] AICL enable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
					break;
				}
				ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(6), FJ_CHG_SET_HIGH);
				if (unlikely(ret != 0)) {
					FJ_CHARGER_ERRLOG("[%s] AICL enable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
					break;
				}
				if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(4), FJ_CHG_SET_HIGH);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL enable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1, BIT(2), FJ_CHG_SET_HIGH);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL enable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
				} else {
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(5), FJ_CHG_SET_HIGH);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL enable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG, BIT(2), FJ_CHG_SET_HIGH);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL enable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
				}
				break;

			case FJ_CHG_AICL_TYPE_DISABLE:
				if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(4), FJ_CHG_SET_LOW);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL disable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1, BIT(2), FJ_CHG_SET_LOW);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL disable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
				} else {
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(5), FJ_CHG_SET_LOW);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL disable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG, BIT(2), FJ_CHG_SET_LOW);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL disable Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
				}
				break;

			case FJ_CHG_AICL_TYPE_RERUN:
				if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1, BIT(2), FJ_CHG_SET_LOW);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL restart Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
					msleep(50);
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1, BIT(2), FJ_CHG_SET_HIGH);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL restart Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(4), FJ_CHG_SET_HIGH);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL restart Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
				} else {
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG, BIT(2), FJ_CHG_SET_LOW);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL restart Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
					msleep(50);
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG, BIT(2), FJ_CHG_SET_HIGH);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL restart Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
					ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(5), FJ_CHG_SET_HIGH);
					if (unlikely(ret != 0)) {
						FJ_CHARGER_ERRLOG("[%s] AICL restart Failed. dev:[%d] ret:[%d]\n", __func__, device, ret);
						break;
					}
				}
				break;

			default:
				break;
		}
	} while(0);

	return ret;
}

/**
 * fj_chg_pmic_slight_charge
 *
 * @param   chip   : charger device info pointer
 *          device : device type
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_slight_charge(struct charge_drv_chip *chip, fj_charger_device_type device)
{
	int ret = 0;

	do {
		if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
			ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_HOLDER, FJ_CHG_AICL_TYPE_DISABLE);
			if (unlikely(ret != 0)) {
				break;
			}
			ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DCIN_IL_CFG, 0x1F, 0x04);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("CMD_SMBCHG_DC_CHGPTH_DCIN_IL_CFG Failed. ret:[%d]\n", ret);
				break;
			}
		} else {
			ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_USB, FJ_CHG_AICL_TYPE_DISABLE);
			if (unlikely(ret != 0)) {
				break;
			}
			ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USBIN_IL_CFG, 0x1F, 0x04);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_USBIN_IL_CFG Failed. ret:[%d]\n", ret);
				break;
			}
		}
	} while (0);

	return ret;
}

/**
 * fj_chg_pmic_set_preferential_device
 *
 * @param   chip   : charger device info pointer
 *          device : preferential device
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_set_preferential_device(struct charge_drv_chip *chip,
											   fj_charger_device_type device)
{
	int ret = 0;

	if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_APSD_CFG, BIT(7), FJ_CHG_SET_LOW);
	} else {
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_APSD_CFG, BIT(7), FJ_CHG_SET_HIGH);
	}
	if (unlikely(ret != 0)) {
		FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_APSD_CFG Failed. ret:[%d]\n", ret);
	}

	return ret;
}

/**
 * fj_chg_pmic_watchdog_pet
 */
static void fj_chg_pmic_watchdog_pet(void)
{
	int ret;

	ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_WDOG_RST, BIT(7), FJ_CHG_SET_HIGH);
	if (unlikely(ret != 0)) {
		FJ_CHARGER_ERRLOG("[%s] WDOG_PET Failed. ret:[%d]\n", __func__, ret);
	}
}

/**
 * fj_chg_pmic_current_setting
 *
 * @param   chip : charger device info pointer
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_current_setting(struct charge_drv_chip *chip)
{
	unsigned int ibat_current = 0;
	unsigned int ilimit_current = 0;
	u8 set_reg = 0;
	int i = 0;
	int j = 0;
	int ret;

	do {
		fj_chg_check_current(chip, &ibat_current, &ilimit_current);

		do {
			if (convert_ibat_table[i].check_current == ibat_current) {
				break;
			}
			i++;
		} while (convert_ibat_table[i].check_current != FJ_CHG_CURRENT_OFF);

		set_reg = convert_ibat_table[i].reg_data;

		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_FCC_CFG, 0x1F, set_reg);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_FCC_CFG Failed. ret:[%d]\n", ret);
			break;
		}

		do {
			if (convert_ilimit_table[j].check_current == ilimit_current) {
				break;
			}
			j++;
		} while (convert_ilimit_table[j].check_current != FJ_CHG_CURRENT_OFF);

		set_reg = convert_ilimit_table[j].reg_data;

		if (FJ_CHG_IS_HOLDER_CHARGE(chip)) {
			ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DCIN_IL_CFG, 0x1F, set_reg);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("CMD_SMBCHG_DC_CHGPTH_DCIN_IL_CFG Failed. ret:[%d]\n", ret);
			}
		} else {
			ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USBIN_IL_CFG, 0x1F, set_reg);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_USBIN_IL_CFG Failed. ret:[%d]\n", ret);
			}
		}
	} while(0);

	return ret;
}

/**
 * fj_chg_pmic_initial_setting
 *
 * @param   chip : charger device info pointer
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_initial_setting(struct charge_drv_chip *chip)
{
	int ret;

	do {
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_PCC_CFG, 0x07, 0x04);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_PCC_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_CFG_P2F, 0x03, 0x03);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CFG_P2F Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USBIN_IL_CFG, 0x1F, 0x04);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_USBIN_IL_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DCIN_IL_CFG, 0x1F, 0x04);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_DC_CHGPTH_DCIN_IL_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_FV_CFG, 0x3F, chip->vmax_adj);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_FV_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_FCC_CFG, 0x1F, 0x1F);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_FCC_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2, BIT(3), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_CFG_TCC, 0x07, 0x02);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_FCC_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CFG, BIT(2), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CFG, BIT(1), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(1), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CMD_IL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_APSD_CFG, BIT(7), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_APSD_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(7), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(6), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(5), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8, BIT(4), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_AICL_WL_SEL_CFG, 0x03, 0x00);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_DC_CHGPTH_AICL_WL_SEL_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG, BIT(2), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1, BIT(2), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_BAT_IF_VBL_SEL_CFG, BIT(1), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_BAT_IF_VBL_SEL_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG2, BIT(0), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2, BIT(2), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_CCMP_CFG, 0x3F, 0x30);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DCIN_CHGR_CFG, 0x07, 0x02);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_DC_CHGPTH_DCIN_CHGR_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USBIN_CHGR_CFG, 0x07, 0x02);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_USBIN_CHGR_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_MISC_CFG_TEMP_SEL, 0x03, 0x02);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_MISC_CFG_TEMP_SEL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CFG, BIT(6), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_SFT_CFG, 0x3F, 0x00);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_SFT_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_BAT_IF_BM_CFG, 0x03, 0x02);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_BAT_IF_BM_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(3), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CMD_IL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(4), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CMD_IL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG, BIT(1), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_BAT_IF_CMD_CHG Failed. ret:[%d]\n", ret);
			break;
		}
	} while(0);

	return ret;
}

/**
 * fj_chg_pmic_normal_charge
 *
 * @param   chip   : charger device info pointer
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_normal_charge(struct charge_drv_chip *chip)
{
	int ret = 0;

	do {
		ret = fj_chg_pmic_current_setting(chip);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("[%s] Normal Charge Setting Failed. ret:[%d]\n", __func__, ret);
			break;
		}

		if (FJ_CHG_IS_HOLDER_CHARGE(chip)) {
			ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_HOLDER, FJ_CHG_AICL_TYPE_ENABLE);
		} else {
			ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_USB, FJ_CHG_AICL_TYPE_ENABLE);
		}
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("[%s] Normal Charge Setting Failed. ret:[%d]\n", __func__, ret);
			break;
		}
	} while (0);

	return ret;
}

/**
 * fj_chg_pmic_quick_charge
 *
 * @param   chip   : charger device info pointer
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_quick_charge(struct charge_drv_chip *chip)
{
	int ret = 0;

	ret = fj_chg_pmic_current_setting(chip);
	if (unlikely(ret != 0)) {
		FJ_CHARGER_ERRLOG("[%s] Quick Charge Setting Failed. ret:[%d]\n", __func__, ret);
	}

	return ret;
}

/**
 * fj_chg_pmic_charge_start
 *
 * @param   chip : charger device info pointer
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_charge_start(struct charge_drv_chip *chip)
{
	int ret;

	do {
		ret = fj_chg_reg_write_byte(FJ_CHG_CMD_SMBCHG_CHGR_SFT_CFG, 0x30, 0x20);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_SFT_CFG Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(0), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CMD_IL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2, BIT(7), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2, BIT(6), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		if (FJ_CHG_IS_HOLDER_CHARGE(chip)) {
			ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(3), FJ_CHG_SET_LOW);
		} else {
			ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(4), FJ_CHG_SET_LOW);
		}
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CMD_IL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG, BIT(1), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_BAT_IF_CMD_CHG Failed. ret:[%d]\n", ret);
			break;
		}
	} while(0);

	return ret;
}

/**
 * fj_chg_pmic_charge_powerpath
 *
 * @param   chip   : charger device info pointer
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_charge_powerpath(struct charge_drv_chip *chip)
{
	int ret;

	do {
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2, BIT(7), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2, BIT(6), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		if (FJ_CHG_IS_HOLDER_CHARGE(chip)) {
			ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(3), FJ_CHG_SET_LOW);
		} else {
			ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(4), FJ_CHG_SET_LOW);
		}
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CMD_IL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG, BIT(1), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_BAT_IF_CMD_CHG Failed. ret:[%d]\n", ret);
			break;
		}
	} while (0);

	return ret;
}

/**
 * fj_chg_pmic_charge_stop
 *
 * @param   chip   : charger device info pointer
 *
 * @retval  Processing result
 */
static int fj_chg_pmic_charge_stop(struct charge_drv_chip *chip)
{
	int ret;

	do {
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2, BIT(7), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2, BIT(6), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_CHGR_CHGR_CFG2 Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(3), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CMD_IL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL, BIT(4), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_USB_CHGPTH_CMD_IL Failed. ret:[%d]\n", ret);
			break;
		}
		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG, BIT(1), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_BAT_IF_CMD_CHG Failed. ret:[%d]\n", ret);
			break;
		}
	} while (0);

	return ret;
}

/**
 * fj_chg_start_monitor
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_start_monitor(struct charge_drv_chip *chip)
{
	unsigned long set_timer;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	set_timer = msecs_to_jiffies(FJ_CHG_SET_TIMER_10S);
	queue_delayed_work(chip->fj_charger_drv_wq, &chip->charge_monitor_work, round_jiffies_relative(set_timer));
}

/**
 * fj_chg_stop_monitor
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_stop_monitor(struct charge_drv_chip *chip)
{
	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	cancel_delayed_work(&chip->charge_monitor_work);
}

/**
 * fj_chg_monitor_work
 *
 * @param   work :
 */
static void fj_chg_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, charge_monitor_work);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.monitor_info.type = FJ_CHG_MONITOR_NORMAL;
	fj_chg_event_req(chip, FJ_CHG_EVENT_MONITOR_PARAM, &evt_data);
}

/**
 * fj_chg_start_full_timer1
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_start_full_timer1(struct charge_drv_chip *chip)
{
	unsigned long set_timer;

	if ((!delayed_work_pending(&chip->full_timer_work1)) &&
		(!delayed_work_pending(&chip->full_timer_work2))) {
		FJ_CHARGER_DBGLOG("[%s] in\n", __func__);
		set_timer = msecs_to_jiffies(FJ_CHG_SET_TIMER_10MIN);
		queue_delayed_work(chip->fj_charger_drv_wq, &chip->full_timer_work1, round_jiffies_relative(set_timer));
	}
}

/**
 * fj_chg_start_full_timer2
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_start_full_timer2(struct charge_drv_chip *chip)
{
	unsigned long set_timer;

	if ((!delayed_work_pending(&chip->full_timer_work1)) &&
		(!delayed_work_pending(&chip->full_timer_work2))) {
		FJ_CHARGER_DBGLOG("[%s] in\n", __func__);
		set_timer = msecs_to_jiffies(FJ_CHG_SET_TIMER_2H);
		queue_delayed_work(chip->fj_charger_drv_wq, &chip->full_timer_work2, round_jiffies_relative(set_timer));
	}
}

/**
 * fj_chg_stop_full_timer
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_stop_full_timer(struct charge_drv_chip *chip)
{
	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	cancel_delayed_work(&chip->full_timer_work1);
	cancel_delayed_work(&chip->full_timer_work2);
}

/**
 * fj_chg_full_timer_work1
 *
 * @param   work :
 */
static void fj_chg_full_timer_work1(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, full_timer_work1);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.timer_info.type = FJ_CHG_TIMER_FULL_1;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_TIMER, &evt_data);
}

/**
 * fj_chg_full_timer_work2
 *
 * @param   work :
 */
static void fj_chg_full_timer_work2(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, full_timer_work2);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.timer_info.type = FJ_CHG_TIMER_FULL_2;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_TIMER, &evt_data);
}

/**
 * fj_chg_start_safety_timer
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_start_safety_timer(struct charge_drv_chip *chip)
{
	unsigned long set_timer;

	if (!delayed_work_pending(&chip->safety_timer_work)) {
		set_timer = msecs_to_jiffies(FJ_CHG_SET_TIMER_840MIN);
		queue_delayed_work(chip->fj_charger_drv_wq, &chip->safety_timer_work, round_jiffies_relative(set_timer));
		FJ_CHARGER_INFOLOG("[%s] set safety_timer = %dms\n", __func__, FJ_CHG_SET_TIMER_840MIN);
	}
}

/**
 * fj_chg_stop_safety_timer
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_stop_safety_timer(struct charge_drv_chip *chip)
{
	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	cancel_delayed_work(&chip->safety_timer_work);
}

/**
 * fj_chg_safety_timer_work
 *
 * @param   work :
 */
static void fj_chg_safety_timer_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, safety_timer_work);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.timer_info.type = FJ_CHG_TIMER_PROTECTION;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_TIMER, &evt_data);
}

/**
 * fj_chg_start_adp_volt_monitor
 *
 * @param   chip       : charger device info pointer
 *          set_period : set period
 */
static void fj_chg_start_adp_volt_monitor(struct charge_drv_chip *chip, int set_period)
{
	unsigned long set_timer;

	if (!delayed_work_pending(&chip->adp_volt_monitor_work)) {
		FJ_CHARGER_DBGLOG("[%s] in\n", __func__);
		set_timer = msecs_to_jiffies(set_period);
		queue_delayed_work(chip->fj_charger_drv_wq, &chip->adp_volt_monitor_work, set_timer);
	}
}

/**
 * fj_chg_stop_adp_volt_monitor
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_stop_adp_volt_monitor(struct charge_drv_chip *chip)
{
	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	cancel_delayed_work(&chip->adp_volt_monitor_work);
}

/**
 * fj_chg_adp_volt_monitor_work
 *
 * @param   work :
 */
static void fj_chg_adp_volt_monitor_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, adp_volt_monitor_work);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.monitor_info.type = FJ_CHG_MONITOR_VOLTAGE;
	fj_chg_event_req(chip, FJ_CHG_EVENT_MONITOR_PARAM, &evt_data);
}

/**
 * fj_chg_start_negotiation_timer
 *
 * @param   chip   : charger device info pointer
 *          device : charge device
 */
static void fj_chg_start_negotiation_timer(struct charge_drv_chip *chip,
										   fj_charger_device_type device)
{
	fj_chg_device_info *dev_info;
	unsigned long set_timer;

	if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
		dev_info = &chip->dev_holder;
	} else {
		dev_info = &chip->dev_usb;
	}

	if (!delayed_work_pending(&dev_info->nego_timer_work)) {
		FJ_CHARGER_DBGLOG("[%s] device:[%d].\n", __func__, device);
		set_timer = msecs_to_jiffies(FJ_CHG_SET_TIMER_500MS);
		queue_delayed_work(chip->fj_charger_drv_wq, &dev_info->nego_timer_work, set_timer);
	}
}

/**
 * fj_chg_stop_negotiation_timer
 *
 * @param   chip   : charger device info pointer
 *          device : charge device
 */
static void fj_chg_stop_negotiation_timer(struct charge_drv_chip *chip,
										  fj_charger_device_type device)
{
	FJ_CHARGER_DBGLOG("[%s] device:[%d].\n", __func__, device);

	if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
		cancel_delayed_work(&chip->dev_holder.nego_timer_work);
	} else {
		cancel_delayed_work(&chip->dev_usb.nego_timer_work);
	}
}

/**
 * fj_chg_usb_nego_timer_work
 *
 * @param   work :
 */
static void fj_chg_usb_nego_timer_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, dev_usb.nego_timer_work);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.timer_info.type = FJ_CHG_TIMER_NEGOTIATION;
	evt_data.timer_info.device = FJ_CHG_DEVICE_TYPE_USB;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_TIMER, &evt_data);
}

/**
 * fj_chg_holder_nego_timer_work
 *
 * @param   work :
 */
static void fj_chg_holder_nego_timer_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, dev_holder.nego_timer_work);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.timer_info.type = FJ_CHG_TIMER_NEGOTIATION;
	evt_data.timer_info.device = FJ_CHG_DEVICE_TYPE_HOLDER;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_TIMER, &evt_data);
}

/**
 * fj_chg_start_voltage_check_timer
 *
 * @param   chip   : charger device info pointer
 *          device : charge device
 */
static void fj_chg_start_voltage_check_timer(struct charge_drv_chip *chip,
											 fj_charger_device_type device)
{
	fj_chg_device_info *dev_info;
	unsigned long set_timer;

	if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
		dev_info = &chip->dev_holder;
	} else {
		dev_info = &chip->dev_usb;
	}

	if (!delayed_work_pending(&dev_info->check_timer_work)) {
		FJ_CHARGER_DBGLOG("[%s] device:[%d].\n", __func__, device);
		set_timer = msecs_to_jiffies(FJ_CHG_SET_TIMER_500MS);
		queue_delayed_work(chip->fj_charger_drv_wq, &dev_info->check_timer_work, set_timer);
	}
}

/**
 * fj_chg_stop_voltage_check_timer
 *
 * @param   chip   : charger device info pointer
 *          device : charge device
 */
static void fj_chg_stop_voltage_check_timer(struct charge_drv_chip *chip,
											fj_charger_device_type device)
{
	FJ_CHARGER_DBGLOG("[%s] device:[%d].\n", __func__, device);

	if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
		cancel_delayed_work(&chip->dev_holder.check_timer_work);
	} else {
		cancel_delayed_work(&chip->dev_usb.check_timer_work);
	}
}

/**
 * fj_chg_usb_check_timer_work
 *
 * @param   work :
 */
static void fj_chg_usb_check_timer_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, dev_usb.check_timer_work);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.timer_info.device = FJ_CHG_DEVICE_TYPE_USB;
	evt_data.timer_info.type = FJ_CHG_TIMER_VOLTAGE_CHECK;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_TIMER, &evt_data);
}

/**
 * fj_chg_holder_check_timer_work
 *
 * @param   work :
 */
static void fj_chg_holder_check_timer_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, dev_holder.check_timer_work);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.timer_info.device = FJ_CHG_DEVICE_TYPE_HOLDER;
	evt_data.timer_info.type = FJ_CHG_TIMER_VOLTAGE_CHECK;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_TIMER, &evt_data);
}

/**
 * fj_chg_start_oki_det_timer
 *
 * @param   chip       : charger device info pointer
 */
static void fj_chg_start_oki_det_timer(struct charge_drv_chip *chip)
{
	unsigned long set_timer;

	if (!delayed_work_pending(&chip->oki_det_work)) {
		FJ_CHARGER_DBGLOG("[%s] in\n", __func__);
		set_timer = msecs_to_jiffies(FJ_CHG_SET_TIMER_500MS);
		queue_delayed_work(chip->fj_charger_drv_wq, &chip->oki_det_work, set_timer);
	}
}

/**
 * fj_chg_stop_oki_det_timer
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_stop_oki_det_timer(struct charge_drv_chip *chip)
{
	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	cancel_delayed_work(&chip->oki_det_work);
}

/**
 * fj_chg_oki_det_work
 *
 * @param   work :
 */
static void fj_chg_oki_det_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, oki_det_work);
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	FJ_CHARGER_DBGLOG("[%s] state=0x%x\n", __func__, chip->charger_info.state);

	evt_data.notice_info.type = FJ_CHG_NOTICE_OKI_DET;
	evt_data.notice_info.oki_det = FJ_CHG_OKI_DET_TYPE_TIMER;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);
}

/**
 * fj_chg_start_watchdog_timer
 *
 * @param   chip       : charger device info pointer
 */
static void fj_chg_start_watchdog_timer(struct charge_drv_chip *chip)
{
	unsigned long set_timer;

	if (!delayed_work_pending(&chip->watchdog_work)) {
		set_timer = msecs_to_jiffies(FJ_CHG_SET_TIMER_10S);
		queue_delayed_work(chip->fj_charger_drv_wq, &chip->watchdog_work, round_jiffies_relative(set_timer));
	}
}

/**
 * fj_chg_stop_watchdog_timer
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_stop_watchdog_timer(struct charge_drv_chip *chip)
{
	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	cancel_delayed_work(&chip->watchdog_work);
}

/**
 * fj_chg_watchdog_work
 *
 * @param   work :
 */
static void fj_chg_watchdog_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charge_drv_chip *chip = container_of(dwork,
					struct charge_drv_chip, watchdog_work);


	FJ_CHARGER_DBGLOG("[%s] WatchDog Pet.\n", __func__);

	fj_chg_pmic_watchdog_pet();
	fj_chg_start_watchdog_timer(chip);
}

/**
 * fj_chg_oki_det_irq_handler
 *
 * @param   irq  : 
 *          data : charger device info pointer
 *
  * @retval IRQ_HANDLED
*/
static irqreturn_t fj_chg_oki_det_irq_handler(int irq, void *data)
{
    struct charge_drv_chip *chip = data;
	fj_chg_evt_data evt_data;

	wake_lock_timeout(&fj_chg_timer_wake_lock, (FJ_CHG_WAKELOCK_TIMER * HZ));

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	evt_data.notice_info.type = FJ_CHG_NOTICE_OKI_DET;
	evt_data.notice_info.oki_det = FJ_CHG_OKI_DET_TYPE_IRQ;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);

	return IRQ_HANDLED;
}

/**
 * fj_chg_batt_set_status
 *
 * @param   chip   : charger device info pointer
 *          status : set status
 *
 * @retval  Processing result
 */
static int fj_chg_batt_set_status(struct charge_drv_chip *chip, int status)
{
    union power_supply_propval prop = {0,};

	FJ_CHARGER_DBGLOG("[%s] in %d\n", __func__, status);

	if (status >= POWER_SUPPLY_STATUS_MAX) {
		FJ_CHARGER_ERRLOG("[%s] Parameter Error! status = %d\n", __func__, status);
		return -1;
	}
	
	prop.intval = status;
	
    chip->battery->set_property(chip->battery, POWER_SUPPLY_PROP_STATUS, &prop);
	
	return 0;
}

/**
 * fj_chg_batt_set_health
 *
 * @param	chip   : charger device info pointer
 *			health : set health
 *
 * @retval  Processing result
 */
static int fj_chg_batt_set_health(struct charge_drv_chip *chip, int health)
{
    union power_supply_propval prop = {0,};

	FJ_CHARGER_DBGLOG("[%s] in %d\n", __func__, health);

	if (health >= POWER_SUPPLY_HEALTH_MAX) {
		FJ_CHARGER_ERRLOG("[%s] Parameter Error! health = %d\n", __func__, health);
		return -1;
	}
	
	prop.intval = health;
	
	chip->battery->set_property(chip->battery, POWER_SUPPLY_PROP_HEALTH, &prop);
	
	return 0;
}

/**
 * fj_chg_get_charge_mode
 *
 * @retval  charge mode
 */
static int fj_chg_get_charge_mode(void)
{
	int charge_mode = CHARGE_MODE_MANUAL;
	u16 val;
	u8 chg_flag = 0;
	int ret;

	do {
		if ((fj_boot_mode == FJ_MODE_MAKER_MODE) || (fj_boot_mode == FJ_MODE_KERNEL_MODE)) {
			ret = get_nonvolatile(&chg_flag, APNV_MC_CHARGE_FLAG, 1);
			if (likely((ret < 0) || (chg_flag == 0))) {
				charge_mode = CHARGE_MODE_NOT_CHARGE;
				break;
			}
		}

		ret = get_nonvolatile((u8 *)&val, APNV_CHARGE_FG_FUNC_LIMITS_I, 2);
		if (likely(ret >= 0)) {
			if ((val & 0x0001) != 0) {
				charge_mode = CHARGE_MODE_NOT_CHARGE;
			}
		} else {
			FJ_CHARGER_ERRLOG("[%s] NV read err result=%d\n", __func__, ret);
		}
	} while (0);

	FJ_CHARGER_DBGLOG("[%s] chg mode:%d\n", __func__, charge_mode);

	return charge_mode;
}

/**
 * fj_chg_get_property
 *
 * @param   psy :
 *          psp :
 *          val :
 *
 * @retval  Processing result
 */
static int fj_chg_get_property(struct power_supply *psy,
							   enum power_supply_property psp,
							   union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_ONLINE:
			spin_lock(&fj_chg_info_lock);
			switch (psy->type) {
				case POWER_SUPPLY_TYPE_FJ_AC:
					val->intval = (((fj_charger_src & FJ_CHG_SOURCE_MASK) == FJ_CHG_SOURCE_AC) ? 1 : 0);
					break;
				case POWER_SUPPLY_TYPE_FJ_USB:
					val->intval = (((fj_charger_src & FJ_CHG_SOURCE_MASK) == FJ_CHG_SOURCE_USB) ? 1 : 0);
					break;
				case POWER_SUPPLY_TYPE_MHL:
					val->intval = (((fj_charger_src & FJ_CHG_SOURCE_MASK) == FJ_CHG_SOURCE_MHL) ? 1 : 0);
					break;
				case POWER_SUPPLY_TYPE_HOLDER:
					val->intval = ((fj_charger_src & FJ_CHG_SOURCE_HOLDER) ? 1 : 0);
					break;
				default:
					val->intval = 0;
					break;
			}
			spin_unlock(&fj_chg_info_lock);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

/**
 * fj_chg_get_battery_voltage
 *
 * @param   chip  : charger device info pointer
 *
 * @retval  battery voltage
 */
static int fj_chg_get_battery_voltage(struct charge_drv_chip *chip)
{
    union power_supply_propval prop = {0,};
	long vol_work = 0;
	int ave_vol = 0;
	int index_max = 0;
	int ret = 0;
	int i;

	ret = chip->fg_adc->get_property(chip->fg_adc, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] get BATT_TEMP error %d\n", __func__, ret);
	}

	chip->batt_vol_data.vol_data[chip->batt_vol_data.index] = prop.intval;
	chip->batt_vol_data.index++;

	if (chip->batt_vol_data.index >= FJ_CHG_BATT_VOL_INDEX) {
		chip->batt_vol_data.index = 0;
		chip->batt_vol_data.data_full = true;
	}

	if (chip->batt_vol_data.data_full) {
		index_max = FJ_CHG_BATT_VOL_INDEX;
	} else {
		index_max = chip->batt_vol_data.index;
	}

	for (i = 0; i < index_max; i++) {
		vol_work += chip->batt_vol_data.vol_data[i];
	}
	ave_vol = (int)(vol_work / index_max);

	return ave_vol;
}

/**
 * fj_chg_get_parameters
 *
 * @param   chip  : charger device info pointer
 *          param : monitor parameter data pointer
 */
static void fj_chg_get_parameters(struct charge_drv_chip *chip,
								  fj_chg_monitor_param *charger_param)
{
    union power_supply_propval prop = {0,};
	int ret = 0;
	long usbin_work = 0;
	long dcin_work = 0;
	long okiadc_work = 0;
	int i;
	u8 sts_work = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	/* Supply Voltage */
	for (i = 0; i < 4; i++) {
		if (i != 0) usleep(25000);

		ret = chip->fj_pmi_adc->get_property(chip->fj_pmi_adc, POWER_SUPPLY_PROP_VOLT_USBIN_PHYS, &prop);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("[%s] get USB SupplyVoltage error %d\n", __func__, ret);
		}
		usbin_work += prop.intval;

		ret = chip->fj_pmi_adc->get_property(chip->fj_pmi_adc, POWER_SUPPLY_PROP_VOLT_DCIN_PHYS, &prop);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("[%s] get DCIN Voltage error %d\n", __func__, ret);
		}
		dcin_work += prop.intval;

		ret = chip->fj_pmi_adc->get_property(chip->fj_pmi_adc, POWER_SUPPLY_PROP_OKI_ADC_PHYS, &prop);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("[%s] get OKI_ADC Voltage error %d\n", __func__, ret);
		}
		okiadc_work += prop.intval;
	}

	charger_param->supply_volt = (int)(usbin_work / 4);
	charger_param->dc_in_volt = (int)(dcin_work / 4);
	charger_param->oki_adc_volt = (int)(okiadc_work / 4);

	/* Battery Voltage */
	charger_param->batt_volt = fj_chg_get_battery_voltage(chip);

	/* Battery Temp */
	ret = chip->battery->get_property(chip->battery, POWER_SUPPLY_PROP_TEMP, &prop);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] get BATT_TEMP error %d\n", __func__, ret);
	}
	charger_param->batt_temp = prop.intval;

	/* Case Temp */
	ret = chip->fj_pm_adc->get_property(chip->fj_pm_adc, POWER_SUPPLY_PROP_TEMP_CASE_PHYS, &prop);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] get CASE_TEMP error %d\n", __func__, ret);
	}
	charger_param->case_temp = prop.intval;

	/* Status of Charge */
	ret = chip->bms->get_property(chip->bms, POWER_SUPPLY_PROP_CAPACITY, &prop);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] get BATT_CAPACITY error %d\n", __func__, ret);
	}
	charger_param->soc = prop.intval;

	/* charge status */
	ret = fj_chg_reg_read(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_STS, &sts_work);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] get CHARGE_STATUS error %d\n", __func__, ret);
	}
	charger_param->chg_sts = sts_work;

	/* FGIC initial status */
	ret = chip->battery->get_property(chip->battery, POWER_SUPPLY_PROP_INITIALIZE, &prop);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] get FGIC_INITIAL_STATUS error %d\n", __func__, ret);
	}
	charger_param->fgic_initial_sts = prop.intval;
}

/**
 * fj_chg_check_supply_voltage
 *
 * @param   chip        : charger device info pointer
 *          device      : check charger device
 *          charge_type : check charge type
 *          supply_vol  : supply voltage value
 *
 * @retval  err_type
 */
static fj_charger_err_type fj_chg_check_supply_voltage(struct charge_drv_chip *chip,
													   fj_charger_device_type device,
													   fj_charger_charge_type charge_type,
													   int supply_vol)
{
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	int uv_threshold = fj_chg_volt_chk_table[charge_type].under_voltage;
	int qc_limit = 0;

	do {
		if (charge_type == FJ_CHG_CHARGE_TYPE_UNKNOWN) {
			FJ_CHARGER_ERRLOG("[%s] charge type unknown. dev:[%d]\n", __func__, device);
			err_type = FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR;
			break;
		}

		if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
			qc_limit = chip->dev_holder.qc_ilimit_value;
		} else {
			qc_limit = chip->dev_usb.qc_ilimit_value;
		}
		if ((charge_type == FJ_CHG_CHARGE_TYPE_QUICK) && (qc_limit == FJ_CHG_ILIMIT_LIMITED_500)) {
			uv_threshold = FJ_CHG_VALUE_ADP_VOLTAGE_9V_UV_L;
		}

		if (supply_vol < uv_threshold) {
			FJ_CHARGER_ERRLOG("[%s] Under SupplyVoltage. dev:[%d] volt:[%d] type:[%d]\n",
													__func__, device, supply_vol, charge_type);
			err_type = FJ_CHG_ERR_UNDER_SUPPLY_VOLTAGE_ERR;
			break;
		}
		if (supply_vol > fj_chg_volt_chk_table[charge_type].over_voltage) {
			FJ_CHARGER_ERRLOG("[%s] Over SupplyVoltage. dev:[%d] volt:[%d] type:[%d]\n",
													__func__, device, supply_vol, charge_type);
			err_type = FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR;
			break;
		}

		FJ_CHARGER_DBGLOG("[%s] dev:[%d] volt:[0x%X] type:[%d]\n",
								__func__, device, supply_vol, charge_type);
	} while (0);

	return err_type;
}

/**
 * fj_chg_check_parameters
 *
 * @param   chip  : charger device info pointer
 *          param : charger parameter data pointer
 *
 * @retval  error type
 */
static fj_charger_err_type fj_chg_check_parameters(struct charge_drv_chip *chip,
												   fj_chg_monitor_param *param)
{
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	unsigned int current_err = FJ_CHG_PARAM_NO_ERROR;
	bool batt_temp_flag = false;
	bool batt_low_flag = false;
	bool batt_temp_hot_flag = false;
	bool update_flag = false;
	int batt_low_volt_threshold;

	fj_charger_err_type usb_err = FJ_CHG_ERR_NON;
	fj_charger_err_type holder_err = FJ_CHG_ERR_NON;

	#define FJ_CHG_SET_ERR_TYPE(type, set)	if (type == FJ_CHG_ERR_NON) type = set;

	FJ_CHARGER_DBGLOG("[%s] USBIN:[%d] DCIN:[%d] BATT_TEMP:[%d]\n",
				__func__, param->supply_volt, param->dc_in_volt, param->batt_temp);

	/* charge disable check */
	if (chip->max_ilimit_value == FJ_CHG_ILIMIT_DISABLE) {
		FJ_CHARGER_ERRLOG("[%s] FJ_CHG_PARAM_CHG_DISABLE\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_CHARGE_DISABLE);
		current_err |= FJ_CHG_PARAM_CHG_DISABLE;
	}
	/* Battery voltage check */
	if ((param->batt_volt >= FJ_CHG_VALUE_BATT_VOLTAGE_OV) ||
		((chip->charger_info.current_err & FJ_CHG_PARAM_BATT_VOLTAGE_OV) == FJ_CHG_PARAM_BATT_VOLTAGE_OV)) {
		FJ_CHARGER_ERRLOG("[%s] FJ_CHG_VALUE_BATT_VOLTAGE_OV(0x%08x)\n", __func__, param->batt_volt);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_OVER_BATT_VOLTAGE);
		current_err |= FJ_CHG_PARAM_BATT_VOLTAGE_OV;
	}

	if (charging_mode) {
		batt_low_volt_threshold = FJ_CHG_VALUE_OFFCHG_BATT_LOW_VOLT;
	} else {
		batt_low_volt_threshold = FJ_CHG_VALUE_ONCHG_BATT_LOW_VOLT;
	}

	if (param->batt_volt < batt_low_volt_threshold) {
		batt_low_flag = true;
	}
	if (chip->battery_low_flag != batt_low_flag) {
		chip->battery_low_flag = batt_low_flag;
		update_flag = true;
	}
	/* usb port failure check */
	if (chip->safety_err_info.usb_port_failure ||
	    ((chip->charger_info.current_err & FJ_CHG_PARAM_USB_PORT_FAILURE) == FJ_CHG_PARAM_USB_PORT_FAILURE)) {
		FJ_CHARGER_ERRLOG("[%s] FJ_CHG_PARAM_USB_PORT_FAILURE\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_USB_PORT_FAILURE);
		current_err |= FJ_CHG_PARAM_USB_PORT_FAILURE;
	}
	/* safe timer expire check */
	if (chip->err_info.safety_timer_expire) {
		FJ_CHARGER_ERRLOG("[%s] FJ_CHG_PARAM_UNSPEC_FAILURE\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_EXPIRE_TIME_OUT);
		current_err |= FJ_CHG_PARAM_UNSPEC_FAILURE;
	}
	/* ovp detect check */
	if (chip->safety_err_info.ovp_detect ||
	    ((chip->charger_info.current_err & FJ_CHG_PARAM_OVP_DETECT_ERROR) == FJ_CHG_PARAM_OVP_DETECT_ERROR)) {
		FJ_CHARGER_ERRLOG("[%s] FJ_CHG_PARAM_OVP_DETECT_ERROR\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_OVP_DETECT);
		current_err |= FJ_CHG_PARAM_OVP_DETECT_ERROR;
	}
	/* holder check */
	if ((FJ_CHG_IS_HOLDER_CHARGE(chip)) && (chip->err_info.oki_det_check)) {
		if (gpio_get_value(FJ_CHG_OKI_DET_GPIO)) {
			FJ_CHARGER_ERRLOG("[%s] FJ_CHG_PARAM_HOLDER_FAILURE\n", __func__);
			FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_HOLDER_FAILURE);
			current_err |= FJ_CHG_PARAM_HOLDER_FAILURE;
		} else {
			chip->err_info.oki_det_check = false;
			enable_irq(fj_charger_oki_det_irq);
		}
	}
	/* weak adapter check */
	if (chip->err_info.weak_adapter) {
		FJ_CHARGER_ERRLOG("[%s] WEAK ADAPTER DETECT.\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_WEAK_ADAPTER);
		current_err |= FJ_CHG_PARAM_WEAK_ADAPTER;
	}
	/* recovery check */
	if (chip->err_info.recovery_err) {
		FJ_CHARGER_ERRLOG("[%s] RECOVERY IMPOSSIBLE.\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_OTHER_ERR);
		current_err |= FJ_CHG_PARAM_RECOVERY_ERROR;
	}
	/* charge type check */
	if (chip->err_info.chg_type_unknown) {
		FJ_CHARGER_ERRLOG("[%s] CHARGE TYPE UNKNOWN.\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_OTHER_ERR);
		current_err |= FJ_CHG_PARAM_CHG_TYPE_UNKNOWN;
	} else {
		if (((chip->charger_info.chg_source & FJ_CHG_SOURCE_HOLDER) != 0) && chip->dev_holder.voltage_check) {
			if (chip->dev_holder.charge_type == FJ_CHG_CHARGE_TYPE_UNKNOWN) {
				FJ_CHARGER_ERRLOG("[%s] HOLDER CHARGE_TYPE_UNKNOWN\n", __func__);
				FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_HOLDER_FAILURE);
				current_err |= FJ_CHG_PARAM_HOLDER_FAILURE;
				fj_chg_start_negotiation_timer(chip, FJ_CHG_DEVICE_TYPE_HOLDER);
			}
		}
		if (((chip->charger_info.chg_source & FJ_CHG_SOURCE_USB_PORT) != 0) && chip->dev_usb.voltage_check) {
			if (chip->dev_usb.charge_type == FJ_CHG_CHARGE_TYPE_UNKNOWN) {
				FJ_CHARGER_ERRLOG("[%s] USB CHARGE_TYPE_UNKNOWN\n", __func__);
				FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_OTHER_ERR);
				current_err |= FJ_CHG_PARAM_USB_PORT_FAILURE;
				fj_chg_start_negotiation_timer(chip, FJ_CHG_DEVICE_TYPE_USB);
			}
		}
	}
	/* Supply voltage check */
	if (chip->err_info.qc_under_voltage) {
		FJ_CHARGER_ERRLOG("[%s] QUICK CHARGE UNDER VOLTAGE.\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_OTHER_ERR);
		current_err |= FJ_CHG_PARAM_QC_UV_ERROR;
	}
	if (chip->charger_info.state != FJ_CHG_STATE_CHARGING) {
		/* over/under voltage check */
		if ((chip->charger_info.chg_source & FJ_CHG_SOURCE_HOLDER) != 0) {
			if (chip->dev_holder.charge_type != FJ_CHG_CHARGE_TYPE_UNKNOWN) {
				holder_err = fj_chg_check_supply_voltage(chip, FJ_CHG_DEVICE_TYPE_HOLDER,
															chip->dev_holder.charge_type, param->dc_in_volt);
			}
		}
		if ((chip->charger_info.chg_source & FJ_CHG_SOURCE_USB_PORT) != 0) {
			if (chip->dev_usb.charge_type != FJ_CHG_CHARGE_TYPE_UNKNOWN) {
				usb_err = fj_chg_check_supply_voltage(chip, FJ_CHG_DEVICE_TYPE_USB,
															chip->dev_usb.charge_type, param->supply_volt);
			}
		}
		if ((holder_err == FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR) || (usb_err == FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR)) {
			FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR);
			current_err |= FJ_CHG_PARAM_ADP_VOLTAGE_OV;
		}
		if ((holder_err == FJ_CHG_ERR_UNDER_SUPPLY_VOLTAGE_ERR) || (usb_err == FJ_CHG_ERR_UNDER_SUPPLY_VOLTAGE_ERR)) {
			FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_UNDER_SUPPLY_VOLTAGE_ERR);
			current_err |= FJ_CHG_PARAM_ADP_VOLTAGE_UV;
		}
	}
	/* Battery temp cold check */
	if (param->batt_temp <= chip->batt_temp_cold_limit) {
		FJ_CHARGER_ERRLOG("[%s] FJ_CHG_VALUE_BATT_TEMP_COLD(%d)\n", __func__, param->batt_temp);
		current_err |= FJ_CHG_PARAM_BATT_TEMP_COLD;
		batt_temp_flag = true;
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_BATT_TEMP_COLD_ERR);
		chip->batt_temp_cold_limit = FJ_CHG_VALUE_BATT_TEMP_COLD_LIMIT + FJ_CHG_VALUE_BATT_TEMP_LIMIT_CANCEL;
	} else {
		chip->batt_temp_cold_limit = FJ_CHG_VALUE_BATT_TEMP_COLD_LIMIT;
	}
	/* Battery temp hot check */
	if (param->batt_temp >= chip->batt_temp_hot_limit) {
		batt_temp_hot_flag = true;
	}
	if (batt_temp_hot_flag) {
		FJ_CHARGER_ERRLOG("[%s] FJ_CHG_VALUE_BATT_TEMP_HOT(%d), CASE_TEMP(%d)\n", __func__, param->batt_temp, param->case_temp);
		current_err |= FJ_CHG_PARAM_BATT_TEMP_HOT;
		batt_temp_flag = true;
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_BATT_TEMP_HOT_ERR);
		if (chip->batt_temp_hot_limit == FJ_CHG_VALUE_BATT_TEMP_START_LIMIT) {
			chip->batt_temp_hot_limit = FJ_CHG_VALUE_BATT_TEMP_START_LIMIT - FJ_CHG_VALUE_BATT_TEMP_LIMIT_CANCEL;
		} else if (chip->batt_temp_hot_limit == FJ_CHG_VALUE_BATT_TEMP_HOT_LIMIT) {
			chip->batt_temp_hot_limit = FJ_CHG_VALUE_BATT_TEMP_HOT_LIMIT - FJ_CHG_VALUE_BATT_TEMP_LIMIT_CANCEL;
		}
	} else {
		chip->batt_temp_hot_limit = FJ_CHG_VALUE_BATT_TEMP_HOT_LIMIT;
	}
	/* Battery temp warm check */
	if (param->batt_temp >= chip->batt_temp_warm_limit) {
		FJ_CHARGER_WARNLOG("[%s] FJ_CHG_VALUE_BATT_TEMP_WARM(%d)\n", __func__, param->batt_temp);
		current_err |= FJ_CHG_PARAM_BATT_TEMP_WARM;
		batt_temp_flag = true;
		chip->batt_temp_warm_limit = FJ_CHG_VALUE_BATT_TEMP_WARM_LIMIT - FJ_CHG_VALUE_BATT_TEMP_LIMIT_CANCEL;
	} else {
		chip->batt_temp_warm_limit = FJ_CHG_VALUE_BATT_TEMP_WARM_LIMIT;
	}
	/* Battery temp cool check */
	if (param->batt_temp <= chip->batt_temp_cool_limit) {
		FJ_CHARGER_WARNLOG("[%s] FJ_CHG_VALUE_BATT_TEMP_COOL(%d)\n", __func__, param->batt_temp);
		current_err |= FJ_CHG_PARAM_BATT_TEMP_COOL;
		batt_temp_flag = true;
		chip->batt_temp_cool_limit = FJ_CHG_VALUE_BATT_TEMP_COOL_LIMIT + FJ_CHG_VALUE_BATT_TEMP_LIMIT_CANCEL;
	} else {
		chip->batt_temp_cool_limit = FJ_CHG_VALUE_BATT_TEMP_COOL_LIMIT;
	}
	if (batt_temp_flag != chip->err_info.batt_temp_restrict) {
		chip->err_info.batt_temp_restrict = batt_temp_flag;
		update_flag = true;
	}
	/* heat charge mode check */
	if (chip->heat_charge_mode == FJ_CHG_CHARGE_MODE_POWERPATH) {
		FJ_CHARGER_WARNLOG("[%s] FJ_CHG_HEAT_CHARGE_MODE [%d]\n",
												__func__, chip->heat_charge_mode);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_HEAT_CHARGE);
	}

	if (!param->fgic_initial_sts) {
		FJ_CHARGER_ERRLOG("[%s] FGIC INITIALIZING.\n", __func__);
		FJ_CHG_SET_ERR_TYPE(err_type, FJ_CHG_ERR_FGIC_INITIALIZING);
		current_err |= FJ_CHG_PARAM_FGIC_INITIALIZING;
	}

	/* update check */
	if (update_flag) {
		fj_chg_event_req(chip, FJ_CHG_EVENT_PARAM_UPDATE, NULL);
	}

	if (chip->charger_info.current_err != current_err) {
		FJ_CHARGER_RECLOG("[%s] error sts update [0x%08x]->[0x%08x] err_type[%d]\n",
					__func__, chip->charger_info.current_err, current_err, err_type);
	}

	chip->charger_info.current_err = current_err;

	return err_type;
}

/**
 * fj_chg_check_weak_adapter
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_check_weak_adapter(struct charge_drv_chip *chip)
{
	int index = (sizeof(convert_ilimit_table) / sizeof(fj_chg_convert_data)) - 1;
	fj_chg_evt_data evt_data;
	fj_chg_device_info *dev_info;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	if (FJ_CHG_IS_HOLDER_CHARGE(chip)) {
		dev_info = &chip->dev_holder;
	} else {
		dev_info = &chip->dev_usb;
	}

	do {
		if (chip->charge_type != FJ_CHG_CHARGE_TYPE_NORMAL) {
			dev_info->weak_adp_state = FJ_CHG_SUB_STATE_TYPE_1;
			break;
		}

		if (dev_info->weak_adp_ilimit != FJ_CHG_ILIMIT_NOT_LIMITED) {
			for ( ; index >= 0; index--) {
				if (convert_ilimit_table[index].check_current == dev_info->weak_adp_ilimit) {
					index--;
					break;
				}
			}
		}

		dev_info->weak_adp_state = FJ_CHG_SUB_STATE_TYPE_1;

		if (index >= 0) {
			if (dev_info->weak_adp_ilimit != FJ_CHG_ILIMIT_NOT_LIMITED) {
				dev_info->weak_adp_ilimit = convert_ilimit_table[index].check_current;
			} else {
				dev_info->weak_adp_ilimit = FJ_CHG_ILIMIT_LIMITED_900;
			}
			FJ_CHARGER_WARNLOG("[%s] Weak Adapter Detect. ILIMIT:[%d]\n",
										__func__, dev_info->weak_adp_ilimit);
		} else {
			fj_chg_stop_monitor(chip);
			chip->err_info.weak_adapter = true;
			evt_data.monitor_info.type = FJ_CHG_MONITOR_NORMAL;
			fj_chg_event_req(chip, FJ_CHG_EVENT_MONITOR_PARAM, &evt_data);
			FJ_CHARGER_ERRLOG("[%s] Weak Adapter Detect. Charging STOP.\n", __func__);
		}
	} while (0);
}

/**
 * fj_chg_watch_usb_adp_voltage
 *
 * @param   chip       : charger device info pointer
 *          err_type   : error type
 *          supply_vol : supply voltage value
 *
 * @retval  result type
 */
static fj_charger_result_type fj_chg_watch_usb_adp_voltage(struct charge_drv_chip *chip,
														   fj_charger_err_type *err_type,
														   int supply_vol)
{
	fj_charger_result_type result = FJ_CHG_RESULT_SUCCESS;
	fj_charger_charge_type check_type;
	u8 hvdcp_sts = 0;
	int ret = 0;

	do {
		if (!chip->dev_usb.voltage_check) {
			break;
		}

		*err_type = fj_chg_check_supply_voltage(chip, FJ_CHG_DEVICE_TYPE_USB,
													chip->dev_usb.charge_type, supply_vol);
		if (*err_type == FJ_CHG_ERR_NON) {
			chip->dev_usb.watch_count = 0;
			break;
		}

		ret = fj_chg_reg_read(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_HVDCP_STS, &hvdcp_sts);
		if (((hvdcp_sts & 0x12) == 0x12) && (ret == 0)) {
			check_type = FJ_CHG_CHARGE_TYPE_QUICK;
		} else {
			check_type = FJ_CHG_CHARGE_TYPE_NORMAL;
		}
		if (chip->dev_usb.charge_type != check_type) {
			chip->dev_usb.watch_count = 0;
			chip->dev_usb.charge_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}

		chip->dev_usb.watch_count++;

		if (chip->dev_usb.watch_count < 3) {
			result = FJ_CHG_RESULT_VOLTAGE_ERROR;
			break;
		}

		chip->dev_usb.watch_count = 0;

		if (chip->dev_usb.charge_type == FJ_CHG_CHARGE_TYPE_NORMAL) {
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}
		if (*err_type == FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR) {
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}
		if (chip->dev_usb.qc_ilimit_value == FJ_CHG_ILIMIT_LIMITED_500) {
			chip->err_info.qc_under_voltage = true;
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}

		if (chip->dev_usb.qc_ilimit_value == FJ_CHG_ILIMIT_NOT_LIMITED) {
			chip->dev_usb.qc_ilimit_value = FJ_CHG_ILIMIT_LIMITED_900;
		} else {
			chip->dev_usb.qc_ilimit_value = FJ_CHG_ILIMIT_LIMITED_500;
		}
		result = FJ_CHG_RESULT_RETRY;
	} while (0);

	return result;
}

/**
 * fj_chg_watch_holder_adp_voltage
 *
 * @param   chip       : charger device info pointer
 *          err_type   : error type
 *          supply_vol : supply voltage value
 *          oki_adc    : oki_adc value
 *
 * @retval  result type
 */
static fj_charger_result_type fj_chg_watch_holder_adp_voltage(struct charge_drv_chip *chip,
															  fj_charger_err_type *err_type,
															  int supply_vol,
															  int oki_adc)
{
	fj_charger_charge_type check_type;
	fj_charger_result_type result = FJ_CHG_RESULT_SUCCESS;
	bool adc_error = false;
	int uv_threshold;

	do {
		if (!chip->dev_holder.voltage_check) {
			break;
		}

		check_type = chip->dev_holder.charge_type;

		if (check_type == FJ_CHG_CHARGE_TYPE_UNKNOWN) {
			*err_type = FJ_CHG_ERR_HOLDER_FAILURE;
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}

		if ((check_type == FJ_CHG_CHARGE_TYPE_QUICK) &&
			(chip->dev_holder.qc_ilimit_value == FJ_CHG_ILIMIT_LIMITED_500)) {
			uv_threshold = FJ_CHG_VALUE_ADP_VOLTAGE_9V_UV_L;
		} else {
			uv_threshold = fj_chg_holder_volt_chk_table[check_type].under_voltage;
		}

		if (oki_adc < uv_threshold) {
			FJ_CHARGER_ERRLOG("[%s] OKI_ADC Voltage failed. volt:[%d] type:[%d]\n",
															__func__, oki_adc, check_type);
			*err_type = FJ_CHG_ERR_UNDER_SUPPLY_VOLTAGE_ERR;
		} else if (oki_adc > fj_chg_holder_volt_chk_table[check_type].over_voltage) {
			FJ_CHARGER_ERRLOG("[%s] OKI_ADC Voltage failed. volt:[%d] type:[%d]\n",
															__func__, oki_adc, check_type);
			*err_type = FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR;
		} else {
			*err_type = fj_chg_check_supply_voltage(chip, FJ_CHG_DEVICE_TYPE_HOLDER, check_type, supply_vol);
			adc_error = false;
		}

		if (*err_type == FJ_CHG_ERR_NON) {
			chip->dev_holder.watch_count = 0;
			break;
		}

		chip->dev_holder.watch_count++;

		if (chip->dev_holder.watch_count < 3) {
			result = FJ_CHG_RESULT_VOLTAGE_ERROR;
			break;
		}

		chip->dev_holder.watch_count = 0;

		if (adc_error) {
			chip->dev_holder.charge_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}
		if (check_type == FJ_CHG_CHARGE_TYPE_NORMAL) {
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}
		if (*err_type == FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR) {
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}
		if (chip->dev_holder.qc_ilimit_value == FJ_CHG_ILIMIT_LIMITED_500) {
			chip->err_info.qc_under_voltage = true;
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}

		if (chip->dev_holder.qc_ilimit_value == FJ_CHG_ILIMIT_NOT_LIMITED) {
			chip->dev_holder.qc_ilimit_value = FJ_CHG_ILIMIT_LIMITED_900;
		} else {
			chip->dev_holder.qc_ilimit_value = FJ_CHG_ILIMIT_LIMITED_500;
		}
		result = FJ_CHG_RESULT_RETRY;

	} while (0);

	return result;
}

/**
 * fj_chg_charge_type_judgement
 *
 * @param   chip     : charger device info pointer
 *          device   : charger device
 *          err_type : error type
 *
 * @retval  result type
 */
static fj_charger_result_type fj_chg_charge_type_judgement(struct charge_drv_chip *chip,
														   fj_charger_device_type device,
														   fj_charger_err_type *err_type)
{
	fj_chg_device_info *dev_info;
	fj_charger_result_type result = FJ_CHG_RESULT_CONTINUE;
    union power_supply_propval prop = {0,};

	if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
		dev_info = &chip->dev_holder;
	} else {
		dev_info = &chip->dev_usb;
	}

	do {
		if (device == FJ_CHG_DEVICE_TYPE_HOLDER) {
			(void)chip->fj_pmi_adc->get_property(chip->fj_pmi_adc, POWER_SUPPLY_PROP_VOLT_DCIN_PHYS, &prop);
		} else {
			(void)chip->fj_pmi_adc->get_property(chip->fj_pmi_adc, POWER_SUPPLY_PROP_VOLT_USBIN_PHYS, &prop);
		}

		*err_type = fj_chg_check_supply_voltage(chip, device, dev_info->charge_type, prop.intval);
		if (likely(*err_type == FJ_CHG_ERR_NON)) {
			dev_info->check_count[dev_info->charge_type]++;
			if (dev_info->check_count[dev_info->charge_type] >= FJ_CHG_ADP_VOLTAGE_DETECT_CNT) {
				FJ_CHARGER_INFOLOG("[%s] %s charge type %d [1:Normal 2:Quick]\n",
											__func__, CHG_TYPE_LOG(chip), dev_info->charge_type);

				memset(dev_info->check_count, 0, sizeof(dev_info->check_count));
				dev_info->check_total = 0;
				result = FJ_CHG_RESULT_SUCCESS;
				break;
			}
		} else {
			memset(dev_info->check_count, 0, sizeof(dev_info->check_count));
			dev_info->save_voltage_error = *err_type;
			result = FJ_CHG_RESULT_FAILURE;
		}

		dev_info->check_total++;

		if (dev_info->check_total >= FJ_CHG_ADP_VOLTAGE_DETECT_MAX) {
			FJ_CHARGER_ERRLOG("[%s] %s SupplyVoltage Total Count Over Error\n",
													__func__, CHG_TYPE_LOG(chip));
			dev_info->check_total = 0;
			*err_type = dev_info->save_voltage_error;
			result = FJ_CHG_RESULT_OVER;
			break;
		}
	} while (0);

	return result;
}

#define CONSECUTIVE_COUNT		3
#define FJ_CHG_TERMCURRENT_MA	200
/**
 * fj_charge_is_battery_full
 *
 * @param   chip : charger device info pointer
 *
 * @retval  battey full
 */
static bool fj_charge_is_battery_full(struct charge_drv_chip *chip)
{
	union power_supply_propval prop = {0,};
	int ibat_ma = 0;
	bool batt_full = false;
	int ret = 0;

	if (likely(chip->bms)) {
		ret = chip->bms->get_property(chip->bms, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
		if (likely(ret == 0)) {
			ibat_ma = prop.intval / 1000;

			FJ_CHARGER_DBGLOG("[%s] ibat_ma:[%d]\n", __func__, ibat_ma);

			if ((ibat_ma * -1) > FJ_CHG_TERMCURRENT_MA) {
				FJ_CHARGER_DBGLOG("[%s] Not at EOC, battery current too high\n", __func__);
				chip->consecutive_count = 0;
			} else if (ibat_ma > 0) {
				FJ_CHARGER_DBGLOG("[%s] Charging but system demand increased\n", __func__);
				chip->consecutive_count = 0;
			} else {
				chip->consecutive_count++;
			}
		} else {
			FJ_CHARGER_DBGLOG("[%s] Get Current Failed. ret:[%d]\n", __func__, ret);
			chip->consecutive_count = 0;
		}
	} else {
		FJ_CHARGER_DBGLOG("[%s] No BMS supply registered return 0\n", __func__);
		chip->consecutive_count = 0;
	}

	if (chip->consecutive_count > CONSECUTIVE_COUNT) {
		batt_full = true;
		chip->consecutive_count = 0;
	}

	return batt_full;
}

/**
 * fj_chg_terminate_usb
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_terminate_usb(struct charge_drv_chip *chip)
{
	fj_chg_stop_negotiation_timer(chip, FJ_CHG_DEVICE_TYPE_USB);
	fj_chg_stop_voltage_check_timer(chip, FJ_CHG_DEVICE_TYPE_USB);

	chip->dev_usb.charge_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
	chip->dev_usb.max_current = FJ_CHG_CURRENT_OFF;
	memset(chip->dev_usb.check_count, 0, sizeof(chip->dev_usb.check_count));
	chip->dev_usb.check_total = 0;
	memset(chip->dev_usb.nego_count, 0, sizeof(chip->dev_usb.nego_count));
	chip->dev_usb.nego_total = 0;
	chip->dev_usb.watch_count = 0;
	chip->dev_usb.save_voltage_error = FJ_CHG_ERR_NON;
	chip->dev_usb.qc_ilimit_value = FJ_CHG_ILIMIT_NOT_LIMITED;
	chip->dev_usb.weak_adp_ilimit = FJ_CHG_ILIMIT_NOT_LIMITED;
	chip->dev_usb.weak_adp_state = FJ_CHG_SUB_STATE_TYPE_1;
	chip->dev_usb.voltage_check = false;
}

/**
 * fj_chg_terminate_holder
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_terminate_holder(struct charge_drv_chip *chip)
{
	disable_irq(fj_charger_oki_det_irq);

	fj_chg_stop_oki_det_timer(chip);
	fj_chg_stop_negotiation_timer(chip, FJ_CHG_DEVICE_TYPE_HOLDER);
	fj_chg_stop_voltage_check_timer(chip, FJ_CHG_DEVICE_TYPE_HOLDER);

	chip->oki_det_count = 0;
	chip->err_info.oki_det_check = false;

	chip->dev_holder.charge_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
	chip->dev_holder.max_current = FJ_CHG_CURRENT_OFF;
	memset(chip->dev_holder.check_count, 0, sizeof(chip->dev_holder.check_count));
	chip->dev_holder.check_total = 0;
	memset(chip->dev_holder.nego_count, 0, sizeof(chip->dev_holder.nego_count));
	chip->dev_holder.nego_total = 0;
	chip->dev_holder.watch_count = 0;
	chip->dev_holder.save_voltage_error = FJ_CHG_ERR_NON;
	chip->dev_holder.qc_ilimit_value = FJ_CHG_ILIMIT_NOT_LIMITED;
	chip->dev_holder.weak_adp_ilimit = FJ_CHG_ILIMIT_NOT_LIMITED;
	chip->dev_holder.weak_adp_state = FJ_CHG_SUB_STATE_TYPE_1;
	chip->dev_holder.voltage_check = false;
}

/**
 * fj_chg_terminate
 *
 * @param   chip     : charger device info pointer
 *          shutdown : [true]shutdown [false]other
 */
static void fj_chg_terminate(struct charge_drv_chip *chip, bool shutdown)
{
	fj_chg_stop_monitor(chip);
	fj_chg_stop_adp_volt_monitor(chip);
	fj_chg_stop_full_timer(chip);
	fj_chg_stop_safety_timer(chip);
	fj_chg_stop_watchdog_timer(chip);

	fj_chg_terminate_usb(chip);
	fj_chg_terminate_holder(chip);

	if (!shutdown) {
		if (chip->safety_err_info.usb_port_failure == true) {
			(void)fj_chg_batt_set_health(chip, POWER_SUPPLY_HEALTH_USB_HOT);
			(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_NOT_CHARGING);
		} else if (chip->safety_err_info.ovp_detect == true) {
			(void)fj_chg_batt_set_health(chip, POWER_SUPPLY_HEALTH_OVER_SUPPLY_VOLTAGE);
			(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_NOT_CHARGING);
		} else {
			(void)fj_chg_batt_set_health(chip, POWER_SUPPLY_HEALTH_GOOD);
			(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_DISCHARGING);
		}
	}
	fj_chg_pmic_initial_setting(chip);

	chip->charger_info.chg_source  = 0;
	chip->charger_info.chg_current = 0;
	chip->charger_info.current_err = FJ_CHG_PARAM_NO_ERROR;

	chip->connect_state = FJ_CHG_CONNECT_STATE_NONE;
	chip->charge_type = FJ_CHG_CHARGE_TYPE_NORMAL;
	chip->consecutive_count = 0;
	chip->battery_low_flag = false;

	memset(&chip->err_info, 0, sizeof(fj_chg_err_info));

	chip->batt_temp_hot_limit = FJ_CHG_VALUE_BATT_TEMP_START_LIMIT;
	chip->batt_temp_warm_limit = FJ_CHG_VALUE_BATT_TEMP_WARM_LIMIT;
	chip->batt_temp_cold_limit = FJ_CHG_VALUE_BATT_TEMP_COLD_LIMIT;
	chip->batt_temp_cool_limit = FJ_CHG_VALUE_BATT_TEMP_COOL_LIMIT;

	chip->recovery_count = 0;
	chip->recovery_invalid = true;

	memset(chip->batt_vol_data.vol_data, 0, sizeof(chip->batt_vol_data.vol_data));
	chip->batt_vol_data.index = 0;
	chip->batt_vol_data.data_full = false;
}

/**
 * fj_chg_charge_control
 *
 * @param   chip        : charger device info pointer
 *          charge_ctrl : charger control type
 */
static int fj_chg_charge_control(struct charge_drv_chip *chip,
								 fj_charger_ctrl_type charge_ctrl)
{
	int ret = 0;

	FJ_CHARGER_DBGLOG("[%s] in charge_ctrl = %d\n", __func__, charge_ctrl);

	switch (charge_ctrl) {
		case FJ_CHG_CONTROL_PRECHARGE:
			ret = fj_chg_pmic_charge_start(chip);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] PreCharge Failed. ret:[%d]\n", __func__, ret);
			}
			chip->recovery_invalid = false;
			break;

		case FJ_CHG_CONTROL_RECHARGE:
		case FJ_CHG_CONTROL_CHARGE_START:
			ret = fj_chg_pmic_charge_start(chip);
			if (likely(ret == 0)) {
				if (chip->charge_type == FJ_CHG_CHARGE_TYPE_QUICK) {
					ret = fj_chg_pmic_quick_charge(chip);
				} else {
					ret = fj_chg_pmic_normal_charge(chip);
				}
			}
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] Charge Start Failed. ret:[%d]\n", __func__, ret);
			}
			chip->recovery_invalid = false;
			break;

		case FJ_CHG_CONTROL_POWERPATH:
			ret = fj_chg_pmic_charge_powerpath(chip);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] PowerPath Failed. ret:[%d]\n", __func__, ret);
			}
			ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_HOLDER, FJ_CHG_AICL_TYPE_DISABLE);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] PowerPath Failed. ret:[%d]\n", __func__, ret);
			}
			ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_USB, FJ_CHG_AICL_TYPE_DISABLE);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] PowerPath Failed. ret:[%d]\n", __func__, ret);
			}
			chip->recovery_invalid = true;
			break;

		case FJ_CHG_CONTROL_STOP:
			ret = fj_chg_pmic_charge_stop(chip);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] Stop Failed. ret:[%d]\n", __func__, ret);
			}
			ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_HOLDER, FJ_CHG_AICL_TYPE_DISABLE);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] Stop Failed. ret:[%d]\n", __func__, ret);
			}
			ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_USB, FJ_CHG_AICL_TYPE_DISABLE);
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] Stop Failed. ret:[%d]\n", __func__, ret);
			}
			chip->recovery_invalid = true;
			break;

		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

/**
 * fj_chg_err_control
 *
 * @param   chip     : charger device info pointer
 *          err_type : error type
 */
static void fj_chg_err_control(struct charge_drv_chip *chip, fj_charger_err_type err_type)
{
	fj_chg_err_ctrl err_ctrl = {0};

	FJ_CHARGER_DBGLOG("[%s] in err_type = %d\n", __func__, err_type);

	if (err_type >= FJ_CHG_ERR_NUM) {
		return;
	}

	fj_chg_stop_safety_timer(chip);
	fj_chg_stop_full_timer(chip);
	fj_chg_stop_adp_volt_monitor(chip);

	memcpy(&err_ctrl, &err_ctrl_table[err_type], sizeof(fj_chg_err_ctrl));

	if (charging_mode && (err_type!=FJ_CHG_ERR_FGIC_INITIALIZING)) {
		if (err_ctrl.charge_ctrl == FJ_CHG_CONTROL_POWERPATH) {
			err_ctrl.charge_ctrl = FJ_CHG_CONTROL_STOP;
		}
		if ((err_ctrl.charge_ctrl == FJ_CHG_CONTROL_STOP)          &&
			(err_ctrl.set_status == POWER_SUPPLY_STATUS_CHARGING)) {
			err_ctrl.set_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}

	(void)fj_chg_batt_set_health(chip, err_ctrl.set_health);
	(void)fj_chg_batt_set_status(chip, err_ctrl.set_status);
	fj_chg_charge_control(chip, err_ctrl.charge_ctrl);
}

/**
 * fj_chg_nego_usb_control
 *
 * @param   chip : charger device info pointer
 *
 * @retval  Processing result
 */
static fj_charger_result_type fj_chg_nego_usb_control(struct charge_drv_chip *chip)
{
	fj_charger_result_type result = FJ_CHG_RESULT_CONTINUE;
	fj_charger_charge_type search_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
	fj_chg_device_info *dev_info = &chip->dev_usb;
	u8 hvdcp_sts = 0;
	int ret = 0;

	do {
		ret = fj_chg_reg_read(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_HVDCP_STS, &hvdcp_sts);
		if (((hvdcp_sts & 0x12) == 0x12) && (ret == 0)) {
			dev_info->nego_count[FJ_CHG_CHARGE_TYPE_NORMAL] = 0;
			search_type = FJ_CHG_CHARGE_TYPE_QUICK;
		} else {
			dev_info->nego_count[FJ_CHG_CHARGE_TYPE_QUICK] = 0;
			search_type = FJ_CHG_CHARGE_TYPE_NORMAL;
		}
		dev_info->nego_count[search_type]++;

		if (dev_info->nego_count[search_type] >= FJ_CHG_NEGO_USB_DETECT_CNT) {
			dev_info->nego_total = 0;
			memset(dev_info->nego_count, 0, sizeof(dev_info->nego_count));
			dev_info->charge_type = search_type;
			dev_info->voltage_check = true;
			result = FJ_CHG_RESULT_SUCCESS;
			break;
		}

		dev_info->nego_total++;
		if (dev_info->nego_total >= FJ_CHG_NEGO_USB_DETECT_MAX) {
			dev_info->nego_total = 0;
			memset(dev_info->nego_count, 0, sizeof(dev_info->nego_count));
			dev_info->charge_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
			dev_info->voltage_check = true;
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}
	} while (0);

	return result;
}

/**
 * fj_chg_nego_holder_control
 *
 * @param   chip : charger device info pointer
 *
 * @retval  Processing result
 */
static fj_charger_result_type fj_chg_nego_holder_control(struct charge_drv_chip *chip)
{
	fj_charger_result_type result = FJ_CHG_RESULT_CONTINUE;
	fj_charger_charge_type search_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
	fj_chg_device_info *dev_info = &chip->dev_holder;
    union power_supply_propval prop_oki_adc = {0,};

	do {
		(void)chip->fj_pmi_adc->get_property(chip->fj_pmi_adc, POWER_SUPPLY_PROP_OKI_ADC_PHYS, &prop_oki_adc);

		if ((prop_oki_adc.intval >= FJ_CHG_VALUE_HOLDER_VOLTAGE_9V_UV_L) &&
			(prop_oki_adc.intval <= FJ_CHG_VALUE_HOLDER_VOLTAGE_9V_OV)) {
			dev_info->nego_count[FJ_CHG_CHARGE_TYPE_QUICK]++;
			search_type = FJ_CHG_CHARGE_TYPE_QUICK;
		} else {
			dev_info->nego_count[FJ_CHG_CHARGE_TYPE_QUICK] = 0;
		}
		if ((prop_oki_adc.intval >= FJ_CHG_VALUE_HOLDER_VOLTAGE_5V_UV) &&
			(prop_oki_adc.intval <= FJ_CHG_VALUE_HOLDER_VOLTAGE_5V_OV)) {
			dev_info->nego_count[FJ_CHG_CHARGE_TYPE_NORMAL]++;
			search_type = FJ_CHG_CHARGE_TYPE_NORMAL;
		} else {
			dev_info->nego_count[FJ_CHG_CHARGE_TYPE_NORMAL] = 0;
		}

		if (search_type != FJ_CHG_CHARGE_TYPE_UNKNOWN) {
			if (dev_info->nego_count[search_type] >= FJ_CHG_NEGO_HOLDER_DETECT_CNT) {
				dev_info->nego_total = 0;
				memset(dev_info->nego_count, 0, sizeof(dev_info->nego_count));
				dev_info->charge_type = search_type;
				dev_info->voltage_check = true;
				result = FJ_CHG_RESULT_SUCCESS;
				break;
			}
		}

		dev_info->nego_total++;
		if (dev_info->nego_total >= FJ_CHG_NEGO_HOLDER_DETECT_MAX) {
			dev_info->nego_total = 0;
			memset(dev_info->nego_count, 0, sizeof(dev_info->nego_count));
			dev_info->charge_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
			dev_info->voltage_check = true;
			chip->err_info.chg_type_unknown = true;
			result = FJ_CHG_RESULT_FAILURE;
			break;
		}
	} while (0);

	return result;
}

/**
 * fj_chg_charge_enable_control
 *
 * @param   chip   : charger device info pointer
 *          ilimit : 
 */
static void fj_chg_charge_enable_control(struct charge_drv_chip *chip, int ilimit)
{
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	if (chip->max_ilimit_value != ilimit) {
		FJ_CHARGER_DBGLOG("[%s] in chg_en:[%d]->[%d]\n",
									__func__, chip->max_ilimit_value, ilimit);
		if (ilimit != FJ_CHG_ILIMIT_DISABLE) {
			fj_chg_event_req(chip, FJ_CHG_EVENT_PARAM_UPDATE, NULL);
		}
		if ((chip->max_ilimit_value == FJ_CHG_ILIMIT_DISABLE) ||
			(ilimit == FJ_CHG_ILIMIT_DISABLE)) {
			fj_chg_stop_monitor(chip);
			evt_data.monitor_info.type = FJ_CHG_MONITOR_NORMAL;
			fj_chg_event_req(chip, FJ_CHG_EVENT_MONITOR_PARAM, &evt_data);
		}
		chip->max_ilimit_value = ilimit;
	}
}

/**
 * fj_chg_change_mode_control
 *
 * @param   chip : charger device info pointer
 *          mode : change charge mode
 */
static void fj_chg_change_mode_control(struct charge_drv_chip *chip, int mode)
{
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	if (chip->heat_charge_mode != mode) {
		FJ_CHARGER_DBGLOG("[%s] in mode:[%d]\n", __func__, mode);
		fj_chg_stop_monitor(chip);
		chip->heat_charge_mode = mode;
		evt_data.monitor_info.type = FJ_CHG_MONITOR_NORMAL;
		fj_chg_event_req(chip, FJ_CHG_EVENT_MONITOR_PARAM, &evt_data);
	}
}

/**
 * fj_chg_notify_error_control
 *
 * @param   chip  : charger device info pointer
 *          error : 
 */
static void fj_chg_notify_error_control(struct charge_drv_chip *chip, fj_chg_notice_info notice_info)
{
	fj_chg_evt_data evt_data;

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	fj_chg_stop_monitor(chip);
	if (notice_info.type == FJ_CHG_NOTICE_OVP_NOTIFY_ERROR) {
		if (notice_info.ovp_detect == FJ_CHG_ERROR_NONE){
			chip->safety_err_info.ovp_detect = false;
		} else {
			chip->safety_err_info.ovp_detect = true;
		}
	}
	
	if (notice_info.type == FJ_CHG_NOTICE_USBHOT_NOTIFY_ERROR) {
		if (notice_info.usb_hot_detect == FJ_CHG_ERROR_NONE){
			chip->safety_err_info.usb_port_failure = false;
		} else {
			chip->safety_err_info.usb_port_failure = true;
		}
	}
	
	evt_data.monitor_info.type = FJ_CHG_MONITOR_NORMAL;
	fj_chg_event_req(chip, FJ_CHG_EVENT_MONITOR_PARAM, &evt_data);
}

/**
 * fj_chg_oki_det_control
 *
 * @param   chip : charger device info pointer
 *          type : oki_det type
 */
static void fj_chg_oki_det_control(struct charge_drv_chip *chip, fj_charger_oki_det_type type)
{
	fj_chg_evt_data evt_data;

	FJ_CHARGER_DBGLOG("[%s] in type:[%d]\n", __func__, type);

	switch (type) {
		case FJ_CHG_OKI_DET_TYPE_IRQ:
			disable_irq(fj_charger_oki_det_irq);
			(void)fj_chg_charge_control(chip, FJ_CHG_CONTROL_STOP);
			fj_chg_start_oki_det_timer(chip);
			break;

		case FJ_CHG_OKI_DET_TYPE_TIMER:
			if (!gpio_get_value(FJ_CHG_OKI_DET_GPIO)) {
				chip->oki_det_count = 0;
				chip->err_info.oki_det_check = false;
				(void)fj_chg_charge_control(chip, FJ_CHG_CONTROL_RECHARGE);
				enable_irq(fj_charger_oki_det_irq);
			} else {
				chip->oki_det_count++;
				if (chip->oki_det_count > 20) {
					FJ_CHARGER_INFOLOG("[%s] OKI_DET detect.\n", __func__);
					fj_chg_stop_monitor(chip);
					chip->oki_det_count = 0;
					chip->err_info.oki_det_check = true;

					memset(&evt_data, 0, sizeof(fj_chg_evt_data));
					evt_data.monitor_info.type = FJ_CHG_MONITOR_NORMAL;
					fj_chg_event_req(chip, FJ_CHG_EVENT_MONITOR_PARAM, &evt_data);
				} else {
					fj_chg_start_oki_det_timer(chip);
				}
			}
			break;
		default:
			break;
	}
}

/**
 * fj_chg_change_current_control
 *
 * @param   chip        : charger device info pointer
 *          change_info : 
 */
static void fj_chg_change_current_control(struct charge_drv_chip *chip,
										  fj_chg_change_info *change_info)
{
	fj_chg_device_info *dev_info;
	bool update_flag = true;

	if (change_info->device == FJ_CHG_DEVICE_TYPE_HOLDER) {
		dev_info = &chip->dev_holder;
	} else {
		dev_info = &chip->dev_usb;
		if (FJ_CHG_IS_HOLDER_CHARGE(chip)) {
			update_flag = false;
		}
	}

	dev_info->max_current = change_info->max_current;

	if (dev_info->voltage_check && update_flag) {
		fj_chg_event_req(chip, FJ_CHG_EVENT_PARAM_UPDATE, NULL);
	}
}

/**
 * fj_chg_recovery_control
 *
 * @param   chip    : charger device info pointer
 *          chg_sts : charge status
 *
 * @retval  error type
 */
static fj_charger_err_type fj_chg_recovery_control(struct charge_drv_chip *chip, u8 chg_sts)
{
	int ret = 0;
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;

	do {
		if ((chg_sts & 0x06) != 0x00) {
			chip->recovery_count = 0;
			break;
		}

		if (chip->recovery_invalid) {
			FJ_CHARGER_DBGLOG("[%s] recovery invalid.\n", __func__);
			break;
		}

		chip->recovery_count++;
		if (chip->recovery_count >= 3) {
			FJ_CHARGER_ERRLOG("[%s] recovery count over error.\n", __func__);
			chip->err_info.recovery_err = true;
			err_type = FJ_CHG_ERR_OTHER_ERR;
			break;
		}

		/* recovery */
		FJ_CHARGER_INFOLOG("[%s] recovery start!!\n", __func__);

		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG, BIT(1), FJ_CHG_SET_HIGH);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_BAT_IF_CMD_CHG Failed. ret:[%d]\n", ret);
			break;
		}

		msleep(10);

		ret = fj_chg_reg_write_bit(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG, BIT(1), FJ_CHG_SET_LOW);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("CMD_SMBCHG_BAT_IF_CMD_CHG Failed. ret:[%d]\n", ret);
			break;
		}
	} while (0);

	return err_type;
}

/**
 * fj_chg_power_supply_update_all
 *
 * @param   chip : charger device info pointer
 */
static void fj_chg_power_supply_update_all(struct charge_drv_chip *chip)
{
	if (likely(chip != NULL)) {
		FJ_CHARGER_RECLOG("[%s] update online chg_src=0x%04x\n",
						__func__, (unsigned int)chip->charger_info.chg_source);
		power_supply_changed(&chip->usb);
		power_supply_changed(&chip->mains);
		power_supply_changed(&chip->mhl);
		power_supply_changed(&chip->holder);
	}
}

static const fj_chg_connect_ctrl connect_ctrl_table[FJ_CHG_CONNECT_STATE_NUM][FJ_CHG_CONNECT_EVENT_NUM] = {
	/* FJ_CHG_CONNECT_STATE_NONE */ 			{
		/* FJ_CHG_CONNECT_EVENT_USB_IN		*/		{ FJ_CHG_CONNECT_STATE_USB,      FJ_CHG_CONTROL_USB_CHARGE    },
		/* FJ_CHG_CONNECT_EVENT_USB_OUT		*/		{ FJ_CHG_CONNECT_STATE_NOCHANGE, FJ_CHG_CONTROL_NONE          },
		/* FJ_CHG_CONNECT_EVENT_HOLDER_IN	*/		{ FJ_CHG_CONNECT_STATE_HOLDER,   FJ_CHG_CONTROL_HOLDER_CHARGE },
		/* FJ_CHG_CONNECT_EVENT_HOLDER_OUT	*/		{ FJ_CHG_CONNECT_STATE_NOCHANGE, FJ_CHG_CONTROL_NONE          },
	/* FJ_CHG_CONNECT_STATE_USB */				} , {
		/* FJ_CHG_CONNECT_EVENT_USB_IN		*/		{ FJ_CHG_CONNECT_STATE_NOCHANGE, FJ_CHG_CONTROL_USB_CHARGE    },
		/* FJ_CHG_CONNECT_EVENT_USB_OUT		*/		{ FJ_CHG_CONNECT_STATE_NONE,     FJ_CHG_CONTROL_STOP          },
		/* FJ_CHG_CONNECT_EVENT_HOLDER_IN	*/		{ FJ_CHG_CONNECT_STATE_BOTH,     FJ_CHG_CONTROL_HOLDER_CHARGE },
		/* FJ_CHG_CONNECT_EVENT_HOLDER_OUT	*/		{ FJ_CHG_CONNECT_STATE_NOCHANGE, FJ_CHG_CONTROL_NONE          },
	/* FJ_CHG_CONNECT_STATE_HOLDER */			} , {
		/* FJ_CHG_CONNECT_EVENT_USB_IN		*/		{ FJ_CHG_CONNECT_STATE_BOTH,     FJ_CHG_CONTROL_NONE          },
		/* FJ_CHG_CONNECT_EVENT_USB_OUT		*/		{ FJ_CHG_CONNECT_STATE_NOCHANGE, FJ_CHG_CONTROL_NONE          },
		/* FJ_CHG_CONNECT_EVENT_HOLDER_IN	*/		{ FJ_CHG_CONNECT_STATE_NOCHANGE, FJ_CHG_CONTROL_NONE          },
		/* FJ_CHG_CONNECT_EVENT_HOLDER_OUT	*/		{ FJ_CHG_CONNECT_STATE_NONE,     FJ_CHG_CONTROL_STOP          },
	/* FJ_CHG_CONNECT_STATE_BOTH */				} , {
		/* FJ_CHG_CONNECT_EVENT_USB_IN		*/		{ FJ_CHG_CONNECT_STATE_NOCHANGE, FJ_CHG_CONTROL_NONE          },
		/* FJ_CHG_CONNECT_EVENT_USB_OUT		*/		{ FJ_CHG_CONNECT_STATE_HOLDER,   FJ_CHG_CONTROL_NONE          },
		/* FJ_CHG_CONNECT_EVENT_HOLDER_IN	*/		{ FJ_CHG_CONNECT_STATE_NOCHANGE, FJ_CHG_CONTROL_NONE          },
		/* FJ_CHG_CONNECT_EVENT_HOLDER_OUT	*/		{ FJ_CHG_CONNECT_STATE_USB,      FJ_CHG_CONTROL_USB_CHARGE    },
												}
};

/**
 * fj_chg_connect_info_update
 *
 * @param   chip         : charger device info pointer
 *          connect_info : connect info pointer
 *
 * @retval  charge control type
 */
static fj_charger_ctrl_type fj_chg_connect_info_update(struct charge_drv_chip *chip,
													   fj_chg_connect_info *connect_info)
{
	const fj_chg_connect_ctrl *connect_ctrl;
	unsigned long new_source = 0;
	unsigned int current_backup;

	new_source = FJ_CHG_SOURCE(connect_info->chg_source);

	spin_lock(&fj_chg_info_lock);
	if (connect_info->max_current != FJ_CHG_CURRENT_OFF) {
		if ((new_source & FJ_CHG_SOURCE_USB_PORT) != 0) {
			fj_charger_src &= ~FJ_CHG_SOURCE_USB_PORT;
		}
		fj_charger_src |= new_source;
	} else {
		fj_charger_src &= ~new_source;
	}
	chip->charger_info.chg_source = fj_charger_src;
	spin_unlock(&fj_chg_info_lock);

	connect_ctrl = &connect_ctrl_table[chip->connect_state][connect_info->event];

	switch(connect_info->event) {
		case FJ_CHG_CONNECT_EVENT_USB_IN:
			chip->dev_usb.max_current = connect_info->max_current;
			fj_chg_start_negotiation_timer(chip, FJ_CHG_DEVICE_TYPE_USB);
			break;
		case FJ_CHG_CONNECT_EVENT_USB_OUT:
			fj_chg_terminate_usb(chip);
			fj_chg_pmic_slight_charge(chip, FJ_CHG_DEVICE_TYPE_USB);
			break;
		case FJ_CHG_CONNECT_EVENT_HOLDER_IN:
			chip->dev_holder.max_current = connect_info->max_current;
			fj_chg_start_negotiation_timer(chip, FJ_CHG_DEVICE_TYPE_HOLDER);
			enable_irq(fj_charger_oki_det_irq);
			break;
		case FJ_CHG_CONNECT_EVENT_HOLDER_OUT:
			fj_chg_terminate_holder(chip);
			fj_chg_pmic_slight_charge(chip, FJ_CHG_DEVICE_TYPE_HOLDER);
			if (connect_ctrl->next_state == FJ_CHG_CONNECT_STATE_USB) {
				current_backup = chip->dev_usb.max_current;
				fj_chg_terminate_usb(chip);
				chip->dev_usb.max_current = current_backup;
				fj_chg_pmic_slight_charge(chip, FJ_CHG_DEVICE_TYPE_USB);
				fj_chg_start_negotiation_timer(chip, FJ_CHG_DEVICE_TYPE_USB);
			}
			break;
		default:
			break;
	}

	switch(connect_ctrl->next_state) {
		case FJ_CHG_CONNECT_STATE_USB:
			chip->charger_info.chg_current = chip->dev_usb.max_current;
			chip->charge_type = chip->dev_usb.charge_type;
			fj_chg_pmic_set_preferential_device(chip, FJ_CHG_DEVICE_TYPE_USB);
			break;
		case FJ_CHG_CONNECT_STATE_HOLDER:
		case FJ_CHG_CONNECT_STATE_BOTH:
			chip->charger_info.chg_current = chip->dev_holder.max_current;
			chip->charge_type = chip->dev_holder.charge_type;
			fj_chg_pmic_set_preferential_device(chip, FJ_CHG_DEVICE_TYPE_HOLDER);
			break;
		case FJ_CHG_CONNECT_STATE_NONE:
		default:
			break;
	}

	if (connect_ctrl->next_state != FJ_CHG_CONNECT_STATE_NOCHANGE) {
		if (chip->connect_state != connect_ctrl->next_state) {
			FJ_CHARGER_INFOLOG("[%s] charger connect state %d -> %d\n",
							__func__, chip->connect_state, connect_ctrl->next_state);
			chip->connect_state = connect_ctrl->next_state;
		}
	}

	if (connect_ctrl->charge_ctrl != FJ_CHG_CONTROL_NONE) {
		fj_chg_power_supply_update_all(chip);
	}

	return connect_ctrl->charge_ctrl;
}

/**
 * fj_chg_func_charging_monitor
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_charging_monitor(struct charge_drv_chip *chip,
														  fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	fj_chg_monitor_param param = {0};
	fj_charger_err_type usb_err_type = FJ_CHG_ERR_NON;
	fj_charger_err_type holder_err_type = FJ_CHG_ERR_NON;
	fj_charger_result_type usb_result = FJ_CHG_RESULT_SUCCESS;
	fj_charger_result_type holder_result = FJ_CHG_RESULT_SUCCESS;
	union power_supply_propval prop = {0,};
	int ret = 0;
	int ibat_ma = 0;

	FJ_CHARGER_DBGLOG("[%s] in type:[%d]\n", __func__, data->monitor_info.type);

	switch (data->monitor_info.type) {
		case FJ_CHG_MONITOR_NORMAL:
			fj_chg_get_parameters(chip, &param);
			err_type = fj_chg_check_parameters(chip, &param);

			switch (err_type) {
				case FJ_CHG_ERR_NON:
					if (likely(chip->bms)) {
						ret = chip->bms->get_property(chip->bms, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
					}
					if (likely(ret == 0)) {
						ibat_ma = prop.intval / 1000;
					}
					FJ_CHARGER_INFOLOG("[%s] usb:[%d]mV dc:[%d]mV oki:[%d]mV sts:[0x%02x], current[%d]mA\n", __func__,
						(param.supply_volt/1000), (param.dc_in_volt/1000), (param.oki_adc_volt/1000), param.chg_sts, ibat_ma);

					err_type = fj_chg_recovery_control(chip, param.chg_sts);
					if (err_type != FJ_CHG_ERR_NON) {
						fj_chg_err_control(chip, FJ_CHG_ERR_OTHER_ERR);
						next_state = FJ_CHG_STATE_ERROR;
						break;
					}
					if (param.soc <= FJ_CHG_VALUE_RECHARGE_LV2) {
						fj_chg_stop_full_timer(chip);
						(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_CHARGING);
					} else if (param.soc >= FJ_CHG_VALUE_FULL_SOC_LV1) {
						fj_chg_start_full_timer1(chip);
					}
					if (param.soc >= FJ_CHG_VALUE_FULL_SOC_LV2 && fj_charge_is_battery_full(chip)) {
						FJ_CHARGER_INFOLOG("[%s] battery full.\n", __func__);
						fj_chg_stop_full_timer(chip);
						fj_chg_stop_adp_volt_monitor(chip);
						fj_chg_stop_safety_timer(chip);
						fj_chg_charge_control(chip, FJ_CHG_CONTROL_POWERPATH);
						fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_FULL);
						next_state = FJ_CHG_STATE_FULL;
					}
					break;

				default:
					fj_chg_err_control(chip, err_type);
					next_state = FJ_CHG_STATE_ERROR;
					break;
			}

			fj_chg_start_monitor(chip);
			break;

		case FJ_CHG_MONITOR_VOLTAGE:
			/* check supply voltage */
			fj_chg_get_parameters(chip, &param);

			if ((chip->charger_info.chg_source & FJ_CHG_SOURCE_HOLDER) != 0) {
				holder_result = fj_chg_watch_holder_adp_voltage(chip, &holder_err_type,
																param.dc_in_volt, param.oki_adc_volt);
			}
			if ((chip->charger_info.chg_source & FJ_CHG_SOURCE_USB_PORT) != 0) {
				usb_result = fj_chg_watch_usb_adp_voltage(chip, &usb_err_type, param.supply_volt);
			}

			if (holder_result == FJ_CHG_RESULT_FAILURE) {
				/* holder error */
				if (holder_err_type == FJ_CHG_ERR_UNDER_SUPPLY_VOLTAGE_ERR) {
					fj_chg_check_weak_adapter(chip);
					if (chip->dev_holder.charge_type == FJ_CHG_CHARGE_TYPE_QUICK) {
						holder_err_type = FJ_CHG_ERR_OTHER_ERR;
					}
				}
				fj_chg_err_control(chip, holder_err_type);
				next_state = FJ_CHG_STATE_ERROR;
			} else if (usb_result == FJ_CHG_RESULT_FAILURE) {
				/* usb error */
				if (usb_err_type == FJ_CHG_ERR_UNDER_SUPPLY_VOLTAGE_ERR) {
					fj_chg_check_weak_adapter(chip);
					if (chip->dev_usb.charge_type == FJ_CHG_CHARGE_TYPE_QUICK) {
						usb_err_type = FJ_CHG_ERR_OTHER_ERR;
					}
				}
				fj_chg_err_control(chip, usb_err_type);
				next_state = FJ_CHG_STATE_ERROR;
			} else if ((usb_result == FJ_CHG_RESULT_VOLTAGE_ERROR) ||
					   (holder_result == FJ_CHG_RESULT_VOLTAGE_ERROR)) {
				/* voltage error */
				if ((holder_err_type == FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR) ||
					(usb_err_type == FJ_CHG_ERR_OVER_SUPPLY_VOLTAGE_ERR)) {
					fj_chg_charge_control(chip, FJ_CHG_CONTROL_STOP);
				}
				fj_chg_start_adp_volt_monitor(chip, FJ_CHG_SET_TIMER_750MS);
			} else if ((usb_result == FJ_CHG_RESULT_RETRY) ||
					   (holder_result == FJ_CHG_RESULT_RETRY)) {
				fj_chg_event_req(chip, FJ_CHG_EVENT_PARAM_UPDATE, NULL);
				fj_chg_start_adp_volt_monitor(chip, FJ_CHG_SET_TIMER_1S);
			} else {
				/* success */
				if ((param.chg_sts & 0x06) == 0x00) {
					fj_chg_charge_control(chip, FJ_CHG_CONTROL_RECHARGE);
				}
				fj_chg_start_adp_volt_monitor(chip, FJ_CHG_SET_TIMER_1S);
			}
			break;

		default:
			break;
	}

	return next_state;
}

/**
 * fj_chg_func_full_monitor
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_full_monitor(struct charge_drv_chip *chip,
													  fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	fj_chg_monitor_param param = {0};
	int ret = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	if (data->monitor_info.type == FJ_CHG_MONITOR_NORMAL) {
		fj_chg_get_parameters(chip, &param);
		err_type = fj_chg_check_parameters(chip, &param);

		switch (err_type) {
			case FJ_CHG_ERR_NON:
				if (param.soc <= FJ_CHG_VALUE_RECHARGE_LV1) {
					ret = fj_chg_charge_control(chip, FJ_CHG_CONTROL_RECHARGE);
					if (ret == 0) {
						fj_chg_start_safety_timer(chip);
						fj_chg_start_adp_volt_monitor(chip, FJ_CHG_SET_TIMER_1S);
						/* keep POWER_SUPPLY_STATUS_FULL status */
						next_state = FJ_CHG_STATE_CHARGING;
					} else {
						fj_chg_err_control(chip, FJ_CHG_ERR_OTHER_ERR);
						next_state = FJ_CHG_STATE_ERROR;
					}
				}
				break;

			default:
				fj_chg_err_control(chip, err_type);
				next_state = FJ_CHG_STATE_ERROR;
				break;
		}

		fj_chg_start_monitor(chip);
	}

	return next_state;
}

/**
 * fj_chg_func_error_monitor
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_error_monitor(struct charge_drv_chip *chip, fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	fj_chg_monitor_param param = {0};
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	int ret = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	if (data->monitor_info.type == FJ_CHG_MONITOR_NORMAL) {
		fj_chg_get_parameters(chip, &param);
		err_type = fj_chg_check_parameters(chip, &param);

		switch (err_type) {
			case FJ_CHG_ERR_NON:
				ret = fj_chg_charge_control(chip, FJ_CHG_CONTROL_RECHARGE);
				if (likely(ret == 0)) {
					fj_chg_start_safety_timer(chip);
					fj_chg_start_adp_volt_monitor(chip, FJ_CHG_SET_TIMER_1S);
					(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_CHARGING);
					(void)fj_chg_batt_set_health(chip, POWER_SUPPLY_HEALTH_GOOD);
					next_state = FJ_CHG_STATE_CHARGING;
				}
				break;

			default:
				fj_chg_err_control(chip, err_type);
				break;
		}
	
		fj_chg_start_monitor(chip);
	}

	return next_state;
}

/**
 * fj_chg_func_idle_info_notice
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_idle_info_notice(struct charge_drv_chip *chip, fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	switch (data->notice_info.type) {
		case FJ_CHG_NOTICE_OVP_NOTIFY_ERROR:
		case FJ_CHG_NOTICE_USBHOT_NOTIFY_ERROR:
		
			if (data->notice_info.type == FJ_CHG_NOTICE_USBHOT_NOTIFY_ERROR) {
				if (data->notice_info.usb_hot_detect == FJ_CHG_ERROR_NONE) {
					chip->safety_err_info.usb_port_failure = false;
				} else {
					chip->safety_err_info.usb_port_failure = true;
				}
			}

			if (data->notice_info.type == FJ_CHG_NOTICE_OVP_NOTIFY_ERROR) {
				if (data->notice_info.ovp_detect == FJ_CHG_ERROR_NONE) {
					chip->safety_err_info.ovp_detect = false;
				} else {
					chip->safety_err_info.ovp_detect = true;
				}
			}

			if (chip->safety_err_info.usb_port_failure == true) {
				(void)fj_chg_batt_set_health(chip, POWER_SUPPLY_HEALTH_USB_HOT);
				(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_NOT_CHARGING);
			} else if (chip->safety_err_info.ovp_detect == true) {
				(void)fj_chg_batt_set_health(chip, POWER_SUPPLY_HEALTH_OVER_SUPPLY_VOLTAGE);
				(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_NOT_CHARGING);
			} else {
				(void)fj_chg_batt_set_health(chip, POWER_SUPPLY_HEALTH_GOOD);
				(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_DISCHARGING);
			}
			break;

		case FJ_CHG_NOTICE_CHG_ENABLE:
			chip->max_ilimit_value = data->notice_info.ilimit;
			break;

		case FJ_CHG_NOTICE_OKI_DET:
			fj_chg_oki_det_control(chip, data->notice_info.oki_det);
			break;

		case FJ_CHG_NOTICE_CHANGE_MODE:
			chip->heat_charge_mode = data->notice_info.charge_mode;
			break;

		default:
			break;
	}

	return next_state;
}

/**
 * fj_chg_func_info_notice
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_info_notice(struct charge_drv_chip *chip,
													 fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;

	FJ_CHARGER_DBGLOG("[%s] in type = %d\n", __func__, data->notice_info.type);

	switch (data->notice_info.type) {
		case FJ_CHG_NOTICE_OVP_NOTIFY_ERROR:
		case FJ_CHG_NOTICE_USBHOT_NOTIFY_ERROR:
			fj_chg_notify_error_control(chip, data->notice_info);
			break;

		case FJ_CHG_NOTICE_CHG_ENABLE:
			fj_chg_charge_enable_control(chip, data->notice_info.ilimit);
			break;

		case FJ_CHG_NOTICE_CHANGE_MODE:
			fj_chg_change_mode_control(chip, data->notice_info.charge_mode);
			break;

		case FJ_CHG_NOTICE_OKI_DET:
			fj_chg_oki_det_control(chip, data->notice_info.oki_det);
			break;

		case FJ_CHG_NOTICE_CHANGE_CURRENT:
			fj_chg_change_current_control(chip, &data->notice_info.change_info);
			break;

		default:
			break;
	}

	return next_state;
}

/**
 * fj_chg_func_idle_control
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_idle_control(struct charge_drv_chip *chip,
													  fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	fj_charger_ctrl_type charge_ctrl;
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	fj_chg_monitor_param param = {0};
	int ret;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	do {
		if (chip->charge_mode == CHARGE_MODE_NOT_CHARGE) {
			fj_chg_charge_control(chip, FJ_CHG_CONTROL_STOP);
			break;
		}

		charge_ctrl = fj_chg_connect_info_update(chip, &data->connect_info);

		switch (charge_ctrl) {
			case FJ_CHG_CONTROL_STOP:
				(void)fj_chg_charge_control(chip, FJ_CHG_CONTROL_STOP);
				fj_chg_terminate(chip, false);
				break;

			case FJ_CHG_CONTROL_USB_CHARGE:
			case FJ_CHG_CONTROL_HOLDER_CHARGE:
				fj_chg_get_parameters(chip, &param);
				err_type = fj_chg_check_parameters(chip, &param);
				if (err_type == FJ_CHG_ERR_NON) {
					ret = fj_chg_charge_control(chip, FJ_CHG_CONTROL_PRECHARGE);
					if (likely(ret == 0)) {
						(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_CHARGING);
						fj_chg_start_safety_timer(chip);
						next_state = FJ_CHG_STATE_CHARGING;
					} else {
						fj_chg_err_control(chip, FJ_CHG_ERR_OTHER_ERR);
						fj_chg_start_monitor(chip);
						next_state = FJ_CHG_STATE_ERROR;
					}
				} else {
					fj_chg_err_control(chip, err_type);
					fj_chg_start_monitor(chip);
					next_state = FJ_CHG_STATE_ERROR;
				}

				fj_chg_start_watchdog_timer(chip);
				break;

			default:
				break;
		}
	} while (0);

	return next_state;
}

/**
 * fj_chg_func_charging_control
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_charging_control(struct charge_drv_chip *chip,
														  fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	fj_charger_ctrl_type charge_ctrl;
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	fj_chg_monitor_param param = {0};
	int ret = 0;

	charge_ctrl = fj_chg_connect_info_update(chip, &data->connect_info);

	switch (charge_ctrl) {
		case FJ_CHG_CONTROL_STOP:
			(void)fj_chg_charge_control(chip, FJ_CHG_CONTROL_STOP);
			fj_chg_terminate(chip, false);
			next_state = FJ_CHG_STATE_IDLE;
			break;

		case FJ_CHG_CONTROL_USB_CHARGE:
		case FJ_CHG_CONTROL_HOLDER_CHARGE:
			fj_chg_stop_adp_volt_monitor(chip);
			fj_chg_stop_monitor(chip);
			fj_chg_stop_full_timer(chip);
			fj_chg_stop_safety_timer(chip);

			fj_chg_get_parameters(chip, &param);
			err_type = fj_chg_check_parameters(chip, &param);

			if (err_type == FJ_CHG_ERR_NON) {
				ret = fj_chg_charge_control(chip, FJ_CHG_CONTROL_PRECHARGE);
				if (likely(ret == 0)) {
					(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_CHARGING);
					fj_chg_start_safety_timer(chip);
				} else {
					fj_chg_err_control(chip, FJ_CHG_ERR_OTHER_ERR);
					fj_chg_start_monitor(chip);
					next_state = FJ_CHG_STATE_ERROR;
				}
			} else {
				fj_chg_err_control(chip, err_type);
				fj_chg_start_monitor(chip);
				next_state = FJ_CHG_STATE_ERROR;
			}
			break;

		default:
			break;
	}

	return next_state;
}

/**
 * fj_chg_func_connect_control
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_connect_control(struct charge_drv_chip *chip,
														 fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	fj_charger_ctrl_type charge_ctrl;
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	fj_chg_monitor_param param = {0};
	int ret = 0;

	charge_ctrl = fj_chg_connect_info_update(chip, &data->connect_info);

	switch (charge_ctrl) {
		case FJ_CHG_CONTROL_STOP:
			(void)fj_chg_charge_control(chip, FJ_CHG_CONTROL_STOP);
			fj_chg_terminate(chip, false);
			next_state = FJ_CHG_STATE_IDLE;
			break;

		case FJ_CHG_CONTROL_USB_CHARGE:
		case FJ_CHG_CONTROL_HOLDER_CHARGE:
			fj_chg_stop_monitor(chip);

			fj_chg_get_parameters(chip, &param);
			err_type = fj_chg_check_parameters(chip, &param);
			if (err_type == FJ_CHG_ERR_NON) {
				ret = fj_chg_charge_control(chip, FJ_CHG_CONTROL_POWERPATH);
				if (unlikely(ret != 0)) {
					fj_chg_err_control(chip, FJ_CHG_ERR_OTHER_ERR);
					next_state = FJ_CHG_STATE_ERROR;
				}
			} else {
				fj_chg_err_control(chip, err_type);
				next_state = FJ_CHG_STATE_ERROR;
			}

			fj_chg_start_monitor(chip);
			break;

		default:
			break;
	}

	return next_state;
}

/**
 * fj_chg_func_charging_timer_expire
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_charging_timer_expire(struct charge_drv_chip *chip,
															   fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;
	fj_charger_result_type result;
    union power_supply_propval prop = {0,};
	int ret = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	switch (data->timer_info.type) {
		case FJ_CHG_TIMER_FULL_1:
			FJ_CHARGER_DBGLOG("[%s] additional charge\n", __func__);
			(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_FULL);
			fj_chg_start_full_timer2(chip);
			break;

		case FJ_CHG_TIMER_FULL_2:
			FJ_CHARGER_INFOLOG("[%s] battery full.\n", __func__);
			fj_chg_stop_adp_volt_monitor(chip);
			fj_chg_stop_safety_timer(chip);
			fj_chg_charge_control(chip, FJ_CHG_CONTROL_POWERPATH);
			(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_FULL);
			next_state = FJ_CHG_STATE_FULL;
			break;

		case FJ_CHG_TIMER_PROTECTION:
			ret = fj_chg_charge_control(chip, FJ_CHG_CONTROL_POWERPATH);
			if (unlikely(ret != 0)) {
				chip->err_info.safety_timer_expire = true;
				fj_chg_err_control(chip, FJ_CHG_ERR_EXPIRE_TIME_OUT);
				next_state = FJ_CHG_STATE_ERROR;
				break;
			}

			msleep(500);

			ret = chip->fg_adc->get_property(chip->fg_adc, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
			if (unlikely(ret != 0)) {
				chip->err_info.safety_timer_expire = true;
				fj_chg_err_control(chip, FJ_CHG_ERR_EXPIRE_TIME_OUT);
				next_state = FJ_CHG_STATE_ERROR;
				break;
			}

			if (prop.intval < FJ_CHG_VALUE_UNSPEC_BATT_VOLTAGE) {
				chip->err_info.safety_timer_expire = true;
				fj_chg_err_control(chip, FJ_CHG_ERR_EXPIRE_TIME_OUT);
				next_state = FJ_CHG_STATE_ERROR;
				break;
			}

			ret = fj_chg_charge_control(chip, FJ_CHG_CONTROL_RECHARGE);
			if (unlikely(ret != 0)) {
				chip->err_info.safety_timer_expire = true;
				fj_chg_err_control(chip, FJ_CHG_ERR_EXPIRE_TIME_OUT);
				next_state = FJ_CHG_STATE_ERROR;
				break;
			}

			fj_chg_start_safety_timer(chip);
			break;

		case FJ_CHG_TIMER_VOLTAGE_CHECK:
			result = fj_chg_charge_type_judgement(chip, data->timer_info.device, &err_type);
			switch (result) {
				case FJ_CHG_RESULT_SUCCESS:
					if (FJ_CHG_IS_HOLDER_CHARGE(chip) &&
						data->timer_info.device == FJ_CHG_DEVICE_TYPE_USB) {
						break;
					}
					if (data->timer_info.device == FJ_CHG_DEVICE_TYPE_HOLDER) {
						chip->charge_type = chip->dev_holder.charge_type;
					} else {
						chip->charge_type = chip->dev_usb.charge_type;
					}
					ret = fj_chg_charge_control(chip, FJ_CHG_CONTROL_RECHARGE);
					if (likely(ret == 0)) {
						fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_CHARGING);
						fj_chg_start_adp_volt_monitor(chip, FJ_CHG_SET_TIMER_1S);
					} else {
						fj_chg_err_control(chip, FJ_CHG_ERR_OTHER_ERR);
						next_state = FJ_CHG_STATE_ERROR;
					}
					fj_chg_start_monitor(chip);
					break;

				case FJ_CHG_RESULT_CONTINUE:
					fj_chg_start_voltage_check_timer(chip, data->timer_info.device);
					break;

				case FJ_CHG_RESULT_FAILURE:
					if (FJ_CHG_IS_HOLDER_CHARGE(chip) &&
						data->timer_info.device == FJ_CHG_DEVICE_TYPE_USB) {
						fj_chg_start_voltage_check_timer(chip, data->timer_info.device);
						break;
					}
					(void)fj_chg_charge_control(chip, FJ_CHG_CONTROL_STOP);
					(void)fj_chg_batt_set_status(chip, POWER_SUPPLY_STATUS_NOT_CHARGING);
					fj_chg_start_voltage_check_timer(chip, data->timer_info.device);
					break;

				case FJ_CHG_RESULT_OVER:
					fj_chg_err_control(chip, err_type);
					fj_chg_start_monitor(chip);
					next_state = FJ_CHG_STATE_ERROR;
					break;

				default:
					break;
			}
			break;

		case FJ_CHG_TIMER_NEGOTIATION:
			if (data->timer_info.device == FJ_CHG_DEVICE_TYPE_HOLDER) {
				result = fj_chg_nego_holder_control(chip);
			} else {
				result = fj_chg_nego_usb_control(chip);
			}
			switch (result) {
				case FJ_CHG_RESULT_SUCCESS:
					fj_chg_start_voltage_check_timer(chip, data->timer_info.device);
					break;

				case FJ_CHG_RESULT_FAILURE:
					fj_chg_err_control(chip, FJ_CHG_ERR_OTHER_ERR);
					fj_chg_start_monitor(chip);
					next_state = FJ_CHG_STATE_ERROR;
					break;

				case FJ_CHG_RESULT_CONTINUE:
					fj_chg_start_negotiation_timer(chip, data->timer_info.device);
					break;

				default:
					break;
			}
			break;

		default:
			break;
	}

	return next_state;
}

/**
 * fj_chg_func_timer_expire
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_timer_expire(struct charge_drv_chip *chip,
													  fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	fj_charger_result_type result;
	fj_charger_err_type err_type = FJ_CHG_ERR_NON;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	switch (data->timer_info.type) {
		case FJ_CHG_TIMER_VOLTAGE_CHECK:
			result = fj_chg_charge_type_judgement(chip, data->timer_info.device, &err_type);
			switch (result) {
				case FJ_CHG_RESULT_SUCCESS:
					if (FJ_CHG_IS_HOLDER_CHARGE(chip) &&
						data->timer_info.device == FJ_CHG_DEVICE_TYPE_USB) {
						break;
					}
					if (data->timer_info.device == FJ_CHG_DEVICE_TYPE_HOLDER) {
						chip->charge_type = chip->dev_holder.charge_type;
					} else {
						chip->charge_type = chip->dev_usb.charge_type;
					}
					fj_chg_start_monitor(chip);
					break;

				case FJ_CHG_RESULT_CONTINUE:
				case FJ_CHG_RESULT_FAILURE:
					fj_chg_start_voltage_check_timer(chip, data->timer_info.device);
					break;

				case FJ_CHG_RESULT_OVER:
					fj_chg_err_control(chip, err_type);
					fj_chg_start_monitor(chip);
					next_state = FJ_CHG_STATE_ERROR;
					break;

				default:
					break;
			}
			break;

		case FJ_CHG_TIMER_NEGOTIATION:
			if (data->timer_info.device == FJ_CHG_DEVICE_TYPE_HOLDER) {
				result = fj_chg_nego_holder_control(chip);
			} else {
				result = fj_chg_nego_usb_control(chip);
			}
			switch (result) {
				case FJ_CHG_RESULT_SUCCESS:
					fj_chg_start_voltage_check_timer(chip, data->timer_info.device);
					break;

				case FJ_CHG_RESULT_FAILURE:
					fj_chg_err_control(chip, FJ_CHG_ERR_OTHER_ERR);
					fj_chg_start_monitor(chip);
					next_state = FJ_CHG_STATE_ERROR;
					break;

				case FJ_CHG_RESULT_CONTINUE:
					fj_chg_start_negotiation_timer(chip, data->timer_info.device);
					break;

				default:
					break;
			}
			break;

		default:
			break;
	}

	return next_state;
}

/**
 * fj_chg_func_charging_update
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_charging_update(struct charge_drv_chip *chip,
														 fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	int ret = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	do {
		ret = fj_chg_pmic_current_setting(chip);
		if (unlikely(ret != 0)) {
			FJ_CHARGER_ERRLOG("[%s] Parameter Update Failed. ret:[%d]\n", __func__, ret);
			break;
		}

		if (chip->charge_type == FJ_CHG_CHARGE_TYPE_NORMAL) {
			if (FJ_CHG_IS_HOLDER_CHARGE(chip)) {
				ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_HOLDER, FJ_CHG_AICL_TYPE_RERUN);
			} else {
				ret = fj_chg_pmic_aicl_setting(chip, FJ_CHG_DEVICE_TYPE_USB, FJ_CHG_AICL_TYPE_RERUN);
			}
			if (unlikely(ret != 0)) {
				FJ_CHARGER_ERRLOG("[%s] Parameter Update Failed. ret:[%d]\n", __func__, ret);
				break;
			}
		}
	} while (0);

	return next_state;
}

/**
 * fj_chg_func_parameter_update
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_parameter_update(struct charge_drv_chip *chip,
														  fj_chg_evt_data *data)
{
	fj_charger_state_type next_state = FJ_CHG_STATE_NOCHANGE;
	int ret = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	ret = fj_chg_pmic_current_setting(chip);
	if (unlikely(ret != 0)) {
		FJ_CHARGER_ERRLOG("[%s] Parameter Update Failed. ret:[%d]\n", __func__, ret);
	}

	return next_state;
}

/**
 * fj_chg_func_null
 *
 * @param   chip : charger device info pointer
 *          data : event data queue pointer
 *
 * @retval  next state
 */
static fj_charger_state_type fj_chg_func_null(struct charge_drv_chip *chip, fj_chg_evt_data *data)
{
	return FJ_CHG_STATE_NOCHANGE;
}

static const fj_charge_func fj_chg_state_func_table[FJ_CHG_STATE_NUM][FJ_CHG_EVENT_NUM] = {
	/* FJ_CHG_STATE_IDLE */					{
		/* FJ_CHG_EVENT_MONITOR_PARAM		*/		fj_chg_func_null,
		/* FJ_CHG_EVENT_CHARGE_INFO_NOTICE	*/		fj_chg_func_idle_info_notice,
		/* FJ_CHG_EVENT_CHARGE_CONTROL		*/		fj_chg_func_idle_control,
		/* FJ_CHG_EVENT_CHARGE_TIMER		*/		fj_chg_func_null,
		/* FJ_CHG_EVENT_PARAM_UPDATE		*/		fj_chg_func_null,
	/* FJ_CHG_STATE_CHARGING */				} , {
		/* FJ_CHG_EVENT_MONITOR_PARAM		*/		fj_chg_func_charging_monitor,
		/* FJ_CHG_EVENT_CHARGE_INFO_NOTICE	*/		fj_chg_func_info_notice,
		/* FJ_CHG_EVENT_CHARGE_CONTROL		*/		fj_chg_func_charging_control,
		/* FJ_CHG_EVENT_CHARGE_TIMER		*/		fj_chg_func_charging_timer_expire,
		/* FJ_CHG_EVENT_PARAM_UPDATE		*/		fj_chg_func_charging_update,
	/* FJ_CHG_STATE_FULL */					} , {
		/* FJ_CHG_EVENT_MONITOR_PARAM		*/		fj_chg_func_full_monitor,
		/* FJ_CHG_EVENT_CHARGE_INFO_NOTICE	*/		fj_chg_func_info_notice,
		/* FJ_CHG_EVENT_CHARGE_CONTROL		*/		fj_chg_func_connect_control,
		/* FJ_CHG_EVENT_CHARGE_TIMER		*/		fj_chg_func_timer_expire,
		/* FJ_CHG_EVENT_PARAM_UPDATE		*/		fj_chg_func_parameter_update,
	/* FJ_CHG_STATE_ERROR */				} , {
		/* FJ_CHG_EVENT_MONITOR_PARAM		*/		fj_chg_func_error_monitor,
		/* FJ_CHG_EVENT_CHARGE_INFO_NOTICE	*/		fj_chg_func_info_notice,
		/* FJ_CHG_EVENT_CHARGE_CONTROL		*/		fj_chg_func_connect_control,
		/* FJ_CHG_EVENT_CHARGE_TIMER		*/		fj_chg_func_timer_expire,
		/* FJ_CHG_EVENT_PARAM_UPDATE		*/		fj_chg_func_parameter_update,
											}
};

/**
 * fj_chg_drv_thread - charger thread main function
 *
 * @param ptr : charger device
 */
int fj_chg_drv_thread(void *ptr)
{
	struct charge_drv_chip *chip = ptr;
	fj_charger_state_type new_state = FJ_CHG_STATE_NOCHANGE;
	fj_chg_evt_queue *evt_queue_p = NULL;
	const fj_charge_func *functbl;

	wake_lock_init(&fj_chg_charging_wake_lock, WAKE_LOCK_SUSPEND, "fj_charging");

	do {
		wait_event(chip->chg_wq,
				  (!list_empty(&fj_chg_evt_list_exec) || kthread_should_stop()));
		if (kthread_should_stop()) {
			break;
		}

		evt_queue_p = fj_chg_queue_get(&fj_chg_evt_list_exec);
		if (evt_queue_p == NULL) {
			continue;
		}

		if ((chip->charger_info.state >= FJ_CHG_STATE_NUM) ||
			(evt_queue_p->event >= FJ_CHG_EVENT_NUM))    {
			FJ_CHARGER_ERRLOG("[%s] thread Error:sts[%d] evt[%d] Fatal\n",
						__func__, chip->charger_info.state, evt_queue_p->event);
			emergency_restart();
		}

		if (chip->charger_info.state == FJ_CHG_STATE_IDLE) {
			wake_lock(&fj_chg_charging_wake_lock);
		}

		FJ_CHARGER_DBGLOG("[%s] charge_state = %d event = %d\n",
									__func__, chip->charger_info.state, evt_queue_p->event);

		functbl = &fj_chg_state_func_table[chip->charger_info.state][evt_queue_p->event];
		new_state = (*functbl)(chip, &evt_queue_p->data);

		if (new_state != FJ_CHG_STATE_NOCHANGE) {
			FJ_CHARGER_INFOLOG("[%s] event:%d old state:%d -> new state:%d\n",
					 __func__, evt_queue_p->event, chip->charger_info.state, new_state);
			chip->charger_info.state = new_state;
		}

		fj_chg_queue_put(evt_queue_p, &fj_chg_evt_list_free);

		if (chip->charger_info.state == FJ_CHG_STATE_IDLE) {
			wake_unlock(&fj_chg_charging_wake_lock);
		}
	} while (1);

	wake_lock_destroy(&fj_chg_charging_wake_lock);

	return 0;
}

/**
 * fj_chg_drv_source_req
 *
 * @param   source :
 *          mA     :
 */
void fj_chg_drv_source_req(int source, unsigned int mA)
{
	struct charge_drv_chip *chip = the_chip;
	unsigned long source_work = 0;
	fj_chg_evt_data evt_data;
	static char chg_src_str[CHG_SOURCE_NUM][8] = {
		"HOLDER", "USB", "AC", "MHL",
	};

    FJ_CHARGER_RECLOG("[%s] %s mA=%d\n", __func__, chg_src_str[source], mA);

	wake_lock_timeout(&fj_chg_timer_wake_lock, (FJ_CHG_WAKELOCK_TIMER * HZ));

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	source_work = FJ_CHG_SOURCE(source);

	if (mA != 0) {
		if (source_work & FJ_CHG_SOURCE_HOLDER) {
			evt_data.connect_info.event = FJ_CHG_CONNECT_EVENT_HOLDER_IN;
		} else {
			evt_data.connect_info.event = FJ_CHG_CONNECT_EVENT_USB_IN;
		}
	} else {
		if (source_work & FJ_CHG_SOURCE_HOLDER) {
			evt_data.connect_info.event = FJ_CHG_CONNECT_EVENT_HOLDER_OUT;
		} else {
			evt_data.connect_info.event = FJ_CHG_CONNECT_EVENT_USB_OUT;
		}
	}

	evt_data.connect_info.chg_source = source;
	evt_data.connect_info.max_current = mA;

	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_CTRL, &evt_data);
}
EXPORT_SYMBOL_GPL(fj_chg_drv_source_req);

/**
 * fj_chg_drv_current_req
 *
 * @param   source :
 *          mA     :
 */
void fj_chg_drv_current_req(int source, unsigned int mA)
{
	struct charge_drv_chip *chip = the_chip;
	unsigned long source_work = 0;
	fj_chg_evt_data evt_data;

    FJ_CHARGER_RECLOG("[%s] %d mA=%d\n", __func__, source, mA);

	wake_lock_timeout(&fj_chg_timer_wake_lock, (FJ_CHG_WAKELOCK_TIMER * HZ));

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	source_work = FJ_CHG_SOURCE(source);

	if (source_work & FJ_CHG_SOURCE_HOLDER) {
		evt_data.notice_info.change_info.device = FJ_CHG_DEVICE_TYPE_HOLDER;
	} else {
		evt_data.notice_info.change_info.device = FJ_CHG_DEVICE_TYPE_USB;
	}

	evt_data.notice_info.type = FJ_CHG_NOTICE_CHANGE_CURRENT;
	evt_data.notice_info.change_info.max_current = mA;

	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);
}
EXPORT_SYMBOL_GPL(fj_chg_drv_current_req);

/**
 * fj_chg_drv_notify_error
 *
 * @param   chg_err : error type
 */
void fj_chg_drv_notify_error(fj_chg_err_type chg_err)
{
	struct charge_drv_chip *chip = the_chip;
	fj_chg_evt_data evt_data;

	FJ_CHARGER_INFOLOG("[%s] in, err=%d\n", __func__, chg_err);

	wake_lock_timeout(&fj_chg_timer_wake_lock, (FJ_CHG_WAKELOCK_TIMER * HZ));

	memset(&evt_data, 0, sizeof(fj_chg_evt_data));

	switch (chg_err) {
		case FJ_CHG_ERROR_OVP_NONE:
			evt_data.notice_info.type = FJ_CHG_NOTICE_OVP_NOTIFY_ERROR;
			evt_data.notice_info.ovp_detect = FJ_CHG_ERROR_NONE;
			fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);
			break;
		case FJ_CHG_ERROR_OVP_DETECT:
			evt_data.notice_info.type = FJ_CHG_NOTICE_OVP_NOTIFY_ERROR;
			evt_data.notice_info.ovp_detect = FJ_CHG_ERROR_DETECT;
			fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);
			break;
		case FJ_CHG_ERROR_USB_HOT_NONE:
			evt_data.notice_info.type = FJ_CHG_NOTICE_USBHOT_NOTIFY_ERROR;
			evt_data.notice_info.usb_hot_detect = FJ_CHG_ERROR_NONE;
			fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);
			break;
		case FJ_CHG_ERROR_USB_HOT_DETECT:
			evt_data.notice_info.type = FJ_CHG_NOTICE_USBHOT_NOTIFY_ERROR;
			evt_data.notice_info.usb_hot_detect = FJ_CHG_ERROR_DETECT;
			fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);
			break;
		default:
			FJ_CHARGER_ERRLOG("[%s] Error Unknown type %d\n", __func__, chg_err);
			break;
	}
}
EXPORT_SYMBOL_GPL(fj_chg_drv_notify_error);

static int set_chg_voltage(const char *val, struct kernel_param *kp)
{
	int result = 0;
	int voltage = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	result = param_set_int(val, kp);
	if (result) {
		FJ_CHARGER_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	FJ_CHARGER_INFOLOG("[%s] set chg_voltage param to %d\n",__func__, mc_chg_voltage);
	voltage = mc_chg_voltage;

	(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USBIN_CHGR_CFG,
								0x07, 0x02);

	(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DCIN_CHGR_CFG,
								0x07, 0x02);

	(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_ENUM_TIMER_STOP,
								0x01, 0x01);

	(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_CHGR_FV_CFG,
								0x3F, voltage);

	return result;
}
module_param_call(mc_chg_voltage, set_chg_voltage, param_get_uint, &mc_chg_voltage, 0644);

/**
 * set_chg_iusb_max
 *
 * @param   val :
 *          kp  :
 *
 * @retval  Processing result
 */
static int set_chg_iusb_max(const char *val, struct kernel_param *kp)
{
	int result = 0;
	int max_current = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	result = param_set_int(val, kp);
	if (result) {
		FJ_CHARGER_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	FJ_CHARGER_INFOLOG("[%s] set chg_iusb_max param to %d\n",__func__, mc_chg_iusb_max);
	max_current = mc_chg_iusb_max;

	(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USBIN_IL_CFG,
								0x1F, max_current);

	return result;
}
module_param_call(mc_chg_iusb_max, set_chg_iusb_max, param_get_uint, &mc_chg_iusb_max, 0644);

/**
 * set_chg_idc_max
 *
 * @param   val :
 *          kp  :
 *
 * @retval  Processing result
 */
static int set_chg_idc_max(const char *val, struct kernel_param *kp)
{
	int result = 0;
	int max_current = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	result = param_set_int(val, kp);
	if (result) {
		FJ_CHARGER_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	FJ_CHARGER_INFOLOG("[%s] set chg_idc_max param to %d\n",__func__, mc_chg_idc_max);
	max_current = mc_chg_idc_max;

	(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DCIN_IL_CFG,
								0x1F, max_current);

	return result;
}
module_param_call(mc_chg_idc_max, set_chg_idc_max, param_get_uint, &mc_chg_idc_max, 0644);

/**
 * set_chg_current
 *
 * @param   val :
 *          kp  :
 *
 * @retval  Processing result
 */
static int set_chg_current(const char *val, struct kernel_param *kp)
{
	int result = 0;
	int max_current = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	result = param_set_int(val, kp);
	if (result) {
		FJ_CHARGER_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	FJ_CHARGER_INFOLOG("[%s] set chg_current param to %d\n",__func__, mc_chg_current);
	max_current = mc_chg_current;

	(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_CHGR_FCC_CFG,
								0x1F, max_current);

	return result;
}
module_param_call(mc_chg_current, set_chg_current, param_get_uint, &mc_chg_current, 0644);

/**
 * set_charger_enable
 *
 * @param   val :
 *          kp  :
 *
 * @retval  Processing result
 */
static int set_charger_enable(const char *val, struct kernel_param *kp)
{
	int result = 0;
	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	result = param_set_int(val, kp);
	if (result) {
		FJ_CHARGER_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	FJ_CHARGER_INFOLOG("[%s] set set_charger_enable param to %d\n",__func__, mc_charger_enable);

	switch(mc_charger_enable) {
		case 0: /* charge OFF */
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG,
										0x02, 0x02);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x10, 0x10);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x08, 0x08);
			break;

		case 1: /* charge ON */
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x04, 0x04);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x01, 0x01);
			mdelay(20);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2,
										0x08, 0x08);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2,
										0x04, 0x04);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8,
										0x80, 0x80);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8,
										0x40, 0x40);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_BAT_IF_VBL_SEL_CFG,
										0x02, 0x02);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG2,
										0x01, 0x01);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG,
										0x04, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1,
										0x04, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG,
										0x04, 0x04);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1,
										0x04, 0x04);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_MISC_WD_CFG,
										0x01, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x08, 0x00);
			mdelay(20);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x10, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CFG,
										0x40, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG,
										0x02, 0x00);
			break;

		case 2: /* power path */
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x04, 0x04);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x01, 0x01);
			mdelay(20);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2,
										0x08, 0x08);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2,
										0x04, 0x04);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8,
										0x80, 0x80);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8,
										0x40, 0x40);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_BAT_IF_VBL_SEL_CFG,
										0x02, 0x02);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG2,
										0x01, 0x01);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG,
										0x04, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1,
										0x04, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG,
										0x04, 0x04);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1,
										0x04, 0x04);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_MISC_WD_CFG,
										0x01, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x08, 0x00);
			mdelay(20);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,
										0x10, 0x00);
			(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG,
										0x02, 0x02);
			break;

		default:
			FJ_CHARGER_ERRLOG("[%s] parameter error:%d\n", __func__, mc_charger_enable);
			break;
	}

	return result;
}
module_param_call(mc_charger_enable, set_charger_enable, param_get_uint, &mc_charger_enable, 0644);

static int get_batt_present(char *buffer, struct kernel_param *kp)
{
	int result = 0;
	u8 param = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	(void)qpnp_charger_write(FJ_CHG_CMD_SMBCHG_BAT_IF_BM_CFG,
								0x01, 0x01);


	(void)qpnp_charger_read(FJ_CHG_CMD_SMBCHG_BAT_IF_BAT_PRES_STATUS,
								&param);

	if (param & 0x80) {
		result = sprintf(buffer, "%d", 1);
	} else {
		result = sprintf(buffer, "%d", 0);
	}

	return result;
}
module_param_call(batt_present, NULL, get_batt_present, NULL, 0444);

/* for PF */
static int set_fj_chg_enable(const char *val, struct kernel_param *kp)
{
	struct charge_drv_chip *chip = the_chip;
	fj_chg_evt_data evt_data;
	int result = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	result = param_set_int(val, kp);
	if (result) {
		FJ_CHARGER_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	FJ_CHARGER_INFOLOG("[%s] set chg_enable param to %d\n",__func__, chg_enable);
	wake_lock_timeout(&fj_chg_timer_wake_lock, (FJ_CHG_WAKELOCK_TIMER * HZ));
	memset(&evt_data, 0, sizeof(fj_chg_evt_data));
	evt_data.notice_info.type = FJ_CHG_NOTICE_CHG_ENABLE;

	switch (chg_enable) {
	case 0:
		evt_data.notice_info.ilimit = FJ_CHG_ILIMIT_DISABLE;
		break;
	case 1:
		evt_data.notice_info.ilimit = FJ_CHG_ILIMIT_NOT_LIMITED;
		break;
	case 2:
		evt_data.notice_info.ilimit = FJ_CHG_ILIMIT_LIMITED_900;
		break;
	case 3:
		evt_data.notice_info.ilimit = FJ_CHG_ILIMIT_LIMITED_500;
		break;
	case 4:
		evt_data.notice_info.ilimit = FJ_CHG_ILIMIT_LIMITED_300;
		break;
	default:
		evt_data.notice_info.ilimit = FJ_CHG_ILIMIT_NOT_LIMITED;
		break;
	}

	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);

	return 0;
}
module_param_call(chg_enable, set_fj_chg_enable, param_get_uint, &chg_enable, 0644);

/* for PF */
static int get_chg_state(char *buffer, struct kernel_param *kp)
{
	struct charge_drv_chip *chip = the_chip;
	int result = 0;

	if (chip->charge_type == FJ_CHG_CHARGE_TYPE_QUICK) {
		result = sprintf(buffer, "%d", 1);
	} else {
		result = sprintf(buffer, "%d", 0);
	}

	return result;
}
module_param_call(chg_state, NULL, get_chg_state, NULL, 0444);

/* for PF */
static int set_fj_chg_mode(const char *val, struct kernel_param *kp)
{
	int result = 0;
	struct charge_drv_chip *chip = the_chip;
	fj_chg_evt_data evt_data;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	result = param_set_int(val, kp);
	if (result) {
		FJ_CHARGER_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	FJ_CHARGER_INFOLOG("[%s] in mode = %d\n", __func__, chg_mode);
	wake_lock_timeout(&fj_chg_timer_wake_lock, (FJ_CHG_WAKELOCK_TIMER * HZ));
	memset(&evt_data, 0, sizeof(fj_chg_evt_data));
	evt_data.notice_info.charge_mode = chg_mode;

	evt_data.notice_info.type = FJ_CHG_NOTICE_CHANGE_MODE;
	fj_chg_event_req(chip, FJ_CHG_EVENT_CHARGE_INFO_NOTICE, &evt_data);

	return 0;
}
module_param_call(chg_mode, set_fj_chg_mode, param_get_uint, &chg_mode, 0644);

/* for debug */
static int set_charger_debug(const char *val, struct kernel_param *kp)
{
	int result = 0;
	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	result = param_set_int(val, kp);
	if (result) {
		FJ_CHARGER_ERRLOG("[%s] error setting value %d\n",__func__, result);
		fj_chg_charger_debug = 0;
		return result;
	}

	return result;
}
module_param_call(dbglog_charger, set_charger_debug, param_get_uint, &fj_chg_charger_debug, 0644);

/**
 * fj_charge_drv_probe
 *
 * @param   pdev : 
 *
 * @retval  Processing result
 */
static int fj_charge_drv_probe(struct platform_device *pdev)
{
	struct charge_drv_chip *chip;
	int ret = 0;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	chip = kzalloc(sizeof(struct charge_drv_chip), GFP_KERNEL);
	if (unlikely(chip == NULL)) {
		FJ_CHARGER_ERRLOG("[%s] No Memory Error!!\n", __func__);
		return -ENOMEM;
	}

	chip->charge_mode = fj_chg_get_charge_mode();

	init_waitqueue_head(&chip->chg_wq);

	chip->connect_state = FJ_CHG_CONNECT_STATE_NONE;
	chip->charge_type = FJ_CHG_CHARGE_TYPE_NORMAL;
	chip->max_ilimit_value = FJ_CHG_ILIMIT_NOT_LIMITED;
	chip->heat_charge_mode = FJ_CHG_CHARGE_MODE_NORMAL;
	chip->oki_det_count = 0;
	chip->consecutive_count = 0;
	chip->batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	chip->battery_low_flag = false;

	chip->dev_usb.charge_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
	chip->dev_usb.max_current = FJ_CHG_CURRENT_OFF;
	chip->dev_usb.watch_count = 0;
	chip->dev_usb.check_total = 0;
	chip->dev_usb.nego_total = 0;
	chip->dev_usb.voltage_check = false;
	memset(chip->dev_usb.check_count, 0, sizeof(chip->dev_usb.check_count));
	memset(chip->dev_usb.nego_count, 0, sizeof(chip->dev_usb.nego_count));
	INIT_DELAYED_WORK(&chip->dev_usb.nego_timer_work, fj_chg_usb_nego_timer_work);
	INIT_DELAYED_WORK(&chip->dev_usb.check_timer_work, fj_chg_usb_check_timer_work);

	chip->dev_holder.charge_type = FJ_CHG_CHARGE_TYPE_UNKNOWN;
	chip->dev_holder.max_current = FJ_CHG_CURRENT_OFF;
	chip->dev_holder.watch_count = 0;
	chip->dev_holder.check_total = 0;
	chip->dev_holder.nego_total = 0;
	chip->dev_holder.voltage_check = false;
	memset(chip->dev_holder.check_count, 0, sizeof(chip->dev_holder.check_count));
	memset(chip->dev_holder.nego_count, 0, sizeof(chip->dev_holder.nego_count));
	INIT_DELAYED_WORK(&chip->dev_holder.nego_timer_work, fj_chg_holder_nego_timer_work);
	INIT_DELAYED_WORK(&chip->dev_holder.check_timer_work, fj_chg_holder_check_timer_work);

	memset(&chip->err_info, 0, sizeof(fj_chg_err_info));

	chip->batt_temp_hot_limit = FJ_CHG_VALUE_BATT_TEMP_START_LIMIT;
	chip->batt_temp_warm_limit = FJ_CHG_VALUE_BATT_TEMP_WARM_LIMIT;
	chip->batt_temp_cold_limit = FJ_CHG_VALUE_BATT_TEMP_COLD_LIMIT;
	chip->batt_temp_cool_limit = FJ_CHG_VALUE_BATT_TEMP_COOL_LIMIT;

	INIT_DELAYED_WORK(&chip->charge_monitor_work, fj_chg_monitor_work);
	INIT_DELAYED_WORK(&chip->full_timer_work1, fj_chg_full_timer_work1);
	INIT_DELAYED_WORK(&chip->full_timer_work2, fj_chg_full_timer_work2);
	INIT_DELAYED_WORK(&chip->safety_timer_work, fj_chg_safety_timer_work);
	INIT_DELAYED_WORK(&chip->adp_volt_monitor_work, fj_chg_adp_volt_monitor_work);
	INIT_DELAYED_WORK(&chip->oki_det_work, fj_chg_oki_det_work);
	INIT_DELAYED_WORK(&chip->watchdog_work, fj_chg_watchdog_work);

	chip->recovery_count = 0;
	chip->recovery_invalid = true;

	memset(chip->batt_vol_data.vol_data, 0, sizeof(chip->batt_vol_data.vol_data));
	chip->batt_vol_data.index = 0;
	chip->batt_vol_data.data_full = false;

	chip->fj_pmi_adc = power_supply_get_by_name(FJ_PMI_ADC_PSY_NAME);
	if (chip->fj_pmi_adc == NULL) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_get_by_name pmi_adc failed\n", __func__);
		ret = -EPROBE_DEFER;
		goto fj_chg_err1;
	}

	chip->fj_pm_adc = power_supply_get_by_name(FJ_PM_ADC_PSY_NAME);
	if (chip->fj_pm_adc == NULL) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_get_by_name pm_adc failed\n", __func__);
		ret = -EPROBE_DEFER;
		goto fj_chg_err1;
	}

	chip->bms = power_supply_get_by_name(QPNP_BMS_PSY_NAME);
	if (chip->bms == NULL) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_get_by_name bms failed\n", __func__);
		ret = -EPROBE_DEFER;
		goto fj_chg_err1;
	}

	chip->battery = power_supply_get_by_name(FJ_BATTERY_PSY_NAME);
	if (chip->battery == NULL) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_get_by_name battery failed\n", __func__);
		ret = -EPROBE_DEFER;
		goto fj_chg_err1;
	}

	chip->fg_adc = power_supply_get_by_name(BCL_FG_ADC_PSY_NAME);
	if (chip->fg_adc == NULL) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_get_by_name fg_adc failed\n", __func__);
		ret = -EPROBE_DEFER;
		goto fj_chg_err1;
	}

	chip->fj_charger_drv_wq = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!chip->fj_charger_drv_wq) {
		FJ_CHARGER_ERRLOG("[%s] create_singlethread_workqueue failed\n", __func__);
		ret = -ESRCH;
		goto fj_chg_err1;
    }

	chip->usb.name = FJ_CHG_USB_PSY_NAME;
	chip->usb.type = POWER_SUPPLY_TYPE_FJ_USB;
	chip->usb.get_property = fj_chg_get_property;
	chip->usb.properties = fj_chg_mains_properties;
	chip->usb.num_properties = ARRAY_SIZE(fj_chg_mains_properties);

	ret = power_supply_register(&pdev->dev, &chip->usb);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_register usb failed ret = %d\n", __func__, ret);
		goto fj_chg_err2;
	}

	chip->mains.name = FJ_CHG_AC_PSY_NAME;
	chip->mains.type = POWER_SUPPLY_TYPE_FJ_AC;
	chip->mains.get_property = fj_chg_get_property;
	chip->mains.properties = fj_chg_mains_properties;
	chip->mains.num_properties = ARRAY_SIZE(fj_chg_mains_properties);

	ret = power_supply_register(&pdev->dev, &chip->mains);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_register mains failed ret = %d\n", __func__, ret);
		goto fj_chg_err3;
	}

	chip->mhl.name = FJ_CHG_MHL_PSY_NAME;
	chip->mhl.type = POWER_SUPPLY_TYPE_MHL;
	chip->mhl.get_property = fj_chg_get_property;
	chip->mhl.properties = fj_chg_mains_properties;
	chip->mhl.num_properties = ARRAY_SIZE(fj_chg_mains_properties);

	ret = power_supply_register(&pdev->dev, &chip->mhl);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_register mhl failed ret = %d\n", __func__, ret);
		goto fj_chg_err4;
	}

	chip->holder.name = FJ_CHG_HOLDER_PSY_NAME;
	chip->holder.type = POWER_SUPPLY_TYPE_HOLDER;
	chip->holder.get_property = fj_chg_get_property;
	chip->holder.properties = fj_chg_mains_properties;
	chip->holder.num_properties = ARRAY_SIZE(fj_chg_mains_properties);

	ret = power_supply_register(&pdev->dev, &chip->holder);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] power_supply_register holder failed ret = %d\n", __func__, ret);
		goto fj_chg_err5;
	}

    platform_set_drvdata(pdev, chip);

	/* GPIO */
	ret = gpio_request(FJ_CHG_OKI_DET_GPIO, "FJ_CHG_OKI_DET");
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] gpio_request failed ret = %d\n", __func__, ret);
		goto fj_chg_err6;
	}

	fj_charger_oki_det_irq = gpio_to_irq(FJ_CHG_OKI_DET_GPIO);
	gpio_direction_input(FJ_CHG_OKI_DET_GPIO);

	ret = request_irq(fj_charger_oki_det_irq,
					  fj_chg_oki_det_irq_handler,
					  IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
					  "oki_det", chip);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] request_irq failed ret = %d\n", __func__, ret);
		goto fj_chg_err7;
	}
	disable_irq(fj_charger_oki_det_irq);

/* FCNT LIMITED:2017-01-18 CHARGER mod start */
#if 0
	ret = get_nonvolatile((u8 *)&chip->vmax_adj, APNV_CHG_ADJ_VMAX_I, 1);
	if (unlikely(ret < 0)) {
		FJ_CHARGER_ERRLOG("[%s] get_nonvolatile failed. ret = %d\n", __func__, ret);
		chip->vmax_adj = 0x29;
	}
#else
	chip->vmax_adj = 0x27;
#endif
/* FCNT LIMITED:2017-01-18 CHARGER mod end */

	FJ_CHARGER_DBGLOG("[%s] vmax_adj = 0x%x\n", __func__, chip->vmax_adj);

	ret = fj_chg_pmic_initial_setting(chip);
	if (unlikely(ret != 0)) {
		FJ_CHARGER_ERRLOG("[%s] pmic initial setting failed ret = %d\n", __func__, ret);
		ret = -EPROBE_DEFER;
		goto fj_chg_err_last;
	}

	the_chip = chip;

	chip->chg_thread = kthread_run(fj_chg_drv_thread, the_chip, "fj_chg");

	FJ_CHARGER_INFOLOG("[%s] probe End\n", __func__);

	return 0;

fj_chg_err_last:
	free_irq(fj_charger_oki_det_irq, chip);
fj_chg_err7:
	gpio_free(FJ_CHG_OKI_DET_GPIO);
fj_chg_err6:
	power_supply_unregister(&chip->holder);
fj_chg_err5:
	power_supply_unregister(&chip->mhl);
fj_chg_err4:
	power_supply_unregister(&chip->mains);
fj_chg_err3:
	power_supply_unregister(&chip->usb);
fj_chg_err2:
	destroy_workqueue(chip->fj_charger_drv_wq);
fj_chg_err1:
	kfree(chip);

	return ret;
}

/**
 * fj_charge_drv_remove
 *
 * @param   pdev : 
 *
 * @retval  Processing result
 */
static int fj_charge_drv_remove(struct platform_device *pdev)
{
    struct charge_drv_chip *chip = the_chip;

	free_irq(fj_charger_oki_det_irq, chip);
	gpio_free(FJ_CHG_OKI_DET_GPIO);

	power_supply_unregister(&chip->usb);
	power_supply_unregister(&chip->mains);
	power_supply_unregister(&chip->mhl);
	power_supply_unregister(&chip->holder);

	kthread_stop(chip->chg_thread);

	destroy_workqueue(chip->fj_charger_drv_wq);

    the_chip = NULL;
    platform_set_drvdata(pdev, NULL);
    kfree(chip);

    return 0;
}

#ifdef CONFIG_PM
/**
 * fj_charge_drv_suspend
 *
 * @param   pdev : 
 *
 * @retval  Processing result
 */
static int fj_charge_drv_suspend(struct device *pdev)
{
	return 0;
}

/**
 * fj_charge_drv_resume
 *
 * @param   pdev : 
 *
 * @retval  Processing result
 */
static int fj_charge_drv_resume(struct device *pdev)
{
	return 0;
}

static const struct dev_pm_ops fj_charge_drv_pm_ops = {
	.suspend = fj_charge_drv_suspend,
	.resume = fj_charge_drv_resume,
};

#else  /* CONFIG_PM */
#define fj_charge_drv_suspend NULL
#define fj_charge_drv_resume  NULL
#endif /* CONFIG_PM */

/**
 * fj_charge_drv_shutdown
 *
 * @param   pdev : 
 */
static void fj_charge_drv_shutdown(struct platform_device *pdev)
{
    struct charge_drv_chip *chip = the_chip;

	FJ_CHARGER_DBGLOG("[%s] in\n", __func__);

	fj_chg_terminate(chip, true);

	return;
}

static struct of_device_id charge_drv_match_table[] = {
    { .compatible = "fj,fj_charge_drv",},
    { },
};

static struct platform_driver fj_charge_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "fj_charge_drv",
		.of_match_table = charge_drv_match_table,
#ifdef CONFIG_PM
		.pm = &fj_charge_drv_pm_ops,
#endif /* CONFIG_PM */
           },
	.probe = fj_charge_drv_probe,
	.remove = fj_charge_drv_remove,
	.shutdown	= fj_charge_drv_shutdown,
#ifndef CONFIG_PM
	.suspend	= fj_charge_drv_suspend,
	.resume		= fj_charge_drv_resume,
#endif /* !CONFIG_PM */
};

/**
 * fj_charge_drv_init
 *
 * @retval  Processing result
 */
static int __init fj_charge_drv_init(void)
{
	int i;

	INIT_LIST_HEAD(&fj_chg_evt_list_free);
	INIT_LIST_HEAD(&fj_chg_evt_list_exec);
	for (i = 0; i < FJ_CHG_EVT_QUEUE_MAX; i++) {
		list_add_tail(&fj_charge_evt_queue[i].head, &fj_chg_evt_list_free);
	}

	wake_lock_init(&fj_chg_timer_wake_lock, WAKE_LOCK_SUSPEND, "fj_chg_timer");

    return platform_driver_register(&fj_charge_driver);
}
module_init(fj_charge_drv_init);

/**
 * fj_charge_drv_exit
 */
static void __exit fj_charge_drv_exit(void)
{
	wake_lock_destroy(&fj_chg_timer_wake_lock);
    platform_driver_unregister(&fj_charge_driver);
}
module_exit(fj_charge_drv_exit);

MODULE_AUTHOR("Fujitsu");
MODULE_DESCRIPTION("Charger Driver");
MODULE_LICENSE("GPL");
