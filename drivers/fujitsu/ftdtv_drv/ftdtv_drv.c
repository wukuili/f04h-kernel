/*----------------------------------------------------------------------------*/
// ftdtv_drv.c COPYRIGHT FUJITSU CONNECTED TECHNOLOGIES LIMITED 2012-2017
/*----------------------------------------------------------------------------*/
/*
 * Copyright(C) 2012 FUJITSU LIMITED
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

#define DTV_FJFEAT_PRODUCT_FJDEV038 1

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>

#include <asm/uaccess.h>
#include <asm/io.h>

//#include <linux/mfd/fj_charger_local.h>
#include <linux/qpnp/pin.h>

#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
//#include <mach/vreg.h>
//#include "../../../arch/arm/mach-msm/board-8064.h"

//#include <asm/system.h>

/* governor s */
#include <linux/poll.h>
/* governor e */

#include <linux/nonvolatile_common.h>

#include <linux/of_gpio.h>

#ifdef	DEBUG_TEST
#include "debug/debug_tp.h"
#endif	/* DEBUG_TEST */

int		g_ftdtv_debug_flg = 0x00000000;

#define	FTDTV_DBG_LOG		0x00000001
#define	FTDTV_DBG2_LOG		0x00000002

#define	FTDTV_ERR_PRINTK(fmt, arg...)	{ printk(KERN_ERR "ftdtv_drv: %s (%d): " fmt, __func__, __LINE__, ##arg); }
#define	FTDTV_INFO_PRINTK(fmt, arg...)	{ printk(KERN_INFO "ftdtv_drv: %s (%d): " fmt, __func__, __LINE__, ##arg); }
#define	FTDTV_DBG_PRINTK(fmt, arg...)	{ if (g_ftdtv_debug_flg & FTDTV_DBG_LOG) printk(KERN_INFO "DBG: ftdtv_drv: %s (%d): " fmt, __func__, __LINE__, ##arg); }
#define	FTDTV_DBG2_PRINTK(fmt, arg...)	{ if (g_ftdtv_debug_flg & FTDTV_DBG2_LOG) printk(KERN_INFO "DBG: ftdtv_drv: %s (%d): " fmt, __func__, __LINE__, ##arg); }

#define DBL_MAX		1.7976931348623157E+308
#define UCHAR_MAX	0xffU

#define DTV_BAND_UHF		0
#define DTV_BAND_VHF		1

#define TYPE_POW_1SEG 0x01
#define TYPE_POW_MM   0x02
#define TYPE_POW_FMT  0x04

#define DTV_ANT_INTERNAL	0
#define DTV_ANT_USB			1
#define DTV_ANT_EARPHONE	2
#define DTV_ANT_OFF			3

struct FTDTVMonitorParam{
    unsigned char   channel;
    double          calc_ber_a;
    double          calc_ber_b;
    double          calc_ber_c;
    double          calc_per_a;
    double          calc_per_b;
    double          calc_per_c;
    double          calc_cn;
    double          calc_rssi;
};

struct FTDTVModeParam{
	unsigned int	state;
	unsigned int	channel;
	unsigned int	subchannel;
	unsigned int	band;
	unsigned int	segment;

	int pidtuner;
	int pidfmt;
};

struct FTDTVPowerParam{
	int	band;
	int	type;
	int	pid;
};

struct FTDTVNvvalue{
	unsigned long	value;
	unsigned long	id;
	unsigned long	size;
};

/* IOCTL CMD */
#define FTDTV_IOCTL_MAGIC		'f'

#define FTDTV_IOCTL_POWER_ON		_IOW(FTDTV_IOCTL_MAGIC, 0, struct FTDTVPowerParam)
#define FTDTV_IOCTL_POWER_OFF		_IOW(FTDTV_IOCTL_MAGIC, 1, struct FTDTVPowerParam)
#define FTDTV_IOCTL_ANTENNA_MAIN	_IOC(_IOC_NONE, FTDTV_IOCTL_MAGIC, 2, 0)
#define FTDTV_IOCTL_ANTENNA_SUB		_IOC(_IOC_NONE, FTDTV_IOCTL_MAGIC, 3, 0)
#define FTDTV_IOCTL_ANTENNA_CYCLIC	_IOR(FTDTV_IOCTL_MAGIC, 4, int)
#define FTDTV_IOCTL_MONITOR_WRITE	_IOW(FTDTV_IOCTL_MAGIC, 5, struct FTDTVMonitorParam)
#define FTDTV_IOCTL_MONITOR_READ	_IOR(FTDTV_IOCTL_MAGIC, 6, struct FTDTVMonitorParam)
#define FTDTV_IOCTL_MODE_WRITE		_IOW(FTDTV_IOCTL_MAGIC, 7, struct FTDTVModeParam)
#define FTDTV_IOCTL_MODE_READ		_IOR(FTDTV_IOCTL_MAGIC, 8, struct FTDTVModeParam)
#define FTDTV_IOCTL_ANTENNA_POWER	_IOW(FTDTV_IOCTL_MAGIC, 9, int)
#define FTDTV_IOCTL_CHANGE_XRESET	_IOW(FTDTV_IOCTL_MAGIC, 10, int)
#define FTDTV_IOCTL_GET_ANT_SELECT	_IOR(FTDTV_IOCTL_MAGIC, 11, int)
#define FTDTV_IOCTL_SET_ANT_SELECT	_IOW(FTDTV_IOCTL_MAGIC, 12, int)
#define FTDTV_IOCTL_GET_USB_ANT		_IOR(FTDTV_IOCTL_MAGIC, 13, int)
#define	FTDTV_IOCTL_ANTENNA_EAR		_IOC( _IOC_NONE, FTDTV_IOCTL_MAGIC, 14, 0 )
#define FTDTV_IOCTL_SET_NONVOLATILE	_IOR(FTDTV_IOCTL_MAGIC, 15, struct FTDTVNvvalue)
#define FTDTV_IOCTL_GET_OPE_STATE	_IOR(FTDTV_IOCTL_MAGIC, 16, int)
#define FTDTV_IOCTL_SET_OPE_STATE	_IOW(FTDTV_IOCTL_MAGIC, 17, int)
#if 1 // RUG
#define FTDTV_IOCTL_GET_ANT_TYPE	_IOR(FTDTV_IOCTL_MAGIC, 18, int)
#define FTDTV_IOCTL_SET_ANT_TYPE	_IOW(FTDTV_IOCTL_MAGIC, 19, int)
#endif

//FUJITSU:2013-05-16 For Panasonic tuner Add -S
#define TUNERX_DEBUG
//FUJITSU:2013-05-16 For Panasonic tuner Add -S

//FUJITSU:2013-04-25 For Panasonic tuner Add -S
#ifdef TUNERX_DEBUG
#define FTDTV_IOCTL_SHOW_STATE		_IOR(FTDTV_IOCTL_MAGIC, 99, int)
#define STATE_VDDA 1
#define STATE_VDDL 2
#define STATE_NRST 4
static struct PowerShowParam
{
	int state;
	int voltage[3];
} g_powerShowParam = {0};
int ftdtv_get_state(void);
#endif //TUNERX_DEBUG
//FUJITSU:2013-04-25 For Panasonic tuner Add -E

#define FTDTV_IOCTL_DEBUG			_IOR(FTDTV_IOCTL_MAGIC, 100, int)

#define	FTDTV_HARD_TYPE_F		0x00	/* FSL  */
#define	FTDTV_HARD_TYPE_P		0x01	/* PANA */
#define	FTDTV_HARD_LEVEL1		0x01	/* 13_2nd */
#define	FTDTV_HARD_LEVEL2		0x02	/* 14_1st */
#define	FTDTV_HARD_MODE_0		0x00
#define	FTDTV_HARD_MODE_1		0x01
#define	FTDTV_HARD_MODE_2		0x02
int	ftdtv_hard_type = (-1);
int	ftdtv_hard_level = FTDTV_HARD_LEVEL1;
int	ftdtv_hard_mode = FTDTV_HARD_MODE_0;

//todo remove these
#ifndef FJ_CHG_DEVICE_TYPE_FJDEV010
 #define FJ_CHG_DEVICE_TYPE_FJDEV010  0x30
#endif
#ifndef FJ_CHG_DEVICE_TYPE_FJDEV011
 #define FJ_CHG_DEVICE_TYPE_FJDEV011  0x40
#endif
#ifndef FJ_CHG_DEVICE_TYPE_FJDEV012
 #define FJ_CHG_DEVICE_TYPE_FJDEV012  0x50
#endif
#ifndef FJ_CHG_DEVICE_TYPE_FJDEV013
 #define FJ_CHG_DEVICE_TYPE_FJDEV013  0x60
#endif

#if 1 // RUG
#else

#define FTDTV_XRESET_GPIO			76	/* DTV_XRESET     ：MSM8974 GPIO[76] */		
#define FTDTV_ANT_SELECT1_GPIO		57	/* DTV_ANT_SELECT1：MSM8974 GPIO[57] */		
#define FTDTV_ANT_SELECT2_GPIO_F	125	/* DTV_ANT_SELECT2：MSM8974 GPIO[125] */	
#define FTDTV_ANT_SELECT2_GPIO_P	50	/* DTV_ANT_SELECT2：MSM8974 GPIO[50] */		
#define FTDTV_NPD_GPIO				105	/* DTV_NPD        ：MSM8974 GPIO[105] */	
#define FTDTV_ANT_28V_ON_GPIO		119	/* DTV_ANT_28V_ON ：MSM8974 GPIO[119] */	

#define FTDTV_ANT_28V_PON_GPIO		9	/* DTV_ANT_28V_ON ：PM8941  GPIO[9] */		
#define FTDTV_28V_ON_GPIO			10	/* DTV_28V_ON     ：PM8941  GPIO[10] */		
#define FTDTV_18V_ON_GPIO			11	/* DTV_18V_ON     ：PM8941  GPIO[11] */		

#endif

//static int ftdtv_ant_28v_on_gpio_no;
//static int ftdtv_28v_on_gpio_no;
//static int ftdtv_18v_on_gpio_no;

//static int ftdtv_ant_select2_gpio_no;

//static int ftdtv_ant_28v_on_flag = 0;

static struct cdev ftdtvDevs;
static int ftdtv_major = 0;
static struct class* ftdtv_class;
struct wake_lock ftdtv_wake_lock;
static int g_antenna = 0;
static int g_ant_select = DTV_ANT_OFF;
#if 1 //RUG
static int g_ant_type = FTDTV_IOCTL_ANTENNA_EAR;
#endif
static int g_ope_state = 0;
static struct FTDTVMonitorParam monitor_param = {
	.channel    = UCHAR_MAX,
	.calc_ber_a = DBL_MAX,
	.calc_ber_b = DBL_MAX,
	.calc_ber_c = DBL_MAX,
	.calc_per_a = DBL_MAX,
	.calc_per_b = DBL_MAX,
	.calc_per_c = DBL_MAX,
	.calc_cn    = DBL_MAX,
	.calc_rssi  = DBL_MAX,
};

static struct FTDTVModeParam mode_param = {
	.state   = 0,
	.channel = 0xffffffff,
	.subchannel = 0xffffffff,
	.band    = 0xffffffff,
	.segment = 0xffffffff,
};

int	ftdtv_band = DTV_BAND_UHF;

static int cntdev = 0;
static int cntpow = 0;

static int flgpower = 0;
static pid_t pidfmt   = -1;
static pid_t pidtuner = -1;

#if 0
static struct qpnp_pin_cfg ftdtv_ant_pin_28v_cfg_param = {
	.mode         = QPNP_PIN_MODE_DIG_OUT,
	.output_type  = QPNP_PIN_OUT_BUF_CMOS,
	.invert       = QPNP_PIN_INVERT_DISABLE,
	.pull         = QPNP_PIN_GPIO_PULL_NO,
	.vin_sel      = QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel      = QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en    = QPNP_PIN_MASTER_ENABLE,
	.aout_ref     = QPNP_PIN_AOUT_1V25,
	.ain_route    = QPNP_PIN_AIN_AMUX_CH5,
	.cs_out       = QPNP_PIN_CS_OUT_20MA,
};
static struct qpnp_pin_cfg ftdtv_pin_28v_cfg_param = {
	.mode         = QPNP_PIN_MODE_DIG_OUT,
	.output_type  = QPNP_PIN_OUT_BUF_CMOS,
	.invert       = QPNP_PIN_INVERT_DISABLE,
	.pull         = QPNP_PIN_GPIO_PULL_NO,
	.vin_sel      = QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel      = QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en    = QPNP_PIN_MASTER_ENABLE,
	.aout_ref     = QPNP_PIN_AOUT_1V25,
	.ain_route    = QPNP_PIN_AIN_AMUX_CH5,
	.cs_out       = QPNP_PIN_CS_OUT_20MA,
};
static struct qpnp_pin_cfg ftdtv_pin_18v_cfg_param = {
	.mode         = QPNP_PIN_MODE_DIG_OUT,
	.output_type  = QPNP_PIN_OUT_BUF_CMOS,
	.invert       = QPNP_PIN_INVERT_DISABLE,
	.pull         = QPNP_PIN_GPIO_PULL_NO,
	.vin_sel      = QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel      = QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en    = QPNP_PIN_MASTER_ENABLE,
	.aout_ref     = QPNP_PIN_AOUT_1V25,
	.ain_route    = QPNP_PIN_AIN_AMUX_CH5,
	.cs_out       = QPNP_PIN_CS_OUT_20MA,
};
#endif

#if 0
/* PM8994 VREG_L3 */
#define VREG_TABLE  1
static struct vreg_info {
    char    *name;
    unsigned int    lvl;
    //    struct vreg *vreg;
    struct regulator *reg;
} vreg_info[VREG_TABLE] = {
    {"pm8994_l3",  1200000, NULL},		//L3
};
static int vreg_enable_flag[VREG_TABLE];
#endif

static int g_gpio_11v_on = 0;
#ifndef DTV_FJFEAT_PRODUCT_FJDEV038
static int g_gpio_18v_on = 0;
#endif
static int g_gpio_rst_n  = 0;
static int g_gpio_npd    = 0;

#define POWER_DELAY_10MS		10
#define POWER_DELAY_5MS			5
#define POWER_DELAY_1MS			1

/* governor s */
static DECLARE_WAIT_QUEUE_HEAD(ftdtv_wait);
static int g_readflag = 0;
/* governor e */

static long power_off(void);

static void initialize_params(void)
{
	monitor_param.channel    = UCHAR_MAX;
//	monitor_param.calc_ber_a = DBL_MAX;
//	monitor_param.calc_ber_b = DBL_MAX;
//	monitor_param.calc_ber_c = DBL_MAX;
//	monitor_param.calc_per_a = DBL_MAX;
//	monitor_param.calc_per_b = DBL_MAX;
//	monitor_param.calc_per_c = DBL_MAX;
//	monitor_param.calc_cn    = DBL_MAX;
//	monitor_param.calc_rssi  = DBL_MAX;

	mode_param.state   = 0;
	mode_param.channel = 0xffffffff;
	mode_param.subchannel = 0xffffffff;
	mode_param.band    = 0xffffffff;
	mode_param.segment = 0xffffffff;
}

#if 0
static void ant_power_on(void)
{
	FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 1);
	gpio_set_value_cansleep(ftdtv_28v_on_gpio_no, 1);
}

static void ant_power_off(void)
{
	FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 0);
	gpio_set_value_cansleep(ftdtv_28v_on_gpio_no, 0);
}
#endif
static void set_antenna_power(int band)
{
	FTDTV_DBG_PRINTK("- S\n");

#if 1	// RUG
	return;
#else
	if (ftdtv_hard_type == FTDTV_HARD_TYPE_F) {
		/* antenna power on */
		if (band == DTV_BAND_UHF) {
			FTDTV_INFO_PRINTK("UHF: gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 1);
			gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 1);
			ftdtv_ant_28v_on_flag = 1;
		} else {
			FTDTV_INFO_PRINTK("VHF: gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
			gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 0);
			ftdtv_ant_28v_on_flag = 0;
		}
	} else {
		if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
			if (ftdtv_hard_mode != FTDTV_HARD_MODE_1) {
				/* antenna power on */
				if (band == DTV_BAND_UHF) {
					FTDTV_INFO_PRINTK("UHF: gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 1);
					gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 1);
					ftdtv_ant_28v_on_flag = 1;
				} else {
					FTDTV_INFO_PRINTK("VHF: gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
					gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 0);
					ftdtv_ant_28v_on_flag = 0;
				}
			}
		}
	}
#endif
}
static void set_antenna_select(int antenna_sel)
{
#if 1	// RUG
	return;
#else
	if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
		switch (antenna_sel) {
		case DTV_ANT_INTERNAL:
			if (ftdtv_hard_type == FTDTV_HARD_TYPE_F) {
				if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 1);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 1);
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 0);
					gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 0);
				} else {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
				}
			} else {
				if (ftdtv_hard_mode != FTDTV_HARD_MODE_1) {
					if (ftdtv_band == DTV_BAND_UHF) {
						if (ftdtv_ant_28v_on_flag == 0) {
							FTDTV_INFO_PRINTK("UHF: gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 1);
							gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 1);
							ftdtv_ant_28v_on_flag = 1;
						}
						FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
						gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
						if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
							FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 1);
							gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 1);
						}
					} else {
						FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
						gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
						if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
							FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 0);
							gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 0);
						}
					}
				} else {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
				}
			}
			g_ant_select = DTV_ANT_INTERNAL;
			FTDTV_DBG_PRINTK("ANTENNA changed INTERNAL %d g_ant_select=%d\n", antenna_sel, g_ant_select);
			break;
		case DTV_ANT_USB:
			if (ftdtv_hard_type == FTDTV_HARD_TYPE_F) {
				if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 1);
					gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 1);
				} else {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 1);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 1);
				}
			} else {
				if (ftdtv_hard_mode != FTDTV_HARD_MODE_1) {
					if (ftdtv_band == DTV_BAND_UHF) {
						FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 1);
						gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 1);
						if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
							FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 1);
							gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 1);
						}
						if (ftdtv_ant_28v_on_flag != 0) {
							FTDTV_INFO_PRINTK("UHF: gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
							gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 0);
							ftdtv_ant_28v_on_flag = 0;
						}
					} else {
						FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 1);
						gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 1);
						if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
							FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 0);
							gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 0);
						}
					}
				} else {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 1);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 1);
				}
			}
			g_ant_select = DTV_ANT_USB;
			FTDTV_DBG_PRINTK("ANTENNA changed USB %d g_ant_select=%d\n", antenna_sel, g_ant_select);
			break;
		case DTV_ANT_EARPHONE:
			if (ftdtv_hard_type == FTDTV_HARD_TYPE_F) {
				if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 1);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 1);
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 1);
					gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 1);
					g_ant_select = DTV_ANT_EARPHONE;
					FTDTV_DBG_PRINTK("ANTENNA changed EARPHONE %d g_ant_select=%d\n", antenna_sel, g_ant_select);
				} else {
					FTDTV_ERR_PRINTK("ANTENNA no changed %d g_ant_select=%d\n", antenna_sel, g_ant_select);
				}
			} else {
				FTDTV_ERR_PRINTK("ANTENNA no changed %d g_ant_select=%d\n", antenna_sel, g_ant_select);
			}
			break;
		case DTV_ANT_OFF:
			if (ftdtv_hard_type == FTDTV_HARD_TYPE_F) {
				if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 0);
					gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 0);
				} else {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
				}
			} else {
				if (ftdtv_hard_mode != FTDTV_HARD_MODE_1) {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
					if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
						FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 0);
						gpio_set_value_cansleep(ftdtv_ant_select2_gpio_no, 0);
					}
					FTDTV_INFO_PRINTK("UHF: gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
					gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 0);
					ftdtv_ant_28v_on_flag = 0;
				} else {
					FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
					gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
				}
			}
			g_ant_select = DTV_ANT_OFF;
			FTDTV_DBG_PRINTK("ANTENNA OFF %d g_ant_select=%d\n", antenna_sel, g_ant_select);
			break;
		default:
			FTDTV_ERR_PRINTK("Not Supported. antenna_sel[%d]\n", antenna_sel);
			break;
		}
	} else {
		/* ftdtv_hard_level == FTDTV_HARD_LEVEL2 */
		switch (antenna_sel) {
		case DTV_ANT_INTERNAL:
			FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 1);
			gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 1);
			g_ant_select = DTV_ANT_INTERNAL;
			FTDTV_DBG_PRINTK("ANTENNA changed INTERNAL %d g_ant_select=%d\n", antenna_sel, g_ant_select);
			break;
		case DTV_ANT_EARPHONE:
			FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
			gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
			g_ant_select = DTV_ANT_EARPHONE;
			FTDTV_DBG_PRINTK("ANTENNA changed EARPHONE %d g_ant_select=%d\n", antenna_sel, g_ant_select);
			break;
		case DTV_ANT_OFF:
			FTDTV_DBG_PRINTK("gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
			gpio_set_value_cansleep(FTDTV_ANT_SELECT1_GPIO, 0);
			g_ant_select = DTV_ANT_OFF;
			FTDTV_DBG_PRINTK("ANTENNA OFF %d g_ant_select=%d\n", antenna_sel, g_ant_select);
			break;
		default:
			FTDTV_ERR_PRINTK("Not Supported. antenna_sel[%d]\n", antenna_sel);
			break;
		}
	}
#endif
}
static long power_on(int band)
{
	long ret = 0;

	FTDTV_INFO_PRINTK("- S\n");

	do{
		initialize_params();

		wake_lock(&ftdtv_wake_lock);

#if 1	// RUG
#ifndef DTV_FJFEAT_PRODUCT_FJDEV038
		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_18V_ON_GPIO):%d %d\n", g_gpio_18v_on, 1);
		gpio_set_value_cansleep(g_gpio_18v_on, 1);

		mdelay(POWER_DELAY_1MS);
#endif
#if 1
		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_11V_ON_GPIO):%d %d\n", g_gpio_11v_on, 1);
		gpio_set_value_cansleep(g_gpio_11v_on, 1);
#endif

		mdelay(POWER_DELAY_1MS * 2);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_NPD_GPIO):%d %d\n", g_gpio_npd, 1);
		gpio_set_value_cansleep(g_gpio_npd, 1);


		mdelay(POWER_DELAY_10MS * 2);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_RST_N_GPIO):%d %d\n", g_gpio_rst_n, 1);
		gpio_set_value_cansleep(g_gpio_rst_n, 1);

#if DTV_FJFEAT_PRODUCT_FJDEV038
		mdelay(POWER_DELAY_10MS * 2);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_NPD_GPIO):%d %d\n", g_gpio_npd, 0);
		gpio_set_value_cansleep(g_gpio_npd, 0);

		mdelay(POWER_DELAY_10MS * 2);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_NPD_GPIO):%d %d\n", g_gpio_npd, 1);
		gpio_set_value_cansleep(g_gpio_npd, 1);

		mdelay(POWER_DELAY_10MS * 2);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_RST_N_GPIO):%d %d\n", g_gpio_rst_n, 0);
		gpio_set_value_cansleep(g_gpio_rst_n, 0);

		mdelay(POWER_DELAY_10MS * 2);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_RST_N_GPIO):%d %d\n", g_gpio_rst_n, 1);
		gpio_set_value_cansleep(g_gpio_rst_n, 1);
#endif
		mdelay(POWER_DELAY_5MS * 2);

#else
		if (ftdtv_hard_type == FTDTV_HARD_TYPE_F) {
			/* 1.8V */
			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_18v_on_gpio_no):%d %d\n", ftdtv_18v_on_gpio_no, 1);
			gpio_set_value_cansleep(ftdtv_18v_on_gpio_no, 1);

			mdelay(POWER_DELAY_1MS);

			/* L3 1.2V */
			FTDTV_INFO_PRINTK("L3 1.2V ENABLE\n");
			if (regulator_enable(vreg_info[0].reg)) {
				FTDTV_ERR_PRINTK("vreg %s enable failed !\n", vreg_info[0].name);
			    ret =  -EINVAL;
			    break;
			}
			vreg_enable_flag[0] = 1;

			mdelay(POWER_DELAY_1MS);

			/* 2.8V */
			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 1);
			gpio_set_value_cansleep(ftdtv_28v_on_gpio_no, 1);

			/* antenna power on */
			set_antenna_power(band);

			mdelay(POWER_DELAY_10MS);

			/* XRESET */
			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_XRESET_GPIO):%d %d\n", FTDTV_XRESET_GPIO, 1);
			gpio_set_value_cansleep(FTDTV_XRESET_GPIO, 1);

			mdelay(POWER_DELAY_5MS);
		} else {	/* FTDTV_HARD_TYPE_P */
			if (ftdtv_hard_mode != FTDTV_HARD_MODE_1) {
				/* 2.8V */
				FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 1);
				gpio_set_value_cansleep(ftdtv_28v_on_gpio_no, 1);

				/* antenna power on */
				set_antenna_power(band);
			}

			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_18v_on_gpio_no):%d %d\n", ftdtv_18v_on_gpio_no, 1);
			gpio_set_value_cansleep(ftdtv_18v_on_gpio_no, 1);

			mdelay(POWER_DELAY_1MS);

			/* L3 1.2V */
			FTDTV_INFO_PRINTK("L3 1.2V ENABLE\n");
			if (regulator_enable(vreg_info[0].reg)) {
				FTDTV_ERR_PRINTK("vreg %s enable failed !\n", vreg_info[0].name);
			    ret =  -EINVAL;
			    break;
			}

			mdelay(POWER_DELAY_1MS);

			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_NPD_GPIO):%d %d\n", FTDTV_NPD_GPIO, 1);
			gpio_set_value_cansleep(FTDTV_NPD_GPIO, 1);

			mdelay(POWER_DELAY_10MS);

			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_XRESET_GPIO):%d %d\n", FTDTV_XRESET_GPIO, 1);
			gpio_set_value_cansleep(FTDTV_XRESET_GPIO, 1);

			mdelay(POWER_DELAY_5MS);
		}
#endif

		cntpow = 1;

	}while(0);

	/* governor s */
   	wake_up_interruptible(&ftdtv_wait);
	g_readflag = 1;
	/* governor e */
	FTDTV_INFO_PRINTK("- E : ret = %ld\n", ret);
	return ret;
}

static long power_off(void)
{
	long ret = 0;

	FTDTV_INFO_PRINTK("- S\n");

	do{
#if 1	// RUG
		mdelay(POWER_DELAY_1MS * 2);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_RST_N_GPIO):%d %d\n", g_gpio_rst_n, 0);
		gpio_set_value_cansleep(g_gpio_rst_n, 0);

		mdelay(POWER_DELAY_5MS * 2);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_NPD_GPIO):%d %d\n", g_gpio_npd, 0);
		gpio_set_value_cansleep(g_gpio_npd, 0);

		mdelay(POWER_DELAY_1MS * 2);

#if 1
		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_11V_ON_GPIO):%d %d\n", g_gpio_11v_on, 0);
		gpio_set_value_cansleep(g_gpio_11v_on, 0);
#endif
#ifndef DTV_FJFEAT_PRODUCT_FJDEV038
		mdelay(POWER_DELAY_1MS);

		FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_18V_ON_GPIO):%d %d\n", g_gpio_18v_on, 0);
		gpio_set_value_cansleep(g_gpio_18v_on, 0);
#endif
#else
		if (ftdtv_hard_type == FTDTV_HARD_TYPE_F) {
			mdelay(POWER_DELAY_1MS);

			/* DTV XRESET */
			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_XRESET_GPIO):%d %d\n", FTDTV_XRESET_GPIO, 0);
			gpio_set_value_cansleep(FTDTV_XRESET_GPIO, 0);

			mdelay(POWER_DELAY_1MS);

			if (vreg_enable_flag[0]) {
				/* L3 1.2V */
				FTDTV_INFO_PRINTK("L3 1.2V DISABLE\n");
				if (regulator_disable(vreg_info[0].reg)) {
					FTDTV_INFO_PRINTK("vreg %s disable failed !\n", vreg_info[0].name);
					ret = -1;
				}
				vreg_enable_flag[0] = 0;
			}

			mdelay(POWER_DELAY_1MS);

			/* 1.8V */
			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_18v_on_gpio_no):%d %d\n", ftdtv_18v_on_gpio_no, 0);
			gpio_set_value_cansleep(ftdtv_18v_on_gpio_no, 0);

			if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
				/* 2.8V */
				FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 0);
				gpio_set_value_cansleep(ftdtv_28v_on_gpio_no, 0);

				/* antenna power off */
				FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
				gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 0);
			} else {
				/* ftdtv_hard_level == FTDTV_HARD_LEVEL2 */
				if (!(flgpower & TYPE_POW_FMT)) {
					/* 2.8V */
					FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 0);
					gpio_set_value_cansleep(ftdtv_28v_on_gpio_no, 0);
				} else {
				}
				/* antenna power off */
				FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
				gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 0);
			}
		} else {	/* FTDTV_HARD_TYPE_P */
			mdelay(POWER_DELAY_1MS);

			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_XRESET_GPIO):%d %d\n", FTDTV_XRESET_GPIO, 0);
			gpio_set_value_cansleep(FTDTV_XRESET_GPIO, 0);

			mdelay(POWER_DELAY_5MS);

			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(FTDTV_NPD_GPIO):%d %d\n", FTDTV_NPD_GPIO, 0);
			gpio_set_value_cansleep(FTDTV_NPD_GPIO, 0);

			mdelay(POWER_DELAY_1MS);

			FTDTV_INFO_PRINTK("L3 1.2V DISABLE\n");
			if (regulator_disable(vreg_info[0].reg)) {
				FTDTV_INFO_PRINTK("vreg %s disable failed !\n", vreg_info[0].name);
				ret = -1;
			}

			FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_18v_on_gpio_no):%d %d\n", ftdtv_18v_on_gpio_no, 0);
			gpio_set_value_cansleep(ftdtv_18v_on_gpio_no, 0);

			if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
				if (ftdtv_hard_mode != FTDTV_HARD_MODE_1) {
					/* 2.8V */
					FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 0);
					gpio_set_value_cansleep(ftdtv_28v_on_gpio_no, 0);

					/* antenna power off */
					FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
					gpio_set_value_cansleep(ftdtv_ant_28v_on_gpio_no, 0);
				}
			} else {
				/* ftdtv_hard_level == FTDTV_HARD_LEVEL2 */
				if (!(flgpower & TYPE_POW_FMT)) {
					/* 2.8V */
					FTDTV_INFO_PRINTK("gpio_set_value_cansleep(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 0);
					gpio_set_value_cansleep(ftdtv_28v_on_gpio_no, 0);
				} else {
				}
			}

			set_antenna_select(DTV_ANT_OFF);
		}

#endif
	}while(0);

	cntpow = 0;

	/* governor s */
	wake_up_interruptible(&ftdtv_wait);
	g_readflag = 1;
	/* governor e */

	wake_unlock(&ftdtv_wake_lock);

	FTDTV_INFO_PRINTK("- E : ret = %ld\n", ret);
	return ret;
}
/*---------------------------------------------------------------------------
    open
---------------------------------------------------------------------------*/
int ftdtv_open(struct inode* inode, struct file* file)
{
	if((file->f_flags & 0x0000000f) != O_RDONLY){
		cntdev++;
	}

	FTDTV_INFO_PRINTK("flag=%08x cnt=%d\n", file->f_flags, cntdev);
	return 0;
}


/*---------------------------------------------------------------------------
    release
---------------------------------------------------------------------------*/
int ftdtv_release(struct inode *inode, struct file *file)
{
	if((file->f_flags & 0x0000000f) != O_RDONLY){
		cntdev--;
	}

	if(cntdev == 0){
		if(flgpower){
			flgpower = 0;
			power_off();
		}
	}

	FTDTV_INFO_PRINTK("flag=%08x cnt=%d\n", file->f_flags, cntdev);
	return 0;
}

/*---------------------------------------------------------------------------
    ftdtv_ioctl
---------------------------------------------------------------------------*/
static long ftdtv_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	int rc = 0;
	int band = 0;
	int antenna_sel;
	int xreset = 0;

	switch (cmd) {
	case FTDTV_IOCTL_POWER_ON:
//		FTDTV_INFO_PRINTK("FTDTV_IOCTL_POWER_ON -S T=0x%02x M=0x%02x L=0x%02x S=0x%02x\n",
//			ftdtv_hard_type, ftdtv_hard_mode, ftdtv_hard_level, system_rev);
		do {
			struct FTDTVPowerParam powparam;

			rc = copy_from_user(&powparam, (int __user *) arg, sizeof(struct FTDTVPowerParam));
			if(rc != 0){
				FTDTV_ERR_PRINTK("copy_from_user Failed. [%d]\n", rc);
				ret = -ENODEV;
				break;
			}
			if(powparam.type == TYPE_POW_1SEG ||
			   powparam.type == TYPE_POW_MM   ){
				if(flgpower & (TYPE_POW_1SEG | TYPE_POW_MM)){
					ret = -EBUSY;
					break;
				}
			}else if(powparam.type == TYPE_POW_FMT){
				if(flgpower & TYPE_POW_FMT){
					ret = -EBUSY;
					break;
				}
			}
			else{
				ret = -ENODEV;
				break;
			}

#if 1
			if ((powparam.band != DTV_BAND_UHF) && (powparam.band != DTV_BAND_VHF)) {
				ret = (-EINVAL);
				break;
			}

			if(powparam.type == TYPE_POW_FMT){
				pidfmt = powparam.pid;
			}else{
				ret = power_on(powparam.band);
				if(ret != 0){
					power_off();
					break;
				}
				pidtuner = powparam.pid;
			}
			flgpower |= powparam.type;
#else
			if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
				if(!flgpower){
					if ((powparam.band == DTV_BAND_UHF) || (powparam.band == DTV_BAND_VHF)) {
						ret = power_on(powparam.band);
						if(ret != 0){
							power_off();
							break;
						}
					} else {
						FTDTV_ERR_PRINTK("Invalid parameter, band=0x%x\n", powparam.band);
						ret = (-EINVAL);
						break;
					}
				}
			} else {
				/* ftdtv_hard_level == FTDTV_HARD_LEVEL2 */
				if (!(flgpower & (TYPE_POW_1SEG | TYPE_POW_MM))) {
					if ((powparam.band == DTV_BAND_UHF) || (powparam.band == DTV_BAND_VHF)) {
						if (powparam.type & (TYPE_POW_1SEG | TYPE_POW_MM)) {
							ret = power_on(powparam.band);
							if(ret != 0){
								power_off();
								break;
							}
						} else {
							ant_power_on();
						}
					} else {
						FTDTV_ERR_PRINTK("Invalid parameter, band=0x%x\n", powparam.band);
						ret = (-EINVAL);
						break;
					}
				} else {
				}
			}
			if(powparam.type == TYPE_POW_FMT){
				pidfmt = powparam.pid;
			}else{
				pidtuner = powparam.pid;
			}
			flgpower |= powparam.type;
#endif
		} while(0);
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_POWER_ON -E : ret = %ld : cnt = %d\n", ret, cntpow);
		break;
	case FTDTV_IOCTL_POWER_OFF:
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_POWER_OFF -S\n");
		do {
			struct FTDTVPowerParam powparam;
//			int tmppower = flgpower;

			rc = copy_from_user(&powparam, (int __user *) arg, sizeof(struct FTDTVPowerParam));
			if(rc != 0){
				FTDTV_ERR_PRINTK("copy_from_user Failed. [%d]\n", rc);
				ret = -ENODEV;
				break;
			}

#if 1
			if(powparam.type == TYPE_POW_FMT){
				pidfmt = -1;
				flgpower &= ~TYPE_POW_FMT;
			}else{
				pidtuner = -1;
				flgpower &= ~(TYPE_POW_1SEG | TYPE_POW_MM);
				power_off();
			}
#else
			if(powparam.type == TYPE_POW_FMT){
				pidfmt = -1;
				flgpower &= ~TYPE_POW_FMT;
			}else{
				pidtuner = -1;
				flgpower &= ~(TYPE_POW_1SEG | TYPE_POW_MM);
			}

			if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
				if((tmppower != flgpower) && !flgpower){
					power_off();
				}
			} else {
				/* ftdtv_hard_level == FTDTV_HARD_LEVEL2 */
				if (tmppower != flgpower) {
					if (powparam.type == TYPE_POW_FMT) {
						if (!(flgpower & (TYPE_POW_1SEG | TYPE_POW_MM))) {
							ant_power_off();
						} else {
						}
					} else {
						power_off();
					}
				}
			}
#endif
		}while(0);
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_POWER_OFF -E : ret = %ld : cnt = %d\n", ret, cntpow);
		break;
	case FTDTV_IOCTL_ANTENNA_MAIN:
		if (cntpow) {
			set_antenna_select(DTV_ANT_INTERNAL);
		} else {
			set_antenna_select(DTV_ANT_OFF);
		}
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_ANTENNA_MAIN g_ant_select=%d\n", g_ant_select);
		break;
	case FTDTV_IOCTL_ANTENNA_SUB:
		set_antenna_select(DTV_ANT_USB);
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_ANTENNA_SUB g_ant_select=%d\n", g_ant_select);
		break;
	case FTDTV_IOCTL_ANTENNA_CYCLIC:
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_ANTENNA_CYCLIC -S [g_antenna:%d]\n", g_antenna);
		rc = copy_to_user((int __user *) arg, &g_antenna, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_to_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		break;
	case FTDTV_IOCTL_MONITOR_WRITE:
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_MONITOR_WRITE -S\n");
		rc = copy_from_user(&monitor_param, (int __user *) arg, sizeof(monitor_param));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_from_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		break;
	case FTDTV_IOCTL_MONITOR_READ:
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_MONITOR_READ -S\n");
		rc = copy_to_user((int __user *) arg, &monitor_param, sizeof(monitor_param));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_to_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		break;
	case FTDTV_IOCTL_MODE_WRITE:
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_MODE_WRITE -S\n");
		rc = copy_from_user(&mode_param, (int __user *) arg, sizeof(mode_param));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_from_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		break;
	case FTDTV_IOCTL_MODE_READ:
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_MODE_READ -S\n");
		mode_param.state    = flgpower;
		mode_param.pidtuner = pidtuner;
		mode_param.pidfmt   = pidfmt;
		rc = copy_to_user((int __user *) arg, &mode_param, sizeof(mode_param));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_to_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		break;
	case FTDTV_IOCTL_ANTENNA_POWER:
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_ANTENNA_POWER called\n");
		rc = copy_from_user(&band, (int __user *) arg, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_from_user Failed. [%d]\n", rc);
			return -ENODEV;
		}
		ftdtv_band = band;
		set_antenna_power(band);
		break;
	case FTDTV_IOCTL_CHANGE_XRESET:
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_CHANGE_XRESET -S\n");
		rc = copy_from_user(&xreset, (int __user *) arg, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_from_user Failed. [%d]\n", rc);
			return -ENODEV;
		}
#if 0
		if(xreset){
			FTDTV_INFO_PRINTK("XRESET is changed into High.\n");
			/* DTV_XRESET is changed into High. */
			gpio_set_value_cansleep(FTDTV_XRESET_GPIO, 1);
		}
		else{
			FTDTV_INFO_PRINTK("XRESET is changed into Low.\n");
			/* DTV_XRESET is changed into Low. */
			gpio_set_value_cansleep(FTDTV_XRESET_GPIO, 0);
		}
#endif
		break;

	case FTDTV_IOCTL_GET_ANT_SELECT:
		rc = copy_to_user((int __user *) arg, &g_ant_select, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("FTDTV_IOCTL_GET_ANT_SELECT copy_to_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_GET_ANT_SELECT g_ant_select=%d\n", g_ant_select);
		break;
	case FTDTV_IOCTL_SET_ANT_SELECT:
		rc = copy_from_user(&antenna_sel, (int __user *) arg, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("FTDTV_IOCTL_SET_ANT_SELECT copy_from_user Failed. [%d]\n", rc);
			return -EFAULT;
		}
		set_antenna_select(antenna_sel);
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_SET_ANT_SELECT g_ant_select=%d\n", g_ant_select);
		break;
	case FTDTV_IOCTL_GET_USB_ANT:
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_GET_USB_ANT -S [g_antenna:%d]\n", g_antenna);
		rc = copy_to_user((int __user *) arg, &g_antenna, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_to_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		break;
//FUJITSU:2013-04-25 For Panasonic tuner Add -S
	case FTDTV_IOCTL_SHOW_STATE:
		if (ftdtv_hard_type == FTDTV_HARD_TYPE_P) {
			int state;
			FTDTV_DBG_PRINTK("FTDTV_IOCTL_SHOW_STATE -S\n");
			state = ftdtv_get_state();
			rc = copy_to_user((struct powerShowParam __user *) arg, &g_powerShowParam, sizeof(g_powerShowParam));
			if(rc != 0){
				FTDTV_ERR_PRINTK("copy_to_user Failed. [%d]\n", rc);
				ret = (-EINVAL);
			}
			FTDTV_DBG_PRINTK("FTDTV_IOCTL_SHOW_STATE -E [%d]\n", state);

		} else {
			FTDTV_ERR_PRINTK("Command Failed cmd=0x%04x\n", cmd);
			ret = (-EINVAL);
		}
		break;
//FUJITSU:2013-04-25 For Panasonic tuner Add -E
	case FTDTV_IOCTL_ANTENNA_EAR:
		set_antenna_select(DTV_ANT_EARPHONE);
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_ANTENNA_EAR g_ant_select=%d\n", g_ant_select);
		break;
	case FTDTV_IOCTL_SET_NONVOLATILE:
		{
		struct FTDTVNvvalue nvvalue;
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_SET_NONVOLATILE -S\n");
		rc = copy_from_user(&nvvalue, (int __user *) arg, sizeof(nvvalue));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_from_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		rc = set_nonvolatile((uint8_t*)&nvvalue.value, nvvalue.id, nvvalue.size);
		if(rc <= 0){
			FTDTV_ERR_PRINTK("get_nonvolatile Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		}
	case FTDTV_IOCTL_GET_OPE_STATE:
		rc = copy_to_user((int __user *) arg, &g_ope_state, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("FTDTV_IOCTL_GET_OPE_STATE copy_to_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		FTDTV_DBG_PRINTK("FTDTV_IOCTL_GET_OPE_STATE g_ope_state=%d\n", g_ope_state);
		break;
	case FTDTV_IOCTL_SET_OPE_STATE:
		rc = copy_from_user(&g_ope_state, (int __user *) arg, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("FTDTV_IOCTL_SET_OPE_STATE copy_from_user Failed. [%d]\n", rc);
			return -EFAULT;
		}
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_SET_OPE_STATE g_ope_state=%d\n", g_ope_state);
		break;
	case FTDTV_IOCTL_DEBUG:
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_DEBUG called\n");
		rc = copy_from_user(&g_ftdtv_debug_flg, (int __user *) arg, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("copy_from_user Failed. [%d]\n", rc);
			return -ENODEV;
		}
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_DEBUG called. g_ftdtv_debug_flg=0x%x\n", g_ftdtv_debug_flg);
		break;
#if 1 //RUG
	case FTDTV_IOCTL_GET_ANT_TYPE:
		rc = copy_to_user((int __user *) arg, &g_ant_type, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("FTDTV_IOCTL_GET_ANT_TYPE copy_to_user Failed. [%d]\n", rc);
			ret = (-EINVAL);
		}
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_GET_ANT_TYPE g_ant_type=%d\n", g_ant_type);
		break;
	case FTDTV_IOCTL_SET_ANT_TYPE:
		rc = copy_from_user(&antenna_sel, (int __user *) arg, sizeof(int));
		if(rc != 0){
			FTDTV_ERR_PRINTK("FTDTV_IOCTL_SET_ANT_TYPE copy_from_user Failed. [%d]\n", rc);
			return -EFAULT;
		}
		g_ant_type = antenna_sel;
		FTDTV_INFO_PRINTK("FTDTV_IOCTL_SET_ANT_TYPE g_ant_type=%d\n", g_ant_type);
		break;
#endif
	default:
		FTDTV_ERR_PRINTK("Command Failed cmd=0x%04x\n", cmd);
		ret = (-EINVAL);
		break;
	}

	FTDTV_DBG_PRINTK("-E\n");
	return ret;
}

/* governor s */
/*---------------------------------------------------------------------------
    ftdtv_read
---------------------------------------------------------------------------*/
static ssize_t ftdtv_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len;
	int ret;
	char temp[32];
	int power;

	if(cntpow){
		power = 1 ;
	}
	else {
		power = 0 ;
	}

	sprintf(temp, "%d\n", power);
	len = strlen(temp) + 1;
	if(len > count) {
		len = count;
	}
	ret = copy_to_user(buf, temp, len);
	if(!g_readflag) {
		len = 0;
	}
	g_readflag = 0;
	return len;
}

/*---------------------------------------------------------------------------
    ftdtv_poll
---------------------------------------------------------------------------*/
static unsigned int ftdtv_poll(struct file *file, struct poll_table_struct *wait)
{
	if(g_readflag) {
		return POLLIN | POLLRDNORM;
	}
	poll_wait(file, &ftdtv_wait, wait);
	return 0;
}

static ssize_t ftdtv_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	unsigned int input;
	int ret;
	char temp[64];
	strncpy(temp, buf, sizeof(temp));
	ret = sscanf(temp, "%u", &input);

	if (ret != 1)
		return -EINVAL;
	if(input) {
        cntpow = 1;
	}
	else {
        cntpow = 0;
	}
	g_readflag = 1;
	wake_up_interruptible(&ftdtv_wait);

	return count;
}
/* governor e */

static struct file_operations ftdtv_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = ftdtv_open,
	.unlocked_ioctl = ftdtv_ioctl,
	.compat_ioctl   = ftdtv_ioctl,
	.release = ftdtv_release,
/* governor s */
	.write = ftdtv_write,
	.read = ftdtv_read,
	.poll = ftdtv_poll
/* governor e */
};

/*---------------------------------------------------------------------------
    ftdtv_setup_cdev
---------------------------------------------------------------------------*/
static void ftdtv_setup_cdev(struct cdev *dev, int minor, struct file_operations *fops)
{
	int err, devno;
	FTDTV_INFO_PRINTK("-S\n");

	devno = MKDEV(ftdtv_major, minor);
	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add(dev, devno, 1);
	if(err){
		FTDTV_ERR_PRINTK("ftdtv_setup_cdev Error %d adding rfs%d\n", err, minor);
	}
	if(IS_ERR(device_create(ftdtv_class, NULL, devno, NULL, "ftdtv"))){
		FTDTV_ERR_PRINTK("ftdtv_setup_cdev can't create device\n");
	}
	FTDTV_INFO_PRINTK("-E\n");
}

/*---------------------------------------------------------------------------
    exit
---------------------------------------------------------------------------*/
static void __exit ftdtv_exit(void)
{
	FTDTV_INFO_PRINTK("-S\n");

	wake_lock_destroy(&ftdtv_wake_lock);

	cdev_del(&ftdtvDevs);
	unregister_chrdev_region(MKDEV(ftdtv_major, 0), 2);
	device_destroy(ftdtv_class, MKDEV(ftdtv_major, 0));
	class_destroy(ftdtv_class);

	FTDTV_INFO_PRINTK("-E\n");
}

/*---------------------------------------------------------------------------
    init
---------------------------------------------------------------------------*/
static int __init ftdtv_init(void)
{
	int result = 0;
//	int i;
	dev_t dev;

	FTDTV_INFO_PRINTK("-S\n");

	do {
		struct device_node*	of_node = NULL;
		
#if DTV_FJFEAT_PRODUCT_FJDEV038
		of_node = of_find_compatible_node(NULL, NULL, "mn88553h");
#else
		of_node = of_find_compatible_node(NULL, NULL, "mn885521h");
#endif
		if(!of_node)
		{
			// 
		    FTDTV_ERR_PRINTK("device node not found [mn885521h]\n");
			break;
		}

		g_gpio_11v_on = of_get_named_gpio_flags(of_node, "dtv_11v_on", 0, NULL);
#ifndef DTV_FJFEAT_PRODUCT_FJDEV038
		g_gpio_18v_on = of_get_named_gpio_flags(of_node, "dtv_18v_on", 0, NULL);
#endif
		g_gpio_rst_n  = of_get_named_gpio_flags(of_node, "dtv_rst_n", 0, NULL);
		g_gpio_npd    = of_get_named_gpio_flags(of_node, "dtv_npd", 0, NULL);
		
#if DTV_FJFEAT_PRODUCT_FJDEV038
		FTDTV_INFO_PRINTK("dtv_11v = %d ; dtv_rst_n = %d ; dtv_npd = %d\n", 
		  g_gpio_11v_on, g_gpio_rst_n, g_gpio_npd);
#else
		FTDTV_INFO_PRINTK("dtv_11v = %d ; dtv_18v = %d ; dtv_rst_n = %d ; dtv_npd = %d\n", 
		  g_gpio_11v_on, g_gpio_18v_on, g_gpio_rst_n, g_gpio_npd);
#endif

	}while(0);

//#ifdef CONFIG_FTDTV_13_2ND
#if 0
	/* 13_2nd */
	ftdtv_hard_level = FTDTV_HARD_LEVEL1;
	if (((system_rev & 0xf0) == FJ_CHG_DEVICE_TYPE_FJDEV010) ||
		((system_rev & 0xf0) == FJ_CHG_DEVICE_TYPE_FJDEV012)) {
		if ((system_rev & 0x0f) <= 0x01) {
			FTDTV_DBG2_PRINTK("FTDTV_HARD_TYPE_F FTDTV_HARD_LEVEL1 FTDTV_HARD_MODE_1\n");
			ftdtv_hard_type = FTDTV_HARD_TYPE_F;
			ftdtv_hard_mode = FTDTV_HARD_MODE_1;
			ftdtv_ant_select2_gpio_no = FTDTV_ANT_SELECT2_GPIO_F;
		} else if ((system_rev & 0x0f) <= 0x02) {
			FTDTV_DBG2_PRINTK("FTDTV_HARD_TYPE_P FTDTV_HARD_LEVEL1 FTDTV_HARD_MODE_0\n");
			ftdtv_hard_type = FTDTV_HARD_TYPE_P;
			ftdtv_hard_mode = FTDTV_HARD_MODE_2;
			ftdtv_ant_select2_gpio_no = FTDTV_ANT_SELECT2_GPIO_P;
		} else {
			FTDTV_DBG2_PRINTK("FTDTV_HARD_TYPE_P FTDTV_HARD_LEVEL1 FTDTV_HARD_MODE_0\n");
			ftdtv_hard_type = FTDTV_HARD_TYPE_P;
			ftdtv_hard_mode = FTDTV_HARD_MODE_0;
			ftdtv_ant_select2_gpio_no = FTDTV_ANT_SELECT2_GPIO_P;
		}
	} else {
		if ((system_rev & 0x0f) <= 0x01) {
			FTDTV_DBG2_PRINTK("FTDTV_HARD_TYPE_P FTDTV_HARD_LEVEL1 FTDTV_HARD_MODE_1\n");
			ftdtv_hard_type = FTDTV_HARD_TYPE_P;
			ftdtv_hard_mode = FTDTV_HARD_MODE_1;
			ftdtv_ant_select2_gpio_no = FTDTV_ANT_SELECT2_GPIO_P;
		} else {
			FTDTV_DBG2_PRINTK("FTDTV_HARD_TYPE_F FTDTV_HARD_LEVEL1 FTDTV_HARD_MODE_0\n");
			ftdtv_hard_type = FTDTV_HARD_TYPE_F;
			ftdtv_hard_mode = FTDTV_HARD_MODE_0;
			ftdtv_ant_select2_gpio_no = FTDTV_ANT_SELECT2_GPIO_F;
		}
	}
#else
#if 1	// RUG
	ftdtv_hard_type = FTDTV_HARD_TYPE_P;
	
#else
	if ((system_rev & 0xf0) == FJ_CHG_DEVICE_TYPE_FJDEV018) {
		/* SMARTPHONE-D(FJDEV018) */
		ftdtv_hard_level = FTDTV_HARD_LEVEL2;
		if ((system_rev & 0x0f) <= 0x01) {
			FTDTV_DBG2_PRINTK("FTDTV_HARD_TYPE_P FTDTV_HARD_LEVEL2 FTDTV_HARD_MODE_0\n");
			ftdtv_hard_type = FTDTV_HARD_TYPE_P;
			ftdtv_hard_mode = FTDTV_HARD_MODE_0;
		} else {
			FTDTV_DBG2_PRINTK("FTDTV_HARD_TYPE_F FTDTV_HARD_LEVEL2 FTDTV_HARD_MODE_0\n");
			ftdtv_hard_type = FTDTV_HARD_TYPE_F;
			ftdtv_hard_mode = FTDTV_HARD_MODE_0;
		}
	}
#endif
#endif

	if (ftdtv_hard_type == (-1)) {
//		FTDTV_ERR_PRINTK("Device mismatch. system_rev=0x%02x\n", system_rev);
		return -ENODEV;
	}
//	FTDTV_INFO_PRINTK("-S [%s][%s] ftdtv_hard_type=0x%02x ftdtv_hard_mode=0x%02x ftdtv_hard_level=0x%02x system_rev=0x%02x\n",
//		__DATE__, __TIME__, ftdtv_hard_type, ftdtv_hard_mode, ftdtv_hard_level, system_rev);

	dev = MKDEV(ftdtv_major, 0);

	if(ftdtv_major){
		result = register_chrdev_region(dev, 2, "ftdtv");
	} else {
		result = alloc_chrdev_region(&dev, 0, 2, "ftdtv");
		ftdtv_major = MAJOR(dev);
	}
	if(result < 0){
		FTDTV_ERR_PRINTK("ftdtv error fail to get major %d\n", ftdtv_major);
		return result;
	}
	ftdtv_class = class_create(THIS_MODULE, "ftdtv");
	if(IS_ERR(ftdtv_class)){
	    FTDTV_ERR_PRINTK("ftdtv IS_ERR(ftdtv_class)\n");
		return PTR_ERR(ftdtv_class);
	}
	ftdtv_setup_cdev(&ftdtvDevs, 0, &ftdtv_fops);

#if 1	// RUG


	FTDTV_DBG2_PRINTK("gpio_request(FTDTV_RST_N_GPIO):%d\n", g_gpio_rst_n);
	result = gpio_request(g_gpio_rst_n, "dtv_rst_n");
	if(result){
		FTDTV_ERR_PRINTK("unable to request dtv_rst_n gpio [%d] : err %d\n", g_gpio_rst_n, result);
		return result;
	}
	FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_RST_N_GPIO):%d %d\n", g_gpio_rst_n, 0);
	result = gpio_direction_output(g_gpio_rst_n, 0);
	if(result){
		FTDTV_ERR_PRINTK("unable to set_direction for dtv_rst_n gpio [%d]\n", g_gpio_rst_n);
		return result;
	}

	FTDTV_DBG2_PRINTK("gpio_request(FTDTV_NPD_GPIO):%d\n", g_gpio_npd);
	result = gpio_request(g_gpio_npd, "dtv_npd");
	if(result){
		FTDTV_ERR_PRINTK("unable to request dtv_npd gpio [%d]\n", g_gpio_npd);
		return result;
	}
	FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_NPD_GPIO):%d %d\n", g_gpio_npd, 0);
	result = gpio_direction_output(g_gpio_npd, 0);
	if(result){
		FTDTV_ERR_PRINTK("unable to set_direction for dtv_npd gpio [%d]\n", g_gpio_npd);
		return result;
	}

#ifndef DTV_FJFEAT_PRODUCT_FJDEV038
	FTDTV_DBG2_PRINTK("gpio_request(FTDTV_18V_ON_GPIO):%d\n", g_gpio_18v_on);
	result = gpio_request(g_gpio_18v_on, "dtv_18v_on");
	if(result){
		FTDTV_ERR_PRINTK("unable to request dtv_18v_on gpio [%d]\n", g_gpio_18v_on);
		return result;
	}
	FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_18V_ON_GPIO):%d %d\n", g_gpio_18v_on, 0);
	result = gpio_direction_output(g_gpio_18v_on, 0);
	if(result){
		FTDTV_ERR_PRINTK("unable to set_direction for dtv_18v_on gpio [%d]\n", g_gpio_18v_on);
		return result;
	}
#endif

	FTDTV_DBG2_PRINTK("gpio_request(FTDTV_11V_ON_GPIO):%d\n", g_gpio_11v_on);
	result = gpio_request(g_gpio_11v_on, "dtv_11v_on");
	if(result){
		FTDTV_ERR_PRINTK("unable to request dtv_vddl gpio [%d] : err %d\n", g_gpio_11v_on, result);
		return result;
	}
	FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_11V_ON_GPIO):%d %d\n", g_gpio_11v_on, 0);
	result = gpio_direction_output(g_gpio_11v_on, 0);
	if(result){
		FTDTV_ERR_PRINTK("unable to set_direction for dtv_vddl gpio [%d]\n", g_gpio_11v_on);
		return result;
	}


#else
	if (ftdtv_hard_type == FTDTV_HARD_TYPE_F) {
		/* XRESET */
		FTDTV_DBG2_PRINTK("gpio_request(FTDTV_XRESET_GPIO):%d\n", FTDTV_XRESET_GPIO);
		result = gpio_request(FTDTV_XRESET_GPIO, "dtv_xreset");
		if(result){
			FTDTV_ERR_PRINTK("unable to request dtv_xreset gpio [%d]\n", FTDTV_XRESET_GPIO);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_XRESET_GPIO):%d %d\n", FTDTV_XRESET_GPIO, 0);
		result = gpio_direction_output(FTDTV_XRESET_GPIO, 0);
		if(result){
			FTDTV_ERR_PRINTK("unable to set_direction for dtv_xreset gpio [%d]\n", FTDTV_XRESET_GPIO);
			return result;
		}

		if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
			/* PM8941 GPIO */
			FTDTV_DBG2_PRINTK("qpnp_pin_map(FTDTV_ANT_28V_PON_GPIO):%d\n",FTDTV_ANT_28V_PON_GPIO);
			result = qpnp_pin_map("pm8941-gpio", FTDTV_ANT_28V_PON_GPIO);
			if (result < 0) {
				FTDTV_ERR_PRINTK("unable to qpnp_pin_map dtv_ant_28v_on gpio [%d]\n", result);
				return result;
			}
			ftdtv_ant_28v_on_gpio_no = result;
			FTDTV_DBG2_PRINTK("qpnp_pin_config(ftdtv_ant_28v_on_gpio_no):%d\n", ftdtv_ant_28v_on_gpio_no);
			result = qpnp_pin_config(ftdtv_ant_28v_on_gpio_no, &ftdtv_ant_pin_28v_cfg_param);
			if(result){
				FTDTV_ERR_PRINTK(" unable to qpnp_pin_config dtv_ant_28v_on gpio [%d]\n", result);
				return result;
			}
			FTDTV_DBG2_PRINTK("gpio_request(ftdtv_ant_28v_on_gpio_no):%d\n", ftdtv_ant_28v_on_gpio_no);
			result = gpio_request(ftdtv_ant_28v_on_gpio_no, "dtv_28v_on");
			if(result){
				FTDTV_ERR_PRINTK("unable to request dtv_ant_28v_on gpio [%d]\n", ftdtv_ant_28v_on_gpio_no);
				return result;
			}
			FTDTV_DBG2_PRINTK("gpio_direction_output(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
			result = gpio_direction_output(ftdtv_ant_28v_on_gpio_no, 0);
			if(result){
				FTDTV_ERR_PRINTK("unable to set_direction for dtv_ant_28v_on gpio [%d]\n", ftdtv_ant_28v_on_gpio_no);
				return result;
			}
		} else {
			ftdtv_ant_28v_on_gpio_no = FTDTV_ANT_28V_ON_GPIO;
			FTDTV_DBG2_PRINTK("gpio_request(ftdtv_ant_28v_on_gpio_no):%d\n", ftdtv_ant_28v_on_gpio_no);
			result = gpio_request(ftdtv_ant_28v_on_gpio_no, "dtv_ant_28v_on");
			if(result){
				FTDTV_ERR_PRINTK("unable to request dtv_ant_28v_on gpio [%d]\n", ftdtv_ant_28v_on_gpio_no);
				return result;
			}
			FTDTV_DBG2_PRINTK("gpio_direction_output(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
			result = gpio_direction_output(ftdtv_ant_28v_on_gpio_no, 0);
			if(result){
				FTDTV_ERR_PRINTK("unable to set_direction for dtv_ant_28v_on gpio [%d]\n", ftdtv_ant_28v_on_gpio_no);
				return result;
			}
		}

		FTDTV_DBG2_PRINTK("qpnp_pin_map(FTDTV_28V_ON_GPIO):%d\n", FTDTV_28V_ON_GPIO);
		result = qpnp_pin_map("pm8941-gpio", FTDTV_28V_ON_GPIO);
		if (result < 0) {
			FTDTV_ERR_PRINTK("unable to qpnp_pin_map dtv_28v_on gpio [%d]\n", result);
			return result;
		}
		ftdtv_28v_on_gpio_no = result;
		FTDTV_DBG2_PRINTK("qpnp_pin_config(ftdtv_28v_on_gpio_no):%d\n", ftdtv_28v_on_gpio_no);
		result = qpnp_pin_config(ftdtv_28v_on_gpio_no, &ftdtv_pin_28v_cfg_param);
		if(result){
			FTDTV_ERR_PRINTK("unable to qpnp_pin_config dtv_28v_on gpio [%d]\n", result);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_request(ftdtv_28v_on_gpio_no):%d\n", ftdtv_28v_on_gpio_no);
		result = gpio_request(ftdtv_28v_on_gpio_no, "dtv_28v_on");
		if(result){
			FTDTV_ERR_PRINTK("unable to request dtv_28v_on gpio [%d]\n", ftdtv_28v_on_gpio_no);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_direction_output(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 0);
		result = gpio_direction_output(ftdtv_28v_on_gpio_no, 0);
		if(result){
			FTDTV_ERR_PRINTK("unable to set_direction for dtv_28v_on gpio [%d]\n", ftdtv_28v_on_gpio_no);
			return result;
		}

		FTDTV_DBG2_PRINTK("qpnp_pin_map(FTDTV_18V_ON_GPIO):%d\n", FTDTV_18V_ON_GPIO);
		result = qpnp_pin_map("pm8941-gpio", FTDTV_18V_ON_GPIO);
		if (result < 0) {
			FTDTV_ERR_PRINTK("unable to qpnp_pin_map dtv_18v_on gpio [%d]\n", result);
			return result;
		}
		ftdtv_18v_on_gpio_no = result;
		FTDTV_DBG2_PRINTK("qpnp_pin_config(ftdtv_18v_on_gpio_no):%d\n", ftdtv_18v_on_gpio_no);
		result = qpnp_pin_config(ftdtv_18v_on_gpio_no, &ftdtv_pin_18v_cfg_param);
		if(result){
			FTDTV_ERR_PRINTK("unable to qpnp_pin_config dtv_18v_on gpio [%d]\n", result);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_request(ftdtv_18v_on_gpio_no):%d\n", ftdtv_18v_on_gpio_no);
		result = gpio_request(ftdtv_18v_on_gpio_no, "dtv_18v_on");
		if(result){
			FTDTV_ERR_PRINTK("unable to request dtv_18v_on gpio [%d]\n", ftdtv_18v_on_gpio_no);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_direction_output(ftdtv_18v_on_gpio_no):%d %d\n", ftdtv_18v_on_gpio_no, 0);
		result = gpio_direction_output(ftdtv_18v_on_gpio_no, 0);
		if(result){
			FTDTV_ERR_PRINTK("unable to set_direction for dtv_18v_on gpio [%d]\n", ftdtv_18v_on_gpio_no);
			return result;
		}

		/* Antenna Select GPIO */
		FTDTV_DBG2_PRINTK("gpio_request(FTDTV_ANT_SELECT1_GPIO):%d\n", FTDTV_ANT_SELECT1_GPIO);
		result = gpio_request(FTDTV_ANT_SELECT1_GPIO, "dtv_ant_select1");
		if(result){
			FTDTV_ERR_PRINTK("unable to request dtv_ant_select1 gpio [%d]\n", FTDTV_ANT_SELECT1_GPIO);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
		result = gpio_direction_output(FTDTV_ANT_SELECT1_GPIO, 0);
		if(result){
			FTDTV_ERR_PRINTK("unable to set_direction for dtv_ant_select1 gpio [%d]\n", FTDTV_ANT_SELECT1_GPIO);
			return result;
		}

		if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
			if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
				FTDTV_DBG2_PRINTK("gpio_request(FTDTV_ANT_SELECT2_GPIO):%d\n", ftdtv_ant_select2_gpio_no);
				result = gpio_request(ftdtv_ant_select2_gpio_no, "dtv_ant_select2");
				if(result){
					FTDTV_ERR_PRINTK("unable to request dtv_ant_select2 gpio [%d]\n", ftdtv_ant_select2_gpio_no);
					return result;
				}
				FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 0);
				result = gpio_direction_output(ftdtv_ant_select2_gpio_no, 0);
				if(result){
					FTDTV_ERR_PRINTK("unable to set_direction for dtv_ant_select2 gpio [%d]\n", ftdtv_ant_select2_gpio_no);
					return result;
				}
			} else {
				FTDTV_DBG_PRINTK("FTDTV_ANT_SELECT2_GPIO Not gpio_request\n");
			}
		}

	} else {	/* FTDTV_HARD_TYPE_P */
		FTDTV_DBG2_PRINTK("gpio_request(FTDTV_XRESET_GPIO):%d\n", FTDTV_XRESET_GPIO);
		result = gpio_request(FTDTV_XRESET_GPIO, "dtv_xreset");
		if(result){
			FTDTV_ERR_PRINTK("unable to request dtv_xreset gpio [%d]\n", FTDTV_XRESET_GPIO);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_XRESET_GPIO):%d %d\n", FTDTV_XRESET_GPIO, 0);
		result = gpio_direction_output(FTDTV_XRESET_GPIO, 0);
		if(result){
			FTDTV_ERR_PRINTK("unable to set_direction for dtv_xreset gpio [%d]\n", FTDTV_XRESET_GPIO);
			return result;
		}

		FTDTV_DBG2_PRINTK("gpio_request(FTDTV_NPD_GPIO):%d\n", FTDTV_NPD_GPIO);
		result = gpio_request(FTDTV_NPD_GPIO, "dtv_xreset");
		if(result){
			FTDTV_ERR_PRINTK("unable to request dtv_npd gpio [%d]\n", FTDTV_NPD_GPIO);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_NPD_GPIO):%d %d\n", FTDTV_NPD_GPIO, 0);
		result = gpio_direction_output(FTDTV_NPD_GPIO, 0);
		if(result){
			FTDTV_ERR_PRINTK("unable to set_direction for dtv_npd gpio [%d]\n", FTDTV_NPD_GPIO);
			return result;
		}

		if (ftdtv_hard_mode != FTDTV_HARD_MODE_1) {
			if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
				FTDTV_DBG2_PRINTK("qpnp_pin_map(FTDTV_ANT_28V_PON_GPIO):%d\n",FTDTV_ANT_28V_PON_GPIO);
				result = qpnp_pin_map("pm8941-gpio", FTDTV_ANT_28V_PON_GPIO);
				if (result < 0) {
					FTDTV_ERR_PRINTK("unable to qpnp_pin_map dtv_ant_28v_on gpio [%d]\n", result);
					return result;
				}
				ftdtv_ant_28v_on_gpio_no = result;
				FTDTV_DBG2_PRINTK("qpnp_pin_config(ftdtv_ant_28v_on_gpio_no):%d\n", ftdtv_ant_28v_on_gpio_no);
				result = qpnp_pin_config(ftdtv_ant_28v_on_gpio_no, &ftdtv_ant_pin_28v_cfg_param);
				if(result){
					FTDTV_ERR_PRINTK(" unable to qpnp_pin_config dtv_ant_28v_on gpio [%d]\n", result);
					return result;
				}
				FTDTV_DBG2_PRINTK("gpio_request(ftdtv_ant_28v_on_gpio_no):%d\n", ftdtv_ant_28v_on_gpio_no);
				result = gpio_request(ftdtv_ant_28v_on_gpio_no, "dtv_28v_on");
				if(result){
					FTDTV_ERR_PRINTK("unable to request dtv_ant_28v_on gpio [%d]\n", ftdtv_ant_28v_on_gpio_no);
					return result;
				}
				FTDTV_DBG2_PRINTK("gpio_direction_output(ftdtv_ant_28v_on_gpio_no):%d %d\n", ftdtv_ant_28v_on_gpio_no, 0);
				result = gpio_direction_output(ftdtv_ant_28v_on_gpio_no, 0);
				if(result){
					FTDTV_ERR_PRINTK("unable to set_direction for dtv_ant_28v_on gpio [%d]\n", ftdtv_ant_28v_on_gpio_no);
					return result;
				}
			}

			FTDTV_DBG2_PRINTK("qpnp_pin_map(FTDTV_28V_ON_GPIO):%d\n", FTDTV_28V_ON_GPIO);
			result = qpnp_pin_map("pm8941-gpio", FTDTV_28V_ON_GPIO);
			if (result < 0) {
				FTDTV_ERR_PRINTK("unable to qpnp_pin_map dtv_28v_on gpio [%d]\n", result);
				return result;
			}
			ftdtv_28v_on_gpio_no = result;
			FTDTV_DBG2_PRINTK("qpnp_pin_config(ftdtv_28v_on_gpio_no):%d\n", ftdtv_28v_on_gpio_no);
			result = qpnp_pin_config(ftdtv_28v_on_gpio_no, &ftdtv_pin_28v_cfg_param);
			if(result){
				FTDTV_ERR_PRINTK("unable to qpnp_pin_config dtv_28v_on gpio [%d]\n", result);
				return result;
			}
			FTDTV_DBG2_PRINTK("gpio_request(ftdtv_28v_on_gpio_no):%d\n", ftdtv_28v_on_gpio_no);
			result = gpio_request(ftdtv_28v_on_gpio_no, "dtv_28v_on");
			if(result){
				FTDTV_ERR_PRINTK("unable to request dtv_28v_on gpio [%d]\n", ftdtv_28v_on_gpio_no);
				return result;
			}
			FTDTV_DBG2_PRINTK("gpio_direction_output(ftdtv_28v_on_gpio_no):%d %d\n", ftdtv_28v_on_gpio_no, 0);
			result = gpio_direction_output(ftdtv_28v_on_gpio_no, 0);
			if(result){
				FTDTV_ERR_PRINTK("unable to set_direction for dtv_28v_on gpio [%d]\n", ftdtv_28v_on_gpio_no);
				return result;
			}
		}

		FTDTV_DBG2_PRINTK("qpnp_pin_map(FTDTV_18V_ON_GPIO):%d\n", FTDTV_18V_ON_GPIO);
		result = qpnp_pin_map("pm8941-gpio", FTDTV_18V_ON_GPIO);
		if (result < 0) {
			FTDTV_ERR_PRINTK("unable to qpnp_pin_map dtv_18v_on gpio [%d]\n", result);
			return result;
		}
		ftdtv_18v_on_gpio_no = result;
		FTDTV_DBG2_PRINTK("qpnp_pin_config(ftdtv_18v_on_gpio_no):%d\n", ftdtv_18v_on_gpio_no);
		result = qpnp_pin_config(ftdtv_18v_on_gpio_no, &ftdtv_pin_18v_cfg_param);
		if(result){
			FTDTV_ERR_PRINTK("unable to qpnp_pin_config dtv_18v_on gpio [%d]\n", result);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_request(ftdtv_18v_on_gpio_no):%d\n", ftdtv_18v_on_gpio_no);
		result = gpio_request(ftdtv_18v_on_gpio_no, "dtv_18v_on");
		if(result){
			FTDTV_ERR_PRINTK("unable to request dtv_18v_on gpio [%d]\n", ftdtv_18v_on_gpio_no);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_direction_output(ftdtv_18v_on_gpio_no):%d %d\n", ftdtv_18v_on_gpio_no, 0);
		result = gpio_direction_output(ftdtv_18v_on_gpio_no, 0);
		if(result){
			FTDTV_ERR_PRINTK("unable to set_direction for dtv_18v_on gpio [%d]\n", ftdtv_18v_on_gpio_no);
			return result;
		}

		FTDTV_DBG2_PRINTK("gpio_request(FTDTV_ANT_SELECT1_GPIO):%d\n", FTDTV_ANT_SELECT1_GPIO);
		result = gpio_request(FTDTV_ANT_SELECT1_GPIO, "dtv_ant_select1");
		if(result){
			FTDTV_ERR_PRINTK("unable to request dtv_ant_select1 gpio [%d]\n", FTDTV_ANT_SELECT1_GPIO);
			return result;
		}
		FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_ANT_SELECT1_GPIO):%d %d\n", FTDTV_ANT_SELECT1_GPIO, 0);
		result = gpio_direction_output(FTDTV_ANT_SELECT1_GPIO, 0);
		if(result){
			FTDTV_ERR_PRINTK("unable to set_direction for dtv_ant_select1 gpio [%d]\n", FTDTV_ANT_SELECT1_GPIO);
			return result;
		}

		if (ftdtv_hard_level == FTDTV_HARD_LEVEL1) {
			if (ftdtv_hard_mode == FTDTV_HARD_MODE_0) {
				FTDTV_DBG2_PRINTK("gpio_request(FTDTV_ANT_SELECT2_GPIO):%d\n", ftdtv_ant_select2_gpio_no);
				result = gpio_request(ftdtv_ant_select2_gpio_no, "dtv_ant_select2");
				if(result){
					FTDTV_ERR_PRINTK("unable to request dtv_ant_select2 gpio [%d]\n", ftdtv_ant_select2_gpio_no);
					return result;
				}
				FTDTV_DBG2_PRINTK("gpio_direction_output(FTDTV_ANT_SELECT2_GPIO):%d %d\n", ftdtv_ant_select2_gpio_no, 0);
				result = gpio_direction_output(ftdtv_ant_select2_gpio_no, 0);
				if(result){
					FTDTV_ERR_PRINTK("unable to set_direction for dtv_ant_select2 gpio [%d]\n", ftdtv_ant_select2_gpio_no);
					return result;
				}
			}
		}
	}
#endif

#if 0
	for (i = 0; i < VREG_TABLE; ++i) {
		vreg_info[i].reg = regulator_get(NULL, vreg_info[i].name);
		FTDTV_DBG2_PRINTK("regulator_get() name=%s level=%d\n", vreg_info[i].name, vreg_info[i].lvl);
		if (IS_ERR(vreg_info[i].reg)) {
			FTDTV_ERR_PRINTK("vreg_get(%s) failed (%ld) !\n", vreg_info[i].name, PTR_ERR(vreg_info[i].reg));
			return -1;
			//break;
		}
	}
	for (i = 0; i < VREG_TABLE; ++i) {
		FTDTV_DBG2_PRINTK("regulator_set_voltage() name=%s level=%d\n", vreg_info[i].name, vreg_info[i].lvl);
		if (regulator_set_voltage(vreg_info[i].reg, vreg_info[i].lvl, vreg_info[i].lvl)) {
			FTDTV_ERR_PRINTK("%s set level failed (%d) !\n", vreg_info[i].name, vreg_info[i].lvl);
			return -1;
		}
	}
#endif

	wake_lock_init(&ftdtv_wake_lock, WAKE_LOCK_SUSPEND,"ftdtv");

	FTDTV_INFO_PRINTK("-E\n");

	return 0;
}

/*---------------------------------------------------------------------------
    dtv_ant_usb_insert_detect
---------------------------------------------------------------------------*/
void dtv_ant_usb_insert_detect(void)
{
	FTDTV_INFO_PRINTK("-S\n");

	g_antenna = 1;

	FTDTV_INFO_PRINTK("-E\n");
}
EXPORT_SYMBOL_GPL(dtv_ant_usb_insert_detect);

/*---------------------------------------------------------------------------
    dtv_ant_usb_remove_detect
---------------------------------------------------------------------------*/
void dtv_ant_usb_remove_detect(void)
{
	FTDTV_INFO_PRINTK("-S\n");

	g_antenna = 0;

	FTDTV_INFO_PRINTK("-E\n");
}
EXPORT_SYMBOL_GPL(dtv_ant_usb_remove_detect);

/// add S
//FUJITSU:2013-04-25 For Panasonic tuner Add -S
#ifdef TUNERX_DEBUG
//FUJITSU:2013-04-25 For Panasonic tuner Add -E
int ftdtv_get_state(void)
{
	enum {VDDA=0,VDDL};
	memset(&g_powerShowParam,0,sizeof(g_powerShowParam));
	FTDTV_DBG_PRINTK("-S\n");

#if 0
	// L21 VDDA
	if(regulator_is_enabled(vreg_info[2].reg))
		g_powerShowParam.state |= STATE_VDDA;
	else
		g_powerShowParam.state &= ~STATE_VDDA;
	g_powerShowParam.voltage[VDDA] = regulator_get_voltage(vreg_info[2].reg);
	// L12 VDDL
	if(regulator_is_enabled(vreg_info[1].reg))
		g_powerShowParam.state |= STATE_VDDL;
	else
		g_powerShowParam.state &= ~STATE_VDDL;
	g_powerShowParam.voltage[VDDL] = regulator_get_voltage(vreg_info[1].reg);
#endif
#if 0
	// GPIO13 XRESET
	if(gpio_get_value_cansleep(FTDTV_XRESET_GPIO))
		g_powerShowParam.state |= STATE_NRST;
	else
		g_powerShowParam.state &= ~STATE_NRST;
#endif
	FTDTV_DBG_PRINTK("State:%d, VDDA:%d, VDDL:%d -E\n",
		   g_powerShowParam.state,g_powerShowParam.voltage[VDDA],g_powerShowParam.voltage[VDDL]);

	return g_powerShowParam.state;
}
//FUJITSU:2013-04-25 For Panasonic tuner Add -S
#endif //TUNERX_DEBUG
//FUJITSU:2013-04-25 For Panasonic tuner Add -E
/// add E

module_init(ftdtv_init);
module_exit(ftdtv_exit);
MODULE_LICENSE("GPL");
