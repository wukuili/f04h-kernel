/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016-2017
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2016
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "SMBCHG: %s: " fmt, __func__

#include <linux/spmi.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/rtc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#include <linux/msm_bcl.h>
#include <linux/ktime.h>
/* FUJITSU LIMITED:2016-01-05 CHARGER add start */
#include "fj_charger_local.h"
/* FUJITSU LIMITED:2016-01-05 CHARGER add end */

/* Mask/Bit helpers */
#define _SMB_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))
/* Config registers */
struct smbchg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct parallel_usb_cfg {
	struct power_supply		*psy;
	int				min_current_thr_ma;
	int				min_9v_current_thr_ma;
	int				allowed_lowering_ma;
	int				current_max_ma;
	bool				avail;
	struct mutex			lock;
	int				initial_aicl_ma;
	ktime_t				last_disabled;
	bool				enabled_once;
};

struct ilim_entry {
	int vmin_uv;
	int vmax_uv;
	int icl_pt_ma;
	int icl_lv_ma;
	int icl_hv_ma;
};

struct ilim_map {
	int			num;
	struct ilim_entry	*entries;
};

struct smbchg_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	int				schg_version;

	/* peripheral register address bases */
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				otg_base;
	u16				misc_base;

	int				fake_battery_soc;
	u8				revision[4];

	/* configuration parameters */
	int				iterm_ma;
	int				usb_max_current_ma;
	int				dc_max_current_ma;
	int				usb_target_current_ma;
	int				usb_tl_current_ma;
	int				dc_target_current_ma;
	int				target_fastchg_current_ma;
	int				cfg_fastchg_current_ma;
	int				fastchg_current_ma;
	int				vfloat_mv;
	int				fastchg_current_comp;
	int				float_voltage_comp;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				bmd_pin_src;
	int				jeita_temp_hard_limit;
	bool				use_vfloat_adjustments;
	bool				iterm_disabled;
	bool				bmd_algo_disabled;
	bool				soft_vfloat_comp_disabled;
	bool				chg_enabled;
	bool				charge_unknown_battery;
	bool				chg_inhibit_en;
	bool				chg_inhibit_source_fg;
	bool				low_volt_dcin;
	bool				vbat_above_headroom;
	bool				force_aicl_rerun;
	bool				enable_hvdcp_9v;
	u8				original_usbin_allowance;
	struct parallel_usb_cfg		parallel;
	struct delayed_work		parallel_en_work;
	struct dentry			*debug_root;

	/* wipower params */
	struct ilim_map			wipower_default;
	struct ilim_map			wipower_pt;
	struct ilim_map			wipower_div2;
	struct qpnp_vadc_chip		*vadc_dev;
	bool				wipower_dyn_icl_avail;
	struct ilim_entry		current_ilim;
	struct mutex			wipower_config;
	bool				wipower_configured;
	struct qpnp_adc_tm_btm_param	param;

	/* flash current prediction */
	int				rpara_uohm;
	int				rslow_uohm;
	int				vled_max_uv;

	/* vfloat adjustment */
	int				max_vbat_sample;
	int				n_vbat_samples;

	/* status variables */
	int				battchg_disabled;
	int				usb_suspended;
	int				dc_suspended;
	int				wake_reasons;
	int				previous_soc;
	int				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	int				otg_retries;
	ktime_t				otg_enable_time;
	bool				aicl_deglitch_short;
	bool				sw_esr_pulse_en;
	bool				safety_timer_en;
	bool				aicl_complete;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	bool				very_weak_charger;
	bool				parallel_charger_detected;
	const char			*battery_type;
	u32				wa_flags;

	/* jeita and temperature */
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	/* irqs */
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int				recharge_irq;
	int				fastchg_irq;
	int				safety_timeout_irq;
	int				power_ok_irq;
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
	int				usbid_change_irq;
	int				chg_error_irq;
	bool				enable_aicl_wake;

	/* psy */
	struct power_supply		*usb_psy;
	struct power_supply		batt_psy;
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	int				dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	struct smbchg_regulator		otg_vreg;
	struct smbchg_regulator		ext_otg_vreg;
	struct work_struct		usb_set_online_work;
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		hvdcp_det_work;
	spinlock_t			sec_access_lock;
	struct mutex			current_change_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			battchg_disabled_lock;
	struct mutex			usb_en_lock;
	struct mutex			dc_en_lock;
	struct mutex			fcc_lock;
	struct mutex			pm_lock;
	/* aicl deglitch workaround */
	unsigned long			first_aicl_seconds;
	int				aicl_irq_count;
	struct mutex			usb_status_lock;
};

enum qpnp_schg {
	QPNP_SCHG,
	QPNP_SCHG_LITE,
};

static char *version_str[] = {
	[QPNP_SCHG]		= "SCHG",
	[QPNP_SCHG_LITE]	= "SCHG_LITE",
};

enum pmic_subtype {
	PMI8994		= 10,
	PMI8950		= 17,
};

enum smbchg_wa {
	SMBCHG_AICL_DEGLITCH_WA = BIT(0),
	SMBCHG_HVDCP_9V_EN_WA	= BIT(1),
};

enum print_reason {
	PR_REGISTER	= BIT(0),
	PR_INTERRUPT	= BIT(1),
	PR_STATUS	= BIT(2),
	PR_DUMP		= BIT(3),
	PR_PM		= BIT(4),
	PR_MISC		= BIT(5),
	PR_WIPOWER	= BIT(6),
};

enum wake_reason {
	PM_PARALLEL_CHECK = BIT(0),
	PM_REASON_VFLOAT_ADJUST = BIT(1),
	PM_ESR_PULSE = BIT(2),
	PM_PARALLEL_TAPER = BIT(3),
};

/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
static int smbchg_debug_mask;
#else
static int smbchg_debug_mask = 0x02;
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
module_param_named(
	debug_mask, smbchg_debug_mask, int, S_IRUSR | S_IWUSR
);

/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
static int smbchg_parallel_en = 1;
#else
static int smbchg_parallel_en = 0;
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
module_param_named(
	parallel_en, smbchg_parallel_en, int, S_IRUSR | S_IWUSR
);

static int wipower_dyn_icl_en;
module_param_named(
	dynamic_icl_wipower_en, wipower_dyn_icl_en,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dcin_interval = ADC_MEAS1_INTERVAL_2P0MS;
module_param_named(
	wipower_dcin_interval, wipower_dcin_interval,
	int, S_IRUSR | S_IWUSR
);

#define WIPOWER_DEFAULT_HYSTERISIS_UV	250000
static int wipower_dcin_hyst_uv = WIPOWER_DEFAULT_HYSTERISIS_UV;
module_param_named(
	wipower_dcin_hyst_uv, wipower_dcin_hyst_uv,
	int, S_IRUSR | S_IWUSR
);

#define pr_smb(reason, fmt, ...)				\
	do {							\
		if (smbchg_debug_mask & (reason))		\
			pr_info(fmt, ##__VA_ARGS__);		\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)

#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		if (smbchg_debug_mask & (reason))			\
			pr_info_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
			pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)

/* FUJITSU LIMITED:2016-01-05 CHARGER add start */
static struct smbchg_chip *the_chip = NULL;

typedef enum {
	CHGR_BASE_ITEM = 0,
	BAT_IF_BASE_ITEM,
	USB_CHGPTH_BASE_ITEM,
	DC_CHGPTH_BASE_ITEM,
	MISC_BASE_ITEM,
	OTG_BASE_ITEM,
} base_item;

struct chg_register_table {
	base_item		base;
	u16				offset;
	bool			sec_addr;
};

static struct chg_register_table qpnp_charger_register_table[FJ_CHG_CMD_NUM] = {
	{ CHGR_BASE_ITEM,			0x0E,		false	},
	{ CHGR_BASE_ITEM,			0xF1,		true	},
	{ CHGR_BASE_ITEM,			0xF2,		true	},
	{ CHGR_BASE_ITEM,			0xF4,		true	},
	{ CHGR_BASE_ITEM,			0xF8,		true	},
	{ CHGR_BASE_ITEM,			0xF9,		true	},
	{ CHGR_BASE_ITEM,			0xFA,		true	},
	{ CHGR_BASE_ITEM,			0xFC,		true	},
	{ CHGR_BASE_ITEM,			0xFD,		true	},
	{ BAT_IF_BASE_ITEM,			0x08,		false	},
	{ BAT_IF_BASE_ITEM,			0x10,		false	},
	{ BAT_IF_BASE_ITEM,			0x42,		false	},
	{ BAT_IF_BASE_ITEM,			0xF2,		true	},
	{ BAT_IF_BASE_ITEM,			0xF3,		true	},
	{ USB_CHGPTH_BASE_ITEM,		0x07,		false	},
	{ USB_CHGPTH_BASE_ITEM,		0x0C,		false	},
	{ USB_CHGPTH_BASE_ITEM,		0x10,		false	},
	{ USB_CHGPTH_BASE_ITEM,		0x40,		false	},
	{ USB_CHGPTH_BASE_ITEM,		0x4E,		false	},
	{ USB_CHGPTH_BASE_ITEM,		0xF1,		true	},
	{ USB_CHGPTH_BASE_ITEM,		0xF2,		true	},
	{ USB_CHGPTH_BASE_ITEM,		0xF3,		true	},
	{ USB_CHGPTH_BASE_ITEM,		0xF4,		true	},
	{ USB_CHGPTH_BASE_ITEM,		0xF5,		true	},
	{ DC_CHGPTH_BASE_ITEM,		0x10,		false	},
	{ DC_CHGPTH_BASE_ITEM,		0xF1,		true	},
	{ DC_CHGPTH_BASE_ITEM,		0xF2,		true	},
	{ DC_CHGPTH_BASE_ITEM,		0xF3,		true	},
	{ DC_CHGPTH_BASE_ITEM,		0xF4,		true	},
	{ DC_CHGPTH_BASE_ITEM,		0xF5,		true	},
	{ MISC_BASE_ITEM,			0x40,		false	},
	{ MISC_BASE_ITEM,			0xF1,		true	},
	{ MISC_BASE_ITEM,			0xF5,		true	},
	{ MISC_BASE_ITEM,			0xFF,		true	},
};

#define GET_BASE_ADDRESS(cmd)	(qpnp_charger_register_table[cmd].base == CHGR_BASE_ITEM ? the_chip->chgr_base : \
								 qpnp_charger_register_table[cmd].base == BAT_IF_BASE_ITEM ? the_chip->bat_if_base : \
								 qpnp_charger_register_table[cmd].base == USB_CHGPTH_BASE_ITEM ? the_chip->usb_chgpth_base : \
								 qpnp_charger_register_table[cmd].base == DC_CHGPTH_BASE_ITEM ? the_chip->dc_chgpth_base : \
								 qpnp_charger_register_table[cmd].base == MISC_BASE_ITEM ? the_chip->misc_base : \
								 qpnp_charger_register_table[cmd].base == OTG_BASE_ITEM ? the_chip->otg_base : 0)

#define CHECK_BLOCK_ADDESS(chip, addr)	(addr == (chip->chgr_base + 0xF2) ? 1 : \
										 addr == (chip->usb_chgpth_base + 0xF2) ? 1 : \
										 addr == (chip->usb_chgpth_base + 0xF3) ? 1 : \
										 addr == (chip->dc_chgpth_base + 0xF3) ? 1 : \
										 addr == (chip->dc_chgpth_base + 0xF2) ? 1 : 0)
/* FUJITSU LIMITED:2016-01-05 CHARGER add end */

static int smbchg_read(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "spmi read failed addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return rc;
	}
	return 0;
}

/*
 * Writes an arbitrary number of bytes to a specified register
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_write(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

/* FUJITSU LIMITED:2016-01-05 CHARGER add start */
    if (CHECK_BLOCK_ADDESS(chip, addr)) {
        pr_smb(PR_REGISTER, "[%s] write block. addr:[0x%x] data:[0x%x]\n",
            __func__, addr, *val);
        return 0;
    }
/* FUJITSU LIMITED:2016-01-05 CHARGER add end */

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "write failed addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_masked_write_raw(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi read failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;

	pr_smb(PR_REGISTER, "addr = 0x%x writing 0x%x\n", base, reg);

	rc = smbchg_write(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi write failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * This function holds a spin lock to ensure secure access register writes goes
 * through. If the secure access unlock register is armed, any old register
 * write can unarm the secure access unlock, causing the next write to fail.
 *
 * Note: do not use this for sec_access registers. Instead use the function
 * below: smbchg_sec_masked_write
 */
static int smbchg_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
								u8 val)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&chip->sec_access_lock, flags);
	rc = smbchg_masked_write_raw(chip, base, mask, val);
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);

	return rc;
}

/*
 * Unlocks sec access and writes to the register specified.
 *
 * This function holds a spin lock to exclude other register writes while
 * the two writes are taking place.
 */
#define SEC_ACCESS_OFFSET	0xD0
#define SEC_ACCESS_VALUE	0xA5
#define PERIPHERAL_MASK		0xFF
static int smbchg_sec_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	unsigned long flags;
	int rc;
	u16 peripheral_base = base & (~PERIPHERAL_MASK);

	spin_lock_irqsave(&chip->sec_access_lock, flags);

	rc = smbchg_masked_write_raw(chip, peripheral_base + SEC_ACCESS_OFFSET,
				SEC_ACCESS_VALUE, SEC_ACCESS_VALUE);
	if (rc) {
		dev_err(chip->dev, "Unable to unlock sec_access: %d", rc);
		goto out;
	}

	rc = smbchg_masked_write_raw(chip, base, mask, val);

out:
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);
	return rc;
}

/* FUJITSU LIMITED:2016-01-05 CHARGER add start */
int qpnp_charger_read(u16 cmd, u8 *val)
{
	u16 addr = 0;
	u16 base = 0;
	struct spmi_device *spmi;
	int ret;

	if (the_chip == NULL) {
		pr_err("[%s] Preparation is not complete.\n", __func__);
		return -EINVAL;
	}

	if (val == NULL) {
		dev_err(the_chip->dev, "[%s] Invalid parameter. val:[NULL]\n", __func__);
		return -EINVAL;
	}

	if (cmd >= FJ_CHG_CMD_NUM) {
		dev_err(the_chip->dev, "[%s] Invalid parameter. cmd:[%d]\n", __func__, cmd);
		return -EINVAL;
	}

	base = GET_BASE_ADDRESS(cmd);
	if (base == 0) {
		dev_err(the_chip->dev, "[%s] base address is invalid. cmd:[%d]\n", __func__, cmd);
		return -EINVAL;
	}

	spmi = the_chip->spmi;
	addr = base + qpnp_charger_register_table[cmd].offset;

	ret = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, 1);
	if (ret) {
		dev_err(the_chip->dev, "[%s] spmi read failed. addr:[0x%04X] sid:[0x%02X] ret:[%d]\n",
															__func__, addr, spmi->sid, ret);
		return ret;
	}

	pr_smb(PR_REGISTER, "[%s] read reg:[0x%04X] val:[0x%02X]\n", __func__, addr, *val);

	return ret;
}
EXPORT_SYMBOL_GPL(qpnp_charger_read);

int qpnp_charger_write(u16 cmd, u8 mask, u8 val)
{
	u16 base = 0;
	u16 addr = 0;
	u8 reg = 0;
	struct spmi_device *spmi;
	unsigned long flags;
	int ret;

	if (the_chip == NULL) {
		pr_err("[%s] Preparation is not complete.\n", __func__);
		return -EINVAL;
	}

	if (cmd >= FJ_CHG_CMD_NUM) {
		dev_err(the_chip->dev, "[%s] Invalid parameter. cmd:[%d]\n", __func__, cmd);
		return -EINVAL;
	}

	base = GET_BASE_ADDRESS(cmd);
	if (base == 0) {
		dev_err(the_chip->dev, "[%s] base address is invalid. cmd:[%d]\n", __func__, cmd);
		return -EINVAL;
	}

	spmi = the_chip->spmi;
	addr = base + qpnp_charger_register_table[cmd].offset;

	spin_lock_irqsave(&the_chip->sec_access_lock, flags);

	if (qpnp_charger_register_table[cmd].sec_addr) {
		ret = spmi_ext_register_readl(spmi->ctrl, spmi->sid, (base + SEC_ACCESS_OFFSET), &reg, 1);
		if (ret) {
			dev_err(the_chip->dev, "[%s] Unable to unlock sec_access. ret:[%d]\n",
																	__func__, ret);
			goto out;
		}

		reg &= ~SEC_ACCESS_VALUE;
		reg |= SEC_ACCESS_VALUE;

		ret = spmi_ext_register_writel(spmi->ctrl, spmi->sid, (base + SEC_ACCESS_OFFSET), &reg, 1);
		if (ret) {
			dev_err(the_chip->dev, "[%s] Unable to unlock sec_access. ret:[%d]\n", __func__, ret);
			goto out;
		}

		reg = 0;
	}

	ret = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, &reg, 1);
	if (ret) {
		dev_err(the_chip->dev, "[%s] spmi read failed. addr:[0x%04X] ret:[%d]\n",
																__func__, addr, ret);
		goto out;
	}

	reg &= ~mask;
	reg |= val & mask;

	ret = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, &reg, 1);
	if (ret) {
		dev_err(the_chip->dev, "[%s] spmi write failed. addr:[0x%04X] ret:[%d]\n",
															__func__, addr, ret);
		goto out;
	}

	pr_smb(PR_REGISTER, "[%s] write reg:[0x%04X] mask:[0x%02X] val:[0x%02X]\n",
													__func__, addr, mask, val);
out:
	spin_unlock_irqrestore(&the_chip->sec_access_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(qpnp_charger_write);
/* FUJITSU LIMITED:2016-01-05 CHARGER add end */

static void smbchg_stay_awake(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons | reason;
	if (reasons != 0 && chip->wake_reasons == 0) {
		pr_smb(PR_PM, "staying awake: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_stay_awake(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
}

static void smbchg_relax(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons & (~reason);
	if (reasons == 0 && chip->wake_reasons != 0) {
		pr_smb(PR_PM, "relaxing: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_relax(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
};

enum pwr_path_type {
	UNKNOWN = 0,
	PWR_PATH_BATTERY = 1,
	PWR_PATH_USB = 2,
	PWR_PATH_DC = 3,
};

#define PWR_PATH		0x08
#define PWR_PATH_MASK		0x03
static enum pwr_path_type smbchg_get_pwr_path(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + PWR_PATH, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read PWR_PATH rc = %d\n", rc);
		return PWR_PATH_BATTERY;
	}

	return reg & PWR_PATH_MASK;
}
/* FUJITSU LIMITED:2015-01-07 H1510087 add start */
#include <linux/qpnp/pin.h>
#include <linux/wakelock.h>
#include <linux/moduleparam.h>
#include <linux/mfd/fj_charger.h>
#include <linux/fj_bto.h>		/* FCNT LIMITED:2016-02-03 H16100722 add */
#include "fj_host_event.h"		/* FCNT LIMITED:2016-02-03 H16100722 add */
#include <linux/mfd/fj_det.h>	/* FCNT LIMITED:2016-02-12 H16100739 add */

extern int charging_mode;

/* FCNT LIMITED:2016-02-03 H16100722 add start */
#define FEATURE_OTG_SET_VBUS_IF
#ifdef FEATURE_OTG_SET_VBUS_IF
extern void dwc3_otg_set_fj_vbus(bool fj_vbus);
#endif
/* FCNT LIMITED:2016-02-03 H16100722 add end */

#define FJ_QPNP_RECLOG(fmt, ...) printk("REC@REC@0 " fmt, ##__VA_ARGS__)

/* #define FJ_GPIO_VBDET		3 *//* PMI8994 */	/* FCNT LIMITED:2016-02-03 H16100726 del */
/* #define FJ_GPIO_VCDET		4 *//* PMI8994 */	/* FCNT LIMITED:2016-02-03 H16100726 del */

#define FJ_DETECT_COUNT		30
#define FJ_DETECT_COUNT_MAX	600
#define FJ_DETECT_WAIT_MS	25
/* FCNT LIMITED:2016-02-03 H16100712 add start */
#define FJ_USB_HOT_DETECT_COUNT	1
/* FCNT LIMITED:2016-02-03 H16100712 add end */
#define FJ_RT_STS			0x10
#define FJ_RID_STS			0x0B
#define FJ_NT_LATCHED_STS	0x18
#define FJ_RT_OV_BIT		0x02 /* bit1 */
#define FJ_RT_USBID_BIT		0x40 /* bit6 */


#define FJ_RID_STS_GND				0x00
#define FJ_USBID_VALID_ID_11_8		0x0E
#define FJ_USBID_VALID_ID_7_0		0x0F

typedef enum {
	FJ_PM_OVP_NONE = 0,
	FJ_PM_OVP_DETECT,
	FJ_PM_OVP_INIT = 0xFF,
}FJ_OVP_TYPE;

typedef enum {
	FJ_STATUS_USB = 0,
	FJ_STATUS_DC,
}FJ_STATUS_CHATT;

typedef enum {
	FJ_OVP_USB = 1,
	FJ_OVP_DC,
}FJ_OVP_CHATT;

typedef enum {
	FJ_OVP_DEV_NONE = 0,
	FJ_OVP_DEV_USB,
	FJ_OVP_DEV_DC = 0xFF,
}FJ_OVP_DEVICE;

/* FUJITSU LIMITED:2016-01-12 H16100714 add start */
#define FJ_DEVICE_CHATTERING		255
#define FJ_DEVICE_PROBE_END			254
#define FJ_DEVICE_CHATTER_END		253
/* FUJITSU LIMITED:2016-01-12 H16100714 add end */

/* FCNT LIMITED:2016-02-12 H16100739 mod start */
typedef enum {
	FJ_NO_DEVICE	= FJ_DET_NO_DEVICE,
	FJ_DEVICE_OTG	= FJ_DET_DEVICE_OTG,
	FJ_DEVICE_1K	= FJ_DET_DEVICE_1K,
	FJ_DEVICE_36_5K	= FJ_DET_DEVICE_36_5K,
	FJ_DEVICE_68K	= FJ_DET_DEVICE_68K,
	FJ_DEVICE_124K	= FJ_DET_DEVICE_124K,
	FJ_DEVICE_180K	= FJ_DET_DEVICE_180K,
	FJ_DEVICE_390K	= FJ_DET_DEVICE_390K,
	FJ_DEVICE_1M	= FJ_DET_DEVICE_1M,
	FJ_DEVICE_UNKNOWN = FJ_DET_DEVICE_UNKNOWN
} FJ_ID_TYPE;
/* FCNT LIMITED:2016-02-12 H16100739 mod end */

typedef enum {
	FJ_DEV_ID_OTG = 0,
	FJ_DEV_ID_1K,
	FJ_DEV_ID_UNKNOWN_1,
	FJ_DEV_ID_36_5K,
	FJ_DEV_ID_UNKNOWN_2,
	FJ_DEV_ID_68K,
	FJ_DEV_ID_UNKNOWN_3,
	FJ_DEV_ID_124K,
	FJ_DEV_ID_UNKNOWN_4,
	FJ_DEV_ID_180K,
	FJ_DEV_ID_UNKNOWN_5,
	FJ_DEV_ID_390K,
	FJ_DEV_ID_UNKNOWN_6,
	FJ_DEV_ID_1M,
	FJ_DEV_ID_MAX,
}FJ_DEV_ID_TYPE;

#define FJ_ADC_OTG_MIN				(0x0000)
#define FJ_ADC_OTG_MAX				(0x0015)
#define FJ_ADC_1K_MIN				(0x0016)
#define FJ_ADC_1K_MAX				(0x013B)
#define FJ_ADC_36_5K_MIN			(0x0370)
#define FJ_ADC_36_5K_MAX			(0x0402)
#define FJ_ADC_68K_MIN				(0x0567)
#define FJ_ADC_68K_MAX				(0x0624)
#define FJ_ADC_124K_MIN				(0x07B5)
#define FJ_ADC_124K_MAX				(0x0882)
#define FJ_ADC_180K_MIN				(0x089D)
#define FJ_ADC_180K_MAX				(0x0A00)
#define FJ_ADC_390K_MIN				(0x0BEC)
#define FJ_ADC_390K_MAX				(0x0C80)
#define FJ_ADC_1M_MIN				(0x0E1E)
#define FJ_ADC_1M_MAX				(0x0FFF)

/* FCNT LIMITED:2016-02-03 H16100712 add start */
typedef enum {
	FJ_USB_SAFETY_ADP_SAFE = 0,
	FJ_USB_SAFETY_USB_TEMP,
	FJ_USB_SAFETY_MAX
} FJ_USB_SAFETY;
/* FCNT LIMITED:2016-02-03 H16100712 add end */
/* FCNT LIMITED:2016-02-03 H16100721 add start */
typedef enum {
	FJ_USBTEMP_ERROR,
	FJ_USBTEMP_NORMAL,
	FJ_USBTEMP_MAX
} FJ_USBTEMP_FUNC;
/* FCNT LIMITED:2016-02-03 H16100721 add end */

struct _fj_adc_to_device_state {
	FJ_DEV_ID_TYPE id_type;
	unsigned int id_value;
	FJ_ID_TYPE device;
};


static const struct _fj_adc_to_device_state fj_device_tbl[FJ_DEV_ID_MAX] = {
	{ FJ_DEV_ID_OTG,		FJ_ADC_OTG_MAX,			FJ_DEVICE_OTG	 	},
	{ FJ_DEV_ID_1K	,		FJ_ADC_1K_MAX,			FJ_DEVICE_1K	 	},
	{ FJ_DEV_ID_UNKNOWN_1,	(FJ_ADC_36_5K_MIN - 1), FJ_DEVICE_UNKNOWN	},
	{ FJ_DEV_ID_36_5K,		FJ_ADC_36_5K_MAX,	 	FJ_DEVICE_36_5K		},
	{ FJ_DEV_ID_UNKNOWN_2,	(FJ_ADC_68K_MIN - 1),	FJ_DEVICE_UNKNOWN	},
	{ FJ_DEV_ID_68K,		FJ_ADC_68K_MAX,			FJ_DEVICE_68K	 	},
	{ FJ_DEV_ID_UNKNOWN_3,	(FJ_ADC_124K_MIN - 1),	FJ_DEVICE_UNKNOWN	},
	{ FJ_DEV_ID_124K,		FJ_ADC_124K_MAX,		FJ_DEVICE_124K		},
	{ FJ_DEV_ID_UNKNOWN_4,	(FJ_ADC_180K_MIN - 1),	FJ_DEVICE_UNKNOWN	},
	{ FJ_DEV_ID_180K,		FJ_ADC_180K_MAX,		FJ_DEVICE_180K		},
	{ FJ_DEV_ID_UNKNOWN_5,	(FJ_ADC_390K_MIN - 1),	FJ_DEVICE_UNKNOWN	},
	{ FJ_DEV_ID_390K,		FJ_ADC_390K_MAX,		FJ_DEVICE_390K		},
	{ FJ_DEV_ID_UNKNOWN_6,	(FJ_ADC_1M_MIN - 1),	FJ_DEVICE_UNKNOWN	},
	{ FJ_DEV_ID_1M,			FJ_ADC_1M_MAX,			FJ_DEVICE_1M		},
};

unsigned int usb_id_info = FJ_DEVICE_CHATTERING;	/* FUJITSU LIMITED:2016-01-12 H16100714 mod */
unsigned int usb_temp = 0;							/* FCNT LIMITED:2016-02-03 H16100721 add */
unsigned int otg_on = 0;							/* FCNT LIMITED:2016-02-03 H16100722 add */
/* FCNT LIMITED:2016-02-12 H16100738 add start */
unsigned int vc_status = 0;
unsigned int vb_status = 0;
/* FCNT LIMITED:2016-02-12 H16100738 add end */
struct _fj_platform_data {
	FJ_ID_TYPE fj_id_device;			/* detect device */
	FJ_OVP_TYPE fj_ovp_state;			/* vbus state */
	bool fj_ext_charge;					/* ext usb charge */
	bool fj_mhl_offcharge;				/* mhl off charge */
/* FCNT LIMITED:2016-02-03 H16100726 add start */
	int vb_det_gpio;					/* VB detect gpio */
	int vc_det_gpio;					/* VC detect_gpio */
/* FCNT LIMITED:2016-02-03 H16100726 add end */
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	int adp_safe_gpio;					/* adp safe gpio */
	int usb_hot_gpio;					/* usb_hot gpio */
	int usb_thsel0;						/* usb thsel0 gpio */
	int usb_thsel1;						/* usb thsel1 gpio */
	int adp_safe_irq;					/* adp safe irq */
	int usb_hot_irq;					/* usb_hot irq */
	int err_detect;						/* error detect */
	struct mutex fj_safety_state_lock;	/* state update mutex */
	struct wake_lock fj_usb_hot_lock;	/* usb_hot wakelock */
	struct wake_lock fj_adp_safe_lock;	/* adp_safe wakelock */
/* FCNT LIMITED:2016-02-03 H16100712 add end */
	int fj_vb_det_irq;					/* vbdet irq */
	int fj_vc_det_irq;					/* vcdet irq */
	int fj_usbin_ov_irq;				/* usbin ov irq */
	int fj_dcin_ov_irq;					/* dcin ov irq */
	struct wake_lock fj_vbdet_lock;		/* vbdet wakelock */
	struct wake_lock fj_vcdet_lock;		/* vcdet wakelock */
	struct wake_lock fj_usbin_ov_lock;	/* usbin ov wakelock */
	struct wake_lock fj_dcin_ov_lock;	/* dcin ov wakelock */
	struct wake_lock fj_usbid_lock;		/* usbid wakelock */
	u8 fj_ovp_device;					/* over voltage device */
	struct mutex fj_state_lock;			/* state update mutex */
/* FCNT LIMITED:2016-02-03 H16100722 add start */
	struct mutex fj_otg_lock;			/* otg mutex */
/* FCNT LIMITED:2016-02-03 H16100722 add end */
	char * fj_vbdet_name;				/* vbdet irq name */
	char * fj_vcdet_name;				/* vcdet irq name */
};

/* fj_det driver state */
static struct _fj_platform_data fj_platform_data = {
	.fj_id_device = FJ_NO_DEVICE,
	.fj_ovp_state = FJ_PM_OVP_INIT,
	.fj_ext_charge = false,
	.fj_mhl_offcharge = false,
	.fj_ovp_device = FJ_OVP_DEV_NONE,
	.fj_vbdet_name = "fj_vbdet",
	.fj_vcdet_name = "fj_vcdet",
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	.err_detect = 0,
/* FCNT LIMITED:2016-02-03 H16100712 add end */
};
/* FCNT LIMITED:2016-02-19 H16100406 add start */
struct status_info {
	int vc_det;
	int vb_det;
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	int adp_safe;
	int usb_hot;
/* FCNT LIMITED:2016-02-03 H16100712 add end */
	FJ_OVP_TYPE ovp_state_dc;
	FJ_OVP_TYPE ovp_state_usb;
};

#define FJ_CHATT		(-1)

#define FJ_CHECK_VCDET		BIT(0)
#define FJ_CHECK_VBDET		BIT(1)
#define FJ_CHECK_DCSTS		BIT(2)
#define FJ_CHECK_USBSTS		BIT(3)
/* FCNT LIMITED:2016-02-03 H16100712 add start */
#define FJ_CHECK_ADPSAFE	BIT(4)
#define FJ_CHECK_USBHOT		BIT(5)
/* FCNT LIMITED:2016-02-03 H16100712 add end */
/* FCNT LIMITED:2016-02-03 H16100712 mod start */
#define FJ_CHECK_INIT	(FJ_CHECK_VCDET|FJ_CHECK_VBDET|FJ_CHECK_DCSTS|FJ_CHECK_USBSTS|FJ_CHECK_ADPSAFE|FJ_CHECK_USBHOT)
/* FCNT LIMITED:2016-02-03 H16100712 mod end */
#define FJ_BIT_CHK(a, bit)	((bit) == ((a) & (bit)))

static void fj_set_otgon(int value);	/* FCNT LIMITED:2016-02-03 H16100722 add */

/* FCNT LIMITED:2016-02-12 H16100739 add start */
FJ_DET_ID_TYPE fj_det_get_device_type(void)
{
	return (FJ_DET_ID_TYPE)fj_platform_data.fj_id_device;
}
/* FCNT LIMITED:2016-02-12 H16100739 add end */

static int fj_vcvb_status_get(int pin, int *value)
{
	*value = gpio_get_value(pin);
	return 0;
}

static int fj_ovp_status_get(struct smbchg_chip *chip, u16 reg, FJ_OVP_TYPE *value)
{
	u8 rt_sts;
	int rc = 0;

	do {
		rc = smbchg_read(chip, &rt_sts, reg, 1);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read 0x%x rt status rc = %d\n", reg, rc);
			break;
		}
		if ((rt_sts & FJ_RT_OV_BIT) == FJ_RT_OV_BIT) {
			*value = FJ_PM_OVP_DETECT;
		} else {
			*value = FJ_PM_OVP_NONE;
		}
	} while (0);

	return rc;
}

#define fj_chattering_count_up(now_value, pre_value, chattering_cnt) \
	if (pre_value == now_value) {	\
		chattering_cnt++;			\
	} else {						\
		pre_value = now_value;		\
		chattering_cnt = 1;			\
	}

#define fj_chattering_comp_check(chattering_cnt, detectcnt) \
	(chattering_cnt >= detectcnt) ? true : false

static void fj_status_check(struct smbchg_chip *chip, int check, struct status_info *status)
{
	int now_det_dc = 0;
	int now_det_usb = 0;
	FJ_OVP_TYPE now_ovp_dc = FJ_PM_OVP_INIT;
	FJ_OVP_TYPE now_ovp_usb = FJ_PM_OVP_INIT;
	int chattering_cnt_det_dc = 0;
	int chattering_cnt_det_usb = 0;
	int chattering_cnt_ovp_dc = 0;
	int chattering_cnt_ovp_usb = 0;
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	int now_adp_safe = 0;
	int now_usb_hot = 0;
	int chattering_cnt_adp_safe = 0;
	int chattering_cnt_usb_hot = 0;
	bool loopbreak = false;
/* FCNT LIMITED:2016-02-03 H16100712 add end */
	int total = 0;
	int ret;
	/* bit      5 |       4 |      3 |     2 |     1 |     0  */
	/*     usbhot | adpsafe | usbsts | dcsts | vbdet | vcdet  */
	int detect = 0;
	
	status->vc_det = -1;
	status->vb_det = -1;
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	status->adp_safe = -1;
	status->usb_hot = -1;
/* FCNT LIMITED:2016-02-03 H16100712 add end */
	status->ovp_state_dc = FJ_PM_OVP_INIT;
	status->ovp_state_usb = FJ_PM_OVP_INIT;
	
	while (1) {

		if (FJ_BIT_CHK(check, FJ_CHECK_VCDET) && !FJ_BIT_CHK(detect, FJ_CHECK_VCDET)) {
/* FCNT LIMITED:2016-02-03 H16100711 mod start */
			if (gpio_is_valid(fj_platform_data.vc_det_gpio)) {
				ret = fj_vcvb_status_get(fj_platform_data.vc_det_gpio, &now_det_dc);	/* FCNT LIMITED:2016-02-03 H16100726 mod */
				if (ret == 0) {
					fj_chattering_count_up(now_det_dc, status->vc_det, chattering_cnt_det_dc);
				}
				if (fj_chattering_comp_check(chattering_cnt_det_dc, FJ_DETECT_COUNT)) {
					detect |= FJ_CHECK_VCDET;
				}
			} else {
				detect |= FJ_CHECK_VCDET;
			}
/* FCNT LIMITED:2016-02-03 H16100711 mod end */
		}
		
		if (FJ_BIT_CHK(check, FJ_CHECK_VBDET) && !FJ_BIT_CHK(detect, FJ_CHECK_VBDET)) {
			ret = fj_vcvb_status_get(fj_platform_data.vb_det_gpio, &now_det_usb);		/* FCNT LIMITED:2016-02-03 H16100726 mod */
			if (ret == 0) {
				fj_chattering_count_up(now_det_usb, status->vb_det, chattering_cnt_det_usb);
			}
			if (fj_chattering_comp_check(chattering_cnt_det_usb, FJ_DETECT_COUNT)) {
				detect |= FJ_CHECK_VBDET;
			}
		}

		if (FJ_BIT_CHK(check, FJ_CHECK_DCSTS) && !FJ_BIT_CHK(detect, FJ_CHECK_DCSTS)) {
/* FCNT LIMITED:2016-02-03 H16100711 mod start */
			if (gpio_is_valid(fj_platform_data.vc_det_gpio)) {
				ret = fj_ovp_status_get(chip, chip->dc_chgpth_base + FJ_RT_STS, &now_ovp_dc);
				if (ret == 0) {
					fj_chattering_count_up(now_ovp_dc, status->ovp_state_dc, chattering_cnt_ovp_dc);
				}
				if (fj_chattering_comp_check(chattering_cnt_ovp_dc, FJ_DETECT_COUNT)) {
					detect |= FJ_CHECK_DCSTS;
				}
			} else {
				detect |= FJ_CHECK_DCSTS;
			}
/* FCNT LIMITED:2016-02-03 H16100711 mod end */
		}

		if (FJ_BIT_CHK(check, FJ_CHECK_USBSTS) && !FJ_BIT_CHK(detect, FJ_CHECK_USBSTS)) {
			ret = fj_ovp_status_get(chip, chip->usb_chgpth_base + FJ_RT_STS, &now_ovp_usb);
			if (ret == 0) {
				fj_chattering_count_up(now_ovp_usb, status->ovp_state_usb, chattering_cnt_ovp_usb);
			}
			if (fj_chattering_comp_check(chattering_cnt_ovp_usb, FJ_DETECT_COUNT)) {
				detect |= FJ_CHECK_USBSTS;
			}
		}
/* FCNT LIMITED:2016-02-03 H16100712 add start */
		if (FJ_BIT_CHK(check, FJ_CHECK_ADPSAFE) && !FJ_BIT_CHK(detect, FJ_CHECK_ADPSAFE)) {
			if (gpio_is_valid(fj_platform_data.adp_safe_gpio)) {
				now_adp_safe = gpio_get_value(fj_platform_data.adp_safe_gpio);
				fj_chattering_count_up(now_adp_safe, status->adp_safe, chattering_cnt_adp_safe);
				if (fj_chattering_comp_check(chattering_cnt_adp_safe, FJ_DETECT_COUNT)) {
					detect |= FJ_CHECK_ADPSAFE;
				}
			} else {
				detect |= FJ_CHECK_ADPSAFE;
			}
		}
		if (FJ_BIT_CHK(check, FJ_CHECK_USBHOT) && !FJ_BIT_CHK(detect, FJ_CHECK_USBHOT)) {
			if (gpio_is_valid(fj_platform_data.usb_hot_gpio)) {
				now_usb_hot = gpio_get_value(fj_platform_data.usb_hot_gpio);
				fj_chattering_count_up(now_usb_hot, status->usb_hot, chattering_cnt_usb_hot);
				if (fj_chattering_comp_check(chattering_cnt_usb_hot, FJ_USB_HOT_DETECT_COUNT)) {
					detect |= FJ_CHECK_USBHOT;
				}
			} else {
				detect |= FJ_CHECK_USBHOT;
			}
		}
/* FCNT LIMITED:2016-02-03 H16100712 add end */
		if (FJ_BIT_CHK(detect, check)) {
/* FCNT LIMITED:2016-02-03 H16100712 mod start */
			pr_info("[DC]ovp_state=%d value=%d [USB]ovp_state=%d value=%d adp_safe=%d usb_hot=%d count=%d\n",
					status->ovp_state_dc, !status->vc_det, status->ovp_state_usb, !status->vb_det, status->adp_safe, status->usb_hot, total);
/* FCNT LIMITED:2016-02-03 H16100712 mod end */
			break;
		}

		total++;
		if (total >= FJ_DETECT_COUNT_MAX) {
/* FCNT LIMITED:2016-02-03 H16100712 mod start */
			if (FJ_BIT_CHK(check, FJ_CHECK_VCDET) && !FJ_BIT_CHK(detect, FJ_CHECK_VCDET)) {
				pr_info("chattering 600 count over usb_dc=[DC]\n");
				if (check == FJ_CHECK_INIT) {
					status->vc_det = 1;
					loopbreak = true;
				}
				total = 0;
			}
			if (FJ_BIT_CHK(check, FJ_CHECK_VBDET) && !FJ_BIT_CHK(detect, FJ_CHECK_VBDET)) {
				pr_info("chattering 600 count over usb_dc=[USB]\n");
				if (check == FJ_CHECK_INIT) {
					status->vb_det = 1;
					loopbreak = true;
				}
				total = 0;
			}
			if (FJ_BIT_CHK(check, FJ_CHECK_DCSTS) && !FJ_BIT_CHK(detect, FJ_CHECK_DCSTS)) {
				status->ovp_state_dc = FJ_PM_OVP_DETECT;
				loopbreak = true;
				FJ_QPNP_RECLOG("SMBCHG: ovp chattering count over -> ovp detect usb_dc=[DC]\n");
			}
			if (FJ_BIT_CHK(check, FJ_CHECK_USBSTS) && !FJ_BIT_CHK(detect, FJ_CHECK_USBSTS)) {
				status->ovp_state_usb = FJ_PM_OVP_DETECT;
				loopbreak = true;
				FJ_QPNP_RECLOG("SMBCHG: ovp chattering count over -> ovp detect usb_dc=[USB]\n");
			}
			if (FJ_BIT_CHK(check, FJ_CHECK_ADPSAFE) && !FJ_BIT_CHK(detect, FJ_CHECK_ADPSAFE)) {
				status->adp_safe = 0;
				loopbreak = true;
				FJ_QPNP_RECLOG("SMBCHG: adp safe chattering count over\n");
			}
			if (loopbreak) {
				break;
			}
/* FCNT LIMITED:2016-02-03 H16100712 mod end */
		}
		
		msleep(FJ_DETECT_WAIT_MS);
	}
}
/* FCNT LIMITED:2016-02-19 H16100406 add end */

/* ----------------------------------------------------------------- */
/*
 * read USB-ID adc value
 * @return   usb_id : adc value
 */
uint16_t fj_get_usb_id(struct smbchg_chip *chip)
{
	int rc = 0;
	uint16_t usb_id = 0x0000;
	u8 rt_sts;

	rc = smbchg_read(chip, &rt_sts, chip->usb_chgpth_base + FJ_USBID_VALID_ID_11_8, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb id_11_8 status rc = %d\n", rc);
	}
	usb_id = rt_sts << 8;
	rc = smbchg_read(chip, &rt_sts, chip->usb_chgpth_base + FJ_USBID_VALID_ID_7_0, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb id_7_0 rc = %d\n", rc);
	}
	usb_id |= rt_sts;

	return usb_id;
}

/* ----------------------------------------------------------------- */
/*
 * convert adc value into device
 * @return   FJ_DEV_ID_TYPE : device type
 */
static FJ_ID_TYPE fj_adc_to_device(uint16_t id_value)
{
	FJ_ID_TYPE device = FJ_NO_DEVICE;
	int i;

	for (i = 0; i < FJ_DEV_ID_MAX; i++) {
		if (id_value <= fj_device_tbl[i].id_value) {
			pr_info("%s usb_id=%x table=%x\n", __func__, id_value, fj_device_tbl[i].id_value);
			device = fj_device_tbl[i].device;
			break;
		}
	}

	return device;
}
/* ----------------------------------------------------------------- */
/*
 * fj driver state update
 * @param device : detect device
 */
static void fj_state_update(FJ_ID_TYPE device)
{
	mutex_lock(&fj_platform_data.fj_state_lock);
/* FCNT LIMITED:2016-02-03 H16100722 add start */
	if (fj_platform_data.fj_id_device == FJ_NO_DEVICE && device == FJ_DEVICE_OTG) {
		fj_host_send_uevent(USB_HOST_EXT_EVENT_DETECT);
	} else if (fj_platform_data.fj_id_device == FJ_DEVICE_OTG && device == FJ_NO_DEVICE) {
		fj_host_send_uevent(USB_HOST_EXT_EVENT_NONE);
	}
/* FCNT LIMITED:2016-02-03 H16100722 add end */
	fj_platform_data.fj_id_device = device;
	mutex_unlock(&fj_platform_data.fj_state_lock);

	pr_err("SMBCHG: %s device=%d\n", __func__, fj_platform_data.fj_id_device);
}



/* FCNT LIMITED:2016-02-19 H16100406 mod start */
/* ----------------------------------------------------------------- */
/*
 * vbdet/vcdet chattering
 * @return   1   :    detect vbdet/vcdet
 *           0   :    don't detect vbdet/vcdet
 */
static bool fj_status_chattering(struct smbchg_chip *chip, FJ_STATUS_CHATT usb_dc)
{
	struct status_info status;
	int now_value;

	if (usb_dc == FJ_STATUS_USB) {
		fj_status_check(chip, FJ_CHECK_VBDET, &status);
		now_value = status.vb_det;
	} else {
		fj_status_check(chip, FJ_CHECK_VCDET, &status);
		now_value = status.vc_det;
	}
	
	if (now_value) { /* gpio = 0:valid 1:invalid */
		return false;
	} else {
		return true;
	}
	
}
/* FCNT LIMITED:2016-02-19 H16100406 mod end */

/* ----------------------------------------------------------------- */
/*
 * eover voltage notify
 */
static void fj_ovp_notify_proc(FJ_OVP_TYPE ovp_detect, FJ_OVP_CHATT usb_dc)
{
	switch (ovp_detect) {
		case FJ_PM_OVP_NONE:
			fj_platform_data.fj_ovp_device &= ~usb_dc;
			if (fj_platform_data.fj_ovp_device) {
				ovp_detect = FJ_PM_OVP_DETECT;
			}
			break;
		case FJ_PM_OVP_DETECT:
			fj_platform_data.fj_ovp_device |= usb_dc;
			break;
		default:
			break;
	}

	if (ovp_detect != fj_platform_data.fj_ovp_state) {
		switch (ovp_detect){
			case FJ_PM_OVP_NONE:
				FJ_QPNP_RECLOG("SMBCHG: over voltage cancel Call API "
							  "fj_chg_notify_error(NO ERROR)\n");
				fj_chg_notify_error(FJ_CHG_ERROR_OVP_NONE);		/* FCNT LIMITED:2016-02-03 H16100712 mod */
				break;
			case FJ_PM_OVP_DETECT:
				FJ_QPNP_RECLOG("SMBCHG: over voltage detect Call API "
							  "fj_chg_notify_error(ERROR DETECT)\n");
				fj_chg_notify_error(FJ_CHG_ERROR_OVP_DETECT);	/* FCNT LIMITED:2016-02-03 H16100712 mod */
				break;
			default :
				break;
		}
		fj_platform_data.fj_ovp_state = ovp_detect;
	}
}

/* FCNT LIMITED:2016-02-19 H16100406 mod start */
/* ----------------------------------------------------------------- */
/*
 * over voltage check and notify
 */
static void fj_ovp_check_notify(struct smbchg_chip *chip, FJ_STATUS_CHATT usb_dc)
{
	struct status_info status;
	FJ_OVP_TYPE ovp_state = FJ_PM_OVP_INIT;

	if (usb_dc == FJ_STATUS_USB) {
		fj_status_check(chip, FJ_CHECK_USBSTS, &status);
		ovp_state = status.ovp_state_usb;
	} else {
		fj_status_check(chip, FJ_CHECK_DCSTS, &status);
		ovp_state = status.ovp_state_dc;
	}

	mutex_lock(&fj_platform_data.fj_state_lock);
	if (usb_dc == FJ_STATUS_USB) {
		fj_ovp_notify_proc(ovp_state, FJ_OVP_USB);
	} else {
		fj_ovp_notify_proc(ovp_state, FJ_OVP_DC);
	}
	mutex_unlock(&fj_platform_data.fj_state_lock);
}
/* FCNT LIMITED:2016-02-19 H16100406 mod end */

/* ----------------------------------------------------------------- */
/*
 * ext usb check (unknown ID, MHL off charge)
 * @return   1   :    detect ext usb
 *           0   :    don't detect ext usb
 */
static bool fj_ext_usb_check(struct smbchg_chip *chip, bool vbdet)
{
	bool rc = false;
	uint16_t usb_id;
	FJ_ID_TYPE device = FJ_NO_DEVICE;

	if (vbdet) {
		usb_id = fj_get_usb_id(chip);
		device = fj_adc_to_device(usb_id);
		/* unknown id check */
		FJ_QPNP_RECLOG("SMBCHG: detect device=%d adc=0x%x\n",device ,usb_id);
		if (device == FJ_DEVICE_UNKNOWN) {
			if (!fj_platform_data.fj_ext_charge) {
				FJ_QPNP_RECLOG("SMBCHG: FJ_DEVICE_UNKNOWN charge start\n");
				fj_platform_data.fj_ext_charge = true;
				fj_chg_other_vbus_draw(FJ_CHG_ENABLE);
				rc = true;
			}
		}
		if (device == FJ_DEVICE_OTG) {
			pr_info("SMBCHG: ignore the VBUS detection during OTG\n");
			rc = true;
		}
		/* off_charge mhl device check */
		if (charging_mode) {
			if (device == FJ_DEVICE_1K) {
				if (!fj_platform_data.fj_mhl_offcharge) {
					pr_info("SMBCHG: MHL off_charge start\n");
					fj_platform_data.fj_mhl_offcharge = true;
					fj_chg_mhl_vbus_draw(FJ_CHG_ENABLE);
					rc = true;
				}
			}
		}
	} else {
		if (fj_platform_data.fj_ext_charge) {
			FJ_QPNP_RECLOG("SMBCHG: FJ_DEVICE_UNKNOWN charge end\n");
			fj_platform_data.fj_ext_charge = false;
			fj_chg_other_vbus_draw(FJ_CHG_DISABLE);
			rc = true;
		}
		if (fj_platform_data.fj_mhl_offcharge) {
			pr_info("SMBCHG: MHL off_charge end\n");
			fj_platform_data.fj_mhl_offcharge = false;
			fj_chg_mhl_vbus_draw(FJ_CHG_DISABLE);
			rc = true;
		}
/* FCNT LIMITED:2016-02-03 H16100722 add start */
		if (fj_platform_data.fj_id_device == FJ_DEVICE_OTG) {
			pr_info("ignore the VBUS detection during OTG\n");
			return true;
		}
/* FCNT LIMITED:2016-02-03 H16100722 add end */
/* FCNT LIMITED:2016-02-03 H16100719 add start */
		if (fj_platform_data.fj_id_device == FJ_DEVICE_1K) {
			pr_info("ignore vbus off from MHL\n");
			return rc;
		}
/* FCNT LIMITED:2016-02-03 H16100719 add start */
	}
	fj_state_update(device);

	return rc;
}


/* ----------------------------------------------------------------- */
/*
 * ext id check (MHL,OTG, MHL off charge)
 * @return   1   :    detect ext id
 *           0   :    don't detect ext id
 */
static bool fj_ext_id_check(struct smbchg_chip *chip)
{
	bool rc = true;
	uint16_t usb_id;
	FJ_ID_TYPE device = FJ_NO_DEVICE;
	
	usb_id = fj_get_usb_id(chip);
	device = fj_adc_to_device(usb_id);

	if ((device == FJ_DEVICE_OTG) || (device == FJ_DEVICE_1K)) {
/* FUJITSU LIMITED:2015-06-19 H1520382 mod start */
		if ((fj_platform_data.fj_id_device == FJ_DEVICE_OTG) && (device == FJ_DEVICE_1K)){
			pr_info("ignore OTG->1K\n");
			rc = true;
			return rc;
		}
/* FUJITSU LIMITED:2015-06-19 H1520382 mod end */
		pr_info("OTG and MHL device notify power_supply\n");
		fj_state_update(device);
		rc = false;
	}
	if (((fj_platform_data.fj_id_device == FJ_DEVICE_OTG) && (device != FJ_DEVICE_OTG)) ||
		((fj_platform_data.fj_id_device == FJ_DEVICE_1K) && (device != FJ_DEVICE_1K))) {
		pr_info("ignore it other than OPEN from OTG and MHL\n");
		fj_set_otgon(0);	/* FCNT LIMITED:2016-02-03 H16100722 add */
		fj_state_update(FJ_NO_DEVICE);
		rc = false;
	}

	if (charging_mode) {
		if (device == FJ_DEVICE_1K) {
			pr_info("off_chage don't notify to MHL device\n");
			rc = true;
		}
	}
	
	return rc;
}

/* ----------------------------------------------------------------- */
/*
 * Change USB State for Debug
 * @return   0   :    function success
 *          !0   :    function fail
 */
static int fj_debug_set_usb(const char *val, struct kernel_param *kp)
{
	int event;
	char *e;
	event = simple_strtoul(val, &e, 10);

	pr_info("SMBCHG: %s start: event is %d device is %d\n",
							__func__, event, fj_platform_data.fj_id_device);

	if ((event == 6) &&
		(fj_platform_data.fj_id_device == FJ_DEVICE_OTG)) {
		
		/* force usb otg detect -> not detect */
		power_supply_set_usb_otg(the_chip->usb_psy, 0);
		fj_set_otgon(0);	/* FCNT LIMITED:2016-02-03 H16100722 add */
		fj_state_update(FJ_NO_DEVICE);
	}
	return 0;
}

/* ----------------------------------------------------------------- */
/*
 * Show USB State for Debug
 * @return   0   :    function success
 *          !0   :    function fail
 */
static int fj_debug_get_usb(char *val, struct kernel_param *kp)
{
/* FUJITSU LIMITED:2016-01-12 H16100714 add start */
	if (charging_mode) {
		return sprintf(val, "%d", usb_id_info);
	}
/* FUJITSU LIMITED:2016-01-12 H16100714 add end */
/* FCNT LIMITED:2016-03-16 H16100722-1 mod start */
/* FCNT LIMITED:2016-02-03 H16100722 mod start */
	if ((bto_info[BTO_INFO_ID_USB_HOST] == BTO_INFO_DISABLE) ||
		(fj_platform_data.fj_id_device != FJ_DEVICE_OTG)) {
		usb_id_info = 0;
	} else {
		if (otg_on) {
			usb_id_info = 3;
		} else {
			usb_id_info = 2;
		}
	}
/* FCNT LIMITED:2016-02-03 H16100722 mod end */
/* FCNT LIMITED:2016-03-16 H16100722-1 mod end */
	return sprintf(val, "%d", usb_id_info);
}

MODULE_PARM_DESC(usb_id_info, "USB State ID");
module_param_call(usb_id_info, fj_debug_set_usb, fj_debug_get_usb,
				  &usb_id_info, 0644);

/* FCNT LIMITED:2016-02-03 H16100722 add start */
static void fj_set_otgon(int value)
{
	mutex_lock(&fj_platform_data.fj_otg_lock);
#ifdef FEATURE_OTG_SET_VBUS_IF
	otg_on = value;
	dwc3_otg_set_fj_vbus(otg_on);
#endif
	mutex_unlock(&fj_platform_data.fj_otg_lock);
}

static int fj_debug_set_otgon(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	unsigned int value;

/* FCNT LIMITED:2016-03-16 H16100722-1 mod start */
	if (bto_info[BTO_INFO_ID_USB_HOST] == BTO_INFO_DISABLE) {
		return ret;
	}
/* FCNT LIMITED:2016-03-16 H16100722-1 mod end */

	if (fj_platform_data.fj_id_device != FJ_DEVICE_OTG) {
		pr_info("OTG is not connected\n");
		return ret;
	}

	value = simple_strtoul(val, NULL, 10);

	pr_info("value:%d\n", value);
	if (value == 1 || value == 0) {
		fj_set_otgon(value);
	}
	return ret;
}

MODULE_PARM_DESC(otg_on, "OTG state");
module_param_call(otg_on, fj_debug_set_otgon, param_get_uint, &otg_on, 0644);
/* FCNT LIMITED:2016-02-03 H16100722 add end */
/* FCNT LIMITED:2016-02-03 H16100721 add start */
static int fj_usb_temp_write(FJ_USBTEMP_FUNC wr)
{
	struct pattern {
		uint64_t pin1;
		uint64_t pin2;
	};
	struct _fj_platform_data *fj_pdata = &fj_platform_data;
	int i;
	uint64_t mask = 1;
	uint64_t pin1;
	uint64_t pin2;

	#define FJDET_PATTERN_NUM		(60)
	#define FJDET_WAIT_NUM			(5)
	#define FJDET_AFTER_WAIT_NUM	(1000)

	static const struct pattern Pattern1 = {
		0xC92492492492491, 0x9C0E001C0038E3B
	};

	static const struct pattern Pattern2 = {
		0xC92492492492491, 0x9C0E001C000003B
	};

	if (!gpio_is_valid(fj_pdata->usb_thsel0) || !gpio_is_valid(fj_pdata->usb_thsel1)) {
		pr_err("not support\n");
		return -1;
	}

	mask <<= (FJDET_PATTERN_NUM - 1);
	if (wr == FJ_USBTEMP_NORMAL) {
		pin1 = Pattern1.pin1;
		pin2 = Pattern1.pin2;
		pr_info("Pattern1\n");
	} else if (wr == FJ_USBTEMP_ERROR) {
		pin1 = Pattern2.pin1;
		pin2 = Pattern2.pin2;
		pr_info("Pattern2\n");
	} else {
		pr_err("parameter error %d\n", wr);
		return -1;
	}
	for (i = 0; i < FJDET_PATTERN_NUM; i++) {
		if (pin1 & mask) {
			gpio_direction_output(fj_pdata->usb_thsel0, 1);
		} else {
			gpio_direction_output(fj_pdata->usb_thsel0, 0);
		}
		if (pin2 & mask) {
			gpio_direction_output(fj_pdata->usb_thsel1, 0);
		} else {
			gpio_direction_output(fj_pdata->usb_thsel1, 1);
		}
		udelay(FJDET_WAIT_NUM);
		mask >>= 1;
	}
	udelay(FJDET_AFTER_WAIT_NUM);

	return 0;
}

static int fj_debug_set_usbtemp(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_uint(val, kp);
	if (ret) {
		return ret;
	}
	pr_info("value:%d\n", usb_temp);

	fj_usb_temp_write(usb_temp);
	return ret;
}

static int fj_debug_get_usbtemp(char *val, struct kernel_param *kp)
{
	struct status_info status;

	wake_lock(&fj_platform_data.fj_usb_hot_lock);
	fj_status_check(the_chip, FJ_CHECK_USBHOT, &status);
	wake_unlock(&fj_platform_data.fj_usb_hot_lock);

	return sprintf(val, "%d", status.usb_hot);
}

MODULE_PARM_DESC(usb_temp, "USB temperature state");
module_param_call(usb_temp, fj_debug_set_usbtemp, fj_debug_get_usbtemp, &usb_temp, 0644);
/* FCNT LIMITED:2016-02-03 H16100721 add end */
/* FCNT LIMITED:2016-02-12 H16100738 add start */
static int fj_get_vc_status(char *val, const struct kernel_param *kp)
{
	struct status_info status;

	wake_lock(&fj_platform_data.fj_vcdet_lock);
	fj_status_check(the_chip, FJ_CHECK_VCDET, &status);
	wake_unlock(&fj_platform_data.fj_vcdet_lock);

	return sprintf(val, "%d", status.vc_det);
}

MODULE_PARM_DESC(vc_status, "VCDET status");
module_param_call(vc_status, NULL, fj_get_vc_status, &vc_status, 0444);

static int fj_get_vb_status(char *val, const struct kernel_param *kp)
{
	struct status_info status;

	wake_lock(&fj_platform_data.fj_vbdet_lock);
	fj_status_check(the_chip, FJ_CHECK_VBDET, &status);
	wake_unlock(&fj_platform_data.fj_vbdet_lock);

	return sprintf(val, "%d", status.vb_det);
}

MODULE_PARM_DESC(vb_status, "VBDET status");
module_param_call(vb_status, NULL, fj_get_vb_status, &vb_status, 0444);
/* FCNT LIMITED:2016-02-12 H16100738 add end */
/* FCNT LIMITED:2016-02-03 H16100712 add start */
static void fj_usb_safety_notify_proc(int err_detect, FJ_USB_SAFETY type)
{
	struct _fj_platform_data *fj_pdata = &fj_platform_data;

	pr_info("type:%d:detect%d err_detect:0x%x\n", type, err_detect, fj_pdata->err_detect);

	mutex_lock(&fj_pdata->fj_safety_state_lock);
	if (!err_detect) {
		FJ_QPNP_RECLOG("SMBCHG: safety error usbhot=%d short=%d\n",
					((fj_pdata->err_detect|(1 << type)) & (1 << FJ_USB_SAFETY_USB_TEMP)) ? 1:0,
					((fj_pdata->err_detect|(1 << type)) & (1 << FJ_USB_SAFETY_ADP_SAFE)) ? 1:0);
		if (!fj_pdata->err_detect) {
			pr_info("USB_HOT_DETECT\n");
			if (fj_pdata->fj_id_device == FJ_DEVICE_OTG) {
				fj_set_otgon(0);
			}
			fj_chg_notify_error(FJ_CHG_ERROR_USB_HOT_DETECT);
		}
		fj_pdata->err_detect |= (1 << type);
	} else {
		if (fj_pdata->err_detect) {
			FJ_QPNP_RECLOG("SMBCHG: safety error usbhot=%d short=%d\n",
						((fj_pdata->err_detect & ~(1 << type)) & (1 << FJ_USB_SAFETY_USB_TEMP)) ? 1:0,
						((fj_pdata->err_detect & ~(1 << type)) & (1 << FJ_USB_SAFETY_ADP_SAFE)) ? 1:0);
			fj_pdata->err_detect &= ~(1 << type);
			if (!fj_pdata->err_detect) {
				pr_info("USB_HOT_NONE\n");
				fj_chg_notify_error(FJ_CHG_ERROR_USB_HOT_NONE);
			}
		}
	}
	mutex_unlock(&fj_pdata->fj_safety_state_lock);
}

static void fj_usb_hot_handler_core(void *_chip, int _ush_hot)
{
	struct smbchg_chip *chip = _chip;
	struct status_info status;

	if (!gpio_is_valid(fj_platform_data.usb_hot_gpio)) {
		return;
	}

	pr_smb(PR_INTERRUPT, "triggered\n");
	wake_lock(&fj_platform_data.fj_usb_hot_lock);

	if (_ush_hot < 0) {
		fj_status_check(chip, FJ_CHECK_USBHOT, &status);
	} else {
		status.usb_hot = _ush_hot;
	}
	fj_usb_safety_notify_proc(status.usb_hot, FJ_USB_SAFETY_USB_TEMP);

	wake_unlock(&fj_platform_data.fj_usb_hot_lock);

	return;
}

static irqreturn_t fj_usb_hot_handler(int irq, void *_chip)
{
	fj_usb_hot_handler_core(_chip, FJ_CHATT);
	return IRQ_HANDLED;
}

static void fj_adp_safe_handler_core(void *_chip, int _adp_safe)
{
	struct smbchg_chip *chip = _chip;
	struct status_info status;

	if (!gpio_is_valid(fj_platform_data.adp_safe_gpio)) {
		return;
	}

	pr_smb(PR_INTERRUPT, "triggered\n");
	wake_lock(&fj_platform_data.fj_adp_safe_lock);

	if (_adp_safe < 0) {
		fj_status_check(chip, FJ_CHECK_ADPSAFE, &status);
	} else {
		status.adp_safe = _adp_safe;
	}
	fj_usb_safety_notify_proc(status.adp_safe, FJ_USB_SAFETY_ADP_SAFE);

	wake_unlock(&fj_platform_data.fj_adp_safe_lock);
	return;
}

static irqreturn_t fj_adp_safe_handler(int irq, void *_chip)
{
	fj_adp_safe_handler_core(_chip, FJ_CHATT);
	return IRQ_HANDLED;
}
/* FCNT LIMITED:2016-02-03 H16100712 add end */
/* FCNT LIMITED:2016-02-19 H16100406 add start */
static void dcin_ov_handler_core(void *_chip, FJ_OVP_TYPE ovp_state)
{
	struct smbchg_chip *chip = _chip;

/* FCNT LIMITED:2016-02-03 H16100711 add start */
	if (!gpio_is_valid(fj_platform_data.vc_det_gpio)) {
		return;
	}
/* FCNT LIMITED:2016-02-03 H16100711 add end */

	pr_smb(PR_INTERRUPT, "triggered\n");

	wake_lock(&fj_platform_data.fj_dcin_ov_lock);

	if (ovp_state == FJ_PM_OVP_INIT) {
		/* vcin ovp check & notify */
		fj_ovp_check_notify(chip, FJ_STATUS_DC);
	} else {
		mutex_lock(&fj_platform_data.fj_state_lock);
		fj_ovp_notify_proc(ovp_state, FJ_OVP_DC);
		mutex_unlock(&fj_platform_data.fj_state_lock);
	}

	wake_unlock(&fj_platform_data.fj_dcin_ov_lock);

	return;
}
/* FCNT LIMITED:2016-02-19 H16100406 add end */

/**
 * dcin_ov_handler() - called when the dc voltage crosses the ov threshold
 * @chip: pointer to smbchg_chip
 */
static irqreturn_t dcin_ov_handler(int irq, void *_chip)
{
	/* FCNT LIMITED:2016-02-19 H16100406 mod start */
	dcin_ov_handler_core(_chip, FJ_PM_OVP_INIT);
	/* FCNT LIMITED:2016-02-19 H16100406 mod end */
	return IRQ_HANDLED;
}
/* FCNT LIMITED:2016-02-19 H16100406 add start */
static void usbin_ov_handler_core(void *_chip, FJ_OVP_TYPE ovp_state)
{
	struct smbchg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "triggered\n");

	wake_lock(&fj_platform_data.fj_usbin_ov_lock);

	if (ovp_state == FJ_PM_OVP_INIT) {
		/* vcin ovp check & notify */
		fj_ovp_check_notify(chip, FJ_STATUS_USB);
	} else {
		mutex_lock(&fj_platform_data.fj_state_lock);
		fj_ovp_notify_proc(ovp_state, FJ_OVP_USB);
		mutex_unlock(&fj_platform_data.fj_state_lock);
	}

	wake_unlock(&fj_platform_data.fj_usbin_ov_lock);

	return;
}
/* FCNT LIMITED:2016-02-19 H16100406 add end */

/**
 * usbin_ov_handler() - called when the usbin voltage crosses the ov threshold
 * @chip: pointer to smbchg_chip chip
 */
static irqreturn_t usbin_ov_handler(int irq, void *_chip)
{
	/* FCNT LIMITED:2016-02-19 H16100406 mod start */
	usbin_ov_handler_core(_chip, FJ_PM_OVP_INIT);
	/* FCNT LIMITED:2016-02-19 H16100406 mod end */
	return IRQ_HANDLED;
}

/*FUJITSU LIMITED:2015-01-07 H1510087 add end */

#define RID_STS				0xB
#define RID_MASK			0xF
#define IDEV_STS			0x8
#define RT_STS				0x10
#define USBID_MSB			0xE
#define USBIN_UV_BIT			BIT(0)
#define USBIN_OV_BIT			BIT(1)
#define USBIN_SRC_DET_BIT		BIT(2)
#define FMB_STS_MASK			SMB_MASK(3, 0)
#define USBID_GND_THRESHOLD		0x495
static bool is_otg_present_schg(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;
	u8 usbid_reg[2];
	u16 usbid_val;
	/*
	 * After the falling edge of the usbid change interrupt occurs,
	 * there may still be some time before the ADC conversion for USB RID
	 * finishes in the fuel gauge. In the worst case, this could be up to
	 * 15 ms.
	 *
	 * Sleep for 20 ms (minimum msleep time) to wait for the conversion to
	 * finish and the USB RID status register to be updated before trying
	 * to detect OTG insertions.
	 */

	msleep(20);

	/*
	 * There is a problem with USBID conversions on PMI8994 revisions
	 * 2.0.0. As a workaround, check that the cable is not
	 * detected as factory test before enabling OTG.
	 */
	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read IDEV_STS rc = %d\n", rc);
		return false;
	}

	if ((reg & FMB_STS_MASK) != 0) {
		pr_smb(PR_STATUS, "IDEV_STS = %02x, not ground\n", reg);
		return false;
	}

	rc = smbchg_read(chip, usbid_reg, chip->usb_chgpth_base + USBID_MSB, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read USBID rc = %d\n", rc);
		return false;
	}
	usbid_val = (usbid_reg[0] << 8) | usbid_reg[1];

	if (usbid_val > USBID_GND_THRESHOLD) {
		pr_smb(PR_STATUS, "USBID = 0x%04x, too high to be ground\n",
				usbid_val);
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RID_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read usb rid status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "RID_STS = %02x\n", reg);

	return (reg & RID_MASK) == 0;
}

#define RID_CHANGE_DET			BIT(3)
static bool is_otg_present_schg_lite(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->otg_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read otg RT status rc = %d\n", rc);
		return false;
	}

	return !!(reg & RID_CHANGE_DET);
}

static bool is_otg_present(struct smbchg_chip *chip)
{
	if (chip->schg_version == QPNP_SCHG_LITE)
		return is_otg_present_schg_lite(chip);

	return is_otg_present_schg(chip);
}

#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)
#define DCIN_9V				BIT(2)
#define DCIN_UNREG			BIT(1)
#define DCIN_LV				BIT(0)
#define INPUT_STS			0x0D
#define DCIN_UV_BIT			BIT(0)
#define DCIN_OV_BIT			BIT(1)
static bool is_dc_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->dc_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read dc status rc = %d\n", rc);
		return false;
	}

	if ((reg & DCIN_UV_BIT) || (reg & DCIN_OV_BIT))
		return false;

	return true;
}

static bool is_usb_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	if (!(reg & USBIN_SRC_DET_BIT) || (reg & USBIN_OV_BIT))
		return false;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + INPUT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}

	return !!(reg & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static char *usb_type_str[] = {
	"SDP",		/* bit 0 */
	"OTHER",	/* bit 1 */
	"DCP",		/* bit 2 */
	"CDP",		/* bit 3 */
	"NONE",		/* bit 4 error case */
};

#define N_TYPE_BITS		4
#define TYPE_BITS_OFFSET	4

static int get_type(u8 type_reg)
{
	unsigned long type = type_reg;
	type >>= TYPE_BITS_OFFSET;
	return find_first_bit(&type, N_TYPE_BITS);
}

/* helper to return the string of USB type */
static inline char *get_usb_type_name(int type)
{
	return usb_type_str[type];
}

static enum power_supply_type usb_type_enum[] = {
/* FUJITSU LIMITED:2016-01-21 USB H16100640 mod start */
	POWER_SUPPLY_TYPE_UNKNOWN,	/* bit 0 */
	POWER_SUPPLY_TYPE_UNKNOWN,	/* bit 1 */
/* FUJITSU LIMITED:2016-01-21 USB H16100640 mod end */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 4 error case, report DCP */
};

/* helper to return enum power_supply_type of USB type */
static inline enum power_supply_type get_usb_supply_type(int type)
{
	return usb_type_enum[type];
}

static void read_usb_type(struct smbchg_chip *chip, char **usb_type_name,
				enum power_supply_type *usb_supply_type)
{
	int rc, type;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		*usb_type_name = "Other";
		*usb_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}
	type = get_type(reg);
	*usb_type_name = get_usb_type_name(type);
	*usb_supply_type = get_usb_supply_type(type);
}

#define CHGR_STS			0x0E
#define BATT_LESS_THAN_2V		BIT(4)
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_EN_BIT			BIT(0)
#define CHG_INHIBIT_BIT		BIT(1)
#define BAT_TCC_REACHED_BIT		BIT(7)
static int get_prop_batt_status(struct smbchg_chip *chip)
{
	int rc, status = POWER_SUPPLY_STATUS_DISCHARGING;
	u8 reg = 0, chg_type;
	bool charger_present, chg_inhibit;

	charger_present = is_usb_present(chip) | is_dc_present(chip);
	if (!charger_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & BAT_TCC_REACHED_BIT)
		return POWER_SUPPLY_STATUS_FULL;

	chg_inhibit = reg & CHG_INHIBIT_BIT;
	if (chg_inhibit)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & CHG_HOLD_OFF_BIT) {
		/*
		 * when chg hold off happens the battery is
		 * not charging
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;
out:
	pr_smb_rt(PR_MISC, "CHGR_STS = 0x%02x\n", reg);
	return status;
}

#define BAT_PRES_STATUS			0x08
#define BAT_PRES_BIT			BIT(7)
static int get_prop_batt_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->bat_if_base + BAT_PRES_STATUS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	return !!(reg & BAT_PRES_BIT);
}

static int get_prop_charge_type(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, chg_type;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (chg_type == BATT_TAPER_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (chg_type == BATT_FAST_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int set_property_on_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->bms_psy->set_property(chip->bms_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"bms psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}

static int get_property_from_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct smbchg_chip *chip)
{
	int capacity, rc;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get capacity rc = %d\n", rc);
		capacity = DEFAULT_BATT_CAPACITY;
	}
	return capacity;
}

#define DEFAULT_BATT_TEMP		200
static int get_prop_batt_temp(struct smbchg_chip *chip)
{
	int temp, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get temperature rc = %d\n", rc);
		temp = DEFAULT_BATT_TEMP;
	}
	return temp;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int get_prop_batt_current_now(struct smbchg_chip *chip)
{
	int ua, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int get_prop_batt_voltage_now(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return uv;
}

#define DEFAULT_BATT_VOLTAGE_MAX_DESIGN	4200000
static int get_prop_batt_voltage_max_design(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_MAX_DESIGN;
	}
	return uv;
}

static int get_prop_batt_health(struct smbchg_chip *chip)
{
	if (chip->batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static const int usb_current_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

static const int dc_current_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

static const int fcc_comp_table[] = {
	250,
	700,
	900,
	1200,
};

static int calc_thermal_limited_current(struct smbchg_chip *chip,
						int current_ma)
{
	int therm_ma;

	if (chip->therm_lvl_sel > 0
			&& chip->therm_lvl_sel < (chip->thermal_levels - 1)) {
		/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = (int)chip->thermal_mitigation[chip->therm_lvl_sel];
		if (therm_ma < current_ma) {
			pr_smb(PR_STATUS,
				"Limiting current due to thermal: %d mA",
				therm_ma);
			return therm_ma;
		}
	}

	return current_ma;
}

#define CMD_CHG_REG	0x42
#define EN_BAT_CHG_BIT		BIT(1)
static int smbchg_charging_en(struct smbchg_chip *chip, bool en)
{
/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
	/* The en bit is configured active low */
	return smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			EN_BAT_CHG_BIT, en ? 0 : EN_BAT_CHG_BIT);
#else
	return 0;
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
}

#define CMD_IL			0x40
#define USBIN_SUSPEND_BIT	BIT(4)
#define CURRENT_100_MA		100
#define CURRENT_150_MA		150
#define CURRENT_500_MA		500
#define CURRENT_900_MA		900
#define CURRENT_1500_MA		1500
#define SUSPEND_CURRENT_MA	2
#define ICL_OVERRIDE_BIT	BIT(2)
static int smbchg_usb_suspend(struct smbchg_chip *chip, bool suspend)
{
/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
	int rc;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	return rc;
#else
	return 0;
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
}

#define DCIN_SUSPEND_BIT	BIT(3)
static int smbchg_dc_suspend(struct smbchg_chip *chip, bool suspend)
{
/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
	int rc = 0;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			DCIN_SUSPEND_BIT, suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set dc suspend rc = %d\n", rc);
	return rc;
#else
	return 0;
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
}

#define IL_CFG			0xF2
#define DCIN_INPUT_MASK	SMB_MASK(4, 0)
static int smbchg_set_dc_current_max(struct smbchg_chip *chip, int current_ma)
{
	int i;
	u8 dc_cur_val;

	for (i = ARRAY_SIZE(dc_current_table) - 1; i >= 0; i--) {
		if (current_ma >= dc_current_table[i])
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dma current_table\n",
				current_ma);
		return -EINVAL;
	}

	chip->dc_max_current_ma = dc_current_table[i];
	dc_cur_val = i & DCIN_INPUT_MASK;

	pr_smb(PR_STATUS, "dc current set to %d mA\n",
			chip->dc_max_current_ma);
	return smbchg_sec_masked_write(chip, chip->dc_chgpth_base + IL_CFG,
				DCIN_INPUT_MASK, dc_cur_val);
}

enum enable_reason {
	/* userspace has suspended charging altogether */
	REASON_USER = BIT(0),
	/*
	 * this specific path has been suspended through the power supply
	 * framework
	 */
	REASON_POWER_SUPPLY = BIT(1),
	/*
	 * the usb driver has suspended this path by setting a current limit
	 * of < 2MA
	 */
	REASON_USB = BIT(2),
	/*
	 * when a wireless charger comes online,
	 * the dc path is suspended for a second
	 */
	REASON_WIRELESS = BIT(3),
	/*
	 * the thermal daemon can suspend a charge path when the system
	 * temperature levels rise
	 */
	REASON_THERMAL = BIT(4),
	/*
	 * an external OTG supply is being used, suspend charge path so the
	 * charger does not accidentally try to charge from the external supply.
	 */
	REASON_OTG = BIT(5),
	/*
	 * the charger is very weak, do not draw any current from it
	 */
	REASON_WEAK_CHARGER = BIT(6),
};

enum battchg_enable_reason {
	/* userspace has disabled battery charging */
	REASON_BATTCHG_USER		= BIT(0),
	/* battery charging disabled while loading battery profiles */
	REASON_BATTCHG_UNKNOWN_BATTERY	= BIT(1),
};

static struct power_supply *get_parallel_psy(struct smbchg_chip *chip)
{
	if (!chip->parallel.avail)
		return NULL;
	if (chip->parallel.psy)
		return chip->parallel.psy;
	chip->parallel.psy = power_supply_get_by_name("usb-parallel");
	if (!chip->parallel.psy)
		pr_smb(PR_STATUS, "parallel charger not found\n");
	return chip->parallel.psy;
}

static void smbchg_usb_update_online_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				usb_set_online_work);
	bool user_enabled = (chip->usb_suspended & REASON_USER) == 0;
	int online;

	online = user_enabled && chip->usb_present && !chip->very_weak_charger;

	mutex_lock(&chip->usb_set_online_lock);
	if (chip->usb_online != online) {
		power_supply_set_online(chip->usb_psy, online);
		chip->usb_online = online;
	}
	mutex_unlock(&chip->usb_set_online_lock);
}

static bool smbchg_primary_usb_is_en(struct smbchg_chip *chip,
		enum enable_reason reason)
{
	bool enabled;

	mutex_lock(&chip->usb_en_lock);
	enabled = (chip->usb_suspended & reason) == 0;
	mutex_unlock(&chip->usb_en_lock);

	return enabled;
}

static bool smcghg_is_battchg_en(struct smbchg_chip *chip,
		enum battchg_enable_reason reason)
{
	bool enabled;

	mutex_lock(&chip->battchg_disabled_lock);
	enabled = !(chip->battchg_disabled & reason);
	mutex_unlock(&chip->battchg_disabled_lock);

	return enabled;
}

static int smbchg_battchg_en(struct smbchg_chip *chip, bool enable,
		enum battchg_enable_reason reason, bool *changed)
{
	int rc = 0, battchg_disabled;

	pr_smb(PR_STATUS, "battchg %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->battchg_disabled == 0 ? "enabled" : "disabled",
			chip->battchg_disabled, enable, reason);

	mutex_lock(&chip->battchg_disabled_lock);
	if (!enable)
		battchg_disabled = chip->battchg_disabled | reason;
	else
		battchg_disabled = chip->battchg_disabled & (~reason);

	/* avoid unnecessary spmi interactions if nothing changed */
	if (!!battchg_disabled == !!chip->battchg_disabled) {
		*changed = false;
		goto out;
	}

	rc = smbchg_charging_en(chip, !battchg_disabled);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't configure batt chg: 0x%x rc = %d\n",
			battchg_disabled, rc);
		goto out;
	}
	*changed = true;

	pr_smb(PR_STATUS, "batt charging %s, battchg_disabled = %02x\n",
			battchg_disabled == 0 ? "enabled" : "disabled",
			battchg_disabled);
out:
	chip->battchg_disabled = battchg_disabled;
	mutex_unlock(&chip->battchg_disabled_lock);
	return rc;
}

static int smbchg_primary_usb_en(struct smbchg_chip *chip, bool enable,
		enum enable_reason reason, bool *changed)
{
	int rc = 0, suspended;

	pr_smb(PR_STATUS, "usb %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->usb_suspended == 0 ? "enabled"
			: "suspended", chip->usb_suspended, enable, reason);
	mutex_lock(&chip->usb_en_lock);
	if (!enable)
		suspended = chip->usb_suspended | reason;
	else
		suspended = chip->usb_suspended & (~reason);

	/* avoid unnecessary spmi interactions if nothing changed */
	if (!!suspended == !!chip->usb_suspended) {
		*changed = false;
		goto out;
	}

	*changed = true;
	rc = smbchg_usb_suspend(chip, suspended != 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set usb suspend: %d rc = %d\n",
			suspended, rc);
		goto out;
	}

	pr_smb(PR_STATUS, "usb charging %s, suspended = %02x\n",
			suspended == 0 ? "enabled"
			: "suspended", suspended);
out:
	chip->usb_suspended = suspended;
	mutex_unlock(&chip->usb_en_lock);
	return rc;
}

static int smbchg_dc_en(struct smbchg_chip *chip, bool enable,
		enum enable_reason reason)
{
	int rc = 0, suspended;

	pr_smb(PR_STATUS, "dc %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->dc_suspended == 0 ? "enabled"
			: "suspended", chip->dc_suspended, enable, reason);
	mutex_lock(&chip->dc_en_lock);
	if (!enable)
		suspended = chip->dc_suspended | reason;
	else
		suspended = chip->dc_suspended & ~reason;

	/* avoid unnecessary spmi interactions if nothing changed */
	if (!!suspended == !!chip->dc_suspended)
		goto out;

	rc = smbchg_dc_suspend(chip, suspended != 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set dc suspend: %d rc = %d\n",
			suspended, rc);
		goto out;
	}

	if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
		power_supply_changed(&chip->dc_psy);
	pr_smb(PR_STATUS, "dc charging %s, suspended = %02x\n",
			suspended == 0 ? "enabled"
			: "suspended", suspended);
out:
	chip->dc_suspended = suspended;
	mutex_unlock(&chip->dc_en_lock);
	return rc;
}

#define CHGPTH_CFG		0xF4
#define CFG_USB_2_3_SEL_BIT	BIT(7)
#define CFG_USB_2		0
#define CFG_USB_3		BIT(7)
#define USBIN_INPUT_MASK	SMB_MASK(4, 0)
#define USBIN_MODE_CHG_BIT	BIT(0)
#define USBIN_LIMITED_MODE	0
#define USBIN_HC_MODE		BIT(0)
#define USB51_MODE_BIT		BIT(1)
#define USB51_100MA		0
#define USB51_500MA		BIT(1)
static int smbchg_set_high_usb_chg_current(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 usb_cur_val;

	for (i = ARRAY_SIZE(usb_current_table) - 1; i >= 0; i--) {
		if (current_ma >= usb_current_table[i])
			break;
	}
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_150_MA);

		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_150_MA, rc);
		else
			chip->usb_max_current_ma = 150;
		return rc;
	}

	usb_cur_val = i & USBIN_INPUT_MASK;
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG,
				USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to config c rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	chip->usb_max_current_ma = usb_current_table[i];
	return rc;
}

/* if APSD results are used
 *	if SDP is detected it will look at 500mA setting
 *		if set it will draw 500mA
 *		if unset it will draw 100mA
 *	if CDP/DCP it will look at 0x0C setting
 *		i.e. values in 0x41[1, 0] does not matter
 */
static int smbchg_set_usb_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;
	bool changed;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	if (!chip->batt_present) {
		pr_info_ratelimited("Ignoring usb current->%d, battery is absent\n",
				current_ma);
		return 0;
	}
	pr_smb(PR_STATUS, "USB current_ma = %d\n", current_ma);

	if (current_ma == SUSPEND_CURRENT_MA) {
		/* suspend the usb if current set to 2mA */
		rc = smbchg_primary_usb_en(chip, false, REASON_USB, &changed);
		chip->usb_max_current_ma = 0;
		goto out;
	} else {
		rc = smbchg_primary_usb_en(chip, true, REASON_USB, &changed);
	}

	read_usb_type(chip, &usb_type_name, &usb_supply_type);

	switch (usb_supply_type) {
	case POWER_SUPPLY_TYPE_USB:
		if (current_ma < CURRENT_150_MA) {
			/* force 100mA */
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 100;
		}
		/* specific current values */
		if (current_ma == CURRENT_150_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 150;
		}
		if (current_ma == CURRENT_500_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 500;
		}
		if (current_ma == CURRENT_900_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 900;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		if (current_ma < CURRENT_1500_MA) {
			/* use override for CDP */
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
			if (rc < 0)
				pr_err("Couldn't set override rc = %d\n", rc);
		}
		/* fall through */
	default:
		rc = smbchg_set_high_usb_chg_current(chip, current_ma);
		if (rc < 0)
			pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
		break;
	}

out:
	pr_smb(PR_STATUS, "usb type = %s current set to %d mA\n",
			usb_type_name, chip->usb_max_current_ma);
	return rc;
}

#define USBIN_HVDCP_STS				0x0C
#define USBIN_HVDCP_SEL_BIT			BIT(4)
#define USBIN_HVDCP_SEL_9V_BIT			BIT(1)
#define SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT	BIT(2)
#define SCHG_LITE_USBIN_HVDCP_SEL_BIT		BIT(0)
static int smbchg_get_min_parallel_current_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel, hvdcp_sel_9v;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return 0;
	}
	if (chip->schg_version == QPNP_SCHG_LITE) {
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT;
	} else {
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = USBIN_HVDCP_SEL_9V_BIT;
	}

	if ((reg & hvdcp_sel) && (reg & hvdcp_sel_9v))
		return chip->parallel.min_9v_current_thr_ma;
	return chip->parallel.min_current_thr_ma;
}

#define ICL_STS_1_REG			0x7
#define ICL_STS_2_REG			0x9
#define ICL_STS_MASK			0x1F
#define AICL_SUSP_BIT			BIT(6)
#define AICL_STS_BIT			BIT(5)
#define USBIN_SUSPEND_STS_BIT		BIT(3)
#define USBIN_ACTIVE_PWR_SRC_BIT	BIT(1)
#define DCIN_ACTIVE_PWR_SRC_BIT		BIT(0)
#define PARALLEL_REENABLE_TIMER_MS	30000
static bool smbchg_is_parallel_usb_ok(struct smbchg_chip *chip)
{
	int min_current_thr_ma, rc, type;
	ktime_t kt_since_last_disable;
	u8 reg;

	if (!smbchg_parallel_en || !chip->parallel_charger_detected) {
		pr_smb(PR_STATUS, "Parallel charging not enabled\n");
		return false;
	}

	kt_since_last_disable = ktime_sub(ktime_get_boottime(),
					chip->parallel.last_disabled);
	if (chip->parallel.current_max_ma == 0
		&& chip->parallel.enabled_once
		&& ktime_to_ms(kt_since_last_disable)
			< PARALLEL_REENABLE_TIMER_MS) {
		pr_smb(PR_STATUS, "Only been %lld since disable, skipping\n",
				ktime_to_ms(kt_since_last_disable));
		return false;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return false;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return false;
	}

	type = get_type(reg);
	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB_CDP) {
		pr_smb(PR_STATUS, "CDP adapter, skipping\n");
		return false;
	}

	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB) {
		pr_smb(PR_STATUS, "SDP adapter, skipping\n");
		return false;
	}

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_2_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 2: %d\n", rc);
		return false;
	}

	/*
	 * If USBIN is suspended or not the active power source, do not enable
	 * parallel charging. The device may be charging off of DCIN.
	 */
	if (!!(reg & USBIN_SUSPEND_STS_BIT) ||
				!(reg & USBIN_ACTIVE_PWR_SRC_BIT)) {
		pr_smb(PR_STATUS, "USB not active power source: %02x\n", reg);
		return false;
	}

	min_current_thr_ma = smbchg_get_min_parallel_current_ma(chip);
	if (min_current_thr_ma <= 0) {
		pr_smb(PR_STATUS, "parallel charger unavailable for thr: %d\n",
				min_current_thr_ma);
		return false;
	}
	if (chip->usb_tl_current_ma < min_current_thr_ma) {
		pr_smb(PR_STATUS, "Weak USB chg skip enable: %d < %d\n",
			chip->usb_tl_current_ma, min_current_thr_ma);
		return false;
	}

	return true;
}

#define FCC_CFG			0xF2
#define FCC_500MA_VAL		0x4
#define FCC_MASK		SMB_MASK(4, 0)
static int smbchg_set_fastchg_current_raw(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 cur_val;

	/* the fcc enumerations are the same as the usb currents */
	for (i = ARRAY_SIZE(usb_current_table) - 1; i >= 0; i--) {
		if (current_ma >= usb_current_table[i])
			break;
	}
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_500_MA);

		rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
					FCC_MASK,
					FCC_500MA_VAL);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_500_MA, rc);
		else
			chip->fastchg_current_ma = 500;
		return rc;
	}

	cur_val = i & FCC_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
				FCC_MASK, cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to fcc cfg rc = %d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "fastcharge current requested %d, set to %d\n",
			current_ma, usb_current_table[cur_val]);

	chip->fastchg_current_ma = usb_current_table[cur_val];
	return rc;
}

static int smbchg_set_fastchg_current(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;

	mutex_lock(&chip->fcc_lock);
	if (chip->sw_esr_pulse_en)
		current_ma = 300;
	/* If the requested FCC is same, do not configure it again */
	if (current_ma == chip->fastchg_current_ma) {
		pr_smb(PR_STATUS, "not configuring FCC current: %d FCC: %d\n",
			current_ma, chip->fastchg_current_ma);
		goto out;
	}
	rc = smbchg_set_fastchg_current_raw(chip, current_ma);
out:
	mutex_unlock(&chip->fcc_lock);
	return rc;
}

static int smbchg_parallel_usb_charging_en(struct smbchg_chip *chip, bool en)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };

	if (!parallel_psy || !chip->parallel_charger_detected)
		return 0;

	pval.intval = en;
	return parallel_psy->set_property(parallel_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
}

static int smbchg_sw_esr_pulse_en(struct smbchg_chip *chip, bool en)
{
	int rc;

	chip->sw_esr_pulse_en = en;
	rc = smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	if (rc)
		return rc;
	rc = smbchg_parallel_usb_charging_en(chip, !en);
	return rc;
}

#define USB_AICL_CFG				0xF3
#define AICL_EN_BIT				BIT(2)
static void smbchg_rerun_aicl(struct smbchg_chip *chip)
{
	pr_smb(PR_STATUS, "Rerunning AICL...\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	/* Add a delay so that AICL successfully clears */
	msleep(50);
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
}

static void taper_irq_en(struct smbchg_chip *chip, bool en)
{
	mutex_lock(&chip->taper_irq_lock);
	if (en != chip->taper_irq_enabled) {
		if (en) {
			enable_irq(chip->taper_irq);
			enable_irq_wake(chip->taper_irq);
		} else {
			disable_irq_wake(chip->taper_irq);
			disable_irq_nosync(chip->taper_irq);
		}
		chip->taper_irq_enabled = en;
	}
	mutex_unlock(&chip->taper_irq_lock);
}

static void smbchg_parallel_usb_disable(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;
	pr_smb(PR_STATUS, "disabling parallel charger\n");
	chip->parallel.last_disabled = ktime_get_boottime();
	taper_irq_en(chip, false);
	chip->parallel.initial_aicl_ma = 0;
	chip->parallel.current_max_ma = 0;
	power_supply_set_current_limit(parallel_psy,
				SUSPEND_CURRENT_MA * 1000);
	power_supply_set_present(parallel_psy, false);
	chip->target_fastchg_current_ma = chip->cfg_fastchg_current_ma;
	smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	chip->usb_tl_current_ma =
		calc_thermal_limited_current(chip, chip->usb_target_current_ma);
	smbchg_set_usb_current_max(chip, chip->usb_tl_current_ma);
	smbchg_rerun_aicl(chip);
}

#define PARALLEL_TAPER_MAX_TRIES		3
#define PARALLEL_FCC_PERCENT_REDUCTION		75
#define MINIMUM_PARALLEL_FCC_MA			500
#define CHG_ERROR_BIT		BIT(0)
#define BAT_TAPER_MODE_BIT	BIT(6)
static void smbchg_parallel_usb_taper(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int parallel_fcc_ma, tries = 0;
	u8 reg = 0;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_TAPER);
try_again:
	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	tries += 1;
	parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_STATUS, "try #%d parallel charger fcc = %d\n",
			tries, parallel_fcc_ma);
	if (parallel_fcc_ma < MINIMUM_PARALLEL_FCC_MA
				|| tries > PARALLEL_TAPER_MAX_TRIES) {
		smbchg_parallel_usb_disable(chip);
		goto done;
	}
	pval.intval = ((parallel_fcc_ma
			* PARALLEL_FCC_PERCENT_REDUCTION) / 100);
	pr_smb(PR_STATUS, "reducing FCC of parallel charger to %d\n",
		pval.intval);
	/* Change it to uA */
	pval.intval *= 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	/*
	 * sleep here for 100 ms in order to make sure the charger has a chance
	 * to go back into constant current charging
	 */
	mutex_unlock(&chip->parallel.lock);
	msleep(100);

	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (reg & BAT_TAPER_MODE_BIT) {
		mutex_unlock(&chip->parallel.lock);
		goto try_again;
	}
	taper_irq_en(chip, true);
done:
	mutex_unlock(&chip->parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_TAPER);
}

static bool smbchg_is_aicl_complete(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return true;
	}
	return (reg & AICL_STS_BIT) != 0;
}

static int smbchg_get_aicl_level_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return 0;
	}
	if (reg & AICL_SUSP_BIT) {
		pr_warn("AICL suspended: %02x\n", reg);
		return 0;
	}
	reg &= ICL_STS_MASK;
	if (reg >= ARRAY_SIZE(usb_current_table)) {
		pr_warn("invalid AICL value: %02x\n", reg);
		return 0;
	}
	return usb_current_table[reg];
}

#define PARALLEL_CHG_THRESHOLD_CURRENT	1800
static void smbchg_parallel_usb_enable(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int current_limit_ma, parallel_cl_ma, total_current_ma;
	int new_parallel_cl_ma, min_current_thr_ma, rc;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	pr_smb(PR_STATUS, "Attempting to enable parallel charger\n");
	/* Suspend the parallel charger if the charging current is < 1800 mA */
	if (chip->cfg_fastchg_current_ma < PARALLEL_CHG_THRESHOLD_CURRENT) {
		pr_smb(PR_STATUS, "suspend parallel charger as FCC is %d\n",
			chip->cfg_fastchg_current_ma);
		goto disable_parallel;
	}
	min_current_thr_ma = smbchg_get_min_parallel_current_ma(chip);
	if (min_current_thr_ma <= 0) {
		pr_smb(PR_STATUS, "parallel charger unavailable for thr: %d\n",
				min_current_thr_ma);
		goto disable_parallel;
	}

	current_limit_ma = smbchg_get_aicl_level_ma(chip);
	if (current_limit_ma <= 0)
		goto disable_parallel;

	if (chip->parallel.initial_aicl_ma == 0) {
		if (current_limit_ma < min_current_thr_ma) {
			pr_smb(PR_STATUS, "Initial AICL very low: %d < %d\n",
				current_limit_ma, min_current_thr_ma);
			goto disable_parallel;
		}
		chip->parallel.initial_aicl_ma = current_limit_ma;
	}

	/*
	 * Use the previous set current from the parallel charger.
	 * Treat 2mA as 0 because that is the suspend current setting
	 */
	parallel_cl_ma = chip->parallel.current_max_ma;
	if (parallel_cl_ma <= SUSPEND_CURRENT_MA)
		parallel_cl_ma = 0;

	/*
	 * Set the parallel charge path's input current limit (ICL)
	 * to the total current / 2
	 */
	total_current_ma = current_limit_ma + parallel_cl_ma;

	if (total_current_ma < chip->parallel.initial_aicl_ma
			- chip->parallel.allowed_lowering_ma) {
		pr_smb(PR_STATUS,
			"Too little total current : %d (%d + %d) < %d - %d\n",
			total_current_ma,
			current_limit_ma, parallel_cl_ma,
			chip->parallel.initial_aicl_ma,
			chip->parallel.allowed_lowering_ma);
		goto disable_parallel;
	}

	rc = power_supply_set_voltage_limit(parallel_psy, chip->vfloat_mv + 50);
	if (rc) {
		dev_err(chip->dev, "Couldn't set float voltage on parallel psy rc: %d\n",
			rc);
		goto disable_parallel;
	}
	chip->target_fastchg_current_ma = chip->cfg_fastchg_current_ma / 2;
	smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	pval.intval = chip->target_fastchg_current_ma * 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);

	chip->parallel.enabled_once = true;
	new_parallel_cl_ma = total_current_ma / 2;

	if (new_parallel_cl_ma == parallel_cl_ma) {
		pr_smb(PR_STATUS,
			"AICL at %d, old ICL: %d new ICL: %d, skipping\n",
			current_limit_ma, parallel_cl_ma, new_parallel_cl_ma);
		return;
	} else {
		pr_smb(PR_STATUS, "AICL at %d, old ICL: %d new ICL: %d\n",
			current_limit_ma, parallel_cl_ma, new_parallel_cl_ma);
	}

	taper_irq_en(chip, true);
	chip->parallel.current_max_ma = new_parallel_cl_ma;
	power_supply_set_present(parallel_psy, true);
	smbchg_set_usb_current_max(chip, chip->parallel.current_max_ma);
	power_supply_set_current_limit(parallel_psy,
				chip->parallel.current_max_ma * 1000);
	return;

disable_parallel:
	if (chip->parallel.current_max_ma != 0) {
		pr_smb(PR_STATUS, "disabling parallel charger\n");
		smbchg_parallel_usb_disable(chip);
	} else if (chip->cfg_fastchg_current_ma !=
			chip->target_fastchg_current_ma) {
		/* There is a possibility that parallel charging is enabled
		 * and a weak charger is connected, AICL result will be
		 * lower than the min_current_thr_ma. In those cases, we
		 * should fall back to configure the FCC of main charger.
		 */
		rc = smbchg_set_fastchg_current(chip,
				chip->cfg_fastchg_current_ma);
		if (rc)
			pr_err("Couldn't set fastchg current rc: %d\n",
				rc);
		else
			chip->target_fastchg_current_ma =
				chip->cfg_fastchg_current_ma;
	}
}

static void smbchg_parallel_usb_en_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				parallel_en_work.work);

	smbchg_relax(chip, PM_PARALLEL_CHECK);
	mutex_lock(&chip->parallel.lock);
	if (smbchg_is_parallel_usb_ok(chip)) {
		smbchg_parallel_usb_enable(chip);
	} else if (chip->parallel.current_max_ma != 0) {
		pr_smb(PR_STATUS, "parallel charging unavailable\n");
		smbchg_parallel_usb_disable(chip);
	}
	mutex_unlock(&chip->parallel.lock);
}

#define PARALLEL_CHARGER_EN_DELAY_MS	3500
static void smbchg_parallel_usb_check_ok(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;
	mutex_lock(&chip->parallel.lock);
	if (smbchg_is_parallel_usb_ok(chip)) {
		smbchg_stay_awake(chip, PM_PARALLEL_CHECK);
		schedule_delayed_work(
			&chip->parallel_en_work,
			msecs_to_jiffies(PARALLEL_CHARGER_EN_DELAY_MS));
	} else if (chip->parallel.current_max_ma != 0) {
		pr_smb(PR_STATUS, "parallel charging unavailable\n");
		smbchg_parallel_usb_disable(chip);
	}
	mutex_unlock(&chip->parallel.lock);
}

static int smbchg_usb_en(struct smbchg_chip *chip, bool enable,
		enum enable_reason reason)
{
	bool changed = false;
	int rc = smbchg_primary_usb_en(chip, enable, reason, &changed);

	if (changed)
		smbchg_parallel_usb_check_ok(chip);
	return rc;
}

static int smbchg_set_fastchg_current_user(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;

	mutex_lock(&chip->parallel.lock);
	pr_smb(PR_STATUS, "User setting FCC to %d\n", current_ma);
	chip->cfg_fastchg_current_ma = current_ma;
	if (smbchg_is_parallel_usb_ok(chip)) {
		smbchg_parallel_usb_enable(chip);
	} else {
		if (chip->parallel.current_max_ma != 0) {
			/*
			 * If parallel charging is not available, disable it.
			 * FCC for main charger will be configured in that.
			 */
			pr_smb(PR_STATUS, "parallel charging unavailable\n");
			smbchg_parallel_usb_disable(chip);
			goto out;
		}
		rc = smbchg_set_fastchg_current(chip,
				chip->cfg_fastchg_current_ma);
		if (rc)
			pr_err("Couldn't set fastchg current rc: %d\n",
				rc);
	}
out:
	mutex_unlock(&chip->parallel.lock);
	return rc;
}

static struct ilim_entry *smbchg_wipower_find_entry(struct smbchg_chip *chip,
				struct ilim_map *map, int uv)
{
	int i;
	struct ilim_entry *ret = &(chip->wipower_default.entries[0]);

	for (i = 0; i < map->num; i++) {
		if (is_between(map->entries[i].vmin_uv, map->entries[i].vmax_uv,
			uv))
			ret = &map->entries[i];
	}
	return ret;
}

static int ilim_ma_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

#define ZIN_ICL_PT	0xFC
#define ZIN_ICL_LV	0xFD
#define ZIN_ICL_HV	0xFE
#define ZIN_ICL_MASK	SMB_MASK(4, 0)
static int smbchg_dcin_ilim_config(struct smbchg_chip *chip, int offset, int ma)
{
	int i, rc;

	for (i = ARRAY_SIZE(ilim_ma_table) - 1; i >= 0; i--) {
		if (ma >= ilim_ma_table[i])
			break;
	}

	if (i < 0)
		i = 0;

	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + offset,
			ZIN_ICL_MASK, i);
	if (rc)
		dev_err(chip->dev, "Couldn't write bat if offset %d value = %d rc = %d\n",
				offset, i, rc);
	return rc;
}

static int smbchg_wipower_ilim_config(struct smbchg_chip *chip,
						struct ilim_entry *ilim)
{
	int rc = 0;

	if (chip->current_ilim.icl_pt_ma != ilim->icl_pt_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_PT, ilim->icl_pt_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_PT, ilim->icl_pt_ma, rc);
		else
			chip->current_ilim.icl_pt_ma =  ilim->icl_pt_ma;
	}

	if (chip->current_ilim.icl_lv_ma !=  ilim->icl_lv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_LV, ilim->icl_lv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_LV, ilim->icl_lv_ma, rc);
		else
			chip->current_ilim.icl_lv_ma =  ilim->icl_lv_ma;
	}

	if (chip->current_ilim.icl_hv_ma !=  ilim->icl_hv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_HV, ilim->icl_hv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_HV, ilim->icl_hv_ma, rc);
		else
			chip->current_ilim.icl_hv_ma =  ilim->icl_hv_ma;
	}
	return rc;
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx);
static int smbchg_wipower_dcin_btm_configure(struct smbchg_chip *chip,
		struct ilim_entry *ilim)
{
	int rc;

	if (ilim->vmin_uv == chip->current_ilim.vmin_uv
			&& ilim->vmax_uv == chip->current_ilim.vmax_uv)
		return 0;

	chip->param.channel = DCIN;
	chip->param.btm_ctx = chip;
	if (wipower_dcin_interval < ADC_MEAS1_INTERVAL_0MS)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_0MS;

	if (wipower_dcin_interval > ADC_MEAS1_INTERVAL_16S)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_16S;

	chip->param.timer_interval = wipower_dcin_interval;
	chip->param.threshold_notification = &btm_notify_dcin;
	chip->param.high_thr = ilim->vmax_uv + wipower_dcin_hyst_uv;
	chip->param.low_thr = ilim->vmin_uv - wipower_dcin_hyst_uv;
	chip->param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	rc = qpnp_vadc_channel_monitor(chip->vadc_dev, &chip->param);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure btm for dcin rc = %d\n",
				rc);
	} else {
		chip->current_ilim.vmin_uv = ilim->vmin_uv;
		chip->current_ilim.vmax_uv = ilim->vmax_uv;
		pr_smb(PR_STATUS, "btm ilim = (%duV %duV %dmA %dmA %dmA)\n",
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
	}
	return rc;
}

static int smbchg_wipower_icl_configure(struct smbchg_chip *chip,
						int dcin_uv, bool div2)
{
	int rc = 0;
	struct ilim_map *map = div2 ? &chip->wipower_div2 : &chip->wipower_pt;
	struct ilim_entry *ilim = smbchg_wipower_find_entry(chip, map, dcin_uv);

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config ilim rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}

	rc = smbchg_wipower_dcin_btm_configure(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config btm rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}
	chip->wipower_configured = true;
	return 0;
}

static void smbchg_wipower_icl_deconfigure(struct smbchg_chip *chip)
{
	int rc;
	struct ilim_entry *ilim = &(chip->wipower_default.entries[0]);

	if (!chip->wipower_configured)
		return;

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc)
		dev_err(chip->dev, "Couldn't config default ilim rc = %d\n",
				rc);

	rc = qpnp_vadc_end_channel_monitor(chip->vadc_dev);
	if (rc)
		dev_err(chip->dev, "Couldn't de configure btm for dcin rc = %d\n",
				rc);

	chip->wipower_configured = false;
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	chip->current_ilim.icl_pt_ma = ilim->icl_pt_ma;
	chip->current_ilim.icl_lv_ma = ilim->icl_lv_ma;
	chip->current_ilim.icl_hv_ma = ilim->icl_hv_ma;
	pr_smb(PR_WIPOWER, "De config btm\n");
}

#define FV_STS		0x0C
#define DIV2_ACTIVE	BIT(7)
static void __smbchg_wipower_check(struct smbchg_chip *chip)
{
	int chg_type;
	bool usb_present, dc_present;
	int rc;
	int dcin_uv;
	bool div2;
	struct qpnp_vadc_result adc_result;
	u8 reg;

	if (!wipower_dyn_icl_en) {
		smbchg_wipower_icl_deconfigure(chip);
		return;
	}

	chg_type = get_prop_charge_type(chip);
	usb_present = is_usb_present(chip);
	dc_present = is_dc_present(chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE
			 && !usb_present
			&& dc_present
			&& chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER) {
		rc = qpnp_vadc_read(chip->vadc_dev, DCIN, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		dcin_uv = adc_result.physical;

		/* check div_by_2 */
		rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS, 1);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		div2 = !!(reg & DIV2_ACTIVE);

		pr_smb(PR_WIPOWER,
			"config ICL chg_type = %d usb = %d dc = %d dcin_uv(adc_code) = %d (0x%x) div2 = %d\n",
			chg_type, usb_present, dc_present, dcin_uv,
			adc_result.adc_code, div2);
		smbchg_wipower_icl_configure(chip, dcin_uv, div2);
	} else {
		pr_smb(PR_WIPOWER,
			"deconfig ICL chg_type = %d usb = %d dc = %d\n",
			chg_type, usb_present, dc_present);
		smbchg_wipower_icl_deconfigure(chip);
	}
}

static void smbchg_wipower_check(struct smbchg_chip *chip)
{
	if (!chip->wipower_dyn_icl_avail)
		return;

	mutex_lock(&chip->wipower_config);
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx)
{
	struct smbchg_chip *chip = ctx;

	mutex_lock(&chip->wipower_config);
	pr_smb(PR_WIPOWER, "%s state\n",
			state  == ADC_TM_LOW_STATE ? "low" : "high");
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static int force_dcin_icl_write(void *data, u64 val)
{
	struct smbchg_chip *chip = data;

	smbchg_wipower_check(chip);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_dcin_icl_ops, NULL,
		force_dcin_icl_write, "0x%02llx\n");

/*
 * set the dc charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int smbchg_set_thermal_limited_dc_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	current_ma = calc_thermal_limited_current(chip, current_ma);
	return smbchg_set_dc_current_max(chip, current_ma);
}

/*
 * set the usb charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int smbchg_set_thermal_limited_usb_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	int rc, aicl_ma;

	aicl_ma = smbchg_get_aicl_level_ma(chip);
	chip->usb_tl_current_ma =
		calc_thermal_limited_current(chip, current_ma);
	rc = smbchg_set_usb_current_max(chip, chip->usb_tl_current_ma);
	if (rc) {
		pr_err("Failed to set usb current max: %d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "AICL = %d, ICL = %d\n",
			aicl_ma, chip->usb_max_current_ma);
	if (chip->usb_max_current_ma > aicl_ma && smbchg_is_aicl_complete(chip))
		smbchg_rerun_aicl(chip);
	smbchg_parallel_usb_check_ok(chip);
	return rc;
}

static int smbchg_system_temp_level_set(struct smbchg_chip *chip,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!chip->thermal_mitigation) {
		dev_err(chip->dev, "Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		dev_err(chip->dev, "Unsupported level selected %d forcing %d\n",
				lvl_sel, chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		/*
		 * Disable charging if highest value selected by
		 * setting the DC and USB path in suspend
		 */
		rc = smbchg_dc_en(chip, false, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = smbchg_usb_en(chip, false, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	rc = smbchg_set_thermal_limited_usb_current_max(chip,
					chip->usb_target_current_ma);
	rc = smbchg_set_thermal_limited_dc_current_max(chip,
					chip->dc_target_current_ma);

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging by taking the DC and USB path
		 * out of suspend.
		 */
		rc = smbchg_dc_en(chip, true, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = smbchg_usb_en(chip, true, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->current_change_lock);
	return rc;
}

static int smbchg_ibat_ocp_threshold_ua = 4500000;
module_param(smbchg_ibat_ocp_threshold_ua, int, 0644);

#define UCONV			1000000LL
#define MCONV			1000LL
#define FLASH_V_THRESHOLD	3000000
#define FLASH_VDIP_MARGIN	100000
#define VPH_FLASH_VDIP		(FLASH_V_THRESHOLD + FLASH_VDIP_MARGIN)
#define BUCK_EFFICIENCY		800LL
static int smbchg_calc_max_flash_current(struct smbchg_chip *chip)
{
	int ocv_uv, esr_uohm, rbatt_uohm, ibat_now, rc;
	int64_t ibat_flash_ua, avail_flash_ua, avail_flash_power_fw;
	int64_t ibat_safe_ua, vin_flash_uv, vph_flash_uv;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_OCV, &ocv_uv);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_RESISTANCE,
			&esr_uohm);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support resistance\n");
		return 0;
	}

	rc = msm_bcl_read(BCL_PARAM_CURRENT, &ibat_now);
	if (rc) {
		pr_smb(PR_STATUS, "BCL current read failed: %d\n", rc);
		return 0;
	}

	rbatt_uohm = esr_uohm + chip->rpara_uohm + chip->rslow_uohm;
	/*
	 * Calculate the maximum current that can pulled out of the battery
	 * before the battery voltage dips below a safe threshold.
	 */
	ibat_safe_ua = div_s64((ocv_uv - VPH_FLASH_VDIP) * UCONV,
				rbatt_uohm);

	if (ibat_safe_ua <= smbchg_ibat_ocp_threshold_ua) {
		/*
		 * If the calculated current is below the OCP threshold, then
		 * use it as the possible flash current.
		 */
		ibat_flash_ua = ibat_safe_ua - ibat_now;
		vph_flash_uv = VPH_FLASH_VDIP;
	} else {
		/*
		 * If the calculated current is above the OCP threshold, then
		 * use the ocp threshold instead.
		 *
		 * Any higher current will be tripping the battery OCP.
		 */
		ibat_flash_ua = smbchg_ibat_ocp_threshold_ua - ibat_now;
		vph_flash_uv = ocv_uv - div64_s64((int64_t)rbatt_uohm
				* smbchg_ibat_ocp_threshold_ua, UCONV);
	}
	/* Calculate the input voltage of the flash module. */
	vin_flash_uv = max((chip->vled_max_uv + 500000LL),
				div64_s64((vph_flash_uv * 1200), 1000));
	/* Calculate the available power for the flash module. */
	avail_flash_power_fw = BUCK_EFFICIENCY * vph_flash_uv * ibat_flash_ua;
	/*
	 * Calculate the available amount of current the flash module can draw
	 * before collapsing the battery. (available power/ flash input voltage)
	 */
	avail_flash_ua = div64_s64(avail_flash_power_fw, vin_flash_uv * MCONV);
	pr_smb(PR_MISC,
		"avail_iflash=%lld, ocv=%d, ibat=%d, rbatt=%d\n",
		avail_flash_ua, ocv_uv, ibat_now, rbatt_uohm);
	return (int)avail_flash_ua;
}

#define FCC_CMP_CFG	0xF3
#define FCC_COMP_MASK	SMB_MASK(1, 0)
static int smbchg_fastchg_current_comp_set(struct smbchg_chip *chip,
					int comp_current)
{
	int rc;
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fcc_comp_table); i++)
		if (comp_current == fcc_comp_table[i])
			break;

	if (i >= ARRAY_SIZE(fcc_comp_table))
		return -EINVAL;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CMP_CFG,
			FCC_COMP_MASK, i);

	if (rc)
		dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
			rc);

	return rc;
}

#define FV_CMP_CFG	0xF5
#define FV_COMP_MASK	SMB_MASK(5, 0)
static int smbchg_float_voltage_comp_set(struct smbchg_chip *chip, int code)
{
	int rc;
	u8 val;

	val = code & FV_COMP_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FV_CMP_CFG,
			FV_COMP_MASK, val);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
			rc);

	return rc;
}

#define VFLOAT_CFG_REG			0xF4
#define MIN_FLOAT_MV			3600
#define MAX_FLOAT_MV			4500
#define VFLOAT_MASK			SMB_MASK(5, 0)

#define MID_RANGE_FLOAT_MV_MIN		3600
#define MID_RANGE_FLOAT_MIN_VAL		0x05
#define MID_RANGE_FLOAT_STEP_MV		20

#define HIGH_RANGE_FLOAT_MIN_MV		4340
#define HIGH_RANGE_FLOAT_MIN_VAL	0x2A
#define HIGH_RANGE_FLOAT_STEP_MV	10

#define VHIGH_RANGE_FLOAT_MIN_MV	4360
#define VHIGH_RANGE_FLOAT_MIN_VAL	0x2C
#define VHIGH_RANGE_FLOAT_STEP_MV	20
static int smbchg_float_voltage_set(struct smbchg_chip *chip, int vfloat_mv)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc, delta;
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (vfloat_mv <= HIGH_RANGE_FLOAT_MIN_MV) {
		/* mid range */
		delta = vfloat_mv - MID_RANGE_FLOAT_MV_MIN;
		temp = MID_RANGE_FLOAT_MIN_VAL + delta
				/ MID_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % MID_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		/* high range */
		delta = vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV;
		temp = HIGH_RANGE_FLOAT_MIN_VAL + delta
				/ HIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % HIGH_RANGE_FLOAT_STEP_MV;
	} else {
		/* very high range */
		delta = vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV;
		temp = VHIGH_RANGE_FLOAT_MIN_VAL + delta
				/ VHIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % VHIGH_RANGE_FLOAT_STEP_MV;
	}

	if (parallel_psy) {
		rc = power_supply_set_voltage_limit(parallel_psy,
				vfloat_mv + 50);
		if (rc)
			dev_err(chip->dev, "Couldn't set float voltage on parallel psy rc: %d\n",
				rc);
	}

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
			VFLOAT_MASK, temp);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
	else
		chip->vfloat_mv = vfloat_mv;

	return rc;
}

static int smbchg_float_voltage_get(struct smbchg_chip *chip)
{
	return chip->vfloat_mv;
}

#define SFT_CFG				0xFD
#define SFT_EN_MASK			SMB_MASK(5, 4)
#define SFT_TO_MASK			SMB_MASK(3, 2)
#define PRECHG_SFT_TO_MASK		SMB_MASK(1, 0)
#define SFT_TIMER_DISABLE_BIT		BIT(5)
#define PRECHG_SFT_TIMER_DISABLE_BIT	BIT(4)
#define SAFETY_TIME_MINUTES_SHIFT	2
static int smbchg_safety_timer_enable(struct smbchg_chip *chip, bool enable)
{
	int rc;
	u8 reg;

	if (enable == chip->safety_timer_en)
		return 0;

	if (enable)
		reg = 0;
	else
		reg = SFT_TIMER_DISABLE_BIT | PRECHG_SFT_TIMER_DISABLE_BIT;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + SFT_CFG,
			SFT_EN_MASK, reg);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s safety timer rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	chip->safety_timer_en = enable;
	return 0;
}

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

#define OTG_TRIM6		0xF6
#define TR_ENB_SKIP_BIT		BIT(2)
#define OTG_EN_BIT		BIT(0)
static int smbchg_otg_pulse_skip_disable(struct smbchg_chip *chip,
				enum skip_reason reason, bool disable)
{
	int rc;
	bool disabled;

	disabled = !!chip->otg_pulse_skip_dis;
	pr_smb(PR_STATUS, "%s pulse skip, reason %d\n",
			disable ? "disabling" : "enabling", reason);
	if (disable)
		chip->otg_pulse_skip_dis |= reason;
	else
		chip->otg_pulse_skip_dis &= ~reason;
	if (disabled == !!chip->otg_pulse_skip_dis)
		return 0;
	disabled = !!chip->otg_pulse_skip_dis;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_TRIM6,
			TR_ENB_SKIP_BIT, disabled ? TR_ENB_SKIP_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg pulse skip rc = %d\n",
			disabled ? "disable" : "enable", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "%s pulse skip\n", disabled ? "disabled" : "enabled");
	return 0;
}

#define LOW_PWR_OPTIONS_REG	0xFF
#define FORCE_TLIM_BIT		BIT(4)
static int smbchg_force_tlim_en(struct smbchg_chip *chip, bool enable)
{
	int rc;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + LOW_PWR_OPTIONS_REG,
			FORCE_TLIM_BIT, enable ? FORCE_TLIM_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg force tlim rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	return rc;
}

static void smbchg_vfloat_adjust_check(struct smbchg_chip *chip)
{
	if (!chip->use_vfloat_adjustments)
		return;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	pr_smb(PR_STATUS, "Starting vfloat adjustments\n");
	schedule_delayed_work(&chip->vfloat_adjust_work, 0);
}

#define FV_STS_REG			0xC
#define AICL_INPUT_STS_BIT		BIT(6)
static bool smbchg_is_input_current_limited(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read FV_STS rc=%d\n", rc);
		return false;
	}

	return !!(reg & AICL_INPUT_STS_BIT);
}

#define SW_ESR_PULSE_MS			1500
static void smbchg_cc_esr_wa_check(struct smbchg_chip *chip)
{
	int rc, esr_count;

	/* WA is not required on SCHG_LITE */
	if (chip->schg_version == QPNP_SCHG_LITE)
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "No inputs present, skipping\n");
		return;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return;
	}

	if (!smbchg_is_input_current_limited(chip)) {
		pr_smb(PR_STATUS, "Not input current limited, skipping\n");
		return;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_ESR_COUNT, &esr_count);
	if (rc) {
		pr_smb(PR_STATUS,
			"could not read ESR counter rc = %d\n", rc);
		return;
	}

	/*
	 * The esr_count is counting down the number of fuel gauge cycles
	 * before a ESR pulse is needed.
	 *
	 * After a successful ESR pulse, this count is reset to some
	 * high number like 28. If this reaches 0, then the fuel gauge
	 * hardware should force a ESR pulse.
	 *
	 * However, if the device is in constant current charge mode while
	 * being input current limited, the ESR pulse will not affect the
	 * battery current, so the measurement will fail.
	 *
	 * As a failsafe, force a manual ESR pulse if this value is read as
	 * 0.
	 */
	if (esr_count != 0) {
		pr_smb(PR_STATUS, "ESR count is not zero, skipping\n");
		return;
	}

	pr_smb(PR_STATUS, "Lowering charge current for ESR pulse\n");
	smbchg_stay_awake(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, true);
	msleep(SW_ESR_PULSE_MS);
	pr_smb(PR_STATUS, "Raising charge current for ESR pulse\n");
	smbchg_relax(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, false);
}

static void smbchg_soc_changed(struct smbchg_chip *chip)
{
	smbchg_cc_esr_wa_check(chip);
}

#define DC_AICL_CFG			0xF3
#define MISC_TRIM_OPT_15_8		0xF5
#define USB_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_LONG		0
#define DC_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_LONG		0
#define AICL_RERUN_MASK			(BIT(5) | BIT(4))
#define AICL_RERUN_ON			(BIT(5) | BIT(4))
#define AICL_RERUN_OFF			0

static int smbchg_hw_aicl_rerun_en(struct smbchg_chip *chip, bool en)
{
	int rc = 0;

	rc = smbchg_sec_masked_write(chip,
		chip->misc_base + MISC_TRIM_OPT_15_8,
		AICL_RERUN_MASK, en ? AICL_RERUN_ON : AICL_RERUN_OFF);
	if (rc)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n",
			rc);
	return rc;
}

static int smbchg_aicl_config(struct smbchg_chip *chip)
{
	int rc = 0;

	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USB_AICL_CFG,
		USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_LONG);
	if (rc) {
		pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	rc = smbchg_sec_masked_write(chip,
		chip->dc_chgpth_base + DC_AICL_CFG,
		DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_LONG);
	if (rc) {
		pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	if (!chip->very_weak_charger) {
		rc = smbchg_hw_aicl_rerun_en(chip, true);
		if (rc)
			pr_err("Couldn't enable AICL rerun rc= %d\n", rc);
	}
	return rc;
}

static void smbchg_aicl_deglitch_wa_en(struct smbchg_chip *chip, bool en)
{
	int rc;

	if (chip->force_aicl_rerun)
		return;
	if (en && !chip->aicl_deglitch_short) {
/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG,
			USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_SHORT);
#else
		rc = qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG,
								USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_SHORT);
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
		if (rc) {
			pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
			return;
		}
/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
		rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + DC_AICL_CFG,
			DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_SHORT);
#else
		rc = qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1,
								DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_SHORT);
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
		if (rc) {
			pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
			return;
		}
		if (!chip->very_weak_charger) {
			rc = smbchg_hw_aicl_rerun_en(chip, true);
			if (rc) {
				pr_err("Couldn't enable AICL rerun rc= %d\n",
						rc);
				return;
			}
		}
		pr_smb(PR_STATUS, "AICL deglitch set to short\n");
	} else if (!en && chip->aicl_deglitch_short) {
/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG,
			USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_LONG);
#else
		rc = qpnp_charger_write(FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG,
								USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_LONG);
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
		if (rc) {
			pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
			return;
		}
/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
		rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + DC_AICL_CFG,
			DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_LONG);
#else
		rc = qpnp_charger_write(FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1,
								DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_LONG);
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
		if (rc) {
			pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
			return;
		}
		rc = smbchg_hw_aicl_rerun_en(chip, false);
		if (rc) {
			pr_err("Couldn't disable AICL rerun rc= %d\n", rc);
			return;
		}
		pr_smb(PR_STATUS, "AICL deglitch set to normal\n");
	}
	chip->aicl_deglitch_short = en;
}

static void smbchg_aicl_deglitch_wa_check(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	int rc;
	u8 reg;
	bool low_volt_chgr = true;

	if (!(chip->wa_flags & SMBCHG_AICL_DEGLITCH_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "Charger removed\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	if (!chip->bms_psy)
		return;

	if (is_usb_present(chip)) {
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
		if (rc < 0) {
			pr_err("Couldn't read hvdcp status rc = %d\n", rc);
			return;
		}
		if (reg & USBIN_HVDCP_SEL_BIT)
			low_volt_chgr = false;
	} else if (is_dc_present(chip)) {
		if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
			low_volt_chgr = false;
		else
			low_volt_chgr = chip->low_volt_dcin;
	}

	if (!low_volt_chgr) {
		pr_smb(PR_STATUS, "High volt charger! Don't set deglitch\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	/* It is possible that battery voltage went high above threshold
	 * when the charger is inserted and can go low because of system
	 * load. We shouldn't be reconfiguring AICL deglitch when this
	 * happens as it will lead to oscillation again which is being
	 * fixed here. Do it once when the battery voltage crosses the
	 * threshold (e.g. 4.2 V) and clear it only when the charger
	 * is removed.
	 */
	if (!chip->vbat_above_headroom) {
		rc = chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MIN, &prop);
		if (rc < 0) {
			pr_err("could not read voltage_min, rc=%d\n", rc);
			return;
		}
		chip->vbat_above_headroom = !prop.intval;
	}
	smbchg_aicl_deglitch_wa_en(chip, chip->vbat_above_headroom);
}

#define UNKNOWN_BATT_TYPE	"Unknown Battery"
#define LOADING_BATT_TYPE	"Loading Battery Data"
static int smbchg_config_chg_battery_type(struct smbchg_chip *chip)
{
	int rc = 0, max_voltage_uv = 0, fastchg_ma = 0, ret = 0;
	struct device_node *batt_node, *profile_node;
	struct device_node *node = chip->spmi->dev.of_node;
	union power_supply_propval prop = {0,};

	rc = chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
	if (rc) {
		pr_smb(PR_STATUS, "Unable to read battery-type rc=%d\n", rc);
		return 0;
	}
	if (!strcmp(prop.strval, UNKNOWN_BATT_TYPE) ||
		!strcmp(prop.strval, LOADING_BATT_TYPE)) {
		pr_smb(PR_MISC, "Battery-type not identified\n");
		return 0;
	}
	/* quit if there is no change in the battery-type from previous */
	if (chip->battery_type && !strcmp(prop.strval, chip->battery_type))
		return 0;

	batt_node = of_parse_phandle(node, "qcom,battery-data", 0);
	if (!batt_node) {
		pr_smb(PR_MISC, "No batterydata available\n");
		return 0;
	}

	profile_node = of_batterydata_get_best_profile(batt_node,
							"bms", NULL);
	if (!profile_node) {
		pr_err("couldn't find profile handle\n");
		return -EINVAL;
	}
	chip->battery_type = prop.strval;

	/* change vfloat */
	rc = of_property_read_u32(profile_node, "qcom,max-voltage-uv",
						&max_voltage_uv);
	if (rc) {
		pr_warn("couldn't find battery max voltage rc=%d\n", rc);
		ret = rc;
	} else {
		if (chip->vfloat_mv != (max_voltage_uv / 1000)) {
			pr_info("Vfloat changed from %dmV to %dmV for battery-type %s\n",
				chip->vfloat_mv, (max_voltage_uv / 1000),
				chip->battery_type);
			rc = smbchg_float_voltage_set(chip,
						(max_voltage_uv / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
				return rc;
			}
		}
	}

	/*
	 * Only configure from profile if fastchg-ma is not defined in the
	 * charger device node.
	 */
	if (!of_find_property(chip->spmi->dev.of_node,
				"qcom,fastchg-current-ma", NULL)) {
		rc = of_property_read_u32(profile_node,
				"qcom,fastchg-current-ma", &fastchg_ma);
		if (rc) {
			ret = rc;
		} else {
			pr_smb(PR_MISC,
				"fastchg-ma changed from %dma to %dma for battery-type %s\n",
				chip->target_fastchg_current_ma, fastchg_ma,
				chip->battery_type);
			chip->target_fastchg_current_ma = fastchg_ma;
			chip->cfg_fastchg_current_ma = fastchg_ma;
			rc = smbchg_set_fastchg_current(chip, fastchg_ma);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set fastchg current rc=%d\n",
					rc);
				return rc;
			}
		}
	}

	return ret;
}

static void check_battery_type(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	bool en;
	bool unused;

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
		en = (strcmp(prop.strval, UNKNOWN_BATT_TYPE) != 0
				|| chip->charge_unknown_battery)
			&& (strcmp(prop.strval, LOADING_BATT_TYPE) != 0);
		smbchg_battchg_en(chip, en, REASON_BATTCHG_UNKNOWN_BATTERY,
				&unused);
	}
}

static void smbchg_external_power_changed(struct power_supply *psy)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0, soc;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->bms_psy) {
		check_battery_type(chip);
		soc = get_prop_batt_capacity(chip);
		if (chip->previous_soc != soc) {
			chip->previous_soc = soc;
			smbchg_soc_changed(chip);
		}

		rc = smbchg_config_chg_battery_type(chip);
		if (rc)
			pr_smb(PR_MISC,
				"Couldn't update charger configuration rc=%d\n",
									rc);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (rc < 0)
		pr_smb(PR_MISC, "could not read USB charge_en, rc=%d\n",
				rc);
	else
		smbchg_usb_en(chip, prop.intval, REASON_POWER_SUPPLY);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if (usb_supply_type != POWER_SUPPLY_TYPE_USB)
		goto  skip_current_for_non_sdp;

	pr_smb(PR_MISC, "usb type = %s current_limit = %d\n",
			usb_type_name, current_limit);
	mutex_lock(&chip->current_change_lock);
	if (current_limit != chip->usb_target_current_ma) {
		pr_smb(PR_STATUS, "changed current_limit = %d\n",
				current_limit);
		chip->usb_target_current_ma = current_limit;
		rc = smbchg_set_thermal_limited_usb_current_max(chip,
				current_limit);
		if (rc < 0)
			dev_err(chip->dev,
				"Couldn't set usb current rc = %d\n", rc);
	}
	mutex_unlock(&chip->current_change_lock);

skip_current_for_non_sdp:
	smbchg_vfloat_adjust_check(chip);

	power_supply_changed(&chip->batt_psy);
}

static int smbchg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	chip->otg_retries = 0;
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, true);
	/* sleep to make sure the pulse skip is actually disabled */
	msleep(20);
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, OTG_EN_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		chip->otg_enable_time = ktime_get();
	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, false);
	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_read(chip, &reg, chip->bat_if_base + CMD_CHG_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smbchg_otg_reg_ops = {
	.enable		= smbchg_otg_regulator_enable,
	.disable	= smbchg_otg_regulator_disable,
	.is_enabled	= smbchg_otg_regulator_is_enable,
};

#define USBIN_CHGR_CFG			0xF1
#define ADAPTER_ALLOWANCE_MASK		0x7
#define USBIN_ADAPTER_9V		0x3
#define USBIN_ADAPTER_5V_9V_UNREG	0x5
#define HVDCP_EN_BIT			BIT(3)
static int smbchg_external_otg_regulator_enable(struct regulator_dev *rdev)
{
	bool changed;
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_primary_usb_en(chip, false, REASON_OTG, &changed);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't suspend charger rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);
		return rc;
	}

	/*
	 * To disallow source detect and usbin_uv interrupts, set the adapter
	 * allowance to 9V, so that the audio boost operating in reverse never
	 * gets detected as a valid input
	 */
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, USBIN_ADAPTER_9V);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_disable(struct regulator_dev *rdev)
{
	bool changed;
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_primary_usb_en(chip, true, REASON_OTG, &changed);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend charger rc=%d\n", rc);
		return rc;
	}

	/*
	 * Reenable HVDCP and set the adapter allowance back to the original
	 * value in order to allow normal USBs to be recognized as a valid
	 * input.
	 */
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, chip->original_usbin_allowance);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	return !smbchg_primary_usb_is_en(chip, REASON_OTG);
}

struct regulator_ops smbchg_external_otg_reg_ops = {
	.enable		= smbchg_external_otg_regulator_enable,
	.disable	= smbchg_external_otg_regulator_disable,
	.is_enabled	= smbchg_external_otg_regulator_is_enable,
};

static int smbchg_regulator_init(struct smbchg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-boost-otg");

	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smbchg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
						&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	if (rc)
		return rc;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-external-otg");
	if (!regulator_node) {
		dev_dbg(chip->dev, "external-otg node absent\n");
		return 0;
	}
	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		if (of_get_property(chip->dev->of_node,
					"otg-parent-supply", NULL))
			init_data->supply_regulator = "otg-parent";
		chip->ext_otg_vreg.rdesc.owner = THIS_MODULE;
		chip->ext_otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->ext_otg_vreg.rdesc.ops = &smbchg_external_otg_reg_ops;
		chip->ext_otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->ext_otg_vreg.rdev = regulator_register(
					&chip->ext_otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->ext_otg_vreg.rdev)) {
			rc = PTR_ERR(chip->ext_otg_vreg.rdev);
			chip->ext_otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"external OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static void smbchg_regulator_deinit(struct smbchg_chip *chip)
{
	if (chip->otg_vreg.rdev)
		regulator_unregister(chip->otg_vreg.rdev);
	if (chip->ext_otg_vreg.rdev)
		regulator_unregister(chip->ext_otg_vreg.rdev);
}

static int vf_adjust_low_threshold = 5;
module_param(vf_adjust_low_threshold, int, 0644);

static int vf_adjust_high_threshold = 7;
module_param(vf_adjust_high_threshold, int, 0644);

static int vf_adjust_n_samples = 10;
module_param(vf_adjust_n_samples, int, 0644);

static int vf_adjust_max_delta_mv = 40;
module_param(vf_adjust_max_delta_mv, int, 0644);

static int vf_adjust_trim_steps_per_adjust = 1;
module_param(vf_adjust_trim_steps_per_adjust, int, 0644);

#define CENTER_TRIM_CODE		7
#define MAX_LIN_CODE			14
#define MAX_TRIM_CODE			15
#define SCALE_SHIFT			4
#define VF_TRIM_OFFSET_MASK		SMB_MASK(3, 0)
#define VF_STEP_SIZE_MV			10
#define SCALE_LSB_MV			17
static int smbchg_trim_add_steps(int prev_trim, int delta_steps)
{
	int scale_steps;
	int linear_offset, linear_scale;
	int offset_code = prev_trim & VF_TRIM_OFFSET_MASK;
	int scale_code = (prev_trim & ~VF_TRIM_OFFSET_MASK) >> SCALE_SHIFT;

	if (abs(delta_steps) > 1) {
		pr_smb(PR_STATUS,
			"Cant trim multiple steps delta_steps = %d\n",
			delta_steps);
		return prev_trim;
	}
	if (offset_code <= CENTER_TRIM_CODE)
		linear_offset = offset_code + CENTER_TRIM_CODE;
	else if (offset_code > CENTER_TRIM_CODE)
		linear_offset = MAX_TRIM_CODE - offset_code;

	if (scale_code <= CENTER_TRIM_CODE)
		linear_scale = scale_code + CENTER_TRIM_CODE;
	else if (scale_code > CENTER_TRIM_CODE)
		linear_scale = scale_code - (CENTER_TRIM_CODE + 1);

	/* check if we can accomodate delta steps with just the offset */
	if (linear_offset + delta_steps >= 0
			&& linear_offset + delta_steps <= MAX_LIN_CODE) {
		linear_offset += delta_steps;

		if (linear_offset > CENTER_TRIM_CODE)
			offset_code = linear_offset - CENTER_TRIM_CODE;
		else
			offset_code = MAX_TRIM_CODE - linear_offset;

		return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
	}

	/* changing offset cannot satisfy delta steps, change the scale bits */
	scale_steps = delta_steps > 0 ? 1 : -1;

	if (linear_scale + scale_steps < 0
			|| linear_scale + scale_steps > MAX_LIN_CODE) {
		pr_smb(PR_STATUS,
			"Cant trim scale_steps = %d delta_steps = %d\n",
			scale_steps, delta_steps);
		return prev_trim;
	}

	linear_scale += scale_steps;

	if (linear_scale > CENTER_TRIM_CODE)
		scale_code = linear_scale - CENTER_TRIM_CODE;
	else
		scale_code = linear_scale + (CENTER_TRIM_CODE + 1);
	prev_trim = (prev_trim & VF_TRIM_OFFSET_MASK)
		| scale_code << SCALE_SHIFT;

	/*
	 * now that we have changed scale which is a 17mV jump, change the
	 * offset bits (10mV) too so the effective change is just 7mV
	 */
	delta_steps = -1 * delta_steps;

	linear_offset = clamp(linear_offset + delta_steps, 0, MAX_LIN_CODE);
	if (linear_offset > CENTER_TRIM_CODE)
		offset_code = linear_offset - CENTER_TRIM_CODE;
	else
		offset_code = MAX_TRIM_CODE - linear_offset;

	return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
}

#define TRIM_14		0xFE
#define VF_TRIM_MASK	0xFF
static int smbchg_adjust_vfloat_mv_trim(struct smbchg_chip *chip,
						int delta_mv)
{
	int sign, delta_steps, rc = 0;
	u8 prev_trim, new_trim;
	int i;

	sign = delta_mv > 0 ? 1 : -1;
	delta_steps = (delta_mv + sign * VF_STEP_SIZE_MV / 2)
			/ VF_STEP_SIZE_MV;

	rc = smbchg_read(chip, &prev_trim, chip->misc_base + TRIM_14, 1);
	if (rc) {
		dev_err(chip->dev, "Unable to read trim 14: %d\n", rc);
		return rc;
	}

	for (i = 1; i <= abs(delta_steps)
			&& i <= vf_adjust_trim_steps_per_adjust; i++) {
		new_trim = (u8)smbchg_trim_add_steps(prev_trim,
				delta_steps > 0 ? 1 : -1);
		if (new_trim == prev_trim) {
			pr_smb(PR_STATUS,
				"VFloat trim unchanged from %02x\n", prev_trim);
			/* treat no trim change as an error */
			return -EINVAL;
		}

		rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_14,
				VF_TRIM_MASK, new_trim);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't change vfloat trim rc=%d\n", rc);
		}
		pr_smb(PR_STATUS,
			"VFlt trim %02x to %02x, delta steps: %d\n",
			prev_trim, new_trim, delta_steps);
		prev_trim = new_trim;
	}

	return rc;
}

#define VFLOAT_RESAMPLE_DELAY_MS	10000
static void smbchg_vfloat_adjust_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				vfloat_adjust_work.work);
	int vbat_uv, vbat_mv, ibat_ua, rc, delta_vfloat_mv;
	bool taper, enable;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	taper = (get_prop_charge_type(chip)
		== POWER_SUPPLY_CHARGE_TYPE_TAPER);
	enable = taper && (chip->parallel.current_max_ma == 0);

	if (!enable) {
		pr_smb(PR_MISC,
			"Stopping vfloat adj taper=%d parallel_ma = %d\n",
			taper, chip->parallel.current_max_ma);
		goto stop;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		goto stop;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support voltage rc = %d\n", rc);
		goto stop;
	}
	vbat_mv = vbat_uv / 1000;

	if ((vbat_mv - chip->vfloat_mv) < -1 * vf_adjust_max_delta_mv) {
		pr_smb(PR_STATUS, "Skip vbat out of range: %d\n", vbat_mv);
		goto reschedule;
	}

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ua);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support current_now rc = %d\n", rc);
		goto stop;
	}

	if (ibat_ua / 1000 > -chip->iterm_ma) {
		pr_smb(PR_STATUS, "Skip ibat too high: %d\n", ibat_ua);
		goto reschedule;
	}

	pr_smb(PR_STATUS, "sample number = %d vbat_mv = %d ibat_ua = %d\n",
		chip->n_vbat_samples,
		vbat_mv,
		ibat_ua);

	chip->max_vbat_sample = max(chip->max_vbat_sample, vbat_mv);
	chip->n_vbat_samples += 1;
	if (chip->n_vbat_samples < vf_adjust_n_samples) {
		pr_smb(PR_STATUS, "Skip %d samples; max = %d\n",
			chip->n_vbat_samples, chip->max_vbat_sample);
		goto reschedule;
	}
	/* if max vbat > target vfloat, delta_vfloat_mv could be negative */
	delta_vfloat_mv = chip->vfloat_mv - chip->max_vbat_sample;
	pr_smb(PR_STATUS, "delta_vfloat_mv = %d, samples = %d, mvbat = %d\n",
		delta_vfloat_mv, chip->n_vbat_samples, chip->max_vbat_sample);
	/*
	 * enough valid samples has been collected, adjust trim codes
	 * based on maximum of collected vbat samples if necessary
	 */
	if (delta_vfloat_mv > vf_adjust_high_threshold
			|| delta_vfloat_mv < -1 * vf_adjust_low_threshold) {
		rc = smbchg_adjust_vfloat_mv_trim(chip, delta_vfloat_mv);
		if (rc) {
			pr_smb(PR_STATUS,
				"Stopping vfloat adj after trim adj rc = %d\n",
				 rc);
			goto stop;
		}
		chip->max_vbat_sample = 0;
		chip->n_vbat_samples = 0;
		goto reschedule;
	}

stop:
	chip->max_vbat_sample = 0;
	chip->n_vbat_samples = 0;
	smbchg_relax(chip, PM_REASON_VFLOAT_ADJUST);
	return;

reschedule:
	schedule_delayed_work(&chip->vfloat_adjust_work,
			msecs_to_jiffies(VFLOAT_RESAMPLE_DELAY_MS));
	return;
}

static int smbchg_charging_status_change(struct smbchg_chip *chip)
{
	smbchg_vfloat_adjust_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
	return 0;
}

static void smbchg_hvdcp_det_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				hvdcp_det_work.work);
	int rc;
	u8 reg, hvdcp_sel;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read hvdcp status rc = %d\n", rc);
		return;
	}

	pr_smb(PR_STATUS, "HVDCP_STS = 0x%02x\n", reg);
	/*
	 * If a valid HVDCP is detected, notify it to the usb_psy only
	 * if USB is still present.
	 */
	if (chip->schg_version == QPNP_SCHG_LITE)
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
	else
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;

	if ((reg & hvdcp_sel) && is_usb_present(chip)) {
		pr_smb(PR_MISC, "setting usb psy type = %d\n",
				POWER_SUPPLY_TYPE_USB_HVDCP);
		power_supply_set_supply_type(chip->usb_psy,
				POWER_SUPPLY_TYPE_USB_HVDCP);
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_aicl_deglitch_wa_check(chip);
	}
}

static void handle_usb_removal(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc;

	pr_smb(PR_STATUS, "triggered\n");
	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->force_aicl_rerun && !chip->very_weak_charger) {
		rc = smbchg_hw_aicl_rerun_en(chip, true);
		if (rc)
			pr_err("Error enabling AICL rerun rc= %d\n",
				rc);
	}
	/* Clear the OV detected status set before */
	if (chip->usb_ov_det)
		chip->usb_ov_det = false;
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy type = %d\n",
				POWER_SUPPLY_TYPE_UNKNOWN);
		power_supply_set_supply_type(chip->usb_psy,
				POWER_SUPPLY_TYPE_UNKNOWN);
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		pr_smb(PR_MISC, "setting usb psy allow detection 0\n");
		power_supply_set_allow_detection(chip->usb_psy, 0);
		schedule_work(&chip->usb_set_online_work);
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNKNOWN);
		if (rc)
			pr_smb(PR_STATUS,
				"usb psy does not allow updating prop %d rc = %d\n",
				POWER_SUPPLY_HEALTH_UNKNOWN, rc);
	}
	if (parallel_psy && chip->parallel_charger_detected)
		power_supply_set_present(parallel_psy, false);
	if (chip->parallel.avail && chip->aicl_done_irq
			&& chip->enable_aicl_wake) {
		disable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = false;
	}
	chip->parallel.enabled_once = false;
	chip->vbat_above_headroom = false;
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			ICL_OVERRIDE_BIT, 0);
	if (rc < 0)
		pr_err("Couldn't set override rc = %d\n", rc);
}

/* FUJITSU LIMITED:2015-07-22 H1520398 del start */
#if 0
static bool is_src_detect_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_SRC_DET_BIT;
}
#endif
/* FUJITSU LIMITED:2015-07-22 H1520398 del end */

#define HVDCP_NOTIFY_MS		2500
/* FUJITSU LIMITED:2016-01-08 CHARGER del start */
#if 0
#define DEFAULT_WALL_CHG_MA	1800
#define DEFAULT_SDP_MA		100
#define DEFAULT_CDP_MA		1500
#endif
/* FUJITSU LIMITED:2016-01-08 CHARGER del end */
static void handle_usb_insertion(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	enum power_supply_type usb_supply_type;
	int rc;
	char *usb_type_name = "null";

	pr_smb(PR_STATUS, "triggered\n");
	/* usb inserted */
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	pr_smb(PR_STATUS,
		"inserted type = %d (%s)", usb_supply_type, usb_type_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy type = %d\n",
				usb_supply_type);
		power_supply_set_supply_type(chip->usb_psy, usb_supply_type);
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		/* Notify the USB psy if OV condition is not present */
		if (!chip->usb_ov_det) {
			/*
			 * Note that this could still be a very weak charger
			 * if the handle_usb_insertion was triggered from
			 * the falling edge of an USBIN_OV interrupt
			 */
			rc = power_supply_set_health_state(chip->usb_psy,
					chip->very_weak_charger
					? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
					: POWER_SUPPLY_HEALTH_GOOD);
			if (rc)
				pr_smb(PR_STATUS,
					"usb psy does not allow updating prop %d rc = %d\n",
					POWER_SUPPLY_HEALTH_GOOD, rc);
		}
		schedule_work(&chip->usb_set_online_work);
	}

	if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)
		schedule_delayed_work(&chip->hvdcp_det_work,
					msecs_to_jiffies(HVDCP_NOTIFY_MS));
/* FUJITSU LIMITED:2016-01-08 CHARGER del start */
#if 0
	mutex_lock(&chip->current_change_lock);
	if (usb_supply_type == POWER_SUPPLY_TYPE_USB)
		chip->usb_target_current_ma = DEFAULT_SDP_MA;
	else if (usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP)
		chip->usb_target_current_ma = DEFAULT_CDP_MA;
	else
		chip->usb_target_current_ma = DEFAULT_WALL_CHG_MA;

	pr_smb(PR_STATUS, "%s detected setting mA = %d\n",
		usb_type_name, chip->usb_target_current_ma);
	rc = smbchg_set_thermal_limited_usb_current_max(chip,
				chip->usb_target_current_ma);
	mutex_unlock(&chip->current_change_lock);
#endif
/* FUJITSU LIMITED:2016-01-08 CHARGER del end */
	if (parallel_psy) {
		rc = power_supply_set_present(parallel_psy, true);
		chip->parallel_charger_detected = rc ? false : true;
		if (rc)
			pr_debug("parallel-charger absent rc=%d\n", rc);
	}

	if (chip->parallel.avail && chip->aicl_done_irq
			&& !chip->enable_aicl_wake) {
		rc = enable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = true;
	}
}

void update_usb_status(struct smbchg_chip *chip, bool usb_present, bool force)
{
	mutex_lock(&chip->usb_status_lock);
	if (force) {
		chip->usb_present = usb_present;
		chip->usb_present ? handle_usb_insertion(chip)
			: handle_usb_removal(chip);
		goto unlock;
	}
	if (!chip->usb_present && usb_present) {
		chip->usb_present = usb_present;
		handle_usb_insertion(chip);
	} else if (chip->usb_present && !usb_present) {
		chip->usb_present = usb_present;
		handle_usb_removal(chip);
	}

	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
unlock:
	mutex_unlock(&chip->usb_status_lock);
}

static int otg_oc_reset(struct smbchg_chip *chip)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, 0);
	if (rc)
		pr_err("Failed to disable OTG rc=%d\n", rc);

	msleep(20);

	/*
	 * There is a possibility that an USBID interrupt might have
	 * occurred notifying USB power supply to disable OTG. We
	 * should not enable OTG in such cases.
	 */
	if (!is_otg_present(chip)) {
		pr_smb(PR_STATUS,
			"OTG is not present, not enabling OTG_EN_BIT\n");
		goto out;
	}

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, OTG_EN_BIT);
	if (rc)
		pr_err("Failed to re-enable OTG rc=%d\n", rc);

out:
	return rc;
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#define AICL_IRQ_LIMIT_SECONDS	60
#define AICL_IRQ_LIMIT_COUNT	25
static void increment_aicl_count(struct smbchg_chip *chip)
{
	bool bad_charger = false;
	int rc;
	u8 reg;
	long elapsed_seconds;
	unsigned long now_seconds;

	pr_smb(PR_INTERRUPT, "aicl count c:%d dgltch:%d first:%ld\n",
			chip->aicl_irq_count, chip->aicl_deglitch_short,
			chip->first_aicl_seconds);

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (!rc)
		chip->aicl_complete = reg & AICL_STS_BIT;
	else
		chip->aicl_complete = false;

	if (chip->aicl_deglitch_short || chip->force_aicl_rerun) {
		if (!chip->aicl_irq_count)
			get_current_time(&chip->first_aicl_seconds);
		get_current_time(&now_seconds);
		elapsed_seconds = now_seconds
				- chip->first_aicl_seconds;

		if (elapsed_seconds > AICL_IRQ_LIMIT_SECONDS) {
			pr_smb(PR_INTERRUPT,
				"resetting: elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
			chip->aicl_irq_count = 1;
			get_current_time(&chip->first_aicl_seconds);
			return;
		}
		chip->aicl_irq_count++;

		if (chip->aicl_irq_count > AICL_IRQ_LIMIT_COUNT) {
			pr_smb(PR_INTERRUPT, "elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
			pr_smb(PR_INTERRUPT, "Disable AICL rerun\n");
			/*
			 * Disable AICL rerun since many interrupts were
			 * triggered in a short time
			 */
			chip->very_weak_charger = true;
			rc = smbchg_hw_aicl_rerun_en(chip, false);
			if (rc)
				pr_err("could not enable aicl reruns: %d", rc);
			bad_charger = true;
			chip->aicl_irq_count = 0;
		} else if ((get_prop_charge_type(chip) ==
				POWER_SUPPLY_CHARGE_TYPE_FAST) &&
					(reg & AICL_SUSP_BIT)) {
			/*
			 * If the AICL_SUSP_BIT is on, then AICL reruns have
			 * already been disabled. Set the very weak charger
			 * flag so that the driver reports a bad charger
			 * and does not reenable AICL reruns.
			 */
			chip->very_weak_charger = true;
			bad_charger = true;
		}
		if (bad_charger) {
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
			if (rc)
				pr_err("Couldn't set health on usb psy rc:%d\n",
					rc);
			schedule_work(&chip->usb_set_online_work);
		}
	}
}

static enum power_supply_property smbchg_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_FLASH_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_FLASH_ACTIVE,
};

static int smbchg_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	bool unused;
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		smbchg_battchg_en(chip, val->intval,
				REASON_BATTCHG_USER, &unused);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smbchg_usb_en(chip, val->intval, REASON_USER);
		smbchg_dc_en(chip, val->intval, REASON_USER);
		chip->chg_enabled = val->intval;
		schedule_work(&chip->usb_set_online_work);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smbchg_system_temp_level_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smbchg_set_fastchg_current_user(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smbchg_float_voltage_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		rc = smbchg_safety_timer_enable(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		rc = smbchg_otg_pulse_skip_disable(chip,
				REASON_FLASH_ENABLED, val->intval);
		break;
	case POWER_SUPPLY_PROP_FORCE_TLIM:
		rc = smbchg_force_tlim_en(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int smbchg_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = smcghg_is_battchg_en(chip, REASON_BATTCHG_USER);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->chg_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = smbchg_float_voltage_get(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = smbchg_calc_max_flash_current(chip);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = smbchg_get_aicl_level_ma(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = (int)chip->aicl_complete;
		break;
	/* properties from fg */
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_batt_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = get_prop_batt_voltage_max_design(chip);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		val->intval = chip->safety_timer_en;
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->otg_pulse_skip_dis;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *smbchg_dc_supplicants[] = {
	"bms",
};

static enum power_supply_property smbchg_dc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int smbchg_dc_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = smbchg_dc_en(chip, val->intval, REASON_POWER_SUPPLY);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smbchg_set_dc_current_max(chip, val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_dc_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = is_dc_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->dc_suspended == 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* return if dc is charging the battery */
		val->intval = (smbchg_get_pwr_path(chip) == PWR_PATH_DC)
				&& (get_prop_batt_status(chip)
					== POWER_SUPPLY_STATUS_CHARGING);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->dc_max_current_ma * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smbchg_dc_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

#define HOT_BAT_HARD_BIT	BIT(0)
#define HOT_BAT_SOFT_BIT	BIT(1)
#define COLD_BAT_HARD_BIT	BIT(2)
#define COLD_BAT_SOFT_BIT	BIT(3)
#define BAT_OV_BIT		BIT(4)
#define BAT_LOW_BIT		BIT(5)
#define BAT_MISSING_BIT		BIT(6)
#define BAT_TERM_MISSING_BIT	BIT(7)
static irqreturn_t batt_hot_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_hot = !!(reg & HOT_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cold_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cold = !!(reg & COLD_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_warm_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_warm = !!(reg & HOT_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cool_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cool = !!(reg & COLD_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_pres_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_present = !(reg & BAT_MISSING_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t vbat_low_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("vbat low\n");
	return IRQ_HANDLED;
}

static irqreturn_t chg_error_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "chg-error triggered\n");
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t fastchg_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
/* FUJITSU LIMITED:2016-01-05 CHARGER add start */
	u8 reg = 0;
/* FUJITSU LIMITED:2016-01-05 CHARGER add end */

/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
	pr_smb(PR_INTERRUPT, "p2f triggered\n");
#else
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_hot_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("chg hot\n");
	smbchg_wipower_check(_chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_term_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg = 0;
	bool terminated = false;

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc) {
		dev_err(chip->dev, "Error reading RT_STS rc= %d\n", rc);
	} else {
		terminated = !!(reg & BAT_TCC_REACHED_BIT);
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	}
	/*
	 * If charging has not actually terminated, then this means that
	 * either this is a manual call to chg_term_handler during
	 * determine_initial_status(), or the charger has instantly restarted
	 * charging.
	 *
	 * In either case, do not do the usual status updates here. If there
	 * is something that needs to be updated, the recharge handler will
	 * handle it.
	 */
/* FUJITSU LIMITED:2016-01-05 CHARGER del start */
#if 0
	if (terminated) {
		smbchg_parallel_usb_check_ok(chip);
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_charging_status_change(chip);
		set_property_on_fg(chip, POWER_SUPPLY_PROP_CHARGE_DONE, 1);
	}
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER del end */
	return IRQ_HANDLED;
}

static irqreturn_t taper_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	taper_irq_en(chip, false);
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_taper(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t recharge_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t safety_timeout_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_warn_ratelimited("safety timeout rt_stat = 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

/**
 * power_ok_handler() - called when the switcher turns on or turns off
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating switcher turning on or off
 */
static irqreturn_t power_ok_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	return IRQ_HANDLED;
}

/* FCNT LIMITED:2016-02-19 H16100406 add start */
static void dcin_uv_handler_core(void *_chip, int _vcdet)
{
	struct smbchg_chip *chip = _chip;
	bool dc_present;
	bool vcdet;

/* FCNT LIMITED:2016-02-03 H16100711 add start */
	if (!gpio_is_valid(fj_platform_data.vc_det_gpio)) {
		return;
	}
/* FCNT LIMITED:2016-02-03 H16100711 add end */

	pr_smb(PR_INTERRUPT, "triggered\n");
	wake_lock(&fj_platform_data.fj_vcdet_lock);

	/* vcdet chattering */
	if (_vcdet < 0)
		vcdet = fj_status_chattering(chip, FJ_STATUS_DC);
	else
		vcdet = (bool)_vcdet;

	dc_present = is_dc_present(chip);

	pr_smb(PR_INTERRUPT, "chip->dc_present = %d dc_present = %d rt_sts=%d\n",
			chip->dc_present, vcdet, dc_present);
	if (chip->dc_present != vcdet) {
		/* dc changed */
		chip->dc_present = vcdet;
		if (vcdet) {
			FJ_QPNP_RECLOG("SMBCHG: cradle detect\n");
			fj_chg_holder_vbus_draw(FJ_CHG_ENABLE);
		} else {
			FJ_QPNP_RECLOG("SMBCHG: cradle not detect\n");
			fj_chg_holder_vbus_draw(FJ_CHG_DISABLE);
		}
		smbchg_charging_status_change(chip);
		smbchg_aicl_deglitch_wa_check(chip);
		chip->vbat_above_headroom = false;
	}
	if (!vcdet) {
		smbchg_wipower_check(chip);
	}
	wake_unlock(&fj_platform_data.fj_vcdet_lock);

	return;
}
/* FCNT LIMITED:2016-02-19 H16100406 add end */

/**
 * dcin_uv_handler() - called when the dc voltage crosses the uv threshold
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating whether dc voltage is uv
 */
#define DCIN_UNSUSPEND_DELAY_MS		1000
static irqreturn_t dcin_uv_handler(int irq, void *_chip)
{
/* FUJITSU LIMITED:2015-01-07 H1510087 mod start */
#if 0
	struct smbchg_chip *chip = _chip;
	bool dc_present = is_dc_present(chip);
	int rc;

	pr_smb(PR_STATUS, "chip->dc_present = %d dc_present = %d\n",
			chip->dc_present, dc_present);

	if (chip->dc_present != dc_present) {
		/* dc changed */
		chip->dc_present = dc_present;
		if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
			power_supply_changed(&chip->dc_psy);
		smbchg_charging_status_change(chip);
		smbchg_aicl_deglitch_wa_check(chip);
		if (chip->force_aicl_rerun && !dc_present) {
			rc = smbchg_hw_aicl_rerun_en(chip, true);
			if (rc)
				pr_err("Error enabling AICL rerun rc= %d\n",
					rc);
		}
		chip->vbat_above_headroom = false;
	}

	smbchg_wipower_check(chip);
#else
/* FCNT LIMITED:2016-02-19 H16100406 mod start */
	dcin_uv_handler_core(_chip, FJ_CHATT);
/* FCNT LIMITED:2016-02-19 H16100406 mod end */
#endif
/* FUJITSU LIMITED:2015-01-07 H1510087 mod end */
	return IRQ_HANDLED;
}

/* FUJITSU LIMITED:2015-03-16 H1510200 del start */
#if 0
/**
 * usbin_ov_handler() - this is called when an overvoltage condition occurs
 * @chip: pointer to smbchg_chip chip
 */
static irqreturn_t usbin_ov_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg;
	bool usb_present;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		goto out;
	}

	/* OV condition is detected. Notify it to USB psy */
	if (reg & USBIN_OV_BIT) {
		chip->usb_ov_det = true;
		if (chip->usb_psy) {
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_OVERVOLTAGE);
			if (rc)
				pr_smb(PR_STATUS,
					"usb psy does not allow updating prop %d rc = %d\n",
					POWER_SUPPLY_HEALTH_OVERVOLTAGE, rc);
		}
	} else {
		chip->usb_ov_det = false;
		/* If USB is present, then handle the USB insertion */
		usb_present = is_usb_present(chip);
		if (usb_present)
			update_usb_status(chip, usb_present, false);
	}
out:
	return IRQ_HANDLED;
}
#endif
/* FUJITSU LIMITED:2015-03-16 H1510200 del end */

/* FUJITSU LIMITED:2015-01-07 H1510087 del start */
#if 0
/**
 * usbin_uv_handler() - this is called when USB charger is removed
 * @chip: pointer to smbchg_chip chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
#define ICL_MODE_MASK		SMB_MASK(5, 4)
#define ICL_MODE_HIGH_CURRENT	0
static irqreturn_t usbin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int aicl_level = smbchg_get_aicl_level_ma(chip);
	int rc;
	bool unused;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc) {
		pr_err("could not read rt sts: %d", rc);
		goto out;
	}

	pr_smb(PR_STATUS,
		"chip->usb_present = %d rt_sts = 0x%02x aicl = %d\n",
		chip->usb_present, reg, aicl_level);

	/*
	 * set usb_psy's allow_detection if this is a new insertion, i.e. it is
	 * not already src_detected and usbin_uv is seen falling
	 */
	if (!(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "setting usb psy allow detection 1\n");
		power_supply_set_allow_detection(chip->usb_psy, 1);
	}

	if ((reg & USBIN_UV_BIT) && (reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_STATUS, "Very weak charger detected\n");
		chip->very_weak_charger = true;
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + ICL_STS_2_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Could not read usb icl sts 2: %d\n",
					rc);
			goto out;
		}
		if ((reg & ICL_MODE_MASK) != ICL_MODE_HIGH_CURRENT) {
			/*
			 * If AICL is not even enabled, this is either an
			 * SDP or a grossly out of spec charger. Do not
			 * draw any current from it.
			 */
			rc = smbchg_primary_usb_en(chip, false,
					REASON_WEAK_CHARGER, &unused);
			if (rc)
				pr_err("could not disable charger: %d", rc);
		} else if ((chip->aicl_deglitch_short || chip->force_aicl_rerun)
			&& aicl_level == usb_current_table[0]) {
			rc = smbchg_hw_aicl_rerun_en(chip, false);
			if (rc)
				pr_err("could not enable aicl reruns: %d", rc);
		}
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		if (rc)
			pr_err("Couldn't set health on usb psy rc:%d\n", rc);
		schedule_work(&chip->usb_set_online_work);
	}

	smbchg_wipower_check(chip);
out:
	return IRQ_HANDLED;
}
#endif
/* FUJITSU LIMITED:2015-01-07 H1510087 del end */
/* FCNT LIMITED:2016-02-19 H16100406 add start */
static void src_detect_handler_core(void *_chip, int _vbdet)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present;
	bool vbdet;

	pr_smb(PR_INTERRUPT, "triggered\n");
	wake_lock(&fj_platform_data.fj_vbdet_lock);

	/* vbdet chattering */
	if (_vbdet < 0)
		vbdet = fj_status_chattering(chip, FJ_STATUS_USB);
	else
		vbdet = (bool)_vbdet;

	usb_present = is_usb_present(chip);

	pr_smb(PR_INTERRUPT, "chip->usb_present = %d usb_present = %d rt_sts=%d\n",
			chip->usb_present, vbdet, usb_present);

	/* extusb detect check */
	if (fj_ext_usb_check(chip, vbdet)) {
		pr_info("src_detect_handler fj_ext_usb_check exit\n");
		goto ext_usb;
	}

	if (!chip->usb_present && vbdet) {
		/* USB inserted */
		chip->usb_present = vbdet;
		handle_usb_insertion(chip);
	}
	if (chip->usb_present && !vbdet) {
		/* USB removed */
		chip->usb_present = vbdet;
		chip->very_weak_charger = false; /* FUJITSU LIMITED:2015-07-10 H1520392 add */
		handle_usb_removal(chip);
		chip->aicl_irq_count = 0;
	}
ext_usb:
	if (!vbdet)
		smbchg_wipower_check(chip);

	wake_unlock(&fj_platform_data.fj_vbdet_lock);

	return;
}
/* FCNT LIMITED:2016-02-19 H16100406 add end */

/**
 * src_detect_handler() - this is called on rising edge when USB charger type
 *			is detected and on falling edge when USB voltage falls
 *			below the coarse detect voltage(1V), use it for
 *			handling USB charger insertion and removal.
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
static irqreturn_t src_detect_handler(int irq, void *_chip)
{
/* FUJITSU LIMITED:2015-01-07 H1510087 mod start */
#if 0
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip), unused;
	bool src_detect = is_src_detect_high(chip);
	int rc;

	pr_smb(PR_STATUS,
		"chip->usb_present = %d usb_present = %d src_detect = %d",
		chip->usb_present, usb_present, src_detect);

	/*
	 * When VBAT is above the AICL threshold (4.25V) - 180mV (4.07V),
	 * an input collapse due to AICL will actually cause an USBIN_UV
	 * interrupt to fire as well.
	 *
	 * Handle USB insertions and removals in the source detect handler
	 * instead of the USBIN_UV handler since the latter is untrustworthy
	 * when the battery voltage is high.
	 */
	chip->very_weak_charger = false;
	rc = smbchg_primary_usb_en(chip, true, REASON_WEAK_CHARGER, &unused);
	if (rc < 0)
		pr_err("could not enable charger: %d\n", rc);

	if (src_detect) {
		update_usb_status(chip, usb_present, 0);
	} else {
		update_usb_status(chip, 0, false);
		chip->aicl_irq_count = 0;
	}

#else
/* FCNT LIMITED:2016-02-19 H16100406 mod start */
	src_detect_handler_core(_chip, FJ_CHATT);
/* FCNT LIMITED:2016-02-19 H16100406 mod end */
#endif
/* FUJITSU LIMITED:2015-01-07 H1510087 mod end */
	return IRQ_HANDLED;
}

/**
 * otg_oc_handler() - called when the usb otg goes over current
 */
#define NUM_OTG_RETRIES			5
#define OTG_OC_RETRY_DELAY_US		50000
static irqreturn_t otg_oc_handler(int irq, void *_chip)
{
	int rc;
	struct smbchg_chip *chip = _chip;
	s64 elapsed_us = ktime_us_delta(ktime_get(), chip->otg_enable_time);

	pr_smb(PR_INTERRUPT, "triggered\n");

	if (chip->schg_version == QPNP_SCHG_LITE) {
		pr_smb(PR_STATUS, "Clear OTG-OC by enable/disable OTG\n");
		rc = otg_oc_reset(chip);
		if (rc)
			pr_err("Failed to reset OTG OC state rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (elapsed_us > OTG_OC_RETRY_DELAY_US)
		chip->otg_retries = 0;

	/*
	 * Due to a HW bug in the PMI8994 charger, the current inrush that
	 * occurs when connecting certain OTG devices can cause the OTG
	 * overcurrent protection to trip.
	 *
	 * The work around is to try reenabling the OTG when getting an
	 * overcurrent interrupt once.
	 */
	if (chip->otg_retries < NUM_OTG_RETRIES) {
		chip->otg_retries += 1;
		pr_smb(PR_STATUS,
			"Retrying OTG enable. Try #%d, elapsed_us %lld\n",
						chip->otg_retries, elapsed_us);
		rc = otg_oc_reset(chip);
		if (rc)
			pr_err("Failed to reset OTG OC state rc=%d\n", rc);
		chip->otg_enable_time = ktime_get();
	}
	return IRQ_HANDLED;
}

/**
 * otg_fail_handler() - called when the usb otg fails
 * (when vbat < OTG UVLO threshold)
 */
static irqreturn_t otg_fail_handler(int irq, void *_chip)
{
	pr_smb(PR_INTERRUPT, "triggered\n");
	return IRQ_HANDLED;
}

/**
 * aicl_done_handler() - called when the usb AICL algorithm is finished
 *			and a current is set.
 */
static irqreturn_t aicl_done_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	int aicl_level = smbchg_get_aicl_level_ma(chip);

	pr_smb(PR_INTERRUPT, "triggered, aicl: %d\n", aicl_level);

	increment_aicl_count(chip);

	if (usb_present)
		smbchg_parallel_usb_check_ok(chip);

	if (chip->aicl_complete)
		power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

/**
 * usbid_change_handler() - called when the usb RID changes.
 * This is used mostly for detecting OTG
 */
static irqreturn_t usbid_change_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool otg_present;

/* FUJITSU LIMITED:2015-01-07 H1510087 add start */
	wake_lock(&fj_platform_data.fj_usbid_lock);
/* FUJITSU LIMITED:2015-01-07 H1510087 add end */
	pr_smb(PR_INTERRUPT, "triggered\n");

	otg_present = is_otg_present(chip);

/* FUJITSU LIMITED:2015-01-07 H1510087 add start */
	/* extusb detect check */
	if (fj_ext_id_check(chip)) {
		goto ext_id;
	}
	pr_smb(PR_INTERRUPT, "otg_present = %d\n", otg_present);
/* FUJITSU LIMITED:2015-01-07 H1510087 add end */
	if (chip->usb_psy)
		power_supply_set_usb_otg(chip->usb_psy, otg_present ? 1 : 0);
	if (otg_present)
		pr_smb(PR_STATUS, "OTG detected\n");

/* FUJITSU LIMITED:2015-01-07 H1510087 mod start */
ext_id:
	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
	wake_unlock(&fj_platform_data.fj_usbid_lock);
/* FUJITSU LIMITED:2015-01-07 H1510087 mod end */
	return IRQ_HANDLED;
}

static int determine_initial_status(struct smbchg_chip *chip)
{
	struct status_info status;	/* FCNT LIMITED:2016-02-19 H16100406 add */
	/*
	 * It is okay to read the interrupt status here since
	 * interrupts aren't requested. reading interrupt status
	 * clears the interrupt so be careful to read interrupt
	 * status only in interrupt handling code
	 */

	batt_pres_handler(0, chip);
	batt_hot_handler(0, chip);
	batt_warm_handler(0, chip);
	batt_cool_handler(0, chip);
	batt_cold_handler(0, chip);
	chg_term_handler(0, chip);
/* FCNT LIMITED:2016-02-19 H16100406 mod start */
#if 0
	usbid_change_handler(0, chip);
	src_detect_handler(0, chip);
#endif
/* FUJITSU LIMITED:2015-01-07 H1510087 mod start */
/* FUJITSU LIMITED:2016-01-12 H16100714 mod start */
	if (charging_mode)
		usb_id_info = FJ_DEVICE_CHATTERING;
	
	fj_status_check(chip, FJ_CHECK_INIT, &status);
	usbid_change_handler(0, chip);
	src_detect_handler_core(chip, !status.vb_det);
	if (charging_mode) {
		if (fj_platform_data.fj_id_device != FJ_NO_DEVICE) {
			usb_id_info = FJ_DEVICE_CHATTER_END;
		}
	}
/* FUJITSU LIMITED:2016-01-12 H16100714 mod end */
	dcin_uv_handler_core(chip, !status.vc_det);
	dcin_ov_handler_core(chip, status.ovp_state_dc);
	usbin_ov_handler_core(chip, status.ovp_state_usb);
/* FCNT LIMITED:2016-02-19 H16100406 mod end */
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	fj_usb_hot_handler_core(chip, status.usb_hot);
	fj_adp_safe_handler_core(chip, status.adp_safe);
/* FCNT LIMITED:2016-02-03 H16100712 add end */
#if 0
	chip->usb_present = is_usb_present(chip);
	chip->dc_present = is_dc_present(chip);

	if (chip->usb_present)
		handle_usb_insertion(chip);
	else
		handle_usb_removal(chip);
#endif
/* FUJITSU LIMITED:2015-01-07 H1510087 mod end */
	return 0;
}

static int prechg_time[] = {
	24,
	48,
	96,
	192,
};
static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

enum bpd_type {
	BPD_TYPE_BAT_NONE,
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
	BPD_TYPE_DEFAULT,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_NONE]		= "bpd_none",
	[BPD_TYPE_BAT_ID]		= "bpd_id",
	[BPD_TYPE_BAT_THM]		= "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID]	= "bpd_thm_id",
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

#define REVISION1_REG			0x0
#define DIG_MINOR			0
#define DIG_MAJOR			1
#define ANA_MINOR			2
#define ANA_MAJOR			3
#define CHGR_CFG1			0xFB
#define RECHG_THRESHOLD_SRC_BIT		BIT(1)
#define TERM_I_SRC_BIT			BIT(2)
#define TERM_SRC_FG			BIT(2)
#define CHGR_CFG2			0xFC
#define CHG_INHIB_CFG_REG		0xF7
#define CHG_INHIBIT_50MV_VAL		0x00
#define CHG_INHIBIT_100MV_VAL		0x01
#define CHG_INHIBIT_200MV_VAL		0x02
#define CHG_INHIBIT_300MV_VAL		0x03
#define CHG_INHIBIT_MASK		0x03
#define USE_REGISTER_FOR_CURRENT	BIT(2)
#define CHG_EN_SRC_BIT			BIT(7)
#define CHG_EN_COMMAND_BIT		BIT(6)
#define P2F_CHG_TRAN			BIT(5)
#define I_TERM_BIT			BIT(3)
#define AUTO_RECHG_BIT			BIT(2)
#define CHARGER_INHIBIT_BIT		BIT(0)
#define CFG_TCC_REG			0xF9
#define CHG_ITERM_50MA			0x1
#define CHG_ITERM_100MA			0x2
#define CHG_ITERM_150MA			0x3
#define CHG_ITERM_200MA			0x4
#define CHG_ITERM_250MA			0x5
#define CHG_ITERM_300MA			0x0
#define CHG_ITERM_500MA			0x6
#define CHG_ITERM_600MA			0x7
#define CHG_ITERM_MASK			SMB_MASK(2, 0)
#define USB51_COMMAND_POL		BIT(2)
#define USB51AC_CTRL			BIT(1)
#define TR_8OR32B			0xFE
#define BUCK_8_16_FREQ_BIT		BIT(0)
#define BM_CFG				0xF3
#define BATT_MISSING_ALGO_BIT		BIT(2)
#define BMD_PIN_SRC_MASK		SMB_MASK(1, 0)
#define PIN_SRC_SHIFT			0
#define CHGR_CFG			0xFF
#define RCHG_LVL_BIT			BIT(0)
#define CFG_AFVC			0xF6
#define VFLOAT_COMP_ENABLE_MASK		SMB_MASK(2, 0)
#define TR_RID_REG			0xFA
#define FG_INPUT_FET_DELAY_BIT		BIT(3)
#define TRIM_OPTIONS_7_0		0xF6
#define INPUT_MISSING_POLLER_EN_BIT	BIT(3)
#define AICL_WL_SEL_CFG			0xF5
#define AICL_WL_SEL_MASK		SMB_MASK(1, 0)
#define AICL_WL_SEL_45S		0
#define CHGR_CCMP_CFG			0xFA
#define JEITA_TEMP_HARD_LIMIT_BIT	BIT(5)
#define HVDCP_ADAPTER_SEL_MASK		SMB_MASK(5, 4)
#define HVDCP_ADAPTER_SEL_9V_BIT	BIT(4)
#define HVDCP_AUTH_ALG_EN_BIT		BIT(6)
#define CMD_APSD			0x41
#define APSD_RERUN_BIT			BIT(0)
static int smbchg_hw_init(struct smbchg_chip *chip)
{
	int rc, i;
	u8 reg, mask;

	rc = smbchg_read(chip, chip->revision,
			chip->misc_base + REVISION1_REG, 4);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read revision rc=%d\n",
				rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Charger Revision DIG: %d.%d; ANA: %d.%d\n",
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);

	rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + AICL_WL_SEL_CFG,
			AICL_WL_SEL_MASK, AICL_WL_SEL_45S);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set AICL rerun timer rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + TR_RID_REG,
			FG_INPUT_FET_DELAY_BIT, FG_INPUT_FET_DELAY_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable fg input fet delay rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable input missing poller rc=%d\n",
				rc);
		return rc;
	}

	/*
	 * Do not force using current from the register i.e. use auto
	 * power source detect (APSD) mA ratings for the initial current values.
	 *
	 * If this is set, AICL will not rerun at 9V for HVDCPs
	 */
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USE_REGISTER_FOR_CURRENT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

	if (chip->enable_hvdcp_9v && (chip->wa_flags & SMBCHG_HVDCP_9V_EN_WA)) {
		/* enable the 9V HVDCP configuration */
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + TR_RID_REG,
			HVDCP_AUTH_ALG_EN_BIT, HVDCP_AUTH_ALG_EN_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't enable hvdcp_alg rc=%d\n",
					rc);
			return rc;
		}

		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_ADAPTER_SEL_9V_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't set hvdcp config in chgpath_chg rc=%d\n",
						rc);
			return rc;
		}
		if (is_usb_present(chip)) {
			rc = smbchg_masked_write(chip,
				chip->usb_chgpth_base + CMD_APSD,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
			if (rc)
				pr_err("Unable to re-run APSD rc=%d\n", rc);
		}
	}
	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast, enable auto recharge by default.
	 * enable current termination and charge inhibition based on
	 * the device tree configuration.
	 */
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
			CHG_EN_SRC_BIT | CHG_EN_COMMAND_BIT | P2F_CHG_TRAN
			| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
			CHG_EN_COMMAND_BIT
			| (chip->chg_inhibit_en ? CHARGER_INHIBIT_BIT : 0)
			| (chip->iterm_disabled ? I_TERM_BIT : 0));
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}
	chip->battchg_disabled = 0;

	/*
	 * Based on the configuration, use the analog sensors or the fuelgauge
	 * adc for recharge threshold source.
	 */

	if (chip->chg_inhibit_source_fg)
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT,
			TERM_SRC_FG | RECHG_THRESHOLD_SRC_BIT);
	else
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * control USB suspend via command bits and set correct 100/500mA
	 * polarity on the usb current
	 */
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
		USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
		return rc;
	}

	check_battery_type(chip);

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smbchg_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set vfloat to %d\n", chip->vfloat_mv);
	}

	/* set the fast charge current compensation */
	if (chip->fastchg_current_comp != -EINVAL) {
		rc = smbchg_fastchg_current_comp_set(chip,
			chip->fastchg_current_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set fastchg current comp to %d\n",
			chip->fastchg_current_comp);
	}

	/* set the float voltage compensation */
	if (chip->float_voltage_comp != -EINVAL) {
		rc = smbchg_float_voltage_comp_set(chip,
			chip->float_voltage_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set float voltage comp to %d\n",
			chip->float_voltage_comp);
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			if (chip->iterm_ma <= 50)
				reg = CHG_ITERM_50MA;
			else if (chip->iterm_ma <= 100)
				reg = CHG_ITERM_100MA;
			else if (chip->iterm_ma <= 150)
				reg = CHG_ITERM_150MA;
			else if (chip->iterm_ma <= 200)
				reg = CHG_ITERM_200MA;
			else if (chip->iterm_ma <= 250)
				reg = CHG_ITERM_250MA;
			else if (chip->iterm_ma <= 300)
				reg = CHG_ITERM_300MA;
			else if (chip->iterm_ma <= 500)
				reg = CHG_ITERM_500MA;
			else
				reg = CHG_ITERM_600MA;

			rc = smbchg_sec_masked_write(chip,
					chip->chgr_base + CFG_TCC_REG,
					CHG_ITERM_MASK, reg);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
			pr_smb(PR_STATUS, "set tcc (%d) to 0x%02x\n",
					chip->iterm_ma, reg);
		}
	}

	/* set the safety time voltage */
	if (chip->safety_time != -EINVAL) {
		reg = (chip->safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
			(chip->prechg_safety_time > 0
			? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

		for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
			if (chip->safety_time <= chg_time[i]) {
				reg |= i << SAFETY_TIME_MINUTES_SHIFT;
				break;
			}
		}
		for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
			if (chip->prechg_safety_time <= prechg_time[i]) {
				reg |= i;
				break;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + SFT_CFG,
				SFT_EN_MASK | SFT_TO_MASK |
				(chip->prechg_safety_time > 0
				? PRECHG_SFT_TO_MASK : 0), reg);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set safety timer rc = %d\n",
				rc);
			return rc;
		}
		chip->safety_timer_en = true;
	} else {
		rc = smbchg_read(chip, &reg, chip->chgr_base + SFT_CFG, 1);
		if (rc < 0)
			dev_err(chip->dev, "Unable to read SFT_CFG rc = %d\n",
				rc);
		else if (!(reg & SFT_EN_MASK))
			chip->safety_timer_en = true;
	}

	/* configure jeita temperature hard limit */
	if (chip->jeita_temp_hard_limit >= 0) {
		rc = smbchg_sec_masked_write(chip,
			chip->chgr_base + CHGR_CCMP_CFG,
			JEITA_TEMP_HARD_LIMIT_BIT,
			chip->jeita_temp_hard_limit
			? 0 : JEITA_TEMP_HARD_LIMIT_BIT);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set jeita temp hard limit rc = %d\n",
				rc);
			return rc;
		}
	}

	/* make the buck switch faster to prevent some vbus oscillation */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + TR_8OR32B,
			BUCK_8_16_FREQ_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set buck frequency rc = %d\n", rc);
		return rc;
	}

	/* battery missing detection */
	mask =  BATT_MISSING_ALGO_BIT;
	reg = chip->bmd_algo_disabled ? BATT_MISSING_ALGO_BIT : 0;
	if (chip->bmd_pin_src < BPD_TYPE_DEFAULT) {
		mask |= BMD_PIN_SRC_MASK;
		reg |= chip->bmd_pin_src << PIN_SRC_SHIFT;
	}
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BM_CFG, mask, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}

	smbchg_charging_status_change(chip);

	/*
	 * The charger needs 20 milliseconds to go into battery supplementary
	 * mode. Sleep here until we are sure it takes into effect.
	 */
	msleep(20);
	smbchg_usb_en(chip, chip->chg_enabled, REASON_USER);
	smbchg_dc_en(chip, chip->chg_enabled, REASON_USER);
	/* resume threshold */
	if (chip->resume_delta_mv != -EINVAL) {

		/*
		 * Configure only if the recharge threshold source is not
		 * fuel gauge ADC.
		 */
		if (!chip->chg_inhibit_source_fg) {
			if (chip->resume_delta_mv < 100)
				reg = CHG_INHIBIT_50MV_VAL;
			else if (chip->resume_delta_mv < 200)
				reg = CHG_INHIBIT_100MV_VAL;
			else if (chip->resume_delta_mv < 300)
				reg = CHG_INHIBIT_200MV_VAL;
			else
				reg = CHG_INHIBIT_300MV_VAL;

			rc = smbchg_sec_masked_write(chip,
					chip->chgr_base + CHG_INHIB_CFG_REG,
					CHG_INHIBIT_MASK, reg);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set inhibit val rc = %d\n",
						rc);
				return rc;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + CHGR_CFG,
				RCHG_LVL_BIT, (chip->resume_delta_mv < 200)
				? 0 : RCHG_LVL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

	/* DC path current settings */
	if (chip->dc_psy_type != -EINVAL) {
		rc = smbchg_set_thermal_limited_dc_current_max(chip,
						chip->dc_target_current_ma);
		if (rc < 0) {
			dev_err(chip->dev, "can't set dc current: %d\n", rc);
			return rc;
		}
	}


	/*
	 * on some devices the battery is powered via external sources which
	 * could raise its voltage above the float voltage. smbchargers go
	 * in to reverse boost in such a situation and the workaround is to
	 * disable float voltage compensation (note that the battery will appear
	 * hot/cold when powered via external source).
	 */
	if (chip->soft_vfloat_comp_disabled) {
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CFG_AFVC,
				VFLOAT_COMP_ENABLE_MASK, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable soft vfloat rc = %d\n",
					rc);
			return rc;
		}
	}

	rc = smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set fastchg current = %d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);

	if (chip->wipower_dyn_icl_avail) {
		rc = smbchg_wipower_ilim_config(chip,
				&(chip->wipower_default.entries[0]));
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set default wipower ilim = %d\n",
				rc);
			return rc;
		}
	}
	/* unsuspend dc path, it could be suspended by the bootloader */
	rc = smbchg_dc_suspend(chip, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unspended dc path= %d\n", rc);
		return rc;
	}

	if (chip->force_aicl_rerun)
		rc = smbchg_aicl_config(chip);

	return rc;
}

static struct of_device_id smbchg_match_table[] = {
	{
		.compatible     = "qcom,qpnp-smbcharger",
		.data           = (void *)ARRAY_SIZE(usb_current_table),
	},
	{ },
};

#define DC_MA_MIN 300
#define DC_MA_MAX 2000
#define OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
do {									\
	if (retval)							\
		break;							\
	if (optional)							\
		prop = -EINVAL;						\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," dt_property	,	\
					&prop);				\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		dev_err(chip->dev, "Error reading " #dt_property	\
				" property rc = %d\n", rc);		\
} while (0)

#define ILIM_ENTRIES		3
#define VOLTAGE_RANGE_ENTRIES	2
#define RANGE_ENTRY		(ILIM_ENTRIES + VOLTAGE_RANGE_ENTRIES)
static int smb_parse_wipower_map_dt(struct smbchg_chip *chip,
		struct ilim_map *map, char *property)
{
	struct device_node *node = chip->dev->of_node;
	int total_elements, size;
	struct property *prop;
	const __be32 *data;
	int num, i;

	prop = of_find_property(node, property, &size);
	if (!prop) {
		dev_err(chip->dev, "%s missing\n", property);
		return -EINVAL;
	}

	total_elements = size / sizeof(int);
	if (total_elements % RANGE_ENTRY) {
		dev_err(chip->dev, "%s table not in multiple of %d, total elements = %d\n",
				property, RANGE_ENTRY, total_elements);
		return -EINVAL;
	}

	data = prop->value;
	num = total_elements / RANGE_ENTRY;
	map->entries = devm_kzalloc(chip->dev,
			num * sizeof(struct ilim_entry), GFP_KERNEL);
	if (!map->entries) {
		dev_err(chip->dev, "kzalloc failed for default ilim\n");
		return -ENOMEM;
	}
	for (i = 0; i < num; i++) {
		map->entries[i].vmin_uv =  be32_to_cpup(data++);
		map->entries[i].vmax_uv =  be32_to_cpup(data++);
		map->entries[i].icl_pt_ma =  be32_to_cpup(data++);
		map->entries[i].icl_lv_ma =  be32_to_cpup(data++);
		map->entries[i].icl_hv_ma =  be32_to_cpup(data++);
	}
	map->num = num;
	return 0;
}

static int smb_parse_wipower_dt(struct smbchg_chip *chip)
{
	int rc = 0;

	chip->wipower_dyn_icl_avail = false;

	if (!chip->vadc_dev)
		goto err;

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_default,
					"qcom,wipower-default-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_pt,
					"qcom,wipower-pt-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_div2,
					"qcom,wipower-div2-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-div2-ilim-map rc = %d\n",
				rc);
		goto err;
	}
	chip->wipower_dyn_icl_avail = true;
	return 0;
err:
	chip->wipower_default.num = 0;
	chip->wipower_pt.num = 0;
	chip->wipower_default.num = 0;
	if (chip->wipower_default.entries)
		devm_kfree(chip->dev, chip->wipower_default.entries);
	if (chip->wipower_pt.entries)
		devm_kfree(chip->dev, chip->wipower_pt.entries);
	if (chip->wipower_div2.entries)
		devm_kfree(chip->dev, chip->wipower_div2.entries);
	chip->wipower_default.entries = NULL;
	chip->wipower_pt.entries = NULL;
	chip->wipower_div2.entries = NULL;
	chip->vadc_dev = NULL;
	return rc;
}

#define DEFAULT_VLED_MAX_UV		3500000
#define DEFAULT_FCC_MA			2000
static int smb_parse_dt(struct smbchg_chip *chip)
{
	int rc = 0, ocp_thresh = -EINVAL;
	struct device_node *node = chip->dev->of_node;
	const char *dc_psy_type, *bpd;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* read optional u32 properties */
	OF_PROP_READ(chip, ocp_thresh,
			"ibat-ocp-threshold-ua", rc, 1);
	if (ocp_thresh >= 0)
		smbchg_ibat_ocp_threshold_ua = ocp_thresh;
	OF_PROP_READ(chip, chip->iterm_ma, "iterm-ma", rc, 1);
	OF_PROP_READ(chip, chip->target_fastchg_current_ma,
			"fastchg-current-ma", rc, 1);
	if (chip->target_fastchg_current_ma == -EINVAL)
		chip->target_fastchg_current_ma = DEFAULT_FCC_MA;
	OF_PROP_READ(chip, chip->vfloat_mv, "float-voltage-mv", rc, 1);
	OF_PROP_READ(chip, chip->safety_time, "charging-timeout-mins", rc, 1);
	OF_PROP_READ(chip, chip->vled_max_uv, "vled-max-uv", rc, 1);
	if (chip->vled_max_uv < 0)
		chip->vled_max_uv = DEFAULT_VLED_MAX_UV;
	OF_PROP_READ(chip, chip->rpara_uohm, "rparasitic-uohm", rc, 1);
	if (chip->rpara_uohm < 0)
		chip->rpara_uohm = 0;
	OF_PROP_READ(chip, chip->prechg_safety_time, "precharging-timeout-mins",
			rc, 1);
	OF_PROP_READ(chip, chip->fastchg_current_comp, "fastchg-current-comp",
			rc, 1);
	OF_PROP_READ(chip, chip->float_voltage_comp, "float-voltage-comp",
			rc, 1);
	if (chip->safety_time != -EINVAL &&
		(chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout-mins %d\n",
						chip->safety_time);
		return -EINVAL;
	}
	if (chip->prechg_safety_time != -EINVAL &&
		(chip->prechg_safety_time >
		 prechg_time[ARRAY_SIZE(prechg_time) - 1])) {
		dev_err(chip->dev, "Bad precharging-timeout-mins %d\n",
						chip->prechg_safety_time);
		return -EINVAL;
	}
	OF_PROP_READ(chip, chip->resume_delta_mv, "resume-delta-mv", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_current_thr_ma,
			"parallel-usb-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_9v_current_thr_ma,
			"parallel-usb-9v-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.allowed_lowering_ma,
			"parallel-allowed-lowering-ma", rc, 1);
	chip->cfg_fastchg_current_ma = chip->target_fastchg_current_ma;
	if (chip->parallel.min_current_thr_ma != -EINVAL
			&& chip->parallel.min_9v_current_thr_ma != -EINVAL)
		chip->parallel.avail = true;
	pr_smb(PR_STATUS, "parallel usb thr: %d, 9v thr: %d\n",
			chip->parallel.min_current_thr_ma,
			chip->parallel.min_9v_current_thr_ma);
	OF_PROP_READ(chip, chip->jeita_temp_hard_limit,
			"jeita-temp-hard-limit", rc, 1);

	/* read boolean configuration properties */
	chip->use_vfloat_adjustments = of_property_read_bool(node,
						"qcom,autoadjust-vfloat");
	chip->bmd_algo_disabled = of_property_read_bool(node,
						"qcom,bmd-algo-disabled");
	chip->iterm_disabled = of_property_read_bool(node,
						"qcom,iterm-disabled");
	chip->soft_vfloat_comp_disabled = of_property_read_bool(node,
					"qcom,soft-vfloat-comp-disabled");
	chip->chg_enabled = !(of_property_read_bool(node,
						"qcom,charging-disabled"));
	chip->charge_unknown_battery = of_property_read_bool(node,
						"qcom,charge-unknown-battery");
	chip->chg_inhibit_en = of_property_read_bool(node,
					"qcom,chg-inhibit-en");
	chip->chg_inhibit_source_fg = of_property_read_bool(node,
						"qcom,chg-inhibit-fg");
	chip->low_volt_dcin = of_property_read_bool(node,
					"qcom,low-volt-dcin");
	chip->force_aicl_rerun = of_property_read_bool(node,
					"qcom,force-aicl-rerun");
	chip->enable_hvdcp_9v = of_property_read_bool(node,
					"qcom,enable-hvdcp-9v");

	/* parse the battery missing detection pin source */
	rc = of_property_read_string(chip->spmi->dev.of_node,
		"qcom,bmd-pin-src", &bpd);
	if (rc) {
		/* Select BAT_THM as default BPD scheme */
		chip->bmd_pin_src = BPD_TYPE_DEFAULT;
		rc = 0;
	} else {
		chip->bmd_pin_src = get_bpd(bpd);
		if (chip->bmd_pin_src < 0) {
			dev_err(chip->dev,
				"failed to determine bpd schema %d\n", rc);
			return rc;
		}
	}

	/* parse the dc power supply configuration */
	rc = of_property_read_string(node, "qcom,dc-psy-type", &dc_psy_type);
	if (rc) {
		chip->dc_psy_type = -EINVAL;
		rc = 0;
	} else {
		if (strcmp(dc_psy_type, "Mains") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
		else if (strcmp(dc_psy_type, "Wireless") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
		else if (strcmp(dc_psy_type, "Wipower") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIPOWER;
	}
	if (chip->dc_psy_type != -EINVAL) {
		OF_PROP_READ(chip, chip->dc_target_current_ma,
				"dc-psy-ma", rc, 0);
		if (rc)
			return rc;
		if (chip->dc_target_current_ma < DC_MA_MIN
				|| chip->dc_target_current_ma > DC_MA_MAX) {
			dev_err(chip->dev, "Bad dc mA %d\n",
					chip->dc_target_current_ma);
			return -EINVAL;
		}
	}

	if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
		smb_parse_wipower_dt(chip);

	/* read the bms power supply name */
	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	/* read the battery power supply name */
	rc = of_property_read_string(node, "qcom,battery-psy-name",
						&chip->battery_psy_name);
	if (rc)
		chip->battery_psy_name = "battery";

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define SUBTYPE_REG			0x5
#define SMBCHG_CHGR_SUBTYPE		0x1
#define SMBCHG_OTG_SUBTYPE		0x8
#define SMBCHG_BAT_IF_SUBTYPE		0x3
#define SMBCHG_USB_CHGPTH_SUBTYPE	0x4
#define SMBCHG_DC_CHGPTH_SUBTYPE	0x5
#define SMBCHG_MISC_SUBTYPE		0x7
#define SMBCHG_LITE_CHGR_SUBTYPE	0x51
#define SMBCHG_LITE_OTG_SUBTYPE		0x58
#define SMBCHG_LITE_BAT_IF_SUBTYPE	0x53
#define SMBCHG_LITE_USB_CHGPTH_SUBTYPE	0x54
#define SMBCHG_LITE_DC_CHGPTH_SUBTYPE	0x55
#define SMBCHG_LITE_MISC_SUBTYPE	0x57
#define REQUEST_IRQ(chip, resource, irq_num, irq_name, irq_handler, flags, rc)\
do {									\
	irq_num = spmi_get_irq_byname(chip->spmi,			\
					resource, irq_name);		\
	if (irq_num < 0) {						\
		dev_err(chip->dev, "Unable to get " irq_name " irq\n");	\
		return -ENXIO;						\
	}								\
	rc = devm_request_threaded_irq(chip->dev,			\
			irq_num, NULL, irq_handler, flags, irq_name,	\
			chip);						\
	if (rc < 0) {							\
		dev_err(chip->dev, "Unable to request " irq_name " irq: %d\n",\
				rc);					\
		return -ENXIO;						\
	}								\
} while (0)

/* FCNT LIMITED:2016-02-03 H16100726 add start */
static void fj_det_parse_dt(struct device_node *node, struct _fj_platform_data *fj_pdata)
{
	fj_pdata->vb_det_gpio = of_get_named_gpio_flags(node, "fj,vb_det-gpio", 0, NULL);
	fj_pdata->vc_det_gpio = of_get_named_gpio_flags(node, "fj,vc_det-gpio", 0, NULL);
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	fj_pdata->adp_safe_gpio = of_get_named_gpio_flags(node, "fj,adp_safe-gpio", 0, NULL);
	fj_pdata->usb_hot_gpio  = of_get_named_gpio_flags(node, "fj,usb_hot-gpio", 0, NULL);
	fj_pdata->usb_thsel0 = of_get_named_gpio_flags(node, "fj,usb_thsel0-gpio", 0, NULL);
	fj_pdata->usb_thsel1 = of_get_named_gpio_flags(node, "fj,usb_thsel1-gpio", 0, NULL);
/* FCNT LIMITED:2016-02-03 H16100712 add end */
}
/* FCNT LIMITED:2016-02-03 H16100726 add end */
/* FCNT LIMITED:2016-02-04 H16100727 add start */
#define FJ_DET_GPIO_SETUP(gpio, name, inout, output, rc)		\
do {															\
	if (!gpio_is_valid(gpio))									\
		break;													\
	rc = gpio_request(gpio, name);								\
	if (rc != 0) {												\
		dev_err(chip->dev, "gpio_request "name" rc=%d\n", rc);	\
		gpio = -1;												\
		break;													\
	}															\
	if (inout)													\
		gpio_direction_output(gpio, output);					\
	else														\
		gpio_direction_input(gpio);								\
} while (0)

static void fj_det_hw_init(struct smbchg_chip *chip, struct _fj_platform_data *fj_pdata)
{
	int rc;
	/* setup VBDET */
	FJ_DET_GPIO_SETUP(fj_pdata->vb_det_gpio, "FJ_GPIO_VBDET", 0, 0, rc);
	/* setup VCDET */
	FJ_DET_GPIO_SETUP(fj_pdata->vc_det_gpio, "FJ_GPIO_VCDET", 0, 0, rc);
	/* setup USB_THSEL0 */
	FJ_DET_GPIO_SETUP(fj_pdata->usb_thsel0, "FJ_USB_THSEL0", 1, 0, rc);		/* FCNT LIMITED:2016-05-27 H16100712-1 mod */
	/* setup USB_THSEL1 */
	FJ_DET_GPIO_SETUP(fj_pdata->usb_thsel1, "FJ_USB_THSEL1", 1, 1, rc);		/* FCNT LIMITED:2016-05-27 H16100712-1 mod */
	/* setup USB_HOT */
	FJ_DET_GPIO_SETUP(fj_pdata->usb_hot_gpio, "FJ_GPIO_USB_HOT", 0, 0, rc);
	/* setup ADP_SAFE */
	FJ_DET_GPIO_SETUP(fj_pdata->adp_safe_gpio, "FJ_GPIO_ADP_SAFE", 0, 0, rc);
}
/* FCNT LIMITED:2016-02-04 H16100727 add end */
/* FCNT LIMITED:2016-02-03 H16100726 mod start */
/* FUJITSU LIMITED:2015-01-07 H1510087 add start */
static int fj_request_usb_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct _fj_platform_data *fj_pdata = &fj_platform_data;

	do {
/* FCNT LIMITED:2016-02-04 H16100727 mod start */
		if (!gpio_is_valid(fj_pdata->vb_det_gpio)) {
			break;
		}

		fj_pdata->fj_vb_det_irq = gpio_to_irq(fj_pdata->vb_det_gpio);
/* FCNT LIMITED:2016-02-04 H16100727 add end */

		/* request_irq VB_DET */
		rc = devm_request_threaded_irq(chip->dev, fj_pdata->fj_vb_det_irq,
						  NULL, src_detect_handler, /* Thread function */
						  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT,
						  fj_pdata->fj_vbdet_name, chip);
		if (rc < 0) {
			dev_err(chip->dev, "Unable to request vbdet irq: %d\n", rc);
			gpio_free(fj_pdata->vb_det_gpio);
			fj_pdata->vb_det_gpio = -1;
			break;
		}
		enable_irq_wake(fj_pdata->fj_vb_det_irq);

	} while (0);

	return rc;
}

static int fj_request_dc_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct _fj_platform_data *fj_pdata = &fj_platform_data;

	do {
/* FCNT LIMITED:2016-02-04 H16100727 mod start */
		if (!gpio_is_valid(fj_pdata->vc_det_gpio)) {
			break;
		}
		fj_pdata->fj_vc_det_irq = gpio_to_irq(fj_pdata->vc_det_gpio);
/* FCNT LIMITED:2016-02-04 H16100727 add end */

		/* request_irq VC_DET */
		rc = devm_request_threaded_irq(chip->dev, fj_pdata->fj_vc_det_irq,
						  NULL, dcin_uv_handler, /* Thread function */
						  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT,
						  fj_pdata->fj_vcdet_name, chip);
		if (rc < 0) {
			dev_err(chip->dev, "Unable to request vcdet irq: %d\n", rc);
			gpio_free(fj_pdata->vc_det_gpio);
			fj_pdata->vc_det_gpio = -1;
			break;
		}
		enable_irq_wake(fj_pdata->fj_vc_det_irq);
	} while (0);

	return rc;
}
/* FUJITSU LIMITED:2015-01-07 H1510087 add end */
/* FCNT LIMITED:2016-02-03 H16100726 mod end */
/* FCNT LIMITED:2016-02-03 H16100712 add start */
static int fj_request_usb_safety_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct _fj_platform_data *fj_pdata = &fj_platform_data;

	if (!gpio_is_valid(fj_pdata->adp_safe_gpio) || !gpio_is_valid(fj_pdata->usb_hot_gpio)) {
		pr_info("usb_safety_irqs disable\n");
		return rc;
	}
/* FCNT LIMITED:2016-02-04 H16100727 mod start */
	do {
		fj_pdata->usb_hot_irq = gpio_to_irq(fj_pdata->usb_hot_gpio);

		/* request_irq usb_hot */
		rc = devm_request_threaded_irq(chip->dev, fj_pdata->usb_hot_irq,
						NULL, fj_usb_hot_handler, /* Thread function */
/* FCNT LIMITED:2017-04-12 H17101051 add start */
/*						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT, */
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
/* FCNT LIMITED:2017-04-12 H17101051 add end */
						"fj_usb_hot", chip);
		if (rc < 0) {
			dev_err(chip->dev, "Unable to request usb_hot irq: %d\n", rc);
			gpio_free(fj_pdata->usb_hot_gpio);
			fj_pdata->usb_hot_gpio = -1;
			break;
		}
		enable_irq_wake(fj_pdata->usb_hot_irq);
	} while (0);

	do {
		fj_pdata->adp_safe_irq = gpio_to_irq(fj_pdata->adp_safe_gpio);

		/* request_irq adp_safe */
		rc = devm_request_threaded_irq(chip->dev, fj_pdata->adp_safe_irq,
						NULL, fj_adp_safe_handler, /* Thread function */
/* FCNT LIMITED:2017-04-12 H17101051 add start */
/*						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT, */
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
/* FCNT LIMITED:2017-04-12 H17101051 add end */
						"fj_adp_safe", chip);
		if (rc < 0) {
			dev_err(chip->dev, "Unable to request adp_safe irq: %d\n", rc);
			gpio_free(fj_pdata->adp_safe_gpio);
			fj_pdata->adp_safe_gpio = -1;
			break;
		}
		enable_irq_wake(fj_pdata->adp_safe_irq);
	} while (0);
/* FCNT LIMITED:2016-02-04 H16100727 mod end */
	return rc;
}
/* FCNT LIMITED:2016-02-03 H16100712 add end */
static int smbchg_request_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							| IRQF_ONESHOT;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->chg_error_irq,
				"chg-error", chg_error_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->taper_irq,
				"chg-taper-thr", taper_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			disable_irq_nosync(chip->taper_irq);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_term_irq,
				"chg-tcc-thr", chg_term_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource, chip->recharge_irq,
				"chg-rechg-thr", recharge_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->fastchg_irq,
				"chg-p2f-thr", fastchg_handler, flags, rc);
			enable_irq_wake(chip->chg_term_irq);
			enable_irq_wake(chip->chg_error_irq);
			enable_irq_wake(chip->fastchg_irq);
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->batt_hot_irq,
				"batt-hot", batt_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_warm_irq,
				"batt-warm", batt_warm_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cool_irq,
				"batt-cool", batt_cool_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cold_irq,
				"batt-cold", batt_cold_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_missing_irq,
				"batt-missing", batt_pres_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->vbat_low_irq,
				"batt-low", vbat_low_handler, flags, rc);
			enable_irq_wake(chip->batt_hot_irq);
			enable_irq_wake(chip->batt_warm_irq);
			enable_irq_wake(chip->batt_cool_irq);
			enable_irq_wake(chip->batt_cold_irq);
			enable_irq_wake(chip->batt_missing_irq);
			enable_irq_wake(chip->vbat_low_irq);
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
/* FUJITSU LIMITED:2015-01-07 H1510087 mod start */
#if 0
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_uv_irq,
				"usbin-uv", usbin_uv_handler,
				flags | IRQF_EARLY_RESUME, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_ov_irq,
				"usbin-ov", usbin_ov_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->src_detect_irq,
				"usbin-src-det",
				src_detect_handler, flags, rc);
#else
			rc = fj_request_usb_irqs(chip);
#endif
/* FUJITSU LIMITED:2015-01-07 H1510087 mod end */
			REQUEST_IRQ(chip, spmi_resource, chip->aicl_done_irq,
				"aicl-done",
				aicl_done_handler, flags, rc);
			if (chip->schg_version != QPNP_SCHG_LITE) {
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_fail_irq, "otg-fail",
					otg_fail_handler, flags, rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_oc_irq, "otg-oc",
					otg_oc_handler,
					(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
					rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->usbid_change_irq, "usbid-change",
					usbid_change_handler,
					(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
					rc);
				enable_irq_wake(chip->otg_oc_irq);
				enable_irq_wake(chip->usbid_change_irq);
				enable_irq_wake(chip->otg_fail_irq);
			}
/* FUJITSU LIMITED:2015-01-07 H1510087 mod start */
#if 0
			enable_irq_wake(chip->usbin_uv_irq);
			enable_irq_wake(chip->usbin_ov_irq);
			enable_irq_wake(chip->src_detect_irq);
#else
			REQUEST_IRQ(chip, spmi_resource, fj_platform_data.fj_usbin_ov_irq,
				"usbin-ov", usbin_ov_handler, flags, rc);
			enable_irq_wake(fj_platform_data.fj_usbin_ov_irq);
#endif
/* FUJITSU LIMITED:2015-01-07 H1510087 mod end */
			if (chip->parallel.avail && chip->usb_present) {
				rc = enable_irq_wake(chip->aicl_done_irq);
				chip->enable_aicl_wake = true;
			}
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
/* FUJITSU LIMITED:2015-01-07 H1510087 mod start */
#if 0
			REQUEST_IRQ(chip, spmi_resource, chip->dcin_uv_irq,
				"dcin-uv", dcin_uv_handler, flags, rc);
			enable_irq_wake(chip->dcin_uv_irq);
#else
/* FCNT LIMITED:2016-02-03 H16100711 mod start */
			if (!gpio_is_valid(fj_platform_data.vc_det_gpio)) {
				pr_info("vc_det disable\n");
				break;
			}
			REQUEST_IRQ(chip, spmi_resource, fj_platform_data.fj_dcin_ov_irq,
				"dcin-ov", dcin_ov_handler, flags, rc);
			enable_irq_wake(fj_platform_data.fj_dcin_ov_irq);
			rc = fj_request_dc_irqs(chip);
/* FCNT LIMITED:2016-02-03 H16100711 mod end */
#endif
/* FUJITSU LIMITED:2015-01-07 H1510087 mod end */
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->power_ok_irq,
				"power-ok", power_ok_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_hot_irq,
				"temp-shutdown", chg_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->safety_timeout_irq,
				"safety-timeout",
				safety_timeout_handler, flags, rc);
			enable_irq_wake(chip->chg_hot_irq);
			enable_irq_wake(chip->safety_timeout_irq);
			break;
		case SMBCHG_OTG_SUBTYPE:
			break;
		case SMBCHG_LITE_OTG_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource,
				chip->usbid_change_irq, "usbid-change",
				usbid_change_handler,
				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
				rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_oc_irq, "otg-oc",
				otg_oc_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_fail_irq, "otg-fail",
				otg_fail_handler, flags, rc);
			enable_irq_wake(chip->usbid_change_irq);
			enable_irq_wake(chip->otg_oc_irq);
			enable_irq_wake(chip->otg_fail_irq);
			break;
		}
	}

	return rc;
}

#define REQUIRE_BASE(chip, base, rc)					\
do {									\
	if (!rc && !chip->base) {					\
		dev_err(chip->dev, "Missing " #base "\n");		\
		rc = -EINVAL;						\
	}								\
} while (0)

static int smbchg_parse_peripherals(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			chip->chgr_base = resource->start;
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			chip->bat_if_base = resource->start;
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			chip->usb_chgpth_base = resource->start;
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			chip->dc_chgpth_base = resource->start;
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			chip->misc_base = resource->start;
			break;
		case SMBCHG_OTG_SUBTYPE:
		case SMBCHG_LITE_OTG_SUBTYPE:
			chip->otg_base = resource->start;
			break;
		}
	}

	REQUIRE_BASE(chip, chgr_base, rc);
	REQUIRE_BASE(chip, bat_if_base, rc);
	REQUIRE_BASE(chip, usb_chgpth_base, rc);
	REQUIRE_BASE(chip, dc_chgpth_base, rc);
	REQUIRE_BASE(chip, misc_base, rc);

	return rc;
}

static inline void dump_reg(struct smbchg_chip *chip, u16 addr,
		const char *name)
{
	u8 reg;

	smbchg_read(chip, &reg, addr, 1);
	pr_smb(PR_DUMP, "%s - %04X = %02X\n", name, addr, reg);
}

/* dumps useful registers for debug */
static void dump_regs(struct smbchg_chip *chip)
{
	u16 addr;

	/* charger peripheral */
	for (addr = 0xB; addr <= 0x10; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Status");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Config");
	/* battery interface peripheral */
	dump_reg(chip, chip->bat_if_base + RT_STS, "BAT_IF Status");
	dump_reg(chip, chip->bat_if_base + CMD_CHG_REG, "BAT_IF Command");
	for (addr = 0xF0; addr <= 0xFB; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF Config");
	/* usb charge path peripheral */
	for (addr = 0x7; addr <= 0x10; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Status");
	dump_reg(chip, chip->usb_chgpth_base + CMD_IL, "USB Command");
	for (addr = 0xF0; addr <= 0xF5; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Config");
	/* dc charge path peripheral */
	dump_reg(chip, chip->dc_chgpth_base + RT_STS, "DC Status");
	for (addr = 0xF0; addr <= 0xF6; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC Config");
	/* misc peripheral */
	dump_reg(chip, chip->misc_base + IDEV_STS, "MISC Status");
	dump_reg(chip, chip->misc_base + RT_STS, "MISC Status");
	for (addr = 0xF0; addr <= 0xF3; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC CFG");
}

static int create_debugfs_entries(struct smbchg_chip *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("qpnp-smbcharger", NULL);
	if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -EINVAL;
	}

	ent = debugfs_create_file("force_dcin_icl_check",
				  S_IFREG | S_IWUSR | S_IRUGO,
				  chip->debug_root, chip,
				  &force_dcin_icl_ops);
	if (!ent) {
		dev_err(chip->dev,
			"Couldn't create force dcin icl check file\n");
		return -EINVAL;
	}
	return 0;
}

static int smbchg_wa_config(struct smbchg_chip *chip)
{
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(chip->spmi->dev.of_node,
					"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property - driver failed\n");
		return -EINVAL;
	}

	pmic_rev_id = get_revid_data(revid_dev_node);
	if (IS_ERR(pmic_rev_id)) {
		pr_err("Unable to get pmic_revid rc=%ld\n",
				PTR_ERR(pmic_rev_id));
		return -EPROBE_DEFER;
	}

	switch (pmic_rev_id->pmic_subtype) {
	case PMI8994:
		chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA;
	case PMI8950:
		if (pmic_rev_id->rev4 < 2) /* PMI8950 1.0 */
			chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA;
		else	/* rev > PMI8950 v1.0 */
			chip->wa_flags |= SMBCHG_HVDCP_9V_EN_WA;
		break;
	default:
		pr_err("PMIC subtype %d not supported, WA flags not set\n",
				pmic_rev_id->pmic_subtype);
	}

	pr_debug("wa_flags=0x%x\n", chip->wa_flags);

	return 0;
}

static int smbchg_check_chg_version(struct smbchg_chip *chip)
{
	int rc;
	u8 val = 0;

	if (!chip->chgr_base) {
		pr_err("CHG base not specifed, unable to detect chg\n");
		return -EINVAL;
	}

	rc = smbchg_read(chip, &val, chip->chgr_base + SUBTYPE_REG, 1);
	if (rc) {
		pr_err("unable to read subtype reg rc=%d\n", rc);
		return rc;
	}

	switch (val) {
	case SMBCHG_CHGR_SUBTYPE:
		chip->schg_version = QPNP_SCHG;
		break;
	case SMBCHG_LITE_CHGR_SUBTYPE:
		chip->schg_version = QPNP_SCHG_LITE;
		break;
	default:
		pr_err("Invalid charger subtype=%x\n", val);
		break;
	}

	rc = smbchg_wa_config(chip);
	if (rc)
		pr_err("Charger WA flags not configured rc=%d\n", rc);

	return rc;
}

static int smbchg_probe(struct spmi_device *spmi)
{
	int rc;
	struct smbchg_chip *chip;
	struct power_supply *usb_psy;
	struct qpnp_vadc_chip *vadc_dev;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_smb(PR_STATUS, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	if (of_find_property(spmi->dev.of_node, "qcom,dcin-vadc", NULL)) {
		vadc_dev = qpnp_get_vadc(&spmi->dev, "dcin");
		if (IS_ERR(vadc_dev)) {
			rc = PTR_ERR(vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc rc=%d\n",
						rc);
			return rc;
		}
	}

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	fj_det_parse_dt(spmi->dev.of_node, &fj_platform_data);	/* FCNT LIMITED:2016-02-03 H16100726 add */
	INIT_WORK(&chip->usb_set_online_work, smbchg_usb_update_online_work);
	INIT_DELAYED_WORK(&chip->parallel_en_work,
			smbchg_parallel_usb_en_work);
	INIT_DELAYED_WORK(&chip->vfloat_adjust_work, smbchg_vfloat_adjust_work);
	INIT_DELAYED_WORK(&chip->hvdcp_det_work, smbchg_hvdcp_det_work);
	chip->vadc_dev = vadc_dev;
	chip->spmi = spmi;
	chip->dev = &spmi->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
	chip->usb_online = -EINVAL;
	dev_set_drvdata(&spmi->dev, chip);

	spin_lock_init(&chip->sec_access_lock);
	mutex_init(&chip->fcc_lock);
	mutex_init(&chip->current_change_lock);
	mutex_init(&chip->usb_set_online_lock);
	mutex_init(&chip->battchg_disabled_lock);
	mutex_init(&chip->usb_en_lock);
	mutex_init(&chip->dc_en_lock);
	mutex_init(&chip->parallel.lock);
	mutex_init(&chip->taper_irq_lock);
	mutex_init(&chip->pm_lock);
	mutex_init(&chip->wipower_config);
	mutex_init(&chip->usb_status_lock);
	device_init_wakeup(chip->dev, true);
/* FUJITSU LIMITED:2015-01-07 H1510087 add start */
	mutex_init(&fj_platform_data.fj_state_lock);

	wake_lock_init(&fj_platform_data.fj_vbdet_lock, WAKE_LOCK_SUSPEND
			, "fj_vbdet_wake_lock");
	wake_lock_init(&fj_platform_data.fj_vcdet_lock, WAKE_LOCK_SUSPEND
			, "fj_vcdet_wake_lock");
	wake_lock_init(&fj_platform_data.fj_usbin_ov_lock, WAKE_LOCK_SUSPEND
			, "fj_usbin_ov_wake_lock");
	wake_lock_init(&fj_platform_data.fj_dcin_ov_lock, WAKE_LOCK_SUSPEND
			, "fj_dcin_ov_wake_lock");
	wake_lock_init(&fj_platform_data.fj_usbid_lock, WAKE_LOCK_SUSPEND
			, "fj_usbid_wake_lock");
/* FUJITSU LIMITED:2015-01-07 H1510087 add end */
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	mutex_init(&fj_platform_data.fj_safety_state_lock);
	wake_lock_init(&fj_platform_data.fj_usb_hot_lock, WAKE_LOCK_SUSPEND , "fj_usb_hot_wake_lock");
	wake_lock_init(&fj_platform_data.fj_adp_safe_lock, WAKE_LOCK_SUSPEND , "fj_adp_safe_wake_lock");
/* FCNT LIMITED:2016-02-03 H16100712 add end */
/* FCNT LIMITED:2016-02-03 H16100722 add start */
	mutex_init(&fj_platform_data.fj_otg_lock);
	host_ext_event_driver_register();
/* FCNT LIMITED:2016-02-03 H16100722 add end */
	rc = smbchg_parse_peripherals(chip);
	if (rc) {
		dev_err(chip->dev, "Error parsing DT peripherals: %d\n", rc);
		return rc;
	}
/* FUJITSU LIMITED:2016-01-05 CHARGER add start */
	the_chip = chip;
/* FUJITSU LIMITED:2016-01-05 CHARGER add end */
	rc = smbchg_check_chg_version(chip);
	if (rc) {
		pr_err("Unable to check schg version rc=%d\n", rc);
		return rc;
	}

	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to parse DT nodes: %d\n", rc);
		return rc;
	}

	rc = smbchg_regulator_init(chip);
	if (rc) {
		dev_err(&spmi->dev,
			"Couldn't initialize regulator rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_hw_init(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto free_regulator;
	}

	fj_det_hw_init(chip, &fj_platform_data);	/* FCNT LIMITED:2016-02-04 H16100727 add */

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto free_regulator;
	}

	chip->previous_soc = -EINVAL;
	chip->batt_psy.name		= chip->battery_psy_name;
/* FUJITSU LIMITED:2016-01-05 CHARGER mod start */
#if 0
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
#else
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_Q_BATTERY;
#endif
/* FUJITSU LIMITED:2016-01-05 CHARGER mod end */
	chip->batt_psy.get_property	= smbchg_battery_get_property;
	chip->batt_psy.set_property	= smbchg_battery_set_property;
	chip->batt_psy.properties	= smbchg_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smbchg_battery_properties);
	chip->batt_psy.external_power_changed = smbchg_external_power_changed;
	chip->batt_psy.property_is_writeable = smbchg_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto free_regulator;
	}
	if (chip->dc_psy_type != -EINVAL) {
		chip->dc_psy.name		= "dc";
		chip->dc_psy.type		= chip->dc_psy_type;
		chip->dc_psy.get_property	= smbchg_dc_get_property;
		chip->dc_psy.set_property	= smbchg_dc_set_property;
		chip->dc_psy.property_is_writeable = smbchg_dc_is_writeable;
		chip->dc_psy.properties		= smbchg_dc_properties;
		chip->dc_psy.num_properties = ARRAY_SIZE(smbchg_dc_properties);
		chip->dc_psy.supplied_to = smbchg_dc_supplicants;
		chip->dc_psy.num_supplicants
			= ARRAY_SIZE(smbchg_dc_supplicants);
		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			dev_err(&spmi->dev,
				"Unable to register dc_psy rc = %d\n", rc);
			goto unregister_batt_psy;
		}
	}
	chip->psy_registered = true;

	rc = smbchg_request_irqs(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to request irqs rc = %d\n", rc);
		goto unregister_dc_psy;
	}
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	fj_request_usb_safety_irqs(chip);
/* FCNT LIMITED:2016-02-03 H16100712 add end */

	power_supply_set_present(chip->usb_psy, chip->usb_present);

	dump_regs(chip);
	create_debugfs_entries(chip);
	dev_info(chip->dev,
		"SMBCHG successfully probe Charger version=%s Revision DIG:%d.%d ANA:%d.%d batt=%d dc=%d usb=%d\n",
			version_str[chip->schg_version],
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR],
			get_prop_batt_present(chip),
			chip->dc_present, chip->usb_present);

/* FUJITSU LIMITED:2016-01-12 H16100714 add start */
	if (charging_mode) {
		if (fj_platform_data.fj_id_device == FJ_NO_DEVICE) {
			usb_id_info = FJ_DEVICE_PROBE_END;
		}
	}
/* FUJITSU LIMITED:2016-01-12 H16100714 add end */
	return 0;

unregister_dc_psy:
	power_supply_unregister(&chip->dc_psy);
unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
free_regulator:
	smbchg_regulator_deinit(chip);
	handle_usb_removal(chip);
	return rc;
}

static int smbchg_remove(struct spmi_device *spmi)
{
	struct smbchg_chip *chip = dev_get_drvdata(&spmi->dev);

/* FUJITSU LIMITED:2015-01-07 H1510087 add start */
	wake_lock_destroy(&fj_platform_data.fj_vbdet_lock);
	wake_lock_destroy(&fj_platform_data.fj_vcdet_lock);
	wake_lock_destroy(&fj_platform_data.fj_usbin_ov_lock);
	wake_lock_destroy(&fj_platform_data.fj_dcin_ov_lock);
	wake_lock_destroy(&fj_platform_data.fj_usbid_lock);
/* FCNT LIMITED:2016-02-03 H16100711 mod start */
	if (gpio_is_valid(fj_platform_data.vb_det_gpio)) {
		free_irq(fj_platform_data.fj_vb_det_irq, NULL);
		gpio_free(fj_platform_data.vb_det_gpio);	/* FCNT LIMITED:2016-02-03 H16100726 mod */
	}
	if (gpio_is_valid(fj_platform_data.vc_det_gpio)) {
		free_irq(fj_platform_data.fj_vc_det_irq, NULL);
		gpio_free(fj_platform_data.vc_det_gpio);	/* FCNT LIMITED:2016-02-03 H16100726 mod */
	}
/* FCNT LIMITED:2016-02-03 H16100711 mod end */
/* FUJITSU LIMITED:2015-01-07 H1510087 add end */
/* FCNT LIMITED:2016-02-03 H16100712 add start */
	if (gpio_is_valid(fj_platform_data.adp_safe_gpio)) {
		free_irq(fj_platform_data.adp_safe_irq, NULL);
		gpio_free(fj_platform_data.adp_safe_gpio);
	}
	if (gpio_is_valid(fj_platform_data.usb_hot_gpio)) {
		free_irq(fj_platform_data.usb_hot_irq, NULL);
		gpio_free(fj_platform_data.usb_hot_gpio);
	}
	wake_lock_destroy(&fj_platform_data.fj_usb_hot_lock);
	wake_lock_destroy(&fj_platform_data.fj_adp_safe_lock);
/* FCNT LIMITED:2016-02-03 H16100712 add end */
	host_ext_event_driver_unregister();				/* FCNT LIMITED:2016-02-03 H16100722 add */
	debugfs_remove_recursive(chip->debug_root);

	if (chip->dc_psy_type != -EINVAL)
		power_supply_unregister(&chip->dc_psy);

	power_supply_unregister(&chip->batt_psy);
	smbchg_regulator_deinit(chip);
/* FUJITSU LIMITED:2016-01-05 CHARGER add start */
    the_chip = NULL;
/* FUJITSU LIMITED:2016-01-05 CHARGER add end */
	return 0;
}

static const struct dev_pm_ops smbchg_pm_ops = {
};

MODULE_DEVICE_TABLE(spmi, smbchg_id);

static struct spmi_driver smbchg_driver = {
	.driver		= {
		.name		= "qpnp-smbcharger",
		.owner		= THIS_MODULE,
		.of_match_table	= smbchg_match_table,
		.pm		= &smbchg_pm_ops,
	},
	.probe		= smbchg_probe,
	.remove		= smbchg_remove,
};

static int __init smbchg_init(void)
{
	return spmi_driver_register(&smbchg_driver);
}

static void __exit smbchg_exit(void)
{
	return spmi_driver_unregister(&smbchg_driver);
}

module_init(smbchg_init);
module_exit(smbchg_exit);

MODULE_DESCRIPTION("QPNP SMB Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qpnp-smbcharger");
