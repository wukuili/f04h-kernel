/*
 * fj_mhl_platform.c
 *
 * COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
 * COPYRIGHT(C) FUJITSU LIMITED 2014-2016
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

#include <linux/hrtimer.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/semaphore.h>
#include <linux/regulator/consumer.h>
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0006 add start */
#include <linux/mfd/fj_charger.h>
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0006 add end */
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0004 add start */
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0004 add end */
#include "../mdss_hdmi_mhl.h"

#include "si_fw_macros.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "mhl_rcp_inputdev.h"
#if (INCLUDE_RBP == 1)
#include "mhl_rbp_inputdev.h"
#endif
#include "mhl_linux_tx.h"
#include "mhl_supp.h"
#include "platform.h"
#include "si_mhl_callback_api.h"
#include "si_8620_regs.h"

#include "fj_mhl_platform.h"



static int gpio_array[] = {
	FJ_TX_HW_INT,
	FJ_TX_HW_RESET,
	FJ_TX_FW_WAKE,
};

enum gpio_direction_types {
	GPIO_OUTPUT,
	GPIO_INPUT
};

struct mhl_hdmi_hpd_ctl {
	struct msm_hdmi_mhl_ops hdmi_ops;
	struct platform_device *hdmi_pdev;
};


static struct gpio mhl_control_gpios[ARRAY_SIZE(gpio_array)];
static struct regulator* mhl_1v;
static struct mhl_hdmi_hpd_ctl hdmi_hpd_ctl;
static int switch_sel_1_gpio;
static int switch_sel_2_gpio;
static struct clk *mhl_clk;

static struct device *mhl_dev;
static int mhl_connect_state = 0;
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0006 add start */
static int mhl_charge_state = 0;
static uint16_t charge_current = 0;
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0006 add end */

#define MHL_1P0_HPM_LOAD    350000 /* qcom,hpm-min-load:10000 */


int mhl_platform_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct platform_device *hdmi_pdev = NULL;
	struct device_node *hdmi_tx_node = NULL;
	int rc = 0;
	uint32_t count;
	int i, pin;
	uint32_t *val_array = NULL;

	pr_info("%s\n", __func__);
	mhl_dev = dev;

	count = of_gpio_count(np);
	if (count != ARRAY_SIZE(gpio_array)) {
		pr_err("%s:gpio invalid device tree data\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(gpio_array); i++) {
		pin = of_get_gpio(np, i);
		if (pin < 0) {
			pr_err("%s: no gpio index %d\n", __func__, i);
			return -EINVAL;
		}
		mhl_control_gpios[gpio_array[i]].gpio = pin;
		pr_debug("%s: gpio_array %d, pin=%d", __func__, gpio_array[i], pin);
	}

	if (!of_get_property(np, "si,gpio-req-num", &count)) {
		pr_err("%s: gpio_req invalid device tree data\n", __func__);
		return -EINVAL;
	}

	count /= sizeof(uint32_t);
	val_array = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!val_array) {
		pr_err("%s: no memory\n", __func__);
		return -ENOMEM;
	}

	rc = of_property_read_u32_array(np, "si,gpio-req-flags",
			val_array, count);
	if (rc < 0) {
		pr_err("%s: read gpio-req-flags fail\n", __func__);
		goto end;
	}
	for (i = 0; i < count; i++) {
		mhl_control_gpios[gpio_array[i]].flags = val_array[i];
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(np, "si,gpio-req-label",
				i, &mhl_control_gpios[i].label);
		if (rc < 0) {
			pr_err("%s: read gpio-req-lable fail\n", __func__);
			goto end;
		}
	}

	for (i = 0; i < count; i++) {
		pr_info("%s: req_tab[%s], gpio=%d, flags=%lu\n", __func__,
			mhl_control_gpios[i].label,
			mhl_control_gpios[i].gpio,
			mhl_control_gpios[i].flags);
	}

	mhl_1v = regulator_get(dev, "si,mhl1v");
	if (IS_ERR(mhl_1v)) {
		pr_err("could not get mhl_1v, rc = %ld\n",
			PTR_ERR(mhl_1v));
		rc = -ENODEV;
		goto end;
	}

	hdmi_tx_node = of_parse_phandle(np, "qcom,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("%s: can't find hdmi phandle\n", __func__);
		rc = -EINVAL;
		goto end;
	}

	hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!hdmi_pdev) {
		pr_err("%s: can't find the device by node\n", __func__);
		rc = -EINVAL;
		goto end;
	}
	hdmi_hpd_ctl.hdmi_pdev = hdmi_pdev;

	/* USB/MHL switch (SEL1) */
	switch_sel_1_gpio = of_get_named_gpio(np, "mhl-switch-sel-1-gpio", 0);
	if (switch_sel_1_gpio < 0) {
		pr_err("%s: Can't get mhl-switch-sel-1-gpio\n", __func__);
		rc = -EINVAL;
		goto end;
	}

	/* USB/MHL switch (SEL2) */
	switch_sel_2_gpio = of_get_named_gpio(np, "mhl-switch-sel-2-gpio", 0);
	if (switch_sel_2_gpio < 0) {
		pr_err("%s: Can't get mhl-switch-sel-2-gpio\n", __func__);
		rc = -EINVAL;
		goto end;
	}

	gpio_index = mhl_get_int_gpio();

	mhl_clk = clk_get(dev, "mhl_clk");
	pr_info("%s():clk_get:%pK\n", __func__, mhl_clk);
	if (!mhl_clk) {
		pr_err("%s: invalid clk\n", __func__);
		rc = -EINVAL;
	}

end:
	kfree(val_array);
	return rc;
}


int mhl_platform_init(void)
{
	int rc = -1;

	pr_info("%s\n", __func__);

	rc = gpio_request_array(mhl_control_gpios, ARRAY_SIZE(mhl_control_gpios));
	if (rc < 0) {
		pr_err("%s gpio_request_array failed, error code %d\n", __func__, rc);
		return rc;
	}

	rc = mhl_set_gpio("mhl-switch-sel-1-gpio", switch_sel_1_gpio, GPIO_OUTPUT, 0);
	if (rc < 0) {
		pr_err("%s switch sel 1\n", __func__);
		return rc;
	}

	rc = mhl_set_gpio("mhl-switch-sel-2-gpio", switch_sel_2_gpio, GPIO_OUTPUT, 0);
	if (rc < 0) {
		pr_err("%s switch sel 2\n", __func__);
		return rc;
	}

	rc = msm_hdmi_register_mhl(hdmi_hpd_ctl.hdmi_pdev, &hdmi_hpd_ctl.hdmi_ops, NULL);
	if (rc) {
		pr_err("%s: register with hdmi failed\n", __func__);
		return rc;
	}

	if (!hdmi_hpd_ctl.hdmi_ops.set_upstream_hpd) {
		pr_err("%s: func ptr is NULL\n", __func__);
		return rc;
	}

	return 0;
}


int mhl_platform_prepare_init(void)
{
	pr_info("%s\n", __func__);
	return 0;
}


void mhl_platform_power_on(void)
{
	int rc = -1;

	pr_info("%s\n", __func__);

	pr_info("regulator_set_optimum_mode:HPM\n");
	rc = regulator_set_optimum_mode(mhl_1v, MHL_1P0_HPM_LOAD);
	if (rc < 0) {
		pr_err("%s: Unable to set HPM of the regulator: MHL_1p0\n", __func__);
		return;
	}

	/* +1.0V_MHL = ON */
	pr_info("enable MHL 1.0V\n");
	if (mhl_1v) {
		rc = regulator_enable(mhl_1v);
		if (rc) {
			pr_err("%s: MHL 1.0V failed\n", __func__);
			return;
		}
	} else {
		pr_err("%s: MHL 1.0V failed(mhl_1v==NULL)\n", __func__);
		return;
	}

	/* MHL clk = ON */
	pr_info("enable MHL clk\n");
	rc = clk_prepare_enable(mhl_clk);
	if (rc) {
		pr_err("%s: MHL clock enable failed\n", __func__);
		return;
	}

	/* FW_WAKE = High */
	pr_info("FW_WAKE:1V\n");
	mhl_gpio_set_value(FJ_TX_FW_WAKE, 1);

	/* RESET = High*/
	pr_info("HW_RESET:1\n");
	mhl_gpio_set_value(FJ_TX_HW_RESET, 1);

	/* RESET = Low, 1[ms] wait, RESET = High */
	pr_info("HW_RESET:0 -> wait(1ms) -> HW_RESET:1\n");
	platform_mhl_tx_hw_reset(100 /*TX_HW_RESET_PERIOD*/, 10 /*TX_HW_RESET_DELAY*/);

	/* Power control register:0xFE */
	if (mhl_dev) {
		struct mhl_dev_context *dev_context;
		struct drv_hw_context *hw_context;

		pr_info("Power Control register:0xFE\n");
		dev_context = dev_get_drvdata(mhl_dev);
		hw_context = (struct drv_hw_context *)&dev_context->drv_context;
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0012 mod start */
#ifdef _ENABLE_MHL3_FEATURE_
		mhl_tx_write_reg(hw_context, REG_DPD, 0xFE);
#else
		pr_info("Power Control register:0xF6(disable HSIC)\n");
		mhl_tx_write_reg(hw_context, REG_DPD, 0xF6);
#endif
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0012 mod end */
	}
}


void mhl_platform_power_off(void)
{
	/* Power control register:0x00 */
	if (mhl_dev) {
		struct mhl_dev_context *dev_context;
		struct drv_hw_context *hw_context;

		pr_info("Power Control register:0x00\n");
		dev_context = dev_get_drvdata(mhl_dev);
		hw_context = (struct drv_hw_context *)&dev_context->drv_context;
		mhl_tx_write_reg(hw_context, REG_DPD, 0x00);
	}

	/* FW_WAKE = Low */
	pr_info("FW_WAKE:0\n");
	mhl_gpio_set_value(FJ_TX_FW_WAKE, 0);

	/* RESET  = Low */
	pr_info("HW_RESET:0\n");
	mhl_gpio_set_value(FJ_TX_HW_RESET, 0);

	/* MHL clk = OFF */
	pr_info("disable MHL clk\n");
	clk_disable_unprepare(mhl_clk);

	/* +1.0V_MHL = OFF */
	pr_info("disable MHL 1.0\n");
	regulator_disable(mhl_1v);
	pr_info("regulator_set_optimum_mode:0\n");
	regulator_set_optimum_mode(mhl_1v, 0);
}


int mhl_get_int_gpio(void)
{
	return mhl_control_gpios[FJ_TX_HW_INT].gpio;
}


int mhl_set_gpio(const char *gpio_name, int gpio_number,
		int direction, int out_val)
{
	int ret = -EBUSY;
	pr_debug("%s()\n", __func__);
	pr_debug("%s:%s=[%d]\n", __func__, gpio_name, gpio_number);

	if (gpio_number < 0)
		return -EINVAL;

	if (gpio_is_valid(gpio_number)) {
		ret = gpio_request((unsigned int)gpio_number, gpio_name);
		if (ret < 0) {
			pr_err("%s:%s=[%d] req failed:%d\n",
				__func__, gpio_name, gpio_number, ret);
			return -EBUSY;
		}
		if (direction == GPIO_OUTPUT) {
			pr_debug("%s:gpio output\n", __func__);
			ret = gpio_direction_output(
				(unsigned int)gpio_number, out_val);
		} else if (direction == GPIO_INPUT) {
			pr_debug("%s:gpio input\n", __func__);
			ret = gpio_direction_input((unsigned int)gpio_number);
		} else {
			pr_err("%s:%s=[%d] invalid direction type :%d\n",
				__func__, gpio_name, gpio_number, ret);
			return -EINVAL;
		}
		if (ret < 0) {
			pr_err("%s: set dirn %s failed: %d\n",
				__func__, gpio_name, ret);
			return -EBUSY;
		}
	}

	return 0;
}


int mhl_gpio_get_value(GPIO_INDEX gpio_index)
{
	return gpio_get_value(mhl_control_gpios[gpio_index].gpio);
}


void mhl_gpio_set_value(GPIO_INDEX gpio_index, int val)
{
	gpio_set_value(mhl_control_gpios[gpio_index].gpio, val);
	return;
}


void mhl_set_hdmi_hpd(uint8_t on)
{
	if (hdmi_hpd_ctl.hdmi_ops.set_upstream_hpd) {
		pr_info("%s:set_upstream_hpd:%d NULL\n", __func__, on);
		hdmi_hpd_ctl.hdmi_ops.set_upstream_hpd(hdmi_hpd_ctl.hdmi_pdev, on);
	} else {
		pr_info("%s: hdmi_ops set_upstream_hpd == NULL\n", __func__);
	}
}

void mhl_set_hev_vic(uint8_t write_burst_vic)
{
	if (hdmi_hpd_ctl.hdmi_pdev && hdmi_hpd_ctl.hdmi_ops.set_hev_vic) {
		hdmi_hpd_ctl.hdmi_ops.set_hev_vic(
			hdmi_hpd_ctl.hdmi_pdev, write_burst_vic);
	}
}

/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0006 add start */
int mhl_charge_control(uint16_t max_current_in_milliamps)
{
	unsigned int type;

	pr_info("%s\n", __func__);
	pr_info("max current:%d\n", max_current_in_milliamps);

	if (max_current_in_milliamps == 2000)
		max_current_in_milliamps = 1500;

	if (charge_current != max_current_in_milliamps) {
		if (max_current_in_milliamps > 100) {
			pr_debug("start charging:%d\n", max_current_in_milliamps);
			/* Implement charge control:value*/
			switch(max_current_in_milliamps) {
			case 500:
				type = FJ_CHG_TYPE_A;
				break;
			case 900:
				type = FJ_CHG_TYPE_B;
				break;
			case 1500:
/* FCNT LIMITED:2016-02-29 H161_5_MHL_01_0015 mod start */
#if 0
				type = FJ_CHG_TYPE_C;
#else
				type = FJ_CHG_TYPE_B;
#endif
/* FCNT LIMITED:2016-02-29 H161_5_MHL_01_0015 mod end */
				break;
			default:
				pr_err("invalid milliamps! %d[mA]\n",max_current_in_milliamps);
				goto error;
			}
			if (mhl_charge_state == 0) {
				fj_chg_mhl_vbus_draw(FJ_CHG_ENABLE);
			}
			fj_chg_mhl_vbus_draw(type);
			mhl_charge_state = 1;
		} else {
			if (mhl_charge_state) {
				pr_err("stop charging\n");
				/* Implement charge control:stop */
				fj_chg_mhl_vbus_draw(FJ_CHG_DISABLE);
				mhl_charge_state = 0;
			}
		}
		charge_current = max_current_in_milliamps;
	}
error:
	return 0;
}
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0006 add end */

int mhl_is_connected(void)
{
	pr_info("%s state:%x\n", __func__, mhl_connect_state);
	return mhl_connect_state;
}

/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0004 add start */
#define GPIO_VBDET         3
#define VBDET_WAIT_MS      25000
#define VBDET_WAIT_CNT     30
#define VBDET_WAIT_CNT_MAX 300
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0004 add end */
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0003 add start */
int mhl_switch(bool onoff)
{
	int status = -EINVAL;
	struct mhl_dev_context *dev_context;

	pr_info("%s start:%d\n", __func__, onoff);

	dev_context = dev_get_drvdata(mhl_dev);
	if (!mhl_dev) {
		return status;
	}

	if (onoff) {
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0004 add start */
		int i;
		int value;
		int chatt_count = 1;
		for (i=0;i<VBDET_WAIT_CNT_MAX;i++) {
			value = gpio_get_value(qpnp_pin_map("pmi8994-gpio", GPIO_VBDET));
			if (value == 0) {
				chatt_count++;
			} else {
				chatt_count = 1;
			}
			if (chatt_count >= VBDET_WAIT_CNT) {
				break;
			}
			usleep(VBDET_WAIT_MS);
		}
		if (i >= VBDET_WAIT_CNT_MAX) {
			pr_err("%s VBUS not detect!\n", __func__);
			return status;
		}
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0004 add end */
		gpio_set_value(switch_sel_1_gpio, 1);
		gpio_set_value(switch_sel_2_gpio, 1);
		mhl_connect_state = 1;
		status = mhl_handle_power_change_request(mhl_dev,true);
		if (status != 0) {
			pr_err("%s si_8620_power_control faild \n", __func__);
			si_8620_power_control(false);
			mhl_connect_state = 0;
			gpio_set_value(switch_sel_1_gpio, 0);
			gpio_set_value(switch_sel_2_gpio, 0);
		}
	} else {
		status = si_mhl_tx_shutdown(dev_context);
		if (status != 0) {
			pr_err("%s si_mhl_tx_shutdown faild \n", __func__);
		}
		mhl_connect_state = 0;
		gpio_set_value(switch_sel_1_gpio, 0);
		gpio_set_value(switch_sel_2_gpio, 0);
	}

	msleep(200);
	
	pr_debug("switch_sel_1_gpio :%d\n",gpio_get_value(switch_sel_1_gpio));
	pr_debug("switch_sel_2_gpio :%d\n",gpio_get_value(switch_sel_2_gpio));
	pr_debug("FJ_TX_FW_WAKE     :%d\n",mhl_gpio_get_value(FJ_TX_FW_WAKE));
	pr_debug("FJ_TX_HW_RESET    :%d\n",mhl_gpio_get_value(FJ_TX_HW_RESET));
	
	pr_info("%s %x end\n", __func__,status);

	return status;
}
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0003 add end */
