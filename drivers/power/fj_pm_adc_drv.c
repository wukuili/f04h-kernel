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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spmi.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/qpnp/pin.h>
#include <linux/fj_mode.h>
#include <linux/mfd/fj_charger.h>
#include "fj_charger_local.h"

#define FJ_PM_ADC_DBGLOG(x, y...)					\
	if (unlikely(fj_pm_adc_debug != 0)){			\
		printk(KERN_ERR "[fj_pm_adc] " x, ## y);	\
	}

#define FJ_PM_ADC_INFOLOG(x, y...) 			printk(KERN_INFO "[fj_pm_adc] " x, ## y)
#define FJ_PM_ADC_WARNLOG(x, y...) 			printk(KERN_WARNING "[fj_pm_adc] " x, ## y)
#define FJ_PM_ADC_ERRLOG(x, y...)			printk(KERN_ERR "[fj_pm_adc] " x, ## y)
#define FJ_PM_ADC_RECLOG(x, y...)			printk("REC@REC@36[fj_pm_adc] " x, ## y)

#define FJ_PM_ADC_MSM_THERM					LR_MUX4_PU1_AMUX_THM1  /* 0x73 */
#define FJ_PM_ADC_TEMP_CHG 					LR_MUX6_AMUX_THM3      /* 0x35 */
#define FJ_PM_ADC_TEMP_CASE					LR_MUX9_AMUX_THM5      /* 0x38 */
#define FJ_PM_ADC_TEMP_CENTER				LR_MUX5_AMUX_THM2      /* 0x34 */
#define FJ_PM_ADC_TEMP_IRIS					LR_MUX8_AMUX_THM4      /* 0x37 */

#define INITIALIZE_CHECK()													\
	if (unlikely(pm_adc_drv_initialized == 0)) {							\
		FJ_PM_ADC_DBGLOG("[%s] Chg-ADC driver uninitialize\n", __func__);	\
		return -1;															\
	}

/* related GPIOs */
#define FJ_TEMP_FETON_GPIO					(2)

struct pm_adc_drv {
	struct device				*dev;
	struct spmi_device			*spmi;
	struct mutex				lock;
	struct qpnp_vadc_chip		*vadc_device;
	struct power_supply			fj_pm_adc;
};

static struct pm_adc_drv *the_chip = 0;
static int pm_adc_drv_initialized = 0;

static enum power_supply_property fj_pm_adc_properties[] = {
	POWER_SUPPLY_PROP_INITIALIZE,
	POWER_SUPPLY_PROP_TEMP_AMBIENT_CODE,
	POWER_SUPPLY_PROP_TEMP_CASE_CODE,
	POWER_SUPPLY_PROP_TEMP_CENTER_CODE,
	POWER_SUPPLY_PROP_TEMP_CHARGE_CODE,
	POWER_SUPPLY_PROP_TEMP_IRIS_CODE,
	POWER_SUPPLY_PROP_TEMP_AMBIENT_PHYS,
	POWER_SUPPLY_PROP_TEMP_CASE_PHYS,
	POWER_SUPPLY_PROP_TEMP_CENTER_PHYS,
	POWER_SUPPLY_PROP_TEMP_CHARGE_PHYS,
	POWER_SUPPLY_PROP_TEMP_IRIS_PHYS,
};

static int fj_pm_adc_debug = 0;

/* internal function */
static int pm_adc_drv_vadc_read_common(enum qpnp_vadc_channels channel, struct qpnp_vadc_result *result_adc)
{
	struct qpnp_vadc_result result_adc_work = {0};
	int result = 0;
	int i = 0;

	for (i = 0; i < 3; i++) {
		result = qpnp_vadc_read(the_chip->vadc_device, channel, &result_adc_work);
		if (result < 0) {
			FJ_PM_ADC_ERRLOG("[%s] read err ADC ID:0x%x raw:0x%x physical:%lld\n", __func__,
							   channel, result_adc_work.adc_code, result_adc_work.physical);
		} else {
			result_adc->adc_code = result_adc_work.adc_code;
			result_adc->physical = result_adc_work.physical;
			break;
		}
	}
	return result;
}

static int fj_pm_adc_drv_get_charge_temp(struct qpnp_vadc_result *result_adc)
{
	int result = 0;

	INITIALIZE_CHECK();
	mutex_lock(&the_chip->lock);
	gpio_set_value(qpnp_pin_map("pmi8994-gpio", FJ_TEMP_FETON_GPIO),0);
	mdelay(2);
	result = pm_adc_drv_vadc_read_common(FJ_PM_ADC_TEMP_CHG, result_adc);
	gpio_set_value(qpnp_pin_map("pmi8994-gpio", FJ_TEMP_FETON_GPIO),1);
	mutex_unlock(&the_chip->lock);

	return result;
}

static int fj_pm_adc_drv_get_msm_therm(struct qpnp_vadc_result *result_adc)
{
	int result = 0;

	INITIALIZE_CHECK();
	mutex_lock(&the_chip->lock);
	result = pm_adc_drv_vadc_read_common(FJ_PM_ADC_MSM_THERM, result_adc);
	mutex_unlock(&the_chip->lock);

	return result;
}

static int fj_pm_adc_drv_get_case_temp(struct qpnp_vadc_result *result_adc)
{
	int result = 0;

	INITIALIZE_CHECK();
	mutex_lock(&the_chip->lock);
	gpio_set_value(qpnp_pin_map("pmi8994-gpio", FJ_TEMP_FETON_GPIO),0);
	mdelay(2);
	result = pm_adc_drv_vadc_read_common(FJ_PM_ADC_TEMP_CASE, result_adc);
	gpio_set_value(qpnp_pin_map("pmi8994-gpio", FJ_TEMP_FETON_GPIO),1);
	mutex_unlock(&the_chip->lock);

	return result;
}

static int fj_pm_adc_drv_get_center_temp(struct qpnp_vadc_result *result_adc)
{
	int result = 0;

	INITIALIZE_CHECK();
	mutex_lock(&the_chip->lock);
	gpio_set_value(qpnp_pin_map("pmi8994-gpio", FJ_TEMP_FETON_GPIO),0);
	mdelay(2);
	result = pm_adc_drv_vadc_read_common(FJ_PM_ADC_TEMP_CENTER, result_adc);
	gpio_set_value(qpnp_pin_map("pmi8994-gpio", FJ_TEMP_FETON_GPIO),1);
	mutex_unlock(&the_chip->lock);

	return result;
}

static int fj_pm_adc_drv_get_iris_temp(struct qpnp_vadc_result *result_adc)
{
	int result = 0;

	INITIALIZE_CHECK();
	mutex_lock(&the_chip->lock);
	gpio_set_value(qpnp_pin_map("pmi8994-gpio", FJ_TEMP_FETON_GPIO),0);
	mdelay(2);
	result = pm_adc_drv_vadc_read_common(FJ_PM_ADC_TEMP_IRIS, result_adc);
	gpio_set_value(qpnp_pin_map("pmi8994-gpio", FJ_TEMP_FETON_GPIO),1);
	mutex_unlock(&the_chip->lock);

	return result;
}

static int fj_pm_adc_get_property(struct power_supply *psy,
								  enum power_supply_property psp,
								  union power_supply_propval *val)
{
	struct qpnp_vadc_result result_adc = {0};
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_INITIALIZE:
			val->intval = pm_adc_drv_initialized;
			break;

		case POWER_SUPPLY_PROP_TEMP_AMBIENT_CODE:
		case POWER_SUPPLY_PROP_TEMP_AMBIENT_PHYS:
			ret = fj_pm_adc_drv_get_msm_therm(&result_adc);
			if (unlikely(ret != 0)) {
				FJ_PM_ADC_DBGLOG("[%s] get error TEMP_AMBIENT:%d\n", __func__, ret);
			}
			if (psp == POWER_SUPPLY_PROP_TEMP_AMBIENT_CODE) {
				val->intval = result_adc.adc_code;
			} else {
				val->intval = (int)result_adc.physical;
			}
			break;

		case POWER_SUPPLY_PROP_TEMP_CASE_CODE:
		case POWER_SUPPLY_PROP_TEMP_CASE_PHYS:
			ret = fj_pm_adc_drv_get_case_temp(&result_adc);
			if (unlikely(ret != 0)) {
				FJ_PM_ADC_DBGLOG("[%s] get error TEMP_CASE:%d\n", __func__, ret);
			}
			if (psp == POWER_SUPPLY_PROP_TEMP_CASE_CODE) {
				val->intval = result_adc.adc_code;
			} else {
				val->intval = (int)result_adc.physical;
			}
			break;

		case POWER_SUPPLY_PROP_TEMP_CENTER_CODE:
		case POWER_SUPPLY_PROP_TEMP_CENTER_PHYS:
			ret = fj_pm_adc_drv_get_center_temp(&result_adc);
			if (unlikely(ret != 0)) {
				FJ_PM_ADC_DBGLOG("[%s] get error TEMP_CENTER:%d\n", __func__, ret);
			}
			if (psp == POWER_SUPPLY_PROP_TEMP_CENTER_CODE) {
				val->intval = result_adc.adc_code;
			} else {
				val->intval = (int)result_adc.physical;
			}
			break;

		case POWER_SUPPLY_PROP_TEMP_CHARGE_CODE:
		case POWER_SUPPLY_PROP_TEMP_CHARGE_PHYS:
			ret = fj_pm_adc_drv_get_charge_temp(&result_adc);
			if (unlikely(ret != 0)) {
				FJ_PM_ADC_DBGLOG("[%s] get error TEMP_CHARGE:%d\n", __func__, ret);
			}
			if (psp == POWER_SUPPLY_PROP_TEMP_CHARGE_CODE) {
				val->intval = result_adc.adc_code;
			} else {
				val->intval = (int)result_adc.physical;
			}
			break;

		case POWER_SUPPLY_PROP_TEMP_IRIS_CODE:
		case POWER_SUPPLY_PROP_TEMP_IRIS_PHYS:
			ret = fj_pm_adc_drv_get_iris_temp(&result_adc);
			if (unlikely(ret != 0)) {
				FJ_PM_ADC_DBGLOG("[%s] get error TEMP_IRIS:%d\n", __func__, ret);
			}
			if (psp == POWER_SUPPLY_PROP_TEMP_IRIS_CODE) {
				val->intval = result_adc.adc_code;
			} else {
				val->intval = (int)result_adc.physical;
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int set_pm_adc_debug(const char *val, struct kernel_param *kp)
{
	int result = 0;

	result = param_set_int(val, kp);
	if (result) {
		FJ_PM_ADC_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	if (result != 0) {
		fj_pm_adc_debug = 0;
	}

	return result;
}
module_param_call(dbglog_adc, set_pm_adc_debug, param_get_uint, &fj_pm_adc_debug, 0644);

static int fj_pm_adc_driver_probe(struct spmi_device *spmi)
{
	int ret = 0;
	struct pm_adc_drv *chip;
	struct spmi_resource *spmi_resource;
	struct resource *resource;

	FJ_PM_ADC_DBGLOG("[%s] in\n", __func__);

	chip = kzalloc(sizeof(struct pm_adc_drv), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &(spmi->dev);
	chip->spmi = spmi;

	chip->fj_pm_adc.name = FJ_PM_ADC_PSY_NAME;
	chip->fj_pm_adc.type = POWER_SUPPLY_TYPE_FJ_PM_ADC;
	chip->fj_pm_adc.get_property = fj_pm_adc_get_property;
	chip->fj_pm_adc.properties = fj_pm_adc_properties;
	chip->fj_pm_adc.num_properties = ARRAY_SIZE(fj_pm_adc_properties);

	ret = power_supply_register(chip->dev, &chip->fj_pm_adc);
	if (unlikely(ret < 0)) {
		FJ_PM_ADC_ERRLOG("[%s] power_supply_register fj-pm-adc failed ret = %d\n", __func__, ret);
		goto fj_pm_adc_err1;
	}

	chip->vadc_device = qpnp_get_vadc(&spmi->dev, "fj_pm_adc");
	if (IS_ERR(chip->vadc_device)) {
		ret = PTR_ERR(chip->vadc_device);
		if (ret != -EPROBE_DEFER)
			FJ_PM_ADC_ERRLOG("[%s] vadc property missing, ret=%d\n",
						   __func__, ret);
		goto fj_pm_adc_err2;
	}

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			FJ_PM_ADC_ERRLOG("[%s] spmi resource absent\n", __func__);
			ret = -ENXIO;
			goto fj_pm_adc_err2;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			FJ_PM_ADC_ERRLOG("[%s] node %s IO resource absent\n",
						   __func__, spmi->dev.of_node->full_name);
			ret = -ENXIO;
			goto fj_pm_adc_err2;
		}
	}

	dev_set_drvdata(&spmi->dev, chip);
	device_init_wakeup(&spmi->dev, 1);

	mutex_init(&chip->lock);

	the_chip = chip;
	pm_adc_drv_initialized = 1;

	FJ_PM_ADC_INFOLOG("[%s] probe End\n", __func__);

	return 0;

fj_pm_adc_err2:
	power_supply_unregister(&chip->fj_pm_adc);
fj_pm_adc_err1:
	kfree(chip);

	return ret;
}

static int fj_pm_adc_driver_remove(struct spmi_device *spmi)
{
	struct pm_adc_drv *chip = the_chip;

	dev_set_drvdata(&spmi->dev, NULL);
	power_supply_unregister(&chip->fj_pm_adc);
	kfree(chip);

	return 0;
}

static struct of_device_id pm_adc_match_table[] = {
	{ .compatible = "fj,fj_pm_adc_drv",},
	{ },
};

static struct spmi_driver fj_pm_adc_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "fj_pm_adc_drv",
		   .of_match_table = pm_adc_match_table,
		   },
	.probe = fj_pm_adc_driver_probe,
	.remove = fj_pm_adc_driver_remove,
};

static int __init fj_pm_adc_driver_init(void)
{
	return spmi_driver_register(&fj_pm_adc_driver);
}
module_init(fj_pm_adc_driver_init);

static void __exit fj_pm_adc_driver_exit(void)
{
	spmi_driver_unregister(&fj_pm_adc_driver);
}
module_exit(fj_pm_adc_driver_exit);

MODULE_AUTHOR("Fujitsu");
MODULE_DESCRIPTION("Charger PM ADC Driver");
MODULE_LICENSE("GPL");
