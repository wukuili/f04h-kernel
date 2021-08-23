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

#define FJ_PMI_ADC_DBGLOG(x, y...)					\
	if (unlikely(fj_pmi_adc_debug != 0)){			\
		printk(KERN_ERR "[fj_pmi_adc] " x, ## y);	\
	}

#define FJ_PMI_ADC_INFOLOG(x, y...) 		printk(KERN_INFO "[fj_pmi_adc] " x, ## y)
#define FJ_PMI_ADC_WARNLOG(x, y...) 		printk(KERN_WARNING "[fj_pmi_adc] " x, ## y)
#define FJ_PMI_ADC_ERRLOG(x, y...)			printk(KERN_ERR "[fj_pmi_adc] " x, ## y)
#define FJ_PMI_ADC_RECLOG(x, y...)			printk("REC@REC@36[fj_pmi_adc] " x, ## y)

#define FJ_PMI_ADC_USBIN					USBIN                  /* 0x00 */
#define FJ_PMI_ADC_DCIN						DCIN                   /* 0x01 */
#define FJ_PMI_ADC_OKI_ADC					P_MUX1_1_1             /* 0x10 */

#define INITIALIZE_CHECK()													\
	if (unlikely(pmi_adc_drv_initialized == 0)) {							\
		FJ_PMI_ADC_DBGLOG("[%s] Chg-ADC driver uninitialize\n", __func__);	\
		return -1;															\
	}

struct pmi_adc_drv {
	struct device				*dev;
	struct spmi_device			*spmi;
	struct mutex				lock;
	struct qpnp_vadc_chip		*vadc_device;
	struct power_supply			fj_pmi_adc;
};

static struct pmi_adc_drv *the_chip = 0;
static int pmi_adc_drv_initialized = 0;

static enum power_supply_property fj_pmi_adc_properties[] = {
	POWER_SUPPLY_PROP_INITIALIZE,
	POWER_SUPPLY_PROP_VOLT_USBIN_CODE,
	POWER_SUPPLY_PROP_VOLT_DCIN_CODE,
	POWER_SUPPLY_PROP_OKI_ADC_CODE,
	POWER_SUPPLY_PROP_VOLT_USBIN_PHYS,
	POWER_SUPPLY_PROP_VOLT_DCIN_PHYS,
	POWER_SUPPLY_PROP_OKI_ADC_PHYS,
};

static int fj_pmi_adc_debug = 0;

/* internal function */
static int pmi_adc_drv_vadc_read_common(enum qpnp_vadc_channels channel, struct qpnp_vadc_result *result_adc)
{
	struct qpnp_vadc_result result_adc_work = {0};
	int result = 0;
	int i = 0;

	for (i = 0; i < 3; i++) {
		result = qpnp_vadc_read(the_chip->vadc_device, channel, &result_adc_work);
		if (result < 0) {
			FJ_PMI_ADC_ERRLOG("[%s] read err ADC ID:0x%x raw:0x%x physical:%lld\n", __func__,
							   channel, result_adc_work.adc_code, result_adc_work.physical);
		} else {
			result_adc->adc_code = result_adc_work.adc_code;
			result_adc->physical = result_adc_work.physical;
			break;
		}
	}
	return result;
}

static int fj_pmi_adc_drv_get_usbin_voltage(struct qpnp_vadc_result *result_adc)
{
	int result = 0;

	INITIALIZE_CHECK();
	mutex_lock(&the_chip->lock);
	result = pmi_adc_drv_vadc_read_common(FJ_PMI_ADC_USBIN, result_adc);
	mutex_unlock(&the_chip->lock);

	return result;
}

static int fj_pmi_adc_drv_get_dcin_voltage(struct qpnp_vadc_result *result_adc)
{
	int result = 0;

	INITIALIZE_CHECK();
	mutex_lock(&the_chip->lock);
	result = pmi_adc_drv_vadc_read_common(FJ_PMI_ADC_DCIN, result_adc);
	mutex_unlock(&the_chip->lock);

	return result;
}

static int fj_pmi_adc_drv_get_oki_adc(struct qpnp_vadc_result *result_adc)
{
	int result = 0;

	INITIALIZE_CHECK();
	mutex_lock(&the_chip->lock);
	result = pmi_adc_drv_vadc_read_common(FJ_PMI_ADC_OKI_ADC, result_adc);
	mutex_unlock(&the_chip->lock);

	return result;
}

static int fj_pmi_adc_get_property(struct power_supply *psy,
								   enum power_supply_property psp,
								   union power_supply_propval *val)
{
	struct qpnp_vadc_result result_adc = {0};
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_INITIALIZE:
			val->intval = pmi_adc_drv_initialized;
			break;

		case POWER_SUPPLY_PROP_VOLT_USBIN_CODE:
		case POWER_SUPPLY_PROP_VOLT_USBIN_PHYS:
			ret = fj_pmi_adc_drv_get_usbin_voltage(&result_adc);
			if (unlikely(ret != 0)) {
				FJ_PMI_ADC_DBGLOG("[%s] get error USBIN:%d\n", __func__, ret);
			}
			if (psp == POWER_SUPPLY_PROP_VOLT_USBIN_CODE) {
				val->intval = result_adc.adc_code;
			} else {
				val->intval = (int)result_adc.physical;
			}
			break;

		case POWER_SUPPLY_PROP_VOLT_DCIN_CODE:
		case POWER_SUPPLY_PROP_VOLT_DCIN_PHYS:
			ret = fj_pmi_adc_drv_get_dcin_voltage(&result_adc);
			if (unlikely(ret != 0)) {
				FJ_PMI_ADC_DBGLOG("[%s] get error DCIN:%d\n", __func__, ret);
			}
			if (psp == POWER_SUPPLY_PROP_VOLT_DCIN_CODE) {
				val->intval = result_adc.adc_code;
			} else {
				val->intval = (int)result_adc.physical;
			}
			break;

		case POWER_SUPPLY_PROP_OKI_ADC_CODE:
		case POWER_SUPPLY_PROP_OKI_ADC_PHYS:
			ret = fj_pmi_adc_drv_get_oki_adc(&result_adc);
			if (unlikely(ret != 0)) {
				FJ_PMI_ADC_DBGLOG("[%s] get error OKI_ADC:%d\n", __func__, ret);
			}
			if (psp == POWER_SUPPLY_PROP_OKI_ADC_CODE) {
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

static int set_pmi_adc_debug(const char *val, struct kernel_param *kp)
{
	int result = 0;

	result = param_set_int(val, kp);
	if (result) {
		FJ_PMI_ADC_ERRLOG("[%s] error setting value %d\n",__func__, result);
		return result;
	}

	if (result != 0) {
		fj_pmi_adc_debug = 0;
	}

	return result;
}
module_param_call(dbglog_adc, set_pmi_adc_debug, param_get_uint, &fj_pmi_adc_debug, 0644);

static int fj_pmi_adc_driver_probe(struct spmi_device *spmi)
{
	int ret = 0;
	struct pmi_adc_drv *chip;
	struct spmi_resource *spmi_resource;
	struct resource *resource;

	FJ_PMI_ADC_DBGLOG("[%s] in\n", __func__);

	chip = kzalloc(sizeof(struct pmi_adc_drv), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &(spmi->dev);
	chip->spmi = spmi;

	chip->fj_pmi_adc.name = FJ_PMI_ADC_PSY_NAME;
	chip->fj_pmi_adc.type = POWER_SUPPLY_TYPE_FJ_PMI_ADC;
	chip->fj_pmi_adc.get_property = fj_pmi_adc_get_property;
	chip->fj_pmi_adc.properties = fj_pmi_adc_properties;
	chip->fj_pmi_adc.num_properties = ARRAY_SIZE(fj_pmi_adc_properties);

	ret = power_supply_register(chip->dev, &chip->fj_pmi_adc);
	if (unlikely(ret < 0)) {
		FJ_PMI_ADC_ERRLOG("[%s] power_supply_register fj-pmi-adc failed ret = %d\n", __func__, ret);
		goto fj_pmi_adc_err1;
	}

	chip->vadc_device = qpnp_get_vadc(&spmi->dev, "fj_pmi_adc");
	if (IS_ERR(chip->vadc_device)) {
		ret = PTR_ERR(chip->vadc_device);
		if (ret != -EPROBE_DEFER)
			FJ_PMI_ADC_ERRLOG("[%s] vadc property missing, ret=%d\n",
						   __func__, ret);
		goto fj_pmi_adc_err2;
	}
	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			FJ_PMI_ADC_ERRLOG("[%s] spmi resource absent\n", __func__);
			ret = -ENXIO;
			goto fj_pmi_adc_err2;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			FJ_PMI_ADC_ERRLOG("[%s] node %s IO resource absent\n",
						   __func__, spmi->dev.of_node->full_name);
			ret = -ENXIO;
			goto fj_pmi_adc_err2;
		}
	}

	dev_set_drvdata(&spmi->dev, chip);
	device_init_wakeup(&spmi->dev, 1);

	mutex_init(&chip->lock);

	the_chip = chip;
	pmi_adc_drv_initialized = 1;

	FJ_PMI_ADC_INFOLOG("[%s] probe End\n", __func__);

	return 0;

fj_pmi_adc_err2:
	power_supply_unregister(&chip->fj_pmi_adc);
fj_pmi_adc_err1:
	kfree(chip);

	return ret;
}

static int fj_pmi_adc_driver_remove(struct spmi_device *spmi)
{
	struct pmi_adc_drv *chip = the_chip;

	dev_set_drvdata(&spmi->dev, NULL);
	power_supply_unregister(&chip->fj_pmi_adc);
	kfree(chip);

	return 0;
}

static struct of_device_id pmi_adc_match_table[] = {
	{ .compatible = "fj,fj_pmi_adc_drv",},
	{ },
};

static struct spmi_driver fj_pmi_adc_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "fj_pmi_adc_drv",
		   .of_match_table = pmi_adc_match_table,
		   },
	.probe = fj_pmi_adc_driver_probe,
	.remove = fj_pmi_adc_driver_remove,
};

static int __init fj_pmi_adc_driver_init(void)
{
	return spmi_driver_register(&fj_pmi_adc_driver);
}
module_init(fj_pmi_adc_driver_init);

static void __exit fj_pmi_adc_driver_exit(void)
{
	spmi_driver_unregister(&fj_pmi_adc_driver);
}
module_exit(fj_pmi_adc_driver_exit);

MODULE_AUTHOR("Fujitsu");
MODULE_DESCRIPTION("Charger PMI ADC Driver");
MODULE_LICENSE("GPL");
