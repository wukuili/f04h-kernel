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
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/nonvolatile_common.h>
#include <linux/slab.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/fj_mode.h>
#include <linux/wait.h>

#include <linux/mfd/fj_charger.h>
#include "fj_charger_local.h"

#define FJ_TEMP_DBGLOG(x, y...)                             \
    if (unlikely(fj_temp_drv_debug != 0)) {                 \
        printk(KERN_ERR "[fj_temp_drv] " x, ## y);          \
    }
#define FJ_TEMP_INFOLOG(x, y...)        printk(KERN_INFO "[fj_temp_drv] " x, ## y)
#define FJ_TEMP_WARNLOG(x, y...)        printk(KERN_WARNING "[fj_temp_drv] " x, ## y)
#define FJ_TEMP_ERRLOG(x, y...)         printk(KERN_ERR "[fj_temp_drv] " x, ## y)
#define FJ_TEMP_RECLOG(x, y...)         printk("REC@REC@36[fj_temp_drv] " x, ## y)

/* monitor time */
#define FJ_TEMP_DRV_MONITOR_DELAY_100MS     msecs_to_jiffies(100)
#define FJ_TEMP_DRV_MONITOR_DELAY_300MS     msecs_to_jiffies(300)
#define FJ_TEMP_DRV_MONITOR_DELAY_500MS     msecs_to_jiffies(500)
#define FJ_TEMP_DRV_MONITOR_DELAY_1S        msecs_to_jiffies(1000)
#define FJ_TEMP_DRV_MONITOR_DELAY_10S       msecs_to_jiffies(10000)
#define FJ_TEMP_DRV_MONITOR_DELAY_60S       msecs_to_jiffies(60000)

/* fj_temp_drv thread events */
#define FJ_TEMP_DRV_EVENT_SUSPEND               0
#define FJ_TEMP_DRV_EVENT_MT_MONITOR            1

#define FJ_TEMP_DRV_GET_COUNT                   4
#define FJ_TEMP_DRV_INTERVAL_MS                 25

#define FJ_TEMP_DRV_GET_COUNT_IRIS              3

typedef enum {
    FJ_TEMP_DRV_TYPE_AMBIENT = 0,
    FJ_TEMP_DRV_TYPE_CASE,
    FJ_TEMP_DRV_TYPE_CHARGE,
    FJ_TEMP_DRV_TYPE_CENTER,
    FJ_TEMP_DRV_TYPE_NUM,
} FJ_TEMP_DRV_TYPE;

enum fj_temp_drv_func_limit {
    FJ_TEMP_DRV_FUNC_LIMIT_CHARGE = 0,         /*  0 */
    FJ_TEMP_DRV_FUNC_LIMIT_BATTERY_PRESENT,    /*  1 */
    FJ_TEMP_DRV_FUNC_LIMIT_LOW_BATTERY,        /*  2 */
    FJ_TEMP_DRV_FUNC_LIMIT_RESERVE_01,         /*  3 */
    FJ_TEMP_DRV_FUNC_LIMIT_BATTERY_TEMP,       /*  4 */
    FJ_TEMP_DRV_FUNC_LIMIT_TERMINAL_TEMP,      /*  5 */
    FJ_TEMP_DRV_FUNC_LIMIT_RECEIVER_TEMP,      /*  6 */
    FJ_TEMP_DRV_FUNC_LIMIT_CHARGE_TEMP,        /*  7 */
    FJ_TEMP_DRV_FUNC_LIMIT_CENTER_TEMP,        /*  8 */
    FJ_TEMP_DRV_FUNC_LIMIT_IRIS_TEMP,          /*  9 */
    FJ_TEMP_DRV_FUNC_LIMIT_NUM,
};

struct temp_drv_chip {
    struct power_supply fj_temp;
    
    struct power_supply *bms;
    struct power_supply *fj_pm_adc;
    
    struct workqueue_struct *fj_temp_drv_wq;
    struct delayed_work mt_monitor;
    
    struct wake_lock wakelock;
    
    /* terminal temp */
    int temp_ambient;
    /* receiver temp */
    int temp_case;
    /* charger temp */
    int temp_charge;
    /* center temp */
    int temp_center;
    
    /* monitor event state */
    unsigned long event_state;
    
    bool check_disable[FJ_TEMP_DRV_FUNC_LIMIT_NUM];
    
    wait_queue_head_t wq;
    void* thread;
};

struct temp_drv_data {
    int temp_max;
    int temp_min;
    int temp_num;
    int temp_count;
};

static struct temp_drv_chip *the_chip = NULL;

static enum power_supply_property fj_temp_properties[] = {
    POWER_SUPPLY_PROP_TEMP_BATT,
    POWER_SUPPLY_PROP_TEMP_AMBIENT,
    POWER_SUPPLY_PROP_TEMP_CASE,
    POWER_SUPPLY_PROP_TEMP_CHARGE,
    POWER_SUPPLY_PROP_TEMP_CENTER,
    POWER_SUPPLY_PROP_TEMP_IRIS,
};

static int fj_temp_drv_debug = 0;
static int set_temp_drv_debug(const char *val, struct kernel_param *kp)
{
    int result = 0;
    
    result = param_set_int(val, kp);
    if (result) {
        FJ_TEMP_ERRLOG("[%s] error setting value %d\n",__func__, result);
        return result;
    }
    
    if (result != 0) {
        fj_temp_drv_debug = 0;
    }
    
    return result;
}
module_param_call(dbglog_temp, set_temp_drv_debug, param_get_uint, &fj_temp_drv_debug, 0644);

static void fj_temp_drv_function_limits_check(struct temp_drv_chip *chip)
{
    int result = 0;
    u16 val = 0;
    int i = 0;
    
    FJ_TEMP_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(chip == NULL)) {
        FJ_TEMP_WARNLOG("[%s] chip pointer is NULL\n", __func__);
        if (unlikely(the_chip == NULL)) {
            FJ_TEMP_ERRLOG("[%s] the_chip pointer is NULL\n", __func__);
            return;
        }
        chip = the_chip;
    }
    
    result = get_nonvolatile((uint8_t*)&val, APNV_CHARGE_FG_FUNC_LIMITS_I, 2);
    if (unlikely(result < 0)) {
        val = 0x0000;
        FJ_TEMP_ERRLOG("[%s] NV read err : %d set value = 0x%x \n", __func__, result, val);
    }
    
    /*
     * Limits function classification
     * check_disable[0] : charge
     * check_disable[1] : battery present
     * check_disable[2] : low battery
     * check_disable[3] : reserve
     * check_disable[4] : battery temperature
     * check_disable[5] : terminal temperature
     * check_disable[6] : receiver temperature
     * check_disable[7] : charge temperature
     * check_disable[8] : center temperature
     * check_disable[9] : iris temperature
     */
     
    if (unlikely(val != 0x0000)) {
        for (i = 0;i < FJ_TEMP_DRV_FUNC_LIMIT_NUM;i++) {
            if (val & 0x0001) {
                chip->check_disable[i] = true;
            } else {
                chip->check_disable[i] = false;
            }
            val = val >> 1;
            FJ_TEMP_DBGLOG("[%s] check_disable[%d] = %d\n", __func__, i, chip->check_disable[i]);
        }
    }
}

static void fj_temp_drv_get_temperature(struct temp_drv_chip* chip,
                                     FJ_TEMP_DRV_TYPE type,
                                     struct temp_drv_data *work_temp)
{
    int ret = 0;
    int value;
    union power_supply_propval prop = {0,};
    
    if (unlikely(work_temp == NULL)) {
        FJ_TEMP_WARNLOG("[%s] work_temp pointer is NULL\n", __func__);
        goto fj_temp_exit;
    }
    
    switch (type) {
        case FJ_TEMP_DRV_TYPE_AMBIENT:
            ret = chip->fj_pm_adc->get_property(chip->fj_pm_adc, POWER_SUPPLY_PROP_TEMP_AMBIENT_PHYS, &prop);
            break;
        case FJ_TEMP_DRV_TYPE_CASE:
            ret = chip->fj_pm_adc->get_property(chip->fj_pm_adc, POWER_SUPPLY_PROP_TEMP_CASE_PHYS, &prop);
            break;
        case FJ_TEMP_DRV_TYPE_CHARGE:
            ret = chip->fj_pm_adc->get_property(chip->fj_pm_adc, POWER_SUPPLY_PROP_TEMP_CENTER_PHYS, &prop);
            break;
        case FJ_TEMP_DRV_TYPE_CENTER:
            ret = chip->fj_pm_adc->get_property(chip->fj_pm_adc, POWER_SUPPLY_PROP_TEMP_CHARGE_PHYS, &prop);
            break;
        default:
            ret = -1;
            break;
    }
    
    if (unlikely(ret < 0)) {
        FJ_TEMP_ERRLOG("[%s] read err ADC type:%d value:%d\n",
                       __func__, type, prop.intval);
        value = 250;
    } else {
        value = prop.intval;
    }
    if (work_temp->temp_count == 0) {
        work_temp->temp_max = value;
        work_temp->temp_min = value;
    }
    if (value > work_temp->temp_max) {
        work_temp->temp_max = value;
    }
    if (value < work_temp->temp_min) {
        work_temp->temp_min = value;
    }
    work_temp->temp_num += value;
    work_temp->temp_count++;
    
    FJ_TEMP_DBGLOG("[%s] type:%d get:%d max:%d min:%d nom:%d cnt:%d\n", __func__, 
                         type, value,
                         work_temp->temp_max,
                         work_temp->temp_min,
                         work_temp->temp_num,
                         work_temp->temp_count);
    
fj_temp_exit:
    return;
}

static void fj_temp_drv_averaging_temperature(struct temp_drv_data *work_temp, int *result)
{
    if (unlikely(work_temp == NULL)) {
        FJ_TEMP_WARNLOG("[%s] work_temp pointer is NULL\n", __func__);
        goto fj_temp_exit;
    }
    
    if (unlikely(result == NULL)) {
        FJ_TEMP_WARNLOG("[%s] result pointer is NULL\n", __func__);
        goto fj_temp_exit;
    }
    
    if (unlikely(work_temp->temp_count == 0)) {
        FJ_TEMP_WARNLOG("[%s] Bad arguments:count\n", __func__);
        goto fj_temp_exit;
    }
    
    if (work_temp->temp_count > 2) {
        work_temp->temp_num -= work_temp->temp_max;
        work_temp->temp_count--;
        work_temp->temp_num -= work_temp->temp_min;
        work_temp->temp_count--;
    }
    
    *result = work_temp->temp_num / work_temp->temp_count;
    
fj_temp_exit:
    return;
}

static void fj_temp_drv_update_temperature(struct temp_drv_chip *chip, int count, unsigned int interval)
{
    int i = 0;
    struct temp_drv_data work_ambient;
    struct temp_drv_data work_case;
    struct temp_drv_data work_charge;
    struct temp_drv_data work_center;
    
    if (unlikely(chip == NULL)) {
        FJ_TEMP_WARNLOG("[%s] chip pointer is NULL\n", __func__);
        goto fj_temp_exit;
    }
    
    if (unlikely(count <= 0)) {
        FJ_TEMP_WARNLOG("[%s] Bad arguments:count\n", __func__);
        goto fj_temp_exit;
    }
    
    memset(&work_ambient, 0, sizeof(work_ambient));
    memset(&work_case, 0, sizeof(work_case));
    memset(&work_charge, 0, sizeof(work_charge));
    memset(&work_center, 0, sizeof(work_center));
    
    for (i = 0;i < count;i++) {
        if (i != 0) {
            msleep(interval);
        }
        fj_temp_drv_get_temperature(chip, FJ_TEMP_DRV_TYPE_AMBIENT, &work_ambient);
        fj_temp_drv_get_temperature(chip, FJ_TEMP_DRV_TYPE_CASE, &work_case);
        fj_temp_drv_get_temperature(chip, FJ_TEMP_DRV_TYPE_CHARGE, &work_charge);
        fj_temp_drv_get_temperature(chip, FJ_TEMP_DRV_TYPE_CENTER, &work_center);
    } 
    
    fj_temp_drv_averaging_temperature(&work_ambient, &chip->temp_ambient);
    fj_temp_drv_averaging_temperature(&work_case, &chip->temp_case);
    fj_temp_drv_averaging_temperature(&work_charge, &chip->temp_charge);
    fj_temp_drv_averaging_temperature(&work_center, &chip->temp_center);
    
fj_temp_exit:
    return;
}

static int fj_temp_drv_prop_temp_batt(struct temp_drv_chip *chip)
{
    int result = 0;
    union power_supply_propval prop = {0,};
    
    if (chip->check_disable[FJ_TEMP_DRV_FUNC_LIMIT_BATTERY_TEMP] == true) {
        result = 250;
    } else {
        chip->bms->get_property(chip->bms, POWER_SUPPLY_PROP_TEMP, &prop);
        result = prop.intval;
    }
    
    return result;
}

static int fj_temp_drv_prop_temp_ambient(struct temp_drv_chip *chip)
{
    int result = 0;
    
    if (chip->check_disable[FJ_TEMP_DRV_FUNC_LIMIT_TERMINAL_TEMP] == true) {
        result = 250;
    } else {
        result = chip->temp_ambient;
    }
    
    return result;
}

static int fj_temp_drv_prop_temp_case(struct temp_drv_chip *chip)
{
    int result = 0;
    
    if (chip->check_disable[FJ_TEMP_DRV_FUNC_LIMIT_RECEIVER_TEMP] == true) {
        result = 250;
    } else {
        result = chip->temp_case;
    }
    
    return result;
}

static int fj_temp_drv_prop_temp_charge(struct temp_drv_chip *chip)
{
    int result = 0;
    
    if (chip->check_disable[FJ_TEMP_DRV_FUNC_LIMIT_CHARGE_TEMP] == true) {
        result = 250;
    } else {
        result = chip->temp_charge;
    }
    
    return result;
}

static int fj_temp_drv_prop_temp_center(struct temp_drv_chip *chip)
{
    int result = 0;
    
    if (chip->check_disable[FJ_TEMP_DRV_FUNC_LIMIT_CENTER_TEMP] == true) {
        result = 250;
    } else {
        result = chip->temp_center;
    }
    
    return result;
}

static int fj_temp_drv_prop_temp_iris(struct temp_drv_chip *chip)
{
    int result = 0;
    int i = 0;
    union power_supply_propval prop = {0,};
    
    if (chip->check_disable[FJ_TEMP_DRV_FUNC_LIMIT_IRIS_TEMP] == true) {
        result = 250;
    } else {
        for (i = 0;i < FJ_TEMP_DRV_GET_COUNT_IRIS;i++) {
            if (i != 0) {
                udelay(1000);
            }
            chip->fj_pm_adc->get_property(chip->fj_pm_adc, POWER_SUPPLY_PROP_TEMP_IRIS_PHYS, &prop);
            result += prop.intval;
        }
        result /= i;
    }
    
    return result;
}

static int fj_temp_drv_get_property(struct power_supply *psy,
                                    enum power_supply_property psp,
                                    union power_supply_propval *val)
{
    struct temp_drv_chip *chip;
    
    if (unlikely(psy == NULL)) {
        FJ_TEMP_ERRLOG("[%s] psy pointer is NULL\n", __func__);
        return -EINVAL;
    }
    
    FJ_TEMP_DBGLOG("[%s] in psp:%d\n", __func__, psp);
    
    chip = container_of(psy, struct temp_drv_chip, fj_temp);
    
    switch (psp) {
        case POWER_SUPPLY_PROP_TEMP_BATT:
            val->intval = fj_temp_drv_prop_temp_batt(chip);
            break;
            
        case POWER_SUPPLY_PROP_TEMP_AMBIENT:
            val->intval = fj_temp_drv_prop_temp_ambient(chip);
            break;
            
        case POWER_SUPPLY_PROP_TEMP_CASE:
            val->intval = fj_temp_drv_prop_temp_case(chip);
            break;
            
        case POWER_SUPPLY_PROP_TEMP_CHARGE:
            val->intval = fj_temp_drv_prop_temp_charge(chip);
            break;
            
        case POWER_SUPPLY_PROP_TEMP_CENTER:
            val->intval = fj_temp_drv_prop_temp_center(chip);
            break;
            
        case POWER_SUPPLY_PROP_TEMP_IRIS:
            val->intval = fj_temp_drv_prop_temp_iris(chip);
            break;
            
        default:
            return -EINVAL;
    }
    
    return 0;
}

static void fj_temp_drv_event_temp_monitor(struct work_struct *work)
{
    struct delayed_work *dwork;
    struct temp_drv_chip *chip;
    
    FJ_TEMP_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(work == NULL)) {
        FJ_TEMP_ERRLOG("[%s] work pointer is NULL\n", __func__);
        return;
    }
    
    dwork = to_delayed_work(work);
    if (unlikely(dwork == NULL)) {
        FJ_TEMP_ERRLOG("[%s] dwork pointer is NULL\n", __func__);
        return;
    }
    
    chip = container_of(dwork, struct temp_drv_chip, mt_monitor);
    if (unlikely(chip == NULL)) {
        FJ_TEMP_ERRLOG("[%s] chip pointer is NULL\n", __func__);
        return;
    }
    
    set_bit(FJ_TEMP_DRV_EVENT_MT_MONITOR, &chip->event_state);
    wake_up(&chip->wq);
}

static void fj_temp_drv_work_temp_monitor(struct temp_drv_chip *chip)
{
    FJ_TEMP_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(chip == NULL)) {
        FJ_TEMP_WARNLOG("[%s] chip pointer is NULL\n", __func__);
        if (unlikely(the_chip == NULL)) {
            FJ_TEMP_ERRLOG("[%s] the_chip pointer is NULL\n", __func__);
            return;
        }
        chip = the_chip;
    }
    
    if ((fj_boot_mode == FJ_MODE_MAKER_MODE) || (fj_boot_mode == FJ_MODE_KERNEL_MODE)) {
        return;
    }
    
    wake_lock(&chip->wakelock);
    
    fj_temp_drv_update_temperature(chip, FJ_TEMP_DRV_GET_COUNT, FJ_TEMP_DRV_INTERVAL_MS);
    
    FJ_TEMP_DBGLOG("[%s] chip->temp_ambient = %d\n", __func__, chip->temp_ambient);
    FJ_TEMP_DBGLOG("[%s] chip->temp_case = %d\n", __func__, chip->temp_case);
    FJ_TEMP_DBGLOG("[%s] chip->temp_charge = %d\n", __func__, chip->temp_charge);
    FJ_TEMP_DBGLOG("[%s] chip->temp_center = %d\n", __func__, chip->temp_center);
    
    wake_unlock(&chip->wakelock);
    
    queue_delayed_work(chip->fj_temp_drv_wq, &chip->mt_monitor, FJ_TEMP_DRV_MONITOR_DELAY_10S);
    
    return;
}

static int fj_temp_drv_thread(void * ptr)
{
    struct temp_drv_chip *chip = ptr;
    int result = 0;
    
    FJ_TEMP_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(chip == NULL)) {
        FJ_TEMP_WARNLOG("[%s] chip pointer is NULL\n", __func__);
        if (unlikely(the_chip == NULL)) {
            FJ_TEMP_ERRLOG("[%s] the_chip pointer is NULL\n", __func__);
            return -1;
        }
        chip = the_chip;
    }
    
    fj_temp_drv_function_limits_check(chip);
    
    if ((fj_boot_mode != FJ_MODE_MAKER_MODE) && (fj_boot_mode != FJ_MODE_KERNEL_MODE)) {
        set_bit(FJ_TEMP_DRV_EVENT_MT_MONITOR, &chip->event_state);
    }
    set_freezable();
    
    do {
        wait_event_freezable(chip->wq, (chip->event_state || kthread_should_stop()));
        
        if (kthread_should_stop()) {
            FJ_TEMP_INFOLOG("[%s] while loop break\n", __func__);
            break;
        }
        
        if (test_bit(FJ_TEMP_DRV_EVENT_SUSPEND, &chip->event_state)) {
            FJ_TEMP_DBGLOG("[%s] event:0x%08x\n", __func__, (int)chip->event_state);
            test_and_clear_bit(FJ_TEMP_DRV_EVENT_MT_MONITOR, &chip->event_state);
        }
        
        if (test_and_clear_bit(FJ_TEMP_DRV_EVENT_MT_MONITOR, &chip->event_state)) {
            FJ_TEMP_DBGLOG("[%s] event %s\n", __func__, "MT_MONITOR");
            fj_temp_drv_work_temp_monitor(chip);
        }
    } while(1);
    
    return result;
}

static int fj_temp_drv_probe(struct platform_device *pdev)
{
    struct temp_drv_chip *chip;
    int ret = 0;
    
    FJ_TEMP_DBGLOG("[%s] in\n", __func__);
    
    chip = kzalloc(sizeof(struct temp_drv_chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;
    
    INIT_DEFERRABLE_WORK(&chip->mt_monitor, fj_temp_drv_event_temp_monitor);
    chip->fj_temp_drv_wq = create_singlethread_workqueue(dev_name(&pdev->dev));
    if (!chip->fj_temp_drv_wq) {
        FJ_TEMP_ERRLOG("[%s] create_singlethread_workqueue failed\n", __func__);
        ret = -ESRCH;
        goto fj_temp_err1;
    }
    
    chip->bms = power_supply_get_by_name(QPNP_BMS_PSY_NAME);
    if (chip->bms == NULL) {
        FJ_TEMP_ERRLOG("[%s] power_supply_get_by_name bms failed\n", __func__);
        ret = -EPROBE_DEFER;
        goto fj_temp_err2;
    }
    chip->fj_pm_adc = power_supply_get_by_name(FJ_PM_ADC_PSY_NAME);
    if (chip->fj_pm_adc == NULL) {
        FJ_TEMP_ERRLOG("[%s] power_supply_get_by_name pm_adc failed\n", __func__);
        ret = -EPROBE_DEFER;
        goto fj_temp_err2;
    }
    
    chip->fj_temp.name = FJ_TEMP_PSY_NAME;
    chip->fj_temp.type = POWER_SUPPLY_TYPE_FJ_TEMP;
    chip->fj_temp.get_property = fj_temp_drv_get_property;
    chip->fj_temp.properties = fj_temp_properties;
    chip->fj_temp.num_properties = ARRAY_SIZE(fj_temp_properties);
    
    ret = power_supply_register(&pdev->dev, &chip->fj_temp);
    if (unlikely(ret < 0)) {
        FJ_TEMP_ERRLOG("[%s] power_supply_register fj-temp failed ret = %d\n", __func__, ret);
        goto fj_temp_err2;
    }
    
    wake_lock_init(&chip->wakelock, WAKE_LOCK_SUSPEND, "temp_drv_status");
    
    /* init wait queue */
    init_waitqueue_head(&chip->wq);
    
    platform_set_drvdata(pdev, chip);
    
    the_chip = chip;
    
    /* kthread start */
    chip->thread = kthread_run(fj_temp_drv_thread, chip, "fj_temp_drv");
    
    FJ_TEMP_INFOLOG("[%s] probe End\n", __func__);
    
    return 0;
    
fj_temp_err2:
    destroy_workqueue(chip->fj_temp_drv_wq);
fj_temp_err1:
    kfree(chip);
    
    return ret;
}

static int fj_temp_drv_remove(struct platform_device *pdev)
{
    struct temp_drv_chip *chip = the_chip;
    
    the_chip = NULL;
    cancel_delayed_work(&chip->mt_monitor);
    destroy_workqueue(chip->fj_temp_drv_wq);
    power_supply_unregister(&chip->fj_temp);
    platform_set_drvdata(pdev, NULL);
    kfree(chip);
    
    return 0;
}

#ifdef CONFIG_PM
static int fj_temp_drv_suspend(struct device *dev)
{
    struct temp_drv_chip *chip = dev_get_drvdata(dev);
    
    FJ_TEMP_DBGLOG("[%s] in\n", __func__);
    set_bit(FJ_TEMP_DRV_EVENT_SUSPEND, &chip->event_state);
    
    cancel_delayed_work(&chip->mt_monitor);
    return 0;
}

static int fj_temp_drv_resume(struct device *dev)
{
    struct temp_drv_chip *chip = dev_get_drvdata(dev);
    
    FJ_TEMP_DBGLOG("[%s] in\n", __func__);
    
    clear_bit(FJ_TEMP_DRV_EVENT_SUSPEND, &chip->event_state);
    if ((fj_boot_mode != FJ_MODE_MAKER_MODE) && (fj_boot_mode != FJ_MODE_KERNEL_MODE)) {
        set_bit(FJ_TEMP_DRV_EVENT_MT_MONITOR, &chip->event_state);
        wake_up(&chip->wq);
    }
    return 0;
}

static const struct dev_pm_ops fj_temp_drv_pm_ops = {
    .suspend = fj_temp_drv_suspend,
    .resume = fj_temp_drv_resume,
};

#else  /* CONFIG_PM */
#define fj_temp_drv_suspend NULL
#define fj_temp_drv_resume  NULL
#endif /* CONFIG_PM */

static struct of_device_id temp_drv_match_table[] = {
    { .compatible = "fj,fj_temp_drv",},
    { },
};

static struct platform_driver fj_temp_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "fj_temp_drv",
        .of_match_table = temp_drv_match_table,
#ifdef CONFIG_PM
        .pm = &fj_temp_drv_pm_ops,
#endif /* CONFIG_PM */
           },
    .probe = fj_temp_drv_probe,
    .remove = fj_temp_drv_remove,
#ifndef CONFIG_PM
    .suspend    = fj_temp_drv_suspend,
    .resume     = fj_temp_drv_resume,
#endif /* !CONFIG_PM */
};

static int __init fj_temp_drv_init(void)
{
    return platform_driver_register(&fj_temp_driver);
}
module_init(fj_temp_drv_init);

static void __exit fj_temp_drv_exit(void)
{
    platform_driver_unregister(&fj_temp_driver);
}
module_exit(fj_temp_drv_exit);

MODULE_AUTHOR("Fujitsu");
MODULE_DESCRIPTION("Temp Driver");
MODULE_LICENSE("GPL");
