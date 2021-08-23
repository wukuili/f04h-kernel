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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/nonvolatile_common.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/fj_mode.h>
#include <linux/wait.h>

#include <linux/mfd/fj_charger.h>
#include "fj_charger_local.h"

#define FJ_BATTERY_DBGLOG(x, y...)              \
    if (unlikely(fj_battery_debug != 0)) {      \
        printk(KERN_ERR "[fj_batt] " x, ## y);  \
    }

#define FJ_BATTERY_INFOLOG(x, y...)             printk(KERN_INFO "[fj_batt] " x, ## y)
#define FJ_BATTERY_WARNLOG(x, y...)             printk(KERN_WARNING "[fj_batt] " x, ## y)
#define FJ_BATTERY_ERRLOG(x, y...)              printk(KERN_ERR "[fj_batt] " x, ## y)
#define FJ_BATTERY_RECLOG(x, y...)              printk("REC@REC@36[fj_batt] " x, ## y)

/* monitor time */
#define FJ_BATTERY_DRV_MONITOR_DELAY_100MS      msecs_to_jiffies(100)
#define FJ_BATTERY_DRV_MONITOR_DELAY_300MS      msecs_to_jiffies(300)
#define FJ_BATTERY_DRV_MONITOR_DELAY_500MS      msecs_to_jiffies(500)
#define FJ_BATTERY_DRV_MONITOR_DELAY_1S         msecs_to_jiffies(1000)
#define FJ_BATTERY_DRV_MONITOR_DELAY_10S        msecs_to_jiffies(10000)
#define FJ_BATTERY_DRV_MONITOR_DELAY_60S        msecs_to_jiffies(60000)

#define FJ_BATTERY_DRV_TIMER_30S                (30 * HZ)
#define FJ_BATTERY_DRV_TIMER_60S                (60 * HZ)
#define FJ_BATTERY_DRV_TIMER_180S               (180 * HZ)

/* thread events */
#define FJ_BATTERY_DRV_EVENT_SUSPEND            0
#define FJ_BATTERY_DRV_EVENT_MONITOR            1

/* batterty monitor time */
#define MONITOR_INTERVAL                        FJ_BATTERY_DRV_MONITOR_DELAY_10S
#define MONITOR_RESUME_INTERVAL                 FJ_BATTERY_DRV_MONITOR_DELAY_500MS
#define LOW_VOLTAGE_INTERVAL                    FJ_BATTERY_DRV_MONITOR_DELAY_1S
#define SHUTDOWN_VOLTAGE_INTERVAL               FJ_BATTERY_DRV_MONITOR_DELAY_100MS

#define BATTERY_INIT                            101
#define BATTERY_FULL                            100
#define BATTERY_98                              98
#define BATTERY_ZERO                            0
#define BATTERY_AGE_INIT                        100
#define BATTERY_FULL_KEEP_THRESHOLD             99

#define LOW_VOLTAGE                             3600
#define SHUTDOWN_VOLTAGE                        3200
#define BATTERY_SHUTDOWN_FIX                    3

static char fj_battery_drv_status_str[POWER_SUPPLY_STATUS_MAX][16] = {
    "UNKNOWN",
    "CHARGING",
    "DISCHARGING",
    "NOT_CHARGING",
    "FULL",
};

static char fj_battery_drv_health_str[POWER_SUPPLY_HEALTH_MAX][32] = {
    "UNKNOWN",
    "GOOD",
    "OVERHEAT",
    "WARM",
    "DEAD",
    "OVERVOLTAGE",
    "UNSPEC_FAILURE",
    "COLD",
    "COOL",
    "WATCHDOG_TIMER_EXPIRE",
    "SAFETY_TIMER_EXPIRE",
    "UNDER_SUPPLY_VOLTAGE",
    "OVER_SUPPLY_VOLTAGE",
    "HOLDER_ERROR",
    "USB_HOT",
    "USB_ERROR",
    "OTHER_ERROR"
};

static enum power_supply_property fj_battery_drv_properties[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_INITIALIZE,
    POWER_SUPPLY_PROP_AGE,
};

enum fj_battery_drv_func_limit {
    FJ_BATTERY_DRV_FUNC_LIMIT_CHARGE = 0,         /*  0 */
    FJ_BATTERY_DRV_FUNC_LIMIT_BATTERY_PRESENT,    /*  1 */
    FJ_BATTERY_DRV_FUNC_LIMIT_LOW_BATTERY,        /*  2 */
    FJ_BATTERY_DRV_FUNC_LIMIT_RESERVE_01,         /*  3 */
    FJ_BATTERY_DRV_FUNC_LIMIT_BATTERY_TEMP,       /*  4 */
    FJ_BATTERY_DRV_FUNC_LIMIT_TERMINAL_TEMP,      /*  5 */
    FJ_BATTERY_DRV_FUNC_LIMIT_RECEIVER_TEMP,      /*  6 */
    FJ_BATTERY_DRV_FUNC_LIMIT_CHARGE_TEMP,        /*  7 */
    FJ_BATTERY_DRV_FUNC_LIMIT_CENTER_TEMP,        /*  8 */
    FJ_BATTERY_DRV_FUNC_LIMIT_IRIS_TEMP,          /*  9 */
    FJ_BATTERY_DRV_FUNC_LIMIT_NUM,
};

enum fj_battery_drv_keep_capacity {
    FJ_BATTERY_DRV_KEEP_CAPACITY_DISABLE = 0,
    FJ_BATTERY_DRV_KEEP_CAPACITY_STANDBY,
    FJ_BATTERY_DRV_KEEP_CAPACITY_ENABLE,
};

struct battery_drv_chip {
    struct power_supply battery;
    
    struct power_supply *bms;
    struct power_supply *fj_chg;
    struct power_supply *fg_adc;
    
    struct workqueue_struct *fj_battery_drv_wq;
    struct delayed_work battery_monitor;
    
    struct wake_lock drv_status;
    
    /* battery status */
    int status;
    /* battery health */
    int health;
    /* battery present */
    int present;
    /* battery capacity */
    int soc;
    int fake_soc;
    /* battery voltage */
    int vcell;
    /* battery age */
    int age;
    
    /* monitor event state */
    unsigned long event_state;
    
    unsigned long keep_capacity_flag;
    unsigned long keep_capacity_time;
    
    unsigned int shutdown_voltage_count;
    unsigned int start_soc_value;
    
    bool check_disable[FJ_BATTERY_DRV_FUNC_LIMIT_NUM];
    
    struct mutex lock;
    wait_queue_head_t wq;
    void* thread;
};

static void fj_battery_drv_update_age_start(struct battery_drv_chip *chip);
static void fj_battery_drv_update_age_finish(struct battery_drv_chip *chip);

static struct battery_drv_chip *the_chip = NULL;
static int drv_initialized = 0; /* Driver init flag (0:no initialize, 1:initialized) */

static int battery_health = POWER_SUPPLY_HEALTH_GOOD;
static int battery_status = POWER_SUPPLY_STATUS_DISCHARGING;

static unsigned char soc_mask_flag = 0;
static unsigned long soc_mask_expire = 0;
static unsigned char soc_full_mask_flag = 0;

static int fake_status = POWER_SUPPLY_STATUS_MAX;
static int fj_battery_drv_set_fake_status(const char *val, struct kernel_param *kp)
{
    int result = 0;
    
    result = param_set_int(val, kp);
    if (result) {
        FJ_BATTERY_ERRLOG("[%s] error setting value %d\n",__func__, result);
        fake_status = POWER_SUPPLY_STATUS_MAX;
    }
    
    return result;
}
module_param_call(dbg_status, fj_battery_drv_set_fake_status, param_get_uint, &fake_status, 0644);

static int fake_health = POWER_SUPPLY_HEALTH_MAX;
static int fj_battery_drv_set_fake_health(const char *val, struct kernel_param *kp)
{
    int result = 0;
    
    result = param_set_int(val, kp);
    if (result) {
        FJ_BATTERY_ERRLOG("[%s] error setting value %d\n",__func__, result);
        fake_health = POWER_SUPPLY_HEALTH_MAX;
    }
    
    return result;
}
module_param_call(dbg_health, fj_battery_drv_set_fake_health, param_get_uint, &fake_health, 0644);

static int fj_battery_debug = 0;
static int set_battery_debug(const char *val, struct kernel_param *kp)
{
    int result = 0;
    
    result = param_set_int(val, kp);
    if (result) {
        FJ_BATTERY_ERRLOG("[%s] error setting value %d\n",__func__, result);
        return result;
    }
    
    if (result != 0) {
        fj_battery_debug = 0;
    }
    
    return result;
}
module_param_call(dbglog_battery, set_battery_debug, param_get_uint, &fj_battery_debug, 0644);

static void fj_battery_drv_function_limits_check(struct battery_drv_chip *chip)
{
    int result = 0;
    u16 val = 0;
    int i = 0;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(chip == NULL)) {
        FJ_BATTERY_WARNLOG("[%s] chip pointer is NULL\n", __func__);
        if (unlikely(the_chip == NULL)) {
            FJ_BATTERY_ERRLOG("[%s] the_chip pointer is NULL\n", __func__);
            return;
        }
        chip = the_chip;
    }
    
    result = get_nonvolatile((uint8_t*)&val, APNV_CHARGE_FG_FUNC_LIMITS_I, 2);
    if (unlikely(result < 0)) {
        val = 0x0000;
        FJ_BATTERY_ERRLOG("[%s] NV read err : %d set value = 0x%x \n", __func__, result, val);
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
     
    if (val != 0x0000) {
        for (i = 0;i < FJ_BATTERY_DRV_FUNC_LIMIT_NUM;i++) {
            if (val & 0x0001) {
                chip->check_disable[i] = true;
            } else {
                chip->check_disable[i] = false;
            }
            val = val >> 1;
            FJ_BATTERY_DBGLOG("[%s] check_disable[%d] = %d\n", __func__, i, chip->check_disable[i]);
        }
    }
}

static int fj_battery_drv_set_status(struct battery_drv_chip *chip, int status)
{
    int result = 0;
    unsigned long now = jiffies;
    int old_status;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (status >= POWER_SUPPLY_STATUS_MAX) {
        FJ_BATTERY_ERRLOG("[%s] Parameter Error! status = %d\n", __func__, status);
        result = -1;
        goto fj_battery_exit;
    }
    
    mutex_lock(&chip->lock);
    
    old_status = chip->status;
    
    if (chip->status != status) {
        if (status == POWER_SUPPLY_STATUS_CHARGING) {
            fj_battery_drv_update_age_start(chip);
            if (chip->keep_capacity_flag == FJ_BATTERY_DRV_KEEP_CAPACITY_STANDBY) {
                chip->keep_capacity_time = jiffies;
                chip->keep_capacity_flag = FJ_BATTERY_DRV_KEEP_CAPACITY_ENABLE;
            }
        } else {
            if (chip->keep_capacity_flag == FJ_BATTERY_DRV_KEEP_CAPACITY_ENABLE) {
                chip->keep_capacity_flag = FJ_BATTERY_DRV_KEEP_CAPACITY_DISABLE;
            }
        }
        
        if (status == POWER_SUPPLY_STATUS_DISCHARGING) {
            fj_battery_drv_update_age_finish(chip);
        }
        
        if ((chip->status == POWER_SUPPLY_STATUS_FULL) &&
            (status != POWER_SUPPLY_STATUS_FULL)) {
            /* 30 sec */
            soc_mask_expire = now + FJ_BATTERY_DRV_TIMER_30S;
            soc_mask_flag = 1;
            FJ_BATTERY_DBGLOG("[%s] now:%lu, expire:%lu\n", __func__, now, soc_mask_expire);
        }
        
        if ((drv_initialized == 1) &&
            (status == POWER_SUPPLY_STATUS_CHARGING) &&
            (chip->soc >= BATTERY_FULL)) {
            soc_full_mask_flag = 1;
            chip->status = POWER_SUPPLY_STATUS_FULL;
        } else {
            soc_full_mask_flag = 0;
            chip->status = status;
        }
        FJ_BATTERY_INFOLOG("[%s] old_status:%s new_status:%s set_status:%s capa:%d volt:%dmV\n",
                    __func__,
                    fj_battery_drv_status_str[old_status],
                    fj_battery_drv_status_str[chip->status],
                    fj_battery_drv_status_str[status],
                    chip->soc,
                    chip->vcell);
        
        power_supply_changed(&chip->battery);
    }
    
    mutex_unlock(&chip->lock);
    
fj_battery_exit:
    return result;
}

static int fj_battery_drv_set_health(struct battery_drv_chip *chip, int health)
{
    int result = 0;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (health >= POWER_SUPPLY_HEALTH_MAX) {
        FJ_BATTERY_ERRLOG("[%s] Parameter Error! health = %d\n", __func__, health);
        result = -1;
        goto fj_battery_exit;
    }
    
    mutex_lock(&chip->lock);
    
    if (chip->health != health) {
        FJ_BATTERY_INFOLOG("[%s] old_health:%s new_status:%s capa:%d volt:%dmV\n", __func__,
                                fj_battery_drv_health_str[chip->health],
                                fj_battery_drv_health_str[health],
                                chip->soc,
                                chip->vcell);
        
        chip->health = health;
        result = 1;
        
        power_supply_changed(&chip->battery);
    }
    
    mutex_unlock(&chip->lock);
    
fj_battery_exit:
    return result;
}

static int fj_battery_drv_get_soc(struct battery_drv_chip *chip,
                                       int *battery_capacity)
{
    int ret;
    union power_supply_propval prop = {0,};
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (chip->bms->get_property(chip->bms, POWER_SUPPLY_PROP_CAPACITY, &prop) < 0) {
        /* Failure */
        FJ_BATTERY_ERRLOG("[%s] Get soc FAILURE!\n", __func__);
        ret = -1;
    } else {
        *battery_capacity = prop.intval;
        ret = 0;
    }
    
    return ret;
}

static int fj_battery_drv_set_soc(struct battery_drv_chip *chip,
                                       int battery_capacity)
{
    int old_soc;
    unsigned char old_full_mask;
    unsigned long now = jiffies;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if ((fj_boot_mode == FJ_MODE_MAKER_MODE) || (fj_boot_mode == FJ_MODE_KERNEL_MODE)) {
        chip->soc = battery_capacity;
        return 0;
    }
    
    old_soc = chip->soc;
    old_full_mask = soc_full_mask_flag;
    
    if (chip->shutdown_voltage_count >= BATTERY_SHUTDOWN_FIX) {
        chip->soc = BATTERY_ZERO;
    } else if (chip->status == POWER_SUPPLY_STATUS_FULL) {
        if ((soc_full_mask_flag == 1) && (battery_capacity < BATTERY_98)) {
            soc_full_mask_flag = 0;
            chip->soc = battery_capacity;
            chip->status = POWER_SUPPLY_STATUS_CHARGING;
        } else {
            chip->soc = BATTERY_FULL;
        }
        soc_mask_flag = 0;
    } else if (time_before(now, soc_mask_expire) && soc_mask_flag) {
        FJ_BATTERY_DBGLOG("[%s] not update soc indicator\n", __func__);
        if (time_before((now + FJ_BATTERY_DRV_TIMER_60S), soc_mask_expire)) {
            FJ_BATTERY_INFOLOG("[%s] Error!!! soc_mask_expire is Unknown Value. now:%lu, expire:%lu\n", __func__, now, soc_mask_expire);
            FJ_BATTERY_RECLOG("[%s] Error!!! soc_mask_expire is Unknown Value. now:%lu, expire:%lu\n", __func__, now, soc_mask_expire);
            soc_mask_flag = 0;
            soc_mask_expire = 0;
        }
    } else if (chip->status == POWER_SUPPLY_STATUS_CHARGING) {
        if (battery_capacity >= BATTERY_FULL) {
            chip->soc = BATTERY_FULL - 1;
        } else if (battery_capacity < chip->soc) {
            chip->soc--;
        } else {
            chip->soc = battery_capacity;
        }
    } else {
        soc_mask_flag = 0;
        soc_mask_expire = 0;
        if (battery_capacity < chip->soc) {
            chip->soc--;
        } 
    }
    
    FJ_BATTERY_INFOLOG("[%s] old_soc:%d new_soc:%d set_soc:%d old_full_mask:%d full_mask:%d volt:%dmV\n",
                   __func__, old_soc, chip->soc, battery_capacity, old_full_mask, soc_full_mask_flag, chip->vcell);
    
    return 0;
}

static int fj_battery_drv_get_voltage(struct battery_drv_chip *chip, int *battery_voltage)
{
    int result = 0;
    union power_supply_propval get_prop = {0,};
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(battery_voltage == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] battery_voltage pointer is NULL\n", __func__);
        result = -1;
        goto fj_battery_exit;
    }
    
    if (chip->bms->get_property(chip->bms, POWER_SUPPLY_PROP_VOLTAGE_NOW, &get_prop) < 0) {
        FJ_BATTERY_ERRLOG("[%s] get prop FAILURE bms battery voltage\n", __func__);
        *battery_voltage = 0;
        result = -1;
        goto fj_battery_exit;
    } else {
        *battery_voltage = get_prop.intval / 1000;
    }
    
fj_battery_exit:
    return result;
}

static int fj_battery_drv_get_bcl_voltage(struct battery_drv_chip *chip, int *battery_voltage)
{
    int result = 0;
    union power_supply_propval get_prop = {0,};
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(battery_voltage == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] battery_voltage pointer is NULL\n", __func__);
        result = -1;
        goto fj_battery_exit;
    }
    
    if (chip->fg_adc->get_property(chip->fg_adc, POWER_SUPPLY_PROP_VOLTAGE_NOW, &get_prop) < 0) {
        FJ_BATTERY_ERRLOG("[%s] get prop FAILURE bcl battery voltage\n", __func__);
        *battery_voltage = 0;
        result = -1;
        goto fj_battery_exit;
    } else {
        *battery_voltage = get_prop.intval / 1000;
    }
    
fj_battery_exit:
    return result;
}

static int fj_battery_drv_get_temperature(struct battery_drv_chip *chip, int *battery_temperature)
{
    int result = 0;
    union power_supply_propval get_prop = {0,};
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(battery_temperature == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] battery_temperature pointer is NULL\n", __func__);
        result =  -1;
        goto fj_battery_exit;
    }
    
    if (chip->bms->get_property(chip->bms, POWER_SUPPLY_PROP_TEMP, &get_prop) < 0) {
        FJ_BATTERY_ERRLOG("[%s] get prop FAILURE battery temperature\n", __func__);
        *battery_temperature = 0;
        result = -1;
        goto fj_battery_exit;
    } else {
        *battery_temperature = get_prop.intval;
    }
    
fj_battery_exit:
    return result;
}

static void fj_battery_drv_get_age(struct battery_drv_chip *chip)
{
    int result = 0;
    u16 age_val = 0;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    result = get_nonvolatile((uint8_t*)&age_val, APNV_CHARGE_FG_AGE_I, 2);
    if ((result < 0) || (age_val > 100)) {
        chip->age = BATTERY_AGE_INIT;
        FJ_BATTERY_ERRLOG("[%s] APNV read err : %d %d set age = 0x%x \n", __func__, result, age_val, chip->age);
    } else {
        chip->age = age_val;
        FJ_BATTERY_INFOLOG("[%s] APNV read  : %d set age = 0x%x \n", __func__, result, chip->age);
    }
    
    return;
}

static void fj_battery_drv_update_age_start(struct battery_drv_chip *chip)
{
    int result = 0;
    int battery_capacity = 0;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (drv_initialized == 0) {
        goto fj_battery_exit;
    }
    
    if (chip->start_soc_value < BATTERY_INIT) {
        goto fj_battery_exit;
    }
    
    result = fj_battery_drv_get_soc(chip, &battery_capacity);
    if (unlikely(result < 0)) {
        goto fj_battery_exit;
    }
    
    chip->start_soc_value = battery_capacity;
    
fj_battery_exit:
    return;

}

static void fj_battery_drv_update_age_finish(struct battery_drv_chip *chip)
{
    int result = 0;
    int battery_capacity = 0;
    u16 count_val = 0;
    u16 age_val = 0;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (drv_initialized == 0) {
        goto fj_battery_exit;
    }
    
    result = fj_battery_drv_get_soc(chip, &battery_capacity);
    if (unlikely(result < 0)) {
        goto fj_battery_exit;
    }
    
    if (battery_capacity <= chip->start_soc_value) {
        goto fj_battery_exit;
    }
    
    result = get_nonvolatile((uint8_t*)&count_val, APNV_CHARGE_FG_VF_COUNT_I, 2);
    if (unlikely(result < 0)) {
        goto fj_battery_exit;
    }
    
    count_val += (battery_capacity - chip->start_soc_value);
    
    if (count_val >= 2000) {
        count_val -= 2000;
        
        result = get_nonvolatile((uint8_t*)&age_val, APNV_CHARGE_FG_AGE_I, 2);
        if (unlikely(result < 0)) {
            goto fj_battery_exit;
        }
        
        if ((age_val > 0) && (age_val <= 100)) {
            age_val--;
        } else {
            FJ_BATTERY_DBGLOG("[%s] AGE Error age_val = %d\n", __func__, age_val);
            goto fj_battery_exit;
        }
        
        result = set_nonvolatile((uint8_t*)&age_val, APNV_CHARGE_FG_AGE_I, 2);
        if (unlikely(result < 0)) {
            goto fj_battery_exit;
        }
        chip->age = age_val;
        
        FJ_BATTERY_INFOLOG("[%s] AGE UPDATE set value = 0x%04X \n", __func__, age_val);
        FJ_BATTERY_RECLOG("[%s] AGE UPDATE set value = 0x%04X \n", __func__, age_val);
    }
    
    result = set_nonvolatile((uint8_t*)&count_val, APNV_CHARGE_FG_VF_COUNT_I, 2);
    if (unlikely(result < 0)) {
        goto fj_battery_exit;
    }
    
fj_battery_exit:
    chip->start_soc_value = BATTERY_INIT;
    return;
}

static int fj_battery_drv_prop_status(struct battery_drv_chip *chip)
{
    if ((fake_status >= POWER_SUPPLY_STATUS_UNKNOWN) &&
        (fake_status < POWER_SUPPLY_STATUS_MAX)) {
        FJ_BATTERY_INFOLOG("[%s] in fake_status:%s\n", __func__, fj_battery_drv_status_str[fake_status]);
        return fake_status;
    }
    FJ_BATTERY_DBGLOG("[%s] in status:%s\n", __func__, fj_battery_drv_status_str[chip->status]);
    return chip->status;
}

static int fj_battery_drv_prop_health(struct battery_drv_chip *chip)
{
    if ((fake_health >= POWER_SUPPLY_HEALTH_UNKNOWN) &&
        (fake_health < POWER_SUPPLY_HEALTH_MAX)) {
        FJ_BATTERY_INFOLOG("[%s] in fake_health:%s\n", __func__, fj_battery_drv_health_str[fake_health]);
        return fake_health;
    }
    FJ_BATTERY_DBGLOG("[%s] in health:%s\n", __func__, fj_battery_drv_health_str[chip->health]);
    return chip->health;
}

static int fj_battery_drv_prop_present(struct battery_drv_chip *chip)
{
    return chip->present;
}

static int fj_battery_drv_prop_capacity(struct battery_drv_chip *chip)
{
    int result;
    int work_capacity;
    
    if (chip->check_disable[FJ_BATTERY_DRV_FUNC_LIMIT_LOW_BATTERY] == true) {
        result = 90;
    } else if ((chip->fake_soc >= 0) && (chip->fake_soc <= 100)) {
        result = chip->fake_soc;
    } else {
        fj_battery_drv_get_soc(chip, &work_capacity);
        fj_battery_drv_set_soc(chip, work_capacity);
        if (chip->soc > BATTERY_FULL) {
            result = BATTERY_FULL;
        } else {
            result = chip->soc;
        }
    }
    
    if (chip->keep_capacity_flag != FJ_BATTERY_DRV_KEEP_CAPACITY_DISABLE) {
        if (result == 0) {
            result = 1;
        }
    }
    
    return result;
}

static int fj_battery_drv_prop_voltage_now(struct battery_drv_chip *chip)
{
    int result;
    int work_voltage;
    
    if (chip->check_disable[FJ_BATTERY_DRV_FUNC_LIMIT_LOW_BATTERY] == true) {
        result = 3900;
    } else {
        fj_battery_drv_get_voltage(chip, &work_voltage);
        result = work_voltage;
    }
    
    return result;
}

static int fj_battery_drv_prop_temp(struct battery_drv_chip *chip)
{
    int result;
    int work_temp;
    
    if (chip->check_disable[FJ_BATTERY_DRV_FUNC_LIMIT_BATTERY_TEMP] == true) {
        result = 250;
    } else if (drv_initialized == 0) {
        result = 250;
    } else {
        fj_battery_drv_get_temperature(chip, &work_temp);
        result = work_temp;
    }
    
    return result;
}

static int fj_battery_drv_prop_age(struct battery_drv_chip *chip)
{
    return chip->age;
}

static int fj_battery_drv_get_property(struct power_supply *psy,
                                       enum power_supply_property psp,
                                       union power_supply_propval *val)
{
    struct battery_drv_chip *chip;
    
    if (unlikely(psy == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] psy pointer is NULL\n", __func__);
        return -EINVAL;
    }
    
    FJ_BATTERY_DBGLOG("[%s] in psp:%d\n", __func__, psp);
    
    chip = container_of(psy, struct battery_drv_chip, battery);
    
    switch (psp) {
        case POWER_SUPPLY_PROP_INITIALIZE:
            val->intval = drv_initialized;
            break;
            
        case POWER_SUPPLY_PROP_TECHNOLOGY:
            val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
            break;
            
        case POWER_SUPPLY_PROP_STATUS:
            val->intval = fj_battery_drv_prop_status(chip);
            break;
            
        case POWER_SUPPLY_PROP_HEALTH:
            val->intval = fj_battery_drv_prop_health(chip);
            break;
            
        case POWER_SUPPLY_PROP_PRESENT:
            val->intval = fj_battery_drv_prop_present(chip);
            break;
            
        case POWER_SUPPLY_PROP_CAPACITY:
            val->intval = fj_battery_drv_prop_capacity(chip);
            break;
            
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = fj_battery_drv_prop_voltage_now(chip);
            break;
            
        case POWER_SUPPLY_PROP_TEMP:
            val->intval = fj_battery_drv_prop_temp(chip);
            break;
            
        case POWER_SUPPLY_PROP_AGE:
            val->intval = fj_battery_drv_prop_age(chip);
            break;
            
        default:
            return -EINVAL;
            
    }
    return 0;
}

static int fj_battery_drv_set_property(struct power_supply *psy,
                                       enum power_supply_property psp,
                                       const union power_supply_propval *val)
{
    struct battery_drv_chip *chip;
    
    if (unlikely(psy == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] psy pointer is NULL\n", __func__);
        return -EINVAL;
    }
    
    FJ_BATTERY_DBGLOG("[%s] in psp:%d\n", __func__, psp);
    
    chip = container_of(psy, struct battery_drv_chip, battery);
    
    switch (psp) {
        case POWER_SUPPLY_PROP_STATUS:
            fj_battery_drv_set_status(chip, val->intval);
            break;
        case POWER_SUPPLY_PROP_HEALTH:
            fj_battery_drv_set_health(chip, val->intval);
            break;
        case POWER_SUPPLY_PROP_CAPACITY:
            chip->fake_soc = val->intval;
            break;
        default:
            return -EINVAL;
    }
    return 0;
}

static int fj_battery_drv_property_is_writeable(struct power_supply *psy,
                                                enum power_supply_property psp)
{
    int result = 0;
    
    if (unlikely(psy == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] psy pointer is NULL\n", __func__);
        return -EINVAL;
    }
    
    FJ_BATTERY_DBGLOG("[%s] in psp:%d\n", __func__, psp);
    
    switch (psp) {
        case POWER_SUPPLY_PROP_CAPACITY:
            result = 1;
            break;
        default:
            result = 0;
            break;
    }
    
    return result;
}

static void fj_battery_drv_external_power_changed(struct power_supply *psy)
{
    if (unlikely(psy == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] psy pointer is NULL\n", __func__);
        return;
    }
    return;
}

static void fj_battery_drv_event_monitor(struct work_struct *work)
{
    struct delayed_work *dwork;
    struct battery_drv_chip *chip;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(work == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] work pointer is NULL\n", __func__);
        return;
    }
    
    dwork = to_delayed_work(work);
    if (unlikely(dwork == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] dwork pointer is NULL\n", __func__);
        return;
    }
    
    chip = container_of(dwork, struct battery_drv_chip, battery_monitor);
    if (unlikely(chip == NULL)) {
        FJ_BATTERY_ERRLOG("[%s] chip pointer is NULL\n", __func__);
        return;
    }
    
    set_bit(FJ_BATTERY_DRV_EVENT_MONITOR, &chip->event_state);
    wake_up(&chip->wq);
}

static void fj_battery_drv_work_monitor(struct battery_drv_chip *chip)
{
    int result;
    int work_volt;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(chip == NULL)) {
        FJ_BATTERY_WARNLOG("[%s] chip pointer is NULL\n", __func__);
        if (unlikely(the_chip == NULL)) {
            FJ_BATTERY_ERRLOG("[%s] the_chip pointer is NULL\n", __func__);
            return;
        }
        chip = the_chip;
    }
    
    result = fj_battery_drv_get_bcl_voltage(chip, &work_volt);
    if (unlikely(result < 0)) {
        queue_delayed_work(chip->fj_battery_drv_wq, &chip->battery_monitor, MONITOR_INTERVAL);
        FJ_BATTERY_ERRLOG("[%s] Error get_voltage result = %d\n", __func__, result);
        goto fj_battery_exit;
    }
    
    chip->vcell = work_volt;
    
    if (chip->vcell <= SHUTDOWN_VOLTAGE) {
        chip->shutdown_voltage_count++;
        if (chip->shutdown_voltage_count >= BATTERY_SHUTDOWN_FIX) {
            FJ_BATTERY_ERRLOG("[%s] low battery detect volt:%dmV\n", __func__, chip->vcell);
            FJ_BATTERY_RECLOG("[%s] low battery detect volt:%dmV\n", __func__, chip->vcell);
        }
        queue_delayed_work(chip->fj_battery_drv_wq, &chip->battery_monitor, SHUTDOWN_VOLTAGE_INTERVAL);
        chip->keep_capacity_flag = FJ_BATTERY_DRV_KEEP_CAPACITY_DISABLE;
    } else if (chip->vcell <= LOW_VOLTAGE) {
        chip->shutdown_voltage_count = 0;
        queue_delayed_work(chip->fj_battery_drv_wq, &chip->battery_monitor, LOW_VOLTAGE_INTERVAL);
    } else {
        chip->shutdown_voltage_count = 0;
        queue_delayed_work(chip->fj_battery_drv_wq, &chip->battery_monitor, MONITOR_INTERVAL);
    }
    
    if (chip->keep_capacity_flag == FJ_BATTERY_DRV_KEEP_CAPACITY_STANDBY) {
        if (time_before((chip->keep_capacity_time + FJ_BATTERY_DRV_TIMER_30S), jiffies)) {
            chip->keep_capacity_flag = FJ_BATTERY_DRV_KEEP_CAPACITY_DISABLE;
        }
    } else if (chip->keep_capacity_flag == FJ_BATTERY_DRV_KEEP_CAPACITY_ENABLE) {
        if (time_before((chip->keep_capacity_time + FJ_BATTERY_DRV_TIMER_180S), jiffies)) {
            chip->keep_capacity_flag = FJ_BATTERY_DRV_KEEP_CAPACITY_DISABLE;
        }
    }
    
    power_supply_changed(&chip->battery);
    
fj_battery_exit:
    return;
}

static int fj_battery_drv_thread(void * ptr)
{
    struct battery_drv_chip *chip = ptr;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    if (unlikely(chip == NULL)) {
        FJ_BATTERY_WARNLOG("[%s] chip pointer is NULL\n", __func__);
        if (unlikely(the_chip == NULL)) {
            FJ_BATTERY_ERRLOG("[%s] the_chip pointer is NULL\n", __func__);
            return -EINVAL;
        }
        chip = the_chip;
    }
    
    while (1) {
        fj_battery_drv_get_soc(chip, &chip->soc);
        if (chip->soc <= BATTERY_FULL) {
            drv_initialized = 1;
            if ((chip->status == POWER_SUPPLY_STATUS_CHARGING) &&
                (chip->soc >= BATTERY_FULL_KEEP_THRESHOLD)) {
                soc_full_mask_flag = 1;
                chip->status = POWER_SUPPLY_STATUS_FULL;
            }
            break;
        } else {
            FJ_BATTERY_INFOLOG("[%s] qpnp_fg Not Ready Soc = %d\n", __func__, chip->soc);
        }
        msleep(100);
    }
    
    if ((fj_boot_mode != FJ_MODE_MAKER_MODE) && (fj_boot_mode != FJ_MODE_KERNEL_MODE)) {
        chip->keep_capacity_time = jiffies;
        if (chip->status == POWER_SUPPLY_STATUS_CHARGING) {
            chip->keep_capacity_flag = FJ_BATTERY_DRV_KEEP_CAPACITY_ENABLE;
        } else {
            chip->keep_capacity_flag = FJ_BATTERY_DRV_KEEP_CAPACITY_STANDBY;
        }
    }
    
    if (battery_status != POWER_SUPPLY_STATUS_DISCHARGING) {
        FJ_BATTERY_INFOLOG("[%s] old_status:%s new_status:%s\n",
                                                       __func__,
                      fj_battery_drv_status_str[battery_status],
                        fj_battery_drv_status_str[chip->status]);
        chip->status = battery_status;
    }
    
    if (battery_health != POWER_SUPPLY_HEALTH_GOOD) {
        FJ_BATTERY_INFOLOG("[%s] health %d > %d\n", __func__, chip->health, battery_health);
        chip->health = battery_health;
    }
    
    if (battery_status == POWER_SUPPLY_STATUS_CHARGING) {
        fj_battery_drv_update_age_start(chip);
    }
    
    fj_battery_drv_get_age(chip);
    fj_battery_drv_function_limits_check(chip);
    
    chip->present = 1;
    
    if ((fj_boot_mode != FJ_MODE_MAKER_MODE) && (fj_boot_mode != FJ_MODE_KERNEL_MODE)) {
        set_bit(FJ_BATTERY_DRV_EVENT_MONITOR, &chip->event_state);
    }
    set_freezable();
    
    do {
        wait_event_freezable(chip->wq, (chip->event_state || kthread_should_stop()));
        
        if (kthread_should_stop()) {
            FJ_BATTERY_INFOLOG("[%s] while loop break\n", __func__);
            break;
        }
        
        if (test_bit(FJ_BATTERY_DRV_EVENT_SUSPEND, &chip->event_state)) {
            test_and_clear_bit(FJ_BATTERY_DRV_EVENT_MONITOR, &chip->event_state);
        }
        
        if (test_and_clear_bit(FJ_BATTERY_DRV_EVENT_MONITOR, &chip->event_state)) {
            fj_battery_drv_work_monitor(chip);
        }
        
    } while (1);
    
    return 0;
}

extern int charging_mode;
static int fj_battery_drv_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct battery_drv_chip *chip;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        return -ENOMEM;
    }
    
    chip->bms = power_supply_get_by_name(QPNP_BMS_PSY_NAME);
    if (chip->bms == NULL) {
        FJ_BATTERY_ERRLOG("[%s] power_supply_get_by_name bms failed\n", __func__);
        devm_kfree(&pdev->dev, chip);
        return -EPROBE_DEFER;
    }
    chip->fg_adc = power_supply_get_by_name(BCL_FG_ADC_PSY_NAME);
    if (chip->fg_adc == NULL) {
        FJ_BATTERY_ERRLOG("[%s] power_supply_get_by_name fg_adc failed\n", __func__);
        devm_kfree(&pdev->dev, chip);
        return -EPROBE_DEFER;
    }
    
    
    platform_set_drvdata(pdev, chip);
    
    mutex_init(&chip->lock);
    wake_lock_init(&chip->drv_status, WAKE_LOCK_SUSPEND, "fj_battery_drv_status");
    
    chip->health = POWER_SUPPLY_HEALTH_GOOD;
    chip->fake_soc = -EINVAL;
    chip->event_state = 0;
    chip->soc = BATTERY_INIT;
    chip->vcell = 0;
    chip->start_soc_value = BATTERY_INIT;
    chip->shutdown_voltage_count = 0;
    chip->keep_capacity_time = 0;
    chip->keep_capacity_flag = FJ_BATTERY_DRV_KEEP_CAPACITY_DISABLE;
    
    if (charging_mode) {
        chip->status = POWER_SUPPLY_STATUS_CHARGING;
        battery_status = POWER_SUPPLY_STATUS_CHARGING;
    } else {
        chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
        battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
    }
    
    chip->battery.name                   = FJ_BATTERY_PSY_NAME;
    chip->battery.type                   = POWER_SUPPLY_TYPE_BATTERY;
    chip->battery.properties             = fj_battery_drv_properties;
    chip->battery.num_properties         = ARRAY_SIZE(fj_battery_drv_properties);
    chip->battery.get_property           = fj_battery_drv_get_property;
    chip->battery.set_property           = fj_battery_drv_set_property;
    chip->battery.property_is_writeable  = fj_battery_drv_property_is_writeable;
    chip->battery.external_power_changed = fj_battery_drv_external_power_changed;
    
    ret = power_supply_register(&pdev->dev, &chip->battery);
    if (ret) {
        dev_err(&pdev->dev, "failed: power supply register\n");
        devm_kfree(&pdev->dev, chip);
        return ret;
    }
	
    INIT_DELAYED_WORK(&chip->battery_monitor, fj_battery_drv_event_monitor);
    chip->fj_battery_drv_wq = create_singlethread_workqueue(dev_name(&pdev->dev));
    if (!chip->fj_battery_drv_wq) {
        FJ_BATTERY_ERRLOG("[%s] create_singlethread_workqueue failed\n", __func__);
        power_supply_unregister(&chip->battery);
        devm_kfree(&pdev->dev, chip);
        return -ESRCH;
    }
    
    /* init wait queue */
    init_waitqueue_head(&chip->wq);
    
    the_chip = chip;
    
    /* kthread start */
    chip->thread = kthread_run(fj_battery_drv_thread, chip, "fj_battery_drv");
    
    FJ_BATTERY_INFOLOG("[%s] probe End\n", __func__);
    
    return 0;
}

static int fj_battery_drv_remove(struct platform_device *pdev)
{
    struct battery_drv_chip *chip = the_chip;
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    the_chip = NULL;
    cancel_delayed_work(&chip->battery_monitor);
    destroy_workqueue(chip->fj_battery_drv_wq);
    power_supply_unregister(&chip->battery);
    platform_set_drvdata(pdev, NULL);
    devm_kfree(&pdev->dev, chip);
    
    return 0;
}

#ifdef CONFIG_PM
static int fj_battery_drv_suspend(struct device *dev)
{
    struct battery_drv_chip *chip = dev_get_drvdata(dev);
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    set_bit(FJ_BATTERY_DRV_EVENT_SUSPEND, &chip->event_state);
    
    cancel_delayed_work(&chip->battery_monitor);
    return 0;
}

static int fj_battery_drv_resume(struct device *dev)
{
    struct battery_drv_chip *chip = dev_get_drvdata(dev);
    
    FJ_BATTERY_DBGLOG("[%s] in\n", __func__);
    
    clear_bit(FJ_BATTERY_DRV_EVENT_SUSPEND, &chip->event_state);
    if ((fj_boot_mode != FJ_MODE_MAKER_MODE) && (fj_boot_mode != FJ_MODE_KERNEL_MODE)) {
        queue_delayed_work(chip->fj_battery_drv_wq, &chip->battery_monitor, MONITOR_RESUME_INTERVAL);
        wake_up(&chip->wq);
    }
    
    return 0;
}

static const struct dev_pm_ops fj_battery_drv_pm_ops = {
    .suspend = fj_battery_drv_suspend,
    .resume = fj_battery_drv_resume,
};

#else  /* CONFIG_PM */
#define fj_battery_drv_suspend NULL
#define fj_battery_drv_resume  NULL
#endif /* CONFIG_PM */

static struct of_device_id battery_drv_match_table[] = {
    { .compatible = "fj,fj_battery_drv",},
    { },
};

static struct platform_driver fj_battery_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "fj_battery_drv",
        .of_match_table = battery_drv_match_table,
#ifdef CONFIG_PM
        .pm = &fj_battery_drv_pm_ops,
#endif /* CONFIG_PM */
           },
    .probe = fj_battery_drv_probe,
    .remove = fj_battery_drv_remove,
#ifndef CONFIG_PM
    .suspend    = fj_battery_drv_suspend,
    .resume     = fj_battery_drv_resume,
#endif /* !CONFIG_PM */
};

static int __init fj_battery_drv_init(void)
{
    return platform_driver_register(&fj_battery_driver);
}
module_init(fj_battery_drv_init);

static void __exit fj_battery_drv_exit(void)
{
    platform_driver_unregister(&fj_battery_driver);
}
module_exit(fj_battery_drv_exit);


MODULE_AUTHOR("FUJITSU");
MODULE_DESCRIPTION("Battery Driver");
MODULE_LICENSE("GPL");
