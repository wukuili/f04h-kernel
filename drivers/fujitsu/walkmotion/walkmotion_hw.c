/*
 * Copyright(C) 2012-2016 FUJITSU LIMITED
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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/walkmotion.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/input.h>
#include <linux/kdev_t.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include "walkmotion_hw.h"
#include "walkmotion_data.h"
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pin.h>

#include <linux/atomic.h>

static int gpio_num_reset;
static int gpio_num_hosu_wup;

int sar_ctl_flg = 0;
static int gpio_num_sar_ctrl = -1;

static struct qpnp_pin_cfg qpnp_pin_cng_hce_config = {
	.mode = QPNP_PIN_MODE_DIG_OUT,				/* DIGITAL OUT		*/
	.output_type = QPNP_PIN_OUT_BUF_CMOS,		/* CMOS				*/
	.invert = QPNP_PIN_INVERT_ENABLE,			/* ENABLE			*/
	.pull = QPNP_PIN_GPIO_PULL_NO,				/* PULL NO			*/
	.vin_sel= QPNP_PIN_VIN2,					/* SMPS3			*/
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,	/* STRENGTH_LOW		*/
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,		/* func constant	*/
	.master_en = QPNP_PIN_MASTER_ENABLE,		/* Enable GPIO		*/
};

int ldo_ctl_flg = 0;
static int gpio_num_sens_hce_ldo_on = -1;

ktime_t g_irq_start_time1;
ktime_t g_irq_start_time2;
#define IRQ_TIMEOUT 100

extern struct fj_wm_data *fj_wm_data;
extern struct input_dev *fj_wm_this_data;
extern struct workqueue_struct *fj_wm_wq;
extern struct work_struct fj_wm_work_queue_motion_irq;

extern atomic_t fj_wm_wq_count;
/** (HW)fj_wm_hw_irq_handler
 *
 * @param irq  : Not use
 * @param data : Not use
 * @return IRQ_HANDLED
 */
irqreturn_t fj_wm_hw_irq_handler(int irq, void *data)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);
    if (fj_wm_data->init_flag == 1) {
        DBG_LOG_WM_DETAIL("%s : first motion_irq\n", __func__);
        fj_wm_data->init_flag = 0;
        wake_up_interruptible(&fj_wm_data->wait_queue_motion_irq);
        DBG_LOG_WM_TRACE("%s : end\n", __func__);
        return IRQ_HANDLED;
    }

    atomic_inc(&fj_wm_wq_count);
    if (atomic_read(&fj_wm_wq_count) == 1) {
        g_irq_start_time1 = ktime_get();
    } else if (atomic_read(&fj_wm_wq_count) == 2) {
        g_irq_start_time2 = ktime_get();
        DBG_LOG_WM_DETAIL("%s: IRQ count %d\n", __func__, atomic_read(&fj_wm_wq_count));
        DBG_LOG_WM_DETAIL("%s: %lldms continuous IRQ\n", __func__, ktime_to_ms(ktime_sub(g_irq_start_time2, g_irq_start_time1)));
    }
    queue_work(fj_wm_wq, &fj_wm_work_queue_motion_irq);
    wake_lock_timeout(&fj_wm_data->wake_lock,
        unlikely(fj_wm_data->prepare_flag) ? FJ_WM_WAKELOCK_TIME_PREPARE*HZ : FJ_WM_WAKELOCK_TIME*HZ);

    DBG_LOG_WM_TRACE("%s : end\n", __func__);

    return IRQ_HANDLED;
}

/** (HW)fj_wm_hw_board_check
 *
 * @param ws : Not use
 * @return void
 */
void fj_wm_hw_motion_irq(struct work_struct *ws)
{
    static int ev_value = 1;
    ktime_t irq_time_diff;

    DBG_LOG_WM_TRACE("%s : start\n", __func__);
    irq_time_diff = ktime_sub(ktime_get(), g_irq_start_time1);
    if (ktime_to_ms(irq_time_diff) > IRQ_TIMEOUT) {
        DBG_LOG_WM_DETAIL("%s : irq time out %lldms\n", __func__, ktime_to_ms(irq_time_diff));
    }

    if (atomic_read(&fj_wm_wq_count) >= 1) {
        atomic_set(&fj_wm_wq_count, 0);
    }
    if (likely(fj_wm_data->irq_flag == 1)) {
        DBG_LOG_WM_DETAIL("%s : normal motion_irq(ev_value : %d)\n", __func__, ev_value);
        input_report_abs(fj_wm_this_data, ABS_X, ev_value);
        input_sync(fj_wm_this_data);
        ev_value =  (ev_value == 1) ? 2 : 1;
    }
    DBG_LOG_WM_TRACE("%s : end\n", __func__);
}

/** (HW)fj_wm_hw_set_gpio
 *
 * @param pin   : gpio pin
 * @param value : gpio value
 * @return 0  Success
 */
static int fj_wm_hw_set_gpio(unsigned int pin, int value)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);
    DBG_LOG_WM_DETAIL("gpio_set_value_cansleep(%d, %d)\n", pin, value);

	gpio_set_value_cansleep(pin, value);

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** (HW)fj_wm_hw_get_gpio_num
 *
 * @param id   : gpios difinition of device tree
 * @return int real gpio_number
 */
static int fj_wm_hw_get_gpio_num(int id)
{
    struct device_node *node;
    enum of_gpio_flags flag;
    int gpio_num;

    node = of_find_compatible_node(NULL, NULL, "fj_wm_device");
    gpio_num = of_get_gpio_flags(node, id, &flag);

    return gpio_num;
}

/** (HW)fj_wm_hw_set_sar_config
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_hw_set_sar_config(void)
{
    struct device_node *node;
    int ret;

    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    node = of_find_compatible_node(NULL, NULL, "fj_wm_device");
    ret = of_property_read_u32(node, "fj_wm_device,sar_ctl", &sar_ctl_flg);
    if(unlikely(ret)) {
        printk(KERN_ERR "%s: of_property_read_u32(sar_ctl)=%d\n", __func__, ret);
        return ret;
    }
    if(sar_ctl_flg) {
        gpio_num_sar_ctrl = of_get_named_gpio_flags(node, "fj_wm_device,gpio_sar", 0, NULL);
        if(gpio_num_sar_ctrl < 0) {
            printk(KERN_ERR "%s: of_get_named_gpio_flags(gpio_sar)=%d\n", __func__, ret);
            return gpio_num_sar_ctrl;
        }
    }
    DBG_LOG_WM_TRACE("%s : end sar_ctl_flg(%d)\n", __func__, sar_ctl_flg);
    return 0;
}

/** (HW)fj_wm_hw_set_ldo_config
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_hw_set_ldo_config(void)
{
    struct device_node *node;
    int ret;

    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    node = of_find_compatible_node(NULL, NULL, "fj_wm_device");
    ret = of_property_read_u32(node, "fj_wm_device,ldo_ctl", &ldo_ctl_flg);
    if(unlikely(ret)) {
        printk(KERN_ERR "%s: of_property_read_u32(ldo_ctl)=%d\n", __func__, ret);
        return ret;
    }
    if(ldo_ctl_flg) {
        gpio_num_sens_hce_ldo_on = of_get_named_gpio_flags(node, "fj_wm_device,gpio_ldo", 0, NULL);
        if(gpio_num_sens_hce_ldo_on < 0) {
            printk(KERN_ERR "%s: of_get_named_gpio_flags(gpio_ldo)=%d\n", __func__, ret);
            return gpio_num_sens_hce_ldo_on;
        }
    }
    DBG_LOG_WM_TRACE("%s : end ldo_ctl_flg(%d)\n", __func__, ldo_ctl_flg);
    return 0;
}


/** (HW)fj_wm_hw_init
 *
 * @return 0  Success
 */
int fj_wm_hw_init(void)
{
    int ret = 0;

    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_hw_get_gpio_num(DT_WM_GPIO_RESET);
	if (unlikely(ret < 0)) {
        printk(KERN_ERR "%s: fj_wm_hw_get_gpio_num(DT_WM_GPIO_RESET)=%d\n", __func__, ret);
        return ret ;
	}
	gpio_num_reset = ret ;

	DBG_LOG_WM_DETAIL("%s : DT_WM_GPIO_RESET GPIO No.(%d)\n", __func__, gpio_num_reset);

	ret = fj_wm_hw_get_gpio_num(DT_WM_GPIO_HOSU_WUP);
	if (unlikely(ret < 0)) {
        printk(KERN_ERR "%s: fj_wm_hw_get_gpio_num(DT_WM_GPIO_HOSU_WUP)=%d\n", __func__, ret);
        return ret ;
	}
	gpio_num_hosu_wup = ret ;

	DBG_LOG_WM_DETAIL("%s : DT_WM_GPIO_HOSU_WUP GPIO No. (%d)\n", __func__, gpio_num_hosu_wup);

    ret = fj_wm_hw_set_ldo_config();
    if ( unlikely(ret < 0) ) {
        printk(KERN_ERR "%s: fj_wm_hw_set_ldo_config()=%d\n", __func__, ret);
        return ret ;
    }
    if(ldo_ctl_flg) {
        usleep(FJ_WM_SLEEP_TIME);
        DBG_LOG_WM_DETAIL("%s : PM8921_SENS_HCE_LDO_ON GPIO No. (%d)\n", __func__, gpio_num_sens_hce_ldo_on);

        ret = qpnp_pin_config(gpio_num_sens_hce_ldo_on, &qpnp_pin_cng_hce_config);
        if (unlikely(ret != 0)) {
            printk(KERN_ERR "%s: qpnp_pin_config(HCE_LDO_ON)=%d\n", __func__, ret);
            goto ON_ERR ;
        }
        fj_wm_hw_set_gpio(gpio_num_sens_hce_ldo_on, FJ_WM_GPIO_HIGH);

        usleep(FJ_WM_SLEEP_TIME_AFTER_LDO_ON);
    }

	ret = fj_wm_hw_set_sar_config();
	if ( unlikely(ret < 0) ) {
        printk(KERN_ERR "%s: fj_wm_hw_set_sar_config()=%d\n", __func__, ret);
        return ret ;
    }

ON_ERR:

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return ret ;
}

/** (HW)fj_wm_hw_exit
 *
 * @return 0  Success
 */
int fj_wm_hw_exit(void)
{

    if(ldo_ctl_flg) {
        DBG_LOG_WM_TRACE("%s : start\n", __func__);

        fj_wm_hw_set_gpio(gpio_num_sens_hce_ldo_on, FJ_WM_GPIO_LOW);

        DBG_LOG_WM_DETAIL("%s : HCE_LDO_ON GPIO No. (%d) OUT (%d) \n", __func__, gpio_num_sens_hce_ldo_on,FJ_WM_GPIO_LOW);

        DBG_LOG_WM_TRACE("%s : end\n", __func__);
    }
    return 0;
}

/** (HW)fj_wm_hw_probe
 *
 * @return 0  Success
 *         !0 Fail
 */
int fj_wm_hw_probe(void)
{
    int ret = 0;
	int gpio_num;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    gpio_num = fj_wm_hw_get_gpio_num(DT_WM_GPIO_MOTION_IRQ);

	DBG_LOG_WM_DETAIL("%s : DT_WM_GPIO_MOTION_IRQ GPIO No. (%d)\n", __func__, gpio_num);

	ret = pinctrl_select_state(fj_wm_data->pin_res.pinctrl, fj_wm_data->pin_res.gpio_state_hce_reset);
	if (ret) {
		printk("%s: pinctrl_select_state reset \n", __func__);
		return ret;
	}
	ret = pinctrl_select_state(fj_wm_data->pin_res.pinctrl, fj_wm_data->pin_res.gpio_state_hce_wakeup);
	if (ret) {
		printk("%s: pinctrl_select_state wakeup \n", __func__);
		return ret;
	}
	ret = pinctrl_select_state(fj_wm_data->pin_res.pinctrl, fj_wm_data->pin_res.gpio_state_hce_irq);
	if (ret) {
		printk("%s: pinctrl_select_state irq \n", __func__);
		return ret;
	}
	if(sar_ctl_flg) {
		ret = pinctrl_select_state(fj_wm_data->pin_res.pinctrl, fj_wm_data->pin_res.gpio_state_hce_sar);
		if (ret) {
			printk("%s: pinctrl_select_state sar \n", __func__);
			return ret;
		}
    }

	ret = gpio_request(gpio_num_reset, "DT_WM_GPIO_RESET");
	if (ret){
		printk("%s: gpio_request(DT_WM_GPIO_RESET)\n", __func__);
		return ret;
	}
	ret = gpio_request(gpio_num_hosu_wup, "DT_WM_GPIO_WAKE_UP");
	if (ret){
		printk("%s: gpio_request(DT_WM_GPIO_WAKE_UP) \n", __func__);
		return ret;
	}
	ret = gpio_request(gpio_num, "DT_WM_GPIO_MOTION_IRQ");
	if (ret){
		printk("%s: gpio_request(DT_WM_GPIO_MOTION_IRQ) \n", __func__);
		return ret;
	}
	if(sar_ctl_flg) {
		ret = gpio_request(gpio_num_sar_ctrl, "DT_WM_GPIO_SAR");
		if (ret){
			printk("%s: gpio_request(DT_WM_GPIO_SAR) \n", __func__);
			return ret;
		}
	}

	gpio_direction_output(gpio_num_reset,0);
	gpio_direction_output(gpio_num_hosu_wup,1);
	gpio_direction_input(gpio_num);
	if(sar_ctl_flg) {
		gpio_direction_output(gpio_num_sar_ctrl,0);
	}

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (HW)fj_wm_hw_remove
 *
 * @return 0  Success
 */
int fj_wm_hw_remove(void)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);
    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** (HW)fj_wm_hw_release
 *
 * @return 0  Success
 */
int fj_wm_hw_release(void)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    if (likely(fj_wm_data->irq_flag == 1)) {
        free_irq(fj_wm_data->motion_irq, fj_wm_data);
        fj_wm_data->irq_flag = 0;
        DBG_LOG_WM_DETAIL("%s : irq_flag = %d\n", __func__, fj_wm_data->irq_flag);
    }

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** (HW)fj_wm_hw_hce_reset
 *
 * @return 0  Success
 *         !0 Fail
 */
long fj_wm_hw_hce_reset(void)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    fj_wm_hw_set_gpio(gpio_num_reset, FJ_WM_GPIO_LOW);

	DBG_LOG_WM_DETAIL("%s : DT_WM_GPIO_RESET GPIO No.(%d) Value(%d)\n", __func__, gpio_num_reset,FJ_WM_GPIO_LOW);


    /* Wait for 20ms */
    msleep(fj_wm_data->mc_init_delay);
    fj_wm_data->init_flag = 1;
    if (likely(fj_wm_data->irq_flag == 0)) {
        ret = request_irq(fj_wm_data->motion_irq, fj_wm_hw_irq_handler,
                                    IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, "fj_wm", fj_wm_data);
        if (unlikely(ret < 0)) {
            printk(KERN_ERR "FJ_WM_IOCT_INITIALIZE : request_irq failed (ret : %d)\n", ret);
            fj_wm_data->init_flag = 0;
            return ret;
        } else {
            fj_wm_data->irq_flag = 1;
            DBG_LOG_WM_DETAIL("FJ_WM_IOCT_INITIALIZE : irq_flag = %d\n", fj_wm_data->irq_flag);
        }
    }
    fj_wm_hw_set_gpio(gpio_num_reset, FJ_WM_GPIO_HIGH);
	DBG_LOG_WM_DETAIL("%s : DT_WM_GPIO_RESET GPIO No.(%d) Value(%d)\n", __func__, gpio_num_reset,FJ_WM_GPIO_HIGH);
    /* Wait IRQ */
    ret = wait_event_interruptible_timeout(fj_wm_data->wait_queue_motion_irq,
                                                    fj_wm_data->init_flag == 0,
                                                    msecs_to_jiffies(FJ_WM_HCE_RESET_IRQ_WAIT_TIME));
    fj_wm_data->init_flag = 0;
    if (unlikely(ret <= 0)) {
        /* If canceled */
        if (likely(fj_wm_data->irq_flag == 1)) {
            free_irq(fj_wm_data->motion_irq, fj_wm_data);
            fj_wm_data->irq_flag = 0;
            DBG_LOG_WM_DETAIL("FJ_WM_IOCT_INITIALIZE : irq_flag = %d\n", fj_wm_data->irq_flag);
        }
        printk(KERN_ERR "FJ_WM_IOCT_INITIALIZE : wait_event_interruptible_timeout canceled (ret : %d)\n", ret);
        return -ECANCELED;
    } else {
        if (likely(fj_wm_data->irq_flag == 1)) {
            free_irq(fj_wm_data->motion_irq, fj_wm_data);
            fj_wm_data->irq_flag = 0;
            DBG_LOG_WM_DETAIL("FJ_WM_IOCT_INITIALIZE : irq_flag = %d\n", fj_wm_data->irq_flag);
        }
    }

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** (HW)fj_wm_hw_hce_request_irq
 *
 * @param value
 * @return 0  Success
 *         !0 Fail
 */
long fj_wm_hw_hce_request_irq(unsigned int value)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    if (value == FJ_WM_EDGE_HIGH) {
        /* High-edge detection */
        DBG_LOG_WM_DETAIL("FJ_WM_IOCS_REQUESTMOTIONIRQ : FJ_WM_EDGE_HIGH\n");
        if (likely(fj_wm_data->irq_flag == 0)) {
            ret = request_irq(fj_wm_data->motion_irq, fj_wm_hw_irq_handler,
                                    IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, "fj_wm", fj_wm_data);
            if (unlikely(ret < 0)) {
                printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ"
                     "(FJ_WM_EDGE_HIGH) : request_irq failed (ret : %d)\n",
                                                                      ret);
                return ret;
            }
            fj_wm_data->irq_flag = 1;
            DBG_LOG_WM_DETAIL("FJ_WM_IOCS_REQUESTMOTIONIRQ(FJ_WM_EDGE_HIGH) : "
                                     "irq_flag = %d\n", fj_wm_data->irq_flag);
        }
    } else {
        /* Low-edge detection */
        DBG_LOG_WM_DETAIL("FJ_WM_IOCS_REQUESTMOTIONIRQ : FJ_WM_EDGE_LOW\n");
        if (likely(fj_wm_data->irq_flag == 0)) {
            ret = request_irq(fj_wm_data->motion_irq, fj_wm_hw_irq_handler,
                                   IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "fj_wm", fj_wm_data);
            if (unlikely(ret < 0)) {
                printk(KERN_ERR "FJ_WM_IOCS_REQUESTMOTIONIRQ"
                                    "(FJ_WM_EDGE_LOW) : request_irq failed"
                                                    " (ret : %d)\n" , ret);
                return ret;
            }
            fj_wm_data->irq_flag = 1;
            DBG_LOG_WM_DETAIL("FJ_WM_IOCS_REQUESTMOTIONIRQ(FJ_WM_EDGE_LOW) : "
                                     "irq_flag = %d\n", fj_wm_data->irq_flag);
        }
    }

    irq_set_irq_wake(fj_wm_data->motion_irq, 1);

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** (HW)fj_wm_hw_hce_cancel_irq
 *
 * @return 0  Success
 */
long fj_wm_hw_hce_cancel_irq(void)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    if (likely(fj_wm_data->irq_flag == 1)) {
        free_irq(fj_wm_data->motion_irq, fj_wm_data);
        fj_wm_data->irq_flag = 0;
        DBG_LOG_WM_DETAIL("FJ_WM_IOCT_CANCELMOTIONIRQ : irq_flag = %d\n",
                                                        fj_wm_data->irq_flag);
    }
    input_report_abs(fj_wm_this_data, ABS_X, 0);
    input_sync(fj_wm_this_data);

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** fj_wm_hw_hce_set_sar
 *
 * @param arg : HCE_SAR_GPIO_HIGH / HCE_SAR_GPIO_LOW
 * @return 0  Success
 */
long fj_wm_hw_hce_set_sar(int arg)
{
	if (arg == HCE_SAR_GPIO_HIGH) {
		DBG_LOG_WM_TRACE("gpio_set_value_cansleep(%d, %d)\n", gpio_num_sar_ctrl, HCE_SAR_GPIO_HIGH);
		gpio_set_value_cansleep(gpio_num_sar_ctrl, HCE_SAR_GPIO_HIGH);
	} else {
		DBG_LOG_WM_TRACE("gpio_set_value_cansleep(%d, %d)\n", gpio_num_sar_ctrl, HCE_SAR_GPIO_LOW);
		gpio_set_value_cansleep(gpio_num_sar_ctrl, HCE_SAR_GPIO_LOW);
	}

	return 0;
}

/** (HW)fj_wm_hw_hce_wakeup_set
 *
 * @param value
 * @return 0  Success
 */
long fj_wm_hw_hce_wakeup_set(unsigned int value)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);
    fj_wm_hw_set_gpio(gpio_num_hosu_wup, value);
	DBG_LOG_WM_DETAIL("%s : DT_WM_GPIO_HOSU_WUP GPIO No.(%d) Value(%d)\n", __func__, gpio_num_hosu_wup,value);
    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

