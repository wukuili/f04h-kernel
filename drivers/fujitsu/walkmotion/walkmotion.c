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
#include <linux/nonvolatile_common.h>
#include "walkmotion_hw.h"
#include "walkmotion_data.h"
#include <linux/compat.h>
#if defined(CONFIG_OF)
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

/* Driver name */
#define DRIVER_NAME "fj-walkmotion"
#define DEV_NAME "fj_wm_ctl"
/* Delay */
#define FJ_WM_DEF_MC_INIT_DELAY 20

/* Initializer */
#define FJ_WM_STS_INITIALIZER        (0x00)
/* Initialized */
#define FJ_WM_STS_MC_INITIALIZED     (0x01)
#define HCE_PINCTRL_STATE_RESET  "hce_reset_active"
#define HCE_PINCTRL_STATE_WAKEUP "hce_wakeup_active"
#define HCE_PINCTRL_STATE_IRQ 	 "hce_irq_active"
#define IRQ_TIMEOUT 100
#define HCE_PINCTRL_STATE_SAR 	 "hce_sar_default"
struct fj_wm_data *fj_wm_data;
struct input_dev *fj_wm_this_data;
struct workqueue_struct *fj_wm_wq;

static int fj_wm_major;
static struct cdev fj_wm_cdev;
static struct class *fj_wm_class;
static int fj_wm_open_cnt;

uint8_t fj_wm_debug_mask;

DECLARE_WORK(fj_wm_work_queue_motion_irq, fj_wm_hw_motion_irq);

/** (SW)fj_wm_sw_init
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_sw_init(void)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_hw_init();

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (SW)fj_wm_sw_exit
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_sw_exit(void)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_hw_exit();

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (SW)fj_wm_sw_probe
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_sw_probe(struct device *dev)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);


	fj_wm_data->pin_res.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(fj_wm_data->pin_res.pinctrl)) {
		printk(KERN_ERR "%s: can not get devm_pinctrl_get\n", __func__);
		return PTR_ERR(fj_wm_data->pin_res.pinctrl);
	}

	fj_wm_data->pin_res.gpio_state_hce_reset = pinctrl_lookup_state(fj_wm_data->pin_res.pinctrl,HCE_PINCTRL_STATE_RESET);
	if (IS_ERR_OR_NULL(fj_wm_data->pin_res.gpio_state_hce_reset)) {
		printk(KERN_ERR "%s: can not get hce_reset pinstate\n", __func__);
		return PTR_ERR(fj_wm_data->pin_res.gpio_state_hce_reset);
	}

	fj_wm_data->pin_res.gpio_state_hce_wakeup = pinctrl_lookup_state(fj_wm_data->pin_res.pinctrl,HCE_PINCTRL_STATE_WAKEUP);
	if (IS_ERR_OR_NULL(fj_wm_data->pin_res.gpio_state_hce_wakeup)) {
		printk(KERN_ERR "%s: can not get hce_wakeup pinstate\n", __func__);
		return PTR_ERR(fj_wm_data->pin_res.gpio_state_hce_wakeup);
	}

	fj_wm_data->pin_res.gpio_state_hce_irq = pinctrl_lookup_state(fj_wm_data->pin_res.pinctrl,HCE_PINCTRL_STATE_IRQ);
	if (IS_ERR_OR_NULL(fj_wm_data->pin_res.gpio_state_hce_irq)) {
		printk(KERN_ERR "%s: can not get hce_irq pinstate\n", __func__);
		return PTR_ERR(fj_wm_data->pin_res.gpio_state_hce_irq);
	}

	if(sar_ctl_flg) {
		fj_wm_data->pin_res.gpio_state_hce_sar = pinctrl_lookup_state(fj_wm_data->pin_res.pinctrl,HCE_PINCTRL_STATE_SAR);
		if (IS_ERR_OR_NULL(fj_wm_data->pin_res.gpio_state_hce_sar)) {
			printk(KERN_ERR "%s: can not get hce_sar pinstate\n", __func__);
			return PTR_ERR(fj_wm_data->pin_res.gpio_state_hce_sar);
		}
    }
    ret = fj_wm_hw_probe();

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (SW)fj_wm_sw_remove
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_sw_remove(void)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_hw_remove();

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (SW)fj_wm_sw_release
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_sw_release(void)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_hw_release();
    if (likely(ret == 0)) {
        fj_wm_data->state = FJ_WM_STS_INITIALIZER;
    }

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (SW)fj_wm_sw_ioct_initialize
 *
 * @return 0  Success
 *         !0 Fail
 */
static long fj_wm_sw_ioct_initialize(void)
{
    long lRet = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    wake_lock(&fj_wm_data->wake_lock);

    lRet = fj_wm_hw_hce_reset();
    if (likely(lRet == 0)) {
        fj_wm_data->state = FJ_WM_STS_MC_INITIALIZED;
    }

    wake_unlock(&fj_wm_data->wake_lock);

    DBG_LOG_WM_TRACE("%s : ret(%ld) end\n", __func__, lRet);
    return lRet;
}

/** (SW)fj_wm_sw_iocs_request_motion_irq
 *
 * @param value
 * @return 0  Success
 *         !0 Fail
 */
static long fj_wm_sw_iocs_request_motion_irq(unsigned int value)
{
    long lRet = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    if (likely(fj_wm_data->state == FJ_WM_STS_MC_INITIALIZED)) {
        lRet = fj_wm_hw_hce_request_irq(value);
    } else {
        lRet = -EINVAL;
    }

    DBG_LOG_WM_TRACE("%s : ret(%ld) end\n", __func__, lRet);
    return lRet;
}

/** (SW)fj_wm_sw_ioct_cancel_motion_irq
 *
 * @return 0  Success
 *         !0 Fail
 */
static long fj_wm_sw_ioct_cancel_motion_irq(void)
{
    long lRet = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    if (likely(fj_wm_data->state == FJ_WM_STS_MC_INITIALIZED)) {
        lRet = fj_wm_hw_hce_cancel_irq();
    } else {
        lRet = -EINVAL;
    }

    DBG_LOG_WM_TRACE("%s : ret(%ld) end\n", __func__, lRet);
    return lRet;
}

/** (SW)fj_wm_sw_iocs_wakeup_control
 *
 * @param value
 * @return 0  Success
 *         !0 Fail
 */
static long fj_wm_sw_iocs_wakeup_control(unsigned int value)
{
    long lRet = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    fj_wm_data->wakeup_flag = value;
    lRet = fj_wm_hw_hce_wakeup_set(value);

    DBG_LOG_WM_TRACE("%s : ret(%ld) end\n", __func__, lRet);
    return lRet;
}

/** (IF)fj_wm_if_init
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_if_init(void)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_sw_init();

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (IF)fj_wm_if_exit
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_if_exit(void)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_sw_exit();

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (IF)fj_wm_if_probe
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_if_probe(struct device *dev)
{
    int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);
    ret = fj_wm_sw_probe(dev);
    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (IF)fj_wm_if_remove
 *
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_if_remove(void)
{
   int ret = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_sw_remove();

    DBG_LOG_WM_TRACE("%s : ret(%d) end\n", __func__, ret);
    return ret;
}

/** (IF)fj_wm_if_open
 *
 * @param inode : Not use
 * @param file  : Not use
 * @return 0  Success
 *         !0 Fail
 */
static int fj_wm_if_open(struct inode *inode, struct file *file)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    if (unlikely(fj_wm_open_cnt > 0)) {
        printk(KERN_ERR "%s : open multiple error\n", __func__);
        return -EMFILE;
    } else {
        fj_wm_open_cnt++;
    }

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** (IF)fj_wm_if_release
 * @param inode : Not use
 * @param file  : Not use
 * @return 0  Success
 */
static int fj_wm_if_release(struct inode *inode, struct file *file)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    if (likely(fj_wm_open_cnt > 0)) {
        fj_wm_open_cnt--;
        fj_wm_sw_release();
    }

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** (IF)fj_wm_if_unlocked_ioctl
 *
 * @param file  : File descriptor
 * @param cmd   : Control command
 * @param arg   : Argument
 * @return 0  Success
 *         !0 Fail
 */
static long
fj_wm_if_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long lRet = 0;
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    mutex_lock(&fj_wm_data->io_lock);

    switch (cmd) {
    /** Initialize MC */
    case FJ_WM_IOCT_INITIALIZE:
        lRet = fj_wm_sw_ioct_initialize();
        break;
    /** Request motion IRQ */
    case FJ_WM_IOCS_REQUESTMOTIONIRQ:
        if (likely((arg == FJ_WM_EDGE_HIGH) || (arg == FJ_WM_EDGE_LOW))) {
            lRet = fj_wm_sw_iocs_request_motion_irq((unsigned int)arg);
        } else {
            printk(KERN_ERR "%s : invalid argument (arg : %ld)\n", __func__, arg);
            lRet = -EINVAL;
        }
        break;
    /** Cancel request motion IRQ */
    case FJ_WM_IOCT_CANCELMOTIONIRQ:
        lRet = fj_wm_sw_ioct_cancel_motion_irq();
        break;
    /* Wakeup */
    case FJ_WM_IOCS_WAKEUPCONTROL:
        if (likely((arg == FJ_WM_WAKEUP_HIGH) || (arg == FJ_WM_WAKEUP_LOW))) {
            if (fj_wm_data->prepare_flag) {
                printk(KERN_WARNING "%s : cmd=FJ_WM_IOCS_WAKEUPCONTROL arg=%ld  but kernel going suspend now. \n", __func__, arg);
            }
            lRet = fj_wm_sw_iocs_wakeup_control((unsigned int)arg);
        } else {
            printk(KERN_ERR "%s : invalid argument (arg : %ld)\n", __func__, arg);
            lRet = -EINVAL;
        }
        break;

    /* Set SAR */
    case FJ_WM_IOCT_SET_SAR_SEL:
        if(likely(sar_ctl_flg)){
			DBG_LOG_WM_DETAIL("+HCE: %s Set SAR arg(%ld)\n", __func__, arg);
			lRet = fj_wm_hw_hce_set_sar((int)arg);
                break;
		}

    default:
        printk(KERN_ERR "%s : invalid command (cmd : %d)\n", __func__, cmd);
        lRet = -EINVAL;
        break;
    }

    mutex_unlock(&fj_wm_data->io_lock);

    DBG_LOG_WM_TRACE("%s : ret(%ld) end\n", __func__, lRet);
    return lRet;
}

#ifdef CONFIG_COMPAT
static long fj_wm_if_compat_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    long lRet;
	DBG_LOG_WM_TRACE("%s : IN cmd=0x%x arg=%ld\n",__func__, cmd, arg);
	arg = (unsigned long)compat_ptr(arg);
	lRet = fj_wm_if_unlocked_ioctl(filep, cmd, arg);
	DBG_LOG_WM_TRACE("%s : OUT ret(%ld)\n",__func__, lRet);
	return lRet;
}
#endif	/* CONFIG_COMPAT */

/** (INIT)fj_wm_setup_cdev
 *
 * @param dev   : cdev
 * @param minor : minor
 * @param fops  : file operations
 * @return void
 */
static void
fj_wm_setup_cdev(struct cdev *dev, int minor,
        struct file_operations *fops)
{
    int err = 0;
    int devno = MKDEV(fj_wm_major, minor);
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    cdev_init(dev, fops);
    dev->owner = THIS_MODULE;
    dev->ops = fops;
    err = cdev_add(dev, devno, 1);
    /* Fail gracefully if need be */
    if (unlikely(err != 0))
        printk(KERN_ERR "%s : cdev_add failed(err : %d)\n", __func__, err);
    if (unlikely(IS_ERR(device_create(fj_wm_class, NULL, devno, NULL, DEV_NAME))))
        printk(KERN_ERR "%s : device_create failed\n", __func__);

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
}

/** Initialize file operations */
static struct file_operations fj_wm_fileops = {
    .owner              = THIS_MODULE,
    .open               = fj_wm_if_open,           /** open */
    .release            = fj_wm_if_release,        /** release */
#if CONFIG_COMPAT
    .compat_ioctl = fj_wm_if_compat_ioctl, /** compat ioctl */
#endif	/* CONFIG_COMPAT */
    .unlocked_ioctl     = fj_wm_if_unlocked_ioctl  /** ioctl */
};

atomic_t fj_wm_wq_count;
/** (INIT)fj_walkmotion_probe
 *
 * @param pdev : Not use
 * @return 0  Success
 *         !0 Fail
 */
static int
fj_walkmotion_probe(struct platform_device *pdev)
{
    int ret = 0;
    static struct input_dev *input_data = NULL;
    struct device_node *node;
    enum of_gpio_flags flag;
    int gpio_num;
    int motion_irq;
    int mc_init_delay;
    node = pdev->dev.of_node;

    DBG_LOG_WM_TRACE("%s : start\n", __func__);
    fj_wm_data = kzalloc(sizeof(*fj_wm_data), GFP_KERNEL);
    if (unlikely(fj_wm_data == 0)) {
        printk(KERN_ERR "%s : could not allocate memory\n", __func__);
        return -ENOMEM;
    }

    input_data = input_allocate_device();
    if (unlikely(input_data == 0)) {
        printk(KERN_ERR "%s : input_allocate_device failed\n", __func__);
        kfree(fj_wm_data);
        return -ENOMEM;
    }
    set_bit(EV_ABS, input_data->evbit);
    input_set_capability(input_data, EV_ABS, ABS_X);
    input_data->name = DRIVER_NAME;

    ret = input_register_device(input_data);
    if (unlikely(ret != 0)) {
        printk(KERN_ERR "%s : input_register_device failed\n", __func__);
        kfree(fj_wm_data);
        return ret;
    }

    fj_wm_this_data = input_data;

    ret = fj_wm_if_probe(&pdev->dev);
    if (unlikely(ret != 0)) {
        printk(KERN_ERR "%s : fj_wm_if_probe failed\n", __func__);
        input_unregister_device(input_data);
        kfree(fj_wm_data);
        return ret;
    }

	atomic_set(&fj_wm_wq_count, 0);
    fj_wm_wq = create_singlethread_workqueue("fj_wm_workq");
    if(unlikely(fj_wm_wq == NULL)) {
        printk(KERN_ERR "%s : couldn't create workqueue\n", __func__);
        fj_wm_if_remove();
        input_unregister_device(input_data);
        kfree(fj_wm_data);
        return -ENOMEM;
    }

    /* Initialize work queue */
    INIT_WORK(&fj_wm_work_queue_motion_irq, fj_wm_hw_motion_irq);
    /* Initialize wait queue */
    init_waitqueue_head(&fj_wm_data->wait_queue_motion_irq);

    /* Initialize */
    mutex_init(&fj_wm_data->io_lock);
    wake_lock_init(&fj_wm_data->wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

    ret = of_property_read_u32(node, "fj_wm_device,motion_irq", &motion_irq);
    if(unlikely(ret)) {
        printk(KERN_ERR "%s: of_property_read_u32(motion_irq)=%d\n", __func__, ret);
        return ret;
    }
	if(!motion_irq) {
		gpio_num = of_get_gpio_flags(node, DT_WM_GPIO_MOTION_IRQ, &flag);
		printk("%s : gpio(%d)\n", __func__, gpio_num);
		fj_wm_data->motion_irq = gpio_to_irq(gpio_num);
	}
	printk("%s : motion_irq(%d)\n", __func__, fj_wm_data->motion_irq);
    ret = of_property_read_u32(node, "fj_wm_device,mc_init_delay", &mc_init_delay);
    if(unlikely(ret)) {
        printk(KERN_ERR "%s: of_property_read_u32(mc_init_delay)=%d\n", __func__, ret);
        return ret;
    }
    if (mc_init_delay < 0) {
        fj_wm_data->mc_init_delay = FJ_WM_DEF_MC_INIT_DELAY;
    } else {
        fj_wm_data->mc_init_delay = mc_init_delay;
    }

    fj_wm_data->state = FJ_WM_STS_INITIALIZER;
    fj_wm_data->dbg_dev = &pdev->dev;
    fj_wm_data->irq_flag = 0;

	fj_wm_data->prepare_flag = 0;
	spin_lock_init( &fj_wm_data->spinlock );

    fj_wm_data->init_flag = 0;
    fj_wm_data->wakeup_flag = 0;

    fj_wm_open_cnt = 0;

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** (INIT)fj_walkmotion_remove
 *
 * @param pdev : not use
 * @return 0  success
 */
static int fj_walkmotion_remove(struct platform_device *pdev)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    fj_wm_if_remove();

    cancel_work_sync(&fj_wm_work_queue_motion_irq);
    destroy_workqueue(fj_wm_wq);

    if (likely(fj_wm_this_data != NULL)) {
        input_unregister_device(fj_wm_this_data);
    }
    kfree(fj_wm_data);

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return 0;
}

/** Prepare module
 *
 * @param pdev  : not use
 * @param state : not use
 * @return 0 success
 */
static int fj_walkmotion_prepare(struct device *pdev)
{
	unsigned long	irq_flags = 0;

	DBG_LOG_WM_TRACE("%s : start\n", __func__);
	DBG_LOG_WM_TRACE("%s : HCE's wakeup setting(%d)---0:WAKEUP, 1:SLEEP. \n", __func__, fj_wm_data->wakeup_flag);

	/* flg on */
	spin_lock_irqsave( &fj_wm_data->spinlock, irq_flags );
	fj_wm_data->prepare_flag = 1;
	spin_unlock_irqrestore( &fj_wm_data->spinlock, irq_flags );

	DBG_LOG_WM_TRACE("%s : end\n", __func__);
	return 0;
}

/** Complete module
 *
 * @param pdev  : not use
 * @return void
 */
static void fj_walkmotion_complete(struct device *pdev)
{
	unsigned long	irq_flags = 0;

	DBG_LOG_WM_TRACE("%s : start\n", __func__);

	/* flg off */
	spin_lock_irqsave( &fj_wm_data->spinlock, irq_flags );
	fj_wm_data->prepare_flag = 0;
	spin_unlock_irqrestore( &fj_wm_data->spinlock, irq_flags );

	DBG_LOG_WM_TRACE("%s : end\n", __func__);
}

/** Walk motion driver */
static const struct dev_pm_ops fj_walkmotion_pm_ops = {
	.prepare		= fj_walkmotion_prepare,
	.complete		= fj_walkmotion_complete,
};

static struct of_device_id walkmotion_match_table[] = {
	{	.compatible = "fj_wm_device", },
	{},
};

static struct platform_driver fj_walkmotion_driver = {
	.probe      = fj_walkmotion_probe,
	.remove     = fj_walkmotion_remove,
	.driver = {
                .name    = DRIVER_NAME,
                .owner   = THIS_MODULE,
		.pm		= &fj_walkmotion_pm_ops,
		.of_match_table = of_match_ptr(walkmotion_match_table),
	},
};

MODULE_DEVICE_TABLE(of, walkmotion_match_table);


/** (INIT)fj_walkmotion_init
 *
 * @return 0  Success
 *         !0 Fail
 */
static int __init fj_walkmotion_init(void)
{
    int ret = 0;
    dev_t dev = 0;
	unsigned char fj_wm_getNvData[APNV_SIZE_WM_DBG_MODE];

	memset(fj_wm_getNvData, 0x00, sizeof(fj_wm_getNvData));
	ret = get_nonvolatile(fj_wm_getNvData, APNV_WM_DBG_MODE, APNV_SIZE_WM_DBG_MODE);
	if (unlikely(ret < 0)) {
		printk(KERN_INFO "%s: get_nonvolatile() [ret=%d]\n", __func__, ret);
		fj_wm_getNvData[APNV_ADR_WM_DBG_MODE] = 0x00;
	}
	fj_wm_debug_mask =  fj_wm_getNvData[APNV_ADR_WM_DBG_MODE];

    DBG_LOG_WM_TRACE("%s : start\n", __func__);

    ret = fj_wm_if_init();
    if (unlikely(ret != 0)) {
        printk(KERN_ERR "%s : fj_wm_if_init error(err : %d )\n", __func__, ret);

        return ret;
    }

    fj_wm_class = class_create(THIS_MODULE, DEV_NAME);
    if (unlikely(IS_ERR(fj_wm_class))) {
        ret = PTR_ERR(fj_wm_class);
        printk(KERN_ERR "%s : class_create error(err : %d )\n", __func__, ret);

        return ret;
    }

    ret = alloc_chrdev_region(&dev, 0, 2, DEV_NAME);
    if (unlikely(ret < 0)) {
        printk(KERN_ERR "%s : Can't allocate chrdev region(ret : %d)\n",
                                                                __func__, ret);
        return ret;
    }

    fj_wm_major = MAJOR(dev);
    if (fj_wm_major == 0)
        fj_wm_major = ret;

    fj_wm_setup_cdev(&fj_wm_cdev, 0, &fj_wm_fileops);

    ret = platform_driver_register(&fj_walkmotion_driver);
    if (unlikely(ret != 0)) {
        printk(KERN_ERR "%s : platform_driver_register(err : %d )\n", __func__, ret);
        class_destroy(fj_wm_class);
        fj_wm_if_exit();
    }

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
    return ret;
}
module_init(fj_walkmotion_init);

/** (INIT)fj_walkmotion_exit
 *
 * @return void
 */
static void __exit fj_walkmotion_exit(void)
{
    DBG_LOG_WM_TRACE("%s : start\n", __func__);
    class_destroy(fj_wm_class);
    platform_driver_unregister(&fj_walkmotion_driver);

    fj_wm_if_exit();

    DBG_LOG_WM_TRACE("%s : end\n", __func__);
}
module_exit(fj_walkmotion_exit);

MODULE_ALIAS("platform:fj-walkmotion");
MODULE_AUTHOR("FUJITSU LIMITED");
MODULE_DESCRIPTION("Fujitsu Walk Motion MC Driver");
MODULE_LICENSE("GPL");
