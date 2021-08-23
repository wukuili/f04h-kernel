/*
 *  cxd224x-i2c.c - cxd224x NFC i2c driver
 *
 * Copyright (C) 2013-2015 Sony Corporation.
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2015-2016
//COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
/*----------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>

#include <linux/nfc/cxd224x.h>
#include <linux/wakelock.h>
/* FUJITSU LIMITED:2016-01-20 ADD-S */
#include <linux/compat.h>
#include <linux/of_gpio.h>
#include <linux/nonvolatile_common.h>
#include <linux/sched.h>

extern int makercmd_mode;      /* 1:MC mode 0:Normal mode */
extern int charging_mode;      /* 1:Off charging mode 0:Normal mode */

#define WKUP_MODE_MC         1
/* This is the master Users and Groups config for the platform. */
#define AID_ROOT             0  /* traditional unix root user */
#define AID_NFC           1027  /* nfc subsystem */
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

#define CXD224X_WAKE_LOCK_TIMEOUT	10		/* wake lock timeout for HOSTINT (sec) */
#define CXD224X_WAKE_LOCK_NAME	"cxd224x-i2c"		/* wake lock for HOSTINT */
#define CXD224X_WAKE_LOCK_TIMEOUT_LP	3		/* wake lock timeout for low-power-mode (sec) */
#define CXD224X_WAKE_LOCK_NAME_LP "cxd224x-i2c-lp"	/* wake lock for low-power-mode */

/* do not change below */
#define MAX_BUFFER_SIZE		780

/* Read data */
#define PACKET_HEADER_SIZE_NCI	(3)
#define PACKET_HEADER_SIZE_HCI	(3)
#define PACKET_TYPE_NCI		(16)
#define PACKET_TYPE_HCIEV	(4)
#define MAX_PACKET_SIZE		(PACKET_HEADER_SIZE_NCI + 255)

/* RESET */
#define RESET_ASSERT_MS         (1)

/* FUJITSU LIMITED:2016-01-20 ADD-S */
#define CXD224X_MINOR 254

/* non-volatile (debug log config) */
#define APNV_CXD224X_DBG_MODE			49073
#define APNV_SIZE_CXD224X_DBG_MODE			4
#define APNV_ADR_CXD224X_DBG_MODE			1

static uint8_t cxd224x_debug_mask;

/* for Attributes(Argument type) */
enum {
	DBG_CXD224X_TRACE       = 1 <<   0,
	DBG_CXD224X_DEBUG       = 1 <<   1,
	DBG_CXD224X_NCI         = 1 <<   2
};

#define	DBGLOG_TRACE(fmt, args...)	\
    do { if (cxd224x_debug_mask & DBG_CXD224X_TRACE) { \
		printk(KERN_INFO fmt, ## args); \
    } } while (0)

#define	DBGLOG_DETAIL(fmt, args...) \
    do { if (cxd224x_debug_mask & DBG_CXD224X_DEBUG) { \
		printk(KERN_DEBUG fmt, ## args); \
    } } while (0)

#define	DBGLOG_NCI(fmt, args...)	\
    do { if (cxd224x_debug_mask & DBG_CXD224X_NCI) { \
		printk(fmt, ## args); \
    } } while (0)
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

struct cxd224x_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice cxd224x_device;
	unsigned int en_gpio;
	unsigned int irq_gpio;
	unsigned int wake_gpio;
	unsigned int rst_gpio;
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	unsigned int hvdd_gpio;
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
	bool irq_enabled;
	struct mutex lock;
	spinlock_t irq_enabled_lock;
	unsigned int users;
	unsigned int count_irq;
	struct wake_lock wakelock;	/* wake lock for HOSTINT */
	struct wake_lock wakelock_lp;	/* wake lock for low-power-mode */
	/* Driver message queue */
	struct workqueue_struct	*wqueue;
	struct work_struct qmsg;
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default_int;
	struct pinctrl_state	*pin_default_wake;
	struct pinctrl_state	*pin_default_hvdd;
	struct pinctrl_state	*pin_default_rst;
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
};

#ifdef CONFIG_CXD224X_NFC_RST
static void cxd224x_workqueue(struct work_struct *work)
{
	struct cxd224x_dev *cxd224x_dev = container_of(work, struct cxd224x_dev, qmsg);
	unsigned long flags;
	DBGLOG_TRACE("[%s(),line(%d)] IN XRST value(%d)\n",__func__, __LINE__, gpio_get_value(cxd224x_dev->rst_gpio)); /* FUJITSU LIMITED:2016-01-20 ADD */

	dev_info(&cxd224x_dev->client->dev, "%s, xrst assert\n", __func__);
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	gpio_set_value(cxd224x_dev->rst_gpio, CXDNFC_RST_ACTIVE);
	cxd224x_dev->count_irq=0; /* clear irq */
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	msleep(RESET_ASSERT_MS);
	mdelay(RESET_ASSERT_MS);
/* FUJITSU LIMITED:2016-01-20 MOD-E   */
	dev_info(&cxd224x_dev->client->dev, "%s, xrst deassert\n", __func__);
	gpio_set_value(cxd224x_dev->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);
	DBGLOG_TRACE("[%s(),line(%d)] OUT XRST value(%d)\n",__func__, __LINE__, gpio_get_value(cxd224x_dev->rst_gpio)); /* FUJITSU LIMITED:2016-01-20 ADD */
}

static int __init init_wqueue(struct cxd224x_dev *cxd224x_dev)
{
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	INIT_WORK(&cxd224x_dev->qmsg, cxd224x_workqueue);
    cxd224x_dev->wqueue = create_singlethread_workqueue("cxd224x-i2c_wrokq");
	if (cxd224x_dev->wqueue == NULL)
		return -EBUSY;
	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	return 0;
}
#endif /* CONFIG_CXD224X_NFC_RST */

static void cxd224x_init_stat(struct cxd224x_dev *cxd224x_dev)
{
	cxd224x_dev->count_irq = 0;
}

static void cxd224x_disable_irq(struct cxd224x_dev *cxd224x_dev)
{
	unsigned long flags;
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (cxd224x_dev->irq_enabled) {
		disable_irq_nosync(cxd224x_dev->client->irq);
		cxd224x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
}

static void cxd224x_enable_irq(struct cxd224x_dev *cxd224x_dev)
{
	unsigned long flags;
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (!cxd224x_dev->irq_enabled) {
		cxd224x_dev->irq_enabled = true;
		enable_irq(cxd224x_dev->client->irq);
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
}

static irqreturn_t cxd224x_dev_irq_handler(int irq, void *dev_id)
{
	struct cxd224x_dev *cxd224x_dev = dev_id;
	unsigned long flags;
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	cxd224x_dev->count_irq++;
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
	wake_up(&cxd224x_dev->read_wq);
	DBGLOG_TRACE("[%s(),line(%d)] OUT cnt(%d)\n",__func__, __LINE__, cxd224x_dev->count_irq ); /* FUJITSU LIMITED:2016-01-20 ADD */
	return IRQ_HANDLED;
}

static unsigned int cxd224x_dev_poll(struct file *filp, poll_table *wait)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	poll_wait(filp, &cxd224x_dev->read_wq, wait);

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (cxd224x_dev->count_irq > 0)
	{
		cxd224x_dev->count_irq--;
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
	if(mask) 
		wake_lock_timeout(&cxd224x_dev->wakelock, CXD224X_WAKE_LOCK_TIMEOUT*HZ);

	DBGLOG_TRACE("[%s(),line(%d)] OUT, count_irq(%d) mask(0x%x)\n",__func__, __LINE__, cxd224x_dev->count_irq, mask); /* FUJITSU LIMITED:2016-01-20 ADD */
	return mask;
}

static ssize_t cxd224x_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	int total, len, ret;
	int total, len, ret, i;
/* FUJITSU LIMITED:2016-01-20 MOD-E */

	total = 0;
	len = 0;
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&cxd224x_dev->read_mutex);

/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	ret = i2c_master_recv(cxd224x_dev->client, tmp, 3);
//	if (ret == 3 && (tmp[0] != 0xff)) {
	ret = i2c_master_recv(cxd224x_dev->client, tmp, PACKET_HEADER_SIZE_NCI);
	if (ret != PACKET_HEADER_SIZE_NCI) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to read header %d\n", ret);
		mutex_unlock(&cxd224x_dev->read_mutex);
		return -EIO;
	}
	if (ret == PACKET_HEADER_SIZE_NCI && (tmp[0] != 0xff)) {
/* FUJITSU LIMITED:2016-01-20 MOD-E   */
		total = ret;

		len = tmp[PACKET_HEADER_SIZE_NCI-1];

		/** make sure full packet fits in the buffer
		**/
		if (len > 0 && (len + total) <= count) {
			/** read the remainder of the packet.
			**/
			ret = i2c_master_recv(cxd224x_dev->client, tmp+total, len);
/* FUJITSU LIMITED:2016-01-20 ADD-S */
			if (ret != len) {
				dev_err(&cxd224x_dev->client->dev,
					"failed to read payload %d\n", ret);
				mutex_unlock(&cxd224x_dev->read_mutex);
				return -EIO;
			}
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
			if (ret == len)
				total += len;
		}
	} 

	mutex_unlock(&cxd224x_dev->read_mutex);

	if (total > count || copy_to_user(buf, tmp, total)) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to copy to user space, total = %d\n", total);
		total = -EFAULT;
	}
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	DBGLOG_NCI("[%s(),line(%d)] total(%d), read(",__func__, __LINE__, total);
	for(i = 0; i < total; i ++) {
	    DBGLOG_NCI("0x%02x, ", (int)tmp[i]);
	}
	DBGLOG_NCI(")\n");
/* FUJITSU LIMITED:2016-01-20 ADD-E */

	DBGLOG_TRACE("[%s(),line(%d)] OUT total(%d)\n",__func__, __LINE__, total); /* FUJITSU LIMITED:2016-01-20 ADD */
	return total;
}

static ssize_t cxd224x_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	int ret;
	int ret,i;
/* FUJITSU LIMITED:2016-01-20 MOD-E */
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */

	if (count > MAX_BUFFER_SIZE) {
		dev_err(&cxd224x_dev->client->dev, "out of memory\n");
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to copy from user space\n");
		return -EFAULT;
	}

/* FUJITSU LIMITED:2016-01-20 ADD-S */
	DBGLOG_NCI("[%s(),line(%d)] count(%d), write(",__func__, __LINE__, (int)count);
	for(i = 0; i < (int)count; i ++) {
	    DBGLOG_NCI("0x%02x, ", (int)tmp[i]);
	}
	DBGLOG_NCI(")\n");
/* FUJITSU LIMITED:2016-01-20 ADD-E */

	mutex_lock(&cxd224x_dev->read_mutex);
	/* Write data */

	ret = i2c_master_send(cxd224x_dev->client, tmp, count);
	if (ret != count) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to write %d\n", ret);
		ret = -EIO;
	}
	mutex_unlock(&cxd224x_dev->read_mutex);

	DBGLOG_TRACE("[%s(),line(%d)] OUT ret(%d)\n",__func__, __LINE__, ret); /* FUJITSU LIMITED:2016-01-20 ADD */
	return ret;
}

static int cxd224x_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	uid_t uid;  /* FUJITSU LIMITED:2016-01-20 ADD */
	int call_enable = 0;
	struct cxd224x_dev *cxd224x_dev = container_of(filp->private_data,
							   struct cxd224x_dev,
							   cxd224x_device);
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */

/* FUJITSU LIMITED:2016-01-20 ADD-S */
	uid = __task_cred(current)->uid;
	DBGLOG_DETAIL("[%s(),line(%d)] uid(%d) makercmd_mode(%d)\n",__func__, __LINE__, uid, makercmd_mode);
	if (makercmd_mode != WKUP_MODE_MC) {
		/* Normal mode */
		if (uid != AID_NFC) {
			dev_err(&cxd224x_dev->client->dev,
				"open NG. UID = %d\n", uid);
			return -EPERM;
		}
	} else {
		/* MC mode */
		if (uid != AID_ROOT) {
			dev_err(&cxd224x_dev->client->dev,
				"open NG. UID = %d\n", uid);
			return -EPERM;
		}
	}
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

	filp->private_data = cxd224x_dev;
	mutex_lock(&cxd224x_dev->lock);
	if (!cxd224x_dev->users)
	{
		cxd224x_init_stat(cxd224x_dev);
		call_enable = 1;
	}
	cxd224x_dev->users++;
	mutex_unlock(&cxd224x_dev->lock);
	if (call_enable)
	{ /* FUJITSU LIMITED:2016-01-20 ADD */
		cxd224x_enable_irq(cxd224x_dev);
/* FUJITSU LIMITED:2016-01-20 ADD-S */
		enable_irq_wake(cxd224x_dev->client->irq);
	}
/* FUJITSU LIMITED:2016-01-20 ADD-E */

	dev_info(&cxd224x_dev->client->dev,
		 "open %d,%d users=%d\n", imajor(inode), iminor(inode), cxd224x_dev->users);

	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	return ret;
}

static int cxd224x_dev_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	int call_disable = 0;
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */

	mutex_lock(&cxd224x_dev->lock);
	cxd224x_dev->users--;
	if (!cxd224x_dev->users)
	{
		call_disable = 1;
	}
	mutex_unlock(&cxd224x_dev->lock);
	if (call_disable)
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	{
		disable_irq_wake(cxd224x_dev->client->irq);
/* FUJITSU LIMITED:2016-01-20 ADD-E */
		cxd224x_disable_irq(cxd224x_dev);
	} /* FUJITSU LIMITED:2016-01-20 ADD */

	dev_info(&cxd224x_dev->client->dev,
		 "release %d,%d users=%d\n", imajor(inode), iminor(inode), cxd224x_dev->users);

	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	return ret;
}

static long cxd224x_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;

	DBGLOG_TRACE("[%s(),line(%d)] IN cmd=0x%x arg=%lu\n",__func__, __LINE__, cmd, arg); /* FUJITSU LIMITED:2016-01-20 ADD */
	switch (cmd) {
	case CXDNFC_RST_CTL:
#ifdef CONFIG_CXD224X_NFC_RST
		dev_info(&cxd224x_dev->client->dev, "%s, rst arg=%d\n", __func__, (int)arg);
		return (queue_work(cxd224x_dev->wqueue, &cxd224x_dev->qmsg) ? 0 : 1);
#endif
		break;
	case CXDNFC_POWER_CTL:
#ifdef CONFIG_CXD224X_NFC_VEN
		if (arg == 0) {
			gpio_set_value(cxd224x_dev->en_gpio, 1);
		} else if (arg == 1) {
			gpio_set_value(cxd224x_dev->en_gpio, 0);  
		} else {
			/* do nothing */
		}
#endif
		break;
	case CXDNFC_WAKE_CTL:
		if (arg == 0) {
			wake_lock_timeout(&cxd224x_dev->wakelock_lp, CXD224X_WAKE_LOCK_TIMEOUT_LP*HZ);
			/* PON HIGH (normal power mode)*/
			gpio_set_value(cxd224x_dev->wake_gpio, 1);  
			DBGLOG_DETAIL("[%s(),line(%d)] PON HIGH wake value(%d)\n",__func__, __LINE__, gpio_get_value(cxd224x_dev->wake_gpio) ); /* FUJITSU LIMITED:2016-01-20 ADD */
		} else if (arg == 1) {
			/* PON LOW (low power mode) */
			gpio_set_value(cxd224x_dev->wake_gpio, 0);
/* FUJITSU LIMITED:2016-01-20 ADD-S */
			DBGLOG_DETAIL("[%s(),line(%d)] PON LOW wake value(%d)\n",__func__, __LINE__, gpio_get_value(cxd224x_dev->wake_gpio) );
			wake_unlock(&cxd224x_dev->wakelock);
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
			wake_unlock(&cxd224x_dev->wakelock_lp);
		} else {
			/* do nothing */
		}
		break;
	default:
		dev_err(&cxd224x_dev->client->dev,
			"%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
		return 0;
	}

	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	return 0;
}

/* FUJITSU LIMITED:2016-01-20 ADD-S */
#ifdef CONFIG_COMPAT
static long cxd224x_dev_compat_unlocked_ioctl(struct file *filep,
					 unsigned int cmd, unsigned long arg)
{
	long ret;

	DBGLOG_TRACE("[%s(),line(%d)] IN cmd=0x%x arg=%lu\n",__func__, __LINE__, cmd, arg);

	arg = (unsigned long)compat_ptr(arg);
	ret = cxd224x_dev_unlocked_ioctl(filep, cmd, arg);
	
	DBGLOG_TRACE("[%s(),line(%d)] OUT ret(%ld)\n",__func__, __LINE__, ret);
	return ret;
}
#endif	/* CONFIG_COMPAT */
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

static const struct file_operations cxd224x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = cxd224x_dev_poll,
	.read = cxd224x_dev_read,
	.write = cxd224x_dev_write,
	.open = cxd224x_dev_open,
	.release = cxd224x_dev_release,
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	.unlocked_ioctl = cxd224x_dev_unlocked_ioctl
	.unlocked_ioctl = cxd224x_dev_unlocked_ioctl,
/* FUJITSU LIMITED:2016-01-20 MOD-E */
/* FUJITSU LIMITED:2016-01-20 ADD-S */
#ifdef CONFIG_COMPAT
	.compat_ioctl = cxd224x_dev_compat_unlocked_ioctl,
#endif  /* CONFIG_COMPAT */
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
};

/* FUJITSU LIMITED:2016-01-20 ADD-S */
static struct cxd224x_platform_data *cxd224x_get_dts_config(struct device *dev)
{
	int ret;
	struct device_node *node = dev->of_node;
	struct cxd224x_platform_data *pdata;

	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "unable to allocate memory for platform data\n");
		ret = -ENOMEM;
		goto err;
	}
	pdata->irq_gpio = of_get_named_gpio(node, "cxd224x,irq_gpio", 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		ret = -EINVAL;
		dev_err(dev,
			"failed to of_get_named_gpio(cxd224x,irq_gpio(%d))\n", pdata->irq_gpio);
		goto err;
	}
	pdata->wake_gpio = of_get_named_gpio(node, "cxd224x,wake_gpio", 0);
	if (!gpio_is_valid(pdata->wake_gpio)) {
		ret = -EINVAL;
		dev_err(dev,
			"failed to of_get_named_gpio(cxd224x,wake_gpio(%d))\n",pdata->wake_gpio);
		goto err;
	}
	
	pdata->hvdd_gpio = of_get_named_gpio(node, "cxd224x,hvdd_gpio", 0);
	if (!gpio_is_valid(pdata->hvdd_gpio)) {
		ret = -EINVAL;
		dev_err(dev,
			"failed to of_get_named_gpio(cxd224x,hvdd_gpio(%d))\n",pdata->hvdd_gpio);
		goto err;
	}

	pdata->rst_gpio = of_get_named_gpio(node, "cxd224x,rst_gpio", 0);
	if (!gpio_is_valid(pdata->rst_gpio)) {
		ret = -EINVAL;
		dev_err(dev,
			"failed to of_get_named_gpio(cxd224x,rst_gpio(%d))\n",pdata->rst_gpio);
		goto err;
	}

	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl)) {
		ret = -EINVAL;
		dev_err(dev, "Failed to get pinctrl\n");
		goto err;
	}
	
	pdata->pin_default_int = pinctrl_lookup_state(pdata->pinctrl, "cxd224x_default_int");
	if (IS_ERR_OR_NULL(pdata->pin_default_int)) {
		ret = -EINVAL;
		dev_err(dev, "Failed to look up default state\n");
		goto err;
	}

	pdata->pin_default_wake = pinctrl_lookup_state(pdata->pinctrl, "cxd224x_default_wake");
	if (IS_ERR_OR_NULL(pdata->pin_default_wake)) {
		ret = -EINVAL;
		dev_err(dev, "Failed to look up default state\n");
		goto err;
	}

	pdata->pin_default_hvdd = pinctrl_lookup_state(pdata->pinctrl, "cxd224x_default_hvdd");
	if (IS_ERR_OR_NULL(pdata->pin_default_hvdd)) {
		ret = -EINVAL;
		dev_err(dev, "Failed to look up default state\n");
		goto err;
	}

	pdata->pin_default_rst = pinctrl_lookup_state(pdata->pinctrl, "cxd224x_default_rst");
	/* FCNT LIMITED:2016-06-27 ADRV mod start */
	if (IS_ERR_OR_NULL(pdata->pin_default_rst)) {
	/* FCNT LIMITED:2016-06-27 ADRV mod end */
		ret = -EINVAL;
		dev_err(dev, "Failed to look up default state\n");
		goto err;
	}

	DBGLOG_DETAIL("[%s(),line(%d)] pdata->irq_gpio(%d)\n",__func__, __LINE__, pdata->irq_gpio);
	DBGLOG_DETAIL("[%s(),line(%d)] pdata->wake_gpio(%d)\n",__func__, __LINE__, pdata->wake_gpio);
	DBGLOG_DETAIL("[%s(),line(%d)] pdata->hvdd_gpio(%d)\n",__func__, __LINE__, pdata->hvdd_gpio);
	DBGLOG_DETAIL("[%s(),line(%d)] pdata->rst_gpio(%d)\n",__func__, __LINE__, pdata->rst_gpio);
	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__);

	return pdata;
err:
	if (pdata)
		devm_kfree(dev, pdata);

	return ERR_PTR(ret);
}
/* FUJITSU LIMITED:2016-01-20 ADD-E */

static int cxd224x_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	struct cxd224x_platform_data *platform_data;
	struct cxd224x_dev *cxd224x_dev;
	int irq_gpio_ok  = 0;
#ifdef CONFIG_CXD224X_NFC_VEN
	int en_gpio_ok   = 0;
#endif
#ifdef CONFIG_CXD224X_NFC_RST
	int rst_gpio_ok = 0;
#endif
	int wake_gpio_ok = 0;
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	int hvdd_gpio_ok = 0;

	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__);
	DBGLOG_DETAIL("[%s(),line(%d)] client->name(%s) client->addr(0x%x)\n",
		__func__, __LINE__, client->name, (int)client->addr);
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

	platform_data = client->dev.platform_data;

/* FUJITSU LIMITED:2016-01-20 ADD-S */	
	if (client->dev.of_node) {
		platform_data = cxd224x_get_dts_config(&client->dev);
		if (IS_ERR(platform_data)) {
			ret =  PTR_ERR(platform_data);
			dev_err(&client->dev, "failed to get_dts_config %d\n", ret);
			goto err_exit;
		}
		client->dev.platform_data = platform_data;
	}
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

	dev_info(&client->dev, "%s, probing cxd224x driver flags = %x\n", __func__, client->flags);
	if (platform_data == NULL) {
		dev_err(&client->dev, "nfc probe fail\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

/* FUJITSU LIMITED:2016-01-20 ADD-S */
	if(charging_mode) {
		dev_info(&client->dev, "%s, off charging hvdd low\n", __func__);
		gpio_set_value(platform_data->hvdd_gpio, 0);
		return 0;
	}
	ret = pinctrl_select_state(platform_data->pinctrl, platform_data->pin_default_hvdd);
	if (ret) {
		dev_err(&client->dev, "Can't select pinctrl state ret(%d)\n", ret);
		return -ENODEV;
	}

	ret = gpio_request_one(platform_data->hvdd_gpio, GPIOF_OUT_INIT_HIGH, "nfc_hvdd");
	if (ret)
		return -ENODEV;
	hvdd_gpio_ok=1;
	gpio_set_value(platform_data->hvdd_gpio, 1);
	dev_info(&client->dev, "%s, hvdd high\n", __func__);
	msleep(10);

	ret = pinctrl_select_state(platform_data->pinctrl, platform_data->pin_default_int);
	if (ret) {
		dev_err(&client->dev, "Can't select pinctrl state ret(%d)\n", ret);
		return -ENODEV;
	}
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

	ret = gpio_request_one(platform_data->irq_gpio, GPIOF_IN, "nfc_int");
	if (ret)
		return -ENODEV;
	irq_gpio_ok=1;

#ifdef CONFIG_CXD224X_NFC_VEN
	ret = gpio_request_one(platform_data->en_gpio, GPIOF_OUT_INIT_LOW, "nfc_cen");
	if (ret)
		goto err_exit;
	en_gpio_ok=1;
	gpio_set_value(platform_data->en_gpio, 0);
#endif

#ifdef CONFIG_CXD224X_NFC_RST
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	ret = gpio_request_one(platform_data->rst_gpio, GPIOF_OUT_INIT_HIGH, "nfc_rst");
	ret = pinctrl_select_state(platform_data->pinctrl, platform_data->pin_default_rst);
	if (ret) {
		dev_err(&client->dev, "Can't select pinctrl state ret(%d)\n", ret);
		return -ENODEV;
	}
	ret = gpio_request_one(platform_data->rst_gpio, GPIOF_OUT_INIT_LOW, "nfc_rst");
/* FUJITSU LIMITED:2016-01-20 MOD-E   */
	if (ret)
		goto err_exit;
	rst_gpio_ok=1;
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	dev_info(&client->dev, "%s, xrst assert\n", __func__);
	gpio_set_value(platform_data->rst_gpio, CXDNFC_RST_ACTIVE);
	mdelay(RESET_ASSERT_MS);
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
	dev_info(&client->dev, "%s, xrst deassert\n", __func__);
	gpio_set_value(platform_data->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);
#endif

/* FUJITSU LIMITED:2016-01-20 ADD-S */
	ret = pinctrl_select_state(platform_data->pinctrl, platform_data->pin_default_wake);
	if (ret) {
		dev_err(&client->dev, "probing cxd224x driver flagsCan't select pinctrl state(%d)\n", ret);
		return ret;
	}
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	ret = gpio_request_one(platform_data->wake_gpio, GPIOF_OUT_INIT_HIGH, "nfc_wake");
//	if (ret)
//		goto err_exit;
//	wake_gpio_ok=1;
//	gpio_set_value(platform_data->wake_gpio, 1);
	ret = gpio_request_one(platform_data->wake_gpio, GPIOF_OUT_INIT_LOW, "nfc_wake");
	if (ret)
		goto err_exit;
	wake_gpio_ok=1;
	gpio_set_value(platform_data->wake_gpio, 0);
/* FUJITSU LIMITED:2016-01-20 MOD-E   */

	cxd224x_dev = kzalloc(sizeof(*cxd224x_dev), GFP_KERNEL);
	if (cxd224x_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	cxd224x_dev->irq_gpio = platform_data->irq_gpio;
	cxd224x_dev->en_gpio = platform_data->en_gpio;
	cxd224x_dev->wake_gpio = platform_data->wake_gpio;
	cxd224x_dev->rst_gpio = platform_data->rst_gpio;
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	cxd224x_dev->hvdd_gpio = platform_data->hvdd_gpio;
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
	cxd224x_dev->client = client;
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	cxd224x_dev->pinctrl = platform_data->pinctrl;
	cxd224x_dev->pin_default_int = platform_data->pin_default_int;
	cxd224x_dev->pin_default_wake = platform_data->pin_default_wake;
	cxd224x_dev->pin_default_hvdd = platform_data->pin_default_hvdd;
	cxd224x_dev->pin_default_rst = platform_data->pin_default_rst;
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
	wake_lock_init(&cxd224x_dev->wakelock, WAKE_LOCK_SUSPEND, CXD224X_WAKE_LOCK_NAME);
	wake_lock_init(&cxd224x_dev->wakelock_lp, WAKE_LOCK_SUSPEND, CXD224X_WAKE_LOCK_NAME_LP);
	cxd224x_dev->users =0;

	/* init mutex and queues */
	init_waitqueue_head(&cxd224x_dev->read_wq);
	mutex_init(&cxd224x_dev->read_mutex);
	mutex_init(&cxd224x_dev->lock);
	spin_lock_init(&cxd224x_dev->irq_enabled_lock);

#ifdef CONFIG_CXD224X_NFC_RST
	if (init_wqueue(cxd224x_dev) != 0) {
		dev_err(&client->dev, "init workqueue failed\n");
		goto err_exit;
	}
#endif

/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	cxd224x_dev->cxd224x_device.minor = MISC_DYNAMIC_MINOR;
	cxd224x_dev->cxd224x_device.minor = CXD224X_MINOR;
/* FUJITSU LIMITED:2016-01-20 MOD-E   */
	cxd224x_dev->cxd224x_device.name = "cxd224x-i2c";
	cxd224x_dev->cxd224x_device.fops = &cxd224x_dev_fops;

	ret = misc_register(&cxd224x_dev->cxd224x_device);
	if (ret) {
		dev_err(&client->dev, "misc_register failed\n");
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	dev_info(&client->dev, "requesting IRQ %d\n", client->irq);
	cxd224x_dev->irq_enabled = true;
	ret = request_irq(client->irq, cxd224x_dev_irq_handler,
			  IRQF_TRIGGER_FALLING, client->name, cxd224x_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	cxd224x_disable_irq(cxd224x_dev);
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	DBGLOG_DETAIL("[%s(),line(%d)]  hvdd value(%d)\n",__func__, __LINE__, gpio_get_value(platform_data->hvdd_gpio) );
	DBGLOG_DETAIL("[%s(),line(%d)]  hostint value(%d)\n",__func__, __LINE__, gpio_get_value(platform_data->irq_gpio) );
	DBGLOG_DETAIL("[%s(),line(%d)]  wake value(%d)\n",__func__, __LINE__, gpio_get_value(platform_data->wake_gpio) );
	DBGLOG_DETAIL("[%s(),line(%d)]  rst value(%d)\n",__func__, __LINE__, gpio_get_value(platform_data->rst_gpio) );
/* FUJITSU LIMITED:2016-01-20 ADD-E */
	i2c_set_clientdata(client, cxd224x_dev);
	dev_info(&client->dev,
		 "%s, probing cxd224x driver exited successfully\n",
		 __func__);
	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	return 0;

err_request_irq_failed:
	misc_deregister(&cxd224x_dev->cxd224x_device);
err_misc_register:
	mutex_destroy(&cxd224x_dev->read_mutex);
	kfree(cxd224x_dev);
err_exit:
	if(irq_gpio_ok)
		gpio_free(platform_data->irq_gpio);
#ifdef CONFIG_CXD224X_NFC_VEN
	if(en_gpio_ok)
		gpio_free(platform_data->en_gpio);
#endif
#ifdef CONFIG_CXD224X_NFC_RST
	if(rst_gpio_ok)
		gpio_free(platform_data->rst_gpio);
#endif
	if(wake_gpio_ok)
		gpio_free(platform_data->wake_gpio);
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	if(hvdd_gpio_ok)
		gpio_free(platform_data->hvdd_gpio);
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

	return ret;
}

static int cxd224x_remove(struct i2c_client *client)
{
	struct cxd224x_dev *cxd224x_dev;
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */

	cxd224x_dev = i2c_get_clientdata(client);
	wake_lock_destroy(&cxd224x_dev->wakelock);
	wake_lock_destroy(&cxd224x_dev->wakelock_lp);
	free_irq(client->irq, cxd224x_dev);
	misc_deregister(&cxd224x_dev->cxd224x_device);
	mutex_destroy(&cxd224x_dev->read_mutex);
	gpio_free(cxd224x_dev->irq_gpio);
	gpio_free(cxd224x_dev->en_gpio);
	gpio_free(cxd224x_dev->wake_gpio);
/* FUJITSU LIMITED:2016-01-20 ADD-S */
	gpio_free(cxd224x_dev->hvdd_gpio);
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
	kfree(cxd224x_dev);

	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	return 0;
}

/* FUJITSU LIMITED:2016-01-20 DEL-S */
//#ifdef CONFIG_PM
//static int cxd224x_suspend(struct device *dev)
//{
//	struct platform_device *pdev = to_platform_device(dev);
//	struct cxd224x_platform_data *platform_data = pdev->dev.platform_data;
//
//	if (device_may_wakeup(&pdev->dev)) {
//		int irq = gpio_to_irq(platform_data->irq_gpio);
//		enable_irq_wake(irq);
//	}
//	return 0;
//}
//
//static int cxd224x_resume(struct device *dev)
//{
//	struct platform_device *pdev = to_platform_device(dev);
//	struct cxd224x_platform_data *platform_data = pdev->dev.platform_data;
//
//	if (device_may_wakeup(&pdev->dev)) {
//		int irq = gpio_to_irq(platform_data->irq_gpio);
//		disable_irq_wake(irq);
//	}
//	return 0;
//}
//
//static const struct dev_pm_ops cxd224x_pm_ops = {
//	.suspend	= cxd224x_suspend,
//	.resume		= cxd224x_resume,
//};
//#endif
/* FUJITSU LIMITED:2016-01-20 DEL-E */

static const struct i2c_device_id cxd224x_id[] = {
	{"cxd224x-i2c", 0},
	{}
};

/* FUJITSU LIMITED:2016-01-20 ADD-S */
static struct of_device_id cxd224x_match_table[] = {
	{ .compatible = "cxd224x-i2c", },
	{ },
};
/* FUJITSU LIMITED:2016-01-20 ADD-E   */

static struct i2c_driver cxd224x_driver = {
	.id_table = cxd224x_id,
	.probe = cxd224x_probe,
	.remove = cxd224x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cxd224x-i2c",
/* FUJITSU LIMITED:2016-01-20 DEL-S */
//#ifdef CONFIG_PM
//		.pm	= &cxd224x_pm_ops,
//#endif
/* FUJITSU LIMITED:2016-01-20 DEL-E */
/* FUJITSU LIMITED:2016-01-20 ADD-S */
		.of_match_table = cxd224x_match_table,
/* FUJITSU LIMITED:2016-01-20 ADD-E   */
	},
};

/*
 * module load/unload record keeping
 */

static int __init cxd224x_dev_init(void)
{
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//	return i2c_add_driver(&cxd224x_driver);
	int ret;
	unsigned char cxd224x_getNvData[APNV_SIZE_CXD224X_DBG_MODE];

	memset(cxd224x_getNvData, 0x00, sizeof(cxd224x_getNvData));
	ret = get_nonvolatile(cxd224x_getNvData, APNV_CXD224X_DBG_MODE, APNV_SIZE_CXD224X_DBG_MODE);
	if (ret < 0) {
		printk(KERN_INFO "%s:get_nonvolatile() [ret=%d]\n", __func__, ret);
		cxd224x_getNvData[APNV_ADR_CXD224X_DBG_MODE] = 0x00;
	}
	cxd224x_debug_mask =  cxd224x_getNvData[APNV_ADR_CXD224X_DBG_MODE];

	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__);
	ret = i2c_add_driver(&cxd224x_driver);

	DBGLOG_TRACE("[%s(),line(%d)] OUT ret(%d)\n",__func__, __LINE__, ret);
	return ret;
/* FUJITSU LIMITED:2016-01-20 MOD-E   */
}
module_init(cxd224x_dev_init);

static void __exit cxd224x_dev_exit(void)
{
	DBGLOG_TRACE("[%s(),line(%d)] IN\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
	i2c_del_driver(&cxd224x_driver);
	DBGLOG_TRACE("[%s(),line(%d)] OUT\n",__func__, __LINE__); /* FUJITSU LIMITED:2016-01-20 ADD */
}
module_exit(cxd224x_dev_exit);

MODULE_AUTHOR("Sony");
MODULE_DESCRIPTION("NFC cxd224x driver");
 /* FUJITSU LIMITED:2016-01-20 MOD-S */
//MODULE_LICENSE("GPL");
MODULE_LICENSE("GPL v2");
 /* FUJITSU LIMITED:2016-01-20 MOD-E */
