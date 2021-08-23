/*
 * haudsig.c
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2014-2015
/*----------------------------------------------------------------------------*/
/* H-FAUDIO-000015000 start */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

/* N-FAUDIO-151064001 */
#include <linux/compat.h>
/* N-FAUDIO-151064001   */

#include <linux/haudsig.h>

#define N  (1 << 2)  /* Number of receiving buffers. This have to be power of 2. See M. */
#define M  (N - 1) /* Mask value of ring buffer index. */
static struct {
	spinlock_t lock; /* To protect values of the struct. */
	wait_queue_head_t wait; /* wait by read function and waked up by recieving data */
	int depth; /* Incremented when opened. Decrimnted when released. */
	int top; /* Index of the top of the stored data of the ring buffer. */
	int cnt; /* Number of the stored data of the ring buffer. */
	int eof; /* Indicate ioctl(SET_EOF) is called. */
	int pend; /* Indicate there is a pending read operation which can be retried. */
	char data[N][8]; /* The receiving ring buffer. See top and cnt. */
} haudsig_priv;

/* external control */
void haudsig_wakeup(const char* data) {
	int of = 0,wu = 0,depth;
	unsigned long flags;
	spin_lock_irqsave(&haudsig_priv.lock, flags);
	if (haudsig_priv.depth) {
		memcpy(haudsig_priv.data[(haudsig_priv.top + haudsig_priv.cnt) & M],data,sizeof haudsig_priv.data[0]);
		if (haudsig_priv.cnt < N) {
			haudsig_priv.cnt++;
		} else {
			haudsig_priv.top = (haudsig_priv.top + 1) & M;
			of = 1;
		}
		wu = 1;
	}
	depth = haudsig_priv.depth;
	spin_unlock_irqrestore(&haudsig_priv.lock, flags);
	pr_debug("%s() depth=%d\n", __func__,depth);
	if (of) {
		pr_err("%s():Overflow\n", __func__);
	}
	if (wu) {
		wake_up(&haudsig_priv.wait);
	}
}

static int haudsig_open(struct inode *inode, struct file *filp) {
	int depth;
	unsigned long flags;
	spin_lock_irqsave(&haudsig_priv.lock, flags);
	if (!haudsig_priv.depth) {
		haudsig_priv.top = 0;
		haudsig_priv.cnt = 0;
		haudsig_priv.eof = 0;
		haudsig_priv.pend = 0;
	}
	depth = ++haudsig_priv.depth;
	spin_unlock_irqrestore(&haudsig_priv.lock, flags);
	pr_debug("%s() #client=%d\n", __func__,depth);
	return 0;
}

static int haudsig_release(struct inode *inode, struct file *filp)
{
	int depth;
	unsigned long flags;
	spin_lock_irqsave(&haudsig_priv.lock, flags);
	if (haudsig_priv.depth && --haudsig_priv.depth == 0) {
	}
	depth = haudsig_priv.depth;
	spin_unlock_irqrestore(&haudsig_priv.lock, flags);
	pr_debug("%s() #client=%d\n", __func__,depth);
	return 0;
}

static ssize_t haudsig_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	int ret,top,cnt;
	unsigned long flags;

	pr_debug("%s() wait start\n", __func__);
	if (!buf || count < sizeof haudsig_priv.data[0]) {
		pr_err("%s():EINVAL\n", __func__);
		return -EINVAL;
	}

	do {
		ret = wait_event_interruptible(haudsig_priv.wait,haudsig_priv.cnt || haudsig_priv.eof);
		pr_debug("%s() wait end ret=%d\n", __func__,ret);
		if (ret) {
			pr_err("%s():ERROR EINTR\n", __func__);
			return -EINTR;
		}

		if (haudsig_priv.eof) {
			pr_debug("%s() EOF\n", __func__);
			return 0;
		}

		spin_lock_irqsave(&haudsig_priv.lock, flags);
		top = haudsig_priv.top;
		cnt = haudsig_priv.cnt;
		if (cnt) {
			ret = copy_to_user(buf,haudsig_priv.data[haudsig_priv.top],sizeof haudsig_priv.data[0]);
			if (ret) {
				ret = -EFAULT;
				if (!haudsig_priv.pend) {
					haudsig_priv.pend = 1;
				} else {
					haudsig_priv.pend = 0;
					haudsig_priv.top = (haudsig_priv.top + 1) & M;
					haudsig_priv.cnt--;
				}
			} else {
				ret = sizeof haudsig_priv.data[0];
				haudsig_priv.pend = 0;
				haudsig_priv.top = (haudsig_priv.top + 1) & M;
				haudsig_priv.cnt--;
			}
		}
		spin_unlock_irqrestore(&haudsig_priv.lock, flags);
	} while (!cnt);

	if (ret == -EFAULT) {
		pr_err("%s():EFAULT, top=%d, cnt=%d\n", __func__, top, cnt);
	} else {
		pr_debug("%s() succeeded, top=%d, cnt=%d\n", __func__, top, cnt);
	}
	return ret;
}

static long haudsig_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case HAUDSIG_IOCTL_SET_EOF:
		haudsig_priv.eof = 1;
		wake_up(&haudsig_priv.wait);
		pr_debug("%s() SET_EOF succeeded\n", __func__);
		ret = 0;
		break;
	default:
		pr_err("%s():ERROR EINVAL cmd=%d\n", __func__,cmd);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/* N-FAUDIO-151064001 */
#ifdef CONFIG_COMPAT
static long haudsig_compat_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    long ret;
	pr_debug("%s : IN cmd=0x%x arg=%ld\n",__func__, cmd, arg);
	arg = (unsigned long)compat_ptr(arg);
	ret = haudsig_ioctl(filep, cmd, arg);
	pr_debug("%s : OUT ret(%ld)\n",__func__, ret);
	return ret;
}
#endif	/* CONFIG_COMPAT */
/* N-FAUDIO-151064001 */

static struct file_operations haudsig_fops = {
	.owner          = THIS_MODULE,
	.open           = haudsig_open,
	.release        = haudsig_release,
	.read           = haudsig_read,
	/* N-FAUDIO-151064001 */
#if CONFIG_COMPAT
    .compat_ioctl = haudsig_compat_ioctl, /** compat ioctl */
#endif	/* CONFIG_COMPAT */
	/* N-FAUDIO-151064001 */
	.unlocked_ioctl = haudsig_ioctl,
};

static struct miscdevice haudsig_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "haudsig",
	.fops = &haudsig_fops,
};

static int __init haudsig_init(void)
{
	int ret;

	spin_lock_init(&haudsig_priv.lock);
	init_waitqueue_head(&haudsig_priv.wait);
	haudsig_priv.depth = 0;

	ret = misc_register(&haudsig_dev);
	if (ret) {
		pr_err("%s() fail to misc_register()\n",__func__);
		return ret;
	}
	pr_info("%s() succeeded\n",__func__); 
	return 0;
}
module_init(haudsig_init);

static void __exit haudsig_exit(void)
{
	misc_deregister(&haudsig_dev);

	pr_info("%s() succeeded\n",__func__); 
}
module_exit(haudsig_exit);

MODULE_DESCRIPTION("Fujitsu HCE Auditory Signal Driver");
MODULE_AUTHOR("FUJITSU LIMITED");
MODULE_LICENSE("GPL");

/* H-FAUDIO-000015000 end */
