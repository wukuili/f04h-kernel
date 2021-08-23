/* H-FAUDIO-000026000 start */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2014
/*----------------------------------------------------------------------------*/
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/wait.h>

#include <linux/msm_audio.h>

bool lastcall_pl_flg = false;
u32 lastcall_pl_ms = 0;

static long speakerrecv_pl_ioctl(struct file *file,
				 unsigned int cmd, unsigned long arg)
{
	struct timeval lastcall_pl_tv;

	switch (cmd) {
	case AUDIO_SET_CALL_START:
		lastcall_pl_flg = true;
		break;
	case AUDIO_SET_CALL_STOP:
		lastcall_pl_flg = false;
		do_gettimeofday(&lastcall_pl_tv);
		lastcall_pl_ms = (lastcall_pl_tv.tv_sec * 1000) + (lastcall_pl_tv.tv_usec / 1000);
		break;
	}
	return 0;
}

static int speakerrecv_pl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int speakerrecv_pl_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* N-FAUDIO-151064002 start */
#ifdef CONFIG_COMPAT
static long speakerrecv_pl_compat_ioctl(struct file *file,
				 unsigned int cmd, unsigned long arg)
{
    long lRet;
	pr_debug("%s : IN cmd=0x%x arg=%ld\n",__func__, cmd, arg);
	arg = (unsigned long)compat_ptr(arg);
	lRet = speakerrecv_pl_ioctl(file, cmd, arg);
	pr_debug("%s : OUT ret(%ld)\n",__func__, lRet);
	return lRet;
}
#endif	/* CONFIG_COMPAT */
/* N-FAUDIO-151064002 end */

static const struct file_operations speakerrecv_pl_fops = {
	.owner		= THIS_MODULE,
	.open		= speakerrecv_pl_open,
	.release	= speakerrecv_pl_release,
/* N-FAUDIO-151064002 start */
#if CONFIG_COMPAT
    .compat_ioctl = speakerrecv_pl_compat_ioctl, /** compat ioctl */
#endif	/* CONFIG_COMPAT */
/* N-FAUDIO-151064002 end */
	.unlocked_ioctl	= speakerrecv_pl_ioctl,
};

struct miscdevice speakerrecv_pl_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_snd_speakerrecv",
	.fops	= &speakerrecv_pl_fops,
};

static int __init speakerrecv_pl_init(void)
{
	return misc_register(&speakerrecv_pl_misc);
}

device_initcall(speakerrecv_pl_init);

/* H-FAUDIO-000026000 end */
