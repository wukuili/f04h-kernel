/*
 * fj_custom.c
 *
 * Copyright(C) 2014-2015 FUJITSU LIMITED
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

//==============================================================================
// include file
//==============================================================================
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
/* FUJITSU LIMITED:2015-04-15 H1520315 add start */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
/* FUJITSU LIMITED:2015-04-15 H1520315 add end   */
/* FUJITSU LIMITED:2014-10-22 H1510008 add start */
#include <linux/fj_mode.h>
/* FUJITSU LIMITED:2014-10-22 H1510008 add end */
/* FUJITSU LIMITED:2015-05-28 H1520369 BTO add start */
#include <linux/fj_bto.h>
/* FUJITSU LIMITED:2015-05-28 H1520369 BTO add end */
/* FUJITSU LIMITED:2014-11-25 H1510020 add start */
#include <linux/timer.h>
#include <linux/fb.h>
#include <linux/fj_mode.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/gpio_keys.h>

/* FUJITSU LIMITED:2014-11-25 H1510020 add end   */
/* FUJITSU LIMITED:2015-04-15 H1520304 add start */
#include <linux/err.h>
#include <linux/io.h>
#include <soc/qcom/smem.h>
/* FUJITSU LIMITED:2015-04-15 H1520304 add end */

/* FUJITSU LIMITED:2015-03-30 H1510212 add start */
#include <linux/nonvolatile_common.h>
/* FUJITSU LIMITED:2015-03-30 H1510212 add end */

//==============================================================================
// define
//==============================================================================
/* FUJITSU LIMITED:2014-11-25 H1510020 add start */
#define HUNG_LOG_I(fmt, args...)	printk("[hung_log]" fmt "\n", ## args)
#define HUNG_LOG_E(fmt, args...)	printk("[hung_log]" fmt "\n", ## args)
#define HUNG_LOG_D(fmt, args...)	/*printk("[hung_log]" fmt "\n", ## args)*/
#define HUNG_UP_INIT_FLG			0x00000001
#define HUNG_UP_TIMER_FLG			0x00000002
#define HUNG_UP_TIMER_S				(120*HZ)
#define HUNG_UP_KEY_INTERVAL_MIN	(200 * NSEC_PER_MSEC)
#define BIT_CHK(a, bit)				((a & (bit)) == bit)
/* FUJITSU LIMITED:2014-11-25 H1510020 add end */

//==============================================================================
// const data
//==============================================================================

//==============================================================================
// global valuable
//==============================================================================
/* FUJITSU LIMITED:2014-10-22 H1510008 add start */
int fj_boot_mode = FJ_MODE_NORMAL;
int charging_mode = 0;		// charg_mode
EXPORT_SYMBOL(charging_mode);
int makercmd_mode = 0;		// makercmd_mode
EXPORT_SYMBOL(makercmd_mode);
/* FUJITSU LIMITED:2014-10-22 H1510008 add end */
/* FUJITSU LIMTIED:2015-08-06 H1520403 add start */
int fjrecovery_mode = 0;
EXPORT_SYMBOL(fjrecovery_mode);
/* FUJITSU LIMTIED:2015-08-06 H1520403 add end */
/* FUJITSU LIMITED:2015-04-15 H1520315 add start */
unsigned int system_rev;
EXPORT_SYMBOL(system_rev);
/* FUJITSU LIMITED:2015-04-15 H1520315 add end   */
/* FUJITSU LIMITED:2014-11-25 H1510020 add start */
struct notifier_block		hung_up_fb_notif;
/* FUJITSU LIMITED:2014-11-25 H1510020 add end */
/* FUJITSU LIMITED:2015-05-28 H1520369 BTO add start */
unsigned char bto_info[BTO_INFO_LEN] = {0};
EXPORT_SYMBOL(bto_info);
/* FUJITSU LIMITED:2015-05-28 H1520369 BTO add end */

//==============================================================================
// private valuable
//==============================================================================
/* FUJITSU LIMITED:2014-11-25 H1510020 add start */
static unsigned int			hung_up_flg = 0;
static int hung_up_fb_status = 0;
static struct timer_list	hung_up_timer;
static struct timespec		hung_up_start_ts;
static DEFINE_SPINLOCK( hung_up_spinlock );
/* FUJITSU LIMITED:2014-11-25 H1510020 add end */

//==============================================================================
// static functions prototype
//==============================================================================

//==============================================================================
// functions
//==============================================================================
/* FUJITSU LIMITED:2014-10-22 H1510008 add start */
static int __init boot_mode_enable(char *str)
{
	if (strcmp(str, "charger") == 0) {
		charging_mode = 1;
	} else if (strcmp(str, "makermode") == 0) {
		makercmd_mode = 1;
	} else if (strcmp(str, "kernelmode") == 0) {
		makercmd_mode = 1;
	}

/* FUJITSU LIMTIED:2015-08-06 H1520403 add start */
	if (!strcmp(str, "recoverymenu")) {
		fjrecovery_mode = 1;
	} else if (!strcmp(str, "fotamode")) {
		fjrecovery_mode = 1;
	} else if (!strcmp(str, "osupdatemode")) {
		fjrecovery_mode = 1;
	} else if (!strcmp(str, "masterclear")) {
		fjrecovery_mode = 1;
	}
	printk(KERN_INFO "recoverymode: %d\n", fjrecovery_mode);
/* FUJITSU LIMTIED:2015-08-06 H1520403 add start */

	printk(KERN_INFO "chargemode: %d\n", charging_mode);
	printk(KERN_INFO "makermode : %d\n", makercmd_mode);
	return 0;
}
early_param("mode", boot_mode_enable);

static int __init boot_mode_setup(char *str)
{
	if (!strcmp(str, "chargemode"))
		fj_boot_mode = FJ_MODE_OFF_CHARGE;
	else if (!strcmp(str, "makermode"))
		fj_boot_mode = FJ_MODE_MAKER_MODE;
	else if (!strcmp(str, "kernelmode"))
		fj_boot_mode = FJ_MODE_KERNEL_MODE;

	printk(KERN_INFO"fj_boot_mode = %d\n",fj_boot_mode);

	return 1;
}
__setup("mode=", boot_mode_setup);

static int __init androidboot_mode_enable(char *str)
{
	if (strcmp(str, "charger") == 0) {
		charging_mode = 1;
	} else if (strcmp(str, "makermode") == 0) {
		makercmd_mode = 1;
	} else if (strcmp(str, "kernelmode") == 0) {
		makercmd_mode = 1;
	}

	printk(KERN_INFO "chargemode: %d\n", charging_mode);
	printk(KERN_INFO "makermode : %d\n", makercmd_mode);
	return 0;
}
early_param("androidboot.mode", androidboot_mode_enable);
/* FUJITSU LIMITED:2014-10-22 H1510008 add end */

/* FUJITSU LIMITED:2015-04-15 H1520315 add start */
static int system_rev_show(struct seq_file *m, void *unused)
{
	seq_printf(m, "Revision\t: %04x\n", system_rev);
	return 0;
}
static int system_rev_open(struct inode *inode, struct file *file)
{
	return single_open(file, system_rev_show, NULL);
}

static const struct file_operations system_rev_fops = {
	.owner = THIS_MODULE,
	.open = system_rev_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init system_rev_init(void)
{
	proc_create("system_rev", S_IRUGO, NULL, &system_rev_fops);
	return 0;
}
module_init(system_rev_init);

static int __init system_rev_setup(char *str)
{
	system_rev = (unsigned int)simple_strtoul(str, NULL, 16);
	return 0;
}
early_param("system_rev", system_rev_setup);
/* FUJITSU LIMITED:2015-04-15 H1520315 add end   */

/* FUJITSU LIMITED:2015-04-17 H1520334 add start */
static int parts_id_show(struct seq_file *m, void *unused)
{
	int ret = 0;
	uint32_t *smem_p;

	smem_p = (uint32_t *)smem_alloc_vendor0(SMEM_OEM_V0_014);
	if (smem_p == NULL) {
		ret = -EFAULT;
		printk(KERN_ERR "smem_alloc_vendor0 failed.\n");
	} else {
		seq_printf(m, "0x%08x\n", *smem_p);
	}
	return ret;
}

static int parts_id_open(struct inode *inode, struct file *file)
{
	return single_open(file, parts_id_show, NULL);
}

static const struct file_operations parts_id_fops = {
	.owner = THIS_MODULE,
	.open = parts_id_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init parts_id_init(void)
{
	proc_create("parts_id", S_IRUGO, NULL, &parts_id_fops);
	return 0;
}
module_init(parts_id_init);
/* FUJITSU LIMITED:2015-04-17 H1520334 add end   */

/* FUJITSU LIMITED:2015-05-14 H1520353 add start */
static int variant_info_show(struct seq_file *m, void *unused)
{
	int ret = 0;
	uint32_t *smem_p;

	smem_p = (uint32_t *)smem_alloc_vendor0(SMEM_OEM_V0_016);
	if (smem_p == NULL) {
		ret = -EFAULT;
		printk(KERN_ERR "smem_alloc_vendor0 failed.\n");
	} else {
		seq_printf(m, "0x%08x\n", *smem_p);
	}
	return ret;
}

static int variant_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, variant_info_show, NULL);
}

static const struct file_operations variant_info_fops = {
	.owner = THIS_MODULE,
	.open = variant_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init variant_info_init(void)
{
	proc_create("variant_info", S_IRUGO, NULL, &variant_info_fops);
	return 0;
}
module_init(variant_info_init);
/* FUJITSU LIMITED:2015-05-14 H1520353 add end   */

/* FUJITSU LIMITED:2014-11-25 H1510020 add start */
static bool hung_up_check_talk_state(void)
{
#ifdef GPIO_KEYS_SUPPORT_CALL_STATUS
	HUNG_LOG_D("Get state:%d", get_gpio_keys_call_status()?1:0);
	return get_gpio_keys_call_status();
#else
	return 0;
#endif
}

static void hung_up_timer_handler(unsigned long data)
{
	if (hung_up_check_talk_state())
		HUNG_LOG_D("%s talking",__func__);
	else
		panic("hung_up_timer expired!\n");
}

static void hung_up_add_timer(void)
{
	unsigned long flg;
	do {
		if (BIT_CHK(hung_up_flg, HUNG_UP_TIMER_FLG))
			break;

		hung_up_timer.expires  = jiffies + HUNG_UP_TIMER_S;
		hung_up_timer.data	   = ( unsigned long )jiffies;
		hung_up_timer.function = hung_up_timer_handler;
		HUNG_LOG_D("add_timer %lu", hung_up_timer.expires);
		add_timer( &hung_up_timer );

		spin_lock_irqsave( &hung_up_spinlock, flg );
		hung_up_flg |= HUNG_UP_TIMER_FLG;
		spin_unlock_irqrestore( &hung_up_spinlock, flg );

		HUNG_LOG_I("timer start");
	} while(0);
}

static void hung_up_timer_stop(bool sync)
{
	unsigned long flg;
	do {
		if (!BIT_CHK(hung_up_flg, HUNG_UP_TIMER_FLG))
			break;

		if (sync) {
			HUNG_LOG_D("del_timer_sync %d", sync);
			del_timer_sync(&hung_up_timer);
		} else {
			HUNG_LOG_D("del_timer %d", sync);
			del_timer(&hung_up_timer);
		}

		spin_lock_irqsave( &hung_up_spinlock, flg );
		hung_up_flg &= ~HUNG_UP_TIMER_FLG;
		spin_unlock_irqrestore( &hung_up_spinlock, flg );

		HUNG_LOG_I("timer stop");
	} while(0);
}

static bool hung_up_check_short_interval(void)
{
	struct timespec now_ts;

	getnstimeofday(&now_ts);

	if (timespec_to_ns(&now_ts) <=
		(timespec_to_ns(&hung_up_start_ts) + HUNG_UP_KEY_INTERVAL_MIN)) {
		HUNG_LOG_D("%s interval of the key is short",__func__);
		HUNG_LOG_D("start_ts=%ld.%ld now_ts=%ld.%ld",
					hung_up_start_ts.tv_sec, hung_up_start_ts.tv_nsec,
					now_ts.tv_sec, now_ts.tv_nsec);
		return true;
	}
	return false;
}

void hung_up_timer_start(void)
{
	do {
		if (!BIT_CHK(hung_up_flg, HUNG_UP_INIT_FLG)) {
			HUNG_LOG_D("%s 0x%x",__func__, hung_up_flg);
			break;
		}

		if (hung_up_check_short_interval()) {
			HUNG_LOG_D("%s 0x%x",__func__, hung_up_flg);
			hung_up_timer_stop(false);
			break;
		}

		if (hung_up_fb_status == FB_BLANK_UNBLANK) {
			HUNG_LOG_D("%s UNBLANK",__func__);
			break;
		}

		if (hung_up_check_talk_state()) {
			HUNG_LOG_D("%s talking",__func__);
			break;
		}
		hung_up_add_timer();
		getnstimeofday(&hung_up_start_ts);

	} while(0);
}

static int hung_up_fb_notifier(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event	*evdata = data;
	int *status;

	do {
		if (event != FB_EVENT_BLANK) {
			HUNG_LOG_D("%s not BLANK EVENT %d",__func__, (unsigned int)event);
			break;
		}

		status = evdata->data;
		hung_up_fb_status = *status;
		HUNG_LOG_D("fb_notifier s event_0x%lx 0x%d", event, hung_up_fb_status);

		if (!BIT_CHK(hung_up_flg, HUNG_UP_INIT_FLG)) {
			HUNG_LOG_D("%s 0x%x",__func__, hung_up_flg);
			break;
		}
		hung_up_timer_stop(true);
	} while(0);

	return 0;
}

void hung_up_init(void)
{
	int rc;

	do {
/* FUJITSU LIMITED:2015-01-21 H1510020-1 add start */
		if (BIT_CHK(hung_up_flg, HUNG_UP_INIT_FLG)) {
			HUNG_LOG_D("%s 0x%x",__func__, hung_up_flg);
			break;
		}
/* FUJITSU LIMITED:2015-01-21 H1510020-1 add end */
		if (fj_boot_mode != FJ_MODE_NORMAL) {
			HUNG_LOG_D("%s bootmode %d",__func__, fj_boot_mode);
			break;
		}

		hung_up_fb_notif.notifier_call = hung_up_fb_notifier;
		rc = fb_register_client( &hung_up_fb_notif );
		if (rc) {
			HUNG_LOG_E("fb_register_client error %d", rc);
			break;
		}
		init_timer(&hung_up_timer);

		hung_up_flg |= HUNG_UP_INIT_FLG;

		HUNG_LOG_D("%s init",__func__);
	} while(0);
}

void hung_up_remove(void)
{
	int rc;

	do {
		if (!BIT_CHK(hung_up_flg, HUNG_UP_INIT_FLG)) {
			HUNG_LOG_D("%s 0x%x",__func__, hung_up_flg);
			break;
		}

		hung_up_flg &= ~HUNG_UP_INIT_FLG;
		rc = fb_unregister_client(&hung_up_fb_notif);
		if (rc) {
			HUNG_LOG_E("fb_unregister_client error %d", rc);
			/* no break */
		}

		hung_up_timer_stop(false);

		hung_up_flg = 0;
		HUNG_LOG_D("%s remove",__func__);
	} while(0);
}
/* FUJITSU LIMITED:2014-11-25 H1510020 add end */

/* FUJITSU LIMITED:2015-04-15 H1520304 add start */
static int sdram_id_show(struct seq_file *m, void *unused)
{
	int ret = 0;
	uint32_t *smem_p;

	smem_p = (uint32_t*)smem_alloc_vendor0(SMEM_OEM_V0_003);
	if (smem_p == NULL) {
		ret = -EFAULT;
		printk(KERN_ERR "smem_alloc_vendor0 failed.\n");
	} else {
		seq_printf(m, "0x%08x\n", *smem_p);
	}
	return ret;
}

static int sdram_id_open(struct inode *inode, struct file *file)
{
	return single_open(file, sdram_id_show, NULL);
}

static const struct file_operations sdram_id_fops = {
	.owner = THIS_MODULE,
	.open = sdram_id_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init sdram_id_init(void)
{
	proc_create("sdram_id", S_IRUGO, NULL, &sdram_id_fops);
	return 0;
}
module_init(sdram_id_init);
/* FUJITSU LIMITED:2015-04-15 H1520304 add end */

/* FUJITSU LIMITED:2015-03-30 H1510212 add start */
#define APNV_AUTO_POWER_OFF_F 48021
#define APNV_SHIP_MODE_I 49264

int ship_mode_stdn = 0;
EXPORT_SYMBOL(ship_mode_stdn);

static int __init ship_mode_setting(void)
{
	unsigned char p_buf, s_buf;
	int result = 0;

	result = get_nonvolatile((uint8_t*)&p_buf, APNV_AUTO_POWER_OFF_F, 1);
	if (unlikely(result < 0)) {
		return 0;
	}
	result = get_nonvolatile((uint8_t*)&s_buf, APNV_SHIP_MODE_I, 1);
	if (unlikely(result < 0)) {
		return 0;
	}
	if ((p_buf == 1) && (s_buf == 1)) {
		ship_mode_stdn = 1;
		printk(KERN_INFO "ship mode shutdown\n");
	}
	return 0;
}
late_initcall(ship_mode_setting);
/* FUJITSU LIMITED:2015-03-30 H1510212 add end */
/* FUJITSU LIMITED:2015-05-28 H1520369 BTO add start */
static int btoinfo_proc_show(struct seq_file *m, void *v)
{
	int j;

	for (j = 0; j < BTO_INFO_LEN; j++) {
		seq_putc(m, bto_info[j]);
	}
    return 0;
}

static int btoinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, btoinfo_proc_show, NULL);
}

static const struct file_operations btoinfo_proc_fops = {
	.open		= btoinfo_proc_open,
	.read		= seq_read,
	.llseek 	= seq_lseek,
	.release	= single_release,
};

static int __init proc_btoinfo_init(void)
{
	proc_create("btoinfo", S_IRUGO, NULL, &btoinfo_proc_fops);
	return 0;
}
module_init(proc_btoinfo_init);

static int __init bto_info_ram_setup(void)
{
	#define K(x) ((x) << (PAGE_SHIFT - 10))
	unsigned int memtotal, i;

	memtotal = K(totalram_pages);
	memtotal = (memtotal + (512 * 1024 -1)) / (512 * 1024);

	bto_info[BTO_INFO_ID_RAM] = memtotal;

	printk(KERN_INFO "[BTO]bto_info == ");
	for (i = 0; i < BTO_INFO_LEN; i++) {
		printk(KERN_CONT "%d",bto_info[i]);
	}
	printk(KERN_CONT "\n");

	return 0;
}
arch_initcall(bto_info_ram_setup);

static int __init bto_info_setup(char *str)
{
	unsigned int i, j;
	unsigned int len = strlen(str);

	for (i=0, j=1; j < BTO_INFO_LEN && i < len; i++, j++) { /* FUJITSU LIMITED:2015-07-01 H1520369-1 BTO mod */
		bto_info[j] = str[i] - '0';
	}

	return 0;
}
early_param("bto_info", bto_info_setup);
/* FUJITSU LIMITED:2015-05-28 H1520369 BTO add end */
