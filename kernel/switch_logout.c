/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2015
/*----------------------------------------------------------------------------*/
/* FUJITSU LIMITED:2015-11-17 H1610003 add start */
#include <linux/switch_logout.h>

#ifdef SWITCH_LOGOUT

#include <linux/kernel.h>
#include <linux/debugfs.h>

static unsigned int debuglog_switch = 0;

extern void clock_dump_info(void);
extern void rmp_vreg_dump_info(void);
extern void msm_tlmm_gp_dbg_out(void);
extern void qpnp_pin_gpio_mpp_dbg_out(int detailflag);

unsigned int check_and_get_table_from_NV(void)
{
	return debuglog_switch;
}

/* Switch log is Output. */
void switch_printk(unsigned int logtype, const char *fmt, ...)
{
	va_list args;

	if (debuglog_switch & logtype) {
		va_start(args, fmt);
		vprintk(fmt, args);
		va_end(args);
	}
}

/* GPIO log is Output. */
void switch_gpio_dump(unsigned int detail)
{
	if (debuglog_switch & GpioLogType) {
		/* show msm gpio. */
		msm_tlmm_gp_dbg_out();
		/* show pmic/mpp gpio. */
		qpnp_pin_gpio_mpp_dbg_out(detail);
	}
}

/* GPIO log is Output. */
void switch_gpio_dump_op(void)
{
	if (debuglog_switch & GpioLogType) {
		/* show pmic/mpp gpio. */
		printk("switch_gpio_dump_op_start\n");
		qpnp_pin_gpio_mpp_dbg_out(1);
	}
}

void switch_powercollapse_dump(void)
{
	if (debuglog_switch & ClockLogType) {
		printk("switch_powercollapse_flg_clock_start\n");
		clock_dump_info();
	}

	if (debuglog_switch & RegulatorLogType) {
		printk("switch_powercollapse_flg_regulator_start\n");
		rmp_vreg_dump_info();
	}

	if (debuglog_switch & GpioLogType) {
		printk("switch_powercollapse_flg_gpio_start\n");
		/* show msm gpio. */
		msm_tlmm_gp_dbg_out();
	}
}

void switch_before_suspend_gpio_dump(void)
{
	if (debuglog_switch & BeforeSuspend_LogType) {
		printk("switch_before_suspend_gpio_dump_start\n");
		msm_tlmm_gp_dbg_out();
		qpnp_pin_gpio_mpp_dbg_out(1);
	}
}

void switch_after_suspend_gpio_dump(void)
{
	if (debuglog_switch & AfterSuspend_LogType) {
		printk("switch_after_suspend_gpio_dump_start\n");
		msm_tlmm_gp_dbg_out();
	}
	if ((debuglog_switch & AfterSuspend_LogType) || (debuglog_switch & GpioLogType)) {
		if ((debuglog_switch & AfterSuspend_LogType) && (debuglog_switch & GpioLogType)) {
			printk("switch_after_suspend_gpio_dump_flg_gpio&suspend_start\n");
		} else if(debuglog_switch & AfterSuspend_LogType) {
			printk("switch_after_suspend_gpio_dump_flg_suspend_start\n");
		} else if (debuglog_switch & GpioLogType) {
			printk("switch_after_suspend_gpio_dump_flg_gpio_start\n");
		}
		qpnp_pin_gpio_mpp_dbg_out(1);
	}
}

void switch_before_init_driver_gpio_dump(void)
{
	if (debuglog_switch & BeforeInitDriver_LogType) {
		printk("switch_before_init_driver_gpio_dump_start\n");
		msm_tlmm_gp_dbg_out();
	}
}

/* debugfs */
static int switch_dump_gpio_show(struct seq_file *m, void *unused)
{
	msm_tlmm_gp_dbg_out();

	return 0;
}

static int switch_dump_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, switch_dump_gpio_show, inode->i_private);
}

static const struct file_operations switch_dump_gpio_ops = {
	.open = switch_dump_gpio_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int switch_dump_qpnp_show(struct seq_file *m, void *unused)
{
	qpnp_pin_gpio_mpp_dbg_out(1);

	return 0;
}

static int switch_dump_qpnp_open(struct inode *inode, struct file *file)
{
	return single_open(file, switch_dump_qpnp_show, inode->i_private);
}

static const struct file_operations switch_dump_qpnp_ops = {
	.open = switch_dump_qpnp_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int switch_dump_clock_show(struct seq_file *m, void *unused)
{
	clock_dump_info();

	return 0;
}

static int switch_dump_clock_open(struct inode *inode, struct file *file)
{
	return single_open(file, switch_dump_clock_show, inode->i_private);
}

static const struct file_operations switch_dump_clock_ops = {
	.open = switch_dump_clock_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int switch_dump_vreg_show(struct seq_file *m, void *unused)
{
	rmp_vreg_dump_info();

	return 0;
}

static int switch_dump_vreg_open(struct inode *inode, struct file *file)
{
	return single_open(file, switch_dump_vreg_show, inode->i_private);
}

static const struct file_operations switch_dump_vreg_ops = {
	.open = switch_dump_vreg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int __init switch_dump_init(void)
{
	struct dentry *root;

	root = debugfs_create_dir("switch_dump", NULL);

	if (!root)
		return -ENOMEM;

	if (!debugfs_create_file("gpio", S_IRUGO, root, NULL, &switch_dump_gpio_ops))
		return -ENOMEM;

	if (!debugfs_create_file("qpnp", S_IRUGO, root, NULL, &switch_dump_qpnp_ops))
		return -ENOMEM;

	if (!debugfs_create_file("clock", S_IRUGO, root, NULL, &switch_dump_clock_ops))
		return -ENOMEM;

	if (!debugfs_create_file("regulator", S_IRUGO, root, NULL, &switch_dump_vreg_ops))
		return -ENOMEM;

	return 0;
}
late_initcall(switch_dump_init);

static int __init boot_debuglog_switch(char *str)
{
	debuglog_switch = (unsigned int)simple_strtoul(str, NULL, 16);
	return 0;
}
early_param("debuglog_switch", boot_debuglog_switch);

#endif /* SWITCH_LOGOUT */
/* FUJITSU LIMITED:2015-11-17 H1610003 add end */
