/*
* COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016-2017
* COPYRIGHT(c)  FUJITSU LIMITED 2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/* MDRV_FJ_RDUMP_003_01 add start */
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/memblock.h>

#ifdef RPM_DUMP_DEBUG
#define RPM_DUMP_DBGLOG(x, y...)			printk(KERN_INFO "[rpm_dump] " x, ## y)
#else
#define RPM_DUMP_DBGLOG(x, y...)
#endif
#define RPM_DUMP_INFOLOG(x, y...)			printk(KERN_INFO "[rpm_dump] " x, ## y)
#define RPM_DUMP_WARNLOG(x, y...)			printk(KERN_WARNING "[rpm_dump] " x, ## y)
#define RPM_DUMP_ERRLOG(x, y...)			printk(KERN_ERR "[rpm_dump] " x, ## y)

/* MDRV_FJ_RDUMP_002_01 add start */
#define RPM_DUMP_EXPORT_ADDR             0x14400000
#define RPM_DUMP_CODERAM_ADDROFFSET  (0)
#define RPM_DUMP_DATARAM_ADDROFFSET  (RPM_DUMP_CODERAM_SIZE)
#define RPM_DUMP_MSGRAM_ADDROFFSET   (RPM_DUMP_CODERAM_SIZE + RPM_DUMP_DATARAM_SIZE)
#define RPM_DUMP_OCIMEM_ADDROFFSET   (RPM_DUMP_CODERAM_SIZE + RPM_DUMP_DATARAM_SIZE + RPM_DUMP_MSGRAM_SIZE)
#define RPM_DUMP_CODERAM_SIZE 0x00028000
#define RPM_DUMP_DATARAM_SIZE 0x00014000
#define RPM_DUMP_MSGRAM_SIZE  0x00004000
#define RPM_DUMP_OCIMEM_SIZE  0x00010000

#define RPM_DUMP_EXPORT_SIZE  (RPM_DUMP_CODERAM_SIZE + RPM_DUMP_DATARAM_SIZE + RPM_DUMP_MSGRAM_SIZE + RPM_DUMP_OCIMEM_SIZE)

enum {
	RPM_DUMP_CODERAM,
	RPM_DUMP_DATARAM,
	RPM_DUMP_MSGRAM,
	RPM_DUMP_OCIMEM,
	RPM_DUMP_NUM
};

struct rpm_dump_type {
	char *name;
	void* addr;
	int size;
	int offset;
};

static struct rpm_dump_type rpm_dump_data[RPM_DUMP_NUM] = {
	{"coderam", NULL, RPM_DUMP_CODERAM_SIZE, RPM_DUMP_CODERAM_ADDROFFSET},
	{"dataram", NULL, RPM_DUMP_DATARAM_SIZE, RPM_DUMP_DATARAM_ADDROFFSET},
	{"msgram" , NULL, RPM_DUMP_MSGRAM_SIZE , RPM_DUMP_MSGRAM_ADDROFFSET },
	{"ocimem" , NULL, RPM_DUMP_OCIMEM_SIZE , RPM_DUMP_OCIMEM_ADDROFFSET }
};

static int rpm_dump_memblock_failed_flg = 0;

void __init rpm_dump_memblock_init(void)
{
	RPM_DUMP_DBGLOG("rpm_dump_memblock_init:start\n");
	if (memblock_reserve(RPM_DUMP_EXPORT_ADDR, RPM_DUMP_EXPORT_SIZE) != 0) {
		RPM_DUMP_ERRLOG("rpm_dump_memblock_init:memblock failed\n");
		rpm_dump_memblock_failed_flg = 1;
		return;
	}
}
/* MDRV_FJ_RDUMP_002_01 add end */
/* MDRV_FJ_RDUMP_002_02 add start */
void __init rpm_dump_export(void)
{
	int num;
	struct rpm_dump_type* data = &rpm_dump_data[0];
	char *ptr = phys_to_virt(RPM_DUMP_EXPORT_ADDR);

	RPM_DUMP_DBGLOG("rpm_dump_export:start\n");

	if (rpm_dump_memblock_failed_flg) {
		RPM_DUMP_ERRLOG("rpm_dump_export:memblock failed \n");
		return;
	}

	/* setup dump info */
	for (num = 0; num < RPM_DUMP_NUM; num++) {
		data->addr = vmalloc(data->size);
		if (data->addr == NULL) {
			RPM_DUMP_ERRLOG("rpm_dump_export:cannot export - no buffer\n");
		} else {
			memcpy(data->addr, ptr + data->offset, data->size);
			RPM_DUMP_DBGLOG("rpm_dump_export:export %s\n", data->name);
		}
		data++;
	}
}
/* MDRV_FJ_RDUMP_002_02 add end */

static int rpm_dump_open(struct inode *inode, struct file *filep)
{
	filep->private_data = PDE_DATA(inode);
	RPM_DUMP_DBGLOG("rpm_dump_open:PDE(inode)->data = %p\n", filep->private_data);
	return 0;
}

static ssize_t rpm_dump_read(struct file *filep, char __user *buf, size_t count, loff_t *pos)
{
	struct rpm_dump_type *data = (struct rpm_dump_type *)filep->private_data;
	size_t copy_size;

	RPM_DUMP_DBGLOG("rpm_dump_read:start, count = %lu, pos = %llu\n", count, *pos);

	if (data == NULL) {
		RPM_DUMP_ERRLOG("rpm_dump_read:data = null\n");
		return 0;
	}

	if (data->addr == NULL) {
		RPM_DUMP_ERRLOG("rpm_dump_read:data->addr = null\n");
		return 0;
	}

	if (*pos < data->size) {
		copy_size = data->size - *pos;
		copy_size = min(copy_size, count);

		if (copy_to_user(buf, data->addr + *pos, copy_size)) {
			*pos = 0;
			RPM_DUMP_ERRLOG("rpm_dump_read:copy_to_user failed\n");
			return -EFAULT;
		}
		*pos += copy_size;
	} else {
		copy_size = 0;
	}

	RPM_DUMP_DBGLOG("rpm_dump_read:copy_size = %lu\n", copy_size);
	return (ssize_t)copy_size;
}

static int rpm_dump_release(struct inode *inode, struct file *filep)
{
	struct rpm_dump_type *data = (struct rpm_dump_type *)filep->private_data;

	if (data == NULL) {
		RPM_DUMP_ERRLOG("rpm_dump_release:data = null\n");
		return 0;
	}

	if (data->addr != NULL) {
		RPM_DUMP_DBGLOG("rpm_dump_release:data->addr %p release\n", data->addr);
		vfree(data->addr);
		data->addr = NULL;
	}
	return 0;
}

static const struct file_operations rpm_dump_ops = {
	.open = rpm_dump_open,
	.read = rpm_dump_read,
	.release = rpm_dump_release,
};

static int __init rpm_dump_init(void)
{
	struct proc_dir_entry *parent;
	int num;
	struct rpm_dump_type* data = &rpm_dump_data[0];

	parent = proc_mkdir("rpm_dump", NULL);
	if (parent == NULL) {
		RPM_DUMP_ERRLOG("rpm_dump_init:create_proc_entry failed\n");
		return -EBUSY;
	}

	for (num = 0; num < RPM_DUMP_NUM; num++) {
		proc_create_data(data->name, 0400, parent, &rpm_dump_ops, data);
		RPM_DUMP_DBGLOG("rpm_dump_init:create_proc_data name=%s, data=%p\n", data->name, data);
		data++;
	}
	return 0;
}

late_initcall(rpm_dump_init);
/* MDRV_FJ_RDUMP_003_01 add end */
