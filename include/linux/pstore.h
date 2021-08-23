/*
 * Persistent Storage - pstore.h
 *
 * Copyright (C) 2010 Intel Corporation <tony.luck@intel.com>
 *
 * This code is the generic layer to export data records from platform
 * level persistent storage via a file system.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
/*----------------------------------------------------------------------------*/

#ifndef _LINUX_PSTORE_H
#define _LINUX_PSTORE_H

#include <linux/time.h>
#include <linux/kmsg_dump.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/errno.h>

/* types */
enum pstore_type_id {
	PSTORE_TYPE_DMESG	= 0,
	PSTORE_TYPE_MCE		= 1,
	PSTORE_TYPE_CONSOLE	= 2,
	PSTORE_TYPE_FTRACE	= 3,
	PSTORE_TYPE_PMSG	= 4, /* Backport: 7 in upstream 3.19.0-rc3 */
/* FUJITSU:2016-03-04 LogImprovement ADD-S */
#ifdef CONFIG_PSTORE_SUMMARY
	PSTORE_TYPE_SUMMARY	= 5,
#endif
/* FUJITSU:2016-03-04 LogImprovement ADD-E */
	PSTORE_TYPE_UNKNOWN	= 255
};

/* FCNT:2016-06-20 GEN-162-SUMMARY-LOG ADD-S */
typedef enum {
    SMR_UNKNOWN             = -1,
    SMR_TEMP_SHUTDOWN       = 1,
    SMR_LOW_BATTERY         = 2,
    SMR_POWER_FLICKER       = 3,
    SMR_SIM_ADDED           = 5,
    SMR_SSR                 = 7,
    SMR_KEN_PANIC           = 8,
    SMR_SW_WDT              = 9,
    SMR_SIGSEGV             = 10,
    SMR_SIGABRT             = 11,
    SMR_WDT_BARK_NOW        = 12,
    SMR_FATAL_EXCEPTION     = 13,
    SMR_LONG_PRESS          = 14,
    SMR_HW_WDT              = 15,
    SMR_NORMAL_OFF          = 16,
    SMR_NORMAL_REBOOT       = 17,
    SMR_CUSTOM_LOG          = 18,
    SMR_SSR_MODEM           = 19,
    SUMMARY_ID_MAX
} SMR_TID;
/* FCNT:2016-06-20 GEN-162-SUMMARY-LOG ADD-E */

struct module;

struct pstore_info {
	struct module	*owner;
	char		*name;
	spinlock_t	buf_lock;	/* serialize access to 'buf' */
	char		*buf;
	size_t		bufsize;
	struct mutex	read_mutex;	/* serialize open/read/close */
	int		(*open)(struct pstore_info *psi);
	int		(*close)(struct pstore_info *psi);
	ssize_t		(*read)(u64 *id, enum pstore_type_id *type,
			int *count, struct timespec *time, char **buf,
			struct pstore_info *psi);
	int		(*write)(enum pstore_type_id type,
			enum kmsg_dump_reason reason, u64 *id,
			unsigned int part, int count, size_t size,
			struct pstore_info *psi);
	int		(*write_buf)(enum pstore_type_id type,
			enum kmsg_dump_reason reason, u64 *id,
			unsigned int part, const char *buf, size_t size,
			struct pstore_info *psi);
	int		(*erase)(enum pstore_type_id type, u64 id,
			int count, struct timespec time,
			struct pstore_info *psi);
	void		*data;
};

#ifdef CONFIG_PSTORE
extern int pstore_register(struct pstore_info *);
extern bool pstore_cannot_block_path(enum kmsg_dump_reason reason);
#else
static inline int
pstore_register(struct pstore_info *psi)
{
	return -ENODEV;
}
static inline bool
pstore_cannot_block_path(enum kmsg_dump_reason reason)
{
	return false;
}
/* FUJITSU:2016-03-04 LogImprovement ADD-S */
#endif

#ifdef CONFIG_PSTORE_SUMMARY
extern ssize_t pstore_write_summary(const int resetid);
/* FUJITSU:2016-03-04 LogImprovement ADD-E */
extern ssize_t pstore_write_summary_msg(const int resetid, const char* msg);
#endif

#endif /*_LINUX_PSTORE_H*/
