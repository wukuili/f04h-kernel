/* COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016-2017 */
/* COPYRIGHT(C) FUJITSU LIMITED 2010-2015 */
/*
 * FJSEC LSM module
 *
 * based on deckard.c
 * based on root_plug.c
 * Copyright (C) 2002 Greg Kroah-Hartman <greg@kroah.com>
 *
 * _xx_is_valid(), _xx_encode(), _xx_realpath_from_path()
 * is ported from security/tomoyo/realpath.c in linux-2.6.32
 *
 * Copyright (C) 2005-2011  NTT DATA CORPORATION
 *
 * _xx_encode(), _xx_get_absolute_path, _xx_get_dentry_path,
 * _xx_get_local_path, _xx_get_socket_name, _xx_realpath_from_path()
 * are ported from security/tomoyo/realpath.c in linux-3.4.0
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/security.h>
#include <linux/moduleparam.h>
#include <linux/mount.h>
#include <linux/mnt_namespace.h>
#include <linux/fs_struct.h>
#include <linux/namei.h>
#include <linux/module.h>
#include <linux/magic.h>
#include <asm/mman.h>
#include <linux/msdos_fs.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/vmalloc.h>
#include <linux/limits.h>
#include <linux/fs.h>
#include <asm/memory.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <asm/page.h>
#include <linux/securebits.h>
#include <linux/capability.h>
#include <linux/binfmts.h>
#include <linux/personality.h>
#include <linux/audit.h>
#include <linux/memblock.h>
#include <linux/fs.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <net/sock.h>
#include <linux/dcache.h>
#include <linux/time.h>

#ifdef CONFIG_DETECT_RH
#include "detect_rh.h"
#endif /* CONFIG_DETECT_RH */

#include "fjsec_config.h"
#include "fjsec_capability.h"
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
#ifndef LSM_STRICT_MEMORY_DEBUG
#include "../arch/arm64/mm/fjsec_pageattr.h"
#endif
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
#include <linux/nonvolatile_common.h>
#include <../drivers/coresight/coresight-priv.h>

/* FUJITSU:2015-3-04 ADD START */
// For FIDO
#ifdef CONFIG_FIDO_SEC
#include <linux/msm_ion_ex.h>
#endif
/* FUJITSU:2015-3-04 ADD END */

//#define PRCONFIG
//#define DUMP_PRINFO
//#define DUMP_PTRACE_REQUEST

//#define dprintk printk
#define dprintk(...)

static enum boot_mode_type boot_mode = BOOT_MODE_NONE;
static bool mount_flag = true;
#ifdef DUMP_PTRACE_REQUEST
static long prev_request = -1;
#endif /* DUMP_PTRACE_REQUEST */


static char lsm_nv_flag = FJSEC_LSM_NV_BEFORE;

/* FUJITSU:2015-3-04 ADD START */
// Page acl check offset
#define PAGE_CHECK_OFFSET	11
/* FUJITSU:2015-3-04 ADD END */

#ifdef CONFIG_LSM_ON
#define __fjsec_check_nv(a) (1UL)
#else
#define __fjsec_check_nv(a) (0UL)
#endif /* defined(CONFIG_LSM_ON) */

#define  FJLSM_REJECT(ret) return ret;

//#define ROOTFS_RW_REMOUNT
enum {
	PG_LEVEL_NONE,
	PG_LEVEL_4K,
	PG_LEVEL_2M,
	PG_LEVEL_NUM
};

#ifdef LSM_PAGEACL_AUTOREGIST
#define AUTO_REGIST_TOKEN	"auto"
void __fjsec_dump_page_acl(void);
#define AUTO_REGIST_DUMP_INTERVAL 20
#define LSM_PAGEACL_DUMPSTACK
#endif /* LSM_PAGEACL_AUTOREGIST */

#ifndef LSM_STRICT_MEMORY_DEBUG
#include <linux/rtmutex.h>
static DEFINE_MUTEX(kernel_hardening_lock);
static DEFINE_MUTEX(kernel_hardening_lock_binder);
#endif /* LSM_STRICT_MEMORY_DEBUG */

/* FUJITSU:2015-3-04 ADD START */
// For pfn range check
#define PFN_MASK	((1UL << (64-PAGE_SHIFT)) - 1)
#define MAX_PFN		0xFFFFF

static int fjsec_check_page_acl_ext(void);

static inline int pfn_sec_valid(unsigned long pfn)
{
	if( pfn > MAX_PFN || !((0 <= pfn) && (pfn < MAX_LOW_PFN)) ) return 0;
	return 1;
}
/* FUJITSU:2015-3-04 ADD END */

static inline bool _xx_is_valid(const unsigned char c)
{
	return c > ' ' && c < 127;
}

char *_xx_encode(const char *str)
{
	int len = 0;
	const char *p = str;
	char *cp;
	char *cp0;

	if (!p) {
	    printk(KERN_INFO "%s:%d: LSM_ERROR_1 arguement str is null.\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	while (*p) {
		const unsigned char c = *p++;
		if (c == '\\')
			len += 2;
		else if (c > ' ' && c < 127)
			len++;
		else
			len += 4;
	}
	len++;
	/* Reserve space for appending "/". */
	cp = kzalloc(len + 10, GFP_NOFS);
	if (!cp) {
	    printk(KERN_INFO "%s:%d: LSM_ERROR_2 cannot allocate memory.\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	cp0 = cp;
	p = str;
	while (*p) {
		const unsigned char c = *p++;

		if (c == '\\') {
			*cp++ = '\\';
			*cp++ = '\\';
		} else if (c > ' ' && c < 127) {
			*cp++ = c;
		} else {
			*cp++ = '\\';
			*cp++ = (c >> 6) + '0';
			*cp++ = ((c >> 3) & 7) + '0';
			*cp++ = (c & 7) + '0';
		}
	}
	return cp0;
}

/**
 * _xx_get_absolute_path - Get the path of a dentry but ignores chroot'ed root.
 *
 * @path:   Pointer to "struct path".
 * @buffer: Pointer to buffer to return value in.
 * @buflen: Sizeof @buffer.
 *
 * Returns the buffer on success, an error code otherwise.
 *
 * If dentry is a directory, trailing '/' is appended.
 */
static char* _xx_get_absolute_path(struct path *path, char * const buffer,
				      const int buflen)
{
	char *pos = ERR_PTR(-ENOMEM);
	if (buflen >= 256) {
		/* go to whatever namespace root we are under */
		pos = d_absolute_path(path, buffer, buflen - 1);
		if (!IS_ERR(pos) && *pos == '/' && pos[1]) {
			struct inode *inode = path->dentry->d_inode;
			if (inode && S_ISDIR(inode->i_mode)) {
				buffer[buflen - 2] = '/';
				buffer[buflen - 1] = '\0';
			}
		}
	}
	return pos;
}

/**
 * _xx_get_dentry_path - Get the path of a dentry.
 *
 * @dentry: Pointer to "struct dentry".
 * @buffer: Pointer to buffer to return value in.
 * @buflen: Sizeof @buffer.
 *
 * Returns the buffer on success, an error code otherwise.
 *
 * If dentry is a directory, trailing '/' is appended.
 */
static char* _xx_get_dentry_path(struct dentry *dentry, char * const buffer,
				    const int buflen)
{
	char *pos = ERR_PTR(-ENOMEM);
	if (buflen >= 256) {
		pos = dentry_path_raw(dentry, buffer, buflen - 1);
		if (!IS_ERR(pos) && *pos == '/' && pos[1]) {
			struct inode *inode = dentry->d_inode;
			if (inode && S_ISDIR(inode->i_mode)) {
				buffer[buflen - 2] = '/';
				buffer[buflen - 1] = '\0';
			}
		}
	}
	return pos;
}

/**
 * _xx_get_local_path - Get the path of a dentry.
 *
 * @dentry: Pointer to "struct dentry".
 * @buffer: Pointer to buffer to return value in.
 * @buflen: Sizeof @buffer.
 *
 * Returns the buffer on success, an error code otherwise.
 */
static char* _xx_get_local_path(struct dentry *dentry, char * const buffer,
				   const int buflen)
{
	struct super_block *sb = dentry->d_sb;
	char *pos = _xx_get_dentry_path(dentry, buffer, buflen);
	if (IS_ERR(pos))
		return pos;
	/* Convert from $PID to self if $PID is current thread. */
	if (sb->s_magic == PROC_SUPER_MAGIC && *pos == '/') {
		char *ep;
		const pid_t pid = (pid_t) simple_strtoul(pos + 1, &ep, 10);
		if (*ep == '/' && pid && pid ==
		    task_tgid_nr_ns(current, sb->s_fs_info)) {
			pos = ep - 5;
			if (pos < buffer)
				goto out;
			memmove(pos, "/self", 5);
		}
		goto prepend_filesystem_name;
	}
	/* Use filesystem name for unnamed devices. */
	if (!MAJOR(sb->s_dev))
		goto prepend_filesystem_name;
	{
		struct inode *inode = sb->s_root->d_inode;
		/*
		 * Use filesystem name if filesystem does not support rename()
		 * operation.
		 */
		if (inode->i_op && !inode->i_op->rename)
			goto prepend_filesystem_name;
	}
	/* Check device name and prepend path name. */
	{
		char name[64];
		int name_len;
		const dev_t dev = sb->s_dev;
		struct fs_path_config *pc;
		struct fs_path_config *work_devs;

#ifdef CHECK_PROTO_NUM
		if (lsm_nv_flag != FJSEC_LSM_NV_BEFORE) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
			work_devs = coresight_fuse_apps_access_disabled()? devs2:devs;
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
		}
		else {
			work_devs = devs;
		}
#else
		work_devs = devs;
#endif
		dprintk(KERN_INFO "%s:%d: lsm_nv_flag = 0x%02x\n", __FUNCTION__, __LINE__, lsm_nv_flag);

		for (pc = work_devs; pc->prefix; pc++) {
			if (dev == pc->rdev) {
			    dprintk(KERN_INFO "%s:%d: dev is found. dev(%u,%u)\n", __FUNCTION__, __LINE__, MAJOR(dev), MINOR(dev));
				if( pc->mnt_pnt ) {
					name_len = strlen(pc->mnt_pnt);
					pos -= name_len;
					if (pos < buffer)
						goto out;
					memmove(pos, pc->mnt_pnt, name_len);
				} else {
					name[sizeof(name) - 1] = '\0';
					snprintf(name, sizeof(name) - 1, "dev(%u,%u):", MAJOR(dev), MINOR(dev));
					name_len = strlen(name);
					pos -= name_len;
					if (pos < buffer)
						goto out;
					memmove(pos, name, name_len);
				}
				break;
			}
		}
		return pos;
	}
	/* Prepend filesystem name. */
prepend_filesystem_name:
	{
		const char *name = sb->s_type->name;
		const int name_len = strlen(name);
		pos -= name_len + 1;
		if (pos < buffer)
			goto out;
		memmove(pos, name, name_len);
		pos[name_len] = ':';
	}
	return pos;
out:
	return ERR_PTR(-ENOMEM);
}

/**
 * _xx_get_socket_name - Get the name of a socket.
 *
 * @path:   Pointer to "struct path".
 * @buffer: Pointer to buffer to return value in.
 * @buflen: Sizeof @buffer.
 *
 * Returns the buffer.
 */
static char* _xx_get_socket_name(struct path *path, char * const buffer,
				    const int buflen)
{
	struct inode *inode = path->dentry->d_inode;
	struct socket *sock = inode ? SOCKET_I(inode) : NULL;
	struct sock *sk = sock ? sock->sk : NULL;
	if (sk) {
		snprintf(buffer, buflen, "socket:[family=%u:type=%u:"
			 "protocol=%u]", sk->sk_family, sk->sk_type,
			 sk->sk_protocol);
	} else {
		snprintf(buffer, buflen, "socket:[unknown]");
	}
	return buffer;
}

/**
 * _xx_realpath_from_path - Returns realpath(3) of the given pathname but ignores chroot'ed root.
 *
 * @path: Pointer to "struct path".
 *
 * Returns the realpath of the given @path on success, NULL otherwise.
 *
 * If dentry is a directory, trailing '/' is appended.
 * Characters out of 0x20 < c < 0x7F range are converted to
 * \ooo style octal string.
 * Character \ is converted to \\ string.
 *
 * These functions use kzalloc(), so the caller must call kfree()
 * if these functions didn't return NULL.
 */
// temporarly fix, add retry count handling and exit it surely.
// some log output are added to it.
#define MAX_RETRY_COUNT 3
static char* _xx_realpath_from_path_2(struct path *path)
{
	char *buf = NULL;
	char *name = NULL;
	unsigned int buf_len = PAGE_SIZE / 2;
	struct dentry *dentry = path->dentry;
	struct super_block *sb;
	unsigned int retry_count = 0;

	if (!dentry) {
	    printk(KERN_INFO "%s:%d: LSM_ERROR_1 dentry null.\n", __FUNCTION__, __LINE__);
		return NULL;
	}
	sb = dentry->d_sb;
	while (1) {
		char *pos;
		struct inode *inode;
		buf_len <<= 1;
		kfree(buf);
		buf = kmalloc(buf_len, GFP_NOFS);
		if (!buf) {
		    printk(KERN_INFO "%s:%d: LSM_ERROR_2 cannot allocate memory. \n", __FUNCTION__, __LINE__);
			break;
		}
		/* To make sure that pos is '\0' terminated. */
		buf[buf_len - 1] = '\0';
		/* Get better name for socket. */
		if (sb->s_magic == SOCKFS_MAGIC) {
			pos = _xx_get_socket_name(path, buf, buf_len - 1);
			goto encode;
		}
		/* For "pipe:[\$]". */
		if (dentry->d_op && dentry->d_op->d_dname) {
			pos = dentry->d_op->d_dname(dentry, buf, buf_len - 1);
			goto encode;
		}
		inode = sb->s_root->d_inode;
		/*
		 * Get local name for filesystems without rename() operation
		 * or dentry without vfsmount.
		 */
/* FCNT LIMITED: 2016-06-27 SEC_LSM DEL -S */
//		if (!path->mnt || (inode->i_op && !inode->i_op->rename))
//			pos = _xx_get_local_path(path->dentry, buf,
//						    buf_len - 1);
		/* Get absolute name for the rest. */
//		else {
			pos = _xx_get_absolute_path(path, buf, buf_len - 1);
			/*
			 * Fall back to local name if absolute name is not
			 * available.
			 */
			if (pos == ERR_PTR(-EINVAL)) {
//				printk(KERN_INFO "_xx_get_absolute_path failed. Maybe lazy unmount race issue happens. <%s>\n", __func__);
//				printk(KERN_INFO "_xx_get_absolute_path. res = %s. <%s>\n", buf, __func__);
				pos = _xx_get_local_path(path->dentry, buf, buf_len - 1);
			    printk(KERN_INFO "%s:%d: _xx_get_local_path. res = %s\n", __FUNCTION__, __LINE__, pos);
			}
//		}
/* FCNT LIMITED: 2016-06-27 SEC_LSM DEL -E */
encode:
		if (IS_ERR(pos)) {
		    printk(KERN_INFO "%s:%d: path null. try again. err = %lx  filename is %s \n", __FUNCTION__, __LINE__, PTR_ERR(pos), path->dentry->d_name.name );

			if (++retry_count > MAX_RETRY_COUNT) {
				kfree(buf);
			    printk(KERN_ERR "%s:%d: Unable to get info.\n", __FUNCTION__, __LINE__ );
				return NULL;
			}
			continue;
		}
		name = _xx_encode(pos);
		break;
	}
	kfree(buf);
	if (!name) {
	    dprintk(KERN_INFO "%s:%d: out of memory happens.\n", __FUNCTION__, __LINE__ );
	}

	return name;
}

static int _xx_realpath_from_path(struct path *path, char *newname, int newname_len)
{
	char *str;

	str = _xx_realpath_from_path_2(path);
	if(!str) {
	    printk(KERN_INFO "%s:%d: LSM_ERROR_1 newname_len=%u \n", __FUNCTION__, __LINE__, newname_len );
		return -1;
	} else {
		memset(newname, '\0', newname_len);
		if( strlen(str) < newname_len ) strncpy(newname, str, strlen(str));
//		newname[newname_len-1] = '\0';
		kfree(str);
		if( strlen(newname) == 0 ) {
			printk(KERN_INFO "%s:%d: LSM_ERROR_2 newname_len=%d\n", __FUNCTION__, __LINE__, newname_len);
			return -1;
		}
	}

	return 0;
}

static int __init setup_mode(char *str)
{
	if (strcmp(str, BOOT_ARGS_MODE_FOTA) == 0) {
		boot_mode = BOOT_MODE_FOTA;
	} else if (strcmp(str, BOOT_ARGS_MODE_SDDOWNLOADER) == 0) {
		boot_mode = BOOT_MODE_SDDOWNLOADER;
//	} else if (strcmp(str, BOOT_ARGS_MODE_RECOVERY) == 0) {
//		boot_mode = BOOT_MODE_RECOVERY;
	} else if (strcmp(str, BOOT_ARGS_MODE_MASTERCLEAR) == 0) {
		boot_mode = BOOT_MODE_MASTERCLEAR;
	} else if (strcmp(str, BOOT_ARGS_MODE_MAKERCMD) == 0) {
		boot_mode = BOOT_MODE_MAKERCMD;
	} else if (strcmp(str, BOOT_ARGS_MODE_OSUPDATE) == 0) {
		boot_mode = BOOT_MODE_OSUPDATE;
	} else if (strcmp(str, BOOT_ARGS_MODE_RECOVERYMENU) == 0) {
		boot_mode = BOOT_MODE_SDDOWNLOADER;
	} else if (strcmp(str, BOOT_ARGS_MODE_KERNEL) == 0) {
		boot_mode = BOOT_MODE_MAKERCMD;
	}

	dprintk(KERN_INFO "boot mode=<%d>\n", boot_mode);
	return 0;
}
early_param("mode", setup_mode);

static char *get_process_path(struct task_struct *task, char *buf, size_t size)
{
	struct mm_struct *mm;
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT DEL */
	char *cp = NULL;

	mm = task->mm;
	if (!mm) {
		dprintk(KERN_INFO "%s:%d mm is null.\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	down_read(&mm->mmap_sem);
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
	if (mm->exe_file) {
		cp = d_path(&mm->exe_file->f_path, buf, size);
	}
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
	up_read(&mm->mmap_sem);

	return cp;
}

static char *get_process_path_mapping(struct task_struct *task, char *buf, size_t size)
{
	struct mm_struct *mm;
/*  FUJITSU:2014-12-08 GEN-141-SCRU-ALT DEL */
	char *cp = NULL;

	mm = task->mm;
	if (!mm) {
		dprintk(KERN_INFO "%s:%d mm is null.\n", __FUNCTION__, __LINE__);
		return NULL;
	}

/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
	if (mm->exe_file) {
		cp = d_path(&mm->exe_file->f_path, buf, size);
	}
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */

	return cp;
}

#ifdef DUMP_PRINFO
static void dump_prinfo(const char *function_name, int line_number)
{
	char *binname;
	char *buf = kzalloc(PATH_MAX, GFP_NOFS);

	if (!buf) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return;
	}
	binname = get_process_path(current, buf, PATH_MAX-1);
	printk(KERN_INFO "%s:%d: current process name=<%s>, path=<%s>, uid=<%d>\n"
			, function_name, line_number, current->comm, binname, CURRENT_UID);
	kfree(buf);
}

static void dump_prinfo_mapping(const char *function_name, int line_number)
{
	char *binname;
	char *buf = kzalloc(PATH_MAX, GFP_NOFS);

	if (!buf) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return;
	}
	binname = get_process_path_mapping(current, buf, PATH_MAX-1);
	printk(KERN_INFO "%s:%d: current process name=<%s>, path=<%s>, uid=<%d>\n"
			, function_name, line_number, current->comm, binname, CURRENT_UID);
	kfree(buf);
}
#else /* DUMP_PRINFO */
#define dump_prinfo(a, b)
#define dump_prinfo_mapping(a, b)
#endif /* DUMP_PRINFO */


static int fjsec_check_access_process_path(char *process_path)
{
	char *buf = kzalloc(PATH_MAX, GFP_NOFS);
	char *binname;

	if (!buf) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	binname = get_process_path(current, buf, PATH_MAX-1);
	if (binname == NULL || IS_ERR(binname)) {
		printk(KERN_INFO "%s:%d: Failed getting process path. process name=%s, uid=%d, pid=%d\n"
			   , __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		kfree(buf);
		return -EPERM;
	}

	dprintk(KERN_INFO "%s:%d: process path=%s\n", __FUNCTION__, __LINE__, binname);

	if (strcmp(binname, process_path) == 0) {
		kfree(buf);
		return 0;
	}

	dprintk(KERN_INFO "%s:%d: mismatched process path. config=<%s>, current=<%s>, process name=%s, uid=%d\n"
			, __FUNCTION__, __LINE__, process_path, binname, current->comm, CURRENT_UID);

	kfree(buf);
	return -EPERM;
}

/* FCNT LIMITED: 2017-03-08 SEC_LSM_047 ADD start */
static int fjsec_check_access_parent_process_path_mapping(struct task_struct *task, char *process_path)
{
	char *buf = kzalloc(PATH_MAX, GFP_NOFS);
	char *binname;

	if (!buf) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s\n",
			 __FUNCTION__, __LINE__, task->comm);
		return -ENOMEM;
	}

	binname = get_process_path_mapping(task, buf, PATH_MAX-1);
	if (binname == NULL || IS_ERR(binname)) {
		printk(KERN_INFO "%s:%d: Failed getting process path. process name=%s\n"
			   , __FUNCTION__, __LINE__, task->comm);
		kfree(buf);
		return -EPERM;
	}

	if (strcmp(binname, process_path) == 0) {
		kfree(buf);
		return 0;
	}

	dprintk(KERN_INFO "%s:%d: process name=%s ,process path=%s\n", __FUNCTION__, __LINE__, task->comm, binname);

	kfree(buf);
	return -EPERM;
}
/* FCNT LIMITED: 2017-03-08 SEC_LSM_047 ADD end */

static int fjsec_check_access_process_path_mapping(char *process_path)
{
	char *buf = kzalloc(PATH_MAX, GFP_NOFS);
	char *binname;

	if (!buf) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	binname = get_process_path_mapping(current, buf, PATH_MAX-1);
	if (binname == NULL || IS_ERR(binname)) {
		printk(KERN_INFO "%s:%d: Failed getting process path. process name=%s, uid=%d, pid=%d\n"
			   , __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		kfree(buf);
		return -EPERM;
	}

	dprintk(KERN_INFO "%s:%d: process path=%s\n", __FUNCTION__, __LINE__, binname);

	/* Add check */
	if( strcmp(process_path, CHECK_COMMON) == 0 ) {
		if( strncmp(binname, SYSTEM_PROCESS_PATH, strlen(SYSTEM_PROCESS_PATH)) == 0 ||
			strncmp(binname, INIT_PROCESS_PATH, strlen(INIT_PROCESS_PATH)) == 0 ) {
			kfree(buf);
			return 0;
		}
	} else if (strcmp(binname, process_path) == 0) {
		kfree(buf);
		return 0;
	}

	dprintk(KERN_INFO "%s:%d: mismatched process path. config=<%s>, current=<%s>\n"
			, __FUNCTION__, __LINE__, process_path, binname);

	kfree(buf);
	return -EPERM;
}

static int chkprocinfo(const struct ac_config_ptrace *config) {

	unsigned long text1, data1;

	struct mm_struct *mm = current->mm;

	if (!mm || !mm->end_code) {
		printk(KERN_ERR "%s:%d: Unable to refer to the mminfo.\n", __FUNCTION__, __LINE__);
		return -1;
	}

	text1 = mm->end_code - mm->start_code;
	data1 = mm->end_data - mm->start_data;

	if( text1 != config->supl_val_1 || data1 != config->supl_val_2 ) {
		printk(KERN_ERR "%s:%d: FJLSM_REJECT PROC INFO not matched. text=%lu, data=%lu\n", __FUNCTION__, __LINE__, text1, data1);
		return -EPERM;
	}

	return 0;
}

#define CTS_CHKBUF	128
int chkprocname(char *buf, size_t buflen) {
	int res = 0;
	unsigned int len;
	char buffer[CTS_CHKBUF]={0};

	struct mm_struct *mm = current->mm;

	if (!mm)
		goto out;

	len = mm->arg_end - mm->arg_start;

/* FUJITSU:2015/03/20 MOD START */
    if (len == 0 || mm->arg_start == 0 || mm->arg_end) {
        goto out;
    }
/* FUJITSU:2015/03/20 MOD END   */

	if (len > CTS_CHKBUF)
		len = CTS_CHKBUF;

/* FUJITSU:2015/03/20 MOD START */
	strncpy(buffer, (char*)mm->arg_start, len);
//	memcpy(buffer, (char*)mm->arg_start, len);
/* FUJITSU:2015/03/20 MOD END   */
	res = len;

	// If the nul at the end of args has been overwritten, then
	// assume application is using setproctitle(3).
	if (buffer[len-1] != '\0' && len < CTS_CHKBUF) {
		len = strnlen(buffer, res);
		if (len < res) {
		    res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > CTS_CHKBUF - res)
				len = CTS_CHKBUF - res;
/* FUJITSU:2015/03/20 MOD START */
			if (mm->env_start != 0) {
				strncpy(buffer+res, (char*)mm->env_start, len);
			}
//			memcpy(buffer+res, (char*)mm->env_start, len);
/* FUJITSU:2015/03/20 MOD END   */
			res = strnlen(buffer, res);
		}
		buffer[res-1] = '\0';
	}

	if( res > buflen ) res = buflen -1;
	memset(buf, '\0', buflen);
/* FUJITSU:2015/03/20 MOD START */
	strncpy(buf, buffer, res);
//	memcpy(buf, buffer, res);
/* FUJITSU:2015/03/20 MOD END   */

out:
	return res;
}


static int getprocname(char *buf, size_t buflen) {
	int res = -1;
	unsigned int len;
	char* buffer;

	struct mm_struct *mm = current->mm;

	if (!mm)
		goto out;

	len = mm->arg_end - mm->arg_start;

	if (len > PAGE_SIZE)
		len = PAGE_SIZE;

	buffer = kmalloc(len, GFP_NOFS);
	if( !buffer ) goto out;

	res = access_remote_vm(mm, mm->arg_start, buffer, len, 0);

	// If the nul at the end of args has been overwritten, then
	// assume application is using setproctitle(3).
	if (res > 0 && buffer[res-1] != '\0' && len < PAGE_SIZE) {
		len = strnlen(buffer, res);
		if (len < res) {
		    res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > PAGE_SIZE - res)
				len = PAGE_SIZE - res;
			res += access_remote_vm(mm, mm->env_start, buffer+res, len, 0);
			res = strnlen(buffer, res);
		}
	}

	if( res > buflen ) res = buflen -1;
	memset(buf, '\0', buflen);
	memcpy(buf, buffer, res);

	kfree(buffer);

out:
	return res;
}

#define MAX_PROC_NAME_BUFLEN		256
static int fjsec_check_access_process(char *process_name, char *process_path)
{
	char procnamebuf[MAX_PROC_NAME_BUFLEN] ={0};
	int process_name_len = 0;
	int ret;
	char *strtoken;

  if(!process_name || !process_path)
    return -EPERM;

	ret = getprocname(procnamebuf, MAX_PROC_NAME_BUFLEN);
	dprintk(KERN_INFO "%s:%d: s_procname=%s, d_procname=%s(%d)\n", __FUNCTION__, __LINE__, process_name, procnamebuf, ret);

	if( ret == -1 ) {
		process_name_len = strlen(process_name);
		if ( process_name_len > (TASK_COMM_LEN - 1)) {
			if( memcmp(current->comm, process_name, TASK_COMM_LEN-1) != 0 ) {
				process_name += (process_name_len - (TASK_COMM_LEN - 1));
				printk(KERN_INFO "%s:%d: procname is adjusted.(%s)\n", __FUNCTION__, __LINE__, process_name);
			} else return fjsec_check_access_process_path(process_path);
		}
	} else {
		if( (strtoken = strrchr(procnamebuf, '/')) != NULL ) {
			process_name_len = strlen(strtoken);
			memmove(procnamebuf, strtoken+1, process_name_len);
			procnamebuf[process_name_len]='\0';
		}
	}
/* FCNT LIMITED:2017-04-05 SEC-LSM-048 mod start*/
	if(( memcmp(procnamebuf, process_name, strlen(process_name)) == 0 ) ||
	   ( memcmp(current->comm, process_name, strlen(process_name)) == 0 )) {
		if (fjsec_check_access_process_path(process_path) == 0) {
			return 0;
		} else {
			dprintk(KERN_INFO "%s:%d: mismatched process path. config=<%s>\n"
				, __FUNCTION__, __LINE__, process_path);
		}
	} else {
		dprintk(KERN_INFO "%s:%d: mismatched process name. config=<%s>, current=<%s>\n"
				, __FUNCTION__, __LINE__, process_name, procnamebuf);
	}
/* FCNT LIMITED:2017-04-05 SEC-LSM-048 end*/
	return -EPERM;
}

static int fjsec_check_access_process_mapping(char *process_name, char *process_path)
{
	dprintk(KERN_INFO "%s:%d: process name=%s\n", __FUNCTION__, __LINE__, current->comm);
	if (strcmp(current->comm, process_name) == 0) {
		if (fjsec_check_access_process_path_mapping(process_path) == 0) {
			return 0;
		}
	} else {
		dprintk(KERN_INFO "%s:%d: mismatched process name. config=<%s>, current=<%s>\n"
				, __FUNCTION__, __LINE__, process_name, current->comm);
	}

	return -EPERM;
}

int fjsec_check_mmcdl_access_process(void)
{
	int index;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	for (index = 0; mmcdl_device_list[index].process_name; index++) {
		if (boot_mode == mmcdl_device_list[index].boot_mode) {
			if (fjsec_check_access_process(mmcdl_device_list[index].process_name,
											mmcdl_device_list[index].process_path) == 0) {
				return 0;
			}
		}
	}

	printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed accessing mmcdl device. process name=%s, uid=%d, pid=%d\n",
		 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
	return -1;
}

int fjsec_check_mkdrv_access_process(void)
{
	int index;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	for (index = 0; mkdrv_device_list[index].process_name; index++) {
		if (boot_mode == mkdrv_device_list[index].boot_mode) {
			if (fjsec_check_access_process(mkdrv_device_list[index].process_name,
											mkdrv_device_list[index].process_path) == 0) {
				return 0;
			}
		}
	}

	printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed accessing mkdrv device. process name=%s, uid=%d, pid=%d\n",
		 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
	return -1;
}

int fjsec_check_devmem_access(unsigned long pfn)
{
	int index;


	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	for (index = 0; index < ARRAY_SIZE(devmem_acl); index++) {
		if ((pfn >= devmem_acl[index].head) && (pfn <= devmem_acl[index].tail)) {
			if (fjsec_check_access_process_mapping(devmem_acl[index].process_name, devmem_acl[index].process_path) == 0) {
				dprintk(KERN_INFO "%s:%d: GRANTED accessing devmem. pfn=<0x%lx> process name=<%s> process path=<%s>\n"
							, __FUNCTION__, __LINE__, pfn, devmem_acl[index].process_name, devmem_acl[index].process_path);
				return 0;
			}
		}
	}

	dump_prinfo_mapping(__FUNCTION__, __LINE__);
	printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed accessing devmem. pfn=<0x%lx> process name=%s, uid=%d, pid=%d\n",
		 __FUNCTION__, __LINE__, pfn, current->comm, CURRENT_UID, current->pid);
	return -1;
}

static int fjsec_check_disk_device_access_offset(struct accessible_area_disk_dev *accessible_areas, loff_t head, loff_t length)
{
	struct accessible_area_disk_dev *accessible_area;
	loff_t tail;

	tail = head + (length - 1);

	if (tail < head) {
		tail = head;
	}

	dprintk(KERN_INFO "%s:%d: head=<0x%llx>, length=<0x%llx>: tail=<0x%llx>\n"
		, __FUNCTION__, __LINE__, head, length, tail);

	for (accessible_area = accessible_areas; accessible_area->tail; accessible_area++) {
		if ((accessible_area->head <= head) && (accessible_area->tail >= head)) {
			if ((accessible_area->head <= tail) && (accessible_area->tail >= tail)) {
				dprintk(KERN_INFO "%s:%d: SUCCESS accessing disk device.\n", __FUNCTION__, __LINE__);
				return 0;
			}
		}
	}

	printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed accessing disk device. process name=%s, uid=%d, pid=%d\n",
		 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
	return -EPERM;
}

static int fjsec_check_disk_device_access(char *realpath, loff_t offset, loff_t length)
{

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_DISK_DEV_PATH) == 0) {
		dprintk(KERN_INFO "%s:%d:boot mode=<%d>\n", __FUNCTION__, __LINE__, boot_mode);

		if (boot_mode == BOOT_MODE_FOTA) {
			if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_NAME,
											CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_PATH) == 0) {
				if(fjsec_check_disk_device_access_offset(accessible_areas_fota, offset, length) == 0) {
					return 0;
				}
			}
		}
		else if (boot_mode == BOOT_MODE_SDDOWNLOADER) {
			if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_NAME,
											CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_PATH) == 0) {
				if(fjsec_check_disk_device_access_offset(accessible_areas_sddownloader, offset, length) == 0) {
					return 0;
				}
			}
		}
		else if (boot_mode == BOOT_MODE_OSUPDATE) {
			if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_NAME,
											CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_PATH) == 0) {
				if(fjsec_check_disk_device_access_offset(accessible_areas_osupdate, offset, length) == 0) {
					return 0;
				}
			}
		}
//		else if (boot_mode == BOOT_MODE_RECOVERY) {
		else if (boot_mode == BOOT_MODE_MASTERCLEAR) {
			if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_NAME,
											CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_PATH) == 0) {
				if(fjsec_check_disk_device_access_offset(accessible_areas_recovery, offset, length) == 0) {
					return 0;
				}
			}
		}
		else if (boot_mode == BOOT_MODE_MAKERCMD) {
			if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_NAME,
											CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_PATH) == 0) {
				if(fjsec_check_disk_device_access_offset(accessible_areas_maker, offset, length) == 0) {
					return 0;
				}
			}
		}

		return -EPERM;
	}

	return 0;
}

static int fjsec_check_system_access_process(void)
{

	if (boot_mode == BOOT_MODE_FOTA) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}
#if 0
	if (boot_mode == BOOT_MODE_SDDOWNLOADER) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}

	if (boot_mode == BOOT_MODE_OSUPDATE) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}

//	if (boot_mode == BOOT_MODE_RECOVERY) {
	if (boot_mode == BOOT_MODE_MASTERCLEAR) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}
#endif
	if (!current->mm) {
		dprintk(KERN_INFO "%s:%d (!current->mm)\n", __FUNCTION__, __LINE__);
		return 0;
	}
#ifdef CONFIG_DETECT_RH
	set_rhflag();
#endif /* CONFIG_DETECT_RH */
	return -1;
}

static int fjsec_check_system_directory_access_process(char *realpath)
{

	if (strncmp(realpath, CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH,
					strlen(CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH)) == 0) {

		if (fjsec_check_system_access_process() == 0) {
			return 0;
		}

		return -EPERM;
	}

	return 0;
}

static int fjsec_check_system_device_access_process(char *realpath)
{

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_SYSTEM_DEV_PATH) == 0) {

		if (fjsec_check_system_access_process() == 0) {
			return 0;
		}

		return -EPERM;
	}

	return 0;
}

#ifdef CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE
static int fjsec_check_secure_storage_access_process(void)
{

#if 0
	if (boot_mode == BOOT_MODE_FOTA) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}
	else
#endif
	if (boot_mode == BOOT_MODE_SDDOWNLOADER) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}
	else if (boot_mode == BOOT_MODE_OSUPDATE) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}
//	else if (boot_mode == BOOT_MODE_RECOVERY) {
	else if (boot_mode == BOOT_MODE_MASTERCLEAR) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}
	else if (boot_mode == BOOT_MODE_MAKERCMD) {
		if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
		}
	}

	if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_SECURE_STORAGE_ACCESS_PROCESS_NAME,
										CONFIG_SECURITY_FJSEC_SECURE_STORAGE_ACCESS_PROCESS_PATH) == 0) {
			return 0;
	}

	if (fjsec_check_access_process(INIT_PROCESS_NAME, INIT_PROCESS_PATH) == 0) {
		printk(KERN_INFO "%s:%d:init proc, boot mode=<%d>\n", __FUNCTION__, __LINE__, boot_mode);
		return 0;
	}

	return -1;
}

static int fjsec_check_secure_storage_directory_access_process(char *realpath)
{

	if (strncmp(realpath, CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DIR_PATH,
					strlen(CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DIR_PATH)) == 0) {
		if (fjsec_check_secure_storage_access_process() == 0) {
			return 0;
		}

		return -EPERM;
	}

	return 0;
}

static int fjsec_check_secure_storage_device_access_process(char *realpath)
{

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DEV_PATH) == 0) {
		if (fjsec_check_secure_storage_access_process () == 0) {
			return 0;
		}

		return -EPERM;
	}

	return 0;
}

#else /* CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE */

#define fjsec_check_secure_storage_directory_access_process(a) (0)
#define fjsec_check_secure_storage_device_access_process(a) (0)

#endif /* CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE */

#ifdef CONFIG_SECURITY_FJSEC_AC_KITTING

static int fjsec_check_kitting_access_process(char *process_name, char *process_path)
{

    dprintk(KERN_INFO "%s:%d: process name=%s\n", __FUNCTION__, __LINE__, current->comm);

	if (strcmp(current->group_leader->comm, process_name) == 0) {
		if (fjsec_check_access_process_path(process_path) == 0) {
			return 0;
		}
	} else {
		dprintk(KERN_INFO "%s:%d: mismatched process name. config=<%s>, current=<%s>\n"
				, __FUNCTION__, __LINE__, process_name, current->group_leader->comm);
	}

	return -EPERM;
}

static int fjsec_check_kitting_directory_access_process(char *realpath)
{

	if (strncmp(realpath, CONFIG_SECURITY_FJSEC_KITTING_DIR_PATH,
					strlen(CONFIG_SECURITY_FJSEC_KITTING_DIR_PATH)) == 0) {
		if (boot_mode == BOOT_MODE_NONE || boot_mode == BOOT_MODE_MAKERCMD) {
			struct kitting_directory_access_process *kd_access_control_process;

			for (kd_access_control_process = kitting_directory_access_process_list; kd_access_control_process->process_path; kd_access_control_process++) {
				char *process_name;
				int process_name_len;

				if (kd_access_control_process->uid != UID_NO_CHECK && kd_access_control_process->uid != CURRENT_UID) {
					dprintk(KERN_INFO "%s:%d: mismatched process UID\n", __FUNCTION__, __LINE__);
					continue;
				}

				process_name_len = strlen(kd_access_control_process->process_name);
				process_name = kd_access_control_process->process_name;

				if (process_name_len > (TASK_COMM_LEN - 1)) {
					process_name += (process_name_len - (TASK_COMM_LEN - 1));
				}

				if (fjsec_check_kitting_access_process(process_name,
												kd_access_control_process->process_path) != 0) {
					dprintk(KERN_INFO "%s:%d: mismatched process name, process path\n", __FUNCTION__, __LINE__);
					continue;
				}

				dprintk(KERN_INFO "%s:%d: SUCCESS realpath=%s\n", __FUNCTION__, __LINE__, realpath);
				return 0;
			}
		}

		if (fjsec_check_access_process(INIT_PROCESS_NAME, INIT_PROCESS_PATH) == 0) {
			printk(KERN_INFO "%s:%d:init proc, boot mode=<%d>\n", __FUNCTION__, __LINE__, boot_mode);
			return 0;
		}

        if (boot_mode == BOOT_MODE_MAKERCMD) {
			if ((fjsec_check_access_process("makercmd", "/system/bin/makercmd") == 0) && (CURRENT_UID == AID_ROOT)) {
				printk(KERN_INFO "%s:%d: boot mode=<%d> and matched process name, process path.\n", __FUNCTION__, __LINE__, boot_mode);
				return 0;
			}
		}

        return -EPERM;
	}

	return 0;
}

static int fjsec_check_kitting_device_access_process(char *realpath)
{

    if (strcmp(realpath, CONFIG_SECURITY_FJSEC_KITTING_DEV_PATH) == 0) {
		if (boot_mode == BOOT_MODE_SDDOWNLOADER) {
			if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_NAME,
											CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_PATH) == 0) {
				return 0;
			}
		}
//		else if (boot_mode == BOOT_MODE_RECOVERY) {
		else if (boot_mode == BOOT_MODE_MASTERCLEAR) {
			if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_NAME,
											CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_PATH) == 0) {
				return 0;
			}
		}

		return -EPERM;
	}

	return 0;
}

#else /* CONFIG_SECURITY_FJSEC_AC_KITTING */

#define fjsec_check_kitting_directory_access_process(a) (0)
#define fjsec_check_kitting_device_access_process(a) (0)

#endif /* CONFIG_SECURITY_FJSEC_AC_KITTING */

static int fjsec_check_device_access_process(char *realpath, struct ac_config *acl)
{

	struct ac_config *config;
	int result = 0;

	for (config = acl; config->prefix; config++) {
		int length = strlen(config->prefix);

		if (config->prefix[length - 1] == '*') {
			if (strncmp(realpath, config->prefix, length - 1) != 0) {
				continue;
			}
		} else {
			if (strcmp(realpath, config->prefix) != 0) {
				continue;
			}
		}

		if (!config->boot_mode && !config->process_name && !config->process_path) {
			printk(KERN_INFO "%s:%d: <%s> is banned access.\n", __FUNCTION__, __LINE__, config->prefix);
			return -EPERM;
		}

		if (boot_mode == config->boot_mode) {
			if (fjsec_check_access_process(config->process_name, config->process_path) == 0) {
				return 0;
			}
		}

		result = -EPERM;
	}

	return result;
}

/* FUJITSU:2015-3-04 ADD START */
#ifdef FIDO_SEC
int fjsec_ion_access_control(void *cpu_addr, unsigned long len, dma_addr_t handle, pid_t pid, const char* heapname, int skip)
{
//	struct read_write_access_control_process *rw_access_control_process;
	struct ac_config_ptrace* rw_access_control_process;
	struct ac_config_fido* acl;
	unsigned int idx = 0;

	dprintk(KERN_INFO "%s:%d: pa=0x%lx, len = %lu, handle=0x%lx, heapn=%s. procn=%s, uid=%d, pid=%d\n", __FUNCTION__, __LINE__, (unsigned long)cpu_addr, len, (unsigned long)handle, heapname, current->comm, CURRENT_UID, pid);

	// FIDO access list check
	for (rw_access_control_process = ac_config_fido_acl; rw_access_control_process->process_name; rw_access_control_process++, idx++) {

			if (rw_access_control_process->uid != UID_NO_CHECK && rw_access_control_process->uid != CURRENT_UID) {
				continue;
			}

//			if (rw_access_control_process->process_name != PRNAME_NO_CHECK) {
			/**************** 2016.02.09 remove info case ***************/
			if (rw_access_control_process->process_name != PRNAME_NO_CHECK && pid!=0) {
				if (fjsec_check_access_process(rw_access_control_process->process_name, rw_access_control_process->supl_info) != 0) {
					continue;
				}
			} else {
				if (fjsec_check_access_process_path(rw_access_control_process->supl_info) != 0) {
					continue;
				}
			}

/* FCNT LIMITED:2016-09-07 SEC-LSM-026 DEL */
			dprintk(KERN_INFO "%s:%d: SUCCESS access fido caller check\n", __FUNCTION__, __LINE__);

			if( !skip ) {
				if( !fido_acl ) {
					printk(KERN_ERR "%s:%d: fido check info not ready. pa=0x%lx, len = %lu, handle=0x%lx, heapn=%s. procn=%s, uid=%d, pid=%d\n", __FUNCTION__, __LINE__, (unsigned long)cpu_addr, len, (unsigned long)handle, heapname, current->comm, CURRENT_UID, pid);
					return -EPERM;
				}

				for(acl = fido_acl; acl->len; acl++) {
					if( acl->cpu_addr == cpu_addr &&
						acl->len == len &&
						acl->handle == handle &&
						acl->pid == pid &&
						!strncmp(acl->heapname, heapname, strlen(acl->heapname))) {
						dprintk(KERN_INFO "%s:%d: SUCCESS access fido detail check\n", __FUNCTION__, __LINE__);
						return 0;
					}
				}
			} else {
				printk(KERN_INFO "%s:%d: Skip fido detail check, idx = %u\n", __FUNCTION__, __LINE__, idx);
				return idx;
			}
	}

	printk(KERN_ERR "%s:%d: access fido caller check failed. pa=0x%lx, len = %lu, handle=0x%lx, heapn=%s. procn=%s, uid=%d, pid=%d\n", __FUNCTION__, __LINE__, (unsigned long)cpu_addr, len, (unsigned long)handle, heapname, current->comm, CURRENT_UID, pid);
	return -EPERM;
}

static inline int process_alive(pid_t pid)
{
	return get_pid_task(find_get_pid(pid), PIDTYPE_PID)? 1:0;
}

static int chk_pnameuid_from_pid(pid_t pid, int idx)
{
	struct ac_config_ptrace* rw_access_control_process = ac_config_fido_acl + idx;
	struct task_struct *task_s = NULL;

	task_s = get_pid_task(find_get_pid(pid), PIDTYPE_PID);

	if( task_s && task_s->cred ) {
		if( strcmp(task_s->comm, rw_access_control_process->process_name) == 0 && rw_access_control_process->uid == task_s->cred->uid ) return 0;
		printk(KERN_ERR "%s:%d: FJLSM_REJECT to regist fido info. pid=%d, pname(s)=%s, uid(s)=%d, pname(s)=%s, uid(s)=%d\n", __FUNCTION__, __LINE__, pid, task_s->comm, task_s->cred->uid, rw_access_control_process->process_name, rw_access_control_process->uid);
	} else {
		printk(KERN_ERR "%s:%d: FJLSM_REJECT to regist fido info. pid=%d\n", __FUNCTION__, __LINE__, pid);
	}

	return -EPERM;
}

// Store the ion info to check the access rights later.
int fjsec_set_ioninfo(void *cpu_addr, unsigned long len, dma_addr_t handle, const char* heapname, pid_t pid)
{
	int ret = 0;
	// To begin with, we check the caller access rights for fido ion.
	if( (ret = fjsec_ion_access_control(cpu_addr, len, handle, pid, heapname, 1)) < 0 ) {
		return -EPERM;
	}

	// set the ion info to the access list data
	if( fido_acl ) {
		struct ac_config_fido* acl;
		unsigned int count = 0;
		unsigned int max_count = sizeof(ac_config_fido_acl)/sizeof(ac_config_fido_acl[0]);

		for(acl = fido_acl; acl->len && count < max_count; acl++, count++);
		if( count == max_count ) {
			printk(KERN_INFO "%s:%d: No room for registering the fido info to the lists.\n", __FUNCTION__, __LINE__);
			acl = fido_acl + ret;
			// process is already terminated.
			if( !process_alive(acl->pid) ) {
				printk(KERN_INFO "%s:%d: This is the abnormal case. overwrite ioninfo. idx = %d\n", __FUNCTION__, __LINE__, ret);
			} else {
				/****************** 2016.02.09 pid is refered to the specified one *******************/
//				if( chk_pnameuid_from_pid(acl->pid, ret) < 0 ) return -EPERM;
				if( chk_pnameuid_from_pid(pid, ret) < 0 ) return -EPERM;
				printk(KERN_INFO "%s:%d: This is the abnormal case. double call. overwrite ioninfo. idx = %d\n", __FUNCTION__, __LINE__, ret);
			}
		}

#ifndef LSM_STRICT_MEMORY_DEBUG
		mutex_lock(&kernel_hardening_lock);
		set_memory_rw2((unsigned long) fido_acl, ALIGN(sizeof(fido_acl), PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
		acl->cpu_addr = cpu_addr;
		acl->len = len;
		acl->handle = handle;
		acl->pid = pid;
		strncpy(acl->heapname, heapname, FJSEC_ION_NAME_MAX);
		dprintk(KERN_INFO "%s:%d: regist fido info.adr=0x%lx, len=%lu, hdl=0x%lx, pid=%d, hn=%s\n", __FUNCTION__, __LINE__,(unsigned long)cpu_addr, len, (unsigned long)handle, pid, heapname);
#ifndef LSM_STRICT_MEMORY_DEBUG
		set_memory_ro2((unsigned long) fido_acl, ALIGN(sizeof(fido_acl), PAGE_SIZE) >> PAGE_SHIFT);
		mutex_unlock(&kernel_hardening_lock);
#endif /* LSM_STRICT_MEMORY_DEBUG */
	}
	return 0;
}

// Remove the ion info when the ion mem corresponding to the fido ion is released.
int fjsec_remove_ioninfo(void *cpu_addr, dma_addr_t handle)
{
	// To begin with, we check the caller access rights for fido ion.
	if( fjsec_ion_access_control(cpu_addr, 0, handle, 0, "", 1) < 0 ) {
		return -EPERM;
	}

	// set the ion info to the access list data
	if( fido_acl ) {
		struct ac_config_fido* acl;
		unsigned int count = 0;
		unsigned int max_count = sizeof(ac_config_fido_acl)/sizeof(ac_config_fido_acl[0]);

		for(acl = fido_acl; acl->len && count < max_count; acl++, count++) {
			if( acl->cpu_addr == cpu_addr && acl->handle == handle ) {
				dprintk(KERN_INFO "%s:%d: deregister fido info.adr=0x%lx, len=%lu, hdl=0x%lx, hn=%s\n", __FUNCTION__, __LINE__,(unsigned long)acl->cpu_addr, acl->len, (unsigned long)acl->handle, acl->heapname);
#ifndef LSM_STRICT_MEMORY_DEBUG
				mutex_lock(&kernel_hardening_lock);
				set_memory_rw2((unsigned long) fido_acl, ALIGN(sizeof(fido_acl), PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
				acl->cpu_addr = 0;
				acl->handle = 0;
				acl->len = 0;
				acl->pid = 0;
				memset(acl->heapname, 0, FJSEC_ION_NAME_MAX);
#ifndef LSM_STRICT_MEMORY_DEBUG
				set_memory_ro2((unsigned long) fido_acl, ALIGN(sizeof(fido_acl), PAGE_SIZE) >> PAGE_SHIFT);
				mutex_unlock(&kernel_hardening_lock);
#endif /* LSM_STRICT_MEMORY_DEBUG */
			}
		}
	}
	return 0;
}
#endif //FIDO_SEC
/* FUJITSU:2015-3-04 ADD END */

static int fjsec_read_write_access_control(char *realpath)
{
	struct read_write_access_control *rw_access_control;
	struct read_write_access_control_process *rw_access_control_process;
	int result = 1;

//	if (!(boot_mode == BOOT_MODE_NONE || boot_mode == BOOT_MODE_RECOVERY)) {
	if (!(boot_mode == BOOT_MODE_NONE || boot_mode == BOOT_MODE_MASTERCLEAR)) {
		return 1;
	}

	for (rw_access_control = rw_access_control_list; rw_access_control->prefix; rw_access_control++) {
		int length = strlen(rw_access_control->prefix);

		if (rw_access_control->prefix[length - 1] == '*') {
			if (strncmp(rw_access_control->prefix, realpath, length - 1) != 0) {
				continue;
			}
		} else {
			if (strcmp(rw_access_control->prefix, realpath) != 0) {
				continue;
			}
		}

        if ( (CURRENT_UID == AID_ROOT || CURRENT_UID == AID_SYSTEM) && (rw_access_control->f_strict == NO_STRICT) ) {
				return 0;
		}

		result = -EPERM;

		for (rw_access_control_process = rw_access_control->process_list; rw_access_control_process->process_path; rw_access_control_process++) {
			char *process_name;
			int process_name_len;

			dprintk (KERN_INFO "  config=<%s><%s><%s><%d>\n"
					, rw_access_control->prefix, rw_access_control_process->process_name, rw_access_control_process->process_path, rw_access_control_process->uid);
			if (rw_access_control_process->uid != UID_NO_CHECK && rw_access_control_process->uid != CURRENT_UID) {
				dprintk(KERN_INFO "%s:%d: mismatched process UID\n", __FUNCTION__, __LINE__);
				continue;
			}

			if (rw_access_control_process->process_name != PRNAME_NO_CHECK) {
				process_name_len = strlen(rw_access_control_process->process_name);
				process_name = rw_access_control_process->process_name;

//				if (process_name_len > (TASK_COMM_LEN - 1)) {
//					process_name += (process_name_len - (TASK_COMM_LEN - 1));
//				}

				if (fjsec_check_access_process(process_name,
												rw_access_control_process->process_path) != 0) {
					dprintk(KERN_INFO "%s:%d: mismatched process name, process path\n", __FUNCTION__, __LINE__);
					continue;
				}
			} else {
				if (fjsec_check_access_process_path(rw_access_control_process->process_path) != 0) {
					dprintk(KERN_INFO "%s:%d: mismatched process path\n", __FUNCTION__, __LINE__);
					continue;
				}
			}

			dprintk(KERN_INFO "%s:%d: SUCCESS realpath=%s\n", __FUNCTION__, __LINE__, realpath);
			return 0;
		}
	}
	return result;
}
/* FUJITSU LIMITED:2015-06-10 SEC-LSM-007 mod start */
#ifdef CONFIG_SECURITY_PATH
/* FUJITSU LIMITED:2015-06-10 SEC-LSM-007  mod end */
static void fjsec_print_path(struct path *path)
{
	char *realpath = kzalloc(PATH_MAX, GFP_NOFS);
	int r;

	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer.\n", __FUNCTION__, __LINE__);
		return;
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		return;
	}
	printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s.\n", __FUNCTION__, __LINE__, realpath);
	kfree(realpath);
}

static int fjsec_check_path_from_path(struct path *path)
{
	char *realpath = kzalloc(PATH_MAX, GFP_NOFS);
	int r;

	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		return r;
	}

	r = fjsec_read_write_access_control(realpath);
	if (r == -EPERM) {
		dump_prinfo(__FUNCTION__, __LINE__);
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s, process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		return -EPERM;
	}
	else if (r == 0) {
		kfree(realpath);
		return 0;
	}

	if (fjsec_check_system_directory_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		return -EPERM;
	}

	if (fjsec_check_device_access_process(realpath, fs_acl) != 0) {
		dump_prinfo(__FUNCTION__, __LINE__);
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		return -EPERM;
	}

	if (fjsec_check_secure_storage_directory_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
		__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		return -EPERM;
	}

	if (fjsec_check_kitting_directory_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
		__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		return -EPERM;
	}

	kfree(realpath);
	return 0;
}

static int fjsec_check_chmod_from_path(struct path *path, mode_t mode)
{
	char *realpath = kzalloc(PATH_MAX, GFP_NOFS);
	int r;
	struct fs_path_config *pc;
	struct fs_path_config *work_devs;

	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		return r;
	}

	dprintk(KERN_INFO "(%s:%d) path=<%s> mode=<%d>\n", __FUNCTION__, __LINE__, realpath, mode);

#ifdef CHECK_PROTO_NUM
	if (lsm_nv_flag != FJSEC_LSM_NV_BEFORE) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
			work_devs = coresight_fuse_apps_access_disabled()? devs2:devs;
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
	}
#else
	work_devs = devs;
#endif
	dprintk(KERN_INFO "%s:%d: lsm_nv_flag = 0x%02x\n", __FUNCTION__, __LINE__, lsm_nv_flag);

	for (pc = work_devs; pc->prefix; pc++) {
		if (strcmp(realpath, pc->prefix) == 0) {
			if (mode == (pc->mode & S_IALLUGO)) {
				break;
			}

			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
	    		 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			return -EPERM;
		}
	}

	kfree(realpath);
	return 0;
}

static int fjsec_check_chmod_from_mode(struct path *path, mode_t mode)
{
	struct inode *inode = path->dentry->d_inode;

	if(boot_mode == BOOT_MODE_FOTA) {
		return 0;
	}

	if((inode->i_uid == 0) && !(inode->i_mode & S_ISUID)) {
		if (mode  & S_ISUID) {
			fjsec_print_path(path);
			printk(KERN_INFO "%s:%d: FJLSM_REJECT process name=%s file name=%s current mode=0%o \n",
				   __FUNCTION__, __LINE__, current->comm, path->dentry->d_iname,inode->i_mode);
			return -EPERM;
		}
	}

	if((inode->i_gid == 0) && !(inode->i_mode & S_ISGID)) {
		if (mode  & S_ISGID) {
			fjsec_print_path(path);
			printk(KERN_INFO "%s:%d: FJLSM_REJECT process name=%s file name=%s current mode=0%o \n",
				   __FUNCTION__, __LINE__, current->comm, path->dentry->d_iname,inode->i_mode);
			return -EPERM;
		}
	}

	return 0;
}

/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT MOD S */
static int fjsec_check_chown_from_path(struct path *path, kuid_t uid, kgid_t gid)
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT MOD E */
{
	char *realpath = kzalloc(PATH_MAX, GFP_NOFS);
	int r;
	struct fs_path_config *pc;
	struct fs_path_config *work_devs;

	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		return r;
	}

	dprintk(KERN_INFO "(%s:%d) <%s>\n", __FUNCTION__, __LINE__, realpath);

#ifdef CHECK_PROTO_NUM
	if (lsm_nv_flag != FJSEC_LSM_NV_BEFORE) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
			work_devs = coresight_fuse_apps_access_disabled()? devs2:devs;
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
	}
	else{
		work_devs = devs;
	}
#else
	work_devs = devs;
#endif
	dprintk(KERN_INFO "%s:%d: lsm_nv_flag = 0x%02x\n", __FUNCTION__, __LINE__, lsm_nv_flag);

	for (pc = work_devs; pc->prefix; pc++) {
		if (strcmp(realpath, pc->prefix) == 0) {
			if (((uid == pc->uid) || (uid == -1)) && ((gid == pc->gid) || (gid == -1))) {
				break;
			}

			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
		    	 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			return -EPERM;
		}
	}

	kfree(realpath);
	return 0;
}

/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT MOD S */
static int fjsec_check_chown_from_id(struct path *path, kuid_t uid, kgid_t gid)
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT MOD E */
{
    struct inode *inode = path->dentry->d_inode;

	if(boot_mode == BOOT_MODE_FOTA) {
		return 0;
	}

	if((inode->i_mode & S_ISUID) && (inode->i_uid != 0)) {
		if (uid == 0) {
			fjsec_print_path(path);
			printk(KERN_INFO "%s:%d: FJLSM_REJECT process name=%s file name=%s current uid=%d \n",
				   __FUNCTION__, __LINE__, current->comm, path->dentry->d_iname, inode->i_uid);
			return -EPERM;
		}
	}

	if((inode->i_mode & S_ISGID) && (inode->i_gid != 0)) {
		if (gid == 0) {
			fjsec_print_path(path);
			printk(KERN_INFO "%s:%d: FJLSM_REJECT process name=%s file name=%s current gid=%d \n",
				   __FUNCTION__, __LINE__, current->comm, path->dentry->d_iname, inode->i_gid);
			return -EPERM;
		}
	}

	return 0;
}

static int fjsec_check_mknod(struct path *path, struct dentry *dentry,
			     int mode, unsigned int dev)
{
	char *realpath = kzalloc(PATH_MAX, GFP_NOFS);
	int r;
	struct fs_path_config *pc;
	struct fs_path_config *work_devs;
	u32 rdev;

	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		return r;
	}

	if (PATH_MAX > strlen(realpath) + strlen(dentry->d_name.name)) {
		strcat(realpath, dentry->d_name.name);
	} else {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		return -ENOMEM;
	}

	dprintk(KERN_INFO "(%s:%d) <%s> <%s>\n", __FUNCTION__, __LINE__, realpath, dentry->d_name.name);

//	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_SYMLINK_PATH) == 0 ||
//		strcmp(realpath, CONFIG_SECURITY_FJSEC_NFC_SYMLINK_PATH) == 0) {
//		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
//			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
//		kfree(realpath);
//		return -EPERM;
//	}

	rdev = new_encode_dev(dev);

#ifdef CHECK_PROTO_NUM
	if (lsm_nv_flag != FJSEC_LSM_NV_BEFORE) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
			work_devs = coresight_fuse_apps_access_disabled()? devs2:devs;
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
	}
	else{
		work_devs = devs;
	}
#else
	work_devs = devs;
#endif
	dprintk(KERN_INFO "%s:%d: lsm_nv_flag = 0x%02x\n", __FUNCTION__, __LINE__, lsm_nv_flag);

	for (pc = work_devs; pc->prefix; pc++) {
		if (rdev != pc->rdev) {
			continue;
		}

		if (mode != pc->mode) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			     __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			return -EPERM;
		}

		if (strcmp(realpath, pc->prefix) != 0) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			     __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			return -EPERM;
		}

		// rdev, mode, realpath all matched
		kfree(realpath);
		return 0;
	}

	for (pc = work_devs; pc->prefix; pc++) {
		if (strcmp(realpath, pc->prefix) != 0) {
			continue;
		}

		if (mode != pc->mode) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			return -EPERM;
		}

		if (rdev != pc->rdev) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
	    		 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			return -EPERM;
		}

		// realpath, mode, rdev all matched
		kfree(realpath);
		return 0;
	}

	kfree(realpath);
	return 0;
}
/* FUJITSU LIMITED:2015-06-10 SEC-LSM-007 mod start */
#endif
/* FUJITSU LIMITED:2015-06-10 SEC-LSM-007  mod end */
static int fjsec_check_mount_point(char *realpath, char *dev_realpath)
{
/* FCNT LIMITED:2016-04-27 sec dm-mount mod start */
	if (strncmp(realpath, CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH, strlen(CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH)) == 0) {
		if ((strcmp(dev_realpath, CONFIG_SECURITY_FJSEC_SYSTEM_DEV_PATH) == 0) || 
			(strncmp(dev_realpath, CONFIG_SECURITY_FJSEC_DM_DEV_PATH, strlen(CONFIG_SECURITY_FJSEC_DM_DEV_PATH))) == 0) {
			return MATCH_SYSTEM_MOUNT_POINT;
		}
		else {
			return MISMATCH_MOUNT_POINT;
		}
	}
	else if (strcmp(dev_realpath, CONFIG_SECURITY_FJSEC_SYSTEM_DEV_PATH) == 0) {
		return MISMATCH_MOUNT_POINT;
	}
/* FCNT LIMITED:2016-04-27 sec dm-mount mod end */

#ifdef CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE
	if (strcmp(dev_realpath, CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DEV_PATH) == 0) {
		if (strcmp(realpath, CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DIR_PATH) == 0) {
			return MATCH_SECURE_STORAGE_MOUNT_POINT;
		}
		else {
			return MISMATCH_MOUNT_POINT;
		}
	}
	else if (strncmp(realpath, CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DIR_PATH,
						strlen(CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DIR_PATH)) == 0) {
		return MISMATCH_MOUNT_POINT;
	}
#endif /* CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE */

#ifdef CONFIG_SECURITY_FJSEC_AC_KITTING
	if (strcmp(dev_realpath, CONFIG_SECURITY_FJSEC_KITTING_DEV_PATH) == 0) {
		if (strcmp(realpath, CONFIG_SECURITY_FJSEC_KITTING_DIR_PATH) == 0) {
			return MATCH_KITTING_MOUNT_POINT;
		}
		else {
			return MISMATCH_MOUNT_POINT;
		}
	}
	else if (strncmp(realpath, CONFIG_SECURITY_FJSEC_KITTING_DIR_PATH,
						strlen(CONFIG_SECURITY_FJSEC_KITTING_DIR_PATH)) == 0) {
		return MISMATCH_MOUNT_POINT;
	}
#endif /* CONFIG_SECURITY_FJSEC_AC_KITTING */

	if (strcmp(realpath, ROOT_DIR) == 0) {
		return MATCH_ROOT_MOUNT_POINT;
	}
#ifdef DEBUG_ACCESS_CONTROL
        if (strcmp(dev_realpath, TEST_PTN_PATH) == 0) {
                if (strcmp(realpath, TEST_MOUNT_POINT) == 0) {
                        return MATCH_MOUNT_POINT;
                }
                else {
                        return MISMATCH_MOUNT_POINT;
                }
        }
        else if (strncmp(realpath, TEST_MOUNT_POINT, strlen(TEST_MOUNT_POINT)) == 0) {
                return MISMATCH_MOUNT_POINT;
        }
#endif /* DEBUG_ACCESS_CONTROL */

	return UNRELATED_MOUNT_POINT;
}

// New:add Check ptrace request check extension
static int fjsec_ptrace_check_ext(void)
{
	int iErr=-1;
	struct ac_config_ptrace *config;
	char buffer[CTS_CHKBUF];

	for (config = ac_config_ptrace_acl; config->process_name; config++) {
		if (strcmp(current->comm, config->process_name) == 0) {
			if (CURRENT_UID == config->uid) {
				dprintk(KERN_INFO "(%s:%d):permit process. process name=<%s>, uid=<%d> \n",__FUNCTION__, __LINE__,  current->comm, CURRENT_UID);
				return 0;
			} else if(config->uid == UID_NO_CHECK) {
				iErr = chkprocname(buffer, CTS_CHKBUF);
				if( iErr == 0 ) {
					printk(KERN_ERR "%s:%d: Cannot get the proc detailed info. process name=%s, uid=%d, pid=%d\n",
						 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
				} else {
					// Compare info
					if (strcmp(buffer, config->supl_info) != 0) {
						printk(KERN_INFO "%s:%d: FJLSM_REJECT Cannot match the proc detailed info. process name=%s, proc_suplname=%s, uid=%d, pid=%d\n",
									 __FUNCTION__, __LINE__, buffer, config->supl_info, CURRENT_UID, current->pid);
						return -EPERM;
					}
					dprintk(KERN_INFO "(%s:%d):permit chkprocessname. process name=<%s>, supl_name=<%s>, uid=<%d> \n",__FUNCTION__, __LINE__,  current->comm, config->supl_info, CURRENT_UID);
				}

				iErr = chkprocinfo(config);
				if( iErr < 0 ) {
					printk(KERN_INFO "%s:%d: FJLSM_REJECT Cannot match the proc detailed info. process name=%s, proc_suplname=%s, uid=%d, pid=%d, errno=%d\n",
						 __FUNCTION__, __LINE__, current->comm, config->supl_info, CURRENT_UID, current->pid, iErr);
					return -EPERM;
				}

				dprintk(KERN_INFO "(%s:%d):permit process. process name=<%s>, uid=<%d> \n",__FUNCTION__, __LINE__,  current->comm, CURRENT_UID);
				return 0;
			}
		}
	}
	printk(KERN_INFO "%s:%d: FJLSM_REJECT invalid process or uid. process name=%s, uid=%d \n", __FUNCTION__, __LINE__,  current->comm, CURRENT_UID);

	return -EPERM;
}

static int fjsec_ptrace_request_check(long request)
{
	int index;
	int iErr=-1;
//	struct ac_config_ptrace *config;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	dprintk(KERN_INFO "(%s:%d): process name=<%s>, uid=<%d>, pid=<%d> request=<%ld> \n",__FUNCTION__, __LINE__,  current->comm, CURRENT_UID, current->pid, request );

	for (index = 0; index < ARRAY_SIZE(ptrace_read_request_policy); index++) {
		if (request == ptrace_read_request_policy[index]) {
#ifdef DUMP_PTRACE_REQUEST
			if (prev_request != request) {
				printk(KERN_INFO "%s:%d: GRANTED read request. request=<%ld>, process name=<%s>, pid=<%d>.\n", __FUNCTION__, __LINE__, request, current->comm, current->pid);
		 		prev_request = request;
			}
#endif /* DUMP_PTRACE_REQUEST */
			iErr = 0;
			break;
		}
	}

	if (iErr) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT write request. request=<%ld>, process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, request, current->comm, CURRENT_UID, current->pid);
		FJLSM_REJECT(-EPERM)
	}

	if (request == PTRACE_PEEKDATA && strcmp(current->comm, SIGNAL_CATCHER_PROCESS_NAME) == 0) {
		dprintk(KERN_INFO "(%s:%d):permit process. process name=<%s>, uid=<%d> \n",__FUNCTION__, __LINE__,  current->comm, CURRENT_UID);
		return 0;
	}

	if( fjsec_ptrace_check_ext() == 0 ) {
		dprintk(KERN_INFO "(%s:%d):permit process. process name=<%s>, uid=<%d> \n",__FUNCTION__, __LINE__,  current->comm, CURRENT_UID);
		return 0;
	}

	printk(KERN_INFO "%s:%d: FJLSM_REJECT invalid process or uid. process name=%s, uid=%d \n",
		   __FUNCTION__, __LINE__,  current->comm, CURRENT_UID);

	FJLSM_REJECT(-EPERM)
}

static int fjsec_ptrace_traceme(struct task_struct *parent)
{
	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	if( fjsec_ptrace_check_ext() == 0 ) {
		dprintk(KERN_INFO "(%s:%d):permit process. process name=<%s>, uid=<%d> \n",__FUNCTION__, __LINE__,  current->comm, CURRENT_UID);
		return 0;
	}

	printk(KERN_INFO "%s:%d: FJLSM_REJECT pid=%d process name=%s, uid=%d, pid=%d\n",
		__FUNCTION__, __LINE__, parent->pid, current->comm, CURRENT_UID, current->pid);
	FJLSM_REJECT(-EPERM)
}

static int fjsec_sb_mount(const char *dev_name, struct path *path,
			   const char *type, unsigned long flags, void *data)
{
	char *realpath;
	char *dev_realpath;
	int r;
	enum result_mount_point result;
	struct path dev_path;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	dev_realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!dev_realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		return -ENOMEM;
	}

	r = kern_path(dev_name, LOOKUP_FOLLOW, &dev_path);
	if (r == 0) {
		r = _xx_realpath_from_path(&dev_path, dev_realpath, PATH_MAX-1);
		path_put(&dev_path);
		if (r != 0) {
	    	printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
			kfree(realpath);
			kfree(dev_realpath);
			FJLSM_REJECT(r)
		}
	}

	dprintk(KERN_INFO "(%s:%d) <%s> <%s>\n", __FUNCTION__, __LINE__, dev_name, dev_realpath);

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		kfree(dev_realpath);
		FJLSM_REJECT(r)
	}

	dprintk(KERN_INFO "(%s:%d) <%s>\n", __FUNCTION__, __LINE__, realpath);

	result = fjsec_check_mount_point(realpath, dev_realpath);

	dprintk(KERN_INFO "(%s:%d) mount point check result=<%d>\n", __FUNCTION__, __LINE__, result);

	if (result == MATCH_SYSTEM_MOUNT_POINT) {
		if (boot_mode == BOOT_MODE_SDDOWNLOADER || boot_mode == BOOT_MODE_OSUPDATE || boot_mode == BOOT_MODE_MASTERCLEAR) {
			kfree(realpath);
			kfree(dev_realpath);
			return 0;
		}

		if ((flags & MS_RDONLY) == 0) {
			if ((flags & MS_REMOUNT) == 0) {
				if (mount_flag == false) {
					mount_flag = true;
				}
				else {
					printk(KERN_INFO "%s:%d: FJLSM_REJECT R/W MOUNT dev_realpath=%s realpath=%s process name=%s, uid=%d, pid=%d\n",
						__FUNCTION__, __LINE__, dev_realpath, realpath, current->comm, CURRENT_UID, current->pid);
					kfree(realpath);
					kfree(dev_realpath);
					FJLSM_REJECT(-EPERM)
				}
			}
			else {
				printk(KERN_INFO "%s:%d: FJLSM_REJECT R/W REMOUNT dev_realpath=%s realpath=%s process name=%s, uid=%d, pid=%d\n",
					__FUNCTION__, __LINE__, dev_realpath, realpath, current->comm, CURRENT_UID, current->pid);
				kfree(realpath);
				kfree(dev_realpath);
				FJLSM_REJECT(-EPERM)
			}
		}

		kfree(realpath);
		kfree(dev_realpath);
		return 0;
	}
#ifdef CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE
	else if (result == MATCH_SECURE_STORAGE_MOUNT_POINT) {
		kfree(realpath);
		kfree(dev_realpath);
		return 0;
	}
#endif /* CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE */
#ifdef CONFIG_SECURITY_FJSEC_AC_KITTING
	else if (result == MATCH_KITTING_MOUNT_POINT) {
		kfree(realpath);
		kfree(dev_realpath);
		return 0;
	}
#endif /* CONFIG_SECURITY_FJSEC_AC_KITTING */
	else if (result == MATCH_MOUNT_POINT || result == UNRELATED_MOUNT_POINT) {
		kfree(realpath);
		kfree(dev_realpath);
		return 0;
	}
	else if (result == MATCH_ROOT_MOUNT_POINT) {
#ifdef ROOTFS_RW_REMOUNT
		printk(KERN_INFO "%s:%d: rootfs is allowed to remount with rw flag for CT.",__FUNCTION__, __LINE__);
		kfree(realpath);
		kfree(dev_realpath);
		return 0;
#endif /* ROOTFS_RW_REMOUNT */
		if (flags & MS_RDONLY) {
			kfree(realpath);
			kfree(dev_realpath);
			return 0;
		}

		if (flags & MS_REMOUNT) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT R/W REMOUNT dev_realpath=%s realpath=%s process name=%s, uid=%d, pid=%d\n",
						__FUNCTION__, __LINE__, dev_realpath, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			kfree(dev_realpath);
			FJLSM_REJECT(-EPERM)
		}else {
			kfree(realpath);
			kfree(dev_realpath);
			return 0;

		}
	}

    printk(KERN_INFO "%s:%d: FJLSM_REJECT MOUNT dev_realpath=%s realpath=%s process name=%s, uid=%d, pid=%d\n",
		__FUNCTION__, __LINE__, dev_realpath, realpath, current->comm, CURRENT_UID, current->pid);
	kfree(realpath);
	kfree(dev_realpath);
	FJLSM_REJECT(-EPERM)
}

static int fjsec_sb_umount(struct vfsmount *mnt, int flags)
{
	char *realpath;
	struct path path = { mnt, mnt->mnt_root };
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);

	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(&path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path.dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	dprintk(KERN_INFO "%s:%d:(%s).\n", __FUNCTION__, __LINE__, realpath);

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH) == 0) {
		if (boot_mode == BOOT_MODE_FOTA) {
			if (fjsec_check_access_process(CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_NAME,
										   CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_PATH) == 0) {
				kfree(realpath);
				return 0;
			}
			if (fjsec_check_access_process(SECURITY_FJSEC_RECOVERY_PROCESS_NAME,
										   SECURITY_FJSEC_RECOVERY_PROCESS_PATH) == 0) {
				kfree(realpath);
				return 0;
			}
		}
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}
#ifdef CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE
	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DIR_PATH) == 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}
#endif /* CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE */
#ifdef CONFIG_SECURITY_FJSEC_AC_KITTING
	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_KITTING_DIR_PATH) == 0) {
/* FUJITSU LIMITED:2015-07-17 SEC-LSM-017 add start */
        if (boot_mode == BOOT_MODE_MASTERCLEAR || boot_mode == BOOT_MODE_SDDOWNLOADER ) {

            if (fjsec_check_access_process(SECURITY_FJSEC_RECOVERY_PROCESS_NAME,
										   SECURITY_FJSEC_RECOVERY_PROCESS_PATH) == 0) {
				kfree(realpath);
				return 0;
			}

        }
/* FUJITSU LIMITED:2015-07-17 SEC-LSM-017  add end */
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}
#endif /* CONFIG_SECURITY_FJSEC_AC_KITTING */

	kfree(realpath);
	return 0;
}

static int fjsec_sb_pivotroot(struct path *old_path, struct path *new_path)
{
	char *old_realpath;
	char *new_realpath;
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	old_realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!old_realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	new_realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!new_realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		kfree(old_realpath);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(old_path, old_realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, old_path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(old_realpath);
		kfree(new_realpath);
		FJLSM_REJECT(r)
	}

	r = _xx_realpath_from_path(new_path, new_realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, new_path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(old_realpath);
		kfree(new_realpath);
		FJLSM_REJECT(r)
	}

	printk(KERN_INFO "%s:%d: FJLSM_REJECT old_path=%s new_path=%s process name=%s, uid=%d, pid=%d\n",
		__FUNCTION__, __LINE__, old_realpath, new_realpath, current->comm, CURRENT_UID, current->pid);
	kfree(old_realpath);
	kfree(new_realpath);
	FJLSM_REJECT(-EPERM)
}

#ifdef CONFIG_SECURITY_FJSEC_PROTECT_CHROOT
static int fjsec_path_chroot(struct path *path)
{
	char *realpath;
	char *tmp;
	char *p, *p2;
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	tmp = kzalloc(PATH_MAX, GFP_NOFS);
	if (!tmp) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		kfree(tmp);
		FJLSM_REJECT(r)
	}

	p = CONFIG_SECURITY_FJSEC_CHROOT_PATH;
	while (*p) {
		p2 = strchr(p, ':');
		if (p2) {
			strncpy(tmp, p, (p2 - p));
			tmp[p2 - p] = 0;
		}
		else {
			strcpy(tmp, p);
		}

		if (strcmp(tmp, realpath) == 0) {
			kfree(realpath);
			kfree(tmp);
			return 0;
		}

		if (p2) {
			p = p2 + 1;
		}
		else {
			p += strlen(p);
		}
	}

	printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
	kfree(realpath);
	kfree(tmp);
	FJLSM_REJECT(-EPERM)
}
#endif	/* CONFIG_SECURITY_FJSEC_PROTECT_CHROOT */

static struct _prevInfo {
	int				pid;
	gid_t			i_gid;
	unsigned long	i_ino;
	loff_t			i_size;
} prevInfo = { -1, -1, 0, 0 };
//static unsigned long repeat_cnt = 0;
static int prev_ret             = -100000;
//static int debug_info           = 0;
static int skip_always          = 0;
//#define REPEAT_LOG_LIMIT        1000
#define CONTENTS_BOUNDARY       (500*1024*1024ULL) //500MB

static void set_prevInfo(const struct inode *d_inode) {

	if( d_inode ) {
		prevInfo.i_ino = d_inode->i_ino;
		prevInfo.i_size = i_size_read(d_inode);
		prevInfo.i_gid = d_inode->i_gid;
	}
	prevInfo.pid = current->pid;
}

static int fjsec_file_permission(struct file *file, int mask, loff_t offset, loff_t length)
{
	int r;
	char *realpath = NULL;
	struct inode *d_inode = file->f_path.dentry->d_inode;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	if( (mask & MAY_WRITE) == 0 && prev_ret != -100000 &&
		prevInfo.i_size > CONTENTS_BOUNDARY &&
		current->pid == prevInfo.pid &&
		d_inode &&
		d_inode->i_ino == prevInfo.i_ino &&
		i_size_read(d_inode) == prevInfo.i_size &&
		d_inode->i_gid == prevInfo.i_gid ) {
		if(!skip_always) {
			dprintk(KERN_INFO "%s:%d: always skip. comm=%s, dname=%s, iino=%lu, off=%lld, size=%lld\n", __FUNCTION__, __LINE__, current->comm, file->f_path.dentry->d_name.name, d_inode->i_ino, file->f_pos, prevInfo.i_size);
			skip_always = 1;
		}
		FJLSM_REJECT(prev_ret)
	}

	skip_always = 0;
	realpath = kzalloc(PATH_MAX, GFP_NOFS);

	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(&file->f_path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, file->f_path.dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	/* For read case */
	if ( (mask & MAY_WRITE) == 0 )
	{
		set_prevInfo(d_inode);

		r = fjsec_read_write_access_control(realpath);
		if (r == -EPERM) {
			if((file->f_mode & FMODE_WRITE) == 0 && (fjsec_check_page_acl_ext() == 0)){
				dprintk(KERN_INFO "%s:%d : fjsec_check_page_acl_ext success path=%s \n",__FUNCTION__, __LINE__, realpath);
				kfree(realpath);
				prev_ret = -EPERM;
				return 0;
			}
			dump_prinfo(__FUNCTION__, __LINE__);
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			prev_ret = -EPERM;
			FJLSM_REJECT(-EPERM)
		}
		else if (r == 0) {
			kfree(realpath);
			prev_ret = 0;
			return 0;
		}

		if (fjsec_check_secure_storage_directory_access_process(realpath) != 0) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			prev_ret = -EPERM;
			FJLSM_REJECT(-EPERM)
		}

		if (fjsec_check_kitting_directory_access_process(realpath) != 0) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			prev_ret = -EPERM;
			FJLSM_REJECT(-EPERM)
		}

		/* read mode -> OK! */
		kfree(realpath);
		prev_ret = 0;
		return 0;
	}
	else /* For write case */
	{
		r = fjsec_read_write_access_control(realpath);
		if (r == -EPERM) {
			dump_prinfo(__FUNCTION__, __LINE__);
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			FJLSM_REJECT(-EPERM)
		}
		else if (r == 0) {
			kfree(realpath);
			return 0;
		}

		if (fjsec_check_secure_storage_directory_access_process(realpath) != 0) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			FJLSM_REJECT(-EPERM)
		}

		if (fjsec_check_kitting_directory_access_process(realpath) != 0) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			FJLSM_REJECT(-EPERM)
		}
		/* For write mode case only check */
		if (fjsec_check_disk_device_access(realpath, offset, length) != 0) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s offset=0x%llx length=0x%llx process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, offset, length, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			FJLSM_REJECT(-EPERM)
		}

		if (fjsec_check_system_directory_access_process(realpath) != 0) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			FJLSM_REJECT(-EPERM)
		}

		if (fjsec_check_device_access_process(realpath, fs_acl) != 0) {
			dump_prinfo(__FUNCTION__, __LINE__);
			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
				__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
			kfree(realpath);
			FJLSM_REJECT(-EPERM)
		}
	}
	kfree(realpath);
	return 0;
}


/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT ADD S */
int	fjsec_mmap_file(struct file *file, unsigned long reqprot,
			     unsigned long prot, unsigned long flags)
{
	char *realpath;
	int r = 0;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	if (!file) {
		dprintk(KERN_INFO "%s:%d: file is null. process name =<%s>\n", __FUNCTION__, __LINE__, current->comm);
		kfree(realpath);
		return 0;
	}

	/* read only mode -> OK! */
	if ((prot & PROT_WRITE) == 0) {
		kfree(realpath);
		return 0;
	}

	dprintk(KERN_INFO "%s:%d: prot=<%ld>\n", __FUNCTION__, __LINE__, prot);

	r = _xx_realpath_from_path(&file->f_path, realpath, PATH_MAX-1);
	if (r != 0) {
		if (file->f_path.mnt->mnt_flags == 0) {
			kfree(realpath);
			return 0;
		} else {
	    	printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
		    	 __FUNCTION__, __LINE__, current->comm, file->f_path.dentry->d_name.name, CURRENT_UID, current->pid);
			kfree(realpath);
			FJLSM_REJECT(r)
		}
	}

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_DISK_DEV_PATH) == 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	kfree(realpath);
	return 0;
}

int fjsec_mmap_addr(unsigned long addr)
{
	int r = 0;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	if (addr < dac_mmap_min_addr) {
		r = cap_capable(current_cred(), &init_user_ns, CAP_SYS_RAWIO,
				  SECURITY_CAP_AUDIT);
		/* set PF_SUPERPRIV if it turns out we allow the low mmap */
		if (r == 0)
			current->flags |= PF_SUPERPRIV;
	}
	if (r) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT mmap. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		FJLSM_REJECT(-EPERM)
	}
	return 0;
}


static int fjsec_file_open(struct file *file, const struct cred *cred)
{
	char *realpath;
	int r;
	struct inode *d_inode = file->f_path.dentry->d_inode;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(&file->f_path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, file->f_path.dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	r = fjsec_read_write_access_control(realpath);
	if (r == -EPERM) {
		if((file->f_mode & FMODE_WRITE) == 0 && (fjsec_check_page_acl_ext() == 0)){
			dprintk(KERN_INFO "%s:%d : fjsec_check_page_acl_ext success path=%s \n",__FUNCTION__, __LINE__, realpath);
			kfree(realpath);
			return 0;
		}
		dump_prinfo(__FUNCTION__, __LINE__);
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s, process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}
	else if (r == 0) {
		if ((file->f_mode & FMODE_WRITE) == 0) {
			set_prevInfo(d_inode);
//			printk(KERN_INFO "%s:%d: SetprevInfo\n", __FUNCTION__, __LINE__);
		}
		kfree(realpath);
		return 0;
	}

	if (fjsec_check_secure_storage_directory_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_secure_storage_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_kitting_directory_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_kitting_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	/* read only mode -> OK! */
	if ((file->f_mode & FMODE_WRITE) == 0) {
		set_prevInfo(d_inode);
		dprintk(KERN_INFO "%s:%d: SetprevInfo comm=%s, dname=%s, uid=%d, pid=%d, iino=%lu, igid=%d, size=%llu\n", 
__FUNCTION__, __LINE__, current->comm, file->f_path.dentry->d_name.name, CURRENT_UID, current->pid, d_inode->i_ino, d_inode->i_gid, i_size_read(d_inode));
		kfree(realpath);
		return 0;
	}

	if (fjsec_check_system_directory_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_device_access_process(realpath, fs_acl) != 0) {
		dump_prinfo(__FUNCTION__, __LINE__);
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_system_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_device_access_process(realpath, ptn_acl) != 0) {
		dump_prinfo(__FUNCTION__, __LINE__);
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	kfree(realpath);
	return 0;
}
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT ADD E */

#ifdef CONFIG_SECURITY_PATH
static int fjsec_path_mknod(struct path *path, struct dentry *dentry, umode_t mode,
			    unsigned int dev)
{
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	r = fjsec_check_path_from_path(path);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	r = fjsec_check_mknod(path, dentry, mode, dev);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	return 0;
}

static int fjsec_path_mkdir(struct path *path, struct dentry *dentry, umode_t mode)
{
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	r = fjsec_check_path_from_path(path);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	return 0;
}

static int fjsec_path_rmdir(struct path *path, struct dentry *dentry)
{
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	r = fjsec_check_path_from_path(path);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	return 0;
}

static int fjsec_path_unlink(struct path *path, struct dentry *dentry)
{
	char *realpath;
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = fjsec_check_path_from_path(path);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	if (PATH_MAX > strlen(realpath) + strlen(dentry->d_name.name)) {
		strcat(realpath, dentry->d_name.name);
	} else {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			   __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-ENOMEM)
	}

    r = fjsec_read_write_access_control(realpath);
    if (r == -EPERM) {
        dump_prinfo(__FUNCTION__, __LINE__);
        printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s, process name=%s, uid=%d, pid=%d\n",
            __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
        kfree(realpath);
        FJLSM_REJECT(-EPERM)
    }
    else if (r == 0) {
        kfree(realpath);
        return 0;
    }

	dprintk(KERN_INFO "(%s:%d) <%s> <%s>\n", __FUNCTION__, __LINE__, realpath, dentry->d_name.name);

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_DEV_PATH) == 0 ||
		strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_DEV2_PATH) == 0 ||
		strcmp(realpath, CONFIG_SECURITY_FJSEC_NFC_DEV_PATH) == 0 ) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	kfree(realpath);
	return 0;
}

static int fjsec_path_symlink(struct path *path, struct dentry *dentry,
			      const char *old_name)
{
	char *realpath;
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = fjsec_check_path_from_path(path);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, path->dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	if (PATH_MAX > strlen(realpath) + strlen(dentry->d_name.name)) {
		strcat(realpath, dentry->d_name.name);
	} else {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			   __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-ENOMEM)
	}

	dprintk(KERN_INFO "(%s:%d) <%s> <%s> <%s>\n", __FUNCTION__, __LINE__, realpath, dentry->d_name.name, old_name);

//	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_SYMLINK_PATH) == 0 ||
//		strcmp(realpath, CONFIG_SECURITY_FJSEC_NFC_SYMLINK_PATH) == 0 ) {
//		if (strcmp(old_name, CONFIG_SECURITY_FJSEC_FELICA_DEV_PATH) != 0) {
//			printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
//				 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
//			kfree(realpath);
//			return -EPERM;
//		}
//	}

	kfree(realpath);
	return 0;
}

static int fjsec_path_link(struct dentry *old_dentry, struct path *new_dir,
			   struct dentry *new_dentry)
{
	int r;
	char *realpath;
	struct path old_path = {new_dir->mnt, old_dentry};
	struct path new_path = {new_dir->mnt, new_dentry};

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = fjsec_check_path_from_path(new_dir);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	r = _xx_realpath_from_path(&old_path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, old_path.dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	dprintk(KERN_INFO "(%s:%d) <%s>\n", __FUNCTION__, __LINE__, realpath);

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_DISK_DEV_PATH) == 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_system_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_secure_storage_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_kitting_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_device_access_process(realpath, ptn_acl) != 0) {
		dump_prinfo(__FUNCTION__, __LINE__);
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	r = _xx_realpath_from_path(&new_path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, new_path.dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	dprintk(KERN_INFO "(%s:%d) <%s>\n", __FUNCTION__, __LINE__, realpath);

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_DEV_PATH) == 0 ||
		strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_DEV2_PATH) == 0 ||
		strcmp(realpath, CONFIG_SECURITY_FJSEC_NFC_DEV_PATH) == 0 ) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	kfree(realpath);
	return 0;
}

static int fjsec_path_rename(struct path *old_dir, struct dentry *old_dentry,
			     struct path *new_dir, struct dentry *new_dentry)
{
	int r;
	char *realpath;
	struct path old_path = {old_dir->mnt, old_dentry};
	struct path new_path = {new_dir->mnt, new_dentry};

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	realpath = kzalloc(PATH_MAX, GFP_NOFS);
	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	r = fjsec_check_path_from_path(new_dir);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	r = _xx_realpath_from_path(&old_path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, old_path.dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	dprintk(KERN_INFO "(%s:%d) <%s> <%s>\n", __FUNCTION__, __LINE__, realpath, old_dentry->d_name.name);

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_DISK_DEV_PATH) == 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_system_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_secure_storage_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_kitting_device_access_process(realpath) != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (fjsec_check_device_access_process(realpath, ptn_acl) != 0) {
		dump_prinfo(__FUNCTION__, __LINE__);
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			__FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_DEV_PATH) == 0 ||
		strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_DEV2_PATH) == 0 ||
		strcmp(realpath, CONFIG_SECURITY_FJSEC_NFC_DEV_PATH) == 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	r = _xx_realpath_from_path(&new_path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, new_path.dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(r)
	}

	if (strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_DEV_PATH) == 0 ||
		strcmp(realpath, CONFIG_SECURITY_FJSEC_FELICA_DEV2_PATH) == 0 ||
		strcmp(realpath, CONFIG_SECURITY_FJSEC_NFC_DEV_PATH) == 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, realpath, current->comm, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(-EPERM)
	}

	kfree(realpath);
	return 0;
}

static int fjsec_path_truncate(struct path *path)
{
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	r = fjsec_check_path_from_path(path);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	return 0;
}

static int fjsec_path_chmod(struct path *path, umode_t mode)
{
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	r = fjsec_check_path_from_path(path);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	r = fjsec_check_chmod_from_path(path, mode);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	r = fjsec_check_chmod_from_mode(path, mode);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}
	return 0;
}

/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT MOD S */
static int fjsec_path_chown(struct path *path, kuid_t uid, kgid_t gid)
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT MOD E */
{
	int r;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	r = fjsec_check_path_from_path(path);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	r = fjsec_check_chown_from_path(path, uid, gid);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	r = fjsec_check_chown_from_id(path, uid, gid);
	if (r) {
		printk(KERN_INFO "%s:%d: r=%d\n", __FUNCTION__, __LINE__, r);
		FJLSM_REJECT(r)
	}

	return 0;
}
#endif	/* CONFIG_SECURITY_PATH */

/* fjsec_vmalloc_to_sg() is ported from drivers/media/video/videobuf-dma-sg.c
 *
 * Return a scatterlist for some vmalloc()'ed memory
 * block (NULL on errors).  Memory for the scatterlist is allocated
 * using kmalloc.  The caller must free the memory.
 */
static struct scatterlist *fjsec_vmalloc_to_sg(void *buf, unsigned long buflen)
{
	struct scatterlist *sglist;
	struct page *pg;
	int index;
	int pages;
	int size;

	pages = PAGE_ALIGN(buflen) >> PAGE_SHIFT;

	sglist = vmalloc((unsigned long)(pages * sizeof(*sglist)));
	if (sglist == NULL) {
		printk(KERN_INFO "%s:%d\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	sg_init_table(sglist, pages);
	for (index = 0; index < pages; index++, buf += PAGE_SIZE) {
		pg = vmalloc_to_page(buf);
		if (pg == NULL) {
			printk(KERN_INFO "%s:%d\n", __FUNCTION__, __LINE__);
			vfree(sglist);
			return NULL;
		}

		size = buflen - (index * PAGE_SIZE);
		if (size > PAGE_SIZE) {
			size = PAGE_SIZE;
		}

		sg_set_page(&sglist[index], pg, size, 0);
	}
	return sglist;
}

static char *get_checksum(void *buf, unsigned long buflen)
{
	int ret;
	char *output;
	struct crypto_hash *tfm;
	struct scatterlist *sg;
	struct hash_desc desc;

	tfm = crypto_alloc_hash("sha256", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm)) {
		 printk(KERN_INFO "%s:%d\n", __FUNCTION__, __LINE__);
		 return NULL;
	}

	output = vmalloc((unsigned long) crypto_hash_digestsize(tfm));
	if (output == NULL) {
		crypto_free_hash(tfm);
		printk(KERN_INFO "%s:%d\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	desc.tfm = tfm;
	ret = crypto_hash_init(&desc);
	if (ret != 0) {
		vfree(output);
		crypto_free_hash(tfm);
		printk(KERN_INFO "%s:%d\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	sg = fjsec_vmalloc_to_sg(buf, buflen);
	if (sg == NULL) {
		vfree(output);
		crypto_free_hash(tfm);
		printk(KERN_INFO "%s:%d\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	ret = crypto_hash_digest(&desc, sg, buflen, output);
	if (ret != 0) {
		vfree(sg);
		vfree(output);
		crypto_free_hash(tfm);
		printk(KERN_INFO "%s:%d\n", __FUNCTION__, __LINE__);
		return NULL;
	}

	vfree(sg);
	crypto_free_hash(tfm);
	return output;
}

#ifdef LSM_PAGEACL_AUTOREGIST
void __fjsec_dump_page_acl()
{
	struct ac_config_page *pc;
	int i =0;
	for (pc = page_acl; strncmp(pc->process_name, "0", 1) != 0; pc++,i++)
  {
		printk(KERN_INFO "%s:%d(LSM_DUMP_PAGEACL) : page_acl[%d].id=%s .process_name=%s .process_path=%s .uid=%d .start_pfn(0x%lx) .end_pfn(0x%lx)\n",
			__FUNCTION__, __LINE__, i, page_acl[i].id, page_acl[i].process_name, page_acl[i].process_path, page_acl[i].uid,  page_acl[i].start_pfn, page_acl[i].end_pfn);
  }
}


void __fjsec_auto_regist_page_acl(char *id, char* procpath, unsigned long set_start_pfn, unsigned long set_end_pfn, int index)
{
	static int count = 0;

	if(strncmp(page_acl[index].id, ISEMPTY, sizeof(ISEMPTY))!=0) {
	  return ;
	}

	if( ++count >= AUTO_REGIST_DUMP_INTERVAL ) {
		__fjsec_dump_page_acl();
		count = 0;
	}
	dprintk(KERN_INFO "%s:%d(LSM_DEBUG) : page_acl[%d].id=%s .process_name=%s .uid=%d, startpfn(0x%lx) endpfn(0x%lx)\n",
		__FUNCTION__, __LINE__, index, id, current->comm, (uid_t)CURRENT_UID, set_start_pfn, set_end_pfn);

	// register to page_acl
#ifndef LSM_STRICT_MEMORY_DEBUG
	mutex_lock(&kernel_hardening_lock);
	set_memory_rw2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif
	strcpy(page_acl[index].id, id);
//	strcat(page_acl[index].id, AUTO_REGIST_TOKEN);
	strcpy(page_acl[index].process_name, current->comm);
	if (procpath) {
		strcpy(page_acl[index].process_path, procpath);
	} else {
		strcpy(page_acl[index].process_path, NO_CHECK_STR);
	}
	page_acl[index].uid = (uid_t)CURRENT_UID;
	page_acl[index].start_pfn = set_start_pfn;
	page_acl[index].end_pfn = set_end_pfn;
#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
	mutex_unlock(&kernel_hardening_lock);
#endif

}
#endif /* LSM_PAGEACL_AUTOREGIST */

// *New:add*
void fjsec_page_acl_preset(const char *id, long param1, long param2)
{
	struct ac_config_page *pc;
	int i =PAGE_CHECK_OFFSET;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return ;

	// For debugging
	dprintk(KERN_INFO "%s:%d : id=%s, param1=%ld, param2=%ld\n", __FUNCTION__, __LINE__, id, param1, param2);

	for (pc = page_acl_pre + PAGE_CHECK_OFFSET; strncmp(pc->process_name, "0", 1) != 0; pc++,i++) {
		/* id is different */
		if (strcmp(id, pc->id) != 0)
			continue;
		/* Process path is not NO_CHECK_STR, then skip */
		if ( strcmp(pc->process_path, NO_CHECK_STR) != 0 )
			continue;
		/* pfn_flg is not NCON, then skip */
		if (pc->pfn_flg != PG_ACL_NCON)
			continue;
		/* Either supl1 or 2 param is filled with some values, then skip */
		if (pc->supl1 != SUPL_NO_CHECK || pc->supl2 != SUPL_NO_CHECK)
			continue;

		/* set supl1, and once it's set, never change it */
		if (pc->supl1 == SUPL_NO_CHECK) {
			pc->supl1 = param1;
			pc->pfn_flg = PG_ACL_NCONS;
			dprintk(KERN_INFO "%s:%d : supl1=%ld\n", __FUNCTION__, __LINE__, pc->supl1);
		}

		/* set supl2, and once it's set, never change it */
		if (pc->supl2 == SUPL_NO_CHECK) {
			pc->supl2 = param2;
			pc->pfn_flg = PG_ACL_NCONS;
			dprintk(KERN_INFO "%s:%d : supl2=%ld\n", __FUNCTION__, __LINE__, pc->supl2);
		}

		if (page_acl != NULL) {
#ifndef LSM_STRICT_MEMORY_DEBUG
		mutex_lock(&kernel_hardening_lock);
		set_memory_rw2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
		page_acl[i].supl1 = pc->supl1;
		page_acl[i].supl2 = pc->supl2;
		page_acl[i].pfn_flg = pc->pfn_flg;
#ifndef LSM_STRICT_MEMORY_DEBUG
		set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
		mutex_unlock(&kernel_hardening_lock);
#endif /* LSM_STRICT_MEMORY_DEBUG */
		dprintk(KERN_INFO "%s:%d : page_acl[%d].supl1=%ld, page_acl[%d].supl2=%ld\n", __FUNCTION__, __LINE__, i, page_acl[i].supl1, i, page_acl[i].supl2);
		}
		break;
	}

}


/* FUJITSU:2015-3-04 ADD START */
// For performance improvement
static inline int getprocinfo(unsigned long* text, unsigned long* data) {

	struct mm_struct *mm = current->mm;

	if (!mm || !mm->end_code || !text || !data) {
		printk(KERN_ERR "%s:%d: Unable to refer to the mminfo.\n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	*text = mm->end_code - mm->start_code;
	*data = mm->end_data - mm->start_data;

	return 0;
}

/*New process reducing info*/
struct _prevacl {
	int				pid;
	int				uid;
	unsigned int	idx;
	char			id[32];
	unsigned long	size_c;
	unsigned long	size_d;
};

static struct _prevacl prev_acl = { -1, -1, 0, "", 0, 0 };
static struct _prevacl *prev_acl_b = NULL;

static struct timeval cur_tv;
#define VALID_INTERVAL_TIME		50	//50 microsec
#define PATH_BUF_SZ		1024
/* FUJITSU:2015-3-04 ADD END */

static int fjsec_check_process_name(const char* cur_name, const char* acl_name)
{
	char* strano = NULL;
	unsigned int len = 0;
	unsigned int acl_len = 0;

	if( !cur_name || !acl_name ) return -EINVAL;

	acl_len = strlen(acl_name);
	strano = strrchr(acl_name, '*');
	len = strano? strano-acl_name : acl_len;

	if( !strncmp(cur_name, acl_name, len) ) {
		return 0;
	}
	return -1;
}


#ifdef LSM_PAGEACL_PERFDBG
// debug
static int mkacl = 0;
#endif

void fjsec_make_page_acl(char *id, unsigned long addr, unsigned long size)
{
	struct ac_config_page *pc;
	int i = PAGE_CHECK_OFFSET;
	unsigned long set_start_pfn, set_end_pfn;
#ifdef LSM_PAGEACL_AUTOREGIST
	int retSts=-1;
#endif /* LSM_PAGEACL_AUTOREGIST */
	char *binname = NULL;
/* FUJITSU:2015-3-04 MOD START */
	// Change buffer type from heap to stack
	char buf[PATH_BUF_SZ] = {0};
/* FUJITSU:2015-3-04 MOD END */

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return ;

	set_start_pfn = __phys_to_pfn(addr) - PHYS_PFN_OFFSET;
	set_end_pfn = set_start_pfn + (PAGE_ALIGN(size) >> PAGE_SHIFT);

		dprintk(KERN_INFO "%s:%d(LSM_DEBUG) : . Id=%s process_name=%s start_pfn=0x%lx end_pfn=0x%lx current->pid=%d, uid=%d\n",
			__FUNCTION__, __LINE__, id, current->comm, set_start_pfn, set_end_pfn, current->pid, CURRENT_UID);

/* FUJITSU:2015-3-04 ADD START */
// For performance improvement
//	getprocinfo(&size_c, &size_d);
	if( strcmp(id, "binder") == 0 &&
		prev_acl_b->pid == current->pid &&
		prev_acl_b->uid == CURRENT_UID ){
//		prev_acl_b->size_c == size_c && prev_acl_b->size_d == size_d ){

		dprintk(KERN_INFO "%s:%d(LSM_DEBUG) : regist skipped. Id=%s process_name=%s start_pfn=0x%lx end_pfn=0x%lx current->pid=%d, uid=%d\n",
			__FUNCTION__, __LINE__, id, current->comm, set_start_pfn, set_end_pfn, current->pid, CURRENT_UID);
		return;
	}
/* FUJITSU:2015-3-04 ADD END */

	binname = get_process_path_mapping(current, buf, PATH_BUF_SZ-1);
	if (binname == NULL || IS_ERR(binname)) {
		printk(KERN_INFO "%s:%d: Failed getting process path. process name=%s, uid=%d, pid=%d\n"
			   , __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return;
	}

	dprintk(KERN_INFO "%s:%d(LSM_DEBUG) :Id=%s process_name=%s procpath=%s start_pfn=0x%lx end_pfn=0x%lx size=0x%lx addr=0x%p current->pid=%d\n",
			__FUNCTION__, __LINE__, id, current->comm, binname, set_start_pfn, set_end_pfn, size, (void *)addr,current->pid);

/* FUJITSU:2015-3-04 MOD START */
	if( !pfn_sec_valid(set_start_pfn + PHYS_PFN_OFFSET) || !pfn_sec_valid(set_end_pfn + PHYS_PFN_OFFSET) )
/* FUJITSU:2015-3-04 MOD END */
	{
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Invalid page file number. process name=%s procpath=%s start_pfn=0x%lx end_pfn=0x%lx size=0x%lx addr=0x%p\n",
		__FUNCTION__, __LINE__, current->comm, binname, set_start_pfn, set_end_pfn, size, (void *)addr);
		return;
	}

#ifdef LSM_PAGEACL_AUTOREGIST
	if (page_acl == NULL) {
		return ;
	}

	for (pc=page_acl + PAGE_CHECK_OFFSET; strncmp(pc->process_name, "0", 1) != 0; pc++,i++) {
		if (strcmp(id, pc->id) != 0)
			continue;
		if ( strcmp(pc->process_name, NO_CHECK_STR) != 0 && strcmp(current->comm, pc->process_name) != 0 )
			continue;
		/* Process path handling is modified */
		if ( strcmp(pc->process_path, NO_CHECK_STR) != 0 ) {
				if( strcmp(pc->process_path, CHECK_COMMON) != 0 ) {
					if( strcmp(binname, pc->process_path) != 0 ) {
						continue;
					}
				} else {
					if( strncmp(binname, SYSTEM_PROCESS_PATH, strlen(SYSTEM_PROCESS_PATH)) != 0 &&
						 strncmp(binname, INIT_PROCESS_PATH, strlen(INIT_PROCESS_PATH)) != 0 ) {
							continue;
					}
				}
		}
		if (pc->uid != UID_NO_CHECK && pc->uid != CURRENT_UID)
			continue;
		
#ifndef LSM_STRICT_MEMORY_DEBUG
		mutex_lock(&kernel_hardening_lock);
		set_memory_rw2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
		if (pc->start_pfn) {
			if(pc->start_pfn > set_start_pfn) 
				pc->start_pfn = set_start_pfn;

			if(pc->end_pfn < set_end_pfn)
				pc->end_pfn = set_end_pfn;
		} else {
			pc->start_pfn = set_start_pfn;
			pc->end_pfn = set_end_pfn;
		}
#ifndef LSM_STRICT_MEMORY_DEBUG
		set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
		mutex_unlock(&kernel_hardening_lock);
#endif /* LSM_STRICT_MEMORY_DEBUG */

		printk(KERN_INFO "%s:%d(LSM_DEBUG) : success. page_acl[%d].id=%s .process_name=%s .process_path=%s .uid=%d, startpfn(0x%lx) endpfn(0x%lx)\n",
			__FUNCTION__, __LINE__, i, pc->id, pc->process_name, pc->process_path, pc->uid, pc->start_pfn, pc->end_pfn);
		retSts=0;
		break;
	}

	if (retSts!=0)
		__fjsec_auto_regist_page_acl(id, binname, set_start_pfn, set_end_pfn, i);

#else /* LSM_PAGEACL_AUTOREGIST */
	for (pc = page_acl_pre + PAGE_CHECK_OFFSET; strncmp(pc->process_name, "0", 1) != 0; pc++,i++) {
		if (strcmp(id, pc->id) != 0)
			continue;
		if ( strcmp(pc->process_name, NO_CHECK_STR) != 0 && strcmp(current->comm, pc->process_name) != 0 )
			continue;
		/* Process path handling is modified */
		if ( strcmp(pc->process_path, NO_CHECK_STR) != 0 ) {
				if( strcmp(pc->process_path, CHECK_COMMON) != 0 ) {
					if( strcmp(binname, pc->process_path) != 0 ) {
						continue;
					}
				} else {
					if( strncmp(binname, SYSTEM_PROCESS_PATH, strlen(SYSTEM_PROCESS_PATH)) != 0 &&
						 strncmp(binname, INIT_PROCESS_PATH, strlen(INIT_PROCESS_PATH)) != 0 ) {
							continue;
					}
				}
		}
		if (pc->uid != UID_NO_CHECK && pc->uid != CURRENT_UID)
			continue;

		/* pfn flag control add */
		if (pc->pfn_flg == PG_ACL_NCON) {
#if 0
			if (pc->start_pfn) {
				if(pc->start_pfn > set_start_pfn) 
					pc->start_pfn = set_start_pfn;

				if(pc->end_pfn < set_end_pfn)
					pc->end_pfn = set_end_pfn;
			} else {
#endif
				// New : always set the current pfn range
				pc->start_pfn = set_start_pfn;
				pc->end_pfn = set_end_pfn;
				break;
//			}
		} else if(pc->pfn_flg == PG_ACL_NCONS){
				printk(KERN_ERR "%s:%d : access denied. supl param check is necessary. id=%s, startpfn(0x%lx) endpfn(0x%lx)\n",
				__FUNCTION__, __LINE__, pc->id, set_start_pfn, set_end_pfn);
				return;
		} else if(pc->pfn_flg == PG_ACL_ONE){
			if (pc->start_pfn) {
				if(pc->start_pfn > set_start_pfn)
					pc->start_pfn = set_start_pfn;

				if(pc->end_pfn < set_end_pfn)
					pc->end_pfn = set_end_pfn;
			} else {
				pc->start_pfn = set_start_pfn;
				pc->end_pfn = set_end_pfn;
			}
			pc->pfn_flg = PG_ACL_CON;
			break;
		} else {
			if( pc->start_pfn != set_start_pfn || pc->end_pfn != set_end_pfn ) {
				printk(KERN_INFO "%s:%d : mod access denied. page_acl[%d].id=%s .process_name=%s .process_path=%s .uid=%d, startpfn(0x%lx) endpfn(0x%lx)\n",
				__FUNCTION__, __LINE__, i, pc->id, pc->process_name, pc->process_path, pc->uid, pc->start_pfn, pc->end_pfn);
			}
			return;
		}
	}

	if (page_acl != NULL && strncmp(pc->process_name, "0", 1) != 0) {
#ifndef LSM_STRICT_MEMORY_DEBUG
		mutex_lock(&kernel_hardening_lock);
		set_memory_rw2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
		page_acl[i].start_pfn = pc->start_pfn;
		page_acl[i].end_pfn = pc->end_pfn;
		page_acl[i].pfn_flg = pc->pfn_flg;
#ifndef LSM_STRICT_MEMORY_DEBUG
		set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
		mutex_unlock(&kernel_hardening_lock);
#endif /* LSM_STRICT_MEMORY_DEBUG */
	}
#endif /* LSM_PAGEACL_AUTOREGIST */
/* FUJITSU:2015-3-04 ADD START */
#ifdef LSM_PAGEACL_PERFDBG
	mkacl = 1;
#endif
/* FUJITSU:2015-3-04 ADD END */
}

/* FUJITSU:2015-3-04 ADD START */
void fjsec_page_acl_pre_update(const char* name, unsigned long start_pfn, unsigned long size)
{
	unsigned int i;
	struct ac_config_page *pc;
	for(i=0, pc=page_acl_pre; i<PAGE_CHECK_OFFSET; i++, pc++) {
		if( strncmp(name, pc->id, strlen(name)) == 0 && strncmp(pc->process_name, current->comm, strlen(pc->process_name)) == 0 ) {
			pc->start_pfn = start_pfn;
			pc->end_pfn = start_pfn + (ALIGN(size, PAGE_SIZE) >> PAGE_SHIFT);
			printk(KERN_INFO "%s:%d(LSM_INFO) : update page_acl_pre ok. page_acl[%d].id=%s prname=%s, startpfn(0x%lx) endpfn(0x%lx) supl1=0x%lx, supl2=%ld\n",
				__FUNCTION__, __LINE__, i, pc->id, current->comm, pc->start_pfn, pc->end_pfn, pc->supl1, pc->supl2);

			if (page_acl != NULL) {
#ifndef LSM_STRICT_MEMORY_DEBUG
				mutex_lock(&kernel_hardening_lock);
				set_memory_rw2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
				page_acl[i].start_pfn = pc->start_pfn;
				page_acl[i].end_pfn = pc->end_pfn;
#ifndef LSM_STRICT_MEMORY_DEBUG
				set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
				mutex_unlock(&kernel_hardening_lock);
#endif /* LSM_STRICT_MEMORY_DEBUG */
			}
			return;
		}
	}

	printk(KERN_INFO "%s:%d(LSM_INFO) : update page_acl_pre ng. name=%s startpfn(0x%lx) size(%lu)\n",
				__FUNCTION__, __LINE__, name, start_pfn, size);
}
/* FUJITSU:2015-3-04 ADD END */

static void fjsec_copy_page_acl(void)
{
	page_acl = vmalloc(sizeof(page_acl_pre));
	if (!page_acl) {
		printk(KERN_INFO "%s:%d: Failed creating page_acl\n", __FUNCTION__, __LINE__);
		return;
	}
	memset(page_acl, 0, sizeof(page_acl_pre));

	memcpy(page_acl, page_acl_pre, sizeof(page_acl_pre));

#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
	set_memory_nx2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
}

static void fjsec_init_prev_acl(struct _prevacl **prev_acl_table)
{
	*prev_acl_table = vmalloc(sizeof(prev_acl));
	if (!prev_acl_table) {
		printk(KERN_INFO "%s:%d: Failed creating prev_acl_table\n", __FUNCTION__, __LINE__);
		return;
	}

	(*prev_acl_table)->pid = -1;
	(*prev_acl_table)->uid = -1;
	(*prev_acl_table)->idx = 0;
	memset((*prev_acl_table)->id, '\0', sizeof((*prev_acl_table)->id));
	(*prev_acl_table)->size_c = 0;
	(*prev_acl_table)->size_d = 0;

#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) *prev_acl_table, ALIGN(sizeof(prev_acl), PAGE_SIZE) >> PAGE_SHIFT);
	set_memory_nx2((unsigned long) *prev_acl_table, ALIGN(sizeof(prev_acl), PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
}

void fjsec_init_list(void)
{
/* FUJITSU:2015-3-04 ADD START */
#ifdef FIDO_SEC
	unsigned long fido_acl_size = 0;
#endif
/* FUJITSU:2015-3-04 ADD END */

	page_free_list = vmalloc(MAPPING_LIST_SIZE);
	memset(page_free_list, 0, MAPPING_LIST_SIZE);
#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) page_free_list, MAPPING_LIST_PAGE);
	set_memory_nx2((unsigned long) page_free_list, MAPPING_LIST_PAGE);
#endif /* LSM_STRICT_MEMORY_DEBUG */

	page_write_list = vmalloc(MAPPING_LIST_SIZE);
	memset(page_write_list, 0, MAPPING_LIST_SIZE);
#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) page_write_list, MAPPING_LIST_PAGE);
	set_memory_nx2((unsigned long) page_write_list, MAPPING_LIST_PAGE);
#endif /* LSM_STRICT_MEMORY_DEBUG */

	page_exec_list = vmalloc(MAPPING_LIST_SIZE);
	memset(page_exec_list, 0, MAPPING_LIST_SIZE);
#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) page_exec_list, MAPPING_LIST_PAGE);
	set_memory_nx2((unsigned long) page_exec_list, MAPPING_LIST_PAGE);
#endif /* LSM_STRICT_MEMORY_DEBUG */

	page_module_list = vmalloc(MAPPING_LIST_SIZE);
	memset(page_module_list, 0, MAPPING_LIST_SIZE);
#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) page_module_list, MAPPING_LIST_PAGE);
	set_memory_nx2((unsigned long) page_module_list, MAPPING_LIST_PAGE);
#endif /* LSM_STRICT_MEMORY_DEBUG */

	spin_lock_init(&page_module_list_lock);

	fjsec_copy_page_acl();

	fjsec_init_prev_acl(&prev_acl_b);

/* FUJITSU:2015-3-04 ADD START */
#ifdef FIDO_SEC
	fido_acl_size = sizeof(struct ac_config_fido) * ( sizeof(ac_config_fido_acl)/sizeof(ac_config_fido_acl[0]) );
	fido_acl = (struct ac_config_fido *)vmalloc(fido_acl_size);
	memset(fido_acl, 0, fido_acl_size);
#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) fido_acl, ALIGN(fido_acl_size, PAGE_SIZE) >> PAGE_SHIFT);
	set_memory_nx2((unsigned long) fido_acl, ALIGN(fido_acl_size, PAGE_SIZE) >> PAGE_SHIFT);
#endif /* LSM_STRICT_MEMORY_DEBUG */
#endif //FIDO_SEC
/* FUJITSU:2015-3-04 ADD END */

}

pte_t *lookup_address(unsigned long address, unsigned int *level)
{
	pgd_t *pgd = pgd_offset_k(address);
	pte_t *pte;
	pmd_t *pmd;

	/* pmds are folded into pgds on ARM */
	*level = PG_LEVEL_NONE;

	if (pgd == NULL || pgd_none(*pgd))
		return NULL;

	/* pmds are folded into pgds on ARM */
	pmd = (pmd_t *)pgd;

	if (pmd == NULL || pmd_none(*pmd) || !pmd_present(*pmd))
		return NULL;

/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
	if (!pmd_present(*pmd)) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
		return NULL;
	} else if (pmd_val(*pmd) & PMD_TYPE_SECT) {
		*level = PG_LEVEL_2M;
		return (pte_t *)pmd;
	}

	pte = pte_offset_kernel(pmd, address);

	if ((pte == NULL) || pte_none(*pte))
		return NULL;

	*level = PG_LEVEL_4K;

	return pte;
}

void fjsec_make_freepage_list(void)
{

	unsigned long pfn = 0;
	void *free_address;
	pte_t *free_pte;
    unsigned int level;


#ifndef LSM_STRICT_MEMORY_DEBUG
	mutex_lock(&kernel_hardening_lock);
	set_memory_rw2((unsigned long) page_free_list, MAPPING_LIST_PAGE);
	set_memory_rw2((unsigned long) page_write_list, MAPPING_LIST_PAGE);
	set_memory_rw2((unsigned long) page_exec_list, MAPPING_LIST_PAGE);
#endif /* LSM_STRICT_MEMORY_DEBUG */

  printk(KERN_INFO "%s:%d: max_low_pfn=0x%x MAPPING_LIST_SIZE=0x%x\n", __FUNCTION__, __LINE__, (int)MAX_LOW_PFN, (int)MAPPING_LIST_SIZE);

	do {
		if(!pfn_sec_valid(pfn+PHYS_PFN_OFFSET))
			continue;

		if(page_count(pfn_to_page(pfn+PHYS_PFN_OFFSET)))
			continue;

		page_free_list[pfn >> 3] |= (1 << (pfn & 7));

		free_address = page_address(pfn_to_page(pfn+PHYS_PFN_OFFSET));
		if(!free_address)
			continue;

		free_pte = lookup_address((unsigned long)free_address,&level);
		if(level == PG_LEVEL_4K) {
			if(pte_write(*free_pte)) {
				page_write_list[pfn >> 3] |= (1 << (pfn & 7));
			}
			if(pte_exec(*free_pte)) {
				page_exec_list[pfn >> 3] |= (1 << (pfn & 7));
			}
		}else if (level == PG_LEVEL_2M) {
			unsigned long pmd;
			pmd = pmd_val(*free_pte);
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
			if (!(pmd & PMD_SECT_UXN)){
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
				page_exec_list[pfn >> 3] |= (1 << (pfn & 7));
			}
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
			if (!(pmd  & PMD_SECT_RDONLY)) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
				page_write_list[pfn >> 3] |= (1 << (pfn & 7));
			}
		}else {
			page_write_list[pfn >> 3] |= (1 << (pfn & 7));
			page_exec_list[pfn >> 3] |= (1 << (pfn & 7));
		}

	} while (pfn++, pfn < MAX_LOW_PFN);

#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long) page_free_list, MAPPING_LIST_PAGE);
	set_memory_ro2((unsigned long) page_write_list, MAPPING_LIST_PAGE);
	set_memory_ro2((unsigned long) page_exec_list, MAPPING_LIST_PAGE);
	mutex_unlock(&kernel_hardening_lock);
#endif /* LSM_STRICT_MEMORY_DEBUG */
}

static int fjsec_check_page_acl_ext(void) {
	struct ac_config_page_ext *pc;
	bool find_module = false;

	for (pc = page_acl_ext; strncmp(pc->process_name, "0", 1) != 0; pc++) {
		if (strcmp(pc->process_name, NO_CHECK_STR) != 0 && fjsec_check_process_name(current->comm, pc->process_name) != 0)
			continue;

		if (strcmp(pc->process_path, NO_CHECK_STR) != 0 && fjsec_check_access_process_path_mapping(pc->process_path))
			continue;

		if (pc->uid != UID_NO_CHECK && pc->uid != CURRENT_UID)
			continue;

		find_module = true;
		break;
	}
/* FCNT LIMITED: 2017-03-08 SEC_LSM_047 ADD start */
#ifdef CTS_TEST
    if(find_module != true){
	    dprintk(KERN_INFO "%s:%d: CTS app's parent process name=%s\n", __FUNCTION__, __LINE__, current->group_leader->comm);
	    for (pc = page_parent_acl_ext; strncmp(pc->process_name, "0", 1) != 0; pc++) {
		    if (strcmp(pc->process_name, NO_CHECK_STR) != 0 && fjsec_check_process_name(current->group_leader->comm, pc->process_name) != 0)
			    continue;

		    if (strcmp(pc->process_path, NO_CHECK_STR) != 0 && fjsec_check_access_parent_process_path_mapping(current->group_leader, pc->process_path))
			    continue;

		    if (pc->uid != UID_NO_CHECK && pc->uid != CURRENT_UID)
		    	continue;

		    find_module = true;
		    break;
	    }
    }
#endif
/* FCNT LIMITED: 2017-03-08 SEC_LSM_047 ADD end */
	dprintk(KERN_INFO "%s:%d: find_module=%d, path=%s\n", __FUNCTION__, __LINE__, find_module, pc->process_path);
	if (find_module && pc->process_path) {
		struct file* filp;
		int file_size = 0;
		mm_segment_t fs;
		char* file_data;
		ssize_t read_size = 0;
		char *checksum = NULL;

		if(pc->supl1 == SUPL_NO_HASHCHECK){
			return 0;
		}

		// file open
		filp = filp_open(pc->process_path, O_RDONLY, 0);
		if (IS_ERR(filp)) {
			printk(KERN_INFO "%s:%d: open error\n", __FUNCTION__, __LINE__);
			return -EPERM;
		}

		fs = get_fs();
		set_fs(get_ds());
		file_size = filp->f_op->llseek(filp, 0, SEEK_END);
		filp->f_op->llseek(filp, 0, SEEK_SET);

		dprintk(KERN_INFO "%s:%d: file open size = %d\n", __FUNCTION__, __LINE__, file_size);
		file_data = vmalloc(file_size);
		if (!file_data) {
			printk(KERN_INFO "%s:%d:allocate error\n", __FUNCTION__, __LINE__);
			set_fs(fs);
			filp_close(filp, NULL);
			return -EPERM;
		}

		memset(file_data, 0, file_size);
		read_size = filp->f_op->read(filp, file_data, file_size, &filp->f_pos);
		set_fs(fs);
		filp_close(filp, NULL);

		if (read_size <= 0) {
			printk(KERN_INFO "%s:%d:read error\n", __FUNCTION__, __LINE__);
			vfree(file_data);
			return -EPERM;
		}

		checksum = get_checksum(file_data, file_size);
		if (checksum == NULL) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating checksum. process name=%s, uid=%d, pid=%d\n",
				 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
			vfree(file_data);
			return -EPERM;
		}

		if (memcmp((const void *)checksum, (const void *)pc->process_hash, SHA256_DIGEST_SIZE) != 0) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT Mismatched checksum. process name=%s, uid=%d, pid=%d\n",
				 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
			print_hex_dump(KERN_INFO, "checksum: ", DUMP_PREFIX_OFFSET, 16, 1, checksum, SHA256_DIGEST_SIZE, false);
			vfree(file_data);
			vfree(checksum);
			return -EPERM;
		}
		vfree(file_data);
		vfree(checksum);
		return 0;
	}

	return -EPERM;
}

/* FUJITSU:2015-3-04 ADD START */
#define ALT_CHK_MASK_BITS (unsigned int)(0x00100000)
/* FUJITSU:2015-3-04 ADD END */
static int fjsec_check_page_acl(unsigned long pfn, unsigned long size)
{
	struct ac_config_page *pc;
	int i = PAGE_CHECK_OFFSET;

/* FUJITSU LIMITED:2015-06-08 SEC-LSM-006 add start */
#ifdef LSM_PAGEACL_AUTOREGIST
    char * buf = NULL;
#else
    char buf[CTS_CHKBUF] = {0};
#endif
/* FUJITSU LIMITED:2015-06-08 SEC-LSM-006  add end */
    char *binname = NULL;
    //char buf1[CTS_CHKBUF] = {0};

#ifndef LSM_PAGEACL_MANUALADR
	struct ac_config_page *pc_alt;
	int j = 0;
#endif
/* FUJITSU:2015-3-04 ADD START */
	unsigned long start_pfn, end_pfn;
	int ret_procn = 0;
	int ret_procp = 0;
	struct timeval chk_tv;
	unsigned long long diff_jiffies;
	unsigned long size_c = 0;
	unsigned long size_d = 0;
/* FUJITSU:2015-3-04 ADD END */

	start_pfn = pfn;
	end_pfn = pfn + (PAGE_ALIGN(size) >> PAGE_SHIFT);

	if( page_acl == NULL ) {
		printk("page_acl is null");
		return -EPERM;
	}

/* FUJITSU:2015-3-04 ADD START */
	if( prev_acl_b->pid == current->pid &&
		prev_acl_b->uid == CURRENT_UID ){
//		prev_acl_b->size_c == size_c && prev_acl_b->size_d == size_d) {

		dprintk(KERN_INFO "%s:%d: check skipped binder. pfn=0x%lx size=0x%lx start_pfn=0x%lx end_pfn=0x%lx, pid = %d, diff= %llu\n",
			__FUNCTION__, __LINE__, pfn, size, start_pfn, end_pfn, current->pid, diff_jiffies);
		return 0;
	}
	getprocinfo(&size_c, &size_d);

	do_gettimeofday(&chk_tv);
	diff_jiffies = (unsigned long long)(chk_tv.tv_sec - cur_tv.tv_sec)*1000000 + chk_tv.tv_usec - cur_tv.tv_usec;
	if( prev_acl.pid == current->pid &&
	prev_acl.uid == CURRENT_UID &&
	prev_acl.size_c == size_c && prev_acl.size_d == size_d &&
	 (diff_jiffies >=0 && diff_jiffies < VALID_INTERVAL_TIME) ) {
		dprintk(KERN_INFO "%s:%d: check skipped. pfn=0x%lx size=0x%lx start_pfn=0x%lx end_pfn=0x%lx, pid = %d, diff= %llu\n",
				__FUNCTION__, __LINE__, pfn, size, start_pfn, end_pfn, current->pid, diff_jiffies);
#ifdef LSM_PAGEACL_PERFDBG
		dprintk(KERN_INFO "%s:%d: check skipped. pfn=0x%lx size=0x%lx start_pfn=0x%lx end_pfn=0x%lx, pid = %d, diff= %llu\n",
				__FUNCTION__, __LINE__, pfn, size, start_pfn, end_pfn, current->pid, diff_jiffies);
		// debug
		mkacl=0;
#endif
		return 0;
	} else {
		dprintk(KERN_INFO "%s:%d: check not skipped. start_pfn=0x%lx end_pfn=0x%lx, prevpid=%d, pid=%d, diff= %llu, pname=%s, prevuid=%d, uid=%d  size_c=%lu, size_d=%lu\n",
				__FUNCTION__, __LINE__, start_pfn, end_pfn, prev_acl.pid, current->pid, diff_jiffies, current->comm, prev_acl.uid, CURRENT_UID, size_c, size_d);

#ifdef LSM_PAGEACL_PERFDBG
		printk(KERN_INFO "%s:%d: check not skipped. start_pfn=0x%lx end_pfn=0x%lx, prevpid=%d, pid=%d, diff= %llu, pname=%s, prevuid=%d, uid=%d mkacl=%d size_c=%lu, size_d=%lu\n",
				__FUNCTION__, __LINE__, start_pfn, end_pfn, prev_acl.pid, current->pid, diff_jiffies, current->comm, prev_acl.uid, CURRENT_UID, mkacl, size_c, size_d);
		// debug
		mkacl=0;
#endif
		prev_acl.pid = current->pid;
		prev_acl.uid = CURRENT_UID;
		prev_acl.size_c = size_c;
		prev_acl.size_d = size_d;
	}
/* FUJITSU:2015-3-04 ADD END */

	for (pc = page_acl+PAGE_CHECK_OFFSET; strncmp(pc->process_name, "0", 1) != 0; pc++, i++) {
/* FUJITSU:2015-3-04 ADD START */
#ifdef LSM_PAGEACL_SSRDBG
		// debug
		if( strcmp(current->comm, "modem_rd") == 0 ) {
			printk(KERN_INFO "%s:%d(NOREGIST) : id = %d, process_name=%s uid=%d, start_pfn=0x%lx end_pfn=0x%lx, spfn=0x%lx, epfn=0x%lx, flag=0x%x, supl1=0x%lx, supl2=0x%lx\n",
							 __FUNCTION__, __LINE__, i, current->comm, CURRENT_UID, pc->start_pfn, pc->end_pfn, start_pfn, end_pfn, (unsigned int)pc->pfn_flg, pc->supl1, pc->supl2);
		return 0;
		}
#endif
/* FUJITSU:2015-3-04 ADD END */
/* FUJITSU:2015-3-04 MOD START */
		if ((ret_procn = strcmp(pc->process_name, NO_CHECK_STR)) != 0 && fjsec_check_process_name(current->comm, pc->process_name) != 0)
			continue;

		if ((ret_procp = strcmp(pc->process_path, NO_CHECK_STR)) != 0 && fjsec_check_access_process_path_mapping(pc->process_path))
			continue;
/* FUJITSU:2015-3-04 MOD END */

		if (pc->uid != UID_NO_CHECK && pc->uid != CURRENT_UID)
			continue;

		if (!(pc->start_pfn <= start_pfn && end_pfn <= pc->end_pfn))
/* FUJITSU:2015-3-04 ADD/MOD START */
#ifdef LSM_PAGEACL_AUTOREGIST
		{
#ifndef LSM_PAGEACL_MANUALADR
			/* alternative check */
			if( pc->supl1 == SUPL_CHK_ALT_ADDR_IONDMA ) {
				for (j=0, pc_alt = page_acl_pre; j<PAGE_CHECK_OFFSET; pc_alt++, j++) {
					if( pc->supl2 & (ALT_CHK_MASK_BITS << j) ) {
						if (pc_alt->start_pfn <= start_pfn && end_pfn <= pc_alt->end_pfn) {
							dprintk(KERN_INFO "%s:%d(LSM_DEBUG) : alt check ok. memid=0x%lx. process_name=%s uid=%d, start_pfn=0x%lx end_pfn=0x%lx, spfn=0x%lx, epfn=0x%lx\n",
							 __FUNCTION__, __LINE__, pc->supl2, current->comm, CURRENT_UID, pc_alt->start_pfn, pc_alt->end_pfn, start_pfn, end_pfn);
							return 0;
						} else {
							dprintk(KERN_INFO "%s:%d(LSM_DEBUG) : alt check ng. process_name=%s uid=%d, start_pfn=0x%lx end_pfn=0x%lx, spfn=0x%lx, epfn=0x%lx, supl2=0x%lx\n",
							 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, pc_alt->start_pfn, pc_alt->end_pfn, start_pfn, end_pfn, pc->supl2);
						}
					}
				}
			}

			if(!(pc->pfn_flg != PG_ACL_NCONS && (i!=PAGE_CHECK_OFFSET+1 && i!=PAGE_CHECK_OFFSET+2 && i!=PAGE_CHECK_OFFSET+3)))
				 continue;
#endif //LSM_PAGEACL_MANUALADR
			/* auto register in the present pc_acl info array */
			if( ret_procn || ret_procp || pc->uid != UID_NO_CHECK ) {
#ifndef LSM_STRICT_MEMORY_DEBUG
				mutex_lock(&kernel_hardening_lock);
				set_memory_rw2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif
				// adjacent address range handling
				if( pc->end_pfn == start_pfn ) {
					pc->end_pfn = end_pfn;
#ifndef LSM_STRICT_MEMORY_DEBUG
				set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
				mutex_unlock(&kernel_hardening_lock);
#endif
					return 0;
				}

				// adjacent address range handling
				if( pc->start_pfn == end_pfn ) {
					pc->start_pfn = start_pfn;
#ifndef LSM_STRICT_MEMORY_DEBUG
				set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
				mutex_unlock(&kernel_hardening_lock);
#endif
					return 0;
				}

				// if pc_acl info status is default, then get dumpstack.
				if( pc->start_pfn == 0 ) {
#ifdef LSM_PAGEACL_DUMPSTACK
					dump_stack();
#endif
				}

				// Unconditionally, pfn is registered.
				pc->start_pfn = start_pfn;
				pc->end_pfn = end_pfn;

#ifndef LSM_STRICT_MEMORY_DEBUG
				set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
				mutex_unlock(&kernel_hardening_lock);
#endif
// get debug info
//				binname = get_process_path_mapping(current, buf1, CTS_CHKBUF-1);
//				chkprocname(buffer, CTS_CHKBUF);
//				printk(KERN_INFO "%s:%d(LSM_INFO) : set. i=%d  prn=%s, prp=%s, curprn=%s curprp=%s uid=%d start_pfn=0x%lx end_pfn=0x%lx\n",
//				__FUNCTION__, __LINE__, i, pc->process_name, pc->process_path, buffer, binname, CURRENT_UID, pc->start_pfn, pc->end_pfn);
				return 0;
			}

			/* present auto register */
			if( strncmp(pc->id+strlen(pc->id)-strlen(AUTO_REGIST_TOKEN), AUTO_REGIST_TOKEN, strlen(AUTO_REGIST_TOKEN))==0 ) {
				// adjacent address range handling
				if( pc->end_pfn == pfn ) {
#ifndef LSM_STRICT_MEMORY_DEBUG
					mutex_lock(&kernel_hardening_lock);
					set_memory_rw2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif
					pc->end_pfn = pfn + (PAGE_ALIGN(size) >> PAGE_SHIFT);
#ifndef LSM_STRICT_MEMORY_DEBUG
					set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
					mutex_unlock(&kernel_hardening_lock);
#endif
					return 0;
				}
				// adjacent address range handling
				if( pc->start_pfn == (pfn + (PAGE_ALIGN(size) >> PAGE_SHIFT))) {
#ifndef LSM_STRICT_MEMORY_DEBUG
					mutex_lock(&kernel_hardening_lock);
					set_memory_rw2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
#endif
					pc->start_pfn = pfn;
#ifndef LSM_STRICT_MEMORY_DEBUG
					set_memory_ro2((unsigned long) page_acl, ALIGN(sizeof(page_acl_pre), PAGE_SIZE) >> PAGE_SHIFT);
					mutex_unlock(&kernel_hardening_lock);
#endif
					return 0;
				}
			}

			continue;
		}
#else /* LSM_PAGEACL_AUTOREGIST */
		{
#ifndef LSM_PAGEACL_MANUALADR
			/* alternative check */
			if( pc->supl1 == SUPL_CHK_ALT_ADDR_IONDMA ) {
				for (j=0, pc_alt = page_acl_pre; j<PAGE_CHECK_OFFSET; pc_alt++, j++) {
					if( pc->supl2 & (ALT_CHK_MASK_BITS << j) ) {
						if (pc_alt->start_pfn <= start_pfn && end_pfn <= pc_alt->end_pfn) {
							dprintk(KERN_INFO "%s:%d(LSM_DEBUG) : alt check ok. memid=0x%lx. process_name=%s uid=%d, start_pfn=0x%lx end_pfn=0x%lx, spfn=0x%lx, epfn=0x%lx\n",
							 __FUNCTION__, __LINE__, pc->supl2, current->comm, CURRENT_UID, pc_alt->start_pfn, pc_alt->end_pfn, start_pfn, end_pfn);
							return 0;
						} else {
							dprintk(KERN_INFO "%s:%d(LSM_DEBUG) : alt check ng. process_name=%s uid=%d, start_pfn=0x%lx end_pfn=0x%lx, spfn=0x%lx, epfn=0x%lx, supl2=0x%lx\n",
							 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, pc_alt->start_pfn, pc_alt->end_pfn, start_pfn, end_pfn, pc->supl2);
						}
					}
				}
			}
#endif //LSM_PAGEACL_MANUALADR
			continue;
		}
#endif /* LSM_PAGEACL_AUTOREGIST */

#ifdef LSM_PAGEACL_AUTOREGIST
		dprintk(KERN_INFO "%s:%d(LSM_DEBUG) : check ok. page_acl[%d]. process_name=%s uid=%d, pc->start_pfn=0x%lx pc->end_pfn=0x%lx\n",
		__FUNCTION__, __LINE__, i, current->comm, CURRENT_UID, pc->start_pfn, pc->end_pfn);
#endif /* LSM_PAGEACL_AUTOREGIST */

		if(strcmp(pc->id,"binder")== 0 &&
			prev_acl_b->pid != current->pid){

#ifndef LSM_STRICT_MEMORY_DEBUG
			mutex_lock(&kernel_hardening_lock_binder);
			set_memory_rw2((unsigned long) prev_acl_b, ALIGN(sizeof(prev_acl), PAGE_SIZE) >> PAGE_SHIFT);
#endif
			prev_acl_b->pid = current->pid;
			prev_acl_b->uid = CURRENT_UID;
//			prev_acl_b->size_c = size_c;
//			prev_acl_b->size_d = size_d;
#ifndef LSM_STRICT_MEMORY_DEBUG
			set_memory_ro2((unsigned long) prev_acl_b, ALIGN(sizeof(prev_acl), PAGE_SIZE) >> PAGE_SHIFT);
			mutex_unlock(&kernel_hardening_lock_binder);
#endif
		}
		return 0;
	}
#ifndef LSM_PAGEACL_AUTOREGIST

	if (fjsec_check_page_acl_ext() == 0) {
		dprintk(KERN_INFO "%s:%d : fjsec_check_page_acl_ext success \n",__FUNCTION__, __LINE__);
		return 0;
	}
	// get debug info
	binname = get_process_path_mapping(current, buf, CTS_CHKBUF-1);
//	chkprocname(buf1, CTS_CHKBUF);
//	printk(KERN_INFO "%s:%d(FJLSM_REJECT) : path=%s procname=%s uid=%d start_pfn=0x%lx end_pfn=0x%lx\n",
//				__FUNCTION__, __LINE__, binname, buf1, CURRENT_UID, start_pfn, end_pfn);
	printk(KERN_INFO "%s:%d(FJLSM_REJECT) : path=%s uid=%d start_pfn=0x%lx end_pfn=0x%lx\n",
				__FUNCTION__, __LINE__, binname, CURRENT_UID, start_pfn, end_pfn);
#ifdef LSM_PAGEACL_DUMPSTACK
	dump_stack();
#endif
	return -EPERM;
#else /* LSM_PAGEACL_AUTOREGIST */

	// get debug info
	binname = get_process_path_mapping(current, buf, CTS_CHKBUF-1);
	printk(KERN_INFO "%s:%d(FJLSM_REJECT) : path=%s uid=%d start_pfn=0x%lx end_pfn=0x%lx\n",
				__FUNCTION__, __LINE__, binname, CURRENT_UID, start_pfn, end_pfn);
#ifdef LSM_PAGEACL_DUMPSTACK
	dump_stack();
#endif
	return 0;
#endif /* LSM_PAGEACL_AUTOREGIST */
/* FUJITSU:2015-3-04 ADD/MOD END */
}

#define is_free(pfn) (page_free_list[pfn >> 3] & (1 << (pfn & 7)))
#define can_write(pfn) (page_write_list[pfn >> 3] & (1 << (pfn & 7)))
#define can_exec(pfn) (page_exec_list[pfn >> 3] & (1 << (pfn & 7)))
#define is_module(pfn) (page_module_list[pfn >> 3] & (1 << (pfn & 7)))

static int fjsec_check_mapping(unsigned long pfn, unsigned long size, pgprot_t prot)
{

	unsigned long offset = 0;
	int ret = 0;

    dprintk(KERN_INFO "LSM_MMAP_DISABLE:fjsec_check_mapping start.\n");
#ifdef CONFIG_SECURITY_MMAP_DISABLE
	dprintk(KERN_INFO "LSM_MMAP_DISABLE=true.\n");
	return 0;
#endif /* CONFIG_SECURITY_MMAP_DISABLE */

	if(__fjsec_check_nv(FJSEC_LSM_MMAP_ENABLED) == 0
		|| boot_mode == BOOT_MODE_MAKERCMD) {
		return 0;
	}

/* FUJITSU:2015-3-04 MOD START */
	if(pfn > MAX_PFN)
/* FUJITSU:2015-3-04 MOD END */
	{
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Invalid Mapping(pfn over) process name=%s pfn=0x%lx size=0x%lx \n",
			   __FUNCTION__, __LINE__, current->comm, pfn, size);
#ifdef LSM_PAGEACL_AUTOREGIST
		return 0;
#else
		return -EPERM;
#endif
	}

	pfn -= PHYS_PFN_OFFSET;
	do {
		if (!((0 <= pfn) && (pfn < MAX_LOW_PFN)))
			continue;

		if (!is_free(pfn)) {
			if ((ret = fjsec_check_page_acl(pfn, size - offset)) != 0) {
				printk(KERN_INFO "%s:%d: FJLSM_REJECT Invalid Mapping(free_list) process name=%s pfn=0x%lx size=0x%lx addr=0x%p\n",
					   __FUNCTION__, __LINE__, current->comm, pfn, size, page_address(pfn_to_page(pfn+PHYS_PFN_OFFSET)));
//				printk(KERN_INFO "!#! STORE !#!\n"); // LSM debug
				return -EPERM;
			} else {
				return 0;
			}
		}

		if (is_module(pfn)) {
			printk(KERN_INFO "%s:%d: FJLSM_REJECT Invalid Mapping(module_list) process name=%s pfn=0x%lx size=0x%lx\n",
				   __FUNCTION__, __LINE__, current->comm, pfn, size);
//			printk(KERN_INFO "!#! STORE !#!\n"); // LSM debug
			return -EPERM;
		}

/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
		if (!(pgprot_val(prot) & PTE_RDONLY) && !can_write(pfn)) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
			printk(KERN_INFO "%s:%d: FJLSM_REJECT Invalid Mapping(write_list) process name=%s pfn=0x%lx size=0x%lx\n",
				   __FUNCTION__, __LINE__, current->comm, pfn, size);
//			printk(KERN_INFO "!#! STORE !#!\n"); // LSM debug
			return -EPERM;
		}

/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
		if (!(pgprot_val(prot) & PTE_UXN) && !can_exec(pfn)) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
			printk(KERN_INFO "%s:%d: FJLSM_REJECT Invalid Mapping(exec_list) process name=%s pfn=0x%lx size=0x%lx\n",
				   __FUNCTION__, __LINE__, current->comm, pfn, size);
//			printk(KERN_INFO "!#! STORE !#!\n"); // LSM debug
			return -EPERM;
		}
	} while (offset += PAGE_SIZE, pfn++, offset < size);

	return 0;
}

static int fjsec_remap_pfn_range(unsigned long pfn, unsigned long size,pgprot_t prot, bool page_acl_flg)
{
	int ret;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0 || page_acl_flg){
                dprintk(KERN_INFO "%s:%d: skip checking by page_acl_flg : process name=%s\n", __FUNCTION__, __LINE__, current->comm);
		return 0;
        }

	ret = fjsec_check_mapping(pfn, size, prot);
	if(ret){
		printk(KERN_INFO "%s:%d: process name=%s\n", __FUNCTION__, __LINE__, current->comm);
		ret = -EFAULT;
	}
	FJLSM_REJECT(ret)
}

static int fjsec_ioremap_page_range(unsigned long addr,unsigned long end, phys_addr_t phys_addr, pgprot_t prot)
{
	int ret;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	ret = fjsec_check_mapping(__phys_to_pfn(phys_addr), end - addr, prot);
	if(ret){
		printk(KERN_INFO "%s:%d: process name=%s\n", __FUNCTION__, __LINE__, current->comm);
		ret = -EFAULT;
	}
	FJLSM_REJECT(ret)
}

static int fjsec_insert_page(struct page *page, pgprot_t prot, bool page_acl_flg)
{
	int ret;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0 || page_acl_flg){
		dprintk(KERN_INFO "%s:%d: skip checking by page_acl_flg : process name=%s\n", __FUNCTION__, __LINE__, current->comm);
		return 0;
        }

	ret = fjsec_check_mapping(page_to_pfn(page), PAGE_SIZE, prot);
	if(ret){
		printk(KERN_INFO "%s:%d: process name=%s\n", __FUNCTION__, __LINE__, current->comm);
		ret = -EFAULT;
	}
	FJLSM_REJECT(ret)
}

static int fjsec_insert_pfn(unsigned long pfn, pgprot_t prot)
{
	int ret;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	ret = fjsec_check_mapping(pfn, PAGE_SIZE, prot);
	if(ret){
		printk(KERN_INFO "%s:%d: process name=%s\n", __FUNCTION__, __LINE__, current->comm);
		ret = -EFAULT;
	}
	FJLSM_REJECT(ret)
}

static int fjsec_check_setid_access_process(void)
{
	char *buf = NULL;
	char *binname;
	struct ac_config_setid *config;

	if (CURRENT_UID != AID_ROOT) {
		return 0;
	}

	buf = kzalloc(PATH_MAX, GFP_NOFS);
	if (!buf) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer. process name=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		return -ENOMEM;
	}

	binname = get_process_path(current, buf, PATH_MAX-1);
	if (binname == NULL || IS_ERR(binname)) {
		printk(KERN_INFO "%s:%d: Failed getting process path. process name=%s, uid=%d, pid=%d\n"
			   , __FUNCTION__, __LINE__, current->comm, CURRENT_UID, current->pid);
		kfree(buf);
		return -EPERM;
	}

	dprintk(KERN_INFO "%s:%d: process path=%s\n", __FUNCTION__, __LINE__, binname);

	if (strncmp(binname, CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH,
				strlen(CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH)) == 0) {
		kfree(buf);
		return 0;
	}

	for (config = setid_acl; config->process_name; config++) {
		if (strcmp(current->comm, config->process_name) == 0) {
			if (strcmp(binname, config->process_path) == 0) {
				kfree(buf);
				return 0;
			}
		}
	}
	printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s process name=%s\n",
		   __FUNCTION__, __LINE__, binname, current->comm);

	kfree(buf);
	return -EPERM;


}
#if 1
static int fjsec_task_fix_setuid(struct cred *new, const struct cred *old,int flags)
{
	int ret;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	ret = cap_task_fix_setuid(new, old, flags);
	if(ret)
		return ret;
	ret = fjsec_check_setid_access_process();
	if (ret) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT flags %d\n", __FUNCTION__, __LINE__, flags);
	}
	FJLSM_REJECT(ret)
}
#else
extern inline void cap_emulate_setxuid(struct cred *new, const struct cred *old);
static int fjsec_task_fix_setuid(struct cred *new, const struct cred *old,int flags)
{
	int ret;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	switch (flags) {
	case LSM_SETID_RE:
	case LSM_SETID_ID:
	case LSM_SETID_RES:
		/* juggle the capabilities to follow [RES]UID changes unless
		 * otherwise suppressed */
		if (!issecure(SECURE_NO_SETUID_FIXUP))
			cap_emulate_setxuid(new, old);
		break;

	case LSM_SETID_FS:
		/* juggle the capabilties to follow FSUID changes, unless
		 * otherwise suppressed
		 *
		 * FIXME - is fsuser used for all CAP_FS_MASK capabilities?
		 *          if not, we might be a bit too harsh here.
		 */
		if (!issecure(SECURE_NO_SETUID_FIXUP)) {
			if (old->fsuid == 0 && new->fsuid != 0)
				new->cap_effective =
					cap_drop_fs_set(new->cap_effective);

			if (old->fsuid != 0 && new->fsuid == 0)
				new->cap_effective =
					cap_raise_fs_set(new->cap_effective,
							 new->cap_permitted);
		}
		break;

	default:
		return -EINVAL;
	}

	ret = fjsec_check_setid_access_process();
	if (ret) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT flags %d\n", __FUNCTION__, __LINE__, flags);
		return ret;
	}

	return 0;
}
#endif

static int fjsec_task_fix_setgid(struct cred *new, const struct cred *old,int flags)
{
	int ret;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	ret = fjsec_check_setid_access_process();
	if (ret) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT flags %d\n", __FUNCTION__, __LINE__, flags);
		FJLSM_REJECT(ret)
	}

	return 0;
}

extern void warn_setuid_and_fcaps_mixed(const char *fname);
extern int get_file_caps(struct linux_binprm *bprm, bool *effective, bool *has_cap);
static int fjsec_bprm_set_creds(struct linux_binprm *bprm)
{
	char *realpath = NULL;
	const struct cred *old = current_cred();
	struct cred *new = bprm->cred;
	bool effective, has_cap = false;
	int ret;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	effective = false;
	ret = get_file_caps(bprm, &effective, &has_cap);
	if (ret < 0)
		return ret;

	if (!issecure(SECURE_NOROOT)) {
		/*
		 * If the legacy file capability is set, then don't set privs
		 * for a setuid root binary run by a non-root user.  Do set it
		 * for a root user just to cause least surprise to an admin.
		 */
		if (has_cap && new->uid != 0 && new->euid == 0) {
			warn_setuid_and_fcaps_mixed(bprm->filename);
			goto skip;
		}
		/*
		 * To support inheritance of root-permissions and suid-root
		 * executables under compatibility mode, we override the
		 * capability sets for the file.
		 *
		 * If only the real uid is 0, we do not set the effective bit.
		 */
		if (new->euid == 0 || new->uid == 0) {
			/* pP' = (cap_bset & ~0) | (pI & ~0) */
			new->cap_permitted = cap_combine(old->cap_bset,
							 old->cap_inheritable);
		}
		if (new->euid == 0)
			effective = true;
	}
skip:

	/* if we have fs caps, clear dangerous personality flags */
	if (!cap_issubset(new->cap_permitted, old->cap_permitted))
		bprm->per_clear |= PER_CLEAR_ON_SETID;


	/* Don't let someone trace a set[ug]id/setpcap binary with the revised
	 * credentials unless they have the appropriate permit
	 */
	if ((new->euid != old->uid ||
	     new->egid != old->gid ||
	     !cap_issubset(new->cap_permitted, old->cap_permitted)) &&
	    bprm->unsafe & ~LSM_UNSAFE_PTRACE_CAP) {
		/* downgrade; they get no more than they had, and maybe less */
		if (!capable(CAP_SETUID)) {
			new->euid = new->uid;
			new->egid = new->gid;
		}
		new->cap_permitted = cap_intersect(new->cap_permitted,
						   old->cap_permitted);
	}

	new->suid = new->fsuid = new->euid;
	new->sgid = new->fsgid = new->egid;

	if (effective)
		new->cap_effective = new->cap_permitted;
	else
		cap_clear(new->cap_effective);
	bprm->cap_effective = effective;

	/*
	 * Audit candidate if current->cap_effective is set
	 *
	 * We do not bother to audit if 3 things are true:
	 *   1) cap_effective has all caps
	 *   2) we are root
	 *   3) root is supposed to have all caps (SECURE_NOROOT)
	 * Since this is just a normal root execing a process.
	 *
	 * Number 1 above might fail if you don't have a full bset, but I think
	 * that is interesting information to audit.
	 */
	if (!cap_isclear(new->cap_effective)) {
		if (!cap_issubset(CAP_FULL_SET, new->cap_effective) ||
		    new->euid != 0 || new->uid != 0 ||
		    issecure(SECURE_NOROOT)) {
			ret = audit_log_bprm_fcaps(bprm, new, old);
			if (ret < 0)
				return ret;
		}
	}

	new->securebits &= ~issecure_mask(SECURE_KEEP_CAPS);

	realpath = kzalloc(PATH_MAX, GFP_NOFS);

	if (!realpath) {
		printk(KERN_INFO "%s:%d: InternalError Failed allocating buffer.\n", __FUNCTION__, __LINE__);
		return -ENOMEM;
	}

	ret = _xx_realpath_from_path(&bprm->file->f_path, realpath, PATH_MAX-1);
	if (ret != 0) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT Failed creating realpath. process name=%s\n file->f_path=%s, uid=%d, pid=%d\n",
			 __FUNCTION__, __LINE__, current->comm, bprm->file->f_path.dentry->d_name.name, CURRENT_UID, current->pid);
		kfree(realpath);
		FJLSM_REJECT(ret)
	}

	if (strncmp(realpath, CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH,
				strlen(CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH)) == 0) {
		kfree(realpath);
		return 0;
	}

	if((current->cred->euid != 0) && (bprm->cred->euid == 0)) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s, process name=%s, current euid %d request euid %d\n",
			   __FUNCTION__, __LINE__, realpath, current->comm,current->cred->euid, bprm->cred->euid);
		kfree(realpath);
		FJLSM_REJECT( -EPERM)
	}

	if((current->cred->egid!= 0) && (bprm->cred->egid == 0)) {
		printk(KERN_INFO "%s:%d: FJLSM_REJECT realpath=%s, process name=%s, current egid %d request egid %d\n",
			   __FUNCTION__, __LINE__, realpath, current->comm,current->cred->egid, bprm->cred->egid);
		kfree(realpath);
		FJLSM_REJECT( -EPERM)
	}

	kfree(realpath);
	return 0;
}

static int fjsec_check_pte_range(struct mm_struct *mm, pmd_t *pmd,
            unsigned long addr, unsigned long end, pgprot_t prot)
{
	pte_t *pte;
	spinlock_t *ptl;
	int ret = 0;

	pte = pte_offset_map_lock(mm, pmd, addr, &ptl);
	arch_enter_lazy_mmu_mode();
	do {
		if (pte_present(*pte)) {
			ret = fjsec_check_mapping(pte_pfn(*pte), 1, prot);
			if (ret) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
				printk(KERN_INFO "%s:%d: FJLSM_REJECT process name=%s, pfn=0x%llx, prot=0x%llx\n",
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
					   __FUNCTION__, __LINE__, current->comm, pte_pfn(*pte) - PHYS_PFN_OFFSET, prot);
				break;
			}
		}
	} while (pte++, addr += PAGE_SIZE, addr != end);
	arch_leave_lazy_mmu_mode();
	pte_unmap_unlock(pte - 1, ptl);

	return ret;
}

static inline int fjsec_check_pmd_range(struct vm_area_struct *vma, pud_t *pud,
                   unsigned long addr, unsigned long end, pgprot_t prot)
{
	pmd_t *pmd;
	unsigned long next;
	int ret = 0;

	pmd = pmd_offset(pud, addr);
	do {
		next = pmd_addr_end(addr, end);
		if (pmd_none_or_clear_bad(pmd))
			continue;
		ret = fjsec_check_pte_range(vma->vm_mm, pmd, addr, next, prot);
		if (ret)
			break;
	} while (pmd++, addr = next, addr != end);

	return ret;
}

static inline int fjsec_check_pud_range(struct vm_area_struct *vma, pgd_t *pgd,
                                    unsigned long addr, unsigned long end, pgprot_t prot)
{
	pud_t *pud;
	unsigned long next;
	int ret = 0;

	pud = pud_offset(pgd, addr);
	do {
		next = pud_addr_end(addr, end);
		if (pud_none_or_clear_bad(pud))
			continue;
		ret= fjsec_check_pmd_range(vma, pud, addr, next, prot);
		if(ret)
			break;

	} while (pud++, addr = next, addr != end);

	return ret;
}

static int fjsec_file_mprotect(struct vm_area_struct *vma, unsigned long reqprot,
			   unsigned long prot, unsigned long start, unsigned long end ,unsigned long newflags)
{
	struct mm_struct *mm = vma->vm_mm;
	pgd_t *pgd;
	unsigned long next;
	int ret = 0;

  return 0;

	pgd = pgd_offset(mm, start);
	do {
		next = pgd_addr_end(start, end);
		if (pgd_none_or_clear_bad(pgd))
			continue;
		ret = fjsec_check_pud_range(vma, pgd, start, next, vm_get_page_prot(newflags));
		if(ret)
		{
			FJLSM_REJECT(ret)
		}
	} while (pgd++, start = next, start != end);

	return 0;
}

#ifdef DEBUG_MAPPING
void fjsec_debug_test_area_setting(unsigned long w_addr, unsigned long x_addr,
				unsigned long nw_addr, unsigned long nx_addr)
{
	unsigned long w_pfn = 0;
	unsigned long x_pfn = 0;
	unsigned long nw_pfn = 0;
	unsigned long nx_pfn = 0;
	int i;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return ;

	w_pfn = page_to_pfn(virt_to_page((unsigned long)w_addr));
	w_pfn -= PHYS_PFN_OFFSET;

	x_pfn = page_to_pfn(virt_to_page((unsigned long)x_addr));
	x_pfn -= PHYS_PFN_OFFSET;

	nw_pfn = page_to_pfn(virt_to_page((unsigned long)nw_addr));
	nw_pfn -= PHYS_PFN_OFFSET;

	nx_pfn = page_to_pfn(virt_to_page((unsigned long)nx_addr));
	nx_pfn -= PHYS_PFN_OFFSET;

#ifndef LSM_STRICT_MEMORY_DEBUG
	mutex_lock(&kernel_hardening_lock);
	set_memory_rw2((unsigned long)page_write_list, MAPPING_LIST_PAGE);
	set_memory_rw2((unsigned long)page_exec_list, MAPPING_LIST_PAGE);
#endif /* LSM_STRICT_MEMORY_DEBUG */

	// ## write area ##
	for ( i = 0; i < 10; i++){
		page_write_list[w_pfn >> 3] |= (1 << (w_pfn & 7));
		page_exec_list[w_pfn >> 3] |= (1 << (w_pfn & 7));
		w_pfn++;
	}

	// ## exec area ##
	for ( i = 0; i < 10; i++){
		page_write_list[x_pfn >> 3] &= ~(1 << (x_pfn & 7));
		page_exec_list[x_pfn >> 3] |= (1 << (x_pfn & 7));
		x_pfn++;
	}

	// ## non write area ##
	for ( i = 0; i < 10; i++){
		page_write_list[nw_pfn >> 3] &= ~(1 << (nw_pfn & 7));
		page_exec_list[nw_pfn >> 3] |= (1 << (nw_pfn & 7));
		nw_pfn++;
	}

	// ## non exec area ##
	for ( i = 0; i < 10; i++){
		page_write_list[nx_pfn >> 3] |= (1 << (nx_pfn & 7));
		page_exec_list[nx_pfn >> 3] &= ~(1 << (nx_pfn & 7));
		nx_pfn++;
	}

#ifndef LSM_STRICT_MEMORY_DEBUG
	set_memory_ro2((unsigned long)page_write_list, MAPPING_LIST_PAGE);
	set_memory_ro2((unsigned long)page_exec_list, MAPPING_LIST_PAGE);
	mutex_unlock(&kernel_hardening_lock);
#endif /* LSM_STRICT_MEMORY_DEBUG */

}
EXPORT_SYMBOL(fjsec_debug_test_area_setting);

enum pfn_mode {
	MODE_WRITE,
	MODE_EXEC,
	MODE_NWRITE,
	MODE_NEXEC,
	MODE_NFREE,
	MODE_MODULE,
};

unsigned long fjsec_debug_find_pfn(int mode, unsigned long addr)
{
	unsigned long pfn = 0;

	if(__fjsec_check_nv(FJSEC_LSM_ENABLED) == 0)
		return 0;

	switch (mode) {
	case MODE_WRITE:
		pfn = page_to_pfn(virt_to_page((unsigned long)addr));
		pfn -= PHYS_PFN_OFFSET;
		break;
	case MODE_EXEC:
		pfn = page_to_pfn(virt_to_page((unsigned long)addr));
		pfn -= PHYS_PFN_OFFSET;
		break;
	case MODE_NWRITE:
		pfn = page_to_pfn(virt_to_page((unsigned long)addr));
		pfn -= PHYS_PFN_OFFSET;
		break;
	case MODE_NEXEC:
		pfn = page_to_pfn(virt_to_page((unsigned long)addr));
		pfn -= PHYS_PFN_OFFSET;
		break;
	case MODE_NFREE:
		do {
			if (!is_free(pfn))
				break;
		} while (pfn++, pfn < MAX_LOW_PFN);
		break;
	case MODE_MODULE:
		do {
			if (is_module(pfn))
				break;
		} while (pfn++, pfn < MAX_LOW_PFN);
		break;
	default:
		break;
	}

	pfn += PHYS_PFN_OFFSET;

	return pfn;

}
EXPORT_SYMBOL(fjsec_debug_find_pfn);
#endif /* DEBUG_MAPPING */

const struct security_operations fjsec_security_ops = {
	.name =					"fjsec",
	.ptrace_traceme =		fjsec_ptrace_traceme,
	.ptrace_request_check =	fjsec_ptrace_request_check,
	.sb_mount =				fjsec_sb_mount,
	.sb_pivotroot =			fjsec_sb_pivotroot,
#ifdef CONFIG_SECURITY_PATH
#ifdef CONFIG_SECURITY_FJSEC_PROTECT_CHROOT
	.path_chroot =			fjsec_path_chroot,
#endif	/* CONFIG_SECURITY_FJSEC_PROTECT_CHROOT */
#endif	/* CONFIG_SECURITY_PATH */
	.sb_umount =			fjsec_sb_umount,
	.file_permission =		fjsec_file_permission,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT DEL */
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD S */
	.mmap_file =			fjsec_mmap_file,
	.mmap_addr =			fjsec_mmap_addr,
	.file_open =			fjsec_file_open,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD E */
#ifdef CONFIG_SECURITY_PATH
	.path_mknod =			fjsec_path_mknod,
	.path_mkdir =			fjsec_path_mkdir,
	.path_rmdir =			fjsec_path_rmdir,
	.path_unlink =			fjsec_path_unlink,
	.path_symlink =			fjsec_path_symlink,
	.path_link =			fjsec_path_link,
	.path_rename =			fjsec_path_rename,
	.path_truncate =		fjsec_path_truncate,
	.path_chmod =			fjsec_path_chmod,
	.path_chown =			fjsec_path_chown,
#endif	/* CONFIG_SECURITY_PATH */
	.remap_pfn_range =		fjsec_remap_pfn_range,
	.ioremap_page_range =	fjsec_ioremap_page_range,
	.insert_page =			fjsec_insert_page,
	.insert_pfn =			fjsec_insert_pfn,
	.task_fix_setuid = 		fjsec_task_fix_setuid,
	.task_fix_setgid = 		fjsec_task_fix_setgid,
	.bprm_set_creds = 		fjsec_bprm_set_creds,
	.file_mprotect = 		fjsec_file_mprotect,
	.ptrace_access_check = 		cap_ptrace_access_check,
//	.ptrace_traceme = 		cap_ptrace_traceme,
//	.ptrace_request_check = 	cap_ptrace_request_check,
	.capget = 			cap_capget,
	.capset = 			cap_capset,
	.capable = 			cap_capable,
	.quotactl = 			cap_quotactl,
	.quota_on = 			cap_quota_on,
	.syslog = 			cap_syslog,
	.settime = 			cap_settime,
	.vm_enough_memory = 		cap_vm_enough_memory,
//	.bprm_set_creds = 		cap_bprm_set_creds,
	.bprm_committing_creds = 	cap_bprm_committing_creds,
	.bprm_committed_creds = 	cap_bprm_committed_creds,
	.bprm_check_security = 		cap_bprm_check_security,
	.bprm_secureexec = 		cap_bprm_secureexec,
	.sb_alloc_security = 		cap_sb_alloc_security,
	.sb_free_security = 		cap_sb_free_security,
	.sb_copy_data = 		cap_sb_copy_data,
	.sb_remount = 			cap_sb_remount,
	.sb_kern_mount = 		cap_sb_kern_mount,
	.sb_show_options = 		cap_sb_show_options,
	.sb_statfs = 			cap_sb_statfs,
//	.sb_mount = 			cap_sb_mount,
//	.sb_umount = 			cap_sb_umount,
//	.sb_pivotroot = 		cap_sb_pivotroot,
	.sb_set_mnt_opts = 		cap_sb_set_mnt_opts,
	.sb_clone_mnt_opts = 		cap_sb_clone_mnt_opts,
	.sb_parse_opts_str = 		cap_sb_parse_opts_str,
	.inode_alloc_security = 	cap_inode_alloc_security,
	.inode_free_security = 		cap_inode_free_security,
	.inode_init_security = 		cap_inode_init_security,
	.inode_create = 		cap_inode_create,
	.inode_link = 			cap_inode_link,
	.inode_unlink = 		cap_inode_unlink,
	.inode_symlink = 		cap_inode_symlink,
	.inode_mkdir = 			cap_inode_mkdir,
	.inode_rmdir = 			cap_inode_rmdir,
	.inode_mknod = 			cap_inode_mknod,
	.inode_rename = 		cap_inode_rename,
	.inode_readlink = 		cap_inode_readlink,
	.inode_follow_link = 		cap_inode_follow_link,
	.inode_permission = 		cap_inode_permission,
	.inode_setattr = 		cap_inode_setattr,
	.inode_getattr = 		cap_inode_getattr,
	.inode_setxattr = 		cap_inode_setxattr,
	.inode_post_setxattr = 		cap_inode_post_setxattr,
	.inode_getxattr = 		cap_inode_getxattr,
	.inode_listxattr = 		cap_inode_listxattr,
	.inode_removexattr = 		cap_inode_removexattr,
	.inode_need_killpriv = 		cap_inode_need_killpriv,
	.inode_killpriv = 		cap_inode_killpriv,
	.inode_getsecurity = 		cap_inode_getsecurity,
	.inode_setsecurity = 		cap_inode_setsecurity,
	.inode_listsecurity = 		cap_inode_listsecurity,
	.inode_getsecid = 		cap_inode_getsecid,
//#ifdef CONFIG_SECURITY_PATH
//	.path_mknod = 			cap_path_mknod,
//	.path_mkdir = 			cap_path_mkdir,
//	.path_rmdir = 			cap_path_rmdir,
//	.path_unlink = 			cap_path_unlink,
//	.path_symlink = 		cap_path_symlink,
//	.path_link = 			cap_path_link,
//	.path_rename = 			cap_path_rename,
//	.path_truncate = 		cap_path_truncate,
//	.path_chmod = 			cap_path_chmod,
//	.path_chown = 			cap_path_chown,
//#ifndef CONFIG_SECURITY_FJSEC_PROTECT_CHROOT
//	.path_chroot = 			cap_path_chroot,
//#endif	/* CONFIG_SECURITY_FJSEC_PROTECT_CHROOT */
//#endif	/* CONFIG_SECURITY_PATH */
//	.file_permission = 		cap_file_permission,
	.file_alloc_security = 		cap_file_alloc_security,
	.file_free_security = 		cap_file_free_security,
	.file_ioctl = 			cap_file_ioctl,
//	.file_mmap = 			cap_file_mmap,
//	.file_mprotect = 		cap_file_mprotect,
	.file_lock = 			cap_file_lock,
	.file_fcntl = 			cap_file_fcntl,
	.file_set_fowner = 		cap_file_set_fowner,
	.file_send_sigiotask = 		cap_file_send_sigiotask,
	.file_receive = 		cap_file_receive,
//	.dentry_open = 			cap_dentry_open,
	.task_create = 			cap_task_create,
	.task_free =			cap_task_free,
	.cred_alloc_blank = 		cap_cred_alloc_blank,
	.cred_free = 			cap_cred_free,
	.cred_prepare = 		cap_cred_prepare,
	.cred_transfer = 		cap_cred_transfer,
	.kernel_act_as = 		cap_kernel_act_as,
	.kernel_create_files_as = 	cap_kernel_create_files_as,
	.kernel_module_request = 	cap_kernel_module_request,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD S */
	.kernel_module_from_file=	cap_kernel_module_from_file,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD E */
//	.kernel_load_module = 		cap_kernel_load_module,
//	.task_fix_setuid = 		cap_task_fix_setuid,
	.task_setpgid = 		cap_task_setpgid,
	.task_getpgid = 		cap_task_getpgid,
	.task_getsid = 			cap_task_getsid,
	.task_getsecid = 		cap_task_getsecid,
	.task_setnice = 		cap_task_setnice,
	.task_setioprio = 		cap_task_setioprio,
	.task_getioprio = 		cap_task_getioprio,
	.task_setrlimit = 		cap_task_setrlimit,
	.task_setscheduler = 		cap_task_setscheduler,
	.task_getscheduler = 		cap_task_getscheduler,
	.task_movememory = 		cap_task_movememory,
	.task_wait = 			cap_task_wait,
	.task_kill = 			cap_task_kill,
	.task_prctl = 			cap_task_prctl,
	.task_to_inode = 		cap_task_to_inode,
	.ipc_permission = 		cap_ipc_permission,
	.ipc_getsecid = 		cap_ipc_getsecid,
	.msg_msg_alloc_security = 	cap_msg_msg_alloc_security,
	.msg_msg_free_security = 	cap_msg_msg_free_security,
	.msg_queue_alloc_security = 	cap_msg_queue_alloc_security,
	.msg_queue_free_security = 	cap_msg_queue_free_security,
	.msg_queue_associate = 		cap_msg_queue_associate,
	.msg_queue_msgctl = 		cap_msg_queue_msgctl,
	.msg_queue_msgsnd = 		cap_msg_queue_msgsnd,
	.msg_queue_msgrcv = 		cap_msg_queue_msgrcv,
	.shm_alloc_security = 		cap_shm_alloc_security,
	.shm_free_security = 		cap_shm_free_security,
	.shm_associate = 		cap_shm_associate,
	.shm_shmctl = 			cap_shm_shmctl,
	.shm_shmat = 			cap_shm_shmat,
	.sem_alloc_security = 		cap_sem_alloc_security,
	.sem_free_security = 		cap_sem_free_security,
	.sem_associate = 		cap_sem_associate,
	.sem_semctl = 			cap_sem_semctl,
	.sem_semop = 			cap_sem_semop,
	.netlink_send = 		cap_netlink_send,
	.d_instantiate = 		cap_d_instantiate,
	.getprocattr = 			cap_getprocattr,
	.setprocattr = 			cap_setprocattr,
	.secid_to_secctx = 		cap_secid_to_secctx,
	.secctx_to_secid = 		cap_secctx_to_secid,
	.release_secctx = 		cap_release_secctx,
	.inode_notifysecctx = 		cap_inode_notifysecctx,
	.inode_setsecctx = 		cap_inode_setsecctx,
	.inode_getsecctx = 		cap_inode_getsecctx,
#ifdef CONFIG_SECURITY_NETWORK
	.unix_stream_connect = 		cap_unix_stream_connect,
	.unix_may_send = 		cap_unix_may_send,
	.socket_create = 		cap_socket_create,
	.socket_post_create = 		cap_socket_post_create,
	.socket_bind = 			cap_socket_bind,
	.socket_connect = 		cap_socket_connect,
	.socket_listen = 		cap_socket_listen,
	.socket_accept = 		cap_socket_accept,
	.socket_sendmsg = 		cap_socket_sendmsg,
	.socket_recvmsg = 		cap_socket_recvmsg,
	.socket_getsockname = 		cap_socket_getsockname,
	.socket_getpeername = 		cap_socket_getpeername,
	.socket_setsockopt = 		cap_socket_setsockopt,
	.socket_getsockopt = 		cap_socket_getsockopt,
	.socket_shutdown = 		cap_socket_shutdown,
	.socket_sock_rcv_skb = 		cap_socket_sock_rcv_skb,
	.socket_getpeersec_stream = 	cap_socket_getpeersec_stream,
	.socket_getpeersec_dgram = 	cap_socket_getpeersec_dgram,
	.sk_alloc_security = 		cap_sk_alloc_security,
	.sk_free_security = 		cap_sk_free_security,
	.sk_clone_security = 		cap_sk_clone_security,
	.sk_getsecid = 			cap_sk_getsecid,
	.sock_graft = 			cap_sock_graft,
	.inet_conn_request = 		cap_inet_conn_request,
	.inet_csk_clone = 		cap_inet_csk_clone,
	.inet_conn_established = 	cap_inet_conn_established,
	.secmark_relabel_packet = 	cap_secmark_relabel_packet,
	.secmark_refcount_inc = 	cap_secmark_refcount_inc,
	.secmark_refcount_dec = 	cap_secmark_refcount_dec,
	.req_classify_flow = 		cap_req_classify_flow,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD S */
	.tun_dev_alloc_security=	cap_tun_dev_alloc_security,
	.tun_dev_free_security=		cap_tun_dev_free_security,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD E */
	.tun_dev_create = 		cap_tun_dev_create,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT DEL */
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD S */
	.tun_dev_attach_queue=	cap_tun_dev_attach_queue,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD E */
	.tun_dev_attach = 		cap_tun_dev_attach,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD S */
	.tun_dev_open=		cap_tun_dev_open,
	.skb_owned_by=		cap_skb_owned_by,
/* FUJITSU:2014-11-18 GEN-141-SCRU-ALT ADD E */
#endif	/* CONFIG_SECURITY_NETWORK */
#ifdef CONFIG_SECURITY_NETWORK_XFRM
	.xfrm_policy_alloc_security = 	cap_xfrm_policy_alloc_security,
	.xfrm_policy_clone_security = 	cap_xfrm_policy_clone_security,
	.xfrm_policy_free_security = 	cap_xfrm_policy_free_security,
	.xfrm_policy_delete_security = 	cap_xfrm_policy_delete_security,
	.xfrm_state_alloc_security = 	cap_xfrm_state_alloc_security,
	.xfrm_state_free_security = 	cap_xfrm_state_free_security,
	.xfrm_state_delete_security = 	cap_xfrm_state_delete_security,
	.xfrm_policy_lookup = 		cap_xfrm_policy_lookup,
	.xfrm_state_pol_flow_match = 	cap_xfrm_state_pol_flow_match,
	.xfrm_decode_session = 		cap_xfrm_decode_session,
#endif	/* CONFIG_SECURITY_NETWORK_XFRM */
#ifdef CONFIG_KEYS
	.key_alloc = 			cap_key_alloc,
	.key_free = 			cap_key_free,
	.key_permission = 		cap_key_permission,
	.key_getsecurity = 		cap_key_getsecurity,
#endif	/* CONFIG_KEYS */
#ifdef CONFIG_AUDIT
	.audit_rule_init = 		cap_audit_rule_init,
	.audit_rule_known = 		cap_audit_rule_known,
	.audit_rule_match = 		cap_audit_rule_match,
	.audit_rule_free = 		cap_audit_rule_free,
#endif	/* CONFIG_AUDIT */
};

#ifdef PRCONFIG

static void fjsec_prconfig(void)
{
	struct fs_path_config *pc;
	struct fs_path_config *work_devs;
	struct accessible_area_disk_dev *accessible_areas;
	struct access_control_mmcdl_device *mmcdl_device;
	int index;
	struct read_write_access_control *rw_access_control;
	struct read_write_access_control_process *rw_access_control_process;
	struct ac_config *ac_cfg;

	printk (KERN_INFO " --------------------\n");
	printk (KERN_INFO "boot mode=<%d>\n", boot_mode);

	printk (KERN_INFO "CONFIG_SECURITY_PATH=<%d>\n", CONFIG_SECURITY_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_DISK_DEV_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_DISK_DEV_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_SYSTEM_DIR_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_SYSTEM_DEV_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_SYSTEM_DEV_PATH);
#ifdef CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE=<%d>\n", CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DIR_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DIR_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DEV_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_SECURE_STORAGE_DEV_PATH);
#endif /* CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE */
#ifdef CONFIG_SECURITY_FJSEC_AC_KITTING
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_AC_KITTING=<%d>\n", CONFIG_SECURITY_FJSEC_AC_KITTING);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_KITTING_DIR_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_KITTING_DIR_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_KITTING_DEV_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_KITTING_DEV_PATH);
#endif /* CONFIG_SECURITY_FJSEC_AC_KITTING */
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_FELICA_DEV_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_FELICA_DEV_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_FELICA_DEV2_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_FELICA_DEV2_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_NFC_DEV_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_NFC_DEV_PATH);
#ifdef CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_SECURE_STORAGE_ACCESS_PROCESS_NAME=<%s>\n", CONFIG_SECURITY_FJSEC_SECURE_STORAGE_ACCESS_PROCESS_NAME);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_SECURE_STORAGE_ACCESS_PROCESS_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_SECURE_STORAGE_ACCESS_PROCESS_PATH);
#endif /* CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE */
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_NAME=<%s>\n", CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_NAME);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_NAME=<%s>\n", CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_NAME);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_NAME=<%s>\n", CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_NAME);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_RECOVERY_MODE_ACCESS_PROCESS_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_NAME=<%s>\n", CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_NAME);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_NAME=<%s>\n", CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_NAME);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_PATH);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_PROTECT_CHROOT=<%d>\n", CONFIG_SECURITY_FJSEC_PROTECT_CHROOT);
	printk (KERN_INFO "CONFIG_SECURITY_FJSEC_CHROOT_PATH=<%s>\n", CONFIG_SECURITY_FJSEC_CHROOT_PATH);

	printk (KERN_INFO " devs\n");
#ifdef CHECK_PROTO_NUM
	if (lsm_nv_flag != FJSEC_LSM_NV_BEFORE) {
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD S */
			work_devs = coresight_fuse_apps_access_disabled()? devs2:devs;
/* FUJITSU:2014-12-08 GEN-141-SCRU-ALT MOD E */
	}
	else {
		work_devs = devs;
	}
#else
	work_devs = devs;
#endif
	dprintk(KERN_INFO "%s:%d: lsm_nv_flag = 0x%02x\n", __FUNCTION__, __LINE__, lsm_nv_flag);

	for (pc = work_devs; pc->prefix; pc++) {
		printk (KERN_INFO "  <0x%08x> <%d> <%d> <0x%08x> <%s>\n", pc->mode, pc->uid, pc->gid, pc->rdev, pc->prefix);
	}

	printk (KERN_INFO " accessible area\n");
	printk (KERN_INFO "  fota mode\n");
	for (accessible_areas = accessible_areas_fota; accessible_areas->tail; accessible_areas++) {
		printk (KERN_INFO "   <0x%llx><0x%llx>\n", accessible_areas->head, accessible_areas->tail);
	}

	printk (KERN_INFO "  sddownloader mode\n");
	for (accessible_areas = accessible_areas_sddownloader; accessible_areas->tail; accessible_areas++) {
		printk (KERN_INFO "   <0x%llx><0x%llx>\n", accessible_areas->head, accessible_areas->tail);
	}

	printk (KERN_INFO "  osupdate mode\n");
	for (accessible_areas = accessible_areas_osupdate; accessible_areas->tail; accessible_areas++) {
		printk (KERN_INFO "   <0x%llx><0x%llx>\n", accessible_areas->head, accessible_areas->tail);
	}

	printk (KERN_INFO "  recovery mode\n");
	for (accessible_areas = accessible_areas_recovery; accessible_areas->tail; accessible_areas++) {
		printk (KERN_INFO "   <0x%llx><0x%llx>\n", accessible_areas->head, accessible_areas->tail);
	}

	printk (KERN_INFO "  maker mode\n");
	for (accessible_areas = accessible_areas_maker; accessible_areas->tail; accessible_areas++) {
		printk (KERN_INFO "   <0x%llx><0x%llx>\n", accessible_areas->head, accessible_areas->tail);
	}

	printk (KERN_INFO " mmcdl_device_list\n");
	for (mmcdl_device = mmcdl_device_list; mmcdl_device->process_name; mmcdl_device++) {
		printk (KERN_INFO "  <%d><%s><%s>\n", mmcdl_device->boot_mode, mmcdl_device->process_name, mmcdl_device->process_path);
	}

	printk(KERN_INFO " ptrace_read_request_policy\n");
	for (index = 0; index < ARRAY_SIZE(ptrace_read_request_policy); index++) {
		printk(KERN_INFO "  <%ld>\n", ptrace_read_request_policy[index]);
	}

	printk (KERN_INFO " rw_access_control_list\n");
	for (rw_access_control = rw_access_control_list; rw_access_control->prefix; rw_access_control++) {
		for (rw_access_control_process = rw_access_control->process_list; rw_access_control_process->process_path; rw_access_control_process++) {
			printk (KERN_INFO "  <%s><%s><%s><%d>\n", rw_access_control->prefix, rw_access_control_process->process_name, rw_access_control_process->process_path, rw_access_control_process->uid);
		}
	}

	printk (KERN_INFO " ptn_acl\n");
	for (ac_cfg = ptn_acl; ac_cfg->prefix; ac_cfg++) {
		printk (KERN_INFO "  <%s><%d><%s><%s>\n", ac_cfg->prefix, ac_cfg->boot_mode, ac_cfg->process_name, ac_cfg->process_path);
	}

	printk (KERN_INFO " fs_acl\n");
	for (ac_cfg = fs_acl; ac_cfg->prefix; ac_cfg++) {
		printk (KERN_INFO "  <%s><%d><%s><%s>\n", ac_cfg->prefix, ac_cfg->boot_mode, ac_cfg->process_name, ac_cfg->process_path);
	}

	printk (KERN_INFO " devmem_acl\n");
	for (index = 0; index < ARRAY_SIZE(devmem_acl); index++) {
		printk (KERN_INFO "  <0x%lx><0x%lx><%s><%s>\n", devmem_acl[index].head, devmem_acl[index].tail, devmem_acl[index].process_name, devmem_acl[index].process_path);
	}

	printk (KERN_INFO " --------------------\n");
}
#else /* PRCONFIG */
#define fjsec_prconfig()
#endif /* PRCONFIG */

static int __init fjsec_init (void)
{

	printk (KERN_INFO "FJSEC LSM module initialized\n");
	fjsec_prconfig();

	return 0;
}


/* FCNT LIMITED: 2017-02-20 SEC_LSM_042 add start */
static int __init setup_fjlsm_mode(char *str)
{
	lsm_nv_flag = FJSEC_LSM_NV_AFTER;
	return 0;
}
early_param("fjlsm.mode", setup_fjlsm_mode);
/* FCNT LIMITED: 2017-02-20 SEC_LSM_042 add end */

security_initcall (fjsec_init);
