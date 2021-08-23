/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016-2017
// COPYRIGHT(C) FUJITSU LIMITED 2015
/*----------------------------------------------------------------------------*/
/*
 * Copyright(C) 2010-2014 FUJITSU LIMITED
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
#include <linux/ptrace.h>
#include <crypto/sha.h>
#include "../../fjfeat/FjFeatMac.h"
#include <linux/kdev_t.h>
#include <linux/spinlock.h>
#include <linux/bootmem.h>

#include <linux/msm_ion.h>
#ifdef CONFIG_FIDO_SEC
#include <linux/msm_ion_ex.h>
#endif
//#define DEBUG_COPY_GUARD
//#define DEBUG_ACCESS_CONTROL
//#define DEBUG_KITTING_ACCESS_CONTROL
//#define DEBUG_SETID_ACCESS_CONTROL
/* FUJITSU LIMITED:2015-06-01 SEC-LSM-014 mod start */
//#define DEBUG_MAPPING
//#define LSM_STRICT_MEMORY_DEBUG
/* FUJITSU LIMITED:2015-06-01 SEC-LSM-014  mod end */
#define CURRENT_UID  (current->cred->uid % 100000)// for Multi User
#define AID_ROOT              0  /* traditional unix root user */
#define AID_SYSTEM         1000  /* system server */
#define AID_RADIO          1001  /* telephony subsystem, RIL */
#define AID_GRAPHICS       1003  /* graphics */
#define AID_AUDIO          1005  /* audio devices */
#define AID_CAMERA         1006
/* FUJITSU LIMITED:2015-07-07 SEC-LSM-002 add start */
#define AID_WIFI           1010  /* wifi subsystem */
/* FUJITSU LIMITED:2015-07-07 SEC-LSM-002  add end */
#define AID_MEDIA          1013
/* FUJITSU LIMITED:2015-07-07 SEC-LSM-002 add start */
#define AID_VPN            1016  /* vpn system */
/* FUJITSU LIMITED:2015-07-07 SEC-LSM-002  add end */
#define AID_KEYSTORE       1017  /* keystore */
#define AID_DRM            1019  /* drm server */
/* FUJITSU LIMITED:2015-11-02 SEC-LSM-051 add start */
#define AID_SLIM_DAEMON    1021
/* FUJITSU LIMITED:2015-11-02 SEC-LSM-051  add end */
#define AID_NFC            1027  /* nfc subsystem */
#define AID_AUDIOSERVER   1041  /* audioserver process */
/* FCNT LIMITED: 2017-02-22 SEC_LSM_043 add start */
#define AID_DEBUGGERD     1045  /* debuggerd unprivileged user */
/* FCNT LIMITED: 2017-02-22 SEC_LSM_043 add end */
#define AID_MEDIA_CODEC    1046  /* mediacodec process */
/* FCNT LIMITED: 2017-02-27 SEC_LSM_044 add start */
#define AID_CAMERASERVER  1047  /* cameraserver process */
/* FCNT LIMITED: 2017-02-27 SEC_LSM_044 add end */
#define AID_SHELL         2000  /* adb and debug shell user */
#define AID_NET_ADMIN      3005
#define AID_CASDRM        4005
#define AID_PREINST_FELC  15003
#define AID_MMP           15020  /* fj mmp */
#define AID_CAMERA_APP    15021
#define AID_NOBODY         9999
#define PRNAME_NO_CHECK 0
#define UID_NO_CHECK -1
// *NEW* add flg 
#define SUPL_NO_CHECK -1
#define SUPL_NO_HASHCHECK -2
#define SUPL_CHK_ALT_ADDR_IONDMA  -10000

#define NO_CHECK_STR "no_check"
#define ISEMPTY "isEmpty"
#ifdef DEBUG_COPY_GUARD
#define AID_LOG           1007  /* log devices */
#endif /* DEBUG_COPY_GUARD */

#define JAVA_APP_PROCESS_PATH32 "/system/bin/app_process32"
#define JAVA_APP_PROCESS_PATH64 "/system/bin/app_process64"
#define CHECK_COMMON		"*"
#define SYSTEM_PROCESS_PATH "/system/bin/"
#define HCE_PROCESS_PATH "/sbin/healthd"


#define SECURITY_FJSEC_RECOVERY_PROCESS_NAME "recovery"
#define SECURITY_FJSEC_RECOVERY_PROCESS_PATH "/sbin/recovery"

#define S2B(x) ((loff_t)x * SECTOR_SIZE)
#define ADDR2PFN(x) (x >> PAGE_SHIFT)

#define INIT_PROCESS_NAME "init"
#define INIT_PROCESS_PATH "/init"

#define BOOT_ARGS_MODE_FOTA "fotamode"
#define BOOT_ARGS_MODE_SDDOWNLOADER "sddownloadermode"
//#define BOOT_ARGS_MODE_RECOVERY "recovery"
#define BOOT_ARGS_MODE_MAKERCMD "makermode"
#define BOOT_ARGS_MODE_OSUPDATE "osupdatemode"
#define BOOT_ARGS_MODE_RECOVERYMENU "recoverymenu"
#define BOOT_ARGS_MODE_KERNEL "kernelmode"
#define BOOT_ARGS_MODE_MASTERCLEAR "masterclear"

#define OFFSET_BYTE_LIMIT LLONG_MAX

#ifdef DEBUG_ACCESS_CONTROL
#define TEST_PTN_PATH "/dev/block/mmcblk0p22"
#define TEST_MOUNT_POINT "/fota2/"
#define TEST_DIRS_PATH TEST_MOUNT_POINT "*"
#endif /* DEBUG_ACCESS_CONTROL */

#define SIGNAL_CATCHER_PROCESS_NAME "Signal Catcher"

enum boot_mode_type {
	BOOT_MODE_NONE = 0,
	BOOT_MODE_FOTA = 1,
	BOOT_MODE_SDDOWNLOADER = 2,
//	BOOT_MODE_RECOVERY = 3,
	BOOT_MODE_MASTERCLEAR = 3,
	BOOT_MODE_MAKERCMD = 4,
	BOOT_MODE_OSUPDATE = 5,
};

#define ROOT_DIR "/"

enum result_mount_point {
	MATCH_SYSTEM_MOUNT_POINT,
#ifdef CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE
	MATCH_SECURE_STORAGE_MOUNT_POINT,
#endif /* CONFIG_SECURITY_FJSEC_AC_SECURE_STORAGE */
#ifdef CONFIG_SECURITY_FJSEC_AC_KITTING
	MATCH_KITTING_MOUNT_POINT,
#endif /* CONFIG_SECURITY_FJSEC_AC_KITTING */
	MATCH_MOUNT_POINT,
	MATCH_ROOT_MOUNT_POINT,
	MISMATCH_MOUNT_POINT,
	UNRELATED_MOUNT_POINT,
};

//#define LSM_FLG_OFF
//#define LSM_PAGEACL_AUTOREGIST
//#define LSM_PAGEACL_DUMPSTACK
//#define LSM_PAGEACL_PERFDBG
//#define LSM_PAGEACL_SSRDBG


/* FCNT LIMITED: 2017-02-20 SEC_LSM_042 add start */
#define BOOT_ARGS_FJLSM_BASE_ON     "base_on"
#define BOOT_ARGS_FJLSM_MMAP_ON     "mmap_on"
#define BOOT_ARGS_FJLSM_ON          "on"
#define BOOT_ARGS_FJLSM_PERMISSIVE  "permissive"
/* FCNT LIMITED: 2017-02-20 SEC_LSM_042 add end */


#define FJSEC_LSM_DISABLED			0x00
#define FJSEC_LSM_ENABLED			0x01
#define FJSEC_LSM_MMAP_ENABLED	0x02
#define FJSEC_LSM_PERMISSIVE	0x07// 0x07  LSM ON(Permissive)

#define FJSEC_LSM_NV_BEFORE		0x00
#define FJSEC_LSM_NV_AFTER			0x01

/**
 * mapping control
 */
spinlock_t page_module_list_lock;
#define MAX_LOW_PFN max_low_pfn
#define MAPPING_LIST_SIZE (MAX_LOW_PFN >> 3)
#define MAPPING_LIST_PAGE (ALIGN(MAPPING_LIST_SIZE, PAGE_SIZE) >> PAGE_SHIFT)
char *page_free_list;
char *page_write_list;
char *page_exec_list;
char *page_module_list;

#define FJSEC_LAOD 0
#define FJSEC_DELETE 1

/* check dev prototyoe */
//#define CHECK_PROTO_NUM

/* for CTS */
#define CTS_TEST

/**
 * Limitation of device file.
 */
struct fs_path_config {
	mode_t		mode;
	uid_t		uid;
	gid_t		gid;
	dev_t		rdev;
	const char	*prefix;
	// *NEW* add the mount point info to check the path 
	const char *mnt_pnt;
};

static struct fs_path_config devs[] = {
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 0), "/dev/block/mmcblk0", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 1), "/dev/block/mmcblk0p1", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 2), "/dev/block/mmcblk0p2", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 3), "/dev/block/mmcblk0p3", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 4), "/dev/block/mmcblk0p4", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 5), "/dev/block/mmcblk0p5", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 6), "/dev/block/mmcblk0p6", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 7), "/dev/block/mmcblk0p7", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 8), "/dev/block/mmcblk0p8", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 9), "/dev/block/mmcblk0p9", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 10), "/dev/block/mmcblk0p10", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 11), "/dev/block/mmcblk0p11", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 12), "/dev/block/mmcblk0p12", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 13), "/dev/block/mmcblk0p13", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 14), "/dev/block/mmcblk0p14", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 15), "/dev/block/mmcblk0p15", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 16), "/dev/block/mmcblk0p16", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 17), "/dev/block/mmcblk0p17", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 18), "/dev/block/mmcblk0p18", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 19), "/dev/block/mmcblk0p19", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 20), "/dev/block/mmcblk0p20", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 21), "/dev/block/mmcblk0p21", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 22), "/dev/block/mmcblk0p22", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 23), "/dev/block/mmcblk0p23", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 24), "/dev/block/mmcblk0p24", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 25), "/dev/block/mmcblk0p25", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 26), "/dev/block/mmcblk0p26", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 27), "/dev/block/mmcblk0p27", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 28), "/dev/block/mmcblk0p28", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 29), "/dev/block/mmcblk0p29", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 30), "/dev/block/mmcblk0p30", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 31), "/dev/block/mmcblk0p31", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 0), "/dev/block/mmcblk0p32", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 1), "/dev/block/mmcblk0p33", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 2), "/dev/block/mmcblk0p34", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 3), "/dev/block/mmcblk0p35", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 4), "/dev/block/mmcblk0p36", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 5), "/dev/block/mmcblk0p37", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 6), "/dev/block/mmcblk0p38", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 7), "/dev/block/mmcblk0p39", "/system" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 8), "/dev/block/mmcblk0p40", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 9), "/dev/block/mmcblk0p41", "/cache" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 10), "/dev/block/mmcblk0p42", "/persist" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 11), "/dev/block/mmcblk0p43", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 12), "/dev/block/mmcblk0p44", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 13), "/dev/block/mmcblk0p45", NULL },
    { (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IFBLK), AID_SYSTEM, AID_SYSTEM, MKDEV(259, 14), "/dev/block/mmcblk0p46", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 15), "/dev/block/mmcblk0p47", "/fota" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 16), "/dev/block/mmcblk0p48", "/sst" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 17), "/dev/block/mmcblk0p49", "/sst2" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 18), "/dev/block/mmcblk0p50", "/persist2" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 19), "/dev/block/mmcblk0p51", "/persist3" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 20), "/dev/block/mmcblk0p52", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 21), "/dev/block/mmcblk0p53", "/cnt" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 22), "/dev/block/mmcblk0p54", "/kitting" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 23), "/dev/block/mmcblk0p55", "/data" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 24), "/dev/block/mmcblk0p56", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 32), "/dev/block/mmcblk0rpmb", NULL },
    { (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IFCHR), AID_NFC, AID_NFC, MKDEV(10, 254), "/dev/cxd224x-i2c", NULL },
    { 0, 0, 0, 0, 0, 0 },
};

#ifdef CHECK_PROTO_NUM
static struct fs_path_config devs2[] = {
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 0), "/dev/block/mmcblk0", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 1), "/dev/block/mmcblk0p1", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 2), "/dev/block/mmcblk0p2", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 3), "/dev/block/mmcblk0p3", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 4), "/dev/block/mmcblk0p4", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 5), "/dev/block/mmcblk0p5", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 6), "/dev/block/mmcblk0p6", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 7), "/dev/block/mmcblk0p7", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 8), "/dev/block/mmcblk0p8", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 9), "/dev/block/mmcblk0p9", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 10), "/dev/block/mmcblk0p10", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 11), "/dev/block/mmcblk0p11", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 12), "/dev/block/mmcblk0p12", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 13), "/dev/block/mmcblk0p13", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 14), "/dev/block/mmcblk0p14", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 15), "/dev/block/mmcblk0p15", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 16), "/dev/block/mmcblk0p16", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 17), "/dev/block/mmcblk0p17", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 18), "/dev/block/mmcblk0p18", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 19), "/dev/block/mmcblk0p19", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 20), "/dev/block/mmcblk0p20", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 21), "/dev/block/mmcblk0p21", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 22), "/dev/block/mmcblk0p22", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 23), "/dev/block/mmcblk0p23", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 24), "/dev/block/mmcblk0p24", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 25), "/dev/block/mmcblk0p25", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 26), "/dev/block/mmcblk0p26", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 27), "/dev/block/mmcblk0p27", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 28), "/dev/block/mmcblk0p28", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 29), "/dev/block/mmcblk0p29", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 30), "/dev/block/mmcblk0p30", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 31), "/dev/block/mmcblk0p31", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 0), "/dev/block/mmcblk0p32", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 1), "/dev/block/mmcblk0p33", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 2), "/dev/block/mmcblk0p34", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 3), "/dev/block/mmcblk0p35", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 4), "/dev/block/mmcblk0p36", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 5), "/dev/block/mmcblk0p37", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 6), "/dev/block/mmcblk0p38", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 7), "/dev/block/mmcblk0p39", "/system" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 8), "/dev/block/mmcblk0p40", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 9), "/dev/block/mmcblk0p41", "/cache" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 10), "/dev/block/mmcblk0p42", "/persist" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 11), "/dev/block/mmcblk0p43", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 12), "/dev/block/mmcblk0p44", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 13), "/dev/block/mmcblk0p45", NULL },
    { (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IFBLK), AID_SYSTEM, AID_SYSTEM, MKDEV(259, 14), "/dev/block/mmcblk0p46", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 15), "/dev/block/mmcblk0p47", "/fota" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 16), "/dev/block/mmcblk0p48", "/sst" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 17), "/dev/block/mmcblk0p49", "/sst2" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 18), "/dev/block/mmcblk0p50", "/persist2" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 19), "/dev/block/mmcblk0p51", "/persist3" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 20), "/dev/block/mmcblk0p52", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 21), "/dev/block/mmcblk0p53", "/cnt" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 22), "/dev/block/mmcblk0p54", "/kitting" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 23), "/dev/block/mmcblk0p55", "/data" },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(259, 24), "/dev/block/mmcblk0p56", NULL },
    { (S_IRUSR | S_IWUSR | S_IFBLK), AID_ROOT, AID_ROOT, MKDEV(179, 32), "/dev/block/mmcblk0rpmb", NULL },
    { (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IFCHR), AID_NFC, AID_NFC, MKDEV(10, 254), "/dev/cxd224x-i2c", NULL },
    { 0, 0, 0, 0, 0, 0 },
};

#endif

struct accessible_area_disk_dev {
	loff_t head;
	loff_t tail;
};

static struct accessible_area_disk_dev accessible_areas_fota[] = {
	{ S2B(0x00000000), OFFSET_BYTE_LIMIT },
	{ 0,          0 },
};

static struct accessible_area_disk_dev accessible_areas_sddownloader[] = {
	{ 0,          0 },
};

static struct accessible_area_disk_dev accessible_areas_maker[] = {
	{ 0,          0 },
};

static struct accessible_area_disk_dev accessible_areas_osupdate[] = {
	{ 0,          0 },
};

static struct accessible_area_disk_dev accessible_areas_recovery[] = {
	{ 0,          0 },
};


struct access_control_mmcdl_device {
	enum boot_mode_type boot_mode;
	char *process_name;
	char *process_path;
};

static struct access_control_mmcdl_device mmcdl_device_list[] = {
	{ BOOT_MODE_SDDOWNLOADER, CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_NAME, CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_PATH },
	{ BOOT_MODE_OSUPDATE, CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_NAME, CONFIG_SECURITY_FJSEC_OSUPDATE_MODE_ACCESS_PROCESS_PATH },
	{ 0, 0, 0 },
};

static struct access_control_mmcdl_device mkdrv_device_list[] = {
	{ 0, 0, 0 },
};

static long ptrace_read_request_policy[] = {
	PTRACE_TRACEME,
	PTRACE_PEEKTEXT,
	PTRACE_PEEKDATA,
	PTRACE_PEEKUSR,
	PTRACE_CONT,
	COMPAT_PTRACE_GETREGS,
	PTRACE_ATTACH,
	PTRACE_DETACH,
	COMPAT_PTRACE_GET_THREAD_AREA,
	COMPAT_PTRACE_GETVFPREGS,
	COMPAT_PTRACE_GETHBPREGS,
	PTRACE_GETEVENTMSG,
	PTRACE_GETSIGINFO,
	PTRACE_GETREGSET,
	PTRACE_SEIZE,
	PTRACE_INTERRUPT,
	PTRACE_LISTEN,
	PTRACE_PEEKSIGINFO,
};

struct read_write_access_control_process {
	char *process_name;
	char *process_path;
	uid_t uid;
};

static struct read_write_access_control_process vdsp_process_list[] = {
	{ "wild", "/system/bin/wild", UID_NO_CHECK },
	{ "ueventd", INIT_PROCESS_NAME, UID_NO_CHECK },
	{ 0, 0, 0 },
};

static struct read_write_access_control_process casdrm_process_list_lib[] = {
    { "jp.co.mmbi.app", JAVA_APP_PROCESS_PATH32, UID_NO_CHECK },
    { "jp.co.mmbi.bookviewer", JAVA_APP_PROCESS_PATH32, UID_NO_CHECK },
    { "com.nttdocomo.mmb.android.mmbsv.process", JAVA_APP_PROCESS_PATH32, UID_NO_CHECK },
    { "jp.pixela.stationtv.localtuner.full.app", JAVA_APP_PROCESS_PATH64, AID_MEDIA },
    { "eInfoInstrument", JAVA_APP_PROCESS_PATH64, UID_NO_CHECK },
    { "MmbCaCasDrmMw", "/system/bin/MmbCaCasDrmMw", AID_ROOT },
    { "MmbFcCtlMw", "/system/bin/MmbFcCtlMw", AID_ROOT },
    { "MmbFcLiceMwServ", "/system/bin/MmbFcLiceMwServer", AID_ROOT },
    { "MmbStCtlMwServi", "/system/bin/MmbStCtlMwService", AID_ROOT },
    { "MmbFcMp4MwServe", "/system/bin/MmbFcMp4MwServer", AID_ROOT },
    { "MmbStRecCmMwSer", "/system/bin/MmbStRecCmMwService", AID_ROOT },
    { "fmt_dtv_conflic", "/system/bin/fmt_dtv_conflict", AID_ROOT },
    { "ficsd", "/system/bin/ficsd", AID_SYSTEM },
    { "debuggerd", "/system/bin/debuggerd", AID_ROOT },
    { "debuggerd64", "/system/bin/debuggerd64", AID_ROOT },
    { "installd", "/system/bin/installd", AID_ROOT },
    { 0, 0, 0 },
};

static struct read_write_access_control_process casdrm_process_list_bin[] = {
    { INIT_PROCESS_NAME, INIT_PROCESS_PATH, AID_ROOT },
    { "ficsd", "/system/bin/ficsd", AID_SYSTEM },
	{ 0, 0, 0 },
};

#ifdef DEBUG_COPY_GUARD
static struct read_write_access_control_process test_process_list[] = {
	{ "com.fujitsu.FjsecTest", JAVA_APP_PROCESS_PATH64, AID_LOG },
	{ "ok_native", "/data/lsm-test/tmp/regza1", AID_AUDIO },
	{ PRNAME_NO_CHECK, "/data/lsm-test/tmp/regza2", AID_AUDIO },
	{ "ok_native", "/data/lsm-test/tmp/regza3", UID_NO_CHECK },
	{ 0, 0, 0 },
};
#endif /* DEBUG_COPY_GUARD */

#ifdef FIDO_SEC
// For FIDO
static struct read_write_access_control_process fido1_process_list_bin[] = {
    { INIT_PROCESS_NAME, INIT_PROCESS_PATH, AID_ROOT },
    { INIT_PROCESS_NAME, INIT_PROCESS_PATH, AID_SYSTEM },
    { "mediaserver", "/system/bin/mediaserver", AID_MEDIA },
    { "com.android.phone", JAVA_APP_PROCESS_PATH64, AID_RADIO },
    { "com.android.phone", JAVA_APP_PROCESS_PATH32, AID_RADIO },
    { "ficsd", "/system/bin/ficsd", AID_SYSTEM },
	{ 0, 0, 0 },
};

static struct read_write_access_control_process fido2_process_list_bin[] = {
	{ "mediaserver", "/system/bin/mediaserver", AID_MEDIA },
/* FCNT LIMITED: 2017-02-27 SEC_LSM_044 add start */
    { "cameraserver", "/system/bin/cameraserver", AID_CAMERASERVER },
/* FCNT LIMITED: 2017-02-27 SEC_LSM_044 add end */
	{ "com.android.phone", JAVA_APP_PROCESS_PATH64, AID_RADIO },
	{ "com.android.phone", JAVA_APP_PROCESS_PATH32, AID_RADIO },
	{ "ficsd", "/system/bin/ficsd", AID_SYSTEM },
	{ 0, 0, 0 },
};

static struct read_write_access_control_process fido3_process_list_bin[] = {
	{ "chamomile_provision", "/system/vendor/bin/chamomile_provision", AID_SYSTEM },
	{ "ficsd", "/system/bin/ficsd", AID_SYSTEM },
	{ 0, 0, 0 },
};
//	{ "iontest", "/data/local/tmp/iontest", AID_ROOT }, // for test
#endif

/* FCNT LIMITED:2017-03-27 SEC-LSM-048 add start*/
static struct read_write_access_control_process preinrestore_process_list_bin[] = {
    { "init_preinresto", "/system/vendor/bin/init_preinrestore.sh", AID_ROOT },
    { "init_preinresto", "/system/vendor/bin/init_preinrestoreOSV.sh", AID_SHELL },
    { "init_preinresto", "/system/bin/sh", AID_ROOT },
    { "init_preinresto", "/system/bin/sh", AID_SHELL },
    { "main", JAVA_APP_PROCESS_PATH32, AID_SHELL },
    { "main", JAVA_APP_PROCESS_PATH64, AID_SHELL },
    { "main", JAVA_APP_PROCESS_PATH32, AID_ROOT },
    { "main", JAVA_APP_PROCESS_PATH64, AID_ROOT },
    { INIT_PROCESS_NAME, INIT_PROCESS_PATH, AID_ROOT },
    { "touch", "/system/bin/toybox", AID_ROOT },
    { "touch", "/system/bin/toybox", AID_SHELL },
    { 0, 0, 0 },
};
/* FCNT LIMITED:2017-03-27 SEC-LSM-048 add end*/

static struct read_write_access_control_process nfc_process_list_lib[] = {
    { 0, 0, 0 },
};

static struct read_write_access_control_process mmp_process_list_lib[] = {
    { PRNAME_NO_CHECK, "/system/bin/fjmmpd", AID_MMP },
    { "ficsd", "/system/bin/ficsd", AID_SYSTEM },
    { 0, 0, 0 },
};
static struct read_write_access_control_process mmp2_process_list_lib[] = {
    { PRNAME_NO_CHECK, JAVA_APP_PROCESS_PATH32, AID_MMP },
    { PRNAME_NO_CHECK, JAVA_APP_PROCESS_PATH64, AID_MMP },
    { PRNAME_NO_CHECK, JAVA_APP_PROCESS_PATH32, AID_CAMERA_APP },
    { PRNAME_NO_CHECK, JAVA_APP_PROCESS_PATH64, AID_CAMERA_APP },
    { 0, 0, 0 },
};

/* FCNT LIMITED:2017-03-23 SEC-LSM-039 ADD S */
static struct read_write_access_control_process mmp3_process_list_lib[] = {
    { PRNAME_NO_CHECK, "/system/bin/fjmmpd", AID_MMP },
    { "ficsd", "/system/bin/ficsd", AID_SYSTEM },
    { PRNAME_NO_CHECK, JAVA_APP_PROCESS_PATH32, AID_MMP },
    { PRNAME_NO_CHECK, JAVA_APP_PROCESS_PATH64, AID_MMP },
    { PRNAME_NO_CHECK, JAVA_APP_PROCESS_PATH32, AID_CAMERA_APP },
    { PRNAME_NO_CHECK, JAVA_APP_PROCESS_PATH64, AID_CAMERA_APP },
    { 0, 0, 0 },
};
/* FCNT LIMITED:2017-03-23 SEC-LSM-039 MOD E */

#define NO_STRICT	-1
#define CHK_STRICT	1
struct read_write_access_control {
	char *prefix;
	struct read_write_access_control_process *process_list;
	int f_strict;
};

static struct read_write_access_control rw_access_control_list[] = {
/* FCNT LIMITED:2017-03-23 SEC-LSM-039 MOD S */
	{ "/system/vendor/lib/libMmpAir.so", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpAirLut.so", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpAirWrap.so", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpMsr.so", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpFrc.so", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpDir8.so", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel14_1.o", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel14_2.o", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel14_3.o", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel14_4.o", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel2_4.o", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel2_1.o", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel2_2.o", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel2_3.o", mmp_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpStl.so", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpStlWrap.so", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpStlLut.so", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libpxDir8.so", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libpxDir8tile.so", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel14_2_1.o", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel14_2_2.o", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel14_2_3.o", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libkernel14_2_4.o", mmp2_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpLpfsr.so", mmp3_process_list_lib,CHK_STRICT },
	{ "/system/vendor/lib/libMmpLpfsrWrap.so", mmp3_process_list_lib,CHK_STRICT },
/* FCNT LIMITED:2017-03-23 SEC-LSM-039 MOD E */
	{ "/system/etc/firmware/vw.bin", vdsp_process_list,NO_STRICT },
	{ "/system/etc/firmware/vw_th.bin", vdsp_process_list,NO_STRICT },
	{ "/system/etc/firmware/vp_vw.bin", vdsp_process_list,NO_STRICT },
	{ "/system/etc/firmware/vp_vw_th.bin", vdsp_process_list,NO_STRICT },
	{ "/system/etc/firmware/2b.bin", vdsp_process_list,NO_STRICT },
	{ "/system/lib/libMmbCaCasDrmMw.so", casdrm_process_list_lib,NO_STRICT },
	{ "/system/lib/libMmbCaKyMngMw.so", casdrm_process_list_lib,NO_STRICT },
	{ "/system/bin/MmbCaCasDrmMw", casdrm_process_list_bin,NO_STRICT },
    { "/system/lib64/libnfc-nci-mc.so", nfc_process_list_lib,NO_STRICT },
    { "/system/lib/libnfc-nci-mc.so", nfc_process_list_lib,NO_STRICT },
    { "/system/lib64/nfc_nci.cxd224x-mc.default", nfc_process_list_lib,NO_STRICT },
    { "/system/lib/nfc_nci.cxd224x-mc.default", nfc_process_list_lib,NO_STRICT },
#ifdef FIDO_SEC
	{ "/system/vendor/bin/fidoprov", fido1_process_list_bin,CHK_STRICT },
	{ "/system/lib/libFjAvSensingDetect.so", fido2_process_list_bin,CHK_STRICT },
	{ "/system/lib64/libFjAvSensingDetect.so", fido2_process_list_bin,CHK_STRICT },
	{ "/system/etc/bkfe", fido3_process_list_bin,CHK_STRICT },
	{ "/system/etc/firmware/chamomile.b00", fido3_process_list_bin,CHK_STRICT },
	{ "/system/etc/firmware/chamomile.b01", fido3_process_list_bin,CHK_STRICT },
	{ "/system/etc/firmware/chamomile.b02", fido3_process_list_bin,CHK_STRICT },
	{ "/system/etc/firmware/chamomile.b03", fido3_process_list_bin,CHK_STRICT },
	{ "/system/etc/firmware/chamomile.mdt", fido3_process_list_bin,CHK_STRICT },
#endif
/* FCNT LIMITED:2017-03-27 SEC-LSM-048 add start*/
    { "/data/master/*", preinrestore_process_list_bin, CHK_STRICT },
/* FCNT LIMITED:2017-03-27 SEC-LSM-048 add end*/
#ifdef DEBUG_COPY_GUARD
	{ "/system/lib/libptct.so", test_process_list,CHK_STRICT },
	{ "/data/lsm-test/tmp/ptct/*", test_process_list,NO_STRICT },
#endif /* DEBUG_COPY_GUARD */
	{ 0, 0, 0 },
};

struct ac_config {
	char *prefix;
	enum boot_mode_type boot_mode;
	char *process_name;
	char *process_path;
};

static struct ac_config ptn_acl[] = {
    { "/dev/block/mmcblk0p1", 0, 0, 0 },
    { "/dev/block/mmcblk0p21", 0, 0, 0 },
    { "/dev/block/mmcblk0p5", 0, 0, 0 },
    { "/dev/block/mmcblk0p6", BOOT_MODE_SDDOWNLOADER, "recovery", "/sbin/recovery" },
    { "/dev/block/mmcblk0p6", BOOT_MODE_MASTERCLEAR, "recovery", "/sbin/recovery" },
    { "/dev/block/mmcblk0p30", 0, 0, 0 },
    { "/dev/block/mmcblk0p26", 0, 0, 0 },
    { "/dev/block/mmcblk0p32", 0, 0, 0 },
    { "/dev/block/mmcblk0p24", 0, 0, 0 },
    { "/dev/block/mmcblk0p10", 0, 0, 0 },
    { "/dev/block/mmcblk0p14", BOOT_MODE_NONE, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p14", BOOT_MODE_FOTA, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p14", BOOT_MODE_SDDOWNLOADER, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p14", BOOT_MODE_MAKERCMD, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p14", BOOT_MODE_OSUPDATE, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p14", BOOT_MODE_MASTERCLEAR, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p15", BOOT_MODE_NONE, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p15", BOOT_MODE_FOTA, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p15", BOOT_MODE_SDDOWNLOADER, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p15", BOOT_MODE_MAKERCMD, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p15", BOOT_MODE_OSUPDATE, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p15", BOOT_MODE_MASTERCLEAR, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p11", BOOT_MODE_NONE, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p11", BOOT_MODE_FOTA, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p11", BOOT_MODE_SDDOWNLOADER, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p11", BOOT_MODE_MAKERCMD, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p11", BOOT_MODE_OSUPDATE, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p11", BOOT_MODE_MASTERCLEAR, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p12", BOOT_MODE_NONE, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p12", BOOT_MODE_FOTA, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p12", BOOT_MODE_SDDOWNLOADER, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p12", BOOT_MODE_MAKERCMD, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p12", BOOT_MODE_OSUPDATE, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p12", BOOT_MODE_MASTERCLEAR, "rmt_storage", "/system/bin/rmt_storage" },
    { "/dev/block/mmcblk0p16", 0, 0, 0 },
    { "/dev/block/mmcblk0p17", 0, 0, 0 },
    { "/dev/block/mmcblk0p18", 0, 0, 0 },
    { "/dev/block/mmcblk0p19", 0, 0, 0 },
    { "/dev/block/mmcblk0p20", 0, 0, 0 },
    { "/dev/block/mmcblk0p35", 0, 0, 0 },
    { "/dev/block/mmcblk0p42", BOOT_MODE_MASTERCLEAR, "recovery", "/sbin/recovery" },
    { "/dev/block/mmcblk0p53", BOOT_MODE_MASTERCLEAR, "recovery", "/sbin/recovery" },
    { "/dev/block/mmcblk0p53", BOOT_MODE_SDDOWNLOADER, "recovery", "/sbin/recovery" },
    { "/dev/block/mmcblk0p54", BOOT_MODE_MASTERCLEAR, "recovery", "/sbin/recovery" },
    { "/dev/block/mmcblk0p54", BOOT_MODE_SDDOWNLOADER, "recovery", "/sbin/recovery" },
    { "/dev/block/mmcblk0p56", 0, 0, 0 },
    {0, 0, 0, 0},
};

static struct ac_config fs_acl[] = {
#ifdef DEBUG_ACCESS_CONTROL
	{ "/data/lsm-test/tmp/fspt-ptct/*", BOOT_MODE_NONE, "test", "/data/lsm-test/tmp/test_tool" },
	{ "/data/lsm-test/tmp/fspt-ptct.txt", BOOT_MODE_NONE, "test", "/data/lsm-test/tmp/test_tool" },
	{ "/data/lsm-test/tmp/fspt-ptct2.txt", 0, 0, 0 },
	{ "/data/lsm-test/tmp/fspt-ptct/*", BOOT_MODE_MASTERCLEAR, "prepare", "/data/lsm-test/tmp/test_tool" },
	{ TEST_DIRS_PATH, BOOT_MODE_NONE, "test", "/data/lsm-test/tmp/test_tool" },
#endif /* DEBUG_ACCESS_CONTROL */

	{ 0, 0, 0, 0 },
};

struct ac_config_devmem {
	unsigned long head;
	unsigned long tail;
	char *process_name;
	char *process_path;
};

static struct ac_config_devmem devmem_acl[] = {
	{ ADDR2PFN(0xC3A00000), ADDR2PFN(0xC3B80000)-1, "rmt_storage", "/system/bin/rmt_storage" },
	{ ADDR2PFN(0xC9400000), ADDR2PFN(0xC9410000)-1, "rfs_access", "/system/bin/rfs_access" },
	{ ADDR2PFN(0xC37A0000), ADDR2PFN(0xC37B0000)-1, "rfs_access", "/system/bin/rfs_access" },
	{ ADDR2PFN(0xC9410000), ADDR2PFN(0xC9420000)-1, "rfs_access", "/system/bin/rfs_access" },
};

#ifdef CONFIG_SECURITY_FJSEC_AC_KITTING
struct kitting_directory_access_process {
	char *process_name;
	char *process_path;
	uid_t uid;
};

static struct kitting_directory_access_process kitting_directory_access_process_list[] = {
	{ "com.android.settings", JAVA_APP_PROCESS_PATH32, AID_SYSTEM },
	{ "com.android.settings", JAVA_APP_PROCESS_PATH64, AID_SYSTEM },
	{ "com.android.defcontainer", JAVA_APP_PROCESS_PATH32, UID_NO_CHECK },
	{ "com.android.defcontainer", JAVA_APP_PROCESS_PATH64, UID_NO_CHECK },
	{ "com.fujitsu.mobile_phone.home", JAVA_APP_PROCESS_PATH32, AID_SYSTEM },
	{ "com.fujitsu.mobile_phone.home", JAVA_APP_PROCESS_PATH64, AID_SYSTEM },
    { "system_server", JAVA_APP_PROCESS_PATH32, AID_SYSTEM },
    { "system_server", JAVA_APP_PROCESS_PATH64, AID_SYSTEM },
    { "com.fujitsu.mobile_phone.kitting", JAVA_APP_PROCESS_PATH32, AID_SYSTEM },
    { "com.fujitsu.mobile_phone.kitting", JAVA_APP_PROCESS_PATH64, AID_SYSTEM },
    { "bootanimation", "/system/bin/bootanimation", AID_MEDIA },
    { "com.fujitsu.mobile_phone.kitting", "/system/bin/busybox", AID_SYSTEM },
#ifdef CTS_TEST
	{ "ernalstorageapp", JAVA_APP_PROCESS_PATH32, UID_NO_CHECK  },
	{ "ernalstorageapp", JAVA_APP_PROCESS_PATH64, UID_NO_CHECK  },
#endif  /* CTS_TEST */
#ifdef DEBUG_KITTING_ACCESS_CONTROL
	{ "com.android.settings", "/system/bin/busybox", AID_SYSTEM },
#endif  /* DEBUG_KITTING_ACCESS_CONTROL */
	{ 0, 0, 0 },
};
#endif /* CONFIG_SECURITY_FJSEC_AC_KITTING */

/* FUJITSU LIMITED:2015-06-01 SEC-LSM-002 add start */
struct ac_config_setid{
	char *process_name;
	char *process_path;
};

static struct ac_config_setid setid_acl[] = {
	{ "init", "/init"},
	{ "ueventd", "/init"},
	{ "adbd", "/sbin/adbd"},
#ifdef DEBUG_SETID_ACCESS_CONTROL
	{ "setuid_test", "/data/lsm-test/setuid_test"},
	{ "setuid_test_NG", "/data/lsm-test-bin/setuid_test_NG"},
#endif /* DEBUG_SETID_ACCESS_CONTROL */
#ifdef DEBUG_COPY_GUARD
	{ "ok_native", "/data/lsm-test/tmp/busybox"},
	{ "ok_native", "/data/lsm-test/tmp/busyboxng"},
	{ "ok_native", "/data/lsm-test/tmp/regza1"},
	{ "ok_native", "/data/lsm-test/tmp/regza2"},
	{ "ok_native", "/data/lsm-test/tmp/regza3"},
	{ "ng_native", "/data/lsm-test/tmp/regza1"},
	{ "ng_native", "/data/lsm-test/tmp/regza2"},
#endif /* DEBUG_COPY_GUARD */
#ifdef DEBUG_KITTING_ACCESS_CONTROL
	{ "busybox", "/data/busybox/busybox"},
	{ "ndroid.settings", "/data/busybox/busybox"},
#endif /* DEBUG_KITTING_ACCESS_CONTROL */

	{ 0, 0 },
};
/* FUJITSU LIMITED:2015-06-01 SEC-LSM-002  add end */

#define FJSEC_ACP_NAME_MAX 64
#define FJSEC_ACP_PATH_MAX 64
#define PG_ACL_CON	0xFF
#define PG_ACL_NCON	0x00
#define PG_ACL_ONE	0x01
#define PG_ACL_NCONS	0x02

struct ac_config_page{
	char id[FJSEC_ACP_NAME_MAX];
	char process_name[FJSEC_ACP_NAME_MAX];
	char process_path[FJSEC_ACP_PATH_MAX];
	uid_t uid;
	unsigned long start_pfn;
	unsigned long end_pfn;
	// *NEW* add flg
	unsigned char pfn_flg;
	// *NEW* add supplement params
	long supl1;
	long supl2;
};


enum {
SECURE_MEM_ID	= 0x00100000,
ADSP_MEM_ID	= 0x00200000,
QSEECOM_MEM_ID	= 0x00400000,
AUDIO_MEM_ID	= 0x00800000,
MEM_HOLE_MEM_ID	= 0x01000000,
CONT_SPLASH_MEM_ID	= 0x02000000,
PERIPHERAL_MEM_ID	= 0x04000000,
FTA_MEM_ID	= 0x08000000,
DEFAULT_MEM_ID	= 0x10000000,
EXTERNAL_IMAGE_MEM_ID	= 0x20000000,
DFPS_DATA_MEM_ID	= 0x40000000,
MODEM_MEM_ID	= ADSP_MEM_ID,
};
/* FUJITSU LIMITED:2015-06-16 SEC-LSM-009  mod end */
static struct ac_config_page page_acl_pre[] = {
/* FUJITSU LIMITED:2015-06-16 SEC-LSM-009 mod start */
// get cma info updates part start.
{"secure_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, SECURE_MEM_ID},
//{"modem_adsp_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, ADSP_MEM_ID},
{"modem_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, MODEM_MEM_ID},
{"qseecom_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, QSEECOM_MEM_ID},
{"audio_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, AUDIO_MEM_ID},
{"memory_hole", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, MEM_HOLE_MEM_ID},
{"cont_splash_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, CONT_SPLASH_MEM_ID},
{"peripheral_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, PERIPHERAL_MEM_ID},
{"fta_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, FTA_MEM_ID},
{"default region", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, DEFAULT_MEM_ID},
{"external_image_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, EXTERNAL_IMAGE_MEM_ID},
{"dfps_data_mem", "swapper", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_NO_CHECK, DFPS_DATA_MEM_ID},
// get cma info updates part end.
/* FUJITSU LIMITED:2015-06-16 SEC-LSM-009  mod end */

	// ion driver updates part start.
	{"system", NO_CHECK_STR, NO_CHECK_STR, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},  // ion
	{"kmalloc", NO_CHECK_STR, NO_CHECK_STR, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // ion
	// ion driver updates part end.

	// gpu-kgsl driver updates part start.
#ifdef LSM_PAGEACL_AUTOREGIST
	{"kgsl_one", NO_CHECK_STR, NO_CHECK_STR, UID_NO_CHECK, 0, 0, PG_ACL_ONE, SUPL_NO_CHECK, SUPL_NO_CHECK},		// disp
	{"kgsl", NO_CHECK_STR, JAVA_APP_PROCESS_PATH32, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},		// disp
	{"kgsl", NO_CHECK_STR, JAVA_APP_PROCESS_PATH64, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},		// disp
#else
	{"kgsl_one", NO_CHECK_STR, CHECK_COMMON, UID_NO_CHECK, 0, 0, PG_ACL_ONE, SUPL_NO_CHECK, SUPL_NO_CHECK},		// disp
	{"kgsl", NO_CHECK_STR, JAVA_APP_PROCESS_PATH32, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},		// disp
	{"kgsl", NO_CHECK_STR, JAVA_APP_PROCESS_PATH64, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},		// disp
#endif
	// gpu-kgsl driver updates part end.
//	{"ion_cma", NO_CHECK_STR, CHECK_COMMON, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
	{"sel_mma", NO_CHECK_STR, CHECK_COMMON, AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},

	/* FUJITSU LIMITED:2015-11-13 SEC-LSM-53 mod start */
	{"sel_mma", "servicemanager", "/system/bin/servicemanager", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
	/* FUJITSU LIMITED:2015-11-13 SEC-LSM-53 mod end */

	{"fb_mmap", CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_NAME, CONFIG_SECURITY_FJSEC_SDDOWNLOADER_MODE_ACCESS_PROCESS_PATH, AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // recovery
	{"fb_mmap", CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_NAME, CONFIG_SECURITY_FJSEC_MAKERCMD_MODE_ACCESS_PROCESS_PATH, AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // makercmd
	{"fb_mmap", CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_NAME, CONFIG_SECURITY_FJSEC_FOTA_MODE_ACCESS_PROCESS_PATH, AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // fota
	{"fb_mmap", "charger", HCE_PROCESS_PATH, AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // charger

#ifdef LSM_PAGEACL_MANUALADR
	{"pil", "ueventd", "/init", AID_ROOT, 0x7000, 0xe000, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // modem, adsp pil
#else
	{"pil", "ueventd", "/init", AID_ROOT, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, MODEM_MEM_ID | PERIPHERAL_MEM_ID | AUDIO_MEM_ID}, // modem, adsp pil
#endif

#ifdef LSM_PAGEACL_MANUALADR
	{"pil", "init", "/init", AID_ROOT, 0x7000, 0xe000, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK},  // modem, adsp pil
#else
	{"pil", "init", "/init", AID_ROOT, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, MODEM_MEM_ID | PERIPHERAL_MEM_ID}, // modem, adsp pil
#endif
//	{"binder", NO_CHECK_STR, HCE_PROCESS_PATH, AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK}, 

#ifdef LSM_PAGEACL_MANUALADR
	{"adspion", "rfs_access", "/system/bin/rfs_access", AID_ROOT, 0xc9400, 0xcd400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, //adsp ion heap
#else
	{"adspion", "rfs_access", "/system/bin/rfs_access", AID_ROOT, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, ADSP_MEM_ID | DEFAULT_MEM_ID | SECURE_MEM_ID}, //adsp ion heap, default region
#endif

#ifdef LSM_PAGEACL_MANUALADR
	{"cmadef", "rfs_access", "/system/bin/rfs_access", AID_ROOT, 0xc3400, 0xc7400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, //(cma: defaut region)
	{"cmadef", "rmt_storage", "/system/bin/rmt_storage", AID_ROOT, 0xc3400, 0xc7400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, //(cma: defaut region)
#else
	{"cmadef", "rmt_storage", "/system/bin/rmt_storage", AID_ROOT, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, DEFAULT_MEM_ID}, //(cma: defaut region)
	{"cmadef", "ipacm", "/system/bin/ipacm", AID_NET_ADMIN, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, DEFAULT_MEM_ID}, //(cma: defaut region)
#endif

#ifdef LSM_PAGEACL_MANUALADR
	{"subsys", "modem_rd", "/system/xbin/modem_rd", AID_ROOT, 0xc3400, 0xc7400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, //(cma: defaut region)
#else
	{"subsys", "modem_rd", "/system/xbin/modem_rd", AID_ROOT, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, MODEM_MEM_ID | MEM_HOLE_MEM_ID | DEFAULT_MEM_ID | EXTERNAL_IMAGE_MEM_ID}, //(cma: defaut region, mem hole, modem)
	{"subsys", "kworker/*", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, MODEM_MEM_ID | MEM_HOLE_MEM_ID | DEFAULT_MEM_ID | PERIPHERAL_MEM_ID}, //(cma: defaut region, mem hole, modem)
	{"subsys", "swapper/0", NO_CHECK_STR, AID_ROOT, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, MEM_HOLE_MEM_ID}, // mem hole
//	{"subsys", "modem_rd", "/system/xbin/modem_rd", AID_ROOT, 0, 0, PG_ACL_CON, DEFAULT_MEM_ID, SUPL_NO_CHECK}, //(cma: defaut region)
#endif

#ifdef LSM_PAGEACL_MANUALADR
	{"binder","pm-service", "/system/bin/pm-service", AID_SYSTEM, 0x7000, 0xc800, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // modem pil
#else
	{"binder",NO_CHECK_STR, "/system/bin/pm-service", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, MODEM_MEM_ID}, // modem pil
#endif
//	{"binder","pm-proxy", "/system/bin/pm-proxy", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder","netd", "/system/bin/netd", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},

#ifdef LSM_PAGEACL_MANUALADR
	{"qseecom", "qseecomd", "/system/bin/qseecomd", AID_SYSTEM, 0xc7c00, 0xc9400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // qseecom ion heap
#else
	{"qseecom", "qseecomd", "/system/bin/qseecomd", AID_SYSTEM, 0, 0, PG_ACL_CON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
#endif

	/* FUJITSU LIMITED:2015-11-13 SEC-LSM-53 mod start */
	{"qseecom","gatekeeperd", "/system/bin/gatekeeperd", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID},// qseecom ion
	/* FUJITSU LIMITED:2015-11-13 SEC-LSM-53 mod end */

//	{"binder", NO_CHECK_STR, "/system/bin/surfaceflinger", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder","drmserver", "/system/bin/drmserver", AID_DRM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder","audiod", "/system/bin/audiod", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},

#ifdef LSM_PAGEACL_MANUALADR
	{"binder", NO_CHECK_STR, "/system/bin/mediaserver", AID_MEDIA, 0xe000, 0xe500, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // venus pil
	{"audioion", NO_CHECK_STR, "/system/bin/mediaserver", AID_MEDIA, 0xc7400, 0xc7c00, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // audio ion heap
	{"qsecomion", NO_CHECK_STR, "/system/bin/mediaserver", AID_MEDIA, 0xc7c00, 0xc9400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // qseecom ion heap
#else
	{"binder", NO_CHECK_STR, "/system/bin/mediaserver", AID_MEDIA, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, PERIPHERAL_MEM_ID | AUDIO_MEM_ID | QSEECOM_MEM_ID}, // venus pil
	{"binder", "PT*", JAVA_APP_PROCESS_PATH64, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, PERIPHERAL_MEM_ID | AUDIO_MEM_ID | QSEECOM_MEM_ID}, // venus pil
	{"binder", NO_CHECK_STR, "/system/bin/dtvdeviceservice", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, PERIPHERAL_MEM_ID | AUDIO_MEM_ID | QSEECOM_MEM_ID}, // venus pil
	{"binder", NO_CHECK_STR, "/system/bin/MmbStCtlMwService", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, PERIPHERAL_MEM_ID | AUDIO_MEM_ID | QSEECOM_MEM_ID}, // venus pil
	{"binder", NO_CHECK_STR, "/system/bin/wfdservice", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, PERIPHERAL_MEM_ID | AUDIO_MEM_ID | QSEECOM_MEM_ID}, // venus pil

	{"binder", "MmbFcMp4MwServe", "/system/bin/MmbFcMp4MwServer", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, PERIPHERAL_MEM_ID | AUDIO_MEM_ID}, // venus pil
#endif

//	{"binder", NO_CHECK_STR, "/system/bin/fsegsaveservice", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},

#ifdef LSM_PAGEACL_MANUALADR
	{"qseecom","chamomile_provi", "/system/vendor/bin/chamomile_provision", AID_SYSTEM, 0xc7c00, 0xc9400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK},// qseecom ion heap
	{"qseecom","keystore", "/system/bin/keystore", AID_KEYSTORE, 0xc7c00, 0xc9400, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},// qseecom ion
	{"qseecom","provd", "/system/bin/provd", AID_SYSTEM, 0xc7c00, 0xc9400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // qseecom ion heap
	{"qseecom","rmpd", "/system/bin/rmpd", AID_SYSTEM, 0xc7c00, 0xc9400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // qseecom ion heap
	{"qseecom","hdcp_provd", "/system/bin/hdcp_provd", AID_SYSTEM, 0xc7c00, 0xc9400, PG_ACL_CON, SUPL_NO_CHECK, SUPL_NO_CHECK}, // qseecom ion heap
#else
	{"binder","chamomile_provi", "/system/vendor/bin/chamomile_provision", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID},// qseecom ion heap
	{"binder","keystore", "/system/bin/keystore", AID_KEYSTORE, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID},// qseecom ion
	{"binder","provd", "/system/bin/provd", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder","rmpd", "/system/bin/rmpd", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder","hdcp_provd", "/system/bin/hdcp_provd", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder","wv_service", "/system/bin/wv_service", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder","sec_service_tz", "/system/bin/sec_service_tz", AID_RADIO, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder","recovery", "/sbin/recovery", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder", NO_CHECK_STR, "/system/bin/sceservice", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder", NO_CHECK_STR, "/system/bin/sdservice", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder", NO_CHECK_STR, "/system/vendor/bin/fidodaemon", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder", NO_CHECK_STR, "/system/bin/drmserver", AID_DRM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder", NO_CHECK_STR, JAVA_APP_PROCESS_PATH64, AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
	{"binder", "vold", "/system/bin/vold", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID}, // qseecom ion heap
#endif

//	{"binder","imscmservice", "/system/bin/imscmservice", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder","ATFWD-daemon", "/system/bin/ATFWD-daemon", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
	{"binder", NO_CHECK_STR, "/system/bin/bootanimation", AID_MEDIA, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, CONT_SPLASH_MEM_ID}, // cont splash
//	{"binder", "fjmmpd", "/system/bin/fjmmpd", AID_MMP, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "FjExtDisplayBuf", "/system/bin/FjExtDisplayBufferServer", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "rild", "/system/bin/rild", AID_RADIO, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "labinder", "/system/bin/labinder", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "dtvdeviceservic", "/system/bin/dtvdeviceservice", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
	
//	{"binder", "mm-qcamera-daem", "/system/bin/mm-qcamera-daemon", AID_CAMERA, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "CAM_img_msg", "/system/bin/mm-qcamera-daemon", AID_CAMERA, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "MmbCaCasDrmMw", "/system/bin/MmbCaCasDrmMw", AID_CASDRM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "MmbPoUsbAudioMp", "/system/bin/MmbPoUsbAudioMp", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "MmbFcLiceMwServ", "/system/bin/MmbFcLiceMwServer", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "MmbStRecCmMwSer", "/system/bin/MmbStRecCmMwService", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},

//	{"binder", "dumpsys", "/system/bin/dumpsys", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "dumpsys", "/system/bin/dumpsys", AID_SHELL, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "atrace", "/system/bin/atrace", AID_SHELL, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "screencap", "/system/bin/screencap", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "screencap", "/system/bin/screencap", AID_SHELL, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "screenrecord", "/system/bin/screenrecord", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "screenrecord", "/system/bin/screenrecord", AID_SHELL, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
/* FUJITSU LIMITED:2015-07-07 SEC-LSM-002 add start */
        {"binder", NO_CHECK_STR, "/system/bin/dtvserver", AID_ROOT, 0,0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID | AUDIO_MEM_ID | PERIPHERAL_MEM_ID},
//	{"binder", "racoon", "/system/bin/racoon", AID_VPN, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", "wpa_supplicant", "/system/bin/wpa_supplicant", AID_WIFI, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//        {"binder", "atrace", "/system/bin/atrace", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
/* FUJITSU LIMITED:2015-07-07 SEC-LSM-002  add end */
//	{"binder", NO_CHECK_STR, CHECK_COMMON, AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", NO_CHECK_STR, CHECK_COMMON, AID_RADIO, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", NO_CHECK_STR, CHECK_COMMON, AID_NFC, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", NO_CHECK_STR, JAVA_APP_PROCESS_PATH32, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//	{"binder", NO_CHECK_STR, JAVA_APP_PROCESS_PATH64, UID_NO_CHECK, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
//        {"binder", "service", "/system/bin/service", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK,SUPL_NO_CHECK},
/* FCNT:2017-01-11 SEC-LSM-001  add start */
/* FCNT LIMITED:2017-04-18 SEC-LSM-049  MOD S */
    {"binder", NO_CHECK_STR, "/system/bin/mediacodec", AID_MEDIA_CODEC, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, PERIPHERAL_MEM_ID | AUDIO_MEM_ID},
/* FCNT LIMITED:2017-04-18 SEC-LSM-049  MOD E */
    {"binder", "audioserver", "/system/bin/audioserver", AID_AUDIOSERVER, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, AUDIO_MEM_ID},
/* FCNT:2017-01-11 SEC-LSM-001  add end */
/* FCNT LIMITED:2017-02-17 SEC-LSM-038 add start */
    {"qseecom", "prov_playready", "/system/bin/prov_playready", AID_MEDIA, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA,QSEECOM_MEM_ID},
    {"binder", NO_CHECK_STR, "/system/bin/mediadrmserver", AID_MEDIA, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID },
/* FCNT LIMITED:2017-02-17 SEC-LSM-038 add end */
/* FCNT LIMITED:2017-02-28 SEC-LSM-035 MOD S */
    {"binder", NO_CHECK_STR, "/system/bin/cameraserver", AID_CAMERASERVER, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, QSEECOM_MEM_ID },
/* FCNT LIMITED:2017-02-28 SEC-LSM-035 MOD E */
/* FCNT LIMITED:2017-03-16 SEC_LSM_028 ADD S */
    {"binder", "subsystem_ramdu", "/system/bin/subsystem_ramdump", AID_SYSTEM, 0, 0, PG_ACL_NCON, SUPL_CHK_ALT_ADDR_IONDMA, PERIPHERAL_MEM_ID},
/* FCNT LIMITED:2017-03-16 SEC_LSM_028 ADD E */
/* FCNT LIMITED:2017-04-26 SEC_LSM_051 ADD S */
    {"packet_mmap", "clatd", "/system/bin/clatd", AID_ROOT, 0, 0, PG_ACL_NCON, SUPL_NO_CHECK, SUPL_NO_CHECK},
/* FCNT LIMITED:2017-04-26 SEC_LSM_051 ADD E */
    {"0", "0","0",0,0,0,false, SUPL_NO_CHECK, SUPL_NO_CHECK },

};

static struct ac_config_page *page_acl=NULL;


struct ac_config_page_ext{
	char process_name[FJSEC_ACP_NAME_MAX];
	char process_path[FJSEC_ACP_PATH_MAX];
	char process_hash[SHA256_DIGEST_SIZE];
	uid_t uid;
	long supl1;
	long supl2;
};

#ifdef CTS_TEST
#define CHECKSUM_NATIVEMEDIATEST_SL64 { 0xd7, 0x90, 0xf0, 0x18, 0x1d, 0x0d, 0x43, 0xd4, 0x81, 0x4d, 0xef, 0x41, 0x88, 0x42, 0x88, 0x7a, 0x85, 0x37, 0x91, 0x8b, 0x85,    0x46, 0x1a, 0x53, 0xd5, 0xab, 0x9a, 0x9f, 0x40, 0xc8, 0xea, 0xb9, }
#define CHECKSUM_NATIVEMEDIATEST_SL32 { 0x7a, 0x46, 0xd7, 0xb9, 0x14, 0xec, 0xae, 0x35, 0xff, 0x3b, 0x8f, 0x53, 0xec, 0xcc, 0xda, 0x08, 0x3a, 0xcc, 0xf2, 0xf2, 0x22,    0x46, 0x39, 0x9a, 0xa6, 0xba, 0xe4, 0xc4, 0x75, 0xd8, 0xd2, 0x73, }
#define CHECKSUM_NATIVEMEDIATEST_XA64 { 0x66, 0x53, 0x2d, 0x5b, 0xfb, 0xf5, 0x8c, 0x23, 0x8f, 0x59, 0x49, 0x4e, 0x3d, 0x57, 0xd3, 0x7c, 0xb3, 0xe4, 0x2b, 0x67, 0x79,    0xdc, 0xda, 0x3f, 0x6c, 0x97, 0xbf, 0xdf, 0x21, 0x8e, 0xe1, 0xa5, }
#define CHECKSUM_NATIVEMEDIATEST_XA32 { 0xd2, 0x27, 0x56, 0xfc, 0x45, 0x68, 0x8e, 0xdc, 0xcd, 0xba, 0x55, 0xfe, 0x93, 0xd6, 0xbe, 0xca, 0x38, 0x58, 0x76, 0x7b, 0x96,    0x9c, 0x2b, 0xc6, 0xc3, 0xc4, 0xdb, 0xf1, 0x21, 0x0f, 0xe3, 0xa3, }
#endif

static struct ac_config_page_ext page_acl_ext[] = {
#ifdef CTS_TEST
	{ "NativeMediaTest", "/data/local/tmp/cts-native-tests/NativeMediaTest_SL32", CHECKSUM_NATIVEMEDIATEST_SL32, AID_SHELL, SUPL_NO_CHECK, SUPL_NO_CHECK},
	{ "NativeMediaTest", "/data/local/tmp/cts-native-tests/NativeMediaTest_SL64", CHECKSUM_NATIVEMEDIATEST_SL64, AID_SHELL, SUPL_NO_CHECK, SUPL_NO_CHECK},
	{ "NativeMediaTest", "/data/local/tmp/cts-native-tests/NativeMediaTest_XA64", CHECKSUM_NATIVEMEDIATEST_XA64, AID_SHELL, SUPL_NO_CHECK, SUPL_NO_CHECK},
	{ "NativeMediaTest", "/data/local/tmp/cts-native-tests/NativeMediaTest_XA32", CHECKSUM_NATIVEMEDIATEST_XA32, AID_SHELL, SUPL_NO_CHECK, SUPL_NO_CHECK},
/* FCNT LIMITED: 2017-03-08 SEC_LSM_047 MOD start */
	{ "pool-2-thread-1", JAVA_APP_PROCESS_PATH32, "", UID_NO_CHECK, SUPL_NO_HASHCHECK, SUPL_NO_CHECK},
	{ "pool-2-thread-1", JAVA_APP_PROCESS_PATH64, "", UID_NO_CHECK, SUPL_NO_HASHCHECK, SUPL_NO_CHECK},
/* FCNT LIMITED: 2017-03-08 SEC_LSM_047 MOD end */
#endif
	{"0", "0", {0} },
};

/* FCNT LIMITED: 2017-03-08 SEC_LSM_047 ADD start */
#ifdef CTS_TEST
static struct ac_config_page_ext page_parent_acl_ext[] = {
    { "android.jni.cts", JAVA_APP_PROCESS_PATH32, "", UID_NO_CHECK, SUPL_NO_HASHCHECK, SUPL_NO_CHECK},
    { "android.jni.cts", JAVA_APP_PROCESS_PATH64, "", UID_NO_CHECK, SUPL_NO_HASHCHECK, SUPL_NO_CHECK},
    { "android.os.cts", JAVA_APP_PROCESS_PATH32, "", UID_NO_CHECK, SUPL_NO_HASHCHECK, SUPL_NO_CHECK},
    { "android.os.cts", JAVA_APP_PROCESS_PATH64, "", UID_NO_CHECK, SUPL_NO_HASHCHECK, SUPL_NO_CHECK},
    { "mmon.deviceinfo", JAVA_APP_PROCESS_PATH32, "", UID_NO_CHECK, SUPL_NO_HASHCHECK, SUPL_NO_CHECK},
    { "mmon.deviceinfo", JAVA_APP_PROCESS_PATH64, "", UID_NO_CHECK, SUPL_NO_HASHCHECK, SUPL_NO_CHECK},
    {"0", "0", {0} },
};
#endif
/* FCNT LIMITED: 2017-03-08 SEC_LSM_047 ADD end */

struct ac_config_ptrace{
	char *process_name;
	uid_t uid;
	char *supl_info;
	unsigned long  supl_val_1;
	unsigned long  supl_val_2;
};

static struct ac_config_ptrace ac_config_ptrace_acl[] = {
	{ "debuggerd", AID_ROOT, "", 0, 0,},
	{ "debuggerd64", AID_ROOT, "", 0, 0,},
/* FCNT LIMITED: 2017-02-22 SEC_LSM_043 add start */
	{ "debuggerd", AID_DEBUGGERD, "", 0, 0,},
	{ "debuggerd64", AID_DEBUGGERD, "", 0, 0,},
/* FCNT LIMITED: 2017-02-22 SEC_LSM_043 add start */
	{ "GC", UID_NO_CHECK, "", 0, 0,},
	{ ".android.chrome", UID_NO_CHECK, "com.android.chrome", 15584, 1400,},
	{ 0, 0, "", 0, 0 },
};

#ifdef FIDO_SEC
static struct ac_config_ptrace ac_config_fido_acl[] = {
/* FCNT LIMITED:2017-02-28 SEC-LSM-035 MOD S */
	{ "cameraserver", AID_CAMERASERVER, "/system/bin/cameraserver",  19060 ,904 },
/* FCNT LIMITED:2017-02-28 SEC-LSM-035 END E */
	{ 0, 0, "", 0, 0 },
};

#define FJSEC_ION_NAME_MAX	32
struct ac_config_fido {
	void *cpu_addr;
	unsigned long len;
	dma_addr_t handle;
	pid_t pid;
	char heapname[FJSEC_ION_NAME_MAX];
};

static struct ac_config_fido *fido_acl = NULL;
#endif
