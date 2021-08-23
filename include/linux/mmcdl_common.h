/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2013-2014
/*----------------------------------------------------------------------------*/
// File Name:
//      mmcdl_common.h
//
// History:
//      2013.06.11  Created.
//
/*----------------------------------------------------------------------------*/

/* FUJITSU LIMITED:2014-10-27 H1515009 add start */

#ifndef _MMCDL_COMMON_H
#define _MMCDL_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <linux/ioctl.h>

/*------------------------------------------------------------------*/
/*------------------------------------------------------------------*/
typedef struct mmcdl_data {
	uint32_t        iSector;       
	uint8_t*        iBuff;         
	loff_t          iSize;         
} mmcdl_data_t;

#define IOC_MAGIC_MMCDL 'w'

#define IOCTL_MMCDLWRITE	_IOW(IOC_MAGIC_MMCDL, 1, mmcdl_data_t)
#define IOCTL_MMCDLREAD		_IOR(IOC_MAGIC_MMCDL, 2, mmcdl_data_t)
#define IOCTL_MMCDLGETSIZE	_IOW(IOC_MAGIC_MMCDL, 3, unsigned long *)
/* FCNT LIMITED:2016-03-28 H16100766 add start */
#define IOCTL_MMCDLFORMAT	_IOW(IOC_MAGIC_MMCDL, 4, mmcdl_data_t)
/* FCNT LIMITED:2016-03-28 H16100766 add end */

/* FUJITSU LIMITED:2014-11-06 H1515016 add start */
#ifdef CONFIG_COMPAT
typedef struct compat_mmcdl_data {
	uint32_t            iSector;
	compat_uptr_t       iBuff;
	loff_t              iSize;
} compat_mmcdl_data_t;

#define COMPAT_IOCTL_MMCDLWRITE		_IOW(IOC_MAGIC_MMCDL, 1, compat_mmcdl_data_t)
#define COMPAT_IOCTL_MMCDLREAD		_IOR(IOC_MAGIC_MMCDL, 2, compat_mmcdl_data_t)
#define COMPAT_IOCTL_MMCDLGETSIZE	_IOW(IOC_MAGIC_MMCDL, 3, compat_uptr_t)
/* FCNT LIMITED:2016-03-28 H16100766 add start */
#define COMPAT_IOCTL_MMCDLFORMAT	_IOW(IOC_MAGIC_MMCDL, 4, compat_mmcdl_data_t)
/* FCNT LIMITED:2016-03-28 H16100766 add end */
#endif /* CONFIG_COMPAT */
/* FUJITSU LIMITED:2014-11-06 H1515016 add end */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MMCDL_COMMON_H */

/* FUJITSU LIMITED:2014-10-27 H1515009 add end */
