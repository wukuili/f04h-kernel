/* COPYRIGHT(C) FUJITSU LIMITED 2015 */

#ifndef _LINUX_MSM_ION_EX_H
#define _LINUX_MSM_ION_EX_H

/* FUJITSU:2015-01-06 SEC Mod <S> */
#define FIDO_SEC
/* FUJITSU:2015-01-06 SEC Mod <E> */

#include <linux/msm_ion.h>
/**
 * Flag to use when allocating to indicate that a heap is for FIDO.
 */
#define ION_FLAG_FIDO (1 << 19)
#define ION_IS_FIDO(__flags)	((__flags) & ION_FLAG_FIDO)

/*
 * struct ion_clone_data - for send command ioctl request
 * @vaddr   - virtual start address to be mapped
 * @length  - data length in the buffer
 * @fd      - ion memory fd
 */
struct ion_clone_data {
	void				*vaddr;
	size_t				length;
	int 				fd;
};

/**
 * ioctl cmd for FIDO.
 */
#define ION_IOC_CLONE	_IOWR(ION_IOC_MAGIC, 8, struct ion_clone_data)

int ion_heap_allow_fido_allocation(enum ion_heap_type type);

/**
 * These are the only ids that should be used for Ion heap ids.
 * The ids listed are the order in which allocation will be attempted
 * if specified. Don't swap the order of heap ids unless you know what
 * you are doing!
 * Id's are spaced by purpose to allow new Id's to be inserted in-between (for
 * possible fallbacks)
 */
#if 0
enum ion_heap_ids {
	INVALID_HEAP_ID = -1,
	ION_CP_MM_HEAP_ID = 8,
	ION_CP_MFC_HEAP_ID = 12,
	ION_CP_WB_HEAP_ID = 16, /* 8660 only */
	ION_CAMERA_HEAP_ID = 20, /* 8660 only */
	ION_SYSTEM_CONTIG_HEAP_ID = 21,
	ION_ADSP_HEAP_ID = 22,
	ION_PIL1_HEAP_ID = 23, /* Currently used for other PIL images */
	ION_SF_HEAP_ID = 24,
	ION_SYSTEM_HEAP_ID = 25,
	ION_PIL2_HEAP_ID = 26, /* Currently used for modem firmware images */
	ION_QSECOM_HEAP_ID = 27,
	ION_AUDIO_HEAP_ID = 28,

	ION_MM_FIRMWARE_HEAP_ID = 29,

	ION_HEAP_ID_RESERVED = 31 /** Bit reserved for ION_FLAG_SECURE flag */
};
#endif

#endif //_LINUX_MSM_ION_EX_H
