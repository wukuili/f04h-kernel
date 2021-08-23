/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2013-2015
/*----------------------------------------------------------------------------*/
// File Name:
//      mmcdl.c
//
// History:
//      2013.06.11  Created.
//
/*----------------------------------------------------------------------------*/

/* FUJITSU LIMITED:2014-10-27 H1515009 add start */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pagemap.h>

#include <linux/fs.h>
#include <linux/err.h>
#include <linux/cdev.h>

#include <linux/mutex.h>

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#include <linux/mmc/card.h>

#include <linux/proc_fs.h>

#include "mmcdl.h"

#ifdef CONFIG_SECURITY_FJSEC
extern int fjsec_check_mmcdl_access_process(void);
#endif

#define DRIVER_NAME "mmcdl"
#define CLASS_NAME  "mmcdl"
#define DEVICE_NAME "mmcdl"

#define SECTOR_SHIFT	9
#define SECTOR_SIZE		512
#define BIO_INFO_SIZE	32 /* FUJITSU LIMITED:2015-03-19 H1515010-1 add */

#define nprintk(...)

static unsigned int mmcdl_devs  = 1; /* device count */
static unsigned int mmcdl_major = 0;
static unsigned int mmcdl_minor = 0;
static struct cdev mmcdl_cdev;
static struct class *mmcdl_class;

struct mmcdl_device {
	struct mutex lock;
	int partition;
	int initialized;
};

static struct mmcdl_device *mmcdl_device;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <linux/bio.h>
#include <linux/blkdev.h>

static int mmdl_rw_wait_init = 0;
static wait_queue_head_t mmcdl_rw_wait;
static unsigned long mmcdl_rw_wait_flag = 0;

struct mmcdl_bio_info {
	int index;
	struct bio *bio;

	struct page *pages;
	int alloc_pages_order;
	int ret;
	unsigned long len;
	unsigned long offset;
}mmcdl_bio_info_t;

static void mmcdl_wait_event(unsigned long mask)
{
	nprintk(KERN_INFO "mmcdl_wait_event [in]\n");
	wait_event(mmcdl_rw_wait, (mask&mmcdl_rw_wait_flag)==mask);
	nprintk(KERN_INFO "mmcdl_wait_event [out]\n");
}

static void mmcdl_blockrw_end_io(struct bio *bio, int err)
{
	struct mmcdl_bio_info *bio_info = (struct mmcdl_bio_info *)bio->bi_private;;

	if ( !test_bit(BIO_UPTODATE, &bio->bi_flags)) {
		printk(KERN_ERR "mmcdl_blockrw_end_io: I/O error\n");
		bio_info->ret = -1;
	} else {
		bio_info->ret = 0;
	}

	bio_put(bio);

	nprintk(KERN_INFO "mmcdl_blockrw_end_io[M]\n");
	set_bit(bio_info->index, &mmcdl_rw_wait_flag);
	wake_up(&mmcdl_rw_wait);
}

static void mmcdl_page_cahache_release(struct address_space *mapping, uint32_t aSector, size_t sectornum)
{
	struct page *page;
	pgoff_t index;
	size_t start;
	size_t size;
	uint64_t len;

	index = aSector >> (PAGE_SHIFT - SECTOR_SHIFT);
	start = (aSector % (PAGE_SIZE / SECTOR_SIZE)) * SECTOR_SIZE;
	len   = ((uint64_t)sectornum * SECTOR_SIZE);
	do {
		if ((start + len) > PAGE_SIZE) {
			size = PAGE_SIZE - start;
		} else {
			size = len;
		}

		page = find_get_page(mapping, index);
		if (page) {
			page_cache_release(page);
		}

		index++;
		start = 0;
		len -= size;
	} while(len > 0);
}

static __inline int mmddl_do_bio(int rw, const char *devname, uint32_t aSector, uint8_t *buf, size_t sectornum)
{
	struct block_device *bdev;
	int ret;
	fmode_t mode=FMODE_READ|FMODE_WRITE;

	unsigned int  bio_max_sectors;
	unsigned long bio_len;
	unsigned long bio_offset;
	unsigned long bio_wait_mask;
	int bio_nr = 0;
	int i;
/*	struct mmcdl_bio_info bio_info[32]; */         /* FUJITSU LIMITED:2015-03-19 H1515010-1 del */
	struct mmcdl_bio_info bio_info[BIO_INFO_SIZE]; /* FUJITSU LIMITED:2015-03-19 H1515010-1 add */
	struct request_queue *q;

	bdev = lookup_bdev(devname);
	if (IS_ERR(bdev)) {
		printk(KERN_ERR "mmddl_do_bio[%s] lookup_bdev fail.", (rw==READ)?"READ":"WRITE");
		return PTR_ERR(bdev);
	}

	ret = blkdev_get(bdev, mode, NULL);
	if (ret) {
		printk(KERN_ERR "mmddl_do_bio[%s] blkdev_get fail.", (rw==READ)?"READ":"WRITE");
		return ret;
	}

	q = bdev_get_queue(bdev);
	bio_max_sectors = queue_max_sectors(q);
	if (bio_max_sectors == 0) {
		ret = -EIO;
		goto END;
	}

	bio_nr = sectornum/bio_max_sectors;
	if ((sectornum%bio_max_sectors)!=0) {
		bio_nr++;
	}
/*	if (bio_nr > (sizeof(unsigned long) * 8)) { */ /* FUJITSU LIMITED:2015-03-19 H1515010-1 del */
	if (bio_nr > BIO_INFO_SIZE) {                  /* FUJITSU LIMITED:2015-03-19 H1515010-1 add */
		bio_nr = 0;
		ret = -EFBIG;
		goto END;
	}

	bio_offset = 0;
	bio_wait_mask = 0;
	ret = 0;
	memset(bio_info, 0, sizeof(bio_info));

	for (i = 0; i < bio_nr; i++)
	{
		if ((sectornum - bio_offset) > bio_max_sectors) {
			bio_len = bio_max_sectors << SECTOR_SHIFT;
		} else {
			bio_len = (sectornum - bio_offset) << SECTOR_SHIFT;
		}
		bio_info[i].len = bio_len;
		bio_info[i].offset = bio_offset;

		bio_info[i].alloc_pages_order = get_order(bio_len);

		if (unlikely(bio_info[i].alloc_pages_order >= MAX_ORDER)) {
			printk(KERN_ERR "alloc_pages_order over MAX_ORDER order=%d max=%d \n", 
			         bio_info[i].alloc_pages_order,MAX_ORDER);
			ret = -EFBIG;
			break;
		}

		bio_info[i].pages = alloc_pages(GFP_KERNEL, bio_info[i].alloc_pages_order);
		if (!bio_info[i].pages) {
			printk(KERN_ERR "mmddl_do_bio[%s] alloc_pages fail. order=%d, index=%d, offset=%ld", 
			         (rw==READ)?"READ":"WRITE", bio_info[i].alloc_pages_order, i, bio_offset);
			ret = -ENOMEM;
			break;
		}

		if (rw == WRITE) {
			memcpy(page_address(bio_info[i].pages), buf + (bio_offset<<SECTOR_SHIFT), bio_len);
		}

		bio_info[i].index = i;
		bio_info[i].ret = -1;

		bio_info[i].bio = bio_alloc(GFP_NOFS, 1);
		if (!bio_info[i].bio) {
			printk(KERN_ERR "mmddl_do_bio[%s] bio_alloc fail. index=%d, ret=%d", 
			         (rw==READ)?"READ":"WRITE", bio_info[i].index, bio_info[i].ret);
			ret = -ENOMEM;
			break;
		}
		bio_info[i].bio->bi_bdev    = bdev;
		bio_info[i].bio->bi_sector  = aSector + bio_offset;
		bio_info[i].bio->bi_end_io  = mmcdl_blockrw_end_io;
		bio_info[i].bio->bi_private = &bio_info[i];

		clear_bit(i, &mmcdl_rw_wait_flag);

		if (bio_add_page(bio_info[i].bio, bio_info[i].pages, bio_len, 0) < bio_len) {
			printk(KERN_ERR "mmddl_do_bio[%s] bio_add_page fail.", (rw==READ)?"READ":"WRITE");
			bio_put(bio_info[i].bio);
			ret = -EIO;
			break;
		}
		if (!bio_info[i].bio->bi_size) {
			printk(KERN_ERR "mmddl_do_bio[%s] bio->bi_size error.", (rw==READ)?"READ":"WRITE");
			bio_put(bio_info[i].bio);
			ret = -EIO;
			break;
		}

		set_bit(i, &bio_wait_mask);

		submit_bio(rw, bio_info[i].bio);

		bio_offset += (bio_len >> SECTOR_SHIFT);
	}

	if (bio_wait_mask) {
		mmcdl_wait_event(bio_wait_mask);

		for (i = 0; i < bio_nr; i++) {
			if (((1<<i)&bio_wait_mask)!=0) {
				if (bio_info[i].ret != 0) {
					printk(KERN_ERR "mmddl_do_bio[%s] submit_bio fail. index=%d", (rw==READ)?"READ":"WRITE", i);
					ret = -EIO;
					break;
				}
				if (rw == READ) {
					memcpy(buf + (bio_info[i].offset<<SECTOR_SHIFT), page_address(bio_info[i].pages), bio_info[i].len);
				}
			}
		}
	}

END:
	if (rw == WRITE || ret != 0) {
		mmcdl_page_cahache_release(bdev->bd_inode->i_mapping, aSector, sectornum);
	}

	for (i = 0; i < bio_nr; i++) {
		if (bio_info[i].pages) {
			__free_pages(bio_info[i].pages, bio_info[i].alloc_pages_order);
		}
	}

	if (rw == WRITE) {
		sync_blockdev(bdev);
	}

	blkdev_put(bdev, mode);

	return ret;
}

static int block_read(const char *devname, uint32_t aSector, uint8_t *buf, size_t sectornum)
{
	return mmddl_do_bio(READ, devname, aSector, buf, sectornum);
}


static int block_write(const char *devname, uint32_t aSector, uint8_t *buf, size_t sectornum)
{
	return mmddl_do_bio(WRITE, devname, aSector, buf, sectornum);
}

/* FCNT LIMITED:2016-03-28 H16100766 add start */
static int block_erase(const char *devname, uint32_t start, size_t len)
{
	int ret = 0;
	struct block_device *bdev;

	bdev = lookup_bdev(devname);
	if (IS_ERR(bdev)) {
		printk(KERN_ERR "%s lookup_bdev fail.", __FUNCTION__);
		return PTR_ERR(bdev);
	}
	ret = blkdev_issue_discard(bdev, start, len, GFP_KERNEL, 0);

	return ret;
}
/* FCNT LIMITED:2016-03-28 H16100766 add end */

int ReadMMCDL(  uint32_t aSector,
			    uint8_t* aBuff,
			    loff_t   aSize )
{
	int ret = 0;

	nprintk(KERN_INFO "[IN ] ReadMMCDL() sector=%d  size=%lld\n", aSector, aSize);

	ret = block_read(MMCDL_BLOCK_DEV,
	                 aSector,
	                 aBuff,
	                 aSize );

	return ret;
}

int WriteMMCDL( uint32_t aSector,
			    uint8_t* aBuff,
			    loff_t   aSize )
{
	int ret = 0;

	nprintk(KERN_INFO "[IN ] WriteMMCDL() sector=%d  size=%lld\n", aSector, aSize);

	ret = block_write(MMCDL_BLOCK_DEV,
	                  aSector,
	                  aBuff,
	                  aSize );

	return ret;
}

/* FCNT LIMITED:2016-03-28 H16100766 add start */
int FormatMMCDL( uint32_t aSector,
			    loff_t   aSize )
{
	int ret = 0;

	nprintk(KERN_INFO "[IN ] FormatMMCDL() sector=%d  size=%lld\n", aSector, aSize);

	ret = block_erase(MMCDL_BLOCK_DEV,
	                  aSector,
	                  aSize );

	return ret;
}
/* FCNT LIMITED:2016-03-28 H16100766 add end */

static int Init_Setup(void)
{
	return 0;
}

static int Initialize(void)
{
	int ret = 0;

	ret = Init_Setup();

	return ret;
}

/* FUJITSU LIMITED:2014-11-06 H1515016 add start */
static long _mmcdl_ioctl(unsigned cmd, unsigned long arg, mmcdl_data_t *data)
{
	long ret = 0;

	char *workBuff = NULL;

	fmode_t mode=FMODE_READ|FMODE_WRITE;
	unsigned long all_sector_size;
	struct block_device *bdev;

/* FCNT LIMITED:2016-03-28 H16100766 mod start */
	if ((cmd == IOCTL_MMCDLWRITE) ||
	    (cmd == IOCTL_MMCDLREAD)  ) {
/* FCNT LIMITED:2016-03-28 H16100766 mod end */
		if ((data->iSize * SECTOR_SIZE) > UINT_MAX) {
			printk(KERN_INFO "ERROR(ioctl) data->iSize OverFlow\n");
			return -EFAULT;
		}
		workBuff = kzalloc((data->iSize * SECTOR_SIZE), GFP_KERNEL);
		if (workBuff == NULL) {
			printk(KERN_INFO "ERROR(ioctl) kzalloc()\n");
			return -EFAULT;
		}
	}

	switch (cmd) {
	case IOCTL_MMCDLWRITE:
		if (copy_from_user(workBuff, data->iBuff, (data->iSize * SECTOR_SIZE)) != 0) {
			
			printk(KERN_INFO "ERROR(data->iBuff) copy_from_user() \n");
			ret =  -EFAULT;
			break;
		}
		ret = WriteMMCDL(data->iSector, workBuff, data->iSize);
		break;

	case IOCTL_MMCDLREAD:
		ret = ReadMMCDL(data->iSector, workBuff, data->iSize);
		if (ret == 0) {
			if (copy_to_user(data->iBuff, workBuff, (data->iSize * SECTOR_SIZE)) != 0) {
				printk(KERN_INFO "ERROR(data->iBuff) copy_to_user() \n");
				ret = -EFAULT;
			}
		} else {
			ret = -EFAULT;
		}
		break;

	case IOCTL_MMCDLGETSIZE:

		bdev = lookup_bdev(MMCDL_BLOCK_DEV);
		if (IS_ERR(bdev)) {
			printk(KERN_ERR "[%s] lookup_bdev fail\n", __func__);
			return PTR_ERR(bdev);
		}

		ret = blkdev_get(bdev, mode, NULL);
		if (ret) {
			printk(KERN_ERR "[%s] blkdev_get fail\n", __func__);
			return ret;
		}

		all_sector_size = (unsigned long)(bdev->bd_inode->i_size/512);
		printk(KERN_INFO "all_sector_size = %ld \n", all_sector_size);

		if (copy_to_user( (unsigned long *)arg, &all_sector_size, sizeof(all_sector_size)) != 0) {
			printk(KERN_INFO "ERROR(arg) copy_to_user() \n");
			ret = -EFAULT;
		}

		break;

/* FCNT LIMITED:2016-03-28 H16100766 add start */
	case IOCTL_MMCDLFORMAT:
		ret = FormatMMCDL(data->iSector, data->iSize);
		break;
/* FCNT LIMITED:2016-03-28 H16100766 add end */

	default:
		printk(KERN_INFO "[%s]cmd Error!\n", __func__);
		ret = -ENOTTY;
		break;
	}

//fail:
	if (workBuff != NULL) {
		kfree(workBuff);
	}

	return ret;
}
/* FUJITSU LIMITED:2014-11-06 H1515016 add end */

/* FUJITSU LIMITED:2014-10-27 H1515010 mod start */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int mmcdl_ioctl(struct inode *inode, struct file *filp, unsigned cmd, unsigned long arg)
{
	int ret = 0;
#else  /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) */

static long mmcdl_ioctl(struct file *filp, unsigned cmd, unsigned long arg)
{
	long ret = 0;
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) */
/* FUJITSU LIMITED:2014-10-27 H1515010 mod end */

/* FUJITSU LIMITED:2014-11-06 H1515016 mod start */
	mmcdl_data_t data;

	if (copy_from_user(&data, (mmcdl_data_t *)arg, sizeof(mmcdl_data_t)) != 0) {
		nprintk(KERN_INFO "ERROR(arg) copy_from_user() \n");
		return -ENOMEM;
	}

	ret = _mmcdl_ioctl(cmd, arg, &data);

/* FCNT LIMITED:2016-03-28 H16100766 mod start */
	if ((cmd == IOCTL_MMCDLWRITE) ||
	    (cmd == IOCTL_MMCDLREAD)  ) {
/* FCNT LIMITED:2016-03-28 H16100766 mod end */
		if (copy_to_user((mmcdl_data_t *)arg, &data, sizeof(mmcdl_data_t)) != 0) {
			printk(KERN_ERR "APPNV_ERROR: copy_to_user(): %s\n", __func__);
			ret = -ENOMEM;
			goto error;
		}
	}
/* FUJITSU LIMITED:2014-11-06 H1515016 mod end */

error:
	return ret;
}

/* FUJITSU LIMITED:2014-10-27 H1515010 add start */
#ifdef CONFIG_COMPAT
/* FUJITSU LIMITED:2014-11-06 H1515016 add start */
static void _mmcdl_convert_from_compat(compat_mmcdl_data_t *compat_data, mmcdl_data_t *data)
{
	data->iSector       = compat_data->iSector;
	data->iBuff         = (uint8_t*)compat_ptr(compat_data->iBuff);
	data->iSize         = compat_data->iSize;
}

static void _mmcdl_convert_to_compat(mmcdl_data_t *data, compat_mmcdl_data_t *compat_data)
{
	compat_data->iSector    = data->iSector;
	compat_data->iBuff      = (compat_uptr_t)ptr_to_compat(data->iBuff);
	compat_data->iSize      = data->iSize;
}
/* FUJITSU LIMITED:2014-11-06 H1515016 add end */

static long mmcdl_compat_ioctl(struct file *filp, unsigned cmd, unsigned long arg)
{
	long ret;
/* FUJITSU LIMITED:2014-11-06 H1515016 mod start */

	compat_mmcdl_data_t compat_data;
	mmcdl_data_t        data;
	unsigned long uarg = (unsigned long)compat_ptr(arg);

	switch (cmd) {
	case COMPAT_IOCTL_MMCDLWRITE:
		cmd = IOCTL_MMCDLWRITE;
		break;
	case COMPAT_IOCTL_MMCDLREAD:
		cmd = IOCTL_MMCDLREAD;
		break;
	case COMPAT_IOCTL_MMCDLGETSIZE:
		cmd = IOCTL_MMCDLGETSIZE;
		break;
/* FCNT LIMITED:2016-03-28 H16100766 add start */
	case COMPAT_IOCTL_MMCDLFORMAT:
		cmd = IOCTL_MMCDLFORMAT;
		break;
/* FCNT LIMITED:2016-03-28 H16100766 add end */
	default:
		printk(KERN_INFO "Unknown cmd![%lx] \n", (long)cmd);
		ret = -EPERM;
		goto error;
		break;
	}

	if (copy_from_user(&compat_data, (compat_mmcdl_data_t *)uarg, sizeof(compat_mmcdl_data_t)) != 0) {
		printk(KERN_ERR "MMCDL_ERROR: copy_from_user(): %s\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	_mmcdl_convert_from_compat(&compat_data, &data);

	ret = _mmcdl_ioctl(cmd, uarg, &data);

/* FCNT LIMITED:2016-03-28 H16100766 mod start */
	if ((cmd == IOCTL_MMCDLWRITE) ||
	    (cmd == IOCTL_MMCDLREAD)  ) {
/* FCNT LIMITED:2016-03-28 H16100766 mod end */
		_mmcdl_convert_to_compat(&data, &compat_data);

		if (copy_to_user((compat_mmcdl_data_t *)uarg, &compat_data, sizeof(compat_mmcdl_data_t)) != 0) {
			printk(KERN_ERR "MMCDL_ERROR: copy_to_user(): %s\n", __func__);
			ret = -ENOMEM;
			goto error;
		}
	}

error:
/* FUJITSU LIMITED:2014-11-06 H1515016 mod start */

	return ret;
}
#endif /* CONFIG_COMPAT */
/* FUJITSU LIMITED:2014-10-27 H1515010 add end */

static int mmcdl_open(struct inode *inode, struct file *filp)
{
	int ret=0;

#ifdef CONFIG_SECURITY_FJSEC
	if (fjsec_check_mmcdl_access_process()) {
		return -EPERM;
	}
#endif

#ifdef INIT_READ_MMC
	if (!mmcdl_device->initialized) {
		ret = Initialize();
		if (ret == 0) {
			mmcdl_device->initialized = 1;
		}
	}
#endif
	mmcdl_device->partition = 0;

	return ret;
}

static int mmcdl_close(struct inode *inode, struct file *filp)
{
	return 0;
}

/* FUJITSU LIMITED:2014-10-27 H1515010 mod start */
struct file_operations mmcdl_fops = {
	.owner   = THIS_MODULE,
	.open    = mmcdl_open,
	.release = mmcdl_close,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	.ioctl		= mmcdl_ioctl,
#else  /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) */
	.unlocked_ioctl = mmcdl_ioctl,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) */
#ifdef CONFIG_COMPAT
	.compat_ioctl = mmcdl_compat_ioctl,
#endif /* CONFIG_COMPAT */
};
/* FUJITSU LIMITED:2014-10-27 H1515010 mod end */

/*----------------------------------------------------------------------*/
/* procfs entry function                                                */
/*----------------------------------------------------------------------*/
/*!
 @brief mmcdl_write

 /proc/driver/mmcdl write process

 @param [in]  filp   pointer of the file strucure
 @param [in]  ptr    pointer of the write data
 @param [in]  len    length of the write data
 @param [in]  ppos   user data

 @retval -ENOSPC:    no left memory space
 @retval -EFAULT:    system error
 @retval -EINVAL:    parameter error
 @retval      >0:    success

 @note this function is called when data write to /proc/driver/mmcdl.
*/
/* FUJITSU LIMITED:2014-10-27 H1515010 mod start */
/* static int mmcdl_write(struct file *filp, const char *ptr, unsigned long len, void *data) */
static ssize_t mmcdl_write(struct file *filp, const char __user *ptr, size_t len, loff_t *ppos)
/* FUJITSU LIMITED:2014-10-27 H1515010 mod end */
{
	char buf[255];

	if (len >= sizeof(buf)) {
		printk(KERN_INFO "mmcdl_write, -ENOSPC");
		return -ENOSPC;
	}
	if (copy_from_user(buf,ptr,len)) {
		printk(KERN_INFO "mmcdl_write, -EFAULT");
		return -EFAULT;
	}

	buf[len] = 0;
	printk(KERN_INFO "%s",buf);

	return len;
} /* mmcdl_write */


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
static int mmcdl_drv_probe(struct platform_device *pdev)
{
	dev_t dev = MKDEV(0, 0);
	int ret;
	struct device *class_dev = NULL;

	nprintk(KERN_INFO "*** mmcdl probe ***\n");

	mmcdl_device = kzalloc(sizeof(struct mmcdl_device), GFP_KERNEL);
	if (mmcdl_device == NULL) {
		goto error;
	}

	ret = alloc_chrdev_region(&dev, 0, mmcdl_devs, DRIVER_NAME);
	if (ret) {
		goto error;
	}

	mmcdl_major = MAJOR(dev);

    cdev_init(&mmcdl_cdev, &mmcdl_fops);

	mmcdl_cdev.owner = THIS_MODULE;
	mmcdl_cdev.ops   = &mmcdl_fops;

	ret = cdev_add(&mmcdl_cdev, MKDEV(mmcdl_major, mmcdl_minor), 1);
	if (ret) {
		goto error;
	}

	// register class
	mmcdl_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(mmcdl_class)) {
		goto error;
	}

	// register class device
	class_dev = device_create(
					mmcdl_class,
					NULL,
					MKDEV(mmcdl_major, mmcdl_minor),
					NULL,
					"%s",
					DEVICE_NAME);

	mutex_init(&mmcdl_device->lock);

	if (mmdl_rw_wait_init==0) {
		init_waitqueue_head(&mmcdl_rw_wait);
		mmdl_rw_wait_init = 1;
	}

#ifdef INIT_READ_MMC
	mmcdl_device->initialized = 0;
#else
	ret = Initialize();
	if (ret) {
		goto error;
	}
#endif
	nprintk(KERN_INFO "(%s:%d)\n", __FUNCTION__, __LINE__);
	return 0;

error:
	nprintk(KERN_INFO "(%s:%d)\n", __FUNCTION__, __LINE__);
	return -1;
}

static int mmcdl_drv_remove(struct platform_device *pdev)
{
	kfree(mmcdl_device);

	nprintk(KERN_INFO "(%s:%d)\n", __FUNCTION__, __LINE__);
	return 0;
}

static struct platform_device mmcdl_devices = {
	.name = DEVICE_NAME,
	.id   = -1,
	.dev = {
		.dma_mask          = NULL,
		.coherent_dma_mask = 0xffffffff,
	},
};

static struct platform_device *devices[] __initdata = {
	&mmcdl_devices,
};

static struct platform_driver mmcdl_driver = {
	.probe    = mmcdl_drv_probe,
	.remove   = mmcdl_drv_remove,
	.driver   = {
		.name = DRIVER_NAME,
	},
};

/* FUJITSU LIMITED:2014-10-27 H1515010 add start */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	/* nop */
#else  /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) */
static const struct file_operations mmcdl_proc_fops = {
	.write		= mmcdl_write,
};
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) */
/* FUJITSU LIMITED:2014-10-27 H1515010 add end */

static int __init mmcdl_init(void)
{

#define MMCDL_PROC "driver/mmcdl"

	struct proc_dir_entry *entry;

/* FUJITSU LIMITED:2014-10-27 H1515010 mod start */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	entry = create_proc_entry(MMCDL_PROC, 0660, NULL);
	if (entry == NULL) {
		printk(KERN_ERR "create_mmcdl_proc_entry failed\n");
		return -EBUSY;
	}
	entry->write_proc = mmcdl_write;
#else  /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) */
	entry = proc_create(MMCDL_PROC, 0660, NULL, &mmcdl_proc_fops);
	if (entry == NULL) {
		printk(KERN_ERR "create_mmcdl_proc_entry failed\n");
		return -EBUSY;
	}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) */
/* FUJITSU LIMITED:2014-10-27 H1515010 mod end */

	platform_add_devices(devices, ARRAY_SIZE(devices));

	return platform_driver_register(&mmcdl_driver);
}

static void __exit mmcdl_exit(void)
{
	platform_driver_unregister(&mmcdl_driver);
}

module_init(mmcdl_init);
module_exit(mmcdl_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("mmcdl device");
MODULE_LICENSE("GPL");


/* FUJITSU LIMITED:2014-10-27 H1515009 add end */
