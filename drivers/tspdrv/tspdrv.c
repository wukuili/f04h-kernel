/*
** =========================================================================
** File:
**     tspdrv.c
**
** Description: 
**     TouchSense Kernel Module main entry-point.
**
** Portions Copyright (c) 2008-2014 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2013-2016
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2017
/*----------------------------------------------------------------------------*/

#ifndef __KERNEL__
#define __KERNEL__
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include "tspdrv.h"
#include <linux/nonvolatile_common.h>    /* FUJITSU:2016-01-20 ADD */

static int g_nTimerPeriodMs = 5; /* 5ms timer by default. This variable could be used by the SPI.*/

#ifdef VIBE_RUNTIME_RECORD
/* Flag indicating whether runtime recorder on or off */
static atomic_t g_bRuntimeRecord;
#endif

/* Flag indicating whether the driver is in use */
static char g_bIsPlaying = false;

#include "ImmVibeSPI.c"
#if (defined(VIBE_DEBUG) && defined(VIBE_RECORD)) || defined(VIBE_RUNTIME_RECORD)
#include <tspdrvRecorder.c>
#endif

/* Device name and version information */
#define VERSION_STR " v5.0.20.1\n"                  /* DO NOT CHANGE - this is auto-generated */
#define VERSION_STR_LEN 16                          /* account extra space for future extra digits in version number */
static char g_szDeviceName[  (VIBE_MAX_DEVICE_NAME_LENGTH 
                            + VERSION_STR_LEN)
                            * NUM_ACTUATORS];       /* initialized in init_module */
static size_t g_cchDeviceName;                      /* initialized in init_module */


/* Flag indicating the debug level*/
static atomic_t g_nDebugLevel;


/* Buffer to store data sent to SPI */
#define MAX_SPI_BUFFER_SIZE (NUM_ACTUATORS * (VIBE_OUTPUT_SAMPLE_SIZE + SPI_HEADER_SIZE))

static char g_cWriteBuffer[MAX_SPI_BUFFER_SIZE];


#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(2,6,0))
#error Unsupported Kernel version
#endif

#ifndef HAVE_UNLOCKED_IOCTL
#define HAVE_UNLOCKED_IOCTL 0
#endif

#ifdef IMPLEMENT_AS_CHAR_DRIVER
static int g_nMajor = 0;
#endif



/* Needs to be included after the global variables because they use them */
#include "tspdrvOutputDataHandler.c"
#ifdef CONFIG_HIGH_RES_TIMERS
    #include "VibeOSKernelLinuxHRTime.c"
#else
    #include "VibeOSKernelLinuxTime.c"
#endif

asmlinkage void _DbgOut(int level, const char *fmt,...)
{
    static char printk_buf[MAX_DEBUG_BUFFER_LENGTH];
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    static char prefix[6][4] = 
//        {" * ", " ! ", " ? ", " I ", " V", " O "};
    char *prefix;
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    int nDbgLevel = atomic_read(&g_nDebugLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    if (0 <= level && level <= nDbgLevel) {
    switch (level)
    {
        case DBL_FATAL:
            prefix = " * ";
            break;
        case DBL_ERROR:
            prefix = " ! ";
            break;
        case DBL_WARNING:
            prefix = " ? ";
            break;
        case DBL_INFO:
            prefix = " I ";
            break;
        case DBL_VERBOSE:
            prefix = " V ";
            break;
        case DBL_OVERKILL:
            prefix = " O ";
            break;
        default:
            return;
    }

    if (level & nDbgLevel) {
/* FUJITSU LIMITED:2016-01-20 MOD-E */
        va_list args;
        int ret;
        size_t size = sizeof(printk_buf);

        va_start(args, fmt);

/* FUJITSU LIMITED:2016-01-20 MOD-S */
//        ret = scnprintf(printk_buf, size, KERN_EMERG "%s:%s %s",
//             MODULE_NAME, prefix[level], fmt);
        ret = scnprintf(printk_buf, size, KERN_EMERG "%s:%s %s",
             MODULE_NAME, prefix, fmt);
/* FUJITSU LIMITED:2016-01-20 MOD-E */

        if (ret < size)
            vprintk(printk_buf, args);

        va_end(args);
    }
}

asmlinkage static void _DbgOutV(int level, const char *fmt,va_list args)
{
    static char printk_buf[MAX_DEBUG_BUFFER_LENGTH];
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    static char prefix[6][4] =
//        {" * ", " ! ", " ? ", " I ", " V", " O "};
    char *prefix;
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    int ret;
    size_t size = sizeof(printk_buf);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    ret = scnprintf(printk_buf, size, KERN_EMERG "%s:%s %s",
//         MODULE_NAME, prefix[level], fmt);
    switch (level)
    {
        case DBL_FATAL:
            prefix = " * ";
            break;
        case DBL_ERROR:
            prefix = " ! ";
            break;
        case DBL_WARNING:
            prefix = " ? ";
            break;
        case DBL_INFO:
            prefix = " I ";
            break;
        case DBL_VERBOSE:
            prefix = " V ";
            break;
        case DBL_OVERKILL:
            prefix = " O ";
            break;
        default:
            return;
    }

    ret = scnprintf(printk_buf, size, KERN_EMERG "%s:%s %s",
         MODULE_NAME, prefix, fmt);
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    if (ret < size)
        vprintk(printk_buf, args);

}

asmlinkage void _DbgOutTemp(const char *fmt,...)
{
    int nDbgLevel = atomic_read(&g_nDebugLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    if (DBL_TEMP <= nDbgLevel)
    if (DBL_TEMP & nDbgLevel)
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    {
        va_list args;
        va_start(args, fmt);
        _DbgOutV(DBL_TEMP, fmt, args);
        va_end(args);
    }
}

asmlinkage void _DbgOutFatal(const char *fmt,...)
{
    int nDbgLevel = atomic_read(&g_nDebugLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    if (DBL_FATAL <= nDbgLevel)
    if (DBL_FATAL & nDbgLevel)
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    {
        va_list args;
        va_start(args, fmt);
        _DbgOutV(DBL_FATAL, fmt, args);
        va_end(args);
    }
}

asmlinkage void _DbgOutErr(const char *fmt,...)
{
    int nDbgLevel = atomic_read(&g_nDebugLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    if (DBL_ERROR <= nDbgLevel)
    if (DBL_ERROR & nDbgLevel)
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    {
        va_list args;
        va_start(args, fmt);
        _DbgOutV(DBL_ERROR, fmt, args);
        va_end(args);
    }
}

asmlinkage void _DbgOutWarn(const char *fmt,...)
{
    int nDbgLevel = atomic_read(&g_nDebugLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    if (DBL_WARNING <= nDbgLevel)
    if (DBL_WARNING & nDbgLevel)
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    {
        va_list args;
        va_start(args, fmt);
        _DbgOutV(DBL_WARNING, fmt, args);
        va_end(args);
    }
}

asmlinkage void _DbgOutInfo(const char *fmt,...)
{
    int nDbgLevel = atomic_read(&g_nDebugLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    if (DBL_INFO <= nDbgLevel)
    if (DBL_INFO & nDbgLevel)
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    {
        va_list args;
        va_start(args, fmt);
        _DbgOutV(DBL_INFO, fmt, args);
        va_end(args);
    }
}

asmlinkage void _DbgOutVerbose(const char *fmt,...)
{
    int nDbgLevel = atomic_read(&g_nDebugLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    if (DBL_VERBOSE <= nDbgLevel)
    if (DBL_VERBOSE & nDbgLevel)
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    {
        va_list args;
        va_start(args, fmt);
        _DbgOutV(DBL_VERBOSE, fmt, args);
        va_end(args);
    }
}

asmlinkage void _DbgOutOverkill(const char *fmt,...)
{
    int nDbgLevel = atomic_read(&g_nDebugLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    if (DBL_OVERKILL <= nDbgLevel)
    if (DBL_OVERKILL & nDbgLevel)
/* FUJITSU LIMITED:2016-01-20 MOD-E */
    {
        va_list args;
        va_start(args, fmt);
        _DbgOutV(DBL_OVERKILL, fmt, args);
        va_end(args);
    }
}



/* File IO */
static int open(struct inode *inode, struct file *file);
static int release(struct inode *inode, struct file *file);
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos);
#if HAVE_UNLOCKED_IOCTL
static long unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#else
static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#endif
/* FUJITSU LIMITED:2016-01-20 ADD-S */
#ifdef CONFIG_COMPAT
static long compat_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
#endif
/* FUJITSU LIMITED:2016-01-20 ADD-E */

static struct file_operations fops = 
{
    .owner =            THIS_MODULE,
    .read =             read,
    .write =            write,
#if HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl =   unlocked_ioctl,
/* FUJITSU LIMITED:2016-01-20 DEL-S */
//#if HAVE_COMPAT_IOCTL
//    .compat_ioctl   =   unlocked_ioctl,
//#endif
/* FUJITSU LIMITED:2016-01-20 DEL-E */
#else
    .ioctl =            ioctl,
#endif
    .open =             open,
    .release =          release,
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    .llseek =           default_llseek    /* using default implementation as declared in linux/fs.h */
    .llseek =           default_llseek,    /* using default implementation as declared in linux/fs.h */
#ifdef CONFIG_COMPAT
    .compat_ioctl = compat_unlocked_ioctl
#endif  /* CONFIG_COMPAT */
/* FUJITSU LIMITED:2016-01-20 MOD-E */
};

#ifndef IMPLEMENT_AS_CHAR_DRIVER
static struct miscdevice miscdev = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     MODULE_NAME,
	.fops =     &fops
};
#endif

static int suspend(struct platform_device *pdev, pm_message_t state);
static int resume(struct platform_device *pdev);
static struct platform_driver platdrv = 
{
    .suspend =  suspend,	
    .resume =   resume,	
    .driver = 
    {		
        .name = MODULE_NAME,	
    },	
};

static void platform_release(struct device *dev);
static struct platform_device platdev = 
{	
	.name =     MODULE_NAME,	
	.id =       -1,                     /* means that there is only one device */
	.dev = 
    {
		.platform_data = NULL, 		
		.release = platform_release,    /* a warning is thrown during rmmod if this is absent */
	},
};

/* Module info */
MODULE_AUTHOR("Immersion Corporation");
MODULE_DESCRIPTION("TouchSense Kernel Module");
MODULE_LICENSE("GPL v2");


/* enable interface from timed output class */

static void qpnp_hap_td_enable(struct timed_output_dev *dev, int value)
{
	struct qpnp_hap *hap = container_of(dev, struct qpnp_hap,
					 timed_dev);

	DbgOutInfo(("tspdrv: qpnp_hap_td_enable value=%d.\n", value));
	mutex_lock(&hap->lock);
	hrtimer_cancel(&hap->hap_timer);

	if (value == 0) {
		if (hap->state == 0) {
			mutex_unlock(&hap->lock);
			return;
		}
		hap->state = 0;
	} else {
		value = (value > hap->timeout_ms ?
				 hap->timeout_ms : value);
		hap->state = 1;
		hrtimer_start(&hap->hap_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	mutex_unlock(&hap->lock);
	queue_work(hap->hap_wq, &hap->work);
}

/* worker to opeate haptics */
static void qpnp_hap_worker(struct work_struct *work)
{
	struct qpnp_hap *hap = container_of(work, struct qpnp_hap,
					 work);

	//if (hap->play_mode == QPNP_HAP_PWM)
	//	qpnp_hap_mod_enable(hap, hap->state);
	//qpnp_hap_set(hap, hap->state);

	char write_cmd[4] = {0x00, 0x08, 0x01, 0x00};

	if(1 == hap->state){
		//Amp enable
		ImmVibeSPI_ForceOut_AmpEnable(1);
		//write()
		g_bIsPlaying = true;
		write_cmd[3] = (char)(hap->vmax_mv * 13 / 1000 - 10);
		SaveOutputData(write_cmd, 4);
		VibeOSKernelLinuxStartTimer();
	} else {
		//write()
		g_bIsPlaying = true;
		write_cmd[3] = 0;
		SaveOutputData(write_cmd, 4);
		VibeOSKernelLinuxStartTimer();
		//Amp disable
		ImmVibeSPI_ForceOut_AmpDisable(1);
	}

}

/* get time api to know the remaining time */
static int qpnp_hap_get_time(struct timed_output_dev *dev)
{
	struct qpnp_hap *hap = container_of(dev, struct qpnp_hap,
							 timed_dev);

	if (hrtimer_active(&hap->hap_timer)) {
		ktime_t r = hrtimer_get_remaining(&hap->hap_timer);
		return (int)ktime_to_us(r);
	} else {
		return 0;
	}
}

/* hrtimer function handler */
static enum hrtimer_restart qpnp_hap_timer(struct hrtimer *timer)
{
	struct qpnp_hap *hap = container_of(timer, struct qpnp_hap,
							 hap_timer);

	hap->state = 0;
	queue_work(hap->hap_wq, &hap->work);

	return HRTIMER_NORESTART;
}

static void set_NvVmax(struct qpnp_hap *hap)
{
	int rc = 0;
	char write_cmd[4] = {0x00, 0x08, 0x01, 0x00};
	unsigned char vibe_getNvData[APNV_SIZE_VIB_MAGNITUDE];

	/* auto resonance period read */
	memset(vibe_getNvData, 0x00, sizeof(vibe_getNvData));
	rc = get_nonvolatile(vibe_getNvData, APNV_VIB_MAGNITUDE, APNV_SIZE_VIB_MAGNITUDE);
	if (unlikely(rc < 0))
	{
		DbgOutErr((": %s ERROR get_nonvolatile(APNV_VIB_MAGNITUDE) - return code: %d.\n", __func__, rc));
		hap->vmax_mv = FJ_HAP_VMAX_DEF_MV;
	} else {
		hap->vmax_mv = ((unsigned int)(vibe_getNvData[APNV_VIB_MAGNITUDE_HI])
			+ ((unsigned int)vibe_getNvData[APNV_VIB_MAGNITUDE_LO] << 8));
	}

	if ((hap->vmax_mv < FJ_HAP_VMAX_MIN_MV) || (hap->vmax_mv > FJ_HAP_VMAX_MAX_MV))
		hap->vmax_mv = FJ_HAP_VMAX_DEF_MV;

	write_cmd[3] = (char)(hap->vmax_mv * 13 / 1000 - 10);
	SaveOutputData(write_cmd, 4);
	SendOutputData();

	return;
}

static int __init tspdrv_init(void)
{
    int nRet, i;   /* initialized below */
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    atomic_set(&g_nDebugLevel, DBL_ERROR);
    int ret;
    uint8_t vibe_debug_mask;
    unsigned char vibe_getNvData[APNV_SIZE_VIBE_DBG_MODE];
    
    memset(vibe_getNvData, 0x00, sizeof(vibe_getNvData));
    ret = get_nonvolatile(vibe_getNvData, APNV_VIBE_DBG_MODE, APNV_SIZE_VIBE_DBG_MODE);
    if (unlikely(ret < 0)) {
        printk(KERN_INFO "%s: get_nonvolatile() [ret=%d]\n", MODULE_NAME, ret);
        vibe_getNvData[APNV_ADR_VIBE_DBG_MODE] = DBL_ERROR|DBL_TEMP;
    }
    vibe_debug_mask = vibe_getNvData[APNV_ADR_VIBE_DBG_MODE];

    atomic_set(&g_nDebugLevel, vibe_debug_mask);
/* FUJITSU LIMITED:2016-01-20 MOD-E */
#ifdef VIBE_RUNTIME_RECORD
    atomic_set(&g_bRuntimeRecord, 0);
    DbgOutErr(("*** tspdrv: runtime recorder feature is ON for debugging which should be OFF in release version.\n"
                        "*** tspdrv: please turn off the feature by removing VIBE_RUNTIME_RECODE macro.\n"));
#endif
    DbgOutInfo(("tspdrv: init_module.\n"));

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    g_nMajor = register_chrdev(0, MODULE_NAME, &fops);
    if (g_nMajor < 0) 
    {
        DbgOutErr(("tspdrv: can't get major number.\n"));
        return g_nMajor;
    }
#else
    nRet = misc_register(&miscdev);
	if (nRet) 
    {
        DbgOutErr(("tspdrv: misc_register failed.\n"));
		return nRet;
	}
#endif

	nRet = platform_device_register(&platdev);
	if (nRet) 
    {
        DbgOutErr(("tspdrv: platform_device_register failed.\n"));
    }

	nRet = platform_driver_register(&platdrv);
	if (nRet) 
    {
        DbgOutErr(("tspdrv: platform_driver_register failed.\n"));
    }

    DbgRecorderInit(());

    ImmVibeSPI_ForceOut_Initialize();
    VibeOSKernelLinuxInitTimer();
    ResetOutputData();

    /* Get and concatenate device name and initialize data buffer */
    g_cchDeviceName = 0;
    for (i=0; i<NUM_ACTUATORS; i++)
    {
        char *szName = g_szDeviceName + g_cchDeviceName;
        ImmVibeSPI_Device_GetName(i, szName, VIBE_MAX_DEVICE_NAME_LENGTH);

        /* Append version information and get buffer length */
        strcat(szName, VERSION_STR);
        g_cchDeviceName += strlen(szName);

    }

    set_NvVmax(g_pTheClient);

    INIT_WORK(&g_pTheClient->work, qpnp_hap_worker);
    g_pTheClient->hap_wq = create_singlethread_workqueue("hap_wq");
    if(!g_pTheClient->hap_wq)
    {
        dev_err(&g_pTheClient->spmi->dev, "hap_wq create_singlethread_workqueue() failed.\n");
        nRet = -ENOMEM;
        goto create_wq_fail;
    }

    hrtimer_init(&g_pTheClient->hap_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    g_pTheClient->hap_timer.function = qpnp_hap_timer;

    g_pTheClient->timed_dev.name = "vibrator";
    g_pTheClient->timed_dev.get_time = qpnp_hap_get_time;
    g_pTheClient->timed_dev.enable = qpnp_hap_td_enable;

    nRet = timed_output_dev_register(&g_pTheClient->timed_dev);
    if(nRet < 0) {
        dev_err(&g_pTheClient->spmi->dev, "timed_output registration failed.\n");
        goto register_timed_output_dev_fail;
    }


    return 0;

register_timed_output_dev_fail:
    hrtimer_cancel(&g_pTheClient->hap_timer);
    destroy_workqueue(g_pTheClient->hap_wq);

create_wq_fail:
    cancel_work_sync(&g_pTheClient->work);

    VibeOSKernelLinuxTerminateTimer();
    ImmVibeSPI_ForceOut_Terminate();

	platform_driver_unregister(&platdrv);
	platform_device_unregister(&platdev);

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    unregister_chrdev(g_nMajor, MODULE_NAME);
#else
    misc_deregister(&miscdev);
#endif

	return nRet;

}

static void __exit tspdrv_exit(void)
{
    DbgOutInfo(("tspdrv: cleanup_module.\n"));

    DbgRecorderTerminate(());

    VibeOSKernelLinuxTerminateTimer();
    ImmVibeSPI_ForceOut_Terminate();

    platform_driver_unregister(&platdrv);
    platform_device_unregister(&platdev);

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    unregister_chrdev(g_nMajor, MODULE_NAME);
#else
    misc_deregister(&miscdev);
#endif

    cancel_work_sync(&g_pTheClient->work);
    hrtimer_cancel(&g_pTheClient->hap_timer);
    destroy_workqueue(g_pTheClient->hap_wq);
    timed_output_dev_unregister(&g_pTheClient->timed_dev);

}

static int open(struct inode *inode, struct file *file) 
{
    DbgOutInfo(("tspdrv: open.\n"));

    if (!try_module_get(THIS_MODULE)) return -ENODEV;

    return 0; 
}

static int release(struct inode *inode, struct file *file) 
{
    DbgOutInfo(("tspdrv: release.\n"));

    /* 
    ** Reset force and stop timer when the driver is closed, to make sure
    ** no dangling semaphore remains in the system, especially when the
    ** driver is run outside of immvibed for testing purposes.
    */
    VibeOSKernelLinuxStopTimer();

    /* 
    ** Clear the variable used to store the magic number to prevent 
    ** unauthorized caller to write data. TouchSense service is the only 
    ** valid caller.
    */
    file->private_data = (void*)NULL;

    module_put(THIS_MODULE);

    return 0; 
}

static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    const size_t nBufSize = (g_cchDeviceName > (size_t)(*ppos)) ? min(count, g_cchDeviceName - (size_t)(*ppos)) : 0;

/* FUJITSU LIMITED:2016-01-20 ADD-S */
    DbgOutInfo(("tspdrv: read.\n"));
/* FUJITSU LIMITED:2016-01-20 ADD-E */

    /* End of buffer, exit */
    if (0 == nBufSize) return 0;

    if (0 != copy_to_user(buf, g_szDeviceName + (*ppos), nBufSize)) 
    {
        /* Failed to copy all the data, exit */
        DbgOutErr(("tspdrv: copy_to_user failed.\n"));
        return 0;
    }

    /* Update file position and return copied buffer size */
    *ppos += nBufSize;
    return nBufSize;
}

static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    *ppos = 0;  /* file position not used, always set to 0 */

/* FUJITSU LIMITED:2016-01-20 ADD-S */
    DbgOutInfo(("tspdrv: write.\n"));
/* FUJITSU LIMITED:2016-01-20 ADD-E */

    /* 
    ** Prevent unauthorized caller to write data. 
    ** TouchSense service is the only valid caller.
    */
    if (file->private_data != (void*)TSPDRV_MAGIC_NUMBER) 
    {
        DbgOutErr(("tspdrv: unauthorized write.\n"));
        return -EACCES;
    }

    /* 
    ** Ignore packets that have size smaller than SPI_HEADER_SIZE or bigger than MAX_SPI_BUFFER_SIZE.
    ** Please note that the daemon may send an empty buffer (count == SPI_HEADER_SIZE)
    ** during quiet time between effects while playing a Timeline effect in order to maintain
    ** correct timing: if "count" is equal to SPI_HEADER_SIZE, the call to VibeOSKernelLinuxStartTimer()
    ** will just wait for the next timer tick.
    */
    if ((count < SPI_HEADER_SIZE) || (count > MAX_SPI_BUFFER_SIZE))
    {
        DbgOutErr(("tspdrv: invalid buffer size.\n"));
        return -EINVAL;
    }

    /* Copy immediately the input buffer */
    if (0 != copy_from_user(g_cWriteBuffer, buf, count))
    {
        /* Failed to copy all the data, exit */
        DbgOutErr(("tspdrv: copy_from_user failed.\n"));
        return -EIO;
    }

    /* Extract force output samples and save them in an internal buffer */
    if (!SaveOutputData(g_cWriteBuffer, count))
    {
        DbgOutErr(("tspdrv: SaveOutputData failed.\n"));
        return -EIO;
    }

    /* Start the timer after receiving new output force */
    g_bIsPlaying = true;

    VibeOSKernelLinuxStartTimer();

    return count;
}

#if HAVE_UNLOCKED_IOCTL
static long unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
/* FUJITSU LIMITED:2016-01-20 ADD-S */
#if HAVE_UNLOCKED_IOCTL
    long    ret = 0,value = 0;
#else
    int    ret = 0,value = 0;
#endif
    char nMcLevel;

    DbgOutInfo(("tspdrv: ioctl cmd=0x%x, arg=%ld.\n",cmd, arg));
/* FUJITSU LIMITED:2016-01-20 ADD-E */

    switch (cmd)
    {
        case TSPDRV_SET_MAGIC_NUMBER:
            file->private_data = (void*)TSPDRV_MAGIC_NUMBER;
            break;

        case TSPDRV_ENABLE_AMP:
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//            ImmVibeSPI_ForceOut_AmpEnable(arg);
            if(makercmd_mode == WKUP_VIB_MODE_MC) {
                if (0 != copy_from_user((void *)&nMcLevel, (const void __user *)arg, sizeof(char))) {
                    /* Error copying the data */
                    DbgOutInfo(("copy_from_user failed to copy MC level data.\n"));
                    mc_mode_AmpEnable((VibeUInt8)64, 0);
                } else {
                    mc_mode_AmpEnable((VibeUInt8)nMcLevel, 0);
                }
            }else{
                ImmVibeSPI_ForceOut_AmpEnable((VibeUInt8)arg);
            }
/* FUJITSU LIMITED:2016-01-20 MOD-E */
#ifdef VIBE_RUNTIME_RECORD
            if (atomic_read(&g_bRuntimeRecord)) {
                DbgRecord((arg,";------- TSPDRV_ENABLE_AMP ---------\n"));
            }
#else
            DbgRecorderReset((arg));
            DbgRecord((arg,";------- TSPDRV_ENABLE_AMP ---------\n"));
#endif
            break;

        case TSPDRV_DISABLE_AMP:
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//            ImmVibeSPI_ForceOut_AmpDisable(arg);
            ImmVibeSPI_ForceOut_AmpDisable((VibeUInt8)arg);
/* FUJITSU LIMITED:2016-01-20 MOD-E */
#ifdef VIBE_RUNTIME_RECORD
            if (atomic_read(&g_bRuntimeRecord)) {
                DbgRecord((arg,";------- TSPDRV_DISABLE_AMP ---------\n"));
            }
#endif
            break;

        case TSPDRV_GET_NUM_ACTUATORS:
            return NUM_ACTUATORS;

#ifdef IMMVIBESPI_MULTIPARAM_SUPPORT
        case TSPDRV_GET_PARAM_FILE_ID:
            return ImmVibeSPI_Device_GetParamFileId();
#endif

        case TSPDRV_SET_DBG_LEVEL:
            {
                long nDbgLevel;
                if (0 != copy_from_user((void *)&nDbgLevel, (const void __user *)arg, sizeof(long))) {
                    /* Error copying the data */
                    DbgOutErr(("copy_from_user failed to copy debug level data.\n"));
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//                    return -1;
                    return -EINVAL;
/* FUJITSU LIMITED:2016-01-20 MOD-E */
                }
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//                if (DBL_TEMP <= nDbgLevel &&  nDbgLevel <= DBL_OVERKILL) {
//                    atomic_set(&g_nDebugLevel, nDbgLevel);
                if (DBL_TEMP <= (int)nDbgLevel &&  (int)nDbgLevel <= DBL_MAX) {
                    atomic_set(&g_nDebugLevel, (int)nDbgLevel);
/* FUJITSU LIMITED:2016-01-20 MOD-E */
                } else {
                    DbgOutErr(("Invalid debug level requested, ignored."));
                }

                break;
            }

        case TSPDRV_GET_DBG_LEVEL:
            return atomic_read(&g_nDebugLevel);

#ifdef VIBE_RUNTIME_RECORD
        case TSPDRV_SET_RUNTIME_RECORD_FLAG:
            {
                long nRecordFlag;
                if (0 != copy_from_user((void *)&nRecordFlag, (const void __user *)arg, sizeof(long))) {
                    /* Error copying the data */
                    DbgOutErr(("copy_from_user failed to copy runtime record flag.\n"));
                    return -1;
                }

                atomic_set(&g_bRuntimeRecord, nRecordFlag);
                if (nRecordFlag) {
                    int i;
                    for (i=0; i<NUM_ACTUATORS; i++) { 
                        DbgRecorderReset((i));
                    }
                }
                break;
            }
        case TSPDRV_GET_RUNTIME_RECORD_FLAG:
            return atomic_read(&g_bRuntimeRecord);
        case TSPDRV_SET_RUNTIME_RECORD_BUF_SIZE:
            {
                long nRecorderBufSize;
                if (0 != copy_from_user((void *)&nRecorderBufSize, (const void __user *)arg, sizeof(long))) {
                    /* Error copying the data */
                    DbgOutErr(("copy_from_user failed to copy recorder buffer size.\n"));
                    return -1;
                }

                if (0 == DbgSetRecordBufferSize(nRecorderBufSize)) {
                    DbgOutErr(("DbgSetRecordBufferSize failed.\n"));
                    return -1;
                }
                break;
            }
        case TSPDRV_GET_RUNTIME_RECORD_BUF_SIZE:
            return DbgGetRecordBufferSize();
#endif

        case TSPDRV_SET_DEVICE_PARAMETER:
            {
                device_parameter deviceParam;

                if (0 != copy_from_user((void *)&deviceParam, (const void __user *)arg, sizeof(deviceParam)))
                {
                    /* Error copying the data */
                    DbgOutErr(("tspdrv: copy_from_user failed to copy kernel parameter data.\n"));
                    return -1;
                }

                switch (deviceParam.nDeviceParamID)
                {
                    case VIBE_KP_CFG_UPDATE_RATE_MS:
                        /* Update the timer period */
                        g_nTimerPeriodMs = deviceParam.nDeviceParamValue;



#ifdef CONFIG_HIGH_RES_TIMERS
                        /* For devices using high resolution timer we need to update the ktime period value */
                        g_ktTimerPeriod = ktime_set(0, g_nTimerPeriodMs * 1000000);
#endif
                        break;

                    case VIBE_KP_CFG_FREQUENCY_PARAM1:
                    case VIBE_KP_CFG_FREQUENCY_PARAM2:
                    case VIBE_KP_CFG_FREQUENCY_PARAM3:
                    case VIBE_KP_CFG_FREQUENCY_PARAM4:
                    case VIBE_KP_CFG_FREQUENCY_PARAM5:
                    case VIBE_KP_CFG_FREQUENCY_PARAM6:
                        if (0 > ImmVibeSPI_ForceOut_SetFrequency(deviceParam.nDeviceIndex, deviceParam.nDeviceParamID, deviceParam.nDeviceParamValue))
                        {
                            DbgOutErr(("tspdrv: cannot set device frequency parameter.\n"));
                            return -1;
                        }
                        break;
                }
                break;    /* FUJITSU LIMITED:2016-01-20 ADD */
            }
/* FUJITSU LIMITED:2016-01-20 ADD-S */
        case TSPDRV_GET_AUTOTUNE:
            {
                if(makercmd_mode == WKUP_VIB_MODE_MC)
                {
                    if (0 != copy_from_user((void *)&nMcLevel, (const void __user *)arg, sizeof(char))) {
                        /* Error copying the data */
                        DbgOutInfo(("copy_from_user failed to copy MC level data.\n"));
                        nMcLevel = (char)64;
                    }
                    ret = mc_mode_AmpEnable((VibeUInt8)nMcLevel, 1);
                    if(ret < 0) {
                        DbgOutErr(("tspdrv: ERROR mc_mode_AmpEnable ret(%d).\n", ret));
                        break;
                    }
                    msleep(1000);
                    ret = mc_mode_GetAutoTune((VibeUInt8)nMcLevel, &value);
                    if(ret < 0) {
                        DbgOutErr(("tspdrv: ERROR mc_mode_GetAutoTune ret(%d).\n", ret));
                    }
                    ImmVibeSPI_ForceOut_AmpDisable((VibeUInt8)arg);
                    if(ret > 0) {
                        ret = set_NvFreq();
                        if(ret) {
                            DbgOutErr(("tspdrv: ERROR set_NvFreq ret(%d).\n", ret));
                        } else {
                            ret = value;
                        }
                    } else if(ret == -ERANGE) {
                            ret = value;
                    }
                    break;
                } else {
                    DbgOutErr(("tspdrv: ERROR GET_AUTOTUNE is MC mode only.\n"));
                    ret = -EBADRQC;
                }
            }
        default:
            DbgOutErr(("tspdrv: default cmd(%d).\n", cmd));
            return -EINVAL;
/* FUJITSU LIMITED:2016-01-20 ADD-E */
        }
/* FUJITSU LIMITED:2016-01-20 MOD-S */
//    return 0;
    DbgOutInfo(("tspdrv: ioctl ret=0x%x.\n",ret));
    return ret;
/* FUJITSU LIMITED:2016-01-20 MOD-E */
}

/* FUJITSU LIMITED:2016-01-20 ADD-S */
#ifdef CONFIG_COMPAT
static long compat_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    long ret;

    DbgOutInfo(("[%s(),line(%d)] IN cmd=0x%x arg=%ld\n",__func__, __LINE__, cmd, arg));

    arg = (unsigned long)compat_ptr(arg);
    ret = unlocked_ioctl(filep, cmd, arg);

    DbgOutInfo(("[%s(),line(%d)] OUT ret(%ld)\n",__func__, __LINE__, ret));

    return ret;
}
#endif    /* CONFIG_COMPAT */
/* FUJITSU LIMITED:2016-01-20 ADD-E */

static int suspend(struct platform_device *pdev, pm_message_t state) 
{
    if (g_bIsPlaying)
    {
        DbgOutInfo(("tspdrv: can't suspend, still playing effects.\n"));
        return -EBUSY;
    }
    else
    {
        DbgOutInfo(("tspdrv: suspend.\n"));
        cancel_work_sync(&g_pTheClient->work);
        hrtimer_cancel(&g_pTheClient->hap_timer);
        return 0;
    }
}

static int resume(struct platform_device *pdev) 
{	
    DbgOutErr(("tspdrv: resume.\n"));

	return 0;   /* can resume */
}

static void platform_release(struct device *dev) 
{	
    DbgOutErr(("tspdrv: platform_release.\n"));
}

module_init(tspdrv_init);
module_exit(tspdrv_exit);
