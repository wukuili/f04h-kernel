/*
  * Copyright(C) 2013 FUJITSU LIMITED
  * Copyright(C) 2016 Fujitsu Connected Technologies Limited
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
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/compass.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include "hscdtd008.h"
#include <linux/nonvolatile_common.h>

const char CMPS_DEVNAME[] = "HSCD";
uint8_t compass_debug_mask;

static const char COMPASS_CLASS_NAME[] = "compass";
#define COMPASS_DRIVER_NAME COMPASS_CLASS_NAME

//// SW //////////////////////////////////////////////////////

static int cmps_sw_init(bool first)
{
	int ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);

	// Set the default collation and retry
	cmps->max_retry_count = 10;

	ret = cmps_hw_init(first);
	if (ret < 0) {
		DBG_LOG_COMPASS_ERR( "(%s %d)\n", __FUNCTION__, __LINE__);
	}

	// *cmps Reset.
	CMPS_SETSTATE(CMPS_STAT_IDLE);	// Idle state

	// past measuring data is invalidated.
	cmps->reg.temp = 0;
	cmps->reg.data.x = 0;
	cmps->reg.data.y = 0;
	cmps->reg.data.z = 0;

	spin_lock_init(&cmps_spin_lock);
	memset(&pre_meas, 0, sizeof(compass_data_t));

	DBG_LOG_COMPASS_INFO("%s() ok ret:[%d]\n",__func__, ret);
	return ret;
}

/*====================== Interrupt Processing=======================*/
/*--------------------------------------------------------------------
Function  :interrupt handler of CMPS.
@param    :(1) irq:IRQ Number.
            (2) data:request_irq() ,Device-specific data (not used).
output    :cmps->measurement
            (1) time:End time measurement
@retval   :CMPS Interrupt: IRQ_HANDLED,other: IRQ_NONE.
--------------------------------------------------------------------*/
irqreturn_t cmps_sw_isr(int irq, void *data)
{
	compass_data_t *const m = &cmps->measurement;

	DBG_LOG_COMPASS_INFO("%s() irq:%d, data:%p\n", __func__, irq, data);

	// If GPIO has not been generated, disregards.
	if (unlikely(!GPIO_ISACTIVE())){

		DBG_LOG_COMPASS_INFO("%s() end IRQ_NONE irq:%d, data:%p \n", __func__, irq, data);

		return IRQ_NONE;
	}
	CMPS_SETSTATE(CMPS_STAT_MEASURED);	// to measurement completion state.

	// Recording Time.
	do_gettimeofday(&m->time);

	// The task is restarted.
	wake_up_all(&cmps->measure_end_queue);

	DBG_LOG_COMPASS_INFO("%s() ok IRQ_HADLED irq:%d, data:%p \n", __func__, irq, data);

	return IRQ_HANDLED;
}

/*--------------------------------------------------------------------
Function  :Command Conflict check.
			now sts is  CMPS_NONE -> set sts & OK Ret
			now sts not CMPS_NONE -> Recognize Conflict & ERR Ret
			not "CMPS_NONE" ->
@param    :sts:Command sts
@retval   :Success:0,Error: -errno.
error     :(1) EAGAIN:Command Conflict
--------------------------------------------------------------------*/
int cmps_sw_module_stscheck(const uint8_t sts)
{
	int ret = 0;
	unsigned long flags;

	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
	spin_lock_irqsave(&cmps_spin_lock, flags);

	if (cmps_sts == CMPS_NONE) {
		cmps_sts = sts;
	} else {
		DBG_LOG_COMPASS_ERR("cmps_sw_module_stscheck ERR -EAGAIN  sts = %x\n",cmps_sts);
		ret = -EAGAIN;
	}

	spin_unlock_irqrestore(&cmps_spin_lock, flags);
	DBG_LOG_COMPASS_INFO("%s() ok ret:[%d]\n",__func__, ret);
	return ret;
}

/*--------------------------------------------------------------------
Function  :Command Status Change.
@param    :sts:set Command sts
--------------------------------------------------------------------*/
int cmps_sw_module_stschg(const uint8_t sts)
{
	unsigned long flags;
	int ret = 0;

	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
	spin_lock_irqsave(&cmps_spin_lock, flags);

	cmps_sts = sts;

	spin_unlock_irqrestore(&cmps_spin_lock, flags);
	DBG_LOG_COMPASS_INFO("%s() ok \n", __func__);

	return ret;
}
/*========================= File Operations ========================*/
/*--------------------------------------------------------------------
Function  :CMPS device is open
@param    :(1) inode:inode Information of Device file (Unused).
           (2) file:close file Structure.
@retval   :(1) 0:success.
           (2) -EINTR:Interrupt occurs during execution.
--------------------------------------------------------------------*/
static int cmps_sw_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	uint8_t mode;

	DBG_LOG_COMPASS_INFO("%s()\n", __func__);

	mode = HSCD_MODE_FORCESTATE;
	cmps->use_count = 0;

	DBG_LOG_COMPASS_DETAIL("cmps.use_count = %d\n", cmps->use_count);
	DBG_LOG_COMPASS_DETAIL("cmps.error_count = %d\n", cmps->error_count);
	DBG_LOG_COMPASS_DETAIL("cmps.retry_count = %d\n", cmps->retry_count);
	DBG_LOG_COMPASS_DETAIL("cmps.total_error_count = %d\n", (int)cmps->total_error_count);
	DBG_LOG_COMPASS_DETAIL("cmps.total_retry_count = %d\n", (int)cmps->total_retry_count);
	DBG_LOG_COMPASS_DETAIL("cmps.compass_gpio = %d\n", cmps->compass_gpio);

	// mode is (O_RDONLY) excep EACCES.
	if (file->f_mode & FMODE_WRITE) {
		ret = -EACCES;
		DBG_LOG_COMPASS_ERR("%s: Write operation not allowed.\n", CMPS_DEVNAME);
		DBG_LOG_COMPASS_ERR("%s() end  mode:%d ret=%d\n", __func__, mode , ret);
		return ret;
	}

	ret = cmps_sw_module_stscheck(CMPS_OPEN);
	if (ret == 0) {
		if (cmps->use_count == 0) {
			ret = cmps_sw_init(false);
			if(ret < 0){
				DBG_LOG_COMPASS_ERR( "cmps_sw_init error ret=%d\n", ret);
				return ret;
			}
			ret = cmps_hw_open();
			if(ret < 0){
				DBG_LOG_COMPASS_ERR( "cmps_hw_open error ret=%d\n", ret);
				return ret;
			}
			// Handler registration
			ret = request_irq(CMPS_IRQ(), cmps_sw_isr, IRQ_TYPE_EDGE_RISING,
							CMPS_DEVNAME, &cmps);
			if (unlikely(ret < 0)) {
				DBG_LOG_COMPASS_ERR("%s: Can't use IRQ%u.\n", CMPS_DEVNAME, CMPS_IRQ());
				DBG_LOG_COMPASS_ERR("%s() error  ret=%d\n", __func__, ret);
				cmps_sw_module_stschg(CMPS_NONE);
				return ret;
			}
			disable_irq(CMPS_IRQ());
		}
		DBG_LOG_COMPASS_INFO("%s() end ret:[%d]\n",__func__, ret);
		cmps_sw_module_stschg(CMPS_NONE);
		cmps->use_count++;
	}

	// changes to an active mode
	ret = cmps_hw_write_verify_regs(HSCD_CNTL1, &mode, 1, HSCD_MODE_FORCESTATE);

	if (ret < 0) {
		cmps_sw_module_stschg(CMPS_NONE);
		DBG_LOG_COMPASS_ERR("cmps_hw_write_verify_regs error ret=%d\n", ret);
		return ret;
	}
	else {
			ret = 0;
	}
	DBG_LOG_COMPASS_INFO("%s() ok mode:%d ret=%d\n", __func__, mode , ret);
	return ret;
}

/*--------------------------------------------------------------------
Function  :CMPS device is release
@param    :(1) inode:inode Information of Device file (Unused).
           (2) file:release file Structure.
@retval   :(1) 0:success.
           (2) -EINTR:Interrupt occurs during execution.
--------------------------------------------------------------------*/
static int cmps_sw_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
	//  Exclusive control start
	ret = cmps_sw_module_stscheck(CMPS_RELEASE);
	if (ret == 0) {
		//  Exclusive control start
		if (--cmps->use_count == 0) {
			// When becoming unused
			// GPIO Interruption prohibition,IRQ Liberating.
			disable_irq(CMPS_IRQ());
			free_irq(CMPS_IRQ(), &cmps);
			ret = cmps_hw_release();
		}
		if (cmps->use_count < 0) {
			cmps->use_count = 0;
		}
		cmps_sw_module_stschg(CMPS_NONE);
	}
	DBG_LOG_COMPASS_INFO("%s() ok ret:[%d]\n",__func__, ret);
	return ret;
}
/*--------------------------------------------------------------------
Function  :Clear interrupt flag CMPS.
output    :cmps->reg.data
@retval   :Success:OK,Error: -errno.
error     :(1) error number:I/O error.
           (2) other:cmps_hw_read_regs()
--------------------------------------------------------------------*/
int cmps_sw_clear_interrupt(void)
{
	bool error = false;
	unsigned retry_count = 0;
	int ret = OK;
	unsigned nRegs;
	uint8_t buffer[CMPS_MAX_CONTIGUOUS_REGS];

	DBG_LOG_COMPASS_INFO("%s()\n", __func__);

	if (likely(GPIO_ISACTIVE())) {

		DBG_LOG_COMPASS_DETAIL("%s gpio active\n", __func__);

		for (;;) {
			// When the GPIO signal is High:
			// Clear interrupt.
			nRegs = sizeof(cmps->reg.data);
			if ((ret = cmps_hw_read_regs(HSCD_XLSB, buffer, nRegs)) >= 0) {
				cmps_hw_read_verify_regs(HSCD_STAT, &cmps->reg.stat, 1);
				DBG_LOG_COMPASS_DETAIL("stat=%x.\n", cmps->reg.stat);
				// check GPIO is Low .
				if (likely(!GPIO_ISACTIVE())){
					DBG_LOG_COMPASS_INFO( "(%s %d) success \n", __FUNCTION__, __LINE__);
					break;		// Success
				}
				else{
					DBG_LOG_COMPASS_INFO( "(%s %d) error \n", __FUNCTION__, __LINE__);
				}
			}
			// At the error
			error = true;
			if (retry_count >= cmps->max_retry_count) {
				// when retries reach upper limit:error
				DBG_LOG_COMPASS_ERR( "Error: Unable to clear GPIO.\n");
				break;
			}
			// retries
			retry_count++;
		}
	}

	if (error) {
		// error/retries total.
		cmps->error_count++;
		cmps->retry_count += retry_count;
	}

	DBG_LOG_COMPASS_DETAIL("%s() error_count:%d  retry_count:%d\n"
						, __func__, cmps->error_count , retry_count);
	DBG_LOG_COMPASS_INFO("%s() ok ret=%d\n", __func__, ret);

	return ret;
}


static long cmps_sw_measure(compass_data_t __user * arg)
{
	long ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
	ret = cmps_hw_measure((compass_data_t __user *)arg);
	DBG_LOG_COMPASS_INFO("%s() ok ret:[%ld]\n",__func__, ret);
	return ret;
}

static long cmps_sw_measure_temp(compass_data_t __user * arg)
{
	long ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
	ret = cmps_hw_measure_temp((compass_data_t __user *)arg);
	DBG_LOG_COMPASS_INFO("%s() ok ret:[%ld]\n",__func__, ret);
	return ret;
}

static int cmps_sw_exit(void)
{
	int ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);

	// Set the default collation and retry
	cmps->max_retry_count = 10;

	ret = cmps_hw_exit();
	if (ret < 0) {
		DBG_LOG_COMPASS_ERR( "(%s %d)\n", __FUNCTION__, __LINE__);
	}

	// *cmps Reset.
	CMPS_SETSTATE(CMPS_STAT_IDLE);	// Idle state

	// past measuring data is invalidated.
	cmps->reg.temp = 0;
	cmps->reg.data.x = 0;
	cmps->reg.data.y = 0;
	cmps->reg.data.z = 0;

	spin_lock_init(&cmps_spin_lock);
	memset(&pre_meas, 0, sizeof(compass_data_t));

	DBG_LOG_COMPASS_INFO("%s() ok ret:[%d]\n",__func__, ret);
	return ret;
}

//// IF //////////////////////////////////////////////////////

static int cmps_if_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
//	ret = cmps_sw_open(inode, file);
	cmps_sw_open(inode, file);
	DBG_LOG_COMPASS_INFO("%s() ok ret:[%d]\n",__func__, ret);
	return ret;
}

static int cmps_if_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
	ret = cmps_sw_release(inode, file);
	DBG_LOG_COMPASS_INFO("%s() ok ret:[%d]\n",__func__, ret);
	return ret;
}

static long cmps_if_ioctl(struct file *file,
			   unsigned int command, unsigned long arg)
{
	long ret;

	DBG_LOG_COMPASS_INFO("%s() command:0x%08X  arg:0x%08lX)\n", __func__, command, arg);

	switch (command) {

	case COMPASS_IOC_GETDATA:
		ret = cmps_sw_measure((compass_data_t __user *)arg);
		break;

	case COMPASS_IOC_TEMP_COR:
		ret = cmps_sw_measure_temp((compass_data_t __user *)arg);
		break;

	default:
		ret = -EINVAL;		// Undefined command number
		DBG_LOG_COMPASS_ERR( "(%s %d)\n", __FUNCTION__, __LINE__);
		break;
	}

	DBG_LOG_COMPASS_INFO("%s() ok command:0x%08X  arg:0x%08lX ret=%ld\n"
							, __func__, command, arg , ret);

	return ret;

}

#ifdef CONFIG_COMPAT
static long cmps_compat_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
        DBG_LOG_COMPASS_INFO("[compass] %s()\n",__func__);
        arg = (unsigned long)compat_ptr(arg);
        return cmps_if_ioctl(filep, cmd, arg);
}
#endif

static int cmps_if_init(bool first)
{
	int ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
	ret = cmps_sw_init(first);
	DBG_LOG_COMPASS_INFO("%s() ok ret:[%d]\n",__func__, ret);
	return ret;
}

static int cmps_if_exit(void)
{
	int ret = 0;
	DBG_LOG_COMPASS_INFO("%s()\n", __func__);
	ret = cmps_sw_exit();
	DBG_LOG_COMPASS_INFO("%s() ok ret:[%d]\n",__func__, ret);
	return ret;
}

static dev_t compass_devno = COMPASS_DEVNO;
static struct class *compass_class = NULL;
static struct cdev compass_cdev;

static struct file_operations compass_fops = {
	.open		= cmps_if_open,
	.release	= cmps_if_release,
	.unlocked_ioctl		= cmps_if_ioctl,
#ifdef CONFIG_COMPAT
        .compat_ioctl = cmps_compat_ioctl,
#endif
};

static const struct i2c_device_id compass_id[] = {
		{"compass", 0},
		{ },
};


/*--------------------------------------------------------------------*
 * Function:When the device starts,
 *           character device  register and Initialization of hardware*
 *--------------------------------------------------------------------*/
static int compass_probe
(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev;
	int result;

	DBG_LOG_COMPASS_INFO( "%s()\n", __func__);

	DBG_LOG_COMPASS_DETAIL( "compass_probe() : %s: module loaded\n", COMPASS_CLASS_NAME);

	cmps->reg.st_client = client;	// Client information is secured.

	// majors device number is automatically allocated
	// register a range of minor device numbers
	result = alloc_chrdev_region (&compass_devno, COMPASS_DEVNO_MINOR,
			COMPASS_NDEVICES, COMPASS_DRIVER_NAME);
	if (unlikely(result < 0)) {
		DBG_LOG_COMPASS_ERR( "compass_probe() : alloc_chrdev_region() failed (%d).\n",result);
		goto Exit;
	}

	DBG_LOG_COMPASS_INFO( "compass_probe() : %s device number: %u.%u\n",
		COMPASS_CLASS_NAME, MAJOR(compass_devno), MINOR(compass_devno));

	// compass_cdev of character device to initialize and register the system
	cdev_init(&compass_cdev, &compass_fops);
	DBG_LOG_COMPASS_INFO( "cdev_init() OK \n");
	compass_cdev.owner = THIS_MODULE;
	result = cdev_add(&compass_cdev, compass_devno, COMPASS_NDEVICES);
	if (unlikely(result < 0)) {
		DBG_LOG_COMPASS_ERR( "compass_probe() : cdev_add() failed.\n");
		goto Error1;
	}

	// create a device class
	compass_class = class_create(THIS_MODULE, COMPASS_CLASS_NAME);
	if (IS_ERR(compass_class)) {
		result = PTR_ERR(compass_class);	// result <- -errno
		DBG_LOG_COMPASS_ERR( "compass_probe() : class_create() failed. (%d)\n", result);
		goto Error2;
	}

	// Create a class devices, sysfs to register.
	// /sys/bus/platform/drivers/compass/
	dev = device_create (compass_class, NULL, compass_devno,
		cmps, "%s", COMPASS_DRIVER_NAME);
	if (unlikely(IS_ERR (dev))) {
		result = PTR_ERR(dev);	// result <- -errno
		DBG_LOG_COMPASS_ERR( "compass_probe() : device_create() failed (%d).\n", result);
		goto Error3;
	}

	cmps->compass_pinctrl.pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(cmps->compass_pinctrl.pinctrl)) {
		result = PTR_ERR(cmps->compass_pinctrl.pinctrl);
		DBG_LOG_COMPASS_ERR( "can not get pinctrl(%d).\n", result);
		goto Error3;
	}
	cmps->compass_pinctrl.gpio_state_irq = pinctrl_lookup_state(cmps->compass_pinctrl.pinctrl,"compass_irq_active");
	if (IS_ERR_OR_NULL(cmps->compass_pinctrl.gpio_state_irq)) {
		result = PTR_ERR(cmps->compass_pinctrl.gpio_state_irq);
		DBG_LOG_COMPASS_ERR( "can not get irq pinstate(%d).\n", result);
		goto Error3;
	}

	cmps->compass_gpio = cmps_hw_get_gpio(DT_CMPS_GPIO_IRQ);
	if ( cmps->compass_gpio < 0 ) {
		DBG_LOG_COMPASS_ERR( "(%s %d) cmps_hw_get_gpio() fail.\n", __FUNCTION__, __LINE__);
		goto Error3;
	}

	result = cmps_if_init(true);		// CMPS module initialization
	DBG_LOG_COMPASS_INFO( "%s() ok ret=%d\n", __FUNCTION__, result);
	return result;

Exit:
	DBG_LOG_COMPASS_INFO( "%s() ok ret=%d\n", __FUNCTION__, result);
	return result;

Error3:
	class_destroy(compass_class);
	DBG_LOG_COMPASS_ERR( "%s() error3 ret=%d\n", __FUNCTION__, result);
Error2:
	compass_class = NULL;
	cdev_del(&compass_cdev);
	DBG_LOG_COMPASS_ERR( "%s() error2 ret=%d\n", __FUNCTION__, result);
Error1:
	unregister_chrdev_region(compass_devno, COMPASS_NDEVICES);
	DBG_LOG_COMPASS_ERR( "%s() error1 ret=%d\n", __FUNCTION__, result);
	goto Exit;
}

/*--------------------------------------------------------------------*
 * Function:When the device ends,
 *           hardware is stopped and character device is deleted.     *
 *--------------------------------------------------------------------*/
static int compass_remove(struct i2c_client *client)
{
	int ret = 0;
	DBG_LOG_COMPASS_INFO( "%s()\n", __FUNCTION__);

	ret = cmps_if_exit();		// CMPS module release

	device_destroy(compass_class, compass_devno);
	if (compass_class != NULL) {
		DBG_LOG_COMPASS_ERR( "%s() class_destroy\n", __FUNCTION__);
		class_destroy(compass_class);
		compass_class = NULL;
	}
	cdev_del(&compass_cdev);
	unregister_chrdev_region(compass_devno, COMPASS_NDEVICES);

	if (ret < 0) {
		DBG_LOG_COMPASS_ERR( "(%s %d)\n", __FUNCTION__, __LINE__);
	}
	else{
		DBG_LOG_COMPASS_INFO( "%s() ok \n", __FUNCTION__);
	}
	return ret;
}

static struct i2c_driver compass_driver = {


	.probe = compass_probe,
	.remove = compass_remove,
	.id_table = compass_id,
	.driver = {
		.name = COMPASS_DRIVER_NAME,
	},
};

/*--------------------------------------------------------------------*
 * Function:When the device starts,                                   *
 *           magnetism sensor driver is registered in the kernel.     *
 *--------------------------------------------------------------------*/
static int compass_init(void)
{
	int res;
	unsigned char compass_getNvData[APNV_SIZE_COMPASS_DBG_MODE];

	memset(compass_getNvData, 0x00, sizeof(compass_getNvData));
	res = get_nonvolatile(compass_getNvData, APNV_COMPASS_DBG_MODE, APNV_SIZE_COMPASS_DBG_MODE);
	if (unlikely(res < 0)) {
		printk(KERN_INFO "%s: get_nonvolatile() [ret=%d]\n", __func__, res);
		compass_getNvData[APNV_ADR_COMPASS_DBG_MODE] = 0x00;
	}
	compass_debug_mask = compass_getNvData[APNV_ADR_COMPASS_DBG_MODE];
	
	DBG_LOG_COMPASS_INFO( "%s()\n", __FUNCTION__);

	DBG_LOG_COMPASS_DETAIL( "Initializing %s module\n", COMPASS_CLASS_NAME);

	res = i2c_add_driver(&compass_driver);

	if (res < 0) {
		DBG_LOG_COMPASS_ERR( "(%s %d)\n", __FUNCTION__, __LINE__);
	}

	DBG_LOG_COMPASS_INFO( "%s() ok ret=%d\n", __FUNCTION__ , res );

	return res;
}

/*--------------------------------------------------------------------*
 * Function:When the device ends,                                     *
 *           magnetism sensor driver is deleted from the kernel.      *
 *--------------------------------------------------------------------*/
static void __exit compass_exit(void)
{
//	int res;

	DBG_LOG_COMPASS_INFO( "%s()\n", __FUNCTION__);

	DBG_LOG_COMPASS_DETAIL( "Exiting %s module\n", COMPASS_CLASS_NAME);

	i2c_del_driver(&compass_driver);

	DBG_LOG_COMPASS_INFO( "%s() ok \n", __FUNCTION__);
}

module_init (compass_init);
module_exit (compass_exit);

MODULE_AUTHOR ("FUJITSU LIMITED");
MODULE_DESCRIPTION ("compass driver");
MODULE_LICENSE ("GPL");
