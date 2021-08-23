/*
  * Copyright(C) 2013-2015 FUJITSU LIMITED
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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/timer.h> 
#include <linux/switch.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/of_gpio.h>
#include <linux/fj_mode.h>
#include <linux/mutex.h>

#include <linux/earphone.h>

#include <linux/irq.h> /* Earphone interupt enable */
#include <linux/nonvolatile_common.h> /* nv */

#include "edc.h"
#include "edc_hw.h"
#include "edc_hw_type.h"

#ifdef FJ_EDC_USB_EARPHONE
#include <linux/fj_det.h>
#endif /* FJ_EDC_USB_EARPHONE */

#include <linux/interrupt.h>
#include <soc/qcom/smem.h>

#ifdef FJ_EDC_USB_EARPHONE
extern int edc_ovp_usbearphone_path_set( unsigned int cmd );
#endif /* FJ_EDC_USB_EARPHONE */

extern int headset_micbias_ctrl(int on);
static inline int edc_hw_micbias_ctrl_on(const char *func)
{
	if(0 != headset_micbias_ctrl(1))
	{
		printk("[EDC] ERROR(%s): headset_micbias_ctrl(1) NG \n", func);
		return EDC_NG;
	}
	return EDC_OK;
}
static inline int edc_hw_micbias_ctrl_off(const char *func)
{
	if(0 != headset_micbias_ctrl(0))
	{
		printk("[EDC] ERROR(%s): headset_micbias_ctrl(0) NG \n", func);
		return EDC_NG;
	}
	return EDC_OK;
}
#define MICBIAS_CTRL_ON()	edc_hw_micbias_ctrl_on(__func__)
#define MICBIAS_CTRL_OFF()	edc_hw_micbias_ctrl_off(__func__)

static int edc_chatt_cnt = 0;
static int earphone_kind = EARPHONE_NOT;

static int pierce_id_1000_ohm_low  = 0;
static int pierce_id_1000_ohm_high = 0;
static int pierce_id_1500_ohm_low  = 0;
static int pierce_id_1500_ohm_high = 0;
static int pierce_id_2200_ohm_low  = 0;
static int pierce_id_2200_ohm_high = 0;

static int adc_threshold_headset_low  = 0;
static int adc_threshold_headset_high = 0;
static int adc_threshold_pierce_low   = 0;
static int adc_threshold_pierce_high  = 0;
static int adc_threshold_mic_low      = 0;
static int adc_threshold_mic_high     = 0;
static int adc_threshold_antenna_low  = 0;
static int adc_threshold_antenna_high = 0;

/* earphone detect skip add start */
/* extern unsigned int mdm_powoff_get(void); */

static int edc_state_num = EDC_STATE_INIT;

int edc_state_matrix[EDC_STATE_MAX][EDC_STATE_MAX]={
{ 0,-1,-1,-1,-1,-1,-1,-1,-1,-1},  /* INT      : S0 */
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* LOOP     : S1 */
{-1, 0, 0,-1,-1,-1, 0,-1,-1,-1},  /* CHATT    : S2 */
{-1,-1, 0,-1,-1,-1, 0,-1,-1,-1},  /* DESITION : S3 */
{-1, 0,-1,-1, 0,-1, 0, 0, 0, 0},  /* ISR      : S4 */
{-1, 0,-1,-1, 0,-1, 0, 0, 0, 0},  /* PLUG     : S5 */
{-1, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* SUSPEND  : S6 */
{-1, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* RESUME   : S7 */
{-1, 0,-1, 0,-1,-1, 0, 0, 0, 0},  /* SWICHISR : S8 */
{-1, 0,-1,-1,-1,-1, 0, 0, 0, 0}}; /* SWICHWQ  : S9 */

int  edc_state_check( int cu_num , int be_num );
void edc_state_NG( int cu_num , int be_num );

static unsigned int   *oem_earphone_control_ptr=NULL;

static int gpio_num_xheadset_det;
static int gpio_num_xheadset_vdd_det;
static int gpio_num_antenna_det;
static int gpio_num_xhscall_det;
#ifdef FJ_EDC_PIERCE_GPIO
static int gpio_num_answ_fet_on;
static int gpio_num_sel_jear_led;
#endif /* FJ_EDC_PIERCE_GPIO */

/**********************************************************************/
/* globals															  */
/**********************************************************************/
extern int edc_interrupt_flag;

/* bit0: Headset  bit1:USB Earphone */
extern int edc_Headset_Definition_Flag;
extern edc_handle_t edc_port[EDC_NUM_OF_COUNT];
extern struct wake_lock edc_wlock_gpio;			/* suspend control */
extern struct wake_lock edc_wlock_sus_cancel;
extern struct switch_dev edc_headset_dev;
extern struct switch_dev edc_antenna_dev;
extern struct switch_dev edc_pierce_dev;
#if OPT_PIERCE_SWITCH
extern struct switch_dev edc_pierce1_dev;
extern struct switch_dev edc_pierce2_dev;
extern struct switch_dev edc_pierce3_dev;
#endif
extern struct input_dev *edc_ipdev;

#if OPT_USB_EARPHONE
extern int edc_UsbEarphoneFlag;			/* 0:USB earphone off 1:USB earphone on */
#endif
extern int edc_FaiEarphoneFlag;			/* 0:3.5 earphone off 1:3.5 earphone off */
extern int edc_Hedset_Switch_enable_Flag;	/* 0:with no switch  1:with switch */
extern int edc_mic_bias_flag;			/* 0:OFF  1:mic bias on */
extern int edc_voice_call_flag;			/* 0:not talking  1:talking */
extern int edc_inout_timer_expire;		/* 0:no timer 1:timer 2:switch timer */
extern int edc_model_type;
extern int edc_base_portstatus;
extern int edc_headset_mic_use_flag;

extern int edc_pierce_retry_flg;

int plug_loop_call=0;
int test_cnt=0;
int edc_switch_irq_flg = 0;
int edc_irq_wake_guard = 0;
int edc_loop_switch_cnt=0; /* > DETECT_SWITCH_NUM ? */

extern int fj_boot_mode;

extern struct qpnp_vadc_chip *edc_vadc;

static DEFINE_MUTEX(edc_plug_lock);

static DECLARE_DELAYED_WORK(work_que_loop, edc_hw_gpio_plug_loop);
static DECLARE_DELAYED_WORK(work_que_chatt, edc_hw_gpio_plug_chatt);
static DECLARE_DELAYED_WORK(work_que_decision, edc_hw_gpio_plug_decision);
static DECLARE_DELAYED_WORK(work_que_plug, edc_hw_handler_plug);
static DECLARE_DELAYED_WORK(work_que_switch, edc_hw_handler_switch);
extern struct workqueue_struct *edc_plug_wq;

/**********************************************************************/
/* edc_hw_get_gpio													  */
/**********************************************************************/
static int edc_hw_get_gpio(int id)
{
    struct device_node *node;
    enum of_gpio_flags flag;
    int gpio_num;
    
    node = of_find_compatible_node(NULL, NULL, "fj-edc");
    gpio_num = of_get_gpio_flags(node, id, &flag);
    EDC_DBG_PRINTK("[EDC] (%s):id=%d gpio_num=%d \n",__func__,id, gpio_num);
    
    return gpio_num;
}

/**********************************************************************/
/* edc_hw_init														  */
/**********************************************************************/
int edc_hw_init(void)
{
	int ret=EDC_OK;
	int cu_num=EDC_STATE_INIT;

	EDC_DBG_PRINTK("[EDC] %s()\n",__func__);

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_state_NG( cu_num , edc_state_num );
		edc_state_num = cu_num;
		return EDC_NG;
	}
	edc_state_num = cu_num;
	/* state check ed */

	ret = edc_hw_setup_config();

	if (ret) {
		printk("[EDC] ERROR(%s):edc_hw_setup_config() (%d)\n",__func__,ret);
		return ret;
	}

	ret = edc_hw_nv_read();

	if (ret) {
		printk("[EDC] ERROR(%s):edc_hw_nv_read() (%d)\n",__func__,ret);
		return ret;
	}

	EDC_DBG_PRINTK("[EDC] %s() OK\n",__func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_exit														  */
/**********************************************************************/
int edc_hw_exit(void)
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	ret = edc_hw_delete_config();

	if (ret) {
		printk("[EDC] ERROR(%s):edc_hw_delete_config() (%d)\n",__func__,ret);
		return ret;
	}

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_suspend													  */
/**********************************************************************/
int edc_hw_suspend( void )
{
	int ret=EDC_OK, ret2=EDC_OK;
	int portstatus;
	int cu_num=EDC_STATE_SUSPEND;

	EDC_DBG_PRINTK("[EDC] %s() status=%d \n",__func__,edc_base_portstatus );

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_state_NG( cu_num , edc_state_num );
		edc_state_num = cu_num;
		return EDC_NG;
	}
	edc_state_num = cu_num;
	/* state check ed */

	cancel_delayed_work_sync(&work_que_loop);

	/*if ( edc_base_portstatus == 0 )
		edc_hw_earphone_disable_irq(edc_port[EDC_PLUG].gpio_det_irq);*/

	if ( edc_base_portstatus == 1) { /* Earphone interupt enable */
		if( edc_voice_call_flag ){
			ret = qpnp_pin_config(gpio_num_xheadset_det, &earphone_xheadset_det_out_en);
			if (ret < 0) {
				printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET qpnp_pin_config() (%d)\n",__func__,ret);
				return ret;
			}
			msleep(10);
			portstatus = gpio_get_value(gpio_num_xheadset_det);
			printk( "[EDC] status = %d\n",portstatus);
	/* notify to mdm */
			oem_earphone_control_ptr = smem_alloc_vendor0(SMEM_PACO_EARPHONE_DET_SUSPEND);
			if( oem_earphone_control_ptr == NULL ) {
				printk( "SMEM_PACO_EARPHONE_DET_SUSPEND error \n" );
			} else {
	    	    printk( "SMEM_PACO_EARPHONE_DET_SUSPEND ctrl_on \n" );
				*oem_earphone_control_ptr = 1;
			}
		} else {
			ret = qpnp_pin_config(gpio_num_xheadset_det, &earphone_xheadset_det_out_en);
			if (ret < 0) {
				printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET qpnp_pin_config() (%d)\n",__func__,ret);
				return ret;
			}
			ret = gpio_direction_output(gpio_num_xheadset_det, 0);
			if( ret < 0 ) {
				printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET gpio_direction_output() (%d)\n",__func__,ret );
				return ret;
			}
		}
	}

	if (edc_mic_bias_flag)
	{
		/*EDC_DBG_PRINTK("[EDC] YMU_MICBIAS_OFF\n");*/

		if( edc_Headset_Definition_Flag & HEDSET_CHECK_BIT ){
			if ( edc_Hedset_Switch_enable_Flag ) {
				/* Earphone interupt enable start */
				irq_set_irq_wake(edc_port[EDC_SWITCH].gpio_det_irq, EDC_WAKE_ON);
			} else {
				/*irq_set_irq_wake(edc_port[EDC_SWITCH].gpio_det_irq, EDC_WAKE_OFF);*/
				edc_hw_earphone_disable_irq(edc_port[EDC_SWITCH].gpio_det_irq);
				ret2 = MICBIAS_CTRL_OFF();
				if(ret2 != EDC_OK) {
					/* Error is ignored, and outputs only the log. */
					printk("[EDC] ERROR(%s):MICBIAS_CTRL_OFF() (%d)\n",__func__,ret2);
				}
			}
		}
		
	}

#ifdef FJ_EDC_PIERCE_GPIO
	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){

		/* GPIO-EXCOL11=L */
		gpio_set_value_cansleep( gpio_num_sel_jear_led , EDC_GPIO_LOW );
	}
#endif /* FJ_EDC_PIERCE_GPIO */
	EDC_DBG_PRINTK("[EDC] %s() OK\n",__func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_resume													  */
/**********************************************************************/
int edc_hw_resume( void )
{
	int ret=EDC_OK, ret2=EDC_OK;
	int cu_num=EDC_STATE_RESUME;

	EDC_DBG_PRINTK("[EDC] %s() status=%d \n", __func__,edc_base_portstatus);

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_state_NG( cu_num , edc_state_num );
		edc_state_num = cu_num;
		return EDC_NG;
	}
	edc_state_num = cu_num;
	/* state check ed */

	/* notify to mdm */
	if ( edc_base_portstatus == 1 ) { /* Earphone interupt enable */
		if( edc_voice_call_flag ){
			oem_earphone_control_ptr = smem_alloc_vendor0(SMEM_PACO_EARPHONE_DET_SUSPEND);
			if( oem_earphone_control_ptr == NULL ) {
				printk( "SMEM_PACO_EARPHONE_DET_SUSPEND error \n" );
			} else {
			    printk( "SMEM_PACO_EARPHONE_DET_SUSPEND ctrl_off \n" );
				*oem_earphone_control_ptr = 0;
			}
		}
	}

	if (edc_mic_bias_flag){

		if( edc_Headset_Definition_Flag & HEDSET_CHECK_BIT ){
			/* Earphone interupt enable start */
			if ( edc_Hedset_Switch_enable_Flag )
				irq_set_irq_wake(edc_port[EDC_SWITCH].gpio_det_irq, EDC_WAKE_OFF);
			else{
				/*EDC_DBG_PRINTK("[EDC] YMU_MICBIAS_ON\n");*/
				ret2 = MICBIAS_CTRL_ON();
				if(ret2 != EDC_OK) {
					/* Error is ignored, and outputs only the log. */
					printk("[EDC] ERROR(%s):MICBIAS_CTRL_ON() (%d)\n",__func__,ret2);
				}
				edc_hw_earphone_enable_irq(edc_port[EDC_SWITCH].gpio_det_irq);
			}
		}
	}

	plug_loop_call = 1;
	queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(RESUME_PHI35_LOOP));

	EDC_DBG_PRINTK("[EDC] %s() OK\n",__func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_setup_config												  */
/**********************************************************************/
int edc_hw_setup_config(void)
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	/* Get GPIO number from Device Tree */
	gpio_num_xheadset_det = edc_hw_get_gpio(DT_XHEADSET_DET);
	if ( gpio_num_xheadset_det < 0 ) {
		printk("[EDC] ERROR(%s):DT_XHEADSET_DET edc_hw_get_gpio() (%d)\n",__func__, gpio_num_xheadset_det);
		return gpio_num_xheadset_det;
	}
	
	gpio_num_xheadset_vdd_det = edc_hw_get_gpio(DT_XHEADSET_VDD_DET);
	if ( gpio_num_xheadset_vdd_det < 0 ) {
		printk("[EDC] ERROR(%s):DT_XHEADSET_VDD_DET edc_hw_get_gpio() (%d)\n",__func__, gpio_num_xheadset_vdd_det);
		return gpio_num_xheadset_vdd_det;
	}
	
	if( edc_model_type & (EDC_MODEL_TYPE_PIERCE | EDC_MODEL_TYPE_ANTENNA ) ){
		gpio_num_antenna_det = edc_hw_get_gpio(DT_EARANT_DET);
		if ( gpio_num_antenna_det < 0 ) {
			printk("[EDC] ERROR(%s):DT_EARANT_DET edc_hw_get_gpio() (%d)\n",__func__, gpio_num_antenna_det);
			return gpio_num_antenna_det;
		}
	}
	
	gpio_num_xhscall_det = edc_hw_get_gpio(DT_XHSCALL_DET);
	if ( gpio_num_xhscall_det < 0 ) {
		printk("[EDC] ERROR(%s):DT_XHSCALL_DET edc_hw_get_gpio() (%d)\n",__func__, gpio_num_xhscall_det);
		return gpio_num_xhscall_det;
	}
	
#ifdef FJ_EDC_PIERCE_GPIO
	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
		gpio_num_answ_fet_on = edc_hw_get_gpio(DT_ANSW_FET_ON);
		if ( gpio_num_answ_fet_on < 0 ) {
			printk("[EDC] ERROR(%s):DT_ANSW_FET_ON edc_hw_get_gpio() (%d)\n",__func__,gpio_num_answ_fet_on);
			return gpio_num_answ_fet_on;
		}

		gpio_num_sel_jear_led = edc_hw_get_gpio(DT_SEL_JEAR_LED);
		if ( gpio_num_sel_jear_led < 0 ) {
			printk("[EDC] ERROR(%s):DT_SEL_JEAR_LED edc_hw_get_gpio() (%d)\n",__func__,gpio_num_sel_jear_led);
			return gpio_num_sel_jear_led;
		}
	}
#endif /* FJ_EDC_PIERCE_GPIO */

	/* headset switch pulldown */
	ret = gpio_request(gpio_num_xhscall_det, "XHSCALL_DET");
	if (ret){
		printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET gpio_request() (%d)\n",__func__,ret);
		goto ON_ERR;
	}
	ret = gpio_direction_input(gpio_num_xhscall_det);
	if (ret){
		printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET gpio_direction_input() (%d)\n",__func__,ret);
		goto ON_ERR;
	}
	edc_port[EDC_SWITCH].gpio_det_irq = gpio_to_irq(gpio_num_xhscall_det);
	ret = request_irq(edc_port[EDC_SWITCH].gpio_det_irq,
					&edc_hw_irq_handler_switch, /* Thread function */
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					"XHSCALL_DET",
					NULL);
	if (ret < 0){
		printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET request_irq() (%d)\n",__func__,ret);
		goto ON_ERR;
	}
	edc_hw_earphone_disable_irq(edc_port[EDC_SWITCH].gpio_det_irq);

	ret = gpio_request(gpio_num_xheadset_det, "XHEADSET_DET");
	if (ret) {
		printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET gpio_request() (%d)\n",__func__,ret);
		goto ON_ERR;
	}

	ret = qpnp_pin_config(gpio_num_xheadset_det, &earphone_xheadset_det_out_en);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET qpnp_pin_config() (%d)\n",__func__,ret);
		goto ON_ERR;
	}
	edc_port[EDC_PLUG].gpio_det_irq = gpio_to_irq(gpio_num_xheadset_det);

	ret = request_irq(edc_port[EDC_PLUG].gpio_det_irq,
					 &edc_hw_irq_handler_inout,
					 IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
					 "XHEADSET_DET",
					 NULL);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):request_irq(XHEADSET_DET) ret=%d\n",__func__,ret);
		goto ON_ERR;
	}
	irq_set_irq_wake(edc_port[EDC_PLUG].gpio_det_irq, EDC_WAKE_ON);

	ret = qpnp_pin_config(gpio_num_xheadset_vdd_det, &earphone_deconfig);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):MPP02 qpnp_pin_config() deconfig (%d)\n",__func__,ret);
		return ret;
	}
	
	if( edc_model_type & (EDC_MODEL_TYPE_PIERCE | EDC_MODEL_TYPE_ANTENNA ) ){
		ret = qpnp_pin_config(gpio_num_antenna_det, &earphone_antenna_deconfig);
		if (ret < 0) {
			printk("[EDC] ERROR(%s):MPP04 qpnp_pin_config() deconfig (%d)\n",__func__, ret);
			return ret;
		}
	}

	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
		printk( "[EDC] GPIO_KO_JMIC_ONOFF set\n");
	}
	else{
		printk( "[EDC] GPIO_KO_JMIC_ONOFF not set\n");
	}

#ifdef FJ_EDC_PIERCE_GPIO
	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
		ret = gpio_request(gpio_num_answ_fet_on, "ANSW_FET_ON");
		if (ret){
			printk("[EDC] ERROR(%s):GPIO_ANSW_FET_ON gpio_request() (%d)\n",__func__,ret);
			goto ON_ERR;
		}
		ret = gpio_tlmm_config(GPIO_CFG(gpio_num_answ_fet_on, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
		if(ret) {
			printk("[EDC] ERROR(%s):GPIO_ANSW_FET_ON gpio_tlmm_config() (%d)\n",__func__,ret );
			goto ON_ERR;
		}
		gpio_direction_output(gpio_num_answ_fet_on,0);

		ret = gpio_request(gpio_num_sel_jear_led, "SEL_JEAR_LED");
		if (ret){
			printk("[EDC] ERROR(%s):GPIO_SEL_JEAR_LED gpio_request() (%d)\n",__func__,ret);
			goto ON_ERR;
		}
		ret = gpio_tlmm_config(GPIO_CFG(gpio_num_sel_jear_led, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if(ret) {
			printk("[EDC] ERROR(%s):GPIO_SEL_JEAR_LED gpio_tlmm_config() (%d)\n",__func__,ret );
			goto ON_ERR;
		}
		gpio_direction_output(gpio_num_sel_jear_led,0);
	}
#endif /* FJ_EDC_PIERCE_GPIO */

	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
		printk( "[EDC] GPIO_KO_JMIC_ONOFF set\n");
	}
	else{
		printk( "[EDC] GPIO_KO_JMIC_ONOFF not set\n");
	}

#ifdef FJ_EDC_PIERCE_GPIO
	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
		gpio_set_value_cansleep( gpio_num_answ_fet_on , EDC_GPIO_HIGH );
		printk( "[EDC] GPIO_ANSW_FET_ON set\n");
	}
	else{
		printk( "[EDC] GPIO_ANSW_FET_ON not set\n");
	}
#endif /* FJ_EDC_PIERCE_GPIO */

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;

ON_ERR:
	edc_hw_delete_config();
	return ret;
}

/**********************************************************************/
/* edc_hw_delete_config												  */
/**********************************************************************/
int edc_hw_delete_config(void)
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	free_irq(edc_port[EDC_PLUG].gpio_det_irq, NULL); /* Earphone interupt enable */
	/* IRQInterrupt Release */
	free_irq(edc_port[EDC_SWITCH].gpio_det_irq, NULL);

	/* setup XHEADSET_DET */
	gpio_free(gpio_num_xheadset_det);


	/* setup XHS_CALL */
	gpio_free(gpio_num_xhscall_det);

#ifdef FJ_EDC_PIERCE_GPIO
	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
		gpio_free(gpio_num_answ_fet_on);
		gpio_free(gpio_num_sel_jear_led);
	}
#endif /* FJ_EDC_PIERCE_GPIO */

	/* headset switch pulldown */
	ret = qpnp_pin_config(gpio_num_xhscall_det, &earphone_xhscall_det_disable);
	if ( ret < 0 ) {
		printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET qpnp_pin_config() (XHS_CALL, DIS) = %d\n",__func__,ret);
		return ret;
	}

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_gpio_plug_loop											  */
/**********************************************************************/
void edc_hw_gpio_plug_loop(struct work_struct *work)
{
	int ret=EDC_OK;
	int portstatus=0;
	int cu_num=EDC_STATE_LOOP;
	int base_num=edc_state_num;
	int recheck_flg=0;
	int earphone_kind_bitset=0;
	int adc_code=0;

#ifdef EDC_DEBUG_LOOP
	EDC_DBG_LOOP_PRINTK("[EDC] %s() st \n", __func__);
#else
	if( plug_loop_call == 1 ){
		printk(KERN_INFO "[EDC] %s() st \n", __func__);
	}
#endif
	mutex_lock(&edc_plug_lock);
	EDC_DBG_LOOP_PRINTK("[EDC] %s() get mutex\n", __func__);
	plug_loop_call = 0;

	/* edc_suspend_cansel_flg = 0; */
	edc_pierce_retry_flg = 0;

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_state_NG( cu_num , edc_state_num );
		edc_state_num = cu_num;
		mutex_unlock(&edc_plug_lock);
		return;
	}
	edc_state_num = cu_num;
	/* state check ed */

	if ( edc_base_portstatus == 0 ){
		edc_hw_earphone_disable_irq(edc_port[EDC_PLUG].gpio_det_irq);
	}

	ret = qpnp_pin_config(gpio_num_xheadset_det, &earphone_xheadset_det_in_en);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET qpnp_pin_config() (%d)\n",__func__,ret);
		goto ON_SKIP;
	}
	msleep(10);
	portstatus = gpio_get_value(gpio_num_xheadset_det);
	EDC_DBG_LOOP_PRINTK("[EDC] edc_base_portstatus = %d , portstatus = %d\n",edc_base_portstatus,portstatus);

	if( edc_base_portstatus == portstatus )
	{
        goto ON_SKIP;
	}
	msleep(20);
	portstatus = gpio_get_value(gpio_num_xheadset_det);
	if( edc_base_portstatus == portstatus )
	{
        goto ON_SKIP;

	}

	if( portstatus == 1 ){

		edc_irq_wake_guard++;
		edc_port[EDC_PLUG].Polling_Counter=0;
		edc_port[EDC_PLUG].timer_flag = 1;
		edc_inout_timer_expire |= 1;

		queue_delayed_work(edc_plug_wq, &work_que_plug, msecs_to_jiffies(PHI35_INSERT_INTERVAL));
		/* edc_suspend_cansel_flg = 1; */
		wake_lock_timeout(&edc_wlock_sus_cancel, EARPHONE_WORKQUE_TO);
		mutex_unlock(&edc_plug_lock);
		return;
	}

    edc_chatt_cnt=0;

	queue_delayed_work(edc_plug_wq, &work_que_chatt, msecs_to_jiffies(PHI35_INSERT_CHATT_AUDIO));
	/* edc_suspend_cansel_flg = 1; */
	wake_lock_timeout(&edc_wlock_sus_cancel, EARPHONE_WORKQUE_TO);

    /* earphone detect skip add start */
	/* if (mdm_powoff_get()) {
		printk("[EDC] HEADPHONE: Headset detect skip \n");
		goto ON_SKIP;
	} else {
		queue_delayed_work(edc_plug_wq, &work_que_chatt, msecs_to_jiffies(PHI35_INSERT_CHATT_AUDIO));
	}*/
	mutex_unlock(&edc_plug_lock);
    return;
ON_SKIP:

	if( ( fj_boot_mode == FJ_MODE_KERNEL_MODE ) || ( fj_boot_mode == FJ_MODE_MAKER_MODE ) ){
		goto ON_SKIP2;
	}

	if( edc_voice_call_flag )
		goto ON_SKIP2;

	if( edc_headset_mic_use_flag )
		goto ON_SKIP2;

	recheck_flg = 0;
	if( edc_model_type & ( EDC_MODEL_TYPE_PIERCE | EDC_MODEL_TYPE_ANTENNA ) ){
		if( ( edc_base_portstatus == 0 ) && ( portstatus == 0 ) )
			recheck_flg = 1;
		else{
			if( ( base_num == EDC_STATE_RESUME ) && ( portstatus == 0 ) ){
				recheck_flg = 1;
			}
		}
	}

	if ( recheck_flg ) {
		ret = edc_hw_earphone_kind_get( &earphone_kind_bitset , 0 , &adc_code );
		if(ret == EDC_OK) {
			int old_flag = ( edc_Headset_Definition_Flag & ~( MIC_SWITCH_CHECK_BIT | EARPHONE_ANTENNA_CHECK_BIT ) );
			int new_flag = (earphone_kind_bitset & ~( MIC_SWITCH_CHECK_BIT | EARPHONE_ANTENNA_CHECK_BIT ) );
			bool was_headset = (switch_get_state(&edc_headset_dev) == 0x0001) ? true : false;
			bool is_headset = (earphone_kind_bitset & MIC_SWITCH_CHECK_BIT) ? true : false;
			bool detect_switch_flag = false;

			if(was_headset != is_headset){
				edc_loop_switch_cnt++;
			} else {
				edc_loop_switch_cnt = 0;
			}
			/* When the state is different six times, call 'handler_plug'. */
			if(edc_loop_switch_cnt > DETECT_SWITCH_NUM){
				detect_switch_flag = true;
			}
			EDC_DBG_LOOP_PRINTK(KERN_INFO "[EDC] %s: edc_loop_switch_cnt=%d detect_switch_flag=%d (%d->%d)\n",
				 __func__, edc_loop_switch_cnt, detect_switch_flag, was_headset, is_headset);

			if (( old_flag != new_flag ) || detect_switch_flag ) {
				printk(KERN_INFO "[EDC] %s earphone old=%d new=%d(ADC:%x) mic_switch(%d->%d)\n",
					__func__, old_flag, new_flag, adc_code, was_headset, is_headset);
				edc_Hedset_Switch_enable_Flag = 0;
				edc_loop_switch_cnt = 0;
				portstatus = gpio_get_value(gpio_num_xheadset_det);
				if( portstatus == 1 ){
					edc_irq_wake_guard++;
					edc_port[EDC_PLUG].Polling_Counter=0;
					edc_port[EDC_PLUG].timer_flag = 1;
					edc_inout_timer_expire |= 1;
					queue_delayed_work(edc_plug_wq, &work_que_plug, msecs_to_jiffies(PHI35_INSERT_INTERVAL));
					wake_lock_timeout(&edc_wlock_sus_cancel, EARPHONE_WORKQUE_TO);
					mutex_unlock(&edc_plug_lock);
					return;
				}
				edc_chatt_cnt=0;
				queue_delayed_work(edc_plug_wq, &work_que_chatt, msecs_to_jiffies(PHI35_INSERT_INTERVAL));
				/* edc_suspend_cansel_flg = 1; */
				wake_lock_timeout(&edc_wlock_sus_cancel, EARPHONE_WORKQUE_TO);
				edc_base_portstatus = -1;
				/* edc_port[EDC_PLUG].Polling_Counter=0;
				edc_port[EDC_PLUG].timer_flag = 1;
				edc_inout_timer_expire |= 1;
				edc_suspend_cansel_flg = 1; */
				mutex_unlock(&edc_plug_lock);
				return;
			}
		}
		else{
			printk("[EDC] ERROR(%s): edc_hw_earphone_kind_get() (%d)\n",__func__,ret);
		}
	}

ON_SKIP2:
	EDC_DBG_LOOP_PRINTK("[EDC] xheadset_det_out_en \n");
	if( portstatus == 1 ){
		ret = qpnp_pin_config(gpio_num_xheadset_det, &earphone_xheadset_det_out_en);
		if (ret < 0) {
			printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET qpnp_pin_config() (%d)\n",__func__,ret);
		}
	}
	queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(PHI35_LOOP));

	if ( edc_base_portstatus == 0 ){
		irq_set_irq_type(edc_port[EDC_PLUG].gpio_det_irq,IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND );
		edc_hw_earphone_enable_irq(edc_port[EDC_PLUG].gpio_det_irq);
	}

	EDC_DBG_LOOP_PRINTK("[EDC] %s() ed \n", __func__);

	mutex_unlock(&edc_plug_lock);
	return;
}

/**********************************************************************/
/* edc_hw_gpio_plug_chatt(base==1,cu==0)							  */
/**********************************************************************/
void edc_hw_gpio_plug_chatt(struct work_struct *work)
{
	/*int ret=EDC_OK;*/
	int portstatus;
	int cu_num=EDC_STATE_CHATT;

#ifdef EDC_DEBUG_LOOP
	EDC_DBG_LOOP_PRINTK("[EDC] %s()\n", __func__);
#else
	if( edc_chatt_cnt == 0 )
		printk(KERN_INFO "[EDC] %s()\n", __func__);
#endif

	/* test */
	/*if( test_cnt == 0 ){
		edc_state_num = 0;
		test_cnt++;
	}*/

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_state_NG( cu_num , edc_state_num );
		edc_state_num = cu_num;
		return;
	}
	edc_state_num = cu_num;
	/* state check ed */

	portstatus = gpio_get_value(gpio_num_xheadset_det);
	if( edc_base_portstatus == portstatus )
	{
        goto ON_SKIP2;
	}

    edc_chatt_cnt++;
	if( edc_chatt_cnt <= CHATT_CNT )
	{
		queue_delayed_work(edc_plug_wq, &work_que_chatt, msecs_to_jiffies(PHI35_INSERT_CHATT_AUDIO));
	}
	else
	{
		queue_delayed_work(edc_plug_wq, &work_que_decision, msecs_to_jiffies(PHI35_INSERT_CHATT_AUDIO));
	}

    return;
ON_SKIP2:

	plug_loop_call = 1;
	queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(PHI35_LOOP));
	wake_unlock(&edc_wlock_sus_cancel);

	return;
}


/**********************************************************************/
/* edc_hw_gpio_plug_decision(base==1,cu==0)							  */
/**********************************************************************/
void edc_hw_gpio_plug_decision(struct work_struct *work)
{
	int ret=EDC_OK;
	int portstatus=0;
	int earphone_kind_bitset=0;
	struct qpnp_vadc_result result_adc;
	/*int c_portstatus=edc_base_portstatus;*/
	int cu_num=EDC_STATE_DESITION;
	int submerge;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	/* test */
	/*if( test_cnt == 0 ){
		edc_state_num = 0;
		test_cnt++;
	}*/

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_state_NG( cu_num , edc_state_num );
		edc_state_num = cu_num;
		return;
	}
	edc_state_num = cu_num;
	/* state check ed */

ON_SKIP3:
	portstatus = gpio_get_value(gpio_num_xheadset_det);
	if( edc_base_portstatus == portstatus )
	{
        goto ON_SKIP2;
	}

	/* remove */
	if( portstatus ){
		/* add 2013/4/11 */
		printk("[EDC] (%s) Error portstatus=%d\n", __func__,portstatus);
		edc_base_portstatus = -1;
		plug_loop_call = 1;
		queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(PHI35_LOOP));
		wake_unlock(&edc_wlock_sus_cancel);
		return;
	}
	/* insert */
	else{

		result_adc.adc_code = 0;
		result_adc.physical = 0;

		/*  SUBMERGE judge */
		ret = qpnp_pin_config(gpio_num_xheadset_vdd_det, &earphone_config);
		if (ret < 0) {
			printk("[EDC] ERROR(%s):qpnp_pin_config() earphone_config (%d)\n",__func__, ret);
			goto ON_SKIP2;
		}
		msleep(20);
		ret = qpnp_vadc_read(edc_vadc, P_MUX2_1_1, &result_adc);
		printk("[EDC] (%s):qpnp_vadc_read() (%d)\n",__func__,ret);
		if (ret) {
			printk("[EDC] ERROR(%s):qpnp_vadc_read() (%d)\n",__func__,ret);
			goto ON_SKIP2;
		}
		printk("[EDC] HEADPHONE: ret=%d ADC adc_code:0x%x physical:%lld\n",ret,result_adc.adc_code, result_adc.physical);
		ret = qpnp_pin_config(gpio_num_xheadset_vdd_det, &earphone_deconfig);
		if (ret < 0) {
			printk("[EDC] ERROR(%s): qpnp_pin_config earphone_deconfig (%d)\n",__func__,ret);
			goto ON_SKIP2;
		}

		submerge = 0;
		if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
			printk( "[EDC] EARPHONE_IN_VOL = %d\n",EARPHONE_IN_VOL_PIERCE);
			if( EARPHONE_IN_VOL_PIERCE <= (result_adc.physical / EARPHONE_ADC) )
				submerge = 1;
		}
		else{
			printk( "[EDC] EARPHONE_IN_VOL = %d\n",EARPHONE_IN_VOL);
			if( EARPHONE_IN_VOL <= (result_adc.physical / EARPHONE_ADC) )
				submerge = 1;
		}

		if( submerge )
		{
			printk("[EDC]  HEADPHONE: SUBMERGE\n");
			ret = qpnp_pin_config(gpio_num_xhscall_det, &earphone_xhscall_det_pulldown);
			if(ret) {
				printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET qpnp_pin_config() (%d)\n",__func__,ret );
			}
			goto ON_SKIP2;
		}

		/* headset or pierce ? */
		earphone_kind_bitset = 0;

		if ( edc_model_type & ( EDC_MODEL_TYPE_PIERCE | EDC_MODEL_TYPE_ANTENNA ) ) {
			ret = edc_hw_earphone_insert( &earphone_kind_bitset );
			if( ret ){
				printk("[EDC] ERROR(%s):edc_hw_earphone_insert() (%d)\n",__func__,ret);
				goto ON_SKIP2;
			}
		}
		else
		{
			earphone_kind_bitset |= HEDSET_CHECK_BIT;
			earphone_kind = EARPHONE_NORMAL;
		}

		if( earphone_kind == EARPHONE_NOT){
			if( edc_pierce_retry_flg == 0 ){
				printk("[EDC] hedset dicision retry \n");
				edc_pierce_retry_flg = 1;
				msleep(200);
				goto ON_SKIP3;
			}
		}

		if( earphone_kind_bitset & PIERCE_ID_ALL_CHECK_BIT ){
			if( edc_pierce_retry_flg == 0 ){
				printk("[EDC] pierce retry \n");
				edc_pierce_retry_flg = 1;
				msleep(200);
				goto ON_SKIP3;
			}
		}
		edc_pierce_retry_flg = 0;

		if( earphone_kind_bitset & PIERCE_ID_ALL_CHECK_BIT )
			edc_mic_bias_flag = 1;

		/* Headsaet */
		if(earphone_kind_bitset & HEDSET_CHECK_BIT) {
			edc_Headset_Definition_Flag |= HEDSET_CHECK_BIT;
			if( edc_Headset_Definition_Flag & MIC_SWITCH_CHECK_BIT ) {
				printk("[EDC] edc_hw_earphone_mic_release insert Headset\n");
				edc_hw_earphone_mic_release( );
			}
			edc_Headset_Definition_Flag &= ~( MIC_SWITCH_CHECK_BIT | EARPHONE_PIERCE_CHECK_BIT );
#if OPT_PIERCE_SWITCH
			if( edc_model_type & EDC_MODEL_TYPE_PIERCE ) {
				edc_Headset_Definition_Flag &= ~PIERCE_ID_ALL_CHECK_BIT;
				if ( switch_get_state(&edc_pierce1_dev) ) {
					switch_set_state(&edc_pierce1_dev, 0x0000);
					printk("[EDC] switch_set_state (pierce1) 0\n");
				}
				if ( switch_get_state(&edc_pierce2_dev) ) {
					switch_set_state(&edc_pierce2_dev, 0x0000);
					printk("[EDC] switch_set_state (pierce2) 0\n");
				}
				if ( switch_get_state(&edc_pierce3_dev) ) {
					switch_set_state(&edc_pierce3_dev, 0x0000);
					printk("[EDC] switch_set_state (pierce3) 0\n");
				}
			}
#endif
			if ( switch_get_state(&edc_pierce_dev) ) {
				switch_set_state(&edc_pierce_dev, 0x0000);
				printk("[EDC] switch_set_state (pierce) 0\n");
			}

#ifdef FJ_EDC_USB_EARPHONE
			ret = edc_ovp_usbearphone_path_set( FJ_DET_USBEARPHONE_PATH_OFF );
			if( ret ){
				printk("[EDC] ERROR(%s):edc_ovp_usbearphone_path_set() (%d)\n",__func__,ret);
				goto ON_SKIP2;
			}
			printk("[EDC] FJ_DET_USBEARPHONE_PATH_OFF\n");
#endif /* FJ_EDC_USB_EARPHONE */

			msleep(100);
			if ( edc_model_type & ( EDC_MODEL_TYPE_PIERCE | EDC_MODEL_TYPE_ANTENNA ) ) {
				if (earphone_kind == EARPHONE_MIC) {
					switch_set_state(&edc_headset_dev, 0x0001); /* EDC_DEVICE */
					printk("[EDC] switch_set_state (headset) 1\n");
				} else {
					switch_set_state(&edc_headset_dev, 0x0002); /* EDC_DEVICE_HEADSET_NO_MIC */
					printk("[EDC] switch_set_state (headset) 2\n");
				}
			} else {
				switch_set_state(&edc_headset_dev, 0x0001); /* EDC_DEVICE */
				printk("[EDC] switch_set_state (headset) 1\n");
			}
			edc_base_portstatus = 0;
			edc_headset_mic_use_flag=0;

#if OPT_AUDIO_POLLING
#else
			edc_switch_irq_flg = 0;

			edc_hw_earphone_phi35_use( );
			/* MIC BIAS ON */
			ret = MICBIAS_CTRL_ON();
			if (ret == EDC_OK)
			{
				edc_mic_bias_flag = 1;
			}
			else
			{
				edc_base_portstatus = -1;
				goto ON_SKIP2;
			}
			edc_hw_earphone_switch_use( );
			msleep(20);
			if( edc_switch_irq_flg == 0 ){
				int irq=0;
				int data;
				printk( "[EDC] edc_switch_irq_flg == 0 \n");
				edc_hw_irq_handler_switch(irq, (void*)&data);
			}
#endif
		}
		/* Pierce */
		else if(earphone_kind_bitset & EARPHONE_PIERCE_CHECK_BIT) {
			edc_Headset_Definition_Flag |= EARPHONE_PIERCE_CHECK_BIT;
			if( edc_Headset_Definition_Flag & MIC_SWITCH_CHECK_BIT ) {
				printk("[EDC] edc_hw_earphone_mic_release insert Pierce\n");
				edc_hw_earphone_mic_release( );
			}
			edc_Headset_Definition_Flag &= ~( HEDSET_CHECK_BIT | MIC_SWITCH_CHECK_BIT );
#if OPT_PIERCE_SWITCH
			if( edc_model_type & EDC_MODEL_TYPE_PIERCE ) {
				edc_Headset_Definition_Flag &= ~PIERCE_ID_ALL_CHECK_BIT;
				if( earphone_kind_bitset & PIERCE_ID_1000_OHM_CHECK_BIT ) {
					edc_Headset_Definition_Flag |= PIERCE_ID_1000_OHM_CHECK_BIT;
					switch_set_state(&edc_pierce1_dev, 0x0001);
					printk("[EDC] switch_set_state (pierce1) 1\n");
				}
				else {
					if ( switch_get_state(&edc_pierce1_dev) ) {
						switch_set_state(&edc_pierce1_dev, 0x0000);
						printk("[EDC] switch_set_state (pierce1) 0\n");
					}
				}
				if( earphone_kind_bitset & PIERCE_ID_1500_OHM_CHECK_BIT ) {
					edc_Headset_Definition_Flag |= PIERCE_ID_1500_OHM_CHECK_BIT;
					switch_set_state(&edc_pierce2_dev, 0x0001);
					printk("[EDC] switch_set_state (pierce2) 1\n");
				}
				else {
					if ( switch_get_state(&edc_pierce2_dev) ) {
						switch_set_state(&edc_pierce2_dev, 0x0000);
						printk("[EDC] switch_set_state (pierce2) 0\n");
					}
				}
				if( earphone_kind_bitset & PIERCE_ID_2200_OHM_CHECK_BIT ) {
					edc_Headset_Definition_Flag |= PIERCE_ID_2200_OHM_CHECK_BIT;
					switch_set_state(&edc_pierce3_dev, 0x0001);
					printk("[EDC] switch_set_state (pierce3) 1\n");
				}
				else {
					if ( switch_get_state(&edc_pierce3_dev) ) {
						switch_set_state(&edc_pierce3_dev, 0x0000);
						printk("[EDC] switch_set_state (pierce3) 0\n");
					}
				}
			}
#endif
			if ( switch_get_state(&edc_headset_dev) ) {
				switch_set_state(&edc_headset_dev, 0x0000);
				printk("[EDC] switch_set_state (headset) 0\n");
			}
			switch_set_state(&edc_pierce_dev, 0x0001);
			printk("[EDC] switch_set_state (pierce) 1\n");
			edc_base_portstatus = 0;
			edc_headset_mic_use_flag=0;
#if OPT_AUDIO_POLLING
#else
			edc_hw_earphone_phi35_unuse( );
			/* MIC BIAS ON */
			ret = MICBIAS_CTRL_ON();
			if (ret == EDC_OK)
			{
				edc_mic_bias_flag = 1;
			}
			else
			{
				goto ON_SKIP2;
			}
			edc_hw_earphone_switch_unuse( );
#endif
		}
		/* antenna */
		else if(earphone_kind_bitset & EARPHONE_ANTENNA_CHECK_BIT) {
			edc_Headset_Definition_Flag |= EARPHONE_ANTENNA_CHECK_BIT;
			if( edc_Headset_Definition_Flag & MIC_SWITCH_CHECK_BIT ) {
				printk("[EDC] edc_hw_earphone_mic_release insert Antenna\n");
				edc_hw_earphone_mic_release( );
			}
			edc_Headset_Definition_Flag &= ~( HEDSET_CHECK_BIT | MIC_SWITCH_CHECK_BIT | EARPHONE_PIERCE_CHECK_BIT );
#if OPT_PIERCE_SWITCH
			if( edc_model_type & EDC_MODEL_TYPE_PIERCE ) {
				edc_Headset_Definition_Flag &= ~PIERCE_ID_ALL_CHECK_BIT;
				if ( switch_get_state(&edc_pierce1_dev) ) {
					switch_set_state(&edc_pierce1_dev, 0x0000);
					printk("[EDC] switch_set_state (pierce1) 0\n");
				}
				if ( switch_get_state(&edc_pierce2_dev) ) {
					switch_set_state(&edc_pierce2_dev, 0x0000);
					printk("[EDC] switch_set_state (pierce2) 0\n");
				}
				if ( switch_get_state(&edc_pierce3_dev) ) {
					switch_set_state(&edc_pierce3_dev, 0x0000);
					printk("[EDC] switch_set_state (pierce3) 0\n");
				}
			}
#endif
			if ( switch_get_state(&edc_pierce_dev) ) {
				switch_set_state(&edc_pierce_dev, 0x0000);
				printk("[EDC] switch_set_state (pierce) 0\n");
			}
			if ( switch_get_state(&edc_headset_dev) ) {
				switch_set_state(&edc_headset_dev, 0x0000);
				printk("[EDC] switch_set_state (headset) 0\n");
			}
			switch_set_state(&edc_antenna_dev, 0x0001);
			printk("[EDC] switch_set_state (antenna) 1\n");
			edc_base_portstatus = 0;
			edc_headset_mic_use_flag=0;
#if OPT_AUDIO_POLLING
#else
			edc_hw_earphone_phi35_unuse( );
			/* MIC BIAS ON */
			ret = MICBIAS_CTRL_ON();
			if (ret == EDC_OK)
			{
				edc_mic_bias_flag = 1;
			}
			else
			{
				goto ON_SKIP2;
			}
			edc_hw_earphone_switch_unuse( );
#endif
		}
		/* detect failed */
		else {
			edc_base_portstatus = -1;
			plug_loop_call = 1;
			queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(PHI35_LOOP));
			wake_unlock(&edc_wlock_sus_cancel);
			return;
			
		}
		/*irq_set_irq_type(edc_port[EDC_PLUG].gpio_det_irq,IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND );
		edc_hw_earphone_enable_irq(edc_port[EDC_PLUG].gpio_det_irq);*/

		printk("[EDC] Insert\n");
	}

	edc_interrupt_flag++;
	edc_hw_wake_up_interruptible();
	/* wake_up_interruptible(&edc_wq); */

ON_SKIP2:
	plug_loop_call = 1;
	queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(PHI35_LOOP));
	wake_unlock(&edc_wlock_sus_cancel);

	return;
}

/**********************************************************************/
/* edc_hw_handler_plug												  */
/**********************************************************************/
void edc_hw_handler_plug(struct work_struct *work)
{
	int ret=EDC_OK;
	int earphone_kind_bitset=0;
	int base_flag=0;
	int cu_num=EDC_STATE_PLUG;
#ifdef FJ_EDC_PLUG_CHATT
	int portstatus=0;
	int chatt_cnt= 0;
#endif

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	cancel_delayed_work_sync(&work_que_loop);
	mutex_lock(&edc_plug_lock);
	EDC_DBG_PRINTK("[EDC] %s() get mutex\n", __func__);
	/* test */
	/*if( test_cnt == 0 ){
		edc_state_num = 0;
		test_cnt++;
	}*/

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_state_NG( cu_num , edc_state_num );
		edc_state_num = cu_num;
		edc_irq_wake_guard--;
		edc_hw_check_wakelock();
		mutex_unlock(&edc_plug_lock);
		return;
	}
	edc_state_num = cu_num;
	/* state check ed */

	edc_hw_earphone_disable_irq(edc_port[EDC_PLUG].gpio_det_irq);

	edc_inout_timer_expire &= 0xfffe;

#ifdef FJ_EDC_PLUG_CHATT
	/* xheadset_det unplug chattering start */
	for(chatt_cnt = 0; chatt_cnt < UNPLUG_CHATT_CNT; chatt_cnt++) {
		portstatus = gpio_get_value(gpio_num_xheadset_det);
		EDC_DBG_LOOP_PRINTK("[EDC] (%s):debug xhedset_read() chatt_cnt(%d) portstatus(%d)\n", __func__, chatt_cnt+1, portstatus);
		if ( portstatus == 0 ){
			printk("[EDC] (%s):debug xhedset chattering portstatus(%d)\n",__func__, portstatus);
			goto GO_LOOP;
		}
		msleep(10);
	}
#endif

	edc_port[EDC_SWITCH].Polling_Counter = 0;
	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
		ret = edc_hw_earphone_remove( &earphone_kind_bitset , 1 );
		if( ret ){
			printk("[EDC] ERROR(%s):edc_hw_earphone_remove() (%d)\n",__func__,ret);
		}
	}
	
	/* MIC BIAS OFF */
	ret =  MICBIAS_CTRL_OFF();
	if (ret == EDC_OK)
	{
		edc_mic_bias_flag = 0;
	}
	/* headset */
	if ( edc_Headset_Definition_Flag & HEDSET_CHECK_BIT ) {
		base_flag = edc_Headset_Definition_Flag;
		edc_Hedset_Switch_enable_Flag = 0;
		if( edc_Headset_Definition_Flag & MIC_SWITCH_CHECK_BIT ) {
			printk("[EDC] edc_hw_earphone_mic_release remove Headset\n");
			edc_hw_earphone_mic_release( );
		}
		edc_Headset_Definition_Flag &= ~(MIC_SWITCH_CHECK_BIT | HEDSET_CHECK_BIT | EARPHONE_ANTENNA_CHECK_BIT );
		if ( switch_get_state(&edc_headset_dev) ) {
			switch_set_state(&edc_headset_dev, 0x0000);
			printk("[EDC] switch_set_state (headset) 0\n");
		}
		if ( switch_get_state(&edc_antenna_dev) ) {
			switch_set_state(&edc_antenna_dev, 0x0000);
			printk("[EDC] switch_set_state (antenna) 0\n");
		}
		edc_base_portstatus = 1;
#ifdef FJ_EDC_USB_EARPHONE
		ret = edc_ovp_usbearphone_path_set( FJ_DET_USBEARPHONE_PATH_ON );
		if( ret ){
			printk("[EDC] ERROR(%s):edc_ovp_usbearphone_path_set() (%d)\n",__func__,ret);
		}
		printk("[EDC] FJ_DET_USBEARPHONE_PATH_ON\n");
#endif /* FJ_EDC_USB_EARPHONE */
#if OPT_AUDIO_POLLING
#else
		edc_hw_earphone_phi35_unuse( );
		edc_hw_earphone_switch_unuse( );
#endif
	}
	else {
		edc_Headset_Definition_Flag &= ~( EARPHONE_PIERCE_CHECK_BIT | PIERCE_ID_ALL_CHECK_BIT | EARPHONE_ANTENNA_CHECK_BIT );
#if OPT_PIERCE_SWITCH
		if ( switch_get_state(&edc_pierce1_dev) ) {
			switch_set_state(&edc_pierce1_dev, 0x0000);
			printk("[EDC] switch_set_state (pierce1) 0\n");
		}
		if ( switch_get_state(&edc_pierce2_dev) ) {
			switch_set_state(&edc_pierce2_dev, 0x0000);
			printk("[EDC] switch_set_state (pierce2) 0\n");
		}
		if ( switch_get_state(&edc_pierce3_dev) ) {
			switch_set_state(&edc_pierce3_dev, 0x0000);
			printk("[EDC] switch_set_state (pierce3) 0\n");
		}
#endif
		if ( switch_get_state(&edc_pierce_dev) ) {
			switch_set_state(&edc_pierce_dev, 0x0000);
			printk("[EDC] switch_set_state (pierce) 0\n");
		}
		if ( switch_get_state(&edc_antenna_dev) ) {
			switch_set_state(&edc_antenna_dev, 0x0000);
			printk("[EDC] switch_set_state (antenna) 0\n");
		}
		edc_base_portstatus = 1;
	}

	printk("[EDC] HEADPHONE: Remove_W\n");

#ifdef FJ_EDC_PLUG_CHATT
GO_LOOP:
	plug_loop_call = 1;
	queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies((portstatus==1) ? PHI35_LOOP : 40));
#else
	plug_loop_call = 1;
	queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(PHI35_LOOP));
#endif

	wake_unlock(&edc_wlock_sus_cancel);
	edc_irq_wake_guard--;
	edc_hw_check_wakelock();

	edc_interrupt_flag++;
	edc_hw_wake_up_interruptible();
	/* wake_up_interruptible(&edc_wq); */
	printk("[EDC] HEADPHONE: Headset %08x\n", edc_Headset_Definition_Flag);
	edc_port[EDC_PLUG].timer_flag = 0;

	mutex_unlock(&edc_plug_lock);
	return;
}

/**********************************************************************/
/* edc_hw_handler_switch											  */
/**********************************************************************/
void edc_hw_handler_switch(struct work_struct *work)
{
	int portstatus;
	int cu_num=EDC_STATE_SWITCHWQ;

	EDC_DBG_LOOP_PRINTK("[EDC] %s()\n", __func__);

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_inout_timer_expire &= 0xfffd;
		edc_port[EDC_SWITCH].timer_flag = 0;
		edc_irq_wake_guard--;
		edc_hw_check_wakelock();
		printk("[EDC] %s() state Error cu_num=%d edc_state_num=%d\n\n", __func__, cu_num, edc_state_num);
		return;
	}
	edc_state_num = cu_num;
	/* state check ed */

	portstatus = gpio_get_value(gpio_num_xhscall_det);
	EDC_DBG_LOOP_PRINTK("[EDC] timer_handler_switch sw_count=%d\n", edc_port[EDC_SWITCH].Polling_Counter);

	edc_inout_timer_expire &= 0xfffd;
	if (edc_port[EDC_SWITCH].io_data == portstatus)
	{
		edc_port[EDC_SWITCH].Polling_Counter++;
		if (!edc_Hedset_Switch_enable_Flag)
		{
			if (edc_port[EDC_SWITCH].Polling_Counter >= FIRST_SWITCH_PULL)
			{
				EDC_DBG_PRINTK("[EDC] timer_handler_switch over\n");
#if OPT_USB_EARPHONE
				EDC_DBG_PRINTK("[EDC] USB(%d) 3.5(%d)\n",edc_UsbEarphoneFlag,edc_FaiEarphoneFlag);
#else
				EDC_DBG_PRINTK("[EDC] 3.5(%d)\n",edc_FaiEarphoneFlag);
#endif

				if (!edc_port[EDC_SWITCH].io_data && (edc_Headset_Definition_Flag & (HEDSET_CHECK_BIT /*| USB_EARPHONE_CHECK_BIT*/)))
				{
					/* Headset switch enable */
					edc_Hedset_Switch_enable_Flag=1;
					printk("[EDC] HEADPHONE: Headset Switch ENABLE\n");
				}
				else
				{
					edc_hw_earphone_disable_irq(edc_port[EDC_SWITCH].gpio_det_irq);
					edc_Hedset_Switch_enable_Flag=0;
					printk("[EDC] HEADPHONE: Headset Switch DISABLE\n");
				}
				edc_irq_wake_guard--;
				edc_hw_check_wakelock();
				edc_interrupt_flag++;
				edc_hw_wake_up_interruptible();
				/* wake_up_interruptible(&edc_wq); */
				edc_port[EDC_SWITCH].timer_flag = 0;
				return;
			}
			else
			{
#if OPT_USB_EARPHONE
				if (edc_Headset_Definition_Flag & (HEDSET_CHECK_BIT | USB_EARPHONE_CHECK_BIT))
#else
				if (edc_Headset_Definition_Flag & HEDSET_CHECK_BIT )
#endif
				{
					queue_delayed_work(edc_plug_wq, &work_que_switch, msecs_to_jiffies(HSCALL_PUSH_INTERVAL));
                }
				else
				{
					printk("[EDC] HEADPHONE: delete timer\n");
					edc_interrupt_flag++;
					edc_hw_wake_up_interruptible();
					/* wake_up_interruptible(&edc_wq); */
					edc_port[EDC_SWITCH].timer_flag = 0;
					edc_irq_wake_guard--;
					edc_hw_check_wakelock();
				}
			}
			
			return;
		}
		
		if (edc_port[EDC_SWITCH].Polling_Counter >= HSCALL_PUSH_POLL_NUM)
		{
			printk("[EDC] \n");
			if ( !edc_port[EDC_SWITCH].io_data && ( edc_Headset_Definition_Flag & MIC_SWITCH_CHECK_BIT ) )
			{
				edc_Headset_Definition_Flag &= ~MIC_SWITCH_CHECK_BIT;
#if OPT_AUDIO_POLLING
#else
				edc_hw_earphone_mic_release( );
#endif
			}
			else if ( edc_port[EDC_SWITCH].io_data && !( edc_Headset_Definition_Flag & MIC_SWITCH_CHECK_BIT ) )
			{
				edc_Headset_Definition_Flag |= MIC_SWITCH_CHECK_BIT;
#if OPT_AUDIO_POLLING
#else
				edc_hw_earphone_mic_push( );
#endif
			}
			else
			{
				printk("[EDC] Not notify. io_data=%d\n", edc_port[EDC_SWITCH].io_data);
			}
			edc_irq_wake_guard--;
			edc_hw_check_wakelock();
			edc_interrupt_flag++;
			edc_hw_wake_up_interruptible();
			/* wake_up_interruptible(&edc_wq); */
			printk("[EDC] HEADPHONE: switch %08x\n", edc_Headset_Definition_Flag);
			edc_port[EDC_SWITCH].timer_flag = 0;
			return ;
		}
	}
	else
	{
		/* status changes under chattering */
		edc_port[EDC_SWITCH].io_data = portstatus;
		edc_port[EDC_SWITCH].Polling_Counter = 0;
	}
	queue_delayed_work(edc_plug_wq, &work_que_switch, msecs_to_jiffies(HSCALL_PUSH_INTERVAL));
	
	return;
}

/**********************************************************************/
/* edc_hw_irq_handler_inout											  */
/**********************************************************************/
irqreturn_t edc_hw_irq_handler_inout(int irq, void *data)
{
	int cu_num=EDC_STATE_ISR;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	/* test */
	/*if( test_cnt == 0 ){
		edc_state_num = 0;
		test_cnt++;
	}*/

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		edc_state_NG( cu_num , edc_state_num );
		edc_state_num = cu_num;
		return IRQ_HANDLED;
	}
	edc_state_num = cu_num;
	/* state check ed */

	/* Earphone detection start */
	if( edc_base_portstatus != 0 )
	{
		/* printk("[EDC] <Earphone: skip>"); */
		return IRQ_HANDLED;
	}
	/* Earphone detection end */
	/* Earphone refactoring start*/
	if(!edc_port[EDC_PLUG].timer_flag )
	{
		edc_irq_wake_guard++;
		/* edc_suspend_cansel_flg = 1; */
		wake_lock_timeout(&edc_wlock_sus_cancel, EARPHONE_WORKQUE_TO);
		/* Earphone interupt enable start */
		wake_lock_timeout(&edc_wlock_gpio, EARPHONE_WAKELOCK_TO);
		edc_port[EDC_PLUG].io_data = 1;
		edc_port[EDC_PLUG].Polling_Counter=0;
		/* Earphone interupt enable end */
		edc_port[EDC_PLUG].timer_flag = 1;
		edc_inout_timer_expire |= 1;
		queue_delayed_work(edc_plug_wq, &work_que_plug, msecs_to_jiffies(PHI35_INSERT_INTERVAL));
	}
	/* Earphone refactoring end*/

	return IRQ_HANDLED;
}

/**********************************************************************/
/* edc_hw_irq_handler_switch										  */
/**********************************************************************/
irqreturn_t edc_hw_irq_handler_switch(int irq, void *data)
{
	int cu_num=EDC_STATE_SWITCHISR;

	EDC_DBG_PRINTK("[EDC] %s() irq(%d)\n", __func__, irq);

	/* state check st */
	if( edc_state_check( cu_num , edc_state_num ) == EDC_STATE_NG ){
		printk("[EDC] %s() state Error cu_num=%d edc_state_num=%d\n\n", __func__, cu_num, edc_state_num);
		return IRQ_HANDLED;
	}
	edc_state_num = cu_num;
	/* state check ed */

	edc_switch_irq_flg = 1;

#if OPT_USB_EARPHONE
	if ((edc_Headset_Definition_Flag & (HEDSET_CHECK_BIT | USB_EARPHONE_CHECK_BIT))==0)
#else
	if ((edc_Headset_Definition_Flag & HEDSET_CHECK_BIT )==0)
#endif
	{
		printk("[EDC] irq_handler_switch not detect headset flag = %x\n",edc_Headset_Definition_Flag);
		return IRQ_HANDLED;
	}
    if (!edc_port[EDC_SWITCH].timer_flag)
	{
		edc_irq_wake_guard++;
		wake_lock_timeout(&edc_wlock_gpio, EARPHONE_WAKELOCK_TO);
		edc_port[EDC_SWITCH].io_data = gpio_get_value(gpio_num_xhscall_det); /* Earphone detection */
		edc_port[EDC_SWITCH].Polling_Counter=0;
		edc_port[EDC_SWITCH].timer_flag = 1;
		edc_inout_timer_expire |= 2;
		queue_delayed_work(edc_plug_wq, &work_que_switch, msecs_to_jiffies(HSCALL_PUSH_INTERVAL));
	}
	return IRQ_HANDLED;
}


/**********************************************************************/
/* edc_hw_earphone_phi35_use										  */
/**********************************************************************/
int edc_hw_earphone_phi35_use( void )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n",__func__);

	edc_FaiEarphoneFlag=1;

#if OPT_USB_EARPHONE

	if (edc_UsbEarphoneFlag==0)
	{
		wake_lock(&edc_wlock_gpio);
	}
	else
	{
		edc_hw_check_wakelock();
	}
#else
	wake_lock(&edc_wlock_gpio);
#endif

	EDC_DBG_PRINTK("[EDC] %s() OK\n",__func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_phi35_unuse										  */
/**********************************************************************/
int edc_hw_earphone_phi35_unuse( void )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n",__func__);

	edc_FaiEarphoneFlag=0;
	
#if OPT_USB_EARPHONE
	if (edc_UsbEarphoneFlag==0)
	{
		/* switch_set_state(&edc_sdev, 0x0000); */
		/* msleep(100); */
	}
#endif

	edc_hw_check_wakelock();

	EDC_DBG_PRINTK("[EDC] %s() OK\n",__func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_switch_use										  */
/**********************************************************************/
int edc_hw_earphone_switch_use( void )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	edc_hw_earphone_enable_irq(edc_port[EDC_SWITCH].gpio_det_irq);

	/* headset switdh pullup */
	ret = qpnp_pin_config(gpio_num_xhscall_det, &earphone_xhscall_det_pullup);
	if(ret) {
		printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET qpnp_pin_config() (%d)\n",__func__,ret );
		return ret;
	}
	edc_hw_check_wakelock();

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_switch_unuse										  */
/**********************************************************************/
int edc_hw_earphone_switch_unuse( void )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	edc_hw_earphone_disable_irq(edc_port[EDC_SWITCH].gpio_det_irq);
	edc_Hedset_Switch_enable_Flag=0;

	/* headset switch pulldown */
	ret = qpnp_pin_config(gpio_num_xhscall_det, &earphone_xhscall_det_pulldown);
	if(ret) {
		printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET qpnp_pin_config() (%d)\n",__func__,ret );
		return ret;
	}
	edc_hw_check_wakelock();

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_mic_push											  */
/**********************************************************************/
int edc_hw_earphone_mic_push( void )
{
	int ret=EDC_OK;
	
	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	input_report_key(edc_ipdev, KEY_MEDIA, 1);
	input_sync(edc_ipdev);

	edc_hw_check_wakelock();

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_mic_release										  */
/**********************************************************************/
int edc_hw_earphone_mic_release( void )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n",__func__);

	input_report_key(edc_ipdev, KEY_MEDIA, 0);
	input_sync(edc_ipdev);

	edc_hw_check_wakelock();

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

#if OPT_USB_EARPHONE
/**********************************************************************/
/* edc_hw_usb_earphone_use	 	 	 								  */
/**********************************************************************/
int edc_hw_usb_earphone_use( int edc_stage )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	if( edc_stage == EARPHONE_USB_STAGE_0 ){
	}
	else if( edc_stage == EARPHONE_USB_STAGE_1 ){
		edc_irq_wake_guard++;
		queue_delayed_work(edc_plug_wq, &work_que_switch, msecs_to_jiffies(HSCALL_PUSH_INTERVAL));
	}
	else{
		printk("[EDC] ERROR(%s):stage No (%d)\n",__func__,edc_stage );
		ret = EDC_NG;
		return ret;
	}

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_usb_earphone_unuse										  */
/**********************************************************************/
int edc_hw_usb_earphone_unuse( int edc_stage )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	if( edc_stage == EARPHONE_USB_STAGE_0 ){
	}
	else if( edc_stage == EARPHONE_USB_STAGE_1 ){
		edc_irq_wake_guard++;
		queue_delayed_work(edc_plug_wq, &work_que_switch, msecs_to_jiffies(HSCALL_PUSH_INTERVAL));
	}
	else{
		printk("[EDC] ERROR(%s):stage No (%d)\n",__func__,edc_stage );
		ret = EDC_NG;
		return ret;
	}
	
	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);
	
	return ret;
}
#endif
/**********************************************************************/
/* edc_hw_earphone_det_output										  */
/**********************************************************************/
int edc_hw_earphone_det_output( void )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	cancel_delayed_work_sync(&work_que_loop);

	ret = qpnp_pin_config(gpio_num_xheadset_det, &earphone_xheadset_det_in_en);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET qpnp_pin_config() (%d)\n",__func__,ret);
		return ret;
	}

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}
/**********************************************************************/
/* edc_hw_earphone_status_get							  			  */
/**********************************************************************/
int edc_hw_earphone_status_get( struct earphone_status *earphone_read_status )
{
	int ret=EDC_OK;
	int	plug_status=0;
	struct qpnp_vadc_result result_adc;
	int earphone_kind_bitset=0;
	int adc_code=0xffff;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	ret = qpnp_pin_config(gpio_num_xheadset_det, &earphone_xheadset_det_in_en);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):GPIO_XHEADSET_DET qpnp_pin_config() (%d)\n",__func__,ret);
		return ret;
	}
	msleep(10);
	/* status get */
	plug_status = gpio_get_value(gpio_num_xheadset_det);
	printk("[EDC] %s 1st EARPHONE_DET_GET=%d\n",__func__, plug_status);
	msleep(20);
	/* status get */
	plug_status = gpio_get_value(gpio_num_xheadset_det);
	printk("[EDC] %s 2nd EARPHONE_DET_GET=%d\n",__func__, plug_status);

	if(edc_model_type & (EDC_MODEL_TYPE_PIERCE | EDC_MODEL_TYPE_ANTENNA)) {
		ret = edc_hw_earphone_kind_get( &earphone_kind_bitset , 1 , &adc_code );
		if( ret ){
			printk("[EDC] ERROR(%s):edc_hw_earphone_kind_get() (%d)\n",__func__,ret);
			return ret;
		}
	}
	else {
		earphone_kind_bitset |= HEDSET_CHECK_BIT;
	}

	if ( plug_status == 0 ) {

			earphone_read_status->plug_status = 1;
			if ( edc_mic_bias_flag == 0 ){
			/* earphone detect */
				ret = qpnp_pin_config(gpio_num_xhscall_det, &earphone_xhscall_det_pullup);
				if(ret) {
					printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET INPUT_PUP qpnp_pin_config (%d)\n",__func__,ret );
					return ret;
				}
				ret = MICBIAS_CTRL_ON();
				if (ret == EDC_OK) {
				}
				else {
					return -EIO;
				}
			}
			earphone_read_status->sw_status = gpio_get_value(gpio_num_xhscall_det);
			printk("[EDC] %s  EARPHONE_SWITCH_GET=%d\n",__func__, earphone_read_status->sw_status);
			if ( edc_mic_bias_flag == 0 ){
				ret = qpnp_pin_config(gpio_num_xhscall_det, &earphone_xhscall_det_pulldown);
				if(ret) {
					printk("[EDC] ERROR(%s):GPIO_XHSCALL_DET INPUT_PDN qpnp_pin_config (%d)\n",__func__,ret );
					return ret;
				}
				ret =  MICBIAS_CTRL_OFF();
				if (ret == EDC_OK) {
				}
				else {
					return -EIO;
				}
			}

			/* ymu_control(YMU_MICBIAS_OFF,0); */
			earphone_read_status->pierce_status = 0;
	}
	else {
		earphone_read_status->sw_status = 0;
		earphone_read_status->plug_status = 0;
		earphone_read_status->pierce_status = 0;
		if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
			ret = edc_hw_earphone_kind_get( &earphone_kind_bitset , 1 , &adc_code );
			if( ret ){
				printk("[EDC] ERROR(%s):edc_hw_earphone_kind_get() (%d)\n",__func__,ret);
				return ret;
			}
		}
	}
	/* adc vol get */

	result_adc.adc_code = 0;
	result_adc.physical = 0;

	ret = qpnp_pin_config(gpio_num_xheadset_vdd_det, &earphone_config);
	if (ret < 0) {
		printk("[EDC] ERROR(%s): qpnp_pin_config() config (%d)\n",__func__,ret);
		return ret;
	}
	msleep(40);

	ret = qpnp_vadc_read(edc_vadc, P_MUX2_1_1, &result_adc);
	printk("[EDC] (%s):qpnp_vadc_read() (%d)\n",__func__,ret);

	if (ret) {
		printk("[EDC] ERROR(%s): qpnp_vadc_read() (%d)\n",__func__,ret);
		return ret;
	}
	printk("[EDC] HEADPHONE: ADC value raw:0x%x physical:%lld\n",result_adc.adc_code, result_adc.physical);
	ret = qpnp_pin_config(gpio_num_xheadset_vdd_det, &earphone_deconfig);
	if (ret < 0) {
		printk("[EDC] ERROR(%s): qpnp_pin_config() deconfig (%d)\n",__func__,ret);
		return ret;
	}
	earphone_read_status->plug_vol[0] = (result_adc.adc_code);
	earphone_read_status->plug_vol[1] = (result_adc.adc_code >> 8);

	/* pierce adc vol get */

	result_adc.adc_code = adc_code;
	result_adc.physical = 0;

	if(edc_model_type & (EDC_MODEL_TYPE_PIERCE | EDC_MODEL_TYPE_ANTENNA)) {
		earphone_read_status->pierce_vol[0] = (result_adc.adc_code);
		earphone_read_status->pierce_vol[1] = (result_adc.adc_code >> 8);
	}
	else {
		earphone_read_status->pierce_vol[0] = 0xff;
		earphone_read_status->pierce_vol[1] = 0xff;
	}

	EDC_DBG_STATE_PRINTK("[EDC] [MC HEAD SET]\n");
	EDC_DBG_STATE_PRINTK("[EDC] plug_status = %d\n", earphone_read_status->plug_status);
	EDC_DBG_STATE_PRINTK("[EDC] sw_status   = %d\n", earphone_read_status->sw_status);
	EDC_DBG_STATE_PRINTK("[EDC] plug_vol[0] = %0x\n", earphone_read_status->plug_vol[0]);
	EDC_DBG_STATE_PRINTK("[EDC] plug_vol[1] = %0x\n", earphone_read_status->plug_vol[1]);
	EDC_DBG_STATE_PRINTK("[EDC] pierce_vol[0] = %0x\n", earphone_read_status->pierce_vol[0]);
	EDC_DBG_STATE_PRINTK("[EDC] pierce_vol[1] = %0x\n", earphone_read_status->pierce_vol[1]);

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_voice_call_stop								  	  */
/**********************************************************************/
int edc_hw_earphone_voice_call_stop( void )
{
	int ret=EDC_OK;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	/* notify to mdm */
	oem_earphone_control_ptr = smem_alloc_vendor0(SMEM_PACO_EARPHONE_DET_SUSPEND);
	if( oem_earphone_control_ptr == NULL ) {
		printk( "SMEM_PACO_EARPHONE_DET_SUSPEND error \n" );
	} else {
	    printk( "SMEM_PACO_EARPHONE_DET_SUSPEND ctrl_off \n" );
		*oem_earphone_control_ptr = 0;
	}

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_insert							  	 		  	  */
/**********************************************************************/
int edc_hw_earphone_insert( int *earphone_kind_bitset )
{
	int ret=EDC_OK;
	struct qpnp_vadc_result result_adc;
	int retry_count=5;

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	*earphone_kind_bitset = 0;

	if( edc_mic_bias_flag == 0 ){
		/* MIC BIAS ON */
		ret = MICBIAS_CTRL_ON();
		if (ret == EDC_OK)
		{
		}
		else
		{
			return EDC_NG;
		}
	}

	/* PM8921-MPP_04 */

	result_adc.adc_code = 0;

	/* adc vol get */
	ret = qpnp_pin_config(gpio_num_antenna_det, &earphone_antenna_config);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):qpnp_pin_config() earphone_antenna_config (%d)\n",__func__, ret);
		return ret;
	}
	
	do {
		retry_count = 5;

		msleep(40);

		ret = qpnp_vadc_read(edc_vadc, P_MUX5_1_1, &result_adc);
		EDC_DBG_PRINTK("[EDC] (%s):qpnp_vadc_read() (%d)\n",__func__,ret);

		if (ret) {
			printk("[EDC] ERROR(%s):qpnp_vadc_read() (%d)\n",__func__,ret);
			return ret;
		}
		printk(KERN_INFO "[EDC] (%s): ADC adc_code:%x count=%d\n", __func__, result_adc.adc_code, retry_count);

#if OPT_PIERCE_SWITCH
		/* pierce1 */
		if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
		    ( result_adc.adc_code >= pierce_id_1000_ohm_low) &&
		    ( result_adc.adc_code <= pierce_id_1000_ohm_high )
		  ) {
			earphone_kind = EARPHONE_PIERCE;
			*earphone_kind_bitset |= ( EARPHONE_PIERCE_CHECK_BIT | PIERCE_ID_1000_OHM_CHECK_BIT ) ;
		}
		/* pierce2 */
		else if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
		    ( result_adc.adc_code >= pierce_id_1500_ohm_low) &&
		    ( result_adc.adc_code <= pierce_id_1500_ohm_high )
		  ) {
			earphone_kind = EARPHONE_PIERCE;
			*earphone_kind_bitset |= ( EARPHONE_PIERCE_CHECK_BIT | PIERCE_ID_1500_OHM_CHECK_BIT ) ;
		}
		/* pierce3 */
		else if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
		    ( result_adc.adc_code >= pierce_id_2200_ohm_low) &&
		    ( result_adc.adc_code <= pierce_id_2200_ohm_high )
		  ) {
			earphone_kind = EARPHONE_PIERCE;
			*earphone_kind_bitset |= ( EARPHONE_PIERCE_CHECK_BIT | PIERCE_ID_2200_OHM_CHECK_BIT ) ;
		}
		else if( ( result_adc.adc_code <= adc_threshold_headset_high ) ){
#else
		/* headset */
		if( ( result_adc.adc_code <= adc_threshold_headset_high ) ){
#endif
			earphone_kind = EARPHONE_NORMAL;
			*earphone_kind_bitset |= HEDSET_CHECK_BIT;
		}
		/* pierce */
		else if( ( result_adc.adc_code <= adc_threshold_pierce_high ) ){
			earphone_kind = EARPHONE_PIERCE;
			*earphone_kind_bitset |= EARPHONE_PIERCE_CHECK_BIT;
		}
		/* mic */
		else if( ( result_adc.adc_code <= adc_threshold_mic_high ) ){
			earphone_kind = EARPHONE_MIC;
			*earphone_kind_bitset |= ( HEDSET_CHECK_BIT | MIC_SWITCH_CHECK_BIT );
		}
		/* antenna */
		else {
			earphone_kind = EARPHONE_ANTENNA;
			*earphone_kind_bitset |= EARPHONE_ANTENNA_CHECK_BIT;
		}
	
		retry_count--;
		
		/* polling */
		while ( retry_count ) {
			msleep(10);
			ret = qpnp_vadc_read(edc_vadc, P_MUX5_1_1, &result_adc);
			EDC_DBG_PRINTK("[EDC] (%s):qpnp_vadc_read() (%d)\n",__func__,ret);
			
			if (ret) {
				printk("[EDC] ERROR(%s):qpnp_vadc_read() (%d)\n",__func__,ret);
				return ret;
			}
			EDC_DBG_PRINTK("[EDC] HEADPHONE: ADC adc_code:%x count=%d\n",result_adc.adc_code, retry_count);

#if OPT_PIERCE_SWITCH
			if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
			    ( result_adc.adc_code >= pierce_id_1000_ohm_low) &&
			    ( result_adc.adc_code <= pierce_id_1000_ohm_high ) ){
				if ( ( *earphone_kind_bitset & PIERCE_ID_1000_OHM_CHECK_BIT ) == 0 ){
					earphone_kind = EARPHONE_NOT;
					break;
				}
			}
			else if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
			    ( result_adc.adc_code >= pierce_id_1500_ohm_low) &&
			    ( result_adc.adc_code <= pierce_id_1500_ohm_high ) ){
				if ( ( *earphone_kind_bitset & PIERCE_ID_1500_OHM_CHECK_BIT ) == 0 ){
					earphone_kind = EARPHONE_NOT;
					break;
				}
			}
			else if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
			    ( result_adc.adc_code >= pierce_id_2200_ohm_low) &&
			    ( result_adc.adc_code <= pierce_id_2200_ohm_high ) ){
				if ( ( *earphone_kind_bitset & PIERCE_ID_2200_OHM_CHECK_BIT ) == 0 ){
					earphone_kind = EARPHONE_NOT;
					break;
				}
			}
			else if( ( result_adc.adc_code <= adc_threshold_headset_high ) ){
#else
			if( result_adc.adc_code <= adc_threshold_headset_high ) {
#endif
				if ( earphone_kind != EARPHONE_NORMAL ){
					earphone_kind = EARPHONE_NOT;
					break;
				}
			}
			else if( result_adc.adc_code <= adc_threshold_pierce_high ) {
				if ( earphone_kind != EARPHONE_PIERCE ) {
					earphone_kind = EARPHONE_NOT;
					break;
				}
			}
			else if( result_adc.adc_code <= adc_threshold_mic_high ) {
				if ( earphone_kind != EARPHONE_MIC ){
					earphone_kind = EARPHONE_NOT;
					break;
				}
			}
			else {
				if( earphone_kind != EARPHONE_ANTENNA ){
					earphone_kind = EARPHONE_NOT;
					break;
				}
			}
			retry_count--;
		}
	
		if ( earphone_kind == EARPHONE_NOT ) {
			*earphone_kind_bitset = 0;
			printk(KERN_WARNING "[EDC] insert not in match: ADC adc_code:%x count=%d\n",result_adc.adc_code, retry_count);
		}
		else {
			break;
		}
	} while( earphone_kind == EARPHONE_NOT );

	ret = qpnp_pin_config(gpio_num_antenna_det, &earphone_antenna_deconfig);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):qpnp_pin_config() earphone_antenna_deconfig (%d)\n",__func__, ret);
		return ret;
	}

	/* pierce detected */
	if( earphone_kind == EARPHONE_PIERCE ){
		*earphone_kind_bitset |= EARPHONE_PIERCE_CHECK_BIT;
		EDC_DBG_PRINTK("[EDC] insert earphone pierce \n");
#ifdef FJ_EDC_PIERCE_GPIO
		if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
			gpio_set_value_cansleep( gpio_num_sel_jear_led , EDC_GPIO_HIGH );
		}
#endif /* FJ_EDC_PIERCE_GPIO */
		
		/* YUM829-MIC3 OFF */
		ret =  MICBIAS_CTRL_OFF();
		if (ret == EDC_OK)
		{
			edc_mic_bias_flag = 0;
		}
		else
		{
			return EDC_NG;
		}
	}
	/* detected other than pierce */
	else {
		if( earphone_kind == EARPHONE_NORMAL ) {
			*earphone_kind_bitset |= HEDSET_CHECK_BIT;
			EDC_DBG_PRINTK("[EDC] insert normal earphone\n");
		}
		else if( earphone_kind == EARPHONE_MIC ) {
			*earphone_kind_bitset |= HEDSET_CHECK_BIT;
			EDC_DBG_PRINTK("[EDC] insert earphone mic \n");
		}
		else if( earphone_kind == EARPHONE_ANTENNA ) {
			*earphone_kind_bitset |= EARPHONE_ANTENNA_CHECK_BIT;
			EDC_DBG_PRINTK("[EDC] insert earphone antenna \n");
		}
		else {
			*earphone_kind_bitset = 0;
			printk(KERN_WARNING "[EDC] insert not in match \n");
		}
			
#ifdef FJ_EDC_PIERCE_GPIO
		if( edc_model_type & EDC_MODEL_TYPE_PIERCE ){
			gpio_set_value_cansleep( gpio_num_sel_jear_led , EDC_GPIO_LOW );
		}
#endif /* FJ_EDC_PIERCE_GPIO */
		if( edc_mic_bias_flag == 0 ){
			/* YUM829-MIC3 OFF */
			ret =  MICBIAS_CTRL_OFF();
			if (ret == EDC_OK) {
				/*edc_mic_bias_flag = 0;*/
			}
			else
			{
				return EDC_NG;
			}
		}
	}

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}
/**********************************************************************/
/* edc_hw_earphone_remove							  	 			  */
/**********************************************************************/
int edc_hw_earphone_remove( int *earphone_kind_bitset , int debug_mode )
{
	int	ret=EDC_OK;

	if( debug_mode )
		EDC_DBG_PRINTK("[EDC] %s()\n", __func__);
	else
		EDC_DBG_LOOP_PRINTK("[EDC] %s()\n", __func__);

	earphone_kind = EARPHONE_NOT ;

	*earphone_kind_bitset  &= ~(PIERCE_ID_1000_OHM_CHECK_BIT|
						 PIERCE_ID_1500_OHM_CHECK_BIT|
						 PIERCE_ID_2200_OHM_CHECK_BIT);

	/* (GPIO-EX COL11=L) */
#ifdef FJ_EDC_PIERCE_GPIO
	gpio_set_value_cansleep( gpio_num_sel_jear_led , EDC_GPIO_LOW );
#endif /* FJ_EDC_PIERCE_GPIO */

    /* PM8921 GPIO_19=H */

	if( debug_mode )
		EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);
	else
		EDC_DBG_LOOP_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_kind_get							  	 		  	  */
/**********************************************************************/
int edc_hw_earphone_kind_get( int *earphone_kind_bitset , int debug_mode , int *adc_code )
{
	int ret=EDC_OK;
	struct qpnp_vadc_result result_adc;

	if( debug_mode )
		EDC_DBG_PRINTK("[EDC] %s()\n", __func__);
	else
		EDC_DBG_LOOP_PRINTK("[EDC] %s()\n", __func__);

	*earphone_kind_bitset = 0;

	if( edc_mic_bias_flag == 0 ){
		/* MIC BIAS ON */
		ret = MICBIAS_CTRL_ON();
		if (ret == EDC_OK) {
		}
		else {
			return EDC_NG;
		}
	}

	/* PM8921-MPP_04 */

	result_adc.adc_code = 0;

	/* adc vol get */
	ret = qpnp_pin_config(gpio_num_antenna_det, &earphone_antenna_config);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):qpnp_pin_config() earphone_antenna_config (%d)\n",__func__, ret);
		return ret;
	}
	
	if( debug_mode )
		msleep(80);
	else
		msleep(40); 

	ret = qpnp_vadc_read(edc_vadc, P_MUX5_1_1, &result_adc);
	EDC_DBG_PRINTK("[EDC] (%s):qpnp_vadc_read() (%d)\n",__func__,ret);

	if (ret) {
		printk("[EDC] ERROR(%s):qpnp_vadc_read() (%d)\n",__func__,ret);
		return ret;
	}

	if( debug_mode )
		printk(KERN_INFO "[EDC] (%s): ADC adc_code:%x\n",__func__, result_adc.adc_code);
	else
		EDC_DBG_LOOP_PRINTK("[EDC] (%s): ADC adc_code:%x\n",__func__, result_adc.adc_code);

	ret = qpnp_pin_config(gpio_num_antenna_det, &earphone_antenna_deconfig);
	if (ret < 0) {
		printk("[EDC] ERROR(%s):qpnp_pin_config() earphone_antenna_deconfig (%d)\n",__func__, ret);
		return ret;
	}

	*adc_code = result_adc.adc_code;

#if OPT_PIERCE_SWITCH
	/* pierce1 */
	if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
	    ( result_adc.adc_code >= pierce_id_1000_ohm_low) &&
	    ( result_adc.adc_code <= pierce_id_1000_ohm_high ) ){
		*earphone_kind_bitset |= ( EARPHONE_PIERCE_CHECK_BIT | PIERCE_ID_1000_OHM_CHECK_BIT ) ;
	}
	/* pierce2 */
	else if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
	    ( result_adc.adc_code >= pierce_id_1500_ohm_low) &&
	    ( result_adc.adc_code <= pierce_id_1500_ohm_high )
	  ) {
		*earphone_kind_bitset |= ( EARPHONE_PIERCE_CHECK_BIT | PIERCE_ID_1500_OHM_CHECK_BIT ) ;
	}
	/* pierce3 */
	else if( ( edc_model_type & EDC_MODEL_TYPE_PIERCE ) &&
	    ( result_adc.adc_code >= pierce_id_2200_ohm_low) &&
	    ( result_adc.adc_code <= pierce_id_2200_ohm_high )
	  ) {
		*earphone_kind_bitset |= ( EARPHONE_PIERCE_CHECK_BIT | PIERCE_ID_2200_OHM_CHECK_BIT ) ;
	}
	else if( ( result_adc.adc_code <= adc_threshold_headset_high ) ){
#else
	/* headset */
	if( ( result_adc.adc_code <= adc_threshold_headset_high ) ){
#endif
		*earphone_kind_bitset |= HEDSET_CHECK_BIT;
	}
	/* pierce */
	else if( ( result_adc.adc_code <= adc_threshold_pierce_high ) ){
		*earphone_kind_bitset |= EARPHONE_PIERCE_CHECK_BIT;
	}
	/* mic */
	else if( ( result_adc.adc_code <= adc_threshold_mic_high ) ){
		*earphone_kind_bitset |= ( HEDSET_CHECK_BIT | MIC_SWITCH_CHECK_BIT );
	}
	/* antenna */
	else {
		*earphone_kind_bitset |= EARPHONE_ANTENNA_CHECK_BIT;
	}

	if( edc_mic_bias_flag == 0 ){
		/* YUM829-MIC3 OFF */
		ret =  MICBIAS_CTRL_OFF();
		if (ret == EDC_OK)
		{
			/*edc_mic_bias_flag = 0;*/
		}
		else
		{
			return EDC_NG;
		}
	}

	if( debug_mode )
		EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);
	else
		EDC_DBG_LOOP_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_nv_read													  */
/**********************************************************************/
int edc_hw_nv_read(void)
{
	int ret=EDC_OK;
	
#if OPT_NV_READ
	uint8_t val[EDC_APNV_PIERCE_JUDGE_I_SIZE];
	uint8_t antval[EDC_APNV_EARANT_JUDGE_I_SIZE];
	int retry_count=0;
	int apnv_id=0;
#endif

	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	if( edc_model_type & EDC_MODEL_TYPE_ANTENNA ) {
		adc_threshold_headset_low  = ADC_HEADSET_LOW;
		adc_threshold_headset_high = ADC_HEADSET_HIGH;
		adc_threshold_pierce_low   = ADC_HEADSET_PIERCE_LOW;
		adc_threshold_pierce_high  = ADC_HEADSET_PIERCE_HIGH;
		adc_threshold_mic_low      = ADC_HEADSET_MIC_LOW;
		adc_threshold_mic_high     = ADC_HEADSET_MIC_HIGH;
		adc_threshold_antenna_low  = ADC_HEADSET_ANTENNA_LOW;
		adc_threshold_antenna_high = ADC_HEADSET_ANTENNA_HIGH;

#if OPT_NV_READ
	memset( antval , 0 , EDC_APNV_EARANT_JUDGE_I_SIZE );

		apnv_id = EDC_APNV_EARANT_JUDGE_I;

		while (retry_count < EDC_RETRY_COUNT_MAX) {
			ret = get_nonvolatile(antval, apnv_id , EDC_APNV_EARANT_JUDGE_I_SIZE);
			if (likely(ret < 0)) {
				retry_count++;
				msleep(20);
			}
			else{
				adc_threshold_headset_low  = antval[0];
				adc_threshold_headset_low  = ((adc_threshold_headset_low << 8 ) | antval[1]);
				adc_threshold_headset_high = antval[2];
				adc_threshold_headset_high = ((adc_threshold_headset_high << 8 ) | antval[3]);
				adc_threshold_pierce_low   = antval[4];
				adc_threshold_pierce_low   = ((adc_threshold_pierce_low << 8 ) | antval[5]);
				adc_threshold_pierce_high  = antval[6];
				adc_threshold_pierce_high  = ((adc_threshold_pierce_high << 8 ) | antval[7]);
				adc_threshold_mic_low      = antval[8];
				adc_threshold_mic_low      = ((adc_threshold_mic_low << 8 ) | antval[9]);
				adc_threshold_mic_high     = antval[10];
				adc_threshold_mic_high     = ((adc_threshold_mic_high << 8 ) | antval[11]);
				adc_threshold_antenna_low  = antval[12];
				adc_threshold_antenna_low  = ((adc_threshold_antenna_low << 8 ) | antval[13]);
				adc_threshold_antenna_high = antval[14];
				adc_threshold_antenna_high = ((adc_threshold_antenna_high << 8 ) | antval[15]);
				break;
			}
		}
		
		if (unlikely(retry_count >= EDC_RETRY_COUNT_MAX)) {
			printk("[EDC] ERROR(%s):get_nonvolatile() (%d)\n",__func__,ret);
		}
		if( ret != EDC_APNV_EARANT_JUDGE_I_SIZE ){
			printk("[EDC] ERROR(%s):get_nonvolatile() read size err(%d)\n",__func__,ret);
		}
		
#endif

		printk("[EDC] adc_threshold_headset_low  = 0x%x\n",adc_threshold_headset_low);
		printk("[EDC] adc_threshold_headset_high = 0x%x\n",adc_threshold_headset_high);
		printk("[EDC] adc_threshold_pierce_low   = 0x%x\n",adc_threshold_pierce_low);
		printk("[EDC] adc_threshold_pierce_high  = 0x%x\n",adc_threshold_pierce_high);
		printk("[EDC] adc_threshold_mic_low      = 0x%x\n",adc_threshold_mic_low);
		printk("[EDC] adc_threshold_mic_high     = 0x%x\n",adc_threshold_mic_high);
		printk("[EDC] adc_threshold_antenna_low  = 0x%x\n",adc_threshold_antenna_low);
		printk("[EDC] adc_threshold_antenna_high = 0x%x\n",adc_threshold_antenna_high);
	}
	
	if( edc_model_type & EDC_MODEL_TYPE_PIERCE ) {
		pierce_id_1000_ohm_low  = ADC_1000_LOW;
		pierce_id_1000_ohm_high = ADC_1000_HIGH;
		pierce_id_1500_ohm_low  = ADC_1500_LOW;
		pierce_id_1500_ohm_high = ADC_1500_HIGH;
		pierce_id_2200_ohm_low  = ADC_2200_LOW;
		pierce_id_2200_ohm_high = ADC_2200_HIGH;

#if OPT_NV_READ

	memset( val , 0 , EDC_APNV_PIERCE_JUDGE_I_SIZE );
	retry_count = 0;

		apnv_id = EDC_APNV_PIERCE_JUDGE_III;

		while (retry_count < EDC_RETRY_COUNT_MAX) {
			ret = get_nonvolatile(val, apnv_id , EDC_APNV_PIERCE_JUDGE_I_SIZE);
			if (likely(ret < 0)) {
				retry_count++;
				msleep(20);
			}
			else{
				pierce_id_1000_ohm_low = val[0];
				pierce_id_1000_ohm_low = ((pierce_id_1000_ohm_low << 8 ) | val[1]);
				
				pierce_id_1000_ohm_high = val[2];
				pierce_id_1000_ohm_high = ((pierce_id_1000_ohm_high << 8 ) | val[3]);
				
				pierce_id_1500_ohm_low = val[4];
				pierce_id_1500_ohm_low = ((pierce_id_1500_ohm_low << 8 ) | val[5]);
				
				pierce_id_1500_ohm_high = val[6];
				pierce_id_1500_ohm_high = ((pierce_id_1500_ohm_high << 8 ) | val[7]);
				
				pierce_id_2200_ohm_low = val[8];
				pierce_id_2200_ohm_low = ((pierce_id_2200_ohm_low << 8 ) | val[9]);
				
				pierce_id_2200_ohm_high = val[10];
				pierce_id_2200_ohm_high = ((pierce_id_2200_ohm_high << 8 ) | val[11]);
				break;
			}
		}
		if (unlikely(retry_count >= EDC_RETRY_COUNT_MAX)) {
			printk("[EDC] ERROR(%s):get_nonvolatile() (%d)\n",__func__,ret);
		}

		if( ret != EDC_APNV_PIERCE_JUDGE_I_SIZE ){
			printk("[EDC] ERROR(%s):get_nonvolatile() read size err(%d)\n",__func__,ret);
		}
#endif
	
		printk("[EDC] pierce_id_1000_ohm_low  = 0x%x\n",pierce_id_1000_ohm_low);
		printk("[EDC] pierce_id_1000_ohm_high = 0x%x\n",pierce_id_1000_ohm_high);
		printk("[EDC] pierce_id_1500_ohm_low  = 0x%x\n",pierce_id_1500_ohm_low);
		printk("[EDC] pierce_id_1500_ohm_high = 0x%x\n",pierce_id_1500_ohm_high);
		printk("[EDC] pierce_id_2200_ohm_low  = 0x%x\n",pierce_id_2200_ohm_low);
		printk("[EDC] pierce_id_2200_ohm_high = 0x%x\n",pierce_id_2200_ohm_high);
	}

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	/* If NV read failed, use driver value instead. */
	return EDC_OK;
}

/**********************************************************************/
/* edc_hw_check_wakwlock											  */
/**********************************************************************/
int edc_hw_check_wakelock( void )
{
	int ret=EDC_OK;
	
	EDC_DBG_PRINTK("[EDC] %s()\n", __func__);

	if (edc_irq_wake_guard)
	{
		EDC_DBG_PRINTK("[EDC] canceled wake_unlock edc_irq_wake_guard=(%d)\n", edc_irq_wake_guard);
		return ret;
	}

#if OPT_USB_EARPHONE
	if ( edc_UsbEarphoneFlag==0 && edc_FaiEarphoneFlag==0 && edc_voice_call_flag == 0)
#else
	if ( edc_FaiEarphoneFlag==0 && edc_voice_call_flag == 0)
#endif
	{
		EDC_DBG_PRINTK("[EDC] wake_unlock 1\n");
		wake_unlock(&edc_wlock_gpio);
		return ret;
	}
	if (edc_inout_timer_expire)
	{
		EDC_DBG_PRINTK("[EDC] timer expire %04x\n", edc_inout_timer_expire);
		return ret;
	}

	if (edc_voice_call_flag)
	{
		if (edc_Hedset_Switch_enable_Flag)
		{
			EDC_DBG_PRINTK("[EDC] wake_lock [voice & switch]\n");
		/*	wake_lock(&edc_wlock_gpio); */
		}
		else
		{
			EDC_DBG_PRINTK("[EDC] wake_lock [voice & non switch]\n");
			wake_unlock(&edc_wlock_gpio);
		}
	}
	else
	{
		EDC_DBG_PRINTK("[EDC] wake_unlock [non voice call]\n");
		wake_unlock(&edc_wlock_gpio);
	}

	return ret;
}

/**********************************************************************/
/* edc_hw_chedule_delayed_work_que_loop								  */
/**********************************************************************/
void edc_hw_chedule_delayed_work_que_loop( void )
{
	plug_loop_call = 1;
	queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(PHI35_LOOP));
}

/**********************************************************************/
/* edc_hw_earphone_enable_irq										  */
/**********************************************************************/
void edc_hw_earphone_enable_irq( int irq )
{
	struct irq_data *d;

	if( irq == edc_port[EDC_PLUG].gpio_det_irq ){
		EDC_DBG_IRQ_PRINTK( "[EDC] PLUG ");
	}
	else if( irq == edc_port[EDC_SWITCH].gpio_det_irq ){
		EDC_DBG_IRQ_PRINTK( "[EDC] SWITCH ");
	}
	else{
		printk( "[EDC] %s Err irq NG %d\n",__func__,irq);
	}

	d = irq_get_irq_data(irq);
	if(d->state_use_accessors & IRQD_IRQ_DISABLED){
		enable_irq(irq);
		EDC_DBG_IRQ_PRINTK( "[EDC] enable_irq() set\n");
		if ( irq == edc_port[EDC_PLUG].gpio_det_irq ) {
			irq_set_irq_wake(edc_port[EDC_PLUG].gpio_det_irq, EDC_WAKE_ON);
			EDC_DBG_IRQ_PRINTK( "irq_set_irq_wake() PLUG ON");
		}
	}
	else{
		EDC_DBG_IRQ_PRINTK( "[EDC] enable_irq() skip\n");
	}
}

/**********************************************************************/
/* edc_hw_earphone_disable_irq										  */
/**********************************************************************/
void edc_hw_earphone_disable_irq( int irq )
{
	struct irq_data *d;

	if( irq == edc_port[EDC_PLUG].gpio_det_irq ){
		EDC_DBG_IRQ_PRINTK( "[EDC] PLUG ");
	}
	else if( irq == edc_port[EDC_SWITCH].gpio_det_irq ){
		EDC_DBG_IRQ_PRINTK( "[EDC] SWITCH ");
	}
	else{
		printk( "[EDC] %s Err irq NG %d\n",__func__,irq);
	}

	d = irq_get_irq_data(irq);
	if( (irq == edc_port[EDC_SWITCH].gpio_det_irq) && (d->state_use_accessors & IRQD_WAKEUP_STATE) ) {
		irq_set_irq_wake(edc_port[EDC_SWITCH].gpio_det_irq, EDC_WAKE_OFF);
		EDC_DBG_IRQ_PRINTK( "irq_set_irq_wake() OFF");
	}

	if((d->state_use_accessors & IRQD_IRQ_DISABLED)==0){
		if ( irq == edc_port[EDC_PLUG].gpio_det_irq ) {
			irq_set_irq_wake(edc_port[EDC_PLUG].gpio_det_irq, EDC_WAKE_OFF);
			EDC_DBG_IRQ_PRINTK( "irq_set_irq_wake() PLUG OFF");
		}
		disable_irq(irq);
		EDC_DBG_IRQ_PRINTK( " disable_irq() set\n");
	}
	else{
		EDC_DBG_IRQ_PRINTK( " disable_irq() skip\n");
	}
}

/**********************************************************************/
/*  edc_state_check													  */
/**********************************************************************/
int edc_state_check( int cu_num , int be_num )
{
	int ret=EDC_STATE_OK;

	if( cu_num >= EDC_STATE_MAX ){
		printk( "[EDC] %s Err cu_num NG %d\n",__func__,cu_num);
		return (EDC_STATE_NG);
	}
	if( be_num >= EDC_STATE_MAX ){
		printk( "[EDC] %s Err be_num NG %d\n",__func__,be_num);
		return (EDC_STATE_NG);
	}
	ret = edc_state_matrix[cu_num][be_num];
	EDC_DBG_STATE_PRINTK( "[EDC] %s cu_num=%d be_num=%d ret=%d\n",__func__,cu_num,be_num,ret);
	return ret;
}

/**********************************************************************/
/*  edc_state_NG													  */
/**********************************************************************/
void edc_state_NG( int cu_num , int be_num )
{
	printk( "[EDC] %s state Error cu_num=%d be_num=%d\n",__func__,cu_num,be_num);
	edc_base_portstatus = -1;
	plug_loop_call = 1;
	queue_delayed_work(edc_plug_wq, &work_que_loop, msecs_to_jiffies(PHI35_LOOP)); /* LOOP(S1) move */
	wake_unlock(&edc_wlock_sus_cancel);
}

#ifdef FJ_EDC_PIERCE_GPIO
/**********************************************************************/
/* edc_hw_earphone_xled_rst_on										  */
/**********************************************************************/
int edc_hw_earphone_xled_rst_on( void )
{
	int ret=EDC_OK;
    struct device_node *node;
    int gpio_num;

	/*EDC_DBG_PRINTK("[EDC] %s()\n", __func__);*/

	/* GPIO-EXCOL7=H */
    node = of_find_compatible_node(NULL, NULL, "fj-an30259c");
    gpio_num = of_get_gpio(node, 0); /* GPIO_KO_XLED_RST */
	gpio_set_value_cansleep( gpio_num , EDC_GPIO_HIGH );

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_xled_rst_off										  */
/**********************************************************************/
int edc_hw_earphone_xled_rst_off( void )
{
	int ret=EDC_OK;
    struct device_node *node;
    int gpio_num;

	/*EDC_DBG_PRINTK("[EDC] %s()\n", __func__);*/

	/* GPIO-EXCOL7=L */
    node = of_find_compatible_node(NULL, NULL, "fj-an30259c");
    gpio_num = of_get_gpio(node, 0); /* GPIO_KO_XLED_RST */
	gpio_set_value_cansleep( gpio_num , EDC_GPIO_LOW );

	EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);

	return ret;
}

int edc_hw_earphone_illumi_log_flg=0;
/**********************************************************************/
/* edc_hw_earphone_illumi_on										  */
/**********************************************************************/
int edc_hw_earphone_illumi_on( void )
{
	int ret=EDC_OK;

	/*EDC_DBG_PRINTK("[EDC] %s()\n", __func__);*/

	if( edc_Headset_Definition_Flag & PIERCE_ID_ALL_CHECK_BIT ){
		/* GPIO-EXCOL11=H */
		gpio_set_value_cansleep( gpio_num_sel_jear_led , EDC_GPIO_HIGH );
		if(  edc_hw_earphone_illumi_log_flg == 0 ){
			EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);
			edc_hw_earphone_illumi_log_flg = 1;
		}
	}

	/*EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);*/

	return ret;
}

/**********************************************************************/
/* edc_hw_earphone_illumi_off										  */
/**********************************************************************/
int edc_hw_earphone_illumi_off( void )
{
	int ret=EDC_OK;

	/*EDC_DBG_PRINTK("[EDC] %s()\n", __func__);*/
	
	/* GPIO-EXCOL11=L */
	gpio_set_value_cansleep( gpio_num_sel_jear_led , EDC_GPIO_LOW );

	if( edc_Headset_Definition_Flag & PIERCE_ID_ALL_CHECK_BIT ){
		if( edc_hw_earphone_illumi_log_flg == 1 ){
			EDC_DBG_PRINTK("[EDC] %s() OK\n", __func__);
			edc_hw_earphone_illumi_log_flg = 0;
		}
	}

	return ret;
}
#endif /* FJ_EDC_PIERCE_GPIO */

