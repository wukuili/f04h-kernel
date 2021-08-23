/*
  * Copyright(C) 2013-2015 FUJITSU LIMITED
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

#ifndef _EDC_HW_H_
#define _EDC_HW_H_

#include <linux/qpnp/pin.h>
#include "edc.h"

/**********************************************************************/
/* macros															  */
/**********************************************************************/
/* for earphone detect */

#define PHI35_INSERT_POLL_NUM	1 	/* 10times -> 5times -> 1times */
#define PHI35_INSERT_INTERVAL	5	/* 25msec -> 50msec -> 5msec */

#define	FIRST_SWITCH_PULL		30/*50*/
#define	HSCALL_PUSH_POLL_NUM	5
#define	HSCALL_PUSH_INTERVAL	20
#define DETECT_SWITCH_NUM		5	/* PHI35_LOOP(1sec) * 6times */

/* add 2013/3/18 */
#define EDC_PIERCE_POSS_MAX		2

enum
{
	DT_XHEADSET_DET     = 0,	/* Earphone jack */
	DT_XHEADSET_VDD_DET = 1,	/* Earphone jack vol */
	DT_EARANT_DET       = 2,	/* JMIC vol */
	DT_XHSCALL_DET      = 3,	/* Earphone switch */
#if 0
	DT_KO_JMIC_ONOFF    = 4,	/* Jmic switch */
#endif
#ifdef FJ_EDC_PIERCE_GPIO
	DT_ANSW_FET_ON      = 5,	/* AN-SW switch */
	DT_SEL_JEAR_LED     = 6,	/* mic-pierce switch */
#endif /* FJ_EDC_PIERCE_GPIO */
};

/** High */
#define EDC_GPIO_HIGH           1
/** Low */
#define EDC_GPIO_LOW            0

#define PHI35_INSERT_CHATT_AUDIO	20	/* 60msec -> 20msec */
#define CHATT_CNT	30	/* 4 -> 30 */
#define UNPLUG_CHATT_CNT	5
#define EARPHONE_IN_VOL			500	/* 500mv */
#define EARPHONE_IN_VOL_PIERCE	100	/* 500mv->100mv */
#define EARPHONE_WAKELOCK_TO	(HZ * 60)
#define EARPHONE_WORKQUE_TO	( HZ * 30 ) /* 30sec safety */

#define EARPHONE_ADC 1000

#define SMEM_PACO_EARPHONE_DET_SUSPEND	10

#define ADC_1000_LOW	0x7486 /* <- 0x82F5 */
#define ADC_1000_HIGH	0x7616 /* <- 0x87AE */
#define ADC_1500_LOW	0x7BEE /* <- 0x8AEB */
#define ADC_1500_HIGH	0x7DA7 /* <- 0x8FF5 */
#define ADC_2200_LOW	0x8261 /* <- 0x935C */
#define ADC_2200_HIGH	0x842E /* <- 0x9870 */

#define ADC_HEADSET_LOW             0x60DD
#define ADC_HEADSET_HIGH            0x74D7  /* <- 0x73AF */
#define ADC_HEADSET_PIERCE_LOW      0x74D8  /* <- 0x73B9 */
#define ADC_HEADSET_PIERCE_HIGH     0x8658  /* <- 0x87B4 */
#define ADC_HEADSET_MIC_LOW         0x8662  /* <- 0x87BE */
#define ADC_HEADSET_MIC_HIGH        0xA2A6  /* <- 0xA155 */
#define ADC_HEADSET_ANTENNA_LOW     0xA2A7  /* <- 0xA15F */
#define ADC_HEADSET_ANTENNA_HIGH    0xA8F1

#define EDC_WAKE_ON		1
#define EDC_WAKE_OFF	0

#define	EDC_APNV_PIERCE_JUDGE_I		47055
#define	EDC_APNV_PIERCE_JUDGE_II	47156
#define	EDC_APNV_PIERCE_JUDGE_III	48018
#define EDC_APNV_EARANT_JUDGE_I		48007

#define	EDC_APNV_PIERCE_JUDGE_I_SIZE		(12)
#define EDC_APNV_EARANT_JUDGE_I_SIZE		(16)

#define EDC_RETRY_COUNT_MAX	100

#define	PIERCE_ID_ALL_CHECK_BIT   ( PIERCE_ID_1000_OHM_CHECK_BIT | PIERCE_ID_1500_OHM_CHECK_BIT | PIERCE_ID_2200_OHM_CHECK_BIT )

static struct qpnp_pin_cfg earphone_config = {
	.mode = QPNP_PIN_MODE_AIN,
	.invert = QPNP_PIN_INVERT_DISABLE,
	.pull = QPNP_PIN_MPP_PULL_UP_OPEN,
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_ENABLE,
	.ain_route = QPNP_PIN_AIN_AMUX_CH6,
};

static struct qpnp_pin_cfg earphone_deconfig = {
	.mode = QPNP_PIN_MODE_AIN,
	.invert = QPNP_PIN_INVERT_DISABLE,
	.pull = QPNP_PIN_MPP_PULL_UP_OPEN,
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_DISABLE,
	.ain_route = QPNP_PIN_AIN_AMUX_CH6,
};

static struct qpnp_pin_cfg earphone_antenna_config = {
	.mode = QPNP_PIN_MODE_AIN,
	.invert = QPNP_PIN_INVERT_DISABLE,
	.pull = QPNP_PIN_MPP_PULL_UP_OPEN,
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_ENABLE,
	.ain_route = QPNP_PIN_AIN_AMUX_CH5,
};

static struct qpnp_pin_cfg earphone_antenna_deconfig = {
	.mode = QPNP_PIN_MODE_AIN,
	.invert = QPNP_PIN_INVERT_DISABLE,
	.pull = QPNP_PIN_MPP_PULL_UP_OPEN,
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_DISABLE,
	.ain_route = QPNP_PIN_AIN_AMUX_CH5,
};

static struct qpnp_pin_cfg earphone_xheadset_det_in_en = {
	.mode = QPNP_PIN_MODE_DIG_IN,
	.pull = QPNP_PIN_GPIO_PULL_NO,
	.vin_sel= QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_ENABLE,
};

static struct qpnp_pin_cfg earphone_xheadset_det_out_en = {
	.mode = QPNP_PIN_MODE_DIG_OUT,
	.output_type = QPNP_PIN_OUT_BUF_CMOS,
	.invert = QPNP_PIN_INVERT_DISABLE,
	.pull = QPNP_PIN_GPIO_PULL_NO,
	.vin_sel = QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_ENABLE,
};

static struct qpnp_pin_cfg earphone_xhscall_det_pullup = {
	.mode = QPNP_PIN_MODE_DIG_IN,
	.pull = QPNP_PIN_GPIO_PULL_UP_1P5,
	.vin_sel= QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_ENABLE,
};

static struct qpnp_pin_cfg earphone_xhscall_det_pulldown = {
	.mode = QPNP_PIN_MODE_DIG_IN,
	.pull = QPNP_PIN_GPIO_PULL_DN,
	.vin_sel= QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_ENABLE,
};

static struct qpnp_pin_cfg earphone_xhscall_det_disable = {
	.mode = QPNP_PIN_MODE_DIG_IN,
	.pull = QPNP_PIN_GPIO_PULL_DN,
	.vin_sel= QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_DISABLE,
};

#endif /* _EDC_HW_H_ */
