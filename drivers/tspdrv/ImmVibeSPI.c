/*
** =============================================================================
**
** File: ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** $Revision$
**
** Copyright (c) 2007-2015 Immersion Corporation. All Rights Reserved.
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
**
** =============================================================================
*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012-2016
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2017
/*----------------------------------------------------------------------------*/

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports only one actuator.
*/
//#error Please Set NUM_ACTUATORS to the number of actuators supported by this SPI.
#define NUM_ACTUATORS       1

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/interrupt.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pin.h>
#include "../staging/android/timed_output.h"

#include <linux/nonvolatile_common.h> /* FUJITSU LIMITED:2016-01-20 ADD */

#define QPNP_IRQ_FLAGS	(IRQF_TRIGGER_RISING | \
			IRQF_TRIGGER_FALLING | \
			IRQF_ONESHOT)

#define QPNP_HAP_STATUS(b)		(b + 0x0A)
#define QPNP_HAP_LRA_AUTO_RES_LO(b)	(b + 0x0B)
#define QPNP_HAP_LRA_AUTO_RES_HI(b)     (b + 0x0C)
#define QPNP_HAP_EN_CTL_REG(b)		(b + 0x46)
#define QPNP_HAP_EN_CTL2_REG(b)		(b + 0x48)
#define QPNP_HAP_ACT_TYPE_REG(b)	(b + 0x4C)
#define QPNP_HAP_WAV_SHAPE_REG(b)	(b + 0x4D)
#define QPNP_HAP_PLAY_MODE_REG(b)	(b + 0x4E)
#define QPNP_HAP_LRA_AUTO_RES_REG(b)	(b + 0x4F)
#define QPNP_HAP_VMAX_REG(b)		(b + 0x51)
#define QPNP_HAP_ILIM_REG(b)		(b + 0x52)
#define QPNP_HAP_SC_DEB_REG(b)		(b + 0x53)
#define QPNP_HAP_RATE_CFG1_REG(b)	(b + 0x54)
#define QPNP_HAP_RATE_CFG2_REG(b)	(b + 0x55)
#define QPNP_HAP_INT_PWM_REG(b)		(b + 0x56)
#define QPNP_HAP_EXT_PWM_REG(b)		(b + 0x57)
#define QPNP_HAP_PWM_CAP_REG(b)		(b + 0x58)
#define QPNP_HAP_SC_CLR_REG(b)		(b + 0x59)
#define QPNP_HAP_BRAKE_REG(b)		(b + 0x5C)
#define QPNP_HAP_WAV_REP_REG(b)		(b + 0x5E)
#define QPNP_HAP_WAV_S_REG_BASE(b)	(b + 0x60)
#define QPNP_HAP_PLAY_REG(b)		(b + 0x70)
#define QPNP_HAP_SEC_ACCESS_REG(b)	(b + 0xD0)
#define QPNP_HAP_TEST2_REG(b)		(b + 0xE3)

#define QPNP_HAP_STATUS_BUSY		0x02
#define QPNP_HAP_ACT_TYPE_MASK		0xFE
#define QPNP_HAP_LRA			0x0
#define QPNP_HAP_ERM			0x1
#define QPNP_HAP_AUTO_RES_MODE_MASK	0x8F
#define QPNP_HAP_AUTO_RES_MODE_SHIFT	4
#define QPNP_HAP_LRA_HIGH_Z_MASK	0xF3
#define QPNP_HAP_LRA_HIGH_Z_SHIFT	2
#define QPNP_HAP_LRA_RES_CAL_PER_MASK	0xFC
#define QPNP_HAP_RES_CAL_PERIOD_MIN	4
#define QPNP_HAP_RES_CAL_PERIOD_MAX	32
#define QPNP_HAP_PLAY_MODE_MASK		0xCF
#define QPNP_HAP_PLAY_MODE_SHFT		4
#define QPNP_HAP_VMAX_MASK		0xC1
#define QPNP_HAP_VMAX_SHIFT		1
#define QPNP_HAP_VMAX_MIN_MV		116
#define QPNP_HAP_VMAX_MAX_MV		3596
#define QPNP_HAP_ILIM_MASK		0xFE
#define QPNP_HAP_ILIM_MIN_MV		400
#define QPNP_HAP_ILIM_MAX_MV		800
#define QPNP_HAP_SC_DEB_MASK		0xF8
#define QPNP_HAP_SC_DEB_SUB		2
#define QPNP_HAP_SC_DEB_CYCLES_MIN	0
#define QPNP_HAP_DEF_SC_DEB_CYCLES	8
#define QPNP_HAP_SC_DEB_CYCLES_MAX	32
#define QPNP_HAP_SC_CLR			1
#define QPNP_HAP_INT_PWM_MASK		0xFC
#define QPNP_HAP_INT_PWM_FREQ_253_KHZ	253
#define QPNP_HAP_INT_PWM_FREQ_505_KHZ	505
#define QPNP_HAP_INT_PWM_FREQ_739_KHZ	739
#define QPNP_HAP_INT_PWM_FREQ_1076_KHZ	1076
#define QPNP_HAP_WAV_SHAPE_MASK		0xFE
#define QPNP_HAP_RATE_CFG1_MASK		0xFF
#define QPNP_HAP_RATE_CFG2_MASK		0xF0
#define QPNP_HAP_RATE_CFG2_SHFT		8
#define QPNP_HAP_RATE_CFG_STEP_US	5
#define QPNP_HAP_WAV_PLAY_RATE_US_MIN	0
#define QPNP_HAP_DEF_WAVE_PLAY_RATE_US	5715
#define QPNP_HAP_WAV_PLAY_RATE_US_MAX	20475
#define QPNP_HAP_WAV_REP_MASK		0x8F
#define QPNP_HAP_WAV_S_REP_MASK		0xFC
#define QPNP_HAP_WAV_REP_SHFT		4
#define QPNP_HAP_WAV_REP_MIN		1
#define QPNP_HAP_WAV_REP_MAX		128
#define QPNP_HAP_WAV_S_REP_MIN		1
#define QPNP_HAP_WAV_S_REP_MAX		8
#define QPNP_HAP_BRAKE_PAT_MASK		0x3
#define QPNP_HAP_ILIM_MIN_MA		400
#define QPNP_HAP_ILIM_MAX_MA		800
#define QPNP_HAP_EXT_PWM_MASK		0xFC
#define QPNP_HAP_EXT_PWM_FREQ_25_KHZ	25
#define QPNP_HAP_EXT_PWM_FREQ_50_KHZ	50
#define QPNP_HAP_EXT_PWM_FREQ_75_KHZ	75
#define QPNP_HAP_EXT_PWM_FREQ_100_KHZ	100
#define PWM_MAX_DTEST_LINES		4
#define QPNP_HAP_EXT_PWM_DTEST_MASK	0x0F
#define QPNP_HAP_EXT_PWM_DTEST_SHFT	4
#define QPNP_HAP_EXT_PWM_PEAK_DATA	0x7F
#define QPNP_HAP_EXT_PWM_HALF_DUTY	50
#define QPNP_HAP_EXT_PWM_FULL_DUTY	100
#define QPNP_HAP_EXT_PWM_DATA_FACTOR	39
#define QPNP_HAP_WAV_SINE		0
#define QPNP_HAP_WAV_SQUARE		1
#define QPNP_HAP_WAV_SAMP_LEN		8
#define QPNP_HAP_WAV_SAMP_MAX		0x7E
#define QPNP_HAP_BRAKE_PAT_LEN		4
#define QPNP_HAP_PLAY_EN		0x80
#define QPNP_HAP_EN			0x80
#define QPNP_HAP_BRAKE_MASK		0xFE
#define QPNP_HAP_TEST2_AUTO_RES_MASK	0x7F
#define QPNP_HAP_SEC_UNLOCK		0xA5
#define AUTO_RES_ENABLE			0x80
#define AUTO_RES_DISABLE		0x00
#define AUTO_RES_ERR_BIT		0x10

#define QPNP_HAP_TIMEOUT_MS_MAX		15000
#define QPNP_HAP_STR_SIZE		20
#define QPNP_HAP_MAX_RETRIES		5
#define QPNP_HAP_CYCLS			5
#define QPNP_TEST_TIMER_MS		5

#define AUTO_RES_ENABLE_TIMEOUT		20000
#define AUTO_RES_ERR_CAPTURE_RES	5
#define AUTO_RES_ERR_MAX		15

#define MISC_TRIM_ERROR_RC19P2_CLK	0x09F5
#define MISC_SEC_ACCESS			0x09D0
#define MISC_SEC_UNLOCK			0xA5
#define PMI8950_MISC_SID		2

#define POLL_TIME_AUTO_RES_ERR_NS	(5 * NSEC_PER_MSEC)

#define LRA_POS_FREQ_COUNT		6
int lra_play_rate_code[LRA_POS_FREQ_COUNT];

#define AUTO_TUNE_VMAX_LIMIT 11

/*
 * auto resonance mode
 * ZXD - Zero Cross Detect
 * QWD - Quarter Wave Drive
 * ZXD_EOP - ZXD with End Of Pattern
 */
enum qpnp_hap_auto_res_mode {
	QPNP_HAP_AUTO_RES_NONE,
	QPNP_HAP_AUTO_RES_ZXD,
	QPNP_HAP_AUTO_RES_QWD,
	QPNP_HAP_AUTO_RES_MAX_QWD,
	QPNP_HAP_AUTO_RES_ZXD_EOP,
};

/* high Z option lines */
enum qpnp_hap_high_z {
	QPNP_HAP_LRA_HIGH_Z_NONE,
	QPNP_HAP_LRA_HIGH_Z_OPT1,
	QPNP_HAP_LRA_HIGH_Z_OPT2,
	QPNP_HAP_LRA_HIGH_Z_OPT3,
};

/* play modes */
enum qpnp_hap_mode {
	QPNP_HAP_DIRECT,
	QPNP_HAP_BUFFER,
	QPNP_HAP_AUDIO,
	QPNP_HAP_PWM,
};

/* pwm channel info */
struct qpnp_pwm_info {
	struct pwm_device *pwm_dev;
	u32 pwm_channel;
	u32 duty_us;
	u32 period_us;
};

/*
 *  qpnp_hap - Haptic data structure
 *  @ spmi - spmi device
 *  @ hap_timer - hrtimer
 *  @ auto_res_err_poll_timer - hrtimer for auto-resonance error
 *  @ timed_dev - timed output device
 *  @ work - worker
 *  @ auto_res_err_work - correct auto resonance error
 *  @ pwm_info - pwm info
 *  @ lock - mutex lock
 *  @ wf_lock - mutex lock for waveform
 *  @ play_mode - play mode
 *  @ auto_res_mode - auto resonace mode
 *  @ lra_high_z - high z option line
 *  @ timeout_ms - max timeout in ms
 *  @ vmax_mv - max voltage in mv
 *  @ ilim_ma - limiting current in ma
 *  @ sc_deb_cycles - short circuit debounce cycles
 *  @ int_pwm_freq_khz - internal pwm frequency in khz
 *  @ wave_play_rate_us - play rate for waveform
 *  @ ext_pwm_freq_khz - external pwm frequency in khz
 *  @ wave_rep_cnt - waveform repeat count
 *  @ wave_s_rep_cnt - waveform sample repeat count
 *  @ play_irq - irq for play
 *  @ sc_irq - irq for short circuit
 *  @ base - base address
 *  @ act_type - actuator type
 *  @ wave_shape - waveform shape
 *  @ wave_samp - array of wave samples
 *  @ shadow_wave_samp - shadow array of wave samples
 *  @ brake_pat - pattern for active breaking
 *  @ reg_en_ctl - enable control register
 *  @ reg_play - play register
 *  @ lra_res_cal_period - period for resonance calibration
 *  @ state - current state of haptics
 *  @ use_play_irq - play irq usage state
 *  @ use_sc_irq - short circuit irq usage state
 *  @ wf_update - waveform update flag
 *  @ pwm_cfg_state - pwm mode configuration state
 *  @ buffer_cfg_state - buffer mode configuration state
 *  @ en_brake - brake state
 *  @ sup_brake_pat - support custom brake pattern
 *  @ correct_lra_drive_freq - correct LRA Drive Frequency
 *  @ misc_trim_error_rc19p2_clk_reg_present - if MISC Trim Error reg is present
 */
struct qpnp_hap {
	struct spmi_device *spmi;
	struct hrtimer hap_timer;
	struct timed_output_dev timed_dev;
	struct workqueue_struct *hap_wq;
	struct work_struct work;
	struct work_struct auto_res_err_work;
	struct hrtimer hap_test_timer;
	struct work_struct test_work;
	struct qpnp_pwm_info pwm_info;
	struct mutex lock;
	struct mutex wf_lock;
	struct completion completion;
	enum qpnp_hap_mode play_mode;
	enum qpnp_hap_auto_res_mode auto_res_mode;
	enum qpnp_hap_high_z lra_high_z;
	u32 timeout_ms;
	u32 vmax_mv;
	u32 ilim_ma;
	u32 sc_deb_cycles;
	u32 int_pwm_freq_khz;
	u32 wave_play_rate_us;
	u32 ext_pwm_freq_khz;
	u32 wave_rep_cnt;
	u32 wave_s_rep_cnt;
	u32 play_irq;
/* FCNT LIMITED:2016-04-05 MOD-S */
//	u32 sc_irq;
	int sc_irq;
/* FCNT LIMITED:2016-04-05 MOD-E */
	u16 base;
	u8 act_type;
	u8 wave_shape;
	u8 wave_samp[QPNP_HAP_WAV_SAMP_LEN];
	u8 shadow_wave_samp[QPNP_HAP_WAV_SAMP_LEN];
	u8 brake_pat[QPNP_HAP_BRAKE_PAT_LEN];
	u8 reg_en_ctl;
	u8 reg_play;
	u8 lra_res_cal_period;
	u8 ext_pwm_dtest_line;
	bool state;
	bool use_play_irq;
	bool use_sc_irq;
	bool wf_update;
	bool pwm_cfg_state;
	bool buffer_cfg_state;
	bool en_brake;
	bool sup_brake_pat;
	bool correct_lra_drive_freq;
	bool misc_trim_error_rc19p2_clk_reg_present;
};

/*
** Global variables
*/
static struct qpnp_hap* g_pTheClient = NULL;
static VibeInt8 g_nLastForce = 0;
static int g_autotune = 0;    /* FUJITSU:2016-01-20 ADD */
/* helper to read a pmic register */
static int qpnp_hap_read_reg(struct qpnp_hap *hap, u8 *data, u16 addr)
{
	int rc;

	rc = spmi_ext_register_readl(hap->spmi->ctrl, hap->spmi->sid,
							addr, data, 1);
	if (rc < 0)
		dev_err(&hap->spmi->dev,
			"Error reading address: %X - ret %X\n", addr, rc);

	return rc;
}

/* helper to write a pmic register */
static int qpnp_hap_write_reg(struct qpnp_hap *hap, u8 *data, u16 addr)
{
	int rc;

	rc = spmi_ext_register_writel(hap->spmi->ctrl, hap->spmi->sid,
							addr, data, 1);
	if (rc < 0)
		dev_err(&hap->spmi->dev,
			"Error writing address: %X - ret %X\n", addr, rc);

/* FCNT LIMITED:2016-03-01 MOD-S */
//	dev_dbg(&hap->spmi->dev, "write: HAP_0x%x = 0x%x\n", addr, *data);
	DbgOutInfo(("write: HAP_0x%x = 0x%x\n", addr, *data));
/* FCNT LIMITED:2016-03-01 MOD-E */

	return rc;
}

static int qpnp_hap_mod_enable(struct qpnp_hap *hap, int on)
{
	u8 val;
	int rc, i;

	val = hap->reg_en_ctl;
	if (on) {
		val |= QPNP_HAP_EN;
	} else {
		for (i = 0; i < QPNP_HAP_MAX_RETRIES; i++) {
			unsigned long sleep_time =
				QPNP_HAP_CYCLS * hap->wave_play_rate_us;

			rc = qpnp_hap_read_reg(hap, &val,
				QPNP_HAP_STATUS(hap->base));

/* FCNT LIMITED:2016-03-01 MOD-S */
//			dev_dbg(&hap->spmi->dev, "HAP_STATUS=0x%x\n", val);
			DbgOutInfo(("HAP_STATUS=0x%x\n", val));
/* FCNT LIMITED:2016-03-01 MOD-E */

			/* wait for QPNP_HAP_CYCLS cycles of play rate */
			if (val & QPNP_HAP_STATUS_BUSY) {
				usleep_range(sleep_time, sleep_time + 1);
				if (hap->play_mode == QPNP_HAP_DIRECT ||
					hap->play_mode == QPNP_HAP_PWM)
					break;
			} else
				break;
		}

		if (i >= QPNP_HAP_MAX_RETRIES)
/* FCNT LIMITED:2016-03-01 MOD-S */
//			dev_dbg(&hap->spmi->dev,
//				"Haptics Busy. Force disable\n");
			DbgOutInfo(("Haptics Busy. Force disable\n"));
/* FCNT LIMITED:2016-03-01 MOD-E */

		val &= ~QPNP_HAP_EN;
	}

	rc = qpnp_hap_write_reg(hap, &val,
			QPNP_HAP_EN_CTL_REG(hap->base));
	if (rc < 0)
		return rc;

	hap->reg_en_ctl = val;

	return 0;
}

static int qpnp_hap_play(struct qpnp_hap *hap, int on)
{
	u8 val;
	int rc;

	val = hap->reg_play;
	if (on)
		val |= QPNP_HAP_PLAY_EN;
	else
		val &= ~QPNP_HAP_PLAY_EN;

	rc = qpnp_hap_write_reg(hap, &val,
			QPNP_HAP_PLAY_REG(hap->base));
	if (rc < 0)
		return rc;

	hap->reg_play = val;

	return 0;
}

/* configuration api for play mode */
static int qpnp_hap_play_mode_config(struct qpnp_hap *hap)
{
	u8 reg = 0;
	int rc, temp;

	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_PLAY_MODE_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_PLAY_MODE_MASK;
	temp = hap->play_mode << QPNP_HAP_PLAY_MODE_SHFT;
	reg |= temp;
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_PLAY_MODE_REG(hap->base));
	if (rc)
		return rc;
	return 0;
}

/* set api for haptics */
#if 0
static int qpnp_hap_set(struct qpnp_hap *hap, int on)
{
	int rc = 0;

	if (hap->play_mode == QPNP_HAP_PWM) {
		if (on)
			rc = pwm_enable(hap->pwm_info.pwm_dev);
		else
			pwm_disable(hap->pwm_info.pwm_dev);
	} else if (hap->play_mode == QPNP_HAP_BUFFER ||
			hap->play_mode == QPNP_HAP_DIRECT) {
		if (on) {
			rc = qpnp_hap_mod_enable(hap, on);
			if (rc < 0)
				return rc;
			rc = qpnp_hap_play(hap, on);
		} else {
			rc = qpnp_hap_play(hap, on);
			if (rc < 0)
				return rc;

			rc = qpnp_hap_mod_enable(hap, on);
		}
	}

	return rc;
}
#endif

/* configuration api for max volatge */
static int qpnp_hap_vmax_config(struct qpnp_hap *hap)
{
	u8 reg = 0;
	int rc, temp;

	if (hap->vmax_mv < QPNP_HAP_VMAX_MIN_MV)
		hap->vmax_mv = QPNP_HAP_VMAX_MIN_MV;
	else if (hap->vmax_mv > QPNP_HAP_VMAX_MAX_MV)
		hap->vmax_mv = QPNP_HAP_VMAX_MAX_MV;

	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_VMAX_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_VMAX_MASK;
	temp = hap->vmax_mv / QPNP_HAP_VMAX_MIN_MV;
	reg |= (temp << QPNP_HAP_VMAX_SHIFT);
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_VMAX_REG(hap->base));
	if (rc)
		return rc;

	return 0;
}

/* configuration api for short circuit debounce */
static int qpnp_hap_sc_deb_config(struct qpnp_hap *hap)
{
	u8 reg = 0;
	int rc, temp;

/* FCNT LIMITED:2016-04-05 MOD-S */
//	if (hap->sc_deb_cycles < QPNP_HAP_SC_DEB_CYCLES_MIN)
	if (hap->sc_deb_cycles <= QPNP_HAP_SC_DEB_CYCLES_MIN)
/* FCNT LIMITED:2016-04-05 MOD-E */
		hap->sc_deb_cycles = QPNP_HAP_SC_DEB_CYCLES_MIN;
	else if (hap->sc_deb_cycles > QPNP_HAP_SC_DEB_CYCLES_MAX)
		hap->sc_deb_cycles = QPNP_HAP_SC_DEB_CYCLES_MAX;

	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_SC_DEB_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_SC_DEB_MASK;
	if (hap->sc_deb_cycles) {
		temp = fls(hap->sc_deb_cycles) - 1;
		reg |= temp - QPNP_HAP_SC_DEB_SUB;
	}
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_SC_DEB_REG(hap->base));
	if (rc)
		return rc;

	return 0;
}

/* short circuit irq handler */
static irqreturn_t qpnp_hap_sc_irq(int irq, void *_hap)
{
	struct qpnp_hap *hap = _hap;
	u8 reg;
	int rc; /* FCNT LIMITED:2016-04-05 ADD */

	/* clear short circuit register */
/* FCNT LIMITED:2016-03-01 MOD-S */
//	dev_dbg(&hap->spmi->dev, "Short circuit detected\n");
	DbgOutInfo(("Short circuit detected\n"));
/* FCNT LIMITED:2016-03-01 MOD-E */
	reg = QPNP_HAP_SC_CLR;
/* FCNT LIMITED:2016-04-05 MOD-S */
//	qpnp_hap_write_reg(hap, &reg, QPNP_HAP_SC_CLR_REG(hap->base));
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_SC_CLR_REG(hap->base));
	if (unlikely(rc)) {
		DbgOutInfo(("%s ERROR qpnp_hap_write_reg(QPNP_HAP_SC_CLR_REG) - return code: %d.\n",__func__ , rc));
	}
/* FCNT LIMITED:2016-04-05 MOD-E */

	return IRQ_HANDLED;
}

/* FUJITSU:2016-01-20 ADD-S */
static int set_NvFreq(void)
{
	int rc = 0;
	unsigned int code = 0;
	u8 reg = 0, offset = 0;
	unsigned char vibe_getNvData[APNV_VIB_FREQ_SIZE];
	
	/* auto resonance period read */
	memset(vibe_getNvData, 0x00, sizeof(vibe_getNvData));
	rc = get_nonvolatile(vibe_getNvData, APNV_VIB_FREQ, APNV_VIB_FREQ_SIZE);
	if (unlikely(rc < 0))
	{
		DbgOutErr(("%s ERROR get_nonvolatile(APNV_VIB_FREQ) - return code: %d.\n", __func__, rc));
		return rc;
	}
	printk(KERN_INFO "tspdrv : %s APNV_VIB_FREQ LO(0x%02x), HI(0x%02x), OFFSET(0x%02x)\n", __func__
		, vibe_getNvData[APNV_VIB_FREQ_LO],vibe_getNvData[APNV_VIB_FREQ_HI],vibe_getNvData[APNV_VIB_FREQ_OFFSET]);

	code = ((unsigned int)(vibe_getNvData[APNV_VIB_FREQ_HI] >> 4) << 8)
		+ (unsigned int)vibe_getNvData[APNV_VIB_FREQ_LO];
	offset = (u8)vibe_getNvData[APNV_VIB_FREQ_OFFSET] & ~BIT(7);

	if(vibe_getNvData[APNV_VIB_FREQ_OFFSET] & BIT(7)){
		code -= offset;
	} else {
		code += offset;
	}

	reg = (u8)(code & 0xFF);
	rc = qpnp_hap_write_reg(g_pTheClient, &reg,
			QPNP_HAP_RATE_CFG1_REG(g_pTheClient->base));
	if (unlikely(rc)) {
		DbgOutErr(("%s ERROR qpnp_hap_write_reg(QPNP_HAP_RATE_CFG1_REG) - return code: %d.\n",__func__ , rc));
		return rc;
	}
	DbgOutInfo(("%s WRITE addr=0x%x, reg=0x%x\n", __func__, QPNP_HAP_RATE_CFG1_REG(g_pTheClient->base), reg));

	rc = qpnp_hap_read_reg(g_pTheClient, &reg,
			QPNP_HAP_RATE_CFG2_REG(g_pTheClient->base));
	if (unlikely(rc < 0)) {
		DbgOutErr(("%s ERROR qpnp_hap_read_reg(QPNP_HAP_RATE_CFG2_REG) - return code: %d.\n",__func__ , rc));
		return rc;
	}
	DbgOutInfo(("%s READ addr=0x%x, reg=0x%x\n", __func__, QPNP_HAP_RATE_CFG2_REG(g_pTheClient->base), reg));

	reg &= QPNP_HAP_RATE_CFG2_MASK;
	reg |= (u8)(code >> 8) & 0x0F;
	rc = qpnp_hap_write_reg(g_pTheClient, &reg,
			QPNP_HAP_RATE_CFG2_REG(g_pTheClient->base));
	if (unlikely(rc)) {
		DbgOutErr(("%s ERROR qpnp_hap_write_reg(QPNP_HAP_RATE_CFG2_REG) - return code: %d.\n",__func__ , rc));
		return rc;
	}
	DbgOutInfo(("%s WRITE addr=0x%x, reg=0x%x\n", __func__, QPNP_HAP_RATE_CFG2_REG(g_pTheClient->base), reg));
	return rc;
}
/* FUJITSU:2016-01-20 ADD-E */

/* Configuration api for haptics registers */
static int qpnp_hap_config(struct qpnp_hap *hap)
{
	u8 reg = 0;
	int rc, i, temp;

	/* Configure the ACTUATOR TYPE register */
	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_ACT_TYPE_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_ACT_TYPE_MASK;
	reg |= hap->act_type;
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_ACT_TYPE_REG(hap->base));
	if (rc)
		return rc;

	/* Configure auto resonance parameters */
	if (hap->act_type == QPNP_HAP_LRA) {
		if (hap->lra_res_cal_period < QPNP_HAP_RES_CAL_PERIOD_MIN)
			hap->lra_res_cal_period = QPNP_HAP_RES_CAL_PERIOD_MIN;
		else if (hap->lra_res_cal_period > QPNP_HAP_RES_CAL_PERIOD_MAX)
			hap->lra_res_cal_period = QPNP_HAP_RES_CAL_PERIOD_MAX;

		rc = qpnp_hap_read_reg(hap, &reg,
					QPNP_HAP_LRA_AUTO_RES_REG(hap->base));
		if (rc < 0)
			return rc;
		reg &= QPNP_HAP_AUTO_RES_MODE_MASK;
		reg |= (hap->auto_res_mode << QPNP_HAP_AUTO_RES_MODE_SHIFT);
		reg &= QPNP_HAP_LRA_HIGH_Z_MASK;
		reg |= (hap->lra_high_z << QPNP_HAP_LRA_HIGH_Z_SHIFT);
		reg &= QPNP_HAP_LRA_RES_CAL_PER_MASK;
		temp = fls(hap->lra_res_cal_period) - 1;
		reg |= (temp - 2);
		rc = qpnp_hap_write_reg(hap, &reg,
					QPNP_HAP_LRA_AUTO_RES_REG(hap->base));
		if (rc)
			return rc;
	} else {
		/* disable auto resonance for ERM */
		reg = 0x00;

		rc = qpnp_hap_write_reg(hap, &reg,
					QPNP_HAP_LRA_AUTO_RES_REG(hap->base));
		if (rc)
			return rc;
	} 

	/* Configure the PLAY MODE register */
	rc = qpnp_hap_play_mode_config(hap);
	if (rc)
		return rc;

	/* Configure the VMAX register */
	rc = qpnp_hap_vmax_config(hap);
	if (rc)
		return rc;

	/* Configure the ILIM register */
	if (hap->ilim_ma < QPNP_HAP_ILIM_MIN_MA)
		hap->ilim_ma = QPNP_HAP_ILIM_MIN_MA;
	else if (hap->ilim_ma > QPNP_HAP_ILIM_MAX_MA)
		hap->ilim_ma = QPNP_HAP_ILIM_MAX_MA;

	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_ILIM_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_ILIM_MASK;
	temp = (hap->ilim_ma / QPNP_HAP_ILIM_MIN_MA) >> 1;
	reg |= temp;
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_ILIM_REG(hap->base));
	if (rc)
		return rc;

	/* Configure the short circuit debounce register */
	rc = qpnp_hap_sc_deb_config(hap);
	if (rc)
		return rc;

	/* Configure the INTERNAL_PWM register */
	if (hap->int_pwm_freq_khz <= QPNP_HAP_INT_PWM_FREQ_253_KHZ) {
		hap->int_pwm_freq_khz = QPNP_HAP_INT_PWM_FREQ_253_KHZ;
		temp = 0;
	} else if (hap->int_pwm_freq_khz <= QPNP_HAP_INT_PWM_FREQ_505_KHZ) {
		hap->int_pwm_freq_khz = QPNP_HAP_INT_PWM_FREQ_505_KHZ;
		temp = 1;
	} else if (hap->int_pwm_freq_khz <= QPNP_HAP_INT_PWM_FREQ_739_KHZ) {
		hap->int_pwm_freq_khz = QPNP_HAP_INT_PWM_FREQ_739_KHZ;
		temp = 2;
	} else {
		hap->int_pwm_freq_khz = QPNP_HAP_INT_PWM_FREQ_1076_KHZ;
		temp = 3;
	}

	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_INT_PWM_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_INT_PWM_MASK;
	reg |= temp;
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_INT_PWM_REG(hap->base));
	if (rc)
		return rc;

	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_PWM_CAP_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_INT_PWM_MASK;
	reg |= temp;
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_PWM_CAP_REG(hap->base));
	if (rc)
		return rc;

	/* Configure the WAVE SHAPE register */
	rc = qpnp_hap_read_reg(hap, &reg,
			QPNP_HAP_WAV_SHAPE_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_WAV_SHAPE_MASK;
	reg |= hap->wave_shape;
	rc = qpnp_hap_write_reg(hap, &reg,
			QPNP_HAP_WAV_SHAPE_REG(hap->base));
	if (rc)
    {
        DbgOutErr(("ERROR ON qpnp_hap_config - wave_shape %d - return code: %d.\n", reg, rc));
		return rc;
    }

/* FUJITSU:2016-01-20 ADD-S */
	if(set_NvFreq())
	{
/* FUJITSU:2016-01-20 ADD-E */

		/* Configure RATE_CFG1 and RATE_CFG2 registers */
		/* Note: For ERM these registers act as play rate and
		   for LRA these represent resonance period */
/* FCNT LIMITED:2016-04-05 MOD-S */
//		if (hap->wave_play_rate_us < QPNP_HAP_WAV_PLAY_RATE_US_MIN)
		if (hap->wave_play_rate_us <= QPNP_HAP_WAV_PLAY_RATE_US_MIN)
/* FCNT LIMITED:2016-04-05 MOD-E */
			hap->wave_play_rate_us = QPNP_HAP_WAV_PLAY_RATE_US_MIN;
		else if (hap->wave_play_rate_us > QPNP_HAP_WAV_PLAY_RATE_US_MAX)
			hap->wave_play_rate_us = QPNP_HAP_WAV_PLAY_RATE_US_MAX;

	    DbgOutInfo(("Configuring LRA frequency - %d.\n", hap->wave_play_rate_us));
	    
		temp = hap->wave_play_rate_us / QPNP_HAP_RATE_CFG_STEP_US;
		reg = temp & QPNP_HAP_RATE_CFG1_MASK;
		rc = qpnp_hap_write_reg(hap, &reg,
				QPNP_HAP_RATE_CFG1_REG(hap->base));
		if (rc)
			return rc;
		DbgOutInfo(("%s WIRTE addr=0x%x, reg=0x%x\n", __func__, QPNP_HAP_RATE_CFG1_REG(hap->base), reg));    /* FUJITSU:2016-01-20 ADD */
		rc = qpnp_hap_read_reg(hap, &reg,
				QPNP_HAP_RATE_CFG2_REG(hap->base));
		if (rc < 0)
			return rc;
		DbgOutInfo(("%s READ addr=0x%x, reg=0x%x\n", __func__, QPNP_HAP_RATE_CFG2_REG(hap->base), reg));    /* FUJITSU:2016-01-20 ADD */
		reg &= QPNP_HAP_RATE_CFG2_MASK;
		temp = temp >> QPNP_HAP_RATE_CFG2_SHFT;
		reg |= temp;
		rc = qpnp_hap_write_reg(hap, &reg,
				QPNP_HAP_RATE_CFG2_REG(hap->base));
		if (rc)
			return rc;
/* FUJITSU:2016-01-20 ADD-S */
		DbgOutInfo(("%s WRITE addr=0x%x, reg=0x%x\n", __func__, QPNP_HAP_RATE_CFG2_REG(hap->base), reg));
	}
/* FUJITSU:2016-01-20 ADD-E */

	/* Configure BRAKE register */
	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_EN_CTL2_REG(hap->base));
	if (rc < 0)
		return rc;
	reg &= QPNP_HAP_BRAKE_MASK;
	reg |= hap->en_brake;
	rc = qpnp_hap_write_reg(hap, &reg, QPNP_HAP_EN_CTL2_REG(hap->base));
	if (rc)
		return rc;

	if (hap->en_brake && hap->sup_brake_pat) {
		for (i = QPNP_HAP_BRAKE_PAT_LEN - 1, reg = 0; i >= 0; i--) {
			hap->brake_pat[i] &= QPNP_HAP_BRAKE_PAT_MASK;
			temp = i << 1;
			reg |= hap->brake_pat[i] << temp;
		}
		rc = qpnp_hap_write_reg(hap, &reg,
					QPNP_HAP_BRAKE_REG(hap->base));
		if (rc)
			return rc;
	}

	/* Cache enable control register */
	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_EN_CTL_REG(hap->base));
	if (rc < 0)
		return rc;
	hap->reg_en_ctl = reg;

	/* Cache play register */
	rc = qpnp_hap_read_reg(hap, &reg, QPNP_HAP_PLAY_REG(hap->base));
	if (rc < 0)
		return rc;
	hap->reg_play = reg;

	/* setup short circuit irq */
	if (hap->use_sc_irq) {
		rc = devm_request_threaded_irq(&hap->spmi->dev, hap->sc_irq,
			NULL, qpnp_hap_sc_irq,
			QPNP_IRQ_FLAGS,
			"qpnp_sc_irq", hap);
		if (rc < 0) {
			dev_err(&hap->spmi->dev,
				"Unable to request sc(%d) IRQ(err:%d)\n",
				hap->sc_irq, rc);
			return rc;
		}
	}

	return rc;
}

/* DT parsing for haptics parameters */
static int qpnp_hap_parse_dt(struct qpnp_hap *hap)
{
	struct spmi_device *spmi = hap->spmi;
	struct property *prop;
	const char *temp_str;
	u32 temp;
	int rc;

	hap->timeout_ms = QPNP_HAP_TIMEOUT_MS_MAX;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,timeout-ms", &temp);
	if (!rc) {
		hap->timeout_ms = temp;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read timeout\n");
		return rc;
	}

	hap->act_type = QPNP_HAP_LRA;
	rc = of_property_read_string(spmi->dev.of_node,
			"qcom,actuator-type", &temp_str);
	if (!rc) {
		if (strcmp(temp_str, "erm") == 0)
			hap->act_type = QPNP_HAP_ERM;
		else if (strcmp(temp_str, "lra") == 0)
			hap->act_type = QPNP_HAP_LRA;
		else {
			dev_err(&spmi->dev, "Invalid actuator type\n");
			return -EINVAL;
		}
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read actuator type\n");
		return rc;
	}

	if (hap->act_type == QPNP_HAP_LRA) {
        hap->auto_res_mode = QPNP_HAP_AUTO_RES_ZXD_EOP;
		rc = of_property_read_string(spmi->dev.of_node,
				"qcom,lra-auto-res-mode", &temp_str);
		if (!rc) {
			if (strcmp(temp_str, "none") == 0)
				hap->auto_res_mode = QPNP_HAP_AUTO_RES_NONE;
			else if (strcmp(temp_str, "zxd") == 0)
				hap->auto_res_mode = QPNP_HAP_AUTO_RES_ZXD;
			else if (strcmp(temp_str, "qwd") == 0)
				hap->auto_res_mode = QPNP_HAP_AUTO_RES_QWD;
			else if (strcmp(temp_str, "max-qwd") == 0)
				hap->auto_res_mode = QPNP_HAP_AUTO_RES_MAX_QWD;
			else
				hap->auto_res_mode = QPNP_HAP_AUTO_RES_ZXD_EOP;
		} else if (rc != -EINVAL) {
			dev_err(&spmi->dev, "Unable to read auto res mode\n");
			return rc;
		}

		hap->lra_high_z = QPNP_HAP_LRA_HIGH_Z_OPT3;
		rc = of_property_read_string(spmi->dev.of_node,
				"qcom,lra-high-z", &temp_str);
		if (!rc) {
			if (strcmp(temp_str, "none") == 0)
				hap->lra_high_z = QPNP_HAP_LRA_HIGH_Z_NONE;
			else if (strcmp(temp_str, "opt1") == 0)
				hap->lra_high_z = QPNP_HAP_LRA_HIGH_Z_OPT1;
			else if (strcmp(temp_str, "opt2") == 0)
				hap->lra_high_z = QPNP_HAP_LRA_HIGH_Z_OPT2;
			else
				hap->lra_high_z = QPNP_HAP_LRA_HIGH_Z_OPT3;
		} else if (rc != -EINVAL) {
			dev_err(&spmi->dev, "Unable to read LRA high-z\n");
			return rc;
		}

		hap->lra_res_cal_period = QPNP_HAP_RES_CAL_PERIOD_MAX;
		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,lra-res-cal-period", &temp);
		if (!rc) {
			hap->lra_res_cal_period = temp;
		} else if (rc != -EINVAL) {
			dev_err(&spmi->dev, "Unable to read cal period\n");
			return rc;
		}
	}

	rc = of_property_read_string(spmi->dev.of_node,
				"qcom,play-mode", &temp_str);
	if (!rc) {
		if (strcmp(temp_str, "direct") == 0)
			hap->play_mode = QPNP_HAP_DIRECT;
		else if (strcmp(temp_str, "buffer") == 0)
			hap->play_mode = QPNP_HAP_BUFFER;
		else if (strcmp(temp_str, "pwm") == 0)
			hap->play_mode = QPNP_HAP_PWM;
		else if (strcmp(temp_str, "audio") == 0)
			hap->play_mode = QPNP_HAP_AUDIO;
		else {
			dev_err(&spmi->dev, "Invalid play mode\n");
			return -EINVAL;
		}
	} else {
		dev_err(&spmi->dev, "Unable to read play mode\n");
		return rc;
	}

	hap->vmax_mv = QPNP_HAP_VMAX_MAX_MV;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vmax-mv", &temp);
	if (!rc) {
		hap->vmax_mv = temp;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vmax\n");
		return rc;
	}

	hap->ilim_ma = QPNP_HAP_ILIM_MIN_MV;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,ilim-ma", &temp);
	if (!rc) {
		hap->ilim_ma = temp;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read ILim\n");
		return rc;
	}

	hap->sc_deb_cycles = QPNP_HAP_DEF_SC_DEB_CYCLES;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,sc-deb-cycles", &temp);
	if (!rc) {
		hap->sc_deb_cycles = temp;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read sc debounce\n");
		return rc;
	}

	hap->int_pwm_freq_khz = QPNP_HAP_INT_PWM_FREQ_505_KHZ;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,int-pwm-freq-khz", &temp);
	if (!rc) {
		hap->int_pwm_freq_khz = temp;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read int pwm freq\n");
		return rc;
	}

    hap->wave_shape = QPNP_HAP_WAV_SQUARE;
	rc = of_property_read_string(spmi->dev.of_node,
			"qcom,wave-shape", &temp_str);
	if (!rc) {
		if (strcmp(temp_str, "sine") == 0)
			hap->wave_shape = QPNP_HAP_WAV_SINE;
		else if (strcmp(temp_str, "square") == 0)
			hap->wave_shape = QPNP_HAP_WAV_SQUARE;
		else {
			dev_err(&spmi->dev, "Unsupported wav shape\n");
			return -EINVAL;
		}
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read wav shape\n");
		return rc;
	}

	hap->wave_play_rate_us = QPNP_HAP_DEF_WAVE_PLAY_RATE_US;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,wave-play-rate-us", &temp);
	if (!rc) {
		hap->wave_play_rate_us = temp;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read play rate\n");
		return rc;
	}
    
	hap->en_brake = of_property_read_bool(spmi->dev.of_node,
				"qcom,en-brake");

	if (hap->en_brake) {
	prop = of_find_property(spmi->dev.of_node,
			"qcom,brake-pattern", &temp);
	if (!prop) {
			dev_err(&spmi->dev, "brake pattern not found");
		} else if (temp != QPNP_HAP_BRAKE_PAT_LEN) {
			dev_err(&spmi->dev, "Invalid length of brake pattern\n");
			return -EINVAL;
		} else {
			hap->sup_brake_pat = true;
			memcpy(hap->brake_pat, prop->value,
					QPNP_HAP_BRAKE_PAT_LEN);
		}
	}
    
	hap->use_sc_irq = of_property_read_bool(spmi->dev.of_node,
				"qcom,use-sc-irq");
	if (hap->use_sc_irq) {
		hap->sc_irq = spmi_get_irq_byname(hap->spmi,
					NULL, "sc-irq");
		if (hap->sc_irq < 0) {
			dev_err(&spmi->dev, "Unable to get sc irq\n");
			return hap->sc_irq;
		}
	}

	return 0;
}

#ifdef CONFIG_PM
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex);
static int qpnp_haptic_suspend(struct device *dev)
{
	VibeUInt8 nActuatorIndex = 0;
	struct qpnp_hap *hap = dev_get_drvdata(dev);
	DbgOutInfo(("tspdrv: suspend.\n"));
	hrtimer_cancel(&hap->hap_timer);
	cancel_work_sync(&hap->work);
	/* turn-off haptic */
	ImmVibeSPI_ForceOut_SetSamples(0, 8, 1, &nActuatorIndex);
	ImmVibeSPI_ForceOut_AmpDisable(1);
	return 0;
}
#endif

static int qpnp_haptic_probe(struct spmi_device *spmi)
{
	struct resource *hap_resource;
	int rc;

	DbgOutInfo(("qpnp_haptic_probe.\n"));

	g_pTheClient = devm_kzalloc(&spmi->dev, sizeof(*g_pTheClient), GFP_KERNEL);
	if (!g_pTheClient)
		return -ENOMEM;

	g_pTheClient->spmi = spmi;

	hap_resource = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!hap_resource) {
		dev_err(&spmi->dev, "Unable to get haptic base address\n");
		return -EINVAL;
	}
	g_pTheClient->base = hap_resource->start;

	dev_set_drvdata(&spmi->dev, g_pTheClient);

	rc = qpnp_hap_parse_dt(g_pTheClient);
	if (rc) {
		dev_err(&spmi->dev, "DT parsing failed\n");
		return rc;
	}

	rc = qpnp_hap_config(g_pTheClient);
	if (rc) {
		dev_err(&spmi->dev, "hap config failed\n");
		return rc;
	}

	mutex_init(&g_pTheClient->lock);
	mutex_init(&g_pTheClient->wf_lock);

	return 0;
}

static int qpnp_haptic_remove(struct spmi_device *spmi)
{
	struct qpnp_hap *hap = dev_get_drvdata(&spmi->dev);

	cancel_work_sync(&hap->work);
	mutex_destroy(&hap->lock);
	mutex_destroy(&hap->wf_lock);

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{ .compatible = "qcom,qpnp-haptic", },
	{ },
};

static SIMPLE_DEV_PM_OPS(qpnp_haptic_pm_ops, qpnp_haptic_suspend, NULL);

static struct spmi_driver qpnp_haptic_driver = {
	.driver		= {
		.name	= "qcom,qpnp-haptic",
		.of_match_table = spmi_match_table,
		.pm	= &qpnp_haptic_pm_ops,
	},
	.probe		= qpnp_haptic_probe,
	.remove		= qpnp_haptic_remove,
};

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
    DbgOutInfo(("ImmVibeSPI_ForceOut_AmpDisable.\n"));

    /* Disable amp */
    /* turn-off haptic */
    qpnp_hap_mod_enable(g_pTheClient, false);
    
    //gpio_set_value_cansleep(1010, 0);
    
    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    DbgOutInfo(("ImmVibeSPI_ForceOut_AmpEnable.\n"));

    /* Enable amp */
    /* To be implemented with appropriate hardware access macros */
    qpnp_hap_mod_enable(g_pTheClient, true);

    // GPIO latency test
    //gpio_set_value_cansleep(1010, 1);
    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    int nRet = 0;
    
    DbgOutInfo(("ImmVibeSPI_ForceOut_Initialize.\n"));
    
    nRet = spmi_driver_register(&qpnp_haptic_driver);
	if (nRet) 
    {
        DbgOutErr(("tspdrv: spmi_driver_register failed.\n"));
        return nRet;    /* FUJITSU:2016-01-20 ADD */
    }
    
    /* Disable amp */    
    ImmVibeSPI_ForceOut_AmpDisable(0);

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
    DbgOutInfo(("ImmVibeSPI_ForceOut_Terminate.\n"));

    spmi_driver_unregister(&qpnp_haptic_driver);

    /* Disable amp */
    ImmVibeSPI_ForceOut_AmpDisable(0);

    return VIBE_S_SUCCESS;
}

static int setAutoTune(int pOn)
{
    u8 reg = 0;
    int rc = 0;
    int temp = 0;;
    DbgOutInfo(("setAutoTune: %d.\n", pOn));
    if(pOn)
    {
        rc = qpnp_hap_read_reg(g_pTheClient, &reg,
                    QPNP_HAP_LRA_AUTO_RES_REG(g_pTheClient->base));
        if (rc < 0)
        {
            DbgOutErr(("ERROR: setAutoTune ON: %d.\n", rc));
            return rc;
        }
        
        reg &= QPNP_HAP_AUTO_RES_MODE_MASK;
        reg |= (g_pTheClient->auto_res_mode << QPNP_HAP_AUTO_RES_MODE_SHIFT);
        reg &= QPNP_HAP_LRA_HIGH_Z_MASK;
        reg |= (g_pTheClient->lra_high_z << QPNP_HAP_LRA_HIGH_Z_SHIFT);
        reg &= QPNP_HAP_LRA_RES_CAL_PER_MASK;
        temp = fls(g_pTheClient->lra_res_cal_period) - 1;
        reg |= (temp - 2);
        rc = qpnp_hap_write_reg(g_pTheClient, &reg,
                    QPNP_HAP_LRA_AUTO_RES_REG(g_pTheClient->base));
        if (rc)
        {
            DbgOutErr(("ERROR: setAutoTune ON: %d.\n", rc));
            return rc;
        }
    }
    else
    {
        // disable auto resonance 
		reg = 0x00;

		rc = qpnp_hap_write_reg(g_pTheClient, &reg,
					QPNP_HAP_LRA_AUTO_RES_REG(g_pTheClient->base));
		if (rc)
		{
            DbgOutErr(("ERROR: setAutoTune OFF: %d.\n", rc));
            return rc;
        }
    }
    return rc;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
	u8 reg = 0;
	int rc, temp;
    
    rc = qpnp_hap_read_reg(g_pTheClient, &reg,
				QPNP_HAP_STATUS(g_pTheClient->base));
    if (rc < 0)
    {
        DbgOutErr(("ImmVibeSPI_ForceOut_SetSamples Error reading QPNP_HAP_STATUS\n"));
    }
    DbgOutInfo(("ImmVibeSPI_ForceOut_SetSamples - QPNP_HAP_STATUS - %d %d \n", reg, pForceOutputBuffer[0]));
    
    if (0 == nBufferSizeInBytes) return VIBE_S_SUCCESS;

    if ((0 == nActuatorIndex) && (8 == nOutputSignalBitDepth) && (1 == nBufferSizeInBytes))
    {
        VibeInt8 force = pForceOutputBuffer[0];

        if (g_nLastForce == force)
        {
            return VIBE_S_SUCCESS;
        }
        else
        {
            g_nLastForce = force;
        }
        
        if (force <= 0)
        {
            
            temp = (-1*force / 4) ;
            if (temp > 31) temp = 31;
            if (temp < 0) temp = 0;
            
            setAutoTune(false);
                
            reg |= (temp << QPNP_HAP_VMAX_SHIFT);
            rc = qpnp_hap_write_reg(g_pTheClient, &reg, QPNP_HAP_VMAX_REG(g_pTheClient->base));
            if (rc)
            {
                DbgOutErr(("ImmVibeSPI_ForceOut_SetSamples Error setting QPNP_HAP_VMAX_REG: %d\n", rc));
                return VIBE_E_FAIL;
            }

            DbgOutInfo(("ImmVibeSPI_ForceOut_SetSamples - Braking %d \n", temp));
            
            
            qpnp_hap_play(g_pTheClient, 0);
            
        } 
        else
        {
            rc = qpnp_hap_read_reg(g_pTheClient, &reg, QPNP_HAP_VMAX_REG(g_pTheClient->base));
            if (rc < 0)
            {
                DbgOutErr(("ImmVibeSPI_ForceOut_SetSamples Error reading QPNP_HAP_VMAX_REG\n"));
                return VIBE_E_FAIL;
            }
            DbgOutInfo(("%s READ addr=0x%x, reg=0x%x\n", __func__, QPNP_HAP_VMAX_REG(g_pTheClient->base), reg));    /* FUJITSU:2016-01-20 ADD */
            reg &= QPNP_HAP_VMAX_MASK;
            // force from 0~127 to VMAx from 0 to 31 = Divide by 4.
            temp = force / 4 ;
            if (temp > 31) temp = 31;
//          if (temp < 0) temp = 0; /* FCNT:2016-04-05 DEL */
            
            if(temp > AUTO_TUNE_VMAX_LIMIT)
            {
/* FUJITSU:2016-01-20 MOD-S */
//                setAutoTune(true);
                if((makercmd_mode == WKUP_VIB_MODE_MC) && (g_autotune))
                {
                    setAutoTune(true);
                }
                else
                {
                    setAutoTune(false);
                }
/* FUJITSU:2016-01-20 MOD-E */
            }
            else
            {
                // Since changing the wave form increases the magnitude, we will remove 2 level.
                temp = temp -2;
                
                if(temp<0)
                {
                    temp = 0;
                }
                setAutoTune(false);
            }
            
            reg |= (temp << QPNP_HAP_VMAX_SHIFT);
            rc = qpnp_hap_write_reg(g_pTheClient, &reg, QPNP_HAP_VMAX_REG(g_pTheClient->base));
            if (rc)
                return VIBE_E_FAIL;
            DbgOutInfo(("%s WRITE addr=0x%x, reg=0x%x\n", __func__, QPNP_HAP_VMAX_REG(g_pTheClient->base), reg));    /* FUJITSU:2016-01-20 ADD */

            // Playing 
            qpnp_hap_play(g_pTheClient, 1);

            DbgOutInfo(("ImmVibeSPI_ForceOut_SetSamples - %d %d %d %d %d\n", nActuatorIndex, nOutputSignalBitDepth, nBufferSizeInBytes, pForceOutputBuffer[0], temp));
            
            return VIBE_S_SUCCESS;
        }
    }
    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
#if 1
    /* 
    ** The following code is provided as sample. If enabled, it will allow device 
    ** frequency parameters tuning via the ImmVibeSetDeviceKernelParameter API.
    ** Please modify as required. 
    */
    switch (nFrequencyParameterID)
    {
        case VIBE_KP_CFG_FREQUENCY_PARAM1:
            /* Update frequency parameter 1 */
            break;
        case VIBE_KP_CFG_FREQUENCY_PARAM2:
            /* Update frequency parameter 2 */
            break;
        case VIBE_KP_CFG_FREQUENCY_PARAM3:
            /* Update frequency parameter 3 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM4:
            /* Update frequency parameter 4 */
            break;

        case VIBE_KP_CFG_FREQUENCY_PARAM5:
            /* Update frequency parameter 5 */
            break;
        case VIBE_KP_CFG_FREQUENCY_PARAM6:
            break;
    }
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    DbgOutInfo(("ImmVibeSPI_Device_GetName.\n"));
#if 0   /* The following code is provided as sample. Please modify as required. */
    if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

    strncpy(szDevName, "Generic", nSize-1);
    szDevName[nSize - 1] = '\0';    /* make sure the string is NULL terminated */
#endif

    return VIBE_S_SUCCESS;
}

/* FUJITSU:2016-01-20 ADD-S */
static int mc_mode_AmpEnable(VibeUInt8 nActuatorIndex, int autotune)
{
    int err_flg =VIBE_S_SUCCESS;
	
    DbgOutVerbose(("mc_mode_AmpEnable.  Level=%d \n", nActuatorIndex));

	g_autotune = autotune;

    err_flg = ImmVibeSPI_ForceOut_SetSamples(0, 8, 1, &nActuatorIndex);
	if(err_flg == VIBE_E_FAIL)
	{
		DbgOutErr(("mc_mode_AmpEnable Error SetSample.\n"));
		return err_flg;
	}

	ImmVibeSPI_ForceOut_AmpEnable(0);

	g_autotune = 0;

    return err_flg;
}

#if HAVE_UNLOCKED_IOCTL
static int mc_mode_GetAutoTune(VibeUInt8 nActuatorIndex, long* value)
#else
static int mc_mode_GetAutoTune(VibeUInt8 nActuatorIndex, int* value)
#endif

{
	u8 reg_lo = 0,reg_hi = 0;
	int rc     = -1;
    int code   = 0;
	unsigned char vibe_setNvData[APNV_VIB_FREQ_SIZE];
	DbgOutInfo(("mc_mode_GetAutoTune in. \n"));

    if(nActuatorIndex <= 0)
	{
        return -EINVAL;
	}
    
	if (g_pTheClient->reg_en_ctl && QPNP_HAP_EN) 
	{
	    rc = qpnp_hap_read_reg(g_pTheClient, &reg_lo, QPNP_HAP_LRA_AUTO_RES_LO(g_pTheClient->base));
		if (rc < 0)
		{
		    DbgOutErr(("ImmVibeSPI_ForceOut_SetSamples Error reading HAPTICS_LRA_AUTO_RES_LO\n"));
			return rc;
		}
	    rc = qpnp_hap_read_reg(g_pTheClient, &reg_hi, QPNP_HAP_LRA_AUTO_RES_HI(g_pTheClient->base));
	    if (rc < 0)
	    {
	        DbgOutErr(("ImmVibeSPI_ForceOut_SetSamples Error reading HAPTICS_LRA_AUTO_RES_HI\n"));
	    	return rc;
	    }
		
		printk(KERN_INFO "mc_mode_GetAutoTune. HAPTICS_LRA_AUTO_RES = LO(0x%02x), HI(0x%02x)\n", reg_lo, reg_hi);
	    code = ((int)((reg_hi & 0xF0) >>4)<<8) + (int)reg_lo;
	    DbgOutInfo(("mc_mode_GetAutoTune. code 0x%02x \n", code));
		if ((code >= AUTO_FREQ_MIN) && (code <= AUTO_FREQ_MAX)) 
		{
			/* nonvolatile write */
			memset(vibe_setNvData, 0x00, sizeof(vibe_setNvData));
			rc = get_nonvolatile(vibe_setNvData, APNV_VIB_FREQ, APNV_VIB_FREQ_SIZE);
			if (unlikely(rc < 0)) {
				DbgOutErr(("%s: get_nonvolatile() [ret=%d]\n", MODULE_NAME, rc));
			}
			else
			{
				vibe_setNvData[APNV_VIB_FREQ_LO] = reg_lo;
				vibe_setNvData[APNV_VIB_FREQ_HI] = reg_hi;
				
				rc = set_nonvolatile(vibe_setNvData, APNV_VIB_FREQ, APNV_VIB_FREQ_SIZE);
				if (unlikely(rc < 0)) {
					DbgOutErr(("%s: set_nonvolatile() [ret=%d]\n", MODULE_NAME, rc));
				}
			}
		}
		else {
			DbgOutErr(("mc_mode_GetAutoTune.  code ERANGE. \n"));
			rc = -ERANGE;
		}
		*value = (int)(reg_lo<<8) + (int)reg_hi;
	}
	else 
	{
		DbgOutErr(("mc_mode_GetAutoTune. Haptics amp already disabled. \n"));
	}
	DbgOutInfo(("mc_mode_GetAutoTune out. rc=%d, value=0x%x \n",rc , *value));
    return rc;
}
/* FUJITSU:2016-01-20 ADD-E */
