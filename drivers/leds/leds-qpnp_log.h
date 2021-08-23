/*
 * Copyright(C) 2014-2015 FUJITSU LIMITED
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

#ifndef _LEDS_LOG_QPNP_H_
#define _LEDS_LOG_QPNP_H_

/* ------------------------------------------------------------------------ */
/* LOG MACRO                                                                */
/* ------------------------------------------------------------------------ */
#define	THIS_FILE	"LED driver: "
#define DBG_LOG_LED(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_FUNCENTER(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_FUNC_ENTER) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() Start. " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_FUNCLEAVE(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_FUNC_LEAVE) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() End. " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_SET(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_SET) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_LO_COLOR(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_LO_COLOR) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_HI_COLOR(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_HI_COLOR) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_LO_PAUSE(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_LO_PAUSE) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_HI_PAUSE(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_HI_PAUSE) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_NUM_DUTY_PCTS(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_NUM_DUTY_PCTS) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_RAMP_STEP(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_RAMP_STEP) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_FLAGS(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_FLAGS) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_DUTY(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_DUTY) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_DISP_OFF(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_DISP_OFF) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_SUSPEND(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_SUSPEND) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_RESUME(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_RESUME) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

#define DBG_LOG_LED_DEBUG(fmt, args...) \
	do { if (led_debug_mask & DBG_OPTN_LED_DEBUG) { \
		printk(KERN_INFO THIS_FILE "L%d: %s() " fmt "\n", \
			__LINE__, __FUNCTION__, ## args); \
	} } while (0)

/* non-volatile */
#define APNV_LED_DBG_MODE		49072
#define APNV_SIZE_LED_DBG_MODE		8
#define APNV_ADR_LED_DBG_MODE_1		0
#define APNV_ADR_LED_DBG_MODE_2		1

/* ------------------------------------------------------------------------ */
/* enum                                                                     */
/* ------------------------------------------------------------------------ */
/* for Attributes(Argument type) */
enum {
	DBG_OPTN_LED				= 1 << 0,
	DBG_OPTN_FUNC_ENTER			= 1 << 1,
	DBG_OPTN_FUNC_LEAVE			= 1 << 2,
	DBG_OPTN_LED_SET			= 1 << 3,
	DBG_OPTN_LED_LO_COLOR		= 1 << 4,
	DBG_OPTN_LED_HI_COLOR		= 1 << 5,
	DBG_OPTN_LED_LO_PAUSE		= 1 << 6,
	DBG_OPTN_LED_HI_PAUSE		= 1 << 7,
	DBG_OPTN_LED_NUM_DUTY_PCTS	= 1 << 8,
	DBG_OPTN_LED_RAMP_STEP		= 1 << 9,
	DBG_OPTN_LED_FLAGS			= 1 << 10,
	DBG_OPTN_LED_DUTY			= 1 << 11,
	DBG_OPTN_LED_DISP_OFF		= 1 << 12,
	DBG_OPTN_LED_SUSPEND		= 1 << 13,
	DBG_OPTN_LED_RESUME			= 1 << 14,
	DBG_OPTN_LED_DEBUG			= 1 << 15
};

extern u32 led_debug_mask;

module_param_named(debug_mask, led_debug_mask, uint, S_IWUSR | S_IRUGO);
#endif // _LEDS_LOG_QPNP_H_
