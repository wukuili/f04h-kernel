/*
 * fj_mhl_platform.h
 *
 * COPYRIGHT(C) FUJITSU LIMITED 2014-2016
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
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/errno.h>

#if !defined(FJ_MHL_PLATFORM_H)
#define FJ_MHL_PLATFORM_H

typedef enum {
	FJ_TX_HW_INT   = 0,
	FJ_TX_HW_RESET = 1,
	FJ_TX_FW_WAKE  = 2,
} GPIO_INDEX;


int mhl_platform_parse_dt(struct device *dev);
int mhl_platform_init(void);
int mhl_platform_prepare_init(void);
void mhl_platform_power_on(void);
void mhl_platform_power_off(void);
int mhl_get_int_gpio(void);
int mhl_set_gpio(const char *gpio_name, int gpio_number, int direction, int out_val);
int mhl_gpio_get_value(GPIO_INDEX gpio_index);
void mhl_gpio_set_value(GPIO_INDEX gpio_index, int val);


int mhl_is_connected(void);
void mhl_set_hdmi_hpd(uint8_t on);
void mhl_set_hev_vic(uint8_t write_burst_vic);
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0006 add start */
int mhl_charge_control(uint16_t max_current_in_milliamps);
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0006 add end */

/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0003 add start */
int mhl_switch(bool onoff);
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0003 add end */

#endif // FJ_MHL_PLATFORM_H
