/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_PLATFORM_DATA_ATMEL_MXT_TS_H
#define __LINUX_PLATFORM_DATA_ATMEL_MXT_TS_H

#include <linux/types.h>
#include <dt-bindings/input/atmel_mxt_ts.h>

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	enum mxt_suspend_mode suspend_mode;
	int t15_num_keys;
	const unsigned int *t15_keymap;
	unsigned gpio_reset;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_irq_active;
	struct pinctrl_state *pinctrl_state_irq_suspend;
	struct pinctrl_state *pinctrl_state_rst_active;
	struct pinctrl_state *pinctrl_state_rst_suspend;
	struct pinctrl_state *pinctrl_state_i2c_active;
	struct pinctrl_state *pinctrl_state_i2c_suspend;
	const char *vdd_name;
	const char *avdd_name;
	u32 avdd_vol[2];
	u32 avdd_optimum_uv[2];
	const char *cfg_name;
	const char *input_name;
	bool async_start_info;
};

#endif /* __LINUX_PLATFORM_DATA_ATMEL_MXT_TS_H */
