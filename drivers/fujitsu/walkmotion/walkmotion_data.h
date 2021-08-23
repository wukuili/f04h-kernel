/*
 * Copyright(C) 2012-2016 FUJITSU LIMITED
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

#ifndef __WALKMOTION_DATA__
#define __WALKMOTION_DATA__

#define FJ_WM_WAKELOCK_TIME           2	/* 2.0s */
#define FJ_WM_WAKELOCK_TIME_PREPARE   5	/* 5.0s	*/

#include <linux/pinctrl/consumer.h>
struct dsi_pinctrl_res {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_hce_reset;
	struct pinctrl_state *gpio_state_hce_wakeup;
	struct pinctrl_state *gpio_state_hce_irq;
	struct pinctrl_state *gpio_state_hce_sar;
};

/* Walk motion data */
struct fj_wm_data {
    /** Driver state */
    int                state;
    /** Wake lock */
    struct wake_lock   wake_lock;
    /** Motion IRQ wait queue */
    wait_queue_head_t  wait_queue_motion_irq;
    /** Motion IRQ */
    int                motion_irq;
    /** Delay */
    int                mc_init_delay;
    /** Device for debug */
    struct device      *dbg_dev;
    /** enable/disable irq flag */
    int                irq_flag;
	/** prepare flag, spinlock */
	int                prepare_flag;
	spinlock_t         spinlock;
    /** IO lock */
    struct mutex       io_lock;

    int                init_flag;

    int                wakeup_flag;
	struct dsi_pinctrl_res pin_res;
};

/* sar ctrl config */
extern int sar_ctl_flg;

/* debug log */
#define DBG_LOG_WM_TRACE(fmt, args...) \
    do { if (fj_wm_debug_mask & DBG_WM_TRACE) { \
		printk(KERN_DEBUG fmt, ##args); \
    } } while (0)

#define DBG_LOG_WM_DETAIL(fmt, args...) \
    do { if (fj_wm_debug_mask & DBG_WM_DETAIL) { \
		printk(KERN_DEBUG fmt, ##args); \
    } } while (0)

/* non-volatile */
#define APNV_WM_DBG_MODE		49072
#define APNV_SIZE_WM_DBG_MODE		8
#define APNV_ADR_WM_DBG_MODE		3

/* ------------------------------------------------------------------------ */
/* enum                                                                     */
/* ------------------------------------------------------------------------ */
/* for Attributes(Argument type) */
enum {
	DBG_WM_TRACE         = 1 <<   0,
	DBG_WM_DETAIL        = 1 <<   1
};

extern uint8_t fj_wm_debug_mask;

#endif /* __WALKMOTION_DATA__ */
