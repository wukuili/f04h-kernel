/*
 * Copyright(C) 2012-2015 FUJITSU LIMITED
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

#ifndef __WALKMOTION_HW__
#define __WALKMOTION_HW__

#include <linux/walkmotion.h>

#if defined(CONFIG_OF)
enum {
    DT_WM_GPIO_MOTION_IRQ = 0,
    DT_WM_GPIO_RESET = 1,
    DT_WM_GPIO_HOSU_WUP = 2,
};
#endif

#define FJ_WM_SLEEP_TIME                (500)
#define FJ_WM_SLEEP_TIME_AFTER_LDO_ON (100)
#define FJ_WM_HCE_RESET_IRQ_WAIT_TIME   (250)

/* High */
#define FJ_WM_GPIO_HIGH                 (1)
/* Low */
#define FJ_WM_GPIO_LOW                  (0)


/* Prototype declaration */
void fj_wm_hw_motion_irq(struct work_struct *ws);
int fj_wm_hw_init(void);
int fj_wm_hw_exit(void);
int fj_wm_hw_probe(void);
int fj_wm_hw_remove(void);
int fj_wm_hw_release(void);
long fj_wm_hw_hce_reset(void);
long fj_wm_hw_hce_request_irq(unsigned int value);
long fj_wm_hw_hce_cancel_irq(void);
long fj_wm_hw_hce_set_sar(int arg);
long fj_wm_hw_hce_wakeup_set(unsigned int value);

#endif /* __WALKMOTION_HW__ */
