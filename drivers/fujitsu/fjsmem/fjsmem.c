/*
 * Copyright(C) 2014 - 2015 FUJITSU LIMITED
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
//==============================================================================
// include file
//==============================================================================

/* FUJITSU LIMITED:2014-10-27 H1515005 Check Master_Clear Running */
/* FUJITSU LIMITED:2014-10-27 H1515008 add start */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <asm/current.h>
#include <linux/slab.h>
#include <soc/qcom/smem.h>
#include <linux/nonvolatile_common.h>
#include "fjsmem.h"

//==============================================================================
// define
//==============================================================================
#define APNV_MDM_MASTER_CLEAR_I		(41070)
#define FJSMEM_MONITOR_DELAY		(3*HZ)
#define FJSMEM_MONITOR_RETRY_NUM	(10)

//==============================================================================
// static valuable
//==============================================================================
static struct delayed_work			modem_fdr_monitor;
static unsigned char				mcflg_modem;
static int							nvget_flag = 0;
static unsigned long				retry_num = 0;

/* FUJITSU LIMTIED:2015-08-06 H1520403 add start */
//==============================================================================
// global valuable
//==============================================================================
extern int fjrecovery_mode;
/* FUJITSU LIMTIED:2015-08-06 H1520403 add end */

//==============================================================================
// functions
//==============================================================================
static void fjsmem_fdrflag_monitor(struct work_struct *work)
{
/* FUJITSU LIMITED:2014-11-06 H1515017 mod start */
	uint32_t* flag_p;
/* FUJITSU LIMITED:2014-11-06 H1515017 mod end */
	int result, retry = 1;

	do {
		if (work == NULL) {
			break;
		}

		if (nvget_flag == 0) {
			result = get_nonvolatile((uint8_t*)&mcflg_modem, APNV_MDM_MASTER_CLEAR_I, sizeof(mcflg_modem));
			if (result < 0) {
				printk(KERN_ERR "[fjsmem] Error get_nonvolatile.\n");
				break;
			}
			nvget_flag = 1;
		}
		if (mcflg_modem == 0) {
			retry = 0;
			break;
		}

		flag_p = smem_alloc_vendor0(SMEM_OEM_V0_007);
		if (!flag_p) {
			printk(KERN_ERR "[fjsmem] Error smem_alloc_vendor0.\n");
			break;
		}

		if (*flag_p == 0) {
			printk(KERN_INFO "[fjsmem] FDR complete \n");
			mcflg_modem = 0;
			result = set_nonvolatile((uint8_t*)&mcflg_modem, APNV_MDM_MASTER_CLEAR_I, sizeof(mcflg_modem));
			retry = 0;
		} else {
			printk(KERN_INFO "[fjsmem] FDR non completion %d \n", (unsigned int)*flag_p);
		}
	} while(0);

	if (retry) {
		if (retry_num < FJSMEM_MONITOR_RETRY_NUM) {
			schedule_delayed_work(&modem_fdr_monitor, FJSMEM_MONITOR_DELAY);
			retry_num++;
		}
	}

	return;
}
static int __init fjsmem_init(void)
{
/* FUJITSU LIMTIED:2015-08-06 H1520403 mod start */
	if (fjrecovery_mode == 0) {
		INIT_DELAYED_WORK(&modem_fdr_monitor, fjsmem_fdrflag_monitor);
		schedule_delayed_work(&modem_fdr_monitor, FJSMEM_MONITOR_DELAY);
	}
/* FUJITSU LIMTIED:2015-08-06 H1520403 mod end */

	return 0;
}

static void __exit fjsmem_exit(void)
{
	cancel_delayed_work_sync(&modem_fdr_monitor);
}

module_init(fjsmem_init);
module_exit(fjsmem_exit);

MODULE_AUTHOR("FUJITSU");
MODULE_DESCRIPTION("fjsmem Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

/* FUJITSU LIMITED:2014-10-27 H1515008 add end */
