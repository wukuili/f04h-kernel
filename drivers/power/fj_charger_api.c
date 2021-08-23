/*
 * COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/fj_charger.h>
#include "fj_charger_local.h"
#include <linux/nonvolatile_common.h>

/************************/
/*	external reference	*/
/************************/
/* charger */
extern void fj_chg_drv_source_req(int source, unsigned int mA);
extern void fj_chg_drv_current_req(int source, unsigned int mA);
extern void fj_chg_drv_notify_error(fj_chg_err_type chg_err);

/*********/
/*	API  */
/*********/
/* charger */
void fj_chg_usb_vbus_draw(unsigned int onoff)
{
	unsigned int mA = 0;

	if (onoff != 0) {
		mA = FJ_CHG_CURRENT_500;
	} else {
		mA = FJ_CHG_CURRENT_OFF;
	}
	fj_chg_drv_source_req(CHG_SOURCE_USB, mA);
}
EXPORT_SYMBOL_GPL(fj_chg_usb_vbus_draw);

void fj_chg_ac_vbus_draw(unsigned int onoff)
{
	unsigned int mA = 0;

	if (onoff != 0) {
		mA = FJ_CHG_CURRENT_1500;
	} else {
		mA = FJ_CHG_CURRENT_OFF;
	}
	fj_chg_drv_source_req(CHG_SOURCE_AC, mA);
}
EXPORT_SYMBOL_GPL(fj_chg_ac_vbus_draw);

void fj_chg_holder_vbus_draw(unsigned int onoff)
{
#ifndef FJ_CHARGER_HOLDER_UNMOUNT
	u8	val;
	int result;
	unsigned int mA = 0;

	if (onoff != 0) {
		mA = FJ_CHG_CURRENT_1500;
	} else {
		mA = FJ_CHG_CURRENT_OFF;
	}

	if (mA) {
		/* Change desktop holder */
		result = get_nonvolatile(&val, APNV_CHG_DESKTOP_FOLDER_CHARGE_I, 1);
		if (result < 0) {
			val = 0x00;
		}

		/* Hispeed mode? */
		if (val == 0x01) {
			/* Change Hispeed current */
			mA = FJ_CHG_CURRENT_500;
		}
	}
	fj_chg_drv_source_req(CHG_SOURCE_HOLDER, mA);
#endif
}
EXPORT_SYMBOL_GPL(fj_chg_holder_vbus_draw);

void fj_chg_mhl_vbus_draw(unsigned int onoff)
{
	switch (onoff) {
		case FJ_CHG_ENABLE:
			fj_chg_drv_source_req(CHG_SOURCE_MHL, FJ_CHG_CURRENT_500);
			break;
		case FJ_CHG_TYPE_A:
			fj_chg_drv_current_req(CHG_SOURCE_MHL, FJ_CHG_CURRENT_500);
			break;
		case FJ_CHG_TYPE_B:
			fj_chg_drv_current_req(CHG_SOURCE_MHL, FJ_CHG_CURRENT_900);
			break;
		case FJ_CHG_TYPE_C:
			fj_chg_drv_current_req(CHG_SOURCE_MHL, FJ_CHG_CURRENT_1500);
			break;
		case FJ_CHG_DISABLE:
		default:
			fj_chg_drv_source_req(CHG_SOURCE_MHL, FJ_CHG_CURRENT_OFF);
			break;
	}
}
EXPORT_SYMBOL_GPL(fj_chg_mhl_vbus_draw);

void fj_chg_other_vbus_draw(unsigned int onoff)
{
	unsigned int mA = 0;

	if (onoff != 0) {
		mA = FJ_CHG_CURRENT_500;
	} else {
		mA = FJ_CHG_CURRENT_OFF;
	}
	fj_chg_drv_source_req(CHG_SOURCE_AC, mA);
}
EXPORT_SYMBOL_GPL(fj_chg_other_vbus_draw);

void fj_chg_notify_error(fj_chg_err_type chg_err)
{
	fj_chg_drv_notify_error(chg_err);
}
EXPORT_SYMBOL_GPL(fj_chg_notify_error);

MODULE_AUTHOR("FUJITSU");
MODULE_DESCRIPTION("Charger Driver");
MODULE_LICENSE("GPL");
