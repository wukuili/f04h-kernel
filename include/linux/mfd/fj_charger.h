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
#ifndef __FJ_CHARGER_H
#define __FJ_CHARGER_H

/* mA */
#define FJ_CHG_CURRENT_100      100
#define FJ_CHG_CURRENT_150      150
#define FJ_CHG_CURRENT_300      300
#define FJ_CHG_CURRENT_500      500
#define FJ_CHG_CURRENT_700      700
#define FJ_CHG_CURRENT_900      900
#define FJ_CHG_CURRENT_1200    1200
#define FJ_CHG_CURRENT_1500    1500
#define FJ_CHG_CURRENT_1600    1600
#define FJ_CHG_CURRENT_1800    1800
#define FJ_CHG_CURRENT_2000    2000
#define FJ_CHG_CURRENT_2100    2100
#define FJ_CHG_CURRENT_2200    2200
#define FJ_CHG_CURRENT_2500    2500
#define FJ_CHG_CURRENT_3000    3000
#define FJ_CHG_CURRENT_OFF        0

#define FJ_CHG_USB_CURRENT     FJ_CHG_CURRENT_500
#define FJ_CHG_AC_CURRENT      FJ_CHG_CURRENT_1500
#define FJ_CHG_HOLDER_CURRENT  FJ_CHG_CURRENT_1500
#define FJ_CHG_MHL_CURRENT     FJ_CHG_CURRENT_500
#define FJ_CHG_OFF_CURRENT     FJ_CHG_CURRENT_OFF

/* notify error */
#define FJ_CHG_ERROR_NONE      0  // no error detection
#define FJ_CHG_ERROR_DETECT    1  // error detection

typedef enum {
	FJ_CHG_ERROR_OVP_NONE = 0,
	FJ_CHG_ERROR_OVP_DETECT,
	FJ_CHG_ERROR_USB_HOT_NONE,
	FJ_CHG_ERROR_USB_HOT_DETECT,

	FJ_CHG_ERROR_UNKNOWN_ID_NONE,
	FJ_CHG_ERROR_UNKNOWN_ID_DETECT
} fj_chg_err_type;

enum {
	FJ_CHG_CHARGE_MODE_NORMAL = 0,
	FJ_CHG_CHARGE_MODE_POWERPATH,
};

#define FJ_CHG_DISABLE         0  // disable charging
#define FJ_CHG_ENABLE          1  // enable charging
#define FJ_CHG_TYPE_A          2  // 500mA
#define FJ_CHG_TYPE_B          3  // 900mA
#define FJ_CHG_TYPE_C          4  // 1500mA
#define FJ_CHG_TYPE_D          5  // 100mA (not used)

#define APNV_CHG_DESKTOP_FOLDER_CHARGE_I	(41042)
#define APNV_CHARGE_FG_RCOMP0_I             (47140)
#define APNV_CHARGE_FG_TEMPCO_I             (47141)
#define APNV_CHARGE_FG_FULLCAP_I            (47142)
#define APNV_CHARGE_FG_CYCLES_I             (47143)
#define APNV_CHARGE_FG_FULLCAPNOM_I         (47144)
#define APNV_CHARGE_FG_IAVG_EMPTY_I         (47145)
#define APNV_CHARGE_FG_QRTABLE00_I          (47146)
#define APNV_CHARGE_FG_QRTABLE10_I          (47147)
#define APNV_CHARGE_FG_QRTABLE20_I          (47148)
#define APNV_CHARGE_FG_QRTABLE30_I          (47149)
#define APNV_CHARGE_FG_AGE_RES_I            (47150)
#define APNV_CHARGE_FG_VF_COUNT_I           (49003)
#define APNV_CHARGE_FG_AGE_I                (49004)

/* fj_charger Prototype */
extern void fj_chg_usb_vbus_draw(unsigned int onoff);
extern void fj_chg_ac_vbus_draw(unsigned int onoff);
extern void fj_chg_holder_vbus_draw(unsigned int onoff);
extern void fj_chg_mhl_vbus_draw(unsigned int onoff);
extern void fj_chg_other_vbus_draw(unsigned int onoff);

extern void fj_chg_notify_error(fj_chg_err_type chg_err);

#endif /* __FJ_CHARGER_H */
