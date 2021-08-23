/*
 * COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2017
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
#ifndef __FJ_CHARGER_LOCAL_H
#define __FJ_CHARGER_LOCAL_H

#undef FJ_CHARGER_LOCAL

#define FJ_CHARGER_HOLDER_UNMOUNT

#define APNV_CHARGE_FG_FUNC_LIMITS_I        (41053)


/* Power Supply name */
#define QPNP_BMS_PSY_NAME					"bms"
#define BCL_FG_ADC_PSY_NAME					"fg_adc"

#define FJ_CHG_USB_PSY_NAME					"fj-usb"
#define FJ_CHG_AC_PSY_NAME					"fj-ac"
#define FJ_CHG_MHL_PSY_NAME					"fj-mhl"
#define FJ_CHG_HOLDER_PSY_NAME				"fj-holder"

#define FJ_CHARGER_PSY_NAME					"fj-chg"
#define FJ_BATTERY_PSY_NAME					"battery"
#define FJ_PM_ADC_PSY_NAME					"fj-pm-adc"
#define FJ_PMI_ADC_PSY_NAME					"fj-pmi-adc"
#define FJ_TEMP_PSY_NAME					"fj-temp"

enum {
	FJ_CHG_SET_LOW = 0,
	FJ_CHG_SET_HIGH,
};

enum {
	CHG_SOURCE_HOLDER = 0,		/* for Cradle     */
	CHG_SOURCE_USB,				/* for USB        */
	CHG_SOURCE_AC,				/* for AC adaptor */
	CHG_SOURCE_MHL,				/* for MHL(USB)   */
	CHG_SOURCE_NUM
};

enum {
	FJ_CHG_CMD_SMBCHG_CHGR_CHGR_STS,					/* 0x0000100E */
	FJ_CHG_CMD_SMBCHG_CHGR_PCC_CFG,						/* 0x000010F1 */
	FJ_CHG_CMD_SMBCHG_CHGR_FCC_CFG,						/* 0x000010F2 */
	FJ_CHG_CMD_SMBCHG_CHGR_FV_CFG,						/* 0x000010F4 */
	FJ_CHG_CMD_SMBCHG_CHGR_CFG_P2F,						/* 0x000010F8 */
	FJ_CHG_CMD_SMBCHG_CHGR_CFG_TCC,						/* 0x000010F9 */
	FJ_CHG_CMD_SMBCHG_CHGR_CCMP_CFG,					/* 0x000010FA */
	FJ_CHG_CMD_SMBCHG_CHGR_CHGR_CFG2,					/* 0x000010FC */
	FJ_CHG_CMD_SMBCHG_CHGR_SFT_CFG,						/* 0x000010FD */
	FJ_CHG_CMD_SMBCHG_BAT_IF_BAT_PRES_STATUS,			/* 0x00001208 */
	FJ_CHG_CMD_SMBCHG_BAT_IF_INT_RT_STS,				/* 0x00001210 */
	FJ_CHG_CMD_SMBCHG_BAT_IF_CMD_CHG,					/* 0x00001242 */
	FJ_CHG_CMD_SMBCHG_BAT_IF_VBL_SEL_CFG,				/* 0x000012F2 */
	FJ_CHG_CMD_SMBCHG_BAT_IF_BM_CFG,					/* 0x000012F3 */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_ICL_STS_1,				/* 0x00001307 */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_HVDCP_STS,				/* 0x0000130C */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_INT_RT_STS,			/* 0x00001310 */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CMD_IL,				/* 0x00001340 */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_ENUM_TIMER_STOP,		/* 0x0000134E */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USBIN_CHGR_CFG,		/* 0x000013F1 */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USBIN_IL_CFG,			/* 0x000013F2 */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_USB_AICL_CFG,			/* 0x000013F3 */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_CFG,					/* 0x000013F4 */
	FJ_CHG_CMD_SMBCHG_USB_CHGPTH_APSD_CFG,				/* 0x000013F5 */
	FJ_CHG_CMD_SMBCHG_DC_CHGPTH_INT_RT_STS,				/* 0x00001410 */
	FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DCIN_CHGR_CFG,			/* 0x000014F1 */
	FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DCIN_IL_CFG,			/* 0x000014F2 */
	FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG1,			/* 0x000014F3 */
	FJ_CHG_CMD_SMBCHG_DC_CHGPTH_DC_AICL_CFG2,			/* 0x000014F4 */
	FJ_CHG_CMD_SMBCHG_DC_CHGPTH_AICL_WL_SEL_CFG,		/* 0x000014F5 */
	FJ_CHG_CMD_SMBCHG_MISC_WDOG_RST,					/* 0x00001640 */
	FJ_CHG_CMD_SMBCHG_MISC_WD_CFG,						/* 0x000016F1 */
	FJ_CHG_CMD_SMBCHG_MISC_CHGR_TRIM_OPTIONS_15_8,		/* 0x000016F5 */
	FJ_CHG_CMD_SMBCHG_MISC_CFG_TEMP_SEL,				/* 0x000016FF */

	FJ_CHG_CMD_NUM,
};

#endif /* __FJ_CHARGER_LOCAL_H */
