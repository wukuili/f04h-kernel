/*
 * Copyright(C) 2015 FUJITSU LIMITED
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
/* FUJITSU LIMITED:2015-05-28 H1520369 add start */
#ifndef __LINUX_FJ_BTO_H
#define __LINUX_FJ_BTO_H

enum {
	BTO_INFO_ID_RAM = 0,
	BTO_INFO_ID_CAMERA,
	BTO_INFO_ID_NFC,
	BTO_INFO_ID_FINGER_KEY,
	BTO_INFO_ID_FINGER_SENSOR,
	BTO_INFO_ID_WLAN,
	BTO_INFO_ID_USB_HOST,
	BTO_INFO_ID_USB_DTCP
};

enum {
	BTO_INFO_NONE,
	BTO_INFO_DISABLE,
	BTO_INFO_ENABLE
};

#define BTO_INFO_LEN			(17)

extern unsigned char bto_info[BTO_INFO_LEN];

#endif /* __LINUX_FJ_BTO_H */
/* FUJITSU LIMITED:2015-05-28 H1520369 add end */
