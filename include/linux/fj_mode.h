/*
 * Copyright(C) 2014 FUJITSU LIMITED
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
/* FUJITSU LIMITED:2014-10-22 H1510008 add start */
#ifndef __LINUX_FJ_MODE_H
#define __LINUX_FJ_MODE_H

#define FJ_MODE_INVALID         0
#define FJ_MODE_FASTBOOT        1
#define FJ_MODE_SD_DOWNLOADER   2
#define FJ_MODE_KERNEL_MODE     3
#define FJ_MODE_SP_MODE         4
#define FJ_MODE_MAKER_MODE      5
#define FJ_MODE_USB_DOWNLOADER  6
#define FJ_MODE_OFF_CHARGE      7

#define FJ_MODE_NORMAL          0xFF

extern int fj_boot_mode;

#endif /* __LINUX_FJ_MODE_H */
/* FUJITSU LIMITED:2014-10-22 H1510008 add end */