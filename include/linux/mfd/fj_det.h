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
/* FCNT LIMITED:2016-02-12 H16100739 add start */
#ifndef __FJ_DET_H
#define __FJ_DET_H

typedef enum {
	FJ_DET_NO_DEVICE = 0,
	FJ_DET_DEVICE_OTG,
	FJ_DET_DEVICE_1K,
	FJ_DET_DEVICE_36_5K,
	FJ_DET_DEVICE_68K,
	FJ_DET_DEVICE_124K,
	FJ_DET_DEVICE_180K,
	FJ_DET_DEVICE_390K,
	FJ_DET_DEVICE_1M,
	FJ_DET_DEVICE_UNKNOWN
} FJ_DET_ID_TYPE;

extern FJ_DET_ID_TYPE fj_det_get_device_type(void);

#endif /* __FJ_DET_H */
/* FCNT LIMITED:2016-02-12 H16100739 add end */
