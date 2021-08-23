/*
 * COPYRIGHT(C) 2016 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
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

/* FUJITSU:2016-01-19 FJ_CAMERA_CUSTOM start */
#ifdef FJ_CAMERA_CUSTOM // start

#ifndef __FJCAMERA_NONACS_H__
#define __FJCAMERA_NONACS_H__

#define FJCAMERA_BIT0					0x01
#define FJCAMERA_BIT1					0x02
#define FJCAMERA_BIT2					0x04
#define FJCAMERA_BIT3					0x08
#define FJCAMERA_BIT4					0x10
#define FJCAMERA_BIT5					0x20
#define FJCAMERA_BIT6					0x40
#define FJCAMERA_BIT7					0x80

#define	CAMERA_DBG_ON					(u8)1
#define	CAMERA_DBG_OFF					(u8)0

#define CAMERA_DEBUG_FLAGS				16
#define CAMERA_DEBUG_DT_NUM				15

#define CAMERA_LOGLV0					(int)0				/* every time logging...			*/
															/* Very important, Same as error.	*/
															/* Hi Level log.					*/
#define CAMERA_LOGLV1					(int)1				/* Important, Same as warning.		*/
#define CAMERA_LOGLV2					(int)2				/* Not Important, Same as tarce		*/
#define CAMERA_LOGLV3					(int)3				/* Information log					*/

#define NOR_CAMERA_FUNCTION_ON

#ifdef NOR_CAMERA_FUNCTION_ON
#include <linux/nonvolatile_common.h>
#endif // NOR_CAMERA_FUNCTION_ON

/*
 *	[data]
 *		 pt dt1 dt2 dt3 
 *
 *	pt :00:Debug OFF
 *			 01:Debug ON
 *	dt1:Log level enable
 *		\	bit0:KERN_WARNING
 *			bit1:KERN_NOTICE
 *			bit2:KERN_INFO
 *			bit3:KERN_DEBUG
 *	dt2: reserved
 *	dt3: reserved
*/

struct cam_nonvolatile_info {
	u8 debug_i[CAMERA_DEBUG_FLAGS];
	u8 data_io_debug;						/* Debug Log ON/OFF flag */
	u8 cam_loglevel[CAMERA_DEBUG_DT_NUM];	/* Output level */
};

#define	CAM_FJ_BUF_DUMP_LEN		16		/* number of data per line (16 or 32) */
#define	CAM_FJ_BUF_DUMP_LINE	256		/* line buffer size */

static inline void _camera_buf_dump(u8 *dptr, u8 cam_loglevel, int size, const char *data_name, int loglevel)
{
	int ii, nLen = 0;
	u8 tmp_buf[CAM_FJ_BUF_DUMP_LINE];

	if (cam_loglevel < loglevel) {
		return;
	}

	pr_info("[CAMD] dump [%s] size = %d", data_name, size);
	memset(&tmp_buf[0], 0, sizeof(u8) * CAM_FJ_BUF_DUMP_LINE);

	for (ii = 0; ii < size; ii++, dptr++) {
		if ((ii % CAM_FJ_BUF_DUMP_LEN) == 0) {
			nLen = snprintf(&tmp_buf[0], CAM_FJ_BUF_DUMP_LINE, "[CAMD] %08X | ", ii);
		}
		nLen += snprintf(&tmp_buf[nLen], (CAM_FJ_BUF_DUMP_LINE - nLen), "%02X ", *dptr);
		if ((ii % CAM_FJ_BUF_DUMP_LEN) == (CAM_FJ_BUF_DUMP_LEN - 1)) {
			pr_info("%s", &tmp_buf[0]);
			nLen = 0;
			memset(&tmp_buf[0], 0, sizeof(unsigned char) * CAM_FJ_BUF_DUMP_LINE);
		}
	}

	if ((ii % CAM_FJ_BUF_DUMP_LEN) != 0) {
		pr_info("%s", &tmp_buf[0]);
	}
}

static inline void _camera_dump_all_non_info(struct cam_nonvolatile_info *nv_info)
{
	int i = 0;
	pr_info("[CAMD]%s :[IN]\n", __func__);

	pr_info("[CAMD] data_io_debug = %d\n", nv_info->data_io_debug);
	for (i = 0; i < CAMERA_DEBUG_DT_NUM; i++) {
		pr_info("[CAMD] cam_loglevel[%d] = 0x%02x\n", i, nv_info->cam_loglevel[i]);
	}

	pr_info("[CAMD]%s :[OUT]\n", __func__);
}

static inline void _camera_set_all_init_info(struct cam_nonvolatile_info *nv_info)
{
	/* set all default info */
	nv_info->data_io_debug	= CAMERA_DBG_OFF;
	memset(&(nv_info->cam_loglevel), 0x00, sizeof(u8)*CAMERA_DEBUG_DT_NUM);
}

static inline void _camera_set_all_non_info(struct cam_nonvolatile_info *nv_info)
{
	int i = 0;
	/* set all NOR read info */
	nv_info->data_io_debug = (nv_info->debug_i[0] & FJCAMERA_BIT0) ? CAMERA_DBG_ON : CAMERA_DBG_OFF;
	for (i = 0; i < CAMERA_DEBUG_DT_NUM; i++) {
		nv_info->cam_loglevel[i] = nv_info->debug_i[i + 1] & (FJCAMERA_BIT0 | FJCAMERA_BIT1 | FJCAMERA_BIT2 | FJCAMERA_BIT3);
	}
}

static inline int _camera_get_all_non_info(struct cam_nonvolatile_info *nv_info)
{
	int retval = 0;
	int i = 0;
	pr_info("[CAMD]%s :[IN]\n", __func__);

	memset(&nv_info->debug_i[0], 0, CAMERA_DEBUG_FLAGS);

#ifdef NOR_CAMERA_FUNCTION_ON
	retval = get_nonvolatile(&nv_info->debug_i[0], 0xBFAE, CAMERA_DEBUG_FLAGS);
#else // NOR_CAMERA_FUNCTION_ON
	retval = -1; /* fujitsu nor function is not support*/
#endif // NOR_CAMERA_FUNCTION_ON

	if (retval < 0) {
		pr_info("[CAMD]%s:set default value \n", __func__);
		_camera_set_all_init_info(nv_info);
	} else {
		_camera_set_all_non_info(nv_info);
		pr_info("[CAMD] debug = %d \n", nv_info->data_io_debug);
		for (i = 0; i < CAMERA_DEBUG_DT_NUM; i++) {
			//pr_info("[CAMD] loglevel[%d]=0x%02X \n", i, nv_info->cam_loglevel[i]);
		}
	}

#if 0
	_camera_dump_all_non_info(nv_info);

	if (nv_info->data_io_debug == CAMERA_DBG_ON) {
		for (i = 0; i < CAMERA_DEBUG_DT_NUM; i++) {
			_camera_buf_dump(&nv_info->debug_i[0], nv_info->cam_loglevel[i], CAMERA_DEBUG_FLAGS, "0xBFAE", CAMERA_LOGLV0);
		}
	}
#endif

	pr_info("[CAMD]%s :[OUT]\n", __func__);

#ifdef NOR_CAMERA_FUNCTION_ON
	return retval;
#else // NOR_CAMERA_FUNCTION_ON
	return retval = 0;
#endif // NOR_CAMERA_FUNCTION_ON
}

extern struct cam_nonvolatile_info cam_nv_info;

#ifndef CAMERA_LOG_TAG
#error CAMERA_LOG_TAG is undefined
#endif // CAMERA_LOG_TAG

#ifndef CAMERA_NV_INDEX
#error CAMERA_NV_INDEX is undefined
#endif // CAMERA_NV_INDEX

#define LOGD(fmt, args...)																			\
	((cam_nv_info.data_io_debug) && (cam_nv_info.cam_loglevel[CAMERA_NV_INDEX] & FJCAMERA_BIT3)		\
	? printk(KERN_DEBUG CAMERA_LOG_TAG fmt, ##args)													\
	: (void)0)																						\

#define LOGI(fmt, args...)																			\
	((cam_nv_info.data_io_debug) && (cam_nv_info.cam_loglevel[CAMERA_NV_INDEX] & FJCAMERA_BIT2)		\
	? printk(KERN_INFO CAMERA_LOG_TAG fmt, ##args)													\
	: (void)0)																						\

#define LOGN(fmt, args...)																			\
	((cam_nv_info.data_io_debug) && (cam_nv_info.cam_loglevel[CAMERA_NV_INDEX] & FJCAMERA_BIT1)		\
	? printk(KERN_NOTICE CAMERA_LOG_TAG fmt, ##args)												\
	: (void)0)																						\

#define LOGW(fmt, args...)																			\
	((cam_nv_info.data_io_debug) && (cam_nv_info.cam_loglevel[CAMERA_NV_INDEX] & FJCAMERA_BIT0)		\
	? printk(KERN_WARNING CAMERA_LOG_TAG fmt, ##args)												\
	: (void)0)																						\

#define LOGE(fmt, args...) printk(KERN_ERR CAMERA_LOG_TAG fmt, ##args)

//#define CONFIG_CAMERA_DEBUG
#undef CDBG
#ifdef CAMERA_CDBG_ENABLE
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#endif // __FJCAMERA_NONACS_H__

#endif // FJ_CAMERA_CUSTOM end
/* FUJITSU:2016-01-19 FJ_CAMERA_CUSTOM end */
