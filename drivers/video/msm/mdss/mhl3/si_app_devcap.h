/*------------------------------------------------------------------*/
/* COPYRIGHT(C) FUJITSU LIMITED 2014-2016                           */
/*------------------------------------------------------------------*/
/*
 * SiI8620 Linux Driver
 *
 * Copyright (C) 2013-2014 Silicon Image, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 * This program is distributed AS-IS WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; INCLUDING without the implied warranty
 * of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
 * See the GNU General Public License for more details at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#ifndef _SI_APP_DEVCAP_H_
#define _SI_APP_DEVCAP_H_

#define DEVCAP_VAL_DEV_STATE		0
#define DEVCAP_VAL_MHL_VERSION		MHL_VERSION

/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add start */
#if 0
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add end */
#define DEVCAP_VAL_DEV_CAT		(MHL_DEV_CAT_SOURCE | \
	MHL_DEV_CATEGORY_POW_BIT)
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add start */
#else
#define DEVCAP_VAL_DEV_CAT		(MHL_DEV_CAT_SOURCE)
#endif
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add end */
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add start */
#if 0
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add end */
#define DEVCAP_VAL_ADOPTER_ID_H	(uint8_t)(SILICON_IMAGE_ADOPTER_ID >> 8)
#define DEVCAP_VAL_ADOPTER_ID_L	(uint8_t)(SILICON_IMAGE_ADOPTER_ID & 0xFF)
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add start */
#else
#define DEVCAP_VAL_ADOPTER_ID_H	(uint8_t)(FJ_ADOPTER_ID >> 8)
#define DEVCAP_VAL_ADOPTER_ID_L	(uint8_t)(FJ_ADOPTER_ID & 0xFF)
#endif
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add end */

/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0008 mod start */
#if 0
#define DEVCAP_VAL_VID_LINK_MODE	(MHL_DEV_VID_LINK_SUPP_RGB444 | \
	MHL_DEV_VID_LINK_SUPP_YCBCR422 | MHL_DEV_VID_LINK_SUPP_YCBCR444 | \
	MHL_DEV_VID_LINK_SUPP_PPIXEL | MHL_DEV_VID_LINK_SUPP_ISLANDS | \
	MHL_DEV_VID_LINK_SUPP_VGA | MHL_DEV_VID_LINK_SUPP_16BPP)
#else
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0012 mod start */
#ifdef _ENABLE_MHL3_FEATURE_
#define DEVCAP_VAL_VID_LINK_MODE	(MHL_DEV_VID_LINK_SUPP_RGB444 | \
    MHL_DEV_VID_LINK_SUPP_VGA | MHL_DEV_VID_LINK_SUPP_16BPP)
#else
#define DEVCAP_VAL_VID_LINK_MODE	(MHL_DEV_VID_LINK_SUPP_RGB444 | \
    MHL_DEV_VID_LINK_SUPP_VGA)
#endif
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0012 mod end */
#endif
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0008 mod end */

/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0008 mod start */
#if 0
#define DEVCAP_VAL_AUD_LINK_MODE	(MHL_DEV_AUD_LINK_2CH | \
	MHL_DEV_AUD_LINK_8CH)
#else
#define DEVCAP_VAL_AUD_LINK_MODE	(MHL_DEV_AUD_LINK_2CH)
#endif
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0008 mod end */

#define DEVCAP_VAL_VIDEO_TYPE		0
#define DEVCAP_VAL_LOG_DEV_MAP		MHL_LOGICAL_DEVICE_MAP
#define DEVCAP_VAL_BANDWIDTH		0x0F

/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0008 mod start */
#if 0
#define	DEVCAP_VAL_FEATURE_FLAG_UCP_SEND	MHL_FEATURE_UCP_SEND_SUPPORT
#define	DEVCAP_VAL_FEATURE_FLAG_UCP_RECV	MHL_FEATURE_UCP_RECV_SUPPORT
#else
#define	DEVCAP_VAL_FEATURE_FLAG_UCP_SEND	0
#define	DEVCAP_VAL_FEATURE_FLAG_UCP_RECV	0
#endif
/* FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0008 mod end */

#if (INCLUDE_RBP == 1)
#define	DEVCAP_VAL_FEATURE_FLAG_RBP		MHL_FEATURE_RBP_SUPPORT
#else
#define	DEVCAP_VAL_FEATURE_FLAG_RBP		0
#endif

#define DEVCAP_VAL_FEATURE_FLAG		(MHL_FEATURE_RCP_SUPPORT | \
	MHL_FEATURE_RAP_SUPPORT | \
	MHL_FEATURE_SP_SUPPORT | \
	DEVCAP_VAL_FEATURE_FLAG_UCP_SEND | \
	DEVCAP_VAL_FEATURE_FLAG_UCP_RECV | \
	DEVCAP_VAL_FEATURE_FLAG_RBP) \

#define DEVCAP_VAL_SCRATCHPAD_SIZE	MHL_SCRATCHPAD_SIZE
#define DEVCAP_VAL_INT_STAT_SIZE	MHL_INT_AND_STATUS_SIZE
#define DEVCAP_VAL_RESERVED		0

#define XDEVCAP_VAL_ECBUS_SPEEDS	(MHL_XDC_ECBUS_S_075 | \
	MHL_XDC_ECBUS_S_8BIT | MHL_XDC_ECBUS_S_12BIT)

#define XDEVCAP_VAL_TMDS_SPEEDS		(MHL_XDC_TMDS_150 | \
	MHL_XDC_TMDS_300 | MHL_XDC_TMDS_600)


#endif
