/**************************************************************************//**
 *
 *  @file		tuner_drv_config.h
 *
 *  @brief		configuration header of the mm_tuner55x driver
 *
 *  @data		2011.08.28
 *
 *  @author	K.Kitamura(*)
 *  @author	K.Okawa(KXDA3)
 *
 ****************************************************************************//*
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************/
/*----------------------------------------------------------------------------*/
// tuner_drv_config.h COPYRIGHT FUJITSU CONNECTED TECHNOLOGIES LIMITED 2015-2017
/*----------------------------------------------------------------------------*/
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/
#ifndef _TUNER_DRV_CONFIG_H
#define _TUNER_DRV_CONFIG_H

/******************************************************************************
 * define
 ******************************************************************************/
#define TUNER_SET_ON                     1       /* setting ON */
#define TUNER_SET_OFF                    0       /* setting OFF */

/* device driver file name */
#define TUNER_CONFIG_DRIVER_NAME		"mmtuner_drv"

/* device number */
//FUJITSU:2015-05-18 For Socionext tuner Modify -S
/* âºçÃî‘ */
#define TUNER_CONFIG_DRV_MAJOR         0 //240       /* MAJOR No. */
//FUJITSU:2015-05-18 For Socionext tuner Modify -E
#define TUNER_CONFIG_DRV_MINOR         200       /* MINOR No. */

/* compile switch for IRQ */
/* #define TUNER_CONFIG_IRQ_PC_LINUX */

/* IRQ */
//#define TUNER_CONFIG_IRQ_ENABLE					/* System IRQ Enable */
#ifdef PANDABOARD
#define TUNER_CONFIG_INT					gpio_to_irq(0x27)	/* GPIO port# 0x27 */
#else
//FUJITSU:2015-05-18 For Socionext tuner Modify -S
#define TUNER_CONFIG_INT					108        /* IRQ No. */
//FUJITSU:2015-05-18 For Socionext tuner Modify -E
#endif

/* kernel thread priority  */
#define TUNER_CONFIG_KTH_PRI            95       /* priority (0-99) */

/* kernel thread priority  */
#define TUNER_CONFIG_TSBTH_PRI          95       /* priority (0-99) */

/* exclusive access control  */
/* #define TUNER_CONFIG_DRV_MULTI */

//FUJITSU:2015-05-18 For Socionext tuner Modify -S
#define TUNER_CONFIG_I2C_BUSNUM			11		/* I2C Bus number */
//FUJITSU:2015-05-18 For Socionext tuner Modify -E

#endif/* _TUNER_DRV_CONFIG_H */
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
