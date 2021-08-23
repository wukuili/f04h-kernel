/***************************************************************************//**
 *
 *  @file		version.h
 *
 *  @brief		define version of mm_tuner driver
 *
 *  @data		2015.04.30
 *
 *  @author	K.Okawa(IoT1)
 *
 ****************************************************************************//*
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/
#ifndef _MMTUNER_VERSION_H
#define _MMTUNER_VERSION_H

#ifndef MMTUENR_DEVICE
#define MMTUNER_DEVICE	(0x53)	//!< Target device id. (MN885xx)
#endif
#ifndef MMTUNER_MAJOR
#define MMTUNER_MAJOR	(0)
#endif
#ifndef MMTUNER_MINOR
#define MMTUNER_MINOR	(1)
#endif
#ifndef MMTUNER_HOTFIX
#define MMTUNER_HOTFIX	(3)
#endif
/*
 * MMTUNER_RC is the release candidate suffix.
 * Should normally be empty.
 */
#ifndef MMTUNER_RC
#define MMTUNER_RC		""
#endif

#ifndef MMTUNER_DESC
#define MMTUNER_DESC		"A2"
#endif

#endif
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
