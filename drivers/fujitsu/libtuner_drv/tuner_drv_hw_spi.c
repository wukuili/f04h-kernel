/**************************************************************************//**
 *
 *  @file		tuner_drv_hw_spi.c
 *
 *  @brief		Implementation of the hardware control layer in SPI.
 *
 *  @data		2014.08.19
 *
 *  @author	K.Okawa (KXDA3)
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
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/
/******************************************************************************
 * include
 ******************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/spi/spi.h>

#include "tuner_drv.h"
#include "tuner_drv_hw.h"

/******************************************************************************
 * function
 ******************************************************************************/
static int tuner_drv_spi_probe(struct spi_device *spidev);
static int tuner_drv_spi_remove(struct spi_device *spidev);

//local
int tuner_drv_spi_idle(void);
int tuner_drv_spi_calibration(void);

#define SPI_CMD_NUM 0x8
#define SPI_DATA_NUM 0x8
#define SPI_PSEQ_ADRS_INIT 0x00

//#define TUNER_CONFIG_TSIF_EXT
//#define TUNER_CONFIG_TSIF_EDGE

/******************************************************************************
 * data
 ******************************************************************************/
static struct spi_driver mn8855x_spi_driver = {
		.driver = {
				.name = "tmm2spi",
				.owner = THIS_MODULE,
		},
		.probe = tuner_drv_spi_probe,
		.remove = tuner_drv_spi_remove,
};

struct spi_drvdata {
	struct spi_device *spi;
	spinlock_t spi_lock;
};

static struct spi_drvdata	*g_spi_drvdata = NULL;
extern size_t g_ts_pkt_size[3];

/******************************************************************************
 * code area
 ******************************************************************************/
#ifdef CPATH_I2C
/**************************************************************************//**
 * Set TPM register
 *
 * The TPM (register of the tuner device) control the I/F port of
 * the tuner device.
 * TPM must be set to 0x02 when the Data-PATH (TS I/F) use SPI and
 * Control-PATH use I2C.
 *
 * @date	2014.09.10
 *
 * @author K.Okawa (KXDA3)
 *
 * @retval 0			Normal end
 * @retval <0			error
 ******************************************************************************/
int tuner_drv_hw_tsif_set_tpm(void)
{
	int ret = 0;
	TUNER_DATA_RW rw[1];

	memset(rw, 0, sizeof(TUNER_DATA_RW));

	/* TPM[3:0] (PINCNT0[3:0]) */
	rw[0].sngl.bank = Main2;
	rw[0].sngl.adr = 0xE8;
	rw[0].sngl.enabit = 0x0F;
	rw[0].sngl.param = 0x02; /* CPATH:I2C, DPATH:SPI(slave-IF) */
	ret = tuner_drv_write_regs(rw, 1);
	if (ret) {
		ERROR_PRINT("writing TPM fail");
		return ret;
	}
	return ret;
}
#else
/**************************************************************************//**
 * address incremental read from some registers of the Tuner device.
 *
 * @date	2014.11.11
 *
 * @author K.Fukuzaki (KXDA3)
 *
 * @retval 0			Normal end
 * @retval <0			error
 *
 * @param [in] bank	register bank enumerator
 * @param [in] adr	address of the register to read-out start
 * @param [in] len	continuous read length
 * @param [out] rd	pointer to the buffer
 ******************************************************************************/
int tuner_drv_hw_read_reg(enum _reg_bank bank, uint8_t adr, uint16_t len, uint8_t *rd)
{
  int ret = 0;
  struct spi_message  msg;
  struct spi_transfer xfer;

  unsigned short     loop_cnt;
  unsigned char      read_data;
  
  unsigned char scmd[SPI_CMD_NUM];
  unsigned char sdata[SPI_DATA_NUM];

  int i;
  uint8_t n = 0;
  
  INFO_PRINT("%s START", __FUNCTION__);

  if( NULL == g_spi_drvdata ) {
    ERROR_PRINT("[%s](%d) ERR:can't spi read. g_spi_drvdata NULL",__FUNCTION__,__LINE__);
    return -1;
  }
  memset(&xfer, 0, sizeof(struct spi_transfer));
  spi_message_init(&msg);

  /* access loop */
  for( loop_cnt = 0; loop_cnt < len; loop_cnt++ ) {
    memset( scmd, 0x00, sizeof(unsigned char) * SPI_CMD_NUM );
    memset( sdata, 0x00, sizeof(unsigned char) * SPI_DATA_NUM );

    //command(read)
    scmd[0] = 0x03;
    //bank select
    if     (bank==Sub  ){scmd[1] = 0x90;}
    else if(bank==Main1){scmd[1] = 0x91;}
    else if(bank==Main2){scmd[1] = 0x92;}
    else {
      ERROR_PRINT("[%s](%d) ERR: Bank address is not correct.",__FUNCTION__,__LINE__);
      return -1;
    }
    scmd[2] = (unsigned char)adr + n;
    //dummy data
    for(i=3; i<8; i++) {
      scmd[i] = 0x00;
    }
    xfer.tx_buf = (void *)scmd;
    xfer.len = 8;
    xfer.bits_per_word = 8;
    xfer.rx_buf = (void *)sdata;

    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(g_spi_drvdata->spi, &msg);
    if (ret) {
      ERROR_PRINT("spi_sync() return with %d", ret);
      return ret;
    }

    //read data
    read_data = sdata[7]; //MOSI = {0x72, xx, xx, xx, RD, RD, RD, RD}
    *(rd + n) = read_data;

    n++;
  }
  return ret;
}

/**************************************************************************//**
 * address incremental write to some registers of the Tuner device.
 *
 * @date	2014.11.11
 *
 * @author K.Fukuzaki (KXDA3)
 *
 * @retval 0					Normal end
 * @retval <0					error (refer the errno)
 *
 * @param [in] bank	register bank enumerator
 * @param [in] adr	start address for continuous write
 * @param [in] len	continuous write length
 * @param [out] wd	pointer to the write data array
 ******************************************************************************/
int tuner_drv_hw_write_reg(enum _reg_bank bank, uint8_t adr, uint16_t len, uint8_t *wd)
{
  int ret = 0;
  struct spi_message  msg;
  struct spi_transfer xfer;

  unsigned short     loop_cnt;
  unsigned char      write_data;
  
  uint8_t scmd[SPI_CMD_NUM];
  uint8_t sdata[SPI_DATA_NUM];

  int i;
  uint8_t n = 0;
  
  INFO_PRINT("%s START", __FUNCTION__);

  if( NULL == g_spi_drvdata ) {
    ERROR_PRINT("[%s](%d) ERR:can't spi read. g_spi_drvdata NULL",__FUNCTION__,__LINE__);
    return -1;
  }
  memset(&xfer, 0, sizeof(struct spi_transfer));
  spi_message_init(&msg);

  /* access loop */
  for( loop_cnt = 0; loop_cnt < len; loop_cnt++ ) {
    memset( scmd, 0x00, sizeof(unsigned char) * SPI_CMD_NUM );
    memset( sdata, 0x00, sizeof(unsigned char) * SPI_DATA_NUM );

    //command
    scmd[0] = 0x03;
    //bank select
    if     (bank==Sub  ){scmd[1] = 0x80;}
    else if(bank==Main1){scmd[1] = 0x81;}
    else if(bank==Main2){scmd[1] = 0x82;}
    else {
      ERROR_PRINT("[%s](%d) ERR: Bank address is not correct.",__FUNCTION__,__LINE__);
      return -1;
    }
    scmd[2] = (unsigned char)adr + n;
    //write data
    write_data = *(wd + n);
    for(i=3; i<8; i++) {
      scmd[i] = write_data;
    }

    xfer.tx_buf = (void *)scmd;
    xfer.len = 8;
    xfer.bits_per_word = 8;
    xfer.rx_buf = (void *)sdata;

    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(g_spi_drvdata->spi, &msg);
    if (ret) {
      ERROR_PRINT("spi_sync() return with %d", ret);
      return ret;
    }
    n++;
  }
  return ret;
}

#endif /* CPATH_I2C */

/**************************************************************************//**
 * Register the TS I/F driver
 *
 * @date	2013.12.10
 *
 * @author T.Abe (FSI)
 * @author K.Okawa (KXDA3)
 *
 * @retval 0					Normal end
 * @retval <0					error
 ******************************************************************************/
int tuner_drv_hw_tsif_register(void)
{
	int ret = 0;

	INFO_PRINT("%s START", __FUNCTION__);

	ret = spi_register_driver(&mn8855x_spi_driver);
	if (ret) {
		TRACE();
		return ret;
	}

	INFO_PRINT("%s END", __FUNCTION__);

	return ret;
}

/**************************************************************************//**
 * Configure the SPI-Slave I/F of the tuner device.
 *
 * @date	2013.12.10
 *
 * @author T.Abe (FSI)
 * @author K.Okawa (KXDA3)
 *
 * @retval 0					Normal end
 * @retval <0					error
 *
 * @param [in]
 ******************************************************************************/
int tuner_drv_hw_tsif_config(struct _tsif_cntxt *tc)
{
	int ret = 0;
	uint8_t slvifth;
	TUNER_DATA_RW rw[8];
	TUNER_DATA_TSIF *tsif;
	TUNER_DATA_EVENT iberint_s;
	TUNER_DATA_EVENT iberint_f;
	uint8_t tx[2] = { 0x01, 0xb1 };
	struct spi_message msg;
	struct spi_transfer xfer;
	int i = 0;

	INFO_PRINT("%s START", __FUNCTION__);

#ifdef CPATH_I2C
	ret = tuner_drv_hw_tsif_set_tpm();
	if (ret) {
		TRACE();
		return ret;
	}
#endif

	if (tc == NULL) {
		TRACE();
		return -EINVAL;
	}
	if (tc->tsif == NULL) {
		TRACE();
		return -EINVAL;
	}

	tsif = (TUNER_DATA_TSIF *)tc->tsif;

	/* configure the SPI(slave) I/F sub-system of Tuner device */
	memset(&xfer, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);

	xfer.tx_buf = (void *)tx;
	xfer.len = 2;
	xfer.bits_per_word = 8;
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	if (ret) {
		ERROR_PRINT("spi_sync() return with %d", ret);
		return ret;
	}

	/* IBERINT_S */
	iberint_s.pack = 0;
	iberint_s.set.bw = TUNER_DRV_BW1;
	iberint_s.set.mode = TUNER_EVENT_MODE_ADD;
	iberint_s.set.intdef1 = 0x80;	/* IBERINT */
	iberint_s.set.intset1 = 0x09;	/* NINTEN, INTMD = 1 */

	/* IBERINT_F */
	iberint_f.pack = 0;
	iberint_f.set.bw = TUNER_DRV_BW13;
	iberint_f.set.mode = TUNER_EVENT_MODE_ADD;
	iberint_f.set.intdef1 = 0x80;	/* IBERINT */
	iberint_f.set.intset1 = 0x09;	/* NINTEN, INTMD = 1 */

	/* SLVIFTH */
	slvifth = ((uint8_t)(tsif->thu[tc->bw]) << 4) | tsif->thl[tc->bw];

	/* STRSET[2:0] (SLVIFSTRSET[2:0]) */
	rwSnglSet(rw[0], Main2, 0xEF, 0x07, 0x01);	/* SYNC condition */
	/* STM_SYNCSEL_F[4:0] (DOSET4_F[4:0]) = 0x01 */
	rwSnglSet(rw[1], Main1, 0xD9, 0x1F, 0x01);
	/* STM_SYNCSEL_S[6:0] (DOSET3_S[6:0]) = 0x01 */
	rwSnglSet(rw[2], Main2, 0x02, 0x7F, 0x01);
	/* SLVIFTHU[3:0], SLVIFTHL[3:0] (SLVIFTH[7:0]) */
	rwSnglSet(rw[3], Main2, 0xEE, 0xFF, slvifth);
	/* SLVINTEN (EXTINTSET[5]) = 0x1 */
	rwSnglSet(rw[4], Main2, 0xED, 0x20, 0x20);
	/* ISEGSEL_F[3:0] (INTSET5_F[7:4]) = 0x9 */
	rwSnglSet(rw[5], Main1, 0xE2, 0xF0, 0x90);
	/* ISEGSEL_S[3:0] (INTSET5_S[7:4]) = 0x9 */
	rwSnglSet(rw[6], Main2, 0xCD, 0xF0, 0x90);
	/* NDRAMATU (DRAMCNT1[0]) = 0x1 */
	rwSnglSet(rw[7], Sub, 0xB1, 0x01, 0x01);

	i = (tc->bw == TUNER_DRV_BW1) ?  8 : 7;

	ret = tuner_drv_write_regs(rw, i);
	if (ret) {
		TRACE();
		return ret;
	}

	ret = tuner_drv_setev(&iberint_s);
	if (ret) {
		TRACE();
		return ret;
	}
	ret = tuner_drv_setev(&iberint_f);
	if (ret) {
		TRACE();
		return ret;
	}

	INFO_PRINT("%s END", __FUNCTION__);
	return 0;
}

/**************************************************************************//**
 * Unregister the TS I/F driver
 *
 * @date	2014.08.21
 *
 * @author K.Okawa (KXDA3)
 *
 ******************************************************************************/
void tuner_drv_hw_tsif_unregister(void)
{
	INFO_PRINT("%s START", __FUNCTION__);

	spi_unregister_driver(&mn8855x_spi_driver);

	INFO_PRINT("%s END", __FUNCTION__);
}

/**************************************************************************//**
 * Set TS I/F context
 *
 * This function detect active OFDM circuit, and calculate suitable
 * memory size for handling TS data.
 * After execution, the following member variables of "_tsif_context"
 * structure are update.
 *		bw: active OFDM circuit
 *		ts_packet_size: byte num of an TS packet
 *		ts_record_size: RX transfer size a transaction.
 *		ts_pktbuf_size: buffer size stored by TS I/F thread
 *
 * @date	2014.08.22
 *
 * @author K.Okawa (KXDA3)
 *
 * @retval 0			Normal end
 * @retval <0			error
 *
 * @param [out] ptscnt	pointer to TS I/F context structure
 ******************************************************************************/
int tuner_drv_hw_tsif_set_cntxt(struct _tsif_cntxt *tc)
{
	int ret = 0;
	TUNER_DATA_RW rw[1];

	if (tc == NULL) {
		TRACE();
		return -EINVAL;
	}
	if (tc->tsif == NULL) {
		TRACE();
		return -EINVAL;
	}
	if (!tc->tsifth_wait) {
		TRACE();
		return -EINPROGRESS;
	}

	/* detect active OFDM circuit */
	memset(rw, 0x00, sizeof(TUNER_DATA_RW));
	rwSnglSet(rw[0], Main1, 0xED, 0x00, 0x00);
	ret = tuner_drv_read_regs(rw, 1);
	if (ret) {
		TRACE();
		return ret;
	}
	tc->bw = ((rw[0].sngl.param & 0xC0) == 0x40) ? TUNER_DRV_BW1 : TUNER_DRV_BW13;

	/* TS record size a RX transaction */
	/* Readable packet number, when DATAREADY is high. */
	tc->ts_rxpkt_num = 32 * (tc->tsif->thl[tc->bw] + 1);
	tc->ts_rx_size = tc->ts_rxpkt_num * g_ts_pkt_size[tc->tsif->ts_pkt_type];

	/* TS buffer size for the TS I/F thread */
	tc->ts_pktbuf_size = tc->ts_rx_size * 48;

	return 0;
}

/**************************************************************************//**
 * Get the DATAREADY flag
 *
 * This function return the DATAREADY flag of the Slave-I/F of
 * tuner device. DATAREADY flag contain OVER/UNER-Run indicator.
 * It is below there bit position.
 * OVER-Run is bit-2. UNDER-Run is bit-1, DATA-Ready is bit-0.
 *
 * @date	2014.08.27
 *
 * @author K.Okawa (KXDA3)
 *
 * @retval >=0		DATAREADY flag (casted from uint8_t)
 * @retval <0			error
 ******************************************************************************/
int tuner_drv_hw_tsif_get_dready(void)
{
	int ret;
	uint8_t tx[5] = { 0x03, 0x10, 0x00, 0x00, 0x00 };
	uint8_t rx[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
	struct spi_message msg;
	struct spi_transfer xfer;

//	INFO_PRINT("%s START", __FUNCTION__);

	memset(&xfer, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);

	xfer.tx_buf = (void *)tx;
	xfer.rx_buf = (void *)rx;
	xfer.len = 5;
	xfer.bits_per_word = 8;

	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	if (ret) {
		ERROR_PRINT("spi_sync() return with %d", ret);
		return ret;
	}

//	INFO_PRINT("%s END (DATAREADY=0x%02x)", __FUNCTION__, rx[4]);

	return (int)rx[4];
}

/**************************************************************************//**
 * Send the transaction command to synchronize slave I/F of tuner.
 *
 * This function send the packet synchronization command.
 * It initialize the read pointer and clear the FIFO buffer.
 *
 * @date	2014.08.27
 *
 * @author K.Okawa (KXDA3)
 *
 * @retval 0		normal
 * @retval <0		error
 ******************************************************************************/
int tuner_drv_hw_tsif_sync_pkt(void)
{
	int ret = 0;
	uint8_t tx[4] = { 0xd8, 0x00, 0x00, 0x00 };
	struct spi_message msg;
	struct spi_transfer xfer;

	INFO_PRINT("%s START", __FUNCTION__);

	memset(&xfer, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);

	xfer.tx_buf = (void *)tx;
	xfer.len = 4;
	xfer.bits_per_word = 8;

	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	if (ret) {
		ERROR_PRINT("spi_sync() return with %d", ret);
		return ret;
	}

//	INFO_PRINT("%s END", __FUNCTION__);

	return 0;
}

/**************************************************************************//**
 * Get the TS packets of the appointed number.
 *
 * @date	2014.08.26
 *
 * @author K.Okawa (KXDA3)
 *
 * @retval >=0		Normal end (number of the get packet)
 * @retval <0			error (refer the errno)
 *
 * @param [in] num			num of packets
 * @param [out] pktbuf		packet storage
 * @param [in] pktsize		packet size enumerator
 ******************************************************************************/
int tuner_drv_hw_tsif_get_pkts(struct _tsif_cntxt *tc)
{
	int ret;
//	/* TS packet size: 188Byte, num of TS packets:256 */
//	uint8_t tx[5] = { 0x0b, 0x00, 0x00, 0xFF, 0x00 };
	struct spi_message msg;
	struct spi_transfer xfer[2];
	int sum = 0;
	TUNER_DATA_TSIF *tsif = tc->tsif;
	unsigned int ts_rdelay = 0;

#ifndef TUNER_CONFIG_TSIF_EXT  //Normal TS read
	/* TS packet size: 188Byte, num of TS packets:256 */
	uint8_t tx[5] = { 0x0b, 0x00, 0x00, 0xFF, 0x00 };
#else  //Extend TS read
#define SPI_TSREAD_DMYCNT 0xf  //EXTRACTL[3:0]
	uint8_t tx[5] = { 0x3b, 0x00, 0x0F, 0xFF, 0x00 };
	ts_rdelay = ts_rdelay + SPI_TSREAD_DMYCNT;
#endif  //TUNER_CONFIG_TSIF_EXT

#ifdef TUNER_CONFIG_TSIF_EDGE
	ts_rdelay = ts_rdelay + 1;  //EDGE==1
#endif  //TUNER_CONFIG_TSIF_EDGE

#ifdef TUNER_CONFIG_TSIF_EXT
	tuner_drv_spi_calibration();
#endif  //TUNER_CONFIG_TSIF_EXT

//	INFO_PRINT("%s(tc=%p tsif=%p) START", __FUNCTION__, tc, tsif);

	if (!g_spi_drvdata) {
		TRACE();
		return -ENXIO;
	}
	if (!tc->pktbuf) {
		TRACE();
		return -EINVAL;
	}
	if (!tc->tsif->ts_pkt_type == TUNER_DRV_TS_TSTAMP) {
		ERROR_PRINT("not support the Time-Stamp TS");
		return -EINVAL;
	}

	tx[2] = (uint8_t)(tc->tsif->ts_pkt_type == TUNER_DRV_TS_NORMAL) ? 0x00 : 0x01;

	do {
		unsigned int rxnum = tc->ts_rxpkt_num - sum;

		rxnum = (rxnum <= 256) ? (rxnum) : (256);

		memset(xfer, 0, sizeof(xfer));
		spi_message_init(&msg);

		tx[3] = rxnum - 1;
		xfer[0].tx_buf = (void *)tx;
		xfer[0].len = 5;
		xfer[0].bits_per_word = 8;
		spi_message_add_tail(&xfer[0], &msg);

		xfer[1].rx_buf = (void *)(tc->pktbuf + tc->pwr);
		xfer[1].len = rxnum * g_ts_pkt_size[tc->tsif->ts_pkt_type] + ts_rdelay;  //add ts_rdelay
		xfer[1].bits_per_word = tsif->spi_ts_bit_per_word;
		spi_message_add_tail(&xfer[1], &msg);

		ret = spi_sync(g_spi_drvdata->spi, &msg);
		if (ret) {
			ERROR_PRINT("spi_sync() return with %d", ret);
			return ret;
		} else if (tsif->spi_ts_bit_per_word == 32) {
			/* exchange endian */
			int i;
			uint8_t *p = tc->pktbuf + tc->pwr;
			for (i=0; i<xfer[1].len; i+=4) {
				*((long *)(p+i)) =
						(long)(((p+i)[0] << 24) |
								((p+i)[1] << 16) |
								((p+i)[2] << 8) |
								(p+i)[3]);
			}
		}

		tc->pwr += rxnum * g_ts_pkt_size[tc->tsif->ts_pkt_type];
		if (tc->pwr == tc->ts_pktbuf_size) {
			tc->pwr = 0;
		}

		sum += rxnum;

	} while (sum != tc->ts_rxpkt_num);

//	INFO_PRINT("%s", __FUNCTION__);

	return sum;
}

/**************************************************************************//**
 * probe function called by spi_register_driver()
 *
 * @date	2013.12.10
 *
 * @author T.Abe (FSI)
 * @author K.Okawa (KXDA3)
 *
 * @retval 0			Normal end
 * @retval <0			error (refer the errno)
 *
 * @param [in] spidev	pointer to the "spi_device" structure
 ******************************************************************************/
static int tuner_drv_spi_probe(struct spi_device *spidev)
{
	int ret = 0;
	struct spi_drvdata *drvdata;

	INFO_PRINT("%s START", __FUNCTION__);

	if (g_spi_drvdata != NULL) {
		TRACE();
		return -EBUSY;
	}
	if (NULL == spidev) {
		TRACE();
		return -EINVAL;
	}
	if (NULL == spidev->dev.platform_data) {
		TRACE();
		//return -EINVAL;
	}

	if (!(drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL))) {
		TRACE();
		return -ENOMEM;
	}

	drvdata->spi = spidev;
	spin_lock_init(&drvdata->spi_lock);
	spi_set_drvdata(spidev, drvdata);
	g_spi_drvdata = drvdata;

	ret = spi_setup(spidev);
	if (ret) {
		ERROR_PRINT("spi_setup() return with %d", ret);
		return ret;
	}

	INFO_PRINT("max_speed_hz   :%d",spidev->max_speed_hz);
	INFO_PRINT("chip_select    :%d",spidev->chip_select);
	INFO_PRINT("mode           :%d",spidev->mode);
	INFO_PRINT("bits_per_word  :%d",spidev->bits_per_word);
	INFO_PRINT("irq            :%d",spidev->irq);
	INFO_PRINT("modalias       :%s",spidev->modalias);

	INFO_PRINT("%s END", __FUNCTION__);

	return ret;
}

/**************************************************************************//**
 * remove function called by spi_register_driver()
 *
 * @date	2013.12.10
 *
 * @author T.Abe (FSI)
 * @author K.Okawa (KXDA3)
 *
 * @retval 0			Normal end
 * @retval <0			error (refer the errno)
 *
 * @param [in] spidev	pointer to the "spi_device" structure
 ******************************************************************************/
static int tuner_drv_spi_remove(struct spi_device *spidev)
{
	INFO_PRINT("%s START",__FUNCTION__);

	spi_set_drvdata(spidev, NULL);
	kfree(g_spi_drvdata);
	g_spi_drvdata = NULL;

	INFO_PRINT("%s END",__FUNCTION__);

	return 0;
}

/**************************************************************************//**
 * TS read Calibration
 *
 * @date	2014.12.02
 *
 * @author K.Fukuzaki (KXDA3)
 *
 * @retval 0		normal
 * @retval <0		error
 ******************************************************************************/
int tuner_drv_spi_calibration(void)
{
  int ret = 0;
  uint8_t tx[1] = { 0x4b };
  uint8_t rx[4] = { 0x00, 0x00, 0x00, 0x00 };
  struct spi_message msg;
  struct spi_transfer xfer[2];
  int i, j;
  int calflg = 0;
  
  INFO_PRINT("%s START", __FUNCTION__);
  memset(&xfer, 0, sizeof(struct spi_transfer));
  spi_message_init(&msg);

  //Command
  xfer[0].tx_buf = (void *)tx;
  xfer[0].len = 1;
  xfer[0].bits_per_word = 8;
  spi_message_add_tail(&xfer[0], &msg);

  for(i=0; i<10; i++){
    //data
    xfer[1].rx_buf = (void *)rx;
    xfer[1].len = 4;
    xfer[1].bits_per_word = 32;
    spi_message_add_tail(&xfer[1], &msg);

    ret = spi_sync(g_spi_drvdata->spi, &msg);
    if (ret) {
      ERROR_PRINT("spi_sync() return with %d", ret);
      return ret;
    }
    for(j=0; j<4; j++){
      if(rx[j]==0x72) calflg++;
      rx[j] = 0x00;
    }
    i++;
  }

  if (ret) {
    ERROR_PRINT("spi_sync() return with %d (@Last Byte)", ret);
    return ret;
  }

  if (calflg != 9) {
    ERROR_PRINT("spi_calibration() failed. calflg = %d", calflg);
    return -1;
  }    
  else{
  //INFO_PRINT("%s END", __FUNCTION__);
  return 0;
  }
}

/**************************************************************************//**
 * TS read Idle
 *
 * @date	2014.12.02
 *
 * @author K.Fukuzaki (KXDA3)
 *
 * @retval 0		normal
 * @retval <0		error
 ******************************************************************************/
int tuner_drv_spi_idle(void)
{
  int ret = 0;
  uint8_t tx[1] = { 0x05 };
  uint8_t rx[1] = { 0x00 };
  struct spi_message msg;
  struct spi_transfer xfer;
  
  INFO_PRINT("%s START", __FUNCTION__);
  memset(&xfer, 0, sizeof(struct spi_transfer));
  spi_message_init(&msg);

  while(1){
    xfer.tx_buf = (void *)tx;
    xfer.len = 1;
    xfer.bits_per_word = 8;
    xfer.rx_buf = (void *)rx;
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(g_spi_drvdata->spi, &msg);

    if (ret) {
      ERROR_PRINT("spi_sync() return with %d", ret);
      return ret;
    }
    if(rx[0] == 0x72) break;
  }

  if (ret) {
    ERROR_PRINT("spi_sync() return with %d", ret);
    return ret;
  }

  //INFO_PRINT("%s END", __FUNCTION__);
  return 0;
}

/**************************************************************************//**
 * SPI Command Break
 *
 * @date	2014.12.19
 *
 * @author K.Fukuzaki (KXDA3)
 *
 * @retval 0		normal
 * @retval <0		error
 ******************************************************************************/
int tuner_drv_spi_break(void)
{
  int ret = 0;
  uint8_t tx[4] = { 0xFF, 0xFE, 0x81, 0x00 };
  struct spi_message msg;
  struct spi_transfer xfer;
  
  INFO_PRINT("%s START", __FUNCTION__);
  memset(&xfer, 0, sizeof(struct spi_transfer));
  spi_message_init(&msg);

  xfer.tx_buf = (void *)tx;
  xfer.len = 1;
  xfer.bits_per_word = 32;
  spi_message_add_tail(&xfer, &msg);
  ret = spi_sync(g_spi_drvdata->spi, &msg);

  if (ret) {
    ERROR_PRINT("spi_sync() return with %d", ret);
    return ret;
  }

  //INFO_PRINT("%s END", __FUNCTION__);
  return 0;
}

/**************************************************************************//**
 * PSEQ ROM Data Download
 *
 * @date	2014.12.19
 *
 * @author K.Fukuzaki (KXDA3)
 *
 * @retval 0		normal
 * @retval <0		error
 ******************************************************************************/
int tuner_drv_spi_download(unsigned char *sadr, unsigned char *eadr)
{
  int ret = 0;
  uint8_t tx[8] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //tx[1]:PESQ, tx[4:7]:write_data
  struct spi_message msg;
  struct spi_transfer xfer;
  int i;
  int loop_cnt;
  unsigned char *padr;
  
  INFO_PRINT("%s START", __FUNCTION__);
  memset(&xfer, 0, sizeof(struct spi_transfer));
  spi_message_init(&msg);

  //init
  padr = SPI_PSEQ_ADRS_INIT;
  if( ( (eadr-sadr)%4 != 0 ) ){
    ERROR_PRINT("Translation size Error! %d", (eadr-sadr) );
    return -1;
  } else {
    loop_cnt = (eadr-sadr)/4;
  }

  for(i=0; i<loop_cnt; i++){
    //Command
    tx[1] = *(padr + (loop_cnt * 4)    );
    tx[4] = *(sadr + (loop_cnt * 4)    );
    tx[5] = *(sadr + (loop_cnt * 4) + 1);
    tx[6] = *(sadr + (loop_cnt * 4) + 2);
    tx[7] = *(sadr + (loop_cnt * 4) + 3);

    xfer.tx_buf = (void *)tx;
    xfer.len = 8;
    xfer.bits_per_word = 32;
    spi_message_add_tail(&xfer, &msg);

    ret = spi_sync(g_spi_drvdata->spi, &msg);
    if (ret) {
      ERROR_PRINT("spi_sync() return with %d", ret);
      return ret;
    }

    loop_cnt++;
  }
  //INFO_PRINT("%s END", __FUNCTION__);
  return 0;
}
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
