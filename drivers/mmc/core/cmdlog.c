/*----------------------------------------------------------------------------*/
/* COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016                   */
/*----------------------------------------------------------------------------*/
/*
 * Copyright(C) 2014 - 2015 FUJITSU LIMITED
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

/* FCNT LIMITED:2016-03-07 H16100340 add start */
//==============================================================================
// include file
//==============================================================================

#include <linux/jiffies.h>
#include <linux/rtc.h>

#include <trace/events/power.h> /* FUJITSU LIMITED:2014-11-26 H1510112 add */

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#include "../card/queue.h"

#include "core.h"
#include "bus.h"
#include "host.h"
#include "sdio_bus.h"

#include "mmc_ops.h"
#include "sd_ops.h"
#include "sdio_ops.h"

#include "cmdlog.h"

//==============================================================================
// define / macro
//==============================================================================
#define MMC_CMDLOG_TIMESTAMP_LEN				(30) /* FCNT LIMITED:2016-03-07 H16100340-1 mod */
#define MMC_CMDLOG_FILTER_LEN					(4)
#define MMC_IS_PRINTK_CMDLOG_ENABLE()			(cmdlog_flag)
#define MMC_CHECK_PRINTK_CMDLOG_FILTER()		((strlen(filter) == 0) || ((strlen(filter) > 0) && (strcmp(mmc_hostname(host), filter) == 0)))
#define MMC_PRINTK_SIZE							(512)
#define MMC_CMDLOG_BUF_SIZE						(16*1024)
#define MMC_CMDLOG_TEMPBUF_SIZE					(512)
#define MMC_CMDLOG_DEV_NUM						(2)

#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)
#define MMC_TRACE_CHAR_LEN						(16)

//==============================================================================
// struct
//==============================================================================
struct mmc_cmdlog {
	char 			buf[MMC_CMDLOG_BUF_SIZE];
	char 			buf_temp[MMC_CMDLOG_TEMPBUF_SIZE];
	spinlock_t		lock;
	bool 			lock_initialized;
	bool 			head_move;
	unsigned long	end_pos;
};

//==============================================================================
// static data
//==============================================================================
static struct mmc_cmdlog	mmc_cmdlog_data[MMC_CMDLOG_DEV_NUM];
static int  cmdlog_flag = 0;
static char filter[MMC_CMDLOG_FILTER_LEN+1];

//==============================================================================
// functions
//==============================================================================
static ssize_t
mmc_cmdlog_dump_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	mmc_cmdlog_dump(host);

	return 0;
}

static ssize_t
mmc_cmdlog_dump_store(struct device *dev, struct device_attribute *attr,
		const char *data, size_t len)
{
	return len;
}

static ssize_t
mmc_cmdlog_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", cmdlog_flag);
}

static ssize_t
mmc_cmdlog_store(struct device *dev, struct device_attribute *attr,
		const char *data, size_t len)
{
	if (!strncmp(data, "1", 1)) {
		cmdlog_flag = 1;
	} else if (!strncmp(data, "0", 1)) {
		cmdlog_flag = 0;
	} else {
		return -EINVAL;
	}

	return len;
}

static ssize_t
mmc_cmdlog_filter_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%s\n", filter);
}

static ssize_t
mmc_cmdlog_filter_store(struct device *dev, struct device_attribute *attr,
		const char *data, size_t len)
{
	if (len != sizeof(filter)) {
		return -EINVAL;
	}
	memset(filter, 0, sizeof(filter));
	memcpy(filter, data, len-1);

	return len;
}

static DEVICE_ATTR(cmdlog_dump,    S_IRUGO,           mmc_cmdlog_dump_show,   mmc_cmdlog_dump_store);
static DEVICE_ATTR(cmdlog,         S_IRUGO | S_IWUSR, mmc_cmdlog_show,        mmc_cmdlog_store);
static DEVICE_ATTR(cmdlog_filter,  S_IRUGO | S_IWUSR, mmc_cmdlog_filter_show, mmc_cmdlog_filter_store);

void mmc_cmdlog_sysfs_init(struct mmc_host *host)
{

	/* FUJITSU LIMITED:2015-01-23 H1510175 add start */
	if (mmc_host_get_type(host) == HOST_TYPE_SDIO)
		return;
	/* FUJITSU LIMITED:2015-01-23 H1510175 add end */

	if (host->index == 0) {
		if (device_create_file(&host->class_dev, &dev_attr_cmdlog))
			pr_err("%s: Failed to create cmdlog sysfs entry\n",
					mmc_hostname(host));

		if (device_create_file(&host->class_dev, &dev_attr_cmdlog_filter))
			pr_err("%s: Failed to create cmdlog_filter sysfs entry\n",
					mmc_hostname(host));
	}
	if (device_create_file(&host->class_dev, &dev_attr_cmdlog_dump))
		pr_err("%s: Failed to create cmdlog_dump_filter sysfs entry\n",
				mmc_hostname(host));

	return;
}

void mmc_cmdlog_dump_init(struct mmc_host *host)
{
	struct mmc_cmdlog * cmdlog;

	if (host->index >= MMC_CMDLOG_DEV_NUM)
		return;

	cmdlog = &mmc_cmdlog_data[host->index];

	if (!cmdlog->lock_initialized) {
		spin_lock_init( &cmdlog->lock );
		cmdlog->lock_initialized = true;
	}
	cmdlog->head_move = 0;
	cmdlog->end_pos = 0;

	return;
}

static void mmc_cmdlog_printf_internal( struct mmc_cmdlog * cmdlog, bool print_flg, const char* format, va_list args )
{
	unsigned log_size = 0;
	unsigned long flags;
	
	spin_lock_irqsave(&cmdlog->lock, flags);

	/* write in the temp buffer */
	log_size = vsnprintf( &cmdlog->buf_temp[0], sizeof(cmdlog->buf_temp), format, args );
	
	if (log_size > 0 && log_size <= MMC_CMDLOG_BUF_SIZE) {
		
		if ((cmdlog->end_pos + log_size) <= MMC_CMDLOG_BUF_SIZE) {
			/* Simple copy */
			memcpy( &cmdlog->buf[ cmdlog->end_pos ], cmdlog->buf_temp, log_size );
			
			cmdlog->end_pos += log_size;
		} else {
			int top_size = MMC_CMDLOG_BUF_SIZE - cmdlog->end_pos;
			int bottom_size = log_size - top_size;
			
			/* copy forward */
			memcpy( &cmdlog->buf[ cmdlog->end_pos ], cmdlog->buf_temp, top_size );
			
			/* Rear copy */
			memcpy( &cmdlog->buf[ 0 ], ( cmdlog->buf_temp + top_size ), bottom_size );
			
			cmdlog->end_pos = bottom_size;
			cmdlog->head_move = true;
		}
	}
	
	if (cmdlog->end_pos >= MMC_CMDLOG_BUF_SIZE)
		cmdlog->end_pos = 0;
	
	if (print_flg)
		printk(KERN_ERR "%s", &cmdlog->buf_temp[0]);
	spin_unlock_irqrestore( &cmdlog->lock, flags );

	return;
}

static void mmc_cmdlog_printf( struct mmc_host *host, const char* format, ... )
{
	va_list args;
	bool print_flg = false;

	if (host->index >= MMC_CMDLOG_DEV_NUM)
		return;

	if (MMC_IS_PRINTK_CMDLOG_ENABLE() && MMC_CHECK_PRINTK_CMDLOG_FILTER())
		print_flg = true;

	va_start( args, format );
	mmc_cmdlog_printf_internal( &mmc_cmdlog_data[host->index], print_flg, format, args );
	va_end( args );

	return;
}

static void mmc_cmdlog_kmsg_output( char *msg, unsigned long size )
{
	int write_size;
	int write_size_total;
	
	for (write_size_total = 0; write_size_total < size; write_size_total += write_size) {
		
		if (size - write_size_total > MMC_PRINTK_SIZE)
			write_size = MMC_PRINTK_SIZE;
		else
			write_size = size - write_size_total;
		
		printk( KERN_CONT "%.*s", write_size, &msg[write_size_total] );
	}

	return;
}

void mmc_cmdlog_dump( struct mmc_host *host )
{
	unsigned long flags;
	struct mmc_cmdlog * cmdlog;
	
	if (host->index >= MMC_CMDLOG_DEV_NUM)
		return;

	if (MMC_IS_PRINTK_CMDLOG_ENABLE())
		return;

	cmdlog = &mmc_cmdlog_data[host->index];
	if (!cmdlog->lock_initialized)
		return;

	spin_lock_irqsave( &cmdlog->lock, flags );
	
	if (cmdlog->head_move)
		mmc_cmdlog_kmsg_output( cmdlog->buf + cmdlog->end_pos, MMC_CMDLOG_BUF_SIZE - cmdlog->end_pos );
	mmc_cmdlog_kmsg_output( cmdlog->buf, cmdlog->end_pos );
	
	cmdlog->head_move = 0;
	cmdlog->end_pos = 0;
	
	spin_unlock_irqrestore( &cmdlog->lock, flags );

	return;
}

/* FUJITSU LIMITED:2014-11-26 H1510112 add start */
static int mmc_trace_rw(int en, struct mmc_host *host, struct mmc_request *mmc_rq)
{
	do {
		if (!mmc_rq->data)
			break;

		/* write operation */
		if (mmc_rq->cmd->opcode == MMC_WRITE_BLOCK ||
			mmc_rq->cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK) {
			char name[MMC_TRACE_CHAR_LEN];
			snprintf(name, MMC_TRACE_CHAR_LEN, "%s_write", mmc_hostname(host));
			trace_clock_set_rate(name, en?mmc_rq->data->blocks:0, raw_smp_processor_id());
			break;
		}

		/* read operation */
		if (mmc_rq->cmd->opcode == MMC_READ_SINGLE_BLOCK ||
			mmc_rq->cmd->opcode == MMC_READ_MULTIPLE_BLOCK) {
			char name[MMC_TRACE_CHAR_LEN];
			snprintf(name, MMC_TRACE_CHAR_LEN, "%s_read", mmc_hostname(host));
			trace_clock_set_rate(name, en?mmc_rq->data->blocks:0, raw_smp_processor_id());
			break;
		}
	} while(0);

	return 0;
}
/* FUJITSU LIMITED:2014-11-26 H1510112 add end */

static void mmc_cmdlog_get_timestamp(char *buf_p)
{
	struct timespec ts;
	struct rtc_time tm;
	
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	snprintf(buf_p, MMC_CMDLOG_TIMESTAMP_LEN, "%d-%02d-%02d %02d:%02d:%02d.%09lu",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); /* FCNT LIMITED:2016-03-07 H16100340-1 mod */

	return;
}

static void mmc_cmdlog_print_packed_list(struct mmc_host *host, struct mmc_async_req *areq, char * time)
{
	struct mmc_queue_req *mq_rq;
	int packed_num, rw, version, i, index = 1;
	char * cmd = NULL;

	mq_rq = container_of(areq, struct mmc_queue_req, mmc_active);
	if (!mq_rq)
		return;

	version		= (mq_rq->packed->cmd_hdr[0] & 0xFF);
	rw			= ((mq_rq->packed->cmd_hdr[0] & 0xFF00) >> 8);
	packed_num	= ((mq_rq->packed->cmd_hdr[0] & 0xFF0000) >> 16);
	if (rw == 0x02)
		cmd = "CMD25";
	else if(rw == 0x01)
		cmd = "CMD18";

	mmc_cmdlog_printf(host, "%s [CMD LOG]%s: ------------Packed Header------------- \n", time, mmc_hostname(host));
	mmc_cmdlog_printf(host, "%s [CMD LOG]%s: | PACKED VERSION 0x%x R/W 0x%x NUM 0x%x \n",
			time, mmc_hostname(host), version, rw, packed_num);
	for (i=0; i < packed_num; i++) {
		mmc_cmdlog_printf(host, "%s [CMD LOG]%s: | CMD23_ARG 0x%08x %s_ARG 0x%08x \n",
				time, mmc_hostname(host), mq_rq->packed->cmd_hdr[(index * 2)], cmd, mq_rq->packed->cmd_hdr[(index * 2) + 1]);
		index++;
	}
	mmc_cmdlog_printf(host, "%s [CMD LOG]%s: -------------------------------------- \n", time, mmc_hostname(host));
	return;
}

void mmc_cmdlog_print(struct mmc_host *host, struct mmc_async_req *areq, struct mmc_request *mmc_rq)
{
	char	buf[MMC_CMDLOG_TIMESTAMP_LEN];

	/* FUJITSU LIMITED:2015-01-23 H1510175 add start */
	if (mmc_host_get_type(host) == HOST_TYPE_SDIO)
		return;
	/* FUJITSU LIMITED:2015-01-23 H1510175 add end */

	mmc_trace_rw(1, host, mmc_rq);	/* FUJITSU LIMITED:2014-11-26 H1510112 add */
	mmc_cmdlog_get_timestamp(&buf[0]);
	if (mmc_rq->sbc) {
		mmc_cmdlog_printf(host, "%s [CMD LOG]%s: CMD%u arg 0x%08x\n",
			 &buf[0], mmc_hostname(host), mmc_rq->sbc->opcode, mmc_rq->sbc->arg);
		if (areq && mmc_rq->sbc->arg & MMC_CMD23_ARG_PACKED) {
			mmc_cmdlog_print_packed_list(host, areq, &buf[0]);
		}
	}

	mmc_cmdlog_printf(host, "%s [CMD LOG]%s: CMD%u arg 0x%08x\n",
		 &buf[0], mmc_hostname(host), mmc_rq->cmd->opcode, mmc_rq->cmd->arg);

	if (mmc_rq->data) {
		mmc_cmdlog_printf(host, "%s [CMD LOG]%s:       blksz %d blocks %d\n",
			&buf[0], mmc_hostname(host), mmc_rq->data->blksz,
			mmc_rq->data->blocks);
	}

	if (mmc_rq->stop && !mmc_rq->sbc) {
		mmc_cmdlog_printf(host, "%s [CMD LOG]%s: CMD%u arg 0x%08x\n",
			 &buf[0], mmc_hostname(host), mmc_rq->stop->opcode, mmc_rq->stop->arg);
	}
	return;
}

void mmc_cmdlog_print_response(struct mmc_host *host, struct mmc_request *mrq)
{
	char	buf[MMC_CMDLOG_TIMESTAMP_LEN];
	struct mmc_command *cmd = mrq->cmd;

	/* FUJITSU LIMITED:2015-01-23 H1510175 add start */
	if (mmc_host_get_type(host) == HOST_TYPE_SDIO)
		return;
	/* FUJITSU LIMITED:2015-01-23 H1510175 add end */

	mmc_cmdlog_get_timestamp(&buf[0]);
	mmc_cmdlog_printf(host, "%s [CMD LOG]%s: CMD%u response 0x%08x 0x%08x 0x%08x 0x%08x\n",
		&buf[0], mmc_hostname(host), cmd->opcode, 
		cmd->resp[0], cmd->resp[1],
		cmd->resp[2], cmd->resp[3]);

	if (mrq->stop && !mrq->sbc) {
		mmc_cmdlog_printf(host, "%s [CMD LOG]%s: CMD%u response 0x%08x 0x%08x 0x%08x 0x%08x\n",
			&buf[0], mmc_hostname(host), mrq->stop->opcode,
			mrq->stop->resp[0], mrq->stop->resp[1],
			mrq->stop->resp[2], mrq->stop->resp[3]);
	}
	mmc_trace_rw(0, host, mrq);	/* FUJITSU LIMITED:2014-11-26 H1510112 add */
	return;
}
/* FCNT LIMITED:2016-03-07 H16100340 add end */
