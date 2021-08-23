/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
/*----------------------------------------------------------------------------*/
/*
===============================================================================================
 * HISTORY
 *
 * Tag                      Date       Author           Description
 * ======================== ========== ===============  =======================================
===============================================================================================
 */
/* FUJITSU:2016-02-25 LogImprovement ADD-S */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include "internal.h"

static DEFINE_MUTEX(smry_lock);
#define SUMMARY_LOG_ID_SIZE  928

//ssize_t pstore_write_summary(const int resetid)
ssize_t pstore_write_summary_msg(const int resetid, const char* msg)
{
	u64 id;
    char buf[SUMMARY_LOG_ID_SIZE + 1];

    memset(buf, 0x00, sizeof(buf));
    sprintf(buf, "%d,%s", resetid, msg);

//	mutex_lock(&smry_lock);
	//psinfo->write_buf(PSTORE_TYPE_SUMMARY, 0, &id, 0, (const char *)(&resetid), sizeof(int),
    psinfo->write_buf(PSTORE_TYPE_SUMMARY, 0, &id, 0, (const char *)buf, sizeof(int),
			  psinfo);

//	mutex_unlock(&smry_lock);

	return 1;
}

ssize_t pstore_write_summary(const int resetid) {
    return pstore_write_summary_msg(resetid, "");
}
/* FUJITSU:2016-03-04 LogImprovement ADD-E */
