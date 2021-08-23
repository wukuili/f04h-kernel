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
 *
 */

/* FUJITSU LIMITED:2015-05-14 H1520355 add start */
#ifndef _MMC_CORE_LOCK_H
#define _MMC_CORE_LOCK_H

#ifdef CONFIG_MMC_PASSWORDS

void mmc_host_sdlock_sysfs_init(struct mmc_host *host);
void mmc_host_lockable_set_result(struct mmc_host *host,int ret);
void mmc_host_lockable_setcid(struct mmc_host *host, struct mmc_card *card);
void mmc_host_lockable_uevent(struct mmc_host *host, struct mmc_card *card);

#define MMC_KEYLEN          16
#define MMC_KEYLEN_MAXBYTES 32

/* result */
#define SDLOCCK_OK   0
#define SDLOCCK_ERR -1

/* HOST_INDEX */
#define HOST_INDEX_MMC 0
#define HOST_INDEX_SD  1

/* SD_STATUS */
#define SDSTATUS_INIT         0
#define SDSTATUS_UNINSERTED   1
#define SDSTATUS_NO_PASSWORD  2
#define SDSTATUS_LOCK         3
#define SDSTATUS_UNLOCK       4

/* LOCKABLE_EVENT */
#define EVENT_INIT   0
#define EVENT_ERASE  1
#define EVENT_REMOVE 2
#define EVENT_ASSIGN 3
#define EVENT_CHANGE 4
#define EVENT_UNLOCK 5

/* sysfs_result */
#define RESULT_SUCCESS             0
#define RESULT_ERR                -1
#define RESULT_ILLEGALPARAM       -2
#define RESULT_BADSTATUS          -3
#define RESULT_NOPASSWORD         -4
#define RESULT_UNMATCHPASSWORD    -5
#define RESULT_INIT             -255

extern int  lockable_sdstatus;
extern int  lockable_event;
extern int  lockable_retry;

struct lock_unlock_key{
    int  len;
    char data[MMC_KEYLEN+1];
};

extern struct lock_unlock_key temp_key;
extern struct lock_unlock_key lockable_key;
extern struct lock_unlock_key lockable_old_key;

/* ----- Log ----- */
// #define DEBUG_SD_DEBUG
#ifdef DEBUG_SD_DEBUG
    #define SDLOG_DEBUG(fmt, args...)  \
            printk("[DBG]MMC/SD: L%04d: %s() " fmt "\n", __LINE__, __func__, ## args)
#else
    #define SDLOG_DEBUG(fmt, args...)
#endif

//#define DEBUG_SD_INFO
#ifdef DEBUG_SD_INFO
    #define SDLOG_INFO(fmt, args...)  \
        printk(KERN_INFO "[INF]MMC/SD: L%04d: %s() " fmt "\n", __LINE__, __func__, ## args)
#else
    #define SDLOG_INFO(fmt, args...)
#endif

#define DEBUG_SD_ERR
#ifdef DEBUG_SD_ERR
    #define SDLOG_ERR(fmt, args...)  \
            printk(KERN_ERR  "[ERR]MMC/SD: L%04d: %s() " fmt "\n", __LINE__, __func__, ## args)
#else
    #define SDLOG_ERR(fmt...)
#endif


#else
#endif

#endif
/* FUJITSU LIMITED:2015-05-14 H1520355 add end */
