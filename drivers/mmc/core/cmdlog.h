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

#ifndef _MMC_CORE_CMDLOG_H
#define _MMC_CORE_CMDLOG_H

/* FCNT LIMITED:2016-03-07 H16100340 add start */
#define MMC_PRINT_RESPONSE(host, mrq)			mmc_cmdlog_print_response(host, mrq)
#define MMC_PRINT_CMDLOG(host, areq, mmc_rq)	mmc_cmdlog_print(host, areq, mmc_rq)

void mmc_cmdlog_sysfs_init(struct mmc_host *host);
/* FCNT LIMITED:2016-03-07 H16100340 add end */

void mmc_cmdlog_dump_init(struct mmc_host *host);
void mmc_cmdlog_dump(struct mmc_host *host);

void mmc_cmdlog_print(struct mmc_host *host, struct mmc_async_req *areq, struct mmc_request *mmc_rq);
void mmc_cmdlog_print_response(struct mmc_host *host, struct mmc_request *mrq);

#endif
