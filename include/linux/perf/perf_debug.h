/*------------------------------------------------------------------------------
 * include/linux/perf/perf_debug.h
 *
 * Copyright(C) 2016 FUJITSU LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *------------------------------------------------------------------------------
 */

#ifndef PERF_DEBUG_H
#define PERF_DEBUG_H

#include <linux/bitops.h>
#include <linux/compat.h>
#include <linux/nonvolatile_common.h>
#include <linux/printk.h>

#define PERF_DEBUG_ADDR (0xC081)
#define PERF_DEBUG_SIZE (1)

enum perf_debug_mode_bit {
    PERF_DEBUG_PERFORM_COMMIT_BIT = BIT(0),
};

int perf_debug_nvinfo_read(int bitflag);
bool perf_debug_mode_perform_commit(void);

#endif /* PERF_DEBUG_H */
