/*------------------------------------------------------------------------------
 * perf/perf_debug.c
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

#include <linux/perf/perf_debug.h>

static bool g_perf_debug_nvinfo_read_success = 0;
static bool g_perf_debug_mode_perform_commit = 0;

int perf_debug_nvinfo_read(int bitflag)
{
    int retval = -1;
    unsigned char nv_value = 0x00;

    retval = get_nonvolatile(&nv_value, PERF_DEBUG_ADDR, PERF_DEBUG_SIZE);

    if (retval < 0) {
        pr_err("[perf_debug]%s:fail get_nonvolatile(), return(%d)\n",
                __func__, retval);
        return retval;
    }

    pr_debug("[perf_debug]%s:success get_nonvolatile(), nv_value(%d)\n",
                __func__, nv_value);

    return nv_value & bitflag;
}

bool perf_debug_mode_perform_commit(void)
{
    int retval = -1;

    if (!g_perf_debug_nvinfo_read_success) {
        retval = perf_debug_nvinfo_read(PERF_DEBUG_PERFORM_COMMIT_BIT);

        if (retval >= 0) {
            g_perf_debug_mode_perform_commit = retval;
        }

        g_perf_debug_nvinfo_read_success = 1;
    }

    return g_perf_debug_mode_perform_commit;
}
