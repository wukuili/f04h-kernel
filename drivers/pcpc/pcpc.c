/*
 * pcpc.c
 *
 * Copyright(C) 2014-2015 FUJITSU LIMITED
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

/* ==========================================================================
 *  INCLUDE HEADER
 * ========================================================================== */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/sysfs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/percpu.h>
#include <linux/kobject.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/pm_qos.h>
#include <asm/cputime.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/topology.h>


/* ==========================================================================
 *  DEFINITION
 * ========================================================================== */

/* define symbol */
#define LITTLE_CLUSTER_MASK     (*(topology_core_cpumask(0)))
#define BIG_CLUSTER_MASK        (*(topology_core_cpumask(4)))
#define DEVICE_NAME_GPU         "fdb00000.qcom,kgsl-3d0"
#define DEVICE_NAME_BUS         "qcom,cpubw"
#define DEVFREQ_LIMIT_DONTUSE   0

#define PCPC_PARAM_SCENE                "scene"
#define PCPC_PARAM_FUNC                 "func"
#define PCPC_PARAM_DURATION             "duration"
#define PCPC_PARAM_PRI                  "pri"
#define PCPC_PARAM_TSK_INIT             "tsk_init"
#define PCPC_PARAM_TSK_SMALL            "tsk_small"
#define PCPC_PARAM_TSK_GO_BIG           "tsk_go_big"
#define PCPC_PARAM_TSK_TO_LIT           "tsk_to_lit"
#define PCPC_PARAM_TSK_NB_NICE          "tsk_nb_nice"
#define PCPC_PARAM_TSK_BOOST            "tsk_boost"
#define PCPC_PARAM_LIT_TGTL             "lit_tgtl"
#define PCPC_PARAM_BIG_TGTL             "big_tgtl"
#define PCPC_PARAM_LIT_AHSPD            "lit_ahspd"
#define PCPC_PARAM_BIG_AHSPD            "big_ahspd"
#define PCPC_PARAM_LIT_SMPT             "lit_smpt"
#define PCPC_PARAM_BIG_SMPT             "big_smpt"
#define PCPC_PARAM_LIT_HSPF             "lit_hspf"
#define PCPC_PARAM_BIG_HSPF             "big_hspf"
#define PCPC_PARAM_LIT_GHSPL            "lit_ghspl"
#define PCPC_PARAM_BIG_GHSPL            "big_ghspl"
#define PCPC_PARAM_LIT_SLKT             "lit_slkt"
#define PCPC_PARAM_BIG_SLKT             "big_slkt"
#define PCPC_PARAM_LIT_HYST             "lit_hyst"
#define PCPC_PARAM_BIG_HYST             "big_hyst"
#define PCPC_PARAM_IOISBSY              "ioisbsy"
#define PCPC_PARAM_LIT_INSF             "lit_insf"
#define PCPC_PARAM_BIG_INSF             "big_insf"
#define PCPC_PARAM_LIT_MAXF             "lit_maxf"
#define PCPC_PARAM_BIG_MAXF             "big_maxf"
#define PCPC_PARAM_LIT_MINF             "lit_minf"
#define PCPC_PARAM_BIG_MINF             "big_minf"
#define PCPC_PARAM_LIT_UPCORE           "lit_upcore"
#define PCPC_PARAM_BIG_UPCORE           "big_upcore"
#define PCPC_PARAM_BUS_MIN              "bus_min"
#define PCPC_PARAM_GPU_MAX              "gpu_max"
#define PCPC_PARAM_GPU_MIN              "gpu_min"
#define PCPC_PARAM_LIT_MLLOAD           "lit_mlload"
#define PCPC_PARAM_LIT_MLFREQ           "lit_mlfreq"
#define PCPC_PARAM_LIT_MLNRRUN          "lit_mlnrrun"
#define PCPC_PARAM_TSK_IDLE             "tsk_idle"
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
#define PCPC_PARAM_CPU_DMA_LAT          "cpu_dma_latency"
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */

#define PCPC_PARAM_SCENE_LEN            (sizeof(PCPC_PARAM_SCENE        ) - 1)
#define PCPC_PARAM_FUNC_LEN             (sizeof(PCPC_PARAM_FUNC         ) - 1)
#define PCPC_PARAM_DURATION_LEN         (sizeof(PCPC_PARAM_DURATION     ) - 1)
#define PCPC_PARAM_PRI_LEN              (sizeof(PCPC_PARAM_PRI          ) - 1)
#define PCPC_PARAM_TSK_INIT_LEN         (sizeof(PCPC_PARAM_TSK_INIT     ) - 1)
#define PCPC_PARAM_TSK_SMALL_LEN        (sizeof(PCPC_PARAM_TSK_SMALL    ) - 1)
#define PCPC_PARAM_TSK_GO_BIG_LEN       (sizeof(PCPC_PARAM_TSK_GO_BIG   ) - 1)
#define PCPC_PARAM_TSK_TO_LIT_LEN       (sizeof(PCPC_PARAM_TSK_TO_LIT   ) - 1)
#define PCPC_PARAM_TSK_NB_NICE_LEN      (sizeof(PCPC_PARAM_TSK_NB_NICE  ) - 1)
#define PCPC_PARAM_TSK_BOOST_LEN        (sizeof(PCPC_PARAM_TSK_BOOST    ) - 1)
#define PCPC_PARAM_LIT_TGTL_LEN         (sizeof(PCPC_PARAM_LIT_TGTL     ) - 1)
#define PCPC_PARAM_BIG_TGTL_LEN         (sizeof(PCPC_PARAM_BIG_TGTL     ) - 1)
#define PCPC_PARAM_LIT_AHSPD_LEN        (sizeof(PCPC_PARAM_LIT_AHSPD    ) - 1)
#define PCPC_PARAM_BIG_AHSPD_LEN        (sizeof(PCPC_PARAM_BIG_AHSPD    ) - 1)
#define PCPC_PARAM_LIT_SMPT_LEN         (sizeof(PCPC_PARAM_LIT_SMPT     ) - 1)
#define PCPC_PARAM_BIG_SMPT_LEN         (sizeof(PCPC_PARAM_BIG_SMPT     ) - 1)
#define PCPC_PARAM_LIT_HSPF_LEN         (sizeof(PCPC_PARAM_LIT_HSPF     ) - 1)
#define PCPC_PARAM_BIG_HSPF_LEN         (sizeof(PCPC_PARAM_BIG_HSPF     ) - 1)
#define PCPC_PARAM_LIT_GHSPL_LEN        (sizeof(PCPC_PARAM_LIT_GHSPL    ) - 1)
#define PCPC_PARAM_BIG_GHSPL_LEN        (sizeof(PCPC_PARAM_BIG_GHSPL    ) - 1)
#define PCPC_PARAM_LIT_SLKT_LEN         (sizeof(PCPC_PARAM_LIT_SLKT     ) - 1)
#define PCPC_PARAM_BIG_SLKT_LEN         (sizeof(PCPC_PARAM_BIG_SLKT     ) - 1)
#define PCPC_PARAM_LIT_HYST_LEN         (sizeof(PCPC_PARAM_LIT_HYST     ) - 1)
#define PCPC_PARAM_BIG_HYST_LEN         (sizeof(PCPC_PARAM_BIG_HYST     ) - 1)
#define PCPC_PARAM_IOISBSY_LEN          (sizeof(PCPC_PARAM_IOISBSY      ) - 1)
#define PCPC_PARAM_LIT_INSF_LEN         (sizeof(PCPC_PARAM_LIT_INSF     ) - 1)
#define PCPC_PARAM_BIG_INSF_LEN         (sizeof(PCPC_PARAM_BIG_INSF     ) - 1)
#define PCPC_PARAM_LIT_MAXF_LEN         (sizeof(PCPC_PARAM_LIT_MAXF     ) - 1)
#define PCPC_PARAM_BIG_MAXF_LEN         (sizeof(PCPC_PARAM_BIG_MAXF     ) - 1)
#define PCPC_PARAM_LIT_MINF_LEN         (sizeof(PCPC_PARAM_LIT_MINF     ) - 1)
#define PCPC_PARAM_BIG_MINF_LEN         (sizeof(PCPC_PARAM_BIG_MINF     ) - 1)
#define PCPC_PARAM_LIT_UPCORE_LEN       (sizeof(PCPC_PARAM_LIT_UPCORE   ) - 1)
#define PCPC_PARAM_BIG_UPCORE_LEN       (sizeof(PCPC_PARAM_BIG_UPCORE   ) - 1)
#define PCPC_PARAM_BUS_MIN_LEN          (sizeof(PCPC_PARAM_BUS_MIN      ) - 1)
#define PCPC_PARAM_GPU_MAX_LEN          (sizeof(PCPC_PARAM_GPU_MAX      ) - 1)
#define PCPC_PARAM_GPU_MIN_LEN          (sizeof(PCPC_PARAM_GPU_MIN      ) - 1)
#define PCPC_PARAM_LIT_MLLOAD_LEN       (sizeof(PCPC_PARAM_LIT_MLLOAD   ) - 1)
#define PCPC_PARAM_LIT_MLFREQ_LEN       (sizeof(PCPC_PARAM_LIT_MLFREQ   ) - 1)
#define PCPC_PARAM_LIT_MLNRRUN_LEN      (sizeof(PCPC_PARAM_LIT_MLNRRUN  ) - 1)
#define PCPC_PARAM_TSK_IDLE_LEN         (sizeof(PCPC_PARAM_TSK_IDLE  ) - 1)
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
#define PCPC_PARAM_CPU_DMA_LAT_LEN      (sizeof(PCPC_PARAM_CPU_DMA_LAT  ) - 1)
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
enum {
PCPC_PARAM_SCENE_ID             ,
PCPC_PARAM_FUNC_ID              ,
PCPC_PARAM_DURATION_ID          ,
PCPC_PARAM_PRI_ID               ,
PCPC_PARAM_TSK_INIT_ID          ,
PCPC_PARAM_TSK_SMALL_ID         ,
PCPC_PARAM_TSK_GO_BIG_ID        ,
PCPC_PARAM_TSK_TO_LIT_ID        ,
PCPC_PARAM_TSK_NB_NICE_ID       ,
PCPC_PARAM_TSK_BOOST_ID         ,
PCPC_PARAM_LIT_TGTL_ID          ,
PCPC_PARAM_BIG_TGTL_ID          ,
PCPC_PARAM_LIT_AHSPD_ID         ,
PCPC_PARAM_BIG_AHSPD_ID         ,
PCPC_PARAM_LIT_SMPT_ID          ,
PCPC_PARAM_BIG_SMPT_ID          ,
PCPC_PARAM_LIT_HSPF_ID          ,
PCPC_PARAM_BIG_HSPF_ID          ,
PCPC_PARAM_LIT_GHSPL_ID         ,
PCPC_PARAM_BIG_GHSPL_ID         ,
PCPC_PARAM_LIT_SLKT_ID          ,
PCPC_PARAM_BIG_SLKT_ID          ,
PCPC_PARAM_LIT_HYST_ID          ,
PCPC_PARAM_BIG_HYST_ID          ,
PCPC_PARAM_IOISBSY_ID           ,
PCPC_PARAM_LIT_INSF_ID          ,
PCPC_PARAM_BIG_INSF_ID          ,
PCPC_PARAM_LIT_MAXF_ID          ,
PCPC_PARAM_BIG_MAXF_ID          ,
PCPC_PARAM_LIT_MINF_ID          ,
PCPC_PARAM_BIG_MINF_ID          ,
PCPC_PARAM_LIT_UPCORE_ID        ,
PCPC_PARAM_BIG_UPCORE_ID        ,
PCPC_PARAM_BUS_MIN_ID           ,
PCPC_PARAM_GPU_MAX_ID           ,
PCPC_PARAM_GPU_MIN_ID           ,
PCPC_PARAM_LIT_MLLOAD_ID        ,
PCPC_PARAM_LIT_MLFREQ_ID        ,
PCPC_PARAM_LIT_MLNRRUN_ID       ,
PCPC_PARAM_TSK_IDLE_ID          ,
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
PCPC_PARAM_CPU_DMA_LAT_ID       ,
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
  PCPC_PARAM_ID_MAX
};
#define PCPC_VALUE_STR_IGNORE   "-"
#define PCPC_IGNORE_VALUE       INT_MIN
#define PCPC_SCENE_NAME_LEN     32

enum {
  PCPC_VALUE_FUNC_INPUT_ID      ,
  PCPC_VALUE_FUNC_HOMEKEY_ID    ,
  PCPC_VALUE_FUNC_POWERKEY_ID   ,
  PCPC_VALUE_FUNC_ID_MAX        ,
  PCPC_VALUE_FUNC_UNKNOWN_ID = PCPC_VALUE_FUNC_ID_MAX,
};
#define PCPC_VALUE_FUNC_STR_INPUT   "input"
#define PCPC_VALUE_FUNC_STR_HOMEKEY "homekey"
#define PCPC_VALUE_FUNC_STR_POWERKEY "powerkey"
#define PCPC_VALUE_FUNC_MAXLEN      32
#define PCPC_FUNC_ACTIVE_SAME_MAX   16
#define PCPC_LOCAL_PRINT_BUF_LIMIT  1280
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
#define PCPC_VALUE_CSTATE_TIMEOUT 60000000
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */

/* data structure */
typedef struct {
  /* parameters */
  char scene[PCPC_SCENE_NAME_LEN];
  int func;
  int duration;
  int pri;
  int tsk_init;
  int tsk_small;
  int tsk_go_big;
  int tsk_to_lit;
  int tsk_nb_nice;
  int tsk_boost;
  unsigned int *lit_tgtl;
  unsigned int *big_tgtl;
  unsigned int *lit_ahspd;
  unsigned int *big_ahspd;
  int lit_smpt;
  int big_smpt;
  int lit_hspf;
  int big_hspf;
  int lit_ghspl;
  int big_ghspl;
  int lit_slkt;
  int big_slkt;
  int lit_hyst;
  int big_hyst;
  int ioisbsy;
  int lit_insf;
  int big_insf;
  int lit_maxf;
  int big_maxf;
  int lit_minf;
  int big_minf;
  int lit_upcore;
  int big_upcore;
  int bus_min;
  int gpu_max;
  int gpu_min;
  int lit_mlload;
  int lit_mlfreq;
  int lit_mlnrrun;
  int tsk_idle;
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
  int cpu_dma_latency;
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
  /* internal use */
  int tsk_init_pri;
  int tsk_small_pri;
  int tsk_go_big_pri;
  int tsk_to_lit_pri;
  int tsk_nb_nice_pri;
  int tsk_boost_pri;
  int lit_tgtl_pri;
  int big_tgtl_pri;
  int lit_ahspd_pri;
  int big_ahspd_pri;
  int lit_smpt_pri;
  int big_smpt_pri;
  int lit_hspf_pri;
  int big_hspf_pri;
  int lit_ghspl_pri;
  int big_ghspl_pri;
  int lit_slkt_pri;
  int big_slkt_pri;
  int lit_hyst_pri;
  int big_hyst_pri;
  int ioisbsy_pri;
  int lit_mlload_pri;
  int lit_mlfreq_pri;
  int lit_mlnrrun_pri;
  int tsk_idle_pri;
} pcpc_scene_param_type;
#define PCPC_SCENE_PARAM_TYPE(name)         \
pcpc_scene_param_type name = {              \
  .scene              = {0},                \
  .func               = PCPC_IGNORE_VALUE,  \
  .duration           = PCPC_IGNORE_VALUE,  \
  .pri                = PCPC_IGNORE_VALUE,  \
  .tsk_init           = PCPC_IGNORE_VALUE,  \
  .tsk_small          = PCPC_IGNORE_VALUE,  \
  .tsk_go_big         = PCPC_IGNORE_VALUE,  \
  .tsk_to_lit         = PCPC_IGNORE_VALUE,  \
  .tsk_nb_nice        = PCPC_IGNORE_VALUE,  \
  .tsk_boost          = PCPC_IGNORE_VALUE,  \
  .lit_tgtl           = NULL,               \
  .big_tgtl           = NULL,               \
  .lit_ahspd          = NULL,               \
  .big_ahspd          = NULL,               \
  .lit_smpt           = PCPC_IGNORE_VALUE,  \
  .big_smpt           = PCPC_IGNORE_VALUE,  \
  .lit_hspf           = PCPC_IGNORE_VALUE,  \
  .big_hspf           = PCPC_IGNORE_VALUE,  \
  .lit_ghspl          = PCPC_IGNORE_VALUE,  \
  .big_ghspl          = PCPC_IGNORE_VALUE,  \
  .lit_slkt           = PCPC_IGNORE_VALUE,  \
  .big_slkt           = PCPC_IGNORE_VALUE,  \
  .lit_hyst           = PCPC_IGNORE_VALUE,  \
  .big_hyst           = PCPC_IGNORE_VALUE,  \
  .ioisbsy            = PCPC_IGNORE_VALUE,  \
  .lit_insf           = PCPC_IGNORE_VALUE,  \
  .big_insf           = PCPC_IGNORE_VALUE,  \
  .lit_maxf           = PCPC_IGNORE_VALUE,  \
  .big_maxf           = PCPC_IGNORE_VALUE,  \
  .lit_minf           = PCPC_IGNORE_VALUE,  \
  .big_minf           = PCPC_IGNORE_VALUE,  \
  .lit_upcore         = PCPC_IGNORE_VALUE,  \
  .big_upcore         = PCPC_IGNORE_VALUE,  \
  .bus_min            = PCPC_IGNORE_VALUE,  \
  .gpu_max            = PCPC_IGNORE_VALUE,  \
  .gpu_min            = PCPC_IGNORE_VALUE,  \
  .lit_mlload         = PCPC_IGNORE_VALUE,  \
  .lit_mlfreq         = PCPC_IGNORE_VALUE,  \
  .lit_mlnrrun        = PCPC_IGNORE_VALUE,  \
  .tsk_idle           = PCPC_IGNORE_VALUE,  \
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */ \
  .cpu_dma_latency    = PCPC_IGNORE_VALUE,  \
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */ \
  /* internal use */                        \
  .tsk_init_pri       = PCPC_IGNORE_VALUE,  \
  .tsk_small_pri      = PCPC_IGNORE_VALUE,  \
  .tsk_go_big_pri     = PCPC_IGNORE_VALUE,  \
  .tsk_to_lit_pri     = PCPC_IGNORE_VALUE,  \
  .tsk_nb_nice_pri    = PCPC_IGNORE_VALUE,  \
  .tsk_boost_pri      = PCPC_IGNORE_VALUE,  \
  .lit_tgtl_pri       = PCPC_IGNORE_VALUE,  \
  .big_tgtl_pri       = PCPC_IGNORE_VALUE,  \
  .lit_ahspd_pri      = PCPC_IGNORE_VALUE,  \
  .big_ahspd_pri      = PCPC_IGNORE_VALUE,  \
  .lit_smpt_pri       = PCPC_IGNORE_VALUE,  \
  .big_smpt_pri       = PCPC_IGNORE_VALUE,  \
  .lit_hspf_pri       = PCPC_IGNORE_VALUE,  \
  .big_hspf_pri       = PCPC_IGNORE_VALUE,  \
  .lit_ghspl_pri      = PCPC_IGNORE_VALUE,  \
  .big_ghspl_pri      = PCPC_IGNORE_VALUE,  \
  .lit_slkt_pri       = PCPC_IGNORE_VALUE,  \
  .big_slkt_pri       = PCPC_IGNORE_VALUE,  \
  .lit_hyst_pri       = PCPC_IGNORE_VALUE,  \
  .big_hyst_pri       = PCPC_IGNORE_VALUE,  \
  .ioisbsy_pri        = PCPC_IGNORE_VALUE,  \
  .lit_mlload_pri     = PCPC_IGNORE_VALUE,  \
  .lit_mlfreq_pri     = PCPC_IGNORE_VALUE,  \
  .lit_mlnrrun_pri    = PCPC_IGNORE_VALUE,  \
  .tsk_idle_pri       = PCPC_IGNORE_VALUE,  \
}

typedef struct {
  /* linking */
  struct list_head list_reg;
  struct list_head list_act;
  struct list_head list_func;
  /* parameters */
  pcpc_scene_param_type v;
  /* internal work */
  char recalc;
  char active;
  char funcable;
  char __pad;
  struct delayed_work work_timer;
  struct pm_qos_request pm_qos_lit_maxf;
  struct pm_qos_request pm_qos_big_maxf;
  struct pm_qos_request pm_qos_lit_minf;
  struct pm_qos_request pm_qos_big_minf;
  struct pm_qos_request pm_qos_lit_upcore;
  struct pm_qos_request pm_qos_big_upcore;
  struct pm_qos_request pm_qos_bus_min;
  struct pm_qos_request pm_qos_gpu_max;
  struct pm_qos_request pm_qos_gpu_min;
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
  struct pm_qos_request pm_qos_cpu_dma_latency;
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
} pcpc_scene_type;

/* on stack work */
typedef struct {
  struct work_struct work;
  struct completion done;
  const char *val;
  int rc;
} pcpc_sync_work_type;
typedef struct {
  struct work_struct work;
  struct completion done;
  int value;
  int target_id;
} pcpc_qos_sync_work_type;

/* QoS control */
typedef struct {
  struct notifier_block nb;
  const char *param_name;
  int param_id;
  int default_value;
} pcpc_qos_notifier_control_type;

/* internal variable */
static DEFINE_SPINLOCK(pcpc_lock);
static struct workqueue_struct *pcpcwq;
static struct work_struct input_event_work;
static struct work_struct homekey_event_work;
static struct work_struct powerkey_event_work;
static int cut_off;
static int block_trans;
static int update_all;
static int initialized;

/* registered list */
static LIST_HEAD(register_list);
/* active/inactive scene list */
static LIST_HEAD(active_list);
static LIST_HEAD(inactive_list);
/* function list */
static LIST_HEAD(func_input_list);
static int func_input_list_count;

/* active scene */
static PCPC_SCENE_PARAM_TYPE(top_most_scene_param);
/* for transaction */
static PCPC_SCENE_PARAM_TYPE(transaction_param);
/* for initialize data */
static const PCPC_SCENE_PARAM_TYPE(_initial_ignore_param);

/* for interface */
static struct kobject *module_kobj;
static struct proc_dir_entry *proc_pcpc;
static struct proc_dir_entry *proc_pcpc_config;

/* debug */
#undef DEBUG_VERY_VERBOSE  /* compile very verbose debug */
#undef DEBUG_INTERNAL      /* compile internal debug */
enum {
  DEBUG_ERR  = 1U << 0,
  DEBUG_DIS  = 1U << 1,
  DEBUG_SET  = 1U << 2,
  DEBUG_EXEC = 1U << 3,
  DEBUG_VERB = 1U << 4,
  DEBUG_VV   = 1U << 5,
};
#ifdef DEBUG_VERY_VERBOSE
static int debug_mask = DEBUG_ERR | DEBUG_DIS | DEBUG_SET | DEBUG_EXEC | DEBUG_VERB | DEBUG_VV;
#else
static int debug_mask = DEBUG_ERR | DEBUG_DIS;
#endif
module_param(debug_mask, int, 0660);

#define __PRINT_MASK(dbgmsk, prtlv, msg...) do {  \
          if(debug_mask & (dbgmsk))               \
            printk(prtlv "pcpc:" msg);            \
        } while(0)
#define PRINT_ERR(msg...)  __PRINT_MASK(DEBUG_ERR , KERN_ERR    , msg)
#define PRINT_DIS(msg...)  __PRINT_MASK(DEBUG_DIS , KERN_WARNING, msg)
#define PRINT_SET(msg...)  __PRINT_MASK(DEBUG_SET , KERN_NOTICE , msg)
#define PRINT_EXEC(msg...) __PRINT_MASK(DEBUG_EXEC, KERN_INFO   , msg)
#define PRINT_VERB(msg...) __PRINT_MASK(DEBUG_VERB, KERN_DEBUG  , msg)
#ifdef DEBUG_VERY_VERBOSE
#define PRINT_VV(msg...)   __PRINT_MASK(DEBUG_VV  , KERN_DEBUG  , msg)
#else
#define PRINT_VV(msg...)
#endif
#define PRINT_ERR_IF(cond, msg...) do { if(cond) PRINT_ERR(msg); } while(0)

#ifdef DEBUG_INTERNAL
static int __exec_work_sync(work_func_t func, const char *val);
static void __store_set_work_fn(struct work_struct *work);
static int param_set_update_all(const char *val, const struct kernel_param *kp)
{
  int ret = param_set_int(val, kp);
  __exec_work_sync(__store_set_work_fn, "");
  return ret;
}
static struct kernel_param_ops module_ops_update_all = {
  .set = param_set_update_all,
  .get = param_get_int,
};
module_param_cb(update_all, &module_ops_update_all, &update_all, 0600);
MODULE_PARM_DESC(update_all, "int");

module_param(initialized, int, 0600);
static int debug_loose_input_wq = 0;
module_param(debug_loose_input_wq, int, 0600);

/* init call test */
static int debug_module_init;
static int __pcpc_init(void);
static int param_set_debug_module_init(const char *val, const struct kernel_param *kp)
{
  if(!initialized) {
    PRINT_VERB("%s call init\n", __FUNCTION__);
    __pcpc_init();
    PRINT_VV("%s done init\n", __FUNCTION__);
  }
  return param_set_int(val, kp);
}
static struct kernel_param_ops module_ops_debug_module_init= {
  .set = param_set_debug_module_init,
  .get = param_get_int,
};
module_param_cb(debug_module_init, &module_ops_debug_module_init, &debug_module_init, 0600);
MODULE_PARM_DESC(debug_module_init, "int");

/* exit call test */
static int debug_module_exit;
static void __pcpc_exit(void);
static int param_set_debug_module_exit(const char *val, const struct kernel_param *kp)
{
  if(initialized) {
    PRINT_VERB("%s call exit\n", __FUNCTION__);
    __pcpc_exit();
    PRINT_VV("%s done exit\n", __FUNCTION__);
  }
  return param_set_int(val, kp);
}
static struct kernel_param_ops module_ops_debug_module_exit= {
  .set = param_set_debug_module_exit,
  .get = param_get_int,
};
module_param_cb(debug_module_exit, &module_ops_debug_module_exit, &debug_module_exit, 0600);
MODULE_PARM_DESC(debug_module_exit, "int");

/* ------------------------------------------------------------------------- */
/* for HMP test */
#include <linux/kthread.h>
#include "../../kernel/sched/sched.h"

extern unsigned int sched_ravg_window;
extern unsigned int pct_task_load(struct task_struct *p);
extern unsigned long long task_sched_runtime(struct task_struct *p);
extern unsigned int max_possible_freq;
extern unsigned int max_possible_efficiency;

static unsigned int debug_stress_blank_ms = 100U;   /* module_param */
static unsigned int debug_stress_report_ms = 3000U; /* module_param */
static int debug_hmp_stress_task(void *data)
{
  unsigned long level = (unsigned long)data;
  struct task_struct *p = current;
  unsigned int timer_rate;
  unsigned long long prev, now, rpt = task_sched_runtime(p);
  unsigned long long wt, wc, freq_base, eff_base;
  struct rq *rq;
  int cpu = task_cpu(p), tmp;
  int exec_cnt = 0;
  int loopcnt = 0;
  int loopcnt_max = 0;
  int loopcnt_min = INT_MAX;
  unsigned int freq_bak = 0;
  int eff_bak = 0;
  unsigned long time_sum;

  while (!kthread_should_stop()) {
    prev = task_sched_runtime(p);
    /* 1 level = about 0.01ms in big cluster max performance */
    wt = (unsigned long long)level * 10000U * max_possible_freq * max_possible_efficiency;
    timer_rate = sched_ravg_window;
    exec_cnt++;
    loopcnt = 0;
    time_sum = 0;
    wc = 0;
    freq_base = eff_base = 0;

#define SWITCH_CHECK(FMT)                                                 \
    preempt_disable();                                                    \
    tmp = task_cpu(p);                                                    \
    preempt_enable();                                                     \
    if(cpu != tmp) {                                                      \
      CHECK_PRINT(FMT)                                                    \
    }                                                                     \
    /* ----------------------------------------------------------------- */
#define CHECK_PRINT(FMT)                                                  \
      PRINT_DIS(FMT,                                                      \
        p->comm, p->ravg.demand, pct_task_load(p),                        \
        cpu, sched_get_busy(cpu),                                         \
        tmp, sched_get_busy(tmp),                                         \
        (time_sum > 0 ? (unsigned int)(freq_base / time_sum) : freq_bak), \
        (time_sum > 0 ? (int)(eff_base / time_sum) : eff_bak),            \
        time_sum,                                                         \
        loopcnt, loopcnt_min, loopcnt_max, exec_cnt);                     \
      cpu = tmp;                                                          \
      rpt = prev;                                                         \
      /* --------------------------------------------------------------- */

    while (!kthread_should_stop()) {

      SWITCH_CHECK("  m %s(%8u=%3u) cpu%d(%8lu)->cpu%d(%8lu) avg-f:%7u eff:%4d span:%8lu  loop:(%d)(avg:%d-%d) #%d\n");
      now = task_sched_runtime(p);
      rq = cpu_rq(cpu);
      freq_bak = rq->cur_freq;
      eff_bak = rq->efficiency;
      if(now > prev) {
        freq_base += (now - prev) * freq_bak;
        eff_base += (now - prev) * eff_bak;
        time_sum += (now - prev);
        wc += (now - prev) * freq_bak * eff_bak;
      }
      if(wc >= wt)
        break;
      prev = now;
      loopcnt++;
    }
    if(loopcnt_max < loopcnt)
      loopcnt_max = loopcnt;
    else
      loopcnt_max += (loopcnt - loopcnt_max) >> 5;
    if(loopcnt_min > loopcnt)
      loopcnt_min = loopcnt;
    else
      loopcnt_min += (loopcnt - loopcnt_min) >> 5;
    msleep_interruptible(debug_stress_blank_ms);

    SWITCH_CHECK("  P %s(%8u=%3u) cpu%d(%8lu)->cpu%d(%8lu) avg-f:%7u eff:%4d span:%8lu  loop:%d(avg:%d-%d) #%d\n");
    if((prev - rpt) > (unsigned long)debug_stress_report_ms * 1000000UL) {
      CHECK_PRINT(" (R)%s(%8u=%3u) cpu%d(%8lu)->cpu%d(%8lu) avg-f:%7u eff:%4d span:%8lu  loop:%d(avg:%d-%d) #%d\n");
    }
  }
  return 0;
}
/* HMP variety stress  */
static unsigned int debug_stress_min;
static unsigned int debug_stress_max;
static unsigned int debug_stress_count;
static struct task_struct *test_task[100];
#define DEBUG_MAX_STRESS 100
static int param_set_debug_stress(const char *val, const struct kernel_param *kp)
{
  unsigned int i;
  PRINT_VERB("%s call stress\n", __FUNCTION__);
  if(debug_stress_count > 0) {
    PRINT_EXEC("stop stress %d\n", debug_stress_count);
    for(i = 0; i < debug_stress_count; i++) {
      if(test_task[i])
        kthread_stop(test_task[i]);
      test_task[i] = NULL;
    }
  }
  debug_stress_min = 0;
  debug_stress_max = 0;
  debug_stress_count = 0;
  sscanf(val, "%u:%u-%u", &debug_stress_count, &debug_stress_min, &debug_stress_max);
  if(debug_stress_count > 100)
    debug_stress_count = 100;
  if(debug_stress_min > debug_stress_max)
    swap(debug_stress_min, debug_stress_max);
  PRINT_VERB("stress count:%d min:%d max:%d\n", debug_stress_count, debug_stress_min, debug_stress_max);
  if(debug_stress_count > 0) {
    PRINT_EXEC("start stress %u\n", debug_stress_count);
    for(i = 0; i < debug_stress_count; i++) {
      unsigned long level;
      if(debug_stress_count > 1)
        level = (debug_stress_max - debug_stress_min) * i / (debug_stress_count - 1) + debug_stress_min;
      else
        level = (debug_stress_max + debug_stress_min) / 2;
      test_task[i] = kthread_run(debug_hmp_stress_task, (void *)level, "pcpcst%02u:%04lu", i, level);
      if(IS_ERR(test_task[i])) {
        PRINT_DIS("start failed pcpcst%02u:%04lu\n", i, level);
        test_task[i] = NULL;
      } else {
        PRINT_EXEC("  pcpcst%02u:%04lu\n", i, level);
      }
    }
  }
  return 0;
}
static int param_get_debug_stress(char *buffer, const struct kernel_param *kp)
{
  return sprintf(buffer, "%u:%u-%u", debug_stress_count, debug_stress_min, debug_stress_max);
}
static struct kernel_param_ops module_ops_debug_stress= {
  .set = param_set_debug_stress,
  .get = param_get_debug_stress,
};
module_param_cb(debug_stress, &module_ops_debug_stress, NULL, 0600);
MODULE_PARM_DESC(debug_stress, "int");
module_param(debug_stress_blank_ms, uint, 0600);
module_param(debug_stress_report_ms, uint, 0600);

#endif


/* ==========================================================================
 *  EXTERNAL FUNCTIONS
 * ========================================================================== */

extern int sched_set_hmp_params(
                        unsigned int init_task_load,
                        unsigned int small_task,
                        unsigned int upmigrate,
                        unsigned int downmigrate,
                        int upmigrate_min_nice,
                        unsigned int flag);
extern int cpufreq_set_limit(
                        cpumask_t cpu_mask,
                        unsigned int max_freq,
                        unsigned int min_freq);
extern int cpufreq_interactive_set_freq(
                        cpumask_t cpu_mask,
                        unsigned int freq);
extern int cpufreq_interactive_set_param(
                        cpumask_t cpu_mask,
                        unsigned int *target_loads,
                        unsigned int *above_hispeed_delay,
                        unsigned long min_sample_time,
                        unsigned int hispeed_freq,
                        unsigned long go_hispeed_load,
                        int timer_slack_val,
                        unsigned int max_freq_hysteresis,
                        unsigned int flag);
extern int cpufreq_interactive_set_io_is_busy(
                        bool io_is_busy);
extern int msm_thermal_set_core_limit(
                        int num_core_limit_little,
                        int num_core_limit_big,
                        unsigned int flag);
extern void msm_thermal_reset_core_limit(void);
extern int devfreq_set_limit_freq(
                        const char *dev_name,
                        unsigned long max_freq,
                        unsigned long min_freq);
extern int sched_set_cpu_params(
                        cpumask_t cpu_mask,
                        unsigned int mostly_idle_load,
                        unsigned int mostly_idle_freq,
                        unsigned int mostly_idle_nr_run,
                        unsigned int prefer_idle,
                        unsigned int flag);
/* FUJITSU LIMITED: 2015-07-15 POWERSAVE add start */
extern int kgsl_pwrctrl_set_freq(
                        unsigned int max_freq,
                        unsigned int min_freq);
/* FUJITSU LIMITED: 2015-07-15 POWERSAVE add end   */


/* ==========================================================================
 *  INTERNAL FUNCTIONS
 * ========================================================================== */

/*!
 @brief __set_string

 set variable as char array string, accepting the ignore value

 @param [out]  set_to    pointer of the valiable to set
 @param [in]   setlen    length of the valiable to set
 @param [in]   valptr    pointer of the string value
 @param [in]   vallen    length of the string value

 @retval       none
*/
static void __set_string(char *set_to, int setlen, const char *valptr, int vallen)
{
  strlcpy(set_to, valptr, min_t(size_t, vallen + 1, setlen));
  if(!strcmp(set_to, PCPC_VALUE_STR_IGNORE)) {
    set_to[0] = '\0';
  }
}

/*!
 @brief __set_num_int

 set variable as integer, accepting the ignore value

 @param [out]  set_to    pointer of the valiable to set
 @param [in]   valptr    pointer of the string value
 @param [in]   vallen    length of the string value

 @retval       none
*/
static void __set_num_int(int *set_to, const char *valptr, int vallen)
{
  int value = 0;
  while(vallen > 0 && *valptr == ' ') {
    valptr++, vallen--;
  }
  if(vallen > 0 && *valptr == '-') {
    value = PCPC_IGNORE_VALUE;
  } else {
    while(vallen > 0 && *valptr >= '0' && *valptr <= '9') {
      value = value * 10 + (*valptr - '0');
      valptr++, vallen--;
    }
  }
  *set_to = value;
}

/*!
 @brief __set_signed_int

 set variable as signed integer, accepting the ignore value

 @param [out]  set_to    pointer of the valiable to set
 @param [in]   valptr    pointer of the string value
 @param [in]   vallen    length of the string value

 @retval       none
*/
static void __set_signed_int(int *set_to, const char *valptr, int vallen)
{
  int value = 0;
  int neg = 0;
  while(vallen > 0 && *valptr == ' ') {
    valptr++, vallen--;
  }
  if(vallen > 0 && *valptr == '-') {
    valptr++, vallen--;
    if(vallen > 0 && *valptr >= '0' && *valptr <= '9')
      neg = 1;
    else
      value = PCPC_IGNORE_VALUE;
  }
  if(value != PCPC_IGNORE_VALUE) {
    while(vallen > 0 && *valptr >= '0' && *valptr <= '9') {
      value = value * 10 + (*valptr - '0');
      valptr++, vallen--;
    }
    if(neg)
      value = -value;
  }
  *set_to = value;
}

/*!
 @brief __set_sep_freq_ptr

 set variable as frequency separated array, accepting the ignore value (=NULL)
 set NULL if ignored
 set ERR_PTR(err) if unsuccessful
 set allocated memory pointer and make as follows to successful:
      array[0] = nargs
      array[1...(nargs + 1)] = args[]
    ex)
      input:  "10 300000:20 600000:30"
      output: array[0] = 5
              array[1] = 10
              array[2] = 300000
              array[3] = 20
              array[4] = 600000
              array[5] = 30

 @param [out]  set_to    pointer of the valiable to set
 @param [in]   valptr    pointer of the string value
 @param [in]   vallen    length of the string value

 @retval       none
*/
static void __set_sep_freq_ptr(unsigned int **set_to, const char *valptr, int vallen)
{
  int i;
  int value;
  unsigned int narg;

  while(vallen > 0 && *valptr == ' ') {
    valptr++, vallen--;
  }
  while(vallen > 0 && valptr[vallen - 1] == ' ') {
    vallen--;
  }
  if(vallen <= 0 || (vallen > 0 && *valptr == '-')) {
    *set_to = NULL;
    return;
  } 
  narg = 1;
  for(i = 0; i < vallen; i++) {
    if(valptr[i] == ' ' || valptr[i] == ':')
      narg++;
  }
  if(!(narg & 0x1)) {
    PRINT_DIS("%s mismatch the number of separator\n", __FUNCTION__);
    *set_to = ERR_PTR(-EINVAL);
    return;
  }

  *set_to = kmalloc((narg + 1) * sizeof(unsigned int), GFP_KERNEL);
  if(!(*set_to)) {
    PRINT_DIS("%s malloc error\n", __FUNCTION__);
    *set_to = ERR_PTR(-ENOMEM);
    return;
  }
  **set_to = narg;

  value = PCPC_IGNORE_VALUE;
  narg = 1;
  while(vallen > 0) {
    if(*valptr == ' ' || *valptr == ':') {
      if(value == PCPC_IGNORE_VALUE || narg > **set_to) {
        PRINT_DIS("%s unspecified value found\n", __FUNCTION__);
        goto err_kfree;
      } else {
        (*set_to)[narg++] = value;
        value = PCPC_IGNORE_VALUE;
      }
    } else if(*valptr >= '0' && *valptr <= '9') {
      if(value == PCPC_IGNORE_VALUE) {
        value = (*valptr - '0');
      } else {
        value = value * 10 + (*valptr - '0');
      }
      if(value < 0) {
        PRINT_DIS("%s overflow value %d\n", __FUNCTION__, value);
        goto err_kfree;
      }
    } else {
      PRINT_DIS("%s illegal caharacter '%c'\n", __FUNCTION__, *valptr);
      goto err_kfree;
    }
    valptr++, vallen--;
  }
  if(value == PCPC_IGNORE_VALUE || narg != **set_to) {
    PRINT_DIS("%s unspecified value found\n", __FUNCTION__);
    goto err_kfree;
  } else {
    (*set_to)[narg] = value;
  }
#ifdef DEBUG_VERY_VERBOSE
  PRINT_VV("%s made array %p={\n", __FUNCTION__, *set_to);
  for(i = 0; i <= narg; i++) {
    PRINT_VV("  [%2d] = %u\n", i, (*set_to)[i]);
  }
  PRINT_VV("};\n");
#endif

  return;

err_kfree:
  kfree(*set_to);
  *set_to = ERR_PTR(-EINVAL);
  return;
}

/*!
 @brief __scnprintf_string

 make formatted string for string parameter

 @param [out]  buf       pointer of the output buffer
 @param [in]   size      length of the output buffer
 @param [in]   value     the string
 @param [in]   sep       additional separator strings

 @retval       length of the string written to buffer (not include null terminated character)
*/
static int __scnprintf_string(char *buf, size_t size, const char *value, const char *sep)
{
  if(value[0] == '\0') {
    return scnprintf(buf, size, "-%s", sep);
  } else {
    return scnprintf(buf, size, "%s%s", value, sep);
  }
}

/*!
 @brief __scnprintf_string

 make formatted string for integer parameter

 @param [out]  buf       pointer of the output buffer
 @param [in]   size      length of the output buffer
 @param [in]   value     the integer value
 @param [in]   sep       additional separator strings

 @retval       length of the string written to buffer (not include null terminated character)
*/
static int __scnprintf_num_int(char *buf, size_t size, int value, const char *sep)
{
  if(value == PCPC_IGNORE_VALUE) {
    return scnprintf(buf, size, "-%s", sep);
  } else {
    return scnprintf(buf, size, "%d%s", value, sep);
  }
}

/*!
 @brief __scnprintf_sep_freq

  make formatted string for frequency separated string parameter

 @param [out]  buf       pointer of the output buffer
 @param [in]   size      length of the output buffer
 @param [in]   dataptr   pointer of the valiable
 @param [in]   sep       additional separator strings

 @retval       length of the string written to buffer (not include null terminated character)
*/
static int __scnprintf_sep_freq(char *buf, size_t size, unsigned int *dataptr, const char *sep)
{
  int count;
  int narg;

  if(dataptr == NULL) {
    return scnprintf(buf, size, "-%s", sep);
  } else {
    count = 0;
    for(narg = (int)(*dataptr++); narg > 0; narg -= 2) {
      count += scnprintf(buf + count, size - count, "%u", *dataptr++);
      if(narg > 1) {
        count += scnprintf(buf + count, size - count, " %u:", *dataptr++);
      }
    }
    count += scnprintf(buf + count, size - count, "%s", sep);
  }
  return count;
}

/*!
 @brief __to_param_func_string

 revert to string func

 @param [in]   none

 @retval    0: none
*/
static const char *__to_param_func_string(int func)
{
  if(func == PCPC_VALUE_FUNC_INPUT_ID) {
    return PCPC_VALUE_FUNC_STR_INPUT;
  }
  if(func == PCPC_VALUE_FUNC_HOMEKEY_ID) {
    return PCPC_VALUE_FUNC_STR_HOMEKEY;
  }
  if(func == PCPC_VALUE_FUNC_POWERKEY_ID) {
    return PCPC_VALUE_FUNC_STR_POWERKEY;
  }
  return PCPC_VALUE_STR_IGNORE;
}

/*!
 @brief __cmp_sep_freq

  compare between two frequency separated string parameter

 @param [in]   data1     first pointer of the valiable
 @param [in]   data2     second pointer of the valiable

 @retval       0: same data (or both null)
               not 0: different data
*/
static int __cmp_sep_freq(unsigned int *data1, unsigned int *data2)
{
  if(data1 == data2)
    return 0;

  if(data1 == NULL || data2 == NULL)
    return ((data1 != NULL ? 1 : 0) - (data2 != NULL ? 1 : 0));

  if(*data1 != *data2)
    return ((int)*data1 - (int)*data2);

  return memcmp(&data1[1], &data2[1], *data1 * sizeof(unsigned int));
}

/*!
 @brief __begin_transaction

 begin transaction

 @param        none

 @retval       none
*/
static void __begin_transaction(void)
{
  unsigned long flags;

  spin_lock_irqsave(&pcpc_lock, flags);
  block_trans++;
  PRINT_VERB("++trans %d\n", block_trans);
  spin_unlock_irqrestore(&pcpc_lock, flags);
}

/*!
 @brief __end_transaction

 execute transaction

 @param [in]   none

 @retval       none
*/
static void __end_transaction(void)
{
  unsigned long flags;
  PCPC_SCENE_PARAM_TYPE(changed_scene_param);
  pcpc_scene_type *pos;
  unsigned int do_request_hmp = 0;
  unsigned int do_request_boost = 0;
  unsigned int do_request_freq = 0;
  unsigned int do_request_limit = 0;
  unsigned int do_request_interact0 = 0;
  unsigned int do_request_interact1 = 0;
  unsigned int do_request_interact_iib = 0;
  unsigned int do_request_core = 0;
  unsigned int do_request_bus = 0;
  unsigned int do_request_gpu = 0;
  unsigned int do_request_cpu = 0;
  int skip = 0;

  spin_lock_irqsave(&pcpc_lock, flags);
  block_trans--;
  PRINT_VERB("--trans %d\n", block_trans);
  if(initialized) {
    if(!block_trans) {

      /* check activated */
      list_for_each_entry(pos, &active_list, list_act) {
        if(pos->recalc) {
          pos->recalc = 0;
          /* raise the cluster frequency when activate */
#define RECALC_INSF_CHECK(mbr)                                                \
          if(pos->v.mbr != PCPC_IGNORE_VALUE) {                               \
            PRINT_VV("check " #mbr " max:%d %s:%d\n",                         \
              changed_scene_param.mbr, pos->v.scene, pos->v.mbr);             \
            if(changed_scene_param.mbr == PCPC_IGNORE_VALUE ||                \
               changed_scene_param.mbr < pos->v.mbr) {                        \
               changed_scene_param.mbr = pos->v.mbr;                          \
            }                                                                 \
          } /*****************************************************************/
          RECALC_INSF_CHECK(lit_insf    )
          RECALC_INSF_CHECK(big_insf    )
        }
      }

      /* check changed */
#define CHECK_CHANGED_INT(mbr,grp,flg)                                        \
      if((transaction_param.mbr != PCPC_IGNORE_VALUE) &&                      \
         (update_all || top_most_scene_param.mbr != transaction_param.mbr)) { \
        PRINT_SET("active." #mbr " %d->%d\n",                                 \
          top_most_scene_param.mbr, transaction_param.mbr);                   \
        grp |= BIT(flg);                                                      \
      } /*********************************************************************/
#define CHECK_CHANGED_INT_READY(mbr,grp,flg)                                  \
      if(changed_scene_param.mbr != PCPC_IGNORE_VALUE) {                      \
        PRINT_SET("active." #mbr " ->%d\n", changed_scene_param.mbr);         \
        grp |= BIT(flg);                                                      \
      } /*********************************************************************/
#define CHECK_CHANGED_SEP_FREQ(mbr,grp,flg)                                   \
      if((transaction_param.mbr != NULL) &&                                   \
         (update_all ||                                                       \
          __cmp_sep_freq(top_most_scene_param.mbr, transaction_param.mbr))) { \
        PRINT_SET("active." #mbr " %p->%p\n",                                 \
          top_most_scene_param.mbr, transaction_param.mbr);                   \
        grp |= BIT(flg);                                                      \
      } /*********************************************************************/
#define SET_CHANGED_PARAM(mbr) changed_scene_param.mbr = transaction_param.mbr
#define SET_CHANGED_PARAM_INT_PAD(mbr,dfl)                                    \
      if(transaction_param.mbr != PCPC_IGNORE_VALUE) {                        \
        changed_scene_param.mbr = transaction_param.mbr;                      \
      } else {                                                                \
        changed_scene_param.mbr = (dfl);                                      \
      } /*********************************************************************/

      /* sched hmp group (depends on pri) */
      CHECK_CHANGED_INT(tsk_init    , do_request_hmp, 0)
      CHECK_CHANGED_INT(tsk_small   , do_request_hmp, 1)
      CHECK_CHANGED_INT(tsk_go_big  , do_request_hmp, 2)
      CHECK_CHANGED_INT(tsk_to_lit  , do_request_hmp, 3)
      CHECK_CHANGED_INT(tsk_nb_nice , do_request_hmp, 4)
      if (do_request_hmp) {
        SET_CHANGED_PARAM(tsk_init    );
        SET_CHANGED_PARAM(tsk_small   );
        SET_CHANGED_PARAM(tsk_go_big  );
        SET_CHANGED_PARAM(tsk_to_lit  );
        SET_CHANGED_PARAM(tsk_nb_nice );
      }
      /* sched boost (depends on pri) */
      CHECK_CHANGED_INT(tsk_boost   , do_request_boost, 0)
      SET_CHANGED_PARAM_INT_PAD(tsk_boost   , 0);
      /* freq group */
      CHECK_CHANGED_INT_READY(lit_insf    , do_request_freq, 0)
      CHECK_CHANGED_INT_READY(big_insf    , do_request_freq, 1)
      /* limit group */
      CHECK_CHANGED_INT(lit_maxf    , do_request_limit, 0)
      CHECK_CHANGED_INT(big_maxf    , do_request_limit, 1)
      CHECK_CHANGED_INT(lit_minf    , do_request_limit, 0)
      CHECK_CHANGED_INT(big_minf    , do_request_limit, 1)
      if (do_request_limit & BIT(0)) {
        SET_CHANGED_PARAM_INT_PAD(lit_maxf    , INT_MAX);
        SET_CHANGED_PARAM_INT_PAD(lit_minf    , 0);
      }
      if (do_request_limit & BIT(1)) {
        SET_CHANGED_PARAM_INT_PAD(big_maxf    , INT_MAX);
        SET_CHANGED_PARAM_INT_PAD(big_minf    , 0);
      }
      /* cpufreq interactive group */
      CHECK_CHANGED_SEP_FREQ(lit_tgtl    , do_request_interact0, 0)
      CHECK_CHANGED_SEP_FREQ(lit_ahspd   , do_request_interact0, 1)
      CHECK_CHANGED_INT(lit_smpt    , do_request_interact0, 2)
      CHECK_CHANGED_INT(lit_hspf    , do_request_interact0, 3)
      CHECK_CHANGED_INT(lit_ghspl   , do_request_interact0, 4)
      CHECK_CHANGED_INT(lit_slkt    , do_request_interact0, 5)
      CHECK_CHANGED_INT(lit_hyst    , do_request_interact0, 6)
      if (do_request_interact0) {
        SET_CHANGED_PARAM(lit_tgtl    );
        SET_CHANGED_PARAM(lit_ahspd   );
        SET_CHANGED_PARAM(lit_smpt    );
        SET_CHANGED_PARAM(lit_hspf    );
        SET_CHANGED_PARAM(lit_ghspl   );
        SET_CHANGED_PARAM(lit_slkt    );
        SET_CHANGED_PARAM(lit_hyst    );
      }
      CHECK_CHANGED_SEP_FREQ(big_tgtl    , do_request_interact1, 0)
      CHECK_CHANGED_SEP_FREQ(big_ahspd   , do_request_interact1, 1)
      CHECK_CHANGED_INT(big_smpt    , do_request_interact1, 2)
      CHECK_CHANGED_INT(big_hspf    , do_request_interact1, 3)
      CHECK_CHANGED_INT(big_ghspl   , do_request_interact1, 4)
      CHECK_CHANGED_INT(big_slkt    , do_request_interact1, 5)
      CHECK_CHANGED_INT(big_hyst    , do_request_interact1, 6)
      if (do_request_interact1) {
        SET_CHANGED_PARAM(big_tgtl    );
        SET_CHANGED_PARAM(big_ahspd   );
        SET_CHANGED_PARAM(big_smpt    );
        SET_CHANGED_PARAM(big_hspf    );
        SET_CHANGED_PARAM(big_ghspl   );
        SET_CHANGED_PARAM(big_slkt    );
        SET_CHANGED_PARAM(big_hyst    );
      }
      CHECK_CHANGED_INT(ioisbsy     , do_request_interact_iib, 0)
      if (do_request_interact_iib) {
        SET_CHANGED_PARAM(ioisbsy     );
      }
      /* core group */
      CHECK_CHANGED_INT(lit_upcore  , do_request_core, 0)
      CHECK_CHANGED_INT(big_upcore  , do_request_core, 1)
      if (do_request_core) {
        SET_CHANGED_PARAM(lit_upcore  );
        SET_CHANGED_PARAM(big_upcore  );
      }
      /* bus group */
      CHECK_CHANGED_INT(bus_min     , do_request_bus, 0)
      if (do_request_bus) {
        SET_CHANGED_PARAM_INT_PAD(bus_min     , DEVFREQ_LIMIT_DONTUSE);
      }
      /* gpu group */
      CHECK_CHANGED_INT(gpu_max     , do_request_gpu, 0)
      CHECK_CHANGED_INT(gpu_min     , do_request_gpu, 1)
      if (do_request_gpu) {
        SET_CHANGED_PARAM_INT_PAD(gpu_max     , DEVFREQ_LIMIT_DONTUSE);
        SET_CHANGED_PARAM_INT_PAD(gpu_min     , DEVFREQ_LIMIT_DONTUSE);
      }
      /* sched hmp group (depends on pri) */
      CHECK_CHANGED_INT(lit_mlload   , do_request_cpu, 0)
      CHECK_CHANGED_INT(lit_mlfreq   , do_request_cpu, 1)
      CHECK_CHANGED_INT(lit_mlnrrun  , do_request_cpu, 2)
      CHECK_CHANGED_INT(tsk_idle     , do_request_cpu, 3)
      if (do_request_cpu) {
        SET_CHANGED_PARAM(lit_mlload   );
        SET_CHANGED_PARAM(lit_mlfreq   );
        SET_CHANGED_PARAM(lit_mlnrrun  );
        SET_CHANGED_PARAM(tsk_idle     );
      }

      /* commit current scene parameters */
      memcpy(&top_most_scene_param, &transaction_param, sizeof(pcpc_scene_param_type));

      if(update_all) {
        PRINT_VV("  update_all %d->%d\n", update_all, 0);
        update_all = 0;
      }
    }
  } else {
    PRINT_DIS("not ready.\n");
    skip = 1;
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(!skip) {
    /* send the parameters */
    if (do_request_hmp) {
      PRINT_EXEC("update"
                 " tsk_init=%d"
                 " tsk_small=%d"
                 " tsk_go_big=%d"
                 " tsk_to_lit=%d"
                 " tsk_nb_nice=%d"
                 " flag=%d\n",
                 changed_scene_param.tsk_init    ,
                 changed_scene_param.tsk_small   ,
                 changed_scene_param.tsk_go_big  ,
                 changed_scene_param.tsk_to_lit  ,
                 changed_scene_param.tsk_nb_nice ,
                 do_request_hmp);
      sched_set_hmp_params((unsigned int)changed_scene_param.tsk_init    ,
                           (unsigned int)changed_scene_param.tsk_small   ,
                           (unsigned int)changed_scene_param.tsk_go_big  ,
                           (unsigned int)changed_scene_param.tsk_to_lit  ,
                           (int)         changed_scene_param.tsk_nb_nice ,
                           do_request_hmp);
    }
    if (do_request_boost) {
      static bool old_request;
      bool new_request = !!changed_scene_param.tsk_boost;
      PRINT_EXEC("update tsk_boost=%d\n", changed_scene_param.tsk_boost   );
      if(old_request != new_request) {
        sched_set_boost(new_request);
      }
      old_request = new_request;
    }
    if (do_request_limit & BIT(0)) {
      if (changed_scene_param.lit_minf > changed_scene_param.lit_maxf)
          changed_scene_param.lit_minf = changed_scene_param.lit_maxf;
      PRINT_EXEC("update lit_maxf=%d lit_minf=%d\n",
                 changed_scene_param.lit_maxf    ,
                 changed_scene_param.lit_minf    );
      cpufreq_set_limit(LITTLE_CLUSTER_MASK,
                        (unsigned int) changed_scene_param.lit_maxf    ,
                        (unsigned int) changed_scene_param.lit_minf    );
    }
    if (do_request_limit & BIT(1)) {
      if (changed_scene_param.big_minf > changed_scene_param.big_maxf)
          changed_scene_param.big_minf = changed_scene_param.big_maxf;
      PRINT_EXEC("update big_maxf=%d big_minf=%d\n",
                 changed_scene_param.big_maxf    ,
                 changed_scene_param.big_minf    );
      cpufreq_set_limit(BIG_CLUSTER_MASK   ,
                        (unsigned int) changed_scene_param.big_maxf    ,
                        (unsigned int) changed_scene_param.big_minf    );
    }
    if (do_request_interact0) {
      PRINT_EXEC("update"
                 " lit_tgtl=%p"
                 " lit_ahspd=%p"
                 " lit_smpt=%d"
                 " lit_hspf=%d"
                 " lit_ghspl=%d"
                 " lit_slkt=%d"
                 " lit_hyst=%d"
                 " flag=%d\n",
                 changed_scene_param.lit_tgtl    ,
                 changed_scene_param.lit_ahspd   ,
                 changed_scene_param.lit_smpt    ,
                 changed_scene_param.lit_hspf    ,
                 changed_scene_param.lit_ghspl   ,
                 changed_scene_param.lit_slkt    ,
                 changed_scene_param.lit_hyst    ,
                 do_request_interact0);
      cpufreq_interactive_set_param(LITTLE_CLUSTER_MASK,
                                    changed_scene_param.lit_tgtl    ,
                                    changed_scene_param.lit_ahspd   ,
                                    (unsigned long)changed_scene_param.lit_smpt    ,
                                    (unsigned int) changed_scene_param.lit_hspf    ,
                                    (unsigned long)changed_scene_param.lit_ghspl   ,
                                    (int)          changed_scene_param.lit_slkt    ,
                                    (unsigned int) changed_scene_param.lit_hyst    ,
                                    do_request_interact0);
    }
    if (do_request_interact1) {
      PRINT_EXEC("update"
                 " big_tgtl=%p"
                 " big_ahspd=%p"
                 " big_smpt=%d"
                 " big_hspf=%d"
                 " big_ghspl=%d"
                 " big_slkt=%d"
                 " big_hyst=%d"
                 " flag=%d\n",
                 changed_scene_param.big_tgtl    ,
                 changed_scene_param.big_ahspd   ,
                 changed_scene_param.big_smpt    ,
                 changed_scene_param.big_hspf    ,
                 changed_scene_param.big_ghspl   ,
                 changed_scene_param.big_slkt    ,
                 changed_scene_param.big_hyst    ,
                 do_request_interact1);
      cpufreq_interactive_set_param(BIG_CLUSTER_MASK,
                                    changed_scene_param.big_tgtl    ,
                                    changed_scene_param.big_ahspd   ,
                                    (unsigned long)changed_scene_param.big_smpt    ,
                                    (unsigned int) changed_scene_param.big_hspf    ,
                                    (unsigned long)changed_scene_param.big_ghspl   ,
                                    (int)          changed_scene_param.big_slkt    ,
                                    (unsigned int) changed_scene_param.big_hyst    ,
                                    do_request_interact1);
    }
    if (do_request_freq & BIT(0)) {
      PRINT_EXEC("update lit_insf=%d\n", changed_scene_param.lit_insf    );
      cpufreq_interactive_set_freq(LITTLE_CLUSTER_MASK, (unsigned int) changed_scene_param.lit_insf);
    }
    if (do_request_freq & BIT(1)) {
      PRINT_EXEC("update big_insf=%d\n", changed_scene_param.big_insf    );
      cpufreq_interactive_set_freq(BIG_CLUSTER_MASK   , (unsigned int) changed_scene_param.big_insf);
    }
    if (do_request_interact_iib) {
      PRINT_EXEC("update ioisbsy=%d\n", changed_scene_param.ioisbsy);
      cpufreq_interactive_set_io_is_busy(changed_scene_param.ioisbsy);
    }
    if (do_request_core) {
      PRINT_EXEC("update lit_upcore=%d big_upcore=%d flag=%d\n",
                 changed_scene_param.lit_upcore  ,
                 changed_scene_param.big_upcore  ,
                 do_request_core);
      msm_thermal_set_core_limit(changed_scene_param.lit_upcore  ,
                                 changed_scene_param.big_upcore  ,
                                 do_request_core);
    }
    if (do_request_bus) {
      PRINT_EXEC("update bus_min=%d\n", changed_scene_param.bus_min);
      devfreq_set_limit_freq(DEVICE_NAME_BUS,
                             DEVFREQ_LIMIT_DONTUSE,
                             (unsigned long)changed_scene_param.bus_min);
    }
    if (do_request_gpu) {
      PRINT_EXEC("update gpu_max=%d gpu_min=%d\n",
                 changed_scene_param.gpu_max     ,
                 changed_scene_param.gpu_min     );
/* FUJITSU LIMITED: 2015-07-15 POWERSAVE mod start */
      kgsl_pwrctrl_set_freq((unsigned int)changed_scene_param.gpu_max,
                            (unsigned int)changed_scene_param.gpu_min);
/* FUJITSU LIMITED: 2015-07-15 POWERSAVE mod end   */
    }
    if (do_request_cpu) {
      PRINT_EXEC("update"
                 " lit_mlload=%d"
                 " lit_mlfreq=%d"
                 " lit_mlnrrun=%d"
                 " tsk_idle=%d"
                 " flag=%d\n",
                 changed_scene_param.lit_mlload   ,
                 changed_scene_param.lit_mlfreq   ,
                 changed_scene_param.lit_mlnrrun  ,
                 changed_scene_param.tsk_idle     ,
                 do_request_cpu);
      sched_set_cpu_params(LITTLE_CLUSTER_MASK,
                           (unsigned int)changed_scene_param.lit_mlload   ,
                           (unsigned int)changed_scene_param.lit_mlfreq   ,
                           (unsigned int)changed_scene_param.lit_mlnrrun  ,
                           (unsigned int)changed_scene_param.tsk_idle     ,
                           do_request_cpu);
    }
  }
}

/*!
 @brief __find_registered_scene_by_name

 find registered scene by scene name string

 @param [in]   name      scene name string (don't care null terminated character)
 @param [in]   name_len  length of scene name string

 @retval  <>0: pointer of the registered scene data that was found
 @retval    0: was not found
*/
static pcpc_scene_type * __find_registered_scene_by_name(const char *name, int name_len)
{
  unsigned long flags;
  pcpc_scene_type *pos, *ret = NULL;

  PRINT_VV("%s %c..(%d)\n", __FUNCTION__, (name[0] != '\0' ? name[0] : '.'), name_len);

  if(name_len <= 0 || name_len >= PCPC_SCENE_NAME_LEN) {
    PRINT_DIS("%s bad name_len %d\n", __FUNCTION__, name_len);
    return ret;
  }
  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {
    list_for_each_entry(pos, &register_list, list_reg) {
      if(pos->v.scene[name_len] =='\0' && !strncmp(name, pos->v.scene, name_len)) {
        PRINT_VV("found\n");
        ret = pos;
        break;
      }
    }
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);
  return ret;
}

/*!
 @brief __exec_work_sync

 execute work and wait conpletion

 @param [in]   func       work function
 @param [in]   val        data pointer

 @retval    0: success
 @retval  <>0: failed
*/
static int __exec_work_sync(work_func_t func, const char *val)
{
  unsigned long flags;
  int skip = 0;
  int ret = -EPERM;
  pcpc_sync_work_type psw;

  spin_lock_irqsave(&pcpc_lock, flags);
  if(!initialized) {
    skip = 1;
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);
  if(!skip) {
    INIT_WORK_ONSTACK(&psw.work, func);
    init_completion(&psw.done);
    psw.val = val;
    queue_work(pcpcwq, &psw.work);
    wait_for_completion(&psw.done);
    ret =psw.rc;
    destroy_work_on_stack(&psw.work);
  }
  return ret;
}

/*!
 @brief __update_param_qos_work_fn

 update the parameter by request from QoS outer

 @param [in]   work     pointer of work_struct

 @retval       none
*/
static void __update_param_qos(int value, int target_id, int requested);
static void __update_param_qos_work_fn(struct work_struct *work)
{
  pcpc_qos_sync_work_type *ppqsw = container_of(work, pcpc_qos_sync_work_type, work);
  __update_param_qos(ppqsw->value, ppqsw->target_id, 0);
  complete(&ppqsw->done);
  return;
}

/*!
 @brief __update_param_qos

 update the parameter via QoS 

 @param [in]   value      parameter data
 @param [in]   target_id  parameter identifier
 @param [in]   requested  1:from qos callback, 0:other

 @retval       none
*/
static void __update_param_qos(int value, int target_id, int requested)
{
  unsigned long flags;
  int do_request_work = 0;
  pcpc_qos_sync_work_type pqsw;
  char *update_str = NULL;

  PRINT_VERB("%s %d,%d,%d\n", __FUNCTION__, value, target_id, requested);
  if(!requested) {
    __begin_transaction();
  }
  spin_lock_irqsave(&pcpc_lock, flags);
  if(block_trans || !requested) {
    switch(target_id) {
#define PCPC_PARAM_QOS_CASE(mbr,lbl) \
    case PCPC_PARAM_##lbl##_ID:      \
      transaction_param.mbr = value; \
      update_str = PCPC_PARAM_##lbl; \
      break /* -------------------- */
    PCPC_PARAM_QOS_CASE(lit_maxf    , LIT_MAXF    );
    PCPC_PARAM_QOS_CASE(big_maxf    , BIG_MAXF    );
    PCPC_PARAM_QOS_CASE(lit_minf    , LIT_MINF    );
    PCPC_PARAM_QOS_CASE(big_minf    , BIG_MINF    );
    PCPC_PARAM_QOS_CASE(lit_upcore  , LIT_UPCORE  );
    PCPC_PARAM_QOS_CASE(big_upcore  , BIG_UPCORE  );
    PCPC_PARAM_QOS_CASE(bus_min     , BUS_MIN     );
    PCPC_PARAM_QOS_CASE(gpu_max     , GPU_MAX     );
    PCPC_PARAM_QOS_CASE(gpu_min     , GPU_MIN     );
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
/*    PCPC_PARAM_QOS_CASE(cpu_dma_latency, CPU_DMA_LAT); */ /* do other */
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
    default:
      PRINT_DIS("%s: ignore id:%d\n", __FUNCTION__, target_id);
      break;
    }
    if(update_str) {
      PRINT_SET("choose.%s=%d\n", update_str, value);
    }
  } else {
    if(initialized) {
      do_request_work = 1;
    } else {
      PRINT_DIS("not ready.\n");
    }
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(!requested) {
    __end_transaction();
  }
  if(do_request_work) {
    INIT_WORK_ONSTACK(&pqsw.work, __update_param_qos_work_fn);
    init_completion(&pqsw.done);
    pqsw.value = value;
    pqsw.target_id = target_id;
    queue_work(pcpcwq, &pqsw.work);
    wait_for_completion(&pqsw.done);
    destroy_work_on_stack(&pqsw.work);
  }

  PRINT_VV("%s ret\n", __FUNCTION__);
}

/*!
 @brief __activate_scene

 activate the scene

 @param [in]   scene       pointer of the scene data to activate
 @param [in]   requested   1:from set request, 0:other

 @retval       none
*/
static void __activate_scene(pcpc_scene_type *scene, int requested)
{
  unsigned long flags;
  pcpc_scene_type *pos;
  struct list_head *set_to = &active_list;
  int ignore_move = 0;
  int ignore_pri         = 0;
  int ignore_tsk_init    = 0;
  int ignore_tsk_small   = 0;
  int ignore_tsk_go_big  = 0;
  int ignore_tsk_to_lit  = 0;
  int ignore_tsk_nb_nice = 0;
  int ignore_tsk_boost   = 0;
  int ignore_lit_tgtl    = 0;
  int ignore_big_tgtl    = 0;
  int ignore_lit_ahspd   = 0;
  int ignore_big_ahspd   = 0;
  int ignore_lit_smpt    = 0;
  int ignore_big_smpt    = 0;
  int ignore_lit_hspf    = 0;
  int ignore_big_hspf    = 0;
  int ignore_lit_ghspl   = 0;
  int ignore_big_ghspl   = 0;
  int ignore_lit_slkt    = 0;
  int ignore_big_slkt    = 0;
  int ignore_lit_hyst    = 0;
  int ignore_big_hyst    = 0;
  int ignore_ioisbsy     = 0;
  int ignore_lit_mlload  = 0;
  int ignore_lit_mlfreq  = 0;
  int ignore_lit_mlnrrun = 0;
  int ignore_tsk_idle    = 0;
  int skip = 0;

  PRINT_VERB("%s %s,%d\n", __FUNCTION__, scene->v.scene, requested);

  if(scene->v.pri == PCPC_IGNORE_VALUE) {
    ignore_pri = 1;
  }
#define ACTIVATE_IGNORE_CHECK(mbr)        \
  if(scene->v.mbr == PCPC_IGNORE_VALUE) { \
    ignore_##mbr = 1;                     \
  } /*************************************/
#define ACTIVATE_NULL_CHECK(mbr)          \
  if(scene->v.mbr == NULL) {              \
    ignore_##mbr = 1;                     \
  } /*************************************/
  ACTIVATE_IGNORE_CHECK(tsk_init    )
  ACTIVATE_IGNORE_CHECK(tsk_small   )
  ACTIVATE_IGNORE_CHECK(tsk_go_big  )
  ACTIVATE_IGNORE_CHECK(tsk_to_lit  )
  ACTIVATE_IGNORE_CHECK(tsk_nb_nice )
  ACTIVATE_IGNORE_CHECK(tsk_boost   )
  ACTIVATE_NULL_CHECK(lit_tgtl    )
  ACTIVATE_NULL_CHECK(big_tgtl    )
  ACTIVATE_NULL_CHECK(lit_ahspd   )
  ACTIVATE_NULL_CHECK(big_ahspd   )
  ACTIVATE_IGNORE_CHECK(lit_smpt    )
  ACTIVATE_IGNORE_CHECK(big_smpt    )
  ACTIVATE_IGNORE_CHECK(lit_hspf    )
  ACTIVATE_IGNORE_CHECK(big_hspf    )
  ACTIVATE_IGNORE_CHECK(lit_ghspl   )
  ACTIVATE_IGNORE_CHECK(big_ghspl   )
  ACTIVATE_IGNORE_CHECK(lit_slkt    )
  ACTIVATE_IGNORE_CHECK(big_slkt    )
  ACTIVATE_IGNORE_CHECK(lit_hyst    )
  ACTIVATE_IGNORE_CHECK(big_hyst    )
  ACTIVATE_IGNORE_CHECK(ioisbsy     )
  ACTIVATE_IGNORE_CHECK(lit_mlload  )
  ACTIVATE_IGNORE_CHECK(lit_mlfreq  )
  ACTIVATE_IGNORE_CHECK(lit_mlnrrun )
  ACTIVATE_IGNORE_CHECK(tsk_idle    )

  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {
    /* function */
    if(scene->v.func == PCPC_VALUE_FUNC_INPUT_ID ||
       scene->v.func == PCPC_VALUE_FUNC_POWERKEY_ID ||
       scene->v.func == PCPC_VALUE_FUNC_HOMEKEY_ID) {
      if(requested) {
        PRINT_SET("function enable scene %s %d->%d\n", scene->v.scene, scene->funcable, 1);
        if(!list_empty(&scene->list_func)) {
          PRINT_VV("  del func\n");
          list_del(&scene->list_func);
          func_input_list_count--;
        }
        PRINT_VV("  add func\n");
        list_add(&scene->list_func, &func_input_list);
        func_input_list_count++;
        scene->funcable = 1;
        if(!scene->active) {
          skip = 1;
        }
      }
      else if(!scene->funcable) {
        PRINT_DIS("already disabled scene %s\n", scene->v.scene);
        skip = 1;
      }
    }
    /* activate */
    if(!skip) {
      /* priority sort (if it is the same as priority, after enabled) */
      PRINT_VV("check priority\n");
      list_for_each_entry(pos, &active_list, list_act) {
        if(pos == scene) {
          PRINT_VV("  already %s\n", pos->v.scene);
          ignore_move = 1;
          break;
        }
        if((pos->v.pri   != PCPC_IGNORE_VALUE ? pos->v.pri   : -1) <=
           (scene->v.pri != PCPC_IGNORE_VALUE ? scene->v.pri : -1)) {
          PRINT_VV("  set previous of %s\n", pos->v.scene);
          set_to = pos->list_act.prev;
          break;
        }
        /* check higher priority parameters */
        if(!ignore_pri && pos->v.pri != PCPC_IGNORE_VALUE) {
          PRINT_VV("  higher %s.pri=%d\n", pos->v.scene, pos->v.pri);
          ignore_pri = 1;
        }
#define ACTIVATE_HIGHER_CHECK_INT(mbr)                                         \
        if(!ignore_##mbr && pos->v.mbr != PCPC_IGNORE_VALUE) {                 \
          PRINT_VV("  higher %s." #mbr "(pri)=%d(%d)\n",                       \
            pos->v.scene, pos->v.mbr, pos->v.pri);                             \
          ignore_##mbr = 1;                                                    \
        } /********************************************************************/
#define ACTIVATE_HIGHER_CHECK_PTR(mbr)                                         \
        if(!ignore_##mbr && pos->v.mbr != NULL) {                              \
          PRINT_VV("  higher %s." #mbr "(pri)=%p(%d)\n",                       \
          pos->v.scene, pos->v.mbr, pos->v.pri);                               \
          ignore_##mbr = 1;                                                    \
        } /********************************************************************/
        ACTIVATE_HIGHER_CHECK_INT(tsk_init    )
        ACTIVATE_HIGHER_CHECK_INT(tsk_small   )
        ACTIVATE_HIGHER_CHECK_INT(tsk_go_big  )
        ACTIVATE_HIGHER_CHECK_INT(tsk_to_lit  )
        ACTIVATE_HIGHER_CHECK_INT(tsk_nb_nice )
        ACTIVATE_HIGHER_CHECK_INT(tsk_boost   )
        ACTIVATE_HIGHER_CHECK_PTR(lit_tgtl    )
        ACTIVATE_HIGHER_CHECK_PTR(big_tgtl    )
        ACTIVATE_HIGHER_CHECK_PTR(lit_ahspd   )
        ACTIVATE_HIGHER_CHECK_PTR(big_ahspd   )
        ACTIVATE_HIGHER_CHECK_INT(lit_smpt    )
        ACTIVATE_HIGHER_CHECK_INT(big_smpt    )
        ACTIVATE_HIGHER_CHECK_INT(lit_hspf    )
        ACTIVATE_HIGHER_CHECK_INT(big_hspf    )
        ACTIVATE_HIGHER_CHECK_INT(lit_ghspl   )
        ACTIVATE_HIGHER_CHECK_INT(big_ghspl   )
        ACTIVATE_HIGHER_CHECK_INT(lit_slkt    )
        ACTIVATE_HIGHER_CHECK_INT(big_slkt    )
        ACTIVATE_HIGHER_CHECK_INT(lit_hyst    )
        ACTIVATE_HIGHER_CHECK_INT(big_hyst    )
        ACTIVATE_HIGHER_CHECK_INT(ioisbsy     )
        ACTIVATE_HIGHER_CHECK_INT(lit_mlload  )
        ACTIVATE_HIGHER_CHECK_INT(lit_mlfreq  )
        ACTIVATE_HIGHER_CHECK_INT(lit_mlnrrun )
        ACTIVATE_HIGHER_CHECK_INT(tsk_idle    )
      }
      /* move to active list */
      PRINT_SET("activate scene %s %d->%d\n", scene->v.scene, scene->active, 1);
      scene->active = 1;
      if(!ignore_move) {
        PRINT_VV("  move act\n");
        list_move(&scene->list_act, set_to);
      }
      /* governor configuration changes if necessary */
      if(!ignore_pri) {
        PRINT_SET("choose.pri=%d\n", scene->v.pri);
        transaction_param.pri = scene->v.pri;
      }
#define ACTIVATE_PARAMETERS_CHANGE_INT(mbr)                                    \
      if(!ignore_##mbr) {                                                      \
        PRINT_SET("choose." #mbr "(pri)=%d(%d)\n",                             \
          scene->v.mbr, scene->v.pri);                                         \
        transaction_param.mbr = scene->v.mbr;                                  \
        transaction_param.mbr##_pri = scene->v.pri;                            \
      } /**********************************************************************/
#define ACTIVATE_PARAMETERS_CHANGE_PTR(mbr)                                    \
      if(!ignore_##mbr) {                                                      \
        PRINT_SET("choose." #mbr "(pri)=%p(%d)\n",                             \
          scene->v.mbr, scene->v.pri);                                         \
        transaction_param.mbr = scene->v.mbr;                                  \
        transaction_param.mbr##_pri = scene->v.pri;                            \
      } /**********************************************************************/
      ACTIVATE_PARAMETERS_CHANGE_INT(tsk_init    )
      ACTIVATE_PARAMETERS_CHANGE_INT(tsk_small   )
      ACTIVATE_PARAMETERS_CHANGE_INT(tsk_go_big  )
      ACTIVATE_PARAMETERS_CHANGE_INT(tsk_to_lit  )
      ACTIVATE_PARAMETERS_CHANGE_INT(tsk_nb_nice )
      ACTIVATE_PARAMETERS_CHANGE_INT(tsk_boost   )
      ACTIVATE_PARAMETERS_CHANGE_PTR(lit_tgtl    )
      ACTIVATE_PARAMETERS_CHANGE_PTR(big_tgtl    )
      ACTIVATE_PARAMETERS_CHANGE_PTR(lit_ahspd   )
      ACTIVATE_PARAMETERS_CHANGE_PTR(big_ahspd   )
      ACTIVATE_PARAMETERS_CHANGE_INT(lit_smpt    )
      ACTIVATE_PARAMETERS_CHANGE_INT(big_smpt    )
      ACTIVATE_PARAMETERS_CHANGE_INT(lit_hspf    )
      ACTIVATE_PARAMETERS_CHANGE_INT(big_hspf    )
      ACTIVATE_PARAMETERS_CHANGE_INT(lit_ghspl   )
      ACTIVATE_PARAMETERS_CHANGE_INT(big_ghspl   )
      ACTIVATE_PARAMETERS_CHANGE_INT(lit_slkt    )
      ACTIVATE_PARAMETERS_CHANGE_INT(big_slkt    )
      ACTIVATE_PARAMETERS_CHANGE_INT(lit_hyst    )
      ACTIVATE_PARAMETERS_CHANGE_INT(big_hyst    )
      ACTIVATE_PARAMETERS_CHANGE_INT(ioisbsy     )
      ACTIVATE_PARAMETERS_CHANGE_INT(lit_mlload  )
      ACTIVATE_PARAMETERS_CHANGE_INT(lit_mlfreq  )
      ACTIVATE_PARAMETERS_CHANGE_INT(lit_mlnrrun )
      ACTIVATE_PARAMETERS_CHANGE_INT(tsk_idle    )
      /* wait for activate */
      scene->recalc = 1;
    }
  } else {
    PRINT_DIS("not ready.\n");
    skip = 1;
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(!skip) {
    /* other configuration request to the QoS */
#define ACTIVATE_QOS_REQUEST(mbr,lbl)                                          \
    if(scene->v.mbr != PCPC_IGNORE_VALUE) {                                    \
      if(!pm_qos_request_active(&scene->pm_qos_##mbr)) {                       \
        PRINT_VV("pm_qos_add_request " #lbl ",%d\n", scene->v.mbr);            \
        pm_qos_add_request(&scene->pm_qos_##mbr, PM_QOS_##lbl, scene->v.mbr);  \
      }                                                                        \
    } /************************************************************************/
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
#define ACTIVATE_QOS_REQUEST_DO_OTHER_TIMEOUT(mbr,lbl,tout)                        \
    if(scene->v.mbr != PCPC_IGNORE_VALUE) {                                        \
      if(!pm_qos_request_active(&scene->pm_qos_##mbr)) {                           \
        PRINT_VV("pm_qos_add_request " #lbl ",%d %d\n", scene->v.mbr, tout);       \
        pm_qos_add_request(&scene->pm_qos_##mbr, PM_QOS_##lbl, scene->v.mbr);      \
        pm_qos_update_request_timeout(&scene->pm_qos_##mbr, scene->v.mbr, tout);   \
        transaction_param.mbr = scene->v.mbr;                                      \
      }                                                                            \
    } /****************************************************************************/
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
    ACTIVATE_QOS_REQUEST(lit_maxf    , LIT_MAXF    )
    ACTIVATE_QOS_REQUEST(big_maxf    , BIG_MAXF    )
    ACTIVATE_QOS_REQUEST(lit_minf    , LIT_MINF    )
    ACTIVATE_QOS_REQUEST(big_minf    , BIG_MINF    )
    ACTIVATE_QOS_REQUEST(lit_upcore  , LIT_UPCORE  )
    ACTIVATE_QOS_REQUEST(big_upcore  , BIG_UPCORE  )
    ACTIVATE_QOS_REQUEST(bus_min     , BUS_MIN     )
    ACTIVATE_QOS_REQUEST(gpu_max     , GPU_MAX     )
    ACTIVATE_QOS_REQUEST(gpu_min     , GPU_MIN     )
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
    ACTIVATE_QOS_REQUEST_DO_OTHER_TIMEOUT(cpu_dma_latency, CPU_DMA_LATENCY, PCPC_VALUE_CSTATE_TIMEOUT)
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
    /* effective time */
    if(scene->v.duration != PCPC_IGNORE_VALUE) {
      if (delayed_work_pending(&scene->work_timer)) {
        PRINT_EXEC("cancel timer\n");
        cancel_delayed_work_sync(&scene->work_timer);
      }
      PRINT_EXEC("timer set %ums\n", (unsigned int)scene->v.duration);
      queue_delayed_work(pcpcwq, &scene->work_timer, msecs_to_jiffies((unsigned int)scene->v.duration));
    }
  }
  PRINT_VV("%s ret\n", __FUNCTION__);
}

/*!
 @brief __remove_scene_effect

 remove scene's timer and QoS request

 @param [in]   scene       pointer of the scene data to remove

 @retval       none
*/
static void __remove_scene_effect(pcpc_scene_type *scene)
{
    /* cancel timer */
    if(scene->v.duration != PCPC_IGNORE_VALUE) {
      if (delayed_work_pending(&scene->work_timer)) {
        PRINT_EXEC("cancel timer scene %s\n", scene->v.scene);
        cancel_delayed_work_sync(&scene->work_timer);
      }
    }
    /* other configuration request to the QoS */
#define INACTIVATE_QOS_REQUEST(mbr,lbl)                                        \
    if(scene->v.mbr != PCPC_IGNORE_VALUE) {                                    \
      if(pm_qos_request_active(&scene->pm_qos_##mbr)) {                        \
        PRINT_VV("pm_qos_remove_request " #lbl "\n");                          \
        pm_qos_remove_request(&scene->pm_qos_##mbr);                           \
      }                                                                        \
    } /************************************************************************/
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
#define INACTIVATE_QOS_REQUEST_DO_OTHER(mbr,lbl)                               \
    if(scene->v.mbr != PCPC_IGNORE_VALUE) {                                    \
      transaction_param.mbr = PCPC_IGNORE_VALUE;                               \
      if(pm_qos_request_active(&scene->pm_qos_##mbr)) {                        \
        PRINT_VV("pm_qos_remove_request " #lbl "\n");                          \
        pm_qos_remove_request(&scene->pm_qos_##mbr);                           \
      }                                                                        \
    } /************************************************************************/
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
    INACTIVATE_QOS_REQUEST(lit_maxf    , LIT_MAXF    )
    INACTIVATE_QOS_REQUEST(big_maxf    , BIG_MAXF    )
    INACTIVATE_QOS_REQUEST(lit_minf    , LIT_MINF    )
    INACTIVATE_QOS_REQUEST(big_minf    , BIG_MINF    )
    INACTIVATE_QOS_REQUEST(lit_upcore  , LIT_UPCORE  )
    INACTIVATE_QOS_REQUEST(big_upcore  , BIG_UPCORE  )
    INACTIVATE_QOS_REQUEST(bus_min     , BUS_MIN     )
    INACTIVATE_QOS_REQUEST(gpu_max     , GPU_MAX     )
    INACTIVATE_QOS_REQUEST(gpu_min     , GPU_MIN     )
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
    INACTIVATE_QOS_REQUEST_DO_OTHER(cpu_dma_latency, CPU_DMA_LAT)
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
}

/*!
 @brief __inactivate_scene

 inactivate the scene

 @param [in]   scene       pointer of the scene data to inactivate
 @param [in]   requested   1:from unset request, 0:other

 @retval       none
*/
static void __inactivate_scene(pcpc_scene_type *scene, int requested)
{
  unsigned long flags;
  pcpc_scene_type *pos;
  pcpc_scene_type *restore_pri         = NULL;
  pcpc_scene_type *restore_tsk_init    = NULL;
  pcpc_scene_type *restore_tsk_small   = NULL;
  pcpc_scene_type *restore_tsk_go_big  = NULL;
  pcpc_scene_type *restore_tsk_to_lit  = NULL;
  pcpc_scene_type *restore_tsk_nb_nice = NULL;
  pcpc_scene_type *restore_tsk_boost   = NULL;
  pcpc_scene_type *restore_lit_tgtl    = NULL;
  pcpc_scene_type *restore_big_tgtl    = NULL;
  pcpc_scene_type *restore_lit_ahspd   = NULL;
  pcpc_scene_type *restore_big_ahspd   = NULL;
  pcpc_scene_type *restore_lit_smpt    = NULL;
  pcpc_scene_type *restore_big_smpt    = NULL;
  pcpc_scene_type *restore_lit_hspf    = NULL;
  pcpc_scene_type *restore_big_hspf    = NULL;
  pcpc_scene_type *restore_lit_ghspl   = NULL;
  pcpc_scene_type *restore_big_ghspl   = NULL;
  pcpc_scene_type *restore_lit_slkt    = NULL;
  pcpc_scene_type *restore_big_slkt    = NULL;
  pcpc_scene_type *restore_lit_hyst    = NULL;
  pcpc_scene_type *restore_big_hyst    = NULL;
  pcpc_scene_type *restore_ioisbsy     = NULL;
  pcpc_scene_type *restore_lit_mlload  = NULL;
  pcpc_scene_type *restore_lit_mlfreq  = NULL;
  pcpc_scene_type *restore_lit_mlnrrun = NULL;
  pcpc_scene_type *restore_tsk_idle    = NULL;
  int skip = 0;

  PRINT_VERB("%s %s,%d\n", __FUNCTION__, scene->v.scene, requested);

  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {

    /* check restore governor */
    if(scene->active) {
      if( scene->v.pri         != PCPC_IGNORE_VALUE ||
          scene->v.tsk_init    != PCPC_IGNORE_VALUE ||
          scene->v.tsk_small   != PCPC_IGNORE_VALUE ||
          scene->v.tsk_go_big  != PCPC_IGNORE_VALUE ||
          scene->v.tsk_to_lit  != PCPC_IGNORE_VALUE ||
          scene->v.tsk_nb_nice != PCPC_IGNORE_VALUE ||
          scene->v.tsk_boost   != PCPC_IGNORE_VALUE ||
          scene->v.lit_tgtl    != NULL              ||
          scene->v.big_tgtl    != NULL              ||
          scene->v.lit_ahspd   != NULL              ||
          scene->v.big_ahspd   != NULL              ||
          scene->v.lit_smpt    != PCPC_IGNORE_VALUE ||
          scene->v.big_smpt    != PCPC_IGNORE_VALUE ||
          scene->v.lit_hspf    != PCPC_IGNORE_VALUE ||
          scene->v.big_hspf    != PCPC_IGNORE_VALUE ||
          scene->v.lit_ghspl   != PCPC_IGNORE_VALUE ||
          scene->v.big_ghspl   != PCPC_IGNORE_VALUE ||
          scene->v.lit_slkt    != PCPC_IGNORE_VALUE ||
          scene->v.big_slkt    != PCPC_IGNORE_VALUE ||
          scene->v.lit_hyst    != PCPC_IGNORE_VALUE ||
          scene->v.big_hyst    != PCPC_IGNORE_VALUE ||
          scene->v.ioisbsy     != PCPC_IGNORE_VALUE ||
          scene->v.lit_mlload  != PCPC_IGNORE_VALUE ||
          scene->v.lit_mlfreq  != PCPC_IGNORE_VALUE ||
          scene->v.lit_mlnrrun != PCPC_IGNORE_VALUE ||
          scene->v.tsk_idle    != PCPC_IGNORE_VALUE ) {
        PRINT_VV("check restore parameters\n");
        list_for_each_entry(pos, &active_list, list_act) {
          if(pos != scene) {
            if(!restore_pri && pos->v.pri != PCPC_IGNORE_VALUE) {
              PRINT_VV("  next %s.pri=%d\n", pos->v.scene, pos->v.pri);
              restore_pri = pos;
            }
#define INACTIVATE_RESTORE_CHECK_INT(mbr)                                      \
            if(!restore_##mbr && pos->v.mbr != PCPC_IGNORE_VALUE) {            \
              PRINT_VV("  next %s." #mbr "(pri)=%d(%d)\n",                     \
                pos->v.scene, pos->v.mbr, pos->v.pri);                         \
              restore_##mbr = pos;                                             \
            } /****************************************************************/
#define INACTIVATE_RESTORE_CHECK_PTR(mbr)                                      \
            if(!restore_##mbr && pos->v.mbr != NULL) {                         \
              PRINT_VV("  next %s." #mbr "(pri)=%p(%d)\n",                     \
                pos->v.scene, pos->v.mbr, pos->v.pri);                         \
              restore_##mbr = pos;                                             \
            } /****************************************************************/
            INACTIVATE_RESTORE_CHECK_INT(tsk_init    )
            INACTIVATE_RESTORE_CHECK_INT(tsk_small   )
            INACTIVATE_RESTORE_CHECK_INT(tsk_go_big  )
            INACTIVATE_RESTORE_CHECK_INT(tsk_to_lit  )
            INACTIVATE_RESTORE_CHECK_INT(tsk_nb_nice )
            INACTIVATE_RESTORE_CHECK_INT(tsk_boost   )
            INACTIVATE_RESTORE_CHECK_PTR(lit_tgtl    )
            INACTIVATE_RESTORE_CHECK_PTR(big_tgtl    )
            INACTIVATE_RESTORE_CHECK_PTR(lit_ahspd   )
            INACTIVATE_RESTORE_CHECK_PTR(big_ahspd   )
            INACTIVATE_RESTORE_CHECK_INT(lit_smpt    )
            INACTIVATE_RESTORE_CHECK_INT(big_smpt    )
            INACTIVATE_RESTORE_CHECK_INT(lit_hspf    )
            INACTIVATE_RESTORE_CHECK_INT(big_hspf    )
            INACTIVATE_RESTORE_CHECK_INT(lit_ghspl   )
            INACTIVATE_RESTORE_CHECK_INT(big_ghspl   )
            INACTIVATE_RESTORE_CHECK_INT(lit_slkt    )
            INACTIVATE_RESTORE_CHECK_INT(big_slkt    )
            INACTIVATE_RESTORE_CHECK_INT(lit_hyst    )
            INACTIVATE_RESTORE_CHECK_INT(big_hyst    )
            INACTIVATE_RESTORE_CHECK_INT(ioisbsy     )
            INACTIVATE_RESTORE_CHECK_INT(lit_mlload  )
            INACTIVATE_RESTORE_CHECK_INT(lit_mlfreq  )
            INACTIVATE_RESTORE_CHECK_INT(lit_mlnrrun )
            INACTIVATE_RESTORE_CHECK_INT(tsk_idle    )
          }
        }
      }
    }
    /* function */
    if(scene->v.func == PCPC_VALUE_FUNC_INPUT_ID ||
       scene->v.func == PCPC_VALUE_FUNC_POWERKEY_ID ||
       scene->v.func == PCPC_VALUE_FUNC_HOMEKEY_ID) {
      if(requested) {
        PRINT_SET("function disable scene %s %d->%d\n", scene->v.scene, scene->funcable, 0);
        if(!list_empty(&scene->list_func)) {
          PRINT_VV("  del func\n");
          list_del_init(&scene->list_func);
          func_input_list_count--;
        }
        scene->funcable = 0;
      }
    }
    /* move to inactive list */
    PRINT_SET("inactivate scene %s %d->%d\n", scene->v.scene, scene->active, 0);
    scene->active = 0;
    PRINT_VV("  move inact\n");
    list_move(&scene->list_act, &inactive_list);

    /* governor configuration changes if necessary */
    if(restore_pri) {
      PRINT_SET("choose.pri=%d\n", restore_pri->v.pri);
      transaction_param.pri = restore_pri->v.pri;
    }
#define INACTIVATE_PARAMETERS_RESTORE_INT(mbr)                                 \
    if(restore_##mbr) {                                                        \
      PRINT_SET("choose." #mbr "(pri)=%d(%d)\n",                               \
        restore_##mbr->v.mbr, restore_##mbr->v.pri);                           \
      transaction_param.mbr = restore_##mbr->v.mbr;                            \
      transaction_param.mbr##_pri = restore_##mbr->v.pri;                      \
    } /************************************************************************/
#define INACTIVATE_PARAMETERS_RESTORE_PTR(mbr)                                 \
    if(restore_##mbr) {                                                        \
      PRINT_SET("choose." #mbr "(pri)=%p(%d)\n",                               \
        restore_##mbr->v.mbr, restore_##mbr->v.pri);                           \
      transaction_param.mbr = restore_##mbr->v.mbr;                            \
      transaction_param.mbr##_pri = restore_##mbr->v.pri;                      \
    } /************************************************************************/
    INACTIVATE_PARAMETERS_RESTORE_INT(tsk_init    )
    INACTIVATE_PARAMETERS_RESTORE_INT(tsk_small   )
    INACTIVATE_PARAMETERS_RESTORE_INT(tsk_go_big  )
    INACTIVATE_PARAMETERS_RESTORE_INT(tsk_to_lit  )
    INACTIVATE_PARAMETERS_RESTORE_INT(tsk_nb_nice )
    INACTIVATE_PARAMETERS_RESTORE_INT(tsk_boost   )
    INACTIVATE_PARAMETERS_RESTORE_PTR(lit_tgtl    )
    INACTIVATE_PARAMETERS_RESTORE_PTR(big_tgtl    )
    INACTIVATE_PARAMETERS_RESTORE_PTR(lit_ahspd   )
    INACTIVATE_PARAMETERS_RESTORE_PTR(big_ahspd   )
    INACTIVATE_PARAMETERS_RESTORE_INT(lit_smpt    )
    INACTIVATE_PARAMETERS_RESTORE_INT(big_smpt    )
    INACTIVATE_PARAMETERS_RESTORE_INT(lit_hspf    )
    INACTIVATE_PARAMETERS_RESTORE_INT(big_hspf    )
    INACTIVATE_PARAMETERS_RESTORE_INT(lit_ghspl   )
    INACTIVATE_PARAMETERS_RESTORE_INT(big_ghspl   )
    INACTIVATE_PARAMETERS_RESTORE_INT(lit_slkt    )
    INACTIVATE_PARAMETERS_RESTORE_INT(big_slkt    )
    INACTIVATE_PARAMETERS_RESTORE_INT(lit_hyst    )
    INACTIVATE_PARAMETERS_RESTORE_INT(big_hyst    )
    INACTIVATE_PARAMETERS_RESTORE_INT(ioisbsy     )
    INACTIVATE_PARAMETERS_RESTORE_INT(lit_mlload  )
    INACTIVATE_PARAMETERS_RESTORE_INT(lit_mlfreq  )
    INACTIVATE_PARAMETERS_RESTORE_INT(lit_mlnrrun )
    INACTIVATE_PARAMETERS_RESTORE_INT(tsk_idle    )

    scene->recalc = 0;

  } else {
    PRINT_DIS("not ready.\n");
    skip = 1;
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(!skip) {
    __remove_scene_effect(scene);
  }
  PRINT_VV("%s ret\n", __FUNCTION__);
}

/*!
 @brief __timer_work_fn

 effective timer expired

 @param [in]   work     pointer of work_struct

 @retval       none
*/
static void __timer_work_fn(struct work_struct *work)
{
  pcpc_scene_type *scene = container_of(to_delayed_work(work), pcpc_scene_type, work_timer);

  PRINT_EXEC("timer expired for %s (duration=%d)\n", scene->v.scene, scene->v.duration);
  __begin_transaction();
  __inactivate_scene(scene, 0);
  __end_transaction();
  PRINT_VV("%s ret\n", __FUNCTION__);
}

/*!
 @brief __input_event_work_fn

 handle input event for input function

 @param [in]   work   pointer of work_struct structure

 @retval       none
*/
static void __input_event_work_fn(struct work_struct *work)
{
  unsigned long flags;
  pcpc_scene_type *pos;
  pcpc_scene_type *func_array[PCPC_FUNC_ACTIVE_SAME_MAX];
  int func_count = 0;
  int i;
  int match_func;

  PRINT_VERB("%s\n", __FUNCTION__);
  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {
    if (work == &homekey_event_work) {
      match_func = PCPC_VALUE_FUNC_HOMEKEY_ID;
    } else if (work == &powerkey_event_work) {
      match_func = PCPC_VALUE_FUNC_POWERKEY_ID;
    } else {
      match_func = PCPC_VALUE_FUNC_INPUT_ID;
    }
    func_count = func_input_list_count;
    if(func_count > 0) {
      PRINT_VV("make func array\n");
      i = 0;
      /* after enabled */
      list_for_each_entry_reverse(pos, &func_input_list, list_func) {
        if(pos->v.func == match_func) {
          if(i >= PCPC_FUNC_ACTIVE_SAME_MAX) {
            PRINT_DIS("too many active func\n");
            break;
          }
          func_array[i++] = pos;
        }
      }
      func_count = i;
    }
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(initialized) {
    if(func_count > 0) {
      __begin_transaction();
      PRINT_VV("activate func list\n");
      for(i = 0; i < func_count; i++) {
#ifdef DEBUG_INTERNAL
        if(debug_loose_input_wq) {
          PRINT_VERB("debug_loose_input_wq wait %d ms begin\n", debug_loose_input_wq);
          msleep(debug_loose_input_wq);
          PRINT_VERB("debug_loose_input_wq wait end\n");
        }
#endif
        if(func_array[i]) {
          __activate_scene(func_array[i], 0);
        }
      }
      __end_transaction();
    } else {
      PRINT_VV("array is empty\n");
    }
  }
  PRINT_VV("%s ret\n", __FUNCTION__);
}


/* ==========================================================================
 *  MODULE PARAM FUNCTIONS
 * ========================================================================== */

/* ----------------------------------------------------------------
 * config
 */

/*!
 @brief __store_config_work_fn

 store work for 'config'

 @param [in]   work   pointer of work_struct structure

 @retval       none
*/
static void __store_config_work_fn(struct work_struct *work)
{
  pcpc_sync_work_type *ppsw = container_of(work, pcpc_sync_work_type, work);
  const char *val = ppsw->val;

  unsigned long flags;
  int ret = 0;
  int skip = 0;
  pcpc_scene_type *scene, *pos;
  const char *endptr, *nxtptr, *tagptr, *valptr;
  int len, taglen, vallen;
  char tmpbuf[PCPC_VALUE_FUNC_MAXLEN];

  PRINT_EXEC("%s %s\n", __FUNCTION__, val);

  PRINT_VV("config malloc\n");
  scene = kzalloc(sizeof(pcpc_scene_type), GFP_KERNEL);
  if(!scene) {
    PRINT_ERR("config malloc error\n");
    ret = -ENOMEM;
    goto done;
  }
  memcpy(&scene->v, &_initial_ignore_param, sizeof(pcpc_scene_param_type));
  INIT_LIST_HEAD(&scene->list_reg);
  INIT_LIST_HEAD(&scene->list_act);
  INIT_LIST_HEAD(&scene->list_func);
  INIT_DELAYED_WORK(&scene->work_timer, __timer_work_fn);

  len = strlen(val);
  if(len > 0 && val[len - 1] == '\n') {
    PRINT_VV("  trim bottom lf\n");
    len--;
  }
  endptr = &val[len];
  for(tagptr = val; tagptr < endptr; ) {

    nxtptr = strnchr(tagptr, endptr - tagptr, ',');
    if(!nxtptr) {
      nxtptr = endptr;
    }
    valptr = strnchr(tagptr, nxtptr - tagptr, '=');

    if(valptr) {
      taglen = (int)(valptr - tagptr);
      valptr++;
      vallen = (int)(nxtptr - valptr);

#define STORE_CONFIG_PARAM_NUM_INT(mbr,lbl)                                    \
      if(taglen == PCPC_PARAM_##lbl##_LEN &&                                   \
          !strncmp(PCPC_PARAM_##lbl, tagptr, taglen)) {                        \
        __set_num_int(&scene->v.mbr, valptr, vallen);                          \
        PRINT_VV("  ." #mbr "=%d\n", scene->v.mbr);                            \
      } /**********************************************************************/
#define STORE_CONFIG_PARAM_SIGNED_INT(mbr,lbl)                                 \
      if(taglen == PCPC_PARAM_##lbl##_LEN &&                                   \
          !strncmp(PCPC_PARAM_##lbl, tagptr, taglen)) {                        \
        __set_signed_int(&scene->v.mbr, valptr, vallen);                       \
        PRINT_VV("  ." #mbr "=%d\n", scene->v.mbr);                            \
      } /**********************************************************************/
#define STORE_CONFIG_PARAM_SEP_FREQ(mbr,lbl)                                   \
      if(taglen == PCPC_PARAM_##lbl##_LEN &&                                   \
          !strncmp(PCPC_PARAM_##lbl, tagptr, taglen)) {                        \
        __set_sep_freq_ptr(&scene->v.mbr, valptr, vallen);                     \
        PRINT_VV("  ." #mbr "=%p\n", scene->v.mbr);                            \
      } /**********************************************************************/

      if(taglen == PCPC_PARAM_SCENE_LEN &&
          !strncmp(PCPC_PARAM_SCENE, tagptr, taglen)) {
        __set_string(scene->v.scene, PCPC_SCENE_NAME_LEN, valptr, vallen);
        PRINT_VV("  .scene=%s\n", scene->v.scene);
      }
      else if(taglen == PCPC_PARAM_FUNC_LEN &&
          !strncmp(PCPC_PARAM_FUNC, tagptr, taglen)) {
        __set_string(tmpbuf, PCPC_VALUE_FUNC_MAXLEN, valptr, vallen);
        if(tmpbuf[0] == '\0') {
          scene->v.func = PCPC_IGNORE_VALUE;
        } else if(!strcmp(tmpbuf, PCPC_VALUE_FUNC_STR_INPUT)) {
          scene->v.func = PCPC_VALUE_FUNC_INPUT_ID;
        } else if(!strcmp(tmpbuf, PCPC_VALUE_FUNC_STR_HOMEKEY)) {
          scene->v.func = PCPC_VALUE_FUNC_HOMEKEY_ID;
        } else if(!strcmp(tmpbuf, PCPC_VALUE_FUNC_STR_POWERKEY)) {
          scene->v.func = PCPC_VALUE_FUNC_POWERKEY_ID;
        } else {
          scene->v.func = PCPC_VALUE_FUNC_UNKNOWN_ID;
        }
        PRINT_VV("  .func=%d\n", scene->v.func);
      }
      else STORE_CONFIG_PARAM_NUM_INT   (duration    ,DURATION    )
      else STORE_CONFIG_PARAM_NUM_INT   (pri         ,PRI         )
      else STORE_CONFIG_PARAM_NUM_INT   (tsk_init    ,TSK_INIT    )
      else STORE_CONFIG_PARAM_NUM_INT   (tsk_small   ,TSK_SMALL   )
      else STORE_CONFIG_PARAM_NUM_INT   (tsk_go_big  ,TSK_GO_BIG  )
      else STORE_CONFIG_PARAM_NUM_INT   (tsk_to_lit  ,TSK_TO_LIT  )
      else STORE_CONFIG_PARAM_SIGNED_INT(tsk_nb_nice ,TSK_NB_NICE )
      else STORE_CONFIG_PARAM_NUM_INT   (tsk_boost   ,TSK_BOOST   )
      else STORE_CONFIG_PARAM_SEP_FREQ  (lit_tgtl    ,LIT_TGTL    )
      else STORE_CONFIG_PARAM_SEP_FREQ  (big_tgtl    ,BIG_TGTL    )
      else STORE_CONFIG_PARAM_SEP_FREQ  (lit_ahspd   ,LIT_AHSPD   )
      else STORE_CONFIG_PARAM_SEP_FREQ  (big_ahspd   ,BIG_AHSPD   )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_smpt    ,LIT_SMPT    )
      else STORE_CONFIG_PARAM_NUM_INT   (big_smpt    ,BIG_SMPT    )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_hspf    ,LIT_HSPF    )
      else STORE_CONFIG_PARAM_NUM_INT   (big_hspf    ,BIG_HSPF    )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_ghspl   ,LIT_GHSPL   )
      else STORE_CONFIG_PARAM_NUM_INT   (big_ghspl   ,BIG_GHSPL   )
      else STORE_CONFIG_PARAM_SIGNED_INT(lit_slkt    ,LIT_SLKT    )
      else STORE_CONFIG_PARAM_SIGNED_INT(big_slkt    ,BIG_SLKT    )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_hyst    ,LIT_HYST    )
      else STORE_CONFIG_PARAM_NUM_INT   (big_hyst    ,BIG_HYST    )
      else STORE_CONFIG_PARAM_NUM_INT   (ioisbsy     ,IOISBSY     )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_insf    ,LIT_INSF    )
      else STORE_CONFIG_PARAM_NUM_INT   (big_insf    ,BIG_INSF    )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_maxf    ,LIT_MAXF    )
      else STORE_CONFIG_PARAM_NUM_INT   (big_maxf    ,BIG_MAXF    )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_minf    ,LIT_MINF    )
      else STORE_CONFIG_PARAM_NUM_INT   (big_minf    ,BIG_MINF    )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_upcore  ,LIT_UPCORE  )
      else STORE_CONFIG_PARAM_NUM_INT   (big_upcore  ,BIG_UPCORE  )
      else STORE_CONFIG_PARAM_NUM_INT   (bus_min     ,BUS_MIN     )
      else STORE_CONFIG_PARAM_NUM_INT   (gpu_max     ,GPU_MAX     )
      else STORE_CONFIG_PARAM_NUM_INT   (gpu_min     ,GPU_MIN     )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_mlload  ,LIT_MLLOAD  )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_mlfreq  ,LIT_MLFREQ  )
      else STORE_CONFIG_PARAM_NUM_INT   (lit_mlnrrun ,LIT_MLNRRUN )
      else STORE_CONFIG_PARAM_NUM_INT   (tsk_idle    ,TSK_IDLE    )
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
      else STORE_CONFIG_PARAM_NUM_INT   (cpu_dma_latency, CPU_DMA_LAT)
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */
      else {
        PRINT_DIS("ignore unsupported param (at %d): %s\n", (int)(tagptr - val), val);
      }
    } else {
      PRINT_DIS("ignore unsupported param (at %d): %s\n", (int)(tagptr - val), val);
    }

    nxtptr++;
    tagptr = nxtptr;
  }
  
  /* valid check */
  if(scene->v.scene[0] == '\0') {
    PRINT_ERR("config requires 'scene' parameter: %s\n", val);
    ret = -EINVAL;
  }
  if((scene->v.tsk_init    != PCPC_IGNORE_VALUE ||
      scene->v.tsk_small   != PCPC_IGNORE_VALUE ||
      scene->v.tsk_go_big  != PCPC_IGNORE_VALUE ||
      scene->v.tsk_to_lit  != PCPC_IGNORE_VALUE ||
      scene->v.tsk_nb_nice != PCPC_IGNORE_VALUE ||
      scene->v.tsk_boost   != PCPC_IGNORE_VALUE ||
      scene->v.lit_tgtl    != NULL ||
      scene->v.big_tgtl    != NULL ||
      scene->v.lit_ahspd   != NULL ||
      scene->v.big_ahspd   != NULL ||
      scene->v.lit_smpt    != PCPC_IGNORE_VALUE ||
      scene->v.big_smpt    != PCPC_IGNORE_VALUE ||
      scene->v.lit_hspf    != PCPC_IGNORE_VALUE ||
      scene->v.big_hspf    != PCPC_IGNORE_VALUE ||
      scene->v.lit_ghspl   != PCPC_IGNORE_VALUE ||
      scene->v.big_ghspl   != PCPC_IGNORE_VALUE ||
      scene->v.lit_slkt    != PCPC_IGNORE_VALUE ||
      scene->v.big_slkt    != PCPC_IGNORE_VALUE ||
      scene->v.lit_hyst    != PCPC_IGNORE_VALUE ||
      scene->v.big_hyst    != PCPC_IGNORE_VALUE ||
      scene->v.ioisbsy     != PCPC_IGNORE_VALUE ||
      scene->v.lit_mlload  != PCPC_IGNORE_VALUE ||
      scene->v.lit_mlfreq  != PCPC_IGNORE_VALUE ||
      scene->v.lit_mlnrrun != PCPC_IGNORE_VALUE ||
      scene->v.tsk_idle    != PCPC_IGNORE_VALUE ) &&
      scene->v.pri         == PCPC_IGNORE_VALUE ) {
    PRINT_ERR("config prioritization parameter requires 'pri' parameter: %s\n", val);
    ret = -EINVAL;
  }
  if(scene->v.func == PCPC_VALUE_FUNC_UNKNOWN_ID) {
    PRINT_ERR("config not provide 'func': %s\n", val);
    ret = -EINVAL;
  }
#define STORE_CONFIG_CHECK_ERR_PTR(mbr)                     \
  if(IS_ERR(scene->v.mbr)) {                                \
    PRINT_ERR("config '" #mbr "' format error: %s\n", val); \
    scene->v.mbr = NULL;                                    \
    ret = -EINVAL;                                          \
  } /*******************************************************/
  STORE_CONFIG_CHECK_ERR_PTR(lit_tgtl    )
  STORE_CONFIG_CHECK_ERR_PTR(big_tgtl    )
  STORE_CONFIG_CHECK_ERR_PTR(lit_ahspd   )
  STORE_CONFIG_CHECK_ERR_PTR(big_ahspd   )
  
  spin_lock_irqsave(&pcpc_lock, flags);
  if(!initialized) {
    PRINT_ERR("not ready.\n");
    ret = -EPERM;
  }
  if(!ret) {
    PRINT_VV("search registered\n");
    list_for_each_entry(pos, &register_list, list_reg) {
      PRINT_VV("..%s\n", pos->v.scene);
      if(!strncmp(scene->v.scene, pos->v.scene, PCPC_SCENE_NAME_LEN)) {
        if(!pos->active && !pos->funcable) {
          PRINT_SET("replace config: scene %s\n", scene->v.scene);
#define STORE_CONFIG_FREE_PREV_PTR(mbr)          \
          if(pos->v.mbr) {                       \
            PRINT_VV("  free prev." #mbr "\n");  \
            kfree(pos->v.mbr);                   \
            pos->v.mbr = NULL;                   \
          } /************************************/
          STORE_CONFIG_FREE_PREV_PTR(lit_tgtl    )
          STORE_CONFIG_FREE_PREV_PTR(big_tgtl    )
          STORE_CONFIG_FREE_PREV_PTR(lit_ahspd   )
          STORE_CONFIG_FREE_PREV_PTR(big_ahspd   )
          memcpy(&pos->v, &scene->v, sizeof(pcpc_scene_param_type));
#define STORE_CONFIG_REMOVE_PTR(mbr)          \
          if(scene->v.mbr) {                  \
            PRINT_VV("  renew " #mbr "\n");   \
            scene->v.mbr = NULL;              \
          } /*********************************/
          STORE_CONFIG_REMOVE_PTR(lit_tgtl    )
          STORE_CONFIG_REMOVE_PTR(big_tgtl    )
          STORE_CONFIG_REMOVE_PTR(lit_ahspd   )
          STORE_CONFIG_REMOVE_PTR(big_ahspd   )
          skip = 1;
        } else {
          PRINT_ERR("can't replace config: scene %s\n", pos->v.scene);
          ret = -EINVAL;
        }
        break;
      }
    }
  }
  if(!ret && !skip) {
    PRINT_VV("  add reg\n");
    list_add_tail(&scene->list_reg, &register_list);
    PRINT_VV("  move inact\n");
    list_add(&scene->list_act, &inactive_list);
    PRINT_SET("add config: scene %s\n", scene->v.scene);
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(ret || skip) {
    PRINT_VV("config free memory\n");
#define STORE_CONFIG_FREE_ERR_PTR(mbr)    \
    if(scene->v.mbr) {                    \
      PRINT_VV("  free " #mbr "\n");      \
      kfree(scene->v.mbr);                \
    }  /**********************************/
    STORE_CONFIG_FREE_ERR_PTR(lit_tgtl    )
    STORE_CONFIG_FREE_ERR_PTR(big_tgtl    )
    STORE_CONFIG_FREE_ERR_PTR(lit_ahspd   )
    STORE_CONFIG_FREE_ERR_PTR(big_ahspd   )
    kfree(scene);
    scene = NULL;
  }

  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);

done:
  ppsw->rc = ret;
  complete(&ppsw->done);
  return;
}

/*!
 @brief __show_config_one

 sub processing of show function for 'config'
 call with pcpc_lock locked

 @param [in]   buffer    pointer of the buffer for read
 @param [in]   size      buffer size
 @param [in]   scene     pointer of the scene data

 @retval       length of the buffer can be read
*/
static int __show_config_one(char *buffer, int size, pcpc_scene_type *scene)
{
  char *ptr = buffer;
  char *bottom = &buffer[size];

  ptr += __scnprintf_string(  ptr, bottom - ptr, scene->v.scene       , "\t");
  ptr += __scnprintf_string(  ptr, bottom - ptr, __to_param_func_string(scene->v.func), "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.duration    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.pri         , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.tsk_init    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.tsk_small   , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.tsk_go_big  , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.tsk_to_lit  , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.tsk_nb_nice , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.tsk_boost   , "\t");
  ptr += __scnprintf_sep_freq(ptr, bottom - ptr, scene->v.lit_tgtl    , "\t");
  ptr += __scnprintf_sep_freq(ptr, bottom - ptr, scene->v.big_tgtl    , "\t");
  ptr += __scnprintf_sep_freq(ptr, bottom - ptr, scene->v.lit_ahspd   , "\t");
  ptr += __scnprintf_sep_freq(ptr, bottom - ptr, scene->v.big_ahspd   , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_smpt    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_smpt    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_hspf    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_hspf    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_ghspl   , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_ghspl   , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_slkt    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_slkt    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_hyst    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_hyst    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.ioisbsy     , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_mlload  , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_mlfreq  , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_mlnrrun , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.tsk_idle    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_insf    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_insf    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_maxf    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_maxf    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_minf    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_minf    , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.lit_upcore  , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.big_upcore  , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.bus_min     , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.gpu_max     , "\t");
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression mod start */
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.gpu_min     , "\t");
  ptr += __scnprintf_num_int( ptr, bottom - ptr, scene->v.cpu_dma_latency, "\n");
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression mod end   */

  return (int)(ptr - buffer);
}

#define __SHOW_CONFIG_PRINT_HEADER \
  "scene\t" \
  "func\t" \
  "duration\t" \
  "pri\t" \
  "tsk_init\t" \
  "tsk_small\t" \
  "tsk_go_big\t" \
  "tsk_to_lit\t" \
  "tsk_nb_nice\t" \
  "tsk_boost\t" \
  "lit_tgtl\t" \
  "big_tgtl\t" \
  "lit_ahspd\t" \
  "big_ahspd\t" \
  "lit_smpt\t" \
  "big_smpt\t" \
  "lit_hspf\t" \
  "big_hspf\t" \
  "lit_ghspl\t" \
  "big_ghspl\t" \
  "lit_slkt\t" \
  "big_slkt\t" \
  "lit_hyst\t" \
  "big_hyst\t" \
  "ioisbsy\t" \
  "lit_mlload\t" \
  "lit_mlfreq\t" \
  "lit_mlnrrun\t" \
  "tsk_idle\t" \
  "lit_insf\t" \
  "big_insf\t" \
  "lit_maxf\t" \
  "big_maxf\t" \
  "lit_minf\t" \
  "big_minf\t" \
  "lit_upcore\t" \
  "big_upcore\t" \
  "bus_min\t" \
  "gpu_max\t" \
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression mod start */ \
  "gpu_min\t" \
  "cpu_dma_latency\n" \
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression mod end   */

/*!
 @brief __show_config

 show function for 'config'

 @param [in]   buffer    pointer of the buffer for read
 @param [in]   size      buffer size

 @retval       length of the buffer can be read
*/
static int __show_config(char *buffer, int size)
{
  unsigned long flags;
  pcpc_scene_type *scene;
  char *ptr = buffer;
  char *bottom = &buffer[size];

  PRINT_VERB("%s\n", __FUNCTION__);

  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {
    ptr += scnprintf(ptr, bottom - ptr, __SHOW_CONFIG_PRINT_HEADER);

    list_for_each_entry(scene, &register_list, list_reg) {
      ptr += __show_config_one(ptr, bottom - ptr, scene);
    }
  } else {
    ptr += scnprintf(ptr, bottom - ptr,
          "not ready.\n");
    PRINT_DIS("not ready.\n");
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  PRINT_VV("%s ret:%d\n", __FUNCTION__, (int)(ptr - buffer));
  return (int)(ptr - buffer);
}

/*!
 @brief store_config

 store function for 'config'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of string of written
 @param [in]   count     length of buffer

 @retval  >=0: success
 @retval   <0: failed
*/
static ssize_t store_config(struct module_attribute *mattr, struct module_kobject *mk, const char *buffer, size_t count)
{
  int ret;
  /* sync work */
  PRINT_VV("%s queue_work\n", __FUNCTION__);
  ret = __exec_work_sync(__store_config_work_fn, buffer);
  if(!ret) {
    ret = count;
  }
  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);
  return ret;
}

/*!
 @brief show_config

 show function for 'config'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of the buffer for read

 @retval       length of the buffer can be read
*/
static ssize_t show_config(struct module_attribute *mattr, struct module_kobject *mk, char *buffer)
{
  return __show_config(buffer, PAGE_SIZE - 1);
}
static struct module_attribute pcpc_param_config = __ATTR(config, 0660, show_config, store_config);


/* ----------------------------------------------------------------
 * set
 */

/*!
 @brief __store_set_work_fn

 store work for 'set'

 @param [in]   work   pointer of work_struct structure

 @retval       none
*/
static void __store_set_work_fn(struct work_struct *work)
{
  pcpc_sync_work_type *ppsw = container_of(work, pcpc_sync_work_type, work);
  const char *val = ppsw->val;

  int set_count = 0;
  pcpc_scene_type *scene;
  const char *endptr, *nxtptr, *namptr;
  int len, namlen;

  PRINT_EXEC("%s %s\n", __FUNCTION__, val);

  __begin_transaction();
  len = strlen(val);
  if(len > 0 && val[len - 1] == '\n') {
    PRINT_VV("  trim bottom lf\n");
    len--;
  }
  endptr = &val[len];
  for(namptr = val; namptr < endptr; ) {

    nxtptr = strnchr(namptr, endptr - namptr, ',');
    if(!nxtptr) {
      nxtptr = endptr;
    }
    namlen = (int)(nxtptr - namptr);

    scene = __find_registered_scene_by_name(namptr, namlen);
    if(scene) {
      __activate_scene(scene, 1);
      set_count++;
    } else {
      PRINT_ERR("can't set unknown scene (at %d): %s\n", (int)(namptr - val), val);
    }

    nxtptr++;
    namptr = nxtptr;
  }
  __end_transaction();

  if(!set_count) {
    PRINT_ERR("can't set any scenes: %s\n", val);
    ppsw->rc = -EINVAL;
  } else {
    ppsw->rc = 0;
  }

  PRINT_VV("%s ret:%d\n", __FUNCTION__, ppsw->rc);
  complete(&ppsw->done);
  return;
}

/*!
 @brief __show_set

 show function for 'set'

 @param [in]   buffer    pointer of the buffer for read
 @param [in]   size      buffer size

 @retval       length of the buffer can be read
*/
static int __show_set(char *buffer, int size)
{
  unsigned long flags;
  pcpc_scene_type *scene;
  pcpc_scene_param_type *scene_param;
  char *ptr = buffer;
  char *bottom = &buffer[size];
  int get_count = 0, standby_count = 0;

  PRINT_VERB("%s\n", __FUNCTION__);

  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {
    ptr += scnprintf(ptr, bottom - ptr, "set scene: ");
    /* show active with adding standby */
    list_for_each_entry(scene, &func_input_list, list_func) {
      if(!scene->active) {
        standby_count++;
      }
    }
    list_for_each_entry(scene, &active_list, list_act) {
      ptr += __scnprintf_string(ptr, bottom - ptr, scene->v.scene,
                                (list_is_last(&scene->list_act, &active_list) ? (standby_count ? "," : "\n") : ","));
      get_count++;
    }
    if(standby_count) {
      list_for_each_entry(scene, &func_input_list, list_func) {
        if(!scene->active) {
          standby_count--;
          ptr += __scnprintf_string(ptr, bottom - ptr, scene->v.scene, (standby_count ? "," : "\n"));
        }
        get_count++;
      }
    }
    if(!get_count) {
      ptr += scnprintf(ptr, bottom - ptr, "\n");
    }
    ptr += scnprintf(ptr, bottom - ptr,
      "pri\t"
      "tsk_init(pri)\t"
      "tsk_small(pri)\t"
      "tsk_go_big(pri)\t"
      "tsk_to_lit(pri)\t"
      "tsk_nb_nice(pri)\t"
      "tsk_boost(pri)\t"
      "lit_tgtl(pri)\t"
      "big_tgtl(pri)\t"
      "lit_ahspd(pri)\t"
      "big_ahspd(pri)\t"
      "lit_smpt(pri)\t"
      "big_smpt(pri)\t"
      "lit_hspf(pri)\t"
      "big_hspf(pri)\t"
      "lit_ghspl(pri)\t"
      "big_ghspl(pri)\t"
      "lit_slkt(pri)\t"
      "big_slkt(pri)\t"
      "lit_hyst(pri)\t"
      "big_hyst(pri)\t"
      "ioisbsy(pri)\t"
      "lit_mlload(pri)\t"
      "lit_mlfreq(pri)\t"
      "lit_mlnrrun(pri)\t"
      "tsk_idle(pri)\t"
      "lit_maxf\t"
      "big_maxf\t"
      "lit_minf\t"
      "big_minf\t"
      "lit_upcore\t"
      "big_upcore\t"
      "bus_min\t"
      "gpu_max\t"
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression mod start */
      "gpu_min\t"
      "cpu_dma_latency[QoS]\n");
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression mod end   */

    scene_param = &top_most_scene_param;
    {
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->pri         , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_init    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_init_pri    , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_small   , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_small_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_go_big  , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_go_big_pri  , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_to_lit  , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_to_lit_pri  , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_nb_nice , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_nb_nice_pri , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_boost , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_boost_pri , ")\t");
      ptr += __scnprintf_sep_freq(ptr, bottom - ptr, scene_param->lit_tgtl    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_tgtl_pri    , ")\t");
      ptr += __scnprintf_sep_freq(ptr, bottom - ptr, scene_param->big_tgtl    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_tgtl_pri    , ")\t");
      ptr += __scnprintf_sep_freq(ptr, bottom - ptr, scene_param->lit_ahspd   , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_ahspd_pri   , ")\t");
      ptr += __scnprintf_sep_freq(ptr, bottom - ptr, scene_param->big_ahspd   , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_ahspd_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_smpt    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_smpt_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_smpt    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_smpt_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_hspf    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_hspf_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_hspf    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_hspf_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_ghspl   , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_ghspl_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_ghspl   , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_ghspl_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_slkt    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_slkt_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_slkt    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_slkt_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_hyst    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_hyst_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_hyst    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_hyst_pri   , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->ioisbsy     , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->ioisbsy_pri    , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_mlload  , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_mlload_pri , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_mlfreq  , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_mlfreq_pri , ")\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_mlnrrun , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_mlnrrun_pri, ")\n");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_idle    , "(");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->tsk_idle_pri   , ")\n");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_maxf    , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_maxf    , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_minf    , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_minf    , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->lit_upcore  , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->big_upcore  , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->bus_min     , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->gpu_max     , "\t");
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression mod start */
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->gpu_min     , "\t");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, scene_param->cpu_dma_latency    , "[");
      ptr += __scnprintf_num_int( ptr, bottom - ptr, pm_qos_request(PM_QOS_CPU_DMA_LATENCY), "]\n");
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression mod end   */
    }
  } else {
    ptr += scnprintf(ptr, bottom - ptr,
          "not ready.\n");
    PRINT_DIS("not ready.\n");
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  PRINT_VV("%s ret:%d\n", __FUNCTION__, (int)(ptr - buffer));
  return (int)(ptr - buffer);
}

/*!
 @brief store_set

 store function for 'set'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of string of written
 @param [in]   count     length of buffer

 @retval  >=0: success
 @retval   <0: failed
*/
static ssize_t store_set(struct module_attribute *mattr, struct module_kobject *mk, const char *buffer, size_t count)
{
  int ret;
  /* sync work */
  PRINT_VV("%s queue_work\n", __FUNCTION__);
  ret = __exec_work_sync(__store_set_work_fn, buffer);
  if(!ret) {
    ret = count;
  }
  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);
  return ret;
}

/*!
 @brief show_set

 show function for 'set'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of the buffer for read

 @retval       length of the buffer can be read
*/
static ssize_t show_set(struct module_attribute *mattr, struct module_kobject *mk, char *buffer)
{
  return __show_set(buffer, PAGE_SIZE - 1);
}

static struct module_attribute pcpc_param_set = __ATTR(set, 0660, show_set, store_set);


/* ----------------------------------------------------------------
 * unset
 */

/*!
 @brief __store_unset_work_fn

 store work for 'unset'

 @param [in]   work   pointer of work_struct structure

 @retval       none
*/
static void __store_unset_work_fn(struct work_struct *work)
{
  pcpc_sync_work_type *ppsw = container_of(work, pcpc_sync_work_type, work);
  const char *val = ppsw->val;

  int unset_count = 0;
  pcpc_scene_type *scene;
  const char *endptr, *nxtptr, *namptr;
  int len, namlen;

  PRINT_EXEC("%s %s\n", __FUNCTION__, val);

  __begin_transaction();
  len = strlen(val);
  if(len > 0 && val[len - 1] == '\n') {
    PRINT_VV("  trim bottom lf\n");
    len--;
  }
  endptr = &val[len];
  for(namptr = val; namptr < endptr; ) {

    nxtptr = strnchr(namptr, endptr - namptr, ',');
    if(!nxtptr) {
      nxtptr = endptr;
    }
    namlen = (int)(nxtptr - namptr);

    scene = __find_registered_scene_by_name(namptr, namlen);
    if(scene) {
      __inactivate_scene(scene, 1);
      unset_count++;
    } else {
      PRINT_ERR("can't unset unknown scene (at %d): %s\n", (int)(namptr - val), val);
    }

    nxtptr++;
    namptr = nxtptr;
  }
  __end_transaction();

  if(!unset_count) {
    PRINT_ERR("can't unset any scenes: %s\n", val);
    ppsw->rc = -EINVAL;
  } else {
    ppsw->rc = 0;
  }

  PRINT_VV("%s ret:%d\n", __FUNCTION__, ppsw->rc);
  complete(&ppsw->done);
  return;
}

/*!
 @brief __show_unset

 show function for 'unset'

 @param [in]   buffer    pointer of the buffer for read
 @param [in]   size      buffer size

 @retval       length of the buffer can be read
*/
static int __show_unset(char *buffer, int size)
{
  unsigned long flags;
  pcpc_scene_type *scene;
  char *ptr = buffer;
  char *bottom = &buffer[size];
  int get_count = 0;

  PRINT_VERB("%s\n", __FUNCTION__);

  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {
    ptr += scnprintf(ptr, bottom - ptr, "unset scene: ");
    /* show inactive excluding standby */
    list_for_each_entry(scene, &inactive_list, list_act) {
      if(!scene->funcable) {
        get_count++;
      }
    }
    if(get_count) {
      list_for_each_entry(scene, &inactive_list, list_act) {
        if(!scene->funcable) {
          get_count--;
          ptr += __scnprintf_string(  ptr, bottom - ptr, scene->v.scene, (get_count ? "," : "\n"));
        }
      }
    }
    else if(!get_count) {
      ptr += scnprintf(ptr, bottom - ptr, "\n");
    }
  } else {
    ptr += scnprintf(ptr, bottom - ptr,
          "not ready.\n");
    PRINT_DIS("not ready.\n");
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  PRINT_VV("%s ret:%d\n", __FUNCTION__, (int)(ptr - buffer));
  return (int)(ptr - buffer);
}

/*!
 @brief store_unset

 store function for 'unset'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of string of written
 @param [in]   count     length of buffer

 @retval  >=0: success
 @retval   <0: failed
*/
static ssize_t store_unset(struct module_attribute *mattr, struct module_kobject *mk, const char *buffer, size_t count)
{
  int ret;
  /* sync work */
  PRINT_VV("%s queue_work\n", __FUNCTION__);
  ret = __exec_work_sync(__store_unset_work_fn, buffer);
  if(!ret) {
    ret = count;
  }
  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);
  return ret;
}

/*!
 @brief show_unset

 show function for 'unset'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of the buffer for read

 @retval       length of the buffer can be read
*/
static ssize_t show_unset(struct module_attribute *mattr, struct module_kobject *mk, char *buffer)
{
  return __show_unset(buffer, PAGE_SIZE - 1);
}

static struct module_attribute pcpc_param_unset = __ATTR(unset, 0660, show_unset, store_unset);


/* ----------------------------------------------------------------
 * boost_cancel
 */

/*!
 @brief __cancel_boost_input

 cancel input event for input function

 @param [in]   match_func       func type

 @retval       none
*/
static void __cancel_boost_input(int match_func)
{
  unsigned long flags;
  pcpc_scene_type *pos;
  pcpc_scene_type *func_array[PCPC_FUNC_ACTIVE_SAME_MAX];
  int func_count = 0;
  int i;

  PRINT_VERB("%s\n", __FUNCTION__);
  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {
    func_count = func_input_list_count;
    if(func_count > 0) {
      PRINT_VV("make func array\n");
      i = 0;
      /* after enabled */
      list_for_each_entry_reverse(pos, &func_input_list, list_func) {
        if(pos->v.func == match_func) {
          if(i >= PCPC_FUNC_ACTIVE_SAME_MAX) {
            PRINT_DIS("too many active func\n");
            break;
          }
          func_array[i++] = pos;
        }
      }
      func_count = i;
    }
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(initialized) {
    if(func_count > 0) {
      __begin_transaction();
      PRINT_VV("cancel func list\n");
      for(i = 0; i < func_count; i++) {
        if(func_array[i]) {
          __inactivate_scene(func_array[i], 0);
        }
      }
      __end_transaction();
    } else {
      PRINT_VV("array is empty\n");
    }
  }
  PRINT_VV("%s ret\n", __FUNCTION__);
}

/*!
 @brief __store_boost_cancel_work_fn

 store work for 'boost_cancel'

 @param [in]   work   pointer of work_struct structure

 @retval       none
*/

static void __store_boost_cancel_work_fn(struct work_struct *work)
{
  pcpc_sync_work_type *ppsw = container_of(work, pcpc_sync_work_type, work);
  const char *val = ppsw->val;

  int cancel_count = 0;
  pcpc_scene_type *scene;
  const char *endptr, *nxtptr, *namptr;
  int len, namlen;

  PRINT_EXEC("%s %s\n", __FUNCTION__, val);

  __begin_transaction();
  len = strlen(val);
  if(len > 0 && val[len - 1] == '\n') {
    PRINT_VV("  trim bottom lf\n");
    len--;
  }
  endptr = &val[len];
  for(namptr = val; namptr < endptr; ) {

    nxtptr = strnchr(namptr, endptr - namptr, ',');
    if(!nxtptr) {
      nxtptr = endptr;
    }
    namlen = (int)(nxtptr - namptr);

    if ( namlen == sizeof(PCPC_VALUE_FUNC_STR_INPUT)-1 &&
         !strncmp(namptr,PCPC_VALUE_FUNC_STR_INPUT,sizeof(PCPC_VALUE_FUNC_STR_INPUT)-1)
       ) {
      __cancel_boost_input(PCPC_VALUE_FUNC_INPUT_ID);
      cancel_count++;
      goto next;
    }
    if ( namlen == sizeof(PCPC_VALUE_FUNC_STR_HOMEKEY)-1 &&
         !strncmp(namptr,PCPC_VALUE_FUNC_STR_HOMEKEY,sizeof(PCPC_VALUE_FUNC_STR_HOMEKEY)-1)
       ) {
      __cancel_boost_input(PCPC_VALUE_FUNC_HOMEKEY_ID);
      cancel_count++;
      goto next;
    }
    if ( namlen == sizeof(PCPC_VALUE_FUNC_STR_POWERKEY)-1 &&
         !strncmp(namptr,PCPC_VALUE_FUNC_STR_POWERKEY,sizeof(PCPC_VALUE_FUNC_STR_POWERKEY)-1)
       ) {
      __cancel_boost_input(PCPC_VALUE_FUNC_POWERKEY_ID);
      cancel_count++;
      goto next;
    }
    scene = __find_registered_scene_by_name(namptr, namlen);
    if(scene) {
      __inactivate_scene(scene, 1);
      cancel_count++;
    } else {
      PRINT_ERR("can't unset unknown scene (at %d): %s\n", (int)(namptr - val), val);
    }

next:
    nxtptr++;
    namptr = nxtptr;
  }
  __end_transaction();

  if(!cancel_count) {
    PRINT_ERR("can't unset any scenes: %s\n", val);
    ppsw->rc = -EINVAL;
  } else {
    ppsw->rc = 0;
  }

  PRINT_VV("%s ret:%d\n", __FUNCTION__, ppsw->rc);
  complete(&ppsw->done);
  return;
}

/*!
 @brief store_boost_cancel

 store function for 'boost_cancel'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of string of written
 @param [in]   count     length of buffer

 @retval  >=0: success
 @retval   <0: failed
*/
static ssize_t store_boost_cancel(struct module_attribute *mattr, struct module_kobject *mk, const char *buffer, size_t count)
{
  int ret;
  /* sync work */
  PRINT_VV("%s queue_work\n", __FUNCTION__);
  ret = __exec_work_sync(__store_boost_cancel_work_fn, buffer);
  if(!ret) {
    ret = count;
  }
  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);
  return ret;
}

static struct module_attribute pcpc_param_boost_cancel = __ATTR(boost_cancel, 0220, NULL, store_boost_cancel);


/* ----------------------------------------------------------------
 * off
 */

/*!
 @brief __store_off_work_fn

 store work for 'off'

 @param [in]   work   pointer of work_struct structure

 @retval       none
*/
static void __store_off_work_fn(struct work_struct *work)
{
  pcpc_sync_work_type *ppsw = container_of(work, pcpc_sync_work_type, work);
  const char *val = ppsw->val;

  unsigned long flags;
  int old_value;
  long new_value;
  int ret = 0;

  PRINT_VERB("%s %s\n", __FUNCTION__, val);

  spin_lock_irqsave(&pcpc_lock, flags);
  old_value = cut_off;
  new_value = cut_off;
  ret = kstrtol(val, 0, &new_value);
  if (ret < 0 || ((int)new_value != new_value)) {
    if(ret >= 0) {
      ret = -EINVAL;
    }
    PRINT_DIS("invalid off parameter: %s\n", val);
  } else {
    cut_off = new_value;
    if(old_value && !new_value) {
      PRINT_VV("  update_all %d->%d\n", update_all, 1);
      update_all = 1;
    }
  } 
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(!ret) {
    if(!old_value && new_value) {
      PRINT_EXEC("off\n");
      __begin_transaction();
    }
    if(old_value && !new_value) {
      PRINT_EXEC("on\n");
      __end_transaction();
    }
  }

  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);
  ppsw->rc = ret;
  complete(&ppsw->done);
  return;
}


/*!
 @brief store_off

 store function for 'off'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of string of written
 @param [in]   count     length of buffer

 @retval  >=0: success
 @retval   <0: failed
*/
static ssize_t store_off(struct module_attribute *mattr, struct module_kobject *mk, const char *buffer, size_t count)
{
  int ret;
  /* sync work */
  PRINT_VV("%s queue_work\n", __FUNCTION__);
  ret = __exec_work_sync(__store_off_work_fn, buffer);
  if(!ret) {
    ret = count;
  }
  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);
  return ret;
}

/*!
 @brief show_off

 show function for 'off'

 @param [in]   dev       pointer of the module_attribute structure
 @param [in]   attr      pointer of module_kobject structure
 @param [in]   buffer    pointer of the buffer for read

 @retval       length of the buffer can be read
*/
static ssize_t show_off(struct module_attribute *mattr, struct module_kobject *mk, char *buffer)
{
  return sprintf(buffer, "%i", cut_off);
}

static struct module_attribute pcpc_param_off = __ATTR(off, 0660, show_off, store_off);


/* ==========================================================================
 *  PROCFS
 * ========================================================================== */

/*!
 @brief show_config_seq_start

 start function for seq_file

 @param [in]   m     pointer of the seq_file struct
 @param [in]   pos   pointer of the offset

 @retval       start pointer of the output data
*/
static void *show_config_seq_start(struct seq_file *m, loff_t *pos)
{
  unsigned long flags;
  void *p;
  int i;

  PRINT_VV("++ show_config_seq_start *pos=%d\n", (int)*pos);

  spin_lock_irqsave(&pcpc_lock, flags);
  if(!*pos)
    p = &register_list;
  else if(!initialized)
    p = NULL;
  else {
    p = list_first_entry_or_null(&register_list, pcpc_scene_type, list_reg);
    for(i = 1; i < *pos && p != NULL; i++) {
      if(!list_is_last(&((pcpc_scene_type *)p)->list_reg, &register_list))
        p = list_next_entry((pcpc_scene_type *)p, list_reg);
      else
        p = NULL;
    }
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);

  PRINT_VV("-- show_config_seq_start (%p)\n", p);
  return p;
}

/*!
 @brief show_config_seq_next

 next function for seq_file

 @param [in]   m     pointer of the seq_file struct
 @param [in]   p     pointer of the output data
 @param [out]  pos   pointer of the offset

 @retval       next pointer of the output data
*/
static void *show_config_seq_next(struct seq_file *m, void *p, loff_t *pos)
{
  unsigned long flags;

  PRINT_VV("++ show_config_seq_next p=%p *pos=%d\n", p, (int)*pos);

  (*pos)++;
  spin_lock_irqsave(&pcpc_lock, flags);
  if(!initialized)
    p = NULL;
  else if(p == &register_list)
    p = list_first_entry_or_null(&register_list, pcpc_scene_type, list_reg);
  else if(!list_is_last(&((pcpc_scene_type *)p)->list_reg, &register_list))
    p = list_next_entry((pcpc_scene_type *)p, list_reg);
  else
    p = NULL;
  spin_unlock_irqrestore(&pcpc_lock, flags);

  PRINT_VV("-- show_config_seq_next (%p) *pos=%d\n", p, (int)*pos);
  return p;
}

/*!
 @brief show_config_seq_show

 output function for seq_file

 @param [in]   m     pointer of the seq_file struct
 @param [in]   p     pointer of the output data

 @retval      0: success
 @retval    <>0: failed
*/
static int show_config_seq_show(struct seq_file *m, void *p)
{
  unsigned long flags;
  int err = 0;
  char buffer[PCPC_LOCAL_PRINT_BUF_LIMIT];

  buffer[0] = 0;
  spin_lock_irqsave(&pcpc_lock, flags);
  /* In the current specification, exclusive of appropriate access to
   * the pointer is unnecessary because the memory is not released
   * except for at the module_exit.
   */
  if(!initialized)
    err = 1;
  else if(p != &register_list)
    __show_config_one(buffer, PCPC_LOCAL_PRINT_BUF_LIMIT, p);
  spin_unlock_irqrestore(&pcpc_lock, flags);

  if(err) {
    seq_puts(m, "not ready.\n");
    PRINT_DIS("not ready.\n");
  } else if(p == &register_list) {
    seq_puts(m, __SHOW_CONFIG_PRINT_HEADER);
  } else {
    seq_puts(m, buffer);
  }

  PRINT_VV(">> show_config_seq_show\n");
  return 0;
}

/*!
 @brief show_config_seq_stop

 stop function for seq_file

 @param [in]   m     pointer of the seq_file struct
 @param [in]   p     pointer of the output data

 @retval       none
*/
static void show_config_seq_stop(struct seq_file *m,void *p)
{
  /* do nothing */
}

static const struct seq_operations show_config_seq_ops = {
  .start  = show_config_seq_start,
  .next   = show_config_seq_next,
  .show   = show_config_seq_show,
  .stop   = show_config_seq_stop,
};

/*!
 @brief pcpc_config_open

 open function for /proc/pcpc/config

 @param [in]   inode   pointer of the inode struct
 @param [in]   file    pointer of the file struct

 @retval      0: success
 @retval    <>0: failed
*/
static int pcpc_config_open(struct inode *inode, struct file *file)
{
  PRINT_VERB("> open buffer\n");
  return seq_open(file, &show_config_seq_ops);
}

static struct file_operations pcpc_config_op = {
    .open = pcpc_config_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = seq_release,
};


/* ==========================================================================
 *  CONTROL FUNCTIONS
 * ========================================================================== */

/*!
 @brief pcpc_input_event

 input_handler event function

 @param [in]   handle
 @param [in]   type
 @param [in]   code
 @param [in]   value

 @retval       none
*/
static void pcpc_input_event(struct input_handle *handle, unsigned int type, 
                             unsigned int code, int value)
{
  unsigned long flags;

  PRINT_VV("%s\n", __FUNCTION__);
  spin_lock_irqsave(&pcpc_lock, flags);
  if(initialized) {
    if (type == EV_KEY && value == 1) {
      switch (code) {
        /* HomeKey */
        case KEY_HOME:
          if(!work_pending(&homekey_event_work)) {
            PRINT_VV("  queue_work homekey\n");
            queue_work(pcpcwq, &homekey_event_work);
          }
          break;
        /* PowerKey */
        case KEY_POWER:
#ifdef KEY_FP
        /* FPKey */
        case KEY_FP:
#endif
        case KEY_WAKEUP:
          if(!work_pending(&powerkey_event_work)) {
            PRINT_VV("  queue_work powerkey\n");
            queue_work(pcpcwq, &powerkey_event_work);
          }
          break;
      }
    }
    if(!work_pending(&input_event_work)) {
      PRINT_VV("  queue_work input\n");
      queue_work(pcpcwq, &input_event_work);
    }
  }
  spin_unlock_irqrestore(&pcpc_lock, flags);
  PRINT_VV("%s ret\n", __FUNCTION__);
}

/*!
 @brief pcpc_input_connect

 input_handler connect function

 @param [in]   handler
 @param [in]   dev
 @param [in]   id

 @retval    0: success
 @retval  <>0: failed
*/
static int pcpc_input_connect(struct input_handler *handler,
                              struct input_dev *dev, const struct input_device_id *id)
{
  struct input_handle *handle;
  int error;

  PRINT_VERB("%s\n", __FUNCTION__);
  /* exclude device */
  if(dev != NULL) {
    if(dev->name != NULL) {
      if(strcmp(dev->name, "fj-walkmotion") == 0) {
        return 0;
      }
    }
  }

  handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
  if (!handle)
    return -ENOMEM;

  handle->dev = dev;
  handle->handler = handler;
  handle->name = "pcpc";

  error = input_register_handle(handle);
  if (error)
    goto err2;

  error = input_open_device(handle);
  if (error)
    goto err1;

  PRINT_VV("%s ret:%d\n", __FUNCTION__, 0);
  return 0;
err1:
  input_unregister_handle(handle);
err2:
  kfree(handle);
  PRINT_VV("%s ret:%d\n", __FUNCTION__, error);
  return error;
}

/*!
 @brief pcpc_input_disconnect

 input_handler disconnect function

 @param [in]   handle

 @retval       none
*/
static void pcpc_input_disconnect(struct input_handle *handle)
{
  PRINT_VERB("%s\n", __FUNCTION__);
  input_close_device(handle);
  input_unregister_handle(handle);
  kfree(handle);
}

static const struct input_device_id pcpc_ids[] = {
  { .driver_info = 1 },
  { },
};

static struct input_handler pcpc_input_handler = {
  .event          = pcpc_input_event,
  .connect        = pcpc_input_connect,
  .disconnect     = pcpc_input_disconnect,
  .name           = "pcpc",
  .id_table       = pcpc_ids,
};

/*!
 @brief __pm_qos_notify_common

 common callback function for PM QoS notify

 @param [in]   nb        pointer of notifier_block structure
 @param [in]   event     notified value that has changed
 @param [in]   data      notified data (always NULL)

 @retval  NOTIFY_OK: success
*/
static int __pm_qos_notify_common(struct notifier_block *nb, unsigned long val, void *data)
{
  pcpc_qos_notifier_control_type *controler = container_of(nb, pcpc_qos_notifier_control_type, nb);

  if((int)val != controler->default_value) {
    PRINT_EXEC("qos-notify-%s %d\n", controler->param_name, (int)val);
    __update_param_qos((int) val, controler->param_id, 1);
  } else {
    PRINT_EXEC("qos-notify-%s ignore default value(%d)\n", controler->param_name, (int)val);
  }
  PRINT_VV("qos-notify-%s ret:%d\n", controler->param_name, NOTIFY_OK);
  return NOTIFY_OK;
}

/* notifiers */
#define PM_QOS_NOTIFY_COMMON_DEF(mbr,lbl)          \
pcpc_qos_notifier_control_type mbr##_notifier = {  \
  .nb = {                                          \
    .notifier_call = __pm_qos_notify_common,       \
  },                                               \
  .param_name = #mbr,                              \
  .param_id = PCPC_PARAM_##lbl##_ID,               \
  .default_value = PM_QOS_##lbl##_DEFAULT_VALUE,   \
}; /***********************************************/
PM_QOS_NOTIFY_COMMON_DEF(lit_maxf    , LIT_MAXF    )
PM_QOS_NOTIFY_COMMON_DEF(big_maxf    , BIG_MAXF    )
PM_QOS_NOTIFY_COMMON_DEF(lit_minf    , LIT_MINF    )
PM_QOS_NOTIFY_COMMON_DEF(big_minf    , BIG_MINF    )
PM_QOS_NOTIFY_COMMON_DEF(lit_upcore  , LIT_UPCORE  )
PM_QOS_NOTIFY_COMMON_DEF(big_upcore  , BIG_UPCORE  )
PM_QOS_NOTIFY_COMMON_DEF(bus_min     , BUS_MIN     )
PM_QOS_NOTIFY_COMMON_DEF(gpu_max     , GPU_MAX     )
PM_QOS_NOTIFY_COMMON_DEF(gpu_min     , GPU_MIN     )
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
/* PM_QOS_NOTIFY_COMMON_DEF(cpu_dma_latency, CPU_DMA_LAT) */ /* do other */
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */


/* ==========================================================================
 *  GLOBAL FUNCTIONS
 * ========================================================================== */

/*!
 @brief pcpc_set_scene

 interface for set the scene

 @param [in]   scene     pointer of the scene string

 @retval    0: success
 @retval  <>0: failed
*/
int pcpc_set_scene(const char *scene)
{
  int ret;
  /* sync work */
  PRINT_VV("%s queue_work\n", __FUNCTION__);
  ret = __exec_work_sync(__store_set_work_fn, scene);
  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);
  return ret;
}

/*!
 @brief pcpc_unset_scene

 interface for unset the scene

 @param [in]   scene     pointer of the scene string

 @retval    0: success
 @retval  <>0: failed
*/
int pcpc_unset_scene(const char *scene)
{
  int ret;
  /* sync work */
  PRINT_VV("%s queue_work\n", __FUNCTION__);
  ret = __exec_work_sync(__store_unset_work_fn, scene);
  PRINT_VV("%s ret:%d\n", __FUNCTION__, ret);
  return ret;
}


/* ==========================================================================
 *  MODULE DEFINITION
 * ========================================================================== */

static const struct attribute *pcpc_param_attrs[] = {
&pcpc_param_config.attr,
&pcpc_param_set.attr,
&pcpc_param_unset.attr,
&pcpc_param_boost_cancel.attr,
&pcpc_param_off.attr,
NULL,
};

/*!
 @brief __pcpc_init

 initialize

 @param        none

 @retval        0: success
 @retval       <0: failed
*/
static int __pcpc_init(void)
{
  int rc;

  PRINT_EXEC("%s\n", __FUNCTION__);

  pcpcwq = create_singlethread_workqueue("pcpcwq");
  PRINT_ERR_IF(!pcpcwq, "create_workqueue() failed\n");

  INIT_WORK(&input_event_work, __input_event_work_fn);
  INIT_WORK(&homekey_event_work, __input_event_work_fn);
  INIT_WORK(&powerkey_event_work, __input_event_work_fn);

  module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
  PRINT_ERR_IF(!module_kobj, "kset_find_obj(KBUILD_MODNAME) failed\n");
  if(module_kobj) {
    rc = sysfs_create_files(module_kobj, pcpc_param_attrs);
    PRINT_ERR_IF(rc, "sysfs_create_files(pcpc_param_attrs) failed. %d\n", rc);
  }
  initialized = 1;

#define PCPC_INIT_ADD_NOTIFIER_COMMON(mbr,lbl)                                 \
  rc = pm_qos_add_notifier(PM_QOS_##lbl, &mbr##_notifier.nb);                  \
  PRINT_ERR_IF(rc, "pm_qos_add_notifier(PM_QOS_" #lbl ") failed. %d\n", rc);   \
  /****************************************************************************/

  PCPC_INIT_ADD_NOTIFIER_COMMON(lit_maxf    , LIT_MAXF    )
  PCPC_INIT_ADD_NOTIFIER_COMMON(big_maxf    , BIG_MAXF    )
  PCPC_INIT_ADD_NOTIFIER_COMMON(lit_minf    , LIT_MINF    )
  PCPC_INIT_ADD_NOTIFIER_COMMON(big_minf    , BIG_MINF    )
  PCPC_INIT_ADD_NOTIFIER_COMMON(lit_upcore  , LIT_UPCORE  )
  PCPC_INIT_ADD_NOTIFIER_COMMON(big_upcore  , BIG_UPCORE  )
  PCPC_INIT_ADD_NOTIFIER_COMMON(bus_min     , BUS_MIN     )
  PCPC_INIT_ADD_NOTIFIER_COMMON(gpu_max     , GPU_MAX     )
  PCPC_INIT_ADD_NOTIFIER_COMMON(gpu_min     , GPU_MIN     )
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
/*  PCPC_INIT_ADD_NOTIFIER_COMMON(cpu_dma_latency, CPU_DMA_LATENCY) */ /* do other */
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */

  rc = input_register_handler(&pcpc_input_handler);
  PRINT_ERR_IF(rc, "input_register_handler() failed %d\n", rc);

  proc_pcpc = proc_mkdir("pcpc", NULL);
  if(proc_pcpc) {
    proc_pcpc_config = proc_create("config", 0440, proc_pcpc, &pcpc_config_op);
  }
  PRINT_ERR_IF(!proc_pcpc_config, "proc_create() failed\n");

  PRINT_VV("%s ret:%d\n", __FUNCTION__, 0);
  return 0;
}

/*!
 @brief __pcpc_exit

 terminate

 @param         none

 @retval        none
*/
static void __pcpc_exit(void)
{
  unsigned long flags;
  int rc;
  pcpc_scene_type *scene, *tmp;

  PRINT_EXEC("%s\n", __FUNCTION__);

  input_unregister_handler(&pcpc_input_handler);

  if(proc_pcpc_config) {
    remove_proc_entry("config", proc_pcpc);
  }
  if(proc_pcpc) {
    remove_proc_entry("pcpc", NULL);
  }

#define PCPC_EXIT_REMOVE_NOTIFIER_COMMON(mbr,lbl)                              \
  rc = pm_qos_remove_notifier(PM_QOS_##lbl, &mbr##_notifier.nb);               \
  PRINT_ERR_IF(rc, "pm_qos_remove_notifier(PM_QOS_" #lbl ") failed %d\n", rc); \
  /****************************************************************************/

  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(lit_maxf    , LIT_MAXF    )
  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(big_maxf    , BIG_MAXF    )
  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(lit_minf    , LIT_MINF    )
  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(big_minf    , BIG_MINF    )
  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(lit_upcore  , LIT_UPCORE  )
  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(big_upcore  , BIG_UPCORE  )
  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(bus_min     , BUS_MIN     )
  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(gpu_max     , GPU_MAX     )
  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(gpu_min     , GPU_MIN     )
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add start */
/*  PCPC_EXIT_REMOVE_NOTIFIER_COMMON(cpu_dma_latency, CPU_DMA_LATENCY) */ /* do other */
/* FUJITSU LIMITED: 2015-07-16 POWERSAVE C-State suppression add end   */

  spin_lock_irqsave(&pcpc_lock, flags);
  cut_off = 0;
  block_trans = 0;
  update_all = 0;
  initialized = 0;
  INIT_LIST_HEAD(&active_list);
  INIT_LIST_HEAD(&inactive_list);
  INIT_LIST_HEAD(&func_input_list);
  func_input_list_count = 0;
  memcpy(&top_most_scene_param, &_initial_ignore_param, sizeof(pcpc_scene_param_type));
  memcpy(&transaction_param, &_initial_ignore_param, sizeof(pcpc_scene_param_type));
  spin_unlock_irqrestore(&pcpc_lock, flags);

  /* delete all */
  list_for_each_entry_safe(scene, tmp, &register_list, list_reg) {
    __remove_scene_effect(scene);
    PRINT_VV("  free scene:%s\n", scene->v.scene);
#define PCPC_EXIT_FREE_ERR_PTR(mbr)    \
    if(scene->v.mbr) {                 \
      PRINT_VV("  free " #mbr "\n");   \
      kfree(scene->v.mbr);             \
    } /********************************/
    PCPC_EXIT_FREE_ERR_PTR(lit_tgtl    )
    PCPC_EXIT_FREE_ERR_PTR(big_tgtl    )
    PCPC_EXIT_FREE_ERR_PTR(lit_ahspd   )
    PCPC_EXIT_FREE_ERR_PTR(big_ahspd   )
    kfree(scene);
  }
  INIT_LIST_HEAD(&register_list);

  if(module_kobj) {
    sysfs_remove_files(module_kobj, pcpc_param_attrs);
  }

  msm_thermal_reset_core_limit();

  destroy_workqueue(pcpcwq);

  PRINT_VV("%s ret\n", __FUNCTION__);
}

/*!
 @brief pcpc_init

 driver initialize

 @param        none

 @retval        0: success
 @retval       <0: failed
*/
static int __init pcpc_init(void)
{
  return __pcpc_init();
}

/*!
 @brief pcpc_exit

 driver terminate

 @param         none

 @retval        none
*/
static void __exit pcpc_exit(void)
{
  __pcpc_exit();
}

module_init(pcpc_init);
module_exit(pcpc_exit);

MODULE_DESCRIPTION("Performance Capability Policy Control");
MODULE_LICENSE("GPL v2");
