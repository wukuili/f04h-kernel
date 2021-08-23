/*
 * filog.c
 *
 * Copyright(C) 2013 FUJITSU LIMITED
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

#include <linux/module.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <asm/current.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel_stat.h>
#include <linux/slab.h>

#ifdef CONFIG_ARM_ARCH_TIMER
#include <asm/arch_timer.h>
#endif /* CONFIG_ARM_ARCH_TIMER */


/* ==========================================================================
 *  DEFINITION
 * ========================================================================== */

#if 0
#define DBGPRINT(msg...) printk(KERN_INFO "filog:" msg)
#define IS_DBGPRINT 1
#else
#define DBGPRINT(msg...)
#define IS_DBGPRINT 0
#endif

/* initial buffer size */
#define FILOG_DEFAULT_BUFFER_SIZE   0x180000  /* (1536kb) */

/* record data structure */
#define FILOG_DECLARE_RECORD_HALF   \
  unsigned char  type;  \
  unsigned char  p8;    \
  unsigned short p16;   \
  unsigned int   p32

typedef struct filog_record_half{
  FILOG_DECLARE_RECORD_HALF;
} filog_record_half;

typedef struct filog_record{
  FILOG_DECLARE_RECORD_HALF;
  unsigned long long time;
} filog_record;

#define FILOG_BUFFER_MIN_RECORD     2

/* record type */
#define FILOGTYPE_MARKER_END        0x00  /* represent the end of the record */
#define FILOGTYPE_MARKER_BREAKRING  0x01  /* represent a break in the ring */
#define FILOGTYPE_LOGGING_CHANGE    0x02  /* change log-mask information: p8:msg_type       */
  #define FILOGTYPE_LOGGING_CHANGE_INIT   0x00  /* initialized            p32:log_mask      */
  #define FILOGTYPE_LOGGING_CHANGE_MASK   0x01  /* change log-mask        p32:log_mask      */
  #define FILOGTYPE_LOGGING_CHANGE_RESET  0x02  /* reset                  p32:log_mask      */
  #define FILOGTYPE_LOGGING_CHANGE_SIZE   0x03  /* change buffer size     p32:rec_buffer_kb */
  #define FILOGTYPE_LOGGING_CHANGE_EXPORT 0x04  /* export buffer          */

#define FILOGTYPE_MACHINE_SUSPEND   0x10  /* machine_suspend: p32:state */
#define FILOGTYPE_CPU_ONLINE        0x11  /* cpu_online:      p32:state, p8:cpu_id */
#define FILOGTYPE_CPU_FREQUENCY     0x12  /* cpu_frequency:   p32:state, p8:cpu_id */
#define FILOGTYPE_DISPLAY_SUSPEND   0x13  /* display_suspend: p32:state */
#define FILOGTYPE_RESUME_IRQ        0x14  /* resume_irq:      p32:irq */


/* condition work */
typedef struct filog_condition {
  filog_record *rec_buffer;     /* buffer address */
  unsigned int rec_w;           /* buffer write pointer */
  unsigned int rec_e;           /* buffer size */
  struct timespec cond_time;    /* timestamp */
} filog_condition;

/* condition */
static filog_record *rec_buffer;      /* buffer address */
static unsigned int rec_w;            /* buffer write pointer */
static unsigned int rec_e;            /* buffer size */
static unsigned int rec_initialized;  /* disable recording until initialized */

/* internal */
static DEFINE_SPINLOCK(rec_lock);
static struct proc_dir_entry *proc_filog;
static struct proc_dir_entry *proc_filog_data;

/* module parameters */
#define FILOGMASK_INFORMATION       (1<<0)  /* memo             */
#define FILOGMASK_SUSPEND_STATE     (1<<1)  /* machine_suspend, cpu_online, display_suspend */
#define FILOGMASK_VARIABLE_STATE    (1<<2)  /* cpu_frequency    */
#define FILOGMASK_RESUME_IRQ        (1<<3)  /* resume_irq       */
static unsigned int log_mask = FILOGMASK_INFORMATION | FILOGMASK_SUSPEND_STATE | FILOGMASK_RESUME_IRQ;
static unsigned int rec_buffer_kb = FILOG_DEFAULT_BUFFER_SIZE / 1024;
static unsigned int reset;


/* ==========================================================================
 *  LOCAL FUNCTIONS
 * ========================================================================== */

/*!
 @brief __filog_write_record_i

 record filog (internal use)

 @param [in] data      address of the data record (ignoring "time")

 @retval     write pointer
*/
static filog_record *__filog_write_record_i(const filog_record_half *data)
{
  filog_record *ret = 0;
  unsigned char term_type;
#ifdef CONFIG_ARM_ARCH_TIMER
  unsigned long long rec_time = arch_counter_get_cntpct();
#else
  unsigned long long rec_time = sched_clock();
#endif /* CONFIG_ARM_ARCH_TIMER */

  term_type = rec_buffer[rec_w].type;
  memcpy(&rec_buffer[rec_w], data, sizeof(filog_record_half));
  memcpy(&rec_buffer[rec_w].time, &rec_time, sizeof(rec_time));
  ret = &rec_buffer[rec_w];

  rec_w++;
  if(rec_w >= rec_e) {
    rec_w = 0;
    term_type = FILOGTYPE_MARKER_BREAKRING;
  }
  rec_buffer[rec_w].type = term_type;

  return ret;
}

/*!
 @brief __filog_reset_buffer_i

 driver initialize

 @param        none

 @retval       none
*/
static void __filog_reset_buffer_i(void)
{
  rec_w = 0;
  rec_buffer[0].type = FILOGTYPE_MARKER_END;
}

/*!
 @brief __filog_set_new_buffer

 driver initialize

 @param [in]  new_buffer new buffer address
 @param [in]  new_max    new buffer record count
 @param [out] old_cond   old condition

 @retval        0: success
 @retval       <0: failed
*/
static void __filog_set_new_buffer(void *new_buffer, unsigned int new_max, filog_condition *old_cond)
{
  old_cond->rec_buffer = rec_buffer;
  old_cond->rec_w      = rec_w;
  old_cond->rec_e      = rec_e;
  getnstimeofday(&old_cond->cond_time);

  rec_buffer = (filog_record *)new_buffer;
  rec_w = 0;
  rec_e = new_max;
  __filog_reset_buffer_i();
}

/*!
 @brief __filog_write_logging_change

 record filog (internal use)

 @param [in]  msg_type  message type
 @param [in]  state     state
 @param [in]  force     not zero=ignore mask

 @retval      <>0: write pointer
 @retval        0: not write
*/
static filog_record *__filog_write_logging_change(unsigned int msg_type, unsigned int state, unsigned int force)
{
  filog_record_half rec = {0};
  if(log_mask & FILOGMASK_INFORMATION || force) {
    rec.type = FILOGTYPE_LOGGING_CHANGE;
    rec.p8 = (unsigned char)msg_type;
    rec.p16  = (unsigned short)current->pid;
    rec.p32  = (unsigned int)state;
    return __filog_write_record_i(&rec);
  }
  return 0;
}

/*!
 @brief filog_write_record

 record filog

 @param [in] data      address of the data record (ignoring "time")
 @param [in] mask      log-mask bit

 @retval     none
*/
static void filog_write_record(const filog_record_half *data, unsigned int mask)
{
  unsigned long flags;
  spin_lock_irqsave(&rec_lock, flags);

  if(rec_initialized && (log_mask & mask) == mask) {
    __filog_write_record_i(data);
  }

  spin_unlock_irqrestore(&rec_lock, flags);
}


/* ==========================================================================
 *  MODULE PARAM FUNCTION
 * ========================================================================== */

/* ----------------------------------------------------------------
 * log_mask
 *    enable recording
 */
static int set_filog_log_mask(const char *val, const struct kernel_param *kp)
{
  unsigned long flags;
  int ret;
  unsigned int old_value = log_mask;
  unsigned int force = 0;

  ret = param_set_uint(val, kp);

  spin_lock_irqsave(&rec_lock, flags);

  if(!ret) {
    if(rec_initialized) {
      if(old_value & FILOGMASK_INFORMATION) {
        force = 1;
      }
      __filog_write_logging_change(FILOGTYPE_LOGGING_CHANGE_MASK, log_mask, force);
    }
  }

  spin_unlock_irqrestore(&rec_lock, flags);

  return ret;
}
static struct kernel_param_ops module_ops_filog_log_mask = {
  .set = set_filog_log_mask,
  .get = param_get_uint,
};
module_param_cb(log_mask, &module_ops_filog_log_mask, &log_mask, 0660);
MODULE_PARM_DESC(log_mask, "uint");

/* ----------------------------------------------------------------
 * rec_buffer_kb
 *    set buffer size
 */
static int set_filog_rec_buffer_kb(const char *val, const struct kernel_param *kp)
{
  unsigned long flags;
  unsigned int old_value = rec_buffer_kb;
  filog_condition old_cond = {0};
  void *new_buffer = 0;
  unsigned int new_max;
  int ret;

  ret = param_set_uint(val, kp);
  new_max = (rec_buffer_kb * 1024) / sizeof(filog_record);

  if(rec_buffer_kb > 0 && new_max >= FILOG_BUFFER_MIN_RECORD) {
    new_buffer = vmalloc(new_max * sizeof(filog_record));
    if(new_buffer) {

      spin_lock_irqsave(&rec_lock, flags);

      if(rec_initialized) {
        __filog_set_new_buffer(new_buffer, new_max, &old_cond);
        __filog_write_logging_change(FILOGTYPE_LOGGING_CHANGE_SIZE, rec_buffer_kb, 0);
      } else {
        ret = -EINVAL;
      }

      spin_unlock_irqrestore(&rec_lock, flags);

    } else {
      ret = -ENOMEM;
    }
  } else {
    ret = -EINVAL;
  }

  if(ret) {
    if(new_buffer) {
      vfree(new_buffer);
    }
    /* restore previous value */
    rec_buffer_kb = old_value;
  } else if(old_cond.rec_buffer) {
    /* discard the old data */
    vfree(old_cond.rec_buffer);
  }

  return ret;
}
static struct kernel_param_ops module_ops_filog_rec_buffer_kb = {
  .set = set_filog_rec_buffer_kb,
  .get = param_get_uint,
};
module_param_cb(rec_buffer_kb, &module_ops_filog_rec_buffer_kb, &rec_buffer_kb, 0660);
MODULE_PARM_DESC(rec_buffer_kb, "uint");

/* ----------------------------------------------------------------
 * reset
 *    reset buffer when write
 */
static int set_filog_reset(const char *val, const struct kernel_param *kp)
{
  unsigned long flags;

  spin_lock_irqsave(&rec_lock, flags);

  if(rec_initialized) {
    __filog_reset_buffer_i();
    __filog_write_logging_change(FILOGTYPE_LOGGING_CHANGE_RESET, log_mask, 0);
  }

  spin_unlock_irqrestore(&rec_lock, flags);

  return 0;
}
static struct kernel_param_ops module_ops_filog_reset = {
  .set = set_filog_reset,
  .get = param_get_uint,
};
module_param_cb(reset, &module_ops_filog_reset, &reset, 0660);
MODULE_PARM_DESC(reset, "uint");

/* for debug (read only) */
module_param(rec_w, uint, 0440);
module_param(rec_e, uint, 0440);


/* ==========================================================================
 *  PROCFS
 * ========================================================================== */

/*!
 @brief d_start

 start function for seq_file

 @param [in]   m     pointer of the seq_file struct
 @param [in]   pos   pointer of the offset

 @retval       start pointer of the output data
*/
static void *d_start(struct seq_file *m, loff_t *pos)
{
  filog_condition *workmem = (filog_condition *)m->private;
  unsigned int n = (unsigned int)(*pos);
  unsigned char term_type;

  if(*pos < 0 || workmem == NULL) {
    DBGPRINT("** d_start (0) *pos=%d rec=%p\n", (int)*pos, workmem);
    return NULL;
  }
  
  term_type = workmem->rec_buffer[workmem->rec_w].type;
  DBGPRINT("++ d_start buf=%p w=%u e=%u term_type=%u:%s *pos=%d\n", workmem, workmem->rec_w, workmem->rec_e, 
    term_type, (term_type == FILOGTYPE_MARKER_END ? "END" : term_type == FILOGTYPE_MARKER_BREAKRING ? "BREAKRING" : "UNKNOWN"), (int)*pos);
  switch(term_type) {

  case FILOGTYPE_MARKER_END:
    if(n >= workmem->rec_w) {
      DBGPRINT("-- d_start (0)\n");
      return NULL;
    }
    break;

  case FILOGTYPE_MARKER_BREAKRING:
    n++;
    if(n >= workmem->rec_e) {
      DBGPRINT("-- d_start (0)\n");
      return NULL;
    }
    n += workmem->rec_w;
    if(n >= workmem->rec_e)
       n -= workmem->rec_e;
    break;

  default:
    DBGPRINT("-- d_start (0)\n");
    return NULL;
  }

  DBGPRINT("-- d_start (%p)[%u]\n", &workmem->rec_buffer[n], n);
  return (void *)&workmem->rec_buffer[n];
}

/*!
 @brief d_next

 next function for seq_file

 @param [in]   m     pointer of the seq_file struct
 @param [in]   p     pointer of the output data
 @param [out]  pos   pointer of the offset

 @retval       next pointer of the output data
*/
static void *d_next(struct seq_file *m, void *p, loff_t *pos)
{
  filog_condition *workmem = (filog_condition *)m->private;
  filog_record *rec = (filog_record *)p;
  filog_record *prev = rec;
  filog_record *pw;

  DBGPRINT("++ d_next p=%p[%d] *pos=%d\n", rec, (int)((rec - workmem->rec_buffer) / sizeof(filog_record)), (int)*pos);
  (*pos)++;

  rec++;
  if(rec >= &workmem->rec_buffer[workmem->rec_e]) {
    rec = &workmem->rec_buffer[0];
    prev -= workmem->rec_e;
  }
  pw = &workmem->rec_buffer[workmem->rec_w];
  if(prev < pw && rec >= pw) {
    DBGPRINT("-- d_next (0)\n");
    return NULL;
  }

  DBGPRINT("-- d_next (%p)[%d] *pos=%d\n", rec, (int)((rec - workmem->rec_buffer) / sizeof(filog_record)), (int)*pos);
  return (void *)rec;
}

/*!
 @brief d_show

 output function for seq_file

 @param [in]   m     pointer of the seq_file struct
 @param [in]   p     pointer of the output data

 @retval      0: success
 @retval    <>0: failed
*/
static int d_show(struct seq_file *m, void *p)
{
  filog_condition *workmem = (filog_condition *)m->private;
  filog_record *rec = (filog_record *)p;
  int pid = 99999;
  unsigned long long t = rec->time;
  struct timespec time_tmp;
#ifdef CONFIG_ARM_ARCH_TIMER
  time_tmp.tv_nsec = do_div(t, 19200000);
  time_tmp.tv_sec = t;
  time_tmp.tv_nsec = time_tmp.tv_nsec * 52 + time_tmp.tv_nsec / 12; /* 52+1/12=1000000000/19200000 */
#else
  time_tmp.tv_nsec = do_div(t, 1000000000);
  time_tmp.tv_sec = t;
#endif /* CONFIG_ARM_ARCH_TIMER */

  switch(rec->type) {
  case FILOGTYPE_LOGGING_CHANGE:
  case FILOGTYPE_MACHINE_SUSPEND:
  case FILOGTYPE_CPU_ONLINE:
  case FILOGTYPE_CPU_FREQUENCY:
  case FILOGTYPE_DISPLAY_SUSPEND:
  case FILOGTYPE_RESUME_IRQ:
    pid = (int)rec->p16;
    break;
  default:
    break;
  }
  seq_printf(m, "           <...>-%-5d [000] .... %5ld.%06ld: ", pid, time_tmp.tv_sec, time_tmp.tv_nsec / 1000);
  
  switch(rec->type) {
  case FILOGTYPE_MACHINE_SUSPEND:
    seq_printf(m, "machine_suspend: state=%u\n", rec->p32);
    break;
  case FILOGTYPE_CPU_ONLINE:
    seq_printf(m, "cpu_online: state=%u cpu_id=%u\n", rec->p32, rec->p8);
    break;
  case FILOGTYPE_CPU_FREQUENCY:
    seq_printf(m, "cpu_frequency: state=%u cpu_id=%u\n", rec->p32, rec->p8);
    break;
  case FILOGTYPE_DISPLAY_SUSPEND:
    seq_printf(m, "display_suspend: state=%u\n", rec->p32);
    break;
  case FILOGTYPE_RESUME_IRQ:
    {
      unsigned int irq = rec->p32;
      struct irq_desc *desc = irq_to_desc(irq);
      unsigned long flags;
  		char buf[256];
      if(desc) {
        raw_spin_lock_irqsave(&desc->lock, flags);
        snprintf(buf, sizeof(buf), "name=%s%s%s action=%s handler=%pf",
          (desc->irq_data.chip ? (desc->irq_data.chip->name ? desc->irq_data.chip->name : "-") : "None"),
          (desc->name ? "-" : ""),
          (desc->name ? desc->name : ""),
          (desc->action ? desc->action->name : ""),
          (desc->action ? desc->action->handler : 0));
        raw_spin_unlock_irqrestore(&desc->lock, flags);
        seq_printf(m, "resume_irq: irq=%u %s\n", irq, buf);
      } else {
        seq_printf(m, "resume_irq: irq=%u name=? action=? handler=?\n", irq);
      }
    }
    break;
  case FILOGTYPE_LOGGING_CHANGE:
    switch (rec->p8){
    case FILOGTYPE_LOGGING_CHANGE_INIT:
      seq_printf(m, "memo: filog-init, log-mask=%08x\n", rec->p32);
      break;
    case FILOGTYPE_LOGGING_CHANGE_MASK:
      seq_printf(m, "memo: filog-change-mask, log-mask=%08x\n", rec->p32);
      break;
    case FILOGTYPE_LOGGING_CHANGE_RESET:
      seq_printf(m, "memo: filog-reset, log-mask=%08x\n", rec->p32);
      break;
    case FILOGTYPE_LOGGING_CHANGE_SIZE:
      seq_printf(m, "memo: filog-reset-rec_buffer_kb, size(kb)=%u\n", rec->p32);
      break;
    case FILOGTYPE_LOGGING_CHANGE_EXPORT:
      {
        struct rtc_time tm;
        rtc_time_to_tm(workmem->cond_time.tv_sec, &tm);
#ifdef CONFIG_ARM_ARCH_TIMER
        seq_printf(m, "memo: filog-export, timestamp=[%d-%02d-%02d %02d:%02d:%02d.%09lu UTC], timetick=[%08X%08X]\n",
                   tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                   tm.tm_hour, tm.tm_min, tm.tm_sec, workmem->cond_time.tv_nsec,
                   ((unsigned int *)&rec->time)[1], ((unsigned int *)&rec->time)[0]);
#else
        seq_printf(m, "memo: filog-export, timestamp=[%d-%02d-%02d %02d:%02d:%02d.%09lu UTC]\n",
                   tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                   tm.tm_hour, tm.tm_min, tm.tm_sec, workmem->cond_time.tv_nsec);
#endif /* CONFIG_ARM_ARCH_TIMER */
      }
      break;
    default:
      seq_puts(m, "?: ?\n");
      break;
    }
    break;
  default:
    seq_puts(m, "?: ?\n");
    break;
  }

  DBGPRINT(">> d_show\n");
  return 0;
}

/*!
 @brief d_stop

 stop function for seq_file

 @param [in]   m     pointer of the seq_file struct
 @param [in]   p     pointer of the output data

 @retval       none
*/
static void d_stop(struct seq_file *m,void *p)
{
    // do nothing
}

static const struct seq_operations show_data_seq_ops = {
  .start  = d_start,
  .next   = d_next,
  .show   = d_show,
  .stop   = d_stop,
};

/*!
 @brief filog_data_open

 open function for /proc/filog/data

 @param [in]   inode   pointer of the inode struct
 @param [in]   file    pointer of the file struct

 @retval      0: success
 @retval    <>0: failed
*/
static int filog_data_open(struct inode *inode, struct file *file)
{
  unsigned long flags;
  int ret = 0;
  void *new_buffer;
  unsigned int new_max;
  filog_condition *workmem;

  workmem = (filog_condition *)kmalloc(sizeof(filog_condition), GFP_ATOMIC);
  if(!workmem) {
    return -ENOMEM;
  }
  memset(workmem, 0, sizeof(filog_condition));

  new_max = (rec_buffer_kb * 1024) / sizeof(filog_record);
  new_buffer = vmalloc(new_max * sizeof(filog_record));
  if(!new_buffer) {
    ret = -ENOMEM;
    goto end;
  }

  spin_lock_irqsave(&rec_lock, flags);

  if(rec_initialized) {
    __filog_write_logging_change(FILOGTYPE_LOGGING_CHANGE_EXPORT, 0, 0);
    __filog_set_new_buffer(new_buffer, new_max, workmem);
  } else {
    ret = -EINVAL;
  }

  spin_unlock_irqrestore(&rec_lock, flags);

  if(ret) {
    vfree(new_buffer);
  }

  if(workmem->rec_buffer) {
    if(!ret) {
      ret = seq_open(file, &show_data_seq_ops);
    }
    if(ret) {
      vfree(workmem->rec_buffer);
    }
  }

end:
  if(!ret) {
    ((struct seq_file *)file->private_data)->private = (void *)workmem;
  } else {
    kfree((void *)workmem);
  }

  return ret;
}

/*!
 @brief filog_data_release

 release function for /proc/filog/data

 @param [in]   inode   pointer of the inode struct
 @param [in]   file    pointer of the file struct

 @retval      0: success
 @retval    <>0: failed
*/
static int filog_data_release(struct inode *inode, struct file *file)
{
  struct seq_file *m = (struct seq_file *)file->private_data;
  filog_condition *workmem = (filog_condition *)m->private;
  m->private = NULL;

  if(workmem) {
    if(workmem->rec_buffer) {
      vfree(workmem->rec_buffer);
    }
    kfree(workmem);
  }
  return seq_release(inode, file);
}

static struct file_operations filog_data_op = {
    .open = filog_data_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = filog_data_release,
};


/* ==========================================================================
 *  GLOBAL FUNCTIONS
 * ========================================================================== */

/*!
 @brief filog_machine_suspend

 record machine_suspend event

 @param [in] state    state

 @retval     none
*/
void filog_machine_suspend(unsigned int state)
{
  filog_record_half rec = {0};
  rec.type = FILOGTYPE_MACHINE_SUSPEND;
  rec.p16  = (unsigned short)current->pid;
  rec.p32  = (unsigned int)state;
  filog_write_record(&rec, FILOGMASK_SUSPEND_STATE);
}
EXPORT_SYMBOL(filog_machine_suspend);

/*!
 @brief filog_cpu_online

 record cpu_online event

 @param [in] state    state
 @param [in] cpu_id   cpu_id

 @retval     none
*/
void filog_cpu_online(unsigned int state, unsigned int cpu_id)
{
  filog_record_half rec = {0};
  rec.type = FILOGTYPE_CPU_ONLINE;
  rec.p8   = (unsigned char)cpu_id;
  rec.p16  = (unsigned short)current->pid;
  rec.p32  = (unsigned int)state;
  filog_write_record(&rec, FILOGMASK_SUSPEND_STATE);
}
EXPORT_SYMBOL(filog_cpu_online);

/*!
 @brief filog_cpu_frequency

 record cpu_frequency event

 @param [in] state    state
 @param [in] cpu_id   cpu_id

 @retval     none
*/
void filog_cpu_frequency(unsigned int state, unsigned int cpu_id)
{
  filog_record_half rec = {0};
  rec.type = FILOGTYPE_CPU_FREQUENCY;
  rec.p8   = (unsigned char)cpu_id;
  rec.p16  = (unsigned short)current->pid;
  rec.p32  = (unsigned int)state;
  filog_write_record(&rec, FILOGMASK_VARIABLE_STATE);
}
EXPORT_SYMBOL(filog_cpu_frequency);

/*!
 @brief filog_display_suspend

 record display_suspend event

 @param [in] state    state

 @retval     none
*/
void filog_display_suspend(unsigned int state)
{
  filog_record_half rec = {0};
  rec.type = FILOGTYPE_DISPLAY_SUSPEND;
  rec.p16  = (unsigned short)current->pid;
  rec.p32  = (unsigned int)state;
  filog_write_record(&rec, FILOGMASK_SUSPEND_STATE);
}
EXPORT_SYMBOL(filog_display_suspend);

/*!
 @brief filog_resume_irq

 record resume_irq event

 @param [in] irq      irq

 @retval     none
*/
void filog_resume_irq(unsigned int irq)
{
  filog_record_half rec = {0};
  rec.type = FILOGTYPE_RESUME_IRQ;
  rec.p16  = (unsigned short)current->pid;
  rec.p32  = (unsigned int)irq;
  filog_write_record(&rec, FILOGMASK_RESUME_IRQ);
}
EXPORT_SYMBOL(filog_resume_irq);


/* -------------------------------- */

/*!
 @brief filog_init

 driver initialize

 @param        none

 @retval        0: success
 @retval       <0: failed
*/
static int __init filog_init(void)
{
  filog_condition dummy_cond = {0};
  void *new_buffer;
  unsigned int new_max;
  int ret = 0;

  new_max = (rec_buffer_kb * 1024) / sizeof(filog_record);
  new_buffer = vmalloc(new_max * sizeof(filog_record));
  if(new_buffer) {
    __filog_set_new_buffer(new_buffer, new_max, &dummy_cond);
    __filog_write_logging_change(FILOGTYPE_LOGGING_CHANGE_INIT, log_mask, 0);

    proc_filog = proc_mkdir("filog", NULL);
    if(proc_filog) {
      proc_filog_data = proc_create("data", 0440, proc_filog, &filog_data_op);
    }

    rec_initialized++;
  } else {
    ret = -ENOMEM;
  }

  return ret;
}

/*!
 @brief filog_exit

 driver terminate

 @param         none

 @retval        none
*/
static void __exit filog_exit(void)
{
  unsigned long flags;

  spin_lock_irqsave(&rec_lock, flags);
  if(rec_initialized)
    rec_initialized--;
  spin_unlock_irqrestore(&rec_lock, flags);

  if(proc_filog_data) {
    remove_proc_entry("data", proc_filog);
  }
  if(proc_filog) {
    remove_proc_entry("filog", NULL);
  }
}

module_init(filog_init);
module_exit(filog_exit);

MODULE_DESCRIPTION("FTPT");
MODULE_LICENSE("GPL v2");
