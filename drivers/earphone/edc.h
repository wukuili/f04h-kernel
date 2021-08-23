/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012-2015
/*----------------------------------------------------------------------------*/

#ifndef _EDC_H_
#define _EDC_H_

#include <linux/spinlock.h>

//#define FJ_EDC_USB_EARPHONE
//#define FJ_EDC_PIERCE_GPIO
#define FJ_EDC_PLUG_CHATT

#define	HSCALL_PUSH_INTERVAL	20

#define PHI35_LOOP 1000/* 1sec */
#define RESUME_PHI35_LOOP 1200/* 1.2sec */ 

enum {
	EDC_PLUG	= 0,
	EDC_SWITCH	= 1,
	EDC_NUM_OF_COUNT,
};

enum {
	EDC_NO_DEVICE	= 0,
	EDC_DEVICE		= 1,
	EDC_DEVICE_HEADSET_NO_MIC	= 2,
};

enum {
	EARPHONE_PROBE_STAGE_0 = 0,
	EARPHONE_PROBE_STAGE_1,
	EARPHONE_PROBE_STAGE_2,
	EARPHONE_PROBE_STAGE_3,
};

enum {
	EARPHONE_REMOVE_STAGE_0 = 0,
	EARPHONE_REMOVE_STAGE_1,
};

enum {
	EARPHONE_RESUME_STAGE_0 = 0,
	EARPHONE_RESUME_STAGE_1,
	EARPHONE_RESUME_STAGE_2,
	EARPHONE_RESUME_STAGE_3,
};

enum {
	EARPHONE_USB_STAGE_0 = 0,
	EARPHONE_USB_STAGE_1,
};

enum {
	EARPHONE_NOT = 0,
	EARPHONE_NORMAL,
	EARPHONE_MIC,
	EARPHONE_PIERCE,
	EARPHONE_ANTENNA,
};

enum {
	EDC_OK = 0,
	EDC_NG,
};

enum{
	EDC_STATE_INIT=0,
	EDC_STATE_LOOP,
	EDC_STATE_CHATT,
	EDC_STATE_DESITION,
	EDC_STATE_ISR,
	EDC_STATE_PLUG,
	EDC_STATE_SUSPEND,
	EDC_STATE_RESUME,
	EDC_STATE_SWITCHISR,
	EDC_STATE_SWITCHWQ,
	EDC_STATE_MAX,
};
#define EDC_STATE_OK  0
#define EDC_STATE_NG -1

#define EDC_MODEL_TYPE_NOT    	0x00
#define EDC_MODEL_TYPE_PIERCE 	0x01
#define EDC_MODEL_TYPE_USB    	0x02
#define EDC_MODEL_TYPE_ANTENNA	0x04

typedef struct edc_handle {
	int io_data;				// IO status
	int Polling_Counter;		// chattering counter
	struct timer_list timer;	// struct timer
	int	gpio_det_irq;			// IO DET data
	int timer_flag;				// timer flag
} edc_handle_t;

#define	EDC_DBG_PRINTK(a...) \
    do { if (edc_debug_mask & DBG_EDC) { \
		printk(KERN_DEBUG a); \
    } } while (0)

#define	EDC_DBG_LOOP_PRINTK(a...) \
    do { if (edc_debug_mask & DBG_EDC_LOOP) { \
		printk(KERN_DEBUG a); \
    } } while (0)
    
#define	EDC_DBG_IRQ_PRINTK(a...) \
    do { if (edc_debug_mask & DBG_EDC_IRQ) { \
		printk(KERN_DEBUG a); \
    } } while (0)
    
#define	EDC_DBG_STATE_PRINTK(a...) \
    do { if (edc_debug_mask & DBG_EDC_STATE) { \
		printk(KERN_DEBUG a); \
    } } while (0)

/* non-volatile */
#define APNV_EDC_DBG_MODE		49072
#define APNV_SIZE_EDC_DBG_MODE		8
#define APNV_ADR_EDC_DBG_MODE		5

#define	OPT_PIERCE_SWITCH	1
#define	OPT_USB_EARPHONE	0
#define	OPT_NV_READ			1
#define OPT_AUDIO_POLLING	0

/* ------------------------------------------------------------------------ */
/* enum                                                                     */
/* ------------------------------------------------------------------------ */
/* for Attributes(Argument type) */
enum {
	DBG_EDC             = 1 <<   0,
	DBG_EDC_LOOP        = 1 <<   1,
	DBG_EDC_IRQ         = 1 <<   2,
	DBG_EDC_STATE       = 1 <<   3
};

extern uint8_t edc_debug_mask;

#endif /* _EDC_H_ */
