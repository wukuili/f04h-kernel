/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2014-2015
/*----------------------------------------------------------------------------*/
#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

/* FUJITSU LIMITED:2014-11-26 H1510044 add start */
#include <asm-generic/ioctl.h>
#define HWKEYS_IOC_MAGIC 'K'
#define HWKEYS_VOICE_CALL_START _IOW(HWKEYS_IOC_MAGIC, 0x01, int)
#define HWKEYS_VOICE_CALL_STOP  _IOW(HWKEYS_IOC_MAGIC, 0x02, int)
/* FUJITSU LIMITED:2014-11-26 H1510044 add end */

struct device;

struct gpio_keys_button {
	/* Configuration parameters */
	unsigned int code;	/* input event code (KEY_*, SW_*) */
	int gpio;		/* -1 if this key does not support gpio */
	int active_low;
	const char *desc;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
/* FUJITSU LIMITED:2014-11-26 H1510041 add start */
	int debounce_count;	/* debounce count for chattering check  */
/* FUJITSU LIMITED:2014-11-26 H1510041 add end */
	bool can_disable;
	int value;		/* axis value for EV_ABS */
	unsigned int irq;	/* Irq number in case of interrupt keys */
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
	unsigned int poll_interval;	/* polling interval in msecs -
					   for polling driver only */
	unsigned int rep:1;		/* enable input subsystem auto repeat */
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;		/* input device name */
};

/* FUJITSU LIMITED:2014-11-26 H1510044 add start */
/************************/
/* Kernel IF            */
/* Get Call Status      */
/************************/
#define GPIO_KEYS_SUPPORT_CALL_STATUS    /* FUJITSU LIMITED:2015-05-20 H1520363 add */
extern bool get_gpio_keys_call_status(void);
/* FUJITSU LIMITED:2014-11-26 H1510044 add end */

#endif
