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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/interrupt.h>

#include "msm_led_flash.h"

#define FLASH_NAME "bd7757mwx"
#define USE_HARD	// if comment out, you will not access gpio.
#define SEND_ADDR_MAX 6
#define DEBUG_LOG_LEVEL KERN_INFO
static int dbg_level = 0;
static int mode = 0;
static spinlock_t slock;
#define LED_MODE_WITHOUT_BREAK 0
#define LED_MODE_FLASH_BREAK 1
#define LED_MODE_TORCH_BREAK 2
#define LED_MODE_TORCH_BREAK_IN_TORCH 3
typedef enum _bd7757mwx_led_state {
	BD7757MWX_LED_OFF,
	BD7757MWX_LED_ON,
} bd7757mwx_led_state;
static bd7757mwx_led_state led_state_bk = BD7757MWX_LED_OFF;
#define T_IRLED_POWER_ON  5000
#define T_IRLED_POWER_OFF 2500
#define T_HI 2
#define T_LO 2
#define T_ACC 501
#define T_OFF 501
#define T_LAT 1001
#define LED_PORT_SEND_DATA_DELAY_USEC (T_ACC + 2*T_LAT + 26*(T_HI+T_LO))
static int port_write_delay = 0;

static int torch_lo = 11;   // 100mA
static int torch_hi = 1;    // 300mA
#define CURRENT2TORCH_DATA(led_current) ((300 - led_current)/20 + 1)
#define TORCH_DATA2CURRENT(data)    (300-(20*(data-1)))

#define DBG_LOG(fmt, args...) \
	do { if (dbg_level > 0) { \
		printk(DEBUG_LOG_LEVEL "%s: " fmt , __FUNCTION__, ## args); \
	} } while(0)

#define REG_LOG(fmt, args...) \
	do { if (dbg_level >= 0) { \
		printk(DEBUG_LOG_LEVEL "%s: " fmt , __FUNCTION__, ## args); \
	} } while(0)

#define BD7757MWX_PINCTRL_STATE_ACTIVE  "led_active"
#define BD7757MWX_PINCTRL_STATE_SUSPEND "led_sleep"
#define LOAD_SWITCH_PINCTRL_STATE_ACTIVE  "load_switch_active"
#define LOAD_SWITCH_PINCTRL_STATE_SUSPEND "load_switch_sleep"

#define SYSTEM_REV_MASK 0x000F
#define GPIO_CONFIG_INPUT 0x02
extern unsigned int system_rev;

struct bd7757mwx_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	struct pinctrl_state *load_switch_gpio_state_active;
	struct pinctrl_state *load_switch_gpio_state_suspend;
	bool use_pinctrl;
	bool use_load_switch;
};

struct bd7757mwx_data {
	struct led_classdev cdev;
	struct msm_led_flash_ctrl_t *fctrl;
	bool irq_requested;
};

static int send_data_bk[SEND_ADDR_MAX];

enum hard_state {
	P_OFF = 0,
	P_ON,
	P_XFAULT,
	P_MAX,
};

enum control_port_index {
	CONTROL_1 = 0,
	CONTROL_2,
	IRLED_XFAULT,
	IRLED_EN,
	CONTROL_MAX,
};

enum control_port_state {
	PORT_DISABLE = 0,
	PORT_ENABLE,
};

struct _port_info {
	int gpio_num;
	int value;
	bool state;
	const char * arg;
};

static struct _port_info port_info[CONTROL_MAX] = {
	{-1, -1, PORT_DISABLE, "led_control"},
	{-1, -1, PORT_DISABLE, "led_flash"},
	{-1, -1, PORT_DISABLE, "irled_xfault"},
	{-1, -1, PORT_DISABLE, "led_en"},
};

static int time_on = 25;
static int time_to_on = 25;
static struct workqueue_struct * work_queue;
static struct work_struct led_on_work;
static struct semaphore led_sem;
static int cpu_id = 0;
static u64 time_sof;
static bool work_started = false;

static int g_current_hard_state = P_OFF;
static struct bd7757mwx_pinctrl_info my_pinctrl_info;
static int bd7757mwx_irq;
static bool bd7757mwx_irq_enabled = false;

static struct msm_led_flash_ctrl_t fctrl;
static struct timer_list g_led_timeout_list;
#define LED_TIMEOUT_TIME_MAX        (5 * 60)
static unsigned int g_led_timeout_time;
static struct tm tm_xfault;
static struct timeval tv_xfault;

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static void led_power_state_on(void);
static void led_power_state_off(void);
//static void led_timeout_timer_add(void);

static void print_logtime(void)
{
	struct timeval tv;
	struct tm tm;
	
	if (g_current_hard_state == P_XFAULT) {
		pr_err("--tm_xfault-- %02d-%02d %02d:%02d:%02d.%03ld\n", tm_xfault.tm_mon+1, tm_xfault.tm_mday,
			tm_xfault.tm_hour, tm_xfault.tm_min, tm_xfault.tm_sec, tv_xfault.tv_usec/1000);
	} else {
		if (dbg_level < 2) {
			return;
		}
		do_gettimeofday(&tv);
		time_to_tm(tv.tv_sec, 9*60*60, &tm);	// JST
		DBG_LOG("--tm-- %02d-%02d %02d:%02d:%02d.%03ld\n", tm.tm_mon+1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, tv.tv_usec/1000);
	}
}

static void print_logtime_xfault(void)
{
	if (g_current_hard_state != P_XFAULT)
		return;
	do_gettimeofday(&tv_xfault);
	time_to_tm(tv_xfault.tv_sec, 9*60*60, &tm_xfault);	// JST
	pr_err("--tm_xfault-- %02d-%02d %02d:%02d:%02d.%03ld\n", tm_xfault.tm_mon+1, tm_xfault.tm_mday,
			tm_xfault.tm_hour, tm_xfault.tm_min, tm_xfault.tm_sec, tv_xfault.tv_usec/1000);
}

static inline void port_write(enum control_port_index idx, int value)
{
#ifdef USE_HARD
	gpio_set_value(port_info[idx].gpio_num, value);
#endif
	port_info[idx].value = value;
}

/* When performing a hardware operation of Driver IC, and to use this function.
   Is intended to exclusively the upic_send_data() and port_ctrl().
*/
static void port_ctrl(enum control_port_index idx, int value)
{
	unsigned long flags;
	spin_lock_irqsave(&slock, flags);
	
	port_write(idx, value);
	
	spin_unlock_irqrestore(&slock, flags);
}

static void upic_send_data(int addr, int data)
{
	unsigned long flags;
	int i;

	DBG_LOG("addr, data = %d, %d.\n", addr, data);
	if ((addr < 1) || (addr > SEND_ADDR_MAX) || (data < 1) || (data > 16)) {
		pr_err("%s:parameter error! addr,data = %d, %d\n", __FUNCTION__,  addr, data);
		return;
	}

	if (g_current_hard_state == P_OFF) {
		DBG_LOG("%s: current_hard_state is OFF!!\n", __FUNCTION__);
		led_power_state_on();
	}

	spin_lock_irqsave(&slock, flags);

	for (i = 0; i < addr; i++) {
		port_write(CONTROL_1, 0);
		udelay(T_LO);
		port_write(CONTROL_1, 1);
		udelay(T_HI);
	}
	udelay(T_LAT - T_HI);

	for (i = 0; i < data + 12; i++) {
		port_write(CONTROL_1, 0);
		udelay(T_LO);
		port_write(CONTROL_1, 1);
		udelay(T_HI);
	}
	udelay(T_LAT - T_HI);
	send_data_bk[addr - 1] = data;
	spin_unlock_irqrestore(&slock, flags);
	DBG_LOG("END.\n");
}

static void led_light(bd7757mwx_led_state data)
{
	switch (mode) {
	case LED_MODE_TORCH_BREAK:
		if (data == BD7757MWX_LED_ON)
			upic_send_data(1, 2);
		else
			upic_send_data(1, 1);
		break;
	case LED_MODE_TORCH_BREAK_IN_TORCH:
		if (data == BD7757MWX_LED_ON)
			upic_send_data(3, torch_hi);
		else
			upic_send_data(3, torch_lo);
		break;
	default:
		port_ctrl(CONTROL_2, (data == BD7757MWX_LED_ON)?1:0);
		break;
	}
	led_state_bk = data;
	DBG_LOG("LED_FLASH_%s\n", (data == BD7757MWX_LED_ON)?"LED_ON":"LED_OFF");
}
#if 0
static void bd7757mwx_reset_device(void)
{
	/* BD7757MWX p-off */
	port_ctrl(CONTROL_1, 0);

	usleep(T_OFF); //TODO: fix p-off mergin time

	/* BD7757MWX p-on and initialize*/
	upic_send_data(2, 16);
	upic_send_data(3, 11);
	upic_send_data(4, 16);
}
#endif
static void bd7757mwx_on_work(struct work_struct *work)
{
	CDBG("%s: start.\n", __FUNCTION__);
	while(1) {
		u64 diff_time;
		int time_to_on_remain_usec;
		led_light(BD7757MWX_LED_OFF);
		down(&led_sem);
		diff_time = cpu_clock(cpu_id) - time_sof;
		do_div(diff_time, 1000);
		time_to_on_remain_usec = 1000 * time_to_on - (int)diff_time - port_write_delay;
		DBG_LOG("time_sof, diff_time = 0x%llx, 0x%llx, time_to_on_remain_usec = %d\n", time_sof, diff_time, time_to_on_remain_usec);
		if (time_to_on_remain_usec < 0) {
			pr_err("%s: something wrong happen!!\n", __FUNCTION__);
			continue;
		}
		usleep(time_to_on_remain_usec);
		led_light(BD7757MWX_LED_ON);
		usleep(1000 * time_on - port_write_delay);
		print_logtime();
	}
	pr_err("%s: end!!.\n", __FUNCTION__);
}

static void incam_light_if_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	if ((time_on == 0) || (mode == LED_MODE_WITHOUT_BREAK)) {
		DBG_LOG("no frame sync control.\n");
		if (value)
			led_light(BD7757MWX_LED_ON);
		else
			led_light(BD7757MWX_LED_OFF);
	} else if (value) {
		if (!work_started) {
			work_started = true;
			queue_work(work_queue, &led_on_work);
		}
		time_sof = cpu_clock(cpu_id);
		up(&led_sem);
		DBG_LOG("up semaphore. time_sof = 0x%llx.\n", time_sof);
	} else {
		led_light(BD7757MWX_LED_OFF);
	}
}

static void pinctrl_info_config(bool active)
{
	int ret;
	struct pinctrl_state *gpio_set_state;
	struct pinctrl_state *gpio_set_state_load_switch;

	DBG_LOG("%s: start. active(%d)\n", __FUNCTION__, active);

	if (active) {
		gpio_set_state = my_pinctrl_info.gpio_state_active;
		gpio_set_state_load_switch = my_pinctrl_info.load_switch_gpio_state_active;
	} else {
		gpio_set_state = my_pinctrl_info.gpio_state_suspend;
		gpio_set_state_load_switch = my_pinctrl_info.load_switch_gpio_state_suspend;
	}

	ret = pinctrl_select_state(my_pinctrl_info.pinctrl, gpio_set_state);
	if (ret) {
		pr_err("%s:%d cannot set pin to active state\n",
			__FUNCTION__, __LINE__);
	}

	if (my_pinctrl_info.use_load_switch) {
		ret = pinctrl_select_state(my_pinctrl_info.pinctrl, gpio_set_state_load_switch);
		if (ret) {
			pr_err("%s:%d cannot set pin to load_switch active state\n",
				__FUNCTION__, __LINE__);
		}
	}
	return; 
}

static void led_timeout_handler(unsigned long data){
	DBG_LOG("start");
	led_power_state_off();
	DBG_LOG("end");
}

static void led_timeout_timer_add(void)
{
	DBG_LOG("start");
	init_timer(&g_led_timeout_list);
	g_led_timeout_list.expires = jiffies + (g_led_timeout_time * HZ);
	DBG_LOG("g_led_timeout_time(%d)", g_led_timeout_time);
	g_led_timeout_list.function = led_timeout_handler;
	add_timer(&g_led_timeout_list);
	DBG_LOG("end");
}

static void led_timeout_timer_del(void){
	DBG_LOG("start\n");
	del_timer(&g_led_timeout_list);
	DBG_LOG("end\n");
}

static void led_power_state_on(void)
{
	DBG_LOG("%s: start\n", __FUNCTION__);

	pinctrl_info_config(true); // active state

	port_ctrl(IRLED_EN, 1);	//IRLED_POWER_ON
	udelay(T_IRLED_POWER_ON);

	port_ctrl(CONTROL_1, 1);	//DriverIC start
	udelay(T_ACC);

	g_current_hard_state = P_ON;

	upic_send_data(3, 6);		// 300mA
	upic_send_data(5, 7);		// Timer start current 100mA

	if (g_led_timeout_time) {
		upic_send_data(4, 15);		// Timer 2000ms
		led_timeout_timer_add();
	} else {
		upic_send_data(4, 16);		// Timer OFF
	}
	DBG_LOG("%s: end\n", __FUNCTION__);
}

static void led_power_state_off(void)
{
	DBG_LOG("%s: start\n", __FUNCTION__);

	led_timeout_timer_del();
    port_ctrl(CONTROL_1, 0);	//DriverIC stop
    udelay(T_OFF);
    port_ctrl(IRLED_EN, 0);	//IRLED_POWER_OFF

	pinctrl_info_config(false); // sleep state
	if (g_current_hard_state != P_XFAULT) {
		g_current_hard_state = P_OFF;
	}

	DBG_LOG("%s: end\n", __FUNCTION__);
}

static bool xfalt_irq_state(void)
{
	bool ret = false;

	DBG_LOG("%s: g_current_hard_state(%d)\n", __FUNCTION__, g_current_hard_state);
	print_logtime();
	if (g_current_hard_state == P_XFAULT) {
		pr_err("%s: XFAULT Interrupt Discovered(%d)\n", __FUNCTION__, g_current_hard_state);
		ret = true;
	}
	return ret;
}

static ssize_t led_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev * led_cdev = dev_get_drvdata(dev);
	char *after;
	unsigned long value;

	print_logtime();
	if (xfalt_irq_state()) {
		return size;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	value = simple_strtoul(buf, &after, 10);
	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\"), size=%d, value=0x%lx]\n", dev, attr, buf, buf, (int)size, value);
	incam_light_if_set(led_cdev, value);
	return size;
}

static ssize_t led_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if (xfalt_irq_state()) {
		return ret;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\")]\n", dev, attr, buf, buf);
	ret = sprintf(buf, "led_state_bk = %d, it means %s",
				led_state_bk,
				(led_state_bk == BD7757MWX_LED_ON)?"LED_ON":"LED_OFF");
	return ret;
}

static ssize_t ctrl1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long value;

	print_logtime();
	if (xfalt_irq_state()) {
		return size;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}
	value = simple_strtoul(buf, &after, 10);
	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\"), size=%d, value=0x%lx]\n", dev, attr, buf, buf, (int)size, value);
	port_ctrl(CONTROL_1, (value ? 1 : 0));

	if (value == 0) {
		int i;
		for (i = 0; i < SEND_ADDR_MAX; i++)
			send_data_bk[i] = -1;
	}
	return size;
}

static ssize_t ctrl1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if (xfalt_irq_state()) {
		return ret;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}
	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\")]\n", dev, attr, buf, buf);
	ret = sprintf(buf, "control_1 is gpio%d, set value is %d\n", port_info[CONTROL_1].gpio_num, port_info[CONTROL_1].value);
	return ret;
}

static ssize_t ctrl2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long value;

	print_logtime();
	if (xfalt_irq_state()) {
		return size;
	}

	if (port_info[CONTROL_2].state) {
		if (unlikely(buf == NULL)) {
			pr_err("%s: parameter err.\n", __FUNCTION__);
			return -EINVAL;
		}

		value = simple_strtoul(buf, &after, 10);
		DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\"), size=%d, value=0x%lx]", dev, attr, buf, buf, (int)size, value);
		port_ctrl(CONTROL_2, (value ? 1 : 0));
	} else {
		DBG_LOG("%s: control_2 is not suport!!\n", __FUNCTION__);
	}
	return size;
}

static ssize_t ctrl2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if (xfalt_irq_state()) {
		return ret;
	}

	if (port_info[CONTROL_2].state) {
		if (unlikely(buf == NULL)) {
			pr_err("%s: parameter err.\n", __FUNCTION__);
			return -EINVAL;
		}

		DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\")]", dev, attr, buf, buf);
		ret = sprintf(buf, "control_2 is gpio%d, set value is %d\n", port_info[CONTROL_2].gpio_num, port_info[CONTROL_2].value);
	} else {
		DBG_LOG("%s: control_2 is not suport!!\n", __FUNCTION__);
		ret = sprintf(buf, "control_2 is not suport!!\n");
	}
	return ret;
}

static ssize_t send_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, data;
	int ret;

	print_logtime();
	if (xfalt_irq_state()) {
		return size;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\"), size=%d]\n", dev, attr, buf, buf, (int)size);
	ret = sscanf(buf, "%d %d", &addr, &data);
	DBG_LOG("ret, [addr, data] = %d, [%d, %d]\n", ret, addr, data);
	if (addr == 1) {
		if (data == 1) { // LED POWER_OFF
			led_state_bk = BD7757MWX_LED_OFF;
		} else {
			led_state_bk = BD7757MWX_LED_ON;
		}
	}
	upic_send_data(addr, data);
	return size;
}

static ssize_t send_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t size = 0;
	ssize_t ret = 0;
	int i;

	if (xfalt_irq_state()) {
		return ret;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\")]\n", dev, attr, buf, buf);
	for (i = 0; i < SEND_ADDR_MAX; i++) {
		size = sprintf(buf + ret, "\naddr[%d]: %d.", i+1, send_data_bk[i]);
		ret += size;
	}
	sprintf(buf + ret, "\n");
	ret += 1;
	return ret;
}

static ssize_t time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if (xfalt_irq_state()) {
		return ret;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\")]\n", dev, attr, buf, buf);
	ret = sprintf(buf, "time on is %d[ms].", time_on);
	ret += sprintf(buf+ret, "time to on is %d[ms].", time_to_on);
	return ret;
}

static ssize_t time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	if (xfalt_irq_state()) {
		return size;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\"), size=%d]\n", dev, attr, buf, buf, (int)size);
	ret = sscanf(buf, "%d %d", &time_on, &time_to_on);
	DBG_LOG("ret, [time_on, time_to_on] = %d, [%d, %d]\n", ret, time_on, time_to_on);
	return size;
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	dbg_level = (int)simple_strtoul(buf, &after, 10);
	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\"), size=%d, dbg_level=%d]\n", dev, attr, buf, buf, (int)size, dbg_level);
	return size;
}

static ssize_t debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\")]\n", dev, attr, buf, buf);
	ret = sprintf(buf, "dbg_level is %d\n", dbg_level);
	return ret;
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;

	print_logtime();
	if (xfalt_irq_state()) {
		return size;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	mode = (int)simple_strtoul(buf, &after, 10);
	if ((mode == LED_MODE_TORCH_BREAK) || (mode == LED_MODE_TORCH_BREAK_IN_TORCH))
		port_write_delay = LED_PORT_SEND_DATA_DELAY_USEC;
	else
		port_write_delay = 0;

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\"), size=%d, mode=%d, port_write_delay=%d]\n", dev, attr, buf, buf, (int)size, mode, port_write_delay);
	return size;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if (xfalt_irq_state()) {
		return ret;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\")]\n", dev, attr, buf, buf);
	ret = sprintf(buf, "mode is %d", mode);
	return ret;
}

static ssize_t torch_lo_hi_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	print_logtime();
	if (xfalt_irq_state()) {
		return ret;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\")]\n", dev, attr, buf, buf);
	ret = sprintf(buf, "torch_lo is %d (%dmA).", torch_lo, TORCH_DATA2CURRENT(torch_lo));
	ret += sprintf(buf+ret, "torch_hi is %d (%dmA).", torch_hi, TORCH_DATA2CURRENT(torch_hi));
	ret += sprintf(buf+ret, "If set data:X, current:Y[mA] = 300 - (20 * (X - 1)).");
	return ret;
}

static ssize_t torch_lo_hi_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	print_logtime();
	if (xfalt_irq_state()) {
		return size;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("[dev=0x%p, attr=0x%p, buf=0x%p(\"%s\"), size=%d]\n", dev, attr, buf, buf, (int)size);
	ret = sscanf(buf, "%d %d", &torch_lo, &torch_hi);
	DBG_LOG("ret, [torch_lo, torch_hi] = %d, [%d, %d]\n", ret, torch_lo, torch_hi);
	return size;
}

static ssize_t hard_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	print_logtime();
	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	DBG_LOG("BD7757MWX hard_state : state %d, timeout %d\n", g_current_hard_state, g_led_timeout_time);
	ret = sprintf(buf, "%d %d\n", g_current_hard_state, g_led_timeout_time);
	return ret;
}

static ssize_t hard_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int setting_hard_state = 0;

	print_logtime();
	if (xfalt_irq_state()) {
		return size;
	}

	if (unlikely(buf == NULL)) {
		pr_err("%s: parameter err.\n", __FUNCTION__);
		return -EINVAL;
	}

	sscanf(buf, "%d %d", &setting_hard_state, &g_led_timeout_time);
	if (g_led_timeout_time > LED_TIMEOUT_TIME_MAX) g_led_timeout_time = LED_TIMEOUT_TIME_MAX;
	REG_LOG("start g_current_hard_state(%d), setting_hard_state(%d), g_led_timeout_time(%d)\n", g_current_hard_state, setting_hard_state, g_led_timeout_time);
	if (setting_hard_state >= P_XFAULT) {
		pr_err("%s: setting value is Invalid\n", __FUNCTION__);
		return -EINVAL;
	}

	if (setting_hard_state != g_current_hard_state) {
		if (setting_hard_state == P_ON) {
			led_power_state_on();
		} else {
			led_power_state_off();
		}
	} else {
		DBG_LOG("Setting value is same : %d\n", g_current_hard_state);
	}

	REG_LOG("%s: end current_hard_state(%d)\n", __FUNCTION__, g_current_hard_state);

	return size;
}

static const struct device_attribute leds_attr[] = {
	__ATTR(led_ctrl,  0660, led_ctrl_show, led_ctrl_store),
	__ATTR(ctrl1,  0660, ctrl1_show, ctrl1_store),
	__ATTR(ctrl2,  0660, ctrl2_show, ctrl2_store),
	__ATTR(send_data,  0660, send_data_show, send_data_store),
	__ATTR(time,  0660, time_show, time_store),
	__ATTR(debug,  0660, debug_show, debug_store),
	__ATTR(mode,  0660, mode_show, mode_store),
	__ATTR(torch_lo_hi,  0660, torch_lo_hi_show, torch_lo_hi_store),
	__ATTR(hard_state,  0660, hard_state_show, hard_state_store),
	__ATTR_NULL,
};

static void bd7757mwx_if_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	DBG_LOG("value=%u\n", value);
	incam_light_if_set(led_cdev, value);
}

static int __init bd7757mwx_if_sysfs_create_files_init(struct device *dev, const struct device_attribute *attrs)
{
	int ret = 0;
	int i;

	for (i = 0; attrs[i].attr.name != NULL ; i++) {
		ret = sysfs_create_file(&dev->kobj, &attrs[i].attr);
		if (ret) {
			dev_err(dev, "failure sysfs_create_file \"%s\"\n", attrs[i].attr.name);
			while (--i >= 0)
				sysfs_remove_file(&dev->kobj, &attrs[i].attr);
			break;
		}
	}
	return ret;
}

static int bd7757mwx_if_sysfs_remove_files(struct device *dev, const struct device_attribute *attrs)
{
	for ( ; attrs->attr.name != NULL; attrs++)
		sysfs_remove_file(&dev->kobj, &attrs->attr);
	return 0;
}

static irqreturn_t bd7757mwx_irq_function(int irq, void *data)
{
	g_current_hard_state = P_XFAULT;
	led_timeout_timer_del();
	port_ctrl(CONTROL_1, 0);	//DriverIC stop
	port_ctrl(IRLED_EN, 0);	//IRLED_POWER_OFF
	print_logtime_xfault();
	pr_err("%s: IRLED_XFAULT is FALLING !! g_current_hard_state(%d)\n", __FUNCTION__, g_current_hard_state);
	return IRQ_HANDLED;
}

static int led_hard_init_config(struct bd7757mwx_data *led_data, int gpio_num, enum of_gpio_flags flag, int gpio_idx)
{
	int ret;

#ifdef	USE_HARD
	if (gpio_idx == IRLED_XFAULT) {
		ret = gpio_direction_input(gpio_num);
		if (unlikely(ret)) {
			pr_err("gpio_direction_input[%d] failed.\n", gpio_num);
			return -EIO;
		}
		bd7757mwx_irq = gpio_to_irq(gpio_num);
		if (request_threaded_irq(bd7757mwx_irq, NULL, bd7757mwx_irq_function, IRQF_TRIGGER_LOW | IRQF_ONESHOT, FLASH_NAME, NULL)) {
			pr_err("%s: request_irq is failed.\n", __FUNCTION__);
			return -EIO;
		} else {
			DBG_LOG("%s: request_irq success!! gpio_%d\n", __FUNCTION__, gpio_num);
			led_data->irq_requested = true;
			enable_irq(bd7757mwx_irq);
			bd7757mwx_irq_enabled = true;
			DBG_LOG("%s: bd7757mwx_irq_enabled(%d), irq_requested(%d)\n", __FUNCTION__, bd7757mwx_irq_enabled, led_data->irq_requested);
		}
	} else {
		ret = gpio_direction_output(gpio_num, (flag & OF_GPIO_ACTIVE_LOW));
		if (unlikely(ret)) {
			pr_err("gpio_direction_output[%d] failed.\n", gpio_num);
			return -EIO;
		}
	}
#endif
	
	return 0;
}

static int __init led_hard_init(struct platform_device *pdev)
{
	struct bd7757mwx_data *led_data = platform_get_drvdata(pdev);
	struct device_node *node;
	enum of_gpio_flags flag;
	int idx;
	int ret;
	uint32_t *val_array = NULL;

	DBG_LOG("%s: start. \n", __FUNCTION__);
	node = of_find_compatible_node(NULL, NULL, FLASH_NAME);
	if (unlikely(!node)){
		pr_err("of_find_compatible_node(%s) fail\n", FLASH_NAME);
		return -ENODEV;
	}

	if(unlikely(!my_pinctrl_info.use_pinctrl)) {
		pr_err("pin ctrl is not initialized\n");
		return -ENODEV;
	}

	pinctrl_info_config(false); //sleep state

	val_array = kzalloc(sizeof(uint32_t) * CONTROL_MAX, GFP_KERNEL);
	if (!val_array) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}
	ret = of_property_read_u32_array(node, "qcom,gpio-enabled-flags", val_array, CONTROL_MAX);
	if (ret < 0) {
		pr_err("%s: of_property_read_u32_array failed %d\n", __func__, __LINE__);
		kfree(val_array);
		return -ENODEV;
	}

	for (idx = 0; idx < CONTROL_MAX; idx++) {
		port_info[idx].state = val_array[idx];
		DBG_LOG("%s: idx:%d, port_state:%d\n", __func__, idx, port_info[idx].state);
		if (port_info[idx].state == PORT_DISABLE) {
			DBG_LOG("%s: Skip for unused GPIO\n", __FUNCTION__);
			continue;
		}

		port_info[idx].gpio_num = of_get_gpio_flags(node, idx, &flag);
		CDBG("of_get_gpio_flags(%d) -> gpio_num(%d), flag(0x%04x)\n", idx ,port_info[idx].gpio_num, flag);
		if (unlikely(port_info[idx].gpio_num < 0)) {
			pr_err("of_get_gpio_flags() for gpio_control_%d failed.\n", idx);
			kfree(val_array);
			return -ENODEV;
		}

		ret = gpio_request(port_info[idx].gpio_num, port_info[idx].arg);
		if (unlikely(ret)) {
			pr_err("gpio_request[%d] failed.\n", port_info[idx].gpio_num);
			kfree(val_array);
			return -ENODEV;
		}
		
		ret = led_hard_init_config(led_data, port_info[idx].gpio_num, flag, idx);
		if (unlikely(ret)) {
			pr_err("%s: led_hard_init_config failed. ret(%d)\n", __FUNCTION__, ret);
			kfree(val_array);
			return ret;
		}
		
		port_info[idx].value = (flag & OF_GPIO_ACTIVE_LOW);
	}
	kfree(val_array);

	CDBG("%s: end.\n", __FUNCTION__);
	return 0;
}

static int bd7757mwx_suspend(struct device *dev)
{
	int ret = 0;
	
	led_power_state_off();
	DBG_LOG("%s: ret = %d.\n", __FUNCTION__, ret);
	return ret;
}

static int bd7757mwx_resume(struct device *dev)
{
	int ret = 0;
	DBG_LOG("%s: ret = %d.\n", __FUNCTION__, ret);
	return ret;
}

static const struct dev_pm_ops bd7757mwx_pm_ops = {
	.suspend	= bd7757mwx_suspend,
	.resume		= bd7757mwx_resume,
};

static int32_t bd7757mwx_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("%s:%d failed\n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	*subdev_id = fctrl->pdev->id;
	CDBG("%s:%d subdev_id %d\n", __FUNCTION__, __LINE__, *subdev_id);
	return 0;
}

static int32_t bd7757mwx_led_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
#if 0
	int led_current = 0;
#endif
	DBG_LOG("%s: called led_state %d\n", __FUNCTION__, cfg->cfgtype);
#if 0

	if (!fctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_OFF:
		// 1) Change to the Off mode
		upic_send_data(1, 1);
		break;

	case MSM_CAMERA_LED_LOW:
		// 1) Set torch current with cfg->torch_current[0]
		led_current = cfg->torch_current[0];
		if (led_current > 300) {
			led_current = 300;
		}
		else if (led_current < 20) {
			led_current = 20;
		}
		upic_send_data(3, CURRENT2TORCH_DATA(led_current));
		// 2) Change to the Torch mode
		upic_send_data(1, 2);
		break;

	case MSM_CAMERA_LED_HIGH:
		// TODO: Fire using Flash mode !!
		//       1) Set flash current with cfg->flash_current[0]
		//       2) Change to the Flash mode
		break;

	case MSM_CAMERA_LED_INIT:
		break;

	case MSM_CAMERA_LED_RELEASE:
		// TODO: should I do this HW reset sequence in release timming ?
		bd7757mwx_reset_device();
		break;

	default:
		rc = -EFAULT;
		break;
	}
	CDBG("%s: flash_set_led_state: return %d\n", __FUNCTION__, rc);
#endif
	return rc;
}

static int __init pinctrl_info_init(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;

	my_pinctrl_info.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(my_pinctrl_info.pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__FUNCTION__, __LINE__);
		return -EINVAL;
	}

	my_pinctrl_info.gpio_state_active =
		pinctrl_lookup_state(my_pinctrl_info.pinctrl,
				BD7757MWX_PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(my_pinctrl_info.gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__FUNCTION__, __LINE__);
		return -EINVAL;
	}

	my_pinctrl_info.gpio_state_suspend =
		pinctrl_lookup_state(my_pinctrl_info.pinctrl,
				BD7757MWX_PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(my_pinctrl_info.gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__FUNCTION__, __LINE__);
		return -EINVAL;
	}

	my_pinctrl_info.load_switch_gpio_state_active =
		pinctrl_lookup_state(my_pinctrl_info.pinctrl,
		        LOAD_SWITCH_PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(my_pinctrl_info.load_switch_gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__FUNCTION__, __LINE__);
		return -EINVAL;
	}

	my_pinctrl_info.load_switch_gpio_state_suspend =
		pinctrl_lookup_state(my_pinctrl_info.pinctrl,
				LOAD_SWITCH_PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(my_pinctrl_info.load_switch_gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
			__FUNCTION__, __LINE__);
		return -EINVAL;
	}

	my_pinctrl_info.use_pinctrl = false;
	my_pinctrl_info.use_pinctrl = of_property_read_bool(of_node, "qcom,enable_pinctrl");
	my_pinctrl_info.use_load_switch = false;
	my_pinctrl_info.use_load_switch = of_property_read_bool(of_node, "fj,ir_led_load_switch");

	return 0;
}

static const struct of_device_id bd7757mwx_dt_match[] = {
	{.compatible = FLASH_NAME},
	{}
};

MODULE_DEVICE_TABLE(of, bd7757mwx_dt_match);

static struct platform_driver bd7757mwx_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bd7757mwx_dt_match,
		.pm	= &bd7757mwx_pm_ops,
	},
};

static int32_t bd7757mwx_probe(struct platform_device *pdev)
{
	struct bd7757mwx_data *led_data;
	struct device_node *of_node = pdev->dev.of_node;

	int ret = 0;
	int i;

	CDBG("%s: start. [pdev=0x%p]\n", __FUNCTION__, pdev);

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (ret < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}
	CDBG("pdev id %d\n", pdev->id);

	g_led_timeout_time = 0;

	fctrl.pdev = pdev;
	fctrl.flash_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	fctrl.flash_num_sources = 0;
	fctrl.torch_num_sources = 0;

	ret = msm_led_flash_create_v4lsubdev(pdev, &fctrl);
	if (ret < 0) {
		pr_err("msm_led_flash_create_v4lsubdev failed\n");
		return -EINVAL;
	}

	led_data = kzalloc(sizeof(*led_data), GFP_KERNEL);
	if (unlikely(!led_data)) {
		dev_err(&pdev->dev, "failure kzalloc\n");
		pr_err("%s:end. ret = -ENOMEM.\n", __FUNCTION__);
		return -ENOMEM;
	}

	led_data->fctrl                 = &fctrl;
	led_data->cdev.name             = FLASH_NAME;
	led_data->cdev.brightness_set   = bd7757mwx_if_set;
	led_data->cdev.brightness       = LED_OFF;
	led_data->irq_requested         = false;
	ret = led_classdev_register(&pdev->dev, &led_data->cdev);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "%d: %s() led_classdev_register() error. [ret=%d]\n", __LINE__, __FUNCTION__, ret);
		kfree(led_data);
		return ret;
	}

	ret = bd7757mwx_if_sysfs_create_files_init(led_data->cdev.dev, leds_attr);
	if (unlikely(ret)) {
		led_classdev_unregister(&led_data->cdev);
		kfree(led_data);
		return ret;
	}

	platform_set_drvdata(pdev, led_data);

	ret = pinctrl_info_init(pdev);
	if (unlikely(ret)) {
		platform_set_drvdata(pdev, NULL);
		led_classdev_unregister(&led_data->cdev);
		kfree(led_data);
		return ret;
	}

	work_queue = create_singlethread_workqueue("bd7757mwx_queue");
	INIT_WORK(&led_on_work, bd7757mwx_on_work);
	sema_init(&led_sem, 0);
	spin_lock_init(&slock);

	for (i = 0; i < SEND_ADDR_MAX; i++)
		send_data_bk[i] = -1;

	ret = led_hard_init(pdev);
	if (unlikely(ret)) {
		platform_set_drvdata(pdev, NULL);
		led_classdev_unregister(&led_data->cdev);
		kfree(led_data);
		return ret;
	}

	CDBG("%s: end. ret = %d.\n", __FUNCTION__, ret);
	return ret;
}

static int __exit bd7757mwx_remove(struct platform_device *pdev)
{
	struct bd7757mwx_data *led_data = platform_get_drvdata(pdev);
	int ret = 0;
	int i;
	int idx;
	CDBG("%s: start.[pdev=0x%p]\n", __FUNCTION__, pdev);

	led_timeout_timer_del();

	bd7757mwx_if_sysfs_remove_files(led_data->cdev.dev, leds_attr);

	led_classdev_unregister(&led_data->cdev);

	DBG_LOG("%s: bd7757mwx_irq_enabled(%d), irq_requested(%d)\n", __FUNCTION__, bd7757mwx_irq_enabled, led_data->irq_requested);
	if (bd7757mwx_irq_enabled) {
		disable_irq(bd7757mwx_irq);
		bd7757mwx_irq_enabled = false;
	}

	if (led_data->irq_requested) {
		free_irq(bd7757mwx_irq, NULL);
		led_data->irq_requested = false;
		bd7757mwx_irq_enabled = false;
	}

	if (g_current_hard_state == P_ON) {
		led_power_state_off();
	}

	kfree(led_data);

	for (idx = 0; idx < CONTROL_MAX; idx++) {
		if (port_info[idx].gpio_num != -1) {
			gpio_free(port_info[idx].gpio_num);
		}
	}

	for (i = 0; i < SEND_ADDR_MAX; i++) {
		send_data_bk[i] = -1;
	}

	CDBG("%s: end. ret = %d.\n", __FUNCTION__, ret);
	return ret;
}

static int __init bd7757mwx_init(void)
{
	CDBG("called\n");
	return platform_driver_probe(&bd7757mwx_driver,
		bd7757mwx_probe);
}

static struct msm_flash_fn_t bd7757mwx_func_tbl = {
	.flash_get_subdev_id    = bd7757mwx_get_subdev_id,
	.flash_led_config       = bd7757mwx_led_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.func_tbl = &bd7757mwx_func_tbl,
};

module_init(bd7757mwx_init);
MODULE_DESCRIPTION("BD7757MWX FLASH DRIVER");
MODULE_LICENSE("GPL v2");
