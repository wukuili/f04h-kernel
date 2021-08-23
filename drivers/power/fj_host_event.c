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
/* FCNT LIMITED:2016-02-03 H16100722 add start */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/fj_bto.h>
#include "fj_host_event.h"

enum {
	USB_HOST_STATE_NONE = 0,
	USB_HOST_STATE_DETECT,

	USB_HOST_STATE_MAX
};

static struct switch_dev usb_host_dev;

int fj_host_send_uevent(enum usb_host_ext_event event)
{
	int ret = 0;

/* FCNT LIMITED:2016-03-16 H16100722-1 mod start */
	if (bto_info[BTO_INFO_ID_USB_HOST] == BTO_INFO_DISABLE) {
		return ret;
	}
/* FCNT LIMITED:2016-03-16 H16100722-1 mod end */

	switch (event) {
	case USB_HOST_EXT_EVENT_NONE:
		printk("SMBCHG: fj_host_send_uevent none\n");
		switch_set_state(&usb_host_dev, USB_HOST_STATE_NONE);
		break;
	case USB_HOST_EXT_EVENT_DETECT:
		printk("SMBCHG: fj_host_send_uevent detect\n");
		switch_set_state(&usb_host_dev, USB_HOST_STATE_DETECT);
		break;
	default:
		printk("SMBCHG: fj_host_send_uevent error\n");
		ret = -1;
		break;
	}

	return ret;
}

static ssize_t usb_host_uevent_print_name(struct switch_dev *sdev_t, char *buf)
{
	switch (switch_get_state(&usb_host_dev)) {
	case USB_HOST_STATE_NONE:
		return snprintf(buf, PAGE_SIZE, "None\n");
	case USB_HOST_STATE_DETECT:
		return snprintf(buf, PAGE_SIZE, "Detect\n");
	}
	return -EINVAL;
}

int host_ext_event_driver_register(void)
{
	int ret;

	usb_host_dev.name	= "usb_fj_otg";
	usb_host_dev.print_name = usb_host_uevent_print_name;
	ret = switch_dev_register(&usb_host_dev);
	if (ret)
		printk(KERN_ERR "SMBCHG: usb_host_ext_event register error\n");
	else
		printk("SMBCHG: usb_host_ext_event has been registered!\n");

	return ret;
}

void host_ext_event_driver_unregister(void)
{
	switch_dev_unregister(&usb_host_dev);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Extra event notifier of USB host");
/* FCNT LIMITED:2016-02-03 H16100722 add end */
