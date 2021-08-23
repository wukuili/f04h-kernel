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
#ifndef __FJ_HOST_EVENT_H__
#define __FJ_HOST_EVENT_H__

enum usb_host_ext_event {
	USB_HOST_EXT_EVENT_NONE = 0,
	USB_HOST_EXT_EVENT_DETECT,
};

int fj_host_send_uevent(enum usb_host_ext_event event);
int host_ext_event_driver_register(void);
void host_ext_event_driver_unregister(void);

#endif /* __FJ_HOST_EVENT_H__ */
/* FCNT LIMITED:2016-02-03 H16100722 add end */
