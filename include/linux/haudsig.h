/* H-FAUDIO-000015000 start */
#ifndef HAUDSIG_H
#define HAUDSIG_H

#include <linux/ioctl.h>
#define	HAUDSIG_IOCTL_SET_EOF	_IOW('w', 0x01, int)	// no parameter

void haudsig_wakeup(const char* data);

#endif//HAUDSIG_H
/* H-FAUDIO-000015000 end */
