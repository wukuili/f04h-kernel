/* H-FAUDIO-000015000 start */
#ifndef	__UAPI_LINUX_WAVEIF_H__
#define	__UAPI_LINUX_WAVEIF_H__

#include <linux/ioctl.h>
#define	WAVEIF_CANCEL_READ	_IOW('w', 0x06, int)	// no parameter
#define	WAVEIF_LOAD_FIRM	_IOW('w', 0x07, int)	// 8, 16 in kHz resolution
#define	WAVEIF_SET_VOIPFIRM	_IOW('w', 0x08, int)	/* H-FAUDIO-000057000 */

#endif	//	__UAPI_LINUX_WAVEIF_H__
/* H-FAUDIO-000015000 end */
