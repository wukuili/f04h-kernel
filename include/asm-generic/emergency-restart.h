/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2015
/*----------------------------------------------------------------------------*/
#ifndef _ASM_GENERIC_EMERGENCY_RESTART_H
#define _ASM_GENERIC_EMERGENCY_RESTART_H

static inline void machine_emergency_restart(void)
{
/* FUJITSU LIMITED:2015-11-12 H1610001 mod start */
/*	machine_restart(NULL);*/
	machine_restart("WARM");
/* FUJITSU LIMITED:2015-11-12 H1610001 mod end */
}

#endif /* _ASM_GENERIC_EMERGENCY_RESTART_H */
