#
# Makefile for the kernel security code
#

obj-$(CONFIG_KEYS)			+= keys/
subdir-$(CONFIG_SECURITY_SELINUX)	+= selinux
subdir-$(CONFIG_SECURITY_SMACK)		+= smack
subdir-$(CONFIG_SECURITY_TOMOYO)        += tomoyo
subdir-$(CONFIG_SECURITY_APPARMOR)	+= apparmor
subdir-$(CONFIG_SECURITY_YAMA)		+= yama

# always enable default capabilities
obj-y					+= commoncap.o
obj-$(CONFIG_MMU)			+= min_addr.o

# Object file lists
# FUJITSU:2014-12-08 GEN-141-SCRU-ALT ADD S
#obj-$(CONFIG_SECURITY)			+= security.o capability.o
obj-$(CONFIG_SECURITY)			+= security.o
ifneq ($(CONFIG_SECURITY_FJSEC),y)
  obj-$(CONFIG_SECURITY)		+= capability.o
endif

# FUJITSU:2014-12-08 GEN-141-SCRU-ALT ADD E

obj-$(CONFIG_SECURITYFS)		+= inode.o
# Must precede capability.o in order to stack properly.
obj-$(CONFIG_SECURITY_SELINUX)		+= selinux/built-in.o
obj-$(CONFIG_SECURITY_SMACK)		+= smack/built-in.o
obj-$(CONFIG_AUDIT)			+= lsm_audit.o
obj-$(CONFIG_SECURITY_TOMOYO)		+= tomoyo/built-in.o
obj-$(CONFIG_SECURITY_APPARMOR)		+= apparmor/built-in.o
obj-$(CONFIG_SECURITY_YAMA)		+= yama/built-in.o
obj-$(CONFIG_CGROUP_DEVICE)		+= device_cgroup.o
# FUJITSU:2014-12-08 GEN-141-SCRU-ALT ADD S
obj-$(CONFIG_SECURITY_FJSEC)		+= fjsec.o
obj-$(CONFIG_DETECT_RH)		+= detect_rh.o
# FUJITSU:2014-12-08 GEN-141-SCRU-ALT ADD E

# Object integrity file lists
subdir-$(CONFIG_INTEGRITY)		+= integrity
obj-$(CONFIG_INTEGRITY)			+= integrity/built-in.o
