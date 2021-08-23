#
# Makefile for the linux pstorefs routines.
#

# /*----------------------------------------------------------------------------*/
# // COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2016
# /*----------------------------------------------------------------------------*/

obj-y += pstore.o

pstore-objs += inode.o platform.o
obj-$(CONFIG_PSTORE_FTRACE)	+= ftrace.o

obj-$(CONFIG_PSTORE_PMSG)	+= pmsg.o

# /* FUJITSU:2016-03-04 LogImprovement ADD-S */
obj-$(CONFIG_PSTORE_SUMMARY)	+= summary.o
# /* FUJITSU:2016-03-04 LogImprovement ADD-S */

ramoops-objs += ram.o ram_core.o
obj-$(CONFIG_PSTORE_RAM)	+= ramoops.o
