#
# Makefile for the kernel mmc core.
#

obj-$(CONFIG_MMC)		+= mmc_core.o
mmc_core-y			:= core.o bus.o host.o \
				   mmc.o mmc_ops.o sd.o sd_ops.o \
				   sdio.o sdio_ops.o sdio_bus.o \
				   sdio_cis.o sdio_io.o sdio_irq.o \
				   quirks.o slot-gpio.o

mmc_core-$(CONFIG_DEBUG_FS)	+= debugfs.o
# FUJITSU LIMITED:2015-05-14 H1520355 add start
mmc_core-$(CONFIG_MMC_PASSWORDS) += lock.o
# FUJITSU LIMITED:2015-05-14 H1520355 add end
# FCNT LIMITED:2016-03-07 H16100340 add start
mmc_core-y += cmdlog.o
# FCNT LIMITED:2016-03-07 H16100340 add end