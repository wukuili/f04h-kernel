#----------------------------------------------------------------------------
# COPYRIGHT(C) FUJITSU LIMITED 2014-2016
#----------------------------------------------------------------------------
#
# Makefile for the Silicon Image 8620 MHL TX device driver
#
# example invocations:	
#	For regular Linux builds
#	make ARCH=arm CROSS_COMPILE=arm-angstrom-linux-gnueabi- clean debug
#	make ARCH=arm CROSS_COMPILE=arm-angstrom-linux-gnueabi- clean release
#	make ARCH=arm CROSS_COMPILE=arm-angstrom-linux-gnueabi- clean clean
#
#	For Android driver builds - Specify different tool-chain and kernel revision
#	export PATH=~/rowboat-android/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
#	make ARCH=arm KERNELPATH=~/rowboat-android/kernel CROSS_COMPILE=arm-eabi- clean debug
#	make ARCH=arm KERNELPATH=~/rowboat-android/kernel CROSS_COMPILE=arm-eabi- clean release
#	make ARCH=arm KERNELPATH=~/rowboat-android/kernel CROSS_COMPILE=arm-eabi- clean clean
#	
#	For Android 4.0.3 (ICS):
#	export PATH=~/rowboat-android/TI-Android-ICS-4.0.3_AM37x_3.0.0/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
#	make ARCH=arm KERNELPATH=~/rowboat-android/TI-Android-ICS-4.0.3_AM37x_3.0.0/kernel CROSS_COMPILE=arm-eabi- clean debug
#	make ARCH=arm KERNELPATH=~/rowboat-android/TI-Android-ICS-4.0.3_AM37x_3.0.0/kernel CROSS_COMPILE=arm-eabi- clean release
#	make ARCH=arm KERNELPATH=~/rowboat-android/TI-Android-ICS-4.0.3_AM37x_3.0.0/kernel CROSS_COMPILE=arm-eabi- clean clean
#
#	For Android 4.2.2 (JB) Linaro release 13.04 for PandaBoard:
#	export PATH=$PATH:~/Linaro-13.04/android/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7-linaro/bin
#	make ARCH=arm KERNELPATH=~/Linaro-13.04/android/kernel/linaro/pandaboard CROSS_COMPILE=arm-eabi- clean debug
#	make ARCH=arm KERNELPATH=~/Linaro-13.04/android/kernel/linaro/pandaboard CROSS_COMPILE=arm-eabi- clean release
#	make ARCH=arm KERNELPATH=~/Linaro-13.04/android/kernel/linaro/pandaboard CROSS_COMPILE=arm-eabi- clean clean
#	

ifeq ($(MHL_PRODUCT_NUM),)
MHL_PRODUCT_NUM=8620
endif

ifeq ($(MHL_BUILD_NUM),)
MHL_BUILD_NUM=0
endif
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add start
# Linux version 3.10.49
LINUX_KERNEL_VER=310
DT_SUPPORT=1
CONFIG_DEBUG_DRIVER=y
INCLUDE_SII6031=1
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 add end

# Silicon Image uses DEVELOPER_BUILD_ID for sandbox build to be identified during testing.
ifneq ($(DEVELOPER_BUILD_ID),)
DEVELOPER_BUILD_COPY=cp sii$(MHL_PRODUCT_NUM)drv.ko sii$(MHL_PRODUCT_NUM)drv$(DEVELOPER_BUILD_ID).ko
endif

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile

#
# color annotations to use instead of leading newline chars
ccflags-y += -DANSI_COLORS

# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0011 mod start
#ccflags-y += -DBUILD_NUM_STRING=\"$(MHL_BUILD_NUM)$(DEVELOPER_BUILD_ID)\"
ccflags-y += -DBUILD_NUM_STRING=\"1.03.24\"
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0011 mod end
ccflags-y += -DMHL_PRODUCT_NUM=$(MHL_PRODUCT_NUM)
ccflags-y += -DMHL_DRIVER_NAME=\"sii$(MHL_PRODUCT_NUM)drv\"
ccflags-y += -DMHL_DEVICE_NAME=\"sii-$(MHL_PRODUCT_NUM)\"

# Support Device Tree?
ifeq ($(DT_SUPPORT),1)
ccflags-y += -DSIMG_USE_DTS
endif

# Kernel level supported
ccflags-y += -DLINUX_KERNEL_VER=$(LINUX_KERNEL_VER)
#
# FORCE_OCBUS_FOR_ECTS is used to identify code added for ECTS temporary fix that prohibits use of eCBUS.
# in addition module parameter force_ocbus_for_ects needs to be set as 1 to achieve ECTS operation.
ccflags-y += -DFORCE_OCBUS_FOR_ECTS
#
# PC_MODE_VIDEO_TIMING_SUPPORT is for cases where no VIC is available from either AVIF or VSIF.
ccflags-y += -DPC_MODE_VIDEO_TIMING_SUPPORT
#
# MANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH is to clear all infoframes 
#	upon driving HPD high instead of when SCDT goes low.
ccflags-y += -DMANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH
#
# MEDIA_DATA_TUNNEL_SUPPORT
#	Default is enabled. Comment next line to disable.
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 del start
#ccflags-y += -DMEDIA_DATA_TUNNEL_SUPPORT
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 del end
#
# Include REMOTE BUTTON PROTOCOL code or not
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 mod start
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DINCLUDE_RBP=1
#
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DINCLUDE_HID=$(INCLUDE_HID)
ccflags-$(CONFIG_DEBUG_DRIVER) += -DINCLUDE_RBP=0
ccflags-$(CONFIG_DEBUG_DRIVER) += -DINCLUDE_HID=0
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 mod end

# Example of use of SiI6031 is wrapped under the following definition
# It also illustrates how 8620 driver may be integrated into MSM platform
# the flag should be disabled if not building for MSM
ccflags-$(CONFIG_DEBUG_DRIVER) += -DINCLUDE_SII6031=$(INCLUDE_SII6031)
#
# MANUAL_EDID_FETCH uses DDC master directly, instead of h/w automated method.
ccflags-y += -DMANUAL_EDID_FETCH

# If CI2CA pin is pulled HIGH, you must define the following flag
#ccflags-y += -DALT_I2C_ADDR

# PRINT_DDC_ABORTS enables logging of all DDC_ABORTs. Default "disabled" - helps MHL2 hot plug.
# Enable only if you must for debugging. 
#ccflags-y += -DPRINT_DDC_ABORTS

# CoC_FSM_MONITORING exports CoC state machine to GPIO pins
# Enable only if you must for debugging. 
#ccflags-y += -DCoC_FSM_MONITORING
#ccflags-y += -DGPIO_OSCILLOSCOPE_DEBUG

# Enable only if you must for debugging. 
#ccflags-y += -DBIST_MONITORING

#BIST_DONE_DEBUG adds register dump prior to RAP{CBUS_MODE_UP}
#ccflags-y += -DBIST_DONE_DEBUG

#USE_MHL_HPD_CALLBACK_API initiates setup for callback from host HDMI driver
# Enable or disable the following two lines as a group.
ccflags-y += -DUSE_MHL_HPD_CALLBACK_API
ccflags-y += -I$(KERNELPATH)/source/drivers/video/msm/mdss

# For si_emsc_hid-mt.c
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 del start
#ccflags-y += -Idrivers/hid
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 del end

# Optimzations and/or workaround
ccflags-y += -DDISABLE_SPI_DMA
ccflags-y += -DUSE_SPIOPTIMIZE
ccflags-y += -DGCS_QUIRKS_FOR_SIMG

ccflags-$(CONFIG_DEBUG_DRIVER) += -DDEBUG 

# Enable VBUS sense and related operations
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 del start
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_VBUS_SENSE
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 del end

#support for DVI sources and sinks in MHL3 mode.
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 del start
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DMHL3_DVI_SUPPORT
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 del end

#add HDMI VSDB to upstream EDID when downstream sink is DVI
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DMHL3_DVI_SUPPORT_FORCE_HDMI
#
# the next lines are optional - they enable greater verbosity in debug output
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 mod start
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_EDID_DEBUG_PRINT
ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_EDID_DEBUG_PRINT
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 mod end
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_DUMP_INFOFRAME
#
# uncomment next line to prevent EDID parsing. Useful for debugging.
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DEDID_PASSTHROUGH

obj-$(CONFIG_SII$(MHL_PRODUCT_NUM)_MHL_TX) += sii$(MHL_PRODUCT_NUM)drv.o
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 mod start
#sii$(MHL_PRODUCT_NUM)drv-objs  += platform.o
#sii$(MHL_PRODUCT_NUM)drv-objs  += mhl_linux_tx.o
#sii$(MHL_PRODUCT_NUM)drv-objs  += mhl_rcp_inputdev.o
#sii$(MHL_PRODUCT_NUM)drv-objs  += mhl_rbp_inputdev.o
#sii$(MHL_PRODUCT_NUM)drv-objs  += mhl_supp.o
#sii$(MHL_PRODUCT_NUM)drv-objs  += si_8620_drv.o
#sii$(MHL_PRODUCT_NUM)drv-objs  += si_mhl2_edid_3d.o
#sii$(MHL_PRODUCT_NUM)drv-objs  += si_mdt_inputdev.o
obj-y += platform.o
obj-y += mhl_linux_tx.o
obj-y += mhl_rcp_inputdev.o
obj-y += mhl_rbp_inputdev.o
obj-y += mhl_supp.o
obj-y += si_8620_drv.o
obj-y += si_mhl2_edid_3d.o
obj-y += si_mdt_inputdev.o
obj-y  += fj_mhl_platform.o
# FUJITSU LIMITED:2016-01-07 H161_5_MHL_01_0001 mod end

ifeq ($(INCLUDE_HID),1)
sii$(MHL_PRODUCT_NUM)drv-objs  += si_emsc_hid.o
sii$(MHL_PRODUCT_NUM)drv-objs  += si_emsc_hid-mt.o
endif
else

# Normal Makefile

# If a kernel is not specified, default to the kernel used with Android Ice Cream Sandwich
ifneq ($(KERNELPATH),)
KERNELDIR=$(KERNELPATH)
else
KERNELDIR=~/src/linux-2.6.36
#KERNELDIR=~/src/Android_ICS/kernel
endif
ARCH=arm

PWD := $(shell pwd)

.PHONY: clean

release:
	make -C $(KERNELDIR) M=$(PWD) CONFIG_SII$(MHL_PRODUCT_NUM)_MHL_TX=m CONFIG_MEDIA_DATA_TUNNEL_SUPPORT=y modules
	$(CROSS_COMPILE)strip --strip-debug sii$(MHL_PRODUCT_NUM)drv.ko
	$(DEVELOPER_BUILD_COPY)

debug:
	rm -f platform.o
	make -C $(KERNELDIR) M=$(PWD) CONFIG_SII$(MHL_PRODUCT_NUM)_MHL_TX=m CONFIG_MEDIA_DATA_TUNNEL_SUPPORT=y CONFIG_DEBUG_DRIVER=y modules
	$(DEVELOPER_BUILD_COPY)

clean:
	make -C $(KERNELDIR) M=$(PWD) CONFIG_SII$(MHL_PRODUCT_NUM)_MHL_TX=m clean
	
endif
