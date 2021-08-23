##############################################################################
# Makefile for TUNER Driver
##############################################################################

#driver name
DRV_NAME = mm_tuner

#FUJITSU:2015-05-23 For Socionext tuner Modify -S
  subdir-ccflags-y += -fno-pic

OBJS := tuner_drv.o
OBJS += tuner_drv_hw.o

  OBJS += tuner_drv_hw_i2c.o
  subdir-ccflags-y += -DCPATH_I2C

$(DRV_NAME)-objs += $(OBJS)

obj-y = $(DRV_NAME).o

#FUJITSU:2015-05-23 For Socionext tuner Modify -E
