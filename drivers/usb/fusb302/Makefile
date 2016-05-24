#
# Makefile for FAIRCHILD_FUSB302 drivers.
#
#obj-$(CONFIG_USB_FAIRCHILD_FUSB302)	+= fusb30x_driver.o fusb30x_global.o platform_helpers.o hostcomm.o platform.o

# Required flag to configure the core to operate with the Linux kernel
ccflags-y += -DPLATFORM_LINUX
# Optional flag to enable interrupt-driven operation
ccflags-y += -DFSC_INTERRUPT_TRIGGERED
obj-y   += fusb30x_whole.o
fusb30x_whole-objs := fusb30x_driver.o \
		      fusb30x_global.o \
		      platform.o \
		      platform_helpers.o \
		      hostcomm.o \
		      core/AlternateModes.o \
		      core/core.o \
		      core/fusb30X.o \
		      core/Log.o \
		      core/PDPolicy.o \
		      core/PDProtocol.o \
		      core/TypeC.o \
		      core/vdm/bitfield_translators.o \
		      core/vdm/vdm.o \
		      core/vdm/vdm_callbacks.o \
		      core/vdm/vdm_config.o \
		      core/vdm/DisplayPort/configure.o \
		      core/vdm/DisplayPort/dp.o \
		      core/vdm/DisplayPort/dp_system_stubs.o
