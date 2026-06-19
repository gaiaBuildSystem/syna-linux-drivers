# SPDX-License-Identifier: GPL-2.0
#

# FB driver for SYNA
obj-$(CONFIG_SYNAPTICS_VIDEO) 		+= syna/
obj-y								+= backlight/
obj-y								+= fbdev/

ifdef CONFIG_SYNAPTICS_VIDEO
$(error Remember we disable: source "drivers/synaptics/gpu/arm/mali/Kconfig")
$(error Remember we disable: source "drivers/synaptics/gpu/vivante/Kconfig")
endif
