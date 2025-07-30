obj-$(CONFIG_SYNA_ISP) += csi_subdev.o

ISP_ABS_PATH := $(srctree)/drivers/synaptics/soc/berlin/modules/isp

#ifeq ($(CONFIG_ISP_DOLPHIN),y)
#	ccflags-y += -D DOLPHIN
#endif

HEADER_PATH += -I$(ISP_ABS_PATH) \
		-I$(ISP_ABS_PATH)/fw \

ccflags-y += $(HEADER_PATH)

ccflags-y += -DLINUX
csi_subdev-objs += csi/dw_mipi_csi.o \
	csi/snps_dphy_gen3.o \
	csi/snps_dphy_csi2.o \
	csi/snps_dphy_wrap.o
