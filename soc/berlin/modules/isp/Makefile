obj-$(CONFIG_SYNA_ISP) += csi_subdev.o
obj-$(CONFIG_SYNA_ISP) += camera_video.o

ISP_ABS_PATH := $(srctree)/drivers/synaptics/soc/berlin/modules/isp

HEADER_PATH += -I$(ISP_ABS_PATH) \
		-I$(ISP_ABS_PATH)/fw \
		-I$(ISP_ABS_PATH)/video \

ccflags-y += $(HEADER_PATH)

ccflags-y += -DLINUX
csi_subdev-objs += csi/dw_mipi_csi.o \
	csi/snps_dphy_gen3.o \
	csi/snps_dphy_csi2.o \
	csi/snps_dphy_wrap.o

camera_video-objs += video/camera_video_register.o \
	video/camera_video_driver.o
