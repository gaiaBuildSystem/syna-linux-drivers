ccflags-y += -I$(srctree)/drivers/synaptics/include
ccflags-y += -I$(srctree)/drivers/synaptics/npu/src
ccflags-y += -I$(srctree)/drivers/synaptics/soc/berlin/modules/tzd/tee/common
ccflags-y += -I$(srctree)/drivers/synaptics/soc/berlin/modules/tzd/tee/client_api

ifeq ($(CONFIG_OPTEE), y)
    obj-$(CONFIG_SYNAPTICS_VIS_NPU)		+= synap_kernel_ca_optee.o
else
    obj-$(CONFIG_SYNAPTICS_VIS_NPU)		+= synap_kernel_ca_tzk.o
endif

obj-$(CONFIG_SYNAPTICS_VIS_NPU)		+= synap_kernel_drv.o
obj-$(CONFIG_SYNAPTICS_VIS_NPU)		+= synap_kernel_stats.o
obj-$(CONFIG_SYNAPTICS_VIS_NPU)		+= synap_kernel.o
obj-$(CONFIG_SYNAPTICS_VIS_NPU)		+= synap_mem.o
obj-$(CONFIG_SYNAPTICS_VIS_NPU)		+= synap_profile.o
obj-$(CONFIG_SYNAPTICS_VIS_NPU)		+= ebg_file.o

EXTRA_CFLAGS += -I $(srctree)/include/linux/
