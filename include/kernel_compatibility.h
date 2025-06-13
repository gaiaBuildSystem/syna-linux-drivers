// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2025 Synaptics Incorporated
   *
   * Author: Long Xiao <Long.Xiao@synaptics.com>
   *
   */
#ifndef __KERNEL_COMPATIBILITY_H__
#define __KERNEL_COMPATIBILITY_H__

#include <linux/version.h>
#include <linux/compiler.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0))
#include <linux/dma-buf-map.h>
#define iosys_map dma_buf_map
#define iosys_map_set_vaddr dma_buf_map_set_vaddr
#define iosys_map_clear dma_buf_map_clear
#else
#include <linux/iosys-map.h>
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0))
typedef u32 FD_FLAGS_TYPE;
typedef u64 HEAP_FLAGS_TYPE;
#else
typedef unsigned long FD_FLAGS_TYPE;
typedef unsigned long HEAP_FLAGS_TYPE;
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0))
typedef void RET_TYPE;
#define RETURN_VALUE
#else
typedef int RET_TYPE;
#define RETURN_VALUE return 0
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0))
#define SYNA_CLASS_CREATE(dev_name) class_create(dev_name);
#else
#define SYNA_CLASS_CREATE(dev_name) class_create(THIS_MODULE, dev_name);
#endif

#endif

