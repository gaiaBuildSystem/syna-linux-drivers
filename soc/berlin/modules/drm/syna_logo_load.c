// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Synaptics Incorporated
 *
 */

#include <linux/fs.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>

#include "drm_syna_drv.h"
#include "avio_core.h"
#include "vpp_mem.h"
#include "syna_vpp.h"
#include "vpp_api.h"
#include "syna_fl_info.h"
#include "syna_drm_priv.h"

#define MAX_LOGO_NUM	10

int syna_vpp_read_logo_from_emmc_device(struct drm_device *dev,
						int width,
						int height,
						VPP_WIN *vpp_res_info,
						void* plogobuf)
{
	//TBD: Implement the fix proposed in Jira#VSSDK-36120
        return -1;
}