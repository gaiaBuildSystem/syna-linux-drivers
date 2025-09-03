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

static fastlogo_info_t* check_validate_logo(int width,
			int height,
			unsigned char* pHEADER,
			int logosize1to1fit)
{
	fastlogo_header_t *fl_header_info = (fastlogo_header_t*)pHEADER;
	int i;
	int validlogocount = (fl_header_info->logoNum > MAX_LOGO_NUM) ? MAX_LOGO_NUM :
						fl_header_info->logoNum;

	for (i = 0; i < validlogocount; i++) {
		pr_info("number of logo %d w[%d] H[%d] O[%d] st[%d]\n",
			fl_header_info->logoNum,
			fl_header_info->info[i].width,
			fl_header_info->info[i].height,
			fl_header_info->info[i].offset,
			fl_header_info->info[i].stride);

		if (logosize1to1fit) {
			if ((fl_header_info->info[i].width == width) &&
				(fl_header_info->info[i].height == height)) {
				return &fl_header_info->info[i];
			}
		} else {
			if((fl_header_info->info[i].width > 0) &&
				(fl_header_info->info[i].width <= width) &&
				(fl_header_info->info[i].height > 0) &&
				(fl_header_info->info[i].height <= height) )
					return &fl_header_info->info[i];
		}
	}

	return NULL;
}

int syna_vpp_read_logo_from_emmc_device(struct drm_device *dev,
						int width,
						int height,
						void* plogobuf)
{
	ssize_t bytes_read;
	loff_t pos;
	struct file *filep;
	char plogopath[32];
	avio_fastlogo_info display_info;
	fastlogo_info_t *fl_info;
	int logo_partition;
	struct syna_drm_private *dev_priv = dev->dev_private;

	display_info = avio_get_fastlogo_status();

	if (display_info.u.status)
		logo_partition = display_info.u.partition;
	else
		logo_partition = dev_priv->vpp_config_param.logo_parition;

	sprintf(plogopath, "%s%d", FASTLOGO_FILE, logo_partition);
	filep = filp_open(plogopath, O_RDWR, 0);
	if (IS_ERR(filep)) {
		printk("Failed to open eMMC part %d Error=%ld \n", logo_partition, PTR_ERR(filep));
		return -1;
	}

	pos = FASTLOGO_PREPEND_GENX_HEADER;
	bytes_read = kernel_read(filep, plogobuf, FASTLOGO_HEADER, &pos);
	if (bytes_read < 0) {
		pr_info("Failed to read fl Header\n");
		return -1;
	}

	/* Provide the resolution info */
	fl_info = check_validate_logo(width, height, plogobuf, !VPP_SUPPORT_SCALAR);
	if (fl_info) {
		pos = FASTLOGO_PREPEND_GENX_HEADER + fl_info->offset;
		bytes_read = kernel_read(filep, plogobuf, (fl_info->stride * fl_info->height),
							&pos);
		if (bytes_read < 0) {
			pr_info("Failed to read logo\n");
			return -1;
		}
	}

	return 0;
}