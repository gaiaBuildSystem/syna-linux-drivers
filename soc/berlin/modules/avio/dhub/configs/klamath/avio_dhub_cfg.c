// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2025 Synaptics Incorporated */

#include <linux/module.h>
#include <linux/kernel.h>

#include "avio_dhub_cfg.h"
#include "avio_dhub_cfg_prv.h"
#include "hal_dhub.h"
#include "hal_dhub_wrap.h"
#include "avio_dhub_drv.h"
#include "tee_ca_dhub.h"

#define CPUINDEX    0

static HDL_dhub2d VPP_dhubHandle;

/* Total size 14KB */
#define AVIO_VPPDHUB_LCDC2Y_SIZE  (6912)
#define AVIO_VPPDHUB_LCDC2C_SIZE  (6912)
#define AVIO_VPPDHUB_BCM_SIZE     (512)

#define AVIO_VPPDHUB_LCDC2Y_BASE  (VPP_DHUB_BANK0_START_ADDR)
#define AVIO_VPPDHUB_LCDC2C_BASE  (AVIO_VPPDHUB_LCDC2Y_BASE + AVIO_VPPDHUB_LCDC2Y_SIZE)
#define AVIO_VPPDHUB_BCM_BASE     (AVIO_VPPDHUB_LCDC2C_BASE + AVIO_VPPDHUB_LCDC2C_SIZE)

static DHUB_channel_config  LCDC_config[] = {
	{avioDhubChMap_vpp128b_LCDC2_Y_R, AVIO_VPPDHUB_LCDC2Y_BASE, AVIO_VPPDHUB_LCDC2Y_BASE+64, 64, \
		(AVIO_VPPDHUB_LCDC2Y_SIZE-64), dHubChannel_CFG_MTU_256byte, 1, 0, 1, 0xF, 0xF}, \
	{avioDhubChMap_vpp128b_LCDC2_C_R, AVIO_VPPDHUB_LCDC2C_BASE, AVIO_VPPDHUB_LCDC2C_BASE+64, 64, \
		(AVIO_VPPDHUB_LCDC2C_SIZE-64), dHubChannel_CFG_MTU_256byte, 1, 0, 1, 0xF, 0xF}, \
	{avioDhubChMap_vpp128b_BCM_R,     AVIO_VPPDHUB_BCM_BASE,    AVIO_VPPDHUB_BCM_BASE+128,  128, \
		(AVIO_VPPDHUB_BCM_SIZE-128),   dHubChannel_CFG_MTU_256byte, 0, 0, 1, 0xF, 0xF}, \
};

int drv_dhub_initialize_dhub(void *h_dhub_ctx)
{
	static atomic_t dhub_init_done = ATOMIC_INIT(0);
	DHUB_CTX *hDhubCtx = (DHUB_CTX *)h_dhub_ctx;
	avio_fastlogo_info display_info;
	unsigned int channel_init_mask;

	//Allow DHUB initialization only once
	if (atomic_cmpxchg(&dhub_init_done, 0, 1))
		return 0;

	display_info = avio_get_fastlogo_status();

	/* Disable Autopush before initialization of VPP DHUB */
	if (!display_info.u.status)
		wrap_DhubEnableAutoPush(false, true, hDhubCtx->fastlogo_framerate);

	channel_init_mask = display_info.u.status ? 0 : (1 << VPP_NUM_OF_CHANNELS) - 1 ;

	DhubInitialization(DHUB_ID_VPP_DHUB, DHUB_TYPE_128BIT,
				CPUINDEX, hDhubCtx->vpp_dhub_base,
				hDhubCtx->vpp_sram_base, &VPP_dhubHandle,
				LCDC_config, VPP_NUM_OF_CHANNELS,
				DHUB_TYPE_128BIT, hDhubCtx->vpp_bcm_base, 0, channel_init_mask);

	return 0;
}

void drv_dhub_config_ctx(void *h_dhub_ctx, UNSG32 avio_base)
{
	DHUB_CTX *hDhubCtx = (DHUB_CTX *)h_dhub_ctx;

	hDhubCtx->vpp_bcm_base   = avio_base + AVIO_MEMMAP_VPP_BCMQ_REG_BASE;

	hDhubCtx->vpp_dhub_base  = avio_base + RA_vpp128bDhub_dHub0;

	hDhubCtx->vpp_sram_base  = avio_base + RA_vpp128bDhub_tcm0;

	hDhubCtx->avio_gbl_base  = avio_base + AVIO_MEMMAP_VPP_GBL_REG_BASE;
}
