/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * DRM fbdev emulation helpers for Synaptics DRM driver
 * Custom implementation to handle VPP memory management
 */

#ifndef __DRM_SYNA_FBDEV_H__
#define __DRM_SYNA_FBDEV_H__

struct drm_device;

#ifdef CONFIG_DRM_FBDEV_EMULATION
int syna_fbdev_init(struct drm_device *dev);
void syna_fbdev_cleanup(struct drm_device *dev);
#else
static inline int syna_fbdev_init(struct drm_device *dev)
{
	return 0;
}
static inline void syna_fbdev_cleanup(struct drm_device *dev)
{
}
#endif

#endif /* __DRM_SYNA_FBDEV_H__ */
