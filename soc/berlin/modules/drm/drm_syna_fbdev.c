// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM fbdev emulation helpers for Synaptics DRM driver
 * Custom implementation to handle VPP memory management
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <asm/pgtable.h>

#include <drm/drm_crtc.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mode.h>

#include "drm_syna_gem.h"
#include "drm_syna_fbdev.h"

static atomic_t syna_fbdev_open_cnt = ATOMIC_INIT(0);

static int syna_fbdev_open(struct fb_info *info, int user)
{
	int open_count;

	open_count = atomic_inc_return(&syna_fbdev_open_cnt);

	/* For the first open, we might need to initialize hardware or memory */
	if (open_count == 1) {
		/* Hardware initialization would go here if needed */
	}

	return 0;
}

static int syna_fbdev_release(struct fb_info *info, int user)
{
	int open_count;

	open_count = atomic_dec_return(&syna_fbdev_open_cnt);
	if (open_count < 0) {
		atomic_set(&syna_fbdev_open_cnt, 0);
		printk(KERN_ERR "syna_fbdev_release: already released\n");
		return -EBUSY;
	}

	/* For the last release, we might need to cleanup hardware */
	if (open_count == 0) {
		/* Hardware cleanup would go here if needed */
	}

	return 0;
}

static void syna_fbdev_destroy(struct fb_info *info)
{
	/* Cleanup framebuffer resources */
	if (info->screen_base) {
		struct drm_fb_helper *helper = info->par;
		struct drm_framebuffer *fb = helper->fb;

		if (fb && fb->obj[0]) {
			struct syna_gem_object *syna_obj = to_syna_obj(fb->obj[0]);

			/* Only unmap if we created the mapping (not using existing kernel_vir_addr) */
			if (info->screen_base != syna_obj->kernel_vir_addr) {
				iounmap(info->screen_base);
			}
		}

		info->screen_base = NULL;
	}

	/* Reset open count */
	atomic_set(&syna_fbdev_open_cnt, 0);
}

static int syna_fbdev_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct drm_fb_helper *helper = info->par;
	struct drm_framebuffer *fb = helper->fb;
	struct drm_gem_object *obj;
	int err;

	if (!fb || !fb->obj[0]) {
		printk(KERN_ERR "syna_fbdev_mmap: no framebuffer or GEM object\n");
		return -ENODEV;
	}

	obj = fb->obj[0];

	/* First call drm_gem_mmap_obj like Tegra does */
	err = drm_gem_mmap_obj(obj, obj->size, vma);
	if (err < 0) {
		printk(KERN_ERR "syna_fbdev_mmap: drm_gem_mmap_obj failed with %d\n", err);
		return err;
	}

	/* Then call our specific mmap implementation */
	err = syna_drm_gem_object_mmap(obj, vma);
	if (err) {
		printk(KERN_ERR "syna_fbdev_mmap: syna_drm_gem_object_mmap failed with %d\n", err);
		return err;
	}

	return 0;
}

static const struct fb_ops syna_fbdev_ops = {
	.owner = THIS_MODULE,
	.fb_open = syna_fbdev_open,
	.fb_release = syna_fbdev_release,
	.fb_destroy = syna_fbdev_destroy,
	__FB_DEFAULT_DMAMEM_OPS_RDWR,
	DRM_FB_HELPER_DEFAULT_OPS,
	__FB_DEFAULT_DMAMEM_OPS_DRAW,
	.fb_mmap = syna_fbdev_mmap,
};

static const struct drm_framebuffer_funcs syna_fb_funcs = {
	.destroy = drm_gem_fb_destroy,
	.create_handle = drm_gem_fb_create_handle,
};

static int syna_fbdev_create(struct drm_fb_helper *helper,
			     struct drm_fb_helper_surface_size *sizes)
{
	struct drm_device *dev = helper->dev;
	struct drm_framebuffer *fb;
	struct drm_gem_object *obj;
	struct fb_info *info;
	u32 format;
	size_t size;
	int ret;

	/* Determine pixel format based on bpp */
	switch (sizes->surface_bpp) {
	case 16:
		format = DRM_FORMAT_RGB565;
		break;
	case 24:
		format = DRM_FORMAT_RGB888;
		break;
	case 32:
		format = DRM_FORMAT_XRGB8888;
		break;
	default:
		printk(KERN_ERR "syna_fbdev_create: unsupported bpp %d\n", sizes->surface_bpp);
		return -EINVAL;
	}

	/* Calculate framebuffer size */
	size = sizes->surface_width * sizes->surface_height * (sizes->surface_bpp / 8);
	size = PAGE_ALIGN(size);

	/* Create GEM object for framebuffer */
	obj = syna_gem_create_object(dev, size, false);
	if (IS_ERR(obj)) {
		ret = PTR_ERR(obj);
		printk(KERN_ERR "syna_fbdev_create: failed to create GEM object: %d\n", ret);
		return ret;
	}

	/* Create DRM framebuffer manually */
	fb = kzalloc(sizeof(*fb), GFP_KERNEL);
	if (!fb) {
		printk(KERN_ERR "syna_fbdev_create: failed to allocate framebuffer\n");
		drm_gem_object_put(obj);
		return -ENOMEM;
	}

	struct drm_mode_fb_cmd2 mode_cmd = {
		.width = sizes->surface_width,
		.height = sizes->surface_height,
		.pitches[0] = sizes->surface_width * (sizes->surface_bpp / 8),
		.pixel_format = format,
	};

	drm_helper_mode_fill_fb_struct(dev, fb, &mode_cmd);
	fb->obj[0] = obj;

	ret = drm_framebuffer_init(dev, fb, &syna_fb_funcs);
	if (ret) {
		printk(KERN_ERR "syna_fbdev_create: failed to initialize framebuffer: %d\n", ret);
		kfree(fb);
		drm_gem_object_put(obj);
		return ret;
	}

	/* Allocate fb_info structure */
	info = drm_fb_helper_alloc_info(helper);
	if (IS_ERR(info)) {
		ret = PTR_ERR(info);
		printk(KERN_ERR "syna_fbdev_create: failed to allocate fb_info: %d\n", ret);
		drm_framebuffer_put(fb);
		return ret;
	}

	/* Set up fb_info */
	helper->fb = fb;
	info->fbops = &syna_fbdev_ops;

	drm_fb_helper_fill_info(info, helper, sizes);

	/* Set framebuffer memory info - use physical address for smem_start */
	struct syna_gem_object *syna_obj = to_syna_obj(obj);
	info->fix.smem_start = syna_obj->phyaddr;
	info->fix.smem_len = size;
	info->screen_size = size;

	/* Map the physical framebuffer memory to virtual address for fbcon access */
	if (syna_obj->kernel_vir_addr) {
		info->screen_base = syna_obj->kernel_vir_addr;
	} else {
		/* Create a virtual mapping if one doesn't exist */
		info->screen_base = ioremap_wc(syna_obj->phyaddr, size);
		if (!info->screen_base) {
			printk(KERN_ERR "syna_fbdev_create: failed to map framebuffer memory\n");
			drm_fb_helper_release_info(helper);
			drm_framebuffer_put(fb);
			return -ENOMEM;
		}
	}

	return 0;
}

static const struct drm_fb_helper_funcs syna_fbdev_helper_funcs = {
	.fb_probe = syna_fbdev_create,
};

int syna_fbdev_init(struct drm_device *dev)
{
	struct drm_fb_helper *helper;
	int ret;

	helper = kzalloc(sizeof(*helper), GFP_KERNEL);
	if (!helper)
		return -ENOMEM;

	drm_fb_helper_prepare(dev, helper, 32, &syna_fbdev_helper_funcs);

	ret = drm_fb_helper_init(dev, helper);
	if (ret) {
		printk(KERN_ERR "syna_fbdev_init: drm_fb_helper_init failed: %d\n", ret);
		kfree(helper);
		return ret;
	}

	ret = drm_fb_helper_initial_config(helper);
	if (ret) {
		printk(KERN_ERR "syna_fbdev_init: drm_fb_helper_initial_config failed: %d\n", ret);
		drm_fb_helper_fini(helper);
		kfree(helper);
		return ret;
	}

	/* Store helper in device for cleanup */
	dev->fb_helper = helper;

	return 0;
}

void syna_fbdev_cleanup(struct drm_device *dev)
{
	struct drm_fb_helper *helper = dev->fb_helper;

	if (!helper)
		return;

	drm_fb_helper_unregister_info(helper);
	drm_fb_helper_fini(helper);
	kfree(helper);
	dev->fb_helper = NULL;
}
