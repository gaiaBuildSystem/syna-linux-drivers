// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ctrls.h>
#include "camera_video_register.h"
#include "camera_v4l2_common.h"

/* V4L2 Control IDs for video device - Use same ID as ISP subdev */
#define V4L2_CID_CAMERA_WB_ENABLE V4L2_CID_USER_WB_ENABLE

#ifndef V4L2_PIX_FMT_P010
/* 24 Y/CbCr 4:2:0 10-bit per component */
#define V4L2_PIX_FMT_P010 v4l2_fourcc('P', '0', '1', '0')
#endif

/**
 * camera_video_s_ctrl - Set V4L2 control value
 * @ctrl: V4L2 control to set
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_video_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct camera_video_dev *camera_vdev =
		container_of(ctrl->handler, struct camera_video_dev, ctrl_handler);
	struct v4l2_subdev *subdev;
	struct media_pad *pad;
	struct v4l2_ctrl *isp_ctrl;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_WB_ENABLE:
		pad = media_pad_remote_pad_first(&camera_vdev->pad);
		if (pad && is_media_entity_v4l2_subdev(pad->entity)) {
			subdev = media_entity_to_v4l2_subdev(pad->entity);
			isp_ctrl = v4l2_ctrl_find(subdev->ctrl_handler, V4L2_CID_USER_WB_ENABLE);
			if (isp_ctrl) {
				ret = v4l2_ctrl_s_ctrl(isp_ctrl, ctrl->val);
				if (ret)
					dev_err(camera_vdev->camera_mdev->dev,
						"Failed to set wb_enable on ISP subdev: %d\n",
						ret);
				else
					camera_vdev->wb_enable = ctrl->val;
			} else {
				dev_err(camera_vdev->camera_mdev->dev,
						"wb_enable control not found in ISP subdev\n");
				ret = -ENOENT;
			}
		} else {
			dev_err(camera_vdev->camera_mdev->dev,
					"No ISP subdevice connected\n");
			ret = -ENODEV;
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/**
 * camera_video_g_ctrl - Get V4L2 control value
 * @ctrl: V4L2 control to get
 *
 * This function retrieves the current value of a V4L2 control.
 * Currently supports the white balance enable control.
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_video_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct camera_video_dev *camera_vdev =
		container_of(ctrl->handler, struct camera_video_dev, ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_WB_ENABLE:
		ctrl->val = camera_vdev->wb_enable;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops camera_video_ctrl_ops = {
	.s_ctrl = camera_video_s_ctrl,
	.g_volatile_ctrl = camera_video_g_ctrl,
};

static const struct v4l2_ctrl_config camera_video_ctrls[] = {
	{
		.ops   = &camera_video_ctrl_ops,
		.id    = V4L2_CID_USER_WB_ENABLE,
		.type  = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
		.name  = "wb_enable",
		.step  = 1,
		.min   = 0,
		.max   = 1,
		.def   = 0,
	},
};

/**
 * camera_video_create_ctrls - Create V4L2 controls for video device
 * @camera_vdev: Camera video device instance
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_video_create_ctrls(struct camera_video_dev *camera_vdev)
{
	int i, ret;

	ret = v4l2_ctrl_handler_init(&camera_vdev->ctrl_handler, ARRAY_SIZE(camera_video_ctrls));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(camera_video_ctrls); i++) {
		struct v4l2_ctrl *ctrl = v4l2_ctrl_new_custom(&camera_vdev->ctrl_handler,
				&camera_video_ctrls[i], NULL);
		if (!ctrl) {
			dev_err(camera_vdev->camera_mdev->dev,
				"Failed to create control: %s\n",
				camera_video_ctrls[i].name);
			v4l2_ctrl_handler_free(&camera_vdev->ctrl_handler);
			return -EINVAL;
		}
	}

	if (camera_vdev->ctrl_handler.error) {
		ret = camera_vdev->ctrl_handler.error;
		v4l2_ctrl_handler_free(&camera_vdev->ctrl_handler);
		return ret;
	}

	camera_vdev->video->ctrl_handler = &camera_vdev->ctrl_handler;
	return 0;
}

/* Essential camera formats - optimized for minimal driver */
static struct camera_video_fmt_info camera_formats_info[] = {
	/* YUV formats - most common for camera applications */
	{
		.fourcc	= V4L2_PIX_FMT_NV12M,
		.mbus	= MEDIA_BUS_FMT_YUYV8_1_5X8,
	},
	{
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.mbus	= MEDIA_BUS_FMT_YUYV8_1X16,
	},
	{
		.fourcc    = V4L2_PIX_FMT_NV16,
		.mbus      = MEDIA_BUS_FMT_YUYV8_2X8,
	},
	/* Raw Bayer formats - essential for camera sensors */
	{
		.fourcc	= V4L2_PIX_FMT_SBGGR8,
		.mbus	= MEDIA_BUS_FMT_SBGGR8_1X8,
	},
	{
		.fourcc	= V4L2_PIX_FMT_SGRBG8,
		.mbus	= MEDIA_BUS_FMT_SGRBG8_1X8,
	},
	{
		.fourcc	= V4L2_PIX_FMT_SRGGB8,
		.mbus	= MEDIA_BUS_FMT_SRGGB8_1X8,
	},
	{
		.fourcc	= V4L2_PIX_FMT_SGBRG8,
		.mbus	= MEDIA_BUS_FMT_SGBRG8_1X8,
	},
	{
		.fourcc	= V4L2_PIX_FMT_SGRBG10,
		.mbus	= MEDIA_BUS_FMT_SGRBG10_1X10,
	},
	{
		.fourcc	= V4L2_PIX_FMT_SRGGB10,
		.mbus	= MEDIA_BUS_FMT_SRGGB10_1X10,
	},
	{
		.fourcc	= V4L2_PIX_FMT_SBGGR10,
		.mbus	= MEDIA_BUS_FMT_SBGGR10_1X10,
	},
	{
		.fourcc	= V4L2_PIX_FMT_SGBRG10,
		.mbus	= MEDIA_BUS_FMT_SGBRG10_1X10,
	},

	/* RGB format - for processed output */
	{
		.fourcc	= V4L2_PIX_FMT_RGB24,
		.mbus	= MEDIA_BUS_FMT_RGB888_3X8,
	},
};

/**
 * camera_video_mbus_to_fourcc - Convert media bus format to fourcc
 * @mbus: Media bus format code
 * @fourcc: Pointer to store fourcc value
 *
 * Return: 0 on success, -EINVAL if format not found
 */
static int camera_video_mbus_to_fourcc(uint32_t mbus, uint32_t *fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(camera_formats_info); i++) {
		if (camera_formats_info[i].mbus == mbus) {
			*fourcc = camera_formats_info[i].fourcc;
			return 0;
		}
	}
	dev_err(NULL, "%s: %d returning -EINVAL\n", __func__, __LINE__);
	return -EINVAL;
}

/**
 * camera_video_fourcc_to_mbus - Convert fourcc to media bus format
 * @fourcc: Fourcc format code
 * @mbus: Pointer to store media bus format
 *
 * Return: 0 on success, -EINVAL if format not found
 */
static int camera_video_fourcc_to_mbus(uint32_t fourcc, uint32_t *mbus)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(camera_formats_info); i++) {
		if (camera_formats_info[i].fourcc == fourcc) {
			*mbus = camera_formats_info[i].mbus;
			return 0;
		}
	}
	dev_err(NULL, "%s: %d returning -EINVAL\n", __func__, __LINE__);
	return -EINVAL;
}

/**
 * camera_video_vfmt_to_mfmt - Convert V4L2 format to subdev format
 * @f: V4L2 format structure
 * @mfmt: Subdev format structure to fill
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_video_vfmt_to_mfmt(struct v4l2_format *f, struct v4l2_subdev_format *mfmt)
{
	mfmt->format.width = f->fmt.pix_mp.width;
	mfmt->format.height = f->fmt.pix_mp.height;
	mfmt->format.field = V4L2_FIELD_NONE;
	mfmt->format.colorspace = f->fmt.pix_mp.colorspace;
	mfmt->format.quantization = f->fmt.pix_mp.quantization;

	return camera_video_fourcc_to_mbus(f->fmt.pix_mp.pixelformat, &mfmt->format.code);
}

/**
 * camera_video_mfmt_to_vfmt - Convert subdev format to V4L2 format
 * @mfmt: Subdev format structure
 * @f: V4L2 format structure to fill
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_video_mfmt_to_vfmt(struct v4l2_subdev_format *mfmt, struct v4l2_format *f)
{
	u32 fourcc;
	int ret;
	const struct v4l2_format_info *info;

	ret = camera_video_mbus_to_fourcc(mfmt->format.code, &fourcc);
	if (ret < 0)
		return ret;

	info = v4l2_format_info(fourcc);
	if (!info) {
		dev_err(NULL, "%s: %d returning -EINVAL\n", __func__, __LINE__);
		return -EINVAL;
	}

	memset(f, 0, sizeof(*f));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f->fmt.pix_mp.width = mfmt->format.width;
	f->fmt.pix_mp.height = mfmt->format.height;
	f->fmt.pix_mp.pixelformat = fourcc;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.colorspace = mfmt->format.colorspace;
	f->fmt.pix_mp.quantization = mfmt->format.quantization;

	/* Fill plane format information */
	f->fmt.pix_mp.num_planes = info->mem_planes;
	if (info->mem_planes == 1) {
		f->fmt.pix_mp.plane_fmt[0].bytesperline = info->bpp[0] * f->fmt.pix_mp.width;
		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline * f->fmt.pix_mp.height;
	} else {
		int i;

		for (i = 0; i < info->mem_planes; i++) {
			f->fmt.pix_mp.plane_fmt[i].bytesperline = info->bpp[i] * f->fmt.pix_mp.width;
			f->fmt.pix_mp.plane_fmt[i].sizeimage =
				f->fmt.pix_mp.plane_fmt[i].bytesperline * f->fmt.pix_mp.height;
		}
	}
	return 0;
}

/**
 * camera_video_remote_subdev - Get remote subdevice connected to video device
 * @camera_vdev: Camera video device instance
 *
 * Return: Pointer to remote subdevice, or NULL if not found
 */
static struct v4l2_subdev *camera_video_remote_subdev(struct camera_video_dev *camera_vdev)
{
	struct media_pad *pad = media_pad_remote_pad_first(&camera_vdev->pad);

	if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
		return NULL;

	return media_entity_to_v4l2_subdev(pad->entity);
}

/**
 * camera_video_destroy_pipeline - Destroy video pipeline
 * @camera_vdev: Camera video device instance
 *
 * Return: 0 on success
 */
static int camera_video_destroy_pipeline(struct camera_video_dev *camera_vdev)
{
	camera_vdev->pipeline = 0;
	return 0;
}

/**
 * camera_videoc_querycap - Query device capabilities
 * @file: File handle
 * @priv: Private data
 * @cap: Capability structure to fill
 *
 * Return: 0 on success
 */
static int camera_videoc_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);

	strscpy(cap->driver, camera_vdev->video->name, sizeof(cap->driver));
	strscpy(cap->card, camera_vdev->video->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", camera_vdev->video->name);

	return 0;
}

/**
 * camera_videoc_enum_fmt_vid_cap_mplane - Enumerate video formats
 * @file: File handle
 * @priv: Private data
 * @f: Format description structure to fill
 *
 * Return: 0 on success, -EINVAL if index out of range
 */
static int camera_videoc_enum_fmt_vid_cap_mplane(struct file *file, void *priv,
		struct v4l2_fmtdesc *f)
{
	/* Direct format enumeration using static format array */
	if (f->index >= ARRAY_SIZE(camera_formats_info))
		return -EINVAL;

	f->pixelformat = camera_formats_info[f->index].fourcc;
	/* Generate description from fourcc */
	snprintf(f->description, sizeof(f->description), "%.4s", (char *)&f->pixelformat);
	f->flags = 0;

	return 0;
}

/**
 * camera_videoc_reqbufs - Request video buffers for streaming
 * @file: File handle
 * @priv: Private data
 * @p: Buffer request parameters
 *
 * This function handles buffer allocation requests from userspace.
 * It delegates to the VB2 framework for actual buffer management.
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_videoc_reqbufs(struct file *file, void *priv,
		struct v4l2_requestbuffers *p)
{
	return vb2_ioctl_reqbufs(file, priv, p);
}

/**
 * camera_videoc_try_fmt_vid_cap_mplane - Try multi-plane video format
 * @file: File handle
 * @priv: Private data
 * @f: Format structure to validate
 *
 * This function validates a multi-plane video format without actually setting it.
 * It communicates with the connected ISP subdevice to check format compatibility
 * and performs format conversion between V4L2 and media bus formats.
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_videoc_try_fmt_vid_cap_mplane(struct file *file, void *priv,
		struct v4l2_format *f)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev_format mfmt;
	struct v4l2_subdev *subdev;
	struct media_pad *pad;
	int ret;
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev_state sd_state = {
		.pads = &pad_cfg,
	};

	subdev = camera_video_remote_subdev(camera_vdev);
	if (!subdev) {
		dev_err(NULL, "%s: %d returning -ENOTTY\n", __func__, __LINE__);
		return -ENOTTY;
	}

	pad = media_pad_remote_pad_first(&camera_vdev->pad);

	memset(&mfmt, 0, sizeof(mfmt));
	/* Convert multi-plane format to media bus format */
	ret = camera_video_vfmt_to_mfmt(f, &mfmt);
	if (ret)
		return ret;

	mfmt.pad = pad->index;
	mfmt.which = V4L2_SUBDEV_FORMAT_TRY;
	ret = v4l2_subdev_call(subdev, pad, set_fmt, &sd_state, &mfmt);
	if (ret)
		return ret;

	/* Convert back to multi-plane format */
	ret = camera_video_mfmt_to_vfmt(&mfmt, f);
	return ret;
}

/**
 * camera_videoc_s_fmt_vid_cap_mplane - Set multi-plane video format
 * @file: File handle
 * @priv: Private data
 * @f: Format structure to set
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_videoc_s_fmt_vid_cap_mplane(struct file *file, void *priv,
		struct v4l2_format *f)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev *subdev;
	struct v4l2_subdev_format sd_fmt;
	struct media_pad *pad;
	int ret;
	int i;
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev_state sd_state = {
		.pads = &pad_cfg,
	};

	if (vb2_is_busy(&camera_vdev->queue))
		return -EBUSY;

	/* Validate format type */
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	/* Validate pixel format against supported formats */
	for (i = 0; i < ARRAY_SIZE(camera_formats_info); i++) {
		if (camera_formats_info[i].fourcc == f->fmt.pix_mp.pixelformat)
			break;
	}
	if (i >= ARRAY_SIZE(camera_formats_info))
		return -EINVAL;

	/* Set reasonable constraints */
	if (f->fmt.pix_mp.width < 64 || f->fmt.pix_mp.width > 4096 ||
			f->fmt.pix_mp.height < 64 || f->fmt.pix_mp.height > 4096)
		return -EINVAL;

	/* Configure format based on pixel format */
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;

	/* Set plane information based on format */
	if (f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_NV12 ||
			f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_NV12M ||
			f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_NV16) {
		/* Multi-plane format */
		f->fmt.pix_mp.num_planes = 2;
		f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width;
		f->fmt.pix_mp.plane_fmt[0].sizeimage = f->fmt.pix_mp.width * f->fmt.pix_mp.height;
		f->fmt.pix_mp.plane_fmt[1].bytesperline = f->fmt.pix_mp.width;
		f->fmt.pix_mp.plane_fmt[1].sizeimage =
			f->fmt.pix_mp.width * f->fmt.pix_mp.height / 2;
	} else {
		/* Single-plane format through MPLANE interface */
		f->fmt.pix_mp.num_planes = 1;
		switch (f->fmt.pix_mp.pixelformat) {
		case V4L2_PIX_FMT_YUYV:
			f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width * 2;
			break;
		case V4L2_PIX_FMT_RGB565:
			f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width * 2;
			break;
		case V4L2_PIX_FMT_RGB24:
			f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width * 3;
			break;
		case V4L2_PIX_FMT_ARGB32:
			f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width * 4;
			break;
		case V4L2_PIX_FMT_GREY:
			f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width;
			break;
		default:
			f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width * 2;
			break;
		}
		f->fmt.pix_mp.plane_fmt[0].sizeimage = f->fmt.pix_mp.plane_fmt[0].bytesperline
			* f->fmt.pix_mp.height;
	}

	subdev = camera_video_remote_subdev(camera_vdev);
	if (!subdev) {
		dev_err(NULL, "%s: %d returning -ENOTTY\n", __func__, __LINE__);
		return -ENOTTY;
	}

	pad = media_pad_remote_pad_first(&camera_vdev->pad);

	memset(&sd_fmt, 0, sizeof(sd_fmt));
	sd_fmt.pad = pad->index;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	/* Convert multi-plane format to media bus format */
	ret = camera_video_vfmt_to_mfmt(f, &sd_fmt);
	if (ret) {
		pr_err("%s %d error %d!!\n", __func__, __LINE__, ret);
		return ret;
	}

	ret = v4l2_subdev_call(subdev, pad, set_fmt, &sd_state, &sd_fmt);
	if (ret) {
		pr_err("%s %d error %d!!\n", __func__, __LINE__, ret);
		return ret;
	}

	/* Store the validated format */
	camera_vdev->format = *f;

	return 0;
}

/** Get current video format */
static int camera_videoc_g_fmt_vid_cap_mplane(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);

	/* Return stored format or set default if not initialized */
	if (camera_vdev->format.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		*f = camera_vdev->format;
	} else {
		/* Try to get current active format from the connected subdevice */
		struct v4l2_subdev *subdev;
		struct media_pad *pad;
		struct v4l2_subdev_format sd_fmt;
		int ret = 0;

		subdev = camera_video_remote_subdev(camera_vdev);
		if (subdev) {
			pad = media_pad_remote_pad_first(&camera_vdev->pad);
			if (pad) {
				memset(&sd_fmt, 0, sizeof(sd_fmt));
				sd_fmt.pad = pad->index;
				sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
				ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &sd_fmt);
				if (!ret) {
					/* Convert subdev format to MPLANE V4L2 format */
					ret = camera_video_mfmt_to_vfmt(&sd_fmt, f);
				}
			} else {
				ret = -ENOTTY;
			}
		} else {
			ret = -ENOTTY;
		}

		if (ret) {
			/* Fallback default MPLANE format: 1920x1080 YUYV */
			memset(f, 0, sizeof(*f));
			f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			f->fmt.pix_mp.width = 1920;
			f->fmt.pix_mp.height = 1080;
			f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUYV;
			f->fmt.pix_mp.field = V4L2_FIELD_NONE;
			f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
			f->fmt.pix_mp.num_planes = 1;
			f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width * 2;
			f->fmt.pix_mp.plane_fmt[0].sizeimage =
				f->fmt.pix_mp.plane_fmt[0].bytesperline * f->fmt.pix_mp.height;
		}

		/* Store discovered or fallback format */
		camera_vdev->format = *f;
	}

	return 0;
}

/** Enumerate video inputs */
static int camera_videoc_enum_input(struct file *file, void *fh,
		struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strscpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

/** Get current input */
static int camera_videoc_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;
	return 0;
}

/** Set input */
static int camera_videoc_s_input(struct file *file, void *fh, unsigned int input)
{
	return (input == 0) ? 0 : -EINVAL;
}

/** Query control */
static int camera_videoc_queryctrl(struct file *file, void *fh,
		struct v4l2_queryctrl *a)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev *subdev = camera_video_remote_subdev(camera_vdev);
	struct camera_pad_queryctrl pad_query_ctrl;
	struct media_pad *pad;
	int ret = 0;

	if (!subdev)
		return -ENODEV;

	pad = media_pad_remote_pad_first(&camera_vdev->pad);
	pad_query_ctrl.pad = pad ? pad->index : 0;
	pad_query_ctrl.query_ctrl = a;
	ret = v4l2_subdev_call(subdev, core, ioctl, CAMERA_PAD_QUERYCTRL, &pad_query_ctrl);

	return ret;
}

/** Query extended control */
static int camera_videoc_query_ext_ctrl(struct file *file, void *fh,
		struct v4l2_query_ext_ctrl *a)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev *subdev = camera_video_remote_subdev(camera_vdev);
	struct camera_pad_query_ext_ctrl pad_query_ext_ctrl;
	struct media_pad *pad;
	int ret = 0;

	if (!subdev)
		return -ENODEV;

	pad = media_pad_remote_pad_first(&camera_vdev->pad);
	pad_query_ext_ctrl.pad = pad ? pad->index : 0;
	pad_query_ext_ctrl.query_ext_ctrl = a;
	ret = v4l2_subdev_call(subdev, core, ioctl, CAMERA_PAD_QUERY_EXT_CTRL, &pad_query_ext_ctrl);

	return ret;
}

/** Get control value */
static int camera_vidioc_g_ctrl(struct file *file, void *fh,
		struct v4l2_control *a)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev *subdev = camera_video_remote_subdev(camera_vdev);
	struct camera_pad_control pad_ctrl;
	struct media_pad *pad;
	int ret = 0;

	if (!subdev)
		return -ENODEV;

	pad = media_pad_remote_pad_first(&camera_vdev->pad);
	pad_ctrl.pad = pad ? pad->index : 0;
	pad_ctrl.control = a;
	ret = v4l2_subdev_call(subdev, core, ioctl, CAMERA_PAD_G_CTRL, &pad_ctrl);

	return ret;
}

/** Set control value */
static int camera_vidioc_s_ctrl(struct file *file, void *fh,
		struct v4l2_control *a)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev *subdev = camera_video_remote_subdev(camera_vdev);
	struct camera_pad_control pad_ctrl;
	struct media_pad *pad;
	int ret = 0;

	if (!subdev)
		return -ENODEV;

	pad = media_pad_remote_pad_first(&camera_vdev->pad);
	pad_ctrl.pad = pad ? pad->index : 0;
	pad_ctrl.control = a;
	ret = v4l2_subdev_call(subdev, core, ioctl, CAMERA_PAD_S_CTRL, &pad_ctrl);

	return ret;
}

/** Get extended controls */
static int camera_vidioc_g_ext_ctrls(struct file *file, void *fh,
		struct v4l2_ext_controls *a)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev *subdev = camera_video_remote_subdev(camera_vdev);
	struct camera_pad_ext_controls pad_ext_ctrls;
	struct media_pad *pad;
	int ret = 0;

	if (!subdev)
		return -ENODEV;

	pad = media_pad_remote_pad_first(&camera_vdev->pad);
	pad_ext_ctrls.pad = pad ? pad->index : 0;
	pad_ext_ctrls.ext_controls = a;
	ret = v4l2_subdev_call(subdev, core, ioctl, CAMERA_PAD_G_EXT_CTRLS, &pad_ext_ctrls);

	return ret;
}

/** Set extended controls */
static int camera_vidioc_s_ext_ctrls(struct file *file, void *fh,
		struct v4l2_ext_controls *a)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev *subdev = camera_video_remote_subdev(camera_vdev);
	struct camera_pad_ext_controls pad_ext_ctrls;
	struct media_pad *pad;
	int ret = 0;

	if (!subdev)
		return -ENODEV;

	pad = media_pad_remote_pad_first(&camera_vdev->pad);
	pad_ext_ctrls.pad = pad ? pad->index : 0;
	pad_ext_ctrls.ext_controls = a;
	ret = v4l2_subdev_call(subdev, core, ioctl, CAMERA_PAD_S_EXT_CTRLS, &pad_ext_ctrls);

	return ret;
}

/** Try extended controls */
static int camera_vidioc_try_ext_ctrls(struct file *file, void *fh,
		struct v4l2_ext_controls *a)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_subdev *subdev = camera_video_remote_subdev(camera_vdev);
	struct camera_pad_ext_controls pad_ext_ctrls;
	struct media_pad *pad;
	int ret = 0;

	if (!subdev)
		return -ENODEV;

	pad = media_pad_remote_pad_first(&camera_vdev->pad);
	pad_ext_ctrls.pad = pad ? pad->index : 0;
	pad_ext_ctrls.ext_controls = a;
	ret = v4l2_subdev_call(subdev, core, ioctl, CAMERA_PAD_TRY_EXT_CTRLS, &pad_ext_ctrls);

	return ret;
}

/** Query control menu */
static int camera_vidioc_querymenu(struct file *file, void *fh,
		struct v4l2_querymenu *a)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	struct camera_pad_querymenu pad_querymenu;
	int ret = 0;

	subdev = camera_video_remote_subdev(camera_vdev);
	if (subdev) {
		pad = media_pad_remote_pad_first(&camera_vdev->pad);
		if (!pad) {
			dev_err(camera_vdev->camera_mdev->dev,
				"No remote pad found for querymenu!\n");
			return -ENOTTY;
		}
		memset(&pad_querymenu, 0, sizeof(pad_querymenu));
		pad_querymenu.pad = pad->index;
		pad_querymenu.querymenu = a;
		ret = v4l2_subdev_call(subdev, core, ioctl,
				CAMERA_PAD_QUERYMENU, &pad_querymenu);
	} else {
		return -ENOTTY;
	}

	return ret;
}


/** Subscribe to events */
static int camera_videoc_subscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub)
{
	return v4l2_event_subscribe(fh, sub, 0, NULL);
}

static const struct v4l2_ioctl_ops camera_video_ioctl_ops = {
	.vidioc_querycap            = camera_videoc_querycap,

	/* Multi-plane format operations (MPLANE-only approach) */
	.vidioc_enum_fmt_vid_cap        = camera_videoc_enum_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_cap_mplane  = camera_videoc_try_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane    = camera_videoc_g_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane    = camera_videoc_s_fmt_vid_cap_mplane,

	.vidioc_reqbufs             = camera_videoc_reqbufs,
	.vidioc_querybuf            = vb2_ioctl_querybuf,
	.vidioc_create_bufs         = vb2_ioctl_create_bufs,
	.vidioc_qbuf                = vb2_ioctl_qbuf,
	.vidioc_expbuf              = vb2_ioctl_expbuf,
	.vidioc_dqbuf               = vb2_ioctl_dqbuf,
	.vidioc_prepare_buf         = vb2_ioctl_prepare_buf,
	.vidioc_streamon            = vb2_ioctl_streamon,
	.vidioc_streamoff           = vb2_ioctl_streamoff,

	.vidioc_enum_input          = camera_videoc_enum_input,
	.vidioc_g_input             = camera_videoc_g_input,
	.vidioc_s_input             = camera_videoc_s_input,
	.vidioc_queryctrl           = camera_videoc_queryctrl,
	.vidioc_query_ext_ctrl      = camera_videoc_query_ext_ctrl,
	.vidioc_g_ctrl              = camera_vidioc_g_ctrl,
	.vidioc_s_ctrl              = camera_vidioc_s_ctrl,
	.vidioc_g_ext_ctrls         = camera_vidioc_g_ext_ctrls,
	.vidioc_s_ext_ctrls         = camera_vidioc_s_ext_ctrls,
	.vidioc_try_ext_ctrls       = camera_vidioc_try_ext_ctrls,
	.vidioc_querymenu           = camera_vidioc_querymenu,
	.vidioc_subscribe_event     = camera_videoc_subscribe_event,
	.vidioc_unsubscribe_event   = v4l2_event_unsubscribe,
};


/** Memory map buffers */
static int camera_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	int ret;

	if (camera_vdev->video->queue->owner &&
			(camera_vdev->video->queue->owner == fh))
		return vb2_fop_mmap(file, vma);

	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
	return ret;
}

/** Open video device */
static int camera_video_open(struct file *file)
{
	return v4l2_fh_open(file);
}

/** Release video device */
static int camera_video_release(struct file *file)
{
	struct camera_video_dev *camera_vdev = video_drvdata(file);
	int ret;

	ret = vb2_fop_release(file);
	if (camera_vdev->video->queue->owner == NULL) {
		if (camera_vdev->pipeline)
			camera_video_destroy_pipeline(camera_vdev);
	}

	return ret;
}

static const struct v4l2_file_operations camera_video_fops = {
	.owner          = THIS_MODULE,
	.open           = camera_video_open,
	.release        = camera_video_release,
	.poll           = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = camera_video_mmap,
};

/**
 * camera_video_vb2_queue_setup - Configure VB2 queue for buffer allocation
 * @queue: VB2 queue instance
 * @num_buffers: Pointer to number of buffers requested/allocated
 * @num_planes: Pointer to number of planes per buffer
 * @sizes: Array of plane sizes for each plane
 * @alloc_devs: Array of devices for each plane (for DMABUF)
 *
 * This function is called by the VB2 framework during buffer allocation
 * (VIDIOC_REQBUFS) to determine the buffer configuration. It supports
 * both single-plane and multi-plane formats in MPLANE-only architecture.
 *
 * For multi-plane formats like NV12:
 * - Plane 0: Y (luminance) data
 * - Plane 1: UV (chrominance) data
 *
 * For single-plane formats like YUYV:
 * - Plane 0: Packed YUV data
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_video_vb2_queue_setup(struct vb2_queue *queue,
		unsigned int *num_buffers,
		unsigned int *num_planes,
		unsigned int sizes[],
		struct device *alloc_devs[])
{
	struct camera_video_dev *camera_vdev = queue->drv_priv;
	struct v4l2_format *format = &camera_vdev->format;
	struct v4l2_pix_format_mplane *pix_mp = &format->fmt.pix_mp;
	unsigned int i;

	dev_dbg(camera_vdev->camera_mdev->dev,
			"VB2 queue setup: %u buffers, %u planes requested\n",
			*num_buffers, *num_planes);
	dev_dbg(camera_vdev->camera_mdev->dev,
			"Queue type: %d\n", queue->type);

	/* Ensure format is initialized as MPLANE */
	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		struct v4l2_subdev *subdev;
		struct media_pad *pad;
		struct v4l2_subdev_format sd_fmt;
		int ret = 0;

		dev_warn(camera_vdev->camera_mdev->dev,
			"Format type mismatch: %u, attempting to initialize from subdevice\n",
			format->type);

		subdev = camera_video_remote_subdev(camera_vdev);
		if (subdev) {
			pad = media_pad_remote_pad_first(&camera_vdev->pad);
			if (pad) {
				memset(&sd_fmt, 0, sizeof(sd_fmt));
				sd_fmt.pad = pad->index;
				sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
				ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &sd_fmt);
				if (!ret) {
					struct v4l2_format tmp;

					memset(&tmp, 0, sizeof(tmp));
					ret = camera_video_mfmt_to_vfmt(&sd_fmt, &tmp);
					if (!ret)
						*format = tmp;
				}
			} else {
				ret = -ENOTTY;
			}
		} else {
			ret = -ENOTTY;
		}

		if (ret || format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			/* Fallback default to 1920x1080 YUYV */
			memset(format, 0, sizeof(*format));
			format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			format->fmt.pix_mp.width = 1920;
			format->fmt.pix_mp.height = 1080;
			format->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUYV;
			format->fmt.pix_mp.field = V4L2_FIELD_NONE;
			format->fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
			format->fmt.pix_mp.num_planes = 1;
			format->fmt.pix_mp.plane_fmt[0].bytesperline = format->fmt.pix_mp.width * 2;
			format->fmt.pix_mp.plane_fmt[0].sizeimage =
				format->fmt.pix_mp.plane_fmt[0].bytesperline * format->fmt.pix_mp.height;
		}
	}

	dev_info(camera_vdev->camera_mdev->dev,
			"Format: %ux%u, pixelformat=0x%x (%c%c%c%c), num_planes=%u\n",
			pix_mp->width, pix_mp->height, pix_mp->pixelformat,
			(pix_mp->pixelformat >> 0) & 0xff,
			(pix_mp->pixelformat >> 8) & 0xff,
			(pix_mp->pixelformat >> 16) & 0xff,
			(pix_mp->pixelformat >> 24) & 0xff,
			pix_mp->num_planes);

	/* MPLANE-only buffer setup */
	if (*num_planes) {
		dev_info(camera_vdev->camera_mdev->dev, "Validating existing plane setup\n");
		if (*num_planes != pix_mp->num_planes) {
			dev_err(camera_vdev->camera_mdev->dev,
					"Plane count mismatch: expected %u, got %u\n",
					pix_mp->num_planes, *num_planes);
			return -EINVAL;
		}
		for (i = 0; i < *num_planes; i++) {
			if (sizes[i] < pix_mp->plane_fmt[i].sizeimage) {
				dev_err(camera_vdev->camera_mdev->dev,
						"Plane %d size too small: %u < %u\n",
						i, sizes[i], pix_mp->plane_fmt[i].sizeimage);
				return -EINVAL;
			}
		}
		dev_info(camera_vdev->camera_mdev->dev, "Plane validation passed\n");
		return 0;
	}

	*num_planes = pix_mp->num_planes;
	for (i = 0; i < *num_planes; i++) {
		sizes[i] = pix_mp->plane_fmt[i].sizeimage;
		dev_dbg(camera_vdev->camera_mdev->dev,
				"Plane %d: size=%u, bytesperline=%u\n",
				i, sizes[i], pix_mp->plane_fmt[i].bytesperline);
	}

	/* Limit maximum number of buffers to prevent memory issues */
	#define MAX_VIDEO_BUFFERS 32
	if (*num_buffers > MAX_VIDEO_BUFFERS) {
		dev_warn(camera_vdev->camera_mdev->dev,
			"Limiting buffers from %u to %u\n",
			*num_buffers, MAX_VIDEO_BUFFERS);
		*num_buffers = MAX_VIDEO_BUFFERS;
	}

	dev_dbg(camera_vdev->camera_mdev->dev,
			"=== QUEUE SETUP SUCCESS: %u buffers, %u planes ===\n",
			*num_buffers, *num_planes);

	return 0;
}

/** Prepare buffer for streaming */
static int camera_video_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct camera_video_dev *camera_vdev = vb->vb2_queue->drv_priv;
	struct v4l2_format *format = &camera_vdev->format;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camera_vb2_buffer *buf = container_of(vbuf, struct camera_vb2_buffer, vb);
	int i;

	/* Validate buffer type for MPLANE capture */
	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	/* Validate plane count */
	if (format->fmt.pix_mp.num_planes == 0 || format->fmt.pix_mp.num_planes > 3) {
		dev_err(camera_vdev->camera_mdev->dev,
				"Invalid plane count: %u\n", format->fmt.pix_mp.num_planes);
		return -EINVAL;
	}

	/* Prepare multi-plane buffer with DMA-contiguous allocation */
	buf->num_planes = format->fmt.pix_mp.num_planes;
	buf->sequence   = vb->index;

	for (i = 0; i < format->fmt.pix_mp.num_planes; i++) {
		unsigned long plane_size = format->fmt.pix_mp.plane_fmt[i].sizeimage;

		if (vb2_plane_size(vb, i) < plane_size) {
			dev_err(camera_vdev->camera_mdev->dev,
					"Plane %d size mismatch: required %lu, available %lu\n",
					i, plane_size, vb2_plane_size(vb, i));
			return -EINVAL;
		}

		/* Get DMA-contiguous address for each plane */
		buf->planes[i].dma_addr = vb2_dma_contig_plane_dma_addr(vb, i);
		buf->planes[i].size     = plane_size;
		vb2_set_plane_payload(vb, i, plane_size);

		dev_dbg(camera_vdev->camera_mdev->dev,
				"Plane %d: DMA addr=0x%lx, size=%u\n",
				i, (unsigned long)buf->planes[i].dma_addr,
				(unsigned int)buf->planes[i].size);
	}

	return 0;
}

/** Queue buffer for streaming */
static void camera_video_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct camera_video_dev *camera_vdev = vb->vb2_queue->drv_priv;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camera_vb2_buffer *buf = container_of(vbuf,
			struct camera_vb2_buffer, vb);
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	struct camera_pad_buf pad_buf;

	subdev = camera_video_remote_subdev(camera_vdev);
	if (subdev) {
		pad = media_pad_remote_pad_first(&camera_vdev->pad);
		memset(&pad_buf, 0, sizeof(pad_buf));
		pad_buf.pad = pad->index;
		pad_buf.buf = buf;
		v4l2_subdev_call(subdev, core, ioctl, CAMERA_PAD_BUF_QUEUE, &pad_buf);
	}
}

/**
 * camera_video_vb2_start_streaming - Start video streaming operation
 * @queue: VB2 queue instance
 * @count: Number of buffers allocated for streaming
 *
 * This function is called by the VB2 framework when streaming is started.
 * It handles the initialization of video streaming, including communication
 * with the ISP subdevice when full integration is enabled.
 *
 * Return: 0 on success, negative error code on failure
 */
static int camera_video_vb2_start_streaming(struct vb2_queue *queue,
		unsigned int count)
{
	struct camera_video_dev *camera_vdev = queue->drv_priv;
	int ret = 0;

	dev_dbg(camera_vdev->camera_mdev->dev,
			"Starting video streaming with %u buffers\n", count);

	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	struct camera_pad_stream_status stream_status;

	subdev = camera_video_remote_subdev(camera_vdev);
	if (subdev) {
		pad = media_pad_remote_pad_first(&camera_vdev->pad);
		if (pad) {
			stream_status.status = 1; /* Enable streaming */
			stream_status.pad = pad->index;
			ret = v4l2_subdev_call(subdev, core, ioctl,
					CAMERA_PAD_S_STREAM, &stream_status);
			if (ret) {
				dev_err(camera_vdev->camera_mdev->dev,
						"Failed to start ISP streaming: %d\n", ret);
				return ret;
			}
		}
	} else {
		dev_err(camera_vdev->camera_mdev->dev,
			"Failed to get remote subdev for streaming\n");
		return -ENODEV;
	}

	dev_dbg(camera_vdev->camera_mdev->dev,
			"Video streaming started successfully\n");

	return ret;
}

/**
 * camera_video_vb2_stop_streaming - Stop video streaming operation
 * @queue: VB2 queue instance
 *
 * This function is called by the VB2 framework when streaming is stopped.
 * It handles the cleanup of video streaming, including communication with
 * the ISP subdevice when full integration is enabled, and ensures all
 * active buffers are properly returned to the framework.
 *
 * The function performs:
 * 1. ISP subdevice streaming control (when enabled)
 * 2. Active buffer cleanup and error state marking
 * 3. Proper streaming termination
 */
static void camera_video_vb2_stop_streaming(struct vb2_queue *queue)
{
	struct camera_video_dev *camera_vdev = queue->drv_priv;
	int i;

	dev_dbg(camera_vdev->camera_mdev->dev, "Stopping video streaming\n");

	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	struct camera_pad_stream_status stream_status;

	subdev = camera_video_remote_subdev(camera_vdev);
	if (subdev) {
		pad = media_pad_remote_pad_first(&camera_vdev->pad);
		if (pad) {
			memset(&stream_status, 0, sizeof(stream_status));
			stream_status.pad = pad->index;
			stream_status.status = 0; /* Disable streaming */
			v4l2_subdev_call(subdev, core, ioctl,
					CAMERA_PAD_S_STREAM, &stream_status);
		}
	}

	/* Return all active buffers to VB2 framework with error state
	 * This ensures proper cleanup and prevents buffer leaks
	 */
	for (i = 0; i < queue->max_num_buffers; i++) {
		if (queue->bufs[i] && queue->bufs[i]->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(queue->bufs[i], VB2_BUF_STATE_ERROR);
	}

	dev_info(camera_vdev->camera_mdev->dev,
			"Video streaming stopped successfully\n");
}

static const struct vb2_ops camera_video_queue_ops = {
	.queue_setup     = camera_video_vb2_queue_setup,
	.buf_prepare     = camera_video_vb2_buf_prepare,
	.buf_queue       = camera_video_vb2_buf_queue,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
	.start_streaming = camera_video_vb2_start_streaming,
	.stop_streaming  = camera_video_vb2_stop_streaming,
};

/** Initialize VB2 queue */
static int camera_video_queue_init(struct camera_video_dev *camera_vdev)
{
	int ret = 0;
	struct vb2_queue *queue;

	queue = &camera_vdev->queue;
	/* Initialize with multi-plane by default for optimal V4L2 support */
	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	queue->drv_priv = camera_vdev;
	queue->ops = &camera_video_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->buf_struct_size = sizeof(struct camera_vb2_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
		V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
	queue->lock = &camera_vdev->video_lock;
	queue->dev = camera_vdev->camera_mdev->dev;

	ret = vb2_queue_init(queue);
	if (ret) {
		dev_err(camera_vdev->camera_mdev->dev, "VB2 queue init failed: %d\n", ret);
		return ret;
	}

	camera_vdev->video->queue = queue;

	return 0;
}

/** Setup media link */
static int camera_video_link_setup(struct media_entity *entity,
		const struct media_pad *local,
		const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations camera_video_entity_ops = {
	.link_setup     = camera_video_link_setup,
	.link_validate  = v4l2_subdev_link_validate,
};

int camera_video_register(struct camera_media_dev *camera_mdev, int port)
{
	int ret = 0;
	struct camera_video_dev *camera_vdev;

	camera_vdev = devm_kzalloc(camera_mdev->dev,
			sizeof(struct camera_video_dev), GFP_KERNEL);
	if (!camera_vdev)
		return -ENOMEM;

	mutex_init(&camera_vdev->video_lock);
	camera_vdev->camera_mdev = camera_mdev;

	camera_vdev->video = video_device_alloc();
	if (!camera_vdev->video) {
		dev_err(camera_mdev->dev, "could not alloc video device\n");
		ret = -ENOMEM;
		goto error_free_camera_vdev;
	}

	snprintf(camera_vdev->video->name, sizeof(camera_vdev->video->name),
			"%s.%d.%d", CAMERA_VIDEO_NAME, camera_mdev->id, port);

	camera_vdev->video->fops = &camera_video_fops;
	camera_vdev->video->ioctl_ops = &camera_video_ioctl_ops;
	camera_vdev->video->release = video_device_release_empty;
	camera_vdev->video->v4l2_dev = &camera_mdev->v4l2_dev;
	camera_vdev->video->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING;
	camera_vdev->video->minor = -1;

	video_set_drvdata(camera_vdev->video, camera_vdev);

	camera_vdev->video->entity.name = camera_vdev->video->name;
	camera_vdev->video->entity.obj_type = MEDIA_ENTITY_TYPE_VIDEO_DEVICE;
	camera_vdev->video->entity.function = MEDIA_ENT_F_IO_V4L;
	camera_vdev->video->entity.ops = &camera_video_entity_ops;
	camera_vdev->pad.flags = MEDIA_PAD_FL_SINK;

	ret = media_entity_pads_init(&camera_vdev->video->entity, 1, &camera_vdev->pad);
	if (ret) {
		dev_err(camera_mdev->dev, "entity pad init error\n");
		goto error_video_device_release;
	}

	ret = camera_video_queue_init(camera_vdev);
	if (ret) {
		dev_err(camera_mdev->dev, "queue init error\n");
		goto err_media_entity_cleanup;
	}

	/* Initialize V4L2 controls */
	ret = camera_video_create_ctrls(camera_vdev);
	if (ret) {
		dev_err(camera_mdev->dev, "Failed to create V4L2 controls\n");
		goto err_media_entity_cleanup;
	}

	ret = video_register_device(camera_vdev->video, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(camera_mdev->dev, "video register device error\n");
		goto err_ctrl_cleanup;
	}

	camera_mdev->video_devs[port] = camera_vdev;

	return 0;

err_ctrl_cleanup:
	v4l2_ctrl_handler_free(&camera_vdev->ctrl_handler);
err_media_entity_cleanup:
	media_entity_cleanup(&camera_vdev->video->entity);

error_video_device_release:
	video_device_release(camera_vdev->video);
	camera_vdev->video = NULL;

error_free_camera_vdev:
	devm_kfree(camera_mdev->dev, camera_vdev);

	return ret;
}

int camera_video_unregister(struct camera_media_dev *camera_mdev, int port)
{
	struct camera_video_dev *camera_vdev = camera_mdev->video_devs[port];

	if (camera_vdev == NULL)
		return 0;

	video_unregister_device(camera_vdev->video);
	media_entity_cleanup(&camera_vdev->video->entity);
	video_device_release(camera_vdev->video);
	devm_kfree(camera_mdev->dev, camera_vdev);
	camera_mdev->video_devs[port] = NULL;

	return 0;
}
