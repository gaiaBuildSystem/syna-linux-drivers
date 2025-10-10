/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_graph.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>

#include "camera_isp_driver.h"
#include <media/v4l2-ctrls.h>
#include "camera_v4l2_common.h"
#include "csipipe_api.h"
#include "isp_shm.h"
#include "csipipe_pvt.h"
#include "camera_isp_wb_sysfs.h"

static const char *isp_clock_list[] = {
	"aviopclk",
};

/**
 * camera_isp_set_sensor_resolution_and_program - Set resolution to sensor driver
 * @isp_dev: ISP device pointer
 * @subdev: CSI subdevice pointer
 * @sd_state: subdevice state
 * @sd_fmt: subdevice format structure to populate
 * @format_code: media bus format code to use
 *
 * This function sets resolution to sensor driver via CSI subdevice.
 * Returns: 0 on success, negative error code on failure
 */
static int camera_isp_set_sensor_resolution_and_program(struct camera_isp_dev *isp_dev,
				struct v4l2_subdev *subdev,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *sd_fmt,
				u32 format_code)
{
	int ret;
	struct v4l2_subdev_format sensor_fmt = {
		.which = sd_fmt->which,
		.pad = 0,
		.format = {
			.width = sd_fmt->format.width,
			.height = sd_fmt->format.height,
			.code = format_code,
		}
	};

	/* Get resolution from sensor driver via CSI subdevice */
	ret = v4l2_subdev_call(subdev, pad, set_fmt, sd_state, &sensor_fmt);
	if (ret == 0 && sensor_fmt.format.width > 0 && sensor_fmt.format.height > 0) {
		dev_dbg(isp_dev->dev, "Got resolution from sensor driver: %dx%d\n",
				 sensor_fmt.format.width, sensor_fmt.format.height);

		/* Program CSI subdevice with sensor resolution */
		sd_fmt->format.width = sensor_fmt.format.width;
		sd_fmt->format.height = sensor_fmt.format.height;
		sd_fmt->format.code = sensor_fmt.format.code;//format_code;

		dev_dbg(isp_dev->dev, "Programming CSI with sensor resolution: %dx%d, code 0x%x\n",
				 sd_fmt->format.width, sd_fmt->format.height, sd_fmt->format.code);
		return 0;
	} else {
		dev_info(isp_dev->dev, "Sensor driver not available (ret=%d)\n", ret);
		return ret;
	}
}

static int camera_isp_ctrl_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct camera_isp_dev *isp_dev = container_of(ctrl->handler,
			struct camera_isp_dev, ctrl_handler);
	struct isp_ctrl wb_ctrl;
	int ret = 0;

	dev_info(isp_dev->dev, "ISP s_ctrl: id=0x%x, val=%d\n", ctrl->id, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_USER_WB_ENABLE:
		/* Update ISP device configuration */
		isp_dev->wb_config.wb_en = ctrl->val;

		/* Apply to all active pipes */
		for (int i = 0; i < CAMERA_ISP_PAD_NR - 1; ++i) {
			if (isp_dev->pipe[i]) {
				CSI_PL_CTX_t *ctx = (CSI_PL_CTX_t *)isp_dev->pipe[i];
				ctx->wb_en = ctrl->val;

				/* Configure white balance module */
				wb_ctrl.id = CID_WB_CONFIG;
				wb_ctrl.handler = ctx;
				wb_ctrl.cfg = &isp_dev->wb_config;
				ret = wb_s_ctrl(&wb_ctrl);
				if (ret) {
					dev_err(isp_dev->dev, "Failed to configure WB module: %d\n", ret);
					break;
				}

				/* Enable/disable white balance module */
				wb_ctrl.id = CID_WB_ENABLE;
				wb_ctrl.handler = ctx;
				wb_ctrl.cfg = &isp_dev->wb_config;
				ret = wb_s_ctrl(&wb_ctrl);
				if (ret) {
					dev_err(isp_dev->dev, "Failed to %s WB module: %d\n",
						   ctrl->val ? "enable" : "disable", ret);
					break;
				}

				dev_info(isp_dev->dev, "WB module %s for pipe %d\n",
						ctrl->val ? "enabled" : "disabled", i);
			}
		}
		break;
	default:
		dev_err(isp_dev->dev, "Unsupported control ID: 0x%x\n", ctrl->id);
		return -EINVAL;
	}
	return ret;
}

static int camera_isp_ctrl_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct camera_isp_dev *isp_dev = container_of(ctrl->handler,
			struct camera_isp_dev, ctrl_handler);
	switch (ctrl->id) {
	case V4L2_CID_USER_WB_ENABLE:
		ctrl->val = isp_dev->wb_config.wb_en;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops wb_enable_ctrl_ops = {
	.s_ctrl = camera_isp_ctrl_s_ctrl,
	.g_volatile_ctrl = camera_isp_ctrl_g_volatile_ctrl,
};

static int camera_isp_link_setup(struct media_entity *entity,
								const struct media_pad *local,
								const struct media_pad *remote, u32 flags)
{
	pr_info("[ENTITY_OPS] link_setup called for %s pad %u -> %s pad %u\n",
			entity->name, local->index, remote->entity->name, remote->index);
	return 0;
}

static const struct media_entity_operations camera_isp_entity_ops = {
	.link_setup = camera_isp_link_setup,
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

static int camera_isp_notifier_bound(struct v4l2_async_notifier *notifier,
			struct v4l2_subdev *sd, struct v4l2_async_connection *asc)
{
	struct camera_isp_dev *isp_dev = container_of(notifier, struct camera_isp_dev, notifier);
	struct device *dev = isp_dev->dev;
	struct fwnode_handle *ep = NULL;
	struct v4l2_fwnode_link link;
	struct media_entity *source, *sink;
	unsigned int source_pad, sink_pad;
	int ret = 0;
	while (1) {
		ep = fwnode_graph_get_next_endpoint(sd->fwnode, ep);
		if (!ep)
			break;
		ret = v4l2_fwnode_parse_link(ep, &link);
		if (ret < 0) {
			dev_err(dev, "failed to parse link for %pOF: %d\n", to_of_node(ep), ret);
			continue;
		}
		if (sd->entity.pads[link.local_port].flags == MEDIA_PAD_FL_SINK)
			continue;
		source	   = &sd->entity;
		sink	   = &isp_dev->sd.entity;
		source_pad = link.local_port;
		sink_pad   = link.remote_port;
		v4l2_fwnode_put_link(&link);
		if (!source || !sink) {
			dev_err(dev, "media_create_pad_link: source or sink entity is NULL!\n");
			ret = -EINVAL;
			break;
		}
		if (!source->pads || !sink->pads) {
			dev_err(dev, "media_create_pad_link: source or sink pads are NULL!\n");
			ret = -EINVAL;
			break;
		}
		dev_info(sd->dev, "%s: linking %s source_pad %d flags %ld and %s sink_pad %d flags %ld\n",
				__func__, source->name, source_pad, source->pads[source_pad].flags,
				sink->name, sink_pad, sink->pads[sink_pad].flags);
		ret = media_create_pad_link(source, source_pad, sink, sink_pad, MEDIA_LNK_FL_ENABLED);
		if (ret) {
			dev_err(dev, "failed to create %s:%u -> %s:%u link\n",
				source->name, source_pad, sink->name, sink_pad);
			break;
		}
	}
	fwnode_handle_put(ep);
	return ret;
}

static void camera_isp_notifier_unbound(struct v4l2_async_notifier *notifier,
		struct v4l2_subdev *sd, struct v4l2_async_connection *asc)
{
	return;
}

static const struct v4l2_async_notifier_operations camera_isp_notify_ops = {
	.bound = camera_isp_notifier_bound,
	.unbind = camera_isp_notifier_unbound,
};

/* Supported media bus formats for main path */
struct camera_isp_mbus_fmt camera_isp_mp_fmts[] = {
	{ .code = MEDIA_BUS_FMT_YUYV8_2X8 },   /* NV16 */
	{ .code = MEDIA_BUS_FMT_YUYV8_1_5X8 }, /* NV12 */
	{ .code = MEDIA_BUS_FMT_YUYV8_1X16 },  /* YUYV */
	{ .code = MEDIA_BUS_FMT_UYVY8_1X16 },  /* UYVY */
	{ .code = MEDIA_BUS_FMT_RGB888_3X8 },  /* RGB24 */
	{ .code = MEDIA_BUS_FMT_BGR888_3X8 },  /* BGR24 */
	{ .code = MEDIA_BUS_FMT_SBGGR8_1X8 },  /*  BGGR 8*/
	{ .code = MEDIA_BUS_FMT_SGRBG8_1X8 },  /*  GRBG 8*/
	{ .code = MEDIA_BUS_FMT_SGRBG10_1X10},	/* GRBG 10 */
	{ .code = MEDIA_BUS_FMT_SGBRG10_1X10},	/* GBRG 10 */
	{ .code = MEDIA_BUS_FMT_SRGGB10_1X10},	/* RGGB 10 */
	{ .code = MEDIA_BUS_FMT_SBGGR10_1X10},	/* BGGR 10 */
};

/* Supported media bus formats for self path */
struct camera_isp_mbus_fmt camera_isp_sp_fmts[] = {
	{ .code = MEDIA_BUS_FMT_YUYV8_2X8 },   /* NV16 */
	{ .code = MEDIA_BUS_FMT_YUYV8_1_5X8 }, /* NV12 */
	{ .code = MEDIA_BUS_FMT_YUYV8_1X16 },  /* YUYV */
	{ .code = MEDIA_BUS_FMT_UYVY8_1X16 },  /* UYVY */
	{ .code = MEDIA_BUS_FMT_RGB888_3X8 },  /* RGB24 */
	{ .code = MEDIA_BUS_FMT_BGR888_3X8 },  /* BGR24 */
	{ .code = MEDIA_BUS_FMT_SBGGR8_1X8 },  /*  BGGR 8*/
	{ .code = MEDIA_BUS_FMT_SGRBG8_1X8 },  /*  GRBG 8*/
	{ .code = MEDIA_BUS_FMT_SGRBG10_1X10},	/* GRBG 10 */
	{ .code = MEDIA_BUS_FMT_SGBRG10_1X10},	/* GBRG 10 */
	{ .code = MEDIA_BUS_FMT_SRGGB10_1X10},	/* RGGB 10 */
	{ .code = MEDIA_BUS_FMT_SBGGR10_1X10},	/* RGGR 10 */
};

static int camera_isp_subscribe_event(struct v4l2_subdev *sd,
				struct v4l2_fh *fh,
				struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_FRAME_SYNC:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return -EINVAL;
	}
}

static int camera_isp_buf_queue(struct v4l2_subdev *sd, void *arg)
{
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);
	struct camera_pad_buf *pad_buf = (struct camera_pad_buf *)arg;
	unsigned long flags;

	CSI_PL_CTX_t *ctx = (CSI_PL_CTX_t *)isp_dev->pipe[pad_buf->pad - 1];

	spin_lock_irqsave(&ctx->buf.lock, flags);
	list_add_tail(&pad_buf->buf->list, &ctx->buf.queue);
	pr_debug("Add addr: 0x%x ctx: %d\n", pad_buf->buf->planes[0].dma_addr, ctx->id);
	spin_unlock_irqrestore(&ctx->buf.lock, flags);

	return 0;
}

static int camera_isp_s_stream(struct v4l2_subdev *sd, void *arg)
{
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);
	struct camera_pad_stream_status *pad_stream = (struct camera_pad_stream_status *)arg;
	struct v4l2_subdev *subdev;
	struct media_pad *pad;
	int ret = 0;

	//TODO pipe index calculation
	if (pad_stream->status) {
		/* Apply customized values from user when stream starts. */
		ret =  v4l2_ctrl_handler_setup(sd->ctrl_handler);
		if (ret)
			return ret;

		CSI_PIPE_Start(isp_dev->pipe[pad_stream->pad - 1]);
	} else {
		CSI_PIPE_Stop(isp_dev->pipe[pad_stream->pad - 1]);
	}

	if ((!pad_stream->status && !isp_dev->streaming) || pad_stream->status) {
		// CSI subdev s_power propagation
		//TODO Sink pad calculation should be neat
		pad = media_pad_remote_pad_first(&isp_dev->pads[CAMERA_ISP_PAD_SINK]);
		if (pad && is_media_entity_v4l2_subdev(pad->entity)) {
			subdev = media_entity_to_v4l2_subdev(pad->entity);
			ret = v4l2_subdev_call(subdev, core, s_power, pad_stream->status);
		}
	}

	return ret;
}

static int camera_isp_ioctl_g_ctrl(struct camera_isp_dev *isp_dev,
		struct v4l2_subdev *sd, void *arg)
{
	struct camera_pad_control *pad_ctrl = (struct camera_pad_control *)arg;
	if (pad_ctrl && pad_ctrl->control &&
		pad_ctrl->control->id == V4L2_CID_USER_WB_ENABLE) {
		struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&isp_dev->ctrl_handler,
				V4L2_CID_USER_WB_ENABLE);
		if (!ctrl) {
			dev_err(isp_dev->dev, "WB control not found in ISP handler\n");
			return -EINVAL;
		}
		pad_ctrl->control->value = ctrl->cur.val;
		dev_dbg(isp_dev->dev, "ISP G_CTRL: wb_enable=%d\n", ctrl->cur.val);
		return 0;
	} else {
		dev_err(isp_dev->dev, "Invalid G_CTRL parameters\n");
		return -EINVAL;
	}
}
static int camera_isp_ioctl_s_ctrl(struct camera_isp_dev *isp_dev,
		struct v4l2_subdev *sd, void *arg)
{
	struct camera_pad_control *pad_ctrl = (struct camera_pad_control *)arg;
	if (pad_ctrl && pad_ctrl->control &&
		pad_ctrl->control->id == V4L2_CID_USER_WB_ENABLE) {
		struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&isp_dev->ctrl_handler,
				V4L2_CID_USER_WB_ENABLE);
		if (!ctrl) {
			dev_err(isp_dev->dev, "WB control not found in ISP handler\n");
			return -EINVAL;
		}
		int ret = v4l2_ctrl_s_ctrl(ctrl, pad_ctrl->control->value);
		dev_dbg(isp_dev->dev, "ISP S_CTRL: wb_enable=%d, ret=%d\n",
				pad_ctrl->control->value, ret);
		return ret;
	} else {
		dev_err(isp_dev->dev, "Invalid S_CTRL parameters\n");
		return -EINVAL;
	}
}
static int camera_isp_ioctl_g_ext_ctrls(struct camera_isp_dev *isp_dev,
		struct v4l2_subdev *sd, void *arg)
{
	  struct camera_pad_ext_controls *pad_ext_ctrls = arg;

	  return v4l2_g_ext_ctrls(&isp_dev->ctrl_handler, sd->devnode,
				sd->v4l2_dev->mdev, pad_ext_ctrls->ext_controls);
}
static int camera_isp_ioctl_s_ext_ctrls(struct camera_isp_dev *isp_dev,
		struct v4l2_subdev *sd, void *arg)
{
	  struct camera_pad_ext_controls *pad_ext_ctrls = arg;

	  return v4l2_s_ext_ctrls(NULL, &isp_dev->ctrl_handler, sd->devnode,
				sd->v4l2_dev->mdev, pad_ext_ctrls->ext_controls);
}
static int camera_isp_ioctl_try_ext_ctrls(struct camera_isp_dev *isp_dev,
		struct v4l2_subdev *sd, void *arg)
{
	  struct camera_pad_ext_controls *pad_ext_ctrls = arg;

	  return v4l2_try_ext_ctrls(&isp_dev->ctrl_handler, sd->devnode,
			   sd->v4l2_dev->mdev, pad_ext_ctrls->ext_controls);
}
static int camera_isp_ioctl_queryctrl(struct camera_isp_dev *isp_dev, void *arg) {
	struct camera_pad_queryctrl *pad_query_ctrl = arg;
	return v4l2_queryctrl(&isp_dev->ctrl_handler, pad_query_ctrl->query_ctrl);
}
static int camera_isp_ioctl_query_ext_ctrl(struct camera_isp_dev *isp_dev, void *arg) {
	struct camera_pad_query_ext_ctrl *pad_query_ext_ctrl = arg;
	return v4l2_query_ext_ctrl(&isp_dev->ctrl_handler, pad_query_ext_ctrl->query_ext_ctrl);
}
static int camera_isp_ioctl_querymenu(struct camera_isp_dev *isp_dev, void *arg) {
	struct camera_pad_querymenu *pad_querymenu = arg;
	return v4l2_querymenu(&isp_dev->ctrl_handler, pad_querymenu->querymenu);
}

static int camera_isp_querycap(struct v4l2_subdev *sd, void *arg)
{
	struct v4l2_capability *cap = (struct v4l2_capability *)arg;

	strncpy(cap->driver, sd->name, sizeof(cap->driver));
	strncpy(cap->card, sd->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", sd->name);

	return 0;
}

static long camera_isp_priv_ioctl(struct v4l2_subdev *sd,
								 unsigned int cmd, void *arg)
{
	int ret = -EINVAL;
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);
	if (!isp_dev)
		return -ENODEV;

	switch (cmd) {
	case CAMERA_ISP_IOC_QUERYCAP:
		return camera_isp_querycap(sd, arg);
	case CAMERA_PAD_G_CTRL:
		ret = camera_isp_ioctl_g_ctrl(isp_dev, sd, arg);
		break;
	case CAMERA_PAD_S_CTRL:
		ret = camera_isp_ioctl_s_ctrl(isp_dev, sd, arg);
		break;
	case CAMERA_PAD_G_EXT_CTRLS:
		ret = camera_isp_ioctl_g_ext_ctrls(isp_dev, sd, arg);
		break;
	case CAMERA_PAD_S_EXT_CTRLS:
		ret = camera_isp_ioctl_s_ext_ctrls(isp_dev, sd, arg);
		break;
	case CAMERA_PAD_TRY_EXT_CTRLS:
		ret = camera_isp_ioctl_try_ext_ctrls(isp_dev, sd, arg);
		break;
	case CAMERA_PAD_QUERYCTRL:
		ret = camera_isp_ioctl_queryctrl(isp_dev, arg);
		break;
	case CAMERA_PAD_QUERY_EXT_CTRL:
		ret = camera_isp_ioctl_query_ext_ctrl(isp_dev, arg);
		break;
	case CAMERA_PAD_QUERYMENU:
		ret = camera_isp_ioctl_querymenu(isp_dev, arg);
		break;
	case CAMERA_PAD_S_STREAM:
		ret = camera_isp_s_stream(sd, arg);
		break;
	case CAMERA_PAD_BUF_QUEUE:
		ret = camera_isp_buf_queue(sd, arg);
		break;
	default:
		dev_err(isp_dev->dev, "Unsupported ioctl: 0x%x\n", cmd);
		ret = -ENOTTY;
	}

	return ret;
}

static struct v4l2_subdev_core_ops camera_isp_core_ops = {
	.ioctl = camera_isp_priv_ioctl,
	.subscribe_event = camera_isp_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

/* V4L2 subdev selection operations */
static int camera_isp_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);
	CSI_PL_CTX_t *ctx;

	if (sel->pad >= CAMERA_ISP_PAD_NR) {
		dev_err(isp_dev->dev, "Invalid pad %d for get_selection\n", sel->pad);
		return -EINVAL;
	}

	/* Only support crop on source pads */
	if (sel->pad == CAMERA_ISP_PAD_SINK) {
		dev_err(isp_dev->dev, "Selection not supported on sink pad\n");
		return -EINVAL;
	}

	ctx = (CSI_PL_CTX_t *)isp_dev->pipe[sel->pad - 1];
	if (!ctx) {
		dev_err(isp_dev->dev, "Pipeline context not available for pad %d\n", sel->pad);
		return -EINVAL;
	}

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		/* Full input frame bounds */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = ctx->hres;
		sel->r.height = ctx->vres;
		break;

	case V4L2_SEL_TGT_CROP:
		/* Current crop rectangle */
		sel->r.left = ctx->crop.x_st;
		sel->r.top = ctx->crop.y_st;
		sel->r.width = ctx->crop.x_end - ctx->crop.x_st + 1;
		sel->r.height = ctx->crop.y_end - ctx->crop.y_st + 1;
		break;

	default:
		dev_err(isp_dev->dev, "Unsupported selection target %d\n", sel->target);
		return -EINVAL;
	}

	dev_dbg(isp_dev->dev, "get_selection pad %d target %d: (%d,%d)/%dx%d\n",
			sel->pad, sel->target, sel->r.left, sel->r.top,
			sel->r.width, sel->r.height);

	return 0;
}

static int camera_isp_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);
	CSI_PL_CTX_t *ctx;
	u32 max_width, max_height;

	if (sel->pad >= CAMERA_ISP_PAD_NR) {
		dev_err(isp_dev->dev, "Invalid pad %d for set_selection\n", sel->pad);
		return -EINVAL;
	}

	/* Only support crop on source pads */
	if (sel->pad == CAMERA_ISP_PAD_SINK) {
		dev_err(isp_dev->dev, "Selection not supported on sink pad\n");
		return -EINVAL;
	}

	/* Only support crop target */
	if (sel->target != V4L2_SEL_TGT_CROP) {
		dev_err(isp_dev->dev, "Unsupported selection target %d\n", sel->target);
		return -EINVAL;
	}

	ctx = (CSI_PL_CTX_t *)isp_dev->pipe[sel->pad - 1];
	if (!ctx) {
		dev_err(isp_dev->dev, "Pipeline ctx not available for pad %d\n", sel->pad);
		return -EINVAL;
	}

	/* Get input frame bounds for validation */
	max_width = ctx->hres;
	max_height = ctx->vres;

	/* Validate and clamp crop rectangle */
	sel->r.left = clamp_t(u32, sel->r.left, 0, max_width - 1);
	sel->r.top = clamp_t(u32, sel->r.top, 0, max_height - 1);
	sel->r.width = clamp_t(u32, sel->r.width, CAMERA_ISP_WIDTH_MIN,
						  max_width - sel->r.left);
	sel->r.height = clamp_t(u32, sel->r.height, CAMERA_ISP_HEIGHT_MIN,
						   max_height - sel->r.top);

	/* Align dimensions */
	sel->r.width = ALIGN(sel->r.width, CAMERA_ISP_WIDTH_ALIGN);
	sel->r.height = ALIGN(sel->r.height, CAMERA_ISP_HEIGHT_ALIGN);

	/* Ensure crop rectangle doesn't exceed bounds after alignment */
	if (sel->r.left + sel->r.width > max_width) {
		sel->r.width = max_width - sel->r.left;
		sel->r.width = ALIGN_DOWN(sel->r.width, CAMERA_ISP_WIDTH_ALIGN);
	}
	if (sel->r.top + sel->r.height > max_height) {
		sel->r.height = max_height - sel->r.top;
		sel->r.height = ALIGN_DOWN(sel->r.height, CAMERA_ISP_HEIGHT_ALIGN);
	}

	/* Update crop window in pipeline context */
	ctx->crop.x_st = sel->r.left;
	ctx->crop.y_st = sel->r.top;
	ctx->crop.x_end = sel->r.left + sel->r.width - 1;
	ctx->crop.y_end = sel->r.top + sel->r.height - 1;

	/* Recalculate scale factor based on new crop and current output format */
	if (ctx->op_wt > 0 && ctx->op_ht > 0) {
		u32 crop_width = sel->r.width;
		u32 crop_height = sel->r.height;
		u32 scale_x = crop_width / ctx->op_wt;
		u32 scale_y = crop_height / ctx->op_ht;

		ctx->crop.scale = (scale_x > scale_y) ? scale_x : scale_y;
		if (ctx->crop.scale < 1) ctx->crop.scale = 1;
		ctx->crop.imgres_oprn = (ctx->crop.scale > 1) ? 1 : 0;	/* 1=scaling, 0=cropping */
	}

	/* Apply crop settings to hardware registers through pipeline reconfiguration */
	/* The crop parameters are now stored in ctx->crop and will be applied
	 * during the next CSI_PIPE_Config call when format is set or streaming starts */

	dev_info(isp_dev->dev, "%s crop (%d,%d)/%dx%d -> (%d,%d)-(%d,%d) scale=%d oprn=%d\n",
			 __func__, sel->r.left, sel->r.top, sel->r.width, sel->r.height,
			 ctx->crop.x_st, ctx->crop.y_st, ctx->crop.x_end, ctx->crop.y_end,
			 ctx->crop.scale, ctx->crop.imgres_oprn);

	return 0;
}

/* V4L2 subdev pad operations */
static int camera_isp_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *format)
{
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);
	//struct v4l2_mbus_framefmt *fmt;
	struct camera_isp_mbus_fmt *supported_fmts;
	int num_fmts;
	int i;
	struct v4l2_subdev_format sd_fmt = {
		.which = format->which,
		.pad = 0,
		.format = {
			.width = format->format.width,
			.height = format->format.height,
		}
	};

	if (format->pad >= CAMERA_ISP_PAD_NR){
		pr_err("%s %d error !!\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* Get supported formats based on pad */
	if (format->pad == CAMERA_ISP_PAD_SOURCE_PATH0 ||
		format->pad == CAMERA_ISP_PAD_SOURCE_PATH1) {
		supported_fmts = camera_isp_mp_fmts;
		num_fmts = ARRAY_SIZE(camera_isp_mp_fmts);
	} else {
		supported_fmts = camera_isp_sp_fmts;
		num_fmts = ARRAY_SIZE(camera_isp_sp_fmts);
	}

	/* Validate format */
	for (i = 0; i < num_fmts; i++) {
		if (supported_fmts[i].code == format->format.code)
			break;
	}

	if (i >= num_fmts) {
		/* Use first supported format as default */
		format->format.code = supported_fmts[0].code;
	}

	/* Clamp dimensions */
	format->format.width = clamp_t(u32, format->format.width,
			CAMERA_ISP_WIDTH_MIN, CAMERA_ISP_WIDTH_MAX);
	format->format.height = clamp_t(u32, format->format.height,
			CAMERA_ISP_HEIGHT_MIN, CAMERA_ISP_HEIGHT_MAX);

	/* Align dimensions */
	format->format.width = ALIGN(format->format.width, CAMERA_ISP_WIDTH_ALIGN);
	format->format.height = ALIGN(format->format.height, CAMERA_ISP_HEIGHT_ALIGN);

	dev_dbg(isp_dev->dev, "request format pad %d: %dx%d, code 0x%x\n",
			format->pad, format->format.width, format->format.height,
			format->format.code);

	CSI_PIPE_Set_Fmt(isp_dev->pipe[format->pad - 1], &format->format);

	// CSI subdev integration
	struct v4l2_subdev *subdev;
	struct media_pad *pad;
	//TODO Sink pad calculation should be neat
	pad = media_pad_remote_pad_first(&isp_dev->pads[CAMERA_ISP_PAD_SINK]);
	/* Get the format that the CSI supports. Same will be applied in the
	 * Pipe_Config */
	if (pad && is_media_entity_v4l2_subdev(pad->entity)) {
		sd_fmt.pad = pad->index;
		subdev = media_entity_to_v4l2_subdev(pad->entity);

		/* Use sensor driver function for production */
		camera_isp_set_sensor_resolution_and_program(isp_dev, subdev, sd_state,
				&sd_fmt, format->format.code);
	}
	dev_dbg(isp_dev->dev, "sd format pad %d: %dx%d, code 0x%x\n",
			sd_fmt.pad, sd_fmt.format.width, sd_fmt.format.height,
			sd_fmt.format.code);


	// Always set output format for both normal and scaling pipelines
	CSI_PIPE_Set_Output_Fmt(isp_dev->pipe[format->pad - 1],
			sd_fmt.format.width, sd_fmt.format.height);
	dev_dbg(isp_dev->dev, "Set output format: %dx%d\n",
			format->format.width, format->format.height);

	CSI_PIPE_Config(isp_dev->pipe[format->pad - 1], sd_fmt.format.code);

	return 0;
}

static int camera_isp_get_fmt(struct v4l2_subdev *sd,
							 struct v4l2_subdev_state *sd_state,
							 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);

	if (format->pad >= CAMERA_ISP_PAD_NR){
		pr_err("%s %d error !!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_state_get_format(sd_state, format->pad);
	else
		fmt = &isp_dev->formats[format->pad];

	format->format = *fmt;

	return 0;
}

static int camera_isp_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct camera_isp_mbus_fmt *supported_fmts;
	int num_fmts;

	if (code->pad >= CAMERA_ISP_PAD_NR){
		pr_err("%s %d error !!\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* Get supported formats based on pad */
	if (code->pad == CAMERA_ISP_PAD_SOURCE_PATH0 ||
		code->pad == CAMERA_ISP_PAD_SOURCE_PATH1) {
		supported_fmts = camera_isp_mp_fmts;
		num_fmts = ARRAY_SIZE(camera_isp_mp_fmts);
	} else {
		supported_fmts = camera_isp_sp_fmts;
		num_fmts = ARRAY_SIZE(camera_isp_sp_fmts);
	}

	if (code->index >= num_fmts) {
		pr_err("%s %d error !!\n", __func__, __LINE__);
		return -EINVAL;
	}

	code->code = supported_fmts[code->index].code;

	return 0;
}

static const struct v4l2_subdev_pad_ops camera_isp_pad_ops = {
	.set_fmt = camera_isp_set_fmt,
	.get_fmt = camera_isp_get_fmt,
	.enum_mbus_code = camera_isp_enum_mbus_code,
	.get_selection = camera_isp_get_selection,
	.set_selection = camera_isp_set_selection,
};

static struct v4l2_subdev_ops camera_isp_subdev_ops = {
	.core = &camera_isp_core_ops,
	/*.video = &camera_isp_video_ops,*/
	.pad = &camera_isp_pad_ops,
};

/* V4L2 subdev internal operations */
static int camera_isp_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	int i;

	/* Initialize default formats for all pads */
	for (i = 0; i < CAMERA_ISP_PAD_NR; i++) {
		//format = v4l2_subdev_get_try_format(sd, fh->state, i);
		format = v4l2_subdev_state_get_format(fh->state, i);
		format->width = CAMERA_ISP_DEFAULT_WIDTH;
		format->height = CAMERA_ISP_DEFAULT_HEIGHT;
		format->field = V4L2_FIELD_NONE;
		format->colorspace = V4L2_COLORSPACE_DEFAULT;

		if (i == CAMERA_ISP_PAD_SOURCE_PATH0 || i == CAMERA_ISP_PAD_SOURCE_PATH1)
			format->code = camera_isp_mp_fmts[0].code;
		else
			format->code = camera_isp_sp_fmts[0].code;

		/* Initialize default crop rectangles for source pads */
		if (i != CAMERA_ISP_PAD_SINK) {
			struct v4l2_rect *crop;
			crop = v4l2_subdev_state_get_crop(fh->state, i);
			crop->left = 0;
			crop->top = 0;
			crop->width = CAMERA_ISP_DEFAULT_WIDTH;
			crop->height = CAMERA_ISP_DEFAULT_HEIGHT;
		}
	}

	dev_dbg(isp_dev->dev, "ISP subdev opened\n");
	return 0;
}

static int camera_isp_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct camera_isp_dev *isp_dev = v4l2_get_subdevdata(sd);

	dev_dbg(isp_dev->dev, "ISP subdev closed\n");
	return 0;
}

static const struct v4l2_subdev_internal_ops camera_isp_internal_ops = {
	.open = camera_isp_open,
	.close = camera_isp_close,
};

static int camera_isp_register_async_notifier(struct camera_isp_dev *isp_dev)
{
	int ret;
	int pad;
	struct fwnode_handle *ep, *remote_ep;

	v4l2_async_subdev_nf_init(&isp_dev->notifier, &isp_dev->sd);

	isp_dev->notifier.ops = &camera_isp_notify_ops;
	if (dev_fwnode(isp_dev->dev) != NULL) {
		for (pad = 0; pad < CAMERA_ISP_PAD_NR; pad++) {

			if (isp_dev->pads[pad].flags != MEDIA_PAD_FL_SINK)
				continue;

			ep = fwnode_graph_get_endpoint_by_id(
					dev_fwnode(isp_dev->dev),
					pad, 0, FWNODE_GRAPH_ENDPOINT_NEXT);
			if (!ep) {
				continue;
			}

			remote_ep = fwnode_graph_get_remote_endpoint(ep);

			if (!remote_ep) {
				dev_info(isp_dev->dev, "No remote endpoint, sink pad %d\n", pad);
				fwnode_handle_put(ep);
				continue;
			}

			if (ep && remote_ep) {
				v4l2_async_nf_add_fwnode_remote(&isp_dev->notifier, ep,
						struct v4l2_async_connection);
				dev_dbg(isp_dev->dev, "Registered async notifier\n");
			} else {
				dev_err(isp_dev->dev, "Skipping async notifier, sink pad %d\n", pad);
			}

			fwnode_handle_put(remote_ep);
			fwnode_handle_put(ep);
		}
	}
	// Register notifier
	ret = v4l2_async_nf_register(&isp_dev->notifier);

	if (ret) {
		dev_err(isp_dev->dev, "Async notifier register error\n");
		v4l2_async_nf_cleanup(&isp_dev->notifier);
	}

	return ret;
}

static void camera_isp_unregister_async_notifier(struct camera_isp_dev *isp_dev)
{
	v4l2_async_nf_unregister(&isp_dev->notifier);
	v4l2_async_nf_cleanup(&isp_dev->notifier);
}

/* Platform driver operations */
static void parse_wb_config_from_dt(struct device_node *node, WB_CONFIG_t *cfg)
{
	struct device_node *wb_np = of_get_child_by_name(node, "white-balance-config");
	if (wb_np) {
		of_property_read_u32(wb_np, "wb-mode", &cfg->wb_mode);
		of_property_read_u32(wb_np, "wb-p00-mantissa", &cfg->wb_p00_mantissa);
		of_property_read_u32(wb_np, "wb-p00-exponent", &cfg->wb_p00_exponent);
		of_property_read_u32(wb_np, "wb-p01-mantissa", &cfg->wb_p01_mantissa);
		of_property_read_u32(wb_np, "wb-p01-exponent", &cfg->wb_p01_exponent);
		of_property_read_u32(wb_np, "wb-p10-mantissa", &cfg->wb_p10_mantissa);
		of_property_read_u32(wb_np, "wb-p10-exponent", &cfg->wb_p10_exponent);
		of_property_read_u32(wb_np, "wb-p11-mantissa", &cfg->wb_p11_mantissa);
		of_property_read_u32(wb_np, "wb-p11-exponent", &cfg->wb_p11_exponent);
		of_property_read_u32(wb_np, "input-sel", &cfg->input_sel);
		of_node_put(wb_np);

		/* Enable WB by default */
		cfg->wb_en = 1;
	} else {
		memset(cfg, 0, sizeof(*cfg));
	}
}

static int isp_fetch_clocks(struct device *dev, struct clk **isp_clks)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp_clock_list); i++) {
		isp_clks[i] = devm_clk_get(dev, isp_clock_list[i]);
		if (IS_ERR(isp_clks[i])) {
			dev_err(dev, "failed to get %s ...!\n", isp_clock_list[i]);
			return PTR_ERR(isp_clks[i]);
		}
	}

	return 0;
}

static int isp_enable_clocks(struct device *dev, struct clk **isp_clks)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(isp_clock_list); i++) {
		ret = clk_prepare_enable(isp_clks[i]);
		if (ret < 0) {
			dev_err(dev, "%s prepare failed..!\n", isp_clock_list[i]);
			goto prepare_failure;
		}
	}
	return 0;
prepare_failure:
	while (--i >= 0)
		clk_disable_unprepare(isp_clks[i]);

	return ret;
}

static int camera_isp_parse_dt(struct camera_isp_dev *isp_dev,
							   struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	struct device *dev = &pdev->dev;

	// Parse WB config from DT
	parse_wb_config_from_dt(node, &isp_dev->wb_config);

	if (!node) {
		dev_err(dev, "No device tree node found\n");
		return -EINVAL;
	}

	/* Parse device ID */
	if (of_property_read_u32(node, "id", &isp_dev->id))
		isp_dev->id = 0;

	/* Fetch IORESOURCE_MEM for core_base_addr */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "No core IORESOURCE_MEM found\n");
		return -ENODEV;
	}
	isp_dev->core_base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(isp_dev->core_base_addr)) {
		dev_err(dev, "Failed to ioremap core resource\n");
		return PTR_ERR(isp_dev->core_base_addr);
	}

	/* Fetch second IORESOURCE_MEM for dhub_base_addr */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		isp_dev->dhub_base_addr = devm_ioremap_resource(dev, res);
		if (IS_ERR(isp_dev->dhub_base_addr)) {
			dev_err(dev, "Failed to ioremap dhub resource\n");
			return PTR_ERR(isp_dev->dhub_base_addr);
		}
	} else {
		isp_dev->dhub_base_addr = NULL;
	}

	/* Fetch IRQ */
	isp_dev->irq_num = platform_get_irq(pdev, 0);
	if (isp_dev->irq_num < 0) {
		dev_err(dev, "Failed to get IRQ\n");
		return isp_dev->irq_num;
	}

	return 0;
}

static void camera_isp_reset(struct reset_control *rst)
{
	reset_control_assert(rst);
	udelay(1);
	reset_control_deassert(rst);
}

static int camera_isp_probe(struct platform_device *pdev)
{
	struct camera_isp_dev *isp_dev;
	struct v4l2_subdev *sd;
	struct media_pad *pads;
	struct reset_control *rst;
	int ret;
	int i;
	struct device *dev = &pdev->dev;

	dev_info(dev, "Camera ISP subdevice probe started\n");

	isp_dev = devm_kzalloc(dev, sizeof(*isp_dev), GFP_KERNEL);
	if (!isp_dev) {
		dev_err(dev, "failed to allocate memory for isp_dev...!\n");
		return -ENOMEM;
	}

	isp_dev->dev = dev;
	platform_set_drvdata(pdev, isp_dev);

	isp_dev->isp_clks = devm_kzalloc(dev,
		sizeof(struct clk *) * ARRAY_SIZE(isp_clock_list), GFP_KERNEL);
	if (IS_ERR(isp_dev->isp_clks)) {
		dev_err(dev, "failed to allocate memory for isp_clocks...!\n");
		return PTR_ERR(isp_dev->isp_clks);
	}

	rst = devm_reset_control_get_optional(&pdev->dev, "isprst");
	if (IS_ERR(rst) && PTR_ERR(rst) == -EPROBE_DEFER) {
		dev_err(dev, "isprst reset failed...!\n");
		return -EPROBE_DEFER;
	}

	camera_isp_reset(rst);

	/* Fetch Clk */
	ret = isp_fetch_clocks(dev, isp_dev->isp_clks);
	if (ret) {
		dev_err(dev, "isp clock fetch failed...!\n");
		return ret;
	}

	/* Parse device tree */
	ret = camera_isp_parse_dt(isp_dev, pdev);
	if (ret)
		return ret;

	ret = isp_enable_clocks(dev, isp_dev->isp_clks);
	if (ret) {
		dev_err(dev, "isp clock enable failed...!\n");
		return ret;
	}

	/* Initialize V4L2 subdevice */
	sd = &isp_dev->sd;
	v4l2_subdev_init(sd, &camera_isp_subdev_ops);

	snprintf(sd->name, sizeof(sd->name), "%s.%d", CAMERA_ISP_NAME, isp_dev->id);
	sd->internal_ops = &camera_isp_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->dev =  dev;
	sd->owner = THIS_MODULE;
	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	sd->entity.obj_type = MEDIA_ENTITY_TYPE_V4L2_SUBDEV;
	sd->entity.name = sd->name;

	v4l2_set_subdevdata(sd, isp_dev);

	/* Initialize media entity pads */
	pads = isp_dev->pads;
	pads[CAMERA_ISP_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[CAMERA_ISP_PAD_SOURCE_PATH0].flags = MEDIA_PAD_FL_SOURCE;
	pads[CAMERA_ISP_PAD_SOURCE_PATH1].flags = MEDIA_PAD_FL_SOURCE;

	for (i = 0; i < CAMERA_ISP_PAD_NR; i++) {
		isp_dev->formats[i].code = camera_isp_mp_fmts[0].code;
		/* Initialize to default resolution from header (supports 1080p by default).
		 * 640x480 and other sizes can be selected later via set_fmt.
		 */
		isp_dev->formats[i].width = CAMERA_ISP_DEFAULT_WIDTH;
		isp_dev->formats[i].height = CAMERA_ISP_DEFAULT_HEIGHT;
	}

	ret = media_entity_pads_init(&sd->entity, CAMERA_ISP_PAD_NR, pads);
	if (ret) {
		dev_err(dev, "Failed to initialize media entity pads: %d\n", ret);
		goto err_cleanup_clk;
	}

	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;
	sd->entity.ops = &camera_isp_entity_ops;

	ret = camera_isp_register_async_notifier(isp_dev);
	if (ret) {
		dev_err(dev, "Failed to register async notifier: %d\n", ret);
		goto err_cleanup_entity;
	}

	/* Register V4L2 async subdevice */
	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(dev, "Failed to register async subdev: %d\n", ret);
		goto err_cleanup_notifier;
	}

	/* Initialize mutex */
	mutex_init(&isp_dev->lock);

	v4l2_ctrl_handler_init(&isp_dev->ctrl_handler, 1);
	v4l2_ctrl_new_std(&isp_dev->ctrl_handler, &wb_enable_ctrl_ops,
					 V4L2_CID_USER_WB_ENABLE, 0, 1, 1, isp_dev->wb_config.wb_en);
	isp_dev->sd.ctrl_handler = &isp_dev->ctrl_handler;
	if (isp_dev->ctrl_handler.error) {
		dev_err(dev, "Failed to register wb_enable v4l2 control\n");
		ret = isp_dev->ctrl_handler.error;
		goto err_cleanup_async;
	}

	ret = camera_isp_create_wb_sysfs(isp_dev);
	if (ret)
		dev_warn(dev, "Failed to create WB sysfs group\n");


	ret = isp_shm_init(dev);
	if (ret) {
		dev_err(dev, "Failed isp_shm_init: %d\n", ret);
		goto err_cleanup_async;
	}

	ret = CSI_PIPE_Init(isp_dev);
	if (ret) {
		dev_err(dev, "Failed CSI_PIPE_Init: %d\n", ret);
		goto err_cleanup_shm;
	}
	/* Enable runtime PM */
	pm_runtime_enable(dev);

	dev_info(dev, "Camera ISP subdevice registered successfully\n");
	return 0;

err_cleanup_shm:
	isp_shm_deinit(dev);
err_cleanup_async:
	v4l2_async_unregister_subdev(sd);
err_cleanup_notifier:
	camera_isp_unregister_async_notifier(isp_dev);
err_cleanup_entity:
	media_entity_cleanup(&sd->entity);
err_cleanup_clk:
	for (i = 0; i < ARRAY_SIZE(isp_clock_list); i++)
		clk_disable_unprepare(isp_dev->isp_clks[i]);
	return ret;
}

static void camera_isp_remove(struct platform_device *pdev)
{
	struct camera_isp_dev *isp_dev = platform_get_drvdata(pdev);
	int i;

	if (!isp_dev)
		return;

	camera_isp_remove_wb_sysfs(isp_dev);
	camera_isp_unregister_async_notifier(isp_dev);
	CSI_PIPE_Exit(isp_dev);
	isp_shm_deinit(&pdev->dev);
	v4l2_async_unregister_subdev(&isp_dev->sd);
	media_entity_cleanup(&isp_dev->sd.entity);
	for (i = 0; i < ARRAY_SIZE(isp_clock_list); i++)
		clk_disable_unprepare(isp_dev->isp_clks[i]);
	pm_runtime_disable(&pdev->dev);
	mutex_destroy(&isp_dev->lock);
	dev_info(&pdev->dev, "Camera ISP subdevice removed\n");
}

/* Device tree matching */
static const struct of_device_id camera_isp_of_match[] = {
	{ .compatible = "syna,camera-isp" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, camera_isp_of_match);

/* Platform driver */
static struct platform_driver camera_isp_driver = {
	.probe = camera_isp_probe,
	.remove = camera_isp_remove,
	.driver = {
		.name = CAMERA_ISP_NAME,
		.owner = THIS_MODULE,
		.of_match_table = camera_isp_of_match,
		//.pm = &camera_isp_pm_ops,
	},
};

static int __init camera_isp_init_module(void)
{
	int ret;

	ret = platform_driver_register(&camera_isp_driver);
	if (ret) {
		pr_err("Failed to register camera ISP driver: %d\n", ret);
		return ret;
	}

	pr_info("Camera ISP driver initialized\n");
	return 0;
}

static void __exit camera_isp_exit_module(void)
{
	platform_driver_unregister(&camera_isp_driver);
	pr_info("Camera ISP driver exited\n");
}

module_init(camera_isp_init_module);
module_exit(camera_isp_exit_module);

MODULE_DESCRIPTION("Camera ISP V4L2 subdevice driver");
MODULE_AUTHOR("Synaptics Camera Team");
MODULE_LICENSE("GPL");
