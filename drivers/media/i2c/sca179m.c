// SPDX-License-Identifier: GPL-2.0+
/*
 * OSA SCA179M AHD Camera Driver
 *
 */

#include <linux/delay.h>
#include <linux/fwnode.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

struct sca179m_device {
	struct device			*dev;
	struct i2c_client		*ser;
	struct i2c_client		*sensor;
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct v4l2_mbus_framefmt	fmt;
};

static inline struct sca179m_device *sd_to_sca179m(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sca179m_device, sd);
}

static int sca179m_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sca179m_device *dev = sd_to_sca179m(sd);

	/*
	 * Enable serial link now that the ISP provides a valid pixel clock
	 * to start serializing video data on the GMSL link.
	 */
	return 0;
	//return max9271_set_serial_link(&dev->serializer, enable);
}

static int sca179m_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_YUYV8_1X16;

	return 0;
}

static int sca179m_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct sca179m_device *dev = sd_to_sca179m(sd);

	if (format->pad)
		return -EINVAL;

	mf->width		= 1920;
	mf->height		= 1080;
	mf->code		= MEDIA_BUS_FMT_YUYV8_1X16;
	mf->colorspace	= V4L2_COLORSPACE_SRGB;
	mf->field		= V4L2_FIELD_NONE;
	mf->ycbcr_enc		= V4L2_YCBCR_ENC_601;
	mf->quantization	= V4L2_QUANTIZATION_FULL_RANGE;
	mf->xfer_func		= V4L2_XFER_FUNC_NONE;

	return 0;
}

static const struct v4l2_subdev_video_ops sca179m_video_ops = {
	.s_stream	= sca179m_s_stream,
};

static const struct v4l2_subdev_pad_ops sca179m_subdev_pad_ops = {
	.enum_mbus_code = sca179m_enum_mbus_code,
	.get_fmt	= sca179m_get_fmt,
	.set_fmt	= sca179m_get_fmt,
};

static const struct v4l2_subdev_ops sca179m_subdev_ops = {
	.video		= &sca179m_video_ops,
	.pad		= &sca179m_subdev_pad_ops,
};

static int sca179m_probe(struct i2c_client *client)
{
	struct sca179m_device *dev;
	struct fwnode_handle  *ep;
	int ret;

	dev = devm_kzalloc(&client->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = &client->dev;
	dev->sensor = client;

	dev_err(dev->dev, "Start sca179m probe\n");

	/* Initialize and register the subdevice. */
	v4l2_i2c_subdev_init(&dev->sd, client, &sca179m_subdev_ops);
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.flags |= MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&dev->sd.entity, 1, &dev->pad);
	if (ret < 0)
		goto error;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!ep) {
		dev_err(&client->dev,
			"Unable to get endpoint in node %pOF\n",
			client->dev.of_node);
		ret = -ENOENT;
		goto error;
	}
	dev->sd.fwnode = ep;

	ret = v4l2_async_register_subdev(&dev->sd);
	if (ret)
		goto error_put_node;

	return 0;

error_put_node:
	fwnode_handle_put(dev->sd.fwnode);
error:
	i2c_unregister_device(dev->ser);

	return ret;
}

static int sca179m_remove(struct i2c_client *client)
{
	struct sca179m_device *dev = sd_to_sca179m(i2c_get_clientdata(client));

	v4l2_async_unregister_subdev(&dev->sd);
	fwnode_handle_put(dev->sd.fwnode);

	return 0;
}

static const struct of_device_id sca179m_of_ids[] = {
	{ .compatible = "osa,sca179m" },
	{ }
};
MODULE_DEVICE_TABLE(of, sca179m_of_ids);

static struct i2c_driver sca179m_i2c_driver = {
	.driver	= {
		.name	= "sca179m",
		.of_match_table = sca179m_of_ids,
	},
	.probe_new	= sca179m_probe,
	.remove		= sca179m_remove,
};

module_i2c_driver(sca179m_i2c_driver);

MODULE_DESCRIPTION("GMSL Camera driver for SCA179M");
MODULE_LICENSE("GPL v2");
