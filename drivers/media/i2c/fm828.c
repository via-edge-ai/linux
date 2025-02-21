// SPDX-License-Identifier: GPL-2.0+
/*
 * REC FM828 FPD-LinkIII Camera Driver
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

struct fm828_device {
	struct device			*dev;
	struct i2c_client		*ser;
	struct i2c_client		*sensor;
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct v4l2_mbus_framefmt	fmt;
	struct v4l2_ctrl_handler	ctrls;
	u32				addrs[2];
};

static inline struct fm828_device *sd_to_fm828(struct v4l2_subdev *sd)
{
	return container_of(sd, struct fm828_device, sd);
}

static int fm828_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct fm828_device *dev = sd_to_fm828(sd);

	/*
	 * Enable serial link now that the ISP provides a valid pixel clock
	 * to start serializing video data on the GMSL link.
	 */
	return 0;
	//return max9271_set_serial_link(&dev->serializer, enable);
}

static int fm828_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_YUYV8_1X16;

	return 0;
}

static int fm828_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct fm828_device *dev = sd_to_fm828(sd);

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

static const struct v4l2_subdev_video_ops fm828_video_ops = {
	.s_stream	= fm828_s_stream,
};

static const struct v4l2_subdev_pad_ops fm828_subdev_pad_ops = {
	.enum_mbus_code = fm828_enum_mbus_code,
	.get_fmt	= fm828_get_fmt,
	.set_fmt	= fm828_get_fmt,
};

static const struct v4l2_subdev_ops fm828_subdev_ops = {
	.video		= &fm828_video_ops,
	.pad		= &fm828_subdev_pad_ops,
};

#if 0
static int fm828_check_id(struct fm828_device *dev)
{
	unsigned int i;
	u8 val;

	/* Read OV10640 ID to test communications. */
	for (i = 0; i < OV10640_PID_TIMEOUT; ++i) {
		ov490_write_reg(dev, OV490_SCCB_SLAVE0_DIR,
				OV490_SCCB_SLAVE_READ);
		ov490_write_reg(dev, OV490_SCCB_SLAVE0_ADDR_HIGH,
				OV10640_CHIP_ID >> 8);
		ov490_write_reg(dev, OV490_SCCB_SLAVE0_ADDR_LOW,
				OV10640_CHIP_ID & 0xff);

		/*
		 * Trigger SCCB slave transaction and give it some time
		 * to complete.
		 */
		ov490_write_reg(dev, OV490_HOST_CMD, OV490_HOST_CMD_TRIGGER);
		usleep_range(1000, 1500);

		ov490_read_reg(dev, OV490_SCCB_SLAVE0_DIR, &val);
		if (val == OV10640_ID_HIGH)
			break;
		usleep_range(1000, 1500);
	}
	if (i == OV10640_PID_TIMEOUT) {
		dev_err(dev->dev, "OV10640 ID mismatch: (0x%02x)\n", val);
		return -ENODEV;
	}

	dev_dbg(dev->dev, "OV10640 ID = 0x%2x\n", val);

	return 0;
}


static int fm828_initialize(struct fm828_device *dev)
{
	int ret;

	max9271_wake_up(&dev->serializer);

	/* Enable reverse channel and disable the serial link. */
	ret = max9271_set_serial_link(&dev->serializer, false);
	if (ret)
		return ret;

	/* Configure I2C bus at 105Kbps speed and configure GMSL. */
	ret = max9271_configure_i2c(&dev->serializer,
				    MAX9271_I2CSLVSH_469NS_234NS |
				    MAX9271_I2CSLVTO_1024US |
				    MAX9271_I2CMSTBT_105KBPS);
	if (ret)
		return ret;

	ret = max9271_verify_id(&dev->serializer);
	if (ret)
		return ret;

	/*
	 * Enable GPIO1 and hold OV490 in reset during max9271 configuration.
	 * The reset signal has to be asserted for at least 250 useconds.
	 */
	ret = max9271_enable_gpios(&dev->serializer, MAX9271_GPIO1OUT);
	if (ret)
		return ret;

	ret = max9271_clear_gpios(&dev->serializer, MAX9271_GPIO1OUT);
	if (ret)
		return ret;
	usleep_range(250, 500);

	ret = max9271_configure_gmsl_link(&dev->serializer);
	if (ret)
		return ret;

	ret = max9271_set_address(&dev->serializer, dev->addrs[0]);
	if (ret)
		return ret;
	dev->serializer.client->addr = dev->addrs[0];

	ret = max9271_set_translation(&dev->serializer, dev->addrs[1],
				      OV490_I2C_ADDRESS);
	if (ret)
		return ret;
	dev->isp->addr = dev->addrs[1];

	/* Release OV490 from reset and initialize it. */
	ret = max9271_set_gpios(&dev->serializer, MAX9271_GPIO1OUT);
	if (ret)
		return ret;
	usleep_range(3000, 5000);

	ret = ov490_initialize(dev);
	if (ret)
		return ret;

	/*
	 * Set reverse channel high threshold to increase noise immunity.
	 *
	 * This should be compensated by increasing the reverse channel
	 * amplitude on the remote deserializer side.
	 */
	return max9271_set_high_threshold(&dev->serializer, true);
}
#endif

static int fm828_probe(struct i2c_client *client)
{
	struct fm828_device *dev;
	struct fwnode_handle *ep;
	int ret;

	dev = devm_kzalloc(&client->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = &client->dev;
	dev->sensor = client;

	ret = of_property_read_u32_array(client->dev.of_node, "reg",
					 dev->addrs, 2);
	if (ret < 0) {
		dev_err(dev->dev, "Invalid DT reg property: %d\n", ret);
		return -EINVAL;
	}

	/* Create the dummy I2C client for the sensor. */
	dev->ser = i2c_new_dummy_device(client->adapter, dev->addrs[1]);
	if (IS_ERR(dev->ser))
		return PTR_ERR(dev->ser);

	/* Initialize and register the subdevice. */
	v4l2_i2c_subdev_init(&dev->sd, client, &fm828_subdev_ops);
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

static int fm828_remove(struct i2c_client *client)
{
	struct fm828_device *dev = sd_to_fm828(i2c_get_clientdata(client));

	v4l2_async_unregister_subdev(&dev->sd);
	i2c_unregister_device(dev->ser);
	fwnode_handle_put(dev->sd.fwnode);

	return 0;
}

static const struct of_device_id fm828_of_ids[] = {
	{ .compatible = "rec,fm828" },
	{ }
};
MODULE_DEVICE_TABLE(of, fm828_of_ids);

static struct i2c_driver fm828_i2c_driver = {
	.driver	= {
		.name	= "fm828",
		.of_match_table = fm828_of_ids,
	},
	.probe_new	= fm828_probe,
	.remove		= fm828_remove,
};

module_i2c_driver(fm828_i2c_driver);

MODULE_DESCRIPTION("GMSL Camera driver for FM828");
MODULE_LICENSE("GPL v2");
