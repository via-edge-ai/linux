// SPDX-License-Identifier: GPL-2.0-only
/*
 * rn6752.c - driver for AP1302 mezzanine
 *
 * Copyright (C) 2020, Witekio, Inc.
 *
 * This driver can only provide limited feature on AP1302.
 * Still need enhancement
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define DRIVER_NAME "rn6752"
#define DEFAULT_WIDTH  1920
#define DEFAULT_HEIGHT 1080

struct rn6752_device;

struct rn6752_size {
	unsigned int width;
	unsigned int height;
};

struct rn6752_device {
	struct device *dev;
	struct i2c_client *client;

	struct gpio_desc *reset_gpio;   //should be gpio19
	struct gpio_desc *standby_gpio; //should be gpio18
	struct gpio_desc *avdd_gpio;    //should be gpio77
	struct gpio_desc *dvdd_gpio;    //should be gpio73
	struct clk *clock;

	struct v4l2_fwnode_endpoint bus_cfg;

	struct mutex lock;	/* Protects formats */

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct v4l2_ctrl_handler ctrls;
};

static inline struct rn6752_device *to_rn6752(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rn6752_device, sd);
}


#define MAX_FW_LOAD_RETRIES		5
#define MAX_CHIP_DETECT_RETRIES	5

static int rn6752_write(struct rn6752_device *rn6752, u8 reg, u8 val)
{
	u8 buf[6];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = val & 0xff;

	ret = i2c_master_send(rn6752->client, buf, 2);
	if (ret < 0) {
		dev_err(rn6752->dev, "%s: register 0x%02x %s failed: %d\n",
			__func__, reg, "write", ret);
		return ret;
	}

	return 0;
}

static inline int rn6752_request_gpio(
	struct rn6752_device *rn6752, int gpio_num, char* lable)
{
	int ret = 0;
	ret = gpio_request(gpio_num, lable);
	if (ret)
		dev_err(rn6752->dev, "%s: request gpio(%d) fail(%d)",
			__func__, gpio_num, ret);

	ret = gpio_direction_output(gpio_num, 0);
	if (ret)
		dev_err(rn6752->dev, "%s: gpio(%d) direction output fail(%d)",
			__func__, gpio_num, ret);

	return ret;
}

static int rn6752_read(struct rn6752_device *rn6752, u8 addr, u8 *data)
{
	int ret;

	ret = i2c_master_send(rn6752->client, &addr, sizeof(addr));
	if (ret < 0) {
		dev_err(rn6752->dev, "I2C write address(%02x) fail ret = %d\n",
			addr, ret);
		return ret;
	}

	ret = i2c_master_recv(rn6752->client, data, 1);
	if (ret < 0) {
		dev_err(rn6752->dev, "I2C read value fail ret = %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int rn6752_power_on(struct rn6752_device *rn6752)
{
	int ret;

	/* Enable VDD gpio */
	if (rn6752->avdd_gpio) {
		gpiod_set_value(rn6752->avdd_gpio, 1);
		usleep_range(200, 1000);
	}

	if (rn6752->dvdd_gpio) {
		gpiod_set_value(rn6752->dvdd_gpio, 1);
		usleep_range(200, 1000);
	}

	/* Enable 24M clock */
	ret = clk_prepare_enable(rn6752->clock);
	if (ret < 0) {
		dev_err(rn6752->dev, "Failed to enable clock: %d\n", ret);
		return ret;
	}
	msleep(50);

	/* Reset. */
	if (rn6752->reset_gpio) {
		gpiod_set_value(rn6752->reset_gpio,1);
		msleep(5);
	}

	/* STANDBY. */
	if (rn6752->standby_gpio) {
		gpiod_set_value(rn6752->standby_gpio, 1);
		usleep_range(50000, 50000);
	}

	return 0;
}

static void rn6752_power_off(struct rn6752_device *rn6752)
{

	/* Turn the clock off. */
	clk_disable_unprepare(rn6752->clock);

	/* Disable VDD gpio */
	if (rn6752->avdd_gpio) {
		gpiod_set_value(rn6752->avdd_gpio, 0);
		usleep_range(200, 1000);
	}

	if (rn6752->dvdd_gpio) {
		gpiod_set_value(rn6752->dvdd_gpio, 0);
		usleep_range(200, 1000);
	}

	/* Disable STANDBY. */
	if (rn6752->standby_gpio) {
		usleep_range(200, 1000);
		gpiod_set_value(rn6752->standby_gpio, 0);
	}
}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

static int rn6752_configure(struct rn6752_device *rn6752)
{
	//const struct rn6752_format *format = &ap1302->formats[AP1302_PAD_SOURCE];
	unsigned int data_lanes = rn6752->bus_cfg.bus.mipi_csi2.num_data_lanes;
	int ret = 0;

	if (ret < 0)
		return ret;

	dev_dbg(rn6752->dev, "get mipi data lanes: %d\n", data_lanes);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static struct v4l2_mbus_framefmt *
rn6752_get_pad_format(struct rn6752_device *rn6752,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&rn6752->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &rn6752->format;
	default:
		return NULL;
	}
}

static int rn6752_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct rn6752_device *rn6752 = to_rn6752(sd);

	if (code->pad == 0) {
		if (code->index)
			return -EINVAL;
		code->code = rn6752->format.code;
	} else {
		/* RN6752 has not Sink port */
		return -EINVAL;
	}

	return 0;
}

static int rn6752_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct rn6752_device *rn6752 = to_rn6752(sd);

	if (fse->index)
		return -EINVAL;

	if (fse->pad == 0) {
		/* On the source pad, Only Support 1920x1080 */
		if (fse->code != rn6752->format.code)
			return -EINVAL;

		fse->min_width  = DEFAULT_WIDTH;
		fse->min_height = DEFAULT_HEIGHT;
		fse->max_width  = DEFAULT_WIDTH;
		fse->max_height = DEFAULT_HEIGHT;
	} else {
		/* RN6752 has no Sink pad */
		return -EINVAL;
	}

	return 0;
}

static int rn6752_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct rn6752_device *rn6752 = to_rn6752(sd);
	const struct v4l2_mbus_framefmt *format;

	format = rn6752_get_pad_format(rn6752, sd_state, fmt->pad, fmt->which);

	mutex_lock(&rn6752->lock);
	fmt->format = *format;
	mutex_unlock(&rn6752->lock);

	return 0;
}

static int rn6752_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct rn6752_device *rn6752 = to_rn6752(sd);
	struct v4l2_mbus_framefmt *format;

	format = rn6752_get_pad_format(rn6752, sd_state, fmt->pad, fmt->which);

	mutex_lock(&rn6752->lock);
	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = MEDIA_BUS_FMT_YUYV8_1X16;
	mutex_unlock(&rn6752->lock);

	dev_dbg(rn6752->dev,
	"[%s:%d] format->width: %d, format->height: %d, format->code: %x\n",
			__func__, __LINE__, format->width, format->height, format->code);

	fmt->format = *format;

	return 0;
}

static int rn6752_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = DEFAULT_WIDTH;
		sel->r.height = DEFAULT_HEIGHT;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int rn6752_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rn6752_device *rn6752 = to_rn6752(sd);
	int ret;

	mutex_lock(&rn6752->lock);

	if (enable) {
		ret = rn6752_configure(rn6752);
		if (ret < 0)
			goto done;

	}

done:
	mutex_unlock(&rn6752->lock);

	if (ret < 0)
		dev_err(rn6752->dev, "Failed to %s stream: %d\n",
			enable ? "start" : "stop", ret);

	return ret;
}

static const struct media_entity_operations rn6752_media_ops = {
	.link_validate = v4l2_subdev_link_validate
};

static const struct v4l2_subdev_pad_ops rn6752_pad_ops = {
	.enum_mbus_code = rn6752_enum_mbus_code,
	.enum_frame_size = rn6752_enum_frame_size,
	.get_fmt = rn6752_get_fmt,
	.set_fmt = rn6752_set_fmt,
	.get_selection = rn6752_get_selection,
	.set_selection = rn6752_get_selection,
};

static const struct v4l2_subdev_video_ops rn6752_video_ops = {
	.s_stream = rn6752_s_stream,
};

static const struct v4l2_subdev_ops rn6752_subdev_ops = {
	.video = &rn6752_video_ops,
	.pad = &rn6752_pad_ops,
};

static int rn6752_load_config(struct rn6752_device *rn6752)
{
	const char config_table[77][2] = {
		{0xE1,0x80}, {0xFA,0x81}, {0xEF,0xAA}, {0xFC,0x60}, {0xFF,0x09},
		{0x03,0x18}, {0xFF,0x0B}, {0x03,0x18}, {0xD2,0x85}, {0xD6,0x17},
		{0xD8,0x18}, {0xFF,0x00}, {0x3E,0x50}, {0x96,0x40}, {0x97,0x02},
		{0x98,0x00}, {0x8D,0x31}, {0x81,0x01}, {0xA3,0x04}, {0xDF,0xFE},
		{0xF0,0xC0}, {0x88,0x00}, {0x8E,0x00}, {0xFF,0x00}, {0x00,0x20},
		{0x06,0x08}, {0x07,0x63}, {0x2A,0x01}, {0x3A,0x20}, {0x3F,0x10},
		{0x4C,0x37}, {0x4F,0x03}, {0x50,0x03}, {0x56,0x02}, {0x5F,0x44},
		{0x63,0xF8}, {0x59,0x00}, {0x5A,0x49}, {0x58,0x01}, {0x59,0x33},
		{0x5A,0x23}, {0x58,0x01}, {0x51,0xF4}, {0x52,0x29}, {0x53,0x15},
		{0x5B,0x01}, {0x5E,0x08}, {0x6A,0x87}, {0x28,0x92}, {0x03,0x80},
		{0x04,0x80}, {0x05,0x04}, {0x57,0x23}, {0x68,0x00}, {0x37,0x33},
		{0x61,0x6C}, {0x33,0x10}, {0x4A,0xA8}, {0x2E,0x30}, {0x2E,0x00},
		{0xFF,0x09}, {0x00,0x03}, {0xFF,0x08}, {0x04,0x03}, {0x6C,0x11},
		{0x06,0x4C}, {0x21,0x01}, {0x34,0x06}, {0x35,0x0B}, {0x78,0xC0},
		{0x79,0x03}, {0x6C,0x01}, {0x04,0x00}, {0x20,0xAA}, {0x07,0x05},
		{0xFF,0x0A}, {0x6C,0x10}
	};

	#define REG_ADDR 0
	#define REG_VAL  1
	int i, ret = 0;
	int reg_size = sizeof(config_table) / sizeof(config_table[0]);

	dev_dbg(rn6752->dev,
		"Start config rn6752, config table size(%d)", reg_size);

	for (i = 0; i < reg_size; i++) {
		ret = rn6752_write(rn6752, config_table[i][REG_ADDR], config_table[i][REG_VAL]);
		if (ret) {
			dev_err(rn6752->dev, "Withe reg(0x%02x) = 0x%02x FAIL(%d)",
				config_table[i][REG_ADDR], config_table[i][REG_VAL], ret);
			break;
		}
	}
	return ret;
}


static int rn6752_detect_chip(struct rn6752_device *rn6752)
{
	u8 chip_id;
	unsigned int retries;
	int ret;

	#define RN6752_CHIP_ID_ADDR 0xFE
	#define RN6752_CHIP_ID      0x26

	for (retries = 0; retries < MAX_CHIP_DETECT_RETRIES; ++retries) {
		ret = rn6752_read(rn6752, RN6752_CHIP_ID_ADDR, &chip_id);
		if (ret)
			return ret;

		if (chip_id == RN6752_CHIP_ID)
			break;

		msleep(100);
	}

	if (retries == MAX_CHIP_DETECT_RETRIES) {
		dev_err(rn6752->dev,
			"Invalid chip id, expected 0x%02x, got 0x%02x\n",
			RN6752_CHIP_ID, chip_id);
		return -EINVAL;
	}

	dev_info(rn6752->dev, "RN6752 get chip id(0x%02x)\n", chip_id);

	return 0;
}

static int rn6752_hw_init(struct rn6752_device *rn6752)
{
	unsigned int retries;
	int ret = 0;

	for (retries = 0; retries < MAX_FW_LOAD_RETRIES; ++retries) {
		ret = rn6752_power_on(rn6752);
		if (ret < 0) {
			dev_err(rn6752->dev, "RN6752 power up fail. ret = %d\n", ret);
			return ret;
		}

		ret = rn6752_detect_chip(rn6752);
		if (ret < 0) {
			dev_err(rn6752->dev, "RN6752 detect id fail. ret = %d\n", ret);
		}

		ret = rn6752_load_config(rn6752);
		if (!ret)
			break;

		rn6752_power_off(rn6752);
	}

	if (retries == MAX_FW_LOAD_RETRIES) {
		dev_err(rn6752->dev,
			"Can not detect chip\n");
		ret = -ETIMEDOUT;
	}

	return ret;
}

static void rn6752_hw_cleanup(struct rn6752_device *rn6752)
{
	rn6752_power_off(rn6752);
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int rn6752_config_v4l2(struct rn6752_device *rn6752)
{
	struct v4l2_subdev *sd;
	int ret;

	sd = &rn6752->sd;
	sd->dev = rn6752->dev;
	v4l2_i2c_subdev_init(sd, rn6752->client, &rn6752_subdev_ops);

	strscpy(sd->name, DRIVER_NAME, sizeof(sd->name));
	strlcat(sd->name, ".", sizeof(sd->name));
	strlcat(sd->name, dev_name(rn6752->dev), sizeof(sd->name));
	dev_dbg(rn6752->dev, "name %s\n", sd->name);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &rn6752_media_ops;

	rn6752->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &rn6752->pad);
	if (ret < 0) {
		dev_err(rn6752->dev, "media_entity_init failed %d\n", ret);
		return ret;
	}

	rn6752->format.width      = DEFAULT_WIDTH;
	rn6752->format.height     = DEFAULT_HEIGHT;
	rn6752->format.code       = MEDIA_BUS_FMT_YUYV8_1X16;
	rn6752->format.field      = V4L2_FIELD_NONE;
	rn6752->format.colorspace = V4L2_COLORSPACE_SRGB;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(rn6752->dev, "v4l2_async_register_subdev failed %d\n", ret);
		goto error_media;
	}

	return 0;

error_media:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static int rn6752_parse_of(struct rn6752_device *rn6752)
{
	struct fwnode_handle *ep;
	int ret;

	/* Clock */
	rn6752->clock = devm_clk_get(rn6752->dev, NULL);
	if (IS_ERR(rn6752->clock)) {
		dev_err(rn6752->dev, "Failed to get clock: %ld\n",
			PTR_ERR(rn6752->clock));
		return PTR_ERR(rn6752->clock);
	}

	/* GPIOs */
	rn6752->reset_gpio = devm_gpiod_get(rn6752->dev, "reset",
					    GPIOD_OUT_HIGH);
	if (IS_ERR(rn6752->reset_gpio)) {
		dev_err(rn6752->dev, "Can't get reset GPIO: %ld\n",
			PTR_ERR(rn6752->reset_gpio));
		return PTR_ERR(rn6752->reset_gpio);
	}

	rn6752->standby_gpio = devm_gpiod_get(rn6752->dev, "power",
						       GPIOD_OUT_LOW);
	if (IS_ERR(rn6752->standby_gpio)) {
		dev_err(rn6752->dev, "Can't get standby GPIO: %ld\n",
			PTR_ERR(rn6752->standby_gpio));
		return PTR_ERR(rn6752->standby_gpio);
	}

	rn6752->avdd_gpio = devm_gpiod_get(rn6752->dev, "avdd",
						       GPIOD_OUT_LOW);
	if (IS_ERR(rn6752->avdd_gpio)) {
		dev_err(rn6752->dev, "Can't get avdd GPIO: %ld\n",
			PTR_ERR(rn6752->avdd_gpio));
		return PTR_ERR(rn6752->avdd_gpio);
	}

	rn6752->dvdd_gpio = devm_gpiod_get(rn6752->dev, "dvdd",
						       GPIOD_OUT_LOW);
	if (IS_ERR(rn6752->dvdd_gpio)) {
		dev_err(rn6752->dev, "Can't get dvdd GPIO: %ld\n",
			PTR_ERR(rn6752->dvdd_gpio));
		return PTR_ERR(rn6752->dvdd_gpio);
	}

	/* Bus configuration */
	ep = fwnode_graph_get_next_endpoint(dev_fwnode(rn6752->dev), NULL);
	if (!ep)
		return -EINVAL;

	rn6752->bus_cfg.bus_type = V4L2_MBUS_CSI2_DPHY;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &rn6752->bus_cfg);
	if (ret < 0) {
		dev_err(rn6752->dev, "Failed to parse bus configuration\n");
		return ret;
	}

	return ret;
}

static void rn6752_cleanup(struct rn6752_device *rn6752)
{
	v4l2_fwnode_endpoint_free(&rn6752->bus_cfg);
	mutex_destroy(&rn6752->lock);
}

static int rn6752_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct rn6752_device *rn6752 = to_rn6752(sd);

	dev_dbg(dev, "rn6752 suspend\n");

	rn6752_hw_cleanup(rn6752);

	return 0;
}

static int rn6752_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct rn6752_device *rn6752 = to_rn6752(sd);
	int ret;

	dev_dbg(dev, "rn6752 resume\n");

	mutex_lock(&rn6752->lock);
	ret = rn6752_hw_init(rn6752);
	mutex_unlock(&rn6752->lock);

	return ret;
}

static const struct dev_pm_ops runtime_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rn6752_suspend, rn6752_resume)
};

static int rn6752_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rn6752_device *rn6752;
	int ret;

	dev_info(&client->dev, "rn6752 probe start\n");

	rn6752 = devm_kzalloc(&client->dev, sizeof(struct rn6752_device), GFP_KERNEL);
	if (!rn6752)
		return -ENOMEM;

	rn6752->dev = &client->dev;
	rn6752->client = client;

	mutex_init(&rn6752->lock);

	ret = rn6752_parse_of(rn6752);
	if (ret < 0)
		goto error;

	ret = rn6752_hw_init(rn6752);
	if (ret)
		goto error;

	ret = rn6752_config_v4l2(rn6752);
	if (ret)
		goto error_hw_cleanup;

	dev_info(rn6752->dev, "rn6752 probe sussces\n");

	return 0;

error_hw_cleanup:
	rn6752_hw_cleanup(rn6752);
error:
	rn6752_cleanup(rn6752);
	return ret;
}

static int rn6752_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct rn6752_device *rn6752 = to_rn6752(sd);

	rn6752_hw_cleanup(rn6752);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct of_device_id rn6752_of_id_table[] = {
	{ .compatible = "richnex,rn6752" },
	{ }
};
MODULE_DEVICE_TABLE(of, rn6752_of_id_table);

static struct i2c_driver rn6752_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= rn6752_of_id_table,
		.pm  = &runtime_pm_ops,
	},
	.probe		= rn6752_probe,
	.remove		= rn6752_remove,
};

module_i2c_driver(rn6752_i2c_driver);

MODULE_AUTHOR("Florian Rebaudo <frebaudo@witekio.com>");
MODULE_DESCRIPTION("Driver for rn6752 AHD2MIPIbridge");
MODULE_LICENSE("GPL");
