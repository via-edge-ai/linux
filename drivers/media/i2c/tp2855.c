// SPDX-License-Identifier: GPL-2.0-only
/*
 * tp2855.c
 *
 * Still need enhancement
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define DRIVER_NAME "tp2855"
#define DEFAULT_WIDTH  1920
#define DEFAULT_HEIGHT 1080
#define TP2855_N_PADS   5
#define TP2855_NUM_GMSL 4
#define TP2855_SRC_PAD  4
#define MAX_FW_LOAD_RETRIES		5
#define MAX_CHIP_DETECT_RETRIES	5

static int enable_pattern = 0;

struct tp2855_device;

struct tp2855_size {
	unsigned int width;
	unsigned int height;
};

struct tp2855_source {
	struct v4l2_subdev *sd;
	struct fwnode_handle *fwnode;
};

struct tp2855_asd {
	struct v4l2_async_subdev base;
	struct tp2855_source *source;
};

static inline struct tp2855_asd *to_tp2855_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct tp2855_asd, base);
}

struct tp2855_device {
	struct device *dev;
	struct i2c_client *client;

	struct gpio_desc *cam0dvdd_gpio; //should be gpio60
	struct gpio_desc *cam1dvdd_gpio; //should be gpio58

	struct clk *clock;

	struct v4l2_fwnode_endpoint bus_cfg;

	struct mutex lock;	/* Protects formats */

	struct v4l2_subdev sd;
	struct media_pad pads[TP2855_N_PADS];
	struct v4l2_mbus_framefmt format[TP2855_N_PADS];

	//v4l2 subdevice source
	unsigned int nsources;
	unsigned int source_mask;
	unsigned int route_mask;
	unsigned int bound_sources;
	unsigned int csi2_data_lanes;
	struct tp2855_source sources[TP2855_NUM_GMSL];
	struct v4l2_async_notifier notifier;

	struct regulator *supply;
};

static inline struct tp2855_device *to_tp2855(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tp2855_device, sd);
}

static struct tp2855_source *next_source(struct tp2855_device *tp2855,
					  struct tp2855_source *source)
{
	if (!source)
		source = &tp2855->sources[0];
	else
		source++;

	for (; source < &tp2855->sources[TP2855_NUM_GMSL]; source++) {
		if (source->fwnode)
			return source;
	}

	return NULL;
}

#define for_each_source(priv, source) \
	for ((source) = NULL; ((source) = next_source((priv), (source))); )

#define to_index(priv, source) ((source) - &(priv)->sources[0])

static int tp2855_write(struct tp2855_device *tp2855, u8 reg, u8 val)
{
	u8 buf[6];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = val & 0xff;

	ret = i2c_master_send(tp2855->client, buf, 2);
	if (ret < 0) {
		dev_err(tp2855->dev, "%s: register 0x%02x %s failed: %d\n",
			__func__, reg, "write", ret);
		return ret;
	}

	return 0;
}

static int tp2855_read(struct tp2855_device *tp2855, u8 addr, u8 *data)
{
	int ret;

	ret = i2c_master_send(tp2855->client, &addr, sizeof(addr));
	if (ret < 0) {
		dev_err(tp2855->dev, "I2C write address(%02x) fail ret = %d\n",
			addr, ret);
		return ret;
	}

	ret = i2c_master_recv(tp2855->client, data, 1);
	if (ret < 0) {
		dev_err(tp2855->dev, "I2C read value fail ret = %d\n",
			ret);
		return ret;
	}

	return 0;
}

static void tp2855_reg_dump(struct tp2855_device *tp2855)
{
	u8 addr = 0, data = 0;
	int vin = 0;

	dev_info(tp2855->dev, "--------------- TP2855 Registers --------------");
	for (vin = 0; vin <= 4; vin++) {
		dev_info(tp2855->dev, "------------vin%d: common regists-------------", vin);
		tp2855_write(tp2855, 0x40, vin);
		for (addr = 0; addr < 0xff; addr++) {
			data = 0;
			tp2855_read(tp2855, addr, &data);
			dev_info(tp2855->dev, "vid%d: reg[0x%02x] = 0x%02x\n",
				vin, addr, data);
		}
		dev_info(tp2855->dev, "------------vin%d: common regists end-------------", vin);
	}
	dev_info(tp2855->dev, "MIPI-CSI-specific registers:");
	tp2855_write(tp2855, 0x40, 0x08);
	for (addr = 0; addr <= 0x36; addr++) {
		data = 0;
		dev_info(tp2855->dev,"MIPI: reg[0x%02x] = 0x%02x\n", addr, data);
	}
	dev_info(tp2855->dev, "MIPI-CSI-specific registers dump end");
}

static inline int tp2855_request_gpio(
	struct tp2855_device *tp2855, int gpio_num, char* lable)
{
	int ret = 0;
	ret = gpio_request(gpio_num, lable);
	if (ret)
		dev_err(tp2855->dev, "%s: request gpio(%d) fail(%d)",
			__func__, gpio_num, ret);

	ret = gpio_direction_output(gpio_num, 0);
	if (ret)
		dev_err(tp2855->dev, "%s: gpio(%d) direction output fail(%d)",
			__func__, gpio_num, ret);
	
	return ret;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev
 */

static int tp2855_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct tp2855_device *tp2855 = to_tp2855(notifier->sd);
	struct tp2855_source *source = to_tp2855_asd(asd)->source;
	unsigned int index = to_index(tp2855, source);
	struct v4l2_subdev_format sub_fmt = {0};
	unsigned int src_pad;
	int ret;

	ret = media_entity_get_fwnode_pad(&subdev->entity,
					  source->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(&tp2855->client->dev,
			"Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	tp2855->bound_sources |= BIT(index);
	source->sd = subdev;
	src_pad = ret;

	ret = media_create_pad_link(&source->sd->entity, src_pad,
				    &tp2855->sd.entity, index,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(&tp2855->client->dev,
			"Unable to link %s:%u -> %s:%u\n",
			source->sd->name, src_pad, tp2855->sd.name, index);
		return ret;
	}

	dev_info(&tp2855->client->dev, "Bound %s pad: %u on index %u\n",
		subdev->name, src_pad, index);

	return 0;
}

static void tp2855_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
	struct tp2855_device *tp2855 = to_tp2855(notifier->sd);
	struct tp2855_source *source = to_tp2855_asd(asd)->source;
	unsigned int index = to_index(tp2855, source);

	source->sd = NULL;
	tp2855->bound_sources &= ~BIT(index);
}

static const struct v4l2_async_notifier_operations tp2855_notify_ops = {
	.bound = tp2855_notify_bound,
	.unbind = tp2855_notify_unbind,
};

static int tp2855_v4l2_notifier_register(struct tp2855_device *tp2855)
{
	struct device *dev = &tp2855->client->dev;
	struct tp2855_source *source = NULL;
	int ret;

	if (!tp2855->nsources)
		return 0;

	v4l2_async_notifier_init(&tp2855->notifier);

	for_each_source(tp2855, source) {
		unsigned int i = to_index(tp2855, source);
		struct tp2855_asd *mas;

		mas = v4l2_async_notifier_add_fwnode_subdev(&tp2855->notifier,
							    source->fwnode,
							    struct tp2855_asd);
		if (IS_ERR(mas)) {
			dev_err(dev, "Failed to add subdev for source %u: %ld",
				i, PTR_ERR(mas));
			v4l2_async_notifier_cleanup(&tp2855->notifier);
			return PTR_ERR(mas);
		}

		mas->source = source;
	}

	tp2855->notifier.ops = &tp2855_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&tp2855->sd, &tp2855->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&tp2855->notifier);
		return ret;
	}

	return 0;
}

static void tp2855_v4l2_notifier_unregister(struct tp2855_device *tp2855)
{
	if (!tp2855->nsources)
		return;

	v4l2_async_notifier_unregister(&tp2855->notifier);
	v4l2_async_notifier_cleanup(&tp2855->notifier);
}


static int tp2855_power_on(struct tp2855_device *tp2855)
{
	int ret;
	msleep(500);

	return 0;
}

static void tp2855_power_off(struct tp2855_device *tp2855)
{
	u8 val = 0;

	dev_info(tp2855->dev, "disable MIPI CSI2 output get 0x23= 0x%02x\n", val);

	/* Disable MIPI CSI2 output */
	tp2855_write(tp2855, 0x40, 0x8);
	tp2855_read(tp2855, 0x23, &val);
	dev_info(tp2855->dev, "disable MIPI CSI2 output get 0x23= 0x%02x\n", val);
	val |= (0x1 << 1); // disable D-PHY clock.
	dev_info(tp2855->dev, "disable MIPI CSI2 output change 0x23= 0x%02x\n", val);
	tp2855_write(tp2855, 0x23, val);

	tp2855_read(tp2855, 0x02, &val);
    val &= ~(0x1 << 3);
    val &= ~(0x1 << 2);
    val &= ~(0x1 << 1);
    val &= ~(0x1 << 0);
	tp2855_write(tp2855, 0x02, val);
}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

static int tp2855_configure(struct tp2855_device *tp2855)
{
	//const struct tp2855_format *format = &ap1302->formats[AP1302_PAD_SOURCE];
	unsigned int data_lanes = tp2855->bus_cfg.bus.mipi_csi2.num_data_lanes;
	int ret = 0;

	if (ret < 0)
		return ret;

	dev_info(tp2855->dev, "get mipi data lanes: %d\n", data_lanes);

	return 0;
}

// AHD1080P30 camera decoder init
static int tp2854_ahd_1080p_hw_init(struct tp2855_device *tp2855, u8 ch)
{
	u8 val;
	int ret = 0;
	const unsigned char SYS_MODE[5]={0x01,0x02,0x04,0x08,0x0f}; 

	dev_info(tp2855->dev, "Start config 1080p on %d ch\n", ch);

	//TP2854_decoder_init
	tp2855_write(tp2855, 0x40, 0x04);
	tp2855_write(tp2855, 0x06, 0x80);
	tp2855_read(tp2855, 0xf4, &val);
	dev_dbg(tp2855->dev, "Get 0xf4:%x\n", val);
	val |= 0x0f; 			//disable all vin 
	val &= ~(tp2855->source_mask);  // enable vin that we want
	dev_info(tp2855->dev, "Set 0xf4:%x\n", val);
	tp2855_write(tp2855, 0xf4, val);

	dev_dbg(tp2855->dev, "config rx%d\n", ch);
	tp2855_write(tp2855, 0x40, ch);

	tp2855_write(tp2855, 0x45, 0x01); 
	tp2855_write(tp2855, 0x06, 0x12); 
	tp2855_write(tp2855, 0x27, 0x2d);

	tp2855_read(tp2855, 0xf5, &val);
	dev_info(tp2855->dev, "Get 0xf5= 0x%02x\n", val);
	val &= ~SYS_MODE[ch];
	dev_dbg(tp2855->dev, "Set 0xf5= 0x%02x\n", val);
	tp2855_write(tp2855, 0xf5, val);

	tp2855_write(tp2855, 0x02, 0x44);
	tp2855_write(tp2855, 0x07, 0xc0); 
	tp2855_write(tp2855, 0x0b, 0xc0);  		
	tp2855_write(tp2855, 0x0c, 0x03); 
	tp2855_write(tp2855, 0x0d, 0x72);  
	tp2855_write(tp2855, 0x15, 0x01);
	tp2855_write(tp2855, 0x16, 0xf0); 
	tp2855_write(tp2855, 0x17, 0x80); 
	tp2855_write(tp2855, 0x18, 0x2a);
	tp2855_write(tp2855, 0x19, 0x38);
	tp2855_write(tp2855, 0x1a, 0x47);				
	tp2855_write(tp2855, 0x1c, 0x08);  //1920*1080, 30fps
	tp2855_write(tp2855, 0x1d, 0x98);  //
	tp2855_write(tp2855, 0x20, 0x38);  
	tp2855_write(tp2855, 0x21, 0x46); 
	tp2855_write(tp2855, 0x22, 0x36);
	tp2855_write(tp2855, 0x23, 0x3c);
	tp2855_write(tp2855, 0x25, 0xfe);
	tp2855_write(tp2855, 0x26, 0x0d);

	//tp2855_write(tp2855, 0x2a, 0x3c);    //blue screen
	tp2855_write(tp2855, 0x2b, 0x60);  
	tp2855_write(tp2855, 0x2c, 0x3a); 
	tp2855_write(tp2855, 0x2d, 0x54);
	tp2855_write(tp2855, 0x2e, 0x40);
	tp2855_write(tp2855, 0x30, 0xa5);  
	tp2855_write(tp2855, 0x31, 0x95); 
	tp2855_write(tp2855, 0x32, 0xe0);
	tp2855_write(tp2855, 0x33, 0x60);
	tp2855_write(tp2855, 0x35, 0x05);
	tp2855_write(tp2855, 0x38, 0x00); 
	tp2855_write(tp2855, 0x39, 0x1C);

    return ret;
}

// AHD720P30 camera decoder init
static int tp2854_ahd_720p_hw_init(struct tp2855_device *tp2855, u8 ch)
{
	u8 val;
	int ret = 0;
	const unsigned char SYS_MODE[5]={0x01,0x02,0x04,0x08,0x0f};

	dev_info(tp2855->dev, "Start config 720p on %d ch\n", ch);

	//TP2854_decoder_init
	tp2855_write(tp2855, 0x40, 0x04);
	tp2855_write(tp2855, 0x06, 0x80);
	tp2855_read(tp2855, 0xf4, &val);
	dev_info(tp2855->dev, "Get 0xf4:%x\n", val);
	val |= 0x0f; 			//disable all vin
	val &= ~(tp2855->source_mask);  // enable vin that we want
	dev_info(tp2855->dev, "Set 0xf4:%x\n", val);
	tp2855_write(tp2855, 0xf4, val);

	dev_dbg(tp2855->dev, "config rx%d\n", ch);
	tp2855_write(tp2855, 0x40, ch);
	tp2855_write(tp2855, 0x45, 0x01);
	tp2855_write(tp2855, 0x06, 0x12);
	tp2855_write(tp2855, 0x27, 0x2d);

	tp2855_read(tp2855, 0xf5, &val);
	dev_info(tp2855->dev, "Get 0xf5= 0x%02x\n", val);
	val |= SYS_MODE[ch];
	dev_info(tp2855->dev, "Set 0xf5= 0x%02x\n", val);
	tp2855_write(tp2855, 0xf5, val);

	tp2855_write(tp2855, 0x02, 0x46);
	tp2855_write(tp2855, 0x07, 0xc0);
	tp2855_write(tp2855, 0x0b, 0xc0);
	tp2855_write(tp2855, 0x0c, 0x13);
	tp2855_write(tp2855, 0x0d, 0x70);

	tp2855_write(tp2855, 0x15, 0x13);
	tp2855_write(tp2855, 0x16, 0x15);
	tp2855_write(tp2855, 0x17, 0x00);
	tp2855_write(tp2855, 0x18, 0x19);
	tp2855_write(tp2855, 0x19, 0xd0);
	tp2855_write(tp2855, 0x1a, 0x25);
	tp2855_write(tp2855, 0x1c, 0x06);  //1280*720, 30fps
	tp2855_write(tp2855, 0x1d, 0x72);  //1280*720, 30fps

	tp2855_write(tp2855, 0x20, 0x40);
	tp2855_write(tp2855, 0x21, 0x46);
	tp2855_write(tp2855, 0x22, 0x36);
	tp2855_write(tp2855, 0x23, 0x3c);
	tp2855_write(tp2855, 0x25, 0xfe);
	tp2855_write(tp2855, 0x26, 0x01);

	//tp2855_write(tp2855, 0x2a, 0x3c);    //blue screen
	tp2855_write(tp2855, 0x2b, 0x60);
	tp2855_write(tp2855, 0x2c, 0x3a);
	tp2855_write(tp2855, 0x2d, 0x5a);

	tp2855_write(tp2855, 0x2e, 0x40);
	tp2855_write(tp2855, 0x30, 0x9d);
	tp2855_write(tp2855, 0x31, 0xca);
	tp2855_write(tp2855, 0x32, 0x01);
	tp2855_write(tp2855, 0x33, 0xd0);
	tp2855_write(tp2855, 0x35, 0x25);

	tp2855_write(tp2855, 0x38, 0x00);
	tp2855_write(tp2855, 0x39, 0x18);

	return ret;
}


static int tp2854_tx_init_4ch4lane_594m(struct tp2855_device *tp2855)
{
    int ret = 0;

	//TP2855_mipi_out
    tp2855_write(tp2855, 0x40, 0x08); //MIPI page 
	tp2855_write(tp2855, 0x01, 0xf0);
	tp2855_write(tp2855, 0x02, 0x01);
	tp2855_write(tp2855, 0x08, 0x0f);

	//MIPI_4CH4LANE_594M
	tp2855_write(tp2855, 0x20, 0x44);
	tp2855_write(tp2855, 0x34, 0xe4);
	tp2855_write(tp2855, 0x15, 0x0c);		
	tp2855_write(tp2855, 0x25, 0x08);
	tp2855_write(tp2855, 0x26, 0x06);
	tp2855_write(tp2855, 0x27, 0x11);
	tp2855_write(tp2855, 0x29, 0x0a);
	tp2855_write(tp2855, 0x33, 0x07);
	tp2855_write(tp2855, 0x33, 0x00);
	tp2855_write(tp2855, 0x14, 0x33);
	tp2855_write(tp2855, 0x14, 0xb3);
	tp2855_write(tp2855, 0x14, 0x33);
	tp2855_write(tp2855, 0x21, 0x03);
	tp2855_write(tp2855, 0x23, 0x02);
	tp2855_write(tp2855, 0x23, 0x00);
	
	tp2855_write(tp2855, 0x40, 0x00);
	return ret;
}

static int tp2855_detect_chip(struct tp2855_device *tp2855)
{
	u8 chip_id_h = 0, chip_id_l = 0;
	unsigned int retries;
	int ret = 0;

	#define TP2855_CHIP_ID_ADDR_H 0xFE
	#define TP2855_CHIP_ID_ADDR_L 0xFF
	#define TP2855_CHIP_ID        0x2855

	for (retries = 0; retries < MAX_CHIP_DETECT_RETRIES; ++retries) {
		ret = tp2855_write(tp2855, 0x40, 0x00);
		if (ret) {
			dev_err(tp2855->dev, "Can't set REG_PAGE reg\n");
			continue;
		}
		
		ret = tp2855_read(tp2855, TP2855_CHIP_ID_ADDR_H, &chip_id_h);
		ret = tp2855_read(tp2855, TP2855_CHIP_ID_ADDR_L, &chip_id_l);

		if ((chip_id_h == ((TP2855_CHIP_ID >> 8) & 0xff)) &&
			(chip_id_l == ((TP2855_CHIP_ID) & 0xff)))
			break;

		msleep(100);
	}

	if (retries == MAX_CHIP_DETECT_RETRIES) {
		dev_err(tp2855->dev,
			"Invalid chip id, expected 0x%04x, got 0x%02x%02x\n",
			TP2855_CHIP_ID, chip_id_h, chip_id_l);
		return -EINVAL;
	}

	dev_info(tp2855->dev, "TP2854 get chip id(0x%02x%02x)\n", 
		chip_id_h, chip_id_l);

	return 0;
}

static int tp2855_hw_init(struct tp2855_device *tp2855)
{
	unsigned int retries;
	int ret = 0, i = 0;
	static int tp_config = 1;

	for (retries = 0; retries < MAX_FW_LOAD_RETRIES; ++retries) {
		ret = tp2855_power_on(tp2855);
		if (ret < 0) {
			dev_err(tp2855->dev, "TP2854 power up fail. ret = %d\n", ret);
			return ret;
		}

		ret = tp2855_detect_chip(tp2855);
		if (ret < 0) {
			dev_err(tp2855->dev, "TP2854 detect id fail. ret = %d\n", ret);
		}

		ret = tp2854_tx_init_4ch4lane_594m(tp2855);
		if (ret < 0) {
			dev_err(tp2855->dev, "tp2854_tx_init_4ch4lane_594m fail\n");
	        continue;
		}
		break;
	}

	if (retries == MAX_FW_LOAD_RETRIES) {
		dev_err(tp2855->dev,
			"Can not detect chip\n");
		ret = -ETIMEDOUT;
	}

	return ret;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static struct v4l2_mbus_framefmt *
tp2855_get_pad_format(struct tp2855_device *tp2855,
		      		  struct v4l2_subdev_state *sd_state,
		      		  unsigned int pad, u32 which)
{

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&tp2855->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &tp2855->format[pad];
	default:
		return NULL;
	}
}

static int tp2855_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;

	return 0;
}



static int tp2855_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct tp2855_device *tp2855 = to_tp2855(sd);
	const struct v4l2_mbus_framefmt *format;

	if (fmt->pad == TP2855_SRC_PAD)
		fmt->pad = __ffs(tp2855->bound_sources);

	format = tp2855_get_pad_format(tp2855, sd_state, fmt->pad, fmt->which);

	mutex_lock(&tp2855->lock);
	fmt->format = *format;
	mutex_unlock(&tp2855->lock);

	dev_dbg(tp2855->dev,
	"[%s:%d] fmt->pad: %d, format->width: %d, format->height: %d, format->code: %x\n",
			__func__, __LINE__, fmt->pad, format->width, format->height, format->code);

	return 0;
}

static int tp2855_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct tp2855_device *tp2855 = to_tp2855(sd);
	struct v4l2_mbus_framefmt *format;
	int ret = 0;

	if (fmt->pad == TP2855_SRC_PAD)
		return -EINVAL;

	format = tp2855_get_pad_format(tp2855, sd_state, fmt->pad, fmt->which);

	mutex_lock(&tp2855->lock);
	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = MEDIA_BUS_FMT_UYVY8_1X16;
	mutex_unlock(&tp2855->lock);

	dev_dbg(tp2855->dev,
	"[%s:%d] fmt->pad: %d, format->width: %d, format->height: %d, format->code: %x\n",
			__func__, __LINE__, fmt->pad, format->width, format->height, format->code);

	//Just for debug
	{
		struct v4l2_mbus_framefmt *tmp=&tp2855->format[fmt->pad];
		dev_dbg(tp2855->dev,
		"[%s:%d] width: %d, height: %d\n",
				__func__, __LINE__, tmp->width, tmp->height);
	}

	if (tp2855->format[fmt->pad].width == 1920 &&
		tp2855->format[fmt->pad].height == 1080) {
		ret = tp2854_ahd_1080p_hw_init(tp2855, fmt->pad);
		if (ret) {
			dev_err(tp2855->dev,
			"tp2854_ahd_1080p_hw_init fail(%d)\n", fmt->pad);
		}
	} else if (tp2855->format[fmt->pad].width == 1280 &&
			   tp2855->format[fmt->pad].height == 720) {
		ret = tp2854_ahd_720p_hw_init(tp2855, fmt->pad);
		if (ret) {
			dev_err(tp2855->dev,
			"tp2854_ahd_720p_hw_init fail(%d)\n", fmt->pad);
		}
	}

	fmt->format = *format;

	return ret;
}

static int tp2855_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tp2855_device *tp2855 = to_tp2855(sd);
	int ret = 0;

	mutex_lock(&tp2855->lock);

	if (enable) {

		ret = tp2855_hw_init(tp2855);
		if (ret < 0)
			goto done;

		ret = tp2855_configure(tp2855);
		if (ret < 0)
			goto done;

		//tp2855_reg_dump(tp2855);
	}

done:
	mutex_unlock(&tp2855->lock);

	if (ret < 0)
		dev_err(tp2855->dev, "Failed to %s stream: %d\n",
			enable ? "start" : "stop", ret);

	return ret;
}

static const struct v4l2_subdev_pad_ops tp2855_pad_ops = {
	.enum_mbus_code = tp2855_enum_mbus_code,
	.get_fmt = tp2855_get_fmt,
	.set_fmt = tp2855_set_fmt,
};

static const struct v4l2_subdev_video_ops tp2855_video_ops = {
	.s_stream = tp2855_s_stream,
};

static const struct v4l2_subdev_ops tp2855_subdev_ops = {
	.video = &tp2855_video_ops,
	.pad = &tp2855_pad_ops,
};


static void tp2855_hw_cleanup(struct tp2855_device *tp2855)
{
	tp2855_power_off(tp2855);
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int tp2855_config_v4l2(struct tp2855_device *tp2855)
{
	struct v4l2_subdev *sd;
	struct device *dev = tp2855->dev;
	struct fwnode_handle *ep;
	int ret, i;

	/* Register v4l2 async notifiers for connected Camera subdevices */
	ret = tp2855_v4l2_notifier_register(tp2855);
	if (ret) {
		dev_err(dev, "Unable to register V4L2 async notifiers\n");
		return ret;
	}

	for (i = 0; i < TP2855_NUM_GMSL; i++) {
		tp2855->format[i].width      = DEFAULT_WIDTH;
		tp2855->format[i].height     = DEFAULT_HEIGHT;
		tp2855->format[i].code       = MEDIA_BUS_FMT_UYVY8_1X16;
		tp2855->format[i].field      = V4L2_FIELD_NONE;
		tp2855->format[i].colorspace = V4L2_COLORSPACE_SRGB;
		tp2855->format[i].ycbcr_enc		= V4L2_YCBCR_ENC_DEFAULT;
		tp2855->format[i].quantization	= V4L2_QUANTIZATION_DEFAULT;
		tp2855->format[i].xfer_func		= V4L2_XFER_FUNC_DEFAULT;
	}

	sd = &tp2855->sd;
	sd->dev = tp2855->dev;
	v4l2_i2c_subdev_init(sd, tp2855->client, &tp2855_subdev_ops);

	strscpy(sd->name, DRIVER_NAME, sizeof(sd->name));
	strlcat(sd->name, ".", sizeof(sd->name));
	strlcat(sd->name, dev_name(tp2855->dev), sizeof(sd->name));
	dev_dbg(tp2855->dev, "name %s\n", sd->name);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;

	//init media pads
	tp2855->pads[TP2855_SRC_PAD].flags = MEDIA_PAD_FL_SOURCE;
	for (i = 0; i < TP2855_SRC_PAD; i++)
		tp2855->pads[i].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&sd->entity, TP2855_N_PADS, tp2855->pads);
	if (ret < 0) {
		dev_err(dev, "media_entity_init failed %d\n", ret);
		return ret;
	}

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), TP2855_SRC_PAD,
					     0, 0);
	if (!ep) {
		dev_err(dev, "Unable to retrieve endpoint on \"port@4\"\n");
		ret = -ENOENT;
		goto err_async;
	}
	tp2855->sd.fwnode = ep;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(tp2855->dev, "v4l2_async_register_subdev failed %d\n", ret);
		goto error_media;
	}

	return 0;

error_media:
	fwnode_handle_put(ep);
	media_entity_cleanup(&sd->entity);
err_async:
	tp2855_v4l2_notifier_unregister(tp2855);

	return ret;
}

static int tp2855_parse_of(struct tp2855_device *tp2855)
{
	struct fwnode_handle *ep;
	struct device *dev = tp2855->dev;
	struct device_node *node = NULL;
	struct device_node *sensor_node;
	const char *str;
	int ret;
#if 0
	/* GPIOs */
	tp2855->cam0dvdd_gpio = devm_gpiod_get(tp2855->dev, "cam0dvdd",
					    GPIOD_OUT_LOW);
	if (IS_ERR(tp2855->cam0dvdd_gpio)) {
		dev_err(tp2855->dev, "Can't get cam0dvdd GPIO: %ld\n",
			PTR_ERR(tp2855->cam0dvdd_gpio));
		return PTR_ERR(tp2855->cam0dvdd_gpio);
	}

	tp2855->cam1dvdd_gpio = devm_gpiod_get(tp2855->dev, "cam1dvdd",
						       GPIOD_OUT_LOW);
	if (IS_ERR(tp2855->cam1dvdd_gpio)) {
		dev_err(tp2855->dev, "Can't get cam1dvdd GPIO: %ld\n",
			PTR_ERR(tp2855->cam1dvdd_gpio));
		return PTR_ERR(tp2855->cam1dvdd_gpio);
	}

	/* Regulator */
	tp2855->supply = regulator_get_optional(tp2855->dev, "power");
	if (IS_ERR(tp2855->supply)) {
		if (PTR_ERR(tp2855->supply) == -ENODEV) {
			tp2855->supply = NULL;
		} else {
			dev_err(tp2855->dev, "Can't get regulator: %ld\n",
				PTR_ERR(tp2855->supply));
			return PTR_ERR(tp2855->supply);
		}
	}
#endif
	/* Parse the endpoints */
	for_each_endpoint_of_node(dev->of_node, node) {
		struct tp2855_source *source;
		struct of_endpoint ep;

		of_graph_parse_endpoint(node, &ep);
		dev_info(dev, "Endpoint %pOF on port %d",
			ep.local_node, ep.port);

		if (ep.port > TP2855_NUM_GMSL) {
			dev_err(dev, "Invalid endpoint %s on port %d",
				of_node_full_name(ep.local_node), ep.port);
			continue;
		}

		/* For the source endpoint just parse the bus configuration. */
		if (ep.port == TP2855_SRC_PAD) {
			struct v4l2_fwnode_endpoint vep = {
				.bus_type = V4L2_MBUS_CSI2_DPHY
			};

			ret = v4l2_fwnode_endpoint_parse(
					of_fwnode_handle(node), &vep);
			if (ret) {
				dev_err(dev, "Can not get fwnode endpoint");
				of_node_put(node);
				return ret;
			}

			tp2855->csi2_data_lanes =
				vep.bus.mipi_csi2.num_data_lanes;

			continue;
		}


		if (tp2855->sources[ep.port].fwnode) {
			dev_err(dev,
				"Multiple port endpoints are not supported: %d",
				ep.port);

			continue;
		}

		source = &tp2855->sources[ep.port];
		source->fwnode = fwnode_graph_get_remote_endpoint(
						of_fwnode_handle(node));
		if (!source->fwnode) {
			dev_err(dev,
				"Endpoint %pOF has no remote endpoint connection\n",
				ep.local_node);

			continue;
		}

		tp2855->source_mask |= BIT(ep.port);
		tp2855->nsources++;
	}
	of_node_put(node);

	return ret;
}

static void tp2855_cleanup(struct tp2855_device *tp2855)
{
	v4l2_fwnode_endpoint_free(&tp2855->bus_cfg);
	mutex_destroy(&tp2855->lock);
}

static int tp2855_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tp2855_device *tp2855 = to_tp2855(sd);

	dev_dbg(dev, "tp2855 suspend\n");

	tp2855_hw_cleanup(tp2855);

	return 0;
}

static int tp2855_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tp2855_device *tp2855 = to_tp2855(sd);
	int ret;

	dev_dbg(dev, "tp2855 resume\n");

	mutex_lock(&tp2855->lock);
	ret = tp2855_hw_init(tp2855);
	mutex_unlock(&tp2855->lock);

	return ret;
}

static const struct dev_pm_ops runtime_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tp2855_suspend, tp2855_resume)
};

static int tp2855_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tp2855_device *tp2855;
	int ret = 0, i = 0;

	dev_info(&client->dev, "tp2855 probe start\n");

	tp2855 = devm_kzalloc(&client->dev, sizeof(struct tp2855_device), GFP_KERNEL);
	if (!tp2855) {
		dev_err(&client->dev, "tp2855 probe out of memory\n");
		return -ENOMEM;
	}

	tp2855->dev = &client->dev;
	tp2855->client = client;

	mutex_init(&tp2855->lock);

	ret = tp2855_parse_of(tp2855);
	if (ret < 0)
		goto error;

	ret = tp2855_power_on(tp2855);
	if(ret < 0) {
		dev_err(&client->dev, "%s: cannot power on the tp2855 chip, abort!\n", __func__);
		goto error_hw_cleanup;
	}

	ret = tp2855_detect_chip(tp2855);
	if(ret < 0) {
		dev_err(&client->dev, "%s: cannot detect the tp2855 chip, abort!\n", __func__);
		goto error_hw_cleanup;
	} else {
		//Set default resolution config
		for (i=0; i<4; i++) {
			tp2854_ahd_1080p_hw_init(tp2855, i);
		}
	}

	ret = tp2855_config_v4l2(tp2855);
	if (ret)
		goto error_hw_cleanup;

	dev_info(tp2855->dev, "tp2855 probe sussces\n");

	return 0;

error_hw_cleanup:
	tp2855_hw_cleanup(tp2855);
error:
	tp2855_cleanup(tp2855);
	if(tp2855)
		devm_kfree(&client->dev, tp2855);
	return ret;
}

static int tp2855_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tp2855_device *tp2855 = to_tp2855(sd);

	tp2855_hw_cleanup(tp2855);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct of_device_id tp2855_of_id_table[] = {
	{ .compatible = "tp,tp2855" },
	{ }
};
MODULE_DEVICE_TABLE(of, tp2855_of_id_table);

static struct i2c_driver tp2855_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= tp2855_of_id_table,
		.pm  = &runtime_pm_ops,
	},
	.probe		= tp2855_probe,
	.remove		= tp2855_remove,
};

module_i2c_driver(tp2855_i2c_driver);

MODULE_DESCRIPTION("Driver for tp2855 Video Decoder");
MODULE_LICENSE("GPL");
