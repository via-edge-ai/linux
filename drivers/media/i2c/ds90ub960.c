// SPDX-License-Identifier: GPL-2.0-only
/*
 * ds90ub960.c
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

#define DRIVER_NAME "ds90ub960"
#define DEFAULT_WIDTH  1920
#define DEFAULT_HEIGHT 1080
#define DS90UB960_N_PADS 5
#define DS90UB960_NUM_GMSL 4
#define DS90UB960_SRC_PAD 4
#define MAX_FW_LOAD_RETRIES		5
#define MAX_CHIP_DETECT_RETRIES	5
#define TI960_PORT_SEL(_rxport_) ((1 << ((_rxport_) & 0x3)) | (((_rxport_) & 0x3) << 4))

static int enable_rx_port = 0;

struct ds90ub960_device;

struct ds90ub960_size {
	unsigned int width;
	unsigned int height;
};

struct ds90ub960_source {
	struct v4l2_subdev *sd;
	struct fwnode_handle *fwnode;
};

struct ds90ub960_asd {
	struct v4l2_async_subdev base;
	struct ds90ub960_source *source;
};

static inline struct ds90ub960_asd *to_ds90ub960_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct ds90ub960_asd, base);
}

struct ds90ub960_device {
	struct device *dev;
	struct i2c_client *client;

	struct gpio_desc *cam0dvdd_gpio; //should be gpio60
	struct gpio_desc *cam1dvdd_gpio; //should be gpio58
	struct gpio_desc *cam0avdd_gpio; //should be gpio56
	struct gpio_desc *pnd_gpio;      //should be gpio26
	struct gpio_desc *rx0_en_gpio;   //should be expander gpio0
	struct gpio_desc *rx1_en_gpio;   //should be expander gpio1
	struct gpio_desc *rx2_en_gpio;   //should be expander gpio2
	struct gpio_desc *rx3_en_gpio;   //should be expander gpio3
	struct clk *clock;

	struct v4l2_fwnode_endpoint bus_cfg;

	struct mutex lock;	/* Protects formats */

	struct v4l2_subdev sd;
	struct media_pad pads[DS90UB960_N_PADS];
	struct v4l2_mbus_framefmt format[DS90UB960_N_PADS];

	//v4l2 subdevice source
	unsigned int nsources;
	unsigned int source_mask;
	unsigned int route_mask;
	unsigned int bound_sources;
	unsigned int csi2_data_lanes;
	struct ds90ub960_source sources[DS90UB960_NUM_GMSL];
	struct v4l2_async_notifier notifier;

	//i2c mux
	struct i2c_mux_core *mux;
	unsigned int mux_channel;
	bool mux_open;

	//gpio chipset
	struct gpio_chip gpio;
	u8 gpio_state;

	//struct v4l2_ctrl_handler ctrls;
	struct dentry  *dentry;

	struct regulator *afvdd_supply;
	struct regulator *avdd_supply;
};

static struct ds90ub960_source *next_source(struct ds90ub960_device *ds90ub960,
					  struct ds90ub960_source *source)
{
	if (!source)
		source = &ds90ub960->sources[0];
	else
		source++;

	for (; source < &ds90ub960->sources[DS90UB960_NUM_GMSL]; source++) {
		if (source->fwnode)
			return source;
	}

	return NULL;
}

#define for_each_source(priv, source) \
	for ((source) = NULL; ((source) = next_source((priv), (source))); )

#define to_index(priv, source) ((source) - &(priv)->sources[0])


static inline struct ds90ub960_device *to_ds90ub960(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ds90ub960_device, sd);
}

static int ds90ub960_slave_write(struct ds90ub960_device *ds90ub960,
								 u8 slave, u8 reg, u8 val)
{
	u8 buf[6];
	int ret = 0;
	u8 slave_addr = ds90ub960->client->addr;

	ds90ub960->client->addr = (slave >> 1);

	buf[0] = reg & 0xff;
	buf[1] = val & 0xff;

	ret = i2c_master_send(ds90ub960->client, buf, 2);
	if (ret < 0) {
		dev_err(ds90ub960->dev, "%s: register 0x%02x %s failed: %d\n",
			__func__, reg, "write", ret);
		ds90ub960->client->addr = slave_addr;
		return ret;
	}

	ds90ub960->client->addr = slave_addr;

	return 0;
}


static int ds90ub960_write(struct ds90ub960_device *ds90ub960, u8 reg, u8 val)
{
	u8 buf[6];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = val & 0xff;

	ret = i2c_master_send(ds90ub960->client, buf, 2);
	if (ret < 0) {
		dev_err(ds90ub960->dev, "%s: register 0x%02x %s failed: %d\n",
			__func__, reg, "write", ret);
		return ret;
	}

	return 0;
}

static int ds90ub960_slave_read(struct ds90ub960_device *ds90ub960, 
								u8 slave, u8 addr, u8 *data)
{
	int ret;
	u8 slave_addr = ds90ub960->client->addr;

	ds90ub960->client->addr = (slave >> 1);

	ret = i2c_master_send(ds90ub960->client, &addr, sizeof(addr));
	if (ret < 0) {
		dev_err(ds90ub960->dev, "I2C write address(%02x) fail ret = %d\n",
			addr, ret);
		ds90ub960->client->addr = slave_addr;
		return ret;
	}

	ret = i2c_master_recv(ds90ub960->client, data, 1);
	if (ret < 0) {
		dev_err(ds90ub960->dev, "I2C read value fail ret = %d\n",
			ret);
		ds90ub960->client->addr = slave_addr;
		return ret;
	}

	ds90ub960->client->addr = slave_addr;

	return 0;
}

static int ds90ub960_read(struct ds90ub960_device *ds90ub960, u8 addr, u8 *data)
{
	int ret;

	ret = i2c_master_send(ds90ub960->client, &addr, sizeof(addr));
	if (ret < 0) {
		dev_err(ds90ub960->dev, "I2C write address(%02x) fail ret = %d\n",
			addr, ret);
		return ret;
	}

	ret = i2c_master_recv(ds90ub960->client, data, 1);
	if (ret < 0) {
		dev_err(ds90ub960->dev, "I2C read value fail ret = %d\n",
			ret);
		return ret;
	}

	return 0;
}

static inline int ds90ub960_request_gpio(
	struct ds90ub960_device *ds90ub960, int gpio_num, char* lable)
{
	int ret = 0;
	ret = gpio_request(gpio_num, lable);
	if (ret)
		dev_err(ds90ub960->dev, "%s: request gpio(%d) fail(%d)",
			__func__, gpio_num, ret);

	ret = gpio_direction_output(gpio_num, 0);
	if (ret)
		dev_err(ds90ub960->dev, "%s: gpio(%d) direction output fail(%d)",
			__func__, gpio_num, ret);
	
	return ret;
}

static int ds90ub960_set_rx_port(void *data, u64 val)
{
	enable_rx_port = (val < 0 || val >= 4) ? 0 : val;
	printk("Set RX port:%d", enable_rx_port);
	return 0;
}

static int ds90ub960_get_rx_port(void *data, u64 *val)
{
	*val = enable_rx_port;
	printk("Get RX port:%d", enable_rx_port);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ds90ub960_rx_port,
	ds90ub960_get_rx_port,
	ds90ub960_set_rx_port, "%16llu");

/* -----------------------------------------------------------------------------
 * I2C Multiplexer
 */

static void ds90ub960_i2c_mux_configure(struct ds90ub960_device *ds90ub960, u8 conf)
{
	ds90ub960_write(ds90ub960, 0x4c, conf);

	/*
	 * We must sleep after any change to the forward or reverse channel
	 * configuration.
	 */
	usleep_range(3000, 5000);
}

static void ds90ub960_i2c_mux_open(struct ds90ub960_device *ds90ub960)
{
	ds90ub960_i2c_mux_configure(ds90ub960, 0x0f);

	ds90ub960->mux_open = true;
}

static void ds90ub960_i2c_mux_close(struct ds90ub960_device *ds90ub960)
{
	/*
	 * Ensure that both the forward and reverse channel are disabled on the
	 * mux, and that the channel ID is invalidated to ensure we reconfigure
	 * on the next max9286_i2c_mux_select() call.
	 */
	ds90ub960_i2c_mux_configure(ds90ub960, 0x00);

	ds90ub960->mux_open = false;
	ds90ub960->mux_channel = -1;
}

static int ds90ub960_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct ds90ub960_device *ds90ub960 = i2c_mux_priv(muxc);

	/* Channel select is disabled when configured in the opened state. */
	if (ds90ub960->mux_open)
		return 0;

	if (ds90ub960->mux_channel == chan)
		return 0;

	if (chan > 3) return -1;

	ds90ub960->mux_channel = chan;

	ds90ub960_i2c_mux_configure(ds90ub960, TI960_PORT_SEL(chan));

	return 0;
}

static int ds90ub960_i2c_mux_init(struct ds90ub960_device *ds90ub960)
{
	struct ds90ub960_source *source;
	int ret;

	if (!i2c_check_functionality(ds90ub960->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	ds90ub960->mux = i2c_mux_alloc(ds90ub960->client->adapter, &ds90ub960->client->dev,
				  ds90ub960->nsources, 0, I2C_MUX_LOCKED,
				  ds90ub960_i2c_mux_select, NULL);
	if (!ds90ub960->mux)
		return -ENOMEM;

	ds90ub960->mux->priv = ds90ub960;

	for_each_source(ds90ub960, source) {
		unsigned int index = to_index(ds90ub960, source);

		ret = i2c_mux_add_adapter(ds90ub960->mux, 0, index, 0);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	i2c_mux_del_adapters(ds90ub960->mux);
	return ret;
}

#if 0
static void ds90ub960_configure_i2c(struct ds90ub960_device *ds90ub960, bool localack)
{
	u8 config = MAX9286_I2CSLVSH_469NS_234NS | MAX9286_I2CSLVTO_1024US |
		    MAX9286_I2CMSTBT_105KBPS;

	if (localack)
		config |= MAX9286_I2CLOCACK;

	ds90ub960_write(ds90ub960, 0x58, config);
	usleep_range(3000, 5000);
}
#endif
//i2c mux end

/* -----------------------------------------------------------------------------
 * V4L2 Subdev
 */

static int ds90ub960_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct ds90ub960_device *ds90ub960 = to_ds90ub960(notifier->sd);
	struct ds90ub960_source *source = to_ds90ub960_asd(asd)->source;
	unsigned int index = to_index(ds90ub960, source);
	unsigned int src_pad;
	int ret;

	ret = media_entity_get_fwnode_pad(&subdev->entity,
					  source->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(&ds90ub960->client->dev,
			"Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	ds90ub960->bound_sources |= BIT(index);
	source->sd = subdev;
	src_pad = ret;

	ret = media_create_pad_link(&source->sd->entity, src_pad,
				    &ds90ub960->sd.entity, index,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(&ds90ub960->client->dev,
			"Unable to link %s:%u -> %s:%u\n",
			source->sd->name, src_pad, ds90ub960->sd.name, index);
		return ret;
	}

	dev_dbg(&ds90ub960->client->dev, "Bound %s pad: %u on index %u\n",
		subdev->name, src_pad, index);

	return 0;
}

static void ds90ub960_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
	struct ds90ub960_device *ds90ub960 = to_ds90ub960(notifier->sd);
	struct ds90ub960_source *source = to_ds90ub960_asd(asd)->source;
	unsigned int index = to_index(ds90ub960, source);

	source->sd = NULL;
	ds90ub960->bound_sources &= ~BIT(index);
}

static const struct v4l2_async_notifier_operations ds90ub960_notify_ops = {
	.bound = ds90ub960_notify_bound,
	.unbind = ds90ub960_notify_unbind,
};

static int ds90ub960_v4l2_notifier_register(struct ds90ub960_device *ds90ub960)
{
	struct device *dev = &ds90ub960->client->dev;
	struct ds90ub960_source *source = NULL;
	int ret;

	if (!ds90ub960->nsources)
		return 0;

	v4l2_async_notifier_init(&ds90ub960->notifier);

	for_each_source(ds90ub960, source) {
		unsigned int i = to_index(ds90ub960, source);
		struct ds90ub960_asd *mas;

		mas = v4l2_async_notifier_add_fwnode_subdev(&ds90ub960->notifier,
							    source->fwnode,
							    struct ds90ub960_asd);
		if (IS_ERR(mas)) {
			dev_err(dev, "Failed to add subdev for source %u: %ld",
				i, PTR_ERR(mas));
			v4l2_async_notifier_cleanup(&ds90ub960->notifier);
			return PTR_ERR(mas);
		}

		mas->source = source;
	}

	ds90ub960->notifier.ops = &ds90ub960_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&ds90ub960->sd, &ds90ub960->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&ds90ub960->notifier);
		return ret;
	}

	return 0;
}

static void ds90ub960_v4l2_notifier_unregister(struct ds90ub960_device *ds90ub960)
{
	if (!ds90ub960->nsources)
		return;

	v4l2_async_notifier_unregister(&ds90ub960->notifier);
	v4l2_async_notifier_cleanup(&ds90ub960->notifier);
}


//gpio chip 
static int ds90ub960_gpio_set(struct ds90ub960_device *ds90ub960,
								unsigned int offset, int value)
{
	struct device *dev = &ds90ub960->client->dev;
	u8 addr = 0x10 + offset;
	u8 data;

	if (value) {
		ds90ub960->gpio_state |= BIT(offset);
		data = (0x4 << 2) | (0x1 << 1) | 0x1;
	} else {
		ds90ub960->gpio_state &= ~BIT(offset);
		data = (0x4 << 2) | (0x0 << 1) | 0x1;
	}

	dev_err(dev, "%s addr(0x%02x) data(0x%02x)", __func__, addr, data);

	return ds90ub960_write(ds90ub960, addr, data);
}

static void ds90ub960_gpiochip_set(struct gpio_chip *chip,
				 unsigned int offset, int value)
{
	struct ds90ub960_device *ds90ub960 = gpiochip_get_data(chip);

	ds90ub960_gpio_set(ds90ub960, offset, value);
}

static int ds90ub960_gpiochip_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ds90ub960_device *ds90ub960 = gpiochip_get_data(chip);

	return ds90ub960->gpio_state & BIT(offset);
}

static int ds90ub960_register_gpio(struct ds90ub960_device *ds90ub960)
{
	struct device *dev = &ds90ub960->client->dev;
	struct gpio_chip *gpio = &ds90ub960->gpio;
	int ret;

	/* Configure the GPIO */
	gpio->label = dev_name(dev);
	gpio->parent = dev;
	gpio->owner = THIS_MODULE;
	gpio->of_node = dev->of_node;
	gpio->ngpio = 8;
	gpio->base = -1;
	gpio->set = ds90ub960_gpiochip_set;
	gpio->get = ds90ub960_gpiochip_get;
	gpio->can_sleep = true;

	ret = devm_gpiochip_add_data(dev, gpio, ds90ub960);
	if (ret)
		dev_err(dev, "Unable to create gpio_chip\n");

	return ret;
}
//end gpio chip

static int ds90ub960_power_on(struct ds90ub960_device *ds90ub960)
{
	int ret;

	if (ds90ub960->afvdd_supply) {
		regulator_set_voltage(ds90ub960->afvdd_supply, 2800000, 2800000);
		ret = regulator_enable(ds90ub960->afvdd_supply);
		if (ret < 0) {
			dev_err(ds90ub960->dev, "Failed to enable afvdd regulator: %d\n", ret);
			return ret;
		}
		usleep_range(200, 1000);
	}

	if (ds90ub960->avdd_supply) {
		regulator_set_voltage(ds90ub960->avdd_supply, 2800000, 2800000);
		ret = regulator_enable(ds90ub960->avdd_supply);
		if (ret < 0) {
			dev_err(ds90ub960->dev, "Failed to enable avdd regulator: %d\n", ret);
			return ret;
		}
		usleep_range(200, 1000);
	}

	
	//Enable RX port power
	if (ds90ub960->rx0_en_gpio) {
		gpiod_set_value_cansleep(ds90ub960->rx0_en_gpio,1);
		msleep(5);
	}


	if (ds90ub960->rx1_en_gpio) {
		gpiod_set_value_cansleep(ds90ub960->rx1_en_gpio,1);
		msleep(5);
	}


	if (ds90ub960->rx2_en_gpio) {
		gpiod_set_value_cansleep(ds90ub960->rx2_en_gpio,1);
		msleep(5);
	}

	if (ds90ub960->rx3_en_gpio) {
		gpiod_set_value_cansleep(ds90ub960->rx3_en_gpio,1);
		msleep(5);
	}

	//Power On Sequence¡G VDD18&VDDIO-->VDD11-->PDB

	

	/* Enable 24M clock */
	ret = clk_prepare_enable(ds90ub960->clock);
	if (ret < 0) {
		dev_err(ds90ub960->dev, "Failed to enable clock: %d\n", ret);
		return ret;
	}
	msleep(50);

	return 0;
}

static void ds90ub960_power_off(struct ds90ub960_device *ds90ub960)
{

	/* Turn the clock off. */
	clk_disable_unprepare(ds90ub960->clock);

	/* Disable VDD gpio */
	if (ds90ub960->rx0_en_gpio) {
		gpiod_set_value_cansleep(ds90ub960->rx0_en_gpio,0);
		msleep(5);
	}


	if (ds90ub960->rx1_en_gpio) {
		gpiod_set_value_cansleep(ds90ub960->rx1_en_gpio,0);
		msleep(5);
	}

	if (ds90ub960->rx2_en_gpio) {
		gpiod_set_value_cansleep(ds90ub960->rx2_en_gpio,0);
		msleep(5);
	}

	if (ds90ub960->rx3_en_gpio) {
		gpiod_set_value_cansleep(ds90ub960->rx3_en_gpio,0);
		msleep(5);
	}

}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

static int ds90ub960_configure(struct ds90ub960_device *ds90ub960)
{
	//const struct ds90ub960_format *format = &ap1302->formats[AP1302_PAD_SOURCE];
	unsigned int data_lanes = ds90ub960->bus_cfg.bus.mipi_csi2.num_data_lanes;
	int ret = 0;

	if (ret < 0)
		return ret;

	dev_dbg(ds90ub960->dev, "get mipi data lanes: %d\n", data_lanes);

	return 0;
}

static int ds90ub960_load_config(struct ds90ub960_device *ds90ub960)
{

	const char config_table[69][2] = {
		{0x21,0x21},{0x10,0x91},{0x11,0x85},{0x12,0xc1},{0x13,0xc5},
		{0x14,0xc9},{0x15,0xcd},{0x16,0x51},{0x17,0x59},{0x1f,0x00},
		{0x32,0x01},{0x33,0x02},{0x20,0xfc},{0x4c,0x0f},{0x7c,0xc0},
		{0x6d,0x7c},
		//rx0
		{0x4c,0x01},{0x58,0xDE},{0x5c,0x30},{0x5d,0x6C},{0x65,0x6C},
		{0x60,0x30},{0x68,0x30},{0x6e,0x88},{0x6f,0x8a},{0x70,0x1E},
		{0x71,0x1E},{0x72,0x00},
		//rx1
		{0x4c,0x12},{0x58,0xDE},{0x5c,0x30},{0x5d,0x6C},{0x65,0x6C},
		{0x60,0x30},{0x68,0x30},{0x6e,0x88},{0x6f,0x8a},{0x70,0x5E},
		{0x71,0x5E},{0x72,0x01},
		//rx2
		{0x4c,0x24},{0x58,0xDE},{0x5c,0x30},{0x5d,0x6C},{0x65,0x6C},
		{0x60,0x30},{0x68,0x30},{0x6e,0x88},{0x6f,0x8a},{0x70,0x9E},
		{0x71,0x9E},{0x72,0x02},
		//rx3
		{0x4c,0x38},{0x58,0xDE},{0x5c,0x30},{0x5d,0x6C},{0x65,0x6C},
		{0x60,0x30},{0x68,0x30},{0x6e,0x88},{0x6f,0x8a},{0x70,0xDE},
		{0x71,0xDE},{0x72,0x03},
		{0x1a,0x0C},{0x1b,0xB7},{0x1c,0x35},{0x33,0x03},{0x20,0x00}
	};

	#define REG_ADDR 0
	#define REG_VAL  1
	int i, ret = 0;
	int reg_size = sizeof(config_table) / sizeof(config_table[0]);

	dev_dbg(ds90ub960->dev, 
		"Start config ds90ub960, config table size(%d)", reg_size);

	for (i = 0; i < reg_size; i++) {
		ret = ds90ub960_write(ds90ub960, config_table[i][REG_ADDR], config_table[i][REG_VAL]);
		if (ret) {
			dev_err(ds90ub960->dev, "Withe reg(0x%02x) = 0x%02x FAIL(%d)",
				config_table[i][REG_ADDR], config_table[i][REG_VAL], ret);
			break;
		}
	}

	dev_info(ds90ub960->dev, "Set %d VC map", ds90ub960->client->adapter->nr);

	//Set csi port0 vc map
	if (ds90ub960->client->adapter->nr == 0) {
		const char table[8][2] = { 
			{0x4c,0x01}, {0x72,0x03}, {0x4c,0x12}, {0x72,0x02},
			{0x4c,0x24}, {0x72,0x01}, {0x4c,0x38}, {0x72,0x00},
		};
		reg_size = sizeof(table) / sizeof(table[0]);

		dev_info(ds90ub960->dev, "Set %s VC map, table size(%d)", 
				dev_name(ds90ub960->dev), reg_size);

		for (i = 0; i < reg_size; i++) {
			ret = ds90ub960_write(ds90ub960, table[i][REG_ADDR], table[i][REG_VAL]);
			if (ret) {
				dev_err(ds90ub960->dev, "Withe reg(0x%02x) = 0x%02x FAIL(%d)",
					table[i][REG_ADDR], table[i][REG_VAL], ret);
				break;
			}
		}
	}

	return ret;
}

static int ds90ub960_detect_chip(struct ds90ub960_device *ds90ub960)
{
	u8 chip_id;
	unsigned int retries;
	int ret = 0;

	#define DS90UB960_CHIP_ID_ADDR 0x00
	#define DS90UB960_CHIP_ID      0x60

	for (retries = 0; retries < MAX_CHIP_DETECT_RETRIES; ++retries) {
		ret = ds90ub960_read(ds90ub960, DS90UB960_CHIP_ID_ADDR, &chip_id);

		if (chip_id == DS90UB960_CHIP_ID)
			break;

		msleep(100);
	}

	if (retries == MAX_CHIP_DETECT_RETRIES) {
		dev_err(ds90ub960->dev,
			"Invalid chip id, expected 0x%02x, got 0x%02x\n",
			DS90UB960_CHIP_ID, chip_id);
		return -EINVAL;
	}

	dev_info(ds90ub960->dev, "DS90UB960 get chip id(0x%02x)\n", chip_id);

	return 0;
}

static int ds90ub960_hw_init(struct ds90ub960_device *ds90ub960)
{
	unsigned int retries;
	int ret = 0;

	for (retries = 0; retries < MAX_FW_LOAD_RETRIES; ++retries) {
		ret = ds90ub960_power_on(ds90ub960);
		if (ret < 0) {
			dev_err(ds90ub960->dev, "DS90UB960 power up fail. ret = %d\n", ret);
			continue;
		}

		ret = ds90ub960_detect_chip(ds90ub960);
		if (ret < 0) {
			dev_err(ds90ub960->dev, "DS90UB960 detect id fail. ret = %d\n", ret);
			continue;
		}

		ret = ds90ub960_load_config(ds90ub960);
		if (!ret)
			break;

		//ds90ub960_power_off(ds90ub960);
	}

	if (retries == MAX_FW_LOAD_RETRIES) {
		dev_err(ds90ub960->dev,
			"Can not detect chip\n");
		ret = -ETIMEDOUT;
	}

	return ret;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static struct v4l2_mbus_framefmt *
ds90ub960_get_pad_format(struct ds90ub960_device *ds90ub960,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ds90ub960->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ds90ub960->format[pad];
	default:
		return NULL;
	}
}

static int ds90ub960_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;

	return 0;
}

static int ds90ub960_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ds90ub960_device *ds90ub960 = to_ds90ub960(sd);
	struct v4l2_mbus_framefmt *format;

	if (fmt->pad == DS90UB960_SRC_PAD)
		fmt->pad = __ffs(ds90ub960->bound_sources);

	format = ds90ub960_get_pad_format(ds90ub960, sd_state, fmt->pad, fmt->which);

	mutex_lock(&ds90ub960->lock);
	fmt->format = *format;
	mutex_unlock(&ds90ub960->lock);

	return 0;
}

static int ds90ub960_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ds90ub960_device *ds90ub960 = to_ds90ub960(sd);
	struct v4l2_mbus_framefmt *format;

	if (fmt->pad == DS90UB960_SRC_PAD)
		return -EINVAL;

	format = ds90ub960_get_pad_format(ds90ub960, sd_state, fmt->pad, fmt->which);

	mutex_lock(&ds90ub960->lock);
	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = MEDIA_BUS_FMT_UYVY8_1X16;
	mutex_unlock(&ds90ub960->lock);

	dev_dbg(ds90ub960->dev, 
	"[%s:%d] format->width: %d, format->height: %d, format->code: %x\n",
			__func__, __LINE__, format->width, format->height, format->code);

	fmt->format = *format;

	return 0;
}

static int ds90ub960_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ds90ub960_device *ds90ub960 = to_ds90ub960(sd);
	struct ds90ub960_source *source;
	int ret;

	mutex_lock(&ds90ub960->lock);

	if (enable) {
		ret = ds90ub960_hw_init(ds90ub960);
		if (ret)
			goto done;
		
		ret = ds90ub960_configure(ds90ub960);
		if (ret < 0)
			goto done;

		/* Start all cameras. */
		for_each_source(ds90ub960, source) {
			ret = v4l2_subdev_call(source->sd, video, s_stream, 1);
			if (ret)
				goto done;
		}
		
	}// else {
	//	ds90ub960_power_off(ds90ub960);
	//}

done:
	mutex_unlock(&ds90ub960->lock);

	if (ret < 0)
		dev_err(ds90ub960->dev, "Failed to %s stream: %d\n",
			enable ? "start" : "stop", ret);

	return ret;
}

static const struct v4l2_subdev_pad_ops ds90ub960_pad_ops = {
	.enum_mbus_code = ds90ub960_enum_mbus_code,
	.get_fmt = ds90ub960_get_fmt,
	.set_fmt = ds90ub960_set_fmt,
};

static const struct v4l2_subdev_video_ops ds90ub960_video_ops = {
	.s_stream = ds90ub960_s_stream,
};

static const struct v4l2_subdev_ops ds90ub960_subdev_ops = {
	.video = &ds90ub960_video_ops,
	.pad = &ds90ub960_pad_ops,
};


static void ds90ub960_hw_cleanup(struct ds90ub960_device *ds90ub960)
{
	ds90ub960_power_off(ds90ub960);
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int ds90ub960_config_v4l2(struct ds90ub960_device *ds90ub960)
{
	struct v4l2_subdev *sd;
	struct device *dev = ds90ub960->dev;
	struct fwnode_handle *ep;
	int ret, i;

	/* Register v4l2 async notifiers for connected Camera subdevices */
	ret = ds90ub960_v4l2_notifier_register(ds90ub960);
	if (ret) {
		dev_err(dev, "Unable to register V4L2 async notifiers\n");
		return ret;
	}

	for (i = 0;i < DS90UB960_NUM_GMSL;i++) {
		ds90ub960->format[i].width      = DEFAULT_WIDTH;
		ds90ub960->format[i].height     = DEFAULT_HEIGHT;
		ds90ub960->format[i].code       = MEDIA_BUS_FMT_UYVY8_1X16;
		ds90ub960->format[i].field      = V4L2_FIELD_NONE;
		ds90ub960->format[i].colorspace = V4L2_COLORSPACE_SRGB;
		ds90ub960->format[i].ycbcr_enc		= V4L2_YCBCR_ENC_DEFAULT;
		ds90ub960->format[i].quantization	= V4L2_QUANTIZATION_DEFAULT;
		ds90ub960->format[i].xfer_func		= V4L2_XFER_FUNC_DEFAULT;
	}

	sd = &ds90ub960->sd;
	sd->dev = ds90ub960->dev;
	v4l2_i2c_subdev_init(sd, ds90ub960->client, &ds90ub960_subdev_ops);

	strscpy(sd->name, DRIVER_NAME, sizeof(sd->name));
	strlcat(sd->name, ".", sizeof(sd->name));
	strlcat(sd->name, dev_name(ds90ub960->dev), sizeof(sd->name));
	dev_dbg(ds90ub960->dev, "name %s\n", sd->name);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;

	//init media pads
	ds90ub960->pads[DS90UB960_SRC_PAD].flags = MEDIA_PAD_FL_SOURCE;
	for (i = 0; i < DS90UB960_SRC_PAD; i++)
		ds90ub960->pads[i].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&ds90ub960->sd.entity, DS90UB960_N_PADS,
				     ds90ub960->pads);
	if (ret < 0) {
		dev_err(ds90ub960->dev, "media_entity_init failed %d\n", ret);
		return ret;
	}

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), DS90UB960_SRC_PAD,
					     0, 0);
	if (!ep) {
		dev_err(dev, "Unable to retrieve endpoint on \"port@4\"\n");
		ret = -ENOENT;
		goto err_async;
	}
	ds90ub960->sd.fwnode = ep;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(ds90ub960->dev, "v4l2_async_register_subdev failed %d\n", ret);
		goto error_media;
	}

	return 0;
	
error_media:
	fwnode_handle_put(ep);
	media_entity_cleanup(&sd->entity);
err_async:
	ds90ub960_v4l2_notifier_unregister(ds90ub960);
	
	return ret;
}

static int ds90ub960_parse_of(struct ds90ub960_device *ds90ub960)
{
	//struct fwnode_handle *ep;
	struct device *dev = &ds90ub960->client->dev;
	struct device_node *i2c_mux;
	struct device_node *node = NULL;
	unsigned int i2c_mux_mask = 0;
	int ret = 0;

	/* GPIOs */
	ds90ub960->rx0_en_gpio = devm_gpiod_get(ds90ub960->dev, "rx0",
						       GPIOD_OUT_LOW);
	if (IS_ERR(ds90ub960->rx0_en_gpio)) {
		dev_err(ds90ub960->dev, "Can't get rx0 enable GPIO: %ld\n",
			PTR_ERR(ds90ub960->rx0_en_gpio));
		return PTR_ERR(ds90ub960->rx0_en_gpio);
	}

	ds90ub960->rx1_en_gpio = devm_gpiod_get(ds90ub960->dev, "rx1",
						       GPIOD_OUT_LOW);
	if (IS_ERR(ds90ub960->rx1_en_gpio)) {
		dev_err(ds90ub960->dev, "Can't get rx1 enable GPIO: %ld\n",
			PTR_ERR(ds90ub960->rx1_en_gpio));
		return PTR_ERR(ds90ub960->rx1_en_gpio);
	}

	ds90ub960->rx2_en_gpio = devm_gpiod_get(ds90ub960->dev, "rx2",
						       GPIOD_OUT_LOW);
	if (IS_ERR(ds90ub960->rx2_en_gpio)) {
		dev_err(ds90ub960->dev, "Can't get rx2 enable GPIO: %ld\n",
			PTR_ERR(ds90ub960->rx2_en_gpio));
		return PTR_ERR(ds90ub960->rx2_en_gpio);
	}

	ds90ub960->rx3_en_gpio = devm_gpiod_get(ds90ub960->dev, "rx3",
						       GPIOD_OUT_LOW);
	if (IS_ERR(ds90ub960->rx3_en_gpio)) {
		dev_err(ds90ub960->dev, "Can't get rx3 enable GPIO: %ld\n",
			PTR_ERR(ds90ub960->rx3_en_gpio));
		return PTR_ERR(ds90ub960->rx3_en_gpio);
	}

	/* Regulator */
	ds90ub960->afvdd_supply = regulator_get_optional(ds90ub960->dev, "afvdd");
	if (IS_ERR(ds90ub960->afvdd_supply)) {
		if (PTR_ERR(ds90ub960->afvdd_supply) == -ENODEV) {
			ds90ub960->afvdd_supply = NULL;
			dev_err(ds90ub960->dev, "1Can't get af vdd regulator: %ld\n",
				PTR_ERR(ds90ub960->afvdd_supply));
		} else {
			dev_err(ds90ub960->dev, "2Can't get af vdd regulator: %ld\n",
				PTR_ERR(ds90ub960->afvdd_supply));
		}
	}

	ds90ub960->avdd_supply = regulator_get_optional(ds90ub960->dev, "avdd");
	if (IS_ERR(ds90ub960->avdd_supply)) {
		if (PTR_ERR(ds90ub960->avdd_supply) == -ENODEV) {
			ds90ub960->avdd_supply = NULL;
			dev_err(ds90ub960->dev, "1Can't get avdd regulator: %ld\n",
				PTR_ERR(ds90ub960->avdd_supply));
		} else {
			dev_err(ds90ub960->dev, "2Can't get avdd regulator: %ld\n",
				PTR_ERR(ds90ub960->avdd_supply));
		}
	}

	/* Balance the of_node_put() performed by of_find_node_by_name(). */
	of_node_get(dev->of_node);
	i2c_mux = of_find_node_by_name(dev->of_node, "i2c-mux");
	if (!i2c_mux) {
		dev_err(dev, "Failed to find i2c-mux node\n");
		return -EINVAL;
	}

	/* Identify which i2c-mux channels are enabled */
	for_each_child_of_node(i2c_mux, node) {
		u32 id = 0;

		of_property_read_u32(node, "reg", &id);
		if (id >= DS90UB960_NUM_GMSL)
			continue;

		if (!of_device_is_available(node)) {
			dev_dbg(dev, "Skipping disabled I2C bus port %u\n", id);
			continue;
		}

		i2c_mux_mask |= BIT(id);
	}
	of_node_put(node);
	of_node_put(i2c_mux);

	/* Parse the endpoints */
	for_each_endpoint_of_node(dev->of_node, node) {
		struct ds90ub960_source *source;
		struct of_endpoint ep;

		of_graph_parse_endpoint(node, &ep);
		dev_info(dev, "Endpoint %pOF on port %d",
			ep.local_node, ep.port);

		if (ep.port > DS90UB960_NUM_GMSL) {
			dev_err(dev, "Invalid endpoint %s on port %d",
				of_node_full_name(ep.local_node), ep.port);
			continue;
		}

		/* For the source endpoint just parse the bus configuration. */
		if (ep.port == DS90UB960_SRC_PAD) {
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

			ds90ub960->csi2_data_lanes =
				vep.bus.mipi_csi2.num_data_lanes;

			continue;
		}

		/* Skip if the corresponding GMSL link is unavailable. */
		if (!(i2c_mux_mask & BIT(ep.port)))
			continue;

		if (ds90ub960->sources[ep.port].fwnode) {
			dev_err(dev,
				"Multiple port endpoints are not supported: %d",
				ep.port);

			continue;
		}

		source = &ds90ub960->sources[ep.port];
		source->fwnode = fwnode_graph_get_remote_endpoint(
						of_fwnode_handle(node));
		if (!source->fwnode) {
			dev_err(dev,
				"Endpoint %pOF has no remote endpoint connection\n",
				ep.local_node);

			continue;
		}

		ds90ub960->source_mask |= BIT(ep.port);
		ds90ub960->nsources++;
	}
	of_node_put(node);

#if 0
	/* Bus configuration */
	ep = fwnode_graph_get_next_endpoint(dev_fwnode(ds90ub960->dev), NULL);
	if (!ep)
		return -EINVAL;
	
	ds90ub960->bus_cfg.bus_type = V4L2_MBUS_CSI2_DPHY;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &ds90ub960->bus_cfg);
	if (ret < 0) {
		dev_err(ds90ub960->dev, "Failed to parse bus configuration\n");
		//return ret;
	}
#endif

	return ret;
}

static void ds90ub960_cleanup(struct ds90ub960_device *ds90ub960)
{
	v4l2_fwnode_endpoint_free(&ds90ub960->bus_cfg);
	mutex_destroy(&ds90ub960->lock);
}

static int ds90ub960_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90ub960_device *ds90ub960 = to_ds90ub960(sd);

	dev_dbg(dev, "ds90ub960 suspend\n");

	ds90ub960_hw_cleanup(ds90ub960);

	return 0;
}

static int ds90ub960_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90ub960_device *ds90ub960 = to_ds90ub960(sd);
	int ret;

	dev_dbg(dev, "ds90ub960 resume\n");

	mutex_lock(&ds90ub960->lock);
	ret = ds90ub960_hw_init(ds90ub960);
	mutex_unlock(&ds90ub960->lock);

	return ret;
}

static const struct dev_pm_ops runtime_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ds90ub960_suspend, ds90ub960_resume)
};

static int ds90ub960_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ds90ub960_device *ds90ub960;
	int ret;

	ds90ub960 = devm_kzalloc(&client->dev, sizeof(struct ds90ub960_device), GFP_KERNEL);
	if (!ds90ub960) {
		dev_err(&client->dev, "ds90ub960 probe out of memory\n");
		return -ENOMEM;
	}

	ds90ub960->dev = &client->dev;
	ds90ub960->client = client;

	mutex_init(&ds90ub960->lock);

	ret = ds90ub960_parse_of(ds90ub960);
	if (ret < 0)
		goto error;

	ret = ds90ub960_config_v4l2(ds90ub960);
	if (ret)
		goto error_hw_cleanup;

	ret = ds90ub960_i2c_mux_init(ds90ub960);
	if (ret) {
		dev_err(&client->dev, "Unable to initialize I2C multiplexer\n");
		goto error_hw_cleanup;
	}

	ret = ds90ub960_register_gpio(ds90ub960);
	if (ret) {
		dev_err(&client->dev, "Unable to register gpio\n");
		goto error_hw_cleanup;
	}

	ds90ub960->dentry = debugfs_create_dir("ds90ub960_debug", NULL);

	if (!debugfs_create_file("ds90ub960_rx_port",
		0644,
		ds90ub960->dentry, NULL,
		&ds90ub960_rx_port)) {
		dev_err(&client->dev, "failed to create cam_ife_csid_debug");
		goto error;
	}

	dev_info(ds90ub960->dev, "ds90ub960 probe sussces\n");

	return 0;

error_hw_cleanup:
	ds90ub960_hw_cleanup(ds90ub960);
error:
	ds90ub960_cleanup(ds90ub960);
	return ret;
}

static int ds90ub960_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90ub960_device *ds90ub960 = to_ds90ub960(sd);

	ds90ub960_hw_cleanup(ds90ub960);

	ds90ub960_v4l2_notifier_unregister(ds90ub960);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct of_device_id ds90ub960_of_id_table[] = {
	{ .compatible = "ti,ds90ub960" },
	{ }
};
MODULE_DEVICE_TABLE(of, ds90ub960_of_id_table);

static struct i2c_driver ds90ub960_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= ds90ub960_of_id_table,
		.pm  = &runtime_pm_ops,
	},
	.probe		= ds90ub960_probe,
	.remove		= ds90ub960_remove,
};

module_i2c_driver(ds90ub960_i2c_driver);

MODULE_DESCRIPTION("Driver for ds90ub960 Deserializer");
MODULE_LICENSE("GPL");
