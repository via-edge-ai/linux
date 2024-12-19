// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define DW9807_MAX_FOCUS_POS	1023
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position.
 */
#define DW9807_FOCUS_STEPS	1
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9807_CTRL_STEPS	16
#define DW9807_CTRL_DELAY_US	1000

#define DW9807_CTL_ADDR		0x02
/*
 * DW9807 separates two registers to control the VCM position.
 * One for MSB value, another is LSB value.
 */
#define DW9807_MSB_ADDR		0x03
#define DW9807_LSB_ADDR		0x04
#define DW9807_STATUS_ADDR	0x05
#define DW9807_MODE_ADDR	0x06
#define DW9807_RESONANCE_ADDR	0x07

#define MAX_RETRY		10

struct dw9807_device {
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev sd;
	u16 current_val;
	bool standalone;
	struct v4l2_device v4l2_dev;
	struct i2c_client *client;
};

static inline struct dw9807_device *sd_to_dw9807_vcm(
					struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9807_device, sd);
}

static int dw9807_i2c_check(struct i2c_client *client)
{
	const char status_addr = DW9807_STATUS_ADDR;
	char status_result;
	int ret;

	ret = i2c_master_send(client, &status_addr, sizeof(status_addr));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write STATUS address fail ret = %d\n",
			ret);
		return ret;
	}

	ret = i2c_master_recv(client, &status_result, sizeof(status_result));
	if (ret < 0) {
		dev_err(&client->dev, "I2C read STATUS value fail ret = %d\n",
			ret);
		return ret;
	}

	return status_result;
}

static int dw9807_set_dac(struct i2c_client *client, u16 data)
{
	const char tx_data[3] = {
		DW9807_MSB_ADDR, ((data >> 8) & 0x03), (data & 0xff)
	};
	int val, ret;

	/*
	 * According to the datasheet, need to check the bus status before we
	 * write VCM position. This ensure that we really write the value
	 * into the register
	 */
	ret = readx_poll_timeout(dw9807_i2c_check, client, val, val <= 0,
			DW9807_CTRL_DELAY_US, MAX_RETRY * DW9807_CTRL_DELAY_US);

	if (ret || val < 0) {
		if (ret) {
			dev_warn(&client->dev,
				"Cannot do the write operation because VCM is busy\n");
		}

		return ret ? -EBUSY : val;
	}

	/* Write VCM position to registers */
	ret = i2c_master_send(client, tx_data, sizeof(tx_data));
	if (ret < 0) {
		dev_err(&client->dev,
			"I2C write MSB fail ret=%d\n", ret);

		return ret;
	}

	return 0;
}

static int dw9807_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dw9807_device *dev_vcm = container_of(ctrl->handler,
		struct dw9807_device, ctrls_vcm);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);

		dev_vcm->current_val = ctrl->val;
		return dw9807_set_dac(client, ctrl->val);
	}

	return -EINVAL;
}

static int dw9807_get_volatile_ctrl(struct v4l2_ctrl *ctrl) {
	struct dw9807_device *dev_vcm = container_of(ctrl->handler,
		struct dw9807_device, ctrls_vcm);

	if(ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);
		char tx_data[1] = { DW9807_MSB_ADDR }, rx_data[2] = {0};
		int err = 0;

		err = i2c_master_send(client, tx_data, sizeof(tx_data));
		if(err < 0) {
			dev_err(&client->dev, "%s: i2c_master_send() failed (err=%d)\n", __func__, err);
			return err;
		}

		err = i2c_master_recv(client, rx_data, sizeof(rx_data));
		if(err < 0) {
			dev_err(&client->dev, "%s: i2c_master_recv() failed (err=%d)\n", __func__, err);
			return err;
		}

		ctrl->val = (((rx_data[0] & 0x3) << 8) | rx_data[1]);
		dev_vcm->current_val = ctrl->val;
	}

	return 0;
}

static int dw9807_try_ctrl(struct v4l2_ctrl *ctrl) {
	return dw9807_set_ctrl(ctrl);
}

static const struct v4l2_ctrl_ops dw9807_vcm_ctrl_ops = {
	.s_ctrl = dw9807_set_ctrl,
	.g_volatile_ctrl = dw9807_get_volatile_ctrl,
	.try_ctrl = dw9807_try_ctrl,
};

static int dw9807_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9807_device *dw9807_dev = container_of(sd, struct dw9807_device, sd);
	int err = 0;

	err = pm_runtime_resume_and_get(sd->dev);
	if(err < 0)
		dev_err(sd->v4l2_dev->dev, "%s: pm_runtime_resume_and_get() failed (%d)\n", __func__, err);

	if(dw9807_dev && dw9807_dev->standalone) {
		// register settings to power on the device.
		const char tx_data[2] = { DW9807_CTL_ADDR, 0x00 };

		err = i2c_master_send(dw9807_dev->client, tx_data, sizeof(tx_data));
		if(err < 0) {
			dev_err(&dw9807_dev->client->dev, "%s: cannot power on device over i2c bus (%d)\n", __func__, err);
			return -ENODEV;
		}
	}

	return 0;
}

static int dw9807_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops dw9807_int_ops = {
	.open = dw9807_open,
	.close = dw9807_close,
};

static const struct v4l2_subdev_ops dw9807_ops = { };

static void dw9807_subdev_cleanup(struct dw9807_device *dw9807_dev)
{
	v4l2_async_unregister_subdev(&dw9807_dev->sd);
	v4l2_ctrl_handler_free(&dw9807_dev->ctrls_vcm);
	media_entity_cleanup(&dw9807_dev->sd.entity);
}

static int dw9807_init_controls(struct dw9807_device *dev_vcm)
{
	struct v4l2_ctrl_handler *hdl = &dev_vcm->ctrls_vcm;
	const struct v4l2_ctrl_ops *ops = &dw9807_vcm_ctrl_ops;
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);
	struct v4l2_ctrl *ctrl_focus_absolute = NULL;

	v4l2_ctrl_handler_init(hdl, 1);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9807_MAX_FOCUS_POS, DW9807_FOCUS_STEPS, 0);

	ctrl_focus_absolute = v4l2_ctrl_find(hdl, V4L2_CID_FOCUS_ABSOLUTE);
	if(ctrl_focus_absolute)
		ctrl_focus_absolute->flags |= V4L2_CTRL_FLAG_VOLATILE;

	dev_vcm->sd.ctrl_handler = hdl;
	if (hdl->error) {
		dev_err(&client->dev, "%s fail error: 0x%x\n",
			__func__, hdl->error);
		return hdl->error;
	}

	return 0;
}

static int dw9807_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9807_device *dw9807_dev;
	int rval;

	dw9807_dev = devm_kzalloc(&client->dev, sizeof(*dw9807_dev),
				  GFP_KERNEL);
	if (dw9807_dev == NULL)
		return -ENOMEM;

	dw9807_dev->client = client;

	v4l2_i2c_subdev_init(&dw9807_dev->sd, client, &dw9807_ops);
	dw9807_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9807_dev->sd.internal_ops = &dw9807_int_ops;

	rval = dw9807_init_controls(dw9807_dev);
	if (rval)
		goto err_cleanup;

	rval = media_entity_pads_init(&dw9807_dev->sd.entity, 0, NULL);
	if (rval < 0)
		goto err_cleanup;

	dw9807_dev->sd.entity.function = MEDIA_ENT_F_LENS;

	dw9807_dev->standalone = of_property_read_bool(dev->of_node, "standalone");
	dev_warn(dev, "%s: is standalone? %s\n", __func__, dw9807_dev->standalone? "Yes" : "No");

	if(dw9807_dev->standalone) {
		dev_warn(dev, "%s: registering the standalone subdev\n", __func__);
		rval = v4l2_device_register(dev, &dw9807_dev->v4l2_dev);
		if(rval < 0) {
			dev_err(dev, "%s: cannot register the standalone v4l2_device\n", __func__);
			goto err_cleanup;
		}
		rval = v4l2_device_register_subdev(&dw9807_dev->v4l2_dev, &dw9807_dev->sd);
		if(rval < 0) {
			dev_err(dev, "%s: cannot register the standalone subdev\n", __func__);
			goto err_cleanup;
		}
		rval = v4l2_device_register_subdev_nodes(&dw9807_dev->v4l2_dev);
		if(rval < 0) {
			dev_err(dev, "%s: cannot register the standalone subdev node\n", __func__);
			goto err_cleanup;
		}
		dev_warn(dev, "%s: registered the standalone subdev\n", __func__);
	} else {
		rval = v4l2_async_register_subdev(&dw9807_dev->sd);
		if (rval < 0)
			goto err_cleanup;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

err_cleanup:
	if(dw9807_dev->standalone) {
		v4l2_device_unregister_subdev(&dw9807_dev->sd);
		v4l2_device_unregister(&dw9807_dev->v4l2_dev);
	}
	v4l2_ctrl_handler_free(&dw9807_dev->ctrls_vcm);
	media_entity_cleanup(&dw9807_dev->sd.entity);

	return rval;
}

static int dw9807_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9807_device *dw9807_dev = sd_to_dw9807_vcm(sd);

	pm_runtime_disable(&client->dev);

	dw9807_subdev_cleanup(dw9807_dev);

	if(dw9807_dev->standalone) {
		v4l2_device_unregister(&dw9807_dev->v4l2_dev);
	}

	return 0;
}

/*
 * This function sets the vcm position, so it consumes least current
 * The lens position is gradually moved in units of DW9807_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int __maybe_unused dw9807_vcm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9807_device *dw9807_dev = sd_to_dw9807_vcm(sd);
	const char tx_data[2] = { DW9807_CTL_ADDR, 0x01 };
	int ret, val;

	if(!dw9807_dev->standalone) {
		for (val = dw9807_dev->current_val & ~(DW9807_CTRL_STEPS - 1);
		     val >= 0; val -= DW9807_CTRL_STEPS) {
			ret = dw9807_set_dac(client, val);
			if (ret)
				dev_err_once(dev, "%s I2C failure: %d", __func__, ret);
			usleep_range(DW9807_CTRL_DELAY_US, DW9807_CTRL_DELAY_US + 10);
		}

		/* Power down */
		ret = i2c_master_send(client, tx_data, sizeof(tx_data));
		if (ret < 0) {
			dev_err(&client->dev, "I2C write CTL fail ret = %d\n", ret);
			//return ret;
		}
	}

	return 0;
}

/*
 * This function sets the vcm position to the value set by the user
 * through v4l2_ctrl_ops s_ctrl handler
 * The lens position is gradually moved in units of DW9807_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int  __maybe_unused dw9807_vcm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9807_device *dw9807_dev = sd_to_dw9807_vcm(sd);
	const char tx_data[2] = { DW9807_CTL_ADDR, 0x00 };
	int ret, val;

	if(!dw9807_dev->standalone) {
		/* Power on */
		ret = i2c_master_send(client, tx_data, sizeof(tx_data));
		if (ret < 0) {
			dev_err(&client->dev, "I2C write CTL fail ret = %d\n", ret);
			//return ret;
		}

		for (val = dw9807_dev->current_val % DW9807_CTRL_STEPS;
		     val < dw9807_dev->current_val + DW9807_CTRL_STEPS - 1;
		     val += DW9807_CTRL_STEPS) {
			ret = dw9807_set_dac(client, val);
			if (ret)
				dev_err_ratelimited(dev, "%s I2C failure: %d", __func__, ret);
			usleep_range(DW9807_CTRL_DELAY_US, DW9807_CTRL_DELAY_US + 10);
		}
	}

	return 0;
}

static const struct of_device_id dw9807_of_table[] = {
	{ .compatible = "dongwoon,dw9807-vcm" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw9807_of_table);

static const struct dev_pm_ops dw9807_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dw9807_vcm_suspend, dw9807_vcm_resume)
	SET_RUNTIME_PM_OPS(dw9807_vcm_suspend, dw9807_vcm_resume, NULL)
};

static struct i2c_driver dw9807_i2c_driver = {
	.driver = {
		.name = "dw9807",
		.pm = &dw9807_pm_ops,
		.of_match_table = dw9807_of_table,
	},
	.probe_new = dw9807_probe,
	.remove = dw9807_remove,
};

module_i2c_driver(dw9807_i2c_driver);

MODULE_AUTHOR("Chiang, Alan");
MODULE_DESCRIPTION("DW9807 VCM driver");
MODULE_LICENSE("GPL v2");
