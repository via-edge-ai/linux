// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Jitao Shi <jitao.shi@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int bpc;

	/**
	 * @width_mm: width of the panel's active display area
	 * @height_mm: height of the panel's active display area
	 */
	struct {
		unsigned int width_mm;
		unsigned int height_mm;
	} size;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	const struct panel_init_cmd *init_cmds;
	unsigned int lanes;
	bool discharge_on_disable;
};

struct nw_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	const struct panel_desc *desc;

	enum drm_panel_orientation orientation;
	struct regulator *pp1800;
	struct regulator *avdd;

	bool prepared;
};

enum dsi_cmd_type {
	INIT_DCS_CMD,
	DELAY_CMD,
};

struct panel_init_cmd {
	enum dsi_cmd_type type;
	size_t len;
	const char *data;
};

#define _INIT_DCS_CMD(...) { \
	.type = INIT_DCS_CMD, \
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} }

#define _INIT_DELAY_CMD(...) { \
	.type = DELAY_CMD,\
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} }

static const struct panel_init_cmd nw_init_cmd[] = {
	_INIT_DELAY_CMD(20),
	_INIT_DCS_CMD(0x11),
	_INIT_DELAY_CMD(20),
	_INIT_DCS_CMD(0x29),
	_INIT_DELAY_CMD(20),	
	{},
};

static inline struct nw_panel *to_nw_panel(struct drm_panel *panel)
{
	return container_of(panel, struct nw_panel, base);
}

static int nw_panel_init_dcs_cmd(struct nw_panel *nw)
{
	struct mipi_dsi_device *dsi = nw->dsi;
	struct drm_panel *panel = &nw->base;
	int i, err = 0;

	if (nw->desc->init_cmds) {
		const struct panel_init_cmd *init_cmds = nw->desc->init_cmds;

		for (i = 0; init_cmds[i].len != 0; i++) {
			const struct panel_init_cmd *cmd = &init_cmds[i];

			switch (cmd->type) {
			case DELAY_CMD:
				msleep(cmd->data[0]);
				err = 0;
				break;

			case INIT_DCS_CMD:
				err = mipi_dsi_dcs_write(dsi, cmd->data[0],
							 cmd->len <= 1 ? NULL :
							 &cmd->data[1],
							 cmd->len - 1);
				break;

			default:
				err = -EINVAL;
			}

			if (err < 0) {
				dev_err(panel->dev,
					"failed to write command %u\n", i);
				return err;
			}
		}
	}
	return 0;
}

static int nw_panel_enter_sleep_mode(struct nw_panel *nw)
{
	struct mipi_dsi_device *dsi = nw->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	return 0;
}

static int nw_panel_unprepare(struct drm_panel *panel)
{
	struct nw_panel *nw = to_nw_panel(panel);
	int ret;

	if (!nw->prepared)
		return 0;

	ret = nw_panel_enter_sleep_mode(nw);
	if (ret < 0) {
		dev_err(panel->dev, "failed to set panel off: %d\n", ret);
		return ret;
	}

	msleep(150);

	if (nw->desc->discharge_on_disable) {
		regulator_disable(nw->avdd);
		usleep_range(5000, 7000);
		usleep_range(5000, 7000);
		if (nw->pp1800)
			regulator_disable(nw->pp1800);
	} else {
		usleep_range(500, 1000);
		regulator_disable(nw->avdd);
		usleep_range(5000, 7000);
		if (nw->pp1800)
			regulator_disable(nw->pp1800);
	}

	nw->prepared = false;

	return 0;
}

static int nw_panel_prepare(struct drm_panel *panel)
{
	struct nw_panel *nw = to_nw_panel(panel);
	int ret;

	if (nw->prepared)
		return 0;

	if (nw->pp1800)
		ret = regulator_enable(nw->pp1800);
	if (ret < 0)
		return ret;

	usleep_range(3000, 5000);

	ret = regulator_enable(nw->avdd);
	if (ret < 0)
		goto poweroff1v8;

	usleep_range(5000, 10000);

	ret = nw_panel_init_dcs_cmd(nw);
	if (ret < 0) {
		dev_err(panel->dev, "failed to init panel: %d\n", ret);
		goto poweroffavdd;
	}

	nw->prepared = true;

	return 0;

poweroffavdd:
	regulator_disable(nw->avdd);
poweroff1v8:
	usleep_range(5000, 7000);
	if (nw->pp1800)
		regulator_disable(nw->pp1800);

	return ret;
}

static int nw_panel_enable(struct drm_panel *panel)
{
	msleep(130);
	return 0;
}

static const struct drm_display_mode nw_nt070r11_default_mode = {
	.clock = 153300,
	.vdisplay = 1920,
	.vsync_start = 1920 + 10,
	.vsync_end = 1920 + 10 + 2,
	.vtotal = 1920 + 10 + 2 + 14,
	.hdisplay = 1200,
	.hsync_start = 1200 + 80,
	.hsync_end = 1200 + 80 + 24,
	.htotal = 1200 + 80 + 24 + 60,
};

static const struct panel_desc nw_nt070r11_desc = {
	.modes = &nw_nt070r11_default_mode,
	.bpc = 8,
	.size = {
		.width_mm = 216,
		.height_mm = 135,
	},
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		      MIPI_DSI_MODE_LPM,
	.init_cmds = nw_init_cmd,
	.discharge_on_disable = false,
};

static int nw_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct nw_panel *nw = to_nw_panel(panel);
	const struct drm_display_mode *m = nw->desc->modes;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, m);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
		return -ENOMEM;
	}

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = nw->desc->size.width_mm;
	connector->display_info.height_mm = nw->desc->size.height_mm;
	connector->display_info.bpc = nw->desc->bpc;
	drm_connector_set_panel_orientation(connector, nw->orientation);

	return 1;
}

static const struct drm_panel_funcs nw_panel_funcs = {
	.unprepare = nw_panel_unprepare,
	.prepare = nw_panel_prepare,
	.enable = nw_panel_enable,
	.get_modes = nw_panel_get_modes,
};

static int nw_panel_add(struct nw_panel *nw)
{
	struct device *dev = &nw->dsi->dev;
	int err;

	nw->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(nw->avdd))
		return PTR_ERR(nw->avdd);

	nw->pp1800 = devm_regulator_get(dev, "pp1800");
	if (IS_ERR(nw->pp1800))
		return PTR_ERR(nw->pp1800);

	drm_panel_init(&nw->base, dev, &nw_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	err = of_drm_get_panel_orientation(dev->of_node, &nw->orientation);
	if (err < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, err);
		return err;
	}

	err = drm_panel_of_backlight(&nw->base);
	if (err)
		return err;

	nw->base.funcs = &nw_panel_funcs;
	nw->base.dev = &nw->dsi->dev;

	drm_panel_add(&nw->base);

	return 0;
}

static int nw_panel_probe(struct mipi_dsi_device *dsi)
{
	struct nw_panel *nw;
	int ret;
	const struct panel_desc *desc;

	nw = devm_kzalloc(&dsi->dev, sizeof(*nw), GFP_KERNEL);
	if (!nw)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->lanes = desc->lanes;
	dsi->format = desc->format;
	dsi->mode_flags = desc->mode_flags;
	nw->desc = desc;
	nw->dsi = dsi;
	ret = nw_panel_add(nw);
	if (ret < 0)
		return ret;

	mipi_dsi_set_drvdata(dsi, nw);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&nw->base);

	return ret;
}

static void nw_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct nw_panel *nw = mipi_dsi_get_drvdata(dsi);

	drm_panel_disable(&nw->base);
	drm_panel_unprepare(&nw->base);
}

static int nw_panel_remove(struct mipi_dsi_device *dsi)
{
	struct nw_panel *nw = mipi_dsi_get_drvdata(dsi);
	int ret;

	nw_panel_shutdown(dsi);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	if (nw->base.dev)
		drm_panel_remove(&nw->base);

	return 0;
}

static const struct of_device_id nw_of_match[] = {
	{ .compatible = "nw,nt070r11",
	  .data = &nw_nt070r11_desc
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nw_of_match);

static struct mipi_dsi_driver nw_panel_driver = {
	.driver = {
		.name = "panel-nw-nt070r11",
		.of_match_table = nw_of_match,
	},
	.probe = nw_panel_probe,
	.remove = nw_panel_remove,
	.shutdown = nw_panel_shutdown,
};
module_mipi_dsi_driver(nw_panel_driver);

MODULE_AUTHOR("Jitao Shi <jitao.shi@mediatek.com>");
MODULE_DESCRIPTION("NW nt070r11 1200x1920 video mode panel driver");
MODULE_LICENSE("GPL v2");
