// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 * Copyright (c) 2020 BayLibre, SAS
 */

#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt8365-clk.h>

static const struct mtk_gate_regs mm0_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x108,
	.sta_ofs = 0x100,
};

static const struct mtk_gate_regs mm1_cg_regs = {
	.set_ofs = 0x114,
	.clr_ofs = 0x118,
	.sta_ofs = 0x110,
};

#define GATE_MM0(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &mm0_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

#define GATE_MM1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &mm1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate mm_clks[] = {
	/* MM0 */
	GATE_MM0(CLK_MM_MM_MDP_RDMA0, "mm_mdp_rdma0", "mm_sel", 0),
	GATE_MM0(CLK_MM_MM_MDP_CCORR0, "mm_mdp_ccorr0", "mm_sel", 1),
	GATE_MM0(CLK_MM_MM_MDP_RSZ0, "mm_mdp_rsz0", "mm_sel", 2),
	GATE_MM0(CLK_MM_MM_MDP_RSZ1, "mm_mdp_rsz1", "mm_sel", 3),
	GATE_MM0(CLK_MM_MM_MDP_TDSHP0, "mm_mdp_tdshp0", "mm_sel", 4),
	GATE_MM0(CLK_MM_MM_MDP_WROT0, "mm_mdp_wrot0", "mm_sel", 5),
	GATE_MM0(CLK_MM_MM_MDP_WDMA0, "mm_mdp_wdma0", "mm_sel", 6),
	GATE_MM0(CLK_MM_MM_DISP_OVL0, "mm_disp_ovl0", "mm_sel", 7),
	GATE_MM0(CLK_MM_MM_DISP_OVL0_2L, "mm_disp_ovl0_2l", "mm_sel", 8),
	GATE_MM0(CLK_MM_MM_DISP_RSZ0, "mm_disp_rsz0", "mm_sel", 9),
	GATE_MM0(CLK_MM_MM_DISP_RDMA0, "mm_disp_rdma0", "mm_sel", 10),
	GATE_MM0(CLK_MM_MM_DISP_WDMA0, "mm_disp_wdma0", "mm_sel", 11),
	GATE_MM0(CLK_MM_MM_DISP_COLOR0, "mm_disp_color0", "mm_sel", 12),
	GATE_MM0(CLK_MM_MM_DISP_CCORR0, "mm_disp_ccorr0", "mm_sel", 13),
	GATE_MM0(CLK_MM_MM_DISP_AAL0, "mm_disp_aal0", "mm_sel", 14),
	GATE_MM0(CLK_MM_MM_DISP_GAMMA0, "mm_disp_gamma0", "mm_sel", 15),
	GATE_MM0(CLK_MM_MM_DISP_DITHER0, "mm_disp_dither0", "mm_sel", 16),
	GATE_MM0(CLK_MM_MM_DSI0, "mm_dsi0", "mm_sel", 17),
	GATE_MM0(CLK_MM_MM_DISP_RDMA1, "mm_disp_rdma1", "mm_sel", 18),
	GATE_MM0(CLK_MM_MM_MDP_RDMA1, "mm_mdp_rdma1", "mm_sel", 19),
	GATE_MM0(CLK_MM_DPI0_DPI0, "mm_dpi0_dpi0", "vpll_dpix", 20),
	GATE_MM0(CLK_MM_MM_FAKE, "mm_fake", "mm_sel", 21),
	GATE_MM0(CLK_MM_MM_SMI_COMMON, "mm_smi_common", "mm_sel", 22),
	GATE_MM0(CLK_MM_MM_SMI_LARB0, "mm_smi_larb0", "mm_sel", 23),
	GATE_MM0(CLK_MM_MM_SMI_COMM0, "mm_smi_comm0", "mm_sel", 24),
	GATE_MM0(CLK_MM_MM_SMI_COMM1, "mm_smi_comm1", "mm_sel", 25),
	GATE_MM0(CLK_MM_MM_CAM_MDP, "mm_cam_mdp", "mm_sel", 26),
	GATE_MM0(CLK_MM_MM_SMI_IMG, "mm_smi_img", "mm_sel", 27),
	GATE_MM0(CLK_MM_MM_SMI_CAM, "mm_smi_cam", "mm_sel", 28),
	GATE_MM0(CLK_MM_IMG_IMG_DL_RELAY, "mm_dl_relay", "mm_sel", 29),
	GATE_MM0(CLK_MM_IMG_IMG_DL_ASYNC_TOP, "mm_dl_async_top", "mm_sel", 30),
	GATE_MM0(CLK_MM_DSI0_DIG_DSI, "mm_dsi0_dig_dsi", "dsi0_lntc_dsick", 31),
	/* MM1 */
	GATE_MM1(CLK_MM_26M_HRTWT, "mm_f26m_hrtwt", "clk26m_ck", 0),
	GATE_MM1(CLK_MM_MM_DPI0, "mm_dpi0", "mm_sel", 1),
	GATE_MM1(CLK_MM_LVDSTX_PXL, "mm_flvdstx_pxl", "vpll_dpix", 2),
	GATE_MM1(CLK_MM_LVDSTX_CTS, "mm_flvdstx_cts", "lvdstx_dig_cts", 3),
};

struct clk_mt8365_mm_driver_data {
	const struct mtk_gate *gates_clk;
	int gates_num;
};

static const struct clk_mt8365_mm_driver_data mt8365_mmsys_driver_data = {
	.gates_clk = mm_clks,
	.gates_num = ARRAY_SIZE(mm_clks),
};

static int clk_mt8365_mm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->parent->of_node;
	const struct clk_mt8365_mm_driver_data *data;
	struct clk_hw_onecell_data *clk_data;
	int ret;

	clk_data = mtk_alloc_clk_data(CLK_MM_NR_CLK);
	if (!clk_data)
		return -ENOMEM;

	data = &mt8365_mmsys_driver_data;

	ret = mtk_clk_register_gates(node, data->gates_clk, data->gates_num,
				     clk_data);
	if (ret)
		return ret;

	ret = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, clk_data);
	if (ret)
		return ret;

	return 0;
}

static struct platform_driver clk_mt8365_mm_drv = {
	.probe = clk_mt8365_mm_probe,
	.driver = {
		.name = "clk-mt8365-mm",
	},
};

builtin_platform_driver(clk_mt8365_mm_drv);
