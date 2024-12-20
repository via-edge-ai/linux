/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __MTK_MMSYS_H
#define __MTK_MMSYS_H

#include <linux/mailbox_controller.h>
#include <linux/mailbox/mtk-cmdq-mailbox.h>
#include <linux/soc/mediatek/mtk-cmdq.h>

enum mtk_ddp_comp_id;
struct device;

enum mtk_ddp_comp_id {
	DDP_COMPONENT_AAL0,
	DDP_COMPONENT_AAL1,
	DDP_COMPONENT_BLS,
	DDP_COMPONENT_CCORR0,
	DDP_COMPONENT_CCORR1,
	DDP_COMPONENT_COLOR0,
	DDP_COMPONENT_COLOR1,
	DDP_COMPONENT_DITHER0,
	DDP_COMPONENT_DITHER1,
	DDP_COMPONENT_DP_INTF0,
	DDP_COMPONENT_DP_INTF1,
	DDP_COMPONENT_DPI0,
	DDP_COMPONENT_DPI1,
	DDP_COMPONENT_DSC0,
	DDP_COMPONENT_DSC1,
	DDP_COMPONENT_DSI0,
	DDP_COMPONENT_DSI1,
	DDP_COMPONENT_DSI2,
	DDP_COMPONENT_DSI3,
	DDP_COMPONENT_ETHDR_MIXER,
	DDP_COMPONENT_GAMMA0,
	DDP_COMPONENT_GAMMA1,
	DDP_COMPONENT_MDP_RDMA0,
	DDP_COMPONENT_MDP_RDMA1,
	DDP_COMPONENT_MDP_RDMA2,
	DDP_COMPONENT_MDP_RDMA3,
	DDP_COMPONENT_MDP_RDMA4,
	DDP_COMPONENT_MDP_RDMA5,
	DDP_COMPONENT_MDP_RDMA6,
	DDP_COMPONENT_MDP_RDMA7,
	DDP_COMPONENT_DISP_PADDING0,
	DDP_COMPONENT_DISP_PADDING1,
	DDP_COMPONENT_DISP_PADDING2,
	DDP_COMPONENT_DISP_PADDING3,
	DDP_COMPONENT_DISP_PADDING4,
	DDP_COMPONENT_DISP_PADDING5,
	DDP_COMPONENT_DISP_PADDING6,
	DDP_COMPONENT_DISP_PADDING7,
	DDP_COMPONENT_MERGE0,
	DDP_COMPONENT_MERGE1,
	DDP_COMPONENT_MERGE2,
	DDP_COMPONENT_MERGE3,
	DDP_COMPONENT_MERGE4,
	DDP_COMPONENT_MERGE5,
	DDP_COMPONENT_OD0,
	DDP_COMPONENT_OD1,
	DDP_COMPONENT_OVL0,
	DDP_COMPONENT_OVL_2L0,
	DDP_COMPONENT_OVL_2L1,
	DDP_COMPONENT_OVL_2L2,
	DDP_COMPONENT_OVL_ADAPTOR,
	DDP_COMPONENT_OVL1,
	DDP_COMPONENT_POSTMASK0,
	DDP_COMPONENT_PWM0,
	DDP_COMPONENT_PWM1,
	DDP_COMPONENT_PWM2,
	DDP_COMPONENT_RDMA0,
	DDP_COMPONENT_RDMA1,
	DDP_COMPONENT_RDMA2,
	DDP_COMPONENT_RDMA4,
	DDP_COMPONENT_UFOE,
	DDP_COMPONENT_WDMA0,
	DDP_COMPONENT_WDMA1,
	DDP_COMPONENT_ID_MAX,
};

enum mtk_mmsys_config_type {
	MMSYS_CONFIG_MERGE_ASYNC_WIDTH,
	MMSYS_CONFIG_MERGE_ASYNC_HEIGHT,
	MMSYS_CONFIG_HDR_BE_ASYNC_WIDTH,
	MMSYS_CONFIG_HDR_BE_ASYNC_HEIGHT,
	MMSYS_CONFIG_HDR_ALPHA_SEL,
	MMSYS_CONFIG_MIXER_IN_ALPHA_ODD,
	MMSYS_CONFIG_MIXER_IN_ALPHA_EVEN,
	MMSYS_CONFIG_MIXER_IN_CH_SWAP,
	MMSYS_CONFIG_MIXER_IN_MODE,
	MMSYS_CONFIG_MIXER_IN_BIWIDTH,
};

void mtk_mmsys_ddp_connect(struct device *dev,
			   enum mtk_ddp_comp_id cur,
			   enum mtk_ddp_comp_id next);

void mtk_mmsys_ddp_disconnect(struct device *dev,
			      enum mtk_ddp_comp_id cur,
			      enum mtk_ddp_comp_id next);

void mtk_mmsys_ddp_config(struct device *dev, enum mtk_mmsys_config_type config,
			  u32 id, u32 val, struct cmdq_pkt *cmdq_pkt);

void mtk_mmsys_merge_async_config(struct device *dev, int idx, int width,
				  int height, struct cmdq_pkt *cmdq_pkt);

void mtk_mmsys_hdr_confing(struct device *dev, int be_width, int be_height,
			   struct cmdq_pkt *cmdq_pkt);

void mtk_mmsys_mixer_in_config(struct device *dev, int idx, bool alpha_sel, u16 alpha,
			       u8 mode, u32 biwidth, struct cmdq_pkt *cmdq_pkt);

void mtk_mmsys_mixer_in_channel_swap(struct device *dev, int idx, bool channel_swap,
				     struct cmdq_pkt *cmdq_pkt);

void mtk_mmsys_cross_sys_prepare(struct device *dev_sys0, struct device *dev_sys1,
				int width, int height, struct cmdq_pkt *cmdq_pkt);

void mtk_mmsys_cross_sys_unprepare(struct device *dev_main_sys,
					struct device *dev_cross_sys);

void mtk_mmsys_vpp_rsz_merge_config(struct device *dev, u32 id, bool enable);

void mtk_mmsys_vpp_rsz_dcm_config(struct device *dev, bool enable);

void mtk_mmsys_vpp_split_out_config(struct device *dev, struct cmdq_pkt *cmdq_pkt);

void mtk_mmsys_lvds_config(struct device *dev);

void mtk_mmsys_reset_mdp_split_pipe(struct device *dev, struct cmdq_pkt *cmdq_pkt);

#endif /* __MTK_MMSYS_H */
