/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Chun-Jie Chen <chun-jie.chen@mediatek.com>
 */

#ifndef __SOC_MEDIATEK_MT8195_PM_DOMAINS_H
#define __SOC_MEDIATEK_MT8195_PM_DOMAINS_H

#include "mtk-pm-domains.h"
#include <dt-bindings/power/mt8195-power.h>

/*
 * MT8195 power domain support
 */

static const struct scpsys_domain_data scpsys_domain_data_mt8195[] = {
	[MT8195_POWER_DOMAIN_PCIE_MAC_P0] = {
		.name = "pcie_mac_p0",
		.sta_mask = BIT(11),
		.ctl_offs = 0x328,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VDNR_PCIE_MAC_P0,
				    MT8195_TOP_AXI_PROT_EN_VDNR_SET,
				    MT8195_TOP_AXI_PROT_EN_VDNR_CLR,
				    MT8195_TOP_AXI_PROT_EN_VDNR_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VDNR_1_PCIE_MAC_P0,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_SET,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_CLR,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_STA1),
		},
	},
	[MT8195_POWER_DOMAIN_PCIE_MAC_P1] = {
		.name = "pcie_mac_p1",
		.sta_mask = BIT(12),
		.ctl_offs = 0x32C,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VDNR_PCIE_MAC_P1,
				    MT8195_TOP_AXI_PROT_EN_VDNR_SET,
				    MT8195_TOP_AXI_PROT_EN_VDNR_CLR,
				    MT8195_TOP_AXI_PROT_EN_VDNR_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VDNR_1_PCIE_MAC_P1,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_SET,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_CLR,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_STA1),
		},
	},
	[MT8195_POWER_DOMAIN_PCIE_PHY] = {
		.name = "pcie_phy",
		.sta_mask = BIT(13),
		.ctl_offs = 0x330,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8195_POWER_DOMAIN_SSUSB_PCIE_PHY] = {
		.name = "ssusb_pcie_phy",
		.sta_mask = BIT(14),
		.ctl_offs = 0x334,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.caps = MTK_SCPD_ACTIVE_WAKEUP | MTK_SCPD_ALWAYS_ON,
	},
	[MT8195_POWER_DOMAIN_CSI_RX_TOP] = {
		.name = "csi_rx_top",
		.sta_mask = BIT(18),
		.ctl_offs = 0x3C4,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_ETHER] = {
		.name = "ether",
		.sta_mask = BIT(3),
		.ctl_offs = 0x344,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8195_POWER_DOMAIN_ADSP] = {
		.name = "adsp",
		.sta_mask = BIT(10),
		.ctl_offs = 0x360,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_2_ADSP,
				    MT8195_TOP_AXI_PROT_EN_2_SET,
				    MT8195_TOP_AXI_PROT_EN_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_2_STA1),
		},
		.caps = MTK_SCPD_SRAM_ISO | MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8195_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		.sta_mask = BIT(8),
		.ctl_offs = 0x358,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_2_AUDIO,
				    MT8195_TOP_AXI_PROT_EN_2_SET,
				    MT8195_TOP_AXI_PROT_EN_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_2_STA1),
		},
	},
	[MT8195_POWER_DOMAIN_MFG0] = {
		.name = "mfg0",
		.sta_mask = BIT(1),
		.ctl_offs = 0x300,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_DOMAIN_SUPPLY,
	},
	[MT8195_POWER_DOMAIN_MFG1] = {
		.name = "mfg1",
		.sta_mask = BIT(2),
		.ctl_offs = 0x304,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MFG1,
				    MT8195_TOP_AXI_PROT_EN_SET,
				    MT8195_TOP_AXI_PROT_EN_CLR,
				    MT8195_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_2_MFG1,
				    MT8195_TOP_AXI_PROT_EN_2_SET,
				    MT8195_TOP_AXI_PROT_EN_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_2_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_1_MFG1,
				    MT8195_TOP_AXI_PROT_EN_1_SET,
				    MT8195_TOP_AXI_PROT_EN_1_CLR,
				    MT8195_TOP_AXI_PROT_EN_1_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_2_MFG1_2ND,
				    MT8195_TOP_AXI_PROT_EN_2_SET,
				    MT8195_TOP_AXI_PROT_EN_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_2_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MFG1_2ND,
				    MT8195_TOP_AXI_PROT_EN_SET,
				    MT8195_TOP_AXI_PROT_EN_CLR,
				    MT8195_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_MFG1,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_SET,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_CLR,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_MFG2] = {
		.name = "mfg2",
		.sta_mask = BIT(3),
		.ctl_offs = 0x308,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_MFG3] = {
		.name = "mfg3",
		.sta_mask = BIT(4),
		.ctl_offs = 0x30C,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_MFG4] = {
		.name = "mfg4",
		.sta_mask = BIT(5),
		.ctl_offs = 0x310,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_MFG5] = {
		.name = "mfg5",
		.sta_mask = BIT(6),
		.ctl_offs = 0x314,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_MFG6] = {
		.name = "mfg6",
		.sta_mask = BIT(7),
		.ctl_offs = 0x318,
		.pwr_sta_offs = 0x174,
		.pwr_sta2nd_offs = 0x178,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_VPPSYS0] = {
		.name = "vppsys0",
		.sta_mask = BIT(11),
		.ctl_offs = 0x364,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VPPSYS0,
				    MT8195_TOP_AXI_PROT_EN_SET,
				    MT8195_TOP_AXI_PROT_EN_CLR,
				    MT8195_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VPPSYS0,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VPPSYS0_2ND,
				    MT8195_TOP_AXI_PROT_EN_SET,
				    MT8195_TOP_AXI_PROT_EN_CLR,
				    MT8195_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VPPSYS0_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_VPPSYS0,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_SET,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_CLR,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_STA1),
		},
	},
	[MT8195_POWER_DOMAIN_VDOSYS0] = {
		.name = "vdosys0",
		.sta_mask = BIT(13),
		.ctl_offs = 0x36C,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VDOSYS0,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VDOSYS0,
				    MT8195_TOP_AXI_PROT_EN_SET,
				    MT8195_TOP_AXI_PROT_EN_CLR,
				    MT8195_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_VDOSYS0,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_SET,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_CLR,
				    MT8195_TOP_AXI_PROT_EN_SUB_INFRA_VDNR_STA1),
		},
	},
	[MT8195_POWER_DOMAIN_VPPSYS1] = {
		.name = "vppsys1",
		.sta_mask = BIT(12),
		.ctl_offs = 0x368,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VPPSYS1,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VPPSYS1_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VPPSYS1,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
	},
	[MT8195_POWER_DOMAIN_VDOSYS1] = {
		.name = "vdosys1",
		.sta_mask = BIT(14),
		.ctl_offs = 0x370,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VDOSYS1,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VDOSYS1_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VDOSYS1,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
	},
	[MT8195_POWER_DOMAIN_DP_TX] = {
		.name = "dp_tx",
		.sta_mask = BIT(16),
		.ctl_offs = 0x378,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VDNR_1_DP_TX,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_SET,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_CLR,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_EPD_TX] = {
		.name = "epd_tx",
		.sta_mask = BIT(17),
		.ctl_offs = 0x37C,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_VDNR_1_EPD_TX,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_SET,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_CLR,
				    MT8195_TOP_AXI_PROT_EN_VDNR_1_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_HDMI_TX] = {
		.name = "hdmi_tx",
		.sta_mask = BIT(18),
		.ctl_offs = 0x380,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8195_POWER_DOMAIN_HDMI_RX] = {
		.name = "hdmi_rx",
		.sta_mask = BIT(19),
		.ctl_offs = 0x384,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8195_POWER_DOMAIN_WPESYS] = {
		.name = "wpesys",
		.sta_mask = BIT(15),
		.ctl_offs = 0x374,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_WPESYS,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_WPESYS,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_WPESYS_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
	},
	[MT8195_POWER_DOMAIN_VDEC0] = {
		.name = "vdec0",
		.sta_mask = BIT(20),
		.ctl_offs = 0x388,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VDEC0,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VDEC0,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VDEC0_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VDEC0_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_VDEC1] = {
		.name = "vdec1",
		.sta_mask = BIT(21),
		.ctl_offs = 0x38C,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VDEC1,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VDEC1_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_VDEC2] = {
		.name = "vdec2",
		.sta_mask = BIT(22),
		.ctl_offs = 0x390,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VDEC2,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VDEC2_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_VENC] = {
		.name = "venc",
		.sta_mask = BIT(23),
		.ctl_offs = 0x394,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VENC,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VENC_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VENC,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_VENC_CORE1] = {
		.name = "venc_core1",
		.sta_mask = BIT(24),
		.ctl_offs = 0x398,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_VENC_CORE1,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_VENC_CORE1,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_IMG] = {
		.name = "img",
		.sta_mask = BIT(29),
		.ctl_offs = 0x3AC,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_IMG,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_IMG_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_DIP] = {
		.name = "dip",
		.sta_mask = BIT(30),
		.ctl_offs = 0x3B0,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_IPE] = {
		.name = "ipe",
		.sta_mask = BIT(31),
		.ctl_offs = 0x3B4,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_IPE,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_IPE,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_CAM] = {
		.name = "cam",
		.sta_mask = BIT(25),
		.ctl_offs = 0x39C,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_infracfg = {
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_2_CAM,
				    MT8195_TOP_AXI_PROT_EN_2_SET,
				    MT8195_TOP_AXI_PROT_EN_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_2_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_CAM,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_1_CAM,
				    MT8195_TOP_AXI_PROT_EN_1_SET,
				    MT8195_TOP_AXI_PROT_EN_1_CLR,
				    MT8195_TOP_AXI_PROT_EN_1_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_CAM_2ND,
				    MT8195_TOP_AXI_PROT_EN_MM_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR(MT8195_TOP_AXI_PROT_EN_MM_2_CAM,
				    MT8195_TOP_AXI_PROT_EN_MM_2_SET,
				    MT8195_TOP_AXI_PROT_EN_MM_2_CLR,
				    MT8195_TOP_AXI_PROT_EN_MM_2_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_CAM_RAWA] = {
		.name = "cam_rawa",
		.sta_mask = BIT(26),
		.ctl_offs = 0x3A0,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_CAM_RAWB] = {
		.name = "cam_rawb",
		.sta_mask = BIT(27),
		.ctl_offs = 0x3A4,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT8195_POWER_DOMAIN_CAM_MRAW] = {
		.name = "cam_mraw",
		.sta_mask = BIT(28),
		.ctl_offs = 0x3A8,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
};

static const struct scpsys_soc_data mt8195_scpsys_data = {
	.domains_data = scpsys_domain_data_mt8195,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt8195),
};

#endif /* __SOC_MEDIATEK_MT8195_PM_DOMAINS_H */
