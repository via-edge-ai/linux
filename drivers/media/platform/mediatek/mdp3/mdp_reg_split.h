/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#ifndef __MDP_REG_SPLIT_H__
#define __MDP_REG_SPLIT_H__

#define VPP_SPLIT_ENABLE	(0x000)
#define VPP_SPLIT_RESET		(0x004)
#define VPP_SPLIT_CFG_0		(0x010)
#define VPP_SPLIT_CFG_1		(0x014)
#define VPP_SPLIT_CFG_2		(0x018)
#define VPP_SPLIT_CFG_3		(0x01c)
#define VPP_SPLIT_CFG_4		(0x020)
#define VPP_SPLIT_CFG_5		(0x024)
#define VPP_SPLIT_CFG_6		(0x028)
#define VPP_SPLIT_CFG_7		(0x02c)
#define VPP_SPLIT_CFG_8		(0x030)
#define VPP_SPLIT_CFG_9		(0x034)
#define VPP_SPLIT_CFG_12	(0x040)
#define VPP_SPLIT_CFG_13	(0x044)
#define VPP_SPLIT_CFG_14	(0x048)
#define VPP_SPLIT_CFG_15	(0x04c)
#define VPP_SPLIT_CFG_16	(0x050)
#define VPP_SPLIT_CFG_17	(0x054)
#define VPP_SPLIT_CFG_20	(0x060)
#define VPP_SPLIT_CFG_21	(0x064)
#define VPP_SPLIT_CFG_22	(0x068)
#define VPP_SPLIT_CFG_23	(0x06c)
#define VPP_SPLIT_CFG_24	(0x070)
#define VPP_SPLIT_CFG_25	(0x074)
#define VPP_SPLIT_CFG_28	(0x080)
#define VPP_SPLIT_CFG_29	(0x084)
#define VPP_SPLIT_CFG_30	(0x088)
#define VPP_SPLIT_CFG_31	(0x08c)
#define VPP_SPLIT_CFG_32	(0x090)
#define VPP_SPLIT_CFG_33	(0x094)
#define VPP_SPLIT_CFG_36	(0x0a0)
#define VPP_SPLIT_CFG_37	(0x0a4)
#define VPP_SPLIT_CFG_38	(0x0a8)
#define VPP_SPLIT_CFG_40	(0x0b0)
#define VPP_SPLIT_CFG_41	(0x0b4)
#define VPP_SPLIT_CFG_42	(0x0b8)
#define VPP_SPLIT_CFG_43	(0x0bc)
#define VPP_SPLIT_CFG_48	(0x0d0)
#define VPP_SPLIT_CFG_49	(0x0d4)
#define VPP_SPLIT_CFG_50	(0x0d8)
#define VPP_SPLIT_CFG_51	(0x0dc)
#define VPP_SPLIT_CFG_52	(0x0e0)
#define VPP_SPLIT_CFG_53	(0x0e4)
#define VPP_SPLIT_CFG_54	(0x0e8)
#define VPP_SPLIT_CFG_55	(0x0ec)
#define VPP_SPLIT_MON_0		(0x100)
#define VPP_SPLIT_MON_1		(0x104)
#define VPP_SPLIT_MON_2		(0x108)
#define VPP_SPLIT_MON_4		(0x110)
#define VPP_SPLIT_MON_5		(0x114)
#define VPP_SPLIT_MON_6		(0x118)
#define VPP_SPLIT_MON_8		(0x120)
#define VPP_SPLIT_MON_9		(0x124)
#define VPP_SPLIT_MON_10	(0x128)
#define VPP_SPLIT_MON_12	(0x130)
#define VPP_SPLIT_MON_13	(0x134)
#define VPP_SPLIT_MON_14	(0x138)
#define VPP_SPLIT_MON_16	(0x140)
#define VPP_SPLIT_MON_17	(0x144)
#define VPP_SPLIT_MON_18	(0x148)
#define VPP_SPLIT_MON_19	(0x14c)
#define VPP_SPLIT_CFG2_0	(0x180)
#define VPP_SPLIT_CFG2_1	(0x184)
#define VPP_SPLIT_CFG2_2	(0x188)
#define VPP_SPLIT_CFG2_4	(0x190)
#define VPP_SPLIT_CFG2_5	(0x194)
#define VPP_SPLIT_CFG2_6	(0x198)
#define VPP_SPLIT_CFG2_7	(0x19c)
#define VPP_SPLIT_CFG2_8	(0x1A0)
#define VPP_SPLIT_CFG2_9	(0x1A4)
#define VPP_SPLIT_CFG2_10	(0x1A8)
#define VPP_SPLIT_CFG2_11	(0x1Ac)
#define VPP_SPLIT_CFG2_12	(0x1B0)
#define VPP_SPLIT_CFG2_13	(0x1B4)
#define VPP_SPLIT_CFG2_16	(0x1C0)
#define VPP_SPLIT_CFG2_17	(0x1C4)
#define VPP_SPLIT_CFG2_18	(0x1C8)
#define VPP_SPLIT_CFG2_20	(0x1D0)
#define VPP_SPLIT_CFG2_21	(0x1D4)
#define VPP_SPLIT_CFG2_22	(0x1D8)
#define VPP_SPLIT_CFG2_23	(0x1Dc)
#define VPP_SPLIT_CFG2_24	(0x1E0)
#define VPP_SPLIT_CFG2_25	(0x1E4)
#define VPP_SPLIT_CFG2_26	(0x1E8)
#define VPP_SPLIT_CFG2_27	(0x1Ec)
#define VPP_SPLIT_CFG2_28	(0x1F0)
#define VPP_SPLIT_CFG2_29	(0x1F4)
#define VPP_SPLIT_PATG_0	(0x200)
#define VPP_SPLIT_PATG_1	(0x204)
#define VPP_SPLIT_PATG_2	(0x208)
#define VPP_SPLIT_PATG_4	(0x210)
#define VPP_SPLIT_PATG_5	(0x214)
#define VPP_SPLIT_PATG_6	(0x218)
#define VPP_SPLIT_PATG_7	(0x21c)
#define VPP_SPLIT_PATG_8	(0x220)
#define VPP_SPLIT_PATG_9	(0x224)
#define VPP_SPLIT_PATG_10	(0x228)
#define VPP_SPLIT_PATG_11	(0x22c)
#define VPP_SPLIT_PATG_12	(0x230)
#define VPP_SPLIT_PATG_13	(0x234)
#define VPP_SPLIT_PATG_14	(0x238)
#define VPP_SPLIT_PATG_16	(0x240)
#define VPP_SPLIT_PATG_17	(0x244)
#define VPP_SPLIT_PATG_18	(0x248)
#define VPP_SPLIT_PATG_20	(0x250)
#define VPP_SPLIT_PATG_21	(0x254)
#define VPP_SPLIT_PATG_22	(0x258)
#define VPP_SPLIT_PATG_24	(0x260)
#define VPP_SPLIT_PATG_25	(0x264)
#define VPP_SPLIT_PATG_26	(0x268)
#define VPP_SPLIT_PATG_27	(0x26c)
#define VPP_SPLIT_PATG_32	(0x280)
#define VPP_SPLIT_PATG_33	(0x284)
#define VPP_SPLIT_PATG_34	(0x288)
#define VPP_SPLIT_PATG_35	(0x28c)
#define VPP_SPLIT_PATG_36	(0x290)
#define VPP_SPLIT_PATG_37	(0x294)
#define VPP_SPLIT_PATG_38	(0x298)
#define VPP_SPLIT_PATG_39	(0x29c)
#define VPP_SPLIT_PATG_40	(0x2a0)
#define VPP_SPLIT_PATG_41	(0x2a4)
#define VPP_SPLIT_PATG_42	(0x2a8)
#define VPP_SPLIT_PATG_44	(0x2b0)
#define VPP_SPLIT_PATG_45	(0x2b4)
#define VPP_SPLIT_PATG_46	(0x2b8)
#define VPP_SPLIT_PATG_48	(0x2c0)
#define VPP_SPLIT_PATG_49	(0x2c4)
#define VPP_SPLIT_PATG_50	(0x2c8)
#define VPP_SPLIT_PATG_52	(0x2d0)
#define VPP_SPLIT_PATG_53	(0x2d4)
#define VPP_SPLIT_PATG_54	(0x2d8)
#define VPP_SPLIT_PATG_55	(0x2dc)

// MASK
#define VPP_SPLIT_ENABLE_MASK	(0x00000001)
#define VPP_SPLIT_RESET_MASK	(0x00000001)
#define VPP_SPLIT_CFG_0_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_1_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_2_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_3_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_4_MASK	(0x00010111)
#define VPP_SPLIT_CFG_5_MASK	(0x00000111)
#define VPP_SPLIT_CFG_6_MASK	(0x00000001)
#define VPP_SPLIT_CFG_7_MASK	(0x00000001)
#define VPP_SPLIT_CFG_8_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_9_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_12_MASK	(0x0000001f)
#define VPP_SPLIT_CFG_13_MASK	(0x0000001f)
#define VPP_SPLIT_CFG_14_MASK	(0x0000001f)
#define VPP_SPLIT_CFG_15_MASK	(0x0000001f)
#define VPP_SPLIT_CFG_16_MASK	(0x33333333)
#define VPP_SPLIT_CFG_17_MASK	(0x00011111)
#define VPP_SPLIT_CFG_20_MASK	(0x0000000f)
#define VPP_SPLIT_CFG_21_MASK	(0x33310011)
#define VPP_SPLIT_CFG_22_MASK	(0x0fff1333)
#define VPP_SPLIT_CFG_23_MASK	(0x0fff0fff)
#define VPP_SPLIT_CFG_24_MASK	(0x733313f1)
#define VPP_SPLIT_CFG_25_MASK	(0x10ffffff)
#define VPP_SPLIT_CFG_28_MASK	(0x0000000f)
#define VPP_SPLIT_CFG_29_MASK	(0x33310011)
#define VPP_SPLIT_CFG_30_MASK	(0x0fff1333)
#define VPP_SPLIT_CFG_31_MASK	(0x0fff0fff)
#define VPP_SPLIT_CFG_32_MASK	(0x733313f1)
#define VPP_SPLIT_CFG_33_MASK	(0x10ffffff)
#define VPP_SPLIT_CFG_36_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_37_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_38_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_40_MASK	(0x00000011)
#define VPP_SPLIT_CFG_41_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_42_MASK	(0x00000011)
#define VPP_SPLIT_CFG_43_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_48_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_49_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_50_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_51_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_52_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_53_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_54_MASK	(0xffffffff)
#define VPP_SPLIT_CFG_55_MASK	(0xffffffff)
#define VPP_SPLIT_MON_0_MASK	(0x00011111)
#define VPP_SPLIT_MON_1_MASK	(0xffffffff)
#define VPP_SPLIT_MON_2_MASK	(0xffffffff)
#define VPP_SPLIT_MON_4_MASK	(0x00011111)
#define VPP_SPLIT_MON_5_MASK	(0xffffffff)
#define VPP_SPLIT_MON_6_MASK	(0xffffffff)
#define VPP_SPLIT_MON_8_MASK	(0x00110111)
#define VPP_SPLIT_MON_9_MASK	(0xffffffff)
#define VPP_SPLIT_MON_10_MASK	(0xffffffff)
#define VPP_SPLIT_MON_12_MASK	(0x00110111)
#define VPP_SPLIT_MON_13_MASK	(0xffffffff)
#define VPP_SPLIT_MON_14_MASK	(0xffffffff)
#define VPP_SPLIT_MON_16_MASK	(0x0000ffff)
#define VPP_SPLIT_MON_17_MASK	(0x0000ffff)
#define VPP_SPLIT_MON_18_MASK	(0x0000ffff)
#define VPP_SPLIT_MON_19_MASK	(0x0000ffff)
#define VPP_SPLIT_CFG2_0_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_1_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_2_MASK	(0x00000001)
#define VPP_SPLIT_CFG2_4_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_5_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_6_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_7_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_8_MASK	(0x00000001)
#define VPP_SPLIT_CFG2_9_MASK	(0x0000000f)
#define VPP_SPLIT_CFG2_10_MASK	(0x0000ffff)
#define VPP_SPLIT_CFG2_11_MASK	(0x00000001)
#define VPP_SPLIT_CFG2_12_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_13_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_16_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_17_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_18_MASK	(0x00000001)
#define VPP_SPLIT_CFG2_20_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_21_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_22_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_23_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_24_MASK	(0x00000001)
#define VPP_SPLIT_CFG2_25_MASK	(0x0000000f)
#define VPP_SPLIT_CFG2_26_MASK	(0x0000000f)
#define VPP_SPLIT_CFG2_27_MASK	(0x00000001)
#define VPP_SPLIT_CFG2_28_MASK	(0xffffffff)
#define VPP_SPLIT_CFG2_29_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_0_MASK	(0x00000001)
#define VPP_SPLIT_PATG_1_MASK	(0x00000001)
#define VPP_SPLIT_PATG_2_MASK	(0x00000001)
#define VPP_SPLIT_PATG_4_MASK	(0x0001ffff)
#define VPP_SPLIT_PATG_5_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_6_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_7_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_8_MASK	(0x0011ffff)
#define VPP_SPLIT_PATG_9_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_10_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_11_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_12_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_13_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_14_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_16_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_17_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_18_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_20_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_21_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_22_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_24_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_25_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_26_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_27_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_32_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_33_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_34_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_35_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_36_MASK	(0x0011ffff)
#define VPP_SPLIT_PATG_37_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_38_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_39_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_40_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_41_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_42_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_44_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_45_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_46_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_48_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_49_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_50_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_52_MASK	(0xffffffff)
#define VPP_SPLIT_PATG_53_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_54_MASK	(0x0000ffff)
#define VPP_SPLIT_PATG_55_MASK	(0x0000ffff)

#endif  // __MDP_REG_SPLIT_H__
