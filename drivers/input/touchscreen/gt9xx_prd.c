/* gt9xx_prd. for factory test
*
* 2010 - 2012 Goodix Technology.
*  
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be a reference
* to you, when you are integrating the GOODiX's CTP IC into your system,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* Version: 2.8.0.3
* Author: meta@goodix.com
* Accomplished Date:2019/2/20
* Revision record:
*
*/

#include "gt9xx_prd.h"
#include <linux/time.h>
#include <linux/string.h>
#include <linux/firmware.h>
//#include <linux/device.h>
/*add by 101003082 for ITO test at 2018/06/07 begin*/
int current_data_index;
int test_error_code;
u8 gtp_ito_test_on = 0;

struct gt9xx_short_fw g_short_fw;
int ITO_TEST_COUNT = 0;
#define SHORT_FW_NAME "gt9xx_short.bin"
/*for save testing data*/
char itosavepath[50]={'\0'};
char *ito_save_dir="/sdcard/";

/*for store tmp data*/
u8 *gtp_config;
u16 *max_limit_vale_re;
u16 *min_limit_vale_re;
u16 *accord_limit_vale_re;
u16 *accord_limit_temp;
u16 *jitter_limit_temp;
/*end for tmp data*/
/*add by 101003082 for ITO test at 2018/06/07 end*/
/************** Customer Config Start ***********************/

/****************** Customer Config End ***********************/
static u8 test_cfg_info_group0[] =  CTP_TEST_CFG_GROUP0;
static u8 test_cfg_info_group1[] =  CTP_TEST_CFG_GROUP1;
static u8 test_cfg_info_group2[] =  CTP_TEST_CFG_GROUP2;
static u8 test_cfg_info_group3[] =  CTP_TEST_CFG_GROUP3;
static u8 test_cfg_info_group4[] =  CTP_TEST_CFG_GROUP4;
static u8 test_cfg_info_group5[] =  CTP_TEST_CFG_GROUP5;

static u8 *send_test_cfg_buf[] = {
	test_cfg_info_group0,
	test_cfg_info_group1,
	test_cfg_info_group2,
	test_cfg_info_group3,
	test_cfg_info_group4,
	test_cfg_info_group5,
};

static u8 tset_cfg_info_len[] = {
	CFG_GROUP_LEN(test_cfg_info_group0),
	CFG_GROUP_LEN(test_cfg_info_group1),
	CFG_GROUP_LEN(test_cfg_info_group2),
	CFG_GROUP_LEN(test_cfg_info_group3),
	CFG_GROUP_LEN(test_cfg_info_group4),
	CFG_GROUP_LEN(test_cfg_info_group5)
};

extern u8 cfg_len;
u8 g_sensor_id = 0;

u8  gt9xx_drv_num = MAX_DRIVER_NUM;
u8  gt9xx_sen_num = MAX_SENSOR_NUM;
u16 gt9xx_pixel_cnt = MAX_DRIVER_NUM * MAX_SENSOR_NUM;
u16 gt9xx_sc_pxl_cnt = MAX_DRIVER_NUM * MAX_SENSOR_NUM;
struct gt9xx_short_info *short_sum;

u8 chip_type_gt9f = 0;
u8 have_key = 0;
u8 key_is_isolated;
u8 key_iso_pos[5];

static u8  rslt_buf_idx = 0;
static s32 *test_rslt_buf;
static struct gt9xx_open_info *touchpad_sum;
static struct gt9xx_iot_result_info  *Ito_result_info;

/********** Test Result Archive *********/
static u16 key_isolated_avrg[4] = {0};
static u32 avrg_raw_len = 0;
static u32 *avrg_raw_buf;
/*
Initialize cfg_drv_order and cfg_sen_order, which is used for report short channels
*/
static u8 cfg_drv_order[MAX_DRIVER_NUM];
static u8 cfg_sen_order[MAX_SENSOR_NUM];

#define _MIN_ERROR_NUM		(GTP_OPEN_SAMPLE_NUM * 9 / 10)

static mm_segment_t old_fs;
static loff_t file_pos = 0;
s32 gtp_jitter_test(struct i2c_client * client, u32 check_types);

static void gtp_arch_file_append(char *str, s32 len);
static void gtp_arch_file_append_no_len(char *str)
{
	gtp_arch_file_append(str, strlen(str));
}

static void gtp_arch_file_append(char *str, s32 len)
{
	struct file *arch_filp;
	mm_segment_t old_fs;

	arch_filp = filp_open(TEST_RSLT_ARCHIVE_PATH, O_RDWR | O_CREAT, 0666);

	if (IS_ERR(arch_filp)) {
		GTP_ERROR("Failed to open %s for test result archive!", TEST_RSLT_ARCHIVE_PATH);
		return;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	arch_filp->f_op->llseek(arch_filp, 0, SEEK_END);
	vfs_write(arch_filp, str, len, &arch_filp->f_pos);
	filp_close(arch_filp, NULL);
	set_fs(old_fs);
}

s32 gt9xx_short_parse_cfg(void)
{
	u8 i = 0;

	gtp_config[0] = (u8)(GTP_REG_CONFIG_DATA >> 8);
	gtp_config[1] = (u8)GTP_REG_CONFIG_DATA;
	if (gtp_i2c_read(i2c_connect_client, gtp_config,
			GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH) <= 0) {
		GTP_ERROR("Failed to read config!\n");
		return FAIL;
	}

	gt9xx_drv_num = (gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT-GT9_REG_CFG_BEG] & 0x1F)
				+ (gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+1 -GT9_REG_CFG_BEG] & 0x1F);
	gt9xx_sen_num = (gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+2-GT9_REG_CFG_BEG] & 0x0F)
				+ ((gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+2-GT9_REG_CFG_BEG]>>4) & 0x0F);
	
	if (gt9xx_drv_num > MAX_DRIVER_NUM) {
		GTP_ERROR("driver number error!");
		return FAIL;
	}
	if (gt9xx_sen_num > MAX_SENSOR_NUM) {
		GTP_ERROR("sensor number error!");
		return FAIL;
	}
	memset(cfg_sen_order, 0xFF, MAX_SENSOR_NUM);
	for (i = 0; i < gt9xx_sen_num; ++i) {
		cfg_sen_order[i] = gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_ORD - GT9_REG_CFG_BEG + i];
	}

	memset(cfg_drv_order, 0xFF, MAX_DRIVER_NUM);
	for (i = 0; i < gt9xx_drv_num; ++i) {
		cfg_drv_order[i] = gtp_config[GTP_ADDR_LENGTH + GT9_REG_DRV_ORD - GT9_REG_CFG_BEG + i];
	}

	return SUCCESS;
}

/*
* @param:
*		phy_chnl: ic detected short channel, is_driver: it's driver or not
* @Return:
*		0xff: the ic channel is not used, otherwise: the tp short channel
*/
u8 gt9_get_short_tp_chnl(u8 phy_chnl, u8 is_driver)
{
	u8 i = 0;
	if (is_driver) {
		for (i = 0; i < MAX_DRIVER_NUM; ++i) {
			if (cfg_drv_order[i] == phy_chnl) {
				return i;
			}
			else if (cfg_drv_order[i] == 0xFF) {
				return 0xFF;
			}
		}
	} else {
		for (i = 0; i < MAX_SENSOR_NUM; ++i) {
			if (cfg_sen_order[i] == phy_chnl) {
				return i;
			}
			else if (cfg_sen_order[i] == 0xFF) {
				return 0xFF;
			}
		}
	}
	return 0xFF;
}

static s32 gtp_i2c_end_cmd(struct i2c_client *client)
{
	u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	s32 ret = 0;

	ret = gtp_i2c_write(client, end_cmd, 3);
	if (ret < 0) {
		GTP_INFO("I2C write end_cmd  error!\n");
	}
	return ret;
}

static s32 gtp_open_test_init(struct i2c_client *client)
{
	u8 sensor_id = 0;
	u8 key_pos = 0;
	u8 key_val = 0;
	u8 i = 0;
	u8 tmp[4] = {(u8)(GTP_REG_SENSOR_ID >> 8), (u8)GTP_REG_SENSOR_ID, 0};

	u8 test_config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH] = {
		(u8)(GTP_REG_CONFIG_DATA >> 8), (u8)GTP_REG_CONFIG_DATA, 0};
	u8 type_buf[12] = {0x80, 0x00};

	gt9xx_pixel_cnt = gt9xx_drv_num * gt9xx_sen_num;
	gt9xx_sc_pxl_cnt = gt9xx_pixel_cnt;
	gtp_i2c_read(client, type_buf, 12);
	if (!memcmp(&type_buf[2], "GOODIX_GT9", 10)) {
		chip_type_gt9f = 0;
		GTP_INFO("Chip type: GT9XX");
	} else {
		chip_type_gt9f = 1;
		GTP_INFO("Chip type: GT9XXF");
	}

	have_key = gtp_config[0x804E - GT9_REG_CFG_BEG + GTP_ADDR_LENGTH] & 0x01;

	if (have_key) {
		GTP_INFO("Have Key");
		for (i = 0; i < 5; ++i) {
			key_iso_pos[i] = 0;
		}

		key_is_isolated = 0;
		for (i = 0; i < 4; ++i) {
			key_val = gtp_config[GTP_ADDR_LENGTH + GT9_REG_KEY_VAL - GT9_REG_CFG_BEG + i];
			key_pos = key_val%0x08;
			if ((key_pos != 0)) {
				key_is_isolated = 0;
				break;
			} else if (key_val == 0x00) {
				continue;
			} else {
				key_iso_pos[0]++;		/*isolated key count*/
				key_iso_pos[i+1] = key_val/0x08 - 1;
				key_is_isolated = 1;
			}
		}

		gt9xx_sc_pxl_cnt = gt9xx_pixel_cnt - 1 * gt9xx_sen_num;
	}

	if (gtp_i2c_read(client, tmp, 1 + GTP_ADDR_LENGTH) <= 0) {
		GTP_ERROR("Failed to read sensor id!");
		return FAIL;
	} else {
		sensor_id = tmp[GTP_ADDR_LENGTH];
		if (sensor_id >= 0x06) {
			GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
			return FAIL;
		}
		g_sensor_id = sensor_id;
		GTP_INFO("sensor_id: %d\n", sensor_id);
	}

	if (tset_cfg_info_len[sensor_id]!= 0) {
		memset(&test_config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
		memcpy(&test_config[GTP_ADDR_LENGTH], send_test_cfg_buf[sensor_id],
					tset_cfg_info_len[sensor_id]);

		gtp_i2c_write(client, test_config, tset_cfg_info_len[sensor_id] + GTP_ADDR_LENGTH);
	}

	switch (sensor_id) {
		case 0:
			GTP_ERROR("GTP:Use sensor_id 0 standard");			
			memcpy(&max_limit_vale_re[0], &max_limit_vale_id0[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&min_limit_vale_re[0], &min_limit_vale_id0[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&accord_limit_vale_re[0], &accord_limit_vale_id0[0], sizeof(u16) * gt9xx_pixel_cnt);		
			break;

		case 1:
			GTP_ERROR("GTP:Use sensor_id 1 standard"); 
			memcpy(&max_limit_vale_re[0], &max_limit_vale_id1[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&min_limit_vale_re[0], &min_limit_vale_id1[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&accord_limit_vale_re[0], &accord_limit_vale_id1[0], sizeof(u16) * gt9xx_pixel_cnt);				
			break;
				
		case 2:
			GTP_ERROR("GTP:Use sensor_id 2 standard"); 
			memcpy(&max_limit_vale_re[0], &max_limit_vale_id2[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&min_limit_vale_re[0], &min_limit_vale_id2[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&accord_limit_vale_re[0], &accord_limit_vale_id2[0], sizeof(u16) * gt9xx_pixel_cnt);	
			break;
				
		case 3:
			GTP_ERROR("GTP:Use sensor_id 3 standard"); 
			memcpy(&max_limit_vale_re[0], &max_limit_vale_id3[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&min_limit_vale_re[0], &min_limit_vale_id3[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&accord_limit_vale_re[0], &accord_limit_vale_id3[0], sizeof(u16) * gt9xx_pixel_cnt);				
			break;
		case 4:
			GTP_ERROR("GTP:Use sensor_id 4 standard"); 
			memcpy(&max_limit_vale_re[0], &max_limit_vale_id4[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&min_limit_vale_re[0], &min_limit_vale_id4[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&accord_limit_vale_re[0], &accord_limit_vale_id4[0], sizeof(u16) * gt9xx_pixel_cnt);				
			break;
		case 5:
			GTP_ERROR("GTP:Use sensor_id 5 standard"); 
			memcpy(&max_limit_vale_re[0], &max_limit_vale_id5[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&min_limit_vale_re[0], &min_limit_vale_id5[0], sizeof(u16) * gt9xx_pixel_cnt);
			memcpy(&accord_limit_vale_re[0], &accord_limit_vale_id5[0], sizeof(u16) * gt9xx_pixel_cnt);				
			break;
			
		default:		
			GTP_ERROR("GTP:Unrecognized sensor_id,keep last!");				
			break;
	}
	return SUCCESS;
}

/*
* Function:
*		write one byte to specified register
* Input:
*		reg: the register address
*		val: the value to write into
* Return:
*		i2c_write function return
*/
s32 gtp_write_register(struct i2c_client * client, u16 addr, u8 val)
{
	s32 ret = 0;
	u8 buf[3];
	buf[0] = (addr >> 8) & 0xFF;
	buf[1] = addr & 0xFF;
	buf[2] = val;
	ret = gtp_i2c_write(client, buf, 3);

	if (ret < 0) {
		return ret;
	} else {
		return 1;
	}
}
/*
* Function:
*		read one byte from specified register into buf
* Input:
*		reg: the register
*		buf: the buffer for one byte
* Return:
*		i2c_read function return
*/
s32 gtp_read_register(struct i2c_client * client, u16 reg, u8* buf)
{
	s32 ret = 0;
	buf[0] = (u8)(reg >> 8);
	buf[1] = (u8)reg;
	ret = gtp_i2c_read(client, buf, 3);

	if (ret < 0) {
		return ret;
	} else {
		return 2;
	}
}

/*
* Function:
*      burn dsp_short code
* Input:
*      i2c_client
* Return:
*      SUCCESS: burning succeed, FAIL: burning failed
*/
s32 gtp_burn_dsp_short(struct i2c_client *client)
{
	s32 ret = 0;
	u8 *opr_buf;
	u16 i = 0;
	u16 addr = GTP_REG_DSP_SHORT;
	u16 opr_len = 0;
	u16 left = 0;
	u16 retry = 0;
	u8 read_buf[3] = {0x00};

	opr_buf = (u8*)kmalloc(sizeof(u8) * (DSP_SHORT_BURN_CHK+2), GFP_KERNEL);
	if (!opr_buf) {
		GTP_ERROR("failed to allocate memory for check buffer!");
		return FAIL;
	}

	left = g_short_fw.fw_total_len;
	while (left > 0) {
		opr_buf[0] = (u8)(addr >> 8);
		opr_buf[1] = (u8)(addr);
		if (left > DSP_SHORT_BURN_CHK) {
			opr_len = DSP_SHORT_BURN_CHK;
		} else {
			opr_len = left;
		}
		memcpy(&opr_buf[2], &g_short_fw.fw_data[addr-GTP_REG_DSP_SHORT], opr_len);

		ret = gtp_i2c_write(client, opr_buf, 2 + opr_len);
		if ( ret < 0 ) {
			GTP_ERROR("write dsp_short code failed!");
			kfree(opr_buf);
			return FAIL;
		}
		addr += opr_len;
		left -= opr_len;
	}

	addr = GTP_REG_DSP_SHORT;
	left = g_short_fw.fw_total_len;
	while (left > 0) {
		memset(opr_buf, 0, opr_len + 2);
		opr_buf[0] = (u8)(addr >> 8);
		opr_buf[1] = (u8)(addr);


		if (left > DSP_SHORT_BURN_CHK) {
			opr_len = DSP_SHORT_BURN_CHK;
		} else {
			opr_len = left;
		}

		ret = gtp_i2c_read(client, opr_buf, opr_len+2);
		if (ret < 0) {
			kfree(opr_buf);
			return FAIL;
		}
		for (i = 0; i < opr_len; ++i) {
			if (opr_buf[i+2] != g_short_fw.fw_data[addr-GTP_REG_DSP_SHORT+i]) {
				GTP_ERROR("check dsp_short code failed!");

				gtp_write_register(client, addr + i, g_short_fw.fw_data[addr-GTP_REG_DSP_SHORT+i]);

				msleep(1);
				gtp_read_register(client, addr + i, read_buf);
				opr_buf[i+2] = read_buf[2];
				i--;
				retry++;
				if (retry >= 200) {
					kfree(opr_buf);
					return FAIL;
				}
			}
		}

		addr += opr_len;
		left -= opr_len;
	}
	kfree(opr_buf);
	return SUCCESS;
}
/*
* Function:
*      check the resistor between shortlike channels if less than threshold confirm as short
* INPUT:
*      Short like Information struct pointer
* Returns:
*      SUCCESS: it's shorted FAIL: otherwise
*/
s32 gtp_short_resist_check(struct gt9xx_short_info *short_node)
{
	s32 short_resist = 0;
	struct gt9xx_short_info *node = short_node;
	u8 master = node->master;
	u8 slave = node->slave;
	u8 chnnl_tx[4] = { GT9_DRV_HEAD|13, GT9_DRV_HEAD|28,
				GT9_DRV_HEAD|29, GT9_DRV_HEAD|42 };
	s32 numberator = 0;
	u32 amplifier = 1000;

	if ((((master > chnnl_tx[0]) && (master <= chnnl_tx[1])) &&
		((slave > chnnl_tx[0]) && (slave <= chnnl_tx[1])) ) ||
		(((master >= chnnl_tx[2]) && (master <= chnnl_tx[3])) &&
		((slave >= chnnl_tx[2]) && (slave <= chnnl_tx[3])))) {
		numberator = node->self_data * 40 * amplifier;
		short_resist = numberator/(node->short_code) - 40 * amplifier;
	} else if ((node->slave & (GT9_DRV_HEAD | 0x01)) == 0x01) {
		numberator = node->self_data * 60 * amplifier;
		short_resist = numberator/node->short_code - 40 * amplifier;
	} else {
		numberator = node->self_data * 60 * amplifier;
		short_resist = numberator / node->short_code - 60 * amplifier;
	}

	if (short_resist < 0) {
		short_resist = 0;
	}

	if (short_resist < (gt900_resistor_threshold * amplifier)) {
		node->impedance = short_resist / amplifier;
		return SUCCESS;
	} else {
		return FAIL;
	}
}
/*
* Function:
*      compute the result, whether there are shorts or not
* Input:
*      i2c_client
* Return:
*      SUCCESS
*/
s32 gtp_compute_rslt(struct i2c_client *client)
{
	u16 short_code;
	u8 i = 0, j = 0;
	u16 result_addr;
	u8 *result_buf;
	u16 *self_data;
	s32 ret = 0;
	u16 data_len = 3 + (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2 + 2;
	struct gt9xx_short_info short_node;
	u16 node_idx = 0; /*short_sum index: 0~_SHORT_INFO_MAX*/

	u8 tx_short_num = 0;
	u8 rx_short_num = 0;

	u8 master, slave;

	self_data = (u16*)kmalloc(sizeof(u16) * ((MAX_DRIVER_NUM + MAX_SENSOR_NUM)), GFP_KERNEL);
	result_buf = (u8*)kmalloc(sizeof(u8) * (data_len+2), GFP_KERNEL);
	short_sum = (struct gt9xx_short_info *) kmalloc(sizeof(struct gt9xx_short_info)\
				* _SHORT_INFO_MAX, GFP_KERNEL);

	if (!self_data || !result_buf || !short_sum) {
		GTP_ERROR("allocate memory for short result failed!");
		if (self_data) {
			kfree(self_data);
		}
		if (result_buf) {
			kfree(self_data);
		}
		if (short_sum) {
			kfree(short_sum);
		}
		return FAIL;
	}

	result_buf[0] = 0xA4;
	result_buf[1] = 0xA1;
	gtp_i2c_read(client, result_buf, 2 + 144);
	for (i = 0, j = 0; i < 144; i += 2) {
		self_data[j++] = (u16)(result_buf[2+i] << 8) + (u16)(result_buf[2+i+1]);
	}

	result_buf[0] = 0x88;
	result_buf[1] = 0x02;
	gtp_i2c_read(client, result_buf, 2 + 2);
	tx_short_num = result_buf[2];
	rx_short_num = result_buf[3];

	result_addr = 0x8860;
	data_len = 3 + (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2 + 2;
	for (i = 0; i < tx_short_num; ++i) {
		result_buf[0] = (u8) (result_addr >> 8);
		result_buf[1] = (u8) (result_addr);
		ret = gtp_i2c_read(client, result_buf, data_len+2);
		if (ret < 0)
		{
			GTP_ERROR("read result data failed!");
		}

		short_node.master_is_driver = 1;
		short_node.master = result_buf[2];

		for (j = i + 1; j < MAX_DRIVER_NUM; ++j) {
			short_code = (result_buf[2+3+j*2] << 8) + result_buf[2+3+j*2+1];
			if (short_code > gt900_short_threshold)
			{
				short_node.slave_is_driver = 1;
				short_node.slave = ChannelPackage_TX[j] | GT9_DRV_HEAD;
				short_node.self_data = self_data[j];
				short_node.short_code = short_code;

				ret = gtp_short_resist_check(&short_node);
				if (ret == SUCCESS) {
					if (node_idx < _SHORT_INFO_MAX) {
						short_sum[node_idx++] = short_node;
					}
				}
			}
		}
		/*Tx - Rx*/
		for (j = 0; j < MAX_SENSOR_NUM; ++j) {
			short_code = (result_buf[2+3+84+j*2] << 8) + result_buf[2+3+84+j*2+1];

			if (short_code > gt900_short_threshold) {
				short_node.slave_is_driver = 0;
				short_node.slave = j | GT9_SEN_HEAD;
				short_node.self_data = self_data[MAX_DRIVER_NUM + j];
				short_node.short_code = short_code;

				ret = gtp_short_resist_check(&short_node);
				if (ret == SUCCESS) {
					if (node_idx < _SHORT_INFO_MAX) {
						short_sum[node_idx++] = short_node;
					}
				}
			}
		}

		result_addr += data_len;
	}

	result_addr = 0xA0D2;
	data_len = 3 + MAX_SENSOR_NUM * 2 + 2;
	for (i = 0; i < rx_short_num; ++i) {
		result_buf[0] = (u8) (result_addr >> 8);
		result_buf[1] = (u8) (result_addr);
		ret = gtp_i2c_read(client, result_buf, data_len + 2);
		if (ret < 0) {
			GTP_ERROR("read result data failed!");
		}

		short_node.master_is_driver = 0;
		short_node.master = result_buf[2];

		/*Rx - Rx*/
		for (j = 0; j < MAX_SENSOR_NUM; ++j) {
			if ((j == i) || ( (j < i) && (j & 0x01) == 0)) {
				continue;
			}
			short_code = (result_buf[2+3+j*2] << 8) + result_buf[2+3+j*2+1];

			if (short_code > gt900_short_threshold) {
				short_node.slave_is_driver = 0;
				short_node.slave = j | GT9_SEN_HEAD;
				short_node.self_data = self_data[MAX_DRIVER_NUM + j];
				short_node.short_code = short_code;

				ret = gtp_short_resist_check(&short_node);
				if (ret == SUCCESS) {
					if (node_idx < _SHORT_INFO_MAX) {
						short_sum[node_idx++] = short_node;
					}
				}
			}
		}

		result_addr += data_len;
	}

	if (node_idx == 0) {
		ret = SUCCESS;
	} else {
		for (i = 0, j = 0; i < node_idx; ++i) {

			if ((short_sum[i].master_is_driver)) {
				master = gt9_get_short_tp_chnl(short_sum[i].master-GT9_DRV_HEAD, 1);
			} else {
				master = gt9_get_short_tp_chnl(short_sum[i].master, 0);
			}

			if ((short_sum[i].slave_is_driver)) {
				slave = gt9_get_short_tp_chnl(short_sum[i].slave-GT9_DRV_HEAD, 1);
			} else {
				slave = gt9_get_short_tp_chnl(short_sum[i].slave, 0);
			}

			if (master == 0xFF && slave == 0xFF) {
				GTP_ERROR("unbonded channel (%d, %d) shorted!", short_sum[i].master, short_sum[i].slave);
				continue;
			} else {
				short_sum[j].slave = slave;
				short_sum[j].master = master;
				short_sum[j].slave_is_driver = short_sum[i].slave_is_driver;
				short_sum[j].master_is_driver = short_sum[i].master_is_driver;
				short_sum[j].impedance = short_sum[i].impedance;
				short_sum[j].self_data = short_sum[i].self_data;
				short_sum[j].short_code = short_sum[i].short_code;
				++j;
			}
		}
		node_idx = j;
		if (node_idx == 0) {
			ret = SUCCESS;
		} else {
			for (i = 0; i < node_idx; ++i) {
				GTP_INFO("  %s%02d & %s%02d Shorted! (R = %dKOhm)",
				(short_sum[i].master_is_driver) ? "Drv" : "Sen", short_sum[i].master,
				(short_sum[i].slave_is_driver) ? "Drv" : "Sen", short_sum[i].slave,
				short_sum[i].impedance);
			}
			ret = FAIL;
		}
	}
	kfree(self_data);
	kfree(short_sum);
	kfree(result_buf);
	return ret;
}

s32 gt9_test_gnd_vdd_short(struct i2c_client *client)
{
	u8 *data;
	s32 ret = 0;
	s32 i = 0;
	u16 len = (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2;
	u16 short_code = 0;
	s32 r = -1;
	u32 short_res = 0;
	u8 short_chnl = 0;
	u16 amplifier = 1000;

	data = (u8 *)kmalloc(sizeof(u8) * (len + 2), GFP_KERNEL);
	if (NULL == data) {
		GTP_ERROR("failed to allocate memory for gnd vdd test data buffer");
		return FAIL;
	}

	data[0] = 0xA5;
	data[1] = 0x31;
	gtp_i2c_read(client, data, 2 + len);

	ret = SUCCESS;
	for (i = 0; i < len; i += 2) {
		short_code = (data[2+i] << 8) + (data[2 + i + 1]);
		if (short_code == 0) {
			continue;
		}
		if ((short_code & 0x8000) == 0) {
			#ifdef GTP_SHORT_GND
			r = 5266285 * 10 / (short_code & (~0x8000)) - 40 * amplifier;
			#endif
		} else {
			/*r = ( 1/(((float)(short_code&(~0x8000)))/0.9*0.7/1024/(sys.avdd-0.9)/40) ) -40;*/
			#ifdef GTP_VDD
			r = 40*9*1024*(100*GTP_VDD - 900)/((short_code&(~0x8000))*7) - 40*1000;
			/*GTP_DEBUG("vdd short_code: %d", short_code & (~0x8000));*/
			#endif
		}
		/*GTP_DEBUG("resistor: %d, short_code: %d", r, short_code);*/

		short_res = (r >= 0) ? r : 0xFFFF;
		if (short_res == 0xFFFF) {

		} else {
			if (short_res < (gt900_gnd_resistor_threshold * amplifier)) {
				if (i < MAX_DRIVER_NUM * 2) {
					short_chnl = gt9_get_short_tp_chnl(ChannelPackage_TX[i/2], 1);
					GTP_INFO("driver%02d & gnd/vdd shorted!", short_chnl);
					if (short_chnl == 0xFF) {
						GTP_INFO("unbonded channel");
					} else {
						GTP_INFO("  Drv%02d & GND/VDD Shorted! (R = %dKOhm)", short_chnl, short_res/amplifier);
					}
				} else {
					short_chnl = gt9_get_short_tp_chnl((i/2) - MAX_DRIVER_NUM, 0);
					GTP_INFO("sensor%02d & gnd/vdd shorted!", short_chnl);
					if (short_chnl == 0xFF)	{
						GTP_INFO("unbonded channel");
					} else {
						GTP_INFO("  Sen%02d & GND/VDD Shorted! (R = %dKOhm)", short_chnl, short_res/amplifier);
					}
				}
				ret = FAIL;
			}
		}
	}
	return ret;
}


/*
* leave short test
*/
void gt9xx_leave_short_test(struct i2c_client *client)
{

	gtp_reset_guitar(client, 60);
	msleep(100);
	GTP_INFO("---gtp short test out reset---");
	/*gtp_send_cfg(client);*/
	GTP_INFO("");
	GTP_INFO("---gtp short test end---");
}

s32 gt9xx_get_short_file(struct i2c_client *client)
{
	s32 ret = 0;

	ret = request_firmware(&g_short_fw.fw, SHORT_FW_NAME, &client->dev);
	if (ret < 0) {
		GTP_ERROR("Failed get shorttest firmware:%d\n", ret);
		return FAIL;
	}

	g_short_fw.fw_data = g_short_fw.fw->data;
	g_short_fw.fw_total_len = g_short_fw.fw->size;
	return SUCCESS;
}

s32 gt9xx_parse_cfg(void)
{
	gtp_config[0] = (u8)(GTP_REG_CONFIG_DATA >> 8);
	gtp_config[1] = (u8)GTP_REG_CONFIG_DATA;
	if (gtp_i2c_read(i2c_connect_client, gtp_config,
			GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH) <= 0) {
		GTP_ERROR("Failed to read config!\n");
		return FAIL;
	}

	gt9xx_drv_num = (gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT-GT9_REG_CFG_BEG] & 0x1F)
				+ (gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+1 -GT9_REG_CFG_BEG] & 0x1F);
	gt9xx_sen_num = (gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+2-GT9_REG_CFG_BEG] & 0x0F)
				+ ((gtp_config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT+2-GT9_REG_CFG_BEG]>>4) & 0x0F);

	if (gt9xx_drv_num > MAX_DRIVER_NUM) {
		GTP_ERROR("driver number error!");
		return FAIL;
	}
	if (gt9xx_sen_num > MAX_SENSOR_NUM) {
		GTP_ERROR("sensor number error!");
		return FAIL;
	}
	return SUCCESS;
}

/*
* Function:
*		gt9 series ic short test function
* Input:
*		I2c_client, i2c device
* Return:
*		SUCCESS: test succeed, FAIL: test failed
*/
s32 gt9xx_short_test(struct i2c_client * client)
{
	s32 ret = 0;
	s32 ret2 = 0;
	u8 i = 0;
	u8 opr_buf[60] = {0};
	u8 retry = 0;
	u8 drv_sen_chksum = 0;
	u8 retry_load = 0;

	GTP_INFO("---gtp short test---");
	GTP_INFO("Step 1: reset guitar, hang up ss51 dsp");
	/*get short bin file*/
	ret = gt9xx_get_short_file(client);
	if(ret == FAIL) {
		GTP_ERROR("failed to get short file \n");
		return ret;
	}

	ret = gt9xx_short_parse_cfg();
	if (FAIL == ret) {
		GTP_ERROR("You May check your IIC connection.");
		goto short_test_exit;
	}

load_dsp_again:
	/*step 1: reset guitar, delay 1ms,  hang up ss51 and dsp*/
	gtp_reset_guitar(client, 20);

	while(retry++ < 200) {
		/*Hold ss51 & dsp*/
		ret = gtp_write_register(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
		if( (ret <= 0)) {
			GTP_ERROR("Hold ss51 & dsp I2C error,retry:%d", retry);
			gtp_reset_guitar(client, 10);
			continue;
		}
		GTP_ERROR("Hold ss51 & dsp confirm 0x4180 failed,value:%d", opr_buf[GTP_ADDR_LENGTH]);
		msleep(2);

		/*confirm hold*/
		opr_buf[GTP_ADDR_LENGTH] = 0x00;

		ret = gtp_read_register(client, _rRW_MISCTL__SWRST_B0_, opr_buf);
		if(ret <= 0) {
			GTP_ERROR("Hold ss51 & dsp I2C error,retry:%d", retry);
			gtp_reset_guitar(client, 10);
			continue;
		}
		if(0x0C == opr_buf[GTP_ADDR_LENGTH]) {
			GTP_ERROR("Hold ss51 & dsp confirm SUCCESS");
			break;
		}
	}

	if(retry >= 200) {
		GTP_ERROR("Enter update Hold ss51 failed.");
		goto short_test_exit;
	}
	/* DSP_CK and DSP_ALU_CK PowerOn */
	ret2 = gtp_write_register(client, 0x4010, 0x00);
	if (ret2 <= 0) {
		GTP_ERROR("Enter update PowerOn DSP failed.");
		goto short_test_exit;
	}

	/*step2: burn dsp_short code*/
	GTP_INFO("step 2: burn dsp_short code");
	gtp_write_register(client, _bRW_MISCTL__TMR0_EN, 0x00); /*clear watchdog*/
	gtp_write_register(client, _bRW_MISCTL__CACHE_EN, 0x00); /*clear cache*/
	gtp_write_register(client, _rRW_MISCTL__BOOTCTL_B0_, 0x02); /*boot from sram*/

	gtp_write_register(client, _bRW_MISCTL__SRAM_BANK, 0x00); /*select bank 0*/
	gtp_write_register(client, _bRW_MISCTL__MEM_CD_EN, 0x01); /*allow AHB bus accessing code sram*/

	/*burn dsp_short code*/
	ret = gtp_burn_dsp_short(client);

	if (ret != SUCCESS) {
		if(retry_load++ < 5) {
			GTP_ERROR("Load dsp failed,times %d retry load!", retry_load);
			goto load_dsp_again;
		} else {
			GTP_INFO("Step 2: burn dsp_short code");
			GTP_ERROR("burn dsp_short Timeout!");
			goto short_test_exit;
		}
	}

	GTP_INFO("Step 2: burn dsp_short code");
	/*step3: run dsp_short, read results*/
	GTP_INFO("Step 3: run dsp_short code, confirm it's runnin'");
	gtp_write_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG, 0x00); /*clear dsp_short running flag*/
	gtp_write_register(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x03);/*set scramble*/

	gtp_write_register(client, _bWO_MISCTL__CPU_SWRST_PULSE, 0x01); /*reset software*/

	gtp_write_register(client, _rRW_MISCTL__SWRST_B0_, 0x08);   /*release dsp*/

	msleep(50);
	/*confirm dsp is running*/
	i = 0;
	while (i < 20) {
		opr_buf[2] = 0x00;
		gtp_read_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG, opr_buf);
		if (opr_buf[2] == 0xAA) {
			break;
		}
		i++;
		msleep(10);
	}
	if (i == 20) {
		GTP_ERROR("step 3: dsp is not running!");
		goto short_test_exit;
	}
	/*step4: host configure ic, get test result*/

	GTP_INFO("Step 4: host config ic, get test result");
	GTP_INFO(" Short Threshold: 10");
	opr_buf[0] = 0x88;
	opr_buf[1] = 0x04;
	opr_buf[2] = 0;
	opr_buf[3] = 10;
	gtp_i2c_write(client, opr_buf, 4);

	/*ADC Read Delay*/
	GTP_INFO(" ADC Read Delay: 150");
	opr_buf[0] = 0x88;
	opr_buf[1] = 0x06;
	opr_buf[2] = (u8)(150 >> 8);
	opr_buf[3] = (u8)(150);
	gtp_i2c_write(client, opr_buf, 4);

	/*DiffCode Short Threshold*/
	GTP_INFO(" DiffCode Short Threshold: 20");
	opr_buf[0] = 0x88;
	opr_buf[1] = 0x51;
	opr_buf[2] = (u8)(20 >> 8);
	opr_buf[3] = (u8)(20);
	gtp_i2c_write(client, opr_buf, 4);

	/*Config Driver & Sensor Order*/
#if GTP_DEBUG_ON
	GTP_INFO("<<-GTP-DEBUG->> Driver Map:\n");
	GTP_INFO("IC Driver:");
	for (i = 0; i < MAX_DRIVER_NUM; ++i) {
		GTP_INFO(" %2d", cfg_drv_order[i]);
	}
	GTP_INFO("\n");
	GTP_INFO("TP Driver:");
	for (i = 0; i < MAX_DRIVER_NUM; ++i) {
		GTP_INFO(" %2d", i);
	}
	GTP_INFO("\n");

	GTP_INFO("<<-GTP-DEBUG->> Sensor Map:\n");
	GTP_INFO("IC Sensor:");
	for (i = 0; i < MAX_SENSOR_NUM; ++i) {
		GTP_INFO(" %2d", cfg_sen_order[i]);
	}
	GTP_INFO("\n");
	GTP_INFO("TP Sensor:");
	for (i = 0; i < MAX_SENSOR_NUM; ++i) {
		GTP_INFO(" %2d", i);
	}
	GTP_INFO("\n");
#endif

	opr_buf[0] = 0x88;
	opr_buf[1] = 0x08;
	for (i = 0; i < MAX_DRIVER_NUM; ++i) {
		opr_buf[2 + i] = cfg_drv_order[i];
		drv_sen_chksum += cfg_drv_order[i];
	}
	gtp_i2c_write(client, opr_buf, MAX_DRIVER_NUM + 2);

	opr_buf[0] = 0x88;
	opr_buf[1] = 0x32;
	for (i = 0; i < MAX_SENSOR_NUM; ++i) {
		opr_buf[2+i] = cfg_sen_order[i];
		drv_sen_chksum += cfg_sen_order[i];
	}
	gtp_i2c_write(client, opr_buf, MAX_SENSOR_NUM + 2);

	opr_buf[0] = 0x88;
	opr_buf[1] = 0x50;
	opr_buf[2] = 0 - drv_sen_chksum;
	gtp_i2c_write(client, opr_buf, 2 + 1);

	/*clear waiting flag, run dsp*/
	gtp_write_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG, 0x04);

	/*inquirying test status until it's okay*/
	for (i = 0;;++i) {
		gtp_read_register(client, 0x8800, opr_buf);
		if (opr_buf[2] == 0x88) {
			break;
		}
		msleep(50);
		if ( i > 100 ) {
			GTP_ERROR("step 4: inquiry test status timeout!");
			goto short_test_exit;
		}
	}

	/* step 5: compute the result
	short flag:
	bit0: Rx & Rx
	bit1: Tx & Tx
	bit2: Tx & Rx
	bit3: Tx/Rx & GND/VDD
	*/
	gtp_read_register(client, 0x8801, opr_buf);
	GTP_INFO("short_flag = 0x%02X", opr_buf[2]);
	GTP_INFO("");
	GTP_INFO("Short Test Result:");

	GTP_INFO("ctptest_TP-short---gt900_resistor_threshold = %d", gt900_resistor_threshold);
	GTP_INFO("ctptest_TP-shrlt---gt900_gnd_resistor_threshold = %d", gt900_gnd_resistor_threshold);

	if ((opr_buf[2] & 0x0f) == 0) {
		GTP_INFO("  PASS!");
		ret = SUCCESS;
	} else {
		ret2 = SUCCESS;
		if ((opr_buf[2] & 0x08) == 0x08) {
			ret2 = gt9_test_gnd_vdd_short(client);
		}
		ret = gtp_compute_rslt(client);
		if (ret == SUCCESS && ret2 == SUCCESS) {
			GTP_INFO("  PASS!");
			ret = SUCCESS;
		} else {
			ret = FAIL;
		}
	}
	/*boot from rom and download code from flash to ram*/
	gtp_write_register(client, _rRW_MISCTL__BOOT_CTL_, 0x99);
	gtp_write_register(client, _rRW_MISCTL__BOOTCTL_B0_, 0x08);

	gt9xx_leave_short_test(client);

	if(g_short_fw.fw != NULL) {
		g_short_fw.fw_data = NULL;
		g_short_fw.fw_total_len = 0;
		release_firmware(g_short_fw.fw);	
	}

	gtp_arch_file_append_no_len("\n");
	return ret;

short_test_exit:
	if(g_short_fw.fw != NULL) {
		g_short_fw.fw_data = NULL;
		g_short_fw.fw_total_len = 0;
		release_firmware(g_short_fw.fw);	
	}
	/*boot from rom and download code from flash to ram*/
	gtp_write_register(client, _rRW_MISCTL__BOOT_CTL_, 0x99);
	gtp_write_register(client, _rRW_MISCTL__BOOTCTL_B0_, 0x08);

	gt9xx_leave_short_test(client);

	gtp_arch_file_append_no_len("\n");
	return FAIL;
}

u32 endian_mode(void)
{
	union {
		s32 i; 
		s8 c;
	} endian;

	endian.i = 1;

	if (1 == endian.c) {
		return MYBIG_ENDIAN;
	} else {
		return MYLITLE_ENDIAN;
	}
}
/*
*********************************************************************************************************
* Function:
*   send read rawdata cmd
* Input:
*   i2c_client* client: i2c device
* Return:
*   SUCCESS: send process succeed, FAIL: failed
*********************************************************************************************************
*/
s32 gt9_read_raw_cmd(struct i2c_client* client)
{
	u8 raw_cmd[3] = {(u8)(GTP_REG_READ_RAW >> 8), (u8)GTP_REG_READ_RAW, 0x01};
	s32 ret = -1;
	GTP_INFO("Send read raw data command");
	ret = gtp_i2c_write(client, raw_cmd, 3);
	if(ret <= 0) {
		GTP_ERROR("i2c write failed.");
		return FAIL;
	}
	msleep(10);
	return SUCCESS;
}

s32 gt9_read_coor_cmd(struct i2c_client *client)
{
	u8 raw_cmd[3] = {(u8)(GTP_REG_READ_RAW >> 8), (u8)GTP_REG_READ_RAW, 0x0};
	s32 ret = -1;

	ret = gtp_i2c_write(client, raw_cmd, 3);
	if (ret < 0)
	{
		GTP_ERROR("i2c write coor cmd failed!");
		return FAIL;
	}
	msleep(10);
	return SUCCESS;
}
/*
*********************************************************************************************************
* Function:
*   read rawdata from ic registers
* Input:
*   u16* data: rawdata buffer
*   i2c_client* client: i2c device
* Return:
*   SUCCESS: read process succeed, FAIL:  failed
*********************************************************************************************************
*/
s32 gtp_read_rawdata(struct i2c_client* client, u16* data)
{
	s32 ret = -1;
	u16 retry = 0;
	u8 read_state[3] = {(u8)(GTP_REG_RAW_READY>>8), (u8)GTP_REG_RAW_READY, 0};
	u16 i = 0, j = 0;
	u8 *read_rawbuf;
	u8 tail, head;

	read_rawbuf = (u8*)kmalloc(sizeof(u8) * (gt9xx_drv_num*gt9xx_sen_num * 2 + GTP_ADDR_LENGTH), GFP_KERNEL);

	if (NULL == read_rawbuf) {
		GTP_ERROR("failed to allocate for read_rawbuf");
		return FAIL;
	}

	if(data == NULL) {
		GTP_ERROR("Invalid raw buffer.");
		goto have_error;
	}

	msleep(10);
	while (retry++ < GTP_WAIT_RAW_MAX_TIMES) {
		ret = gtp_i2c_read(client, read_state, 3);
		if(ret <= 0) {
			GTP_ERROR("i2c read failed.return: %d", ret);
			continue;
		}
		if ((read_state[GTP_ADDR_LENGTH] & 0x80) == 0x80) {
			/*GTP_INFO("Raw data is ready.");*/
			break;
		}
		if ((retry%20) == 0) {
			/*GTP_DEBUG("(%d)read_state[2] = 0x%02X", retry, read_state[GTP_ADDR_LENGTH]);*/
			if (retry == 100) {
				gt9_read_raw_cmd(client);
			}
		}
		msleep(5);
	}
	if (retry >= GTP_WAIT_RAW_MAX_TIMES) {
		GTP_ERROR("Wait raw data ready timeout.");
		goto have_error;
	}

	if (chip_type_gt9f) {
		read_rawbuf[0] = (u8)( GTP_REG_RAW_DATA_GT9F >> 8);
		read_rawbuf[1] = (u8)( GTP_REG_RAW_DATA_GT9F );
	} else {
		read_rawbuf[0] = (u8)( GTP_REG_RAW_DATA >> 8);
		read_rawbuf[1] = (u8)( GTP_REG_RAW_DATA );
	}

	ret = gtp_i2c_read(client, read_rawbuf, GTP_ADDR_LENGTH + ((gt9xx_drv_num*gt9xx_sen_num)*2));
	if(ret <= 0) {
		GTP_ERROR("i2c read rawdata failed.");
		goto have_error;
	}
	gtp_i2c_end_cmd(client);

	if (endian_mode() == MYBIG_ENDIAN) {
		head = 0;
		tail =1;
		/*GTP_DEBUG("Big Endian.");*/
	} else {
		head = 1;
		tail = 0;
		/*GTP_DEBUG("Little Endian.");*/
	}
	/*GTP_DEBUG("raw addr:%d,%d\n",read_rawbuf[0],read_rawbuf[1]);*/
	for(i=0,j = 0; i < ((gt9xx_drv_num*gt9xx_sen_num)*2); i+=2) {
		data[i/2] = (u16)(read_rawbuf[i+head+GTP_ADDR_LENGTH]<<8) + (u16)read_rawbuf[GTP_ADDR_LENGTH+i+tail];
	#if GTP_DEBUG_ON
		GTP_INFO("%4d ", data[i/2]);
		++j;
		if((j%gt9xx_drv_num) == 0) {
			GTP_INFO("\n");
		}
	#endif
	}

	kfree(read_rawbuf);
	return SUCCESS;
have_error:
	kfree(read_rawbuf);
	return FAIL;
}
/*
*********************************************************************************************************
* Function:
*   read rawdata from ic registers
* Input:
*   u16* data: rawdata buffer
*   i2c_client* client: i2c device
* Return:
*   SUCCESS: read process succeed, FAIL:  failed
*********************************************************************************************************
*/
s32 gtp_read_diffdata(struct i2c_client* client)
{
	s32 ret = -1;
	u16 retry = 0;
	u8 read_state[3] = {(u8)(GTP_REG_RAW_READY>>8), (u8)GTP_REG_RAW_READY, 0};
	u16 i = 0, j = 0;
	u8 *read_diffbuf;
	u8 tail, head;

	read_diffbuf = (u8*)kmalloc(sizeof(u8) *\
		(gt9xx_drv_num*gt9xx_sen_num * 2 + GTP_ADDR_LENGTH), GFP_KERNEL);

	if (NULL == read_diffbuf)
	{
		GTP_ERROR("failed to allocate for read_rawbuf");
		return FAIL;
	}

	msleep(10);
	while (retry++ < GTP_WAIT_RAW_MAX_TIMES) {
		ret = gtp_i2c_read(client, read_state, 3);
		if(ret <= 0) {
			GTP_ERROR("i2c read failed.return: %d", ret);
			continue;
		}
		if ((read_state[GTP_ADDR_LENGTH] & 0x80) == 0x80) {
			/*GTP_DEBUG("Raw data is ready.");*/
			break;
		}
		if ((retry%20) == 0) {
			/*GTP_INFO("(%d)read_state[2] = 0x%02X", retry, read_state[GTP_ADDR_LENGTH]);*/
			if (retry == 100) {
				gt9_read_raw_cmd(client);
			}
		}
		msleep(5);
	}
	if (retry >= GTP_WAIT_RAW_MAX_TIMES) {
		GTP_ERROR("Wait raw data ready timeout.");
		goto have_error;
	}
	GTP_ERROR("Wait diff data ready.");
	if (chip_type_gt9f) {
		read_diffbuf[0] = (u8)( GTP_REG_RAW_DATA_GT9F >> 8);
		read_diffbuf[1] = (u8)( GTP_REG_RAW_DATA_GT9F );
	} else {
		read_diffbuf[0] = (u8)( GTP_REG_DIFF_DATA >> 8);
		read_diffbuf[1] = (u8)( GTP_REG_DIFF_DATA );
		/*read_diffbuf[0] = 0xA1;
		read_diffbuf[1] = 0x60;*/
	}

	ret = gtp_i2c_read(client, read_diffbuf, GTP_ADDR_LENGTH + ((gt9xx_drv_num*gt9xx_sen_num)*2));
	if(ret <= 0) {
		GTP_ERROR("i2c read rawdata failed.");
		goto have_error;
	}
	gtp_i2c_end_cmd(client);

	if (endian_mode() == MYBIG_ENDIAN) {
		head = 0;
		tail =1;
		/*GTP_INFO("Big Endian.");*/
	} else {
		head = 1;
		tail = 0;
		/*GTP_DEBUG("Little Endian.");*/
	}

	for(i=0,j = 0; i < ((gt9xx_drv_num*gt9xx_sen_num)*2); i+=2) {
		jitter_limit_temp[i/2] = (u16)(read_diffbuf[i+head+GTP_ADDR_LENGTH]<<8) +\
			(u16)read_diffbuf[GTP_ADDR_LENGTH+i+tail];
		if(jitter_limit_temp[i/2] > 32767) {
			jitter_limit_temp[i/2] = 65535-jitter_limit_temp[i/2];
		}
	#if GTP_DEBUG_ON
		GTP_INFO("%4d", jitter_limit_temp[i/2]);
		++j;
		if((j%gt9xx_drv_num) == 0) {
			GTP_INFO("\n");
		}
	#endif
	}

	kfree(read_diffbuf);
	return SUCCESS;
have_error:
	kfree(read_diffbuf);
	return FAIL;
}
/*
*********************************************************************************************************
* Function:
*   rawdata test initilization function
* Input:
*   u32 check_types: test items
*********************************************************************************************************
*/
static s32 gtp_raw_test_init(void)
{
	u16 i = 0;

	test_rslt_buf = (s32*) kmalloc(sizeof(s32)*GTP_OPEN_SAMPLE_NUM, GFP_KERNEL);
	touchpad_sum = (struct gt9xx_open_info*) kmalloc(sizeof(struct gt9xx_open_info)\
					* (4 * _BEYOND_INFO_MAX + 1), GFP_KERNEL);

	if (have_key && key_is_isolated) {
		avrg_raw_len = gt9xx_sc_pxl_cnt;
	} else {
		avrg_raw_len = gt9xx_pixel_cnt;
	}
	avrg_raw_buf = (u32 *)kzalloc(sizeof(u32) * avrg_raw_len, GFP_KERNEL);

	if (NULL == test_rslt_buf || NULL == touchpad_sum || NULL == avrg_raw_buf) {
		GTP_ERROR("alloc test_rslt_buf or touchpad_sum or avrg_raw_buf failed\n");
		return FAIL;
	}
	memset(touchpad_sum, 0, sizeof(struct gt9xx_open_info) * (4 * _BEYOND_INFO_MAX + 1));

	for (i = 0; i < (4 * _BEYOND_INFO_MAX); ++i)
	{
		touchpad_sum[i].driver = 0xFF;
	}

	for (i = 0; i < GTP_OPEN_SAMPLE_NUM; i++) {
		test_rslt_buf[i] = _CHANNEL_PASS;
	}
	return SUCCESS;
}
/*
*********************************************************************************************************
* Function:
*   touchscreen rawdata max limit test
* Input:
*   u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
static s32 gtp_raw_max_test_re(u16 *raw_buf)
{
	u16 i, j;
	u8 driver, sensor;
	u8 sum_base = 0 * _BEYOND_INFO_MAX;
	u8 new_flag = 0;
	s32 ret = SUCCESS;

	for (i = 0; i < gt9xx_sc_pxl_cnt; i++) {
		if (raw_buf[i] > max_limit_vale_re[i]) {
			test_rslt_buf[rslt_buf_idx] |= _BEYOND_MAX_LIMIT;
			driver = (i/gt9xx_sen_num);
			sensor = (i%gt9xx_sen_num);
			new_flag = 0;
			for (j = sum_base; j < (sum_base+_BEYOND_INFO_MAX); ++j) {
				if (touchpad_sum[j].driver == 0xFF) {
					new_flag = 1;
					break;
				}
				if ((driver == touchpad_sum[j].driver)
					&& (sensor == touchpad_sum[j].sensor)) {
					touchpad_sum[j].times++;
					new_flag = 0;
					break;
				}
			}
			if (new_flag) {
				touchpad_sum[j].driver = driver;
				touchpad_sum[j].sensor = sensor;
				touchpad_sum[j].beyond_type |= _BEYOND_MAX_LIMIT;
				touchpad_sum[j].raw_val = raw_buf[i];
				touchpad_sum[j].times = 1;
				GTP_INFO("[%d, %d]rawdata: %d, raw max limit: %d",
					driver, sensor, raw_buf[i], max_limit_vale_re[i]);
			}
			return FAIL;
		}	
	}
	return ret;
}


/*
*********************************************************************************************************
* Function:
*   touchscreen rawdata min limit test
* Input:
*   u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
static s32 gtp_raw_min_test_re(u16 *raw_buf)
{
	u16 i, j=0;
	u8 driver, sensor;
	u8 sum_base = 1 * _BEYOND_INFO_MAX;
	u8 new_flag = 0;
	s32 ret = SUCCESS;

	for (i = 0; i < gt9xx_sc_pxl_cnt; i++) {
		if (raw_buf[i] < min_limit_vale_re[i]) {
			test_rslt_buf[rslt_buf_idx] |= _BEYOND_MIN_LIMIT;
			driver = (i/gt9xx_sen_num);
			sensor = (i%gt9xx_sen_num);
			new_flag = 0;
			for (j = sum_base; j < (sum_base+_BEYOND_INFO_MAX); ++j) {
				if (touchpad_sum[j].driver == 0xFF) {
					new_flag = 1;
					break;
				}
				if ((driver == touchpad_sum[j].driver)
					&& (sensor == touchpad_sum[j].sensor)) {
					touchpad_sum[j].times++;
					new_flag = 0;
					break;
				}
			}
			if (new_flag) {
				touchpad_sum[j].driver = driver;
				touchpad_sum[j].sensor = sensor;
				touchpad_sum[j].beyond_type |= _BEYOND_MIN_LIMIT;
				touchpad_sum[j].raw_val = raw_buf[i];
				touchpad_sum[j].times = 1;
				GTP_INFO("[%d, %d]rawdata: %d, raw min limit: %d",
					driver, sensor, raw_buf[i], min_limit_vale_re[i]);
			}
			ret = FAIL;
		}
	}
	return ret;
}
 
unsigned char AreaAccordCheck(u16* raw_buf)
{
	int i,j,index;
	u16 temp;
	u16 accord_temp;
	s32 ret = SUCCESS;

	for (i = 0; i < gt9xx_sen_num; i++) {
		for(j = 0; j < gt9xx_drv_num; j++) {
			index = i+j*gt9xx_sen_num;
			
			accord_temp = 0;
			temp = 0;
	
			if (j == 0) {
				if(raw_buf[i+(j+1)*gt9xx_sen_num] > raw_buf[index]) {
					accord_temp = ((1000*(raw_buf[i+(j+1)*gt9xx_sen_num] - raw_buf[index])) / raw_buf[index]);
				} else {
					accord_temp = ((1000*(raw_buf[index] - raw_buf[i+(j+1)*gt9xx_sen_num])) / raw_buf[index]);
				}
			} else if(j == gt9xx_drv_num-1) {
				if(raw_buf[i+(j-1)*gt9xx_sen_num] > raw_buf[index]) {
					accord_temp = ((1000*(raw_buf[i+(j-1)*gt9xx_sen_num] - raw_buf[index])) / raw_buf[index]);
				} else {
					accord_temp = ((1000*(raw_buf[index] - raw_buf[i+(j-1)*gt9xx_sen_num])) / raw_buf[index]);
				}
			} else {
				if(raw_buf[i+(j+1)*gt9xx_sen_num] > raw_buf[index]) {
					accord_temp = ((1000*(raw_buf[i+(j+1)*gt9xx_sen_num] - raw_buf[index]))/ raw_buf[index]);
				} else {
					accord_temp = ((1000*(raw_buf[index] - raw_buf[i+(j+1)*gt9xx_sen_num]))/ raw_buf[index]);
				}

				if(raw_buf[i+(j-1)*gt9xx_sen_num] > raw_buf[index]) {
					temp = ((1000*(raw_buf[i+(j-1)*gt9xx_sen_num] - raw_buf[index])) / raw_buf[index]);
				} else {
					temp = ((1000*(raw_buf[index] - raw_buf[i+(j-1)*gt9xx_sen_num])) / raw_buf[index]);
				}

				if (temp > accord_temp) {
					accord_temp = temp;
				}
			}
			if (i == 0) {
				if(raw_buf[i+1+j*gt9xx_sen_num] > raw_buf[index]) {
					temp = ((1000*(raw_buf[i+1+j*gt9xx_sen_num] - raw_buf[index])) / raw_buf[index]);
				} else {
					temp = ((1000*(raw_buf[index] - raw_buf[i+1+j*gt9xx_sen_num])) / raw_buf[index]);
				}
				if (temp > accord_temp) {
					accord_temp = temp;
				}
			} else if (i == gt9xx_sen_num-1) {
				if(raw_buf[i-1+j*gt9xx_sen_num] > raw_buf[index]) {
					temp = ((1000*(raw_buf[i-1+j*gt9xx_sen_num] - raw_buf[index])) / raw_buf[index]);
				} else {
					temp = ((1000*(raw_buf[index] - raw_buf[i-1+j*gt9xx_sen_num])) / raw_buf[index]);
				}
				if (temp > accord_temp) {
					accord_temp = temp;
				}
			} else {
				if(raw_buf[i+1+j*gt9xx_sen_num] > raw_buf[index]) {
					temp = ((1000*(raw_buf[i+1+j*gt9xx_sen_num] - raw_buf[index]))/ raw_buf[index]);
				} else {
					temp = ((1000*(raw_buf[index] - raw_buf[i+1+j*gt9xx_sen_num]))/ raw_buf[index]);
				}

				if (temp > accord_temp) {
					accord_temp = temp;
				}
				if(raw_buf[i-1+j*gt9xx_sen_num] > raw_buf[index]) {
					temp = ((1000*(raw_buf[i-1+j*gt9xx_sen_num] - raw_buf[index])) /raw_buf[index]);
				} else {
					temp = ((1000*(raw_buf[index] - raw_buf[i-1+j*gt9xx_sen_num])) /raw_buf[index]);
				}

				if (temp > accord_temp) {
					accord_temp = temp;
				}
			}

			if (accord_temp > accord_limit_vale_re[index]) {         
				GTP_ERROR("AreaAccordCheck----gt9xx_drv_num=%d,gt9xx_sen_num=%d",
					gt9xx_drv_num,gt9xx_sen_num);
				ret = FAIL;
			}
			accord_limit_temp[index] = accord_temp;		
		}
	} /*end of for (j = 0; j < gt9xx_sen_num*DRIVER_NUM; j++)*/
	return ret;
}

/*
*********************************************************************************************************
* Function:
*   analyse rawdata retrived from ic registers
* Input:
*   u16 *raw_buf, buffer for rawdata,
*   u32 check_types, test items
* Return:
*   SUCCESS: test process succeed, FAIL: failed
*********************************************************************************************************
*/
static u32 gtp_raw_test_re(u16 *raw_buf, u32 check_types)
{
	s32 ret =0;
	s32 ret1;
	ret1 = 0;
	if (raw_buf == NULL) {
		GTP_ERROR("Invalid raw buffer pointer!");
		return FAIL;
	}

	if (check_types & _MAX_CHECK) {
		GTP_INFO("accord max test");
		ret = gtp_raw_max_test_re(raw_buf);
		GTP_INFO("accord---max ret = %d", ret);
		if(ret) {
			ret1++ ;
			Ito_result_info[RAWDATA_MAXDATA_ID].testitem=RAWDATA_MAXDATA_ID;
			Ito_result_info[RAWDATA_MAXDATA_ID].result='P';
			GTP_INFO("gtp_raw_test_re max test result = pass,ret1 =%d\n",ret1);
		} else {
			ret1 = 0;
			Ito_result_info[RAWDATA_MAXDATA_ID].testitem=RAWDATA_MAXDATA_ID;
			Ito_result_info[RAWDATA_MAXDATA_ID].result='F';
			test_error_code|=0x01;
			GTP_INFO("gtp_raw_test_re max test result = failed\n");
		}
	}

	if (check_types & _MIN_CHECK) {
		GTP_INFO("accord min test");
		ret = gtp_raw_min_test_re(raw_buf);
		GTP_INFO("accord---min ret = %d", ret);
		if(ret) {
			ret1++ ;
			Ito_result_info[RAWDATA_MINDATA_ID].testitem=RAWDATA_MINDATA_ID;
			Ito_result_info[RAWDATA_MINDATA_ID].result='P';
			GTP_INFO("gtp_raw_test_re min test result = pass,ret1 = %d\n",ret1);
		} else {
			ret1 = 0;
			Ito_result_info[RAWDATA_MINDATA_ID].testitem=RAWDATA_MINDATA_ID;
			Ito_result_info[RAWDATA_MINDATA_ID].result='F';
			test_error_code|=0x02;
			GTP_INFO("gtp_raw_test_re min test result = failed\n");
		}
	}

	if (check_types & _ACCORD_CHECK) {
		GTP_INFO("accord check");
		ret  = AreaAccordCheck(raw_buf);   
		GTP_INFO("accord---accord ret = %d", ret);
		if(ret) {
			ret1++ ;
			Ito_result_info[RAWDATA_ACCORD_ID].testitem=RAWDATA_ACCORD_ID;
			Ito_result_info[RAWDATA_ACCORD_ID].result='P';
			GTP_INFO("gtp_raw_test_re check result = pass,ret1 = %d\n",ret1);
		} else {
			ret1 = 0;
			Ito_result_info[RAWDATA_ACCORD_ID].testitem=RAWDATA_ACCORD_ID;
			Ito_result_info[RAWDATA_ACCORD_ID].result='F';
			test_error_code|=0x04;
			GTP_INFO("gtp_raw_test_re check result = failed\n");
		}
	}
	GTP_INFO("gtp_raw_test_re result ret1 = %d", ret1);

	if (ret1 == 3) {
		GTP_INFO(" Rawdata Test PASS!");
		ret = SUCCESS;
	} else {
		ret = FAIL;
	}

	return ret;
}

/*
====================================================================================================
* Function:
*   output the test result
* Return:
*   return the result. if result == 0, the TP is ok, otherwise list the beyonds
====================================================================================================
*/
void gtp_set_avrg_rawbuf(u16 *raw_buf, s32 index)
{
	s32 i = 0;

	if (0 == index) {
		for (i = 0; i < avrg_raw_len; ++i) {
			avrg_raw_buf[i] = raw_buf[i];
		}
		if (have_key && key_is_isolated) {
			for (i = 0; i < 4; ++i) {
				key_isolated_avrg[i] = raw_buf[avrg_raw_len + key_iso_pos[i+1]];
			}
		}
	} else {
		for (i = 0; i < avrg_raw_len; ++i) {
			avrg_raw_buf[i] += raw_buf[i];
		}
		if (have_key && key_is_isolated) {
			for (i = 0; i < 4; ++i) {
				key_isolated_avrg[i] += raw_buf[avrg_raw_len + key_iso_pos[i+1]];
			}
		}
	}
}

void gtp_opentest_arch_raw(void)
{
	u32 i = 0, j = 0;
	u16 avrg_min = 0, avrg_max = 0, avrg_arg = 0;
	long tmp = 0;
	u32 buf_len = 0;
	char *line_buf;

	line_buf = (char *) kzalloc(200, GFP_KERNEL);
	if (!line_buf) {
		GTP_INFO("Failed to allocate memory for line buf");
		return;
	}
	for (i = 0; i < avrg_raw_len; ++i) {
		avrg_raw_buf[i] /= GTP_OPEN_SAMPLE_NUM;
		tmp += avrg_raw_buf[i];
		if (avrg_raw_buf[i] > avrg_max) {
			avrg_max = avrg_raw_buf[i];
		}
		if (0 == i) {
			avrg_min = avrg_raw_buf[i];
		} else if (avrg_raw_buf[i] < avrg_min) {
			avrg_min = avrg_raw_buf[i];
		}
    }
	avrg_arg = tmp / avrg_raw_len;

	sprintf(line_buf, "\nAverage Raw Data(Drv*Sen: %d*%d, Key:%d): \n",\
				gt9xx_drv_num, gt9xx_sen_num, have_key);
	gtp_arch_file_append_no_len(line_buf);
	for (i = 0; i < avrg_raw_len; i += gt9xx_sen_num) {
		memset(line_buf, 0, 200);
		line_buf[0] = '\t';
		buf_len = 1;
		for (j = 0; j < gt9xx_sen_num; ++j) {
			buf_len += sprintf(&line_buf[buf_len], "%04d  ", avrg_raw_buf[i+j]);
		}
		gtp_arch_file_append_no_len(line_buf);
		gtp_arch_file_append_no_len("\n");
	}
	gtp_arch_file_append_no_len("Statics:\n");
	memset(line_buf, 0, 200);
	sprintf(line_buf, "\tmax: %04d  min: %04d  average: %04d\n",
				avrg_max, avrg_min, avrg_arg);
	gtp_arch_file_append_no_len(line_buf);

	if (have_key && key_is_isolated) {
		gtp_arch_file_append_no_len("Key Average: \n");
		for (i = 0; i < 4; ++i) {
			key_isolated_avrg[i] /= GTP_OPEN_SAMPLE_NUM;
		}
		memset(line_buf, 0, 200);
		line_buf[0] = '\t';
		buf_len = 1;
		for (i = 0; i < key_iso_pos[0]; ++i) {
			buf_len += sprintf(&line_buf[buf_len], "key%d: %04d  ", i+1, key_isolated_avrg[i]);
		}
		gtp_arch_file_append_no_len(line_buf);
	}
}

/*
 ===================================================
 * Function:
 *      test gt9 series ic open test
 * Input:
 *      client, i2c_client
 * Return:
 *      SUCCESS: test process success, FAIL, test process failed
 *
 ===================================================
*/
s32 gt9xx_open_test(struct i2c_client * client)
{
	u16 i = 0;
	s32 ret = FAIL;
	s32 ret1 = FAIL;
	u16 *raw_buf = NULL;
	struct goodix_ts_data *ts = NULL;

	ts = i2c_get_clientdata(i2c_connect_client);
	set_bit(RAW_DATA_MODE, &ts->flags);
	ITO_TEST_COUNT++;
	if (ITO_TEST_COUNT>50) {
		ITO_TEST_COUNT=0;
	}

	GTP_INFO("---gtp open test---");
	raw_buf = (u16*)kmalloc(sizeof(u16)* (gt9xx_drv_num*gt9xx_sen_num) * 6, GFP_KERNEL);
	if (NULL == raw_buf) {
		GTP_ERROR("failed to allocate mem for raw_buf!");
		ret = FAIL;
		goto open_test_exit;
	}
	memset(raw_buf, 0, sizeof(u16)* (gt9xx_drv_num*gt9xx_sen_num) * 6);
	max_limit_vale_re = raw_buf + sizeof(u16)* (gt9xx_drv_num*gt9xx_sen_num);
	min_limit_vale_re = raw_buf + sizeof(u16)* (gt9xx_drv_num*gt9xx_sen_num) *2;
	accord_limit_vale_re = raw_buf + sizeof(u16)* (gt9xx_drv_num*gt9xx_sen_num) *3;
	accord_limit_temp = raw_buf + sizeof(u16)* (gt9xx_drv_num*gt9xx_sen_num) *4;
	jitter_limit_temp = raw_buf + sizeof(u16)* (gt9xx_drv_num*gt9xx_sen_num) *5;

	GTP_INFO("Parsing configuration...");	
	ret = gtp_open_test_init(client);
	if (ret == FAIL) {
		GTP_ERROR("failed to parse config...");
		ret = FAIL;
		goto open_test_exit;
	}
	GTP_INFO("Step 1: Send Rawdata Cmd");
	ret = gtp_raw_test_init();
	if (FAIL == ret) {
		GTP_ERROR("Allocate memory for open test failed!");
		ret = FAIL;
		goto open_test_exit;
	}
	ret = gt9_read_raw_cmd(client);
	if (ret == FAIL) {
		GTP_ERROR("Send Read Rawdata Cmd failed!");
		ret = FAIL;
		goto open_test_exit;
	}

	GTP_INFO("Step 2: Sample Rawdata");
	for (i = 0; i < GTP_OPEN_SAMPLE_NUM; ++i) {
		rslt_buf_idx = i;
		ret = gtp_read_rawdata(client, raw_buf);
		current_data_index=i;

		gtp_set_avrg_rawbuf(raw_buf, i);
		ret = gtp_raw_test_re(raw_buf, DEFAULT_TEST_ITEMS);
		/*jitter test start*/
		if((DEFAULT_TEST_ITEMS & _JITTER_CHECK)
			&& (current_data_index == (GTP_OPEN_SAMPLE_NUM -1))) {
			gtp_reset_guitar(client, 20);
			gtp_send_cfg(client);
			msleep(300);
			gt9_read_raw_cmd(client);
			gtp_read_diffdata(client);
			gtp_jitter_test(client, DEFAULT_TEST_ITEMS);
		}
		/*jitter test end*/
#if GTP_SAVE_TEST_DATA
		GTP_INFO("GTP:Save_testing_data---in");
		ret1= Save_testing_data(ito_save_dir, DEFAULT_TEST_ITEMS, raw_buf);
		if (ret1 == FAIL) {
			GTP_ERROR("save testing data failed!");
		}
		if(current_data_index==15) {
			GTP_INFO(" Save rawdata %d frame",current_data_index);
			Save_test_result_data(ito_save_dir, DEFAULT_TEST_ITEMS);
		}
#endif 
		if (ret == FAIL) {
			gtp_i2c_end_cmd(client);
			continue;
		}
	}

open_test_exit:
	GTP_INFO("---gtp open test end---");
	gtp_opentest_arch_raw();
	gtp_arch_file_append_no_len("\n");
	if (raw_buf) {
		kfree(raw_buf);
		raw_buf = NULL;
		max_limit_vale_re = NULL;
		min_limit_vale_re = NULL;
		accord_limit_vale_re = NULL;
		accord_limit_temp = NULL;
		jitter_limit_temp = NULL;
	}
	if (test_rslt_buf) {
		kfree(test_rslt_buf);
		test_rslt_buf = NULL;
	}
	if (touchpad_sum) {
		kfree(touchpad_sum);
		touchpad_sum = NULL;
	}
	if (avrg_raw_buf) {
		kfree(avrg_raw_buf);
		avrg_raw_buf = NULL;
	}

	clear_bit(RAW_DATA_MODE, &ts->flags);
	gt9_read_coor_cmd(client);
	gtp_reset_guitar(client, 20);

	return ret;
}
/*
===================================================
* Function:
*		test gt9 series ic jitter test
* Input:
*		client, i2c_client
* Return:
*		SUCCESS: test process success, FAIL, test process failed
*
===================================================
*/
s32 gtp_jitter_test(struct i2c_client * client, u32 check_types)
{
	s32 result = SUCCESS;
	u16 over_count = 0;
	u16 i = 0;

	if (check_types & _JITTER_CHECK) {
		for (i = 0; i < (gt9xx_drv_num*gt9xx_sen_num); i++) {
			if(jitter_limit_temp[i] > jitter_limit_vale_id0[i]){
				over_count ++;
			}
			if(over_count > 1) {
				result = FAIL;
				break;
			}
		}

		if (result == SUCCESS) {
			Ito_result_info[JITTER_TEST_ID].testitem=JITTER_TEST_ID;
			Ito_result_info[JITTER_TEST_ID].result='P';
			GTP_INFO("gtp_jitter_test test result = pass\n");
		} else {
			Ito_result_info[JITTER_TEST_ID].testitem=JITTER_TEST_ID;
			Ito_result_info[JITTER_TEST_ID].result='F';
			test_error_code|=0x10;
			GTP_INFO("gtp_jitter_test test result = failed\n");
		}
	}
	return result;
}

/*for save test data*/

FILE *fopen(const char *path, const char *mode)
{
	FILE *filp = NULL;

	if (!strcmp(mode, "a+")) {
		if(file_pos == 0) {
			filp = filp_open(path, O_RDWR | O_CREAT, 0666);
		} else {
			filp = filp_open(path, O_RDWR | O_CREAT, 0666);
		}
		if (!IS_ERR(filp)) {
			filp->f_op->llseek(filp, 0, SEEK_END);
		}

		if(filp == NULL) {
			pr_err("open file as a+ mode filp == NULL 1\n");
		}

	} else if (!strcmp(mode, "r")) {
		filp = filp_open(path, O_RDONLY, 0666);
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	if (filp == NULL) {
		pr_err("open file as a+ mode filp == NULL 2\n");
	}
	return filp;
}

int fclose(FILE * filp)
{
	filp_close(filp, NULL);

	filp = NULL;

	set_fs(old_fs);

	return 0;
}
size_t fread(void *buffer, size_t size, size_t count, FILE * filp)
{
	return filp->f_op->read(filp, (char *)buffer, count, &filp->f_pos);
}

size_t fwrite(const void *buffer, size_t size, size_t count, FILE * filp)
{
	ssize_t  writeCount = -1;
	writeCount = vfs_write(filp, (char *)buffer, size, &file_pos);
	return writeCount;
}

s32 Save_testing_data(char *save_test_data_dir, int test_types,u16 *current_rawdata_temp)
{
	FILE *fp = NULL;
	s32 ret;
	s32 tmp = 0;
	u8 *data = NULL;
	s32 i = 0, j=0;
	s32 bytes = 0;
	int max, min;
	int average;
	data = (u8 *) malloc(_GT9_MAX_BUFFER_SIZE);

	GTP_INFO("Save_testing_data---in");
	if (NULL == data) {
		GTP_ERROR(" memory error!");
		GTP_INFO(" memory error!");
		return MEMORY_ERR;
	}
	for(i=0;i<_GT9_MAX_BUFFER_SIZE;i++) {
		data[i]=0;
	}

	sprintf((char *)itosavepath, "%sITO_Test_Data_%02d.csv",
							save_test_data_dir, ITO_TEST_COUNT);
	fp = fopen(itosavepath, "a+");
	if (NULL == fp) {
		GTP_ERROR("open %s failed!", itosavepath);
		free(data);
		return FILE_OPEN_CREATE_ERR;
	}

	if (current_data_index == 0) {
		bytes = (s32) sprintf((char *)data, "Device Type:%s\n", "GT917D");
		bytes += (s32) sprintf((char *)&data[bytes], "Config:\n");
		for (i = 0; i < tset_cfg_info_len[g_sensor_id]; i++) {
			bytes += (s32) sprintf((char *)&data[bytes], "0x%02X,",
						send_test_cfg_buf[g_sensor_id][i]);
		}

		bytes += (s32) sprintf((char *)&data[bytes], "\n");
		ret = fwrite(data, bytes, 1, fp);
		bytes = 0;
		if (ret < 0) {
			GTP_ERROR("write to file fail.");
			goto exit_save_testing_data;
		}

		if ((test_types & _MAX_CHECK) != 0) {
			bytes = (s32) sprintf((char *)data, "Channel maximum:\n");
			for (i = 0; i < gt9xx_sen_num; i++) {
				for (j = 0; j < gt9xx_drv_num; j++) {
					if(g_sensor_id==0) {
						bytes += (s32) sprintf((char *)&data[bytes], "%d,",
								max_limit_vale_id0[i + j * gt9xx_sen_num]);
					}
					if(g_sensor_id==2) {
						bytes += (s32) sprintf((char *)&data[bytes], "%d,",
								max_limit_vale_id2[i + j * gt9xx_sen_num]);
					}
				}

				bytes += (s32) sprintf((char *)&data[bytes], "\n");
				ret = fwrite(data, bytes, 1, fp);
				bytes = 0;
				if (ret < 0) {
					GTP_ERROR("write to file fail.");
					goto exit_save_testing_data;
				}
			}
		}

		if ((test_types & _MIN_CHECK) != 0) {
			bytes = (s32) sprintf((char *)data, "\nChannel minimum:\n");
			for (i = 0; i < gt9xx_sen_num; i++) {
				for (j = 0; j < gt9xx_drv_num; j++) {
					if(g_sensor_id==0) {
						bytes += (s32) sprintf((char *)&data[bytes], "%d,",
							min_limit_vale_id0[i + j * gt9xx_sen_num]);
					}
					if(g_sensor_id==2) {
						bytes += (s32) sprintf((char *)&data[bytes], "%d,",
							min_limit_vale_id2[i + j * gt9xx_sen_num]);
					}
				}

				bytes += (s32) sprintf((char *)&data[bytes], "\n");
				ret = fwrite(data, bytes, 1, fp);
				bytes = 0;
				if (ret < 0) {
					GTP_ERROR("write to file fail.");
					goto exit_save_testing_data;
				}
			}
		}

		if ((test_types & _ACCORD_CHECK) != 0) {
			bytes = (s32) sprintf((char *)data, "\nChannel average:(%d)\n", FLOAT_AMPLIFIER);
			for (i = 0; i < gt9xx_sen_num; i++) {
				for (j = 0; j < gt9xx_drv_num; j++) {
					if(g_sensor_id==0) {
						bytes += (s32) sprintf((char *)&data[bytes], "%d,",
							accord_limit_vale_id0[i + j * gt9xx_sen_num]);
					}
					if(g_sensor_id==2) {
						bytes += (s32) sprintf((char *)&data[bytes], "%d,",
							accord_limit_vale_id2[i + j * gt9xx_sen_num]);
					}
				}

				bytes += (s32) sprintf((char *)&data[bytes], "\n");
				ret = fwrite(data, bytes, 1, fp);
				bytes = 0;
				if (ret < 0) {
					GTP_ERROR("write to file fail.");
					goto exit_save_testing_data;
				}
			}
			bytes = (s32) sprintf((char *)data, "\n");
			ret = fwrite(data, bytes, 1, fp);
			if (ret < 0) {
				GTP_ERROR("write to file fail.");
				goto exit_save_testing_data;
			}
		}

		bytes = (s32) sprintf((char *)data, " Rawdata\n");
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			GTP_ERROR("write to file fail.");
			goto exit_save_testing_data;
		}
	}

	bytes = (s32) sprintf((char *)data, "No.%d\n", current_data_index);
	ret = fwrite(data, bytes, 1, fp);
	if (ret < 0) {
		GTP_ERROR("write to file fail.");
		goto exit_save_testing_data;
	}

	max=1000;
	min=5000;
	average = 0;

	for (i = 0; i < gt9xx_sen_num; i++) {
		bytes = 0;
		for (j = 0; j <gt9xx_drv_num; j++) {
			tmp = current_rawdata_temp[i + j * gt9xx_sen_num];
			bytes += (s32) sprintf((char *)&data[bytes], "%d,", tmp);
			if (tmp > max) {
				max = tmp;
			}
			if (tmp < min) {
				min = tmp;
			}
			average += tmp;
		}
		bytes += (s32) sprintf((char *)&data[bytes], "\n");
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			GTP_ERROR("write to file fail.");
			goto exit_save_testing_data;
		}
	}
	average = average / (gt9xx_drv_num* gt9xx_sen_num);

	bytes = (s32) sprintf((char *)data, "  Maximum:%d  Minimum:%d  Average:%d\n\n",
						max, min, average);
	ret = fwrite(data, bytes, 1, fp);
	if (ret < 0) {
		GTP_ERROR("write to file fail.");
		goto exit_save_testing_data;
	}

	if ((test_types & _ACCORD_CHECK) != 0) {
		bytes = (s32) sprintf((char *)data, "Channel_Accord :(%d)\n", FLOAT_AMPLIFIER);
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			GTP_ERROR("write to file fail.");
			goto exit_save_testing_data;
		}
		for (i = 0; i < gt9xx_sen_num; i++) {
			bytes = 0;
			for (j = 0; j < gt9xx_drv_num; j++) {
				bytes += (s32) sprintf((char *)&data[bytes], "%d.%d%d%d,",
					(accord_limit_temp[i + j * gt9xx_sen_num])/1000,
					((accord_limit_temp[i + j * gt9xx_sen_num])%1000)/100,
					((accord_limit_temp[i + j * gt9xx_sen_num])%100)/10,
					(accord_limit_temp[i + j * gt9xx_sen_num])%10);

			}
			bytes += (s32) sprintf((char *)&data[bytes], "\n");
			ret = fwrite(data, bytes, 1, fp);
			if (ret < 0) {
				GTP_ERROR("write to file fail.");
				goto exit_save_testing_data;
			}
		}

		bytes = (s32) sprintf((char *)data, "\n");
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			GTP_ERROR("write to file fail.");
			goto exit_save_testing_data;
		}
	}
	if ((current_data_index == 15) && ((test_types & _JITTER_CHECK) != 0)) {
		bytes = (s32) sprintf((char *)data, "Diff data :\n");
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			GTP_ERROR("write to file fail.");
			goto exit_save_testing_data;
		}
		for (i = 0; i < gt9xx_sen_num; i++) {
			bytes = 0;
			for (j = 0; j < gt9xx_drv_num; j++) {
				bytes += (s32) sprintf((char *)&data[bytes], "%d,",
					jitter_limit_temp[i + j * gt9xx_sen_num]);
			}
			bytes += (s32) sprintf((char *)&data[bytes], "\n");
			ret = fwrite(data, bytes, 1, fp);
			if (ret < 0) {
				GTP_ERROR("write to file fail.");
				goto exit_save_testing_data;
			}
		}
	}

exit_save_testing_data:
	free(data);
	fclose(fp);
	return ret;
}

s32 Save_test_result_data(char *save_test_data_dir, int test_types)
{
	FILE *fp = NULL;
	s32 ret;
	u8 *data = NULL;
	s32 bytes = 0;

	data = (u8 *) malloc(_GT9_MAX_BUFFER_SIZE);
	if (NULL == data) {
		GTP_ERROR("memory error!");
		return MEMORY_ERR;
	}
	GTP_ERROR("before fopen path = %s\n",save_test_data_dir);
	sprintf((char *)itosavepath, "%sITO_Test_Data_%02d.csv", save_test_data_dir, ITO_TEST_COUNT);
	fp = fopen(itosavepath, "a+");
	if (NULL == fp) {
		GTP_ERROR("open %s failed!", itosavepath);
		free(data);
		return FILE_OPEN_CREATE_ERR;
	}

	bytes = (s32) sprintf((char *)data, "Test Result:");
	if (test_error_code == _CHANNEL_PASS) {
		bytes += (s32) sprintf((char *)&data[bytes], "Pass\n\n");
	} else {
		bytes += (s32) sprintf((char *)&data[bytes], "Fail\n\n");
	}
	bytes += (s32) sprintf((char *)&data[bytes], "Test items:\n");
	if ((test_types & _MAX_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Max Rawdata:  ");
		if (test_error_code & _BEYOND_MAX_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if ((test_types & _MIN_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Min Rawdata:  ");
		if (test_error_code & _BEYOND_MIN_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if ((test_types & _ACCORD_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Area Accord:  ");
		if (test_error_code & _BETWEEN_ACCORD_AND_LINE) {
			bytes += (s32) sprintf((char *)&data[bytes], "Fuzzy !\n");
		} else {
			if (test_error_code & _BEYOND_ACCORD_LIMIT) {
				bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
			} else {
				bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
			}
		}
	}

	if ((test_types & _SHORT_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Moudle Short Test:  ");
		if (test_error_code & _SENSOR_SHORT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if ((test_types & _OFFSET_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Max Offest:  ");
		if (test_error_code & _BEYOND_OFFSET_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if ((test_types & _JITTER_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Max Jitier:  ");
		if (test_error_code & _BEYOND_JITTER_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if (test_types & _UNIFORMITY_CHECK) {
		bytes += (s32) sprintf((char *)&data[bytes], "Uniformity:  ");
		if (test_error_code & _BEYOND_UNIFORMITY_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if ((test_types & _KEY_MAX_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Key Max Rawdata:  ");
		if (test_error_code & _KEY_BEYOND_MAX_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if ((test_types & _KEY_MIN_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Key Min Rawdata:  ");
		if (test_error_code & _KEY_BEYOND_MIN_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if (test_types & (_VER_EQU_CHECK | _VER_GREATER_CHECK | _VER_BETWEEN_CHECK)) {
		bytes += (s32) sprintf((char *)&data[bytes], "Device Version:  ");
		if (test_error_code & _VERSION_ERR) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if (test_types & _MODULE_TYPE_CHECK) {
		bytes += (s32) sprintf((char *)&data[bytes], "Module Type:  ");
		if (test_error_code & _MODULE_TYPE_ERR) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	ret = fwrite(data, bytes, 1, fp);

	if (ret < 0) {
		GTP_ERROR("write to file fail.");
		free(data);
		fclose(fp);
		return ret;
	}

	free(data);
	fclose(fp);
	return 1;
}

/*end for save test data*/

static ssize_t ito_test_show(struct device *dev, 
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int buf_size = 0;
	int i=0;
	struct goodix_ts_data *ts = NULL;

	ts = i2c_get_clientdata(i2c_connect_client);
	gtp_work_control_enable(ts, false);
	if (test_and_set_bit(ITO_TEST_RUNNING, &ts->flags)) {
		GTP_ERROR("ito_test may already running\n");
		return ret;
	}
	gtp_esd_off(ts);
	msleep(20);

	Ito_result_info = (struct gt9xx_iot_result_info*)\
		kmalloc(sizeof(struct gt9xx_iot_result_info) * ITO_TEST_ITEM_NUM, GFP_KERNEL);
	if (Ito_result_info == NULL) {
		GTP_ERROR("%s : memory alloc Ito_result_info failed\n", __func__);
		return -EFAULT;
	}
	gtp_config = (u8*)kmalloc(sizeof(u8*) * GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH, GFP_KERNEL);
	if (gtp_config == NULL) {
		GTP_ERROR("%s : memory alloc gtp_config failed\n", __func__);
		kfree(Ito_result_info);
		return -EFAULT;
	}
	memset(gtp_config, 0, sizeof(u8*) * GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
	memset(Ito_result_info, 0, sizeof(struct gt9xx_iot_result_info) * ITO_TEST_ITEM_NUM);

	GTP_INFO("gtp_ito_test_show enter!!\n");
	for (i=0;i<ITO_TEST_ITEM_NUM-1;i++) {
		Ito_result_info[i].stringline='-';
	}

	test_error_code=0;
	if (test_bit(SLEEP_MODE, &ts->flags)) {
		GTP_ERROR("tp is in sleep mode.\n");
		buf_size += snprintf(buf + buf_size, 30, "IO-IDLE\t\n");
		goto exit_ito_test;
	}
	ret = gt9xx_parse_cfg();
	if (ret == FAIL) {
		GTP_ERROR("parse cfg error.\n");
		buf_size +=snprintf(buf+buf_size, 30, "IO-error\t\n");
		goto exit_ito_test;
	}
	ret = gt9xx_short_test(i2c_connect_client);

	if (ret) {
		Ito_result_info[SHORT_TEST_ID].testitem=SHORT_TEST_ID;
		Ito_result_info[SHORT_TEST_ID].result='P';
		GTP_INFO("gtp_ito_test_show short test result = pass \n");
	} else {
		Ito_result_info[SHORT_TEST_ID].testitem=SHORT_TEST_ID;
		Ito_result_info[SHORT_TEST_ID].result='F';
		test_error_code|=0x20;
		GTP_INFO("gtp_ito_test_show short test result = failed \n");
	}

	ret = gt9xx_open_test(i2c_connect_client);

	for (i=0;i<ITO_TEST_ITEM_NUM;i++) {
		GTP_INFO("%d%c",Ito_result_info[i].testitem,Ito_result_info[i].result);
		buf_size +=snprintf(buf+buf_size, 10
			, "%d%c", Ito_result_info[i].testitem, Ito_result_info[i].result);
		if(i < ITO_TEST_ITEM_NUM-1) {
			GTP_INFO("%c",Ito_result_info[i].stringline);
			buf_size +=snprintf(buf+buf_size, 5, "%c", Ito_result_info[i].stringline);
		}
	}
	buf_size +=snprintf(buf+buf_size, 5, "\t\n");
	GTP_INFO("\ngtp_ito_test_show end!!\n");

exit_ito_test:

	if (Ito_result_info) {
		kfree(Ito_result_info);
		Ito_result_info = NULL;
	}
	if (gtp_config) {
		kfree(gtp_config);
		gtp_config = NULL;
	}

	gtp_esd_on(ts);
	clear_bit(ITO_TEST_RUNNING, &ts->flags);
	gtp_work_control_enable(ts, true);

	return buf_size;
}

static DEVICE_ATTR(gtp_ito_test, (S_IRUGO | S_IWUSR | S_IWGRP), ito_test_show, NULL);

static struct attribute *prd_attribute_list[] = {
	&dev_attr_gtp_ito_test.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

s32 gt9xx_prd_register_sysfs(struct i2c_client *client)
{
	s32 ret = 0;

	ret = sysfs_create_group(&client->dev.kobj, &prd_attribute_group);
	if (ret) {
		GTP_ERROR("Failure create prd sysfs group %d\n", ret);
		/*TODO: debug change */
		return -ENODEV;
	}
	return 0;
}
