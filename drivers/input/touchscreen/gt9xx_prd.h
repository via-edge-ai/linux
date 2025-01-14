/* production_test.h
*
* Copyright (C) 2015 LGE.
*
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/
#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>

/*
*  Include to Local Header File
*/
#include "gt9xx.h"

/*short test*/

#define MYBIG_ENDIAN				1
#define MYLITLE_ENDIAN				0
#define GTP_SHORT_GND
#define GTP_VDD					33
#define MAX_DRIVER_NUM				42
#define MAX_SENSOR_NUM				30

/*ito test items*/
#define DEFAULT_TEST_ITEMS  (_SHORT_CHECK | _MAX_CHECK\
	| _MIN_CHECK | _ACCORD_CHECK/* | _JITTER_CHECK*/)
/*for print test result info, cat gtp_ito_test: 0P-1P-2P-3P-4P */
#define ITO_TEST_ITEM_NUM			4
#define SHORT_TEST_ID				0
#define RAWDATA_MAXDATA_ID			1
#define RAWDATA_MINDATA_ID			2
#define RAWDATA_ACCORD_ID			3
#define JITTER_TEST_ID				4

/*short test threshold*/
unsigned short gt900_short_threshold = 10;
unsigned short gt900_resistor_threshold = 800;
unsigned short gt900_resistor_warn_threshold = 500;
unsigned short gt900_gnd_resistor_threshold = 500;
/*end short test threshold*/
#define DSP_SHORT_BURN_CHK		256		/*burn short chuck size*/
#define _SHORT_INFO_MAX			50		/*short test max show 50 pairs short channels*/
#define _BEYOND_INFO_MAX		20		/*pen test max show 20 infos for each test item*/
#define GTP_OPEN_SAMPLE_NUM		16		/*open test raw data sampled count*/
#define GTP_TEST_INFO_MAX		200		/*test info lines max count*/
#define GTP_SAVE_TEST_DATA		1
#define _GT9_MAX_BUFFER_SIZE	2048
#define FLOAT_AMPLIFIER			1000
#define GTP_WAIT_RAW_MAX_TIMES	200
#define TEST_RSLT_ARCHIVE_PATH 	"/data/gtp_test_rslt.txt"

/*TODO: define your own default or for Sensor_ID == 0 config here.*/

#define CTP_TEST_CFG_GROUP0 {\
0x00,0x38,0x04,0x80,0x07,0x0A,0x3C,0x10,0x01,0x0F,\
0x28,0x0F,0x5A,0x38,0x05,0xF2,0x00,0x00,0x60,0x62,\
0x12,0x33,0x04,0x16,0x18,0x1D,0x14,0x87,0x26,0xBB,\
0x39,0x3B,0xD3,0x07,0x7C,0x06,0x00,0xC3,0x33,0x91,\
0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x0A,0x0C,0x00,\
0x1A,0x28,0x5B,0x94,0xD0,0x63,0x08,0x00,0x00,0x04,\
0x92,0x2B,0x00,0x80,0x33,0x00,0x71,0x3C,0x00,0x64,\
0x47,0x00,0x5A,0x54,0x00,0x5A,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x03,0x00,\
0x00,0x00,0x00,0x00,0x17,0x02,0x00,0x00,0x0F,0x28,\
0x00,0x00,0x00,0x00,0x00,0x06,0x50,0x00,0x00,0x00,\
0x00,0x37,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,\
0x13,0x14,0x15,0x0A,0x09,0x08,0x07,0x06,0x05,0x04,\
0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xFF,0xFF,0x28,0x24,0x22,0x20,0x1F,0x1E,0x0A,0x05,\
0x04,0x02,0x00,0x06,0x08,0xFF,0xFF,0xFF,0xFF,0x78,\
0x31,0x00,0x00,0x00,0x87,0x70,0x12,0x00,0x28,0x4E,\
0x5A,0x14,0x1E,0x00,0x00,0x3C,0x08,0x88,0x88,0x00,\
0x82,0x86,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,0x00,\
0x00,0x00,0x00,0x23,0x50,0x00,0x78,0x78,0x33,0x0A,\
0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0xD6,0x01\
}

/*TODO: define your config for Sensor_ID == 1 here, if needed*/
#define CTP_TEST_CFG_GROUP1 {\
}

/*TODO: define your config for Sensor_ID == 2 here, if needed*/
#define CTP_TEST_CFG_GROUP2 {\
}

/*TODO: define your config for Sensor_ID == 3 here, if needed*/
#define CTP_TEST_CFG_GROUP3 {\
}

/*TODO: define your config for Sensor_ID == 4 here, if needed*/
#define CTP_TEST_CFG_GROUP4 {\
}

/*TODO: define your config for Sensor_ID == 5 here, if needed*/
#define CTP_TEST_CFG_GROUP5 {\
}

/**********rawdata test threshold for each sensor ID*********/
/************************sensor_id is 0**********************/
const u16 max_limit_vale_id0[]={\
6748,6760,6675,6648,6586,6591,6507,6393,6361,6259,5839,6822,6859,
6774,6841,6823,6889,6847,6864,6807,6859,6517,6918,6937,6852,6831,
6760,6769,6679,6571,6523,6427,6007,6571,6742,6666,6726,6718,6763,
6738,6738,6705,6753,6396,6918,6954,6870,6849,6784,6795,6700,6597,
6555,6459,6043,6484,6654,6579,6637,6633,6669,6637,6639,6597,6648,
6306,6910,6948,6868,6849,6775,6792,6696,6589,6553,6460,6036,6379,
6555,6475,6534,6526,6553,6528,6526,6487,6523,6186,7041,7101,7006,
6985,6904,6928,6826,6718,6670,6600,6151,6418,6589,6508,6558,6549,
6582,6550,6547,6502,6549,6190,7038,7089,7009,6999,6927,6943,6852,
6750,6696,6612,6177,6360,6526,6438,6486,6463,6498,6474,6463,6429,
6469,6108,6927,6994,6910,6912,6844,6870,6792,6691,6648,6564,6148,
6222,6385,6295,6349,6328,6360,6333,6324,6286,6333,5970,6960,7027,
6963,6949,6886,6925,6832,6747,6700,6621,6202,6180,6339,6264,6301,
6288,6318,6280,6276,6243,6285,5917,6960,7051,6979,6967,6907,6957,
6864,6768,6727,6643,6271,6130,6282,6201,6256,6229,6256,6222,6219,
6183,6198,5847,6991,7078,7015,7011,6954,7006,6918,6820,6786,6705,
6340,6109,6253,6166,6222,6198,6228,6190,6192,6154,6171,5829,7116,
7188,7135,7149,7099,7149,7068,6979,6948,6880,6481,6165,6322,6232,
6274,6246,6282,6253,6253,6211,6256,5952,7125,7200,7147,7173,7128,
7185,7104,7027,6994,6928,6544,6120,6276,6181,6223,6202,6232,6196,
6202,6159,6192,5824,6927,7032,6991,6999,6970,7015,6948,6876,6859,
6847,6495,5877,6007,5913,5944,5919,5941,5904,5967,5925,5923,5578
};

const u16 min_limit_vale_id0[]={\
1875,1894,1865,1844,1828,1817,1795,1784,1774,1750,1646,1871,1894,\
1872,1891,1888,1885,1875,1916,1903,1903,1811,1920,1947,1915,1894,\
1876,1867,1842,1833,1821,1800,1695,1828,1874,1852,1871,1868,1863,\
1857,1898,1887,1882,1783,1926,1951,1922,1902,1884,1877,1851,1843,\
1830,1808,1703,1807,1851,1831,1848,1846,1838,1830,1870,1860,1857,\
1756,1925,1943,1916,1900,1880,1875,1849,1839,1827,1806,1702,1780,\
1822,1801,1819,1815,1807,1800,1837,1826,1822,1722,1952,1986,1957,\
1939,1920,1905,1879,1879,1866,1846,1740,1785,1835,1816,1830,1827,\
1818,1810,1840,1830,1834,1734,1949,1982,1954,1936,1920,1906,1881,\
1881,1869,1846,1744,1762,1812,1791,1804,1800,1791,1781,1812,1801,\
1806,1707,1898,1942,1922,1929,1918,1891,1866,1873,1863,1833,1713,\
1703,1775,1755,1761,1756,1749,1743,1754,1744,1768,1671,1905,1950,\
1932,1940,1930,1903,1880,1890,1879,1851,1729,1696,1766,1746,1751,\
1746,1740,1732,1743,1732,1758,1659,1901,1952,1933,1941,1933,1909,\
1887,1892,1885,1858,1738,1681,1748,1728,1737,1729,1723,1717,1728,\
1714,1739,1644,1906,1961,1944,1952,1947,1923,1900,1908,1901,1875,\
1756,1675,1742,1720,1729,1723,1716,1710,1720,1707,1731,1637,1945,\
1998,1985,1997,1985,1961,1938,1957,1952,1924,1807,1696,1766,1744,\
1750,1744,1739,1730,1741,1726,1751,1671,1942,1989,1974,1987,1981,\
1971,1950,1956,1950,1928,1809,1669,1738,1717,1722,1716,1711,1701,\
1711,1699,1728,1633,1900,1957,1951,1966,1950,1930,1906,1929,1926,\
1900,1812,1626,1686,1657,1677,1669,1665,1653,1664,1651,1659,1559,\
};

const u16 accord_limit_vale_id0[]={\
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
330,300,300,300,300,300,300,300,300,300,300,300,320,
320,300,300,300,300,300,300,300,300,300,300,300,320,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
300,300,300,300,300,300,300,300,300,300,300,300,300,
};
const u16 jitter_limit_vale_id0[]={\
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,
};
/************************sensor_id is 1****************/
u16 max_limit_vale_id1[]={\
};

u16 min_limit_vale_id1[]={\
};

u16 accord_limit_vale_id1[]={\
};

u16 jitter_limit_vale_id1[]={\
};

/************************sensor_id is 2****************/
const u16 max_limit_vale_id2[]={\
};
const u16 min_limit_vale_id2[]={\
};
//1000*accord
const u16 accord_limit_vale_id2[]={\
};

const u16 jitter_limit_vale_id2[]={\
};

/************************sensor_id is 3***************/
const u16 max_limit_vale_id3[]={\
};

const u16 min_limit_vale_id3[]={\
};

const u16 accord_limit_vale_id3[]={\
};

const u16 jitter_limit_vale_id3[]={\
};

/************************sensor_id is 4***************/
const u16 max_limit_vale_id4[]={\
};

const u16 min_limit_vale_id4[]={\
};

const u16 accord_limit_vale_id4[]={\
};

const u16 jitter_limit_vale_id4[]={\
};

/************************sensor_id is 5**************/
const u16 max_limit_vale_id5[]={\
};

const u16 min_limit_vale_id5[]={\
};

const u16 accord_limit_vale_id5[]={\
};

const u16 jitter_limit_vale_id5[]={\
};
/*end rawdata test threshold for each sensor ID*/

/*test options (check_types)*/
#define _MAX_CHECK						0x0001
#define _MIN_CHECK						0x0002
#define _ACCORD_CHECK					0x0004
#define _OFFSET_CHECK					0x0008
#define _JITTER_CHECK					0x0010
#define _SHORT_CHECK					0x0020
#define _KEY_MAX_CHECK					0x0040
#define _KEY_MIN_CHECK					0x0080
#define _IC_RESET_CHECK					0x0100
#define _VER_EQU_CHECK					0x0200
#define _VER_GREATER_CHECK				0x0400
#define _VER_BETWEEN_CHECK				0x0800
#define _MAX_CURRENT_CHECK				0x1000
#define _MODULE_TYPE_CHECK				0x2000
#define _UNIFORMITY_CHECK				0x4000

/****************************************************/

/*-------------------error_type---------------------*/

#define _CHANNEL_PASS					0x0000
#define _BEYOND_MAX_LIMIT				0x0001
#define _BEYOND_MIN_LIMIT				0x0002
#define _BEYOND_ACCORD_LIMIT			0x0004
#define _BEYOND_OFFSET_LIMIT			0x0008
#define _BEYOND_JITTER_LIMIT			0x0010
#define _SENSOR_SHORT					0x0020
#define _DRIVER_SHORT					0x0040
#define _KEY_BEYOND_MAX_LIMIT			0x0080
#define _KEY_BEYOND_MIN_LIMIT			0x0100
#define _VERSION_ERR					0x0200
#define _MODULE_TYPE_ERR				0x0400
#define _BEYOND_UNIFORMITY_LIMIT		0x0800
#define _BETWEEN_ACCORD_AND_LINE		0x1000
#define FILE_OPEN_CREATE_ERR			0x2000
#define MEMORY_ERR						0x4000

/*----------------- SHORT TEST PART-----------------*/
#define _bRW_MISCTL__SRAM_BANK			0x4048
#define _bRW_MISCTL__MEM_CD_EN			0x4049
#define _bRW_MISCTL__CACHE_EN			0x404b
#define _bRW_MISCTL__TMR0_EN			0x40b0
#define _rRW_MISCTL__SWRST_B0_			0x4180
#define _bWO_MISCTL__CPU_SWRST_PULSE	0x4184
#define _rRW_MISCTL__BOOTCTL_B0_		0x4190
#define _rRW_MISCTL__BOOT_OPT_B0_		0x4218
#define _bRW_MISCTL__RG_OSC_CALIB		0x4268
#define _rRW_MISCTL__BOOT_CTL_			0x5094
#define _rRW_MISCTL__SHORT_BOOT_FLAG	0x5095

#define GTP_REG_DSP_SHORT				0xc000
#define GTP_REG_READ_RAW				0x8040
#define GTP_REG_RAW_READY				0x814E
#define GTP_REG_RAW_DATA_GT9F			0x87C0
#define GT9_REG_SEN_DRV_CNT				0x8062
#define GT9_REG_CFG_BEG					0x8047
#define GT9_REG_KEY_VAL					0x8093
#define GT9_REG_SEN_ORD					0x80B7
#define GT9_REG_DRV_ORD					0x80D5

#define GT9_DRV_HEAD					0x80
#define GT9_SEN_HEAD					0x00
/**************T9XX RAW_DATA MAPPING*********
*		|------FLASH-----RAM--------|		*
*		|------917D------0x9b60-----|		*
*		|------615-------0x8b98-----|		*
*		|------9293------0x8b98-----|		*
*		|------915L------0x8b98-----|		*
*		|------9110------0x8b98-----|		*
*		|------9110P-----0x8b98-----|		*
*		|------9271------0x8b98-----|		*
********************************************/
#define GTP_REG_RAW_DATA				0x9b60
#define GTP_REG_DIFF_DATA				0xa160

/*QFN88 package TX Pin, used for get channel number from Chip to Pad*/
const u8 ChannelPackage_TX[MAX_DRIVER_NUM] =  {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,
	20,21,22,23,24,25,/*26,*/27,28,29,30,31,32,33,34,35,36,37,
	38,39,40,41,42
};

typedef struct file FILE;
struct gt9xx_short_info
{
	u8 master;
	u8 master_is_driver;
	u8 slave;
	u8 slave_is_driver;
	u16 short_code;
	u16 self_data;
	u16 impedance;
};

struct gt9xx_open_info
{
	u8 driver;
	u8 sensor;
	u16 raw_val;
	s32 beyond_type;
	u8 times;
	u8 key;
};
struct gt9xx_iot_result_info
{
	u8 testitem;
	char result;
	char stringline;
};
struct gt9xx_short_fw {
	const u8 *fw_data;
	u32 fw_total_len;
	const struct firmware *fw;
};

#define FORMAT_PATH(path,mdir,name) do{\
							struct timex txc;\
							struct rtc_time tm;\
							do_gettimeofday(&(txc.time));\
							rtc_time_to_tm(txc.time.tv_sec,&tm);\
							sprintf((char*)path,"%s%s_%04d%02d%02d%02d%02d%02d.csv",\
							mdir,name,(tm.tm_year+1900),(tm.tm_mon + 1),tm.tm_mday,\
							tm.tm_hour,tm.tm_min,tm.tm_sec);\
						}while(0)
#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))
#define HEX(a) ((a >= '0' && a <= '9') || (a >= 'A' && a <= 'F') || (a >= 'a' && a <= 'f'))
#define GTP_DEBUG(fmt, arg...)			dev_dbg(&i2c_connect_client->dev, fmt"\n", ##arg)
#define GTP_INFO(fmt, arg...)			dev_info(&i2c_connect_client->dev, fmt"\n", ##arg)
#define GTP_ERROR(fmt, arg...)			dev_err(&i2c_connect_client->dev, fmt"\n", ##arg)

#define free(p) kfree(p)
#define malloc(len) kmalloc(len, GFP_KERNEL)

extern s32 gtp_send_cfg(struct i2c_client *client);
s32 gt9xx_prd_register_sysfs(struct i2c_client *client);
s32 Save_testing_data(char *save_test_data_dir,
					int test_types,u16 *current_rawdata_temp);
s32 Save_test_result_data(char *save_test_data_dir, int test_types);

#endif
