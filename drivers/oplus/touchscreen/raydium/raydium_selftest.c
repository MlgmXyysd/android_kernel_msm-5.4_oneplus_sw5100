/*
 * File:    raydium_selftest.c
 * Author:  Valentine <valentine.hsu@rad-ic.com>
 * Brief:   rm31080 touch screen test tool.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

#include "drv_interface.h"
#include "chip_raydium/ic_drv_interface.h"
#include "raydium_selftest.h"
#include "chip_raydium/ic_drv_global.h"
#include "tpselftest_30.h"
#if defined(FW_MAPPING_EN)
#include "tpselftest_21.h"
#endif

#define RM_SELF_TEST_CUSTOMER_VERSION	0x01
#define RM_SELF_TEST_PLATFORM_VERSION	0x01
#define RM_SELF_TEST_PROJECT_VERSION		0x40
#define RM_SELF_TEST_MAIN_VERSION		1
#define RM_SELF_TEST_SUB_VERSION		0

#define RESULT_SUCCESS 					0
#define RESULT_NG 					1

#define RELATIVE_PATH	 				0

unsigned char g_u8_normal_fw_version_buf[4];
char str_ini_path[100];
char self_test_result_buffer[1024*2] = {0};
ssize_t self_test_result_buffer_index = 0;

static int self_test_all(void)
{
	int ret = 0;

	g_u8_raydium_flag |= ENG_MODE;
	handle_ic_test();
	ret = g_u32_wearable_test_result;

	/*g_u8_raydium_flag &= ~ENG_MODE;*/
	DEBUGOUT("self_test_all end \r\n");

	return ret;
}

void self_test_result_buffer_init() {
	memset(self_test_result_buffer, 0, sizeof(self_test_result_buffer));
	self_test_result_buffer_index = 0;
}

void self_test_result_write_to_buffer(char *p_string, short len) {
	if ((sizeof(self_test_result_buffer) - self_test_result_buffer_index - 1) > len) {
		memcpy(&self_test_result_buffer[self_test_result_buffer_index], p_string, len);
		self_test_result_buffer_index += len;
	} else {
		DEBUGOUT("[touch] ERROR: self_test_result_buffer is not enough!\n");
	}
}

ssize_t self_test_result_read_from_buffer(char *p_string, ssize_t len) {
	int result_len = 0;
	if (self_test_result_buffer_index + 1 < len) {
		result_len = self_test_result_buffer_index;
	} else {
		result_len = len - 1;
	}
	memcpy(p_string, self_test_result_buffer, result_len);
	p_string[result_len] = '\0';
	return result_len + 1;
}

int self_test_save_to_file(char *file_name, char *p_string, short len)
{
	struct file *filp = NULL;
	mm_segment_t old_fs;

	self_test_result_write_to_buffer(p_string, len);
	return 0;
	filp = filp_open(file_name, O_RDWR | O_CREAT | O_APPEND, 0666);
	if (IS_ERR(filp)) {
		DEBUGOUT("can't open file:%s\n", RM_SELF_TEST_LOGFILE);
		return 0;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	filp->f_op->write(filp, p_string, len, &filp->f_pos);
	set_fs(old_fs);
	filp_close(filp, NULL);

	return 1;
}

#if 1
static int raydium_check_ini_version(void)
{
	int ret = 0;
	unsigned int u32_test_version;
	memcpy(&u32_test_version, &g_rad_testpara_image[4], 4);

	if (u32_test_version != g_st_test_para_resv.u32_test_fw_version) {
		DEBUGOUT("test fw versio 0x%X != ini version 0x%X\n"
			 , u32_test_version, g_st_test_para_resv.u32_test_fw_version);
		ret = WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG;
	}
	if (g_u16_dev_id == DEVICE_ID_3X) {

		g_u32_dongle_flash_ini_addr = F303_DONGLE_FLASH_INI_ADDR;
		g_u32_ini_threshold_addr = F303_INI_THRESHOLD_ADDR;
		g_u32_ini_para_addr = F303_INI_PARA_ADDR;
		g_u32_ini_raw_data_3_cc_addr = F303_INI_RAW_DATA_3_CC_ADDR;
		g_u32_ini_uc_cc_addr = F303_INI_UC_CC_ADDR;
		g_u32_initial_code_start_addr = F303_INITIAL_CODE_START_ADDR;
		DEBUGOUT("[out_set_ic_version] F303 Set INI ADDR!\r\n");
	} else if (g_u16_dev_id == DEVICE_ID_2X) {
		g_u32_dongle_flash_ini_addr = F302_DONGLE_FLASH_INI_ADDR;
		g_u32_ini_threshold_addr = F302_INI_THRESHOLD_ADDR;
		g_u32_ini_para_addr = F302_INI_PARA_ADDR;
		g_u32_ini_raw_data_3_cc_addr = F302_INI_RAW_DATA_3_CC_ADDR;
		g_u32_ini_uc_cc_addr = F302_INI_UC_CC_ADDR;
		g_u32_initial_code_start_addr = F302_INITIAL_CODE_START_ADDR;
		DEBUGOUT("[out_set_ic_version] F302 Set INI ADDR!\r\n");
	}
	return ret;
}
#else
static int raydium_check_ini_version(void)
{
	int ret = 0;
	unsigned int u32_test_version, u32_version_20, u32_version_21;
	memcpy(&u32_test_version, &g_st_test_para_resv.u32_test_fw_version, 4);

	if (g_u16_dev_id == DEVICE_ID_2X) {
		switch (g_raydium_ts->id) {
		case RAD_SELFTEST_20:
			memcpy(&u32_version_20, &u8_rad_testpara_20[4], 4);
			DEBUGOUT("ini version 0x%X, 20 version 0x%X\n"
				 , u32_test_version, u32_version_20);

			if (u32_test_version == u32_version_20) {
				DEBUGOUT("map version= 0x%X\r\n", u32_version_20);
			}  else
				ret = WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG;
		case RAD_SELFTEST_21:
			memcpy(&u32_version_21, &u8_rad_testpara_21[4], 4);
			DEBUGOUT("ini version 0x%X, 21 version 0x%X\n"
				 , u32_test_version, u32_version_21);

			if (u32_test_version == u32_version_21) {
				DEBUGOUT("map version= 0x%X\r\n", u32_version_21);
			}  else
				ret = WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG;
		}
	}

	return ret;
}
#endif
static int self_test_init(void)
{
	int ret = 0;
	unsigned int u32_dev_id;

	self_test_result_buffer_init();

	g_u8_drv_interface = I2C_INTERFACE;
	g_u16_dev_id = DEVICE_ID_3X;

	if (handle_ic_read(RAYDIUM_CHK_I2C_CMD, 4, (unsigned char *)&u32_dev_id, g_u8_drv_interface, I2C_WORD_MODE) == ERROR) {
		ret = WEARABLE_FT_TEST_RESULT_SYSFS_NG;
		return ret;
	}
	g_u16_dev_id = ((u32_dev_id & 0xFFFF0000) >> 16);

	if (g_u16_dev_id == DEVICE_ID_2X) {
		handle_ic_read(0x00006a04, 4, g_u8_normal_fw_version_buf, g_u8_drv_interface, I2C_WORD_MODE);
		DEBUGOUT("FW Version=0x%.2X%.2X%.2X%.2X\n", g_u8_normal_fw_version_buf[0], g_u8_normal_fw_version_buf[1],
			 g_u8_normal_fw_version_buf[3], g_u8_normal_fw_version_buf[2]);
	} else if (g_u16_dev_id == DEVICE_ID_3X) {
		handle_ic_read(0x00007b04, 4, g_u8_normal_fw_version_buf, g_u8_drv_interface, I2C_WORD_MODE);
		DEBUGOUT("FW Version=0x%.2X%.2X%.2X%.2X\n", g_u8_normal_fw_version_buf[0], g_u8_normal_fw_version_buf[1],
			 g_u8_normal_fw_version_buf[3], g_u8_normal_fw_version_buf[2]);
	} else {
		DEBUGOUT("read ic namd fail \n");
		ret = WEARABLE_FT_TEST_RESULT_TEST_INIT_NG;
		return ret;
	}

	if (raydium_check_ini_version() != 0) {
		ret = WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG;
	}

	return ret;
}

int self_test_save_test_raw_data_to_file(int i32_ng_type)
{
	/*struct tm *time_infor;*/
	/*time_t raw_time;*/
	char write_string[1000];
	unsigned char u8_i, u8_j;
	short *p_i16_buf = (short *)g_u16_raw_data_tmp;
#if 0
	/*Date*/
	time(&raw_time);
	time_infor = localtime(&raw_time);
	memset(write_string, 0, strlen(write_string));
	sprintf(write_string, "Date=%s\n", asctime(time_infor));
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
#endif
	/*FW Version*/
	memset(write_string, 0, strlen(write_string));
	if (g_u16_dev_id != 0) {
		sprintf(write_string, "FW Version=0x%.2X%.2X%.2X%.2X\n", g_u8_normal_fw_version_buf[0], g_u8_normal_fw_version_buf[1],
			g_u8_normal_fw_version_buf[2], g_u8_normal_fw_version_buf[3]);
	}
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	/*Version*/
	memset(write_string, 0, strlen(write_string));
	sprintf(write_string, "Selftest Version=%x.%x.%x.%x.%x\n\n", RM_SELF_TEST_CUSTOMER_VERSION, RM_SELF_TEST_PLATFORM_VERSION,
		RM_SELF_TEST_PROJECT_VERSION, RM_SELF_TEST_MAIN_VERSION, RM_SELF_TEST_SUB_VERSION);
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	/*Test result*/
	memset(write_string, 0, strlen(write_string));
	sprintf(write_string, "Test Result = 0x%08X\n\n", i32_ng_type);
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	if (i32_ng_type == 0) {
		memset(write_string, 0, strlen(write_string));
		sprintf(write_string, "All pass\n\n\n");
		self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
	} else {
		memset(write_string, 0, strlen(write_string));

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_SYSFS_NG) {
			sprintf(write_string + strlen(write_string), "System NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_I2C_NG) {
			sprintf(write_string + strlen(write_string), "I2C NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_INT_NG) {
			sprintf(write_string + strlen(write_string), "INT NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_RESET_NG) {
			sprintf(write_string + strlen(write_string), "RESET NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_PRAM_NG) {
			sprintf(write_string + strlen(write_string), "PRAM NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_NORMAL_FW_NG) {
			sprintf(write_string + strlen(write_string), "NORMAL_FW_NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_OPEN_NG) {
			sprintf(write_string + strlen(write_string), "OPEN NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_SHORT_NG) {
			sprintf(write_string + strlen(write_string), "SHORT NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_BURN_CC_NG) {
			sprintf(write_string + strlen(write_string), "BURN CC NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_GET_DATA_NG) {
			sprintf(write_string + strlen(write_string), "GET DATA NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_FLASH_ID_NG) {
			sprintf(write_string + strlen(write_string), "FLASH ID NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_NORMAL_FW_VER_NG) {
			sprintf(write_string + strlen(write_string), "NORMAL FW VER NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG) {
			sprintf(write_string + strlen(write_string), "TEST FW VER NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_TEST_INIT_NG) {
			sprintf(write_string + strlen(write_string), "TEST INIT NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_LOAD_TESTFW_NG) {
			sprintf(write_string + strlen(write_string), "LOAD TESTFW NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_BURN_FW_NG) {
			sprintf(write_string + strlen(write_string), "BURN FW NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_SINGLE_CC_OPEN_NG) {
			sprintf(write_string + strlen(write_string), "Open NG (Single Pin CC) ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_SINGLE_CC_SHORT_NG) {
			sprintf(write_string + strlen(write_string), "Short NG (Single Pin CC) ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_UB_NG) {
			sprintf(write_string + strlen(write_string), "Uniformity Baseline NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_UC_NG) {
			sprintf(write_string + strlen(write_string), "Uniformity CC NG ");
		}

		sprintf(write_string + strlen(write_string), "\n");
		self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
	}

	/*Threshold*/
	memset(write_string, 0, strlen(write_string));
	sprintf(write_string, "0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n",
		(g_st_test_thd.i16_ft_test_open_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_open_lower_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_short_upper_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_short_upper_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_short_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_short_lower_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_single_cc_upper_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_single_cc_upper_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_single_cc_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_single_cc_lower_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_uniformity_bl_upper_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_uniformity_bl_upper_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_uniformity_bl_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_uniformity_bl_lower_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_uniformity_cc_upper_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_uniformity_cc_upper_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_uniformity_cc_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_uniformity_cc_lower_thd & 0xFF));
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	for (u8_i = 0; u8_i < MAX_SENSING_PIN_NUM; u8_i++) {
		memset(write_string, 0, strlen(write_string));
		sprintf(write_string, "0x%2X,", g_u16_uc_golden_cc_buf[u8_i]);
		self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
	}

	memset(write_string, 0, strlen(write_string));
	sprintf(write_string, "\n");
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	for (u8_i = 0; u8_i < MAX_SENSING_PIN_NUM; u8_i++) {
		memset(write_string, 0, strlen(write_string));
		sprintf(write_string, "0x%2X,", g_u16_raw_data3_golden_cc_buf[u8_i]);
		self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
	}

	memset(write_string, 0, strlen(write_string));
	sprintf(write_string, "\n\n\n\n\n\n\n\n\n\n\n\n\n");
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	if ((i32_ng_type & 0xFFF0FBF8) < 4) {

		if ((g_st_test_info.u16_ft_test_item & (IC_TEST_ITEMS_SHORT)) != 0) {
			/*Raw data*/
			/*Raw data slow*/
			memset(write_string, 0, strlen(write_string));
			sprintf(write_string, "\r\n\n\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			memset(write_string, 0, strlen(write_string));
			sprintf(write_string, "Raw Data 1\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			/*Raw data1*/
			memset(p_i16_buf, 0, MAX_IMAGE_BUFFER_SIZE * 2);
			for (u8_i = 0; u8_i < MAX_IMAGE_BUFFER_SIZE; u8_i++)
				if (g_u8_wearable_pin_map[u8_i] != F303_NA_P)
					p_i16_buf[g_u8_wearable_pin_map[u8_i]] = g_i16_raw_data_1_short_buf[u8_i];

			for (u8_j = 0; u8_j < g_u8_channel_y; u8_j++) {
				for (u8_i = 0; u8_i < g_u8_channel_x; u8_i++) {
					memset(write_string, 0, strlen(write_string));
					sprintf(write_string, "%05d,", p_i16_buf[u8_i + u8_j * g_u8_channel_x]);
					self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
				}

				memset(write_string, 0, strlen(write_string));
				sprintf(write_string, "\r\n");
				self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			}
		}
		if ((g_st_test_info.u16_ft_test_item & (IC_TEST_ITEMS_OPEN)) != 0) {
			memset(write_string, 0, strlen(write_string));
			sprintf(write_string, "\r\n\n\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			/*Raw data slow*/
			memset(write_string, 0, strlen(write_string));
			sprintf(write_string, "Raw Data 2\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			/*Raw data2*/
			memset(p_i16_buf, 0, MAX_IMAGE_BUFFER_SIZE * 2);
			for (u8_i = 0; u8_i < MAX_IMAGE_BUFFER_SIZE; u8_i++)
				if (g_u8_wearable_pin_map[u8_i] != F303_NA_P)
					p_i16_buf[g_u8_wearable_pin_map[u8_i]] = g_i16_raw_data_2_open_buf[u8_i];

			for (u8_j = 0; u8_j < g_u8_channel_y; u8_j++) {
				for (u8_i = 0; u8_i < g_u8_channel_x; u8_i++) {
					memset(write_string, 0, strlen(write_string));
					sprintf(write_string, "%05d,", p_i16_buf[u8_i + u8_j * g_u8_channel_x]);
					self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
				}

				memset(write_string, 0, strlen(write_string));
				sprintf(write_string, "\r\n");
				self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			}
		}
		if ((g_st_test_info.u16_ft_test_item & (IC_TEST_ITEMS_OPEN | IC_TEST_ITEMS_SHORT)) != 0) {
			memset(write_string, 0, strlen(write_string));
			sprintf(write_string, "\r\n\n\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			/*Raw data 3*/
			memset(write_string, 0, strlen(write_string));
			sprintf(write_string, "Raw Data 3\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			/*Raw data3*/
			memset(p_i16_buf, 0, MAX_IMAGE_BUFFER_SIZE * 2);
			for (u8_i = 0; u8_i < MAX_IMAGE_BUFFER_SIZE; u8_i++)
				if (g_u8_wearable_pin_map[u8_i] != F303_NA_P)
					p_i16_buf[g_u8_wearable_pin_map[u8_i]] = g_u16_raw_data3_cc_buf[u8_i];

			for (u8_j = 0; u8_j < g_u8_channel_y; u8_j++) {
				for (u8_i = 0; u8_i < g_u8_channel_x; u8_i++) {
					memset(write_string, 0, strlen(write_string));
					sprintf(write_string, "%05d,", p_i16_buf[u8_i + u8_j * g_u8_channel_x]);
					self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
				}

				memset(write_string, 0, strlen(write_string));
				sprintf(write_string, "\r\n");
				self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			}
		}
		if ((g_st_test_info.u16_ft_test_item & (IC_TEST_ITEMS_UC)) != 0) {
			memset(write_string, 0, strlen(write_string));
			sprintf(write_string, "\r\n\n\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			/*Raw data Uniformity CC*/
			memset(write_string, 0, strlen(write_string));
			sprintf(write_string, "Raw Data_UC\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			/*Raw data uc*/
			memset(p_i16_buf, 0, MAX_IMAGE_BUFFER_SIZE * 2);
			for (u8_i = 0; u8_i < MAX_IMAGE_BUFFER_SIZE; u8_i++)
				if (g_u8_wearable_pin_map[u8_i] != F303_NA_P)
					p_i16_buf[g_u8_wearable_pin_map[u8_i]] = g_u16_uc_buf[u8_i];

			DEBUGOUT("Image:0x%x\r\n", p_i16_buf[0]);

			for (u8_j = 0; u8_j < g_u8_channel_y; u8_j++) {
				for (u8_i = 0; u8_i < g_u8_channel_x; u8_i++) {
					memset(write_string, 0, strlen(write_string));
					sprintf(write_string, "%05d,", p_i16_buf[u8_i + u8_j * g_u8_channel_x]);
					self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
				}

				memset(write_string, 0, strlen(write_string));
				sprintf(write_string, "\r\n");
				self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			}
		}
	}

	memset(write_string, 0, strlen(write_string));
	sprintf(write_string, "\r\n\n\n");
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	return 1;
}

int self_test_read_setting_from_ini(void)
{
	int i32_ret;
	char *p_rbuf, *free_p_rbuf, *p_r;
	const char *const delim = "=,{}";
	char *p_sep, *p_temp;
	char *p_sep1;

	st_test_info st_ini_info;
	st_test_threshold st_ini_thd;

	signed short i16_thd[16], i16_val;
	unsigned short u16_test_info[6];
	unsigned char u8_i, u8_val;
	unsigned char u8_test_para[48];
	unsigned short u16_raw3_cc[MAX_IMAGE_BUFFER_SIZE];
	unsigned short u16_uc_cc[MAX_IMAGE_BUFFER_SIZE];
	unsigned short u16_offset = 0, u16_val;
	struct i2c_client *client = g_raydium_ts->client;
	const struct firmware *ini = NULL;

	i32_ret = request_firmware(&ini, RM_SELF_TEST_THRESHOLD_FILE, &client->dev);
	if (i32_ret < 0) {
		DEBUGOUT("Request firmware ini failed\r\n");
		return ERROR;
	}

	if (ini->size > 0) {
		DEBUGOUT("ini size %d\r\n", ini->size);
		p_rbuf = kzalloc(ini->size, GFP_KERNEL);
		if (p_rbuf == NULL) {
			DEBUGOUT("kzalloc p_rbuf failed\r\n");
			return -ENOMEM;
		}
	} else {
		DEBUGOUT("%s ini download failed %d\r\n", RM_SELF_TEST_THRESHOLD_FILE);
		return ERROR;
	}

	free_p_rbuf = p_rbuf;
	memcpy((void *)p_rbuf, (void *)ini->data, ini->size);
	p_r = p_rbuf;

	p_sep1 = strsep(&p_r, "\n");
	p_sep = strsep(&p_sep1, "\r");
	strsep(&p_sep, delim);
	p_temp = p_sep;
	if (p_temp != NULL) {
		i32_ret = kstrtou8(p_temp, 10, &st_ini_info.u8_device_id);
		if (i32_ret < 0)
			goto exit_ini;
	} else {
		goto exit_ini;
	}
	//DEBUGOUT("st_ini_info.u8_device_id %d \r\n", st_ini_info.u8_device_id);

	p_sep1 = strsep(&p_r, "\n");
	p_sep = strsep(&p_sep1, "\r");
	strsep(&p_sep, delim);
	p_temp = p_sep;
	if (p_temp != NULL) {
		i32_ret = kstrtou8(p_temp, 10, &st_ini_info.u8_company_id);
		if (i32_ret < 0)
			goto exit_ini;
	} else {
		goto exit_ini;
	}
	//DEBUGOUT("st_ini_info.u8_company_id %d \r\n", st_ini_info.u8_company_id);

	p_sep1 = strsep(&p_r, "\n");
	p_sep = strsep(&p_sep1, "\r");
	strsep(&p_sep, delim);
	p_temp = p_sep;
	if (p_temp != NULL) {
		i32_ret = kstrtou8(p_temp, 10, &st_ini_info.u8_project_id);
		if (i32_ret < 0)
			goto exit_ini;
	} else {
		goto exit_ini;
	}
	//DEBUGOUT("st_ini_info.u8_project_id %d \r\n", st_ini_info.u8_project_id);

	p_sep1 = strsep(&p_r, "\n");
	p_sep = strsep(&p_sep1, "\r");
	strsep(&p_sep, delim);
	p_temp = p_sep;
	if (p_temp != NULL) {
		i32_ret = kstrtou8(p_temp, 10, &st_ini_info.u8_station_id);
		if (i32_ret < 0)
			goto exit_ini;
	} else {
		goto exit_ini;
	}
	//DEBUGOUT("st_ini_info.u8_station_id %d \r\n", st_ini_info.u8_station_id);

	for (u8_i = 0; u8_i < 6; u8_i ++) {
		p_sep1 = strsep(&p_r, "\n");
		p_sep = strsep(&p_sep1, "\r");
		strsep(&p_sep, delim);
		p_temp = p_sep;
		if (p_temp != NULL) {
			i32_ret = kstrtou16(p_temp, 10, &u16_val);
			if (i32_ret < 0)
				goto exit_ini;
			u16_test_info[u8_i] = u16_val;
		} else {
			goto exit_ini;
		}
	}
	memcpy((void *)(&st_ini_info.u16_ft_test_item) , &u16_test_info[0], sizeof(u16_test_info));
	DEBUGOUT("test item :0x%x\n",  st_ini_info.u16_ft_test_item);

	for (u8_i = 0; u8_i < 16; u8_i ++) {
		p_sep1 = strsep(&p_r, "\n");
		p_sep = strsep(&p_sep1, "\r");
		strsep(&p_sep, delim);
		p_temp = p_sep;
		if (p_temp != NULL) {
			i32_ret = kstrtos16(p_temp, 10, &i16_val);
			if (i32_ret < 0)
				goto exit_ini;
			i16_thd[u8_i] = i16_val;
		} else {
			goto exit_ini;
		}
	}
	memcpy((void *)(&st_ini_thd.i16_ft_test_open_lower_thd) , &i16_thd[0], sizeof(i16_thd));

	for (u8_i = 0; u8_i < 48; u8_i ++) {
		p_sep1 = strsep(&p_r, "\n");
		p_sep = strsep(&p_sep1, "\r");
		strsep(&p_sep, delim);
		p_temp = p_sep;
		if (p_temp != NULL) {
			i32_ret = kstrtou8(p_temp, 10, &u8_val);
			if (i32_ret < 0)
				goto exit_ini;
			u8_test_para[u8_i] = u8_val;
		} else {
			goto exit_ini;
		}
	}
	DEBUGOUT("u8_test_para[0] :0x%x\n",  u8_test_para[0]);

	p_sep1 = strsep(&p_r, "\n");
	p_sep = strsep(&p_sep1, "\r");

	p_temp = strsep(&p_sep, delim);
	p_temp = strsep(&p_sep, delim);
	for (u8_i = 0; u8_i < 48; u8_i++) {
		p_temp = strsep(&p_sep, delim);

		if (p_temp != NULL) {
			i32_ret = kstrtou16(p_temp, 10, &u16_val);
			if (i32_ret < 0)
				goto exit_ini;
			u16_raw3_cc[u8_i] = u16_val;
		}
	}
	DEBUGOUT("u16_raw3_cc[0] :0x%x\n",  u16_raw3_cc[0]);


	p_sep1 = strsep(&p_r, "\n");
	p_sep = strsep(&p_sep1, "\r");

	p_temp = strsep(&p_sep, delim);
	p_temp = strsep(&p_sep, delim);
	for (u8_i = 0; u8_i < 48; u8_i++) {
		p_temp = strsep(&p_sep, delim);

		if (p_temp != NULL) {
			i32_ret = kstrtou16(p_temp, 10, &u16_val);
			if (i32_ret < 0)
				goto exit_ini;
			u16_uc_cc[u8_i] = u16_val;
		}
	}
	DEBUGOUT("u16_uc_cc[0] :0x%x\n",  u16_uc_cc[0]);


	memcpy((void *)(&g_st_test_para_resv) , &u8_test_para[0], sizeof(g_st_test_para_resv));

	u16_offset = 0;
	memcpy(&g_u8_ini_flash[u16_offset], (void *)&st_ini_info.u8_device_id, sizeof(g_st_test_info));
	u16_offset += 16;
	memcpy(&g_u8_ini_flash[u16_offset], (void *)&st_ini_thd.u8_ft_test_company_id, sizeof(g_st_test_thd));
	u16_offset += 36;
	memcpy(&g_u8_ini_flash[u16_offset], &u8_test_para, sizeof(u8_test_para));
	u16_offset += 48;

	u16_offset += 128;//reserve for BL
	memcpy(&g_u8_ini_flash[u16_offset], &u16_raw3_cc, sizeof(u16_raw3_cc));
	u16_offset += 128;
	memcpy(&g_u8_ini_flash[u16_offset], &u16_uc_cc, sizeof(u16_uc_cc));
	u16_offset += 128;
	DEBUGOUT("ini length = %d\r\n", u16_offset);

	kfree(free_p_rbuf);
	return SUCCESS;

exit_ini:
	kfree(free_p_rbuf);
	return ERROR;
}


int self_test_read_setting_from_file(void)
{
	unsigned short u16_offset = 0;
	DEBUGOUT("[touch]g_raydium_ts->id = 0x%x\r\n", g_raydium_ts->id);
	switch (g_raydium_ts->id) {
	case RAD_SELFTEST_30:
		u16_offset = 0;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_test_info_30, sizeof(u8_test_info_30));
		u16_offset += 16;
		memcpy(&g_u8_ini_flash[u16_offset], &i8_ft_test_thd_30, sizeof(i8_ft_test_thd_30));
		u16_offset += 36;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_test_para_30, sizeof(u8_test_para_30));
		u16_offset += 48;
		u16_offset += 128;/*reserve for BL*/
		memcpy(&g_u8_ini_flash[u16_offset], &u8_raw_data_3_cc_30, sizeof(u8_raw_data_3_cc_30));
		u16_offset += 128;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_raw_uc_cc_30, sizeof(u8_raw_uc_cc_30));
		u16_offset += 128;

		memcpy((void *)(&g_st_test_para_resv), &u8_test_para_30[0], sizeof(g_st_test_para_resv));
		DEBUGOUT("ini length = %d\r\n", u16_offset);
		break;
#if defined(FW_MAPPING_EN)
	case RAD_SELFTEST_31:
		u16_offset = 0;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_test_info_31, sizeof(u8_test_info_31));
		u16_offset += 16;
		memcpy(&g_u8_ini_flash[u16_offset], &i8_ft_test_thd_31, sizeof(i8_ft_test_thd_31));
		u16_offset += 36;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_test_para_31, sizeof(u8_test_para_31));
		u16_offset += 48;
		u16_offset += 128;/*reserve for BL*/
		memcpy(&g_u8_ini_flash[u16_offset], &u8_raw_data_3_cc_31, sizeof(u8_raw_data_3_cc_31));
		u16_offset += 128;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_raw_uc_cc_31, sizeof(u8_raw_uc_cc_31));
		u16_offset += 128;

		memcpy((void *)(&g_st_test_para_resv), &u8_test_para_31[0], sizeof(g_st_test_para_resv));
		DEBUGOUT("ini length = %d\r\n", u16_offset);
		break;
#endif
	}

	return 0;
}

int raydium_do_selftest(struct raydium_ts_data *ts)
{
	int ret = RESULT_SUCCESS;
	unsigned int time_start, time_end, time_start2, time_end2;
	int test_result = 1;
	time_start = get_system_time();

	pr_info("Selftest Version=%x.%x.%x.%x.%x\n", RM_SELF_TEST_CUSTOMER_VERSION, RM_SELF_TEST_PLATFORM_VERSION,
		RM_SELF_TEST_PROJECT_VERSION, RM_SELF_TEST_MAIN_VERSION, RM_SELF_TEST_SUB_VERSION);

#if 1
	if (self_test_read_setting_from_ini() != SUCCESS) {
		DEBUGOUT("Use Test FW Threhold\n");
		self_test_read_setting_from_file();
	} else {
		DEBUGOUT("Read ini Success\r\n");
	}
#else
	self_test_read_setting_from_file();
#endif

	ic_drv_init();
	set_raydium_ts_data(ts);

	ret = self_test_init();
	if (ret != 0) {
		DEBUGOUT("mapping ic fw fail \n");
		test_result = 2;
	} else {
		DEBUGOUT("Test all\n");
		ret |= self_test_all();
	}
#if 1
	if (ret != WEARABLE_FT_TEST_RESULT_SYSFS_NG) {
		gpio_touch_hw_reset();
		g_u8_raydium_flag &= ~ENG_MODE;
	}

	raydium_i2c_mode_control(ts->client, ENABLE_TOUCH_INT);
#if ENABLE_TIME_MEASURMENT
	time_start2 = get_system_time();
#endif
	self_test_save_test_raw_data_to_file(ret);

#if ENABLE_TIME_MEASURMENT
	time_end2 = get_system_time();
	DEBUGOUT("Write log Finish(%ums)\n", time_end2 - time_start2);
#endif
	if (ret != 0) {
		if (ret & WEARABLE_FT_TEST_RESULT_I2C_NG) {
			test_result = 5;
		} else {
			test_result = 6;
		}
		DEBUGOUT("Selftest Test Result=0x%x\n", ret);
		ret = RESULT_NG;
		DEBUGOUT("Selftest Result=%d\n", ret);
	} else {
		DEBUGOUT("Selftest Pass ^_^!!!\n");
		ret = RESULT_SUCCESS;
	}

	time_end = get_system_time();
	DEBUGOUT("All Test Finish(%ums)\n", time_end - time_start);

#endif
	return test_result;
}
