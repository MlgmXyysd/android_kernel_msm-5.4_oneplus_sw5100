//****************************************************************************
//;***************************************************************************
//; Raydium Semiconductor Corporation.
//; Design Division V.,
//; Touch System and Firmware Design Depr II.
//; Raydium Application Software.
//;***************************************************************************
//; Project Name : SDK3.0
//; File Name      : ic_drv_global.c
//; Built Date      :
//;
//; Copyright 2006 Raydium Semiconductor Corporation.
//; 2F, No.23, Li-Hsin Rd.,
//; Hsinchu Science Park, Hsinchu 300,
//; Taiwan, R.O.C.
//; Email : @rad-ic.com
//; Website : www.rad-ic.com
//; All rights are reserved. Reproduction in whole or part is prohibited
//; without the prior written consent of the copyright owner.
//;
//; COMPANY CONFIDENTIAL
//;***************************************************************************
//
// Revision History:
// Rev     Date        Author
//____________________________________________________________________________
// 0.1
// Build.
//
// Sensor IC Scan and Get Raw Data Related Functions
//;---------------------------------------------------------------------------
//****************************************************************************

#include <Config.h>
#include "ic_drv_global.h"
#include "ic_drv_interface.h"
#ifdef __KERNEL__
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#endif

#include "drv_interface.h"



#if !SELFTEST
#include "usb.h"
#include "Descriptors.h"
#endif
/*****************************************************************************
**
**                  Declared Global Variable
**
*****************************************************************************/

//********************* Global *********************//


unsigned short g_u16_dev_id;

unsigned char g_u8_drv_interface;

volatile unsigned char g_u8_gpio_irq_trigger;

unsigned char g_u8_raw_data_buffer[MAX_IMAGE_BUFFER_SIZE * 2];
unsigned short g_u16_raw_data_tmp[MAX_IMAGE_BUFFER_SIZE];

// IC Test
short g_i16_raw_data_1_short_buf[MAX_SENSING_PIN_NUM];
short g_i16_raw_data_2_open_buf[MAX_SENSING_PIN_NUM];
unsigned short g_u16_raw_data3_cc_buf[MAX_SENSING_PIN_NUM];
unsigned short g_u16_uc_buf[MAX_SENSING_PIN_NUM];//CC
unsigned short g_u16_raw_data3_golden_cc_buf[MAX_SENSING_PIN_NUM];
unsigned short g_u16_uc_golden_cc_buf[MAX_SENSING_PIN_NUM];

unsigned int g_u32_test_result[MAX_SENSING_PIN_NUM]; // each node test result (bit[0]:open ng, bit[1]:short ng, bit[2]:uniformity ng..etc)
unsigned char g_u8_test_result[MAX_SENSING_PIN_NUM];


unsigned int g_u32_wearable_test_result;
unsigned char g_u8_wearable_pin_map[MAX_IMAGE_BUFFER_SIZE];


unsigned char g_u8_data_buf[DATA_BUFFER_SIZE];
unsigned char g_u8_mipi_read_buf[56];

unsigned char g_u8_channel_x;
unsigned char g_u8_channel_y;

unsigned char g_u8_is_normal_fw;

volatile unsigned short g_u16_test_items_host_cmd;
unsigned short g_u16_test_items_tool_cmd;
unsigned char g_u8_ic_power_on_ng;
char g_i8_test_baseline_msg[30];
volatile unsigned short g_u16_panel_jig_set_test_items;
bool b_is_auo_jig_testing;

#if ENABLE_TEST_TIME_MEASURMENT || ENABLE_TEST_TIME_MEASURMENT_CC
unsigned int g_u32_spend_time;
#endif

unsigned int g_u32_fw_cc_version;
unsigned char g_u8_print_debug_msg;
unsigned char g_u8_display_interface;
unsigned char g_u8_PAGE_ADDR;

int g_u32_dongle_flash_ini_addr;  //      0xF000
int g_u32_ini_threshold_addr;     //      DONGLE_FLASH_INI_ADDR + 16
int g_u32_ini_para_addr;          //      INI_THRESHOLD_ADDR + 36
//int u32_ini_raw_data_2_bl_addr;   //      INI_PARA_ADDR+48
int g_u32_ini_raw_data_3_cc_addr; //      INI_RAW_DATA2_BL_ADDR+72
int g_u32_ini_uc_cc_addr;         //      INI_RAW_DATA_3_CC_ADDR + 72// INI_RAW_DATA2_ADDR

int g_u32_initial_code_start_addr;

void ic_drv_init(void)
{
	g_u16_test_items_host_cmd = 0xFFFF;
	g_u16_test_items_tool_cmd = 0;
	g_u16_dev_id = DEVICE_ID_3X;
	g_u8_print_debug_msg = 0;
	g_u8_display_interface = DISPLAY_TOUCH_2_DRIVER_MODE;
	b_is_auo_jig_testing = false;
#if !SELFTEST
	g_u8_ic_power_on_ng = FALSE;
#endif
}

/*****************************************************************************
**                            End Of File
******************************************************************************/
