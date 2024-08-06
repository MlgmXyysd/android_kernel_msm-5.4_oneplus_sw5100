//****************************************************************************
//;***************************************************************************
//; Raydium Semiconductor Corporation.
//; Design Division V.,
//; Touch System and Firmware Design Depr II.
//; Raydium Application Software.
//;***************************************************************************
//; Project Name :
//; File Name      :
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
//
//;---------------------------------------------------------------------------
//****************************************************************************
#ifndef __CONFIG_H
#define __CONFIG_H


//  directory is setting in makefile : e.g. /SDK3.0/Firmware/Driver/interface
//
//#include <main.h>

/*****************************************************************************
**                            GLOBAL MARCO DEFINITION
******************************************************************************/
#define MINIMUM(x, y) (((x) < (y)) ? (x) : (y)) //  Compares two parameters, return minimum.
#define MAXIMUM(x, y) (((x) > (y)) ? (x) : (y)) //  Compares two parameters, return maximum.
#define ABS(x) ((x) >= 0 ? (x) : -(x))          // return absolute value

/*****************************************************************************
**                            GLOBAL FUNCTIONAL DEFINITION
******************************************************************************/

//Bootloader
#define BOOTLOADER                          1  //1: with bootloader, 0: without bootloader
#define SELFTEST                       	     1  //1: For System Selftest, 0:For Dongle Open/Short Tool
#if SELFTEST
#define SELFTEST_3X	1
#define	SELFTEST_2X	0
#else
#define SELFTEST_3X	0
#define	SELFTEST_2X	0
#endif
#define ENABLE_TEST_TIME_MEASURMENT		1
#define ENABLE_TEST_TIME_MEASURMENT_CC		0
#define ENABLE_WDT 				0
#define ENABLE_TEST_RSU_DATA_SHOW		0
#define ENABLE_AUO_VERIFY_LOG			0
#define ENABLE_CONTROL_OPENSHORT_WDT		0
#define ENABLE_TEST_GPIO_MEASURMENT		0
#endif /* end __CONFIG_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
