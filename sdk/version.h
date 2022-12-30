/********************************************************************************************************
 * @file     version.h 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#pragma once
#include "proj/mcu/config.h"

// can't use "enum" here, because cstartup.S not support
// please make sure make clean, if modify any value in this file.

//#define   BUILD_VERSION  	"Revision: 45:49M"
//#define   BUILD_TIME  		"2012-07-24-21:37:43"

#define LIGHT_MODE_POSITION     (0x1c)  // 0x1c--0x1d

// ------------- light mode --------
#define	LIGHT_MODE_DIM	    0x01
#define	LIGHT_MODE_CCT	    0x02
#define	LIGHT_MODE_RGBW	    0x03
#define	LIGHT_MODE_RGB	    0x04
#define	LIGHT_MODE_CSLEEP	0x05
//---
#define	LIGHT_MODE_SWITCH	        0x20

//---
#define	LIGHT_MODE_MOTION_SENSOR	0x30

//---
#define	LIGHT_MODE_LPN	            0x40

//---
#define	LIGHT_MODE_GATEWAY	        0x50

//---
#define	LIGHT_MODE_MASTER_LIGHT	    0x60

//--- END

#if (__PROJECT_LIGHT_SWITCH__)  // must define in TC32_CC_Assember ->General , too. because cstartup.s can't read predefine value in TC32_compiler-->symbols
#define LIGHT_MODE		LIGHT_MODE_SWITCH
#elif (__PROJECT_MOTIONSENSOR_8267__)  // must define in TC32_CC_Assember ->General , too. because cstartup.s can't read predefine value in TC32_compiler-->symbols
#define LIGHT_MODE		LIGHT_MODE_MOTION_SENSOR
#elif (__PROJECT_LPN__)         // must define in TC32_CC_Assember ->General , too. because cstartup.s can't read predefine value in TC32_compiler-->symbols
#define LIGHT_MODE		LIGHT_MODE_LPN
#elif (__PROJECT_LIGHT_GATEWAY__)
#define LIGHT_MODE		LIGHT_MODE_GATEWAY
#elif (__PROJECT_MASTER_LIGHT_8266__ || __PROJECT_MASTER_LIGHT_8267__)
#define LIGHT_MODE		LIGHT_MODE_MASTER_LIGHT
#else
#define LIGHT_MODE		LIGHT_MODE_RGB  // Note: user should define different type for different IC, because it will be used in mesh OTA.
#endif

#define DEV_INFO_USER   (0x0000)

#define FW_DEV_INFO     (LIGHT_MODE | (DEV_INFO_USER << 16))    // Note: light mode use 2bytes


#if(LIGHT_MODE == LIGHT_MODE_CCT)
#define RGB_MAP_MAX			51
#else   //if(LIGHT_MODE == LIGHT_MODE_RGB)
#define RGB_MAP_MAX			64
#endif

