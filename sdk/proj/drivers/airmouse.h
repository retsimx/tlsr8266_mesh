/********************************************************************************************************
 * @file     airmouse.h 
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

#define AIRMOUSE_CALIBRATION_ADDR   (0x18000)
#define	MPU6050_I2C_ID				(0x68 << 1)

typedef struct {
	s8 btn;
	s8 x;
	s8 y;
	s8 w;
}amouse_data_t;

int airmouse_getxy(amouse_data_t *mouse_data);

