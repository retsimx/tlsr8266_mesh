/********************************************************************************************************
 * @file     mouse_type.h 
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
/*
 * mouse_type.h
 *
 *  Created on: Feb 10, 2014
 *      Author: xuzhen
 */

#ifndef MOUSE_TYPE_H_
#define MOUSE_TYPE_H_
#include "..\..\proj\common\types.h"
#include "device_led.h"

#define MOUSE_FRAME_DATA_NUM   4

typedef struct {
	//u8 buff_id;
	u8 btn;
	s8 x;
	s8 y;
	s8 wheel;
	//s8 tl_wheel;
	//u8 hotkey;
}mouse_data_t;




#endif /* MOUSE_TYPE_H_ */
