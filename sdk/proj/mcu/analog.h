/********************************************************************************************************
 * @file     analog.h 
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

#include "../common/types.h"

u8 analog_read(u8 addr);
void analog_write(u8 addr, u8 v);
void analog_read_multi(u8 addr, u8 *v, int len);
void analog_write_multi(u8 addr, u8 *v, int len);
void analog_set_freq_offset(int off);

// the analog version of SET_FLD_V
#define ANALOG_SET_FLD_V(addr, ...)				\
	do{											\
		u8 v = analog_read(addr);		\
		SET_FLD_V(v, __VA_ARGS__);				\
		analog_write(addr, v);			\
	}while(0)

