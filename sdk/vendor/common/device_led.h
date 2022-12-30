/********************************************************************************************************
 * @file     device_led.h 
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
 * device_led.h
 *
 *  Created on: Feb 11, 2014
 *      Author: xuzhen
 */

#pragma once
#include "user_config.h"
/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#ifndef DEVICE_LED_MODULE_EN
#define DEVICE_LED_MODULE_EN    1
#endif


#define dbg_led_high_low    0
#if dbg_led_high_low
#else
#define dbg_led_init
#define dbg_led_high
#define dbg_led_low
#endif

typedef struct{
    u8  on_time;        //led on time: *64ms
    u8  off_time;       //led off time: *64ms 
    u8  repeat_count;   //led on-off repeat count bit7-~bit0
    u8  over_wrt;       //BIT[5:0]led on-off repeat count bit13-~bit8
                        //BIT[7:6]-over_wrt priority: over-write last led event (11>10>01>00)
} led_cfg_t;


typedef struct {
	u32 gpio;
    
	u8  level_on;
	u8  is_on;
	u8  over_wrt;    
	u8  cnt_rate;
    
	u32 repeat_count;

	u32 on_time;
	u32 off_time;

	u32 clock;

}device_led_t;

extern device_led_t      device_led;

#if DEVICE_LED_MODULE_EN
void device_led_init(u32 led_pin, u32 led_level, u8 cnt_rate);
void device_led_process(device_led_t * led);
int device_led_setup(led_cfg_t led_cfg);
#else
static inline void device_led_init(u32 led_pin, u32 led_level, u8 cnt_rate) {}
static inline void device_led_process(device_led_t * led) {}
static inline void device_led_setup(led_cfg_t led_cfg) {}
#endif

#define LED_EVENT_BUSY  (device_led.repeat_count)

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
