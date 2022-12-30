/********************************************************************************************************
 * @file     device_led.c 
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
 * device_led.c
 *
 *  Created on: Feb 11, 2014
 *      Author: xuzhen
 */

#include "../../proj/tl_common.h"
#include "device_led.h"

device_led_t      device_led;

#if DEVICE_LED_MODULE_EN

void led_on_off(device_led_t *led, u32 on_off)
{
	led->is_on = on_off;
	gpio_write( led->gpio, led->level_on^led->is_on );
    led->clock = 0;
}

/// \param led_level - 0: level-low led-on,  else: level high led-on
void device_led_init(u32 led_pin, u32 led_level, u8 cnt_rate){
    device_led.gpio = led_pin;
    device_led.level_on = !led_level;
    device_led.cnt_rate = cnt_rate;
    gpio_set_func(led_pin,AS_GPIO);
	gpio_set_input_en( device_led.gpio, 0 ); //input disable
    gpio_set_output_en( device_led.gpio, 1 );//output enable
    led_on_off( &device_led, 0);
    dbg_led_init;
}

void led_dbg_low(){
    gpio_write(device_led.gpio, 0);
}

void led_dbg_high(){
    gpio_write(device_led.gpio, 1);
}

void device_led_process(device_led_t *led)
{
	if(led->repeat_count ==0)
		return;  //led flash finished

    led->clock ++;
	if( led->is_on || led->on_time==0 ){
		if( led->clock >= led->on_time ){
			if( led->off_time ){
                led_on_off(led, 0);
            }            
            led->repeat_count--;
        }
	}
	else{
		if( led->clock >= led->off_time ){			
			if( led->on_time )
                led_on_off(led, 1);
        }
	}
    
}

int device_led_setup(led_cfg_t led_cfg)
{
	u32 led_over_wrt = (led_cfg.over_wrt & 0xc0) > (device_led.over_wrt & 0xc0);
	if( (device_led.repeat_count !=0) && !led_over_wrt )
		return 0;  //can't update led setting now
	else{
		device_led.on_time = led_cfg.on_time * device_led.cnt_rate; //time_per_cycle * device_led.cnt_rate = 8ms * 8 = 64ms
		device_led.off_time = led_cfg.off_time * device_led.cnt_rate;
		device_led.repeat_count = led_cfg.repeat_count | ((led_cfg.over_wrt&0x3f)<<8);
        device_led.over_wrt = led_cfg.over_wrt;
        device_led.clock = 0;
        if( device_led.on_time && !dbg_led_high_low)
            led_on_off(&device_led, 1);
		return 1;
	}
}

#endif

