/********************************************************************************************************
 * @file     device_power.c 
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
 * mouse_power.c
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/pm.h"

void cpu_suspend_wakeup_sim ( u32 time_us ){
    static u32 tick;
    while (!clock_time_exceed (tick, time_us));
    tick = clock_time ();
}

#include "device_power.h"
#include "device_led.h"

sleep_cfg_t device_sleep = {
    0,
    100,
    
    0,
    0xff,           //thresh_8ms    
    0,
    600,            //thresh_100ms * x
    
    M_SUSPEND_8MS,  //mcu 8ms
    0,              //mcu_sleep_en
    
    0,              //not sensor sleep 
    0,              //sensor sleep enable
    
    0,              //quick sleep disable
    1,              //busy
};


/*
 * Based on input parameters, and current condition
 * to determine mouse power saving mode, which can be
 * suspend, deep sleep, or just cpu stall
 */
void device_sleep_mode_machine(sleep_cfg_t *s_cfg){
    if ( s_cfg->device_busy ){
        s_cfg->mode = M_SUSPEND_8MS;
        s_cfg->cnt_8ms = 0;
    }
    else if (s_cfg->mode == M_SUSPEND_8MS){
        s_cfg->cnt_8ms ++;
        if ( s_cfg->cnt_8ms >= s_cfg->thresh_8ms ){          
            s_cfg->mode = M_SUSPEND_100MS;
            s_cfg->cnt_100ms = 0;
        }
    }
    
    if ( s_cfg->mode & M_SUSPEND_100MS ) {
		s_cfg->cnt_100ms ++;
		if ( s_cfg->cnt_100ms > s_cfg->thresh_100ms )	{	
			s_cfg->mode = M_SUSPEND_DEEP_SLP;            
		}
	}
    
	if ( (s_cfg->mode & M_SUSPEND_DEEP_SLP) || s_cfg->quick_sleep ) {     
        s_cfg->mode = M_SUSPEND_DEEP_SLP;
        if ( !s_cfg->mcu_sleep_en ){
			s_cfg->mode = M_SUSPEND_100MS;
            s_cfg->cnt_100ms = 0;
        }
        if ( s_cfg->sensor_sleep_en ){
			s_cfg->sensor_sleep = M_SUSPEND_SNSR_SLP;
        }
	}
}

