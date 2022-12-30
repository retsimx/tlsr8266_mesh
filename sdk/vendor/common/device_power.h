/********************************************************************************************************
 * @file     device_power.h 
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
 * mouse_power.h
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#ifndef DEVICE_POWER_H_
#define DEVICE_POWER_H_

#define MCU_SIM_SUSPEND     0   //define 1: debug mode, no suspend or deep-sleep

#define M_SUSPEND_0 		0
#define M_SUSPEND_8MS 		1
#define M_SUSPEND_100MS 	2
#define M_SUSPEND_SNSR_SLP	0x08
#define M_SUSPEND_MCU_SLP	0x10
#define M_SUSPEND_DEEP_SLP	0x18

typedef struct{ 
    u32  wakeup_src;
    u32  wakeup_time;

    u16  cnt_8ms;               //count increase if no mouse action within 8ms
    u16  thresh_8ms;    

    u16  cnt_100ms;             //count increase if no mouse action for longer suspend, // may still keep RF link, and after reach some limit may need goto deep sleep mode
    u16  thresh_100ms;

    u8   mode;   
    u8   mcu_sleep_en;      
    u8   sensor_sleep; 
    u8   sensor_sleep_en;

    u8   quick_sleep;    
    u8   device_busy;     
} sleep_cfg_t;

extern sleep_cfg_t device_sleep;
void device_sleep_mode_machine(sleep_cfg_t *s_cfg);
int cpu_sleep_wakeup_rc (int deepsleep, int wakeup_src, u32 wakeup_ms);

/*
 *   based on power saving mode, need setup the wake up pin, 32K timer etc
 *
 */
#if 0
extern int device_sync;
static inline void device_sleep_wakeup( sleep_cfg_t *s_cfg ){
	ll_add_clock_time (8000);
	if ( MCU_SIM_SUSPEND || s_cfg->mode == M_SUSPEND_0 ){
        cpu_suspend_wakeup_sim( 8000 );
	}
	else{
		int wakeup_status = cpu_sleep_wakeup_rc (M_SUSPEND_MCU_SLP & s_cfg->mode, s_cfg->wakeup_src, s_cfg->wakeup_time);   // 8ms wakeup
		if(!(wakeup_status & 2)){
            device_sync = 0;	//ll_channel_alternate_mode ();
		}
    }
}
#endif
#endif /* MOUSE_POWER_H_ */
