/********************************************************************************************************
 * @file     main_light.c 
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
#include "proj/tl_common.h"
#include "proj/mcu/watchdog_i.h"
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/light_ll/light_ll.h"
#include "proj_lib/light_ll/light_frame.h"
#include "proj_lib/ble_ll/service.h"
#include "vendor/common/rtc.h"
#include "proj/drivers/flash.h"
#include "vendor/common/common.h"
#include "proj_lib/ble_ll/blueLight.h"


u16 		ota_pkt_cnt = 0;

fp_rf_led_ota_ok 			p_vendor_rf_led_ota_ok 			= 0;
fp_rf_led_ota_error			p_vendor_rf_led_ota_error 		= 0;
fp_irq_timer1 				p_vendor_irq_timer1 			= 0;
fp_proc_led 				p_vendor_proc_led 				= 0;
fp_rf_link_data_callback	p_vendor_rf_link_data_callback	= 0;
fp_user_init 				p_vendor_user_init				= 0;

rf_custom_dat_t slave_tx_cmd = {
		0x0, 0x0,					//nid,ttc
		0xffff,						//group
		0xffffffff,					//destination id
		{0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	};
	
rf_custom1_dat_t slave_tx_cmd1 = {
	{0, 0, 0},
	{0x11, 0x11}
	};