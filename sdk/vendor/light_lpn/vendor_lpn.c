/********************************************************************************************************
 * @file     vendor_switch.c 
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
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/light_ll/light_ll.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj/drivers/keyboard.h"
#include "../../proj_lib/ble_ll/service.h"
#include "../../proj_lib/ble_ll/gatt_server.h"
FLASH_ADDRESS_EXTERN;

ll_adv_private_t adv_pri_data = {
	VENDOR_ID,
	VENDOR_ID,
	0,
};
extern u8	adv_private_data_len;

ll_adv_rsp_private_t adv_rsp_pri_data = {
	VENDOR_ID,
	VENDOR_ID,
	0,
	0x1234,
	0x01,
};

void vendor_rf_led_ota_ok(void){
	return;
}

void vendor_rf_led_ota_error(void){
	return;
}

void vendor_irq_timer1(void){
	return;
}

void vendor_proc_led(void){
	return;
}

void vendor_rf_link_data_callback (void *p){
	return;
}

void vendor_user_init(void){
	return;
}

void vendor_set_adv_data(void){
	// config adv data
	extern u8 *slave_p_mac;
	adv_pri_data.MacAddress = *(u32 *)slave_p_mac;
	rf_link_slave_set_adv_private_data((u8*)(&adv_pri_data), sizeof(ll_adv_private_t));
    adv_rsp_pri_data.ProductUUID = LIGHT_MODE;

	// config adv rsp data
	adv_rsp_pri_data.MacAddress = *(u32 *)slave_p_mac;
	foreach(i, 16){
	    adv_rsp_pri_data.rsv[i] = i;
	}
}

void set_vendor_function(void){
	return;

	p_vendor_rf_led_ota_ok 			= &vendor_rf_led_ota_ok;
	p_vendor_rf_led_ota_error		= &vendor_rf_led_ota_error;
	p_vendor_irq_timer1				= &vendor_irq_timer1;
	p_vendor_proc_led 				= &vendor_proc_led;
	p_vendor_rf_link_data_callback	= &vendor_rf_link_data_callback;
	p_vendor_user_init				= &vendor_user_init;


	extern attribute_t gAttributes_vendor[];
	extern attribute_t *p_vendor_att;
	p_vendor_att = gAttributes_vendor;
}

