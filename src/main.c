/********************************************************************************************************
 * @file     main.c 
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

#include "common.h"

void user_init();
void main_loop(void);

u32 flash_adr_mac = 0x76000;
u32 flash_adr_pairing = 0x77000;
u32 flash_adr_dev_grp_adr = 0x79000;
u32 flash_adr_lum = 0x78000;
u32 flash_adr_ota_master = 0x20000;
u32 flash_adr_reset_cnt = 0x7A000;
u32 flash_adr_alarm = 0x7B000;
u32 flash_adr_scene = 0x7C000;
u32 flash_adr_user_data = 0x7D000;
u32 flash_adr_light_new_fw = 0x40000;

int main_entrypoint(void) {
    cpu_wakeup_init();

    clock_init();

    dma_init();

    gpio_init();

    irq_init();

    rf_drv_init(0);

    user_init();

    irq_enable();

    while (1) {
#if(MODULE_WATCHDOG_ENABLE)
        wd_clear();
#endif
        main_loop();
    }
}


