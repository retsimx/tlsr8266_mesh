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

#include "../../proj/tl_common.h"

#include "../../proj/mcu/watchdog_i.h"
#include "../../vendor/common/user_config.h"

#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/light_ll/light_frame.h"

extern void user_init();
extern void proc_button_ahead();
void main_loop(void);

FLASH_ADDRESS_DEFINE;

#if (PM_DEEPSLEEP_RETENTION_ENABLE)
_attribute_ram_code_
#endif
int main (void) {
	FLASH_ADDRESS_CONFIG;
#if PINGPONG_OTA_DISABLE
	ota_fw_check_over_write();	// must at first for main_
#endif
	cpu_wakeup_init();
	//int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp

	usb_dp_pullup_en (1);

#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	clock_init(SYS_CLK_16M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 32000000)
	clock_init(SYS_CLK_32M_Crystal);
#endif

	dma_init();

	gpio_init();

	irq_init();

//	usb_init();

	rf_drv_init(RF_MODE_BLE_1M);
	
	blc_app_loadCustomizedParameters();

#if	(PM_DEEPSLEEP_RETENTION_ENABLE)
    if(pm_is_MCU_deepRetentionWakeup()){
        user_init_deepRetn ();
    }else
#endif
    {
        user_init();
    }

    usb_log_init ();

    irq_enable();

	while (1) {
#if(MODULE_WATCHDOG_ENABLE)
		wd_clear();
#endif
		main_loop ();
	}
}


