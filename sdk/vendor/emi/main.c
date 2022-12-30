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

#if(!__PROJECT_PM_TEST__)

#include "../../proj/mcu/watchdog_i.h"
#include "../../vendor/common/user_config.h"

#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"

extern void user_init();
void main_loop(void);
u8 rssi=0;
u32 Adebug_Rx_irq=0;
extern u8 rx_packet[];
void irq_rx_handle()
{
	if((rx_packet[0] >= 15) && RF_PACKET_LENGTH_OK(rx_packet) && RF_PACKET_CRC_OK(rx_packet))
	{
		Adebug_Rx_irq++;

		rssi = rx_packet[4]-110;
	}
}
u32 A_irq=0;
_attribute_ram_code_ void irq_handler(void)
{
	u32 src = reg_irq_src;
	src = src;
#if 0
	if(src & FLD_IRQ_TMR1_EN){
		irq_host_timer1();
		reg_tmr_sta = FLD_TMR_STA_TMR1;
	}
#endif
	A_irq++;
	u8  src_rf = reg_rf_irq_status;
	if(src_rf & FLD_RF_IRQ_RX){
		reg_rf_irq_status = FLD_RF_IRQ_RX;
		irq_rx_handle();
		//irq_ll_rx();
	}

	if(src_rf & FLD_RF_IRQ_TX){
		//irq_ll_tx();
	}
}

FLASH_ADDRESS_DEFINE;
int main (void) {
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();
	usb_dp_pullup_en (1);
	clock_init();

	dma_init();

//	gpio_init();

//	irq_init();

//	usb_init();

	rf_drv_init(0);

//    user_init ();

    irq_disable();

	while (1) {
		main_loop ();
	}
}

#endif


