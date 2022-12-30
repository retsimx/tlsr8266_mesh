/********************************************************************************************************
 * @file     pm_test.c 
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

#if(!(__TL_LIB_5328__ || __TL_LIB_5320__ || __TL_LIB_5332__ || __TL_LIB_5330__))

#include "../../proj/config/user_config.h"
#include "../../proj/mcu/config.h"
#include "../../proj/common/types.h"
#include "../../proj/common/compatibility.h"
#include "../../proj/common/bit.h"
#include "../../proj/common/utility.h"
#include "../../proj/common/static_assert.h"
#include "../../proj/mcu/compiler.h"
#include "../../proj/mcu/register.h"
#include "../../proj/mcu/analog.h"
#include "../../proj/mcu/clock.h"
#include "../../proj/mcu/gpio.h"
#include "../../proj/mcu/irq_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"

#if(0 && __PROJECT_PM_TEST__)

void pm_led_flash(int gpio, int cnt){
	static u32 led_count = 0;
	static u32 led_on = 0;
	if(++led_count > cnt){
		led_count = 0;
		gpio_write(gpio, led_on);
		led_on = led_on ? 0 : 1;
	}
}


u8 chn = 0;
u8 wakeup_no;
u8 test_packet[256] = {0x20,0x00,0x00,0x00,0x21,0x00,0x00,0x00,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78};

void send_test_pkt(void)
{
	static u32 dat = 0x55;
	static int fnum = 0;
	fnum++;
	
	for(int i=0;i<16;i++)
	{
		test_packet[i+12] = fnum;
	}
	extern u8 cap_32m, cap_32k;
#if (MCU_CORE_TYPE == MCU_CORE_5320)
	extern unsigned char cap_rx[16];
	extern unsigned char cap_tx[16];
#else
	extern unsigned short cap_rx[16];
	extern unsigned short cap_tx[16];
#endif

	test_packet[6] = cap_32m;
	test_packet[7] = cap_32k;

	test_packet[8] = wakeup_no;
	test_packet[9] = cap_rx[chn];
	*(u16 *)(test_packet + 10) = cap_tx[chn];

	//*(u32 *)(test_packet + 8) = wakeup_no;
	test_packet[12] = analog_read (0x20);
	test_packet[13] = analog_read (0x21);
	test_packet[14] = analog_read (0x22);
	test_packet[15] = 0;

	TxPkt((void*)(&test_packet[0]));

}

void gpio_wakeup_test(void){
	int i;
	for(i=0;i<10;i++){
		write_reg8(0x8005ae,0x10);
		//delay(10000);
		WaitUs (100000);
		write_reg8(0x8005ae,0x00);
		//delay(10000);
		WaitUs (100000);
	}
}


volatile u32		acnt = 0;
FLASH_ADDRESS_DEFINE;
int main(void){
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();
	gpio_init ();
	reg_wakeup_en = FLD_WAKEUP_SRC_GPIO;
//	reg_gpio_f_wakeup_en = MOUSE_BUTTON_LEFT | FLD_GPIO_WAKEUP_EN;
#if 0
while(1){
	gpio_write (GPIO_SWM, 0);
	WaitUs (1000000);
	gpio_write (GPIO_SWM, 1);
	WaitUs (1000000);

}
#endif
	static int count = 0;

	clock_init();
	gpio_write (GPIO_SWM, 0);

	int deepsleep;

	deepsleep = rf_drv_init (0);

	//while (1);
	wakeup_no = analog_read (0x1b);
	if (deepsleep) {
		wakeup_no++;
	}
	else {
		wakeup_no = 0x80;
		analog_write (0x1f, 0x01);
	}
	analog_write (0x1b, wakeup_no);

	SetTxMode (chn, RF_CHN_TABLE);


	while(1)
	{	
		send_test_pkt();
		//delay(5000);
		WaitUs (800);
#if 1

#if (MCU_CORE_TYPE == MCU_CORE_5328)
		cpu_sleep_wait ();
#endif

		u8 r = irq_disable();

		//cpu_sleep_wakeup(SUSPEND_MODE, 0, WAKEUP_SRC_GPIO2,  WAKEUP_LEVEL_L);
		//cpu_sleep_wakeup(DEEPSLEEP_MODE, 0, WAKEUP_SRC_GPIO2,  WAKEUP_LEVEL_L);		// suspend

		//cpu_sleep_wakeup(SUSPEND_MODE, 0, WAKEUP_SRC_GPIO0, WAKEUP_LEVEL_L);		// suspend
		//cpu_sleep_wakeup(SUSPEND_MODE, 8, 0,  0);		// suspend
		cpu_sleep_wakeup(SUSPEND_MODE, 8, 0,  0);		// suspend
		//cpu_sleep_wakeup(SUSPEND_MODE, 0, WAKEUP_SRC_GPIO1,  0);		// suspend
		//cpu_sleep_wakeup(DEEPSLEEP_MODE, 50, 0,  0);		// deepsleep

		//gpio_write (GPIO_SWS, 1);
		clock_init();						// re-init clock
		rf_drv_init (1);
		irq_restore(r);
		WaitUs (800);


#else
		WaitUs (8000);
#endif

		SetTxMode (chn, RF_CHN_TABLE);	// set tx again
		WaitUs (200);

		if(((wakeup_no++) & 0x7f) > 20) {
			//while (1);
		}
	}
	return 0;
}

#endif

#endif


