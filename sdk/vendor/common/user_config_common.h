/********************************************************************************************************
 * @file     user_config_common.h 
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

#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#define _USER_CONFIG_DEFINED_	1	// must define this macro to make others known 


#define CHIP_TYPE				CHIP_TYPE_8566		// 8868-24, 8568-32 & 8568-56
#define APPLICATION_DONGLE		0			// or else APPLICATION_DEVICE


/////////////////// Clock  /////////////////////////////////
#if 1
#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#ifndef CLOCK_SYS_CLOCK_HZ
#define CLOCK_SYS_CLOCK_HZ  	24000000
#endif
#else
#define CLOCK_SYS_TYPE  		CLOCK_TYPE_OSC	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#ifndef CLOCK_SYS_CLOCK_HZ
#define CLOCK_SYS_CLOCK_HZ  	32000000
#endif
#endif
/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE	0

///////////////////  interrupt  //////////////////////////////

///////////////////  GPIO  /////////////////////////////////
//  only need to define those are not default
//  all gpios are default to be output disabled, input disabled, output to 0, output strength to 1


#define	CN_FUNC					AS_GPIO
#define	CK_FUNC					AS_GPIO
#define	DO_FUNC					AS_GPIO
#define	DI_FUNC					AS_GPIO

#define CN_OUTPUT_ENABLE		0
#define DI_OUTPUT_ENABLE		1
#define DI_INPUT_ENABLE			1
#define DI_DATA_OUT				0

#define CK_INPUT_ENABLE			1
#define CK_DATA_OUT				0


////////////////// keyboard gpio
//  T0,T1,R0,R1功能由一个bit控制，所以必须定义相同

//// scan pin ////
#define	GPIO23_INPUT_ENABLE     1
#define	GPIO16_INPUT_ENABLE     1
#define	PWM1_FUNC               AS_GPIO
#define MSC_FUNC				AS_GPIO
#define	PWM1_INPUT_ENABLE  	 	1
#define	MSC_INPUT_ENABLE  	  	1

/// drive pin ////
#define	GPIO19_INPUT_ENABLE     1
//#define	GPIO18_INPUT_ENABLE     1
#define	T0_INPUT_ENABLE	        1
#define	T1_INPUT_ENABLE	        1

#define	GPIO15_OUTPUT_ENABLE	    1

#define GPIO8_OUTPUT_ENABLE		1
#define	GPIO8_DATA_OUT    		1


// led gpio
#define	PWM0_FUNC               AS_GPIO
#define PWM0_OUTPUT_ENABLE	    1
#define	PWM0_INPUT_ENABLE	    0

#define	SWM_FUNC               	AS_GPIO
#define SWM_OUTPUT_ENABLE	    1
#define	SWM_INPUT_ENABLE	    0

#define GPIO24_OUTPUT_ENABLE    1


//////////////////    RF configuration //////////////
#define RF_PROTOCOL				RF_PROTO_PROPRIETARY		//  RF_PROTO_PROPRIETARY / RF_PROTO_RF4CE / RF_PROTO_ZIGBEE

///////////////////  ADC  /////////////////////////////////

///////////////////  battery  /////////////////////////////////

#define BATT_ADC_CHANNEL		0
#define BATT_FULL_VOLT			(4100)	//  mV
#define BATT_LOW_VOLT			(3700)	//  mV
#define BATT_NO_PWR_VOLT		(3400)	//  mV
#define	ADC_CHN0_ANA_INPUT		ADC_CHN_INP_ANA_7
#define ADC_CHN0_REF_SRC		ADC_REF_SRC_INTERNAL


/*
the should be the combination of the followings:
DEEPSLEEP_WAKEUP_PIN_GPIO0 to DEEPSLEEP_WAKEUP_PIN_GPIO3
DEEPSLEEP_WAKEUP_PIN_ANA01 to DEEPSLEEP_WAKEUP_PIN_ANA12
*/
/////////////////// set default   ////////////////

#include "../common/default_config.h"

/////////////////// main loop, event loop  ////////////////
enum{ 
	EV_FIRED_EVENT_MAX = 8
};

typedef enum{
	EV_SUSPEND_NOTIFY,
	EV_WAKEUP_NOTIFY,
	EV_KEY_PRESS,
#if(MOUSE_USE_RAW_DATA)
	EV_MOUSE_RAW_DATA,
#endif	
	EV_RF_PKT_RECV,
	EV_PAIRING_START,
	EV_PAIRING_STOP,
	EV_MOUSE_EVENT,
	EV_KEYBOARD_EVENT,
#if(MODULE_SOMATIC_ENABLE)	
	EV_SOMATIC_EVENT,	
#endif
	EV_EVENT_MAX,
}ev_event_e;

typedef enum{
	EV_POLL_MOUSE_EVENT,
	EV_POLL_KEYBOARD_EVENT,
#if(MODULE_SOMATIC_ENABLE)	
	EV_POLL_SOMATIC_EVENT,
#endif	
	EV_POLL_RF_RECV,
	EV_POLL_DEVICE_PKT,
	EV_POLL_RF_CHN_HOPPING,
	EV_POLL_IDLE, //  Must be the last item in ev_poll_e
	EV_POLL_MAX,
}ev_poll_e;

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

