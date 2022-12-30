/********************************************************************************************************
 * @file     app_config.h 
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
#define	__LOG_RT_ENABLE__		0

#define CHIP_TYPE				CHIP_TYPE_8267		// 8866-24, 8566-32

#define	RF_FAST_MODE_1M			1					// BLE mode

#define	PM_PIN_PULL_DEFAULT		1

#define PULL_WAKEUP_SRC_PA0           PM_PIN_PULL_DEFAULT  //SWS
#define PULL_WAKEUP_SRC_PA1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA7           PM_PIN_PULL_DEFAULT  //SWM
#define PULL_WAKEUP_SRC_PB0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB4           PM_PIN_PULL_DEFAULT  // redefine in wakeup_io_init ()
#define PULL_WAKEUP_SRC_PB5           PM_PIN_PULL_DEFAULT  //DM
#define PULL_WAKEUP_SRC_PB6           PM_PIN_PULL_DEFAULT  // redefine in wakeup_io_init ()
#define PULL_WAKEUP_SRC_PB7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE6           PM_PIN_PULLUP_10K			//MSCN
#define PULL_WAKEUP_SRC_PE7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PF0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PF1           PM_PIN_PULL_DEFAULT

//open SWS output to avoid MCU err
#define PB0_DATA_OUT				  1	

#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ  	32000000

#define MODULE_WATCHDOG_ENABLE	0

#define STACK_CHECK_ENABLE      0

#define ADC_SET_CHN_ENABLE		1
#if ADC_SET_CHN_ENABLE
#define	ADC_CHNM_ANA_INPUT_8267	B7      // for sample
#define ADC_CHNM_REF_SRC_8267	RV_AVDD

#define GPIO_ADC_POWER      GPIO_PA4

#define GPIO_ADC_DAY_NIGHT  GPIO_PB1
#define GPIO_ADC_DELAY      GPIO_PB5
#define GPIO_ADC_LUX_DET    GPIO_PB7
#define GPIO_ADC_PIN        {GPIO_ADC_DELAY, GPIO_ADC_DAY_NIGHT, GPIO_ADC_LUX_DET}
#endif

#define PCB_VER1        1
#define PCB_VER2        2

#define PCB_VER0_SEL    PCB_VER2


#if (PCB_VER0_SEL == PCB_VER1)
#define	PIN_SERDO			GPIO_PA0
#define	PIN_BUTTON			GPIO_PA3
#define	PIN_MOTION			GPIO_PB4
#define	GPIO_LED			GPIO_PB6
#elif (PCB_VER0_SEL == PCB_VER2)   // new pcb
#define PIN_SERDO           GPIO_PE1
#define PIN_BUTTON          GPIO_PE2
#define PIN_MOTION          GPIO_PA3
#define GPIO_LED            GPIO_PC3
#endif

#define PIN_MOTION_HIGH_VALID           1
#define PIN_BUTTON_HIGH_VALID           0

#if 0
#define PWM_R     GPIO_PC2			//red
#define PWM_G     GPIO_PC3			//green
#define PWM_B     GPIO_PB6			//blue
#define PWM_W     GPIO_PB4			//while

#define PWMID_R     2
#define PWMID_G     3
#define PWMID_B     5
#define PWMID_W     4
#endif

#include "../common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
