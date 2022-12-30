/********************************************************************************************************
 * @file     light_8266.h 
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

#include "../common/chip_type_project.h"

#define _USER_CONFIG_DEFINED_	1	// must define this macro to make others known
#define	__LOG_RT_ENABLE__		0

#define CHIP_TYPE				CHIP_TYPE_8266		// 8866-24, 8566-32

#define	RF_FAST_MODE_1M			1					// BLE mode

#define	PM_PIN_PULL_DEFAULT		1

#define PULL_WAKEUP_SRC_PA0           PM_PIN_PULL_DEFAULT  //SWS
#define PULL_WAKEUP_SRC_PA1           0 // LIGHT_PWM must 0(float) or 3(pull_down_100K )
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
#define PULL_WAKEUP_SRC_PB4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB5           PM_PIN_PULL_DEFAULT  //DM
#define PULL_WAKEUP_SRC_PB6           PM_PIN_PULL_DEFAULT  //DP
#define PULL_WAKEUP_SRC_PB7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC0           0 // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PC1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC2           0 // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PC3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC4           0 // LIGHT_PWM must 0(float) or 3(pull_down_100K )
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
#define PULL_WAKEUP_SRC_PE6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PF0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PF1           PM_PIN_PULL_DEFAULT

//open SWS output to avoid MCU err
#define PA0_DATA_OUT				  1	

#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ  	32000000

#define MODULE_WATCHDOG_ENABLE	1
#define WATCHDOG_INIT_TIMEOUT   2000 //ms

#define STACK_CHECK_ENABLE      0

#define ADC_SET_CHN_ENABLE		0
#if ADC_SET_CHN_ENABLE
//#define CHIP_TYPE_8266_A2			1

#define	ADC_CHNM_ANA_INPUT		FLD_ADC_CHN_C7
#define ADC_CHNM_REF_SRC		ADC_REF_AVDD

#define PC7_INPUT_ENABLE		1
#define PC7_OUTPUT_ENABLE		0
#define PC7_FUNC				AS_GPIO
#define PC7_DATA_OUT			0

#ifdef PULL_WAKEUP_SRC_PC7
#undef PULL_WAKEUP_SRC_PC7
#endif

#define PULL_WAKEUP_SRC_PC7		0
#endif

//-------------------------PWM
// don't use GPIO_PB0(pwm5) of 8266, because its default status is ouput 1 as GPIO.
#define PWM_R     GPIO_PC4
#define PWM_G     GPIO_PC0
#define PWM_B     GPIO_PC2
#define PWM_W     GPIO_PA1

#define PWMID_R     (GET_PWMID(PWM_R))
#define PWMID_G     (GET_PWMID(PWM_G))
#define PWMID_B     (GET_PWMID(PWM_B))
#define PWMID_W     (GET_PWMID(PWM_W))
                    
#define PWM_INV_R   (GET_PWM_INVERT_VAL(PWM_R))
#define PWM_INV_G   (GET_PWM_INVERT_VAL(PWM_G))
#define PWM_INV_B   (GET_PWM_INVERT_VAL(PWM_B))
#define PWM_INV_W   (GET_PWM_INVERT_VAL(PWM_W))

//-------------------------
#define MODE_MASTER_SLAVE       1
#if (MODE_MASTER_SLAVE)
#define	GPIO_BUTTON_MASTER      GPIO_PD4			//SW2/BTN2
#define	GPIO_BUTTON_SLAVE       GPIO_PD5			//SW1/BTN1
#endif

#define HCI_ACCESS		        HCI_USE_USB

#define GATEWAY_EN              1
#if (GATEWAY_EN)
#define SWITCH_MODE_BUTTON1     0   // please define this pin if call proc_ui_gateway ()
#define SWITCH_MODE_BUTTON2     0   // please define this pin if call proc_ui_gateway ()
#endif

#if (HCI_ACCESS==HCI_USE_UART)
#define UART_ENABLE             1
#define UART_GPIO_SEL           UART_GPIO_8266
#define UART_FIFO_ENABLE        1
#endif

#define ALARM_EN                0
#define SCENE_EN                0

#define SYNC_TIME_EN            0
#if (SYNC_TIME_EN)
#define SYNC_LIGHT_ONOFF_EN     1
#define SYNC_ALTER_TIME         (1000 * CLOCK_SYS_CLOCK_1MS)
#define SYNC_CMD_DELAY_TIME_MS  (400) // unit: ms.   shoule not be change, and just for sync command not for reset command from app
#define SYNC_TIME               (10*60)    //(10)// unit: 1s
#define SYNC_TIME2ALTERS        ((SYNC_TIME * (CLOCK_SYS_CLOCK_1S/CLOCK_SYS_CLOCK_1MS)) \
                                / (SYNC_ALTER_TIME/CLOCK_SYS_CLOCK_1MS))    // unit: alter
#endif

#include "../common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
