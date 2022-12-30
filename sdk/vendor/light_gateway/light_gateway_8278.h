/********************************************************************************************************
 * @file     light_gateway_8258.h 
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

#define CHIP_TYPE				CHIP_TYPE_8278		// 8866-24, 8566-32

#define	RF_FAST_MODE_1M			1					// BLE mode

#define	PM_PIN_PULL_DEFAULT		1
#define	PM_PIN_PULL_PWM_LED		0
#if 1
#define PULL_WAKEUP_SRC_PA0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA2           PM_PIN_PULL_PWM_LED // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PA3           PM_PIN_PULL_PWM_LED // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PA4           PM_PIN_PULL_PWM_LED // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PA5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA7           PM_PIN_PULL_DEFAULT  //SWS
#define PULL_WAKEUP_SRC_PB0           PM_PIN_PULL_PWM_LED // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PB1           PM_PIN_PULL_PWM_LED // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PB2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB6           PM_PIN_PULL_DEFAULT
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
#endif

#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ  	16000000

#define MODULE_WATCHDOG_ENABLE	1
#define WATCHDOG_INIT_TIMEOUT   2000 //ms

#define STACK_CHECK_ENABLE      0
#define	MY_RF_POWER_INDEX		RF_POWER_P3p50dBm

#define BATT_CHECK_ENABLE       			1   //must enable
#if (BATT_CHECK_ENABLE)
//telink device: you must choose one gpio with adc function to output high level(voltage will equal to vbat), then use adc to measure high level voltage
	//use PC5 output high level, then adc measure this high level voltage
	#define GPIO_VBAT_DETECT				GPIO_PC5
	#define PC5_FUNC						AS_GPIO
	#define PC5_INPUT_ENABLE				0
	#define ADC_INPUT_PCHN					C5P    //corresponding  ADC_InputPchTypeDef in adc.h
#endif

#define ADC_SET_CHN_ENABLE		0
#if ADC_SET_CHN_ENABLE
#define ADC_BASE_MODE	1	//GPIO voltage
#define ADC_VBAT_MODE	2	//Battery Voltage

#define ADC_MODE		ADC_BASE_MODE
#define ADC_CHNM_ANA_INPUT 		GPIO_PB4 // one of ADC_GPIO_tab[]
#define ADC_PRESCALER	ADC_PRESCALER_1F4
#endif




//-------------------------PWM
// GPIO_PC2 / GPIO_PC3 of 8267 dongle PCB was used as UART.

#define PWM_R       GPIO_PWM1A3		//red
#define PWM_G       GPIO_PWM0A2		//green
#define PWM_B       GPIO_PWM3B0		//blue
#define PWM_W       GPIO_PWM4B1		//white
#define PWM_Y       GPIO_PWM2A4		//yellow

#define PWM_FUNC_R  AS_PWM  // AS_PWM_SECOND
#define PWM_FUNC_G  AS_PWM  // AS_PWM_SECOND
#define PWM_FUNC_B  AS_PWM  // AS_PWM_SECOND
#define PWM_FUNC_W  AS_PWM  // AS_PWM_SECOND

#define PWMID_R     (GET_PWMID(PWM_R, PWM_FUNC_R))
#define PWMID_G     (GET_PWMID(PWM_G, PWM_FUNC_G))
#define PWMID_B     (GET_PWMID(PWM_B, PWM_FUNC_B))
#define PWMID_W     (GET_PWMID(PWM_W, PWM_FUNC_W))
                    
#define PWM_INV_R   (GET_PWM_INVERT_VAL(PWM_R, PWM_FUNC_R))
#define PWM_INV_G   (GET_PWM_INVERT_VAL(PWM_G, PWM_FUNC_G))
#define PWM_INV_B   (GET_PWM_INVERT_VAL(PWM_B, PWM_FUNC_B))
#define PWM_INV_W   (GET_PWM_INVERT_VAL(PWM_W, PWM_FUNC_W))

//-------------------------
#define PROVISIONING_ENABLE                 0
#if (PROVISIONING_ENABLE)
#define HCI_ACCESS		        HCI_USE_UART
#else
#define HCI_ACCESS		        HCI_USE_USB
#endif

#define GATEWAY_EN              1
#if (GATEWAY_EN)
#define SWITCH_MODE_BUTTON1     GPIO_PD6
#define SWITCH_MODE_BUTTON2     GPIO_PD5
#define PD6_INPUT_ENABLE		1
#define PD6_OUTPUT_ENABLE		0
#define PD6_FUNC				AS_GPIO
#define PD6_DATA_OUT			1
#define PD5_INPUT_ENABLE		1
#define PD5_OUTPUT_ENABLE		0
#define PD5_FUNC				AS_GPIO
#define PD5_DATA_OUT			1
#endif

#if (HCI_ACCESS==HCI_USE_UART)
#define UART_ENABLE             1
#define UART_TX_PIN		UART_TX_PB1
#define UART_RX_PIN 	UART_RX_PB0
#elif(HCI_ACCESS==HCI_USE_USB)
#define PA5_INPUT_ENABLE	1	//USB
#define PA6_INPUT_ENABLE	1   //USB
#define PA5_FUNC			AS_USB
#define PA6_FUNC			AS_USB
#endif

#define ALARM_EN                0
#define SCENE_EN                0
#define	MESH_RSSI_RECORD_EN		0

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
