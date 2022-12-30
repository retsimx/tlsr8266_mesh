/********************************************************************************************************
 * @file     light.h 
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

#define CHIP_TYPE				CHIP_TYPE_8269		// 8866-24, 8566-32

#define	RF_FAST_MODE_1M			1					// BLE mode

#define	PM_PIN_PULL_DEFAULT		1

#define PULL_WAKEUP_SRC_PA0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA7           PM_PIN_PULL_DEFAULT  //SWM
#define PULL_WAKEUP_SRC_PB0           PM_PIN_PULL_DEFAULT  //SWS
#define PULL_WAKEUP_SRC_PB1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB4           0 // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PB5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB6           0 // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PB7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC2           0 // LIGHT_PWM must 0(float) or 3(pull_down_100K )
#define PULL_WAKEUP_SRC_PC3           0 // LIGHT_PWM must 0(float) or 3(pull_down_100K )
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
#define PULL_WAKEUP_SRC_PE2           PM_PIN_PULL_DEFAULT  //DM
#define PULL_WAKEUP_SRC_PE3           PM_PIN_PULL_DEFAULT  //DP
#define PULL_WAKEUP_SRC_PE4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PF0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PF1           PM_PIN_PULL_DEFAULT

//open SWS output to avoid MCU err
#define PB0_DATA_OUT				  1				

#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ  	32000000

#define MODULE_WATCHDOG_ENABLE	1
#define WATCHDOG_INIT_TIMEOUT   2000 //ms

#define STACK_CHECK_ENABLE      0
#define BQB_EN                  0

#define ADC_SET_CHN_ENABLE		0
#if ADC_SET_CHN_ENABLE
#define	ADC_CHNM_ANA_INPUT_8269	B7
#define ADC_CHNM_REF_SRC_8269	RV_AVDD

#define PB7_INPUT_ENABLE		1
#define PB7_OUTPUT_ENABLE		0
#define PB7_FUNC				AS_GPIO
#define PB7_DATA_OUT			0

#ifdef PULL_WAKEUP_SRC_PB7
#undef PULL_WAKEUP_SRC_PB7
#endif

#define PULL_WAKEUP_SRC_PB7		0   // 0 : float
#endif

#define ADC_ENABLE 			0
#if ADC_ENABLE
//#define BAT_DET_PIN 		GPIO_PB7

//#define LOW_LEVL_MV			2100//2100mv as the low level warning 
#endif

#define ADC_TEMP_ENABLE				1

#define PWM_R     GPIO_PC2			//red
#define PWM_G     GPIO_PC3			//green
#define PWM_B     GPIO_PB6			//blue
#define PWM_W     GPIO_PB4			//while

#define PWMID_R     2
#define PWMID_G     3
#define PWMID_B     5
#define PWMID_W     4

#define GATEWAY_EN              0
#if (GATEWAY_EN)
#define SWITCH_MODE_BUTTON1     GPIO_PC5
#define SWITCH_MODE_BUTTON2     GPIO_PD2
#endif

#define ALARM_EN                1
#define SCENE_EN                1
#define MESH_OTA_MASTER_FLAG_EN 1

#define SYNC_TIME_EN            0
#if (SYNC_TIME_EN)
#define SYNC_LIGHT_ONOFF_EN     1
#define SYNC_ALTER_TIME         (1000 * CLOCK_SYS_CLOCK_1MS)
#define SYNC_CMD_DELAY_TIME_MS  (400) // unit: ms.   shoule not be change, and just for sync command not for reset command from app
#define SYNC_TIME               (10*60)    //(10)// unit: 1s
#define SYNC_TIME2ALTERS        ((SYNC_TIME * (CLOCK_SYS_CLOCK_1S/CLOCK_SYS_CLOCK_1MS)) \
                                / (SYNC_ALTER_TIME/CLOCK_SYS_CLOCK_1MS))    // unit: alter
#endif

#define I2C_HW_MODOULE_EN       0
#if I2C_HW_MODOULE_EN
#define I2C_USE_SIMULATION	    0

#define PIN_I2C_SDA              GPIO_PA3    // GPIO_PC0
#define PIN_I2C_SCL              GPIO_PA4    // GPIO_PC1

#define I2C_SLAVE_GPIO_REQ_ENABLE       (0)
#if (I2C_SLAVE_GPIO_REQ_ENABLE)
#define PIN_I2C_SLAVE_DATA_READY        (GPIO_PD3)
#define SLAVE_DATA_READY_LEVEL     		(0)
#endif

#define I2C_MASTER_MODE         1
#if (I2C_MASTER_MODE)
#define I2C_IRQ_EN              0
#else
#define I2C_IRQ_EN              1
#endif
#endif

#define FEATURE_FRIEND_EN       0
#if FEATURE_FRIEND_EN
#define FN_DEBUG_PIN_EN         0
#endif

#define DUAL_MODE_ADAPT_EN 	    0

#define NOTIFY_MESH_COMMAND_TO_MASTER_EN        0

#if NOTIFY_MESH_COMMAND_TO_MASTER_EN
#define NOTIFY_MESH_FIFO_EN     1       // should be 1
#define NOTIFY_MESH_FIFO_CNT    (32)
#else
#define NOTIFY_MESH_FIFO_EN     1
#define NOTIFY_MESH_FIFO_CNT    (32)
#endif

#define SUB_ADDR_EN             0

#include "../common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
