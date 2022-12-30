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

#define CHIP_TYPE				CHIP_TYPE_8278

#define FLASH_1M_ENABLE         0
#if FLASH_1M_ENABLE
#define PINGPONG_OTA_DISABLE    0 // it can disable only when 1M flash.
#if	PINGPONG_OTA_DISABLE
#define SWITCH_FW_ENABLE		0 // set to 0, just for particular customer 
#endif
#endif

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
#define BQB_EN                  0
#define PM_DEEPSLEEP_RETENTION_ENABLE   1
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



#define PCBA_8278_DONGLE_48PIN          1
#define PCBA_8278_C1T197A30_V1_0        2
#define PCBA_SEL			    PCBA_8278_DONGLE_48PIN

#define UART_ENABLE             0 
#if(UART_ENABLE)
#if (PCBA_SEL == PCBA_8278_DONGLE_48PIN)
#define UART_TX_PIN		UART_TX_PD7
#define UART_RX_PIN 	UART_RX_PA0
#else
#define UART_TX_PIN		UART_TX_PB1
#define UART_RX_PIN 	UART_RX_PB0
#endif
#endif

//---------------  Button 
#if (PCBA_SEL == PCBA_8278_DONGLE_48PIN)
#define PD6_INPUT_ENABLE		1
#define PD5_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PD6
#define	SW2_GPIO				GPIO_PD5
#elif(PCBA_SEL == PCBA_8278_C1T197A30_V1_0)
#define PB2_INPUT_ENABLE		1
#define PB3_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PB2            // SW2 in board
#define	SW2_GPIO				GPIO_PB3            // SW4 in board
#endif


#if(PCBA_SEL == PCBA_8278_DONGLE_48PIN)
#define PWM_R       GPIO_PWM1A3		//red
#define PWM_G       GPIO_PWM0A2		//green
#define PWM_B       GPIO_PWM3B0		//blue
#define PWM_W       GPIO_PWM4B1		//white
#elif(PCBA_SEL == PCBA_8278_C1T197A30_V1_0)   // PCBA_8258_DEVELOPMENT_BOARD
#define PWM_R       GPIO_PWM1ND3	//red
#define PWM_G       GPIO_PWM2ND4	//green
#define PWM_B       GPIO_PD5		//blue
#define PWM_W       GPIO_PWM3D2		//white
#endif

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

#define GATEWAY_EN              0
#if (GATEWAY_EN)
#define SWITCH_MODE_BUTTON1     SW1_GPIO
#define SWITCH_MODE_BUTTON2     SW2_GPIO
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
#define I2C_MASTER_MODE         1
#if 0
#define I2C_USE_SIMULATION	    0

#define PIN_I2C_SDA              GPIO_PA3    // GPIO_PC0
#define PIN_I2C_SCL              GPIO_PA4    // GPIO_PC1

#define I2C_SLAVE_GPIO_REQ_ENABLE       (0)
#if (I2C_SLAVE_GPIO_REQ_ENABLE)
#define PIN_I2C_SLAVE_DATA_READY        (GPIO_PD3)
#define SLAVE_DATA_READY_LEVEL     		(0)
#endif

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
