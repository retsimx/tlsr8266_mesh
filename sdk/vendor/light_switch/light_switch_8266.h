/********************************************************************************************************
 * @file     light_switch_8266.h 
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

#define _USER_CONFIG_DEFINED_	1	// must define this macro to make others known
#define	__LOG_RT_ENABLE__		0

#define CHIP_TYPE				CHIP_TYPE_8266		// 8866-24, 8566-32

#define	RF_FAST_MODE_1M			1					// BLE mode

#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ  	32000000

#define MODULE_WATCHDOG_ENABLE	1
#define WATCHDOG_INIT_TIMEOUT	2000		//  in ms

#define STACK_CHECK_ENABLE	    0
#define	DEEP_SLEEP_EN			1

#define PA0_DATA_OUT 			1   //sws pullup: output high, output disable
#define IC_8266F512			    1

#define 	SW_GRP_SIZE 		4

#define PANEL_ENABLE	        0
#if(PANEL_ENABLE)//i2c
#define PE7_OUTPUT_ENABLE		1
#define PE7_INPUT_ENABLE		1
#define PE7_DATA_OUT			0

#define PF1_INPUT_ENABLE		1
#define PF1_DATA_OUT			0

#define I2C_WAKUP_PIN			GPIO_PA1

#define PANEL_LED		        GPIO_PA5
#define	GPIO_LED			    GPIO_PA5			//PWM1->PD5 not pwm
#define	PWMID_LED			    0//invalid
#else

#define	GPIO_LED			GPIO_PD5			//PWM1->PD5 not pwm
#define	PWMID_LED			0//invalid

#define	RC_KEY_1_ON			VK_A
#define	RC_KEY_1_OFF		VK_B
#define	RC_KEY_2_ON			VK_C
#define	RC_KEY_2_OFF		VK_D
#define	RC_KEY_3_ON			VK_E
#define	RC_KEY_3_OFF		VK_F
#define	RC_KEY_4_ON			VK_G
#define	RC_KEY_4_OFF		VK_H
#define	RC_KEY_A_ON			VK_I
#define	RC_KEY_A_OFF		VK_J
#define	RC_KEY_UP			VK_UP
#define	RC_KEY_DN			VK_DOWN
#define	RC_KEY_L			VK_LEFT
#define	RC_KEY_R			VK_RIGHT
#define	RC_KEY_M			VK_O//NULL

#define	KB_MAP_NORMAL	{\
		{RC_KEY_1_OFF,		RC_KEY_2_OFF,	  	RC_KEY_1_ON}, \
		{RC_KEY_3_ON,		RC_KEY_3_OFF,		RC_KEY_2_ON}, \
		{RC_KEY_4_ON,		RC_KEY_4_OFF,		RC_KEY_R}, \
		{RC_KEY_A_OFF,		RC_KEY_A_ON,		RC_KEY_UP}, \
		{RC_KEY_L,			RC_KEY_DN,			RC_KEY_M}, }

#define		KB_MAP_NUM		KB_MAP_NORMAL
#define		KB_MAP_FN		KB_MAP_NORMAL

#define	MATRIX_ROW_PULL		PM_PIN_PULLDOWN_100K
#define	MATRIX_COL_PULL		PM_PIN_PULLUP_10K
#define	KB_LINE_HIGH_VALID	0

#define	PM_PIN_PULL_DEFAULT	0   // 0:float, reduce sleep current

#define  KB_DRIVE_PINS  {GPIO_PC2, GPIO_PC4, GPIO_PC6}
#if(IC_8266F512)
#define  KB_SCAN_PINS   {GPIO_PF1, GPIO_PF0, GPIO_PE7, GPIO_PE6, GPIO_PE5}
#else
#define  KB_SCAN_PINS   {GPIO_PF1, GPIO_PF0, GPIO_PE7, GPIO_PE6, GPIO_PE2}
#endif

#define	PULL_WAKEUP_SRC_PC2		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_PC4		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_PC6		MATRIX_ROW_PULL

#define	PULL_WAKEUP_SRC_PF1		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PF0		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PE7		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PE6		MATRIX_COL_PULL
#if(IC_8266F512)
#define	PULL_WAKEUP_SRC_PE5		MATRIX_COL_PULL
#else
#define	PULL_WAKEUP_SRC_PE2		MATRIX_COL_PULL
#endif

#define PULL_WAKEUP_SRC_PA0           PM_PIN_PULL_DEFAULT  //SWS
#define PULL_WAKEUP_SRC_PA1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA4           PM_PIN_PULL_DEFAULT			//C43 verison QFN56
#if(PANEL_ENABLE)//i2c
#define PULL_WAKEUP_SRC_PA5           0                     //for LED
#else
#define PULL_WAKEUP_SRC_PA5           PM_PIN_PULL_DEFAULT
#endif
#define PULL_WAKEUP_SRC_PA6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA7           PM_PIN_PULL_DEFAULT  //SWM
#define PULL_WAKEUP_SRC_PB0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB3           PM_PIN_PULLUP_10K			//MSCN
#define PULL_WAKEUP_SRC_PB4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB5           PM_PIN_PULL_DEFAULT  //DM
#define PULL_WAKEUP_SRC_PB6           PM_PIN_PULL_DEFAULT  //DP
#define PULL_WAKEUP_SRC_PB7           PM_PIN_PULL_DEFAULT
#if (!DEEP_SLEEP_EN)
#define PULL_WAKEUP_SRC_PC0           1                     //mic+  //PC0 should not float when suspend,if not current will be more
#define PULL_WAKEUP_SRC_PC1           1                     //mic-  //PC1 should not float when suspend,if not current will be more
#else
#define PULL_WAKEUP_SRC_PC0           PM_PIN_PULL_DEFAULT   //mic+  //PC0 should not float when suspend,if not current will be more
#define PULL_WAKEUP_SRC_PC1           PM_PIN_PULL_DEFAULT   //mic-  //PC1 should not float when suspend,if not current will be more
#endif
//#define PULL_WAKEUP_SRC_PC2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC3           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PC4           PM_PIN_PULL_DEFAULT			// bat detection
#define PULL_WAKEUP_SRC_PC5           PM_PIN_PULL_DEFAULT			// mic bias
//#define PULL_WAKEUP_SRC_PC6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD4           PM_PIN_PULL_DEFAULT
#if(PANEL_ENABLE)//i2c
#define PULL_WAKEUP_SRC_PD5           PM_PIN_PULL_DEFAULT
#else
#define PULL_WAKEUP_SRC_PD5           0                     //for LED
#endif
#define PULL_WAKEUP_SRC_PD6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD7           PM_PIN_PULL_DEFAULT				// Gyro power enable, active low
#define PULL_WAKEUP_SRC_PE0           PM_PIN_PULL_DEFAULT				//C43 verison QFN56
#define PULL_WAKEUP_SRC_PE1           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE2           PM_PIN_PULL_DEFAULT				//C43 verison QFN56
#define PULL_WAKEUP_SRC_PE3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE4           PM_PIN_PULL_DEFAULT
#if(IC_8266F512)
#define PULL_WAKEUP_SRC_PE2           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE5           PM_PIN_PULL_DEFAULT         //PE5 should not float when suspend,if not current will be more
#else
//#define PULL_WAKEUP_SRC_PE2           PM_PIN_PULL_DEFAULT				//C43 verison QFN56
#define PULL_WAKEUP_SRC_PE5           PM_PIN_PULL_DEFAULT   //PE5 should not float when suspend,if not current will be more
#endif
//#define PULL_WAKEUP_SRC_PE6           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE7           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PF0           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PF1           PM_PIN_PULL_DEFAULT
#endif

#define ADC_SET_CHN_ENABLE      0
#if ADC_SET_CHN_ENABLE
//#define CHIP_TYPE_8266_A2			1

#define	ADC_CHNM_ANA_INPUT		FLD_ADC_CHN_C7
#define ADC_CHNM_REF_SRC		ADC_REF_1_3V    // should not use AVDD for Battery supply

#define PC7_INPUT_ENABLE		1
#define PC7_OUTPUT_ENABLE		0
#define PC7_FUNC				AS_GPIO
#define PC7_DATA_OUT			0

#ifdef PULL_WAKEUP_SRC_PC7
#undef PULL_WAKEUP_SRC_PC7
#endif

#define PULL_WAKEUP_SRC_PC7		0   // 0 : float
#endif

#include "../common/default_config.h"

