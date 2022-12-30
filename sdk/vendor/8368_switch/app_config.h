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

#define _USER_CONFIG_DEFINED_   1   // must define this macro to make others known
#define __LOG_RT_ENABLE__       0

//////////// product  Information  //////////////////////////////
#define ID_VENDOR               0x248a          // for report
#define ID_PRODUCT_BASE         0x880C
// If ID_PRODUCT left undefined, it will default to be combination of ID_PRODUCT_BASE and the USB config USB_SPEAKER_ENABLE/USB_MIC_ENABLE...
// #define ID_PRODUCT           0x8869

#define STRING_VENDOR           L"Telink"
#define STRING_PRODUCT          L"Mesh Switch"
#define STRING_SERIAL           L"TLSR8368"

#define CHIP_TYPE               CHIP_TYPE_8368      // 8866-24, 8566-32

#define FLOW_NO_OS              1
#define RF_FAST_MODE_1M     	1

/////////////////// Clock  /////////////////////////////////
//#define CLOCK_SYS_TYPE          CLOCK_TYPE_PLL  //  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_TYPE          CLOCK_TYPE_OSC  //  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ      16000000

//////////////////Extern Crystal Type///////////////////////
#define CRYSTAL_TYPE			XTAL_12M		//  extern 16M crystal

////////////////Moudule Enable  ////////////////////////////
#define MODULE_ADC_ENABLE		0


//extern definition
#define LED_USE_PWM						1


#define MSDI_DATA_OUT 			1
#define MSDO_DATA_OUT 			1
//////////// EEPROM //////////////////////////////////////////
#define		PIN_I2C_SCL			GPIO_GP30			//SCL
#define		PIN_I2C_SDA			GPIO_GP31			//SDA

///////////////////////////// GPIO Setting initial///////////////////////////////////
#define PM_PIN_PULL_DEFAULT     1

#define PULL_WAKEUP_SRC_GPIO0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO3           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_GPIO4           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_GPIO5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO6           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_GPIO7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO8           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO9           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_GPIO10           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO11           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO12           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO13           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO14           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO15           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO16           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_GPIO17           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO19           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO20           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO21           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_GPIO22           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_GPIO23           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO24           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO25           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO26           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO27           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO28           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO29           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO30           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO31           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_GPIO32           PM_PIN_PULL_DEFAULT


//GP7 LED
#define	PULL_WAKEUP_SRC_GPIO7			0// PM_PIN_PULLDOWN_100K
#define GPIO7_DATA_OUT					0			// valid level:low
#define GPIO7_INPUT_ENABLE 				1
#define GPIO7_DATA_STRENGTH 			0
#define GPIO7_OUTPUT_ENABLE 			1
#define GPIO_LED						GPIO_GP7
#define PWM_ID_LED  				PWM0


/*Button GPIO17*/
#define	PULL_WAKEUP_SRC_GPIO17		PM_PIN_PULLDOWN_100K
#define GPIO17_DATA_OUT					0			// valid level:low
#define GPIO17_INPUT_ENABLE 			1
#define GPIO17_OUTPUT_ENABLE 			0

/*Button GPIO18*/
#define	PULL_WAKEUP_SRC_GPIO18		PM_PIN_PULLDOWN_100K
#define GPIO18_DATA_OUT				0			// valid level:low
#define GPIO18_INPUT_ENABLE 		1
#define GPIO18_OUTPUT_ENABLE 		0

/*Button GPIO22*/
#define	PULL_WAKEUP_SRC_GPIO22		PM_PIN_PULLDOWN_100K
#define GPIO22_DATA_OUT				0			// valid level:low
#define GPIO22_INPUT_ENABLE 		1
#define GPIO22_OUTPUT_ENABLE 		0

/*Button GPIO23*/
#define	PULL_WAKEUP_SRC_GPIO23		PM_PIN_PULLUP_1M
#define GPIO23_DATA_OUT				0			// valid level:low
#define GPIO23_INPUT_ENABLE 		1
#define GPIO23_OUTPUT_ENABLE 		0


#ifndef 	DEBUG_FROM_FLASH
#define 	DEBUG_FROM_FLASH		0
#endif

#if(DEBUG_FROM_FLASH)   // firmware in flash (flash max size decided by flash type)
//#define 	CFG_ADR_MAC   			0x3fe0				// this address must be out of code space
//#define		CFG_FREQUENCY_OFFSET    0x3fe8
//// gpio init , for compiler here
//
//#else					// firmware in otp(otp max size is (0x3fff - 20))
//#define 	CFG_ADR_MAC   			0x3fe0				// this address must be out of code space
//#define		CFG_FREQUENCY_OFFSET    0x3fe8				// this address must be out of code space
//
// gpio init , for compiler here
#define MSDO_INPUT_ENABLE 		0
#define MSDI_INPUT_ENABLE 		0
#define MCLK_INPUT_ENABLE 		0
#define MSCN_INPUT_ENABLE 		0
#endif

///////////////////  ADC  /////////////////////////////////
#if(MODULE_ADC_ENABLE)

#define ADC_CHNM_ANA_INPUT      ADC_CHN_GP18  //others parameters (ADC_CHN_GP17 ADC_CHN_GP18 ADC_CHN_GP22 ADC_CHN_GP23)
#define ADC_CHNM_REF_SRC        ADC_REF_1P3V  //others parameters (ADC_REF_1P3V ADC_REF_VDDH)

#endif


///////////////////  RAM&OTP Optimization    /////////////////////////////////
/*Disable interrupt for 8263_remote application*/
#define ENABLE_INTERRUPT  0

#if ENABLE_INTERRUPT  
#define FLASH_OP_DISABLE_IRQ 				1 /*1 Means flash operation needs to disable IRQ*/
#define ANALOG_OP_DISABLE_IRQ				1
#define KEYSCAN_OP_DISABLE_IRQ                     1
#define RING_BUF_OP_DISABLE_IRQ                   1
#define BLT_LL_OP_DISABLE_IRQ                        1

#else
#define FLASH_OP_DISABLE_IRQ 				0
#define ANALOG_OP_DISABLE_IRQ				0
#define KEYSCAN_OP_DISABLE_IRQ                     0
#define RING_BUF_OP_DISABLE_IRQ                   0
#define BLT_LL_OP_DISABLE_IRQ                        0
#endif

/*Define low resource cost event call back functions*/
#define LOW_COST_EVENT_CB_MODE  1

#define 	CFG_ADR_MAC   			0x3fe0				// this address must be out of code space


///////////////////  RAMCODE  optimize  ////////////////////////


/////////////////// set default   ////////////////

#include "../common/default_config.h"


/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
