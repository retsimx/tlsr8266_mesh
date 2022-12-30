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
#define	USB_PRINTER				1
//////////// product  Information  //////////////////////////////
#define ID_VENDOR				0x248a			// for report
#define ID_PRODUCT_BASE			0x820a
// If ID_PRODUCT left undefined, it will default to be combination of ID_PRODUCT_BASE and the USB config USB_SPEAKER_ENABLE/USB_MIC_ENABLE...
#define ID_PRODUCT				(0x820a + USB_PRINTER)

#define STRING_VENDOR			L"Telink"
#define STRING_PRODUCT			L"BLE Remote KMA Dongle"

#define	MASTER_DONGLE_8266		1
#define	MASTER_DONGLE_8267		2

#if (MASTER_DONGLE_TYPE_SEL == MASTER_DONGLE_8267)
#define STRING_SERIAL			L"TLSR8267"
#define CHIP_TYPE				CHIP_TYPE_8267		// 
#else
#define STRING_SERIAL			L"TLSR8266"
#define CHIP_TYPE				CHIP_TYPE_8266		// 8866-24, 8566-32
#endif

#define APPLICATION_DONGLE		0					// or else APPLICATION_DEVICE
#define	FLOW_NO_OS				1

#define	RF_FAST_MODE_1M			1					// BLE mode

/////////////////// MODULE /////////////////////////////////


/////////////////// Clock  /////////////////////////////////

#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
//#define CLOCK_SYS_TYPE  		CLOCK_TYPE_OSC	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ  	32000000

/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE	0

///////////////////  interrupt  //////////////////////////////

///////////////////  GPIO  /////////////////////////////////
//  only need to define those are not default
//  all gpios are default to be output disabled, input disabled, output to 0, output strength to 1
////////////////////////////////////////////////////////////////////////////
#define		VK_MIC				0xf5
#define		VK_LEFTB			0xf6

#define	GENERIC_RC				0
#if(GENERIC_RC)
#define	GPIO_LED				GPIO_PD5			//PWM1->PD5 not pwm
#define	PWMID_LED				0//invalid

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

#define  KB_DRIVE_PINS  {GPIO_PC2, GPIO_PC4, GPIO_PC6}
#define  KB_SCAN_PINS   {GPIO_PF1, GPIO_PF0, GPIO_PE7, GPIO_PE6, GPIO_PE2}

#define	MATRIX_ROW_PULL		PM_PIN_PULLDOWN_100K
#define	MATRIX_COL_PULL		PM_PIN_PULLUP_10K
#define	KB_LINE_HIGH_VALID	0


#define	PULL_WAKEUP_SRC_PC2		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_PC4		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_PC6		MATRIX_ROW_PULL

#define	PULL_WAKEUP_SRC_PF1		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PF0		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PE7		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PE6		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PE2		MATRIX_COL_PULL

#define	PM_PIN_PULL_DEFAULT		0

#define PULL_WAKEUP_SRC_PA0           PM_PIN_PULL_DEFAULT  //SWS
#define PULL_WAKEUP_SRC_PA1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA4           PM_PIN_PULL_DEFAULT			//C43 verison QFN56
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
#define PULL_WAKEUP_SRC_PC0           PM_PIN_PULL_DEFAULT			//mic+
#define PULL_WAKEUP_SRC_PC1           PM_PIN_PULL_DEFAULT			//mic-
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
#define PULL_WAKEUP_SRC_PD5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD7           PM_PIN_PULL_DEFAULT				// Gyro power enable, active low
#define PULL_WAKEUP_SRC_PE0           PM_PIN_PULL_DEFAULT				//C43 verison QFN56
#define PULL_WAKEUP_SRC_PE1           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE2           PM_PIN_PULL_DEFAULT				//C43 verison QFN56
#define PULL_WAKEUP_SRC_PE3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE5           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE6           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE7           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PF0           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PF1           PM_PIN_PULL_DEFAULT
#else

#define	GPIO_LED				GPIO_PC2			//PWM1
#define	PWM_LED					GPIO_PC2			//PWM1
#define	PWMID_LED				1

#define		KB_MAP_NORMAL	{\
				{VK_W_MUTE,		VK_3,	  	VK_1,		VK_MEDIA,	}, \
				{VK_2,	 		VK_LEFTB,	VK_MIC,		VK_4,	 	}, \
				{VK_RIGHT,	 	VK_NONE,	VK_ENTER,	VK_LEFT,	}, \
				{VK_W_BACK,	 	VK_NONE,	VK_DOWN,	VK_HOME,	}, \
				{VK_VOL_UP,	 	VK_NONE,	VK_MMODE,	VK_VOL_DN,	}, \
				{VK_WEB,		VK_NONE,	VK_UP,		VK_POWER,	}, }

#define		KB_MAP_NUM		KB_MAP_NORMAL
#define		KB_MAP_FN		KB_MAP_NORMAL

#if 1

#define  KB_DRIVE_PINS  {GPIO_PA1, GPIO_PA5, GPIO_PA6, GPIO_PA7}
#define  KB_SCAN_PINS   {GPIO_PB7, GPIO_PC6, GPIO_PE5, GPIO_PE6, GPIO_PF0, GPIO_PE4}

#else			//C43 version QFN56

#define  KB_DRIVE_PINS  {GPIO_PA1, GPIO_PA4, GPIO_PA5, GPIO_PA6}
#define  KB_SCAN_PINS   {GPIO_PB7, GPIO_PC6, GPIO_PE3, GPIO_PE6, GPIO_PF0, GPIO_PE0}

#endif

#define	MATRIX_ROW_PULL		PM_PIN_PULLDOWN_100K
#define	MATRIX_COL_PULL		PM_PIN_PULLUP_10K
#define	KB_LINE_HIGH_VALID	0

#define	PULL_WAKEUP_SRC_PA1		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_PA4		MATRIX_ROW_PULL				//C43 verison QFN56
#define	PULL_WAKEUP_SRC_PA5		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_PA6		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_PA7		MATRIX_ROW_PULL

#define	PULL_WAKEUP_SRC_PB7		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PC6		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PE5		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PE6		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PF0		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_PE4		MATRIX_COL_PULL

#define	PULL_WAKEUP_SRC_PE0		MATRIX_COL_PULL			//C43 verison QFN56
#define	PULL_WAKEUP_SRC_PE3		MATRIX_COL_PULL			//C43 verison QFN56

#define	PA7_FUNC				AS_GPIO

#define	PM_PIN_PULL_DEFAULT		0

#define PULL_WAKEUP_SRC_PA0           PM_PIN_PULL_DEFAULT  //SWS
//#define PULL_WAKEUP_SRC_PA1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PA3           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PA4           PM_PIN_PULL_DEFAULT			//C43 verison QFN56
//#define PULL_WAKEUP_SRC_PA5           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PA6           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PA7           PM_PIN_PULL_DEFAULT  //SWM
#define PULL_WAKEUP_SRC_PB0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PB4           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PB5           PM_PIN_PULL_DEFAULT  //DM
//#define PULL_WAKEUP_SRC_PB6           PM_PIN_PULL_DEFAULT  //DP
//#define PULL_WAKEUP_SRC_PB7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC0           PM_PIN_PULL_DEFAULT			//mic+
#define PULL_WAKEUP_SRC_PC1           PM_PIN_PULL_DEFAULT			//mic-
#define PULL_WAKEUP_SRC_PC2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC4           PM_PIN_PULL_DEFAULT			// bat detection
#define PULL_WAKEUP_SRC_PC5           PM_PIN_PULL_DEFAULT			// mic bias
//#define PULL_WAKEUP_SRC_PC6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PC7           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD2           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD3           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD4           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD5           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PD7           PM_PIN_PULLUP_1M				// Gyro power enable, active low
//#define PULL_WAKEUP_SRC_PE0           PM_PIN_PULL_DEFAULT				//C43 verison QFN56
#define PULL_WAKEUP_SRC_PE1           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE2           PM_PIN_PULL_DEFAULT				//C43 verison QFN56
//#define PULL_WAKEUP_SRC_PE3           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE4           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE5           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PE6           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PE7           PM_PIN_PULL_DEFAULT
//#define PULL_WAKEUP_SRC_PF0           PM_PIN_PULL_DEFAULT
#define PULL_WAKEUP_SRC_PF1           PM_PIN_PULL_DEFAULT
#endif
///////////////////  ADC  /////////////////////////////////


///////////////////  POWER MANAGEMENT  //////////////////
#define PM_USB_WAKEUP_TIME  			15 		// in ms
#define PM_SUSPEND_WAKEUP_TIME  		10000	// in ms
#define PM_ENTER_SUSPEND_TIME  			4000	// in ms
#define PM_ENTER_DEEPSLEEP_TIME			600000	// in ms: 10 minute, motion,L M R wakeup
#define PM_SUSPEND_WAKEUP_FUNC_PIN 		0
#define PM_SUSPEND_WAKEUP_FUNC_LEVEL 	0

/*
 the should be the combination of the followings:
 DEEPSLEEP_WAKEUP_PIN_GPIO0 to DEEPSLEEP_WAKEUP_PIN_GPIO3
 DEEPSLEEP_WAKEUP_PIN_ANA01 to DEEPSLEEP_WAKEUP_PIN_ANA12
 */
#define PM_SUSPEND_WAKEUP_GPIO_PIN  	0
#define PM_SUSPEND_WAKEUP_GPIO_LEVEL  	0
#define PM_DEEPSLEEP_WAKEUP_PIN 		0
#define PM_DEEPSLEEP_WAKEUP_LEVEL 		0xf0
#define PM_SEARCH_TIMEOUT				3800


///////////////////  USB   /////////////////////////////////
#if(APPLICATION_DONGLE)
#define	USB_PRINTER_ENABLE 		USB_PRINTER	//
#define	USB_SPEAKER_ENABLE 		0
#define	USB_MIC_ENABLE 			0
#define	USB_MOUSE_ENABLE 		0
#define	USB_KEYBOARD_ENABLE 	0
#define	USB_SOMATIC_ENABLE      0   //  when USB_SOMATIC_ENABLE, USB_EDP_PRINTER_OUT disable
#define USB_CUSTOM_HID_REPORT	0
#endif

//////////////////// Audio /////////////////////////////////////
#define MIC_RESOLUTION_BIT		16
#define MIC_SAMPLE_RATE			8000
#define MIC_CHANNLE_COUNT		1
#define	MIC_ENOCDER_ENABLE		0

//#define	TL_MIC_BUFFER_SIZE				1024
//#define	TL_SDM_BUFFER_SIZE				1024
/////////////////// set default   ////////////////

#if (MASTER_DONGLE_TYPE_SEL == MASTER_DONGLE_8267)
#define PWM_R     GPIO_PC2			//red
#define PWM_G     GPIO_PC3			//green
#define PWM_B     GPIO_PB6			//blue
#define PWM_W     GPIO_PB4			//while

#define PWMID_R     2
#define PWMID_G     3
#define PWMID_B     5
#define PWMID_W     4
#else	// (MASTER_DONGLE_TYPE_SEL == MASTER_DONGLE_8266)
#define PWM_R     GPIO_PC4
#define PWM_G     GPIO_PC0
#define PWM_B     GPIO_PC2
#define PWM_W     GPIO_PA1

#define PWMID_R     2
#define PWMID_G     0
#define PWMID_B     1
#define PWMID_W     3
#endif


#include "../common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
