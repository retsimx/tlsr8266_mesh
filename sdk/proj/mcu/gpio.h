/********************************************************************************************************
 * @file     gpio.h 
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


enum{
	GPIO_DIR_IN 	= 0,
	GPIO_DIR_OUT	= 1
};

#if(__TL_LIB_8258__ || __TL_LIB_8278__)
#else
#if ((MCU_CORE_TYPE != MCU_CORE_8258) && (MCU_CORE_TYPE != MCU_CORE_8278))
// do not use enum,  because we use this in preprocessor diretive,  #if
#define AS_GPIO		0
#define AS_MSPI		1
#define AS_SWIRE	2
#define AS_UART		3
#define AS_PWM		4
#define AS_I2C		5
#define AS_SPI		6
#define AS_ETH_MAC	7
#define AS_I2S		8
#define AS_SDM		9
#define AS_DMIC		10
#define AS_USB		11
#define AS_SWS		12
#define AS_SWM		13
#define AS_TEST		14
#define AS_ADC		15
#endif
#endif

#include "../common/static_assert.h"

#if(__TL_LIB_8266__ || MCU_CORE_TYPE == MCU_CORE_8266)
#include "../mcu_spec/gpio_default_8266.h"
#include "../mcu_spec/gpio_8266.h"
#elif(__TL_LIB_8267__ || MCU_CORE_TYPE == MCU_CORE_8267)
#include "../mcu_spec/gpio_default_8267.h"
#include "../mcu_spec/gpio_8267.h"
#elif(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8263)
#include "../mcu_spec/gpio_default_8263.h"
#include "../mcu_spec/gpio_8263.h"
#elif(__TL_LIB_8366__ || MCU_CORE_TYPE == MCU_CORE_8366)
#include "../mcu_spec/gpio_default_8366.h"
#include "../mcu_spec/gpio_8366.h"
#elif(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8368)
#include "../mcu_spec/gpio_default_8368.h"
#include "../mcu_spec/gpio_8368.h"
#elif(__TL_LIB_8269__ || MCU_CORE_TYPE == MCU_CORE_8269)
#include "../mcu_spec/gpio_default_8269.h"
#include "../mcu_spec/gpio_8269.h"
#elif(__TL_LIB_8258__ || MCU_CORE_TYPE == MCU_CORE_8258)
#include "../mcu_spec/gpio_default_8258.h"
#include "../mcu_spec/gpio_8258.h"
#elif(__TL_LIB_8278__ || MCU_CORE_TYPE == MCU_CORE_8278)
#include "../mcu_spec/gpio_default_8278.h"
#include "../mcu_spec/gpio_8278.h"
#endif

