/********************************************************************************************************
 * @file     clock.h 
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

#ifdef WIN32
#include <time.h>
#endif
#include "compiler.h"
// constants
// system clock

//  must use macro,  because used in #if
#define	CLOCK_TYPE_PLL	0
#define	CLOCK_TYPE_OSC	1
#define	CLOCK_TYPE_PAD	2
#define	CLOCK_TYPE_ADC	3

enum{
	CLOCK_SEL_32M_RC = 	0,
	CLOCK_SEL_HS_DIV = 	1,
	CLOCK_SEL_16M_PAD =	2,
	CLOCK_SEL_32M_PAD =	3,
	CLOCK_SEL_SPI  	  = 4,
	CLOCK_SEL_40M_INTERNAL = 5,
	CLOCK_SEL_32K_RC  =	6,
};

enum{
	CLOCK_HS_240M_PLL =	0,
	CLOCK_HS_40M_ADC = 	1,
	CLOCK_HS_32M_OSC =	2,
	CLOCK_HS_16M_OSC = 	3,
};

#if ((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
#define CLOCK_SYS_CLOCK_1S		CLOCK_16M_SYS_TIMER_CLK_1S
#define CLOCK_SYS_CLOCK_1MS		CLOCK_16M_SYS_TIMER_CLK_1MS
#define CLOCK_SYS_CLOCK_1US		CLOCK_16M_SYS_TIMER_CLK_1US
#else
enum{
	CLOCK_PLL_CLOCK = 192000000,

	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
	CLOCK_SYS_CLOCK_4S = CLOCK_SYS_CLOCK_1S << 2,
	CLOCK_MAX_MS = (U32_MAX / CLOCK_SYS_CLOCK_1MS),
	CLOCK_MAX_US = (U32_MAX / CLOCK_SYS_CLOCK_1US),
};
#endif

enum{
	CLOCK_MCU_RUN_CODE_1S = CLOCK_SYS_CLOCK_HZ,         // 8258: not same with CLOCK_SYS_CLOCK_1S.
	CLOCK_MCU_RUN_CODE_1MS = (CLOCK_MCU_RUN_CODE_1S / 1000),
	CLOCK_MCU_RUN_CODE_1US = (CLOCK_MCU_RUN_CODE_1S / 1000000),
};

enum{
	CLOCK_MODE_SCLK = 0,
	CLOCK_MODE_GPIO = 1,
	CLOCK_MODE_WIDTH_GPI = 2,
	CLOCK_MODE_TICK = 3
};

#if(CLOCK_SYS_TYPE == CLOCK_TYPE_PLL)
	#define CLK_FHS_MZ		192
#elif(CLOCK_SYS_TYPE == CLOCK_TYPE_PAD)
	#if(CLOCK_SYS_CLOCK_HZ == 32000000)
		#define CLK_FHS_MZ		32
	#elif(CLOCK_SYS_CLOCK_HZ == 12000000)
		#define CLK_FHS_MZ		12
	#else
		#error
	#endif
#elif(CLOCK_SYS_TYPE == CLOCK_TYPE_OSC)
	#if(CLOCK_SYS_CLOCK_HZ == 32000000)
		#define CLK_FHS_MZ		192
	#elif(CLOCK_SYS_CLOCK_HZ == 16000000)
		#define CLK_FHS_MZ		32			//  DIVIDE == 2,  32/2 = 16, see reg 0x66
	#elif(CLOCK_SYS_CLOCK_HZ == 8000000)
		#define CLK_FHS_MZ		32			//  DIVIDE == 2,  32/2 = 16, see reg 0x66
	#else
		#error
	#endif
#else
		#error
#endif

typedef enum{
	SYS_CLK_12M_Crystal = 0x44,
	SYS_CLK_16M_Crystal = 0x43,
	SYS_CLK_24M_Crystal = 0x42,
	SYS_CLK_32M_Crystal = 0x60,
	SYS_CLK_48M_Crystal = 0x20,
	SYS_CLK_24M_RC      = 0x00,
}SYS_CLK_TYPEDEF, SYS_CLK_TypeDef;

//#define clock_init(sys_clk) 	   ( reg_clk_sel = (sys_clk) )

#if(MCU_CORE_TYPE == MCU_CORE_8258)
void clock_init(SYS_CLK_TYPEDEF SYS_CLK);
#elif(__TL_LIB_8278__ || MCU_CORE_TYPE == MCU_CORE_8278)
/**
 * @brief 	Power type for different application
 */
typedef enum{
	LDO_MODE 		=0x40,	//LDO mode
	DCDC_MODE		=0x43,	//DCDC mode (16pin is not suported this mode.)
	DCDC_LDO_MODE	=0x41,	//DCDC_LDO mode (synchronize mode,Use the asynchronize 
								//mode with DCDC_LDO may cause the current abnormal(A0 version))
}POWER_MODE_TypeDef;
/**
 * @brief 	crystal for different application
 */
typedef enum{
	EXTERNAL_XTAL_24M	= 0,
	EXTERNAL_XTAL_48M	= 1,
}XTAL_TypeDef;

/**
 * @brief 32K clock type.
 */

typedef enum{
	CLK_32K_RC   =0,
	CLK_32K_XTAL =1,
}CLK_32K_TypeDef;

/**
 * @brief       This function to select the system clock source.
 * @param[in]   SYS_CLK - the clock source of the system clock.
 * @return      none
 */
void clock_init(SYS_CLK_TypeDef SYS_CLK);
#else
void clock_init();
#endif
_attribute_ram_code_ void sleep_us (u32 microsec);		//  use register counter to delay 

static inline void delay(int us){						// use no register counter to delay 
	for(volatile int i = 0; i < us * CLOCK_SYS_CLOCK_HZ / (1000*1000); ++i){
	}
}
enum{
	CLOCK_16M_SYS_TIMER_CLK_1S =  16000000,
	CLOCK_16M_SYS_TIMER_CLK_1MS = 16000,
	CLOCK_16M_SYS_TIMER_CLK_1US = 16,
};

//  delay precisely
#define		CLOCK_DLY_1_CYC    _ASM_NOP_
#define		CLOCK_DLY_2_CYC    _ASM_NOP_;_ASM_NOP_
#define		CLOCK_DLY_3_CYC    _ASM_NOP_;_ASM_NOP_;_ASM_NOP_
#define		CLOCK_DLY_4_CYC    _ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_
#define		CLOCK_DLY_5_CYC    _ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_
#define		CLOCK_DLY_6_CYC    _ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_
#define		CLOCK_DLY_7_CYC    _ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_
#define		CLOCK_DLY_8_CYC    _ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_
#define		CLOCK_DLY_9_CYC    _ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_
#define		CLOCK_DLY_10_CYC   _ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_;_ASM_NOP_

#if (CLOCK_SYS_CLOCK_HZ == 30000000 || CLOCK_SYS_CLOCK_HZ == 32000000)
	#define		CLOCK_DLY_100NS		CLOCK_DLY_3_CYC							// 100,  94
	#define		CLOCK_DLY_200NS		CLOCK_DLY_6_CYC							// 200, 188
	#define 	CLOCK_DLY_600NS 	CLOCK_DLY_10_CYC;CLOCK_DLY_10_CYC 		// 200, 188
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
	#define 	CLOCK_DLY_63NS 		CLOCK_DLY_3_CYC 		//  63 ns
	#define		CLOCK_DLY_100NS		CLOCK_DLY_4_CYC			//  100 ns
	#define		CLOCK_DLY_200NS		CLOCK_DLY_8_CYC			//  200 ns
	#define 	CLOCK_DLY_600NS 	CLOCK_DLY_10_CYC 		//	600 ns
#elif (CLOCK_SYS_CLOCK_HZ == 12000000 || CLOCK_SYS_CLOCK_HZ == 16000000)
	#define 	CLOCK_DLY_63NS 		CLOCK_DLY_1_CYC 		//  63 ns
	#define		CLOCK_DLY_100NS		CLOCK_DLY_2_CYC			//  128 ns
	#define		CLOCK_DLY_200NS		CLOCK_DLY_4_CYC			//  253 ns
	#define 	CLOCK_DLY_600NS 	CLOCK_DLY_10_CYC 		//	253 ns
#elif (CLOCK_SYS_CLOCK_HZ == 48000000)
	#define		CLOCK_DLY_100NS		CLOCK_DLY_5_CYC			// 104
	#define		CLOCK_DLY_200NS		CLOCK_DLY_10_CYC		// 208
	#define 	CLOCK_DLY_600NS 	CLOCK_DLY_10_CYC;CLOCK_DLY_10_CYC;CLOCK_DLY_10_CYC		//	600 ns
#elif (CLOCK_SYS_CLOCK_HZ == 6000000 || CLOCK_SYS_CLOCK_HZ == 8000000)
	#define		CLOCK_DLY_100NS		CLOCK_DLY_1_CYC			//  125 ns
	#define		CLOCK_DLY_200NS		CLOCK_DLY_2_CYC			//  250
	#define 	CLOCK_DLY_600NS 	CLOCK_DLY_5_CYC 		//  725
#else
	#error clock not set properly
#endif

