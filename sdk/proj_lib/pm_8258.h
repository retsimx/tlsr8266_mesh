/********************************************************************************************************
 * @file     pm_8258.h 
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

#include "../proj/mcu/gpio.h"
#include "../proj/common/compatibility.h"

#if(__TL_LIB_8258__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8258))

static inline void usb_dp_pullup_en (int en)
{
	unsigned char dat = ReadAnalogReg(0x0b);
	if (en) {
		dat = dat | BIT(7);
	}
	else
	{
		dat = dat & 0x7f ;
	}

	WriteAnalogReg (0x0b, dat);
}





//analog register below can store infomation when MCU in deepsleep mode
//store your information in these ana_regs before deepsleep by calling analog_write function
//when MCU wakeup from deepsleep, read the information by by calling analog_read function

//these five below are stable
#define DEEP_ANA_REG0    0x3a
#define DEEP_ANA_REG1    0x3b
#define DEEP_ANA_REG2    0x3c


//these analog register below may have some problem when user enter deepsleep but ERR wakeup
// for example, when set a GPIO PAD high wakeup deepsleep, but this gpio is high before
// you call func cpu_sleep_wakeup, then deepsleep will be ERR wakeup, these analog register
//   infomation loss.
#define DEEP_ANA_REG6    0x35
#define DEEP_ANA_REG7    0x36
#define DEEP_ANA_REG8    0x37
#define DEEP_ANA_REG9    0x38
#define DEEP_ANA_REG10   0x39


#define ADV_DEEP_FLG	 0x01
#define CONN_DEEP_FLG	 0x02



typedef enum {
	SUSPEND_MODE						= 0,
	DEEPSLEEP_MODE						= 1,    // init all SRAM include retention
    DEEPSLEEP_MODE_RET_SRAM             = 2,    // modify by qfshang, because user no need care retention size. just auto setting by blc_pm_setDeepsleepRetentionType.
}SleepMode_TypeDef;


typedef enum {	
	DEEPSLEEP_MODE_RET_SRAM_LOW16K  	= 0x43,
	DEEPSLEEP_MODE_RET_SRAM_LOW32K  	= 0x07,
}SleepMode_SetPar;



//set wakeup source
typedef enum {
	 PM_WAKEUP_PAD   = BIT(4),    		SUSPENDWAKEUP_SRC_PAD = BIT(4),    DEEPWAKEUP_SRC_PAD   = BIT(4),
	 PM_WAKEUP_CORE  = BIT(5),
	 PM_WAKEUP_TIMER = BIT(6),	  		SUSPENDWAKEUP_SRC_TIMER = BIT(6),  DEEPWAKEUP_SRC_TIMER = BIT(6),
	 PM_WAKEUP_COMP  = BIT(7),	  		SUSPENDWAKEUP_SRC_COMP  = BIT(7),  DEEPWAKEUP_SRC_COMP  = BIT(7),


	 PM_WAKEUP_CORE_GPIO   = BIT(5) | 0X0800,      SUSPENDWAKEUP_SRC_DIG_GPIO   = BIT(5) | 0X0800,
	 PM_WAKEUP_CORE_USB    = BIT(5) | 0X0400,      SUSPENDWAKEUP_SRC_DIG_USB    = BIT(5) | 0X0400,
	 PM_WAKEUP_CORE_QDEC   = BIT(5) | 0X1000,      SUSPENDWAKEUP_SRC_DIG_QDEC   = BIT(5) | 0X1000,
}SleepWakeupSrc_TypeDef;




//wakeup status from return value of "cpu_sleep_wakeup"
enum {
	 WAKEUP_STATUS_COMP   = BIT(0),  //wakeup by comparator
	 WAKEUP_STATUS_TIMER  = BIT(1),
	 WAKEUP_STATUS_CORE   = BIT(2),
	 WAKEUP_STATUS_PAD    = BIT(3),

	 STATUS_GPIO_ERR_NO_ENTER_PM  = BIT(7),
};

#define 	WAKEUP_STATUS_TIMER_CORE	( WAKEUP_STATUS_TIMER | WAKEUP_STATUS_CORE)
#define 	WAKEUP_STATUS_TIMER_PAD		( WAKEUP_STATUS_TIMER | WAKEUP_STATUS_PAD)



typedef struct{
	unsigned char is_deepRetn_back;
	unsigned char is_pad_wakeup;
	unsigned char wakeup_src;
}pm_para_t;

extern pm_para_t	pmParam;



void cpu_stall_wakeup_by_timer0(unsigned int tick_stall);
void cpu_stall_wakeup_by_timer1(unsigned int tick_stall);
void cpu_stall_wakeup_by_timer2(unsigned int tick_stall);

typedef int (*suspend_handler_t)(void);
void	bls_pm_registerFuncBeforeSuspend (suspend_handler_t func );

typedef unsigned short (*tick_32k_get_t)(void);
void pm_register_tick32kGet_callback(tick_32k_get_t cb);


void cpu_wakeup_init(void);
void cpu_set_gpio_wakeup (GPIO_PinTypeDef pin, GPIO_LevelTypeDef pol, int en);

int cpu_sleep_wakeup (SleepMode_TypeDef sleep_mode, SleepWakeupSrc_TypeDef wakeup_src, unsigned int wakeup_tick);


static inline int pm_is_MCU_deepRetentionWakeup(void)
{
	return pmParam.is_deepRetn_back;
}



static inline int pm_is_deepPadWakeup(void)
{
	return pmParam.is_pad_wakeup;
}

extern unsigned int flash_rdid;
static inline unsigned int pm_get_flash_rdid(void)
{
	return flash_rdid;
}

//only for debug below, will remove them later
void shutdown_gpio(void);  //for debug
void rc_24m_cal (void);


#define PM_Get32kTick			 cpu_get_32k_tick
#define pm_start				 sleep_start


static inline void check_and_set_1p95v_to_zbit_flash()
{
	if(1 == zbit_flash_flag){ // use "== 1"" should be better than "ture"
		analog_write(0x0c, ((analog_read(0x0c) & 0xf8)  | 0x7));//1.95
	}
}

static inline void blc_app_loadCustomizedParameters(void)
{
	if(!pm_is_MCU_deepRetentionWakeup()){
		zbit_flash_flag = flash_is_zb();
	}

	u8 calib_value = *(unsigned char*)(FLASH_ADR_CALIB_OFFSET_VREF);

	if((0xff == calib_value))
	{
		check_and_set_1p95v_to_zbit_flash();
	}
	else
	{
		analog_write(0x0c, ((analog_read(0x0c) & 0xf8)  | (calib_value&0x7)));
	}

}

#ifndef ZBIT_FLASH_WRITE_TIME_LONG_WORKAROUND_EN
#define ZBIT_FLASH_WRITE_TIME_LONG_WORKAROUND_EN					1
#endif

#endif
