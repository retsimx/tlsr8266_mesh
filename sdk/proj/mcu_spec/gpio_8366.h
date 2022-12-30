/********************************************************************************************************
 * @file     gpio_8366.h 
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

#include "../common/types.h"
#include "../common/bit.h"
#include "../common/utility.h"
#include "../mcu/compiler.h"
#include "../mcu/register.h"
#include "gpio_default_8366.h"
//#include "register_8366.h"



enum{
	    GPIO_MSDO = BIT(0),
		GPIO_MSDI = BIT(1),
		GPIO_MCLK = BIT(2),
		GPIO_MSCN = BIT(3),
		GPIO_GP7  = BIT(4), GPIO_DO = GPIO_GP7,
		GPIO_GP8  = BIT(5), GPIO_DI = GPIO_GP8,
		GPIO_GP9  = BIT(6), GPIO_CK = GPIO_GP9,
		GPIO_GP10 = BIT(7), GPIO_CN = GPIO_GP10,

		GPIO_GP0  = BIT(8),
		GPIO_GP1  = BIT(9),
		GPIO_GP2  = BIT(10),
		GPIO_GP3  = BIT(11),
		GPIO_GP4  = BIT(12),
		GPIO_GP5  = BIT(13),
		GPIO_GP6  = BIT(14),
		GPIO_SWS  = BIT(15),

		GPIO_DM   = BIT(21),
		GPIO_DP   = BIT(22),

		GPIO_MAX_COUNT = 18,
};


/*******************************************************************************

*******************************************************************************/
static inline int gpio_is_input_en(u32 pin){
	return (reg_gpio_ie & pin);
}


static inline void gpio_set_data_strength(u32 pin, u32 value){
	if(value){
		BM_SET(reg_gpio_ds, pin);
	}else{
		BM_CLR(reg_gpio_ds, pin);
	}
}


#if(1)  //func faster


static inline void gpio_set_output_en(u32 pin, u32 value){
	if(value){
		(*((volatile u32*)0x800588)) |= pin;
	}else{
		(*((volatile u32*)0x800588)) &= (~pin);
	}
}


static inline void gpio_set_input_en(u32 pin, u32 value){
	if(value){
		(*((volatile u32*)0x800590)) |= pin;
	}else{
		(*((volatile u32*)0x800590)) &= (~pin);
	}
}

static inline void gpio_write(u32 pin, u32 value){
	if(value){
		(*((volatile u32*)0x800584)) |= pin;
	}else{
		(*((volatile u32*)0x800584)) &= (~pin);
	}
}


static inline u32 gpio_read(u32 pin){
	return  ( (*((volatile u32*)0x800580)) & pin );
}

void gpio_write_in_ram(u32 pin, u32 value);

#else


static inline void gpio_set_output_en(u32 pin, u32 value){
	if(value){
		BM_SET(reg_gpio_oe, pin);
	}else{
		BM_CLR(reg_gpio_oe, pin);
	}
}


static inline void gpio_set_input_en(u32 pin, u32 value){
	if(value){
		BM_SET(reg_gpio_ie, pin);
	}else{
		BM_CLR(reg_gpio_ie, pin);
	}
}


static inline void gpio_write(u32 pin, u32 value){
	if(value){
		BM_SET(reg_gpio_datao, pin);
	}else{
		BM_CLR(reg_gpio_datao, pin);
	}
}


static inline u32 gpio_read(u32 pin){
	return BM_IS_SET(reg_gpio_datai, pin);
}

#endif

/*******************************************************************************

*******************************************************************************/
#if(0)   //m0

static inline void gpio_set_interrupt(u32 pin, u32 falling){
	if(falling){
		BM_SET(reg_gpio_pol, pin);
	}else{
		BM_CLR(reg_gpio_pol, pin);
	}
	BM_SET(reg_gpio_2risc0, pin);
}

static inline void gpio_clr_interrupt(u32 pin){
	BM_CLR(reg_gpio_2risc0, pin);
}

#else  //m2

static inline void gpio_set_interrupt(u32 pin, u32 falling){
	if(falling){
		BM_SET(reg_gpio_pol, pin);
	}else{
		BM_CLR(reg_gpio_pol, pin);
	}
	BM_SET(reg_gpio_2risc2, pin);
}

static inline void gpio_clr_interrupt(u32 pin){
	BM_CLR(reg_gpio_2risc2, pin);
}


static inline void gpio_set_interrupt_and_wakeup(u32 pins, u32 level){
	if(level){  //1:rising edge irq¡¢   high level wakeup
		        //0:falling edge irq¡¢low level wakeup
		BM_CLR(reg_gpio_pol, pins);
	}else{
		BM_SET(reg_gpio_pol, pins);
	}

	BM_SET(reg_gpio_2risc2, pins);
	reg_gpio_wakeup_en |= pins;

}

static inline void gpio_clr_interrupt_and_wakeup(u32 pins){
	BM_CLR(reg_gpio_2risc2, pins);
	reg_gpio_wakeup_en &= ~pins;
}

#endif


static inline void gpio_set_interrupt_pol(u32 pin, u32 falling){
	if(falling){
		BM_SET(reg_gpio_pol, pin);
	}else{
		BM_CLR(reg_gpio_pol, pin);
	}
}

//enable interrupt wheel interrupt and wakeup
static inline void gpio_enable_irq_wakeup_pin(u32 pins, u32 levels){

}

static inline void gpio_enable_wakeup_pin(u32 pins, u32 level, int en){
#if 1
	if (level) {   //1:high level wakeup   0:low level wakeup
 		reg_gpio_pol &= ~pins;
	}
	else {
		reg_gpio_pol |= pins;
	}

	if (en) {
		reg_gpio_wakeup_en |= pins;
	}
	else {
		reg_gpio_wakeup_en &= ~pins;
	}
#endif
}

static inline void gpio_pullup_dpdm_internal( u32 dp_dm_pullup ){
    reg_gpio_wakeup_en |= dp_dm_pullup;
}


/*******************************************************************************

*******************************************************************************/
#define GPIO_VALUE(type,n)				(GPIO##n##_##type?(GPIO_GP##n):0)
#define GPIO_FUNC_VALUE(type, func)		(func##_##type?(GPIO_##func):0)

#define GPIO_FUNC_REG_VALUE(type)		\
	( GPIO_VALUE(type, 0) | GPIO_VALUE(type, 1) | GPIO_VALUE(type, 2) | GPIO_VALUE(type, 3)			\
	| GPIO_VALUE(type, 4) | GPIO_VALUE(type, 5) | GPIO_VALUE(type, 6) | GPIO_VALUE(type, 7)			\
	| GPIO_VALUE(type, 8) | GPIO_VALUE(type, 9) | GPIO_VALUE(type, 10) \
	| GPIO_FUNC_VALUE(type, MSDO) | GPIO_FUNC_VALUE(type, MSDI) | GPIO_FUNC_VALUE(type, MCLK) | GPIO_FUNC_VALUE(type, MSCN) \
	| GPIO_FUNC_VALUE(type, SWS) | GPIO_FUNC_VALUE(type, DM) | GPIO_FUNC_VALUE(type, DP))

static inline void gpio_init(void){
	reg_gpio_oe	   = (u32)GPIO_FUNC_REG_VALUE(OUTPUT_ENABLE);
	reg_gpio_datao = (u32)GPIO_FUNC_REG_VALUE(DATA_OUT);
	reg_gpio_ie    = (u32)GPIO_FUNC_REG_VALUE(INPUT_ENABLE);
	reg_gpio_ds    = (u32)GPIO_FUNC_REG_VALUE(DATA_STRENGTH);

 	// to avoid codes blow
	reg_gpio_nrm = (
		  ((MSDO_FUNC == AS_GPIO) ? FLD_MSDO_FUNC : 0)
		| ((SWS_FUNC  == AS_GPIO) ? FLD_SWS_FUNC : 0)
		| ((DP_FUNC   == AS_GPIO && DM_FUNC == AS_GPIO) ? FLD_DP_FUNC : 0));


	analog_write (0x08, PULL_WAKEUP_SRC_GPIO1 |
						(PULL_WAKEUP_SRC_GPIO2<<2) |
						(PULL_WAKEUP_SRC_GPIO3<<4) |
						(PULL_WAKEUP_SRC_GPIO4<<6));

	analog_write (0x09,  PULL_WAKEUP_SRC_GPIO7 |
						(PULL_WAKEUP_SRC_GPIO8<<2) |
						(PULL_WAKEUP_SRC_GPIO9<<4) |
						(PULL_WAKEUP_SRC_GPIO10<<6));

	analog_write (0x0a,  PULL_WAKEUP_SRC_GPIO0 |
						(PULL_WAKEUP_SRC_GPIO5<<2) |
						(PULL_WAKEUP_SRC_GPIO6<<4) |
						(PULL_WAKEUP_SRC_MSDI<<6));

	u8 areg = analog_read (0x0b) & 0xfc;
	analog_write (0x0b, areg  | PULL_WAKEUP_SRC_MSDO);

}


static inline void gpio_set_func(u32 pin, u32 func){
	switch(pin){
			case GPIO_MCLK:
			case GPIO_MSDO:
			case GPIO_MSDI:
			case GPIO_MSCN:
				if(AS_GPIO == func){
					BM_SET(reg_gpio_nrm, FLD_MSDO_FUNC);
				}else{
					BM_CLR(reg_gpio_nrm, FLD_MSDO_FUNC);
				}
				break;
			case GPIO_SWS:
				if(AS_GPIO == func){
					BM_SET(reg_gpio_nrm, FLD_SWS_FUNC);
				}else{
					BM_CLR(reg_gpio_nrm, FLD_SWS_FUNC);
				}
				break;
			case GPIO_DP:
			case GPIO_DM:
				if(AS_GPIO == func){
					BM_SET(reg_gpio_nrm, FLD_DP_FUNC);
				}else{
					BM_CLR(reg_gpio_nrm, FLD_DP_FUNC);
				}
				break;
			default:break;
	}
}

