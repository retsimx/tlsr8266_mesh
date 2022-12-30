/********************************************************************************************************
 * @file     gpio_8366.c 
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


#include "../config/user_config.h"
#include "../mcu/config.h"


#if(__TL_LIB_8366__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8366))

#include "../common/types.h"
#include "../common/compatibility.h"
#include "../common/bit.h"
#include "../common/utility.h"
#include "../common/static_assert.h"
#include "../mcu/compiler.h"
#include "../mcu/register.h"
#include "../mcu/anareg.h"
#include "../mcu/analog.h"

#include "../mcu/gpio.h"


#define    MOUSE_GPIO_MAX        18
#define    PULL_UPDM_GPIO_NUM    13

u32 mouse_gpio_table[MOUSE_GPIO_MAX] =   //for hamster
{
		GPIO_GP0,  // 		0
		GPIO_GP1,  // 		1
		GPIO_GP2,  // 		2
		GPIO_GP3,  // 		3
		GPIO_GP4,  // 		4
		GPIO_GP5,  // 		5
		GPIO_GP6,  // 		6
		GPIO_GP7,  // 		7
		GPIO_GP8,  // 		8
		GPIO_GP9,  // 		9
		GPIO_GP10, // 		10
		GPIO_MSDI, // 		11
		GPIO_MSDO, // 		12
		//only above 13 pin have pull_up resistor

		GPIO_MCLK, // 		13
		GPIO_MSCN, // 		14
		GPIO_DM,   // 		15
		GPIO_DP,   // 		16
		GPIO_SWS,  //       17
};


/************
 *
 * gpio:         indicate the pin
 * up_down:      1 need pull up, 0 need pull down
 */

u8  anareg_base_08_value[4] = {0,0,0,0};


/*************************************************************************
time use experiment data:
16M  switch     GPIO_GP6  write_analog   	 12us
16M  switch     GPIO_GP6  not write_analog   6us      -> write_analog:6us
16M  no select  GPIO_GP6  not write_analog   2.5us    -> switch:3.5 us
16M  switch     GPIO_GP6  write_analog       9.5us    -> 6+3.5=9.5
16M  if         GPIO_GP6  write_analog       13.4 us  -> if:4.9us
16M  if         GPIO_GP6  not write_analog   6.9us    -> 2.5+4.9=7.4 us

if:           worst 5 us
switch:       3.5us
write_analog: 6 us
other:        2.5 us

16M时钟从外部flash取值调用gpio_setup_up_down_resistor函数时间：
采用switch结构，若GPIO没有复用为两个按键，时间6us，若复用，时间为12us
*************************************************************************/
void gpio_setup_up_down_resistor(u32 gpio, u32 up_down)
{

	u8 r_val = 0;


	if(up_down == PM_PIN_UP_DOWN_FLOAT){
		r_val = 0;
	}
	else if(up_down){
		r_val =	PM_PIN_PULLUP_1M;
	}else{
		r_val = PM_PIN_PULLDOWN_100K;
	}


	u8 cur_vlaue,shift_num;
	u8 anareg_offset = 0;
	u8 * wkp_src_ptr = &anareg_base_08_value[0];


#if(1)
	switch(gpio){      //16M 3.5us
	case GPIO_GP1:
		shift_num = 0;
		break;
	case GPIO_GP2:
		shift_num = 2;
		break;
	case GPIO_GP3:
		shift_num = 4;
		break;
	case GPIO_GP4:
		shift_num = 6;
		break;
	case GPIO_GP7:
		anareg_offset = 1;
		shift_num = 0;
		break;
	case GPIO_GP8:
		anareg_offset = 1;
		shift_num = 2;
		break;
	case GPIO_GP9:
		anareg_offset = 1;
		shift_num = 4;
		break;
	case GPIO_GP10:
		anareg_offset = 1;
		shift_num = 6;
		break;
	case GPIO_GP0:
		anareg_offset = 2;
		shift_num = 0;
		break;
	case GPIO_GP5:
		anareg_offset = 2;
		shift_num = 2;
		break;
	case GPIO_GP6:
		anareg_offset = 2;
		shift_num = 4;
		break;
	case GPIO_MSDI:
		anareg_offset = 2;
		shift_num = 6;
		break;
	case GPIO_MSDO:
		anareg_offset = 3;
		shift_num = 0;
		break;
	default:
		return;
	}
#else
	if(gpio==GPIO_GP1){   //16M last one 4.9us
		shift_num = 0;
	}
	else if(gpio==GPIO_GP2){
		shift_num = 2;
	}
	else if(gpio==GPIO_GP3){
		shift_num = 4;
	}
	else if(gpio==GPIO_GP4){
		shift_num = 6;
	}
	else if(gpio==GPIO_GP7){
		anareg_offset = 1;
		shift_num = 0;
	}
	else if(gpio==GPIO_GP8){
		anareg_offset = 1;
		shift_num = 2;
	}
	else if(gpio==GPIO_GP9){
		anareg_offset = 1;
		shift_num = 4;
	}
	else if(gpio==GPIO_GP10){
		anareg_offset = 1;
		shift_num = 6;
	}
	else if(gpio==GPIO_GP0){
		anareg_offset = 2;
		shift_num = 0;
	}
	else if(gpio==GPIO_GP5){
		anareg_offset = 2;
		shift_num = 2;
	}
	else if(gpio==GPIO_GP6){
		anareg_offset = 2;
		shift_num = 4;
	}
	else{
		return;
	}

#endif

	u8 *cur_anareg = wkp_src_ptr + anareg_offset;
	cur_vlaue =	( (*cur_anareg) & (~(3<<shift_num))) | (r_val<<shift_num);
	if(cur_vlaue != (*cur_anareg)){       //16M   analog_write 6 us
		*cur_anareg = cur_vlaue;
		analog_write(0x08+anareg_offset,cur_vlaue);
	}
}


_attribute_ram_code_ void gpio_write_in_ram(u32 pin, u32 value){
	if(value){
		(*((volatile u32*)0x800584)) |= pin;
	}else{
		(*((volatile u32*)0x800584)) &= (~pin);
	}
}



#if(0)

const unsigned char resistor_at[MOUSE_GPIO_MAX*2] ={
	0x0a, 0,     //GPIO_GP0	 	0

	0x08, 0,     //GPIO_GP1		1
	0x08, 2,     //GPIO_GP2		2
	0x08, 4,     //GPIO_GP3		3
	0x08, 6,     //GPIO_GP4		4

	0x0a, 2,     //GPIO_GP5	 	5
	0x0a, 4,     //GPIO_GP6	 	6

	0x09, 0,     //GPIO_GP7		7
	0x09, 2,     //GPIO_GP8		8
	0x09, 4,     //GPIO_GP9		9
	0x09, 6,     //GPIO_GP10	10

	0x0a, 6,	 //GPIO_MSDI	11
	0x0b, 0,	 //GPIO_MSDO	12
};




void gpio_setup_up_down_resistor(u32 gpio, u32 up_down)   //16M  GPIO_GP10  18.3us
{
	if(gpio >= PULL_UPDM_GPIO_NUM){ //13-17  GPIO no pull updn resistor
		return;
	}
	u8 r_val;

	if( up_down == PM_PIN_UP_DOWN_FLOAT )
		r_val = 0;
	else if(up_down)
		r_val = PM_PIN_PULLUP_1M;
	else
		r_val = PM_PIN_PULLDOWN_100K;

	analog_write(resistor_at[gpio*2],
			(analog_read(resistor_at[gpio*2]) & (~(0x3<<resistor_at[gpio*2+1])))
												 |(r_val<<resistor_at[gpio*2+1]));
}

#endif

#endif

