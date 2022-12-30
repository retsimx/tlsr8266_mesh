/********************************************************************************************************
 * @file     ir.h 
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

#include "../tl_common.h"
#include "pwm.h"

#if(IR_PROTOCOL == IR_PROTOCOL_NEC)
#define IR_HIGH_CARR_TIME			565			// in us
#define IR_HIGH_NO_CARR_TIME		1685
#define IR_LOW_CARR_TIME			560
#define IR_LOW_NO_CARR_TIME			565
#define IR_INTRO_CARR_TIME			9000
#define IR_INTRO_NO_CARR_TIME		4500

#define IR_SWITCH_CODE              0x0d
#define IR_ADDR_CODE                0x00
#define IR_CMD_CODE                 0xbf

#define IR_REPEAT_INTERVAL_TIME     40500
#define IR_REPEAT_NO_CARR_TIME      2250
#define IR_END_TRANS_TIME			563

#define IR_CARRIER_FREQ				38222
#define IR_CARRIER_DUTY				3
#else

#endif

static inline void ir_init(void){
#if(IR_USE_PWM)
#if(IR_PROTOCOL == IR_PROTOCOL_NEC)
    pwm_set(IR_PWM_ID, IR_CARRIER_FREQ, -1, IR_CARRIER_DUTY);
#else
#endif
#else
#endif
}

void ir_send_switch(u8 addr, u8 cmd);


#ifndef GPIO_IR_LEARN_IN
#define GPIO_IR_LEARN_IN			GPIO_GP26
#endif

void ir_record(u32 tm, int pol);
int ir_record_end(void * data);
void ir_learn(void);
void ir_learn_init(void);
void ir_learn_test(void);


