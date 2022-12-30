/********************************************************************************************************
 * @file     phytest.h 
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

#ifndef  PHYTEST_H
#define  PHYTEST_H
#define REG_BASE_ADDR			0x800000
#define REG_ADDR8(a)			(*(volatile u8*) (REG_BASE_ADDR + (a)))
#define REG_ADDR16(a)			(*(volatile u16*)(REG_BASE_ADDR + (a)))
#define REG_ADDR32(a)			(*(volatile u32*)(REG_BASE_ADDR + (a)))

#define UART_C2C3 1
#define UART_B2B3 0
#define UART_A6A7 0

void  user_init_bqb(void);
void  main_loop_bqb(void);
void Bqb_Init(void);

extern unsigned char bqb_en_flag;

#endif
