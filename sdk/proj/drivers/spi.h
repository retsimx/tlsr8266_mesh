/********************************************************************************************************
 * @file     spi.h 
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

void spi_sim_init(u32 pin_clk, u32 pin_dat_out);
u8 spi_sim_read(u32 pin_clk, u32 pin_dat_in);
void spi_sim_write(u32 pin_clk, u32 pin_dat_out, u8 data);

void spi_init(void);
void spi_write(u8 addr, u8 dat);
u8 spi_read(u8 addr);


