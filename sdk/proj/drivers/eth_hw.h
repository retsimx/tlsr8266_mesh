/********************************************************************************************************
 * @file     eth_hw.h 
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

#define ETH_RX_PKT_BUFF_SIZE	(ETH_RX_PKT_BUFF_LEN * ETH_RX_PKT_BUFF_COUNT)

void eth_hw_init(void);
void eth_hw_send_pkt(u8 *pkt);
u16 eth_hw_read_phy_reg(u8 reg);
void eth_hw_write_phy_reg(u8 reg, u16 data);
