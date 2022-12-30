/********************************************************************************************************
 * @file     ethernet.h 
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

typedef struct eth_pkt_t
{   u32 pkt_len;
	u8 	dmac[6];
	u8 	smac[6];
	u16 type;
}eth_pkt_t; 
typedef eth_pkt_t* p_eth_pkt_t;

void eth_init(void);
void eth_pkt_recv_irq_handler(void);
void eth_send_pkt(u8 *pkt, u32 len);


