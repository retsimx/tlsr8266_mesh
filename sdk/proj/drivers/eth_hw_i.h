/********************************************************************************************************
 * @file     eth_hw_i.h 
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

#include "../config/user_config.h"
#include "../mcu/register.h"
#include "eth_hw.h"

#if(MODULE_ETH_ENABLE)

extern u8* eth_rx_pkt_buffer;
extern u8* eth_tx_pkt_buffer;

static inline void eth_hw_set_rx_done(void){
	// moved to irq_handler.c
//	reg_mac_irq_sta = 0xff;	// write 1 to clear status. indicate packet handled
}

static inline int eth_hw_is_rx_ready(void){
	return	reg_mac_irq_sta & FLD_MAC_STA_RX_DONE;
}

static inline u8* eth_hw_pkt_buff_begin(void){
	return	eth_rx_pkt_buffer;
}
static inline u8* eth_hw_pkt_buff_end(void){
	return	eth_rx_pkt_buffer + ETH_RX_PKT_BUFF_SIZE;
}

static inline void eth_hw_set_rx_addr(u8 * addr){
	reg_dma_eth_rx_addr = (u16)(u32)(addr);
}

static inline void eth_hw_set_tx_addr(u8 * addr){
	reg_dma_eth_tx_addr = (u16)(u32)(addr);
}

static inline u8* eth_hw_get_tx_buff(void){
	return &eth_tx_pkt_buffer[0];
}

#endif

