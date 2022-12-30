/********************************************************************************************************
 * @file     rfhw_i.h 
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

#include "rfhw.h"
#include "../mcu/register.h"

#if (MODULE_RF_ENABLE)

#ifdef WIN32
static inline void rfhw_set_rx_ready(int i){
	if(0 == i){
		SET_FLD(reg_dma_rx_rdy0, FLD_DMA_RF_RX);
	}else{
		SET_FLD(reg_dma_rx_rdy1, FLD_DMA_RF_RX);
	}
}
static inline void rfhw_set_rx_done(int i){
	if(0 == i){
		CLR_FLD(reg_dma_rx_rdy0,FLD_DMA_RF_RX);	// write 1 to clear status. indicate packet handled
	}else{
		CLR_FLD(reg_dma_rx_rdy1, FLD_DMA_RF_RX);	// write 1 to clear status. indicate packet handled
	}
}
#else
static inline void rfhw_set_rx_done(int i){
	if(0 == i){
		reg_dma_rx_rdy0 = FLD_DMA_RF_RX;	// write 1 to clear status. indicate packet handled
	}else{
		reg_dma_rx_rdy1 = FLD_DMA_RF_RX;	// write 1 to clear status. indicate packet handled
	}
}
#endif

static inline int rfhw_is_rx_ready(int i){
	if(0 == i){
		return	reg_dma_rx_rdy0 & FLD_DMA_RF_RX;
	}else{
		return	reg_dma_rx_rdy1 & FLD_DMA_RF_RX;
	}
}

static inline void rfhw_set_rx_buff_addr(u8 * addr){
	reg_dma_rf_rx_addr = (u16)(u32)(addr);
}

static inline void rfhw_set_tx_buff_addr(u8 * addr){
	reg_dma_rf_tx_addr = (u16)(u32)(addr);
}

static inline void rfhw_rx_radio_on(void){
}

static inline void rfhw_rx_radio_off(void){
	reg_pll_rx_ctrl = 0;
}


extern u8 rf_rx_pkt_buff[];
#if(MCU_CORE_TYPE == MCU_CORE_5320)
#define RF_TX_MAX_POWER		0x07
#define RF_TX_MIN_POWER		0x00
#elif(MCU_CORE_TYPE == MCU_CORE_5328)
#define RF_TX_MAX_POWER		0x44
#define RF_TX_MIN_POWER		0x45
#elif(MCU_CORE_TYPE == MCU_CORE_5332 || MCU_CORE_TYPE == MCU_CORE_5330)
#define RF_TX_MAX_POWER		0x10
#define RF_TX_MIN_POWER		0x11
#endif
extern u8 rfhw_tx_power;
static inline void rfhw_set_tx_power_high(void){
	rfhw_tx_power = RF_TX_MAX_POWER;
}

static inline void rfhw_set_tx_power_low(void){
	rfhw_tx_power = RF_TX_MIN_POWER;
}

extern u8 rf_tx_mode;
static inline void rfhw_set_tx_mode(u8 mode){
	rf_tx_mode = mode;
}

#endif
