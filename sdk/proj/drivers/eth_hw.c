/********************************************************************************************************
 * @file     eth_hw.c 
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
#if(MODULE_ETH_ENABLE)
#include "eth_hw.h"
#include "eth_hw_i.h"
#include "../ext_drv/eth_phy_i.h"

STATIC_ASSERT_INT_DIV(ETH_RX_PKT_BUFF_LEN, 16);
// if using pingpong mode,   eth_hw_set_buffer , must be 16 aligned. 

u8 _attribute_aligned_(16) eth_pkt_buffer[ETH_RX_PKT_BUFF_SIZE + ETH_RX_PKT_BUFF_LEN];
u8 *eth_rx_pkt_buffer = &eth_pkt_buffer[0];
u8 *eth_tx_pkt_buffer = &eth_pkt_buffer[ETH_RX_PKT_BUFF_SIZE];

static inline void eth_hw_enable_analog(void){						//   I don't know WTF this is, to fix
	rega_set_ldo_output(4, ANALOG_LDO_VOL_1P6);
	analog_write(rega_xtal_ctrl, MASK_VAL(FLDA_XTAL_CS, 0x14));
}

static inline void eth_hw_wait_mii(void){
#ifndef WIN32
	while(reg_mii_ctrl & FLD_MII_BUSY);
#else
	CLR_FLD(reg_mii_ctrl, FLD_MII_BUSY);
#endif
}
u16 eth_hw_read_phy_reg(u8 reg){
	reg_mii_ctrl = MASK_VAL(FLD_MII_INTERNAL_REG, reg, FLD_MII_PREAM_EN, 1, FLD_MII_BUSY, 1);
	eth_hw_wait_mii();
	return reg_mii_rx_data;
}

void eth_hw_write_phy_reg(u8 reg, u16 data){
	reg_mii_tx_data = data;
	reg_mii_ctrl = MASK_VAL(FLD_MII_INTERNAL_REG, reg, FLD_MII_WR, 1, FLD_MII_PREAM_EN, 1, FLD_MII_BUSY, 1);
	eth_hw_wait_mii();
}

static inline void eth_hw_set_ctrl(int rx_en, int tx_en){
	reg_mac_ctrl = (ETH_SPEED_100M ? FLD_MAC_CTRL_SPD_100M : 0) | (rx_en?FLD_MAC_CTRL_RX_EN:0) | (tx_en?FLD_MAC_CTRL_TX_EN:0);
}

void eth_hw_init(void){
	u8 r = irq_disable();
	eth_hw_enable_analog();
	reg_mii_clk = MASK_VAL(FLD_MII_CLK_DIV, 3, FLD_MII_PHY_ID, eth_phy_get_addr());
	eth_phy_init();
	eth_hw_set_rx_addr(&eth_rx_pkt_buffer[0]);
	reg_dma_eth_rx_ctrl = FLD_DMA_WR_MEM;		//  not pingpong mode
	eth_hw_set_ctrl(1, 0);
	irq_restore(r);
}

static inline int eth_hw_wait_pkt_sent(){
#ifndef WIN32	
	u32 wait_start = clock_time();
	while(reg_mac_ctrl & FLD_MAC_CTRL_TX_EN){
		if(clock_time_exceed(wait_start, 10000)){	//   10 ms,  to avoid dead-loop when no wire connection
			return 0;
		}
	}
#endif	
	return 1;
}

void eth_hw_send_pkt(u8 *pkt){
	if(eth_hw_wait_pkt_sent()){		//  wait LAST success for busy cleared
		u8 r = irq_disable();		//  must ? , why ??
		eth_hw_set_tx_addr(pkt);
		SET_FLD(reg_dma_tx_rdy0, FLD_DMA_ETH_TX);
		eth_hw_set_ctrl(1, 1);
		irq_restore(r);
	}
}
#endif
