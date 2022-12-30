/********************************************************************************************************
 * @file     ethernet.c 
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
#include "ethernet.h"
#include "ip_adapt_i.h"

#ifdef WIN32
#include <stdio.h>
#endif

void eth_send_pkt(u8 *pkt, u32 len){
	// https://supportforums.cisco.com/thread/2046314
	// uip_len = 64; // arp must padded to 64 for some older pc 10M/100M phy.  or else, ping will fail
	if(len < 64){		// minimum pkt length.  if < 64,  arp pkt can't be sent out
		len = 64;
	}
	u8 *eth_pkt = (pkt - 4);
	*(u32*)eth_pkt = len;
	eth_hw_send_pkt(eth_pkt);
#if (__DEBUG_PRINT__)
	u8* pp = pkt;
	printf("eth:s: len=%d, %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n"
		,len, pp[0],pp[1],pp[2],pp[3],pp[4],pp[5],pp[6],pp[7],pp[8],pp[9],pp[10],pp[11],pp[12],pp[13],pp[14],pp[15],pp[16],pp[17],pp[18],pp[19],pp[20]);
#endif
}

static inline u8* eth_get_next_buffer_ptr(u8 * ptr){
	ptr += ETH_RX_PKT_BUFF_LEN;
	if(ptr >= eth_hw_pkt_buff_end()){
		ptr = eth_hw_pkt_buff_begin();
	}
	return ptr;
}
void eth_process_pkt(void * p){
	p_eth_pkt_t pkt = (p_eth_pkt_t)p;
	u8* eth_head = &pkt->dmac[0];
#if (__DEBUG_PRINT__)
	u8* pp = eth_head;
	printf("eth:r: len=%d, %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n"
		,pkt->pkt_len, pp[0],pp[1],pp[2],pp[3],pp[4],pp[5],pp[6],pp[7],pp[8],pp[9],pp[10],pp[11],pp[12],pp[13],pp[14],pp[15],pp[16],pp[17],pp[18],pp[19],pp[20]);
#endif
	if(ipa_is_broadcast_mac(eth_head) || ipa_is_my_mac(eth_head))
	{
		if(pkt->type == 0x0608){ 		// arp
			ipa_handle_arp_pkt(eth_head, pkt->pkt_len);
		}else if(pkt->type == 0x0008){
			ipa_handle_ip_pkt(eth_head, pkt->pkt_len);
		}
		ipa_set_uip_buf_to_default_tx();
	}
}

//  标识最后一个 buffer (假定读得很慢)，是否成功
// 如果不是，假定buffer 刚好满了，而且没有再多的pkt。那么最后一个 pkt 就不会被处理
// 不过，网络中应该不停有各种各样的包吧。所以这种情况不会出现
// static u8 eth_last_buff_success;
u8 *eth_write_ptr = 0, *eth_read_ptr = 0;
void eth_recv_pkt(void){
	if(eth_read_ptr != eth_write_ptr){
		eth_process_pkt(eth_read_ptr);
		eth_read_ptr = eth_get_next_buffer_ptr(eth_read_ptr);
	}
}

static inline void eth_set_next_buffer(void){
	u8 * next_wr_ptr = eth_get_next_buffer_ptr(eth_write_ptr);
	if(next_wr_ptr == eth_read_ptr){
		return;
	}
	eth_write_ptr = next_wr_ptr;
	eth_hw_set_rx_addr(next_wr_ptr);
}

//  mac irq handler, when a pkt received by DMA
void eth_rx_irq_handler(void){
#ifdef WIN32
	eth_set_next_buffer();
#else
//	if(eth_hw_is_rx_ready()){			// using DMA interrupt,  this must be ready
		eth_set_next_buffer();
		eth_hw_set_rx_done();		//  write clear
//	}
#endif

}
#if(MODULE_ETH_ENABLE)
void eth_init(){
	eth_hw_init();
	eth_write_ptr = eth_read_ptr = eth_hw_pkt_buff_begin();

	ipa_init();
	
	ev_on_poll(EV_POLL_ETH_RECV, eth_recv_pkt);
}
#endif

#endif
