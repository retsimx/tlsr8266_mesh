/********************************************************************************************************
 * @file     rf.h 
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

#include "../common/static_assert.h"

void rf_init(void);

#define	RF_PROTO_VER		1		// important, this combined with RF_PROTOCOL indicate the rf protocol compatibility, pkt format, etc

enum{
	RF_PROTO_PROPRIETARY,			// must be zero
	RF_PROTO_RF4CE,
	RF_PROTO_ZIGBEE,
};

#if 0
// for receiving packets
/*	
	struct {			
		u8 rsv0;
		u8 rsv1;
		u8 freq_off_l;
		u8 freq_off_h;
	};
*/

u32 dma_len;		//	valid only for sending out packets
u8 rf_len;			//	be very careful with alighnment, used to verify packet correctness
u8 proto;			// !!!	must be same as rf_pkt_t before gid
struct{
	u8 rsv:3;
	u8 ack_req:1;
	u8 conn_sta:2;	// for dongle initiate pkt, this is the MIC states
	u8 token:1;
	u8 direction:1;
};
u8 seq_no;
u8 per;
u8 rssi;			// rssi of last receiving pkt
u16 vendor;
u32 sender;

#endif

#define RF_PKT_HEAD_FIELDS		\
	u32 dma_len;				\
	u8 rf_len;					\
	u8 proto;					\
	struct{						\
		u8 rsv:3;				\
		u8 ack_req:1;			\
		u8 conn_sta:2;			\
		u8 token:1;				\
		u8 direction:1;			\
	};							\
	u8 seq_no;					\
	u8 per;						\
	u8 rssi;					\
	u16 vendor;					\
	u32 sender


typedef struct rf_pkt_head_t{
	RF_PKT_HEAD_FIELDS;
	u8 data[1];
}rf_pkt_head_t;

typedef rf_pkt_head_t* p_rf_pkt_head_t;
typedef rf_pkt_head_t rf_pkt_t;
typedef p_rf_pkt_head_t p_rf_pkt_t;

enum{
	FLD_RF_PROTO_VER = BIT_RNG(0,4),
	FLD_RF_PROTO_TYPE = BIT_RNG(5,7),	//   8 protocols supported
};

#define RF_PROTO_BYTE	MASK_VAL(FLD_RF_PROTO_TYPE, RF_PROTOCOL, FLD_RF_PROTO_VER, RF_PROTO_VER)

// check the hw packet structure
static inline int rf_get_rssi(u8* pkt){
	return *(pkt - 4);
}

// check the hw packet structure
static inline u32 rf_get_time_phase(u8* pkt){
	u8 *p = pkt - 8;
	u8 *pt = p + p[0];
	return pt[0] + (pt[1]<<8) + (pt[2]<<16);
}

int rf_add_tx_pkt(u8 * pkt);
void rf_rx_irq_handler(void);

static inline void rf_set_pkt_rx_time(u8* pkt){
	u32 t = clock_time();
	*(u32*)(pkt + (RF_PKT_BUFF_LEN - sizeof(u32))) = t;
}

static inline u32 rf_get_pkt_rx_time(u8* pkt){
	return *(u32*)(pkt + (RF_PKT_BUFF_LEN - sizeof(u32) - 8));
}

#define RF_SEND_PKT_BUSY_TIMEOUT	1500			//  us
#define RF_RECV_PKT_BURST_TIMEOUT	1500			//  us

#if(APPLICATION_DONGLE)		
#define RF_RECV_PKT_SUSPEND_TIMEOUT	(100*1000)	//  dongle wait 100 ms
#else
#define RF_RECV_PKT_SUSPEND_TIMEOUT	(1100)		//  device wait 1.1 ms
#endif

int rf_allow_suspend(void);

typedef struct{
	u32 pkt_wait_start_time;	// use for MAC layer
	u32 last_sender;
	rf_pkt_head_t *last_send_pkt;
	u32 search_channel_pkt_time;
	u32 pkt_cnt_missing_ack;
#if(APPLICATION_DONGLE)	
	u32 debug_send_time;
#else		
	u32 conn_start_time;			
	u32 last_ack_req_time;
	u32 search_channel_time;
	u32 heartbeat_send_time;
	u8 wait_audio_stat;	// we send mic-on req, waiting the dongle send back mic stat
	u8 from_cur_chn;	//  searching the very first time, from the current channel, otherwise from the next channel
#endif
	u8 token_recv;
	u8 conn_sta;
	u8 token_sent;
	u8 waiting_ack;
	u8 send_ack_req;
	u8 recv_ack_req;
	u8 sending_pkt;			// this round started,  we had token, and we already sending pkts
	u8 recv_pkt_left_cnt;
	u8 send_pkt_seq_no;
	u8 send_waiting;
	
}rf_mac_t;
extern rf_mac_t rf_mac;

enum{
	RF_CONN_STATE_SEARCHING,
	RF_CONN_STATE_CONNECTING,	//  in connecting state or connected state, normal packets are still sending 
	RF_CONN_STATE_CONNECTED,	//  in connecting state or connected state, normal packets are still sending 
};

void rf_set_connecting_state(void);
void rf_set_searching_state(int from_cur_chn);
void rf_set_tx_mode(int force);
void rf_set_rx_mode(int force);


