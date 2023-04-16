/********************************************************************************************************
 * @file     blueLight.h 
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

typedef unsigned char u8 ;
typedef signed char s8;

typedef unsigned short u16;
typedef signed short s16;

typedef int s32;
typedef unsigned int u32;

typedef long long s64;
typedef unsigned long long u64;

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef signed long long int64_t;
typedef unsigned long long uint64_t;

/////////////////////////////  PARING HEAD /////////////////////////////////////////////
#define			BLE_GATT_OP_PAIR_REQ			1
#define			BLE_GATT_OP_PAIR_RSP			2
#define			BLE_GATT_OP_PAIR_REJECT			3
#define			BLE_GATT_OP_PAIR_NETWORK_NAME	4
#define			BLE_GATT_OP_PAIR_NETWORK_PASS	5
#define			BLE_GATT_OP_PAIR_NETWORK_LTK	6
#define			BLE_GATT_OP_PAIR_CONFIRM		7
#define			BLE_GATT_OP_PAIR_LTK_REQ		8
#define			BLE_GATT_OP_PAIR_LTK_RSP		9
#define			BLE_GATT_OP_PAIR_DELETE			10
#define			BLE_GATT_OP_PAIR_DEL_RSP		11
#define			BLE_GATT_OP_ENC_REQ				12
#define			BLE_GATT_OP_ENC_RSP				13
#define			BLE_GATT_OP_ENC_FAIL			14
#define			BLE_GATT_OP_ENC_READY			15

#define         FLG_MESH_PAIRE_ENABLE           0x01    // one byte flag after LTK.

enum{
	EVENT_PKT_CUA = 0,			// connected undirected advertising event
	EVENT_PKT_ADV = EVENT_PKT_CUA,
	EVENT_PKT_CDA = 1,			// connected directed advertising event
	EVENT_PKT_NUA = 2,			// non-connected undirected advertising
	EVENT_PKT_SCAN_REQ = 3,
	EVENT_PKT_SCAN_RSP = 4,
	EVENT_PKT_CONNECT = 5,
	EVENT_PKT_DISCV = 6,		// connected discovery advertising event 
	
	EVENT_SYS_TIMER,
	EVENT_PKT_PAIR,		// PKT �����ͣ�ͬʱ��Ϊ event ����
	EVENT_PKT_RELAY,	// PKT �����ͣ�ͬʱ��Ϊ event ����
	EVENT_PKT_DATA,
	EVENT_PKT_DATA_RESP,
	EVENT_PKT = 0xfe,
	EVENT_NONE = 0xff,
};

typedef struct {
    u8 llid   :2;
    u8 nesn   :1;
    u8 sn     :1;
    u8 md     :1;
    u8 rfu1   :3;
}rf_data_head_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
}rf_packet_head_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
}rf_packet_auto_reply_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)

	u8	advA[6];			//address
	u8	data[15];			//0-31 byte
}rf_packet_adv_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4): connect request PDU
	u8  rf_len;				//LEN(6)_RFU(2)

	u8	scanA[6];			//
	u8	advA[6];			//
	u8	aa[4];				// access code
	u8	crcinit[3];
	u8	wsize;
	u16	woffset;
	u16 interval;
	u16 latency;
	u16 timeout;
	u8	chm[5];
	u8	hop;				//sca(3)_hop(5)
}rf_packet_relay_t;

/*
LLID(2) - NESN(1) - SN(1) - MD(1) - RFU(3) - Length(5) - RFU(3)
*/


typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u16	l2capLen;
	u16	chanId;
}rf_packet_l2cap_head_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 data[1];
}rf_packet_l2cap_req_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u8	data;
}rf_packet_l2cap_cust_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 	errOpcode;
	u16 errHandle;
	u8  errReason;
}rf_packet_att_errRsp_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)

	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8	startingHandle;
	u8	startingHandle1;
	u8	endingHandle;
	u8	endingHandle1;
	u8	attType[2];				//
}rf_packet_att_readByType_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)

	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8	startingHandle;
	u8	startingHandle1;
	u8	endingHandle;
	u8	endingHandle1;
	u8	attType[2];
	u8  attValue[2];
}rf_packet_att_findByTypeReq_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)

	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u16 	data[1];
}rf_packet_att_findByTypeRsp_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle;
	u8 handle1;
}rf_packet_att_read_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 	value[22];
}rf_packet_att_readRsp_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8  datalen;
	u8  data[1];			// character_handle / property / value_handle / value
}rf_packet_att_readByTypeRsp_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8  datalen;
	u16  data[3];
}rf_packet_att_readByGroupTypeRsp_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8  format;
	u8  data[1];			// character_handle / property / value_handle / value
}rf_packet_att_findInfoReq_t;

typedef struct{
	u32 dma_len;
	u8	type;
	u8  rf_len;
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle;
	u8 handle1;
	u8 value;//sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}rf_packet_att_write_t;

typedef struct{
	u32 dma_len;
	u8	type;
	u8  rf_len;
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle;
	u8 handle1;
	u8 value[30];//sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}rf_packet_att_cmd_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle[2];
	u8 data;
}rf_packet_att_notification_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 mtu[2];
}rf_packet_att_mtu_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
}rf_packet_att_writeRsp_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8 	opcode;
	u8	data[8];
}rf_packet_feature_rsp_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8 	opcode;
	u8  mainVer;
	u16 vendor;
	u16 subVer;
}rf_packet_version_ind_t;

typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8 	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8 	opcode;
	u8 	winSize;
	u16 winOffset;
	u16 interval;
	u16 latency;
	u16 timeout;
	u16 instant;
}rf_packet_connect_upd_req_t;

typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8 	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8 	opcode;
	u8 	chm[5];
	u16 instant;
} rf_packet_chm_upd_req_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8 	opcode;
	u8	data[1];
}rf_packet_ctrl_unknown_t;


typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8  id;
	u16 siglen;		
    u16 min_interval;
    u16 max_interval;
    u16 latency;
    u16 timeout;
}rf_packet_connParUpReq_t;

typedef struct {
	u8  valid;
	u8 	winSize;
	u16 winOffset;
	u16 interval;
	u16 latency;
	u16 timeout;
	u16 instant;
}connect_upd_data_t;

typedef struct {
	u8  valid;
	u8 	chm[5];
	u16 instant;
}connect_chm_upd_data_t;

