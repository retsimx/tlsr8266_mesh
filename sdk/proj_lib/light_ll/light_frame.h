/********************************************************************************************************
 * @file     light_frame.h 
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
#ifndef _RF_LIGHT_LINK_LAYER_FRAME_H_
#define _RF_LIGHT_LINK_LAYER_FRAME_H_

#include "../../proj/tl_common.h"

#define	PKT_CMD_LEN		11

////////////////////////////////////////////////////////////////
// ble mode
////////////////////////////////////////////////////////////////

enum {
	FLG_BLE_ADV_IND				= 0,
	FLG_BLE_ADV_DIRECT_IND		= 1,
	FLG_BLE_ADV_NONCONN_IND		= 2,
	FLG_BLE_SCAN_REQ			= 3,
	FLG_BLE_SCAN_RSP			= 4,
	FLG_BLE_CONNECT_REQ			= 5,
	FLG_BLE_ADV_DISCOVER_IND	= 6,
	FLG_BLE_TXADR_RANDOM		= BIT(6),
	FLG_BLE_RXADR_RANDOM		= BIT(7),

	FLG_BLE_LIGHT_ADV			= FLG_BLE_ADV_IND,
	FLG_BLE_LIGHT_SCAN_REQ		= FLG_BLE_SCAN_REQ,
	FLG_BLE_LIGHT_SCAN_RSP		= FLG_BLE_SCAN_RSP,
	FLG_BLE_LIGHT_CONNECT_REQ	= FLG_BLE_CONNECT_REQ,
	FLG_BLE_LIGHT_BROADCAST		= FLG_BLE_ADV_NONCONN_IND,
	FLG_BLE_LIGHT_DATA			= 0x02,
	FLG_BLE_LIGHT_DATA_RSP		= 0x02,

	FLG_SYS_LINK_IDLE					= 0,
	FLG_SYS_LINK_ADV					= 1,
	//FLG_SYS_LINK_SCAN					= 2,
	FLG_SYS_LINK_WAKEUP					= 3,
	FLG_SYS_LINK_LISTEN					= 4,
	FLG_SYS_LINK_BROADCAST				= 5,
	FLG_SYS_LINK_CONNECTTING			= 6,
	FLG_SYS_LINK_CONNECTED				= 7,
	FLG_SYS_LINK_BRIDGE					= FLG_SYS_LINK_BROADCAST,

	FLG_SYS_LINK_START					= 8,
	FLG_SYS_LINK_LOST					= 9,
	FLG_SYS_LINK_STOP					= 10,
	FLG_SYS_LINK_MISSING				= 11,

	FLG_SYS_BEACON_FOUND				= 20,

    FLG_SYS_DEVICE_FOUND				= 0x40,
	FLG_SYS_DEVICE_SCAN_TIMEOUT			= 0x41,
	
	FLG_SYS_LINK_OTA					= 15,
	FLG_SYS_LINK_CONNECTED_OK			= 16,	// receive the first packet from slave / master
	FLG_SYS_LINK_NOTIFY_DATA			= 17,
};

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8	advA[6];			//address
	u8	data[40];			//0-31 byte
}rf_packet_adv_ind_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8	advA[6];			//address
	u8	data[31];			//0-31 byte
}rf_packet_adv_ind_module_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8	advA[6];			//address
	u8	initA[6];			//init address

}rf_packet_adv_dir_ind_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8	scanA[6];			//
	u8	advA[6];			//

}rf_packet_scan_req_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RA(1)_TA(1)_RFU(2)_TYPE(4)
	u8  rf_len;				//LEN(6)_RFU(2)
	u8	advA[6];			//address
	u8	data[31];			//0-31 byte

}rf_packet_scan_rsp_t;

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
}rf_packet_ll_init_t;

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
}rf_packet_ll_broadcast_t;

typedef struct{
	u32 dma_len;
	u8	type;
	u8  rf_len;
	u16	l2capLen;
	u16	chanId;
	u16 src_tx;
	u8 handle1; // for flag
    u8 sno[3];
    u16 src_adr;
    u16 dst_adr;
    u8 op;
    u16 vendor_id;
    u8 par[10];
    u8 internal_par1[5];
    u8 ttl;
    u8 internal_par2[4];
    u8 no_use[5];  // size must 48, when is set to be rf tx address.
}mesh_pkt_t;

//rf_packet_att_write_t
typedef struct{
    u8 sno[3];
    u8 src[2];
    u8 dst[2];
    u8 val[23];// op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
    // get status req: params[0]=tick  mac-app[2-3]=src-mac1...
    // get status rsp: mac-app[0]=ttc  mac-app[1]=hop-count
}rf_packet_att_value_t;

enum{
    op_type_1 = 1,
    op_type_2 = 2,
    op_type_3 = 3,
};

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)

	u16	l2cap;				//0x17
	u16	chanid;				//0x04,

	u8	att;				//0x12 for master; 0x1b for slave// as ttl when relay
	u8	hl;					// assigned by master
	u8	hh;					//
	u8	sno;

	u8	nid;
	u8  ttc;
	u16	group;

	u16	sid[2];

	u8	cmd[PKT_CMD_LEN];
}rf_packet_ll_data_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)

	u16	l2cap;				//0x17
	u16	chanid;				//0x04,

	u8	att;				//0x12 for master; 0x1b for slave// as ttl when relay
	u8	hl;				// assigned by master
	u8	hh;				//
	u8	init;
	u8	dat[14];
}rf_packet_ll_data_rsp_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)

	u16	l2cap;				//0x17
	u16	chanid;				//0x04,

	u16	op;
}rf_packet_ll_write_rsp_t;

#define		FLG_RF_RC_DATA			0xccdd
#define		FLG_RF_OTA_DATA			0xeedd
#define		FLG_RF_STATUS_DATA		0xffff
#define		FLG_RF_ALARM_TIME_DATA  0xeeff
#define		FLG_RF_MESH_CMD         0xff04

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	flag;

	u32	src_id;

	u8	att;				//0x12 for master; 0x1b for slave// as ttl when relay
	u8	hl;					// assigned by master
	u8	hh;					//
	u8	sno;

	u8	nid;
	u8  ttc;
	u16	group;

	u32	dst_id;

	u8	cmd[PKT_CMD_LEN];
}rf_packet_ll_rc_data_t;//rf_packet_att_cmd_t
typedef struct{
	u32	src_id;

	u8	att;				//0x12 for master; 0x1b for slave// as ttl when relay
	u8	hl;					// assigned by master
	u8	hh;					//
	u8	sno;

	u8	nid;
	u8  ttc;
	u16	group;

	u32	dst_id;

	u8	cmd[PKT_CMD_LEN];			//byte
}	ll_packet_data_t;

typedef struct{
	u8	nid;
	u8  ttc;
	u16	group;
	u32	sid;
	u8	cmd[PKT_CMD_LEN];
}rf_custom_dat_t;

typedef struct{
	u16	ManufactureID;  // must vendor id to follow spec
	u16 MeshProductUUID;
	u32	MacAddress;// low 4 byte
}ll_adv_private_t;
extern u8 *p_adv_pri_data;

typedef struct{
	u16	ManufactureID;  // must vendor id to follow spec
	u16 MeshProductUUID;
	u32	MacAddress;// low 4 byte
	u16 ProductUUID;
	u8	status;
	u8  DeviceAddress[2];
	u8  rsv[16];
}ll_adv_rsp_private_t;
extern u8 *p_adv_rsp_data;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	flag;

	u32	adr;

	u8	dat[32];
	u16	crc;

}rf_packet_ll_ota_data_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	l2cap;				//0x17
	u16	chanid;				//0x04,

	u8	att;				//0x12 for master; 0x1b for slave// as ttl when relay
	u8	hl;					// assigned by master
	u8	hh;					//

	u8	dat[23];

}rf_packet_att_data_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)

	u8	opcode;				//
	u8	dat[1];				//
}rf_packet_ll_control_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)

	u16	l2cap;				//0x17
	u16	chanid;				//0x04,

	u8	att;				//0x12 for master; 0x1b for slave//as ttl when relay
	u8	hl;					// assigned by master
	u8	hh;					//
	u8	sno;

	u8	ctype;
	u8	cmd[PKT_CMD_LEN+7];				//byte
}rf_packet_ll_write_data_t;

typedef struct{
	u8	type;
	u8	len;
	u16	l2cap;
	u16	chanid;				//0x04,
	u8	att;				//0x12 for master; 0x1b for slave// as ttl when relay
	u8	hl;				// assigned by master
	u8	hh;				//
	u8	sid;
	u8	ctype;			//type of command: mouse/keyboard/mic
	u8	cmd[PKT_CMD_LEN+7];
}rf_custom_master_dat_t;

typedef struct{
	u8  sno[3];
	u8	src[2];
	u8	dst[2];
	u8	dat[1];
}rf_custom1_dat_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8	type;				//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
	u8  rf_len;				//LEN(5)_RFU(3)
	u16	flag;

	rf_custom1_dat_t data;
}rf_packet_user_data_t;

typedef struct{
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle;
	u8 handle1;
	u8 value[30];
}ll_packet_l2cap_data_t;

typedef struct{
	u32 dma_len;
	u8	type;
	u8  rf_len;
	u16	l2capLen;
	u16	chanId;
	u8  code;
	u8  id;
	u16 dataLen;
	u16  result;
}rf_pkt_l2cap_sig_connParaUpRsp_t;

#endif
