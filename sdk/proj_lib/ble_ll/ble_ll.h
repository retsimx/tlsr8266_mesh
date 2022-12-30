/********************************************************************************************************
 * @file     ble_ll.h 
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
#ifndef _RF_LIGHT_LINK_LAYER_H_
#define _RF_LIGHT_LINK_LAYER_H_

#include "../../proj/tl_common.h"

#include "ble_common.h"
#include "att.h"
#include "gatt_uuid.h"
#include "service.h"
#include "gatt_server.h"
#include "gap_const.h"
#include "../../vendor/common/crc.h"

typedef void (*irq_st_func) (void);

#define		SYS_LINK_NORMAL						0
#define		SYS_LINK_STOP_ON_RESPONSE			BIT(0)

#define		MASTER_CONNECT_ANY_DEVICE			BIT(6)

#define			LL_CHANNEL_MAP					{0xff, 0xff, 0xff, 0xff, 0x1f}
//#define			LL_CHANNEL_MAP					{0x0, 0x10, 0x0, 0x0, 0x0}
#define			LL_CHANNEL_HOP					0x0c

//74 4C 69 67 68 74 31 30
#define		TLIGHT_ID0			0x67694c74
#define		TLIGHT_ID1			0x30317468

#define			LGT_CMD_PAIRING_LIGHT_OFF		0xc1
#define			LGT_CMD_PAIRING_LIGHT_ON		0xc2
#define			LGT_CMD_PAIRING_LIGHT_SEL		0xc3
#define			LGT_CMD_PAIRING_LIGHT_CONFIRM	0xc4
#define			LGT_CMD_PAIRING_LIGHT_RESPONSE	0xc9
#define			LGT_CMD_PAIRING_LIGHT_OTA_EN	0xce
#define			LGT_CMD_MESH_PAIR_TIMEOUT		0xcf

#define			LGT_CMD_LIGHT_ONOFF				0x10
#define         LIGHT_ON_PARAM                  0x01
#define         LIGHT_OFF_PARAM                 0x00
#define			LGT_CMD_LIGHT_ON				0x10
#define			LGT_CMD_LIGHT_OFF				0x11
#define			LGT_CMD_LIGHT_SET				0x12
#define			LGT_CMD_LIGHT_SCENE				0x18

#define			LGT_CMD_LUM_UP					0x20
#define			LGT_CMD_LUM_DOWN				0x21
#define			LGT_CMD_SAT_UP					0x22
#define			LGT_CMD_SAT_DOWN				0x23

#define			CUSTOM_DATA_MOUSE				0
#define			CUSTOM_DATA_KEYBOARD			1
#define			CUSTOM_DATA_SOMATIC				2
#define			CUSTOM_DATA_MIC					3

////////////////////////////////////////////////////////////////////////////////////////
static inline int ble_word_match (u8 *ps, u8 * pd)
{
	return (ps[0] == pd[0]) && (ps[1] == pd[1]) && (ps[2] == pd[2]) && (ps[3] == pd[3]);
}

void	ble_set_tick_per_us (int tick);

void	irq_handler(void);
void	irq_ble_master_handler ();
void	irq_ble_slave_handler ();

u8		ble_master_get_sno ();
void	ble_master_set_sno (u8 sno);

void	ble_master_init ();
int		ble_master_start (u32 interval, u32 timeout, void *p_data, int mode);
int		ble_master_command (u8 * p);
void 	ble_master_stop ();
void *	ble_master_get_response ();
int		ble_master_status ();

void	ble_master_set_slave_mac (const u8 *p);
int		ble_master_write (u8 *p);
int		ble_master_ll_data (u8 *p, int n);
void	ble_master_set_channel_mask (u8 * p);

void	ble_slave_init (u32 interval, u32 timeout, int sniffer_mode);
void	ble_slave_proc (void);
int		ble_slave_connected ();
void 	ble_set_debug_adv_channel (u8 chn);
void 	ble_slave_set_adv (const u8 * tbl_pkt_adv, int n);
void 	ble_slave_set_adv_interval (int ms);
void	ble_slave_set_rsp (u8 * tbl_pkt_rsp, int n);

void	ble_slave_set_att_handler (u8 * pAtt);

void	ble_event_callback (u8 status, u8 *p, u8 ble_rssi);
void 	ble_slave_data_callback (u8 *);
void	ble_master_data_callback (u8 *p);

int		ble_master_add_tx_packet (u32 p);

int		ble_ll_lastUnmappedCh;
int		ble_ll_channelNum;
u8		ble_ll_channelTable[40];
u8		ble_ll_chn;

u8*		ble_slave_p_mac;

///////////////////////////// SMP ///////////////////////////////////
#define			SMP_OP_PAIRING_REQ					1
#define			SMP_OP_PAIRING_RSP					2
#define			SMP_OP_PAIRING_CONFIRM				3
#define			SMP_OP_PAIRING_RANDOM				4
#define			SMP_OP_PAIRING_FAIL					5
#define			SMP_OP_ENC_INFO						6
#define			SMP_OP_ENC_IDX						7
#define			SMP_OP_ENC_IINFO					8
#define			SMP_OP_ENC_IADR						9
#define			SMP_OP_ENC_SIGN						0x0a
#define			SMP_OP_SEC_REQ						0x0b
#define			SMP_OP_WAIT							0x0f


typedef int (*smp_brx_handler_t)(u8 * p, int crcok);
typedef void (*smp_init_handler_t)(u8 *p);
typedef void (*smp_enc_handler_t)(u8 *p);

void		master_smp_func_init ();

#endif
