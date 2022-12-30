/********************************************************************************************************
 * @file     rfhw.h 
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

#define RF_RX_PKT_BUFF_SIZE		(RF_PKT_BUFF_LEN * RF_RX_PKT_BUFF_COUNT)

#define RF_CHANNEL_MAX			16
#define RF_CHANNEL_MASK			(RF_CHANNEL_MAX - 1)

STATIC_ASSERT_POW2(RF_CHANNEL_MAX);		//  if not, may fail in rf_get_msk_chn and rf_get_sub_chn

enum{
	RF_RXTX_MODE_RX,
	RF_RXTX_MODE_TX,
};

#define RFHW_XTAL_VOLT						3			// 0x81 analog xtal vol setting

#if(APPLICATION_DONGLE)
#define RFHW_XTAL_FREQ_OFFSET_DEF			0x0e		// 0x81 analog xtal vol setting
#else
#define RFHW_XTAL_FREQ_OFFSET_DEF			0x0e		// 0x81 analog xtal vol setting
#endif
#define RFHW_XTAL_REGA_DEFALUT				(MASK_VAL(FLDA_XTAL_VOLT, RFHW_XTAL_VOLT) | MASK_VAL(FLDA_XTAL_FREQ_OFF, RFHW_XTAL_FREQ_OFFSET_DEF))

void rf_freq_compensation (s8 l, s8 h);

typedef struct{
    s8 i;
    s8 q;
    int a;
    int th;
}rf_chn_iq_dc_t;
void rfhw_detect_err(void);
void rfhw_init(void);
void rfhw_send_pkt(u8 *pkt);
int rf_send_pkt_async (void);
int rfhw_cca_clear(void);

static inline u8 rf_get_msk_chn(u8 chn){
	return chn & RF_CHANNEL_MASK;
}

static inline u8 rf_make_sub_chn(u8 chn){
	return (chn + RF_CHANNEL_MAX);
}

static inline u8 rf_is_sub_chn(u8 chn){
	return (chn >= RF_CHANNEL_MAX);
}

/////////////////////////////
// TX
////////////////////////////
extern u32 rf_tx_send_pkt_tick;
extern u32	rf_tx_send_pkt_busy;



