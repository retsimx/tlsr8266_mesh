/********************************************************************************************************
 * @file     bqb_rf.h 
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
//extern  void  just_a_test(void);
/**********************************************************************************************************************
* 文件名称： rf.h
* 内容摘要：8267_bleplus
* 当前版本： V1.0
* 作    者：陈凯鑫
* 完成日期： 2015.06.25
*
* 修改记录1：
**********************************************************************************************************************/
#ifndef		RF_H
#define     RF_H
#include "../proj/tl_common.h"

#if(!(__TL_LIB_8258__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8258)))
/**********************************************************************************************************************
 *                                            全局常量定义                                                            *
 *********************************************************************************************************************/
//#define		RF_CRYSTAL_12M_OR_16M        0x01
//#define		RF_LONG_PACKET_EN			 0x02
//#define		RF_MANUAL_AGC                0x04
//#define RF_CHN_TABLE 		0x8000
//#define RF_SET_TX_MANAUL	0x4000
enum  M_OSC_SEL {
  RF_OSC_16M = 0,  
  RF_OSC_12M = 1
};
enum  M_RF_TR_MODE {
	STX = 0,
	STR = 1,
	SRT = 2,
};
enum  M_RF_MODE {     // now only 2M mode is supported
	 RF_MODE_BLE_2M = 0,
	 RF_MODE_BLE_1M = 1,
	 RF_MODE_ZIGBEE_250K = 2
};
enum  M_RF_STATE {
  RF_MODE_TX = 0,
  RF_MODE_RX = 1,
  RF_MODE_AUTO=2,
  RF_MODE_OFF = 3

};
enum M_RF_POWER{
	RF_POWER_7dBm	= 0,
	RF_POWER_5dBm	= 1,
	RF_POWER_m0P6dBm	= 2,
	RF_POWER_m4P3dBm	= 3,
	RF_POWER_m9P5dBm	= 4,
	RF_POWER_m13P6dBm	= 5,
	RF_POWER_m18P8dBm	= 6,
	RF_POWER_m23P3dBm	= 7,
	RF_POWER_m27P5dBm	= 8,
	RF_POWER_m30dBm_bqb	= 9,
	RF_POWER_m37dBm_bqb	= 10,
	RF_POWER_OFF_bqb	= 11,
};

/*****************************************************************************************/
extern unsigned  char  TP_GAIN0[];////added by C.Q.W
extern unsigned  char  TP_GAIN1[];
/*****************************************************************************************/

/**********************************************************************************************************************
 *                                             全局函数原型                                                           *
 *********************************************************************************************************************/
extern enum   M_RF_MODE   g_rf_mode;

extern	int	 Rf_Init( enum  M_OSC_SEL  osc_sel,enum M_RF_MODE m);
extern	 void Rf_ModeSet(enum M_RF_MODE m);
extern void Rf_BaseBandReset (void);
extern   unsigned char  Rf_TrxStateSet(enum M_RF_STATE  state ,signed char channel);
extern unsigned char  Rf_TrxStateSet_emi(enum M_RF_STATE  state ,signed char channel);

extern  void Rf_TxPkt (unsigned char* addr);
extern	void Rf_PowerLevelSet(enum M_RF_POWER level);
//extern   void  Rf_SetMaxRcvLen(unsigned char len);
extern void  Rf_RxBufferSet(unsigned char *  addr, int size, unsigned char  pingpong_en);
extern unsigned char Rf_RxBufferRequest(void);
extern void  Rf_RxBufferClearFlag(unsigned char  idx);
extern unsigned char  RF_TxFinish(void);
extern void Rf_TxFinishClearFlag (void);

//extern void Rf_StxrxFifoSetting(void* tx_addr,void* rx_addr,unsigned short tx_single_fifo_length,unsigned short rx_single_fifo_length,unsigned char tx_fifo_size,unsigned char rx_fifo_size);
//extern unsigned char *Rf_RxReadDataAddr(void);
//extern unsigned char *Rf_TxPushPktAddr(void);
extern void Rf_StartStx  (unsigned char* tx_addr,unsigned int tick);
extern void Rf_StartStxToRx  ( unsigned char* tx_addr ,unsigned int tick,unsigned short timeout_us);
extern void Rf_StartSrxToTx  (unsigned char* tx_addr  ,unsigned int tick,unsigned int timeout_us);

extern void Rf_TxAccessCodeSelect (unsigned char  idx);
extern void Rf_AccessCodeSetting01 (unsigned char  idx,unsigned long long  access_code);
extern void Rf_AccessCodeSetting2345 (unsigned char  idx,unsigned char  prefix);
extern void Rf_RxAccessCodeEnable (unsigned char enable);
extern void Rf_AccessCodeLengthSetting (unsigned char length);


extern void	Rf_EmiCarrierOnlyTest(enum M_RF_POWER power_level,signed char rf_chn);
extern void	Rf_EmiCarrierDataTest(enum M_RF_POWER power_level,signed char rf_chn);

extern unsigned char Rf_GetBleFreChannel(unsigned char chn);
#endif
#endif

