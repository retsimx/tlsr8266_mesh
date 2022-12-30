/********************************************************************************************************
 * @file     emi.h 
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
/*
 * emitest.h
 *
 *  Created on: 2016-6-30
 *      Author: Administrator
 */

#ifndef EMITEST_H_
#define EMITEST_H_
#include "../../proj_lib/bqb_rf.h"


extern int pnGen(int state);
extern int Rf_EmiInit(void);
extern void	Rf_EmiCarrierOnly(enum M_RF_POWER power_level,signed char rf_chn);
extern void	Rf_EmiCarrierData(enum M_RF_POWER power_level,signed char rf_chn);
extern void Rf_EmiDataUpdate(void);
extern void Rf_EmiRxTest(unsigned char *addr,signed char rf_chn,unsigned char buffer_size,unsigned char  pingpong_en);
extern unsigned char Rf_EmiTxInit(enum M_RF_POWER power_level,signed char rf_chn);
extern unsigned char Rf_EmiSingleTx(unsigned char *addr,enum M_RF_POWER power_level);
extern void phyTest_PRBS9 (unsigned char *p, int n);
extern int Rf_EmiCarrierRecovery(void);
extern u8  Rf_TrxStateSet_emi(enum M_RF_STATE  state ,signed char channel);

#endif /* EMITEST_H_ */
