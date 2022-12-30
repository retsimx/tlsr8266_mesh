/********************************************************************************************************
 * @file     emi.c 
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
 * emitest.c
 *
 *  Created on: 2016-6-30
 *      Author: Administrator
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/Bqb_rf.h"

#if(MCU_CORE_TYPE != MCU_CORE_8258)

unsigned char  emi_var[5];
unsigned char  emi_tx[16]  __attribute__ ((aligned (4))) = {0xc,0x00,0x00,0x00,0x00,0x20,0xaa,0xbb};
unsigned char  prbs9[128];
int state0,state1,state2,state3;
unsigned char depth=1;
#define STATE0		0x1234
#define STATE1		0x5678
#define STATE2		0xabcd
#define STATE3		0xef01
#define  AUTO_TX     0
#define  PA_POWER_ON     1
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) test  initialization
*
*	@param		None
*
*
*	@return		None
*/

int Rf_EmiInit(void)
{
	// for registers recover.
	emi_var[0] = ReadAnalogReg(0xa5);
	emi_var[1] = read_reg8(0x8004e8);
	//emi_var[2] = read_reg8(0x800408);
	//emi_var[2] = read_reg8(0x800402);
	emi_var[3] = read_reg8(0x80050f);
	emi_var[4] = read_reg8(0x80050e);
	//emi_var[6] = read_reg8(0x800400);
	return 1;

}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) test  recovery setting
*
*	@param		None
*
*
*	@return		None
*/

int Rf_EmiCarrierRecovery(void)
{
	//reset zb & dma
	write_reg16(0x800060, 0x0480);
	write_reg16(0x800060, 0x0000);

	WriteAnalogReg (0xa5, emi_var[0]);
    write_reg8 (0x8004e8, emi_var[1]);

    if(( g_rf_mode == RF_MODE_BLE_2M ) )
	{
    	write_reg8 (0x800402, 0x26);
	}
	else if(g_rf_mode == RF_MODE_BLE_1M )
	{
		write_reg8 (0x800402, 0x26);
	}
	else if(g_rf_mode == RF_MODE_ZIGBEE_250K )
	{
		write_reg8 (0x800402, 0x26);
	}

//	write_reg8 (0x800402, emi_var[2]);
	write_reg8(0x80050f, emi_var[3]);
    write_reg8(0x80050e, emi_var[4]);
   // write_reg8(0x800400, emi_var[6]);
    return 1;

}

/*void phyTest_PRBS9 (unsigned char *p, int n)
{
	//PRBS9: (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100)
	unsigned short x = 0x1ff;
	int i;
	int j;
	for ( i=0; i<n; i++)
	{
		unsigned char d = 0;
		for (j=0; j<8; j++)
		{
			if (x & 1)
			{
				d |= BIT(j);
			}
			x = (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100);
		}
		*p++ = d;
	}
}*/


int pnGen(int state)
{
	int feed = 0;
	feed = (state&0x4000) >> 1;
	state ^= feed;
	state <<= 1;
	state = (state&0xfffe) + ((state&0x8000)>>15);
	return state;
}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) CarrierOnly Test
*
*	@param		power_level: set power level(0~14)
*				rf_chn	   : set tx channel((0 �� channel �� 100))
*
*
*	@return		None
*/
/*void Rf_EmiCarrierOnlyTest(enum M_RF_POWER power_level,signed char rf_chn)
{
	Rf_EmiCarrierRecovery();
	Rf_TrxStateSet_emi(RF_MODE_TX,rf_chn);
	WaitUs(150);//wait pllclock

	Rf_PowerLevelSet(power_level);
	WriteAnalogReg(0xa5,0x44);   // for carrier  mode
	write_reg8 (0x8004e8, 0x04); // for  carrier mode
}*/
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) CarrierData Test
*
*	@param		power_level: set power level(0~14)
*				rf_chn	   : set tx channel((0 �� channel �� 100))
*
*
*	@return		None
*/
/*
void Rf_EmiCarrierDataTest(enum M_RF_POWER power_level,signed char rf_chn)
{

	int i;

	Rf_EmiCarrierRecovery();
	
	Rf_PowerLevelSet(power_level);
	Rf_TrxStateSet_emi(RF_MODE_TX,rf_chn);
	WaitUs(150);//wait pllclock

	write_reg8(0x80050e,depth); // this size must small than the beacon_packet dma send length

	state0 = STATE0;
	state1 = STATE1;
	state2 = STATE2;
	state3 = STATE3;
	phyTest_PRBS9(prbs9,128);
	emi_tx[0] = depth*16-4;
	write_reg8(0x80050f, 0x80);  // must fix to 0x80
	write_reg8(0x800402, 0x21);	//preamble length=1
    Rf_TxPkt(emi_tx);
}*/
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) CarrierData Test Data Update
*
*	@param		None
*
*
*	@return		None
*/
unsigned char cnt;
void Rf_EmiDataUpdate(void)
{
//	write_reg32((emi_tx+depth*16-4),(state0<<16)+state1); // the last value
//	//advance PN generator
//	state0 = pnGen(state0);
//	state1 = pnGen(state1);
	//write_reg32((emi_tx+depth*16-4),0xf0f0f0f0); // the last value
	//phyTest_PRBS9((emi_tx+12),0x04);
//	int i;
//	for(i=0;i<64;i++)
//	{
//		phyTest_PRBS9((emi_tx+depth*16-4),64,i);
//	}
	if(cnt>=128)
	{
		cnt=0;
	}
	emi_tx[depth*16-1]=prbs9[cnt++];
	emi_tx[depth*16-2]=prbs9[cnt++];
	emi_tx[depth*16-3]=prbs9[cnt++];
	emi_tx[depth*16-4]=prbs9[cnt++];

	//phyTest_PRBS9((emi_tx+depth*16-4),64,cnt++);



}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) Rx Test
*
*	@param		addr       :set receiving address pointer
*
*				buffer_size: set power level(0~14)
*				rf_chn	   : set tx channel((0 �� channel �� 100))
*
*
*	@return		None
*/

void Rf_EmiRxTest(unsigned char *addr,signed char rf_chn,unsigned char buffer_size,unsigned char  pingpong_en)
{


	Rf_RxBufferSet(addr,buffer_size,pingpong_en);
	Rf_TrxStateSet_emi(RF_MODE_RX,rf_chn);
	WaitUs(200);//wait pllclock
	Rf_EmiCarrierRecovery();

	//Rf_BaseBandReset();
}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) Tx Test initialization
*
*	@param		power_level: set power level(0~14)
*				rf_chn	   : set tx channel((0 �� channel �� 100))
*
*
*	@return		None
*/

void Rf_EmiTxInit(enum M_RF_POWER power_level,signed char rf_chn)
{

	Rf_PowerLevelSet(power_level);
#if (AUTO_TX)
	    Rf_TrxStateSet_emi(RF_MODE_AUTO,rf_chn);
#else
		Rf_TrxStateSet_emi(RF_MODE_TX,rf_chn);
#endif

	WaitUs(200);//wait pllclock

	//Rf_BaseBandReset();
	Rf_EmiCarrierRecovery();

}
/********************************************************
*
*	@brief		Emi ( Electro Magnetic Interference ) CarrierData Test
*
*	@param		addr       :set tx address pointer
*
*
*	@return		None
*/

void Rf_EmiSingleTx(unsigned char *addr,enum M_RF_POWER power_level)
{


#if	PA_POWER_ON
	Rf_PowerLevelSet(power_level);
	WriteAnalogReg(0xa5,0x04);   // for carrier  mode
	write_reg8 (0x8004e8, 0x04); // for  carrier mode//tx_cyc1
#endif
#if (AUTO_TX)
	write_reg8(0x800f16, read_reg8(0x800f16) &(0xfb));	// Enable cmd_schedule mode
	reg_dma_rf_tx_addr = (unsigned short)(addr);
	write_reg8 (0x800f00, 0x85);
	//wait start cmd
	while(Rf_FsmIsIdle());
	//wait for cmd_done
	while (!Rf_FsmIsIdle());
#else
	Rf_TxPkt(addr);
	while(RF_TxFinish()==0);
	Rf_TxFinishClearFlag();
#if	PA_POWER_ON
	//WaitUs(100);
	//Rf_PowerLevelSet(RF_POWER_6dBm);
	//WaitUs(100);
	//Rf_PowerLevelSet(RF_POWER_5dBm);
	//WaitUs(100);
	//Rf_PowerLevelSet(RF_POWER_3P5dBm);
	//WaitUs(100);
	//Rf_PowerLevelSet(RF_POWER_1P65dBm);
	//WaitUs(100);
	//Rf_PowerLevelSet(RF_POWER_m0P6dBm);
//	//WaitUs(100);
//	Rf_PowerLevelSet(RF_POWER_m4P3dBm);
	///Rf_PowerLevelSet(RF_POWER_m9P5dBm);
//	Rf_PowerLevelSet(RF_POWER_m13P6dBm);
	//Rf_PowerLevelSet(RF_POWER_m18P8dBm);
	//Rf_PowerLevelSet(RF_POWER_m23P3dBm);
//	Rf_PowerLevelSet(RF_POWER_m27P5dBm);
	Rf_PowerLevelSet(RF_POWER_m30dBm);
	//Rf_PowerLevelSet(RF_POWER_m37dBm);
	//Rf_PowerLevelSet(RF_POWER_OFF);
#endif
#endif
}
#endif
