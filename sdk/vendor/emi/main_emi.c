/********************************************************************************************************
 * @file     main_emi.c 
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
 * Main_emi.c
 *
 *  Created on: Sep 1, 2014
 *      Author: qcmao
 */
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/emi.h"
#include "../common/rf_frame.h"
#include "../common/common.h"
#include "../../proj_lib/Bqb_rf.h"

#if FLASH_128K
#define EMI_TEST_TX_MODE  			0xf005
#define EMI_TEST_RUN  				0xf006
#define EMI_TEST_CMD  				0xf007
#define EMI_TEST_POWER_LEVEL  		0xf008
#define EMI_TEST_CHANNEL  			0xf009
#define EMI_TEST_MODE  				0xf00a
#define EMI_TEST_CD_MODE_HOPPING_CHN 0xf00b

#define CAP_VALUE					0x1e000
#define TP_LOW_VALUE				0x1e040
#define TP_HIGH_VALUE				0x1e041
#else
#define EMI_TEST_TX_MODE  			0x3f005
#define EMI_TEST_RUN  				0x3f006
#define EMI_TEST_CMD  				0x3f007
#define EMI_TEST_POWER_LEVEL  		0x3f008
#define EMI_TEST_CHANNEL  			0x3f009
#define EMI_TEST_MODE  				0x3f00a
#define EMI_TEST_CD_MODE_HOPPING_CHN 0x3f00b

#define CAP_VALUE					0x77000
#define TP_LOW_VALUE				0x77040
#define TP_HIGH_VALUE				0x77041
#endif
#define TX_PACKET_MODE_ADDR 		0x808005
#define RUN_STATUE_ADDR 			0x808006
#define TEST_COMMAND_ADDR			0x808007
#define POWER_ADDR 					0x808008
#define CHANNEL_ADDR				0x808009
#define RF_MODE_ADDR				0x80800a
#define CD_MODE_HOPPING_CHN			0x80800b

/********************************
cmd_now:cmd _id,reference to struct ate_list .
tx_mode:total tx packet counts,0->Unlimited    1->1000	
**********************************/
unsigned char crc_flag=0;
unsigned long bug_cc;
unsigned char  mode=1;
unsigned char  power_level = 0;
unsigned char  chn = 2;
unsigned char cmd_now=1;//reference to ate_list
unsigned char run=0;
unsigned char  *packet;
unsigned char  tx_mode=1;
unsigned char  hopping_enable=0;

#undef MAX_RF_CHANNEL
#define MAX_RF_CHANNEL  40
const unsigned char rf_chn_hipping[MAX_RF_CHANNEL] = {
	48,  4, 66, 62,44,
	12, 38, 16 ,26 ,20,
	22, 68, 18, 28,	42,
	32, 34, 36, 14,	40,
	30, 54, 46, 2,	50,
	52, 80, 56, 78,	74,
	8,  64, 6,  24,	70,
	72, 60, 76, 58, 10,
};

//zigbee 250K
unsigned char  zigbee_tx_packet[64]  __attribute__ ((aligned (4))) = {36,0x00,0x00,0x00,37,0x11,0x22,0x33,0x55,0x33,0x44};
//Ble 1M
unsigned char  ble_tx_packet [64]  __attribute__ ((aligned (4))) = {39, 0, 0, 0,0, 37,	0, 1, 2, 3, 4, 5, 6, 7};

u8 rx_packet[128] __attribute__ ((aligned (4)));

//0280510f
typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;
	u8 data[30]; //30 : 400us(1MBPS)

}rf_packet_emi_t;

rf_packet_emi_t pkt_km = {
		sizeof (rf_packet_emi_t) - 4,	//0x10=20-4,dma_len

		sizeof (rf_packet_emi_t) - 5,	//0x0f=20-5,rf_len
		RF_PROTO_BYTE,						// 0x51, proto
		PKT_FLOW_DIR,						// 0x80, kb data flow
		FRAME_TYPE_KEYBOARD,				// 0x02, type

//		U32_MAX,			// gid0

		0,					// rssi
		0,					// per
		0,					// seq_no
		1,					// number of frame
};

#define LIGHT_IO_ENABLE    0
#if (LIGHT_IO_ENABLE)
#define LIGHT_IO    GPIO_PB6
void light_init(){
	gpio_set_func(LIGHT_IO, AS_GPIO); 
	gpio_set_input_en(LIGHT_IO, 0);
	gpio_set_output_en(LIGHT_IO, 1); 
	gpio_write(LIGHT_IO, 1);
}
#endif

enum{
    EMI_FRQ_L = 0,
    EMI_FRQ_M = 1,
    EMI_FRQ_H = 2, 
    EMI_FRQ_MODE_MAX, 
};

enum{
    EMI_MODE_CARRIER = 0,
    EMI_MODE_CD = 1,
    EMI_MODE_RX = 2,
    EMI_MODE_TX = 3,
    EMI_MODE_MAX,    
};

const u8 GAIN0[5] = {0x40,0x1d,0x40,0x40,0x1d};
const u8 GAIN1[5] = {0x39,0x19,0x39,0x39,0x19};
int Rf_TpInit()
{
	memcpy(TP_GAIN0, GAIN0, 5);// max size 5
	memcpy(TP_GAIN1, GAIN1, 5);
	return 1;
}

void Rf_UpdateTpValue(enum M_RF_MODE mode ,unsigned  char tp0,unsigned  char tp1)
{
	if((mode == RF_MODE_BLE_1M))//||(mode == RF_MODE_STANDARD_BLE_1M))
	{
		if(( tp0 < (0x1d+10)) && (tp0 > (0x1d-10)) && ( tp1 < (0x19+10)) && (tp1 > (0x19-10)))
		{
			TP_GAIN0[1]=tp0;
			TP_GAIN1[1]=tp1;
			TP_GAIN0[4]=tp0;
			TP_GAIN1[4]=tp1;
		}

	}
	else
	{
		if(( tp0 < (0x40+10)) && (tp0 > (0x40-10)) && ( tp1 < (0x39+10)) && (tp1 > (0x39-10)))
		{
			TP_GAIN0[0]=tp0;
			TP_GAIN1[0]=tp1;
			TP_GAIN0[2]=tp0;
			TP_GAIN1[2]=tp1;
			TP_GAIN0[3]=tp0;
			TP_GAIN1[3]=tp1;
		}

	}

}

unsigned  char g_tp0;
unsigned  char g_tp1;
void read_flash_para(void)
{
//	unsigned char  tp0,tp1;
	u8 cap;
	unsigned char temp=0;
	flash_read_page(EMI_TEST_POWER_LEVEL,1,&temp);
	if( temp != 0xff )
	{
	
		power_level=temp;
		write_reg8(POWER_ADDR,power_level);
	}

	flash_read_page(EMI_TEST_CHANNEL,1,&temp);
	if( temp != 0xff )
	{
		chn=temp;
		write_reg8(CHANNEL_ADDR,chn);
	}
	flash_read_page(EMI_TEST_MODE,1,&temp);
	if( temp != 0xff )
	{
	
		mode=temp;
		write_reg8(RF_MODE_ADDR,mode);
	}
	flash_read_page(EMI_TEST_CMD,1,&temp);
	if(temp != 0xff )
	{
	
		cmd_now=temp;
		write_reg8(TEST_COMMAND_ADDR,cmd_now);
	}
	flash_read_page(EMI_TEST_TX_MODE,1,&temp);
	if( temp != 0xff )
	{
		tx_mode=temp;
		write_reg8(TX_PACKET_MODE_ADDR,tx_mode);
	}
	flash_read_page(CAP_VALUE,1,&cap);
	if(cap != 0xff && cap > 0xbf && cap < 0xe0 )
	{
		WriteAnalogReg(0x81,cap);
	}
	else
	{
		WriteAnalogReg(0x81,0xd0);
	}
	flash_read_page(TP_LOW_VALUE,1,&g_tp0);
	flash_read_page(TP_HIGH_VALUE,1,&g_tp1);
	flash_read_page(EMI_TEST_CD_MODE_HOPPING_CHN,1,&temp);
	if( temp != 0xff )
	{
		hopping_enable=temp;
		write_reg8(CD_MODE_HOPPING_CHN,hopping_enable);
	}
	/*if( (tp0 != 0xff ) && (tp1 != 0xff))
	{
		g_tp0 = tp0;
		g_tp1 = tp1;
		if(mode == 1)//1M
		{
			if(( tp0 < (0x1d+10)) && (tp0 > (0x1d-10)) && ( tp1 < (0x19+10)) && (tp1 > (0x19-10)))
			{
				Rf_UpdateTpValue(mode ,tp0, tp1);
			}

		}
		else if(mode == 0 || mode == 2)//2M/250K
		{
			if(( tp0 < (0x40+10)) && (tp0 > (0x40-10)) && ( tp1 < (0x39+10)) && (tp1 > (0x39-10)))
			{
				Rf_UpdateTpValue(mode ,tp0, tp1);
			}
		}

	}*/



}

void EmiCarrierOnly(enum M_RF_POWER power_level,signed char rf_chn)
{
	Rf_EmiCarrierOnlyTest(power_level,rf_chn);
}

extern u32 tick_per_us;
void EmiCarrierData(enum M_RF_POWER power_level,signed char rf_chn)
{
	unsigned char run = read_reg8(RUN_STATUE_ADDR);  // get the run state!
	unsigned char cmd_now = read_reg8(TEST_COMMAND_ADDR) ;
	unsigned char power = read_reg8(POWER_ADDR);
	unsigned char chn = read_reg8(CHANNEL_ADDR);
	unsigned char mode=read_reg8(RF_MODE_ADDR);
	unsigned char tx_mode_temp=read_reg8(TX_PACKET_MODE_ADDR);
	unsigned char hopping=read_reg8(CD_MODE_HOPPING_CHN);
	int i,j;
    if(hopping_enable==0)
    {
    	Rf_EmiCarrierRecovery();
    	Rf_EmiCarrierDataTest(power_level,rf_chn);

    	while( ((read_reg8(RUN_STATUE_ADDR)) == run ) &&  ((read_reg8(TEST_COMMAND_ADDR)) == cmd_now )\
			&& ((read_reg8(POWER_ADDR)) == power ) &&  ((read_reg8(CHANNEL_ADDR)) == chn )\
			&& ((read_reg8(RF_MODE_ADDR)) == mode )&&  ((read_reg8(TX_PACKET_MODE_ADDR)) == tx_mode_temp )\
			&&  ((read_reg8(CD_MODE_HOPPING_CHN)) == hopping )) // if not new command
		{
			Rf_EmiDataUpdate();
		}
    }
    else
    {
    	while( ((read_reg8(RUN_STATUE_ADDR)) == run ) &&  ((read_reg8(TEST_COMMAND_ADDR)) == cmd_now )\
    		&& ((read_reg8(POWER_ADDR)) == power ) &&  ((read_reg8(CHANNEL_ADDR)) == chn )\
    		&& ((read_reg8(RF_MODE_ADDR)) == mode )&&  ((read_reg8(TX_PACKET_MODE_ADDR)) == tx_mode_temp )\
    		&&  ((read_reg8(CD_MODE_HOPPING_CHN)) == hopping )) // if not new command
    	{
    		for(j=0;j<MAX_RF_CHANNEL;j++)
    		{
    			for(i=0; i<MAX_RF_CHANNEL; i++){
					if( ((read_reg8(RUN_STATUE_ADDR)) == run ) &&  ((read_reg8(TEST_COMMAND_ADDR)) == cmd_now )\
						&& ((read_reg8(POWER_ADDR)) == power ) &&  ((read_reg8(CHANNEL_ADDR)) == chn )\
						&& ((read_reg8(RF_MODE_ADDR)) == mode )&&  ((read_reg8(TX_PACKET_MODE_ADDR)) == tx_mode_temp )\
						&&  ((read_reg8(CD_MODE_HOPPING_CHN)) == hopping )) // if not new command
					{
						Rf_EmiCarrierRecovery();
						Rf_EmiCarrierDataTest(power_level,rf_chn_hipping[(j+i)%MAX_RF_CHANNEL]);
						unsigned int tick = clock_time();
						while(((unsigned int)(clock_time() - tick) <= 1000 * tick_per_us))//1ms
						{
							Rf_EmiDataUpdate();
						}
						Rf_EmiRxTest(rx_packet,rf_chn_hipping[i],128,0);
						WaitUs(10000);//10ms
						//WaitUs(1000000);//10ms

					}
					else
					{
						break;
					}
				}
    		}

    	}

    }
}
unsigned long rx_packet_num;
void EmiRx(enum M_RF_POWER power_level,signed char rf_chn)
{
/*	unsigned char rx_now;
	unsigned char run = read_reg8(RUN_STATUE_ADDR);  // get the run state!
	unsigned char cmd_now = read_reg8(TEST_COMMAND_ADDR) ;
	unsigned char power = read_reg8(POWER_ADDR);
	unsigned char chn = read_reg8(CHANNEL_ADDR);
	unsigned char mode=read_reg8(RF_MODE_ADDR);*/
	irq_enable();
	reg_irq_mask |= FLD_IRQ_ZB_RT_EN;
	reg_rf_irq_mask |= FLD_RF_IRQ_RX;
	Rf_EmiRxTest(rx_packet,rf_chn,128,0);

}

void EmiTxPrbs9(enum M_RF_POWER power_level,signed char rf_chn)
{
	unsigned char run = read_reg8(RUN_STATUE_ADDR);  // get the run state!
	unsigned char cmd_now = read_reg8(TEST_COMMAND_ADDR) ;
	unsigned char power = read_reg8(POWER_ADDR);
	unsigned char chn = read_reg8(CHANNEL_ADDR);
	unsigned char mode=read_reg8(RF_MODE_ADDR);
	unsigned char tx_mode_temp=read_reg8(TX_PACKET_MODE_ADDR);
	unsigned char hopping=read_reg8(CD_MODE_HOPPING_CHN);
//	unsigned char rx_now;
	unsigned int tx_cnt=0;
	Rf_EmiTxInit(power_level,rf_chn);
	while( ((read_reg8(RUN_STATUE_ADDR)) == run ) &&  ((read_reg8(TEST_COMMAND_ADDR)) == cmd_now )\
		&& ((read_reg8(POWER_ADDR)) == power ) &&  ((read_reg8(CHANNEL_ADDR)) == chn )\
		&& ((read_reg8(RF_MODE_ADDR)) == mode )&&  ((read_reg8(TX_PACKET_MODE_ADDR)) == tx_mode_temp )\
		&&  ((read_reg8(CD_MODE_HOPPING_CHN)) == hopping )) // if not new command
	{
		if((mode==1)||(mode==0))
		{
			packet[4] = 0;//type
			phyTest_PRBS9 (packet + 6, 37);

		}
		else
		{
			phyTest_PRBS9 (packet + 5, 37);

		}

		Rf_EmiSingleTx(packet,power_level);
		if(tx_mode==1)
		{
			tx_cnt++;
			if(tx_cnt>=1000)
			{
				break;
			}
		}

	}

}

void EmiTx55(enum M_RF_POWER power_level,signed char rf_chn)
{
	unsigned char run = read_reg8(RUN_STATUE_ADDR);  // get the run state!
	unsigned char cmd_now = read_reg8(TEST_COMMAND_ADDR) ;
	unsigned char power = read_reg8(POWER_ADDR);
	unsigned char chn = read_reg8(CHANNEL_ADDR);
	unsigned char mode=read_reg8(RF_MODE_ADDR);
	unsigned char tx_mode_temp=read_reg8(TX_PACKET_MODE_ADDR);
	unsigned char hopping=read_reg8(CD_MODE_HOPPING_CHN);
//	unsigned char rx_now;
	unsigned int tx_cnt=0;
	Rf_EmiTxInit(power_level,rf_chn);
	while( ((read_reg8(RUN_STATUE_ADDR)) == run ) &&  ((read_reg8(TEST_COMMAND_ADDR)) == cmd_now )\
		&& ((read_reg8(POWER_ADDR)) == power ) &&  ((read_reg8(CHANNEL_ADDR)) == chn )\
		&& ((read_reg8(RF_MODE_ADDR)) == mode )&&  ((read_reg8(TX_PACKET_MODE_ADDR)) == tx_mode_temp )\
		&&  ((read_reg8(CD_MODE_HOPPING_CHN)) == hopping )) // if not new command
	{
		int i;
		if((mode==1)||(mode==0))
		{
			packet[4] = 2;//type

			for( i=0;i<37;i++)
			{
				packet[6+i]=0x55;
			}

		}
		else
		{
			for( i=0;i<37;i++)
			{
				packet[5+i]=0x55;
			}

		}
		Rf_EmiSingleTx(packet,power_level);
		if(tx_mode==1)
		{
			tx_cnt++;
			if(tx_cnt>=1000)
			{
				break;
			}

		}

	}

}

void EmiTxff(enum M_RF_POWER power_level,signed char rf_chn)
{
	unsigned char run = read_reg8(RUN_STATUE_ADDR);  // get the run state!
	unsigned char cmd_now = read_reg8(TEST_COMMAND_ADDR) ;
	unsigned char power = read_reg8(POWER_ADDR);
	unsigned char chn = read_reg8(CHANNEL_ADDR);
	unsigned char mode=read_reg8(RF_MODE_ADDR);
	unsigned char tx_mode_temp=read_reg8(TX_PACKET_MODE_ADDR);
	unsigned char hopping=read_reg8(CD_MODE_HOPPING_CHN);
//	unsigned char rx_now;
	unsigned int tx_cnt=0;
	Rf_EmiTxInit(power_level,rf_chn);
	while( ((read_reg8(RUN_STATUE_ADDR)) == run ) &&  ((read_reg8(TEST_COMMAND_ADDR)) == cmd_now )\
		&& ((read_reg8(POWER_ADDR)) == power ) &&  ((read_reg8(CHANNEL_ADDR)) == chn )\
		&& ((read_reg8(RF_MODE_ADDR)) == mode )&&  ((read_reg8(TX_PACKET_MODE_ADDR)) == tx_mode_temp )\
		&&  ((read_reg8(CD_MODE_HOPPING_CHN)) == hopping )) // if not new command
	{
		int i;
		if((mode==1)||(mode==0))
		{
			packet[4] = 1;//type
			
			for( i=0;i<37;i++)
			{
				packet[6+i]=0x0f;
			}

		}
		else
		{
			for( i=0;i<37;i++)
			{
				packet[5+i]=0x0f;
			}

		}
		Rf_EmiSingleTx(packet,power_level);
		if(tx_mode==1)
		{
			tx_cnt++;
			if(tx_cnt>=1000)
			{
				break;
			}

		}
	}

}

// Test List
struct  test_list_s {
	unsigned char  cmd_id;
	void	 (*func)(enum M_RF_POWER power_level,signed char rf_chn);
};
struct  test_list_s  ate_list[] = {
		{0x01,EmiCarrierOnly},        // osc test
		{0x02,EmiCarrierData},

		{0x03,EmiRx},		//Rx
		{0x04,EmiTxPrbs9},
		{0x05,EmiTx55},
		{0x06,EmiTxff},
};

void main_loop(void)
{
	static int first_flg = 0;
#if(PA_ENABLE)
    pa_init(0, 0);
#endif

#if (LIGHT_IO_ENABLE)
    light_init();
#endif
	Rf_EmiInit();
	Rf_TpInit();
	tick_per_us = CLOCK_SYS_CLOCK_1US;
	write_reg8(RUN_STATUE_ADDR,run);//run
	write_reg8(TEST_COMMAND_ADDR,cmd_now);//cmd
	write_reg8(POWER_ADDR,power_level);//power
	write_reg8(CHANNEL_ADDR,chn);//chn
	write_reg8(RF_MODE_ADDR,mode);//mode
	write_reg8(TX_PACKET_MODE_ADDR,tx_mode);//tx_mode
	write_reg8(CD_MODE_HOPPING_CHN,hopping_enable);//hipping channel
#if 0
	read_flash_para();
	phyTest_PRBS9(ble_tx_packet + 6, 37);
#endif
	write_reg32(0x800408, 0x29417671);	//accesscode: 1001-0100 1000-0010 0110-1110 1000-1110   29 41 76 71	
#if EMI_USE_EXTERNAL_CAP
	WriteAnalogReg (0x80, 0x21);
	WriteAnalogReg (0x81, 0xc0);
#endif
	
	while(1)
	{
	  
	   run = read_reg8(RUN_STATUE_ADDR);  // get the run state!
	   if(!first_flg || run!=0)
	   {
			if(!first_flg){
				first_flg = 1;
			}
			else{
			   power_level = read_reg8(POWER_ADDR);
			   chn = read_reg8(CHANNEL_ADDR);
			   mode=read_reg8(RF_MODE_ADDR);
			   cmd_now = read_reg8(TEST_COMMAND_ADDR);	// get the command!
			   tx_mode = read_reg8(TX_PACKET_MODE_ADDR);  // get the command!
			   hopping_enable = read_reg8(CD_MODE_HOPPING_CHN);  // get the command!
			}
			irq_disable();

			write_reg32(0x80800c,0);
			write_reg8(0x808004,0);

			for (u8 i=0; i<sizeof (ate_list)/sizeof (struct test_list_s); i++)
			{
				if(cmd_now == ate_list[i].cmd_id)
				{
#if 0
					if(mode==1)
					{
						Rf_ModeSet(RF_MODE_BLE_1M);
					//	write_reg8 (0x800402,  0x20|(preamble&0x1f));
						write_reg32 (0x800408, 0x29417671); //accesscode: 1001-0100 1000-0010 0110-1110 1000-1110	29 41 76 71
						packet = ble_tx_packet;
						Rf_UpdateTpValue(RF_MODE_BLE_1M ,g_tp0,g_tp1);
					}
					else if(mode==0)
					{
						Rf_ModeSet(RF_MODE_BLE_2M);
						//write_reg8 (0x800402,  0x20|(preamble&0x1f));
						write_reg32 (0x800408, 0x29417671); //accesscode: 1001-0100 1000-0010 0110-1110 1000-1110	29 41 76 71
						packet = ble_tx_packet;
						Rf_UpdateTpValue(RF_MODE_BLE_2M ,g_tp0,g_tp1);
					}
					else if(mode==2)
					{
						Rf_ModeSet(RF_MODE_ZIGBEE_250K);
						packet = zigbee_tx_packet;
						Rf_UpdateTpValue(RF_MODE_ZIGBEE_250K ,g_tp0,g_tp1);
					}
#else
//default 1M mode
					g_rf_mode = RF_MODE_BLE_1M;
					packet = ble_tx_packet;
					Rf_UpdateTpValue(RF_MODE_BLE_1M ,g_tp0,g_tp1);
#endif
					ate_list[i].func(power_level,chn);
					break;

				}
			}

			run = 0;

			write_reg8(RUN_STATUE_ADDR, 0);

	   }

	}

}
