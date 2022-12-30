/********************************************************************************************************
 * @file     phytest.c 
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
#include "../../proj_lib/bqb_rf.h"
//#include "./drivers/uart.h"
#include "../../proj_lib/bqb_bsp.h"
//#include "./drivers/types.h"
#include "phytest.h"
#include "./Rf_test.h"
#include "../../proj/common/string.h"
#include "../../proj/mcu/analog.h"
#include "../../proj/common/compatibility.h"
#include "../../proj/common/utility.h"
#include "../../proj/common/Types.h"
#include "../../proj/drivers/Flash.h"
#include "../../proj_lib/pm_8267.h"
#include "../../proj/mcu/watchdog_i.h"
#if(MCU_CORE_TYPE != MCU_CORE_8258)

#define DBG_RX_RESULT    0
#define			CMD_RX				1
#define			CMD_TX				2
#define			CMD_END				3
#define			CMD_RESET			0
#define reg_rf_irq_mask			REG_ADDR16(0xf1c)
#define reg_rf_irq_status		REG_ADDR16(0xf20)
#if (DBG_RX_RESULT)
u8 rx_flg;
#endif
#define		RF_PACKET_LENGTH_OK(p)		(p[0] == (p[13]&0x3f)+17)
#define		RF_PACKET_CRC_OK(p)			((p[p[0]+3] & 0x51) == 0x40)
//#define CLOCK_SYS_CLOCK_HZ  	16000000

u8 bqb_en_flag = 0;
#if 0
enum{
	CLOCK_PLL_CLOCK = 192000000,

	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
	CLOCK_SYS_CLOCK_4S = CLOCK_SYS_CLOCK_1S << 2,
	CLOCK_MAX_MS = (U32_MAX / CLOCK_SYS_CLOCK_1MS),
	CLOCK_MAX_US = (U32_MAX / CLOCK_SYS_CLOCK_1US),
};

enum{
	FLD_RF_IRQ_RX = 			BIT(0),
	FLD_RF_IRQ_TX =				BIT(1),
	FLD_RF_IRQ_RX_TIMEOUT =		BIT(2),
	FLD_RF_IRQ_CRC =			BIT(4),
	FLD_RF_IRQ_CMD_DONE  =		BIT(5),
	FLD_RF_IRQ_RETRY_HIT =		BIT(7),
	FLD_RF_IRQ_FIRST_TIMEOUT =	BIT(10),
};
#endif

#if 1		// use shared memory
#define				LIGHT_PKT_BUFF_SIZE		64				// don't change
extern unsigned char light_rx_buff[LIGHT_PKT_BUFF_SIZE*4];
#define buffer_phytest	light_rx_buff
#else
unsigned char	buffer_phytest[256] __attribute__ ((aligned (4)));
#endif

u8		pkt_phytest [64] = {
		39, 0, 0, 0,
		0, 37,
		0, 1, 2, 3, 4, 5, 6, 7
};

void phyTest_PRBS9 (u8 *p, int n)
{
	//PRBS9: (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100)
	u16 x = 0x1ff;
	int i;
	int j;
	for ( i=0; i<n; i++)
	{
		u8 d = 0;
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
}
#if 0
// check if the current time is exceed span_us from ref time
static inline u32 clock_time_exceed(u32 ref, u32 span_us){
	return ((u32)(clock_time_bqb() - ref) > span_us * CLOCK_SYS_CLOCK_1US);
}
#endif
unsigned short uart_phyTestGet(unsigned short* cmd)
{

	static u32 tick_rx = 0;

	if (!tick_rx && (REG_ADDR8(0x9c)&15) == 1)
	{
		tick_rx = clock_time_bqb ();
	}
	else if ((REG_ADDR8(0x9c)&15) == 2)
	{
		u16		dat = REG_ADDR16 (0x90);

		*cmd = (dat >> 8) | (dat << 8);
		REG_ADDR8(0x9d) = 0xc0;		//clear
		tick_rx = 0;
		return 1;
	}
	else if (tick_rx && clock_time_exceed (tick_rx, 5000))
	{
		tick_rx = 0;
		REG_ADDR8(0x9d) = 0xc0;		//clear
	}
	return 0;
}
void uart_phyTestSend (unsigned short st)
{
	u16 dat = (st >> 8) | (st << 8);
	REG_ADDR16(0x90) = dat;
}
///////////////////////////////////////////////////////////////////////////////
//////PRBS9 and 0x0f and 0x55
#define MY_PACKET_LEN 32
//unsigned char my_packet_0x55[MY_PACKET_LEN] __attribute__ ((aligned (4))) = {0x22,0x00,0x00,0x00,0x00,0x20,0x00,0xaa,0xaa,0xaa,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
//unsigned char my_packet_0x0f[MY_PACKET_LEN] __attribute__ ((aligned (4))) = {0x22,0x00,0x00,0x00,0x00,0x20,0x00,0xaa,0xaa,0xaa,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
//unsigned char my_packet_PRBS9[MY_PACKET_LEN] __attribute__ ((aligned (4))) ={0x22,0x00,0x00,0x00,0x00,0x20,0x00,0xaa,0xaa,0xaa,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
unsigned char my_packet_0x55[] __attribute__ ((aligned (4))) = {0x22,0x00,0x00,0x00,0x00,0x20,0x00,0xaa,0xaa,0xaa,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
unsigned char my_packet_0x0f[] __attribute__ ((aligned (4))) = {0x22,0x00,0x00,0x00,0x00,0x20,0x00,0xaa,0xaa,0xaa,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
unsigned char my_packet_PRBS9[] __attribute__ ((aligned (4))) ={0x22,0x00,0x00,0x00,0x00,0x20,0x00,0xaa,0xaa,0xaa,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
void my_Rf_EmiCarrierDataTest(enum M_RF_POWER power_level,signed char rf_chn,unsigned char* my_packet)
{

	//////int i;
	//////int state0,state1,state2,state3, feed;
//	unsigned char cmd_now = read_reg8(0x808007) & 0x7f;	//unused
	Rf_TrxStateSet(RF_MODE_TX,rf_chn);
	Rf_PowerLevelSet(power_level);

	write_reg8(0x80050e,0x2); // this size must small than the beacon_packet dma send length
	#if 0
	state0 = STATE0;
	state1 = STATE1;
	state2 = STATE2;
	state3 = STATE3;
	write_reg32((packet+28),(state0<<16)+state1); // the last value
	#endif
	write_reg8(0x80050f, 0x80);  // must fix to 0x80
	write_reg8(0x800402, 0x21);	//preamble length=1
    Rf_TxPkt(my_packet);
    #if 0
    while(1)
	{
		write_reg32((packet+28),(state0<<16)+state1); // the last value
		//advance PN generator
		state0 = pnGen(state0);
		state1 = pnGen(state1);		
	}
	#endif
	//return 1;
}
/////////////////////////////////////////////////////////////////////////////
unsigned char EMI_count = 0;
u8 type;
void EMI_test(){
	u16 cmd;
//	u16 st = 0;
//	u16 pkts;
	u32 tick_tx;
	do{
		if(uart_phyTestGet (&cmd)){
			EMI_count++;/////for test
			tick_tx = clock_time_bqb ();
			static u8 chn,len;
			u8 rf_power_level;
			chn = (cmd >> 8) & 0x3f;/////get frequence
			len =  (cmd >> 2) & 0x3f;/////get power level
			rf_power_level = len;
			type = cmd & 3;////juge
//			u8 ct = cmd >> 14;
			if(type == 0x03){
				///set frequence / set power_level
				chn = Rf_GetBleFreChannel(phyTest_Channel(chn));
				Rf_EmiCarrierOnlyTest(rf_power_level,chn);
				/////////
				WriteAnalogReg(0xa3,ReadAnalogReg(0xa3)&0xef);/////close
				WriteAnalogReg(0xa3,ReadAnalogReg(0xa3)|0x10);//////open
				uart_phyTestSend (rf_power_level);
			}
			else if (type == 0x02)
			{
				/////stop EMI mode
				WriteAnalogReg(0xa5,0x00);   // for carrier  mode
				write_reg8 (0x8004e8, 0x00); // for  carrier mode
				/////
				write_reg32((int)(my_packet_0x55+28),(0x00005555<<16)+0x5555); // the last value
				chn = Rf_GetBleFreChannel(phyTest_Channel(chn));
				my_Rf_EmiCarrierDataTest(rf_power_level,chn,my_packet_0x55);/////carrier  while(1)
				////////
				WriteAnalogReg(0xa3,ReadAnalogReg(0xa3)&0xef);/////close
				WriteAnalogReg(0xa3,ReadAnalogReg(0xa3)|0x10);//////open
				uart_phyTestSend (0x55);
			}
			else if (type == 0x01)
			{
				/////stop EMI mode
				WriteAnalogReg(0xa5,0x00);   // for carrier  mode
			    write_reg8 (0x8004e8, 0x00); // for  carrier mode
				/////
				write_reg32((int)(my_packet_0x0f+28),(0x0000f0f0<<16)+0xf0f0); // the last value
				chn = Rf_GetBleFreChannel(phyTest_Channel(chn));
				my_Rf_EmiCarrierDataTest(rf_power_level,chn,my_packet_0x0f);/////carrier  while(1)
				////////
				WriteAnalogReg(0xa3,ReadAnalogReg(0xa3)&0xef);/////close
				WriteAnalogReg(0xa3,ReadAnalogReg(0xa3)|0x10);//////open
				uart_phyTestSend (0xf0);
			}
			else if(type == 0x00){////the destination is that whether test is ok
				/////stop EMI mode
				WriteAnalogReg(0xa5,0x00);   // for carrier  mode
				write_reg8 (0x8004e8, 0x00); // for  carrier mode
				/////
				chn = Rf_GetBleFreChannel(phyTest_Channel(chn));
				my_Rf_EmiCarrierDataTest(rf_power_level,chn,my_packet_PRBS9);
				uart_phyTestSend (23);
				while(1){
					phyTest_PRBS9((my_packet_PRBS9+28),0x04);
				}
			}
			else{}
		}
	}while(1);
}

u32	dbg_tx, dbg_rx, dbg_end, dbg_pkts;
void phytest ()
{
//	static u32	dbg_tx, dbg_rx, dbg_end, dbg_pkts;
	//accesscode: 1001-0100 1000-0010 0110-1110 1000-1110   29 41 76 71
	// pn disable: REG_ADDR8(0x401) = 0; REG_ADDR8(0x401) = 0x08;
	//PRBS9: (x >> 1) | (((x<<4) ^ (x<<8)) & 0x100)
	// 37-byte: 675 us interval
	// 0:PRBS9; 1: 0f; 2: 55
	// test_status_event: 0001 00000
	// test_report: BIT(15) | packet_count
	// test_reset:	0x0000
	// test_end:	0xc000
	// test_tx:		0x8000 | (chn << 8) | (len << 2) | type
	// test_rx:		0x4000 | (chn << 8) | (len << 2) | type

	static u16 cmd;
	static u16 st = 0;
	u16 pkts = 0;
	u32 tick_tx = 0;

	//		cmd = 0x8095;
	//		cmd = 0x4094;	//rx	
	do
	{
#if(MODULE_WATCHDOG_ENABLE)
		wd_clear();
#endif
		if (uart_phyTestGet (&cmd) || cmd!=0)
		{
			tick_tx = clock_time_bqb ();
			u8 chn = (cmd >> 8) & 0x3f;/////frequency
			u8 len =  (cmd >> 2) & 0x3f;/////power level
//			u8 rf_power_level = len;	//unused
			u8 type = cmd & 3;
			u8 ct = cmd >> 14;

			cmd = 0;
			if (type == 0)
			{
				phyTest_PRBS9 (pkt_phytest + 6, len);
			}
			else if (type == 1)
			{
				memset (pkt_phytest + 6, 0x0f, len);
			}
			else
			{
				memset (pkt_phytest + 6, 0x55, len);
			}

			pkt_phytest[4] = type;
			//while(1);


			if (ct == CMD_RESET)		//reset
			{
				rf_set_tx_rx_off_bqb ();
				pkts = 0;
				uart_phyTestSend (0);
			}
			else if (ct == CMD_RX)	//rx
			{
				dbg_pkts = 0;
				dbg_rx++;
				pkts = 0;
				chn = Rf_GetBleFreChannel(phyTest_Channel(chn));
				Rf_TrxStateSet(RF_MODE_RX,chn);
				//rf_set_ble_channel ( phyTest_Channel(chn) );
				//rf_set_rxmode ();
				uart_phyTestSend (0);


#if(DBG_RX_RESULT)
				rx_flg =1;
#endif

			}
			else if (ct == CMD_TX)	//tx
			{
				dbg_pkts = 0;
				dbg_tx++;
				pkts = 0;
				chn = Rf_GetBleFreChannel(phyTest_Channel(chn));
				Rf_TrxStateSet(RF_MODE_TX,chn);
				//rf_set_ble_channel ( phyTest_Channel(chn) );
				uart_phyTestSend (0);
			}
			else  if(ct == CMD_END)				//end
			{
				dbg_end++;
				rf_set_tx_rx_off_bqb ();
				uart_phyTestSend (BIT(15) | (pkts & 0x0fff));
#if(DBG_RX_RESULT)

				if(rx_flg){
					write_reg8(0x808004,0x88);
					write_reg32(0x808000,pkts);
				}
#endif

			}
			else {
				write_reg8(0x808004,0x99);//ERR
			}
			st = ct;
		}
		if (st == CMD_TX)
		{
			if (reg_rf_irq_status & FLD_RF_IRQ_TX || pkts == 0)
			{
				Rf_StartStx(pkt_phytest, tick_tx + 100 * CLOCK_SYS_CLOCK_1US);
				//rf_start_stx (pkt_phytest, tick_tx + 100 * CLOCK_SYS_CLOCK_1US);
				tick_tx += 625 * CLOCK_SYS_CLOCK_1US;
				reg_rf_irq_status = FLD_RF_IRQ_TX;
				pkts++;
				dbg_pkts = pkts;
			}
		}
		else if (st == CMD_RX)
		{
			if (reg_rf_irq_status & FLD_RF_IRQ_RX)
			{
				reg_rf_irq_status = FLD_RF_IRQ_RX;
				if	( buffer_phytest[0] >= 15 && RF_PACKET_LENGTH_OK(buffer_phytest) && RF_PACKET_CRC_OK(buffer_phytest) )
				{
					pkts++;
				}
				dbg_pkts = pkts;
				//clear status
			}
		}
	} while (st == 1 || st == 2);

}

unsigned int bqb_loop_cnt;
void main_loop_bqb(void)
{
	bqb_loop_cnt++;

	phytest ();
}

unsigned char uart_phyTest_init(unsigned short uartCLKdiv, unsigned char bwpc){

	if(bwpc<3)
		return 0;
	write_reg16(0x800094,(uartCLKdiv|0x8000));//set uart clk divider and enable clock divider
	write_reg8(0x800096,bwpc);			//register mode
	write_reg8(0x80009a,(bwpc+1)*10);
	write_reg8(0x80009b,1);//For save

	write_reg8(0x800097,0x00);//No clts and rts
	return 1;

}

void  user_init_bqb(void)
{
	//usb_log_init ();
	/////////// ID initialization for host control software //////////
	//REG_ADDR8(0x74) = 0x53;
	//REG_ADDR16(0x7e) = 0x82bd;
	//REG_ADDR8(0x74) = 0x00;

	/////////// enable USB device /////////////////////////////////////
	//usb_dp_pullup_en (1);

	////////// set up wakeup source: driver pin of keyboard  //////////
	///////Rf_PowerLevelSet (RF_POWER_7dBm);////removed by C.Q.W
	Rf_PowerLevelSet (RF_POWER_5dBm);
	//uart_Init(9,13,0,0,NOCONTROL);
//	uart_phyTest_init (9, 13);/////115200
//	uart_phyTest_init (103, 15);/////9600
	uart_phyTest_init (118, 13);/////9600

	phyTest_PRBS9 (pkt_phytest + 6, 37);
#if UART_C2C3
	//Initial IO, take PC2/PC3 as UART_TX/UART_RX
	write_reg8(0x800596,0xC3);
	write_reg8(0x8005B2,0x3C);
#elif UART_A6A7
	//Init IO,take PA6/PA7 ad UART_TX/UART_RX
	write_reg8(0x8005B1,read_reg8(0x8005B1)&0xf3);///disable B2 and B3 as UART
	write_reg8(0x8005B2,read_reg8(0x8005B2)&0xf3);///disable C2 and C3 as UART
	write_reg8(0x800586,read_reg8(0x800586)&0x3f);///disable A6 and A7 as GPIO
	write_reg8(0x8005B0,read_reg8(0x8005B0)&0xbf);///enable A6 as UART_TX
	write_reg8(0x8005B0,read_reg8(0x8005B0)|0x80);///enable A7 as UART_RX
#else
	//Init IO,take PB2/PB3 ad UART_TX/UART_RX
	write_reg8(0x8005B0,read_reg8(0x8005B0)|0x40);///disable A6 uart function
	write_reg8(0x8005B0,read_reg8(0x8005B0)&0x7F);///disable A7 uart function
	write_reg8(0x8005B2,read_reg8(0x8005B2)&0xf3);///disable C2 and C3 as UART
	write_reg8(0x8005B1,read_reg8(0x8005B1)|0x0C);///enable  B2 and B3 as UART
	write_reg8(0x80058e,read_reg8(0x80058e)&0xf3);///disable B2 and B3 as GPIO
#endif
	//gpio_set_func (GPIO_PC6, !AS_GPIO);
	//gpio_set_func (GPIO_PC7, !AS_GPIO);
	Rf_RxBufferSet(buffer_phytest,256,0);
//	reg_dma_rf_rx_addr = (u16)(u32) (buffer_phytest);
	//reg_dma2_ctrl = FLD_DMA_WR_MEM | (256>>4);   // rf rx buffer enable & size
	//reg_dma_chn_irq_msk = 0;

#if 1
	REG_ADDR8(0x401) = 0;				//disable PN
	write_reg32 (0x800408, 0x29417671);	//accesscode: 1001-0100 1000-0010 0110-1110 1000-1110   29 41 76 71
#endif

	//REG_ADDR8(0x404) &= ~(BIT(5));  //whitening off
	//REG_ADDR8(0x0420) = 0x18;
}

void USB_DpPullUpEn(int En)
{
    unsigned char dat = ReadAnalogReg(0x08);
    if (En) {
        dat = (dat & 0x3f) | BIT(7);
    }
    else {
        dat = (dat & 0x3f) | BIT(6);
    }
    WriteAnalogReg (0x08, dat);
}

void Bqb_Init(void)
{
	write_reg8(0x800643,0);	
	
//	unsigned long TX_bug_cc;
	unsigned char f_tp[2];
	unsigned char f_cap;
	struct  S_SYS_CTL  sys_ctl;
	sys_ctl.rst0.all = 0x00;
	sys_ctl.rst1.all = 0x00;
	sys_ctl.rst2.all = 0x00;
	sys_ctl.clk0.all = 0xff;
	sys_ctl.clk1.all = 0xff;
	sys_ctl.clk2.all = 0xff;
	Sys_Init(&sys_ctl,SYS_CLK_HS_DIV,12);
	USB_DpPullUpEn(1); //enable usb pullup resistor
//	usb_dp_pullup_en (1);
	//PM_WakeupInit();
	Rf_Init(RF_OSC_12M,RF_MODE_BLE_1M);
	///////flash:76010:Ƶƫ;76011����ƵTP;76012:��ƵTP

	flash_read_page(0x76010,1,&f_cap);  /////0x77000   FLASH_ReadPage(0x77000,1,&f_cap);
	if(f_cap!=0xff){
		WriteAnalogReg(0x81,f_cap);
	}
	flash_read_page(0x76011,sizeof(f_tp),f_tp);///read TP value  0x77040   FLASH_ReadPage(0x77040,sizeof(f_tp),f_tp);
	/////telink ble only has BLE_1M mode

	if((f_tp[0]!=TP_GAIN0[RF_MODE_BLE_1M]) && (f_tp[0]!=0xff)){
		TP_GAIN0[RF_MODE_BLE_1M] = f_tp[0];
	}
	if((f_tp[1]!=TP_GAIN1[RF_MODE_BLE_1M]) && (f_tp[1]!=0xff)){
		TP_GAIN1[RF_MODE_BLE_1M] = f_tp[1];
	}

	////
#if 0
	unsigned char chn = Rf_GetBleFreChannel(phyTest_Channel(24));
	Rf_TrxStateSet(RF_MODE_RX,chn);
#endif
	/////
	user_init_bqb ();
}
#endif

