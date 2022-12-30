/********************************************************************************************************
 * @file     main.c 
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

#include "../../proj/tl_common.h"


#include "../../proj/mcu/watchdog_i.h"
#include "../../vendor/common/user_config.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "myi2c.h"

extern void user_init();
extern void main_loop ();
void irq_handler(void)
{
}

////////////////////////////////////////////////////////
#define				CYSTAL_12M			1

const u8 rf_init_ana[] = {
	//crystal cap setting
	0x80, 0x61,  //crystal cap 40-5f

	0x81, 0xd5,  //crystal cap 40-5f  //0x4f
	0x83, 0x10,  //enable baseband PLL
	0x84, 0x20,  //enable 192M clock to dig
	0x8d, 0x62,  //tx/rx vco icur [2:0]

	0x93, 0x50,  //dac gain setting
	0xa0, 0x26,  //dac datapath delay
	0xa2, 0x2c,  //pa_ramp_target
	0xac, 0xaa,  //filter center frequency

	0x8e, 0x6a,	 //tx vco prescaler bias current controller  20uA

	0xa3, 0xf0,  //pa_ramp_en = 1, pa ramp
	0xaa, 0x26,  //filter iq_swap, 1M bandwidth
	0x8f, 0x38,

	//		cyrstal 12M
#if 1
	0x99, 	0xb1,  // gauss filter sel: 16M
	0x82,	0x00,  //enable rxadc clock
	0x9e, 	0x56,  //reg_dc_mod (500K)
#else
	0x99, 	0x31,  TCMD_UNDER_BOTH | TCMD_WAREG,	// gauss filter sel: 16M
	0x82,	0x14,  TCMD_UNDER_BOTH | TCMD_WAREG,	// gauss filter sel: 16M
#endif
	////////////////////////////8886 deepsleep analog register recover////////////////////////////
	0x06, 0x00,  //power down control
};


const u16 rf_ini_core[] =
{
	///////////// baseband //////////////////////
	0x074f, 0x01,	//enable system timer

	0x0f03, 0x1e,	// bit3: crc2_en; normal 1e
	0x0f04, 0x50,	//tx settle time: 80us
	0x0f06, 0x00,	//rx wait settle time: 1us
	0x0f0c, 0x50,	//rx settle time: 80us
	0x0f10, 0x00,	//wait time on NAK
//	0x0f16, 0x23,	//192M bbpll reset enable


	////////////////////////////////////////////////////////////
	0x0400, 0x0d,	// 1M mode
	0x0401, 0x00,	// pn disable
	0x0402, 0x24,	// 8-byte pre-amble
	0x0404, 0xf5,	// head_mode/crc_mode: normal c0; 0xf7 for RX shockburst
	0x0405, 0x04,	// access code length 4

	0x0420, 0x1f,	// threshold
	0x0421, 0x04,	// no avg
	0x0422, 0x00,	// threshold
	0x0424, 0x12,	// number fo sync: bit[6:4]
	0x042b, 0xf1,	// access code: 1
	0x042c, 0x30,	// maxiumum length 48-byte

	0x0430, 0x12,	// for 8us preamble
	0x0439, 0x72,	//
	0x043d, 0x71,	// for 8us preamble
	0x043b, 0xfc,	//enable timer stamp & dc output

	0x0464, 0x07,	// new sync: bit0
	0x04cd, 0x04,	// enable packet length = 0

	//dcoc
	0x04ca, 0x88,	//head_chn, report rx sync channel in packet, enable DC distance bit 3
	0x04cb, 0x04,
	0x042d, 0x33,	//DC alpha=1/8, bit[6:4] ???? 33

	// MAX GAIN
	0x0433, 0x00,	// set mgain disable 01 -> 00
	0x0434, 0x01,	// pel0: 21 -> 01
	0x043a, 0x77,	// Rx signal power change threshold: 22 -> 77
	0x043e, 0xc9,	// set rx peak detect manual: 20 -> c9
	0x04cd, 0x06,	// fix rst_pga=0: 8266(06)

	0x04c0, 0x81,	// lowpow agc/sync: 87 -> 71

#if CYSTAL_12M
#else
	0x04eb, 0x60,
#endif
};

void	rf_init_reg ()
{
	for (int i=0; i< (sizeof (rf_ini_core)/2); i+= 2)
	{
		REG_ADDR8(rf_ini_core[i]) = rf_ini_core[i+1];
	}
	for (int i=0; i< sizeof (rf_init_ana); i+= 2)
	{
		analog_write (rf_init_ana[i], rf_init_ana[i+1]);
	}
}

void mesh_stall_mcu(u32 tick_stall)
{
  /*Write 0x00 here may cause problem, it is removed to blt_sleep_wakeup*/
  //write_reg8(0x6f,0x00);//clear bit[0] suspend enable

#if 0
   reg_tmr1_tick = 0;

   reg_tmr1_capt = tick_stall;
   reg_tmr_ctrl8 |= FLD_TMR1_EN;//enable TIMER1,mode:using sclk
   REG_ADDR8(0x78) |= FLD_IRQ_TMR1_EN;//timer1 mask enable
   reg_tmr_sta = FLD_TMR_STA_TMR1; // clean interrupt

   REG_ADDR8(0x6f) = x80;//stall mcu
   asm("tnop");
   asm("tnop");
   asm("tnop");
   asm("tnop");

   reg_tmr_sta = FLD_TMR_STA_TMR1; // clean interrupt
   reg_tmr_ctrl8 &= ~FLD_TMR1_EN;//disable TIMER1
#else
   REG_ADDR8(0x79) = BIT(5);	//RF irq enable
   REG_ADDR8(0x6f) = 0x80;		//stall mcu
   asm("tnop");
   asm("tnop");
   asm("tnop");
   asm("tnop");
#endif

}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
u8 				mesh_chn[8] = {2, 12, 23, 34};			//8, 30, 52, 74
u32				mesh_tx_buffer[12];
u32				mesh_rx_buffer[12];
u32				mesh_pn_table[4][11] = {};
u32				adv_pn_table[11] = {};
const u8		tbl_pn[44 * 4] = {		//2, 12, 23, 34
		0xd2,0x57,0xa1,0x3d,0xa7,0x66,0xb0,0x75,0x31,0x11,0x48,0x96,0x77,0xf8,0xe3,0x46,
		0xe9,0xab,0xd0,0x9e,0x53,0x33,0xd8,0xba,0x98,0x08,0x24,0xcb,0x3b,0xfc,0x71,0xa3,
		0xf4,0x55,0x68,0xcf,0xa9,0x19,0x6c,0x5d,0x4c,0x04,0x92,0xe5,0x2c,0xef,0xf0,0xc7,
		0x8d,0xd2,0x57,0xa1,0x3d,0xa7,0x66,0xb0,0x75,0x31,0x11,0x48,0x96,0x77,0xf8,0xe3,
		0x46,0xe9,0xab,0xd0,0x9e,0x53,0x33,0xd8,0xba,0x98,0x08,0x24,0xcb,0x3b,0xfc,0x71,
		0xa3,0xf4,0x55,0x68,0xcf,0xa9,0x19,0x6c,0xaf,0x42,0x7b,0x4e,0xcd,0x60,0xeb,0x62,
		0x22,0x90,0x2c,0xef,0xf0,0xc7,0x8d,0xd2,0x57,0xa1,0x3d,0xa7,0x66,0xb0,0x75,0x31,
		0x11,0x48,0x96,0x77,0xf8,0xe3,0x46,0xe9,0xab,0xd0,0x9e,0x53,0x33,0xd8,0xba,0x98,
		0x08,0x24,0xcb,0x3b,0xf2,0x0e,0x7f,0xdc,0x28,0x7d,0x15,0xda,0x73,0x6a,0x06,0x5b,
		0x17,0x13,0x81,0x64,0x79,0x87,0x3f,0x6e,0x94,0xbe,0x0a,0xed,0x39,0x35,0x83,0xad,
		0x8b,0x89,0x40,0xb2,0xbc,0xc3,0x1f,0x37,0x4a,0x5f,0x85,0xf6,0x9c,0x9a,0xc1,0xd6,
};

u8	tbl_adv [] =
		{
		 //20, 0, 0, 0, 								// DMA length
		 0x00, 15,
		 0xef, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5,		//mac address
		 0x02, 0x01, 0x06, 							// BLE limited discoverable mode and BR/EDR not supported
		 0x05, 0x02, 0x12, 0x18, 0x0F, 0x18,		// incomplete list of service class UUIDs (0x1812, 0x180F)
		 0, 0, 0			//reserve 3 bytes for CRC
		};

void mesh_set_channel (signed char chn)
{
	if (chn < 11)
    	chn += 2;
    else if (chn < 37)
    	chn += 3;
    else if (chn == 37)
    	chn = 1;
    else if (chn == 38)
    	chn = 13;
    else
    	chn = 40;

    chn = chn << 1;

	/////////////////// turn on LDO and baseband PLL ////////////////
	write_reg8 (0x800f16, 0x29);

	write_reg8 (0x800428, 0x80);	// rx disable
	write_reg8 (0x800f02, 0x44);	// reset tx/rx state machine

	u32 fre = 2400 + chn;
	//u32 fre_fraction = 0;

	//auto tx
	write_reg16 (0x8004d6, fre);	// {intg_N}
	write_reg32 (0x8004d0, (fre - 2) * 58254 + 1125);	// {intg_N, frac}

	//auto tx
//	write_reg16 (0x8004d6, fre);	// {intg_N}
//	write_reg16 (0x8004d4, ((fre_fraction / 1000) << 16));

	//manual rx
//	write_reg16 (0x8004d2, fre);	// {intg_N}
//	write_reg32 (0x8004d0, );	// {intg_N, frac}

	rf_set_tp_gain (chn);
}

void mesh_pn_tbl_calc( int pn_init, u32 *pt){
	pn_init |= BIT(6);
	int		poly[2]={0, 0x44};              //0x8005 <==> 0xa001

	for (int i=0; i<11; i++) {
		*pt = 0;
		for (int j=0; j<32; j++) {
			*pt |= (pn_init & 1) << j;
			pn_init = (pn_init >> 1) ^ poly[pn_init & 1];
		}
		pt++;
	}

}

void	mesh_packet_tx_pn (u32 *pd, u32 *ps, u32 *px)
{
    int n = ((u8 *)ps)[1] + 5;
	*pd++ = n;
	for (int i=0; i<n; i+=4){
		*pd ++ = *ps++ ^ *px++;
	}
}

int	mesh_packet_crc24 (unsigned char *p, int n, int crc)
{
	//crc16¡êo G(X) = X16 + X15 + X2 + 1

	//crc16:  x24 + x10 + x9 + x6 + x4 + x3 + x + 1 ==> 0x00065b
	//               13   14   17   19   20  22  23 ==> 0xda6000
   // static unsigned short poly[2]={0, 0xa001};              //0x8005 <==> 0xa001

	//int		poly[2]={0, 0xda6000};              //0x8005 <==> 0xa001
    for (int j=0; j<n; j++) {
    	u8 ds = p[j];
    	for (int i=0; i<8; i++) {

        	int cond = (crc ^ ds ) & 1;
        	crc >>= 1;
        	if (cond) {
        		 crc ^= 0xda6000;
        	}
            //crc = (crc >> 1) ^ poly[(crc ^ ds ) & 1];
            ds = ds >> 1;
    	}
    }
     return crc;
}

int	mesh_packet_tx_crc (u8 *pd, u8 *ps)
{
	u32 crc_init = 0xaaaaaa;
	int n = ps[1] + 2;
	if (pd != ps) {
		memcpy (pd, ps, n);
	}
	int crc = mesh_packet_crc24 (ps, n, crc_init);
	u8 *pcrc = pd + n;
	*pcrc++ = crc;
	*pcrc++ = crc >> 8;
	*pcrc++ = crc >> 16;
	return n;
}

void mesh_pn_init ()
{
	for (int i=0; i<4; i++)
	{
		if (mesh_chn[i] == 2)
		{
			memcpy (mesh_pn_table[i], tbl_pn + 0, 44);
		}
		else if (mesh_chn[i] == 12)
		{
			memcpy (mesh_pn_table[i], tbl_pn + 44, 44);
		}
		else if (mesh_chn[i] == 23)
		{
			memcpy (mesh_pn_table[i], tbl_pn + 88, 44);
		}
		else if (mesh_chn[i] == 34)
		{
			memcpy (mesh_pn_table[i], tbl_pn + 132, 44);
		}
		else
		{
			mesh_pn_tbl_calc (mesh_chn[i], mesh_pn_table[i]);
		}
	}
}

void mesh_init ()
{
	reg_dma_rf_rx_addr = (u16)(u32) mesh_rx_buffer;
	reg_dma2_ctrl = FLD_DMA_WR_MEM | (64>>4);   // rf rx buffer enable & size
	reg_dma_chn_irq_msk = 0;

	//reg_irq_mask |= FLD_IRQ_ZB_RT_EN;    //enable RF & timer1 interrupt
	reg_rf_irq_mask = FLD_RF_IRQ_TX;

	REG_ADDR16(0x50c) = (u16)((u32)mesh_tx_buffer);
	write_reg32(0x800f18, clock_time() + 32);
	REG_ADDR8(0xf16) |= BIT(4);
}

// PN, CRC, access code
void	mesh_send_adv (u8 chn)
{
		static u32 tick;
		tick = clock_time ();
		REG_ADDR8(0x428) = 0x80;						// RX disable
		REG_ADDR8(0xf00) = 0x80;						// stop SM
		REG_ADDR8(0x404) = 0xf5;						// disable shock-burst mode

		REG_ADDR32(0x408) =  0xd6be898e; //0x8e89bed6;
		tick = clock_time () - tick;

		mesh_pn_tbl_calc (chn, adv_pn_table);
		mesh_packet_tx_crc (tbl_adv, tbl_adv);

        for (int i=0; i<8; i++)
        {
        	mesh_set_channel (chn);

        	reg_rf_irq_status = FLD_RF_IRQ_TX;

			REG_ADDR8(0xf00) = 0x85;							// single TX

			mesh_packet_tx_pn (mesh_tx_buffer, (u32*)&tbl_adv, adv_pn_table);

			u32 t = clock_time ();
			///////////// wait for TX
			while (!(reg_rf_irq_status & FLD_RF_IRQ_TX) && (clock_time() - t) < 600*CLOCK_SYS_CLOCK_1US);

			sleep_us (10);
        }
}

void	mesh_command_send (u32 * cmd)
{
		static u32 tick_cmd;
		tick_cmd = clock_time ();
		REG_ADDR8(0x428) = 0x80;						// RX disable
		REG_ADDR8(0xf00) = 0x80;						// stop SM
		REG_ADDR8(0x404) = 0xf5;						// disable shock-burst mode

		//mesh_pn_tbl_calc (chn, mesh_pn_table);
        //mesh_packet_tx_pn (mesh_tx_buffer, (u32*)&mesh_user_cmd, mesh_pn_table)
		//mesh_packet_tx_crc (tbl_adv, tbl_adv);

        for (int i=0; i<16; i++)
        {
        	tick_cmd = i + 1;
        	int k = i & 3;
        	mesh_set_channel (mesh_chn[k]);

        	reg_rf_irq_status = FLD_RF_IRQ_TX;

			REG_ADDR8(0xf00) = 0x85;							// single TX

			//mesh_packet_tx_pn (mesh_tx_buffer, (u32*)&tbl_adv, mesh_pn_table);
			mesh_packet_tx_pn (mesh_tx_buffer, cmd, mesh_pn_table[k]);

			u32 t = clock_time ();

			mesh_stall_mcu (500*CLOCK_SYS_CLOCK_1US);
			///////////// wait for TX
			while (!(reg_rf_irq_status & FLD_RF_IRQ_TX) && (clock_time() - t) < 600*CLOCK_SYS_CLOCK_1US);

			sleep_us (10);
        }
}
////////////////////////////////////////
//	LTK, group, 48-byte on, 48-byte off
////////////////////////////////////////
u8		i2c_buff[16] = {};

_attribute_ram_code_ _attribute_no_inline_ u8 read_mspi_io ()
{
	return REG_ADDR8 (0x5a0);			// bit3: MSDO, bit4: MSDI
}

u8 		mesh_cmd[48];
u32		tick_i2c, tick_drv, tick_init ;

int main (void)
{
	cpu_wakeup_init();
	//clock_init();   // use default 32M RC clock to run code faster.
	gpio_init();
#if 0
	rf_drv_init(CRYSTAL_TYPE);
	extern void rf_drv_1m();
	rf_drv_1m ();
#else
	rf_init_reg ();
#endif

	 //////////// load setting from EEPROM
	do
	{
		myi2c_sim_burst_read (0xa0, 0, mesh_chn, 8);

		// access code
		REG_ADDR32(0x408) =  mesh_chn[7] | (mesh_chn[6]<<8) | (mesh_chn[5]<<16) | (mesh_chn[4]<<24);

		mesh_pn_init();

		//////////// load mesh command, depends on the IO status
	    u8 button_status = read_mspi_io ();

	    if ((button_status & 0x18) != 0x18)
	    {
	    	myi2c_sim_burst_read (0xa0, 56, mesh_cmd, 48);		//light off
	    }
	    else
	    {
	    	myi2c_sim_burst_read (0xa0, 8, mesh_cmd, 48);		//light on
	    }

	    tick_i2c = clock_time ();

	} while (0);


	rf_set_power_level_index (RF_POWER_0dBm);

	tick_drv = clock_time () - tick_i2c;

	mesh_init ();

	tick_init = clock_time () - tick_i2c;

	clock_init();   // set clock to 16MHz to decrease current.

	mesh_command_send ((u32 *)mesh_cmd);

	//mesh_send_adv (37);

	while (1)
    {
    	/////////////// flag @80ff ///////////////////////////////////
		if (REG_ADDR8(0x80ff) == 0xa5)
		{
			REG_ADDR8(0x80ff) = 0xa0;
			myi2c_sim_burst_write (0xa0, 0, (u8*)(0x808000), 256);
		}
	}
}



