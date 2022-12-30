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
#include "../../proj_lib/ble_ll/blueLight.h"
#include "../../proj_lib/light_ll/light_ll.h"
extern u8 sys_chn_listen[];// {2, 12, 23, 34};	//8, 30, 52, 74
u8 mesh_cmd[48] = {0};
u32 sw_cmd_sno = 0;
u16 sw_device_address = 0;
u32 mesh_tx_buffer[12] = {0};
u32 mesh_rx_buffer[12] = {0};
u32 mesh_pn_table[4][11] = {};
const u8 tbl_pn[44 * 4] = {		//2, 12, 23, 34
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

////////////////////////////////////////////////////////
#define     CYSTAL_12M      1

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
#if CYSTAL_12M
	0x99, 	0xb1,  // gauss filter sel: 16M
	0x82,	0x00,  //enable rxadc clock
	0x9e, 	0x56,  //reg_dc_mod (500K)
#else
	0x99, 	0x31,	// gauss filter sel: 16M
	0x82,	0x14,	// gauss filter sel: 16M
	0x9e,	0x41,	// gauss filter sel: 16M
#endif
	////////////////////////////8886 deepsleep analog register recover////////////////////////////
	0x06, 0x00,  //power down control
};


const u16 rf_ini_core[] = {
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

void irq_handler(void){
}

void rf_init_reg(){
	for (int i=0; i< (sizeof (rf_ini_core)/2); i+= 2)
	{
		REG_ADDR8(rf_ini_core[i]) = rf_ini_core[i+1];
	}
	for (int i=0; i< sizeof (rf_init_ana); i+= 2)
	{
		analog_write (rf_init_ana[i], rf_init_ana[i+1]);
	}
}

void mesh_stall_mcu(u32 tick_stall){
   REG_ADDR8(0x79) = BIT(5);	//RF irq enable
   REG_ADDR8(0x6f) = 0x80;		//stall mcu
   asm("tnop");
   asm("tnop");
   asm("tnop");
   asm("tnop");
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////


void mesh_set_channel(signed char chn){
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

	//auto tx
	write_reg16 (0x8004d6, fre);	// {intg_N}
	write_reg32 (0x8004d0, (fre - 2) * 58254 + 1125);	// {intg_N, frac}

	rf_set_tp_gain (chn);
}

void mesh_pn_tbl_calc( int pn_init, u32 *pt){
	pn_init |= BIT(6);
	int poly[2]={0, 0x44};              //0x8005 <==> 0xa001

	for (int i=0; i<11; i++) {
		*pt = 0;
		for (int j=0; j<32; j++) {
			*pt |= (pn_init & 1) << j;
			pn_init = (pn_init >> 1) ^ poly[pn_init & 1];
		}
		pt++;
	}

}

void mesh_packet_tx_pn(u32 *pd, u32 *ps, u32 *px){
    int n = ((u8 *)ps)[1] + 5;
	*pd++ = n;
	for (int i=0; i<n; i+=4){
		*pd ++ = *ps++ ^ *px++;
	}
}

int mesh_packet_crc24(unsigned char *p, int n, int crc){
    for (int j=0; j<n; j++) {
    	u8 ds = p[j];
    	for (int i=0; i<8; i++) {
            int cond = (crc ^ ds ) & 1;
        	crc >>= 1;
        	if (cond) {
        	    crc ^= 0xda6000;
        	}
            ds = ds >> 1;
    	}
    }
    return crc;
}

int mesh_packet_tx_crc(u8 *pd, u8 *ps){
	u32 crc_init = 0xaaaaaa;
	int n = ps[1] + 2;
	if (pd != ps) {
		memcpy(pd, ps, n);
	}
	int crc = mesh_packet_crc24(ps, n, crc_init);
	u8 *pcrc = pd + n;
	*pcrc++ = crc;
	*pcrc++ = crc >> 8;
	*pcrc++ = crc >> 16;
	return n;
}

void mesh_pn_init(){
	for (int i=0; i<4; i++){
		if (sys_chn_listen[i] == 2){
			memcpy (mesh_pn_table[i], tbl_pn + 0, 44);
		}
		else if (sys_chn_listen[i] == 12){
			memcpy (mesh_pn_table[i], tbl_pn + 44, 44);
		}
		else if (sys_chn_listen[i] == 23){
			memcpy (mesh_pn_table[i], tbl_pn + 88, 44);
		}
		else if (sys_chn_listen[i] == 34){
			memcpy (mesh_pn_table[i], tbl_pn + 132, 44);
		}else{
			mesh_pn_tbl_calc (sys_chn_listen[i], mesh_pn_table[i]);
		}
	}
}

void mesh_init()
{
	reg_dma_rf_rx_addr = (u16)(u32) mesh_rx_buffer;
	reg_dma2_ctrl = FLD_DMA_WR_MEM | (64>>4);   // rf rx buffer enable & size
	reg_dma_chn_irq_msk = 0;

	//reg_irq_mask |= FLD_IRQ_ZB_RT_EN;    //enable RF & timer1 interrupt
	reg_rf_irq_mask = FLD_RF_IRQ_TX;

	REG_ADDR16(0x50c) = (u16)((u32)mesh_tx_buffer);
	write_reg32(0x800f18, clock_time() + 32);
	REG_ADDR8(0xf16) |= BIT(4);

	extern u8 sw_flag;
	sw_flag = 1;
}

void mesh_command_send(u32 * cmd){
	REG_ADDR8(0x428) = 0x80;						// RX disable
	REG_ADDR8(0xf00) = 0x80;						// stop SM
	REG_ADDR8(0x404) = 0xf5;						// disable shock-burst mode

    foreach(i, 16){
    	int k = i & 3;
    	mesh_set_channel (sys_chn_listen[k]);
    	reg_rf_irq_status = FLD_RF_IRQ_TX;
		REG_ADDR8(0xf00) = 0x85;							// single TX

		mesh_packet_tx_pn (mesh_tx_buffer, cmd, mesh_pn_table[k]);

		u32 t = clock_time ();
		mesh_stall_mcu (500*CLOCK_SYS_CLOCK_1US);
		///////////// wait for TX
		while (!(reg_rf_irq_status & FLD_RF_IRQ_TX) && (clock_time() - t) < 600*CLOCK_SYS_CLOCK_1US);
		sleep_us (10);
    }
}

typedef struct{
	u32 dma_len;
	u8	type;
	u8  rf_len;
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle;
	u8 handle1;
	u8 value[29];//sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}rf_packet_sw_cmd_t;

rf_packet_sw_cmd_t pkt_sw_cmd = {
		sizeof (rf_packet_att_cmd_t) - 4,	// dma_len
		0x02,					// type
		sizeof (rf_packet_att_cmd_t) - 6,	// rf_len
		0xccdd, 					// u16
};

int mesh_pkt_assemble_8368(u8 *p_cmd, u16 dst){
	pkt_sw_cmd.dma_len = 38;
	pkt_sw_cmd.type = 0x02;
	pkt_sw_cmd.rf_len = pkt_sw_cmd.dma_len - 2;
	pkt_sw_cmd.l2capLen = pkt_sw_cmd.dma_len - 6;
	pkt_sw_cmd.chanId = 0xff04;
	pkt_sw_cmd.opcode = 0x12;
	pkt_sw_cmd.handle = 0x00;
	pkt_sw_cmd.handle1 = 0xf5;
	memset(pkt_sw_cmd.value + 7, 0, 21);
	memcpy(pkt_sw_cmd.value, &sw_cmd_sno, 3);
	memcpy(pkt_sw_cmd.value + 3, &sw_device_address, 2);
	memcpy(pkt_sw_cmd.value + 5, &dst, 2);
	memcpy(pkt_sw_cmd.value + 7, p_cmd, 13);
	pkt_sw_cmd.value[25] = 4;//hops

	return 1;
}

int main(void){
	cpu_wakeup_init();
	gpio_init();
	clock_init();
	rf_init_reg ();
	u32 sw_ac = 0xa3923752;//telink_8368
	rf_set_ble_access_code ((u8 *)&sw_ac);
	rf_set_power_level_index (RF_POWER_8dBm);
	mesh_pn_init();
	mesh_init ();
	sw_device_address = (u16)(*(u32*)(CFG_ADR_MAC));
	
	sw_cmd_sno = (clock_time() | sw_device_address) + analog_read(0x19);
	u8 op_para[13] = {0};
	op_para[0] = LGT_CMD_LIGHT_ONOFF | 0xc0;
	op_para[1] = VENDOR_ID & 0xFF;
	op_para[2] = VENDOR_ID >> 8;
	sleep_us(1000*1000);
	while(1){
		u8 onoff = analog_read(0x1a);
		op_para[3] = (onoff++)%2;//params[0], on or off
		sw_cmd_sno++;
		mesh_pkt_assemble_8368(op_para, 0xFFFF);
		memcpy(mesh_cmd, &pkt_sw_cmd.type, 48);
		mesh_packet_tx_crc(mesh_cmd, mesh_cmd);
		mesh_command_send((u32 *)mesh_cmd);

		//deep sleep
		//irq_disable();
		analog_write(0x19, sw_cmd_sno);
		analog_write(0x1a, onoff);
		cpu_sleep_wakeup(1, PM_WAKEUP_TIMER, 1000);
	}
	
	return 0;
}

