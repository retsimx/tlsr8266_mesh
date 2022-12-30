/********************************************************************************************************
 * @file     rf_drv_8366.h 
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
#ifndef _RF_DRV_H_
#define _RF_DRV_H_

#ifndef XTAL_12M
#define XTAL_12M				0
#endif

#define			RF_TRX_MODE					0x80
#define			RF_TRX_OFF					0x44		//f02
#define         RF_TRX_OFF_MANUAL           0x55		//f02

#define RF_CHN_AUTO_CAP 	0xff00
#define RF_CHN_TABLE 		0x8000
#define RF_SET_TX_MANAUL	0x4000

#define FRE_OFFSET 	0
#define FRE_STEP 	5
#define MAX_RF_CHANNEL  16

#define RF_CHANNEL_MAX			16
#define RF_CHANNEL_MASK			(RF_CHANNEL_MAX - 1)

extern const unsigned char	rf_chn[RF_CHANNEL_MAX];
extern unsigned char	cap_tp[RF_CHANNEL_MAX];

enum{
	RF_TX_MODE_NORMAL = 0,
	RF_TX_MODE_CARRIER,
	RF_TX_MODE_CONTINUE,

	RF_POWER_LEVEL_MAX = 0,
	RF_POWER_LEVEL_M2 = 1,
	RF_POWER_LEVEL_M3 = 2,
	RF_POWER_LEVEL_MIN = 100,
};

enum {
	RF_POWER_8dBm	= 0,
	RF_POWER_4dBm	= 1,
	RF_POWER_3dBm	= 2,
	RF_POWER_2dBm	= 3,
	RF_POWER_0dBm	= 4,
	RF_POWER_m4dBm	= 5,
	RF_POWER_m8dBm	= 6,
	RF_POWER_m12dBm	= 7,
	RF_POWER_m16dBm	= 8,
	RF_POWER_m20dBm	= 9,
	RF_POWER_m24dBm	= 10,
	RF_POWER_m28dBm	= 11,
	RF_POWER_m32dBm	= 12,
	RF_POWER_OFF	= 13,
};

#define	SET_RF_TX_DMA_ADR(a)	write_reg16 (0x80050c, a)

void SetRxMode (signed char chn, unsigned short set);
void SetTxMode (signed char chn, unsigned short set);
void TxPkt (void* addr);

#define	rf_get_pipe(p)		p[7]

static inline void rf_set_tx_pipe (u8 pipe)
{
	write_reg8 (0x800f15, 0xf0 | pipe);
}

static inline void rf_set_ble_crc (u8 *p)
{
	write_reg32 (0x80044c, p[0] | (p[1]<<8) | (p[2]<<16));
}

static inline void rf_set_ble_crc_adv ()
{
	write_reg32 (0x80044c, 0x555555);
}

static inline void rf_set_ble_access_code (u8 *p)
{
	write_reg32 (0x800408, p[3] | (p[2]<<8) | (p[1]<<16) | (p[0]<<24));
}

static inline void rf_set_ble_access_code_adv ()
{
	write_reg32 (0x800408, 0xd6be898e);
}

static inline void rf_set_access_code0 (u32 code)
{
	write_reg32 (0x800408, (read_reg32(0x800408) & 0xff) | (code & 0xffffff00));
	write_reg8  (0x80040c, code);
}

static inline u32 rf_get_access_code0 (void)
{
	return read_reg8 (0x80040c) | (read_reg32(0x800408) & 0xffffff00);
}

static inline void rf_set_access_code1 (u32 code)
{
	write_reg32 (0x800410, (read_reg32(0x800410) & 0xff) | (code & 0xffffff00));
	write_reg8  (0x800414, code);
}

static inline u32 rf_get_access_code1 (void)
{
	return read_reg8 (0x800414) | (read_reg32(0x800410) & 0xffffff00);
}

static inline u32 rf_access_code_16to32 (u16 code)
{
	u32 r = 0;
	for (int i=0; i<16; i++) {
		r = r << 2;
		r |= code & BIT(i) ? 1 : 2;
	}
	return r;
}

static inline void rf_stop_trx (void)
{
	write_reg8  (0x800f00, 0x80);			// stop
}

static inline void rf_stop_blt_state_machine2normal (void)
{
    rf_stop_trx();
}

static inline void rf_reset_sn (void)
{
	write_reg8  (0x800f01, 0x3f);
	write_reg8  (0x800f01, 0x00);
}

extern u8 emi_var[];
static inline void emi_init( u8 tx_power_emi ){
    emi_var[0] = analog_read(0xa5);    
    emi_var[1] = read_reg8(0x8004e8);
    //emi_var[2] = read_reg8(0x800524);
    //emi_var3 = read_reg8(0x800402);
    emi_var[4] = read_reg8(0x80050f);
    emi_var[5] = read_reg8(0x80050e);
    emi_var[6]  = read_reg8(0x800400);
    
	rf_power_enable (1);
    rf_set_power_level_index (tx_power_emi);
}

static inline void emi_carrier_init( void ){
    write_reg8 (0x800f02, 0x45);  // reset tx/rx enable reset
}

static inline void emi_cd_prepare( void ){
    write_reg8 (0x800f02, RF_TRX_OFF);  //must trx disable before cd switch
}

static inline void emi_cd_init( u32 cd_fifo){
	//reset zb & dma
#if 0	//the same with 8266
	write_reg16(0x800060, 0x0480);
	write_reg16(0x800060, 0x0000);

#else
	write_reg8(0x800060, 0x0a);
	write_reg8(0x800060, 0x00);
#endif
	//TX mode
	//write_reg8 (0x800400,0x0b);//0b for 2Mbps, 03 for Zigbee, 0f for New2M and BLE Nrd 1Mbps
	//write_reg8 (0x800408,0x00);//0 for random , 1 for 0xf0f0, 2 fro 0x5555
    
	//txsetting    
	//write_reg8(0x800402, 0x21);	//preamble length=1
	
	//txdma( &fifo_emi.start, FIFO_DEPTH );
    write_reg8(0x80050c,  cd_fifo & 0xff );
	write_reg8(0x80050d, (cd_fifo >> 8) & 0xff );
	write_reg8(0x80050e, (*(unsigned int*)cd_fifo -1) >> 4 );   //reg_50e = TX_buffer_size /16
	write_reg8(0x80050f, *(unsigned int*)cd_fifo -1 );

	//txpktsend    
	write_reg8(0x800524, 0x08);
}

//as zhongqi's suggestion
static inline void emi_carrier_generate( void ){
    //write_reg8 (0x800400, 0x6f);//[6:5] 11: send 1, 10: send 0
    //tx_cyc1 manual
    analog_write (0xa5, 0x44);
    write_reg8 (0x8004e8, 0x04);
}

static inline void emi_carrier_recovery( void ){
    analog_write (0xa5, emi_var[0]);
    write_reg8 (0x8004e8, emi_var[1]);
}

static inline void emi_cd_recovery( void ){
    write_reg8(0x800524, 0);            
    //write_reg8(0x800402, emi_var3); 
    
    write_reg8(0x80050f, emi_var[4]);
    write_reg8(0x80050e, emi_var[5]);
    write_reg8(0x800400, emi_var[6]);
}

int		rf_drv_init (int wakeup_from_suspend);
void	rf_power_enable (int);
void	rf_set_channel (signed char chn, unsigned short set);
void	rf_set_rxmode (void);
void	rf_set_txmode (void);
void	rf_send_packet (void* addr, u16 rx_waittime, u8 retry);
void 	rf_multi_receiving_init (u8 channel_mask);
void	rf_multi_receiving_start  (signed char chn, unsigned short set);
void	rf_set_ack_packet  (void* addr);
void 	rf_set_power_level_index (int level);

//#define		RF_FAST_MODE_1M		1
//#define		RF_MODE_250K		1

#ifdef		RF_MODE_250K
#define		RF_FAST_MODE_2M		0
#define		RF_FAST_MODE_1M		0
#endif

#ifndef		RF_FAST_MODE_1M
#define		RF_FAST_MODE_1M		0
#endif

#ifndef		RF_FAST_MODE_2M
#define		RF_FAST_MODE_2M		(!RF_FAST_MODE_1M)
#endif


#if			RF_FAST_MODE_2M
#define		RF_PACKET_LENGTH_OK(p)		(p[0] == (p[12]&0x3f)+15)
#define		RF_PACKET_CRC_OK(p)			((p[p[0]+3] & 0x51) == 0x40)
#elif		RF_FAST_MODE_1M
#define		RF_PACKET_LENGTH_OK(p)		(p[0] == (p[13]&0x3f)+17)
#define		RF_PACKET_CRC_OK(p)			((p[p[0]+3] & 0x51) == 0x40)
#else
#define		RF_PACKET_LENGTH_OK(p)		(p[0] == p[12]+13)
#define		RF_PACKET_CRC_OK(p)			((p[p[0]+3] & 0x51) == 0x10)
#endif

#endif
