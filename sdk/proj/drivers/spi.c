/********************************************************************************************************
 * @file     spi.c 
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

#include "../tl_common.h"
#include "spi.h"

#if 0
//   spi using gpio simulation
static inline void spi_sim_clk_high(u32 pin_clk){
	gpio_write(pin_clk,1);
}
static inline void spi_sim_clk_low(u32 pin_clk){
	gpio_write(pin_clk,0);
}
static inline void spi_sim_dat_high(u32 pin_dat_out){
	gpio_write(pin_dat_out,1);
}
static inline void spi_sim_dat_low(u32 pin_dat_out){
	gpio_write(pin_dat_out,0);
}
static inline int spi_sim_has_input(u32 pin_dat_in){
	return gpio_read(pin_dat_in);
}
void spi_sim_out_en(u32 pin_dat_out){
	gpio_set_output_en(pin_dat_out,1);
	gpio_set_input_en(pin_dat_out,0);
}
static inline void spi_sim_in_en(u32 pin_dat_in){
	gpio_set_output_en(pin_dat_in,0);
	gpio_set_input_en(pin_dat_in,1);
}
static inline void spi_sim_clk_en(u32 pin_clk){
	gpio_set_output_en(pin_clk,1);
	gpio_set_input_en(pin_clk,0);
}

static inline void spi_sim_re_syn(u32 pin_clk){
	spi_sim_clk_high(pin_clk);
	sleep_us(100);
	spi_sim_clk_low(pin_clk);
	sleep_us(1);
	spi_sim_clk_high(pin_clk);
	sleep_us(2000);
}

void spi_sim_init(u32 pin_clk, u32 pin_dat_out){
	spi_sim_clk_high(pin_clk);
	spi_sim_dat_high(pin_dat_out);
	spi_sim_re_syn(pin_clk);
}


u8 spi_sim_read(u32 pin_clk, u32 pin_dat_in){
	u8 i = 0;
	u8 dat = 0;
	for(i=0; i<8; i++){
		spi_sim_clk_low(pin_clk);
		CLOCK_DLY_100NS;
		spi_sim_clk_high(pin_clk);
		CLOCK_DLY_100NS;
		if(spi_sim_has_input(pin_dat_in)){
			dat |= (1<<(7-i));
		}else{
			dat &= ~(1<<(7-i));
		}
	}
	return dat;
}

void spi_sim_write(u32 pin_clk, u32 pin_dat_out, u8 data){
	u8 i = 0;
	for(i=0; i<8; i++){
		spi_sim_clk_low(pin_clk);
		if(data&(1<<(7-i))){
			spi_sim_dat_high(pin_dat_out);
		}else{
			spi_sim_dat_low(pin_dat_out);
		}
		CLOCK_DLY_100NS;
		spi_sim_clk_high(pin_clk);
		CLOCK_DLY_100NS;
	}
}
#endif

//  secondary SPI using CN, CK, DI, DO  !!!!!
//  please define CN_FUNC, CK_FUNC, DI_FUNC, DO_FUNC as AS_SPI

static inline void spi_wait(void){
	while(reg_spi_ctrl & FLD_SPI_BUSY)
		;
}
static inline void spi_high(void){
	reg_spi_ctrl = FLD_SPI_MASTER_MODE_EN | FLD_SPI_CS;
}

static inline void spi_low(void){
	reg_spi_ctrl = FLD_SPI_MASTER_MODE_EN;	//  FLD_SPI_CS == 0
}

static inline void spi_out(u8 dat){
	reg_spi_data = dat;
	spi_wait();
}

static inline u8 spi_in(void){
	reg_spi_ctrl = FLD_SPI_MASTER_MODE_EN | FLD_SPI_RD;	//  FLD_SPI_CS == 0
	spi_wait();
	reg_spi_data;
	spi_wait();
	return reg_spi_data;
	
}

void spi_init(void){
	// pls define spi pin function ahead
#ifdef FLD_CK_DO_GPIO_MODE
	reg_spi_sp = MASK_VAL(FLD_CK_DO_GPIO_MODE, 1, FLD_MASTER_SPI_CLK, 10);	// 1M clck
	spi_high();
#else
	write_reg8(0x80000a, 0x8a); 		 //spi master mode
	write_reg8(0x800009, 0x03); 		 //cs=1
#endif
}

void spi_write(u8 addr, u8 dat){

#if(1)
	spi_high();
	spi_low();
	spi_out(addr);
	spi_out(dat);
	spi_high();	
#else
	write_reg8(0x800009, 0x03);          //cs=1
	write_reg8(0x800009, 0x02);          //cs=0
	
	write_reg8(0x800008, addr);          //address[7:0]
	while((read_reg8(0x800009)&0x40)!=0);
	
	write_reg8(0x800008, dat);          //data
	while((read_reg8(0x800009)&0x40)!=0);

	write_reg8(0x800009, 0x03);          //cs=1
#endif

}

u8 spi_read(u8 addr){
#if(1)
	spi_high();
	spi_low();
	spi_out(addr);
	u8 dat = spi_in();
	spi_high();
	return dat;
#else
	u8  dat;
	write_reg8(0x800009, 0x03); 		 //cs=1
	write_reg8(0x800009, 0x02); 		 //cs=0

	write_reg8(0x800008, addr); 			//address[7:0]
	while((read_reg8(0x800009)&0x40)!=0);

	write_reg8(0x800009, 0x0a);
	while(read_reg8(0x800009) & 0x40);

	read_reg8(0x800008);
	while(read_reg8(0x800009) & 0x40);

	dat = read_reg8(0x800008);

	write_reg8(0x800009, 0x03); 		 //cs=1
	return (dat);
#endif	
}


