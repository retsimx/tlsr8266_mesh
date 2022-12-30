/********************************************************************************************************
 * @file     myi2c.c 
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
#include "myi2c.h"

//////////////////////////////////////////////////////////////////////
///// i2c_nb: non-blocking access
//////////////////////////////////////////////////////////////////////
#ifndef I2C_USE_SIMULATION
#define I2C_USE_SIMULATION	0
#endif
#ifndef I2C_SPEED
#define I2C_SPEED	50
#endif
#ifndef I2C_16BIT_ADDR
#define I2C_16BIT_ADDR		0
#endif

///////////////  I2C simulation ////////////////////////////////////

#ifndef PIN_I2C_SCL
#define PIN_I2C_SCL				GPIO_CK
#endif
#ifndef PIN_I2C_SDA
#define PIN_I2C_SDA				GPIO_DI
#endif

static inline void myi2c_sim_wait(void){
}

static inline void myi2c_sim_long_wait(void){
	//CLOCK_DLY_600NS;
}

// Pulling the line to ground is considered a logical zero while letting the line float is a logical one.   http://en.wikipedia.org/wiki/I%C2%B2C
static inline void myi2c_sim_scl_out(int v){
	gpio_set_output_en(PIN_I2C_SCL,(!v));
}

static inline int myi2c_sim_scl_in(void){
	return gpio_read(PIN_I2C_SCL);
}

// Pulling the line to ground is considered a logical zero while letting the line float is a logical one.   http://en.wikipedia.org/wiki/I%C2%B2C
static inline void myi2c_sim_sda_out(int v){
	gpio_set_output_en(PIN_I2C_SDA,(!v));
}

static inline int myi2c_sim_sda_in(void){
	return gpio_read(PIN_I2C_SDA);
}

static inline void myi2c_sim_scl_init(void){
	gpio_set_func(PIN_I2C_SCL, AS_GPIO); 
}

static inline void myi2c_sim_sda_init(void){
	gpio_set_func(PIN_I2C_SDA, AS_GPIO); 
	gpio_set_input_en(PIN_I2C_SDA, 1);
}

static inline void myi2c_sim_scl_idle(void){
	gpio_set_output_en(PIN_I2C_SCL, 0); 
	gpio_write(PIN_I2C_SCL, 0);
}

static inline void myi2c_sim_sda_idle(void){
	gpio_set_output_en(PIN_I2C_SDA, 0); 
	gpio_write(PIN_I2C_SDA, 0);
}

void myi2c_sim_init(){}

/*
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\\ void i2c_sim_start(void)
\\   Sets clock high, then data high.  This will do a stop if data was low.
\\   Then sets data low, which should be a start condition.
\\   After executing, data is left low, while clock is left high
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
*/
void myi2c_sim_start(void)
{
	myi2c_sim_scl_init();
	myi2c_sim_sda_init();
	myi2c_sim_sda_idle();
	myi2c_sim_scl_idle();
	myi2c_sim_sda_out(0);		//sda: 0
	myi2c_sim_wait();

}

void myi2c_sim_stop(void)
{
	myi2c_sim_scl_out(0);
	myi2c_sim_wait();
	myi2c_sim_sda_out(0);
	myi2c_sim_wait();
	myi2c_sim_scl_out(1);
	myi2c_sim_wait();
	myi2c_sim_sda_out(1);
}

static inline void myi2c_sim_wirte_bit(int bit)
{
	myi2c_sim_scl_out(0);
	myi2c_sim_sda_out(bit);
	myi2c_sim_long_wait();
	myi2c_sim_scl_out(1);
}

// Read a bit from I2C bus
static int myi2c_sim_read_bit(void){
	myi2c_sim_wirte_bit(1);
	return myi2c_sim_sda_in();
}

int myi2c_sim_write_byte(u8 dat){
	int i = 0x80;
	while(i){
		myi2c_sim_wirte_bit((dat & i));
		i = i >> 1;
	}
	return myi2c_sim_read_bit();
}

static inline u8 myi2c_sim_read_byte(int last){
	u8 dat = 0;
	foreach(i, 8){
		myi2c_sim_wirte_bit(1);
		if(myi2c_sim_sda_in()){
			dat =(dat << 1) | 0x01;
		}else{
			dat = dat << 1;
		}
	}
	myi2c_sim_wirte_bit(last);
	return dat;
}

void myi2c_sim_write(u8 id, u8 addr, u8 dat)
{
	myi2c_sim_start();
	myi2c_sim_write_byte(id);
	myi2c_sim_write_byte(addr);
	myi2c_sim_write_byte(dat);
	myi2c_sim_stop();
}

u8 myi2c_sim_read(u8 id, u8 addr)
{
	u8 dat;
	myi2c_sim_burst_read(id, addr, &dat, 1);
	return dat;
}

void myi2c_sim_burst_read(u8 id, u32 addr,u8 *p,u32 n)
{
	myi2c_sim_start();
	
	myi2c_sim_write_byte(id);
	myi2c_sim_write_byte(addr);
	myi2c_sim_sda_out(1);
	myi2c_sim_scl_out(0);
	myi2c_sim_long_wait();
	myi2c_sim_scl_out(1);
	myi2c_sim_sda_out(0);
	
	myi2c_sim_write_byte(id | 1);

	for(int k = 0; k < n; ++k){
		*p++ = myi2c_sim_read_byte( k ==(n-1) );
	}
	myi2c_sim_stop();
	
}

void myi2c_sim_burst_write_ll(u8 id, u32 addr,u8 *p,u32 n)
{
	myi2c_sim_start();
	myi2c_sim_write_byte(id);
	myi2c_sim_write_byte(addr);
	foreach(i, n){
		myi2c_sim_write_byte(*p++);
	}
	myi2c_sim_stop();
	
	sleep_us(10*1000);  // at least 8ms
}

#define I2C_PAGE_SIZE     8
void myi2c_sim_burst_write(u8 id, u32 addr,u8 *p,u32 n){
    u32 len_empty = I2C_PAGE_SIZE - (addr & 0x07);
    while(n){
        if(n >= len_empty){
            myi2c_sim_burst_write_ll(id, addr, p, len_empty);
            n -= len_empty;
            addr += len_empty;
            p += len_empty;
            len_empty = I2C_PAGE_SIZE;
        }else{
            myi2c_sim_burst_write_ll(id, addr, p, n);
            n = 0;
        }
    }
}

