/********************************************************************************************************
 * @file     printf.c 
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
#include <stdarg.h>
#include "../../proj/tl_common.h"  


#if(PRINT_DEBUG_INFO)

#define			DECIMAL_OUTPUT		10
#define			OCTAL_OUTPUT		8
#define			HEX_OUTPUT			16


//#define va_start(ap,v)    (ap = (char *)((int)&v + sizeof(v)))
//#define va_arg(ap,t)      ((t *)(ap += sizeof(t)))[-1]

#ifndef		BIT_INTERVAL
#define		BIT_INTERVAL	(CLOCK_SYS_CLOCK_1S/PRINT_BAUD_RATE)
#endif

_attribute_no_retention_bss_ static int tx_pin_initialed = 0;

/**
 * @brief  DEBUG_INFO_TX_PIN initialize. Enable 1M pull-up resistor,
 *   set pin as gpio, enable gpio output, disable gpio input.
 * @param  None
 * @retval None
 */
_attribute_no_inline_ void debug_info_tx_pin_init()
{
    gpio_set_func(DEBUG_INFO_TX_PIN, AS_GPIO);
	gpio_write(DEBUG_INFO_TX_PIN, 1);
    gpio_set_output_en(DEBUG_INFO_TX_PIN, 1);
    gpio_set_input_en(DEBUG_INFO_TX_PIN, 0);
}

/* Put it into a function independently, to prevent the compiler from 
 * optimizing different pins, resulting in inaccurate baud rates.
 */
_attribute_ram_code_ 
_attribute_no_inline_ 
static void uart_do_put_char(u32 pcTxReg, u8 *bit)
{
	int j;
#if PRINT_BAUD_RATE == 1000000
	/*! Make sure the following loop instruction starts at 4-byte alignment */
	// _ASM_NOP_; 
	
	for(j = 0;j<10;j++) 
	{
	#if CLOCK_SYS_CLOCK_HZ == 16000000
		CLOCK_DLY_8_CYC;
	#elif CLOCK_SYS_CLOCK_HZ == 32000000
		CLOCK_DLY_7_CYC;CLOCK_DLY_7_CYC;CLOCK_DLY_10_CYC;
	#elif CLOCK_SYS_CLOCK_HZ == 48000000
		CLOCK_DLY_8_CYC;CLOCK_DLY_8_CYC;CLOCK_DLY_10_CYC;
		CLOCK_DLY_8_CYC;CLOCK_DLY_6_CYC;
	#else
	#error "error CLOCK_SYS_CLOCK_HZ"
	#endif
		write_reg8(pcTxReg, bit[j]); 	   //send bit0
	}
#else
	u32 t1 = 0, t2 = 0;
	t1 = read_reg32(0x740);
	for(j = 0;j<10;j++)
	{
		t2 = t1;
		while(t1 - t2 < BIT_INTERVAL){
			t1	= read_reg32(0x740);
		}
		write_reg8(pcTxReg,bit[j]); 	   //send bit0
	}
#endif
}



_attribute_ram_code_ static void uart_put_char(u8 byte){
	if (!tx_pin_initialed) {
	    debug_info_tx_pin_init();
		tx_pin_initialed = 1;
	}

	u32 pcTxReg = (0x583+((DEBUG_INFO_TX_PIN>>8)<<3));//register GPIO output
	u8 tmp_bit0 = read_reg8(pcTxReg) & (~(DEBUG_INFO_TX_PIN & 0xff));
	u8 tmp_bit1 = read_reg8(pcTxReg) | (DEBUG_INFO_TX_PIN & 0xff);


	u8 bit[10] = {0};
	bit[0] = tmp_bit0;
	bit[1] = (byte & 0x01)? tmp_bit1 : tmp_bit0;
	bit[2] = ((byte>>1) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[3] = ((byte>>2) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[4] = ((byte>>3) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[5] = ((byte>>4) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[6] = ((byte>>5) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[7] = ((byte>>6) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[8] = ((byte>>7) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[9] = tmp_bit1;

	
	/*! Minimize the time for interrupts to close and ensure timely 
	    response after interrupts occur. */
#if SIMU_UART_IRQ_EN
	u8 r = irq_disable();
#endif
	uart_do_put_char(pcTxReg, bit);
#if SIMU_UART_IRQ_EN
	irq_restore(r);
#endif
}

static int puts(char *s){
	while((*s != '\0')){
		uart_put_char(*s++);
	}
	return 0;
}

static void puti(unsigned int num, int base){
	char re[]="0123456789ABCDEF";

	char buf[50];

	char *addr = &buf[49];

	*addr = '\0';

	do{
		*--addr = re[num%base];
		num/=base;
	}while(num!=0);

	puts(addr);
}


int mini_printf(const char *format, ...){

	char span;
	unsigned long j;
	char *s;
	//char *msg;
	va_list arg_ptr;
	va_start(arg_ptr, format);

	while((span = *(format++))){
		if(span != '%'){
			uart_put_char(span);
		}else{
			span = *(format++);
			if(span == 'c'){
				j = va_arg(arg_ptr,int);//get value of char
				uart_put_char(j);
			}else if(span == 'd'){
				j = va_arg(arg_ptr,int);//get value of char
				#if 0
				if(j<0){
					uart_put_char('-');
					j = -j;
				}
				#endif
				puti(j,DECIMAL_OUTPUT);
			}else if(span == 's'){
				s = va_arg(arg_ptr,char *);//get string value
				puts(s);
			}else if(span == 'o'){
				j = va_arg(arg_ptr,unsigned int);//get octal value
				puti(j,OCTAL_OUTPUT);
			}else if(span == 'x'){
					j = va_arg(arg_ptr,unsigned int);//get hex value
					puti(j,HEX_OUTPUT);
			}else if(span == 0){
				break;
			}else{
				uart_put_char(span);
			}
		}

	}
	va_end(arg_ptr);
	return 0;
}

u8 HexTable[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
void PrintHex(u8 x)
{
	uart_put_char('0');
	uart_put_char('x');
	uart_put_char(HexTable[x>>4]);
	uart_put_char(HexTable[x&0xf]);
	uart_put_char(' ');
}

#if (PRINT_DEBUG_INFO)

void printfserial(char *str, u8 *p, int len)
{
//	u8 r = irq_disable();
	printf("\n");
	printf("%s:",str);
	for(int i=0;i<len;i++)
	{
		//printf("%x",*p++);
	}
	foreach(i,len){PrintHex(p[i]);}
	
	printf("\n");
//	irq_restore(r);
}

////////////////////////////////////// Test printf///////////////////////////////
void Test_printf(void)
{
	#if 1
		static u32 indicate_tick=0;
		static u32 tick_loop;
		if(clock_time_exceed(indicate_tick, 1000000))
		{
			tick_loop ++;
			
			indicate_tick=clock_time();
			#if (PRINT_DEBUG_INFO)
			//printf("indicate_tick=%x\n",tick_loop);
			#endif
			printfserial("indicate_tick",(u8 *)&indicate_tick,4);
		}
	#endif	
}	
///////////////////////////////////////////////////////////////////////////



#endif



#endif

