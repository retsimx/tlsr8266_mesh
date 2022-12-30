/********************************************************************************************************
 * @file     uart.c 
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
/**************************************************************************************************
  Filename:       	uart.c
  Author:			junjun.xu@telink-semi.com
  Created Date:	2016/06/05
  
  Description:    This file contains the uart driver functions for the Telink 8267/8269. Before the uart can work, user should first choose the UART TX, RX output
  			  pins (disable the correspond pins' gpio functions, gpio initate)

  
**************************************************************************************************/
#include "../../proj/tl_common.h"
#include "uart.h"
#include "../mcu/watchdog_i.h"

#define		STARTTX			write_reg8(0x800524,0x02)//trigger dma
#define		TXDONE			(((*(volatile unsigned char  *)0x80009e) & 0x01) ? 1:0)
#define		RXERRORCLR		(*(volatile unsigned char  *)0x80009d |= 0x40)
#define		RXERROR			(((*(volatile unsigned char  *)0x80009d) & 0x80) ? 1:0)

#if(MCU_CORE_TYPE == MCU_CORE_8266)
#define UART_CONTINUE_DELAY_EN          1
#else
#define UART_CONTINUE_DELAY_EN          0
#endif

volatile unsigned char uart_tx_busy_flag = 0;                   // must "volatile"
static unsigned char *tx_buff = NULL;
#if(UART_CONTINUE_DELAY_EN)
static volatile unsigned int uart_continue_delay_time = 0;      // must "volatile"
static unsigned int baudrate_set = 0;
#endif

/**********************************************************
*
*	@brief	reset uart module
*	
*	@param	none
*
*	@return	none
*/
void uart_Reset(void){

	write_reg8(0x800061,0x80);
	write_reg8(0x800061,0x00);
	
}
/**********************************************************
*	
*	@brief	clear error state of uart rx, maybe used when application detected UART not work
*
*	@parm	none
*
*	@return	'1' RX error flag rised and cleard success; '0' RX error flag not rised 
*
*/
unsigned char uart_ErrorCLR(void){
	if(RXERROR){
		RXERRORCLR;
		return 1;
	}
	return 0;
}

void uart_set_tx_busy_flag(){
    uart_tx_busy_flag = 1;
    #if(UART_CONTINUE_DELAY_EN)
    uart_continue_delay_time = 0;
    #endif
}

void uart_clr_tx_busy_flag(){
    #if(UART_CONTINUE_DELAY_EN)
    uart_continue_delay_time = clock_time() | 1; // make sure not zero
    #else
    uart_tx_busy_flag = 0;
    #endif
}

unsigned char uart_tx_is_busy(){
#if(UART_CONTINUE_DELAY_EN)
    return uart_tx_busy_flag;
#else
    return (!TXDONE);
#endif
}

/*******************************************************
*
*	@brief	uart initiate, set uart clock divider, bitwidth and the uart work mode
*
*	@param	uartCLKdiv - uart clock divider
*			bwpc - bitwidth, should be set to larger than 2
*			en_rx_irq - '1' enable rx irq; '0' disable.
*			en_tx_irq - enable tx irq; '0' disable.
*			hdwC - enum variable of hardware control functions
*
*	@return	'1' set success; '0' set error probably bwpc smaller than 3.
*
*		BaudRate = sclk/((uartCLKdiv+1)*(bwpc+1))  
*		SYCLK = 16Mhz
		115200		9			13
		9600		103			15
*
*		SYCLK = 32Mhz
*		115200		19			13
		9600		237			13	
*/

unsigned char uart_Init(unsigned short uartCLKdiv, unsigned char bwpc,unsigned char en_rx_irq,unsigned char en_tx_irq,enum HARDWARECONTROL hdwC){
	
	if(bwpc<3)
		return 0;

    unsigned int  baudrate = (CLOCK_SYS_CLOCK_HZ / (uartCLKdiv + 1)) / (bwpc + 1);
	#if(UART_CONTINUE_DELAY_EN)
	baudrate_set = baudrate;
	#endif
	write_reg16(0x800094,(uartCLKdiv|0x8000));//set uart clk divider and enable clock divider
	write_reg8(0x800096,(0x30|bwpc));//set bit width and enable rx/tx DMA
	
	#if (UART_115200RX_TIMEOUT_2MS_EN)
	if(baudrate > 100000){          // 115200
    	write_reg8(0x80009a,0xff);
    	write_reg8(0x80009b,3);
	}else
	#endif
	{
    	write_reg8(0x80009a,(bwpc+1)*12);//one byte includes 12 bits at most
    	write_reg8(0x80009b,1);//For save
	}
	
	//write_reg8(0x800097,0x00);//No clts and rts
	write_reg8(0x800097,(unsigned char)hdwC);//set hardware control function
	
	//receive DMA and buffer details
	
	write_reg8(0x800503,0x01);//set DMA 0 mode to 0x01 for receive
	write_reg8(0x800507,0x00);//DMA1 mode to send
	uart_IRQSourceGet();//clear uart irq
	if(en_rx_irq){
		*(volatile unsigned char  *)0x800521 |= 0x01;//open dma1 interrupt mask
		*(volatile unsigned char  *)0x800640 |= 0x10;//open dma interrupt mask
		//irq_enable(); // write_reg8(0x800643,0x01);//enable intterupt
	}
	if(en_tx_irq){
		*(volatile unsigned char  *)0x800521 |= 0x02;//open dma1 interrupt mask
		*(volatile unsigned char  *)0x800640 |= 0x10;//open dma interrupt mask
		//irq_enable(); // write_reg8(0x800643,0x01);//enable intterupt
	}
	return 1;
	
}

/********************************************************************************
*	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start 
*			the DMA send function
*
*	@param	sendBuff - send data buffer
*
*	@return	'1' send success; '0' DMA busy
*/

unsigned char uart_Send(unsigned char* addr){
    unsigned long len = *((unsigned long *)addr);
    if((len > 252) && (!tx_buff)){
        return 0;
    }
    
    unsigned long t_timeout = clock_time();
    while(uart_tx_is_busy() && (!clock_time_exceed(t_timeout, 400*1000))){
        #if(MODULE_WATCHDOG_ENABLE)
		wd_clear();
        #endif
        
        #if(UART_CONTINUE_DELAY_EN)
        if(uart_continue_delay_time && clock_time_exceed(uart_continue_delay_time, (baudrate_set > 100000 ? 800 : 9000))){
            uart_continue_delay_time = 0;    // minimum delay :  115200 delay 600us;  9600 delay 7200us
            uart_tx_busy_flag = 0;
        }
        #endif   
    }
    
    uart_set_tx_busy_flag();
    memcpy(tx_buff, addr, len + 4);
	write_reg16(0x800504,(unsigned short)((unsigned int)tx_buff));//packet data, start address is sendBuff+1

	STARTTX;
	
	return 1;
}

unsigned char uart_tx_busy_check(){
    #if(UART_CONTINUE_DELAY_EN)
    if(uart_tx_is_busy()){      
        if(uart_continue_delay_time && clock_time_exceed(uart_continue_delay_time, (baudrate_set > 100000 ? 800 : 9000))){
            uart_continue_delay_time = 0;    // minimum delay :  115200 delay 600us;  9600 delay 7200us
            uart_tx_busy_flag = 0;
        }else{
            return 1;
        }
    }
    return 0;
    
    #else
	return uart_tx_is_busy();
	#endif
}

void uart_tx_busy_clear(){
    #if(UART_CONTINUE_DELAY_EN)
    uart_continue_delay_time = 0;
    uart_tx_busy_flag = 0;  
    #else
	//TXBUSYCLR;    // no need
	#endif
}

/****************************************************************************************
*
*	@brief	data receive buffer initiate function. DMA would move received uart data to the address space, uart packet length
*			needs to be no larger than (recBuffLen - 4).
*
*	@param	*recAddr:	receive buffer's address info.
*			recBuffLen:	receive buffer's length, the maximum uart packet length should be smaller than (recBuffLen - 4)
*
*	@return	none
*/

void uart_BuffInit(unsigned char *recAddr, unsigned short recBuffLen, unsigned char *txAddr){
	unsigned char bufLen;
	bufLen = recBuffLen/16;
	write_reg16(0x800500,(unsigned short)((unsigned int)(recAddr)));//set receive buffer address
	write_reg8(0x800502,bufLen);//set receive buffer size

    tx_buff = txAddr;
}

/******************************************************************************
*
*	@brief		get the uart IRQ source and clear the IRQ status, need to be called in the irq process function
*
*	@return		uart_irq_src- enum variable of uart IRQ source, 'UARTRXIRQ' or 'UARTTXIRQ'
*
*/
u8 uart_IRQSourceGet(void){
	unsigned char irqS;
	irqS = read_reg8(0x800526);
	write_reg8(0x800526,irqS);//CLR irq source
#if 0
	if(irqS & 0x01)
		return UARTRXIRQ;
	
	if(irqS & 0x02)
		return UARTTXIRQ;

    return UARTNOIRQ;
#else
    return (irqS & UARTIRQ_MASK);
#endif    
}

void uart_io_init(unsigned char uart_io_sel){
#if (MCU_CORE_TYPE == MCU_CORE_8267)
    if(UART_GPIO_8267_PA6_PA7 == uart_io_sel){
        UART_GPIO_CFG_PA6_PA7();
    }else if(UART_GPIO_8267_PC2_PC3 == uart_io_sel){
        UART_GPIO_CFG_PC2_PC3();
    }else if(UART_GPIO_8267_PB2_PB3 == uart_io_sel){
        UART_GPIO_CFG_PB2_PB3();
    }
#elif (MCU_CORE_TYPE == MCU_CORE_8269)
    if(UART_GPIO_8269_PA6_PA7 == uart_io_sel){
        UART_GPIO_CFG_PA6_PA7();
    }else if(UART_GPIO_8269_PC2_PC3 == uart_io_sel){
        UART_GPIO_CFG_PC2_PC3();
    }else if(UART_GPIO_8269_PB2_PB3 == uart_io_sel){
        UART_GPIO_CFG_PB2_PB3();
    }
#elif (MCU_CORE_TYPE == MCU_CORE_8266)
    uart_io_sel = uart_io_sel;
    gpio_set_func(GPIO_UTX, AS_UART); 
    gpio_set_func(GPIO_URX, AS_UART); 
#endif
}
