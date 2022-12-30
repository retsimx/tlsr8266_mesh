/********************************************************************************************************
 * @file     uart.h 
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
  Filename:       	uart.h
  Author:			junjun.xu@telink-semi.com
  Created Date:	2016/06/05
  
  Description:    This file contains the details of enum variables and functions used in the uart.c file

  
**************************************************************************************************/
#include "../tl_common.h"

enum{
	UARTRXIRQ_MASK  = BIT(0),
	UARTTXIRQ_MASK  = BIT(1),
	UARTIRQ_MASK    = UARTRXIRQ_MASK | UARTTXIRQ_MASK,
};

#if (MCU_CORE_TYPE == MCU_CORE_8258)
#include "uart_8258.h"
#define uart_tx_busy_clear()        //do{}while(0);
#define uart_clr_tx_busy_flag()     //do{}while(0);
#elif (MCU_CORE_TYPE == MCU_CORE_8278)
#include "uart_8278.h"
#define uart_tx_busy_clear()        //do{}while(0);
#define uart_clr_tx_busy_flag()     //do{}while(0);
#else
#ifndef 	uart_H
#define 	uart_H
enum HARDWARECONTROL{
	CTSWODDPARITY = 0x0e,
	CTSWEVENPARITY = 0x06,
	CTSONLY = 0x02,
	ODDPARITY = 0x0C,
	EVENPARITY = 0x04,
	NOCONTROL = 0x00,
};
enum UARTIRQSOURCE{
	UARTRXIRQ,
	UARTTXIRQ,
	UARTNOIRQ,
};

#define UART_115200RX_TIMEOUT_2MS_EN        1

#define CLK32M_UART9600			uart_Init(237,13,1,1,NOCONTROL)     // default rx timeout: 2.5ms
#if (UART_115200RX_TIMEOUT_2MS_EN)
#define	CLK32M_UART115200		uart_Init(69,3,1,1,NOCONTROL)
#define	CLK16M_UART115200		uart_Init(34,3,1,1,NOCONTROL)
#else
#define	CLK32M_UART115200		uart_Init(19,13,1,1,NOCONTROL)      // default rx timeout: 0.21ms
#define	CLK16M_UART115200		uart_Init(9,13,1,1,NOCONTROL)       // default rx timeout: 0.21ms
#endif
#define	CLK16M_UART9600			uart_Init(103,15,1,1,NOCONTROL)     // default rx timeout: 2.5ms

//UART_TX/UART_RX gpio pin config
#define	   UART_GPIO_CFG_PA6_PA7()  do{\
										*(volatile unsigned char  *)0x8005b0 |= 0x80;\
										*(volatile unsigned char  *)0x800586 &= 0x3f;\
								    }while(0) 
#define	   UART_GPIO_CFG_PB2_PB3()  do{\
										*(volatile unsigned char  *)0x8005b1 |= 0x0c;\
										*(volatile unsigned char  *)0x80058e &= 0xf3;\
								    }while(0)  
#define	   UART_GPIO_CFG_PC2_PC3()  do{\
										*(volatile unsigned char  *)0x8005b2 |= 0x0c;\
										*(volatile unsigned char  *)0x800596 &= 0xf3;\
								    }while(0)  

#define UART_GPIO_8266              1

#define UART_GPIO_8267_PA6_PA7      1
#define UART_GPIO_8267_PC2_PC3      2
#define UART_GPIO_8267_PB2_PB3      3
#define UART_GPIO_8269_PA6_PA7      UART_GPIO_8267_PA6_PA7
#define UART_GPIO_8269_PC2_PC3      UART_GPIO_8267_PC2_PC3
#define UART_GPIO_8269_PB2_PB3      UART_GPIO_8267_PB2_PB3
#define UART_GPIO_8258_PA0_PA2		4 //RX:PA0 TX:PA2
#define UART_GPIO_8258_PB0_PB1		5 //RX:PB0 TX:PB1

void uart_io_init(unsigned char uart_io_sel);

/**********************************************************
*
*	@brief	reset uart module
*	
*	@param	none
*
*	@return	none
*/
extern void uart_Reset(void );


/**********************************************************
*	
*	@brief	clear error state of uart rx, maybe used when application detected UART not work
*
*	@parm	none
*
*	@return	'1' RX error flag rised and cleard success; '0' RX error flag not rised 
*
*/
unsigned char uart_ErrorCLR(void);


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
extern unsigned char uart_Init(unsigned short uartCLKdiv, unsigned char bwpc,unsigned char en_rx_irq,unsigned char en_tx_irq,enum HARDWARECONTROL hdwC);



/********************************************************************************
*	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start 
*			the DMA send function
*
*	@param	sendBuff - send data buffer
*
*	@return	'1' send success; '0' DMA busy
*/
extern unsigned char uart_Send(unsigned char* addr);


/********************************************************************
*	
*	@brief	uart receive function, call this function to get the UART data
*
*	@param	userDataBuff - data buffer to store the uart data
*
*	@return	'0' rx error; 'rxLen' received data length
*/
//extern unsigned short uart_Rec(unsigned char* addr);

/******************************************************************************
*
*	@brief		get the uart IRQ source and clear the IRQ status, need to be called in the irq process function
*
*	@return		uart_irq_src- enum variable of uart IRQ source, 'UARTRXIRQ' or 'UARTTXIRQ'
*
*/
extern u8 uart_IRQSourceGet(void);

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

extern void uart_BuffInit(unsigned char *recAddr, unsigned short recBuffLen, unsigned char *txAddr);

void uart_clr_tx_busy_flag();
unsigned char uart_tx_busy_check();
void uart_tx_busy_clear();

#endif
#endif
