/********************************************************************************************************
 * @file     bqb_bsp.h 
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
/**********************************************************************************************************************
* 閺傚洣娆㈤崥宥囆為敍锟� bsp.h
* 閸愬懎顔愰幗妯款洣閿涳拷8267_bleplus
* 瑜版挸澧犻悧鍫熸拱閿涳拷 V1.0
* 娴ｏ拷    閼板拑绱伴梽鍫濆殹闁硷拷
* 鐎瑰本鍨氶弮銉︽埂閿涳拷 2015.06.25
*
* 娣囶喗鏁肩拋鏉跨秿1閿涳拷
**********************************************************************************************************************/

#ifndef BSP_H
#define BSP_H

#define BIT(n)                  		( 1<<(n) )

#define BIT0                            0x1
#define BIT1                            0x2
#define BIT2                            0x4
#define BIT3                            0x8
#define BIT4                            0x10
#define BIT5                            0x20
#define BIT6                            0x40
#define BIT7                            0x80
#define BIT8                            0x100
#define BIT9                            0x200
#define BIT10                           0x400
#define BIT11                           0x800
#define BIT12                           0x1000
#define BIT13                           0x2000
#define BIT14                           0x4000
#define BIT15                           0x8000
#define BIT16                           0x10000
#define BIT17                           0x20000
#define BIT18                           0x40000
#define BIT19                           0x80000
#define BIT20                           0x100000
#define BIT21                           0x200000
#define BIT22                           0x400000
#define BIT23                           0x800000
#define BIT24                           0x1000000
#define BIT25                           0x2000000
#define BIT26                           0x4000000
#define BIT27                           0x8000000
#define BIT28                           0x10000000
#define BIT29                           0x20000000
#define BIT30                           0x40000000
#define BIT31                           0x80000000

#define REG_BASE_ADDR			0x800000

//#define write_reg32(addr,data)          (*(volatile unsigned long  *)(addr)=data)
//#define write_reg16(addr,data)          (*(volatile unsigned short *)(addr)=data)
//#define write_reg8(addr,data)           (*(volatile unsigned char  *)(addr)=data)
//#define read_reg32(addr)                (*(volatile unsigned long  *)(addr))
//#define read_reg16(addr)                (*(volatile unsigned short *)(addr))
//#define read_reg8(addr)                 (*(volatile unsigned char  *)(addr))

#define IOBASE                          0x800000



#define EDPS_DAT                        0x800118
#define TICK0                           0x800630
//-------------------complier---------------------

#define attribute_ram_code   __attribute__((section(".ram_code")))

//-------------------SET_BIT---------------------
#define 	SETB(v,n)	(*(volatile unsigned char  *)v |= n)
#define 	CLRB(v,n)	(*(volatile unsigned char  *)v &= ~n)
#define 	TEST(v,n)	(((*(volatile unsigned char  *)v) & n) ? 1:0)

//-------------------Clock_init---------------------

struct  S_SYS_RST0_BITS {
   unsigned char  spi:1;
   unsigned char  i2c:1;
   unsigned char  usb:1;
   unsigned char  usb_phy:1;
   unsigned char  mcu:1;
   unsigned char  mac:1;
   unsigned char  aif:1;
   unsigned char  zb:1;
};
union   U_RST0 {
  struct  S_SYS_RST0_BITS  bits;
  unsigned  char   all;
};

struct  S_SYS_RST1_BITS {
	unsigned char  system_timer:1;
	unsigned char   algm:1;
	unsigned char   dma:1;
	unsigned char   rs232:1;
	unsigned char   pwm0:1;
	unsigned char   aes:1;
	unsigned char   bbpll48m:1;
	unsigned char   swires:1;
};
union  U_RST1 {
   struct   S_SYS_RST1_BITS  bits;
	unsigned char  all;
};
struct  S_SYS_RST2_BITS {
	unsigned char   sbc:1;
	unsigned char   audio:1;
	unsigned char   dfifo:1;
	unsigned char   adc:1;
	unsigned char   mcic:1;
	unsigned char   reset_mcic_enable:1;
	unsigned char   mspi:1;
	unsigned char   algs:1;
};

union  U_RST2 {
   struct   S_SYS_RST2_BITS  bits;
	unsigned char  all;
};

union  U_CLK0 {
	struct   S_SYS_RST0_BITS  bits;
	unsigned char  all;
};

struct  S_SYS_CLK1_BITS {
	unsigned char  system_timer:1;
	unsigned char   algm:1;
	unsigned char   dma:1;
	unsigned char   rs232:1;
	unsigned char   pwm0:1;
	unsigned char   aes:1;
	unsigned char   clk_32k_system_timer:1;
	unsigned char   swires:1;
};
union  U_CLK1 {
   struct   S_SYS_CLK1_BITS  bits;
	unsigned char  all;
};

struct  S_SYS_CLK2_BITS {
	unsigned char  clk32k_qdec:1;
    unsigned char  audio:1;
    unsigned char  dfifo:1;
    unsigned char  key_scan:1;
	unsigned char  mcic:1;
	unsigned char  qdec:1;
	unsigned char  pwm32k:1;
	unsigned char  keyscan32K:1;

};
union  U_CLK2 {
    struct  S_SYS_CLK2_BITS  bits;
	unsigned char  all;
};


enum  M_SYSCLK_SEL {
  SYS_CLK_RC = 0,
  SYS_CLK_HS_DIV = 1,
 // SYS_CLK_PAD_16M = 2,
 // SYS_CLK_RC_32K = 3
};
struct  S_CLK_CTL {
    unsigned short   clk_div:5;
    unsigned short   sysclk_sel:2;
	unsigned short   hs_sel:2;
};
union  U_CLK_CTL {
    struct  S_CLK_CTL   bits;
	unsigned short      all;

};

struct  S_SYS_CTL {
	union U_RST0   rst0;
	union U_RST1   rst1;
	union U_RST2   rst2;
    union U_CLK0   clk0;
	union U_CLK1   clk1;
	union U_CLK2   clk2;
	union U_CLK_CTL  clkctl;

};



// ****************************************************************************
// extern   interface
// ****************************************************************************

extern  void   Sys_Init(struct  S_SYS_CTL  * p_sys_ctl,enum M_SYSCLK_SEL clock_src,unsigned char clock_divider);
#define NULL        0




//-------------------Read/Write register---------------------

#define DEBUG 0
#define DEBUG_FALSH_WR 0
#define DEBUG_TBL 0
#define DEBUG_FUNC 0

#define	ADR_IO		0x800000

#define TCMD_UNDER_RD		0x80
#define TCMD_UNDER_WR		0x40
#define TCMD_UNDER_BOTH		0xc0
#define TCMD_MASK		0x3f

#define TCMD_WRITE		0x3
#define TCMD_WAIT		0x7
#define TCMD_WAREG		0x8

//typedef struct TBLCMDSET_BQB {
//	unsigned short	adr;
//	unsigned char	dat;
//	unsigned char	cmd;
//} TBLCMDSET_BQB;

//int LoadTblCmdSet_bqb (
//		const TBLCMDSET_BQB * pt,		//pointer to command table
//		int size		//size of command table
//		);

#define    GET_VERSION()    ((unsigned char)100)     //Version:1.0.0

extern unsigned long	tick_per_us;
 
extern void WriteAnalogReg (unsigned char adr, unsigned char dat);
extern unsigned char ReadAnalogReg(unsigned char addr);
extern void WaitUs_bqb (int microsec);
extern void WaitMs_bqb (int millisec);
extern unsigned long clock_time_bqb(void);
void SetTickUs (unsigned int t);

#endif //#ifndef BSP_H
