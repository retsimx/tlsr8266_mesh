
#ifdef MCU_STARTUP_8266

#ifndef __LOAD_RAM_SIZE__
#define	__LOAD_RAM_SIZE__		0xc
#endif

	.code	16
@********************************************************************************************************
@                                           MACROS AND DEFINIITIONS
@********************************************************************************************************
.include "version.in"
#include "version.h"

					@ Mode, correspords to bits 0-5 in CPSR
	.equ MODE_BITS,		0x1F	@ Bit mask for mode bits in CPSR
	.equ IRQ_MODE, 		0x12	@ Interrupt Request mode
	.equ SVC_MODE, 		0x13	@ Supervisor mode 

	.equ IRQ_STK_SIZE,	0x600
	.equ __LOAD_RAM, 	__LOAD_RAM_SIZE__
	.equ __RAM_START_ADDR, 	(0x808000) 
	.equ __RAM_SIZE_MAX, 	(16*1024)
	.equ __DATA_ADR_OFFSET, 	(0)  @size for no ramcode when use 826x_ram.S.must 0 for cache mode.
	

@********************************************************************************************************
@                                            TC32 EXCEPTION VECTORS
@********************************************************************************************************

	.section	.vectors,"ax"
	.global		__reset
	.global	 	__irq
	.global 	__start
	.global		__LOAD_RAM
	.global		__RAM_START_ADDR 
	.global		__RAM_SIZE_MAX 
	.global		__DATA_ADR_OFFSET

__start:					@ MUST,  referenced by boot.link

	.extern irq_handler

	.org 0x0
	tj	__reset
	.word	(XTAL_16MHZ << 31 | BUILD_VERSION)
	.org 0x8
	.word	(0x544c4e4b)
	.word	(0x00880120)    @(0x00880000 + _ramcode_size_div_16_align_256_)

	.org 0x10
	tj		__irq
	.org 0x18
	.word	(_bin_size_)
	.word   (FW_DEV_INFO)
@********************************************************************************************************
@                                   LOW-LEVEL INITIALIZATION
@********************************************************************************************************
	.extern  main_entrypoint

	.org 0x20
__reset:
@	tloadr     	r0, DAT0 + 36
@	tmov		r1, #1024		@ set sws to GPIO
@	tstorer 	r1, [r0, #0]

@	tloadr     	r0, DAT0 + 40		@**** enable watchdog at the very first time
@	tloadr     	r1, DAT0 + 44
@	tstorer		r1, [r0, #0]

	tloadr	r0, FLL_D
	tloadr	r1, FLL_D+4
	tloadr	r2, FLL_D+8

FLL_STK:
	tcmp	r1, r2
	tjge	FLL_STK_END
	tstorer r0, [r1, #0]
	tadd    r1, #4
	tj		FLL_STK
FLL_STK_END:

	tloadr	r0, DAT0
	tmcsr	r0			
	tloadr	r0, DAT0 + 8
	tmov	r13, r0  

	tloadr	r0, DAT0 + 4
	tmcsr	r0	
	tloadr	r0, DAT0 + 12
	tmov	r13, r0  

	tmov	r0, #0
	tloadr	r1, DAT0 + 16
	tloadr	r2, DAT0 + 20

ZERO:
	tcmp	r1, r2
	tjge	ZERO_END
	tstorer	r0, [r1, #0]
	tadd    r1, #4
	tj		ZERO
ZERO_END:

	tloadr	r1, DAT0 + 28
	tloadr	r2, DAT0 + 32

ZERO_TAG:
	tcmp	r1, r2
	tjge	ZERO_TAG_END
	tstorer	r0, [r1, #0]
	tadd    r1, #4
	tj		ZERO_TAG
ZERO_TAG_END:

SETIC:
	tloadr     	r1, DAT0 + 24
	tloadr      r0, DAT0 + 36					@ IC tag start
#ifdef MCU_CORE_8266_OTA_BOOT
	tloadr      r0, DAT0 + 36
	tloadr      r0, DAT0 + 36
	tloadr      r0, DAT0 + 36
#else
	tstorerb	r0, [r1, #0]
	tadd    	r0, #1							@ IC tag end
	tstorerb	r0, [r1, #1]
#endif
	@tmov		r0, #0;
	@tstorerb	r0, [r1, #2]

//COPY_CODE_INIT:
//     tmov		r3, #63			@ OTA FW ready flag at 0x3f000; (0x3f000 = #63 << 12)
//     tshftl		r3, r3, #12		@ 0x3f<<12
//     tloadr		r3, [r3, #0]
//#ifdef MCU_CORE_8266_OTA_BOOT
//	 tcmp		r3, #0			@ 0x0
//#else
//     tcmp		r3, #165		@ *(u32 *)0x3f000 == 0xa5
//#endif
//     tjne		COPY_DATA_INIT
//
//     tmov		r2, #31		@ OTA boot code at: 0x1f000 (OTA128K: 0x1f000 = 31 << 12)
//     tloadr		r3, COPY_CODE_DAT
//     tloadr		r0, COPY_CODE_DAT + 4
//     tshftl		r2, r2, #12		@ (OTA128K: #12)
//COPY_CODE_START:
//     tloadm		r2!, {r1}
//     tstorem	r3!, {r1}
//     tcmp		r3, r0
//     tjne		COPY_CODE_START
//
//     tloadr		r3, COPY_CODE_DAT + 8
//     tmov		r2, #136		@0x88
//     tstorerb	r2, [r3, #0]
//
//COPY_CODE_END:
//     tj		COPY_CODE_END
//	.balign	4
//COPY_CODE_DAT:
//	.word	(0x808000)
//	.word	(0x808600)
//	.word	(0x800602)

COPY_DATA_INIT:
	tloadr		r1, DATA_I
	tloadr		r2, DATA_I+4
	tloadr		r3, DATA_I+8
COPY_DATA:
	tcmp		r2, r3
	tjge		COPY_DATA_END
	tloadr		r0, [r1, #0]
	tstorer 	r0, [r2, #0]
	tadd    	r1, #4
	tadd		r2, #4
	tj			COPY_DATA
COPY_DATA_END:

#if 0
SETSPISPEED:
	tloadr     	r1, DAT0 + 36
	tmov		r0, #0xbb		@0x0b for fast read; 0xbb for dual dat/adr
	tstorerb	r0, [r1, #0]
	tmov		r0, #3			@3 for dual dat/adr
	tstorerb	r0, [r1, #1]
#endif

	tjl	main_entrypoint
END:	tj	END

	.balign	4
DAT0:
	.word	0x12			    @IRQ    @0
	.word	0x13			    @SVC    @4
	.word	(irq_stk + IRQ_STK_SIZE)
	.word	(__RAM_START_ADDR + __RAM_SIZE_MAX)		    @12  stack end
	.word	(_start_bss_)               @16
	.word	(_end_bss_)                 @20
	.word	(0x80060c)                  @24
	.word	_ictag_start_               @28		@ IC tag start
	.word	_ictag_end_	            	@32		@ IC tag end
	.word	_ramcode_size_div_256_		@36
@	.word	(0x808000 + __LOAD_RAM * 0x100)                  @28		@ IC tag start
@	.word	(0x808000 + (__LOAD_RAM + 1) * 0x100)                  @32		@ IC tag end
@	.word	(0x80000e)    				@36
@	.word	(0x80058c)                  @36		gpio
@	.word	(0x800620)                  @40		watchdog
@	.word	(0x802c01)                  @44		watchdog
DATA_I:	
	.word	_dstored_
	.word	_start_data_
	.word	_end_data_

FLL_D:
	.word	0xffffffff
	.word	(_start_data_)
	.word	(__RAM_START_ADDR + __RAM_SIZE_MAX)
	.word   (_rstored_)

	.align 4
__irq:
	tpush    	{r14}
	tpush    	{r0-r7}
	tmrss    	r0
	
	tmov		r1, r8
	tmov		r2, r9
	tmov		r3, r10
	tmov		r4, r11
	tmov		r5, r12
	tpush		{r0-r5}
	
	tjl      	irq_handler

	tpop		{r0-r5}
	tmov		r8, r1
	tmov		r9, r2
	tmov		r10,r3
	tmov		r11,r4
	tmov		r12,r5

	tmssr    	r0
	tpop		{r0-r7}
	treti    	{r15}

ASMEND:

	.section .bss
	.align 4
	.lcomm irq_stk, IRQ_STK_SIZE
	.end

#endif
