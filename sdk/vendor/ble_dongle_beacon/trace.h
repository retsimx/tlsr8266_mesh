/********************************************************************************************************
 * @file     trace.h 
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

#ifndef TRACE_H_
#define TRACE_H_


#define TR_T_abuf_add           0
#define TR_T_abuf_dec           1
#define TR_T_abuf_usb           2
#define TR_T_abuf_zero          3
#define TR_T_abuf_overflow      4
#define TR_T_abuf_overflow_mic  5

#define TR_24_abuf_mic_wptr          0
#define TR_24_abuf_dec_wptr          1
#define TR_24_abuf_dec_rptr          2
#define TR_24_abuf_reset             3
#define TR_24_abuf_rf_header         4

#endif
