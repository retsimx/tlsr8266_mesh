/********************************************************************************************************
 * @file     rf_test.c 
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
#include "../../proj_lib/bqb_bsp.h"
#include "../../proj/common/types.h"
#include "../../proj_lib/bqb_rf.h"
#include "./Phytest.h"
#include "../../proj/mcu/analog.h"
#include "../../proj/common/compatibility.h"
#include "../../proj/common/utility.h"

u8 phyTest_Channel(u8 chn) {
	if (chn == 0) {
		return 37;
	} else if (chn < 12) {
		return chn - 1;
	} else if (chn == 12) {
		return 38;
	} else if (chn < 39) {
		return chn - 2;
	} else {
		return 39;
	}
}

void rf_set_tx_rx_off_bqb() {
	/////////////////// turn on LDO and baseband PLL ////////////////
	//analog_write (0x06, 0xfe);
	write_reg8 (0x800f16, 0x29);
	write_reg8 (0x800428, 0x80); // rx disable
	write_reg8 (0x800f02, 0x45); // reset tx/rx state machine
}
