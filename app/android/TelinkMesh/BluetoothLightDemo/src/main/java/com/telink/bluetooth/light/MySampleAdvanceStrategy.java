/********************************************************************************************************
 * @file     MySampleAdvanceStrategy.java 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
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
package com.telink.bluetooth.light;

public class MySampleAdvanceStrategy extends AdvanceStrategy {

    public final static String TAG = "AdvanceStrategy";

    public MySampleAdvanceStrategy() {
    }

    @Override
    public boolean postCommand(byte opcode, int address, byte[] params, int delay, Object tag, boolean noResponse, boolean immediate) {
        //所有采样到的命令立即交给回调接口处理
        if (this.mCallback != null)
            return this.mCallback.onCommandSampled(opcode, address, params, tag, delay);
        return false;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onStop() {

    }
}