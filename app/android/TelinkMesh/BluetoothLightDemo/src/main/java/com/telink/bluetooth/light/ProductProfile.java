/********************************************************************************************************
 * @file     ProductProfile.java 
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

public enum ProductProfile {

    DIM(0x0001, "单色灯"), CCT(0x0002, ""), RGBW(0x0003, "四色灯，红绿蓝白"), RGB(0x0004, "三色灯，红绿蓝"), C_SLEEP(0x0005, "色温灯"), UNKNOWN(-1, "未定义");

    private int value;
    private String info;

    ProductProfile(int value, String info) {
        this.value = value;
        this.info = info;
    }

    public static ProductProfile valueOf(int value) {
        if (value == DIM.getValue())
            return DIM;
        if (value == CCT.getValue())
            return CCT;
        if (value == RGBW.getValue())
            return RGBW;
        if (value == RGB.getValue())
            return RGB;
        if (value == C_SLEEP.getValue())
            return C_SLEEP;

        return UNKNOWN;
    }

    public int getValue() {
        return value;
    }

    public String getInfo() {
        return info;
    }
}
