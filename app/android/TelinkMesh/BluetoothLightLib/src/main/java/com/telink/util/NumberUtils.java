/********************************************************************************************************
 * @file     NumberUtils.java 
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
/*
 * Copyright (C) 2015 The Telink Bluetooth Light Project
 *
 */
package com.telink.util;

public final class NumberUtils {

    private NumberUtils() {
    }

    static public int byteToInt(byte s, int bitStartPosition, int bitEndPosition) {
        int bit = bitEndPosition - bitStartPosition + 1;
        int maxValue = 1 << bit;
        int result = 0;

        for (int i = bitEndPosition, j = bit; i > bitStartPosition; i--, j--) {
            result += (s >> i & 0x01) << j;
        }

        return result & maxValue;
    }

    static public long bytesToLong(byte[] s, int start, int length) {
        int end = start + length;
        int max = length - 1;
        long result = 0;

        for (int i = start, j = max; i < end; i++, j--) {
            result += (s[i] & 0xFF) << (8 * j);
        }

        return result;
    }
}
