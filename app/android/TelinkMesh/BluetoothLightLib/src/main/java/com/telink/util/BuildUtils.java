/********************************************************************************************************
 * @file     BuildUtils.java 
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

import android.os.Build;

public final class BuildUtils {

    private BuildUtils() {
    }

    public static int assetSdkVersion(String version) {

        String[] v1 = version.split("\\.");
        String[] v2 = Build.VERSION.RELEASE.split("\\.");

        int len1 = v1.length;
        int len2 = v2.length;

        int len = len1 > len2 ? len2 : len1;

        int tempV1;
        int tempV2;

        for (int i = 0; i < len; i++) {
            tempV1 = Integer.parseInt(v1[i]);
            tempV2 = Integer.parseInt(v2[i]);

            if (tempV2 < tempV1)
                return -1;
            else if (tempV2 > tempV1)
                return 1;
        }

        return 0;
    }
}
