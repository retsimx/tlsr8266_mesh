/********************************************************************************************************
 * @file     OtaDeviceInfo.java 
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
package com.telink.bluetooth.light;

import android.os.Parcel;

/**
 * OTA设备信息
 */
public class OtaDeviceInfo extends DeviceInfo {

    public static final Creator<OtaDeviceInfo> CREATOR = new Creator<OtaDeviceInfo>() {
        @Override
        public OtaDeviceInfo createFromParcel(Parcel in) {
            return new OtaDeviceInfo(in);
        }

        @Override
        public OtaDeviceInfo[] newArray(int size) {
            return new OtaDeviceInfo[size];
        }
    };

    /**
     * firmware数据
     */
    public byte[] firmware;
    /**
     * ota进度
     */
    public int progress;

    public OtaDeviceInfo() {
    }

    public OtaDeviceInfo(Parcel in) {
        super(in);
        this.progress = in.readInt();
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        super.writeToParcel(dest, flags);
        dest.writeInt(this.progress);
    }
}
