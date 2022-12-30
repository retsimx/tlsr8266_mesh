/********************************************************************************************************
 * @file     MeshOtaDeviceInfo.java 
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
public class MeshOtaDeviceInfo extends DeviceInfo {

    public static final Creator<MeshOtaDeviceInfo> CREATOR = new Creator<MeshOtaDeviceInfo>() {
        @Override
        public MeshOtaDeviceInfo createFromParcel(Parcel in) {
            return new MeshOtaDeviceInfo(in);
        }

        @Override
        public MeshOtaDeviceInfo[] newArray(int size) {
            return new MeshOtaDeviceInfo[size];
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

    public int type;

    public int mode;

    public MeshOtaDeviceInfo() {
    }

    public MeshOtaDeviceInfo(Parcel in) {
        super(in);
        this.progress = in.readInt();
        this.type = in.readInt();
        this.mode = in.readInt();
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        super.writeToParcel(dest, flags);
        dest.writeInt(this.progress);
        dest.writeInt(this.type);
        dest.writeInt(this.mode);
    }
}
