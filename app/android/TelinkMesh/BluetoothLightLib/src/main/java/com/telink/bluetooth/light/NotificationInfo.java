/********************************************************************************************************
 * @file     NotificationInfo.java 
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
import android.os.Parcelable;

/**
 * NotificationInfo封装收到的蓝牙Notification信息
 */
public final class NotificationInfo implements Parcelable {

    public static final Creator<NotificationInfo> CREATOR = new Creator<NotificationInfo>() {
        @Override
        public NotificationInfo createFromParcel(Parcel in) {
            return new NotificationInfo(in);
        }

        @Override
        public NotificationInfo[] newArray(int size) {
            return new NotificationInfo[size];
        }
    };

    /**
     * 操作码
     */
    public int opcode;

    /**
     * vendor Id
     */
    public int vendorId;
    /**
     * 源地址
     */
    public int src;
    /**
     * 参数
     */
    public byte[] params = new byte[10];

    /**
     * 当前连接的设备
     */
    public DeviceInfo deviceInfo;

    public NotificationInfo() {
    }

    public NotificationInfo(Parcel in) {
        this.opcode = in.readInt();
        this.vendorId = in.readInt();
        this.src = in.readInt();
        in.readByteArray(this.params);
        Object ret = in.readValue(DeviceInfo.class.getClassLoader());
        if (ret != null) {
            this.deviceInfo = (DeviceInfo) ret;
        }
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeInt(this.opcode);
        dest.writeInt(this.vendorId);
        dest.writeInt(this.src);
        dest.writeByteArray(this.params);

        if (this.deviceInfo != null) {
            dest.writeValue(this.deviceInfo);
        }
    }

    @Override
    public int describeContents() {
        return 0;
    }
}
