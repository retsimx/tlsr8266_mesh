/********************************************************************************************************
 * @file     ServiceEvent.java 
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
package com.telink.bluetooth.event;

import android.os.IBinder;

/**
 * LightService事件
 */
public class ServiceEvent extends DataEvent<IBinder> {

    /**
     * 服务启动
     */
    public static final String SERVICE_CONNECTED = "com.telink.bluetooth.light.EVENT_SERVICE_CONNECTED";
    /**
     * 服务关闭
     */
    public static final String SERVICE_DISCONNECTED = "com.telink.bluetooth.light.EVENT_SERVICE_DISCONNECTED";

    public ServiceEvent(Object sender, String type, IBinder args) {
        super(sender, type, args);
    }

    public static ServiceEvent newInstance(Object sender, String type, IBinder args) {
        return new ServiceEvent(sender, type, args);
    }
}