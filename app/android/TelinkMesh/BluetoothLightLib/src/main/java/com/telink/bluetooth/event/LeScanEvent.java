/********************************************************************************************************
 * @file     LeScanEvent.java 
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

import com.telink.bluetooth.light.DeviceInfo;

/**
 * 扫描事件
 */
public class LeScanEvent extends DataEvent<DeviceInfo> {

    /**
     * 扫描到设备
     */
    public static final String LE_SCAN = "com.telink.bluetooth.light.EVENT_LE_SCAN";
    public static final String LE_SCAN_COMPLETED = "com.telink.bluetooth.light.EVENT_LE_SCAN_COMPLETED";
    /**
     * 扫描超时
     */
    public static final String LE_SCAN_TIMEOUT = "com.telink.bluetooth.light.EVENT_LE_SCAN_TIMEOUT";

    public LeScanEvent(Object sender, String type, DeviceInfo args) {
        super(sender, type, args);
    }

    public static LeScanEvent newInstance(Object sender, String type, DeviceInfo args) {
        return new LeScanEvent(sender, type, args);
    }
}
