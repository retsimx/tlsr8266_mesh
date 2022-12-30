/********************************************************************************************************
 * @file     AdvertiseDataFilter.java 
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

import android.bluetooth.BluetoothDevice;

/**
 * 广播包过滤接口
 *
 * @param <E>
 */
public interface AdvertiseDataFilter<E extends LightPeripheral> {

    /**
     * 过滤接口
     *
     * @param device     扫描到的蓝牙设备
     * @param rssi       信号强度
     * @param scanRecord 广播数据包
     * @return
     */
    E filter(BluetoothDevice device, int rssi,
             byte[] scanRecord);
}
