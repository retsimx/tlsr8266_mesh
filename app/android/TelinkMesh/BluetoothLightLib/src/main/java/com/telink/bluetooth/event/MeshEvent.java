/********************************************************************************************************
 * @file     MeshEvent.java 
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

/**
 * 网络事件
 */
public class MeshEvent extends DataEvent<Integer> {

    public static final String UPDATE_COMPLETED = "com.telink.bluetooth.light.EVENT_UPDATE_COMPLETED";
    /**
     * 连接到不任何设备的时候分发此事件
     */
    public static final String OFFLINE = "com.telink.bluetooth.light.EVENT_OFFLINE";
    /**
     * 出现异常时分发此事件,比如蓝牙关闭了
     */
    public static final String ERROR = "com.telink.bluetooth.light.EVENT_ERROR";

    public MeshEvent(Object sender, String type, Integer args) {
        super(sender, type, args);
    }

    public static MeshEvent newInstance(Object sender, String type, Integer args) {
        return new MeshEvent(sender, type, args);
    }
}
