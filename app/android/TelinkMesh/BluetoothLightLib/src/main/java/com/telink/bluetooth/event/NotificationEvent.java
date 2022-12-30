/********************************************************************************************************
 * @file     NotificationEvent.java 
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

import android.app.Application;

import com.telink.bluetooth.light.NotificationInfo;
import com.telink.bluetooth.light.NotificationParser;
import com.telink.bluetooth.light.Opcode;

import java.util.HashMap;
import java.util.Map;

/**
 * 通知事件,比如设备的状态/分组信息发生变化等
 */
public class NotificationEvent extends DataEvent<NotificationInfo> {

    /**
     * 设备的状态变化事件
     */
    public static final String RAW = "com.telink.bluetooth.light.EVENT_NOTIFY_RAW";


    /**
     * 设备的状态变化事件
     */
    public static final String ONLINE_STATUS = "com.telink.bluetooth.light.EVENT_ONLINE_STATUS";
    /**
     * 分组事件
     */
    public static final String GET_GROUP = "com.telink.bluetooth.light.EVENT_GET_GROUP";
    /**
     * 闹铃事件
     */
    public static final String GET_ALARM = "com.telink.bluetooth.light.EVENT_GET_ALARM";
    /**
     * 场景事件
     */
    public static final String GET_SCENE = "com.telink.bluetooth.light.EVENT_GET_SCENE";

    /**
     * 时间同步事件
     */
    public static final String GET_TIME = "com.telink.bluetooth.light.EVENT_GET_TIME";


    /**
     * UserAll
     */
    public static final String USER_ALL_NOTIFY = "com.telink.bluetooth.light.USER_ALL_NOTIFY";

    /**
     * 获取设备版本号
     */
    public static final String GET_DEVICE_STATE = "com.telink.bluetooth.light.GET_DEVICE_STATE";

    /**
     * 获取MeshOTA上报的进度,与 GET_DEVICE_STATE 是同一个opCode返回
     */
    public static final String GET_MESH_OTA_PROGRESS = "com.telink.bluetooth.light.GET_MESH_OTA_PROGRESS";


    /**
     * 获取当前mesh内所有设备的deviceId， mac地址
     */
    public static final String GET_MESH_DEVICE_LIST = "com.telink.bluetooth.light.GET_MESH_DEVICE_LIST";


    /**
     * mesh加灯完成标志
     */
    public static final String UPDATE_MESH_COMPLETE = "com.telink.bluetooth.light.GET_UPDATE_MESH_COMPLETE";
    /**
     * GET_DEVICE_STATE 返回数据的第一个字节
     * 获取版本号
     */
    public static final byte DATA_GET_VERSION = 0x00;

    /**
     * GET_DEVICE_STATE 返回数据的第一个字节
     * 获取进度
     */
    public final static byte DATA_GET_MESH_OTA_PROGRESS = 0x04;

    /**
     * GET_DEVICE_STATE 返回数据的第一个字节
     * 获取OTA状态信息
     */
    public final static byte DATA_GET_OTA_STATE = 0x05;

    /**
     * 设置设备OTA模式， gatt OTA / meshOTA
     * 第一个字节返回0x00， 则表示支持
     */
    public final static byte DATA_SET_OTA_MODE_NOTIFY = 0x06;
    /**
     * READ_ST_IDLE = 0,    // 没有处于mesh ota状态
     * READ_ST_SLAVE = 1,       // 处于正在接收mesh ota firmware的状态，并且会保存该firmware数据
     * READ_ST_MASTER = 2,    // 处于正在发送mesh ota firmware的状态
     * READ_ST_ONLY_RELAY = 3,    //处于正在接收mesh ota firmware的状态，但是不会保存该firmware数据(比如版本号不符合等)，字节把该数据relay出去。
     */
    public final static byte OTA_STATE_IDLE = 0;
    public final static byte OTA_STATE_SLAVE = 1;
    public final static byte OTA_STATE_MASTER = 2;
    public final static byte OTA_STATE_ONLY_RELAY = 3;
    public final static byte OTA_STATE_COMPLETE = 4;


    private static final Map<Byte, String> EVENT_MAPPING = new HashMap<>();

    static {
        register(Opcode.BLE_GATT_OP_CTRL_DC, ONLINE_STATUS);
        register(Opcode.BLE_GATT_OP_CTRL_D4, GET_GROUP);
        register(Opcode.BLE_GATT_OP_CTRL_E7, GET_ALARM);
        register(Opcode.BLE_GATT_OP_CTRL_E9, GET_TIME);
        register(Opcode.BLE_GATT_OP_CTRL_C1, GET_SCENE);
        register(Opcode.BLE_GATT_OP_CTRL_C8, GET_DEVICE_STATE);

        register(Opcode.BLE_GATT_OP_CTRL_CA, UPDATE_MESH_COMPLETE);

        register(Opcode.BLE_GATT_OP_CTRL_EB, USER_ALL_NOTIFY);

        register(Opcode.BLE_GATT_OP_CTRL_E1, GET_MESH_DEVICE_LIST);
    }

    /**
     * 操作码
     */
    protected int opcode;
    /**
     * 源地址,即设备/组地址
     */
    protected int src;

    public NotificationEvent(Object sender, String type, NotificationInfo args) {
        super(sender, type, args);

        this.opcode = args.opcode;
        this.src = args.src;
    }

    /**
     * 注册事件类型
     *
     * @param opcode
     * @param eventType
     * @return
     */
    public static boolean register(byte opcode, String eventType) {
        opcode = (byte) (opcode & 0xFF);
        synchronized (NotificationEvent.class) {
            if (EVENT_MAPPING.containsKey(opcode))
                return false;
            EVENT_MAPPING.put(opcode, eventType);
            return true;
        }
    }

    /**
     * 注册事件类型
     *
     * @param opcode
     * @param eventType
     * @return
     * @see NotificationEvent#register(byte, String)
     */
    public static boolean register(Opcode opcode, String eventType) {
        return register(opcode.getValue(), eventType);
    }

    /**
     * 获取事件类型
     *
     * @param opcode 操作码
     * @return
     */
    public static String getEventType(byte opcode) {
        opcode = (byte) (opcode & 0xFF);
        synchronized (NotificationEvent.class) {
            if (EVENT_MAPPING.containsKey(opcode))
                return EVENT_MAPPING.get(opcode);
        }
        return null;
    }

    public static String getEventType(Opcode opcode) {
        return getEventType(opcode.getValue());
    }

    public static NotificationEvent newInstance(Application sender, String type, NotificationInfo args) {
        return new NotificationEvent(sender, type, args);
    }

    public Object parse() {
        NotificationParser parser = NotificationParser.get(this.opcode);
        return parser == null ? null : parser.parse(this.args);
    }
}
