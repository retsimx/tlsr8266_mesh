/********************************************************************************************************
 * @file     NotificationParser.java 
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

import android.util.SparseArray;

/**
 * Notification解析器接口
 * <p>继承NotificationParser编写自定义的解析器,通过{@link NotificationParser#register(NotificationParser)}来注册.
 *
 * @param <E>
 */
public abstract class NotificationParser<E> {

    private static final SparseArray<NotificationParser> PARSER_ARRAY = new SparseArray<>();

    /**
     * 注册解析器
     *
     * @param parser
     */
    public static void register(NotificationParser parser) {
        synchronized (NotificationParser.class) {
            PARSER_ARRAY.put(parser.opcode() & 0xFF, parser);
        }
    }

    /**
     * 获取解析器
     *
     * @param opcode 操作码
     * @return
     */
    public static NotificationParser get(int opcode) {
        synchronized (NotificationParser.class) {
            return PARSER_ARRAY.get(opcode & 0xFF);
        }
    }

    public static NotificationParser get(Opcode opcode) {
        return get(opcode.getValue());
    }

    /**
     * 操作码
     *
     * @return
     */
    abstract public byte opcode();

    /**
     * 将{@link NotificationInfo#params}转换成自定义的数据格式
     *
     * @param notifyInfo
     * @return
     */
    abstract public E parse(NotificationInfo notifyInfo);
}
