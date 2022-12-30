/********************************************************************************************************
 * @file     GetTimeNotificationParser.java 
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

import java.util.Calendar;

/**
 * 时间同步通知解析器
 */
public final class GetTimeNotificationParser extends NotificationParser<Calendar> {

    private GetTimeNotificationParser() {
    }

    public static GetTimeNotificationParser create() {
        return new GetTimeNotificationParser();
    }

    @Override
    public byte opcode() {
        return Opcode.BLE_GATT_OP_CTRL_E9.getValue();
    }

    @Override
    public Calendar parse(NotificationInfo notifyInfo) {

        byte[] params = notifyInfo.params;
        int offset = 0;

        int year = ((params[offset++] & 0xFF) << 8) + (params[offset++] & 0xFF);
        int month = (params[offset++] & 0xFF) - 1;
        int day = params[offset++] & 0xFF;
        int hour = params[offset++] & 0xFF;
        int minute = params[offset++] & 0xFF;
        int second = params[offset] & 0xFF;

        Calendar calendar = Calendar.getInstance();
        calendar.set(year, month, day, hour, minute, second);

        return calendar;
    }
}
