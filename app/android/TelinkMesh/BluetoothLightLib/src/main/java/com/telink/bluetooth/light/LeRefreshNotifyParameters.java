/********************************************************************************************************
 * @file     LeRefreshNotifyParameters.java 
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

/**
 * 自动刷新Notify参数
 *
 * @see LightService#autoRefreshNotify(Parameters)
 */
public final class LeRefreshNotifyParameters extends Parameters {

    /**
     * 创建{@link LeRefreshNotifyParameters}实例
     *
     * @return
     */
    public static LeRefreshNotifyParameters create() {
        return new LeRefreshNotifyParameters();
    }

    /**
     * 刷新次数
     *
     * @param value
     * @return
     */
    public LeRefreshNotifyParameters setRefreshRepeatCount(int value) {
        this.set(PARAM_AUTO_REFRESH_NOTIFICATION_REPEAT, value);
        return this;
    }

    /**
     * 间隔时间,单位毫秒
     *
     * @param value
     * @return
     */
    public LeRefreshNotifyParameters setRefreshInterval(int value) {
        this.set(PARAM_AUTO_REFRESH_NOTIFICATION_DELAY, value);
        return this;
    }
}
