/********************************************************************************************************
 * @file     LeOtaParameters.java 
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
 * OTA参数
 *
 * @see LightService#startOta(Parameters)
 */
public final class LeOtaParameters extends Parameters {

    /**
     * 创建{@link LeOtaParameters}实例
     *
     * @return
     */
    public static LeOtaParameters create() {
        return new LeOtaParameters();
    }

    /**
     * 网络名
     *
     * @param value
     * @return
     */
    public LeOtaParameters setMeshName(String value) {
        this.set(PARAM_MESH_NAME, value);
        return this;
    }

    /**
     * 密码
     *
     * @param value
     * @return
     */
    public LeOtaParameters setPassword(String value) {
        this.set(PARAM_MESH_PASSWORD, value);
        return this;
    }

    /**
     * 扫描超时时间,单位秒
     *
     * @param value
     * @return
     */
    public LeOtaParameters setLeScanTimeoutSeconds(int value) {
        this.set(PARAM_SCAN_TIMEOUT_SECONDS, value);
        return this;
    }

    /**
     * 要进行OTA的设备
     *
     * @param value
     * @return
     * @see OtaDeviceInfo
     */
    public LeOtaParameters setDeviceInfo(OtaDeviceInfo value) {
        this.set(PARAM_DEVICE_LIST, value);
        return this;
    }
}
