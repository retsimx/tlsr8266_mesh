/********************************************************************************************************
 * @file     LeUpdateParameters.java 
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

import java.util.Arrays;

/**
 * 更新网络参数
 *
 * @see LightService#updateMesh(Parameters)
 */
public final class LeUpdateParameters extends Parameters {

    /**
     * 创建{@link LeOtaParameters}实例
     *
     * @return
     */
    public static LeUpdateParameters create() {
        return new LeUpdateParameters();
    }

    /**
     * 旧的网络名
     *
     * @param value
     * @return
     */
    public LeUpdateParameters setOldMeshName(String value) {
        this.set(PARAM_MESH_NAME, value);
        return this;
    }

    /**
     * 新的网络名
     *
     * @param value
     * @return
     */
    public LeUpdateParameters setNewMeshName(String value) {
        this.set(PARAM_NEW_MESH_NAME, value);
        return this;
    }

    /**
     * 旧的密码
     *
     * @param value
     * @return
     */
    public LeUpdateParameters setOldPassword(String value) {
        this.set(PARAM_MESH_PASSWORD, value);
        return this;
    }

    /**
     * 新的密码
     *
     * @param value
     * @return
     */
    public LeUpdateParameters setNewPassword(String value) {
        this.set(PARAM_NEW_PASSWORD, value);
        return this;
    }

    /**
     * LTK,如果不设置将使用厂商默认值,即{@link Manufacture#getFactoryLtk()}
     *
     * @param value
     * @return
     */
    public LeUpdateParameters setLtk(byte[] value) {
        this.set(PARAM_LONG_TERM_KEY, value);
        return this;
    }

    /**
     * 更新的设备列表
     *
     * @param value
     * @return
     * @see DeviceInfo
     */
    public LeUpdateParameters setUpdateDeviceList(DeviceInfo... value) {
        this.set(PARAM_DEVICE_LIST, Arrays.asList(value));
        return this;
    }
}
