/********************************************************************************************************
 * @file     LeScanParameters.java 
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
 * 扫描参数类
 * <p>{@link LeScanParameters}定义了{@link LightService#startScan(Parameters)}方法的必须要设置的几项参数.
 *
 * @see LightService#startScan(Parameters)
 */
public final class LeScanParameters extends Parameters {

    /**
     * 创建LeScanParameters实例
     *
     * @return
     */
    public static LeScanParameters create() {
        return new LeScanParameters();
    }

    /**
     * 网络名
     *
     * @param value
     * @return
     */
    public LeScanParameters setMeshName(String value) {
        this.set(Parameters.PARAM_MESH_NAME, value);
        return this;
    }

    /**
     * 超时时间(单位秒),在这个时间段内如果没有发现任何设备将停止扫描.
     *
     * @param value
     * @return
     */
    public LeScanParameters setTimeoutSeconds(int value) {
        this.set(Parameters.PARAM_SCAN_TIMEOUT_SECONDS, value);
        return this;
    }

    /**
     * 踢出网络后的名称,默认值为out_of_mesh
     *
     * @param value
     * @return
     */
    public LeScanParameters setOutOfMeshName(String value) {
        this.set(PARAM_OUT_OF_MESH, value);
        return this;
    }

    /**
     * 扫描模式,true时扫描到一个设备就会立即停止扫描.
     *
     * @param singleScan
     * @return
     */
    public LeScanParameters setScanMode(boolean singleScan) {
        this.set(Parameters.PARAM_SCAN_TYPE_SINGLE, singleScan);
        return this;
    }

    /**
     * 扫描的设备mac
     *
     * @param mac 目标地址
     * @return this
     */
    public LeScanParameters setScanMac(String mac) {
        this.set(PARAM_SCAN_MAC, mac);
        return this;
    }


    /**
     * 扫描设备类型过滤
     *
     * @param type 目标类型
     * @return
     */
    public LeScanParameters setScanTypeFilter(int type) {
        this.set(PARAM_SCAN_TYPE_FILTER, type);
        return this;
    }

}
