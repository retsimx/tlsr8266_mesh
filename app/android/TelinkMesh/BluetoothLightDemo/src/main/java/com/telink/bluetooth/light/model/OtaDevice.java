/********************************************************************************************************
 * @file     OtaDevice.java 
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
package com.telink.bluetooth.light.model;

import java.io.Serializable;

/**
 * MeshOTA升级过程中会将正在升级的设备保存在本地
 * 在意外退出或者连接状态不稳定时，优先连接保存的设备；
 * Created by Administrator on 2017/4/25.
 */

public class OtaDevice implements Serializable {
    private static final long serialVersionUID = 2L;

    // saved mesh info
    public String meshName;
    public String meshPwd;
    public String mac;
}
