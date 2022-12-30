/********************************************************************************************************
 * @file GetMeshDeviceNotificationParser.java
 *
 * @brief for TLSR chips
 *
 * @author telink
 * @date Sep. 30, 2010
 *
 * @par Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
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

import com.telink.bluetooth.TelinkLog;
import com.telink.util.Arrays;

/**
 * 获取mesh设备列表 notify数据解析
 */
public final class GetMeshDeviceNotificationParser extends NotificationParser<GetMeshDeviceNotificationParser.MeshDeviceInfo> {

    private GetMeshDeviceNotificationParser() {
    }

    public static GetMeshDeviceNotificationParser create() {
        return new GetMeshDeviceNotificationParser();
    }

    @Override
    public byte opcode() {
        return Opcode.BLE_GATT_OP_CTRL_E1.getValue();
    }

    @Override
    public MeshDeviceInfo parse(NotificationInfo notifyInfo) {

        byte[] params = notifyInfo.params;


        TelinkLog.d("mesh device info notification parser: " + Arrays.bytesToHexString(params, ":"));
        // 63,71,FB,6C,00,C3,02,E1,11,02,6C,00,6C,88,1D,63,FF,FF,00,00
        // params : 6C,00,6C,88,1D,63,FF,FF,00,00

//        int offset = 0;
        MeshDeviceInfo meshDeviceInfo = new MeshDeviceInfo();
        meshDeviceInfo.deviceId = (params[0] & 0xFF) | (params[1] & 0xFF << 8);
        meshDeviceInfo.macBytes = new byte[6];
        System.arraycopy(params, 2, meshDeviceInfo.macBytes, 0, 6);
        meshDeviceInfo.productUUID = (params[8] & 0xFF) | (params[9] & 0xFF << 8);
        // 反转高低位
//        Arrays.reverse(meshDeviceInfo.macBytes);
        return meshDeviceInfo;
    }


    public final class MeshDeviceInfo {

        public int deviceId;

        public int newDeviceId = -1;

        public byte[] macBytes;

        public int productUUID;
    }
}
