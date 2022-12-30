/********************************************************************************************************
 * @file Lights.java
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
package com.telink.bluetooth.light.model;

import android.text.TextUtils;

import com.telink.bluetooth.light.ConnectionStatus;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Lights extends DataStorageImpl<Light> {

    private static Lights mThis;

    private Lights() {
        super();
    }

    public static Lights getInstance() {

        if (mThis == null)
            mThis = new Lights();

        return mThis;
    }

    public boolean contains(int meshAddress) {
        synchronized (this) {
            return this.contains("meshAddress", meshAddress);
        }
    }

    public Light getByMeshAddress(int meshAddress) {
        synchronized (this) {
            return this.get("meshAddress", meshAddress);
        }
    }

    /**
     * 获取本地存储的设备列表
     *
     * @return
     */
    public List<Light> getLocalList(boolean online) {
        if (data == null || data.isEmpty()) {
            return null;
        }
        List<Light> localList = new ArrayList<>();
        for (Light light : data) {
            if (!TextUtils.isEmpty(light.macAddress) && (!online || light.connectionStatus != ConnectionStatus.OFFLINE)) {
                localList.add(light);
            }
        }

        Collections.sort(localList, new Comparator<Light>() {
            @Override
            public int compare(Light lhs, Light rhs) {
                return
                        lhs.productUUID != rhs.productUUID ?
                                (lhs.productUUID - rhs.productUUID) : (lhs.meshAddress - rhs.meshAddress);
            }
        });
        return localList;
    }


    public int getOnlineCount() {
        if (data == null || data.isEmpty()) {
            return 0;
        }
        int result = 0;
        for (Light light : data) {
            if (light.connectionStatus != ConnectionStatus.OFFLINE) {
                result++;
            }
        }
        return 0;
    }
}
