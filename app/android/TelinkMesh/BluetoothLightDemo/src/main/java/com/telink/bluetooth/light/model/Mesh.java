/********************************************************************************************************
 * @file     Mesh.java 
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

import android.content.Context;
import android.text.TextUtils;

import com.telink.bluetooth.light.util.FileSystem;
import com.telink.util.MeshUtils;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Mesh implements Serializable {

    private static final long serialVersionUID = 1L;

    public String name;
    public String password;
    public String factoryName;
    public String factoryPassword;
    public OtaDevice otaDevice;

    //public String otaDevice;

//    public List<Integer> allocDeviceAddress;
    public List<Light> devices = new ArrayList<>();



    public int getDeviceAddress() {
        if (devices == null || devices.size() == 0) {
            return 1;
        }

        flag_index:
        for (int i = MeshUtils.DEVICE_ADDRESS_MIN; i <= MeshUtils.DEVICE_ADDRESS_MAX; i++) {
            for (Light lxDeviceInfo : devices) {
                if (lxDeviceInfo.meshAddress == i) {
                    continue flag_index;
                }
            }
            return i;
        }

        return -1;


        /*int address = MeshUtils.allocDeviceAddress(this.allocDeviceAddress);

        if (address != -1) {
            if (this.allocDeviceAddress == null)
                this.allocDeviceAddress = new ArrayList<>();
            this.allocDeviceAddress.add(address);
        }

        return address;*/
    }

    public Light getDevice(int meshAddress) {
        if (this.devices == null)
            return null;

        for (Light info : devices) {
            if (info.meshAddress == meshAddress)
                return info;
        }
        return null;
    }

    public boolean removeDeviceByMeshAddress(int meshAddress) {
        if (devices == null || devices.size() == 0) {
            return false;
        }

        Iterator<Light> infoIterator = devices.iterator();
        while (infoIterator.hasNext()) {
            if (infoIterator.next().meshAddress == meshAddress) {
                infoIterator.remove();
                return true;
            }
        }

        return false;
    }

    public boolean saveOrUpdate(Context context) {
        return FileSystem.writeAsObject(context, name + "." + password, this);
//        return FileSystem.writeAsObject("telink.meshs", this);
    }

    public boolean isOtaProcessing() {
        if (name == null || password == null || otaDevice == null ||
                TextUtils.isEmpty(otaDevice.mac) || TextUtils.isEmpty(otaDevice.meshName) || TextUtils.isEmpty(otaDevice.meshPwd)) {
            return false;
        }
        return name.equals(otaDevice.meshName) && password.equals(otaDevice.meshPwd);
    }
}
