/********************************************************************************************************
 * @file QRCodeDataOperator.java
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
package com.telink.bluetooth.light.activity.share;

import android.text.TextUtils;

import com.google.gson.Gson;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Mesh;
import com.telink.bluetooth.light.model.SharedPreferencesHelper;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by kee on 2017/12/27.
 */

public class QRCodeDataOperator {

    public static String provideStr() {
        Mesh mesh = TelinkLightApplication.getApp().getMesh();
        if (mesh == null) {
            return "{}";
        }
        TmpMesh tmpMesh = new TmpMesh();
        tmpMesh.n = mesh.name;
        tmpMesh.p = mesh.password;
        if (mesh.devices != null) {
            List<TmpDeviceInfo> deviceInfoList = new ArrayList<>();

            TmpDeviceInfo tmpDeviceInfo;
            for (Light deviceInfo : mesh.devices) {
                tmpDeviceInfo = new TmpDeviceInfo();
                tmpDeviceInfo.m = deviceInfo.macAddress;
                tmpDeviceInfo.a = deviceInfo.meshAddress;
                tmpDeviceInfo.v = deviceInfo.firmwareRevision;
                tmpDeviceInfo.pu = deviceInfo.productUUID;

                deviceInfoList.add(tmpDeviceInfo);
            }
            tmpMesh.d = deviceInfoList;
        }

        Gson gson = new Gson();
        return gson.toJson(tmpMesh);
        /*JSONObject jsonObject = new JSONObject();
        try {
            jsonObject.put("n", tmpMesh.n);
            jsonObject.put("p", tmpMesh.p);
//            jsonObject.put("d", tmpMesh.d);
            jsonObject.putOpt("d", tmpMesh.d);
            return jsonObject.toString();
        }catch (Exception e){
            return null;
        }*/
    }

    public static TmpMesh parseData(String data) {
        Gson gson = new Gson();
        TmpMesh tmpMesh = gson.fromJson(data, TmpMesh.class);
        if (tmpMesh != null && !TextUtils.isEmpty(tmpMesh.n) && !TextUtils.isEmpty(tmpMesh.p)) {
            return tmpMesh;
        }

        return null;
    }


    public static boolean importData(TmpMesh tmpMesh) {
        if (tmpMesh != null && !TextUtils.isEmpty(tmpMesh.n) && !TextUtils.isEmpty(tmpMesh.p)) {
            Mesh newMesh = new Mesh();
            Mesh oldMesh = TelinkLightApplication.getApp().getMesh();

            newMesh.name = tmpMesh.n;
            newMesh.password = tmpMesh.p;
            newMesh.factoryName = oldMesh.factoryName;
            newMesh.factoryPassword = oldMesh.factoryPassword;

            newMesh.devices = new ArrayList<>();
            if (tmpMesh.d != null) {
                Light deviceInfo;
                for (TmpDeviceInfo tmpDeviceInfo : tmpMesh.d) {
                    deviceInfo = new Light();
//                    deviceInfo.meshName = newMesh.name;
                    deviceInfo.deviceName = newMesh.name;
                    deviceInfo.macAddress = tmpDeviceInfo.m;
                    deviceInfo.meshAddress = tmpDeviceInfo.a;
                    deviceInfo.firmwareRevision = tmpDeviceInfo.v;
                    deviceInfo.productUUID = tmpDeviceInfo.pu;
                    newMesh.devices.add(deviceInfo);
                }
            }
            newMesh.saveOrUpdate(TelinkLightApplication.getApp());
            SharedPreferencesHelper.saveMeshName(TelinkLightApplication.getApp(), newMesh.name);
            SharedPreferencesHelper.saveMeshPassword(TelinkLightApplication.getApp(), newMesh.password);
            TelinkLightApplication.getApp().setupMesh(newMesh);
            return true;
        }

        return false;
    }

    public static class TmpMesh {
        String n;
        String p;
        List<TmpDeviceInfo> d;
    }

    public static class TmpDeviceInfo {
        String m;
        int a;
        String v;
        int pu;
    }
}
