/********************************************************************************************************
 * @file     DefaultAdvertiseDataFilter.java 
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

import android.bluetooth.BluetoothDevice;

import com.telink.bluetooth.TelinkLog;
import com.telink.util.Arrays;

/**
 * 默认的广播过滤器
 * <p>根据VendorId识别设备.
 */
public final class DefaultAdvertiseDataFilter implements AdvertiseDataFilter<LightPeripheral> {

    private DefaultAdvertiseDataFilter() {
    }

    public static DefaultAdvertiseDataFilter create() {
        return new DefaultAdvertiseDataFilter();
    }

    @Override
    public LightPeripheral filter(BluetoothDevice device, int rssi, byte[] scanRecord) {

        TelinkLog.d(device.getName() + "-->" + Arrays.bytesToHexString(scanRecord, ":"));

        int length = scanRecord.length;
        int packetPosition = 0;
        int packetContentLength;
        int packetSize;
        int position;
        int type;
        byte[] meshName = null;

        int rspData = 0;

        while (packetPosition < length) {

            packetSize = scanRecord[packetPosition];

            if (packetSize == 0)
                break;

            position = packetPosition + 1;
            type = scanRecord[position] & 0xFF;
            position++;

            if (type == 0x09) {

                packetContentLength = packetSize - 1;

                if (packetContentLength > 16 || packetContentLength <= 0)
                    return null;

                meshName = new byte[16];
                System.arraycopy(scanRecord, position, meshName, 0, packetContentLength);
            } else if (type == 0xFF) {

                rspData++;

                if (rspData == 2) {

                    int vendorId = ((scanRecord[position++] & 0xFF) ) + ((scanRecord[position++] & 0xFF )<< 8);

                    if (vendorId != Manufacture.getDefault().getVendorId())
                        return null;

                    int meshUUID = (scanRecord[position++] & 0xFF) + ((scanRecord[position++] & 0xFF) << 8);
                    position += 4;
                    int productUUID = (scanRecord[position++] & 0xFF) + ((scanRecord[position++] & 0xFF) << 8);
                    int status = scanRecord[position++] & 0xFF;
                    int meshAddress = (scanRecord[position++] & 0xFF) + ((scanRecord[position] & 0xFF) << 8);

                    LightPeripheral light = new LightPeripheral(device, scanRecord, rssi, meshName, meshAddress);
                    light.putAdvProperty(LightPeripheral.ADV_MESH_NAME, meshName);
                    light.putAdvProperty(LightPeripheral.ADV_MESH_ADDRESS, meshAddress);
                    light.putAdvProperty(LightPeripheral.ADV_MESH_UUID, meshUUID);
                    light.putAdvProperty(LightPeripheral.ADV_PRODUCT_UUID, productUUID);
                    light.putAdvProperty(LightPeripheral.ADV_STATUS, status);

                    return light;
                }
            }

            packetPosition += packetSize + 1;
        }

        return null;
    }
}
