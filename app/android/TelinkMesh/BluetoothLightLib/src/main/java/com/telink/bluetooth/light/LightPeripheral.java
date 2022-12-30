/********************************************************************************************************
 * @file     LightPeripheral.java 
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
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.Context;

import com.telink.bluetooth.Peripheral;
import com.telink.util.Strings;

import java.nio.charset.Charset;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

public class LightPeripheral extends Peripheral {

    public static final String ADV_MESH_NAME = "com.telink.bluetooth.light.ADV_MESH_NAME";
    public static final String ADV_MESH_ADDRESS = "com.telink.bluetooth.light.ADV_MESH_ADDRESS";
    public static final String ADV_MESH_UUID = "com.telink.bluetooth.light.ADV_MESH_UUID";
    public static final String ADV_PRODUCT_UUID = "com.telink.bluetooth.light.ADV_PRODUCT_UUID";
    public static final String ADV_STATUS = "com.telink.bluetooth.light.ADV_STATUS";

    protected final Map<String, Object> advProperties = new HashMap<>();
    protected final Map<UUID, byte[]> characteristicsValue = new HashMap<>();

    public boolean meshChanged;

    private byte[] meshName;
    private byte[] password;
    private byte[] longTermKey;
    private int meshAddress;

    private Callback mCallback;

    private String meshNameStr;
    private int newMeshAddress = -1;
    private int retry = 0;

    public LightPeripheral(BluetoothDevice device, byte[] scanRecord, int rssi,
                           byte[] meshName, int meshAddress) {
        super(device, scanRecord, rssi);

        this.setMeshName(meshName);
        this.setMeshAddress(meshAddress);
    }

    public byte[] getMeshName() {
        return this.meshName;
    }

    public void setMeshName(String value) {
        this.meshNameStr = value;
    }

    public void setMeshName(byte[] value) {
        this.meshNameStr = Strings.bytesToString(value);
        this.meshName = value;
    }

    public String getMeshNameStr() {
        return this.meshNameStr;
    }

    public byte[] getPassword() {
        return password;
    }

    public void setPassword(byte[] password) {
        this.password = password;
    }

    public byte[] getLongTermKey() {
        return this.longTermKey;
    }

    public void setLongTermKey(byte[] value) {
        this.longTermKey = value;
    }

    public int getMeshAddress() {
        return meshAddress;
    }

    public void setMeshAddress(int value) {
        this.meshAddress = value;
        this.newMeshAddress = value;
    }

    public int getNewMeshAddress() {
        return newMeshAddress;
    }

    public void setNewMeshAddress(int newMeshAddress) {
        this.newMeshAddress = newMeshAddress;
    }

    public int getMeshUUID() {
        return this.getAdvPropertyAsInt(ADV_MESH_UUID);
    }

    public int getProductUUID() {
        return this.getAdvPropertyAsInt(ADV_PRODUCT_UUID);
    }

    public int getStatus() {
        return this.getAdvPropertyAsInt(ADV_STATUS);
    }

    public void putAdvProperty(String key, Object value) {
        this.advProperties.put(key, value);
    }

    public Object getAdvProperty(String key) {
        return this.advProperties.get(key);
    }

    public String getAdvPropertyAsString(String key) {
        return (String) this.advProperties.get(key);
    }

    public int getRetry() {
        return this.retry;
    }

    public void addRetry() {
        retry++;
    }

    public int getAdvPropertyAsInt(String key) {
        return (int) this.advProperties.get(key);
    }

    public long getAdvPropertyAsLong(String key) {
        return (long) this.advProperties.get(key);
    }

    public byte[] getAdvPropertyAsBytes(String key) {
        return (byte[]) this.advProperties.get(key);
    }

    public void putCharacteristicValue(UUID characteristicUUID, byte[] value) {
        this.characteristicsValue.put(characteristicUUID, value);
    }

    public byte[] getCharacteristicValue(UUID characteristicUUID) {
        if (this.characteristicsValue.containsKey(characteristicUUID))
            return this.characteristicsValue.get(characteristicUUID);
        return null;
    }

    public String getCharacteristicValueAsString(UUID characteristicUUID) {
        byte[] value = this.getCharacteristicValue(characteristicUUID);
        return value != null ? new String(value).trim() : null;
    }

    public String getFirmwareRevision() {
        UUID characteristicUUID = UuidInformation.CHARACTERISTIC_FIRMWARE.getValue();
        return this.getCharacteristicValueAsString(characteristicUUID);
    }

    public String getSubversion() {
        UUID characteristicUUID = UuidInformation.SUBVERSION_INFORMATION.getValue();
        return this.getCharacteristicValueAsString(characteristicUUID);
    }

    public void connect(Context context, Callback callback) {
        this.mCallback = callback;
        super.connect(context);
    }

    @Override
    public void disconnect() {
        super.disconnect();
    }

    public BluetoothGattService findService(UUID serviceUUID) {

        if (this.mServices == null || this.mServices.size() == 0)
            return null;

        BluetoothGattService mService = null;

        for (BluetoothGattService service : mServices) {
            if (service.getUuid().equals(serviceUUID)) {
                mService = service;
                break;
            }
        }

        return mService;
    }

    public BluetoothGattCharacteristic findCharacteristic(UUID serviceUUID, UUID characteristicUUID) {

        BluetoothGattService mService = this.findService(serviceUUID);

        if (mService == null)
            return null;

        List<BluetoothGattCharacteristic> characteristics = mService.getCharacteristics();
        BluetoothGattCharacteristic mCharacteristic = null;

        for (BluetoothGattCharacteristic characteristic : characteristics) {
            if (characteristic.getUuid().equals(characteristicUUID)) {
                mCharacteristic = characteristic;
                break;
            }
        }

        return mCharacteristic;
    }

    @Override
    protected void onConnect() {
        super.onConnect();

        if (this.mCallback != null)
            this.mCallback.onConnect(this);
    }

    @Override
    protected void onDisconnect() {
        super.onDisconnect();

        if (this.mCallback != null) {
            this.mCallback.onDisconnect(this);
//            this.mCallback = null;
        }
    }

    @Override
    protected void onServicesDiscovered(List<BluetoothGattService> services) {
        super.onServicesDiscovered(services);

        if (this.mCallback != null)
            this.mCallback.onServicesDiscovered(this, services);
    }

    @Override
    protected void onNotify(byte[] data, UUID serviceUUID,
                            UUID characteristicUUID, Object tag) {
        super.onNotify(data, serviceUUID, characteristicUUID, tag);

        if (this.mCallback != null)
            this.mCallback.onNotify(this, data, serviceUUID,
                    characteristicUUID, tag);
    }

    @Override
    protected void onRssiChanged() {
        super.onRssiChanged();

        if (this.mCallback != null)
            this.mCallback.onRssiChanged(this);
    }

    public interface Callback {
        void onConnect(LightPeripheral light);

        void onDisconnect(LightPeripheral light);

        void onServicesDiscovered(LightPeripheral light,
                                  List<BluetoothGattService> services);

        void onNotify(LightPeripheral light, byte[] data,
                      UUID serviceUUID, UUID characteristicUUID, Object tag);

        void onRssiChanged(LightPeripheral light);
    }
}
