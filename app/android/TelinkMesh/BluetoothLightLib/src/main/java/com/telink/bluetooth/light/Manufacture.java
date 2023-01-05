/********************************************************************************************************
 * @file     Manufacture.java 
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
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

/**
 * 厂商信息设置接口
 */
public final class Manufacture {

    private static final Manufacture DEFAULT_MANUFACTURE = new Builder().build();
    private static Manufacture definitionManufacture;

    private final Map<String, UUID> uuidMap = new HashMap<>();

    private String name;
    private String version;
    private String info;
    private String factoryName;
    private String factoryPassword;
    private byte[] factoryLtk;
    private int vendorId;
    private int otaDelay;
    private int otaSize;

    private Manufacture(String name, String version, String info, String defaultMeshName, String defaultPassword, byte[] defaultLongTermKey, int vendorId, int otaDelay, int otaSize, UUID serviceUUID, UUID pairUUID, UUID commandUUID, UUID notifyUUID, UUID otaUUID) {
        this.name = name;
        this.version = version;
        this.info = info;
        this.factoryName = defaultMeshName;
        this.factoryPassword = defaultPassword;
        this.factoryLtk = Arrays.copyOf(defaultLongTermKey, 16);
        this.vendorId = vendorId;
        this.otaDelay = otaDelay;
        this.otaSize = otaSize;

        this.putUUID(UUIDType.SERVICE.getKey(), serviceUUID);
        this.putUUID(UUIDType.PAIR.getKey(), pairUUID);
        this.putUUID(UUIDType.COMMAND.getKey(), commandUUID);
        this.putUUID(UUIDType.OTA.getKey(), otaUUID);
        this.putUUID(UUIDType.NOTIFY.getKey(), notifyUUID);
    }

    /**
     * 获取默认厂商,即Telink相关描述
     *
     * @return
     */
    public static Manufacture getDefaultManufacture() {
        return DEFAULT_MANUFACTURE;
    }

    /**
     * 获取自定义的厂商
     *
     * @return
     */
    public static Manufacture getDefinitionManufacture() {
        synchronized (Manufacture.class) {
            return definitionManufacture;
        }
    }

    /**
     * 设置自定义厂商
     *
     * @param manufacture
     */
    public static void setManufacture(Manufacture manufacture) {
        synchronized (Manufacture.class) {
            definitionManufacture = manufacture;
        }
    }

    /**
     * 获取当前的厂商,如果不设置自定义厂商则为默认厂商
     *
     * @return
     */
    public static Manufacture getDefault() {
        synchronized (Manufacture.class) {
            if (definitionManufacture == null)
                return DEFAULT_MANUFACTURE;
        }

        return definitionManufacture;
    }

    /**
     * 厂商名称
     *
     * @return
     */
    public String getName() {
        return name;
    }

    /**
     * 版本信息
     *
     * @return
     */
    public String getVersion() {
        return version;
    }

    /**
     * 描述信息
     *
     * @return
     */
    public String getInfo() {
        return info;
    }

    /**
     * 设备的出厂名
     *
     * @return
     */
    public String getFactoryName() {
        return factoryName;
    }

    /**
     * 设备的出厂密码
     *
     * @return
     */
    public String getFactoryPassword() {
        return factoryPassword;
    }

    /**
     * 设备的出厂LTK
     *
     * @return
     */
    public byte[] getFactoryLtk() {
        return factoryLtk;
    }

    /**
     * 厂商Id
     *
     * @return
     */
    public int getVendorId() {
        return vendorId;
    }

    /**
     * OTA数据包写入间隔时间
     * <p>可以根据不同的手机设置此参数.
     *
     * @return
     */
    public int getOtaDelay() {
        return otaDelay;
    }

    public int getOtaSize() {
        return otaSize;
    }

    public UUID getUUID(UUIDType uuidType) {
        return this.getUUID(uuidType.getKey());
    }

    public UUID getUUID(String key) {
        UUID result = null;

        synchronized (this.uuidMap) {
            if (this.uuidMap.containsKey(key))
                result = this.uuidMap.get(key);
        }

        return result;
    }

    public void putUUID(String key, UUID value) {
        synchronized (this.uuidMap) {
            if (!this.uuidMap.containsKey(key))
                this.uuidMap.put(key, value);
        }
    }

    public enum UUIDType {

        SERVICE("SERVICE_UUID"), PAIR("PAIR_UUID"), COMMAND("COMMAND_UUID"), OTA("OTA_UUID"), NOTIFY("NOTIFY_UUID");

        private final String key;

        UUIDType(String key) {
            this.key = key;
        }

        public String getKey() {
            return key;
        }
    }

    public static final class Builder {

        private String name = "telink";
        private String version = "1.0";
        private String info = "TELINK SEMICONDUCTOR (Shanghai) CO, LTD is a fabless IC design company";

        private String factoryName = "out_of_mesh";
        private String factoryPassword = "123";
        private byte[] factoryLtk = new byte[]{
                (byte) 0xC0, (byte) 0xC1, (byte) 0xC2, (byte) 0xC3, (byte) 0xC4,
                (byte) 0xC5, (byte) 0xC6, (byte) 0xC7, (byte) 0xD8, (byte) 0xD9,
                (byte) 0xDA, (byte) 0xDB, (byte) 0xDC, (byte) 0xDD, (byte) 0xDE,
                (byte) 0xDF};

        private int vendorId = 0x0211; // 0x1102 big end
        private UUID serviceUUID = UuidInformation.TELINK_SERVICE.getValue();
        private UUID pairUUID = UuidInformation.TELINK_CHARACTERISTIC_PAIR.getValue();
        private UUID commandUUID = UuidInformation.TELINK_CHARACTERISTIC_COMMAND.getValue();
        private UUID notifyUUID = UuidInformation.TELINK_CHARACTERISTIC_NOTIFY.getValue();
        private UUID otaUUID = UuidInformation.TELINK_CHARACTERISTIC_OTA.getValue();
        private int otaDelay = 100;
        private int otaSize = 128;

        public Builder() {
        }

        /**
         * 设置厂商名称
         *
         * @param name
         * @return
         */
        public Builder setName(String name) {
            this.name = name;
            return this;
        }

        /**
         * 设置版本信息
         *
         * @param version
         * @return
         */
        public Builder setVersion(String version) {
            this.version = version;
            return this;
        }

        /**
         * 设置厂商描述信息
         *
         * @param info
         * @return
         */
        public Builder setInfo(String info) {
            this.info = info;
            return this;
        }

        /**
         * 设置出厂名
         *
         * @param factoryName
         * @return
         */
        public Builder setFactoryName(String factoryName) {
            this.factoryName = factoryName;
            return this;
        }

        /**
         * 设置出厂密码
         *
         * @param factoryPassword
         * @return
         */
        public Builder setFactoryPassword(String factoryPassword) {
            this.factoryPassword = factoryPassword;
            return this;
        }

        /**
         * 设置出厂LTK
         *
         * @param factoryLtk
         * @return
         */
        public Builder setFactoryLtk(byte[] factoryLtk) {
            this.factoryLtk = factoryLtk;
            return this;
        }

        /**
         * 设置厂商标识
         *
         * @param vendorId
         * @return
         */
        public Builder setVendorId(int vendorId) {
            this.vendorId = vendorId;
            return this;
        }

        public Builder setOtaDelay(int otaDelay) {
            this.otaDelay = otaDelay;
            return this;
        }

        public Builder setOtaSize(int otaSize) {
            this.otaSize = otaSize;
            return this;
        }

        /**
         * 设置设备的ServiceUUID
         *
         * @param serviceUUID
         * @return
         */
        public Builder setServiceUUID(UUID serviceUUID) {
            this.serviceUUID = serviceUUID;
            return this;
        }

        /**
         * 设置配对用的UUID
         *
         * @param pairUUID
         * @return
         */
        public Builder setPairUUID(UUID pairUUID) {
            this.pairUUID = pairUUID;
            return this;
        }

        /**
         * 设置发送命令用的UUID
         *
         * @param commandUUID
         * @return
         */
        public Builder setCommandUUID(UUID commandUUID) {
            this.commandUUID = commandUUID;
            return this;
        }

        /**
         * 设置Notification用的UUID
         *
         * @param notifyUUID
         * @return
         */
        public Builder setNotifyUUID(UUID notifyUUID) {
            this.notifyUUID = notifyUUID;
            return this;
        }

        /**
         * 设置OTA用的UUID
         *
         * @param otaUUID
         * @return
         */
        public Builder setOtaUUID(UUID otaUUID) {
            this.otaUUID = otaUUID;
            return this;
        }

        public Manufacture build() {
            return new Manufacture(this.name, this.version, this.info, this.factoryName, this.factoryPassword, this.factoryLtk, this.vendorId, this.otaDelay, this.otaSize, this.serviceUUID, this.pairUUID, this.commandUUID, this.notifyUUID, this.otaUUID);
        }
    }
}
