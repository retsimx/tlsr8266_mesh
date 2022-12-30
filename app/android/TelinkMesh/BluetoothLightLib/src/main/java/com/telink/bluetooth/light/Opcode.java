/********************************************************************************************************
 * @file     Opcode.java 
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


import androidx.annotation.Nullable;

public enum Opcode {

    /********************************************************************************
     * Mesh Pairing Opcode
     *******************************************************************************/

    BLE_GATT_OP_PAIR_REQ((byte) 0x01, ""),
    BLE_GATT_OP_PAIR_RSP((byte) 0x02, ""),
    BLE_GATT_OP_PAIR_REJECT((byte) 0x03, ""),
    BLE_GATT_OP_PAIR_NETWORK_NAME((byte) 0x04, ""),
    BLE_GATT_OP_PAIR_PASS((byte) 0x05, ""),
    BLE_GATT_OP_PAIR_LTK((byte) 0x06, ""),
    BLE_GATT_OP_PAIR_CONFIRM((byte) 0x07, ""),
    BLE_GATT_OP_PAIR_LTK_REQ((byte) 0x08, ""),
    BLE_GATT_OP_PAIR_LTK_RSP((byte) 0x09, ""),
    BLE_GATT_OP_PAIR_DELETE((byte) 0x0A, ""),
    BLE_GATT_OP_PAIR_DEL_RSP((byte) 0x0B, ""),
    BLE_GATT_OP_PAIR_ENC_REQ((byte) 0x0C, ""),
    BLE_GATT_OP_PAIR_ENC_RSP((byte) 0x0D, ""),
    BLE_GATT_OP_PAIR_ENC_FAIL((byte) 0x0E, ""),

    /********************************************************************************
     * Mesh Control Opcode
     *******************************************************************************/

    BLE_GATT_OP_CTRL_D0((byte) 0xD0, "on off"),
    BLE_GATT_OP_CTRL_D1((byte) 0xD1, "reserved"),
    BLE_GATT_OP_CTRL_D2((byte) 0xD2, "set brightness and color"),
    BLE_GATT_OP_CTRL_D3((byte) 0xD3, "reserved"),
    BLE_GATT_OP_CTRL_D4((byte) 0xD4, "response to short group id query"),
    BLE_GATT_OP_CTRL_D5((byte) 0xD5, "response to long group id query"),
    BLE_GATT_OP_CTRL_D6((byte) 0xD6, "response to long group id query"),
    BLE_GATT_OP_CTRL_D7((byte) 0xD7, "add group or remove group"),
    BLE_GATT_OP_CTRL_D8((byte) 0xD8, "reserved"),
    BLE_GATT_OP_CTRL_D9((byte) 0xD9, "reserved"),
    BLE_GATT_OP_CTRL_DA((byte) 0xDA, "status query"),
    BLE_GATT_OP_CTRL_DB((byte) 0xDB, "status response"),
    BLE_GATT_OP_CTRL_DC((byte) 0xDC, "online status report"),
    BLE_GATT_OP_CTRL_DD((byte) 0xDD, "group id query"),
    BLE_GATT_OP_CTRL_DE((byte) 0xDE, "reserved"),
    BLE_GATT_OP_CTRL_DF((byte) 0xDF, "reserved"),
    BLE_GATT_OP_CTRL_E0((byte) 0xE0, "set device address"),
    BLE_GATT_OP_CTRL_E1((byte) 0xE1, "notify the device address information"),
    BLE_GATT_OP_CTRL_E2((byte) 0xE2, "configure rgb value"),
    BLE_GATT_OP_CTRL_E3((byte) 0xE3, "kick out / reset factory"),
    BLE_GATT_OP_CTRL_E4((byte) 0xE4, "set device time"),
    BLE_GATT_OP_CTRL_E5((byte) 0xE5, "alarm operation opcode"),
    BLE_GATT_OP_CTRL_E6((byte) 0xE6, "get device alarm"),
    BLE_GATT_OP_CTRL_E7((byte) 0xE7, "alarm response"),
    BLE_GATT_OP_CTRL_E8((byte) 0xE8, "get device time"),
    BLE_GATT_OP_CTRL_E9((byte) 0xE9, "time response"),
    BLE_GATT_OP_CTRL_EA((byte) 0xEA, "user all"),
    BLE_GATT_OP_CTRL_EB((byte) 0xEB, "user all notify"),
    BLE_GATT_OP_CTRL_EE((byte) 0xEE, "scene operation opcode"),
    BLE_GATT_OP_CTRL_EF((byte) 0xEF, "load scene opcode"),
    BLE_GATT_OP_CTRL_C0((byte) 0xC0, "get scene opcode"),
    BLE_GATT_OP_CTRL_C1((byte) 0xC1, "scene response"),
    BLE_GATT_OP_CTRL_C8((byte) 0xC8, "get device state"), // @modifier ke

    BLE_GATT_OP_CTRL_CA((byte) 0xCA, "update device"),; // @modifier ke

    private final byte value;
    private final String info;

    Opcode(byte value, @Nullable String info) {
        this.value = value;
        this.info = info;
    }

    public static Opcode valueOf(byte value) {
        value = (byte) (value & 0xFF);
        byte opcodeValue;
        Opcode[] opcodes = Opcode.values();

        for (Opcode opcode : opcodes) {
            opcodeValue = opcode.getValue();
            if (opcodeValue == value)
                return opcode;
        }

        return null;
    }

    public byte getValue() {
        return (byte) (value & 0xFF);
    }

    public String getInfo() {
        return info;
    }
}
