/********************************************************************************************************
 * @file     Groups.java 
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

public class Groups extends DataStorageImpl<Group> {

    private static Groups mThis;

    private Groups() {
        super();
    }

    public static Groups getInstance() {

        if (mThis == null)
            mThis = new Groups();

        return mThis;
    }

    public boolean contains(int meshAddress) {
        return this.contains("meshAddress", meshAddress);
    }

    public Group getByMeshAddress(int meshAddress) {
        return this.get("meshAddress", meshAddress);
    }

}
