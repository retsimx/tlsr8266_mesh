/********************************************************************************************************
 * @file     JsonTest.java 
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
package com.telink.bluetooth.light.util;

import com.google.gson.Gson;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by kee on 2018/8/23.
 */

public class JsonTest {

    public static void main(String[] args) {
        Gson gson = new Gson();
        Mesh mesh = new Mesh();
        mesh.address = 1;
        mesh.name = "telink_mesh1";
        mesh.meshKey = new byte[]{12, 23};
//        mesh.groups = new ArrayList<>();
        String jsonResult = gson.toJson(mesh);
        System.out.print(jsonResult);


        Mesh mesh1 = gson.fromJson(jsonResult, Mesh.class);
    }


    public static class Mesh implements Serializable {
        public String name;
        public String pwd;
        public byte[] meshKey;
        public int address;
        public List<String> groups;
    }
}
