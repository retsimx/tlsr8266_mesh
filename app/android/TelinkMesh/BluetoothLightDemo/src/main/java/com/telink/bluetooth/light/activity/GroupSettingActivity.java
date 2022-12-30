/********************************************************************************************************
 * @file GroupSettingActivity.java
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
package com.telink.bluetooth.light.activity;

import android.os.Bundle;
import android.view.Window;

import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.fragments.GroupSettingFragment;

public final class GroupSettingActivity extends TelinkBaseActivity {

    private GroupSettingFragment settingFragment;

    private int groupAddress;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);

        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        this.setContentView(R.layout.activity_group_setting);
        enableBackNav(true);
        setTitle("Group Setting");
        this.groupAddress = this.getIntent().getIntExtra("groupAddress", 0);

        this.settingFragment = (GroupSettingFragment) this.getSupportFragmentManager()
                .findFragmentById(R.id.group_setting_fragment);

        this.settingFragment.groupAddress = groupAddress;
    }

}
