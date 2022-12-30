/********************************************************************************************************
 * @file DeviceSettingActivity.java
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

import android.content.Intent;
import android.os.Bundle;
import android.text.TextUtils;
import android.view.MenuItem;
import android.view.Window;

import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.fragments.DeviceSettingFragment;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Lights;

import androidx.appcompat.widget.Toolbar;

public final class DeviceSettingActivity extends TelinkBaseActivity {
    private DeviceSettingFragment settingFragment;

    private int meshAddress;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);

        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        this.setContentView(R.layout.activity_device_setting);

        enableBackNav(true);
        setTitle("Light Setting");
        Toolbar toolbar = findViewById(R.id.title_bar);
        toolbar.inflateMenu(R.menu.device_setting);
        toolbar.setOnMenuItemClickListener(new Toolbar.OnMenuItemClickListener() {
            @Override
            public boolean onMenuItemClick(MenuItem item) {
                if (item.getItemId() == R.id.item_grouping) {
                    Light light = Lights.getInstance().getByMeshAddress(meshAddress);
                    if (light == null || TextUtils.isEmpty(light.macAddress)) {
                        showToast("error! Lack of mac");
                        return false;
                    }
                    Intent intent = new Intent(DeviceSettingActivity.this,
                            DeviceGroupingActivity.class);
                    intent.putExtra("meshAddress", meshAddress);
                    startActivity(intent);
                }
                return false;
            }
        });
        this.meshAddress = this.getIntent().getIntExtra("meshAddress", 0);
        this.settingFragment = (DeviceSettingFragment) this
                .getFragmentManager().findFragmentById(
                        R.id.device_setting_fragment);
        this.settingFragment.meshAddress = meshAddress;
    }
}
