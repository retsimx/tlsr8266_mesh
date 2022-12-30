/********************************************************************************************************
 * @file     TelinkMeshErrorDealActivity.java 
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
package com.telink.bluetooth.light;

import android.annotation.SuppressLint;
import android.app.Activity;
import androidx.appcompat.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.provider.Settings;
import android.view.View;
import android.widget.Toast;

import com.telink.bluetooth.LeBluetooth;
import com.telink.bluetooth.event.MeshEvent;
import com.telink.util.ContextUtil;
import com.telink.util.Event;
import com.telink.util.EventListener;


// 添加 扫描过程中出现的因定位未开启而导致的扫描不成功问题
public abstract class TelinkMeshErrorDealActivity extends TelinkBaseActivity implements EventListener<String> {

    protected final static int ACTIVITY_REQUEST_CODE_LOCATION = 0x11;
    private AlertDialog mErrorDialog;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        ((TelinkLightApplication) getApplication()).addEventListener(MeshEvent.ERROR, this);
    }

    @Override
    public void performed(Event<String> event) {
        switch (event.getType()) {
            case MeshEvent.ERROR:
                onMeshError((MeshEvent) event);
                break;
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        ((TelinkLightApplication) getApplication()).removeEventListener(this);
    }

    private void dismissDialog() {

        if (mErrorDialog != null && mErrorDialog.isShowing()) {
            mErrorDialog.dismiss();
        }
    }

    protected void onMeshError(MeshEvent event) {
        if (event.getArgs() == LeBluetooth.SCAN_FAILED_LOCATION_DISABLE) {
            if (mErrorDialog == null) {
                TelinkLightService.Instance().idleMode(true);
                AlertDialog.Builder dialogBuilder = new AlertDialog.Builder(this);
                dialogBuilder.setTitle("Error")
                        .setMessage("Location not enabled, bluetooth searching may require this")
                        .setNegativeButton("Ignore", null)
                        .setPositiveButton("Go Settings", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialogInterface, int i) {
                                Intent enableLocationIntent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
                                startActivityForResult(enableLocationIntent, ACTIVITY_REQUEST_CODE_LOCATION);
                            }
                        });
                mErrorDialog = dialogBuilder.create();
            }
            mErrorDialog.show();
        } else {
            new AlertDialog.Builder(this).setMessage("Bluetooth ERR").show();
        }
    }

    protected abstract void onLocationEnable();

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == ACTIVITY_REQUEST_CODE_LOCATION) {
            if (ContextUtil.isLocationEnable(this)) {
                dismissDialog();
                onLocationEnable();
            }
        }
    }
}
