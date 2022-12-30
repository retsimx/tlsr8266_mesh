/********************************************************************************************************
 * @file MainTestFragment.java
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
package com.telink.bluetooth.light.fragments;

import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.TextView;

import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.activity.TestInfoActivity;
import com.telink.bluetooth.light.widget.HexFormatTextWatcher;
import com.telink.util.Arrays;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.widget.Toolbar;

/**
 * 主页测试 fragment
 * Created by kee on 2018/1/8.
 */

public class TestFragment extends BaseFragment implements View.OnClickListener {

    private AlertDialog addDeviceDialog;

    private AlertDialog offlineDeviceDialog;

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_test, null);
    }

    @Override
    public void onViewCreated(View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        view.findViewById(R.id.view_add_device).setOnClickListener(this);
        view.findViewById(R.id.view_offline_device).setOnClickListener(this);
        view.findViewById(R.id.view_add_50_device).setOnClickListener(this);
        view.findViewById(R.id.view_offline_50_device).setOnClickListener(this);

        view.findViewById(R.id.iv_tip_add).setOnClickListener(this);
        view.findViewById(R.id.iv_tip_offline).setOnClickListener(this);
        view.findViewById(R.id.iv_tip_add_50).setOnClickListener(this);
        view.findViewById(R.id.iv_tip_offline_50).setOnClickListener(this);

        Toolbar toolbar = view.findViewById(R.id.title_bar);
        toolbar.inflateMenu(R.menu.test_info);
        toolbar.setNavigationIcon(null);
        toolbar.setOnMenuItemClickListener(new Toolbar.OnMenuItemClickListener() {
            @Override
            public boolean onMenuItemClick(MenuItem item) {
                if (item.getItemId() == R.id.item_test_info) {
                    startActivity(new Intent(getActivity(), TestInfoActivity.class));
                }
                return false;
            }
        });
        setTitle(view, "Test");
    }

    @Override
    public void onResume() {
        super.onResume();
    }

    private void showAddDialog() {
        if (addDeviceDialog == null) {
            final AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
            builder.setTitle("Add Device");
            View view = getActivity().getLayoutInflater().inflate(R.layout.dialog_add_device_test, null);
            builder.setView(view);
            final EditText et_params = view.findViewById(R.id.et_params);
            et_params.setText("80FF640081FF64000000");
            TextView tv_params_preview = view.findViewById(R.id.tv_params_preview);
            et_params.addTextChangedListener(new HexFormatTextWatcher(tv_params_preview));
            builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    byte[] params = getInputParams(et_params);
                    if (params != null) {
                        byte opcode = (byte) 0xDC;
                        int address = 0;
                        TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                                params);
                        dialog.dismiss();
                        showToast("add device complete");
                    }
                }
            });
            builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.dismiss();
                }
            });
            addDeviceDialog = builder.create();
        }
        addDeviceDialog.show();
    }

    private byte[] getInputParams(EditText editText) {
        String input = editText.getText().toString().trim();
        byte[] bytes = Arrays.hexToBytes(input);
        if (bytes == null || bytes.length == 0) {
            showToast("error : input params ");
            return null;
        }

        if (bytes.length > 10) {
            showToast("error : params length too long");
            return null;
        }
        return bytes;
    }

    private void showOfflineDialog() {
        if (offlineDeviceDialog == null) {
            final AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
            builder.setTitle("Offline Device");
            View view = getActivity().getLayoutInflater().inflate(R.layout.dialog_add_device_test, null);
            builder.setView(view);
            final EditText et_params = view.findViewById(R.id.et_params);
            et_params.setText("80000000810000000000");
            TextView tv_params_preview = view.findViewById(R.id.tv_params_preview);
            et_params.addTextChangedListener(new HexFormatTextWatcher(tv_params_preview));
            builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    byte[] params = getInputParams(et_params);
                    if (params != null) {
                        byte opcode = (byte) 0xDC;
//                        int address = 0xFFFF;
                        int address = 0;
                        TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                                params);
                        dialog.dismiss();
                        showToast("offline device complete");
                    }
                }
            });
            builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.dismiss();
                }
            });
            offlineDeviceDialog = builder.create();
        }
        offlineDeviceDialog.show();
    }

    /**
     * @param add add or offline
     */
    private void set50Devices(boolean add) {
        byte opcode = (byte) 0xDC;
//        int address = add ? 0 : 0xFFFF;
        int address = 0;

        int nodeAdr = 0x80;
        byte[] params;
        byte[] node0;
        byte[] node1;
        for (int i = 0; i < 25; i++) {
            node0 = assembleNode(nodeAdr++, add);
            node1 = assembleNode(nodeAdr++, add);
            params = new byte[10];
            System.arraycopy(node0, 0, params, 0, node0.length);
            System.arraycopy(node1, 0, params, node0.length, node1.length);
            TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                    params, true);
        }

        showToast((add ? "add" : "offline") + " 50 devices complete");
    }

    private byte[] assembleNode(int address, boolean add) {
        byte brightness = 0x64;
        byte status = add ? (byte) 0xFF : 0;
        byte reserve = 0;
        byte[] nodeInfo = new byte[4];
        nodeInfo[0] = (byte) address;
        nodeInfo[1] = status;
        nodeInfo[2] = brightness;
        nodeInfo[3] = reserve;
        return nodeInfo;
    }


    private void showConfirmDialog(final boolean add) {
        AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        builder.setTitle("Warning");
        builder.setMessage(add ? "Add 50 Devices?" : "Offline 50 devices?");
        builder.setPositiveButton("Confirm", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                set50Devices(add);
            }
        });

        builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });
        builder.show();
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.view_add_device:
                showAddDialog();
                break;

            case R.id.view_offline_device:
                showOfflineDialog();
                break;

            case R.id.view_add_50_device:
                showConfirmDialog(true);
                break;

            case R.id.view_offline_50_device:
                showConfirmDialog(false);
                break;


            case R.id.iv_tip_add:
                showToast("add device");
                break;
            case R.id.iv_tip_offline:
                showToast("offline device");
                break;
            case R.id.iv_tip_add_50:
                showToast("add 50 devices from 0x80");
                break;
            case R.id.iv_tip_offline_50:
                showToast("offline 50 devices from 0x80");
                break;
        }
    }

}
