/********************************************************************************************************
 * @file     ModelTestSettingActivity.java 
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
package com.telink.bluetooth.light.activity;

import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.adapter.BaseRecyclerViewAdapter;
import com.telink.bluetooth.light.fragments.TestModelListAdapter;
import com.telink.bluetooth.light.model.TestModel;
import com.telink.bluetooth.light.util.FileSystem;
import com.telink.util.Arrays;

import java.util.ArrayList;
import java.util.List;

import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

/**
 * Created by kee on 2018/1/11.
 */

public class ModelTestSettingActivity extends TelinkBaseActivity implements View.OnClickListener {


    TestModelListAdapter mAdapter;
    private List<TestModel> models;
    private TextView tv_model_title;
    private EditText et_op, et_vendor, et_params, et_address;
    private Button btn_send, btn_save;
    private static final String TEST_FILE_NAME = "TEST_MODELS";


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_model_test_setting);
        setTitle("Model Setting");
        enableBackNav(true);
        RecyclerView rv_models = (RecyclerView) findViewById(R.id.rv_models);
        tv_model_title = (TextView) findViewById(R.id.tv_model_title);
        et_op = (EditText) findViewById(R.id.et_op);
        et_vendor = (EditText) findViewById(R.id.et_vendor);
        et_params = (EditText) findViewById(R.id.et_params);
        et_address = (EditText) findViewById(R.id.et_address);
        btn_send = (Button) findViewById(R.id.btn_send);
        btn_save = (Button) findViewById(R.id.btn_save);
        btn_send.setOnClickListener(this);
        btn_save.setOnClickListener(this);


        getRealModels();

        mAdapter = new TestModelListAdapter(this, models);
        mAdapter.setSettingMode(true);


        rv_models.setLayoutManager(new LinearLayoutManager(this, LinearLayoutManager.HORIZONTAL, false));
        rv_models.setAdapter(mAdapter);

        mAdapter.setOnItemClickListener(new BaseRecyclerViewAdapter.OnItemClickListener() {
            @Override
            public void onItemClick(int position) {
                if (mAdapter.getSelectPosition() != position) {
                    mAdapter.setSelectPosition(position);
                    fillData();
                }
            }
        });


        et_params.addTextChangedListener(new FormatTextWatcher(et_params));

        et_op.addTextChangedListener(new FormatTextWatcher(et_op));

        et_vendor.addTextChangedListener(new FormatTextWatcher(et_vendor));

        et_address.addTextChangedListener(new FormatTextWatcher(et_address));
        fillData();
    }

    private void getRealModels() {
        List<TestModel> localModels = (List<TestModel>) FileSystem.readAsObject(this, TEST_FILE_NAME);
        models = new ArrayList<>();
        for (TestModel model : localModels) {
            if (!model.isHolder()) {
                models.add(model);
            }
        }
    }


    private void fillData() {
        TestModel model = models.get(mAdapter.getSelectPosition());

        tv_model_title.setText(model.getName());

        et_op.setText(Integer.toHexString(model.getOpCode() & 0xFF).toUpperCase());


        String vendor = String.format("%02X", ((byte) (model.getVendorId() & 0xFF))) + " " + String.format("%02X", (byte) (model.getVendorId() >> 8 & 0xFF));
        et_vendor.setText(vendor);


        String address = String.format("%02X", ((byte) (model.getAddress() & 0xFF))) + " " + String.format("%02X", (byte) (model.getAddress() >> 8 & 0xFF));
//        String address = Integer.toHexString((byte)(model.getAddress() & 0xFF)) + " " + Integer.toHexString((byte)(model.getAddress() >> 8 & 0xFF));
        et_address.setText(address);

        et_params.setText(Arrays.bytesToHexString(model.getParams(), " ").toUpperCase());
    }

    private boolean isValidInput(String input) {
        if (input.length() == 0) return true;
        if (input.charAt(0) == ' ') {
            return false;
        }

        char[] chars = input.toCharArray();
        for (int i = 0; i < chars.length; i++) {
            if ((i + 1) % 3 == 0) {
                if (chars[i] != ' ') {
                    return false;
                }
            } else if ((i + 1) % 3 != 0) {
                if (chars[i] == ' ') {
                    return false;
                }
            }
        }

        return true;
    }


    private String formatParamsInput(String input) {
        String hexString = input.replace(" ", "");

//        String[] hexBytes =
        int len = hexString.length() % 2 == 0 ? hexString.length() / 2 : hexString.length() / 2 + 1;

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < len; i++) {
            if (i == len - 1 && hexString.length() % 2 != 0) {
                sb.append(hexString.substring(i * 2, i * 2 + 1));
            } else {

                sb.append(hexString.substring(i * 2, i * 2 + 2));
                if (i != len - 1) {
                    sb.append(" ");
                }
            }
        }
        return sb.toString().toUpperCase();
    }

    private void dealInput(EditText et, String input, int before) {

        if (!isValidInput(input)) {
            int sel = et.getSelectionStart();
            String result = formatParamsInput(input);

            et.setText(result);

            int inputLen = input.length();
            int resultLen = result.length();

            // 添加
            if (before < input.length() && inputLen + 1 == resultLen) {
                et.setSelection(sel + 1);
                TelinkLog.w("onTextChanged: sel+1");
            } else { // 删除
                et.setSelection(sel);
            }

        }
    }

    private byte[] getInputBytes(String input) {
        String[] byteStrs = input.split(" ");
        byte[] result = new byte[byteStrs.length];

        for (int i = 0; i < byteStrs.length; i++) {
//            result[i] = Byte.parseByte(byteStrs[i], 16);
            result[i] = (byte) (Integer.parseInt(byteStrs[i], 16) & 0xFF);

        }

        return result;

    }


    private TestModel getModelByInput() {


        String opInput = et_op.getText().toString().trim();
        if (opInput.length() > 2) {
            showErrorMsg("opCode input error");
            return null;
        }

        String vendorIdInput = et_vendor.getText().toString().trim();
        if (vendorIdInput.length() != 5) {
            showErrorMsg("vendorId input error");
            return null;
        }

        String addressInput = et_address.getText().toString().trim();
        if (addressInput.length() != 5) {
            showErrorMsg("address input error");
            return null;
        }

        byte opCode = (byte) (getInputBytes(opInput)[0] & 0xFF);
        byte[] vendorBytes = getInputBytes(vendorIdInput);
        if (vendorBytes.length != 2) return null;
        int vendorId = (vendorBytes[0] & 0xFF) + ((vendorBytes[1] & 0xFF) << 8);

        byte[] addressBytes = getInputBytes(addressInput);
        int address = (addressBytes[0] & 0xFF) + ((addressBytes[1] & 0xFF) << 8);

        byte[] params = getInputBytes(et_params.getText().toString().trim());


        TestModel testModel = new TestModel();
        testModel.setId(models.get(mAdapter.getSelectPosition()).getId());
        testModel.setOpCode(opCode);
        testModel.setVendorId(vendorId);
        testModel.setAddress(address);
        testModel.setParams(params);
        return testModel;
    }

    private void sendModel(TestModel model) {
        if (model == null) return;
        TelinkLightService.Instance().sendVendorCommand(model.getOpCode(), model.getVendorId(), model.getAddress(), model.getParams());
        showErrorMsg("send success");
    }

    private void saveModel(TestModel model) {
        if (model == null) return;


        models.set(mAdapter.getSelectPosition(), model);

        saveLocalModel();

        showErrorMsg("save success");
    }

    private void saveLocalModel() {
        List<TestModel> localModels = (List<TestModel>) FileSystem.readAsObject(this, TEST_FILE_NAME);
        flag_outer:
        for (TestModel localModel : localModels) {
            for (TestModel realModel : models) {
                if (localModel.getId() ==realModel.getId()) {
                    localModel.setAddress(realModel.getAddress());
                    localModel.setParams(realModel.getParams());
                    localModel.setOpCode(realModel.getOpCode());
                    localModel.setVendorId(realModel.getVendorId());
                    continue flag_outer;
                }
            }
        }

        FileSystem.writeAsObject(this, TEST_FILE_NAME, localModels);
    }


    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.btn_send:

                sendModel(getModelByInput());

                break;

            case R.id.btn_save:
                saveModel(getModelByInput());
                break;

            default:
                break;
        }
    }


    public void showErrorMsg(String msg) {
        showToast(msg);
    }


    class FormatTextWatcher implements TextWatcher {
        EditText editText;

        FormatTextWatcher(EditText editText) {
            this.editText = editText;
        }

        @Override
        public void beforeTextChanged(CharSequence s, int start, int count, int after) {

        }

        @Override
        public void onTextChanged(CharSequence s, int start, int before, int count) {
            dealInput(editText, s.toString(), before);
        }

        @Override
        public void afterTextChanged(Editable s) {

        }
    }
}
