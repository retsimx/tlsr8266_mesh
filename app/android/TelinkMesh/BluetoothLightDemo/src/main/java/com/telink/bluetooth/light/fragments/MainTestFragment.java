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

import android.content.Intent;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.activity.MainActivity;
import com.telink.bluetooth.light.activity.ModelTestSettingActivity;
import com.telink.bluetooth.light.adapter.BaseRecyclerViewAdapter;
import com.telink.bluetooth.light.model.TestModel;
import com.telink.bluetooth.light.util.FileSystem;

import java.util.ArrayList;
import java.util.List;

import androidx.annotation.Nullable;
import androidx.appcompat.widget.Toolbar;
import androidx.recyclerview.widget.GridLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

/**
 * 主页测试 fragment
 * Created by kee on 2018/1/8.
 */

public class MainTestFragment extends BaseFragment implements View.OnClickListener {

    TestModelListAdapter mAdapter;
    private List<TestModel> models;
    private TextView tv_model_setting;
    private static final String TEST_FILE_NAME = "TEST_MODELS";

    @Nullable
    @Override
    public View onCreateView(LayoutInflater inflater, @Nullable ViewGroup container, Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_main_test, null);
    }


    @Override
    public void onViewCreated(View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        setTitle(view, "Model Test");
        Toolbar toolbar = view.findViewById(R.id.title_bar);
        toolbar.setNavigationIcon(null);

        RecyclerView rv_models = (RecyclerView) view.findViewById(R.id.rv_test_models);

//        tv_model_setting = (TextView) view.findViewById(R.id.tv_model_setting);
        tv_model_setting.setOnClickListener(this);


        if (FileSystem.exists(getActivity(), TEST_FILE_NAME)) {
            models = (List<TestModel>) FileSystem.readAsObject(getActivity(), TEST_FILE_NAME);
        } else {
            models = new ArrayList<>();

            String[] names = getResources().getStringArray(R.array.model_names);

            // cnt: 12
            byte[][] params = new byte[][]{
                    {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

                    {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // holder

                    {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // holder

                    {0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04},

                    {0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

                    {0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

                    {0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

                    {0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04},

                    {0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

                    {0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04},


                    {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // holder

                    {0x00, 0x00, 0x00, (byte) 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04},
            };
            TestModel model;
            for (int i = 0; i < 12; i++) {
                model = new TestModel();
                model.setId(i);
                model.setName(names[i]);
                model.setOpCode((byte) 0xCA);
                model.setVendorId(0x0211);
                model.setAddress(0xFFFF);
                if (i == 1 || i == 2 || i == 10) {
                    model.setHolder(true);
                } else {
                    model.setHolder(false);
                }
//            model.setParams(new byte[]{0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11});
                model.setParams(params[i]);
                models.add(model);
            }

            FileSystem.writeAsObject(getActivity(), TEST_FILE_NAME, models);
        }


        mAdapter = new TestModelListAdapter(getActivity(), models);
        mAdapter.setSettingMode(false);
//        rv_models.setLayoutManager(new LinearLayoutManager(getActivity(), LinearLayoutManager.HORIZONTAL, false));
        rv_models.setLayoutManager(new GridLayoutManager(getActivity(), 3));
        rv_models.setAdapter(mAdapter);

        mAdapter.setOnItemClickListener(new BaseRecyclerViewAdapter.OnItemClickListener() {
            @Override
            public void onItemClick(int position) {
                sendModel(models.get(position));
            }
        });
    }

    @Override
    public void onResume() {
        super.onResume();
        models = (List<TestModel>) FileSystem.readAsObject(getActivity(), TEST_FILE_NAME);
        mAdapter.notifyDataSetChanged();
    }

    private void sendModel(TestModel model) {
        if (model == null) return;

        boolean sendResult = TelinkLightService.Instance().sendVendorCommand(model.getOpCode(), model.getVendorId(), model.getAddress(), model.getParams());
        if (sendResult) {
            showErrorMsg("send success");
        } else {
            showErrorMsg("send fail");
        }
    }


    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            /*case R.id.tv_model_setting:
                startActivity(new Intent(getActivity(), ModelTestSettingActivity.class));
                break;*/
        }
    }


    public void showErrorMsg(String msg) {
        ((MainActivity) getActivity()).showToast(msg);
    }

}
