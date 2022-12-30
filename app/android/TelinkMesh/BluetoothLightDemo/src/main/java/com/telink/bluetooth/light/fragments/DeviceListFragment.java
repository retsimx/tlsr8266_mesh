/********************************************************************************************************
 * @file DeviceListFragment.java
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

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.text.TextUtils;
import android.view.LayoutInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.view.inputmethod.InputMethodManager;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.AdapterView.OnItemLongClickListener;
import android.widget.BaseAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.GridView;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.telink.bluetooth.light.ConnectionStatus;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.activity.DeviceMeshScanningActivity;
import com.telink.bluetooth.light.activity.DeviceScanningActivity;
import com.telink.bluetooth.light.activity.DeviceSettingActivity;
import com.telink.bluetooth.light.activity.LogActivity;
import com.telink.bluetooth.light.activity.MeshOTAActivity;
import com.telink.bluetooth.light.activity.MeshSettingsActivity;
import com.telink.bluetooth.light.activity.OnlineStatusTestActivity;
import com.telink.bluetooth.light.activity.ScanningTestActivity;
import com.telink.bluetooth.light.activity.TempTestActivity;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Lights;
import com.telink.bluetooth.light.model.Mesh;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.widget.Toolbar;

public final class DeviceListFragment extends BaseFragment implements OnClickListener {

    private static final String TAG = DeviceListFragment.class.getSimpleName();
    private static final int UPDATE = 1;
    private LayoutInflater inflater;
    private DeviceListAdapter adapter;

    private Button btnAllOn;
    private Button btnAllOff;
    private Button btnOta;

    private Activity mContext;

    private EditText txtSendInterval;
    private EditText txtSendNumbers;
    private TextView txtNotifyCount;
    private TextView log;

    // interval on off test
    private EditText et_adr, et_interval;
    private Button btn_start_test;
    private Handler mIntervalHandler;
    private boolean testStarted;
    private long interval;
    private int address;
    private boolean onOff = false;
    private TextView tv_test_count;
    private int testCount;

    private Button btn_online_status, btn_scan_test;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        this.inflater = inflater;

        View view = inflater.inflate(R.layout.fragment_device_list, null);

        GridView listView = (GridView) view.findViewById(R.id.list_devices);

        listView.setOnItemClickListener(this.itemClickListener);
        listView.setOnItemLongClickListener(this.itemLongClickListener);
        listView.setAdapter(this.adapter);

        this.btnAllOn = (Button) view.findViewById(R.id.btn_on);
        this.btnAllOn.setOnClickListener(this);

        this.btnAllOff = (Button) view.findViewById(R.id.btn_off);
        this.btnAllOff.setOnClickListener(this);

        this.btnOta = (Button) view.findViewById(R.id.btn_ota);
        this.btnOta.setOnClickListener(this);

        this.txtSendInterval = (EditText) view.findViewById(R.id.sendInterval);
        this.txtSendNumbers = (EditText) view.findViewById(R.id.sendNumbers);
        this.txtNotifyCount = (TextView) view.findViewById(R.id.notifyCount);
        this.log = (TextView) view.findViewById(R.id.log);
        this.log.setOnClickListener(this);
        view.findViewById(R.id.userAll).setOnClickListener(this);

        et_adr = (EditText) view.findViewById(R.id.et_adr);
        et_interval = (EditText) view.findViewById(R.id.et_interval);
        btn_start_test = (Button) view.findViewById(R.id.btn_start_test);
        btn_start_test.setOnClickListener(this);
        btn_online_status = (Button) view.findViewById(R.id.online_status);
        btn_online_status.setOnClickListener(this);

        btn_scan_test = view.findViewById(R.id.btn_scan_test);
        btn_scan_test.setOnClickListener(this);

        tv_test_count = (TextView) view.findViewById(R.id.tv_test_count);

        return view;
    }

    private boolean isMeshScan = false;

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        setTitle(view, "Lights");
        Toolbar toolbar = view.findViewById(R.id.title_bar);
        toolbar.setNavigationIcon(R.drawable.ic_setting);
        toolbar.setNavigationOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(mContext, MeshSettingsActivity.class);
                startActivity(intent);
            }
        });
        toolbar.inflateMenu(R.menu.devices);
        toolbar.setOnMenuItemClickListener(new Toolbar.OnMenuItemClickListener() {
            @Override
            public boolean onMenuItemClick(MenuItem item) {
                switch (item.getItemId()) {
                    case R.id.item_scan_mode:
                        isMeshScan = !isMeshScan;
                        item.setTitle(isMeshScan ? "mesh" : "gatt");
                        break;

                    case R.id.item_add:
                        Mesh mesh = TelinkLightApplication.getApp().getMesh();
                        if (TextUtils.isEmpty(mesh.factoryName) || TextUtils.isEmpty(mesh.factoryPassword)) {
                            Toast.makeText(mContext, "pls set mesh factory info!", Toast.LENGTH_SHORT).show();
                            return false;
                        }

                        Intent intent;
                        if (isMeshScan) {
                            intent = new Intent(mContext, DeviceMeshScanningActivity.class);
                        } else {
                            intent = new Intent(mContext, DeviceScanningActivity.class);
                        }
                        startActivity(intent);
                        break;

                }
                return false;
            }
        });
    }

    private void startIntervalTest() {
        try {
            interval = Long.parseLong(et_interval.getText().toString().trim());
            address = Integer.parseInt(et_adr.getText().toString(), 16);
            testStarted = true;
            testCount = 0;
            btn_start_test.setText("stop");
            tv_test_count.setText(testCount + "");
            mIntervalHandler.removeCallbacksAndMessages(null);
            mIntervalHandler.post(intervalTask);

        } catch (Exception e) {
            Toast.makeText(mContext, "input error", Toast.LENGTH_SHORT).show();
        }
    }

    private void stopIntervalTest() {
        testStarted = false;
        btn_start_test.setText("start");
        mIntervalHandler.removeCallbacksAndMessages(null);
    }

    private Runnable intervalTask = new Runnable() {
        @Override
        public void run() {
            if (!testStarted) return;
            if (onOff) {
                byte opcode = (byte) 0xD0;
//                int address = 0xFFFF;
                byte[] params = new byte[]{0x01, 0x00, 0x00};
                TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                        params);
            } else {
                byte opcode = (byte) 0xD0;
//                int address = 0xFFFF;
                byte[] params = new byte[]{0x00, 0x00, 0x00};
                TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                        params);
            }
            testCount++;
            tv_test_count.setText(testCount + "");
            onOff = !onOff;
            mIntervalHandler.removeCallbacks(this);
            mIntervalHandler.postDelayed(this, interval);
        }
    };

    private OnItemClickListener itemClickListener = new OnItemClickListener() {

        @Override
        public void onItemClick(AdapterView<?> parent, View view, int position,
                                long id) {

            Light light = adapter.getItem(position);

            if (light.connectionStatus == ConnectionStatus.OFFLINE)
                return;

            int dstAddr = light.meshAddress;

            byte opcode = (byte) 0xD0;


            if (light.connectionStatus == ConnectionStatus.OFF) {


                TelinkLightService.Instance().sendCommandNoResponse(opcode, dstAddr, new byte[]{0x01, 0x00, 0x00});
            } else if (light.connectionStatus == ConnectionStatus.ON) {


                TelinkLightService.Instance().sendCommandNoResponse(opcode, dstAddr, new byte[]{0x00, 0x00, 0x00});
            }
        }
    };

    private OnItemLongClickListener itemLongClickListener = new OnItemLongClickListener() {

        @Override
        public boolean onItemLongClick(AdapterView<?> parent, View view,
                                       int position, long id) {

            Intent intent = new Intent(getActivity(),
                    DeviceSettingActivity.class);
            Light light = adapter.getItem(position);

            intent.putExtra("meshAddress", light.meshAddress);
            startActivity(intent);
            return true;
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.mContext = this.getActivity();
        this.adapter = new DeviceListAdapter();
        mIntervalHandler = new Handler();
        onOff = false;
        testStarted = false;
        testCount = 0;
    }

    @Override
    public void onResume() {
        super.onResume();
        notifyDataSetChanged();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        mIntervalHandler.removeCallbacksAndMessages(null);
    }


    public void addDevice(Light light) {
        this.adapter.add(light);
    }

    public Light getDevice(int meshAddress) {
        return this.adapter.get(meshAddress);
    }

    public void notifyDataSetChanged() {
        if (this.adapter != null)
            this.adapter.notifyDataSetChanged();
    }

    @Override
    public void onClick(View v) {

        if (v == btnAllOn) {
//            startScan();
            byte opcode = (byte) 0xD0;
            int address = 0xFFFF;
            byte[] params = new byte[]{0x01, 0x00, 0x00};
            TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                    params);
        } else if (v == btnAllOff) {
            byte opcode = (byte) 0xD0;
            int address = 0xFFFF;
            byte[] params = new byte[]{0x00, 0x00, 0x00};
            TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                    params);
        }else if (v == btnOta) {

//            MeshCommandUtil.getDeviceOTAState();
//            TelinkLightService.Instance().updateNotification();
            Intent intent = new Intent(mContext, MeshOTAActivity.class);
            startActivity(intent);
            /*List<Light> lights = Lights.getInstance().get();
            for (Light light : lights) {
                if (light.connectionStatus != ConnectionStatus.OFFLINE) {
                    Intent intent = new Intent(mContext, MeshOTAActivity.class);
                    startActivity(intent);
                    return;
                }
            }
            Toast.makeText(getActivity(), "No Device Online!", Toast.LENGTH_SHORT).show();*/
        } else if (v == log) {
            startActivity(new Intent(getActivity(), LogActivity.class));
        } else if (v.getId() == R.id.userAll) {
//            startActivity(new Intent(getActivity(), UserAllActivity.class));
            startActivity(new Intent(getActivity(), TempTestActivity.class));
        } else if (v == btn_start_test) {
            if (!testStarted) {
                startIntervalTest();
            } else {
                stopIntervalTest();
            }
        } else if (v == btn_online_status) {
            startActivity(new Intent(getActivity(), OnlineStatusTestActivity.class));
        } else if (v == btn_scan_test) {
            startActivity(new Intent(getActivity(), ScanningTestActivity.class));
        }
    }

    private static class DeviceItemHolder {
        public ImageView statusIcon;
        public TextView txtName;
    }

    final class DeviceListAdapter extends BaseAdapter {

        public DeviceListAdapter() {

        }

        @Override
        public int getCount() {
            return Lights.getInstance().size();
        }

        @Override
        public Light getItem(int position) {
            return Lights.getInstance().get(position);
        }

        @Override
        public long getItemId(int position) {
            return 0;
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {

            DeviceItemHolder holder;

            if (convertView == null) {

                convertView = inflater.inflate(R.layout.device_item, null);

                ImageView statusIcon = (ImageView) convertView
                        .findViewById(R.id.img_icon);
                TextView txtName = (TextView) convertView
                        .findViewById(R.id.txt_name);

                holder = new DeviceItemHolder();

                holder.statusIcon = statusIcon;
                holder.txtName = txtName;

                convertView.setTag(holder);
            } else {
                holder = (DeviceItemHolder) convertView.getTag();
            }

            Light light = this.getItem(position);

            holder.txtName.setText(light.getLabel());
            holder.txtName.setTextColor(getResources().getColor(light.textColor));

            if (light.connectionStatus == ConnectionStatus.OFFLINE) {
                holder.statusIcon.setImageResource(R.drawable.icon_light_offline);
            } else if (light.connectionStatus == ConnectionStatus.OFF) {
                holder.statusIcon.setImageResource(R.drawable.icon_light_off);
            } else if (light.connectionStatus == ConnectionStatus.ON) {
                holder.statusIcon.setImageResource(R.drawable.icon_light_on);
            }

            return convertView;
        }

        public void add(Light light) {
            Lights.getInstance().add(light);
        }

        public Light get(int meshAddress) {
            return Lights.getInstance().getByMeshAddress(meshAddress);
        }
    }

}
