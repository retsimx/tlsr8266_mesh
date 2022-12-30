/********************************************************************************************************
 * @file DeviceGroupingActivity.java
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

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.res.ColorStateList;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.LayoutInflater;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.view.Window;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.BaseAdapter;
import android.widget.GridView;
import android.widget.ImageView;
import android.widget.TextView;

import com.telink.bluetooth.event.NotificationEvent;
import com.telink.bluetooth.light.NotificationInfo;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.model.Group;
import com.telink.bluetooth.light.model.Groups;
import com.telink.util.Event;
import com.telink.util.EventListener;

public final class DeviceGroupingActivity extends TelinkBaseActivity implements EventListener {

    private final static int UPDATE = 1;

    private LayoutInflater inflater;
    private GroupListAdapter adapter;

    private int meshAddress;

    private OnItemClickListener itemClickListener = new OnItemClickListener() {

        @Override
        public void onItemClick(AdapterView<?> parent, View view, int position,
                                long id) {
            Group group = adapter.getItem(position);
            allocDeviceGroup(group);

        }
    };

    @SuppressLint("HandlerLeak")
    private Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);

            switch (msg.what) {
                case UPDATE:
                    adapter.notifyDataSetChanged();
                    break;
            }
        }
    };

    private TelinkLightApplication mApplication;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        this.mApplication = (TelinkLightApplication) this.getApplication();
        this.mApplication.addEventListener(NotificationEvent.GET_GROUP, this);

        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        this.setContentView(R.layout.activity_device_grouping);
        enableBackNav(true);
        setTitle("Grouping");
        this.meshAddress = this.getIntent().getIntExtra("meshAddress", 0);

        this.inflater = this.getLayoutInflater();
        this.adapter = new GroupListAdapter();

        GridView listView = (GridView) this.findViewById(R.id.list_groups);
        listView.setOnItemClickListener(this.itemClickListener);
        listView.setAdapter(this.adapter);

        this.testData();
        this.getDeviceGroup();
    }

    private void testData() {

        Groups.getInstance().clear();

        Group living = new Group();
        living.name = "Living Room";
        living.meshAddress = 0x8001;
        living.brightness = 100;
        living.temperature = 100;
        living.color = 0xFFFFFF;

        Group family = new Group();
        family.name = "Family Room";
        family.meshAddress = 0x8002;
        family.brightness = 100;
        family.temperature = 100;
        family.color = 0xFFFFFF;

        Group kitchen = new Group();
        kitchen.name = "Kitchen";
        kitchen.meshAddress = 0x8003;
        kitchen.brightness = 100;
        kitchen.temperature = 100;
        kitchen.color = 0xFFFFFF;

        Group bedroom = new Group();
        bedroom.name = "Bedroom";
        bedroom.meshAddress = 0x8004;
        bedroom.brightness = 100;
        bedroom.temperature = 100;
        bedroom.color = 0xFFFFFF;

        Groups.getInstance().add(living);
        Groups.getInstance().add(family);
        Groups.getInstance().add(kitchen);
        Groups.getInstance().add(bedroom);

        this.adapter.notifyDataSetChanged();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        this.mApplication.removeEventListener(this);
    }

    private void getDeviceGroup() {
        byte opcode = (byte) 0xDD;
        int dstAddress = this.meshAddress;
        byte[] params = new byte[]{0x08, 0x01};

        TelinkLightService.Instance().sendCommandNoResponse(opcode, dstAddress, params);
//        TelinkLightService.Instance().updateNotification();
    }

    private void allocDeviceGroup(Group group) {

        int groupAddress = group.meshAddress;
        int dstAddress = this.meshAddress;
        byte opcode = (byte) 0xD7;
        byte[] params = new byte[]{0x01, (byte) (groupAddress & 0xFF),
                (byte) (groupAddress >> 8 & 0xFF)};

        if (!group.checked) {
            params[0] = 0x01;
            TelinkLightService.Instance().sendCommandNoResponse(opcode, dstAddress, params);

        } else {
            params[0] = 0x00;
            TelinkLightService.Instance().sendCommandNoResponse(opcode, dstAddress, params);
        }
    }

    @Override
    public void performed(Event event) {

        if (event.getType() == NotificationEvent.GET_GROUP) {
            NotificationEvent e = (NotificationEvent) event;
            NotificationInfo info = e.getArgs();

            int srcAddress = info.src & 0xFF;
            byte[] params = info.params;

            if (srcAddress != this.meshAddress)
                return;

            int count = this.adapter.getCount();

            Group group;

            for (int i = 0; i < count; i++) {
                group = this.adapter.getItem(i);

                if (group != null)
                    group.checked = false;
            }

            int groupAddress;
            int len = params.length;

            for (int j = 0; j < len; j++) {

                groupAddress = params[j];

                if (groupAddress == 0x00 || groupAddress == 0xFF)
                    break;

                groupAddress = groupAddress | 0x8000;

                group = this.adapter.get(groupAddress);

                if (group != null) {
                    group.checked = true;
                }
            }

            mHandler.obtainMessage(UPDATE).sendToTarget();
        }
    }

    private static class GroupItemHolder {
        public TextView name;
    }

    private final class GroupListAdapter extends BaseAdapter {

        @Override
        public int getCount() {
            return Groups.getInstance().size();
        }

        @Override
        public Group getItem(int position) {
            return Groups.getInstance().get(position);
        }

        @Override
        public long getItemId(int position) {
            return 0;
        }

        public Group get(int addr) {
            return Groups.getInstance().getByMeshAddress(addr);
        }

        @Override
        @Deprecated
        public View getView(int position, View convertView, ViewGroup parent) {

            GroupItemHolder holder;

            if (convertView == null) {

                convertView = inflater.inflate(R.layout.grouping_item, null);

                TextView txtName = (TextView) convertView
                        .findViewById(R.id.txt_name);

                holder = new GroupItemHolder();
                holder.name = txtName;

                convertView.setTag(holder);

            } else {
                holder = (GroupItemHolder) convertView.getTag();
            }

            Group group = this.getItem(position);

            if (group != null) {
                holder.name.setText(group.name);

                Activity mContext = DeviceGroupingActivity.this;
                if (group.checked) {
                    ColorStateList color = mContext.getResources()
                            .getColorStateList(R.color.theme_positive_color);
                    holder.name.setTextColor(color);
                } else {
                    ColorStateList color = mContext.getResources()
                            .getColorStateList(R.color.black);
                    holder.name.setTextColor(color);
                }

            }

            return convertView;
        }
    }
}
