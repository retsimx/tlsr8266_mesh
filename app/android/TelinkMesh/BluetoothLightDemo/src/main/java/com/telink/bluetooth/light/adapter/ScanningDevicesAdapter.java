/********************************************************************************************************
 * @file TypeSelectAdapter.java
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
package com.telink.bluetooth.light.adapter;

import android.content.Context;
import android.content.Intent;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import com.telink.bluetooth.light.DeviceInfo;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.activity.MeshOTAActivity;

import java.util.List;

import androidx.recyclerview.widget.RecyclerView;

/**
 * Created by kee on 2017/12/19.
 */

public class ScanningDevicesAdapter extends BaseRecyclerViewAdapter<ScanningDevicesAdapter.ViewHolder> {
    private List<DeviceInfo> devices;
    private Context context;

    public ScanningDevicesAdapter(Context context, List<DeviceInfo> devices) {
        this.context = context;
        this.devices = devices;
    }


    @Override
    public ViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        View itemView = LayoutInflater.from(context).inflate(R.layout.item_scanning_device, parent, false);
        ViewHolder holder = new ViewHolder(itemView);
        holder.tv_info = (TextView) itemView.findViewById(R.id.tv_info);
        return holder;
    }

    @Override
    public void onBindViewHolder(ViewHolder holder, final int position) {
        super.onBindViewHolder(holder, position);
        DeviceInfo deviceInfo = devices.get(position);
        String info = "Mac: " + deviceInfo.macAddress
                + " Type: 0x" + String.format("%02X", deviceInfo.productUUID)
                + " Rssi: " + deviceInfo.rssi;
        holder.tv_info.setText(info);
    }

    @Override
    public int getItemCount() {
        return devices == null ? 0 : devices.size();
    }

    class ViewHolder extends RecyclerView.ViewHolder {

        TextView tv_info;

        public ViewHolder(View itemView) {
            super(itemView);
        }
    }

}
