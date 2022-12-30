/********************************************************************************************************
 * @file     OtaDeviceAdapter.java 
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
package com.telink.bluetooth.light.adapter;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.TextView;

import com.telink.bluetooth.light.DeviceInfo;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.model.Light;

import java.util.List;

import androidx.recyclerview.widget.RecyclerView;

/**
 * Created by kee on 2017/12/19.
 */

public class OtaDeviceAdapter extends BaseRecyclerViewAdapter<OtaDeviceAdapter.ViewHolder> {
    private List<Light> models;
    private Context context;

    public OtaDeviceAdapter(Context context, List<Light> models) {
        this.context = context;
        this.models = models;
    }


    @Override
    public ViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        View itemView = LayoutInflater.from(context).inflate(R.layout.ota_device_item, parent, false);
        ViewHolder holder = new ViewHolder(itemView);
        holder.tv_name = (TextView) itemView.findViewById(R.id.txt_name);
        holder.cb_select = (CheckBox) itemView.findViewById(R.id.cb_select);
        return holder;
    }

    @Override
    public void onBindViewHolder(ViewHolder holder, int position) {
        super.onBindViewHolder(holder, position);
        Light light = models.get(position);
        holder.cb_select.setChecked(light.selected);
        holder.tv_name.setText("Type: 0x" + Integer.toHexString(light.productUUID) +
                "\nmeshAddress: " + light.meshAddress +
                "\nmac: " + light.macAddress);
    }

    @Override
    public int getItemCount() {
        return models == null ? 0 : models.size();
    }

    class ViewHolder extends RecyclerView.ViewHolder {

        TextView tv_name;
        CheckBox cb_select;

        public ViewHolder(View itemView) {
            super(itemView);
        }
    }


}
