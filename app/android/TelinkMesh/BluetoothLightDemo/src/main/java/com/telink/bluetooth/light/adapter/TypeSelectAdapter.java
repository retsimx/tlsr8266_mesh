/********************************************************************************************************
 * @file     TypeSelectAdapter.java 
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
import android.content.Intent;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.activity.MeshOTAActivity;
import com.telink.bluetooth.light.file.FileSelectActivity;
import com.telink.bluetooth.light.model.MeshDeviceType;
import com.telink.bluetooth.light.model.TestModel;

import java.util.ArrayList;
import java.util.List;

import androidx.recyclerview.widget.RecyclerView;

/**
 * Created by kee on 2017/12/19.
 */

public class TypeSelectAdapter extends BaseRecyclerViewAdapter<TypeSelectAdapter.ViewHolder> {
    private List<MeshDeviceType> models;
    //    private List<String> filePathList;
    private int selectPosition = -1;
    private boolean enable = true;
    private Context context;

    public TypeSelectAdapter(Context context, List<MeshDeviceType> models) {
        this.context = context;
        this.models = models;
        /*filePathList = new ArrayList<>();
        for (int i = 0; i < models.size(); i++) {
            filePathList.add(null);
        }*/
    }


    @Override
    public ViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        View itemView = LayoutInflater.from(context).inflate(R.layout.item_mesh_ota_file_select, parent, false);
        ViewHolder holder = new ViewHolder(itemView);
        holder.tv_name = (TextView) itemView.findViewById(R.id.tv_type_name);
        holder.iv_select = (ImageView) itemView.findViewById(R.id.iv_select);
        holder.ll_select = itemView.findViewById(R.id.ll_select);
        holder.tv_file_path = (TextView) itemView.findViewById(R.id.tv_file_path);
        return holder;
    }

    @Override
    public void onBindViewHolder(ViewHolder holder, final int position) {
        super.onBindViewHolder(holder, position);
        if (position == selectPosition) {
            holder.iv_select.setImageResource(R.drawable.light_group_select2);
        } else {
            holder.iv_select.setImageResource(R.drawable.light_group_select);
        }
        holder.tv_name.setText("Type: 0x" + Integer.toHexString(models.get(position).type) +
                "（online: " + models.get(position).deviceList.size() + "）");
        holder.ll_select.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (enable)
                    ((MeshOTAActivity) context).startActivityForResult(new Intent(context, FileSelectActivity.class).putExtra(FileSelectActivity.KEY_SUFFIX, ".bin"), position);
            }
        });

        holder.tv_file_path.setText(models.get(position).filePath == null ? "Select file(NULL)" : models.get(position).filePath);
    }

    @Override
    public int getItemCount() {
        return models == null ? 0 : models.size();
    }

    class ViewHolder extends RecyclerView.ViewHolder {

        TextView tv_name, tv_file_path;
        ImageView iv_select;
        View ll_select;

        public ViewHolder(View itemView) {
            super(itemView);
        }
    }

    public int getSelectPosition() {
        return selectPosition;
    }

    public void setSelectPosition(int selectPosition) {
        this.selectPosition = selectPosition;
        notifyDataSetChanged();
    }

    public void insertFileInfo(int position, String filePath) {
//        filePathList.set(position, filePath);
        models.get(position).filePath = filePath;
        notifyDataSetChanged();
    }

    public boolean isEnable() {
        return enable;
    }

    public void setEnable(boolean enable) {
        this.enable = enable;
    }
}
