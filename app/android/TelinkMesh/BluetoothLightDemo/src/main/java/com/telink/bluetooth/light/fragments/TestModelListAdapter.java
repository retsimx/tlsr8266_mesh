/********************************************************************************************************
 * @file TestModelListAdapter.java
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

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.adapter.BaseRecyclerViewAdapter;
import com.telink.bluetooth.light.model.TestModel;

import java.util.List;

import androidx.recyclerview.widget.RecyclerView;

/**
 * Created by kee on 2017/12/19.
 */

public class TestModelListAdapter extends BaseRecyclerViewAdapter<TestModelListAdapter.ViewHolder> {
    private List<TestModel> models;
    private int selectPosition = 0;
    private Context context;
    private boolean isSettingMode = false;

    public TestModelListAdapter(Context context, List<TestModel> models) {
        this.context = context;
        this.models = models;
    }


    @Override
    public ViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        View itemView = LayoutInflater.from(context).inflate(R.layout.item_test_model, null);
        ViewHolder holder = new ViewHolder(itemView);
        holder.tv_name = (TextView) itemView.findViewById(R.id.tv_model_name);
        return holder;
    }

    @Override
    public void onBindViewHolder(ViewHolder holder, int position) {
        super.onBindViewHolder(holder, position);
//        holder.tv_name.setText(models.get(position).getOpCode() + "");
        if (isSettingMode && position == selectPosition) {
            holder.itemView.setBackgroundResource(R.color.theme_positive_color);
        } else {
            holder.itemView.setBackgroundResource(0);
        }


        if (!isSettingMode) {
            if (models.get(position).isHolder()) {
                holder.itemView.setVisibility(View.INVISIBLE);
            } else {
                holder.itemView.setVisibility(View.VISIBLE);
            }
        }

        holder.tv_name.setText(models.get(position).getName());
    }

    @Override
    public int getItemCount() {
        return models == null ? 0 : models.size();
    }

    class ViewHolder extends RecyclerView.ViewHolder {

        TextView tv_name;

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

    public void setSettingMode(boolean settingMode) {
        isSettingMode = settingMode;
    }
}
