/********************************************************************************************************
 * @file     UserAllActivity.java 
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

import androidx.appcompat.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.os.Handler;
import android.text.TextUtils;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import com.telink.bluetooth.event.NotificationEvent;
import com.telink.bluetooth.light.NotificationInfo;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.util.Arrays;
import com.telink.util.Event;
import com.telink.util.EventListener;

import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

/**
 * Created by Administrator on 2017/3/22.
 */

public class UserAllActivity extends TelinkBaseActivity implements EventListener<String> {
    TextView tv_info;
    SimpleDateFormat format = new SimpleDateFormat("HH:mm:ss");

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_user_all);
        tv_info = (TextView) findViewById(R.id.info);
        tv_info.setText("log:");
//        TelinkLightApplication.getApp().addEventListener(NotificationEvent.ONLINE_STATUS, this);
        TelinkLightApplication.getApp().addEventListener(NotificationEvent.USER_ALL_NOTIFY, this);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        TelinkLightApplication.getApp().removeEventListener(this);
        if (this.handler != null) {
            handler.removeCallbacksAndMessages(null);
        }
    }

    private Handler handler = new Handler();
    //    private boolean userAllRunning = false;
    private final static long PERIOD = 5 * 1000;
    private Runnable notifyTask = new Runnable() {
        @Override
        public void run() {
            userAll();
            tv_info.append("\n");
            handler.postDelayed(this, PERIOD);
        }
    };

    private long lastTime = 0;

    public void get(View view) {
        /*if (userAllRunning) {
            handler.removeCallbacks(notifyTask);
            ((Button) view).setText("Stopped");
        } else {
            handler.post(notifyTask);
            ((Button) view).setText("Started");
        }
        userAllRunning = !userAllRunning;*/
        lastTime = System.currentTimeMillis();
        userAll();
        index = 0;
    }


    private void userAll() {
        byte opcode = (byte) 0xEA;
        byte params[] = new byte[]{0x10};
        int address = 0xFFFF;
        TelinkLightService.Instance().sendCommandNoResponse(opcode, address, params);
    }

    private int index = 0;

    @Override
    public void performed(Event<String> event) {
        if (event.getType().equals(NotificationEvent.USER_ALL_NOTIFY)) {
            NotificationInfo info = ((NotificationEvent) event).getArgs();
            final String msg = Arrays.bytesToHexString(info.params, ":");
            index++;
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    tv_info.append("\n" + (System.currentTimeMillis() - lastTime) + "ms   " + index + ":\t  " + msg);
//                    tv_info.append("\n" + format.format(Calendar.getInstance().getTimeInMillis())  + ":\t  " + msg);
//                    tv_info.append("\n" + format.format(Calendar.getInstance().getTimeInMillis()) + index + ":\t  " + msg);

                }
            });
        }
    }

    public void clear(View view) {
        index = 0;
        tv_info.setText("log:");
    }


    AlertDialog dialog;

    public void save(View view) {
        if (dialog == null) {
            AlertDialog.Builder dialogBuilder = new AlertDialog.Builder(this);
            final EditText editText = new EditText(this);
            dialogBuilder.setView(editText);
            dialogBuilder.setNegativeButton("cancel", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.dismiss();
                }
            }).setPositiveButton("confirm", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    if (TextUtils.isEmpty(editText.getText().toString())) {
                        Toast.makeText(UserAllActivity.this, "fileName cannot be null", Toast.LENGTH_SHORT).show();
                    } else {
                        TelinkLightApplication.getApp().saveLogInFile(editText.getText().toString().trim(), tv_info.getText().toString());
                    }
                }
            });
            dialog = dialogBuilder.create();
        }
        dialog.show();


    }
}
