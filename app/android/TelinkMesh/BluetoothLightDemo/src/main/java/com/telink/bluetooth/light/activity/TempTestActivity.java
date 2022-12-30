/********************************************************************************************************
 * @file     TempTestActivity.java 
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
import android.os.Handler;
import android.os.HandlerThread;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.TextView;

import com.telink.bluetooth.event.DeviceEvent;
import com.telink.bluetooth.event.NotificationEvent;
import com.telink.bluetooth.light.ConnectionStatus;
import com.telink.bluetooth.light.DeviceInfo;
import com.telink.bluetooth.light.LightAdapter;
import com.telink.bluetooth.light.OnlineStatusNotificationParser;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Lights;
import com.telink.bluetooth.light.util.MeshCommandUtil;
import com.telink.util.Event;
import com.telink.util.EventListener;

import java.text.SimpleDateFormat;
import java.util.List;

/**
 * Created by Administrator on 2017/3/22.
 */

public class TempTestActivity extends TelinkBaseActivity implements EventListener<String>, View.OnClickListener {
    TextView tv_info;
    SimpleDateFormat format = new SimpleDateFormat("HH:mm:ss");
    private Handler handler = new Handler();
//    private final static long PERIOD = 5 * 1000;

    private int index = 0;
//    private int notifyCount = 0;

    // online connectionStatus on count
    private int onNfCnt = 0;

    // online connectionStatus off count
    private int offNfCnt = 0;

    // online connectionStatus offline count
    private int offlineNfCnt = 0;

    private int initState = 0;

    private boolean isStarted = false;

    private TextView tv_cur_adr;
    private EditText et_count, et_period, et_address, et_vendor_id;

    private int count, period, address;

    private Button btn_start;

    private ScrollView scrollView;
    private Handler scrollRefresh = new Handler();

    private DeviceInfo curDevice;
    private TelinkLightApplication app;
    private int vendorId;


    private HandlerThread thread1;
    private Handler handler1;

    private HandlerThread thread2;
    private Handler handler2;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_temp_test);
        tv_info = (TextView) findViewById(R.id.info);
        tv_info.setText("");
//        TelinkLightApplication.getApp().addEventListener(NotificationEvent.ONLINE_STATUS, this);
        TelinkLightApplication.getApp().addEventListener(NotificationEvent.ONLINE_STATUS, this);
        TelinkLightApplication.getApp().addEventListener(DeviceEvent.STATUS_CHANGED, this);

        app = TelinkLightApplication.getApp();

        et_count = (EditText) findViewById(R.id.et_count);
        et_period = (EditText) findViewById(R.id.et_period);
        et_address = (EditText) findViewById(R.id.et_address);
        et_vendor_id = (EditText) findViewById(R.id.et_vendor);
        TestInput testInput = TelinkLightApplication.getApp().getTestInput();
        if (testInput != null) {
            et_count.setText(testInput.count + "");
            et_period.setText(testInput.period + "");
            et_address.setText(Integer.toHexString(testInput.adr));
        }


        tv_cur_adr = (TextView) findViewById(R.id.tv_cur_adr);

        curDevice = TelinkLightApplication.getApp().getConnectDevice();
//        if (curDevice == null)

        refreshAdrInfo();


        btn_start = (Button) findViewById(R.id.btn_start);

        scrollView = (ScrollView) findViewById(R.id.sv_log);
        btn_start.setOnClickListener(this);
        isStarted = false;

        findViewById(R.id.thread_test).setOnClickListener(this);
        thread1 = new HandlerThread("thread_command_test1");
        thread1.start();
        handler1 = new Handler(thread1.getLooper());

        thread2 = new HandlerThread("thread_command_test2");
        thread2.start();
        handler2 = new Handler(thread2.getLooper());

    }



    @Override
    protected void onDestroy() {
        super.onDestroy();
        TelinkLightApplication.getApp().removeEventListener(this);
        if (this.handler != null) {
            handler.removeCallbacksAndMessages(null);
        }
        if (scrollRefresh != null) {
            scrollRefresh.removeCallbacksAndMessages(null);
        }
    }

    private Runnable notifyTask = new Runnable() {
        @Override
        public void run() {
            if (index < count) {
                index++;
                testOnOff();
                handler.postDelayed(this, period);
            } else {
                updateUI(true);
                isStarted = false;
                refreshBtn();
            }
        }
    };

    private void refreshBtn() {
        btn_start.setText(isStarted ? "Stop" : "Start");
    }

    private void updateUI(boolean complete) {
        final StringBuilder sb = new StringBuilder("\n");
//        sb.append(format.format(new Date()));

        int totalReceive = onNfCnt + offNfCnt;
        sb.append("\n\t\tSend:").append(index).append("/").append(count)
                .append(" ---- ").append(" Receive:").append(totalReceive)
                .append("\n\t\tOnNotify: ").append(onNfCnt)
                .append("\n\t\tOffNotify: ").append(offNfCnt)

                .append("\n\t\t(OfflineNotify: ").append(offlineNfCnt).append(")");
        if (complete) {
            sb.append("\n\t\t").append("测试完成 ");
            sb.append("收包率: ").append(totalReceive * 100 / index).append("%");
        }

        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                tv_info.setText(sb.toString());
//                scrollRefresh.post(scrollDown);
            }
        });

    }

    private void testOnOff() {
        if (index % 2 == initState) {
            byte opcode = (byte) 0xD0;
//            int address = this.address;
            byte[] params = new byte[]{0x01, 0x00, 0x00};

            TelinkLightService.Instance().sendVendorCommand(opcode, vendorId, address, params);

//            TelinkLightService.Instance().sendCommandNoResponse(opcode, address, params);
//            appendLog("CMD: " + "all on -- " + (index + 1));
            updateUI(false);
        } else {
            byte opcode = (byte) 0xD0;
//            int address = 0xFFFF;
            byte[] params = new byte[]{0x00, 0x00, 0x00};
            TelinkLightService.Instance().sendVendorCommand(opcode, vendorId, address, params);

//            TelinkLightService.Instance().sendCommandNoResponse(opcode, address, params);

//            appendLog("CMD: " + "all off -- " + (index + 1));
            updateUI(false);
        }
    }


    @Override
    public void performed(Event<String> event) {
        if (event.getType().equals(NotificationEvent.ONLINE_STATUS)) {
            if (!isStarted) return;
            List<OnlineStatusNotificationParser.DeviceNotificationInfo> notificationInfoList;
            //noinspection unchecked
            notificationInfoList = (List<OnlineStatusNotificationParser.DeviceNotificationInfo>) ((NotificationEvent) event).parse();
            for (OnlineStatusNotificationParser.DeviceNotificationInfo info : notificationInfoList) {
                if (info.meshAddress == address) {
//                    notifyCount++;
                    switch (info.connectionStatus) {
                        case ON:
                            onNfCnt++;
                            updateUI(false);
//                            appendLog("notify: on " + onNfCnt + "\n");
                            break;
                        case OFF:
                            offNfCnt++;
                            updateUI(false);
//                            appendLog("notify: off " + offNfCnt + "\n");
                            break;
                        case OFFLINE:
                            offlineNfCnt++;
                            updateUI(false);
//                            appendLog("notify: offline " + offlineNfCnt + "\n");
                            break;
                    }
                }
            }

        } else if (event.getType().equals(DeviceEvent.STATUS_CHANGED)) {
            refreshAdrInfo();
            final int status = ((DeviceEvent) event).getArgs().status;
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    if (status == LightAdapter.STATUS_LOGOUT) {
                        handler.removeCallbacksAndMessages(null);
                        isStarted = false;
                        refreshBtn();
                        tv_info.append("\n\n连接断开");
                    } else if (status == LightAdapter.STATUS_LOGIN) {
                        tv_info.append("\n\n连接成功");
                    }
                }
            });
        }
    }

    private void refreshAdrInfo() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
//                curDevice = TelinkLightApplication.getApp().getConnectDevice();
                tv_cur_adr.setText(getString(R.string.temp_test_cur_adr, curDevice == null ? "null" : "0x" + Integer.toHexString(curDevice.meshAddress)));
            }
        });
    }

/*    private void appendLog(final String info) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                tv_info.append("\n" + format.format(new Date()) + " -- " + info);
                scrollRefresh.post(scrollDown);
            }
        });
    }*/


    private Runnable scrollDown = new Runnable() {
        @Override
        public void run() {
            scrollView.fullScroll(ScrollView.FOCUS_DOWN);
        }
    };

    @Override
    public void onClick(View v) {
        if (v.getId() == R.id.btn_start) {
            if (!isStarted) {
                if (curDevice == null) {
                    showToast("当前未连接");
                    return;
                }

                try {
                    count = Integer.parseInt(et_count.getText().toString().trim());
                    period = Integer.parseInt(et_period.getText().toString().trim());
                    address = Integer.parseInt(et_address.getText().toString().trim(), 16);
                    vendorId = Integer.parseInt(et_vendor_id.getText().toString().trim(), 16);
                    if (address == curDevice.meshAddress) {
                        showToast("目标设备不可为直连");
                        return;
                    }

                    Light light = Lights.getInstance().getByMeshAddress(address);
                    if (light == null || light.connectionStatus == ConnectionStatus.OFFLINE) {
                        showToast("未发现目标设备");
                        return;
                    }


                    if (light.connectionStatus == ConnectionStatus.ON) {
                        initState = 1;
                    } else {
                        initState = 0;
                    }

                    index = 0;
//                notifyCount = 0;
                    onNfCnt = 0;
                    offNfCnt = 0;
                    offlineNfCnt = 0;
                    isStarted = true;
                    refreshBtn();

                    TestInput testInput = new TestInput();
                    testInput.adr = address;
                    testInput.count = count;
                    testInput.period = period;
                    TelinkLightApplication.getApp().setTestInput(testInput);

                    handler.removeCallbacksAndMessages(null);
                    handler.post(notifyTask);
                } catch (Exception e) {
                    showToast("input error");
                    e.printStackTrace();
                }

            } else {
                handler.removeCallbacksAndMessages(null);
                isStarted = false;
                refreshBtn();
//                appendLog("测试手动停止!");
            }
        }else if (v.getId() == R.id.thread_test){
            if (!testRunning){
                testRunning = true;
                handler1.post(test1);
                handler2.post(test2);
            }else {
                testRunning = false;
            }
        }
    }

    private boolean testRunning = false;
    private Runnable test1 = new Runnable() {
        @Override
        public void run() {
            if (testRunning){
                TelinkLightService.Instance().updateNotification();
//                MeshCommandUtil.getDeviceOTAState();
                handler1.postDelayed(this, 10);
            }
        }
    };

    private Runnable test2 = new Runnable() {
        @Override
        public void run() {
            if (testRunning){
                TelinkLightService.Instance().updateNotification();
                handler2.postDelayed(this, 10);
            }
        }
    };



    public class TestInput {
        int adr = 0;
        int period = 0;
        int count = 0;
    }
}
