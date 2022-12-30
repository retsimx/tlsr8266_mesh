/********************************************************************************************************
 * @file OtaActivity.java
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

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.Handler;
import android.text.TextUtils;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.event.DeviceEvent;
import com.telink.bluetooth.event.LeScanEvent;
import com.telink.bluetooth.event.NotificationEvent;
import com.telink.bluetooth.light.DeviceInfo;
import com.telink.bluetooth.light.LeScanParameters;
import com.telink.bluetooth.light.LightAdapter;
import com.telink.bluetooth.light.OtaDeviceInfo;
import com.telink.bluetooth.light.Parameters;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.file.FileSelectActivity;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Mesh;
import com.telink.util.Event;
import com.telink.util.EventListener;
import com.telink.util.Strings;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

public class OtaActivity extends TelinkBaseActivity implements EventListener<String>, View.OnClickListener {

    private static final int REQUEST_CODE_FILE_SELECT = 1;
    private static final long OTA_START_DELAY = 3 * 1000;
    private TelinkLightApplication mApp;
    private byte[] firmware;
    private String tarVersion;
    private int meshAddress;
    private Light selectedDevice;
    private TextView tv_device_info;
    private TextView tip;
    private TextView progress;
    private EditText otaDelay;
    private EditText otaSize;
    private boolean flag;

    private Button chooseOta, startOta;

    private String path;
    private boolean otaStarted = false;
    private boolean otaCompleted = false;

    private Handler delayHandler;

    private BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            switch (action) {
                case BluetoothAdapter.ACTION_STATE_CHANGED:
                    int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, 0);
                    switch (state) {
                        case BluetoothAdapter.STATE_OFF:
                            Toast.makeText(OtaActivity.this, "蓝牙关闭", Toast.LENGTH_SHORT).show();
                            break;
                        case BluetoothAdapter.STATE_ON:
                            Toast.makeText(OtaActivity.this, "蓝牙开启", Toast.LENGTH_SHORT).show();
                            break;
                        case BluetoothAdapter.STATE_TURNING_OFF:
                            Toast.makeText(OtaActivity.this, "正在关闭蓝牙", Toast.LENGTH_SHORT).show();
                            break;
                        case BluetoothAdapter.STATE_TURNING_ON:
                            Toast.makeText(OtaActivity.this, "正在打开蓝牙", Toast.LENGTH_SHORT).show();
                            break;
                    }
                    break;
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.setContentView(R.layout.activity_ota);

        enableBackNav(true);
        setTitle("OTA");
        this.meshAddress = this.getIntent().getIntExtra("meshAddress", 0);
        this.mApp = (TelinkLightApplication) this.getApplication();

        //监听事件
        this.mApp.addEventListener(LeScanEvent.LE_SCAN, this);
        this.mApp.addEventListener(LeScanEvent.LE_SCAN_COMPLETED, this);
        this.mApp.addEventListener(DeviceEvent.STATUS_CHANGED, this);
        this.mApp.addEventListener(NotificationEvent.GET_DEVICE_STATE, this);

        this.tv_device_info = (TextView) this.findViewById(R.id.tv_device_info);
        this.tip = (TextView) this.findViewById(R.id.tip);
        this.progress = (TextView) this.findViewById(R.id.progress);
        this.otaDelay = (EditText) this.findViewById(R.id.otadelay);
        this.otaSize = (EditText) this.findViewById(R.id.otaSize);

        this.chooseOta = (Button) this.findViewById(R.id.chooseFile);
        this.chooseOta.setOnClickListener(this);
        this.startOta = (Button) this.findViewById(R.id.startOta);
        this.startOta.setOnClickListener(this);
        Mesh mesh = this.mApp.getMesh();

        this.selectedDevice = mesh.getDevice(this.meshAddress);

        if (this.selectedDevice == null || TextUtils.isEmpty(this.selectedDevice.macAddress)) {
            Toast.makeText(getApplicationContext(), "ota升级,需要把灯加入到网络!", Toast.LENGTH_SHORT).show();
            return;
        }

        this.tv_device_info.setText(getString(R.string.ota_device_info, this.selectedDevice.meshAddress + ""));
        IntentFilter filter = new IntentFilter();
        filter.addAction(BluetoothAdapter.ACTION_STATE_CHANGED);
        registerReceiver(mReceiver, filter);

        delayHandler = new Handler();
    }

    private void append(final String msg) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                tip.append(msg + "\r\n");
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unregisterReceiver(mReceiver);
        //移除事件并断开连接
        this.mApp.removeEventListener(this);
        if (delayHandler != null)
            delayHandler.removeCallbacksAndMessages(null);
        // idle
        TelinkLightService.Instance().idleMode(true);
    }

    private void showFileChooser() {
        startActivityForResult(new Intent(this, FileSelectActivity.class).putExtra(FileSelectActivity.KEY_SUFFIX, ".bin"), REQUEST_CODE_FILE_SELECT);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (resultCode != Activity.RESULT_OK)
            return;
        path = data.getStringExtra(FileSelectActivity.KEY_RESULT);
        chooseOta.setText(getString(R.string.select_file, path));
        readFirmware(path);

        TelinkLog.d(path);
    }

    /**
     * 开始扫描
     */
    private void startScan() {
        this.append("startScan");
        TelinkLightService.Instance().idleMode(true);
        LeScanParameters params = Parameters.createScanParameters();
        params.setMeshName(this.mApp.getMesh().name);
        params.setTimeoutSeconds(15);
        TelinkLightService.Instance().startScan(params);
    }

    /**
     * 开始OTA
     */
    private void startOta() {
        this.append("startOta");
        delayHandler.removeCallbacksAndMessages(null);
        delayHandler.postDelayed(startOTATask, OTA_START_DELAY);
    }

    private Runnable startOTATask = new Runnable() {
        @Override
        public void run() {
            getDeviceOTAState();
        }
    };

    private void getDeviceOTAState() {
        byte opcode = (byte) 0xC7;
        int address = 0x0000;
        byte[] params = new byte[]{0x20, 0x05};
        TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                params);
        append("get device ota state");
        delayHandler.postDelayed(deviceOtaStateTimeoutTask, 1000);
    }

    private Runnable deviceOtaStateTimeoutTask = new Runnable() {
        @Override
        public void run() {
            append("get device ota state timeout");
            setDeviceOTAMode();
        }
    };

    private void setDeviceOTAMode() {

        byte opcode = (byte) 0xC7;
        int address = 0x0000;

        int type = this.selectedDevice.productUUID;
        byte[] params = new byte[]{0x10, 0x06, (byte) 0x00, (byte) (type & 0xFF), (byte) (type >> 8 & 0xFF)};

        this.append("set device OTA mode");

        TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                params);
        delayHandler.postDelayed(otaModeCmdCheckTask, 500);
    }

    private Runnable otaModeCmdCheckTask = new Runnable() {
        @Override
        public void run() {
            append("set mode timeout");
            onSetModeComplete();
        }
    };

    private void onSetModeComplete() {
        append("start push firmware");
        TelinkLightService.Instance().startOta(firmware);
    }

    /**
     * 事件处理方法
     *
     * @param event
     */
    @Override
    public void performed(Event<String> event) {
        if (event instanceof LeScanEvent) {
            this.onLeScanEvent((LeScanEvent) event);
        } else if (event instanceof DeviceEvent) {
            this.onDeviceEvent((DeviceEvent) event);
        } else if (event instanceof NotificationEvent) {
            onNotificationEvent((NotificationEvent) event);
        }
    }

    private void readFirmware(String fileName) {
        try {
            InputStream stream = new FileInputStream(fileName);
            int length = stream.available();
            this.firmware = new byte[length];
            stream.read(this.firmware);

            byte[] version = new byte[4];
            System.arraycopy(firmware, 2, version, 0, 4);
            tarVersion = new String(version);
            stream.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * 处理LeScanEvent事件
     *
     * @param event
     */
    private void onLeScanEvent(LeScanEvent event) {
        String type = event.getType();

        switch (type) {
            case LeScanEvent.LE_SCAN:
                //处理扫描到的设备
                DeviceInfo deviceInfo = event.getArgs();
                if (deviceInfo.meshAddress == (this.selectedDevice.meshAddress)) {
//                    this.flag = true;
                    this.append("connecting: " + deviceInfo.macAddress);
                    TelinkLightService.Instance().idleMode(true);
                    TelinkLightService.Instance().connect(deviceInfo.macAddress, 10);
                }

                break;
            case LeScanEvent.LE_SCAN_COMPLETED:
                this.append("scan timeout, device not found");
                onOTAFinish();

                break;
        }
    }

    private void login() {
        Mesh currentMesh = this.mApp.getMesh();
        TelinkLightService.Instance().login(Strings.stringToBytes(currentMesh.name, 16),
                Strings.stringToBytes(currentMesh.password, 16));
    }

    /**
     * 处理DeviceEvent事件
     *
     * @param event
     */
    private void onDeviceEvent(DeviceEvent event) {
        String type = event.getType();
        if (!otaStarted) return;
        switch (type) {
            case DeviceEvent.STATUS_CHANGED:
                int status = event.getArgs().status;
                if (status == LightAdapter.STATUS_OTA_PROGRESS) {
                    OtaDeviceInfo deviceInfo = (OtaDeviceInfo) event.getArgs();
                    this.progress.setText("ota progress :" + deviceInfo.progress + "%");
                } else if (status == LightAdapter.STATUS_CONNECTED) {
                    this.append("connected");
                    login();

                } else if (status == LightAdapter.STATUS_LOGIN) {
//                    this.append("login");
                    TelinkLightService.Instance().enableNotification();
                    if (!otaCompleted) {
                        startOta();
                    }
                } else if (status == LightAdapter.STATUS_LOGOUT) {
                    delayHandler.removeCallbacksAndMessages(null);
                    this.append("disconnected");
                    /*if (otaCompleted) {
                        this.append("ota success");
                        onOTAFinish();
                    }*/
                    if (otaStarted || otaCompleted) {
                        startScan();
                    }
                } else if (status == LightAdapter.STATUS_OTA_COMPLETED) {
                    otaCompleted = true;
                    this.append("otaCompleted");
                } else if (status == LightAdapter.STATUS_OTA_FAILURE) {
                    this.append("ota fail");
                    onOTAFinish();
                } else if (status == LightAdapter.STATUS_GET_FIRMWARE_COMPLETED) {
                    String version = event.getArgs().firmwareRevision;
                    if (otaCompleted) {
                        if (tarVersion.equals(version)) {
                            this.append("ota success");
                            onOTAFinish();
                        }
                    }
                }
                break;
        }
    }

    private void onNotificationEvent(NotificationEvent event) {
        // 解析版本信息
        byte[] data = event.getArgs().params;
        if (data[0] == NotificationEvent.DATA_GET_OTA_STATE) {
            delayHandler.removeCallbacks(deviceOtaStateTimeoutTask);
            int otaState = data[1];
            append("OTA State response--" + otaState);
            if (otaState == NotificationEvent.OTA_STATE_IDLE) {
                setDeviceOTAMode();
            } else {
                append("OTA State error: " + otaState);
            }
        } else if (data[0] == NotificationEvent.DATA_SET_OTA_MODE_NOTIFY) {
            delayHandler.removeCallbacks(otaModeCmdCheckTask);
            append("set OTA mode notify:" + data[1]);
            onSetModeComplete();
            /*if (data[1] == 0x00) {
                TelinkLightService.Instance().startOta(firmware);
            } else {
                append("mode set notify check err");
            }*/
        }

    }

    private void onOTAFinish() {
        otaStarted = false;
        otaCompleted = false;
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                chooseOta.setEnabled(true);
                startOta.setEnabled(true);
            }
        });

    }

    @Override
    public void onClick(View v) {
        if (v == this.chooseOta) {
            this.showFileChooser();
        } else if (v == this.startOta) {
            if (firmware == null) {
                showToast("OTA firmware invalid!");
                return;
            }

            otaCompleted = false;
            otaStarted = true;
            chooseOta.setEnabled(false);
            startOta.setEnabled(false);
            this.tip.setText("");
            if (mApp.getConnectDevice() != null) {
                if (mApp.getConnectDevice().meshAddress == (selectedDevice.meshAddress)) {
                    TelinkLightService.Instance().idleMode(false);
                    append("already connected");
                    startOta();
                } else {
                    TelinkLightService.Instance().idleMode(true);
                }
            } else {
                this.startScan();
            }
            /*if (mApp.getConnectDevice() != null && mApp.getConnectDevice().macAddress.equals(selectedDevice.macAddress)) {
                TelinkLightService.Instance().idleMode(false);
                startOta();
            } else {
                this.startScan();
            }*/

        }
    }
}
