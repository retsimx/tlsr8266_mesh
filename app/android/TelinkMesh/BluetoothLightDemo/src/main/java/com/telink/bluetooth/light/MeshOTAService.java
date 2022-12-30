/********************************************************************************************************
 * @file MeshOTAService.java
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
package com.telink.bluetooth.light;

import android.app.Service;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;

import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.event.DeviceEvent;
import com.telink.bluetooth.event.LeScanEvent;
import com.telink.bluetooth.event.MeshEvent;
import com.telink.bluetooth.event.NotificationEvent;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Lights;
import com.telink.bluetooth.light.model.Mesh;
import com.telink.bluetooth.light.model.OtaDevice;
import com.telink.bluetooth.light.util.MeshCommandUtil;
import com.telink.bluetooth.light.widget.ProgressViewManager;
import com.telink.util.ContextUtil;
import com.telink.util.Event;
import com.telink.util.EventListener;
import com.telink.util.Strings;

import java.util.Arrays;
import java.util.List;

import androidx.annotation.Nullable;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

/**
 * 通过广播形式转发
 * Created by kee on 2018/4/23.
 */

public class MeshOTAService extends Service implements EventListener<String> {
    private static final String TAG = "MeshOTAService";

    private static MeshOTAService instance;
    public static boolean isRunning;
    private DeviceInfo otaDevice;
    private byte[] firmware;
    private int mode;
    private int type;

    public static final int MODE_IDLE = 1;
    //    private static final int MODE_OTA = 2;
    public static final int MODE_MESH_OTA = 4;
    public static final int MODE_CONTINUE_MESH_OTA = 8;

    /**
     * 传入的状态为complete, 仅作为临时状态
     */
    public static final int MODE_COMPLETE = 16;

    public static final String INTENT_KEY_INIT_MODE = "com.telink.bluetooth.light.INTENT_KEY_INIT_MODE";

    public static final String INTENT_KEY_OTA_MODE = "com.telink.bluetooth.light.INTENT_KEY_OTA_MODE";
    public static final String INTENT_KEY_OTA_FIRMWARE = "com.telink.bluetooth.light.INTENT_KEY_OTA_FIRMWARE";
    public static final String INTENT_KEY_OTA_DEVICE = "com.telink.bluetooth.light.INTENT_KEY_OTA_DEVICE";
    public static final String INTENT_KEY_OTA_TYPE = "com.telink.bluetooth.light.INTENT_KEY_OTA_TYPE";

    public static final int OTA_STATE_START = 0x50;
    public static final int OTA_STATE_FAIL = 0x51;
    public static final int OTA_STATE_NOT_SUPPORT = 0x52;

//    public static final int OTA_STATE_SUCCESS = 0x52;

    public static final int OTA_STATE_PUSHING = 0x53;
    public static final int OTA_STATE_MESHING = 0x54;
    public static final int OTA_STATE_REBOOTING = 0x55;
    public static final int OTA_STATE_COMPLETE = 0x56;

    /*
     * broadcast 动作
     */

    /**
     * log
     */
    public static final String ACTION_LOG = "com.telink.bluetooth.light.ACTION_LOG";

    /**
     * 扫描， 连接状态等的变化
     */
    public static final String ACTION_STATUS_CHANGE = "com.telink.bluetooth.light.ACTION_STATUS_CHANGE";

    /**
     * gatt firmware push progress
     */
    public static final String ACTION_PUSH_PROGRESS = "com.telink.bluetooth.light.ACTION_PUSH_PROGRESS";

    /**
     * mesh progress
     */
    public static final String ACTION_MESH_PROGRESS = "com.telink.bluetooth.light.ACTION_MESH_PROGRESS";

    public static final String INFO_LOG = "com.telink.bluetooth.light.INFO_LOG";
    public static final String INFO_STATE = "com.telink.bluetooth.light.INFO_STATE";
    public static final String INFO_STATE_DESC = "com.telink.bluetooth.light.INFO_STATE_DESC";

    /**
     * firmware push complete
     */
    private boolean isOTAComplete = false;

    /**
     * mesh ota complete
     */
    private boolean isMeshOTAComplete = false;

    /**
     * delayed timer
     */
    private Handler delayHandler;

    /**
     * lights when start
     */
    private List<Light> onlineLights;

    @Nullable
    @Override
    public IBinder onBind(Intent intent) {

        TelinkLog.d(TAG + "#onBind");
        return null;
    }

    public static MeshOTAService getInstance() {
        return instance;
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {

        TelinkLog.d(TAG + "#onStartCommand");
        Bundle bundle = intent.getExtras();
        if (bundle == null) {
            stopSelf();
            return Service.START_NOT_STICKY;
        }

        delayHandler = new Handler();

        mode = intent.getIntExtra(INTENT_KEY_INIT_MODE, MODE_IDLE);
        onlineLights = Lights.getInstance().getLocalList(true);
        if (!ContextUtil.isOverlayDisable(this)) {
            ProgressViewManager.getInstance().createFloatWindow(this);
        }
        if (mode == MODE_IDLE) {
            otaDevice = bundle.getParcelable(INTENT_KEY_OTA_DEVICE);
            firmware = bundle.getByteArray(INTENT_KEY_OTA_FIRMWARE);
            mode = bundle.getInt(INTENT_KEY_OTA_MODE);
            type = bundle.getInt(INTENT_KEY_OTA_TYPE);
            start();
        } else if (mode == MODE_CONTINUE_MESH_OTA) {
            ProgressViewManager.getInstance().updateState("continue meshing...");
            addEventListener();
        } else if (mode == MODE_COMPLETE) {
            ProgressViewManager.getInstance().updateState("mesh OTA complete");
            ProgressViewManager.getInstance().updateProgress(100);
            this.mode = MODE_IDLE;
        }
        return START_NOT_STICKY;
    }

    @Override
    public void onCreate() {
        isRunning = true;
        instance = this;
        super.onCreate();
    }

    @Override
    public void onDestroy() {
        TelinkLog.d(TAG + "#onDestroy");
        isRunning = false;
        super.onDestroy();
        if (delayHandler != null) {
            delayHandler.removeCallbacksAndMessages(null);
        }
        ProgressViewManager.getInstance().removeFloatWindowManager();
        TelinkLightApplication.getApp().removeEventListener(this);
    }

    public int getMode() {
        return mode;
    }

    public void start() {
        this.mode = MODE_MESH_OTA;
        addEventListener();
        if (TelinkLightApplication.getApp().getConnectDevice().macAddress.equals(otaDevice.macAddress)) {
            sendGetDeviceOtaStateCommand();
        } else {
            TelinkLightService.Instance().disconnect();
        }
    }

    public void restart(Intent intent) {
        if (this.mode != MODE_IDLE) return;
        Bundle bundle = intent.getExtras();
        if (bundle == null) return;
        otaDevice = bundle.getParcelable(INTENT_KEY_OTA_DEVICE);
        firmware = bundle.getByteArray(INTENT_KEY_OTA_FIRMWARE);
        mode = bundle.getInt(INTENT_KEY_OTA_MODE);
        type = bundle.getInt(INTENT_KEY_OTA_TYPE);
        onlineLights = Lights.getInstance().getLocalList(true);
        start();
    }

    public void stop() {
        TelinkLightApplication.getApp().removeEventListener(this);
    }


    private void addEventListener() {
        TelinkLightApplication.getApp().addEventListener(DeviceEvent.STATUS_CHANGED, this);
        TelinkLightApplication.getApp().addEventListener(LeScanEvent.LE_SCAN, this);
        TelinkLightApplication.getApp().addEventListener(LeScanEvent.LE_SCAN_COMPLETED, this);
        TelinkLightApplication.getApp().addEventListener(LeScanEvent.LE_SCAN_TIMEOUT, this);
        TelinkLightApplication.getApp().addEventListener(NotificationEvent.GET_DEVICE_STATE, this);
        TelinkLightApplication.getApp().addEventListener(MeshEvent.OFFLINE, this);
    }

    @Override
    public void performed(Event<String> event) {
//        if (this.mode == MODE_COMPLETE) return;
        switch (event.getType()) {
            case DeviceEvent.STATUS_CHANGED:
                onDeviceEvent((DeviceEvent) event);
                break;
            case NotificationEvent.GET_DEVICE_STATE:
                onNotificationEvent((NotificationEvent) event);
                break;

            case MeshEvent.OFFLINE:
                onMeshOffline((MeshEvent) event);
                break;
        }
    }

    private void onMeshOffline(MeshEvent event) {

        if (mode != MODE_IDLE && TelinkLightApplication.getApp().getMesh().isOtaProcessing()) {
            log("Mesh offline, back and retry");
        }
        /*if (TelinkLightApplication.getApp().getMesh().isOtaProcessing()) {
            TelinkLightService.Instance().idleMode(true);


        }*/
    }

    private void onDeviceEvent(DeviceEvent event) {
        int status = event.getArgs().status;
        switch (status) {
            case LightAdapter.STATUS_LOGOUT:
                TelinkLog.i("OTAUpdate#STATUS_LOGOUT");
                log("logout");
                delayHandler.removeCallbacksAndMessages(null);
                if (this.mode != MODE_IDLE) {
                    if (this.mode == MODE_MESH_OTA && !isOTAComplete) {
                        log("OTA fail, click start to retry");
                        onStateChange(OTA_STATE_FAIL, "push firmware fail");
                        this.mode = MODE_IDLE;
//                        enableUI(true);
                    } else {
//                        startScan();
                    }
                }

                break;

            case LightAdapter.STATUS_LOGIN:
                TelinkLog.i("OTAUpdate#STATUS_LOGIN");
                log("login success");
                if (this.mode == MODE_MESH_OTA || this.mode == MODE_CONTINUE_MESH_OTA) {
                    sendGetDeviceOtaStateCommand();
                }
                break;

            case LightAdapter.STATUS_CONNECTED:
                log("connected");
                break;

            case LightAdapter.STATUS_OTA_PROGRESS:
                OtaDeviceInfo deviceInfo = (OtaDeviceInfo) event.getArgs();
//                log("ota progress :" + deviceInfo.progress + "%");

                onProgressChange(deviceInfo.progress);
//                msgHandler.obtainMessage(MSG_MESH_OTA_PROGRESS, "firmware push -- " + deviceInfo.progress + "%").sendToTarget();
                break;

            case LightAdapter.STATUS_OTA_COMPLETED:
                log("OTA complete");
                log("mesh OTA processing...");
                isOTAComplete = true;
                onStateChange(OTA_STATE_MESHING, "package meshing...");
//                msgHandler.obtainMessage(MSG_MESH_OTA_PROGRESS, "firmware push -- complete").sendToTarget();

                // 开始meshOTA
                Mesh mesh = TelinkLightApplication.getApp().getMesh();
                mesh.otaDevice = new OtaDevice();
                DeviceInfo curDevice = event.getArgs();
                mesh.otaDevice.mac = curDevice.macAddress;
                mesh.otaDevice.meshName = mesh.name;
                mesh.otaDevice.meshPwd = mesh.password;
                mesh.saveOrUpdate(this);

                TelinkLightService.Instance().setAutoConnectMac(otaDevice.macAddress);
                break;

            case LightAdapter.STATUS_OTA_FAILURE:
                log("OTA fail");
                onStateChange(OTA_STATE_FAIL, "push firmware fail");
//                startScan();
                break;
        }
    }


    private void onNotificationEvent(NotificationEvent event) {
//        log("service notification");
        // 解析版本信息
        byte[] data = event.getArgs().params;
        if (data[0] == NotificationEvent.DATA_GET_VERSION) {

            if (this.mode == MODE_IDLE) {
                return;
            }
            String version = Strings.bytesToString(Arrays.copyOfRange(data, 1, 5));

            int meshAddress = event.getArgs().src;

            Lights.getInstance().getByMeshAddress(meshAddress).selected = true;
            TelinkLog.w(" src:" + meshAddress + " get version success: " + version);
            log("getVersion: 0x" + Integer.toHexString(meshAddress) + "  version:" + version);

            Lights.getInstance().getByMeshAddress(meshAddress).firmwareRevision = version;
            /*for (Light light : onlineLights) {
                if (light.meshAddress == meshAddress) {
                    light.firmwareRevision = version;
                }
            }*/

        } else if (data[0] == NotificationEvent.DATA_GET_MESH_OTA_PROGRESS) {
            TelinkLog.w("mesh ota progress: " + data[1]);
            int progress = (int) data[1];
            onProgressChange(progress);
//            msgHandler.obtainMessage(MSG_MESH_OTA_PROGRESS, "MeshOTA -- " + progress + "%").sendToTarget();

            if (progress == 99) {
                isMeshOTAComplete = true;
                onStateChange(OTA_STATE_REBOOTING, "device rebooting...");
//                msgHandler.obtainMessage(MSG_MESH_OTA_PROGRESS, "MeshOTA -- device rebooting...").sendToTarget();
            }

        } else if (data[0] == NotificationEvent.DATA_GET_OTA_STATE) {
            delayHandler.removeCallbacks(deviceOtaStateTimeoutTask);
            log("OTA State notification");
            int otaState = data[1];
//            log("OTA State response--" + otaState);
            if (this.mode == MODE_IDLE) return;
            if (otaState == NotificationEvent.OTA_STATE_IDLE && this.mode == MODE_MESH_OTA && !isOTAComplete) {
                setDeviceOTAMode();
            } else if (otaState == NotificationEvent.OTA_STATE_MASTER &&
                    (this.mode == MODE_MESH_OTA || this.mode == MODE_CONTINUE_MESH_OTA)) {

                /*if (this.mode == MODE_MESH_OTA) {
//                    sendStartMeshOTACommand();
                } else {
                    sendGetVersionCommand();
                }*/
            } else if (otaState == NotificationEvent.OTA_STATE_COMPLETE && isMeshOTAComplete) {
                log("mesh ota complete");

                onProgressChange(100);
//                this.mode = MODE_IDLE;
                onStateChange(OTA_STATE_COMPLETE, "mesh ota complete");
                Mesh mesh = TelinkLightApplication.getApp().getMesh();
                mesh.otaDevice = null;
                mesh.saveOrUpdate(this);
                for (Light light : onlineLights) {
                    light.selected = false;
                }
                getVersionRetry = 0;
                sendGetVersionCommand();
            } else {
                log("OTA State error: " + otaState);
                onStateChange(OTA_STATE_FAIL, "OTA State error: " + otaState);
                sendStopMeshOTACommand();
                doComplete();
            }
        } else if (data[0] == NotificationEvent.DATA_SET_OTA_MODE_NOTIFY) {
            log("set OTA mode notify:" + data[1]);
            delayHandler.removeCallbacks(otaModeCmdCheckTask);
            if (data[1] == 0x00) {
                log("OTA firmware pushing ...");
                onStateChange(OTA_STATE_PUSHING, "OTA firmware pushing ...");
                TelinkLightService.Instance().startOta(firmware);
            } else {
                onMeshOTANotSupport("mode set notify check err");
            }
        }

    }


    private void doComplete() {
        stop();
        this.mode = MODE_IDLE;
        isOTAComplete = false;
        isMeshOTAComplete = false;
        Mesh mesh = TelinkLightApplication.getApp().getMesh();
        mesh.otaDevice = null;
        mesh.saveOrUpdate(this);
        log("action finish!");
    }

    // 获取本地设备OTA状态信息
    private void sendGetDeviceOtaStateCommand() {
        delayHandler.removeCallbacks(deviceOtaStateObtainCommand);
        delayHandler.postDelayed(deviceOtaStateObtainCommand, 500);
    }

    private Runnable deviceOtaStateObtainCommand = new Runnable() {
        @Override
        public void run() {
            log("getDeviceOtaState(0xC7)");
            MeshCommandUtil.getDeviceOTAState();
            delayHandler.removeCallbacks(deviceOtaStateTimeoutTask);
            delayHandler.postDelayed(deviceOtaStateTimeoutTask, 1000);
        }
    };

    private Runnable deviceOtaStateTimeoutTask = new Runnable() {
        @Override
        public void run() {
            onMeshOTANotSupport("get device ota state timeout");
        }
    };


    /**
     * 设置设备的OTA mode
     * mode 0, 单灯OTA
     */
    private void setDeviceOTAMode() {
        isOTAComplete = false;
        isMeshOTAComplete = false;

        byte opcode = (byte) 0xC7;
        int address = 0x0000;

        int type = this.type;
        byte[] params = new byte[]{0x10, 0x06, (byte) 0x01, (byte) (type & 0xFF), (byte) (type >> 8 & 0xFF)};

        TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                params);
        delayHandler.postDelayed(otaModeCmdCheckTask, 500);

        log("set device OTA mode");
    }

    private Runnable otaModeCmdCheckTask = new Runnable() {
        @Override
        public void run() {
            onMeshOTANotSupport("not support mode set");
        }
    };

    private void sendGetVersionCommand() {
        MeshCommandUtil.getVersion();
        log("getVersion");
        // 转发次数 * () * interval + 500
        delayHandler.removeCallbacks(getVersionTask);
        delayHandler.postDelayed(getVersionTask, 0x20 * 2 * 40 + 500);
    }

    private int getVersionRetry = 0;
    private Runnable getVersionTask = new Runnable() {
        @Override
        public void run() {

            boolean miss = false;
            for (Light light : onlineLights) {
                if (!light.selected) {
                    miss = true;
//                    log("miss: " + light.meshAddress + " -- " + light.macAddress);
                }
            }

            if (!miss || getVersionRetry >= 2) {
                log("get version complete");
                getVersionRetry = 0;
                for (Light light : onlineLights) {
                    log("complete version: " + light.meshAddress + " -- " + light.macAddress + " -- " + light.firmwareRevision +
                            (!light.selected ? " -- missed" : ""));
                }
                doComplete();
            } else {
                getVersionRetry++;
                log("get version retry: " + getVersionRetry);
                sendGetVersionCommand();
            }
        }
    };

    private void sendStopMeshOTACommand() {
        MeshCommandUtil.sendStopMeshOTACommand();
    }

    private void onMeshOTANotSupport(String reason) {
        onStateChange(OTA_STATE_NOT_SUPPORT, "MeshOTA not support!" + reason);
        this.mode = MODE_IDLE;
        log("MeshOTA not support !!!");
    }


    public void log(String info) {
        info = "(S)" + info;
        TelinkLog.w("MeshOTAService: " + info);
        Intent intent = new Intent();
        intent.setAction(ACTION_LOG);
        intent.putExtra(INFO_LOG, info);
        broadcast(intent);
    }

    private void onStateChange(int state, String stateDesc) {
        TelinkLog.d(TAG + " -- onStateChange:" + state + " -- " + stateDesc);
        ProgressViewManager.getInstance().updateState(stateDesc);
        Intent intent = new Intent();
        intent.setAction(ACTION_STATUS_CHANGE);
        intent.putExtra(INFO_STATE, state);
        intent.putExtra(INFO_STATE_DESC, stateDesc);
        broadcast(intent);
    }

    private void onProgressChange(int progress) {
        ProgressViewManager.getInstance().updateProgress(progress);
    }

    private void broadcast(Intent intent) {
        LocalBroadcastManager.getInstance(MeshOTAService.this)
                .sendBroadcast(intent);
    }

}
