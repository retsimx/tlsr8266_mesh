/********************************************************************************************************
 * @file MeshOTAActivity.java
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

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.provider.Settings;
import android.text.TextUtils;
import android.view.View;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.event.DeviceEvent;
import com.telink.bluetooth.event.LeScanEvent;
import com.telink.bluetooth.event.NotificationEvent;
import com.telink.bluetooth.light.DeviceInfo;
import com.telink.bluetooth.light.MeshOTAService;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.adapter.BaseRecyclerViewAdapter;
import com.telink.bluetooth.light.adapter.TypeSelectAdapter;
import com.telink.bluetooth.light.file.FileSelectActivity;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Lights;
import com.telink.bluetooth.light.model.Mesh;
import com.telink.bluetooth.light.model.MeshDeviceType;
import com.telink.bluetooth.light.util.MeshCommandUtil;
import com.telink.util.ContextUtil;
import com.telink.util.Event;
import com.telink.util.EventListener;
import com.telink.util.Strings;

import java.io.FileInputStream;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.List;

import androidx.appcompat.app.AlertDialog;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

/**
 * 演示mesh ota相关功能
 * 要求， 准备升级的设备必须是之前做过加灯操作的
 * 流程：
 * 1. 判断当前mesh在线状态， 如果是非在线状态，则不可操作
 * 2. 获取所有设备的状态信息， 并刷新设备列表 类型列表{@link MeshOTAActivity#rv_type}；
 * -> 由用户选择firmware版本并勾选对应升级的设备类型
 * -> 点击开始后判断是否选择设备类型和对应文件
 * -> 无误后获取设备的OTA状态{@link #sendGetDeviceOtaStateCommand()}
 * -> 获取失败， 提示不支持； 获取成功后， 设置设备的1
 */
public class MeshOTAActivity extends TelinkBaseActivity implements EventListener<String>, View.OnClickListener {

    public static final String INTENT_KEY_CONTINUE_MESH_OTA = "com.telink.bluetooth.light.INTENT_KEY_CONTINUE_MESH_OTA";
    // 有进度状态上报 时跳转进入的
    public static final int CONTINUE_BY_REPORT = 0x21;

    // 继续之前的OTA操作，连接指定设备
//    public static final int CONTINUE_BY_PREVIOUS = 0x22;

//    private int continueType = 0;

    private static final int REQUEST_CODE_CHOOSE_FILE = 11;

    private byte[] mFirmwareData;
    private String mFileVersion;
    private Mesh mesh;
    //    private String mPath;
    private SimpleDateFormat mTimeFormat;
//    private int successCount = 0;

    //    private TextView otaProgress;
    private TextView meshOtaProgress;
    private TextView tv_log;
    private ScrollView sv_log;

    private static final int MSG_MESH_OTA_PROGRESS = 12;
    private static final int MSG_LOG = 13;
    private static final int MSG_SCROLL = 14;
    private boolean versionGetting = false;
    private Handler delayHandler = new Handler();
    private AlertDialog confirmDialog;

    private Handler msgHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            switch (msg.what) {
                case MSG_MESH_OTA_PROGRESS:
                    meshOtaProgress.setText(getString(R.string.progress_mesh_ota, msg.obj.toString()));
                    break;

                case MSG_LOG:
                    String time = mTimeFormat.format(Calendar.getInstance().getTimeInMillis());
                    tv_log.append("\n" + time + ":" + msg.obj.toString());
                    msgHandler.obtainMessage(MSG_SCROLL).sendToTarget();
                    break;

                case MSG_SCROLL:
                    sv_log.fullScroll(View.FOCUS_DOWN);
                    break;
            }
        }
    };

    /**
     * 设备列表与类型列表
     * device list and type list
     */
    private RecyclerView rv_type;
    private TypeSelectAdapter mTypeAdapter;

    private List<MeshDeviceType> mTypeList;
    private List<Light> onlineLights;

    private Button btn_start, btn_check, btn_read_version;

    private MeshDeviceType selectType;

    private DeviceInfo opDevice = null;

    private BroadcastReceiver mReceiver;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_mesh_ota);
        setTitle("Mesh OTA");
        enableBackNav(true);
        mTimeFormat = new SimpleDateFormat("HH:mm:ss.S");
//        TelinkLightService.Instance().idleMode(false);
        // 获取所有 【存储于本地】&【在线】 设备
        onlineLights = Lights.getInstance().getLocalList(true);
        initView();
        enableUI(true);
        log("local online lights:" + (onlineLights == null ? 0 : onlineLights.size()));
        Intent intent = getIntent();
        if (intent.hasExtra(INTENT_KEY_CONTINUE_MESH_OTA)) {
//            this.mode = MODE_CONTINUE_MESH_OTA;
            this.mFileVersion = TelinkLightApplication.getApp().getConnectDevice().firmwareRevision;
            log("continue mesh ota");
            enableUI(false);
        }

        opDevice = TelinkLightApplication.getApp().getConnectDevice();
        if (opDevice == null || onlineLights == null || onlineLights.isEmpty()) {
            log("offline or no valid device!");
            enableUI(false);
        } else {
            log("direct device:" + opDevice.macAddress);
            addEventListener();
            addServiceReceiver();
            mesh = TelinkLightApplication.getApp().getMesh();
            if (TelinkLightApplication.getApp().isEmptyMesh()) {
                toast("Mesh Error!");
                log("mesh empty");
                enableUI(false);
            } else {
                if (MeshOTAService.getInstance() != null && MeshOTAService.getInstance().getMode() != MeshOTAService.MODE_IDLE) {
                    log("mesh ota already running");
                    enableUI(false);
                }
            }
        }

        if (ContextUtil.isOverlayDisable(this)) {
            //没有权限，需要申请权限，因为是打开一个授权页面，所以拿不到返回状态的，所以建议是在onResume方法中从新执行一次校验
//            log("overlay window will not show");
            showPermissionDialog();
        }

    }


    private void addServiceReceiver() {
        mReceiver = new BroadcastReceiver() {

            @Override
            public void onReceive(Context context, Intent intent) {
                if (intent == null || intent.getAction() == null) return;
                switch (intent.getAction()) {
                    case MeshOTAService.ACTION_LOG:
                        log(intent.getStringExtra(MeshOTAService.INFO_LOG));
                        break;
                    case MeshOTAService.ACTION_STATUS_CHANGE:
                        int state = intent.getIntExtra(MeshOTAService.INFO_STATE, 0);
                        String stateDesc = intent.getStringExtra(MeshOTAService.INFO_STATE_DESC);
                        log("state change: " + stateDesc);

                        switch (state) {
//                            case MeshOTAService.OTA_STATE_FAIL:
                            case MeshOTAService.OTA_STATE_COMPLETE:
                                enableUI(true);
                        }
                        break;
                }
            }
        };

        final IntentFilter filter = new IntentFilter();
        filter.addAction(MeshOTAService.ACTION_LOG);
        filter.addAction(MeshOTAService.ACTION_STATUS_CHANGE);
        LocalBroadcastManager.getInstance(this).registerReceiver(mReceiver, filter);
    }

    private void initView() {
        meshOtaProgress = (TextView) findViewById(R.id.progress_mesh_ota);
        tv_log = (TextView) findViewById(R.id.tv_log);
        sv_log = (ScrollView) findViewById(R.id.sv_log);

        btn_start = (Button) findViewById(R.id.btn_start);
        btn_start.setOnClickListener(this);

        btn_check = (Button) findViewById(R.id.btn_check);
        btn_check.setOnClickListener(this);

        btn_read_version = (Button) findViewById(R.id.btn_read_version);
        btn_read_version.setOnClickListener(this);

        mTypeList = new ArrayList<>();
        if (onlineLights != null) {
            for (Light light : onlineLights) {

                MeshDeviceType type = new MeshDeviceType();
                type.type = light.productUUID;

                if (!mTypeList.contains(type)) {
                    type.deviceList.add(light);
                    type.filePath = null;

                    mTypeList.add(type);
                } else {
                    mTypeList.get(mTypeList.indexOf(type)).deviceList.add(light);
                }
            }
        }

        mTypeAdapter = new TypeSelectAdapter(this, mTypeList);
        mTypeAdapter.setOnItemClickListener(new BaseRecyclerViewAdapter.OnItemClickListener() {
            @Override
            public void onItemClick(int position) {
                if (!mTypeAdapter.isEnable()) return;
                if (mTypeAdapter.getSelectPosition() != position) {
                    mTypeAdapter.setSelectPosition(position);
                } else {
                    mTypeAdapter.setSelectPosition(-1);
                }
            }
        });

        rv_type = (RecyclerView) findViewById(R.id.rv_type);
        rv_type.setLayoutManager(new LinearLayoutManager(this));
        rv_type.setAdapter(mTypeAdapter);
    }


    @Override
    protected void onDestroy() {
        super.onDestroy();
        TelinkLightApplication.getApp().removeEventListener(this);
        if (this.delayHandler != null) {
            this.delayHandler.removeCallbacksAndMessages(null);
        }

        if (mReceiver != null)
            LocalBroadcastManager.getInstance(this).unregisterReceiver(this.mReceiver);
    }

    private void addEventListener() {
        TelinkLightApplication.getApp().addEventListener(DeviceEvent.STATUS_CHANGED, this);
        TelinkLightApplication.getApp().addEventListener(LeScanEvent.LE_SCAN, this);
        TelinkLightApplication.getApp().addEventListener(LeScanEvent.LE_SCAN_COMPLETED, this);
        TelinkLightApplication.getApp().addEventListener(LeScanEvent.LE_SCAN_TIMEOUT, this);
        TelinkLightApplication.getApp().addEventListener(NotificationEvent.GET_DEVICE_STATE, this);
    }

    private void enableUI(final boolean enable) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                btn_start.setEnabled(enable);
                btn_check.setEnabled(enable);
//                btn_check.setEnabled(true);
                btn_read_version.setEnabled(enable);
//                btn_read_version.setEnabled(true);
                mTypeAdapter.setEnable(enable);
                mTypeAdapter.notifyDataSetChanged();
            }
        });

    }


    /**
     * Action start: after get versions
     * hasHigh Confirm action OTA or MeshOTA
     * hasLow Confirm OTA needed
     */
    private void start() {
        // 1. 判断是否有选择
        if (mTypeAdapter.getSelectPosition() == -1) {
            showToast("select a type first!");
            return;
        }
        MeshDeviceType deviceType = mTypeList.get(mTypeAdapter.getSelectPosition());
        String path = deviceType.filePath;
        if (TextUtils.isEmpty(path)) {
            showToast("Select a firmware for type!");
            return;
        }

        parseFile(path);

        if (mFileVersion == null) {
            Toast.makeText(this, "File parse error!", Toast.LENGTH_SHORT).show();
            mFileVersion = null;
            deviceType.filePath = null;
            mTypeAdapter.notifyDataSetChanged();
            log("File parse error!");
            showToast("File parse error!");
        } else {
            selectType = deviceType;
            log("ota mode: meshOTA");
            startMeshService();
        }
    }

    private void startMeshService() {

        enableUI(false);

        if (!MeshOTAService.isRunning) {
            Intent serviceIntent = new Intent(this, MeshOTAService.class);
            serviceIntent.putExtra(MeshOTAService.INTENT_KEY_OTA_MODE, MeshOTAService.MODE_IDLE);
            serviceIntent.putExtra(MeshOTAService.INTENT_KEY_OTA_TYPE, selectType.type);
            serviceIntent.putExtra(MeshOTAService.INTENT_KEY_OTA_DEVICE, opDevice);
            serviceIntent.putExtra(MeshOTAService.INTENT_KEY_OTA_FIRMWARE, mFirmwareData);
            startService(serviceIntent);
        } else {
            if (MeshOTAService.getInstance() != null && MeshOTAService.getInstance().getMode() == MeshOTAService.MODE_IDLE) {
                Intent intent = new Intent();
                intent.putExtra(MeshOTAService.INTENT_KEY_OTA_MODE, MeshOTAService.MODE_IDLE);
                intent.putExtra(MeshOTAService.INTENT_KEY_OTA_TYPE, selectType.type);
                intent.putExtra(MeshOTAService.INTENT_KEY_OTA_DEVICE, opDevice);
                intent.putExtra(MeshOTAService.INTENT_KEY_OTA_FIRMWARE, mFirmwareData);
                MeshOTAService.getInstance().restart(intent);
            }
        }

        for (Light light : onlineLights) {
            log("init version : " + light.meshAddress + " -- " + light.macAddress + light.firmwareRevision);
        }
    }


    private void showPermissionDialog() {
        if (confirmDialog == null) {
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setCancelable(true);
            builder.setTitle("Warn");
            builder.setMessage("Request for overlay permission for showing OTA progress?");
            builder.setPositiveButton("Go Setting", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    Intent intent = new Intent(Settings.ACTION_MANAGE_OVERLAY_PERMISSION);
                    intent.setData(Uri.parse("package:" + getPackageName()));
                    startActivity(intent);
                }
            });

            builder.setNegativeButton("Ignore", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.dismiss();
                }
            });
            confirmDialog = builder.create();
        }
        confirmDialog.show();
    }


    // 获取本地设备OTA状态信息
    private void sendGetDeviceOtaStateCommand() {
        byte opcode = (byte) 0xC7;
        int address = 0x0000;
        byte[] params = new byte[]{0x20, 0x05};
        TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                params);
        log("getDeviceOtaState(0xC7)");
        delayHandler.removeCallbacks(deviceOtaStateTimeoutTask);
        delayHandler.postDelayed(deviceOtaStateTimeoutTask, 1000);
    }

    private Runnable deviceOtaStateTimeoutTask = new Runnable() {
        @Override
        public void run() {
            log("get device ota state timeout!");
        }
    };


    private void sendGetVersionCommand() {
        versionGetting = true;
        MeshCommandUtil.getVersion();
        log("getVersion");
        // 转发次数 * () * interval + 500
        delayHandler.removeCallbacks(getVersionTask);
        delayHandler.postDelayed(getVersionTask, 0x20 * 2 * 40 + 500);
    }

    private Runnable getVersionTask = new Runnable() {
        @Override
        public void run() {
            versionGetting = false;
            log("get version complete!");
        }
    };

    private void log(String log) {
        msgHandler.obtainMessage(MSG_LOG, log).sendToTarget();
    }

    private void toast(String msg) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
    }

    @Override
    public void finish() {
        super.finish();
//        TelinkLightService.Instance().idleMode(false);
        TelinkLog.i("OTAUpdate#onStop#removeEventListener");
        TelinkLightApplication.getApp().removeEventListener(this);
    }


    public void back() {
        finish();
    }

    @Override
    public void onBackPressed() {
        back();
    }

    private void parseFile(String filePath) {
        try {
            byte[] version = new byte[4];
            InputStream stream = new FileInputStream(filePath);
            int length = stream.available();
            mFirmwareData = new byte[length];
            stream.read(mFirmwareData);
            // 0x1c position: type
            stream.close();
            System.arraycopy(mFirmwareData, 2, version, 0, 4);
            mFileVersion = new String(version);
        } catch (Exception e) {
            mFileVersion = null;
            mFirmwareData = null;
        }

    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (resultCode == RESULT_OK) {
            mTypeAdapter.insertFileInfo(requestCode, data.getStringExtra(FileSelectActivity.KEY_RESULT));
        }
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {

            case R.id.btn_start:
//                ProgressViewManager.getInstance().createFloatWindow(this);

                start();
                break;

            case R.id.btn_check:

//                ProgressViewManager.getInstance().removeFloatWindowManager();
                sendGetDeviceOtaStateCommand();
                break;

            case R.id.btn_read_version:
                sendGetVersionCommand();
                break;
        }
    }

    /**
     * 是否可升级
     * 0:最新， 1：可升级， -n：null
     */
    public int compareVersion(String lightVersion, String newVersion) {

//        return lightVersion.equals(newVersion) ? 0 : 1;
        if (lightVersion == null || newVersion == null) {
            return 0;
        }
        int compareResult = newVersion.compareTo(lightVersion);

        return compareResult == 0 ? 0 : 1;
//        return compareResult > 1 ? 1 : compareResult;
    }

    @Override
    public void performed(Event<String> event) {
        if (event.getType().equals(NotificationEvent.GET_DEVICE_STATE)) {
            byte[] data = ((NotificationEvent) event).getArgs().params;
            if (data[0] == NotificationEvent.DATA_GET_OTA_STATE) {
                delayHandler.removeCallbacks(deviceOtaStateTimeoutTask);
                log("OTA State response--" + data[1]);
            } else if (data[0] == NotificationEvent.DATA_GET_VERSION) {
                if (!versionGetting) return;
                String version = Strings.bytesToString(Arrays.copyOfRange(data, 1, 5));
                int meshAddress = ((NotificationEvent) event).getArgs().src;
                Lights.getInstance().getByMeshAddress(meshAddress).selected = true;
                TelinkLog.w(" src:" + meshAddress + " get version success: " + version);
                log("getVersion: 0x" + Integer.toHexString(meshAddress) + "  version:" + version);

                Lights.getInstance().getByMeshAddress(meshAddress).firmwareRevision = version;
            }
        }
    }


}
