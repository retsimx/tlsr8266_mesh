/********************************************************************************************************
 * @file MainActivity.java
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
import androidx.appcompat.app.AlertDialog;
import android.app.Fragment;
import android.bluetooth.BluetoothAdapter;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.text.TextUtils;
import android.util.Log;
import android.view.MenuItem;
import android.view.Window;
import android.widget.RadioGroup;
import android.widget.Toast;

import com.google.android.material.bottomnavigation.BottomNavigationView;
import com.telink.bluetooth.LeBluetooth;
import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.event.DeviceEvent;
import com.telink.bluetooth.event.ErrorReportEvent;
import com.telink.bluetooth.event.MeshEvent;
import com.telink.bluetooth.event.NotificationEvent;
import com.telink.bluetooth.event.ServiceEvent;
import com.telink.bluetooth.light.ConnectionStatus;
import com.telink.bluetooth.light.DeviceInfo;
import com.telink.bluetooth.light.ErrorReportInfo;
import com.telink.bluetooth.light.GetAlarmNotificationParser;
import com.telink.bluetooth.light.LeAutoConnectParameters;
import com.telink.bluetooth.light.LeRefreshNotifyParameters;
import com.telink.bluetooth.light.LightAdapter;
import com.telink.bluetooth.light.MeshOTAService;
import com.telink.bluetooth.light.OnlineStatusNotificationParser;
import com.telink.bluetooth.light.Parameters;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.TelinkMeshErrorDealActivity;
import com.telink.bluetooth.light.fragments.DeviceListFragment;
import com.telink.bluetooth.light.fragments.GroupListFragment;
import com.telink.bluetooth.light.fragments.MainTestFragment;
import com.telink.bluetooth.light.fragments.TestFragment;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Lights;
import com.telink.bluetooth.light.model.Mesh;
import com.telink.bluetooth.light.util.MeshCommandUtil;
import com.telink.util.Event;
import com.telink.util.EventListener;

import java.util.List;

import androidx.annotation.NonNull;
import androidx.fragment.app.FragmentManager;

public final class MainActivity extends TelinkMeshErrorDealActivity implements EventListener<String>, BottomNavigationView.OnNavigationItemSelectedListener {

    private final static String TAG = MainActivity.class.getSimpleName();

    private static final int UPDATE_LIST = 0;
    private FragmentManager fm;
    private DeviceListFragment deviceFragment;
    private GroupListFragment groupFragment;
    private TestFragment settingFragment;

    private TelinkLightApplication mApplication;

    private int connectMeshAddress;

    @SuppressLint("HandlerLeak")
    private Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            switch (msg.what) {
                case UPDATE_LIST:
                    deviceFragment.notifyDataSetChanged();
                    break;
            }
        }
    };

    private Handler mDelayHandler = new Handler();

    private BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
                int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, 0);

                switch (state) {
                    case BluetoothAdapter.STATE_ON:
                        Log.d(TAG, "蓝牙开启");
                        TelinkLightService.Instance().idleMode(true);
                        autoConnect();
                        break;
                    case BluetoothAdapter.STATE_OFF:
                        Log.d(TAG, "蓝牙关闭");
                        break;
                }
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);

        Log.d(TAG, "onCreate");
        //TelinkLog.ENABLE = false;
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        this.setContentView(R.layout.activity_main);

        this.mApplication = (TelinkLightApplication) this.getApplication();

        this.deviceFragment = new DeviceListFragment();
        this.groupFragment = new GroupListFragment();
        this.settingFragment = new TestFragment();
        fm = getSupportFragmentManager();
        initBottomNav();

        this.mApplication.doInit();

        TelinkLog.d("-------------------------------------------");
        TelinkLog.d(Build.MANUFACTURER);
        TelinkLog.d(Build.TYPE);
        TelinkLog.d(Build.BOOTLOADER);
        TelinkLog.d(Build.DEVICE);
        TelinkLog.d(Build.HARDWARE);
        TelinkLog.d(Build.SERIAL);
        TelinkLog.d(Build.BRAND);
        TelinkLog.d(Build.DISPLAY);
        TelinkLog.d(Build.FINGERPRINT);

        TelinkLog.d(Build.PRODUCT + ":" + Build.VERSION.SDK_INT + ":" + Build.VERSION.RELEASE + ":" + Build.VERSION.CODENAME + ":" + Build.ID);

        IntentFilter filter = new IntentFilter();
        filter.addAction(BluetoothAdapter.ACTION_STATE_CHANGED);
        filter.setPriority(IntentFilter.SYSTEM_HIGH_PRIORITY - 1);
        registerReceiver(mReceiver, filter);
    }


    private void initBottomNav() {
        BottomNavigationView bottomNavigationView = findViewById(R.id.bottom_nav);
        bottomNavigationView.setOnNavigationItemSelectedListener(this);
        fm.beginTransaction()
                .add(R.id.content, deviceFragment).add(R.id.content, groupFragment).add(R.id.content, settingFragment)
                .show(deviceFragment).hide(groupFragment).hide(settingFragment)
                .commit();
    }


    @Override
    protected void onStart() {
        super.onStart();
        Log.d(TAG, "onStart");
        // 监听各种事件
        this.mApplication.addEventListener(DeviceEvent.STATUS_CHANGED, this);
        this.mApplication.addEventListener(NotificationEvent.ONLINE_STATUS, this);
        this.mApplication.addEventListener(NotificationEvent.GET_ALARM, this);
        this.mApplication.addEventListener(NotificationEvent.GET_DEVICE_STATE, this);
        this.mApplication.addEventListener(ServiceEvent.SERVICE_CONNECTED, this);
        this.mApplication.addEventListener(MeshEvent.OFFLINE, this);

        this.mApplication.addEventListener(ErrorReportEvent.ERROR_REPORT, this);

        this.autoConnect();
    }

    @Override
    protected void onRestart() {
        super.onRestart();
        Log.d(TAG, "onRestart");
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.d(TAG, "onPause");
    }

    @Override
    protected void onResume() {
        super.onResume();
        //检查是否支持蓝牙设备
        if (!LeBluetooth.getInstance().isSupport(getApplicationContext())) {
            Toast.makeText(this, "ble not support", Toast.LENGTH_SHORT).show();
            this.finish();
            return;
        }

        if (!LeBluetooth.getInstance().isEnabled()) {
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setMessage("开启蓝牙，体验智能灯!");
            builder.setNeutralButton("cancel", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    finish();
                }
            });
            builder.setNegativeButton("enable", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    LeBluetooth.getInstance().enable(getApplicationContext());
                }
            });
            builder.show();
        }

        DeviceInfo deviceInfo = this.mApplication.getConnectDevice();

        if (deviceInfo != null) {
            this.connectMeshAddress = this.mApplication.getConnectDevice().meshAddress & 0xFF;
        }

        Log.d(TAG, "onResume");
    }

    public static void getAlarm() {
        TelinkLightService.Instance().sendCommandNoResponse((byte) 0xE6, 0x0000, new byte[]{0x10, (byte) 0x00});
    }

    @Override
    protected void onStop() {
        super.onStop();
        Log.d(TAG, "onStop");
        if (TelinkLightService.Instance() != null)
            TelinkLightService.Instance().disableAutoRefreshNotify();
    }


    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.d(TAG, "onDestroy");
        unregisterReceiver(mReceiver);
        this.mApplication.doDestroy();
        this.mDelayHandler.removeCallbacksAndMessages(null);
        //移除事件
        this.mApplication.removeEventListener(this);
        Lights.getInstance().clear();
    }


    /**
     * 自动重连
     */
    private void autoConnect() {
        if (TelinkLightService.Instance() != null && !confirmWaiting) {

            if (TelinkLightService.Instance().getMode() != LightAdapter.MODE_AUTO_CONNECT_MESH) {


                if (this.mApplication.isEmptyMesh())
                    return;

//                Lights.getInstance().clear();
                this.mApplication.refreshLights();


                this.deviceFragment.notifyDataSetChanged();

                Mesh mesh = this.mApplication.getMesh();

                if (TextUtils.isEmpty(mesh.name) || TextUtils.isEmpty(mesh.password)) {
                    TelinkLightService.Instance().idleMode(true);
                    return;
                }

                //自动重连参数
                LeAutoConnectParameters connectParams = Parameters.createAutoConnectParameters();
                connectParams.setMeshName(mesh.name);
                connectParams.setPassword(mesh.password);
                connectParams.autoEnableNotification(true);

                // 之前是否有在做MeshOTA操作，是则继续
                if (mesh.isOtaProcessing()) {
                    connectParams.setConnectMac(mesh.otaDevice.mac);
//                    saveLog("Action: AutoConnect:" + mesh.otaDevice.mac);
                } else {
//                    saveLog("Action: AutoConnect:NULL");
                }
                connectParams.set(Parameters.PARAM_OFFLINE_TIMEOUT_SECONDS, 40);
                //自动重连
                TelinkLightService.Instance().autoConnect(connectParams);

                //刷新Notify参数, 重新回到主页时不刷新
                LeRefreshNotifyParameters refreshNotifyParams = Parameters.createRefreshNotifyParameters();
                refreshNotifyParams.setRefreshRepeatCount(2);
                refreshNotifyParams.setRefreshInterval(2000);
                //开启自动刷新Notify
                TelinkLightService.Instance().autoRefreshNotify(refreshNotifyParams);
            }


        }
    }

    private void onDeviceStatusChanged(DeviceEvent event) {

        DeviceInfo deviceInfo = event.getArgs();

        switch (deviceInfo.status) {
            case LightAdapter.STATUS_LOGIN:
                this.connectMeshAddress = this.mApplication.getConnectDevice().meshAddress;
//                this.showToast("login success");
                if (TelinkLightService.Instance().getMode() == LightAdapter.MODE_AUTO_CONNECT_MESH) {
                    mHandler.postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            TelinkLightService.Instance().sendCommandNoResponse((byte) 0xE4, 0xFFFF, new byte[]{});
                        }
                    }, 3 * 1000);
                }

                if (TelinkLightApplication.getApp().getMesh().isOtaProcessing() && !MeshOTAService.isRunning) {
                    // 获取本地设备OTA状态信息
                    MeshCommandUtil.getDeviceOTAState();
                }
                break;
            case LightAdapter.STATUS_CONNECTING:
//                this.showToast("login");
                break;
            case LightAdapter.STATUS_LOGOUT:
//                this.showToast("disconnect");
                onLogout();
                break;

            case LightAdapter.STATUS_ERROR_N:
                onNError(event);
            default:
                break;
        }

    }

    private void onNError(final DeviceEvent event) {

        TelinkLightService.Instance().idleMode(true);
        TelinkLog.d("DeviceScanningActivity#onNError");

        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setMessage("当前环境:Android7.0!连接重试:" + " 3次失败!");
        builder.setNegativeButton("confirm", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });
        builder.setCancelable(false);
        builder.show();
    }

    private void onLogout() {
        List<Light> lights = Lights.getInstance().get();
        for (Light light : lights) {
            light.connectionStatus = ConnectionStatus.OFFLINE;
        }
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                deviceFragment.notifyDataSetChanged();
            }
        });
    }

    private void onAlarmGet(NotificationEvent notificationEvent) {
        GetAlarmNotificationParser.AlarmInfo info = GetAlarmNotificationParser.create().parse(notificationEvent.getArgs());
        if (info != null)
            TelinkLog.d("alarm info index: " + info.index);
    }


    /**
     * 处理{@link NotificationEvent#ONLINE_STATUS}事件
     */
    private void onOnlineStatusNotify(NotificationEvent event) {

        TelinkLog.i("MainActivity#onOnlineStatusNotify#Thread ID : " + Thread.currentThread().getId());
        List<OnlineStatusNotificationParser.DeviceNotificationInfo> notificationInfoList;
        //noinspection unchecked
        notificationInfoList = (List<OnlineStatusNotificationParser.DeviceNotificationInfo>) event.parse();

        if (notificationInfoList == null || notificationInfoList.size() <= 0)
            return;

        /*if (this.deviceFragment != null) {
            this.deviceFragment.onNotify(notificationInfoList);
        }*/

        for (OnlineStatusNotificationParser.DeviceNotificationInfo notificationInfo : notificationInfoList) {

            int meshAddress = notificationInfo.meshAddress;
            int brightness = notificationInfo.brightness;

            Light light = this.deviceFragment.getDevice(meshAddress);

            if (light == null) {
                if (notificationInfo.connectionStatus == ConnectionStatus.OFFLINE) continue;

                light = new Light();
                this.deviceFragment.addDevice(light);
            }

            light.meshAddress = meshAddress;
            light.brightness = brightness;
            light.connectionStatus = notificationInfo.connectionStatus;

            if (light.meshAddress == this.connectMeshAddress) {
                light.textColor = R.color.theme_positive_color;
            } else {
                light.textColor = R.color.black;
            }
        }

        mHandler.obtainMessage(UPDATE_LIST).sendToTarget();
    }

    private void onServiceConnected(ServiceEvent event) {
        this.autoConnect();
    }

    private void onServiceDisconnected(ServiceEvent event) {

    }

    AlertDialog.Builder mTimeoutBuilder;
    boolean confirmWaiting = false;

    private void onMeshOffline(MeshEvent event) {
        TelinkLog.w("auto connect offline");
        List<Light> lights = Lights.getInstance().get();
        for (Light light : lights) {
            light.connectionStatus = ConnectionStatus.OFFLINE;
        }
        this.deviceFragment.notifyDataSetChanged();

        if (TelinkLightApplication.getApp().getMesh().isOtaProcessing()) {
            confirmWaiting = true;
            TelinkLightService.Instance().idleMode(true);
            if (mTimeoutBuilder == null) {
                mTimeoutBuilder = new AlertDialog.Builder(this);
                mTimeoutBuilder.setTitle("AutoConnect Fail");
                mTimeoutBuilder.setMessage("Connect device:" + TelinkLightApplication.getApp().getMesh().otaDevice.mac + " Fail, Quit? \nYES: quit MeshOTA process, NO: retry");
                mTimeoutBuilder.setNeutralButton("Quit", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        confirmWaiting = false;
                        Mesh mesh = TelinkLightApplication.getApp().getMesh();
                        mesh.otaDevice = null;
                        mesh.saveOrUpdate(MainActivity.this);

                        TelinkLightService.Instance().setAutoConnectMac(null);

                        autoConnect();
                        dialog.dismiss();
                    }
                });
                mTimeoutBuilder.setNegativeButton("Retry", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        confirmWaiting = false;
                        autoConnect();
                        dialog.dismiss();
                    }
                });
                mTimeoutBuilder.setCancelable(false);
            }
            mTimeoutBuilder.show();
        }
    }


    /**
     * 事件处理方法
     *
     * @param event
     */
    @Override
    public void performed(Event<String> event) {
        switch (event.getType()) {
            case NotificationEvent.ONLINE_STATUS:
                this.onOnlineStatusNotify((NotificationEvent) event);
                break;

            case NotificationEvent.GET_ALARM:
//                this.onAlarmGet((NotificationEvent) event);
                break;
            case DeviceEvent.STATUS_CHANGED:
                this.onDeviceStatusChanged((DeviceEvent) event);
                break;
            case MeshEvent.OFFLINE:
                this.onMeshOffline((MeshEvent) event);
                break;
            case ServiceEvent.SERVICE_CONNECTED:
                this.onServiceConnected((ServiceEvent) event);
                break;
            case ServiceEvent.SERVICE_DISCONNECTED:
                this.onServiceDisconnected((ServiceEvent) event);
                break;
            case NotificationEvent.GET_DEVICE_STATE:
                onNotificationEvent((NotificationEvent) event);
                break;

            case ErrorReportEvent.ERROR_REPORT:
                ErrorReportInfo info = ((ErrorReportEvent) event).getArgs();
                TelinkLog.d("MainActivity#performed#ERROR_REPORT: " + " stateCode-" + info.stateCode
                        + " errorCode-" + info.errorCode
                        + " deviceId-" + info.deviceId);
                break;
        }
    }

    @Override
    protected void onLocationEnable() {
        autoConnect();
    }


    private void onNotificationEvent(NotificationEvent event) {
        if (!foreground) return;
        // 解析版本信息

        byte[] data = event.getArgs().params;

        if (data[0] == NotificationEvent.DATA_GET_MESH_OTA_PROGRESS) {
            /*if (!MeshOTAService.isRunning) {
                Intent serviceIntent = new Intent(this, MeshOTAService.class);
                serviceIntent.putExtra(MeshOTAService.INTENT_KEY_INIT_MODE, MeshOTAService.MODE_CONTINUE_MESH_OTA);
                startService(serviceIntent);
            }*/
        } else if (data[0] == NotificationEvent.DATA_GET_OTA_STATE) {
            if (TelinkLightApplication.getApp().getMesh().isOtaProcessing() && !MeshOTAService.isRunning) {
                if (data[1] == NotificationEvent.OTA_STATE_MASTER) {
                    Intent serviceIntent = new Intent(this, MeshOTAService.class);
                    serviceIntent.putExtra(MeshOTAService.INTENT_KEY_INIT_MODE, MeshOTAService.MODE_CONTINUE_MESH_OTA);
                    startService(serviceIntent);
                } else if (data[1] == NotificationEvent.OTA_STATE_COMPLETE) {
                    Mesh mesh = TelinkLightApplication.getApp().getMesh();
                    mesh.otaDevice = null;
                    mesh.saveOrUpdate(this);

                    Intent serviceIntent = new Intent(this, MeshOTAService.class);
                    serviceIntent.putExtra(MeshOTAService.INTENT_KEY_INIT_MODE, MeshOTAService.MODE_COMPLETE);
                    startService(serviceIntent);
                } else if (data[1] == NotificationEvent.OTA_STATE_IDLE) {
                    MeshCommandUtil.sendStopMeshOTACommand();
                }
            }
        }


    }

    @Override
    public boolean onNavigationItemSelected(@NonNull MenuItem item) {
        switch (item.getItemId()) {
            case R.id.item_device:
                fm.beginTransaction().hide(groupFragment).hide(settingFragment).show(deviceFragment).commit();
                break;
            case R.id.item_group:
                fm.beginTransaction().hide(deviceFragment).hide(settingFragment).show(groupFragment).commit();
                break;
            case R.id.item_setting:
                fm.beginTransaction().hide(deviceFragment).hide(groupFragment).show(settingFragment).commit();
                break;

        }
        return true;
    }
}
