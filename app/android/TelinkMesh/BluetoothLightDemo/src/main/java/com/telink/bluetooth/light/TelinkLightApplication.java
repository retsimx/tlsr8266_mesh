/********************************************************************************************************
 * @file TelinkLightApplication.java
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

import android.content.Intent;
import android.os.Build;
import android.text.TextUtils;
import android.widget.Toast;

import com.telink.TelinkApplication;
import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.event.LogEvent;
import com.telink.bluetooth.light.activity.TempTestActivity;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Lights;
import com.telink.bluetooth.light.model.LogInfo;
import com.telink.bluetooth.light.model.Mesh;
import com.telink.bluetooth.light.model.SharedPreferencesHelper;
import com.telink.bluetooth.light.util.FileSystem;
import com.telink.crypto.AES;
import com.telink.util.Arrays;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;


public final class TelinkLightApplication extends TelinkApplication {

    private Mesh mesh;
    private List<LogInfo> logInfoList = new ArrayList<>();
    private static TelinkLightApplication thiz;
//    private List<String> macFilters = new ArrayList<>();

    private Toast toast;
    private int onlineCount = 0;

    private TempTestActivity.TestInput testInput;

    public TempTestActivity.TestInput getTestInput() {
        return testInput;
    }

    public void setTestInput(TempTestActivity.TestInput testInput) {
        this.testInput = testInput;
    }

    @Override
    public void onCreate() {
        super.onCreate();
        //this.doInit();
        thiz = this;
        toast = Toast.makeText(this, "", Toast.LENGTH_SHORT);
        closePErrorDialog();
        testAesEnc();
//        BlockCanary.install(this, new AppBlockCanaryContext()).start();
    }

    private void testAesEnc(){
        byte[] sessionKey = new byte[]{46, -90, 7, 35, -68, -15, -112, -11, 9, -62, -22, 72, -126, 30, 83, 62};
        byte[] nonce = new byte[]{1, 72, 8, 34, 1, -49, 77, -65};
        byte[] cmdData = new byte[]{-49, 77, -65, 0, 0, 1, 0, -35, 17, 2, 8, 1, 0, 0, 0, 0, 0, 0, 0, 0};
        byte[] enc = AES.encrypt(sessionKey, nonce, cmdData);
        TelinkLog.d("enc data: " + java.util.Arrays.toString(enc));

    }


    private void closePErrorDialog() {
        if (Build.VERSION.SDK_INT <= 27) {
            return;
        }
        try {
            Class aClass = Class.forName("android.content.pm.PackageParser$Package");
            Constructor declaredConstructor = aClass.getDeclaredConstructor(String.class);
            declaredConstructor.setAccessible(true);
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            Class cls = Class.forName("android.app.ActivityThread");
            Method declaredMethod = cls.getDeclaredMethod("currentActivityThread");
            declaredMethod.setAccessible(true);
            Object activityThread = declaredMethod.invoke(null);
            Field mHiddenApiWarningShown = cls.getDeclaredField("mHiddenApiWarningShown");
            mHiddenApiWarningShown.setAccessible(true);
            mHiddenApiWarningShown.setBoolean(activityThread, true);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public int getOnlineCount() {
        return onlineCount;
    }

    public void setOnlineCount(int onlineCount) {
        this.onlineCount = onlineCount;
    }

    public static TelinkLightApplication getApp() {
        return thiz;
    }

    @Override
    public void doInit() {

        String fileName = "telink-";
        fileName += System.currentTimeMillis();
        fileName += ".log";
        TelinkLog.LOG2FILE_ENABLE = false;
//        TelinkLog.onCreate(fileName);
        super.doInit();
        //AES.Security = true;

        String name = SharedPreferencesHelper.getMeshName(this);
        String pwd = SharedPreferencesHelper.getMeshPassword(this);

        if (!TextUtils.isEmpty(name) && !TextUtils.isEmpty(pwd)) {
            if (FileSystem.exists(this, name + "." + pwd)) {
                Mesh mesh = (Mesh) FileSystem.readAsObject(this, name + "." + pwd);
                setupMesh(mesh);
            }
        } else {
            Mesh mesh = getMesh();
            setupMesh(mesh);
        }


/*

        if (FileSystem.exists("telink.meshs")) {
            this.mesh = (Mesh) FileSystem.readAsObject("telink.meshs");
        }
*/


        //启动LightService
        this.startLightService(TelinkLightService.class);
    }

    @Override
    public void doDestroy() {
        TelinkLog.onDestroy();
        super.doDestroy();
    }


    public Mesh getMesh() {
        if (this.mesh == null) {
            this.mesh = new Mesh();
            this.mesh.name = "telink_mesh1";
            this.mesh.password = "123";

//            this.mesh.factoryName = "telink_mesh1";
//            this.mesh.factoryPassword = "123";
        }
        return this.mesh;
    }

    public void setupMesh(Mesh mesh) {
        this.mesh = mesh;
        refreshLights();
    }

    public void refreshLights() {
        if (mesh != null && mesh.devices != null) {
            Lights.getInstance().clear();
            Lights.getInstance().add(mesh.devices);
            for (Light light : Lights.getInstance().get()) {
                light.connectionStatus = ConnectionStatus.OFFLINE;
                light.textColor = R.color.black;
            }
            /*Light light;
            for (com.telink.bluetooth.light.model.DeviceInfo deviceInfo : mesh.devices) {
                light = new Light();
                light.macAddress = deviceInfo.macAddress;
                light.meshAddress = deviceInfo.meshAddress;
                light.brightness = 0;
                light.connectionStatus = ConnectionStatus.OFFLINE;
                light.textColor = this.getResources().getColorStateList(
                        R.color.black);
                light.updateIcon();

                Lights.getInstance().add(light);
            }*/
        }
    }

    public List<LogInfo> getLogInfoList() {
        return logInfoList;
    }

    public boolean isEmptyMesh() {

        return this.mesh == null || TextUtils.isEmpty(mesh.name) || TextUtils.isEmpty(mesh.password);
    }

    /**********************************************
     * Log api
     **********************************************/

//    SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
    SimpleDateFormat format = new SimpleDateFormat("HH:mm:ss.S");

    @Override
    public void saveLog(String action) {
        String time = format.format(Calendar.getInstance().getTimeInMillis());
        this.logInfoList.add(new LogInfo("TelLog", action, 0));
        dispatchEvent(new LogEvent(this, LogEvent.LOG_REPORT));
        TelinkLog.w("SaveLog: " + action);
    }

    @Override
    protected void onNotify(NotificationInfo notifyInfo) {
        saveLog(String.format("onNotify: opcode--%02X vendorId--%04X src--%04X params--%s",
                notifyInfo.opcode,
                notifyInfo.vendorId,
                notifyInfo.src,
                Arrays.bytesToHexString(notifyInfo.params, "")));

        // dispatch notify raw data event
        /*NotificationEvent rawEvent = NotificationEvent.newInstance(this, NotificationEvent.RAW, notifyInfo);
        this.dispatchEvent(rawEvent);*/

        super.onNotify(notifyInfo);
    }

    public void saveLogInFile(String fileName, String logInfo) {
        if (FileSystem.writeAsString(fileName + ".txt", logInfo)) {
            showToast("save success --" + fileName);
        }
    }


    public void showToast(CharSequence s) {

        if (this.toast != null) {
            this.toast.setView(this.toast.getView());
            this.toast.setDuration(Toast.LENGTH_SHORT);
            this.toast.setText(s);
            this.toast.show();
        }
    }

    /**
     * super method
     *
     * @param intent
     */
    @Override
    protected void onLeScan(Intent intent) {
        super.onLeScan(intent);
        DeviceInfo deviceInfo = intent.getParcelableExtra(LightService.EXTRA_DEVICE);
        saveLog("scan: " + deviceInfo.macAddress);
    }

    @Override
    protected void onStatusChanged(Intent intent) {
        super.onStatusChanged(intent);
        DeviceInfo deviceInfo = intent.getParcelableExtra(LightService.EXTRA_DEVICE);
        saveLog("device " + deviceInfo.macAddress + " " + getDeviceState(deviceInfo.status));
    }

    private String getDeviceState(int connectionStatus) {
        switch (connectionStatus) {
            case LightAdapter.STATUS_CONNECTING:
                return "STATUS_CONNECTING";
            case LightAdapter.STATUS_CONNECTED:
                return "STATUS_CONNECTED";
            case LightAdapter.STATUS_LOGINING:
                return "STATUS_LOGINING";
            case LightAdapter.STATUS_LOGIN:
                return "STATUS_LOGIN_SUCCESS";
            case LightAdapter.STATUS_LOGOUT:
                return "LOGIN_FAILURE | CONNECT_FAILURE";
            case LightAdapter.STATUS_UPDATE_MESH_COMPLETED:
            case LightAdapter.STATUS_UPDATING_MESH:
            case LightAdapter.STATUS_UPDATE_MESH_FAILURE:
            case LightAdapter.STATUS_UPDATE_ALL_MESH_COMPLETED:
            case LightAdapter.STATUS_GET_LTK_COMPLETED:
            case LightAdapter.STATUS_GET_LTK_FAILURE:
            case LightAdapter.STATUS_MESH_OFFLINE:
            case LightAdapter.STATUS_MESH_SCAN_COMPLETED:
            case LightAdapter.STATUS_MESH_SCAN_TIMEOUT:
            case LightAdapter.STATUS_OTA_COMPLETED:
            case LightAdapter.STATUS_OTA_FAILURE:
            case LightAdapter.STATUS_OTA_PROGRESS:
            case LightAdapter.STATUS_GET_FIRMWARE_COMPLETED:
            case LightAdapter.STATUS_GET_FIRMWARE_FAILURE:
            case LightAdapter.STATUS_DELETE_COMPLETED:
            case LightAdapter.STATUS_DELETE_FAILURE:
            default:
                return "OTHER";
        }
    }
}
