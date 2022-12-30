/********************************************************************************************************
 * @file LightService.java
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
/*
 * Copyright (C) 2015 The Telink Bluetooth Light Project
 *
 */
package com.telink.bluetooth.light;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;

import com.telink.bluetooth.Command;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

/**
 * LightService是一个抽象类,封装了扫描加灯,自动重连,设备控制等方法.
 *
 * @see LightAdapter
 * @see LeScanParameters
 * @see LeAutoConnectParameters
 * @see LeUpdateParameters
 * @see LeOtaParameters
 * @see LeRefreshNotifyParameters
 */
public abstract class LightService extends Service implements
        LightAdapter.Callback, AdvanceStrategy.Callback {

    public static final String ACTION_LE_SCAN = "com.telink.bluetooth.light.ACTION_LE_SCAN";
    public static final String ACTION_SCAN_COMPLETED = "com.telink.bluetooth.light.ACTION_SCAN_COMPLETED";
    public static final String ACTION_LE_SCAN_TIMEOUT = "com.telink.bluetooth.light.ACTION_LE_SCAN_TIMEOUT";
    public static final String ACTION_NOTIFICATION = "com.telink.bluetooth.light.ACTION_NOTIFICATION";
    public static final String ACTION_STATUS_CHANGED = "com.telink.bluetooth.light.ACTION_STATUS_CHANGED";
    public static final String ACTION_UPDATE_MESH_COMPLETED = "com.telink.bluetooth.light.ACTION_UPDATE_MESH_COMPLETED";
    public static final String ACTION_OFFLINE = "com.telink.bluetooth.light.ACTION_OFFLINE";
    public static final String ACTION_ERROR = "com.telink.bluetooth.light.ACTION_ERROR";
    public static final String ACTION_ERROR_REPORT = "com.telink.bluetooth.light.ACTION_ERROR_REPORT";

    public static final String EXTRA_MODE = "com.telink.bluetooth.light.EXTRA_MODE";
    public static final String EXTRA_DEVICE = "com.telink.bluetooth.light.EXTRA_DEVICE";
    public static final String EXTRA_NOTIFY = "com.telink.bluetooth.light.EXTRA_NOTIFY";
    public static final String EXTRA_ERROR_CODE = "com.telink.bluetooth.light.EXTRA_ERROR_CODE";

    public static final String EXTRA_ERROR_REPORT_INFO = "com.telink.bluetooth.light.EXTRA_ERROR_REPORT_INFO";

    protected LightAdapter mAdapter;
    protected IBinder mBinder;

    public LightAdapter getAdapter() {
        return mAdapter;
    }

    @Override
    public IBinder onBind(Intent intent) {
        return this.mBinder;
    }

    @Override
    public void onCreate() {
        super.onCreate();
        AdvanceStrategy.getDefault().setCallback(this);
        AdvanceStrategy.getDefault().onStart();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        AdvanceStrategy.getDefault().setCallback(null);
        AdvanceStrategy.getDefault().onStop();
        if (this.mAdapter != null) {
            this.mAdapter.stop();
        }
    }

    /**
     * 启动扫描
     * <p>通常{@link LightService#startScan(Parameters)}方法会和其他方法组合使用.
     * <p>比如扫描添加设备{@link LightService#updateMesh(Parameters)},扫描并ota{@link LightService#startOta(byte[])}.
     * <p><strong>示例.</strong>
     * <pre>
     * {@code
     * LeScanParameters params = LeScanParameters.create();
     * params.setMeshName(Manufacture.getDefault().getFactoryName());
     * params.setOutOfMeshName("out_of_mesh");
     * params.setTimeoutSeconds(5);
     * params.setScanMode(true);
     * SmartLightService.getInstance().startScan(params);
     * }</pre>
     * <p>
     * <p>启动扫描后,通过{@link LightService#ACTION_LE_SCAN}动作广播发现到的设备,广播参数{@link LightService#EXTRA_DEVICE}
     * 表示发现的设备{@link DeviceInfo}.
     * <p>当扫描超过指定的时间,会发送{@link LightService#ACTION_LE_SCAN_TIMEOUT}动作的广播.
     *
     * @param params {@link LeScanParameters}类型
     * @see LeScanParameters
     * @see com.telink.bluetooth.event.LeScanEvent
     * @see com.telink.bluetooth.event.DeviceEvent
     * @see com.telink.bluetooth.event.MeshEvent
     */
    public void startScan(Parameters params) {

        if (this.mAdapter == null)
            return;

        this.mAdapter.startScan(params, this);
    }

    /**
     * 自动重连
     * <p>当启动自动重连时,如果当前连接的设备连接断开了,会自动扫描一个设备进行连接.
     * <p>通常{@link LightService#autoConnect(Parameters)}会和{@link LightService#autoRefreshNotify(Parameters)}成对出现.
     * <p>连接设备的状态会通过{@link LightService#ACTION_STATUS_CHANGED}动作广播出来</p>
     * <p><strong>示例</strong>.
     * <pre>
     * {@code
     * SmartLightService service = SmartLightService.getInstance();
     * Mesh mesh = this.mApp.getCurrentMesh();
     * LeAutoConnectParameters params = LeAutoConnectParameters.create();
     * params.setMeshName(mesh.getMeshName());
     * params.setPassword(mesh.getPassword());
     * params.autoEnableNotification(true);
     * service.autoConnect(params);
     * ......
     * LeRefreshNotifyParameters refreshNotifyParams = LeRefreshNotifyParameters.create();
     * refreshNotifyParams.setRefreshRepeatCount(2);
     * refreshNotifyParams.setRefreshInterval(1000);
     * service.autoRefreshNotify(refreshNotifyParams);
     * }</pre>
     *
     * @param params {@link LeAutoConnectParameters}类型参数
     * @see LeAutoConnectParameters
     * @see com.telink.bluetooth.event.DeviceEvent
     * @see com.telink.bluetooth.event.MeshEvent
     */
    public void autoConnect(Parameters params) {

        if (this.mAdapter == null)
            return;

        this.mAdapter.autoConnect(params, this);
    }

    public void setAutoConnectMac(String mac) {
        if (this.mAdapter == null)
            return;
        this.mAdapter.setAutoConnectMac(mac);
    }

    /**
     * 更新设备的名称,密码,LTK
     * <p><strong>示例.</strong>
     * <pre>
     * {@code
     * LeUpdateParameters params = LeUpdateParameters.create();
     * params.setOldMeshName(Manufacture.getDefault().getFactoryName());
     * params.setOldPassword(Manufacture.getDefault().getFactoryPassword());
     * params.setNewMeshName(this.mApp.getCurrentMesh().getMeshName());
     * params.setNewPassword(this.mApp.getCurrentMesh().getPassword());
     * params.setLtk(this.mApp.getCurrentMesh().getLtk());
     * params.setUpdateDeviceList(deviceInfo);
     * SmartLightService.getInstance().idleMode(true);
     * SmartLightService.getInstance().updateMesh(params);
     * }</pre>
     *
     * @param params {@link LeUpdateParameters}类型
     * @see LeUpdateParameters
     * @see com.telink.bluetooth.event.DeviceEvent
     */
    public void updateMesh(Parameters params) {

        if (this.mAdapter == null)
            return;

        this.mAdapter.updateMesh(params, this);
    }

    /**
     * 空闲模式
     *
     * @param disconnect 是否断开当前的连接
     */
    public void idleMode(boolean disconnect) {

        if (this.mAdapter == null)
            return;
        this.mAdapter.idleMode(disconnect);
    }

    /**
     * 向设备发送控制命令
     * <p>使用{@link android.bluetooth.BluetoothGattCharacteristic#WRITE_TYPE_NO_RESPONSE}方式发送.
     * <p><strong>示例.</strong>
     * <pre>
     * {@code
     * LightService service = ...
     * service.sendCommandNoResponse((byte) 0xD0, 0xFFFF, new byte[]{0x01, 0x00, 0x00});
     * }
     * </pre>
     *
     * @param opcode  操作码
     * @param address 设备的地址
     * @param params  参数
     * @return
     */
    public boolean sendCommandNoResponse(byte opcode, int address, byte[] params) {
        return this.sendCommandNoResponse(opcode, address, params, null, 0, false);
    }

    public boolean sendCommandNoResponseImmediate(byte opcode, int address, byte[] params) {
        return this.sendCommandNoResponse(opcode, address, params, null, 0, true);
    }

    public boolean sendCommandNoResponse(byte opcode, int address, byte[] params, boolean immediate) {
        return this.sendCommandNoResponse(opcode, address, params, null, 0, immediate);
    }

    public boolean sendCommandNoResponse(byte opcode, int address, byte[] params, Object tag, boolean immediate) {
        return this.sendCommandNoResponse(opcode, address, params, tag, 0, immediate);
    }

    public boolean sendCommandNoResponse(byte opcode, int address, byte[] params, int delay, boolean immediate) {
        return this.sendCommandNoResponse(opcode, address, params, null, delay, immediate);
    }

    public boolean sendCommandNoResponse(byte opcode, int address, byte[] params, Object tag, int delay, boolean immediate) {
        return this.mAdapter != null && AdvanceStrategy.getDefault().postCommand(opcode, address, params, delay, tag, true, immediate);
//        return this.mAdapter.sendCommandNoResponse(opcode, address, params, tag, delay);
    }

    public void autoRefreshNotify(boolean enable, Parameters params) {

        if (this.mAdapter == null)
            return;

        if (enable) {
            this.mAdapter.enableAutoRefreshNotify(params);
        } else {
            this.mAdapter.disableAutoRefreshNotify();
        }
    }

    /**
     * 开启自动刷新网络通知
     * <p>{@link LightService#autoRefreshNotify(Parameters)}通常会和自动重连{@link LightService#autoConnect(Parameters)}配合使用.
     * <p>此方法主要是解决偶尔的设备状态收不到的问题.当启动自动刷新网络,会自动想直接连接的设备发送命令获取当前网络内的设备状态信息
     * <pre>
     * {@code
     * LeRefreshNotifyParameters refreshNotifyParams = LeRefreshNotifyParameters.create();
     * refreshNotifyParams.setRefreshRepeatCount(2);
     * refreshNotifyParams.setRefreshInterval(1000);
     * service.autoRefreshNotify(refreshNotifyParams);
     * }</pre>
     *
     * @param params {@link LeRefreshNotifyParameters}类型.
     * @see LeRefreshNotifyParameters
     */
    public void autoRefreshNotify(Parameters params) {

        if (this.mAdapter == null)
            return;
        this.mAdapter.enableAutoRefreshNotify(params);
    }

    /**
     * 关闭自动刷新网络通知
     */
    public void disableAutoRefreshNotify() {
        if (this.mAdapter == null)
            return;
        this.mAdapter.disableAutoRefreshNotify();
    }

    /**
     * 开启通知
     * <p>默认不会开启.
     */
    public void enableNotification() {
        if (this.mAdapter == null)
            return;
        this.mAdapter.enableNotification();
    }

    /**
     * 关闭通知
     */
    public void disableNotification() {
        if (this.mAdapter == null)
            return;
        this.mAdapter.disableNotification();
    }

    /**
     * 刷新指定的的设备状态
     * <p/>
     * <pre>
     * {@code
     * LightService service = ...;
     * //刷新0x22,0x11号设备状态
     * service.updateNotification(new byte[]{0x01, 0x22, 0x11});
     * }</pre>
     *
     * @param params 设备地址数组
     */
    public void updateNotification(byte[] params) {
        if (this.mAdapter == null)
            return;
        this.mAdapter.updateNotification(params);
    }

    /**
     * 刷新所有设备状态
     */
    public void updateNotification() {
        if (this.mAdapter == null)
            return;
        this.mAdapter.updateNotification();
    }

    /**
     * 获取当前工作模式
     *
     * @return
     * @see LightAdapter#MODE_SCAN_MESH
     * @see LightAdapter#MODE_AUTO_CONNECT_MESH
     * @see LightAdapter#MODE_UPDATE_MESH
     * @see LightAdapter#MODE_IDLE
     * @see LightAdapter#MODE_OTA
     */
    public int getMode() {
        return this.mAdapter != null ? this.mAdapter.getMode() : -1;
    }

    /**
     * 是否已经登录到设备
     *
     * @return
     * @see LightAdapter#isLogin()
     */
    public boolean isLogin() {
        return this.mAdapter != null && this.mAdapter.isLogin();
    }

    /**
     * 连接到指定的设备
     * <p>要连接到指定的设备需要先调用{@link LightService#startScan(Parameters)}扫描设备.
     *
     * @param mac            设备地址
     * @param timeoutSeconds 连接超时时间
     * @return
     */
    public boolean connect(String mac, int timeoutSeconds) {
        return this.mAdapter != null && this.mAdapter.connect(mac, timeoutSeconds);
    }

    public void disconnect() {
        if (this.mAdapter != null)
            this.mAdapter.disconnect();
    }

    /**
     * 登录到当前连接的设备
     *
     * @param meshName
     * @param password
     * @return
     */
    public boolean login(byte[] meshName, byte[] password) {
        return this.mAdapter != null && this.mAdapter.login(meshName, password);
    }

    public boolean resetByMesh(String meshName, String password) {
        return this.mAdapter != null && this.mAdapter.resetByMesh(meshName, password, null);
    }

    /**
     * 开始ota
     * <p>OTA必要的流程,1.扫描到设备 2.连接设备并获取firmware版本作比较 3.开始ota
     *
     * @param parameters {@link LeOtaParameters}类型
     * @see LeOtaParameters
     */
    public void startOta(Parameters parameters) {
        if (this.mAdapter == null)
            return;
        this.mAdapter.startOta(parameters, this);
    }

    /**
     * 当前登录的设备直接开始ota
     *
     * @param firmware firmware的数据
     * @return
     */
    public boolean startOta(byte[] firmware) {
        return this.mAdapter != null && this.mAdapter.startOta(firmware);
    }

    /**
     * 获取当前连接设备的firmware信息
     * <p>firmware获取成功或失败会通过{@link LightService#ACTION_STATUS_CHANGED}动作广播出去.
     * 广播参数{@link LightService#EXTRA_DEVICE}是个{@link DeviceInfo}类型,
     * 通过{@link DeviceInfo#status}即可获取firmware是否获取成功
     *
     * @return
     * @see LightAdapter#STATUS_GET_FIRMWARE_COMPLETED
     * @see LightAdapter#STATUS_GET_FIRMWARE_FAILURE
     */
    public boolean getFirmwareVersion() {
        return this.mAdapter != null && this.mAdapter.getFirmwareVersion();
    }

    public boolean getSubversion() {
        return this.mAdapter != null && this.mAdapter.getSubversion();
    }


    /*public boolean delete() {
        return this.mAdapter != null && this.mAdapter.delete();
    }*/

    @Override
    public boolean onLeScan(LightPeripheral light, int mode, byte[] scanRecord) {

        DeviceInfo deviceInfo = new DeviceInfo();
        deviceInfo.macAddress = light.getMacAddress();
        deviceInfo.deviceName = light.getDeviceName();
        deviceInfo.meshName = light.getMeshNameStr();
        deviceInfo.meshAddress = light.getMeshAddress();
        deviceInfo.meshUUID = light.getMeshUUID();
        deviceInfo.productUUID = light.getProductUUID();
        deviceInfo.status = light.getStatus();
        deviceInfo.rssi = light.getRssi();
        Intent intent = new Intent();
        intent.setAction(ACTION_LE_SCAN);
        intent.putExtra(EXTRA_MODE, mode);
        intent.putExtra(EXTRA_DEVICE, deviceInfo);

        LocalBroadcastManager.getInstance(LightService.this)
                .sendBroadcast(intent);

        return true;
    }

    @Override
    public void onStatusChanged(LightController controller, int mode, int oldStatus,
                                int newStatus) {

        LightPeripheral light = controller.getCurrentLight();

        Intent intent = new Intent();

        if (newStatus == LightAdapter.STATUS_MESH_SCAN_TIMEOUT) {
            intent.setAction(ACTION_LE_SCAN_TIMEOUT);
        } else if (newStatus == LightAdapter.STATUS_MESH_SCAN_COMPLETED) {
            intent.setAction(ACTION_SCAN_COMPLETED);
        } else if (newStatus == LightAdapter.STATUS_MESH_OFFLINE) {
            intent.setAction(ACTION_OFFLINE);
        } else if (newStatus == LightAdapter.STATUS_UPDATE_ALL_MESH_COMPLETED) {
            intent.setAction(ACTION_UPDATE_MESH_COMPLETED);
        } else if (newStatus == LightAdapter.STATUS_OTA_PROGRESS) {
            OtaDeviceInfo deviceInfo = new OtaDeviceInfo();
            deviceInfo.firmwareRevision = light.getFirmwareRevision();
            deviceInfo.subversion = light.getSubversion();
            deviceInfo.macAddress = light.getMacAddress();
            deviceInfo.progress = controller.getOtaProgress();
            deviceInfo.status = newStatus;
            intent.setAction(ACTION_STATUS_CHANGED);
            intent.putExtra(EXTRA_MODE, mode);
            intent.putExtra(EXTRA_DEVICE, deviceInfo);
        } else {
            DeviceInfo deviceInfo = new DeviceInfo();
            deviceInfo.macAddress = light.getMacAddress();
            deviceInfo.deviceName = light.getDeviceName();
            deviceInfo.meshName = light.getMeshNameStr();
            deviceInfo.meshAddress = light.getMeshAddress();
            deviceInfo.meshUUID = light.getMeshUUID();
            deviceInfo.productUUID = light.getProductUUID();
            deviceInfo.status = newStatus;
            deviceInfo.firmwareRevision = light.getFirmwareRevision();
            deviceInfo.subversion = light.getSubversion();
            intent.setAction(ACTION_STATUS_CHANGED);
            intent.putExtra(EXTRA_MODE, mode);
            intent.putExtra(EXTRA_DEVICE, deviceInfo);
        }

        LocalBroadcastManager.getInstance(LightService.this)
                .sendBroadcast(intent);
    }

    @Override
    public void onNotify(LightPeripheral light, int mode, int opcode, int vendorId, int src,
                         byte[] params) {

        Intent intent = new Intent();
        intent.setAction(ACTION_NOTIFICATION);
        intent.putExtra(EXTRA_MODE, mode);

        NotificationInfo notifyInfo = new NotificationInfo();
        notifyInfo.vendorId = vendorId;
        notifyInfo.src = src;
        notifyInfo.opcode = opcode;
        notifyInfo.params = params;

        if (light != null) {
            DeviceInfo deviceInfo = new DeviceInfo();
            deviceInfo.macAddress = light.getMacAddress();
            deviceInfo.deviceName = light.getDeviceName();
            deviceInfo.meshName = light.getMeshNameStr();
            deviceInfo.meshAddress = light.getMeshAddress();
            deviceInfo.meshUUID = light.getMeshUUID();
            deviceInfo.productUUID = light.getProductUUID();
            notifyInfo.deviceInfo = deviceInfo;
        }

        intent.putExtra(EXTRA_NOTIFY, notifyInfo);

        LocalBroadcastManager.getInstance(LightService.this)
                .sendBroadcast(intent);
    }

    @Override
    public void onCommandResponse(LightPeripheral light, int mode, Command command, boolean success) {

    }

    @Override
    public void onError(int errorCode) {
        Intent intent = new Intent();
        intent.setAction(ACTION_ERROR);
        intent.putExtra(EXTRA_ERROR_CODE, errorCode);

        LocalBroadcastManager.getInstance(LightService.this)
                .sendBroadcast(intent);
    }

    @Override
    public void onErrorReport(int stateCode, int errorCode, int deviceId) {
        Intent intent = new Intent();
        intent.setAction(ACTION_ERROR_REPORT);
        ErrorReportInfo errorReportInfo = new ErrorReportInfo();
        errorReportInfo.stateCode = stateCode;
        errorReportInfo.errorCode = errorCode;
        errorReportInfo.deviceId = deviceId;
        intent.putExtra(EXTRA_ERROR_REPORT_INFO, errorReportInfo);

        LocalBroadcastManager.getInstance(LightService.this)
                .sendBroadcast(intent);
    }

    @Override
    public boolean onCommandSampled(byte opcode, int address, byte[] params, Object tag, int delay) {
        if (this.mAdapter == null)
            return false;
        return this.mAdapter.sendCommandNoResponse(opcode, address, params, tag, delay);
    }


    public boolean sendVendorCommand(byte opcode, int vendorId, int address, byte[] params) {
        if (this.mAdapter == null)
            return false;
        return this.mAdapter.sendVendorCommand(opcode, vendorId, address, params);
    }
}
