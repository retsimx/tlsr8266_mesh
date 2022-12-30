/********************************************************************************************************
 * @file DeviceMeshScanningActivity.java
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

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.event.DeviceEvent;
import com.telink.bluetooth.event.LeScanEvent;
import com.telink.bluetooth.event.MeshEvent;
import com.telink.bluetooth.event.NotificationEvent;
import com.telink.bluetooth.light.DeviceInfo;
import com.telink.bluetooth.light.GetMeshDeviceNotificationParser;
import com.telink.bluetooth.light.LeScanParameters;
import com.telink.bluetooth.light.LightAdapter;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.TelinkLightService;
import com.telink.bluetooth.light.adapter.BaseRecyclerViewAdapter;
import com.telink.bluetooth.light.model.Light;
import com.telink.bluetooth.light.model.Mesh;
import com.telink.util.Arrays;
import com.telink.util.Event;
import com.telink.util.EventListener;
import com.telink.util.Strings;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import androidx.recyclerview.widget.GridLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

/**
 * 以mesh形式批量加灯
 * 只会添加 telink_mesh1 的设备
 * 当前处于连接状态下: 切换为默认网络 -> loop (获取device信息 -> 分配deviceId) 直到连续{TIMEOUT_CNT}次timeout -> 设置name/pwd/ltk -> 切换为正常网络
 * <p>
 * 当前处于非连接状态下: 扫描telink_mesh1设备 -> 连接登录 -> 获取device信息 -> 分配deviceId -> 设置name/pwd/ltk
 */
public final class DeviceMeshScanningActivity extends TelinkBaseActivity implements EventListener<String> {
    private static final String LOG_TAG = "DeviceMeshScan -- ";


    private static final String DEFAULT_NAME = "telink_mesh1";
    private static final String DEFAULT_PASSWORD = "123";


    private int getListRetry = 0;
    private static final int GET_LIST_TIMEOUT_MAX = 2;
    private static final int GET_LIST_TIMEOUT = 2 * 1000;

    private int setIdRetry = 0;
    private static final int SET_ID_RSP_TIMEOUT_MAX = 5;
    private static final int SET_ID_RSP_TIMEOUT = 2 * 1000;


    private static final int CMD_RELAY_CNT = 0x10;
    private static final long COMPLETE_DELAY = 60 * 1000;
    //    private AtomicBoolean isSetProcessing = new AtomicBoolean(false);
    private GetMeshDeviceNotificationParser.MeshDeviceInfo processingDevice;
    private TelinkLightApplication mApplication;
    private Mesh mesh;
    private Button btn_back;
    private List<GetMeshDeviceNotificationParser.MeshDeviceInfo> mMeshDevices;
    private Handler delayHandler;
    private MeshDeviceListAdapter mAdapter;

    private boolean updateComplete;

    // init state : false means not connected
    private boolean prepared = false;

    private int connectRetry = 0;
    private static final int CONNECT_RETRY_MAX = 2;
    /**
     * 默认网络持续时间
     */
    private static final int DEFAULT_TIMEOUT_SEC = 100;

    private DeviceInfo directDevice;

    private Set<Integer> supportedDevices;

    // reconnect when direct connected device not support mesh scan
    private boolean reconnecting;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_mesh_scanning);

        setTitle("Mesh Scan");
        enableBackNav(false);

        //监听事件
        this.mApplication = (TelinkLightApplication) this.getApplication();

        if (mApplication.isEmptyMesh()) {
            finish();
            Toast.makeText(mApplication, "mesh info null", Toast.LENGTH_SHORT).show();
            return;
        }
        this.mesh = mApplication.getMesh();

        this.mApplication.addEventListener(LeScanEvent.LE_SCAN, this);
        this.mApplication.addEventListener(LeScanEvent.LE_SCAN_TIMEOUT, this);
        this.mApplication.addEventListener(DeviceEvent.STATUS_CHANGED, this);
        this.mApplication.addEventListener(MeshEvent.UPDATE_COMPLETED, this);
        this.mApplication.addEventListener(MeshEvent.ERROR, this);
        this.mApplication.addEventListener(NotificationEvent.GET_MESH_DEVICE_LIST, this);
        this.mApplication.addEventListener(NotificationEvent.UPDATE_MESH_COMPLETE, this);
        this.mApplication.addEventListener(NotificationEvent.GET_DEVICE_STATE, this);

        TelinkLightService.Instance().idleMode(false);
        mMeshDevices = new ArrayList<>();
        delayHandler = new Handler();
        findViewById(R.id.btn_log).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startActivity(new Intent(DeviceMeshScanningActivity.this, LogActivity.class));
            }
        });
        btn_back = (Button) findViewById(R.id.btn_back);
        btn_back.setEnabled(false);
        btn_back.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                finish();
            }
        });
        RecyclerView rl = (RecyclerView) findViewById(R.id.list_devices);
        rl.setLayoutManager(new GridLayoutManager(this, 3));
        mAdapter = new MeshDeviceListAdapter();
        rl.setAdapter(this.mAdapter);
        scanLog(" -- create -- ");
        if (this.mApplication.getConnectDevice() != null) {
            directDevice = this.mApplication.getConnectDevice();
            prepared = true;
            setDefault();
        } else {
            prepared = false;
            startScan();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

    }

    @Override
    public void finish() {
        super.finish();
        this.mApplication.removeEventListener(this);
        if (delayHandler != null) {
            delayHandler.removeCallbacksAndMessages(null);
        }
    }

    private void startScan() {
        scanLog("startScan");
        TelinkLightService.Instance().idleMode(true);

        if (mApplication.isEmptyMesh())
            return;

        //扫描参数
        LeScanParameters params = LeScanParameters.create();
        params.setMeshName(DEFAULT_NAME);
        params.setOutOfMeshName(DEFAULT_NAME);
        params.setTimeoutSeconds(10);
        params.setScanMode(!reconnecting);
//        params.setScanTypeFilter(0x00);
        TelinkLightService.Instance().startScan(params);
    }


    private void onLeScan(LeScanEvent event) {
        scanLog("startConnect");
        DeviceInfo deviceInfo = event.getArgs();
        boolean connect = false;
        if (!reconnecting || supportedDevices.contains(event.getArgs().meshAddress)) {
            TelinkLightService.Instance().idleMode(true);
            TelinkLightService.Instance().connect(deviceInfo.macAddress, 10);

        }
    }

    private void onLeScanTimeout() {
        scanLog("scan timeout");
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                btn_back.setEnabled(true);
            }
        });
    }

    private void onConnected(DeviceInfo deviceInfo) {
        TelinkLightService.Instance().login(Strings.stringToBytes(DEFAULT_NAME, 16), Strings.stringToBytes(DEFAULT_PASSWORD, 16));
    }

    private void onLogin(DeviceInfo deviceInfo) {
        TelinkLightService.Instance().enableNotification();

        directDevice = deviceInfo;
        if (reconnecting) {
            getDissociateList();
        } else {
            checkMeshScanSupportState();
        }
    }

    /**
     * 切换为默认网络
     */
    private void setDefault() {
        scanLog("set default");
        byte opcode = (byte) (0xC9 & 0xFF);
        int addr = 0xFFFF;
        byte[] params = {0x08, (byte) (DEFAULT_TIMEOUT_SEC & 0xFF), 0x00};
        TelinkLightService.Instance().sendCommandNoResponse(opcode, addr, params);

        // getDissociateList();
        checkMeshScanSupportState();
    }

    /**
     * 从默认网络切回原先网络
     */
    private void setDefaultBack() {
        scanLog("set default back");
        byte opcode = (byte) (0xC9 & 0xFF);
        int addr = 0xFFFF;
        byte[] params = {0x08, 0x00, 0x00};
        TelinkLightService.Instance().sendCommandNoResponse(opcode, addr, params);
    }


    /**
     * 获取未配置过的设备列表
     * <p>
     * 16 * 80ms + 100ms 完成后delay， 获取两次
     */
    private void getDissociateList() {
//        mMeshDevices.clear();
        scanLog("get device list");
        byte opcode = (byte) (0xE0 & 0xFF);
        int addr = 0xFFFF;
        byte[] params = {(byte) 0xFF, (byte) 0xFF, 0x01, 0x10};
        TelinkLightService.Instance().sendCommandNoResponse(opcode, addr, params);

        delayHandler.removeCallbacksAndMessages(null);
        delayHandler.postDelayed(getListTimeoutTask, GET_LIST_TIMEOUT);
    }

    /**
     * 分配新的device id 或 获取设备device id
     */
    private void setDeviceId(GetMeshDeviceNotificationParser.MeshDeviceInfo deviceInfo) {
        scanLog("set device id start: " + deviceInfo.newDeviceId + " -- " + Arrays.bytesToHexString(deviceInfo.macBytes, ":"));
        byte opcode = (byte) (0xE0 & 0xFF);

        // 在配置失败时， 用新的地址配置, 再次失败时用旧地址
        int srcId = setIdRetry % 2 == 0 ? deviceInfo.deviceId : deviceInfo.newDeviceId;

        int desId = deviceInfo.newDeviceId;

        byte[] params = new byte[10];

        params[0] = (byte) (desId & 0xFF);
        params[1] = (byte) (desId >> 8 & 0xFF);

        params[2] = 0x01;
        params[3] = 0x10;

        System.arraycopy(deviceInfo.macBytes, 0, params, 4, deviceInfo.macBytes.length);

        TelinkLightService.Instance().sendCommandNoResponse(opcode, srcId, params);

        delayHandler.removeCallbacks(deviceIdRspCheck);
        delayHandler.postDelayed(deviceIdRspCheck, SET_ID_RSP_TIMEOUT);
    }

    private Runnable deviceIdRspCheck = new Runnable() {
        @Override
        public void run() {
            scanLog("set device id rsp timeout");
            if (setIdRetry <= SET_ID_RSP_TIMEOUT_MAX) {
                setIdRetry++;
                setDeviceId(processingDevice);
            } else {
                processingDevice = null;
                getDissociateList();
            }
        }
    };


    /**
     * check is mesh ota supported by devices in mesh network
     */
    private void checkMeshScanSupportState() {
        // 0x0A for get device state
        scanLog("checkDeviceSupportState");
        byte opcode = (byte) 0xC7;
        int address = 0xFFFF;
        byte[] params = new byte[]{0x10, 0x0A};
        TelinkLightService.Instance().sendCommandNoResponse(opcode, address,
                params);
        delayHandler.postDelayed(checkMeshScanSupportTimeoutTask, 1000);
    }

    private Runnable checkMeshScanSupportTimeoutTask = new Runnable() {
        @Override
        public void run() {
            if (supportedDevices == null || supportedDevices.size() == 0) {
                saveLog("no supported device found!");
                btn_back.setEnabled(true);
            } else {
                // check if supported devices contains direct connect device
                if (directDevice != null) {
                    if (supportedDevices.contains(directDevice.meshAddress)) {
                        getDissociateList();
                    } else {
                        scanLog("reconnect for supported device");
                        reconnecting = true;
                        TelinkLightService.Instance().idleMode(true);
                    }
                }
            }
        }
    };



    /*private synchronized void updateDeviceList(GetMeshDeviceNotificationParser.MeshDeviceInfo deviceInfo) {

        for (GetMeshDeviceNotificationParser.MeshDeviceInfo info : mMeshDevices) {
            if (Arrays.equals(info.macBytes, deviceInfo.macBytes)) {
                info.deviceId = deviceInfo.deviceId;
                return;
            }
        }


        mMeshDevices.add(deviceInfo);

        Light localDeviceInfo = new Light();
        localDeviceInfo.meshAddress = mesh.getDeviceAddress();
        localDeviceInfo.macAddress = Arrays.bytesToHexString(Arrays.reverse(deviceInfo.macBytes), ":");
        mesh.devices.add(localDeviceInfo);
        mesh.saveOrUpdate(this);
        setDeviceId(deviceInfo, localDeviceInfo.meshAddress);
    }*/

    private void resetMeshInfo() {
        updateComplete = false;
        scanLog("reset mesh info:  " + mesh.name + " -- " + mesh.password);
        TelinkLightService.Instance().resetByMesh(mesh.name, mesh.password);
//        delayHandler.removeCallbacksAndMessages(null);
//        delayHandler.postDelayed(resetCompleteTask, 5 * 1000);
    }


    @Override
    public void performed(Event<String> event) {
        switch (event.getType()) {
            case LeScanEvent.LE_SCAN:
                onLeScan((LeScanEvent) event);
                break;

            case LeScanEvent.LE_SCAN_TIMEOUT:
                onLeScanTimeout();
                break;

            case DeviceEvent.STATUS_CHANGED:
                if (updateComplete) return;
                DeviceInfo deviceInfo = ((DeviceEvent) event).getArgs();
                int state = deviceInfo.status;
                if (state == LightAdapter.STATUS_CONNECTED) {
                    scanLog("connected");
                    onConnected(deviceInfo);
                } else if (state == LightAdapter.STATUS_LOGIN) {
                    scanLog("login success");
                    prepared = true;
                    onLogin(deviceInfo);
                } else if (state == LightAdapter.STATUS_LOGOUT) {
                    directDevice = null;
                    scanLog("disconnected");
                    delayHandler.removeCallbacksAndMessages(null);
                    if (!prepared && connectRetry <= CONNECT_RETRY_MAX) {
                        connectRetry++;
                        startScan();
                    } else if (reconnecting) {
                        startScan();
                    } else {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                btn_back.setEnabled(true);
                            }
                        });
                    }


                } else if (state == LightAdapter.STATUS_UPDATE_MESH_COMPLETED) {
                    scanLog("update complete");
                    if (!updateComplete) {
                        delayHandler.removeCallbacks(resetCompleteTask);
                        delayHandler.postDelayed(resetCompleteTask, COMPLETE_DELAY);
                    }

                }
                break;
            case NotificationEvent.GET_MESH_DEVICE_LIST:
                onDeviceListNotify((NotificationEvent) event);
                break;

            case NotificationEvent.UPDATE_MESH_COMPLETE:
                scanLog("reset complete notification");
                updateComplete = true;
                delayHandler.removeCallbacks(resetCompleteTask);
                delayHandler.post(resetCompleteTask);
                break;

            case NotificationEvent.GET_DEVICE_STATE:
                onDeviceStateGet((NotificationEvent) event);
                break;
        }
    }


    private void onDeviceStateGet(NotificationEvent event) {
        byte[] params = event.getArgs().params;
        TelinkLog.d("device state: " + Arrays.bytesToHexString(event.getArgs().params, ":"));
        if (params != null && params.length > 0 && params[0] != 0) {
            if (supportedDevices == null) {
                supportedDevices = new HashSet<>();
            }

            if (!supportedDevices.add(params[0] & 0xFF)) {
                scanLog("get repeat device address");
            }
            delayHandler.removeCallbacks(checkMeshScanSupportTimeoutTask);
            delayHandler.postDelayed(checkMeshScanSupportTimeoutTask, 1000);
        } else {
            scanLog("device state err");
        }


    }

    private synchronized void onDeviceListNotify(NotificationEvent event) {
        GetMeshDeviceNotificationParser.MeshDeviceInfo meshDeviceInfo
                = GetMeshDeviceNotificationParser.create().parse((event).getArgs());
        scanLog("onNotify -- id: " + meshDeviceInfo.deviceId + " -- mac: " + Arrays.bytesToHexString(meshDeviceInfo.macBytes, ":"));

        // 是否正在配置设备deviceId， 防止出现循环状态
        if (processingDevice == null) {

            // && 该设备未被配置过, 即不存在于设备列表内
            for (GetMeshDeviceNotificationParser.MeshDeviceInfo device :
                    mMeshDevices) {
                if (Arrays.equals(device.macBytes, meshDeviceInfo.macBytes)) {
                    TelinkLog.e("get settle device");
                    return;
                }
            }
            if (!supportedDevices.contains(meshDeviceInfo.deviceId)) {
                scanLog("unsupported device notify");
                return;
            }
            delayHandler.removeCallbacks(getListTimeoutTask);
            getListRetry = 0;
            setIdRetry = 0;
            processingDevice = meshDeviceInfo;
            processingDevice.newDeviceId = mesh.getDeviceAddress();
            setDeviceId(processingDevice);
        } else {
            if (Arrays.equals(meshDeviceInfo.macBytes, processingDevice.macBytes) && meshDeviceInfo.deviceId == processingDevice.newDeviceId) {
                delayHandler.removeCallbacks(deviceIdRspCheck);
                setIdRetry = 0;
                scanLog("set deviceId complete: " + meshDeviceInfo.deviceId);
                mMeshDevices.add(meshDeviceInfo);
                Light localDeviceInfo = new Light();
                localDeviceInfo.meshAddress = meshDeviceInfo.deviceId;
                localDeviceInfo.macAddress = Arrays.bytesToHexString(Arrays.reverse(meshDeviceInfo.macBytes), ":");
                localDeviceInfo.deviceName = mesh.name;
                localDeviceInfo.meshName = mesh.name;
                localDeviceInfo.productUUID = meshDeviceInfo.productUUID;
                mesh.devices.add(localDeviceInfo);
                mesh.saveOrUpdate(this);
                notifyListData();
                processingDevice = null;
                getDissociateList();
            } else {
                scanLog("notify other device");
            }
        }
    }

    private void scanLog(String msg) {
//        TelinkLog.d(LOG_TAG + msg);
        saveLog(msg);
    }


    private void notifyListData() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mAdapter.notifyDataSetChanged();
            }
        });
    }


    private Runnable getListTimeoutTask = new Runnable() {
        @Override
        public void run() {
            scanLog("get device list timeout");

            if (getListRetry <= GET_LIST_TIMEOUT_MAX) {
                getListRetry++;
                getDissociateList();
            } else {
                if (mMeshDevices.size() != 0) {
                    resetMeshInfo();
                } else {
                    btn_back.setEnabled(true);
                }
            }


        }
    };

    private Runnable resetCompleteTask = new Runnable() {
        @Override
        public void run() {
            scanLog("-- reset complete --");
            setDefaultBack();
            btn_back.setEnabled(true);
//            TelinkLightService.Instance().updateNotification();
        }
    };

    public class MeshDeviceListAdapter extends BaseRecyclerViewAdapter<ViewHolder> {

        @Override
        public ViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
            View itemView = LayoutInflater.from(DeviceMeshScanningActivity.this).inflate(R.layout.device_item, null);
            ViewHolder holder = new ViewHolder(itemView);
            holder.img_icon = (ImageView) itemView.findViewById(R.id.img_icon);
            holder.txt_name = (TextView) itemView.findViewById(R.id.txt_name);
            return holder;
        }


        @Override
        public void onBindViewHolder(ViewHolder holder, int position) {
            super.onBindViewHolder(holder, position);
            final GetMeshDeviceNotificationParser.MeshDeviceInfo deviceInfo = mMeshDevices.get(position);
            holder.txt_name.setText(deviceInfo.deviceId + " -- " + Arrays.bytesToHexString(deviceInfo.macBytes, ":"));
        }


        @Override
        public int getItemCount() {
            return mMeshDevices.size();
        }
    }

    class ViewHolder extends RecyclerView.ViewHolder {

        ImageView img_icon;
        TextView txt_name;

        public ViewHolder(View itemView) {
            super(itemView);
        }
    }
}
