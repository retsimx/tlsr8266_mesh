/********************************************************************************************************
 * @file     Peripheral.java 
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
/*
 * Copyright (C) 2015 The Telink Bluetooth Light Project
 *
 */
package com.telink.bluetooth;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.content.Context;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;

import com.telink.util.Arrays;

import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class Peripheral extends BluetoothGattCallback {

    public static final int CONNECTION_PRIORITY_BALANCED = 0;
    public static final int CONNECTION_PRIORITY_HIGH = 1;
    public static final int CONNECTION_PRIORITY_LOW_POWER = 2;

    private static final int CONN_STATE_IDLE = 1;
    private static final int CONN_STATE_CONNECTING = 2;
    private static final int CONN_STATE_CONNECTED = 4;
    private static final int CONN_STATE_DISCONNECTING = 8;
    private static final int CONN_STATE_CLOSED = 16;

    private static final int RSSI_UPDATE_TIME_INTERVAL = 2000;

    protected final Queue<CommandContext> mInputCommandQueue = new ConcurrentLinkedQueue<>();
    protected final Queue<CommandContext> mOutputCommandQueue = new ConcurrentLinkedQueue<>();
    protected final Map<String, CommandContext> mNotificationCallbacks = new ConcurrentHashMap<>();

    protected final Handler mTimeoutHandler = new Handler(Looper.getMainLooper());
    protected final Handler mRssiUpdateHandler = new Handler(Looper.getMainLooper());
    protected final Handler mDelayHandler = new Handler(Looper.getMainLooper());
    protected final Runnable mRssiUpdateRunnable = new RssiUpdateRunnable();
    protected final Runnable mCommandTimeoutRunnable = new CommandTimeoutRunnable();
    protected final Runnable mCommandDelayRunnable = new CommandDelayRunnable();

    private final Object mStateLock = new Object();
    private final Object mProcessLock = new Object();

    protected BluetoothDevice device;
    protected BluetoothGatt gatt;
    protected int rssi;
    protected byte[] scanRecord;
    protected String name;
    protected String mac;
    protected byte[] macBytes;
    protected int type;
    protected List<BluetoothGattService> mServices;

    protected AtomicBoolean processing = new AtomicBoolean(false);

    protected boolean monitorRssi;
    protected int updateIntervalMill = 5 * 1000;
    protected int commandTimeoutMill = 10 * 1000;
    protected long lastTime;
    //    private int mConnState = CONN_STATE_IDLE;
    private AtomicInteger mConnState = new AtomicInteger(CONN_STATE_IDLE);

    public Peripheral(BluetoothDevice device, byte[] scanRecord, int rssi) {

        this.device = device;
        this.scanRecord = scanRecord;
        this.rssi = rssi;

        this.name = device.getName();
        this.mac = device.getAddress();
        this.type = device.getType();
    }

    /********************************************************************************
     * Public API
     *******************************************************************************/

    public BluetoothDevice getDevice() {
        return this.device;
    }

    public String getDeviceName() {
        return this.name;
    }

    public String getMacAddress() {
        return this.mac;
    }

    public List<BluetoothGattService> getServices() {
        return mServices;
    }

    public byte[] getMacBytes() {

        if (this.macBytes == null) {
            String[] strArray = this.getMacAddress().split(":");
            int length = strArray.length;
            this.macBytes = new byte[length];

            for (int i = 0; i < length; i++) {
                this.macBytes[i] = (byte) (Integer.parseInt(strArray[i], 16) & 0xFF);
            }

            Arrays.reverse(this.macBytes, 0, length - 1);
        }

        return this.macBytes;
    }

    public int getType() {
        return this.type;
    }

    public int getRssi() {
        return this.rssi;
    }

    public boolean isConnected() {
        /*synchronized (this.mStateLock) {
            return this.mConnState == CONN_STATE_CONNECTED;
        }*/
        return this.mConnState.get() == CONN_STATE_CONNECTED;
    }

    public void connect(Context context) {

        this.lastTime = 0;
        if (this.mConnState.get() == CONN_STATE_IDLE) {
            TelinkLog.d("Peripheral#connect " + this.getDeviceName() + " -- "
                    + this.getMacAddress());
            this.mConnState.set(CONN_STATE_CONNECTING);
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                this.gatt = this.device.connectGatt(context, false, this, BluetoothDevice.TRANSPORT_LE);
            } else {
                this.gatt = this.device.connectGatt(context, false, this);
            }
            if (this.gatt == null) {
                this.disconnect();
                this.mConnState.set(CONN_STATE_IDLE);
                TelinkLog.d("Peripheral# gatt NULL onDisconnect:" + this.getDeviceName() + " -- "
                        + this.getMacAddress());
                this.onDisconnect();
            }
        }

    }

    public void disconnect() {

        if (this.mConnState.get() != CONN_STATE_CONNECTING && this.mConnState.get() != CONN_STATE_CONNECTED)
            return;

        TelinkLog.d("disconnect " + this.getDeviceName() + " -- "
                + this.getMacAddress());

        this.clear();

        synchronized (this.mStateLock) {
            if (this.gatt != null) {
                int connState = this.mConnState.get();
                if (connState == CONN_STATE_CONNECTED) {
                    this.gatt.disconnect();
                    this.mConnState.set(CONN_STATE_DISCONNECTING);
                } else {
                    this.gatt.disconnect();
                    this.gatt.close();
                    this.mConnState.set(CONN_STATE_CLOSED);
                }
            } else {
                this.mConnState.set(CONN_STATE_IDLE);
            }
        }
    }

    private void clear() {
        this.processing.set(false);
        this.stopMonitoringRssi();
        this.cancelCommandTimeoutTask();
        this.mInputCommandQueue.clear();
        this.mOutputCommandQueue.clear();
        this.mNotificationCallbacks.clear();
        this.mDelayHandler.removeCallbacksAndMessages(null);
    }

    public boolean sendCommand(Command.Callback callback, Command command) {

        if (this.mConnState.get() != CONN_STATE_CONNECTED)
            return false;

        CommandContext commandContext = new CommandContext(callback, command);
        this.postCommand(commandContext);

        return true;
    }

    public final void startMonitoringRssi(int interval) {

        this.monitorRssi = true;

        if (interval <= 0)
            this.updateIntervalMill = RSSI_UPDATE_TIME_INTERVAL;
        else
            this.updateIntervalMill = interval;
    }

    public final void stopMonitoringRssi() {
        this.monitorRssi = false;
        this.mRssiUpdateHandler.removeCallbacks(this.mRssiUpdateRunnable);
        this.mRssiUpdateHandler.removeCallbacksAndMessages(null);
    }

    public final boolean requestConnectionPriority(int connectionPriority) {
        return Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP && this.gatt.requestConnectionPriority(connectionPriority);
    }

    /********************************************************************************
     * Protected API
     *******************************************************************************/

    protected void onConnect() {
        //this.requestConnectionPriority(CONNECTION_PRIORITY_BALANCED);
        this.enableMonitorRssi(this.monitorRssi);
    }

    protected void onDisconnect() {
        this.enableMonitorRssi(false);
    }

    protected void onServicesDiscovered(List<BluetoothGattService> services) {
    }

    protected void onNotify(byte[] data, UUID serviceUUID,
                            UUID characteristicUUID, Object tag) {
    }

    protected void onRssiChanged() {
    }

    protected void enableMonitorRssi(boolean enable) {

        if (enable) {
            this.mRssiUpdateHandler.removeCallbacks(this.mRssiUpdateRunnable);
            this.mRssiUpdateHandler.postDelayed(this.mRssiUpdateRunnable, this.updateIntervalMill);
        } else {
            this.mRssiUpdateHandler.removeCallbacks(this.mRssiUpdateRunnable);
            this.mRssiUpdateHandler.removeCallbacksAndMessages(null);
        }
    }

    /********************************************************************************
     * Command Handler API
     *******************************************************************************/

    private void postCommand(CommandContext commandContext) {
        TelinkLog.d("postCommand");
//        if (commandContext.command.delay < 0) {
//            synchronized (this.mOutputCommandQueue) {
//                this.mOutputCommandQueue.add(commandContext);
//                this.processCommand(commandContext);
//            }
//            return;
//        }

        this.mInputCommandQueue.add(commandContext);
        synchronized (this.mProcessLock) {
            if (!this.processing.get()) {
                this.processCommand();
            }
        }
    }

    private void processCommand() {
        TelinkLog.d("processing : " + this.processing);

        CommandContext commandContext;
        Command.CommandType commandType;

        synchronized (mInputCommandQueue) {
            if (this.mInputCommandQueue.isEmpty())
                return;
            commandContext = this.mInputCommandQueue.poll();
        }

        if (commandContext == null)
            return;

        commandType = commandContext.command.type;

        if (commandType != Command.CommandType.ENABLE_NOTIFY && commandType != Command.CommandType.DISABLE_NOTIFY) {
//            synchronized (this.mProcessLock) {
//            if (!this.processing.get())
            this.processing.set(true);
//            }

            synchronized (mOutputCommandQueue) {
                this.mOutputCommandQueue.add(commandContext);
            }

        }

        int delay = commandContext.command.delay;
        if (delay > 0) {
            long currentTime = System.currentTimeMillis();
            if (lastTime == 0 || (currentTime - lastTime) >= delay)
                this.processCommand(commandContext);
            else
                this.mDelayHandler.postDelayed(this.mCommandDelayRunnable, delay);
        } else {
            this.processCommand(commandContext);
        }
    }

    synchronized private void processCommand(CommandContext commandContext) {

        Command command = commandContext.command;
        Command.CommandType commandType = command.type;

        TelinkLog.d("processCommand : " + command.toString());

        switch (commandType) {
            case READ:
                this.postCommandTimeoutTask();
                this.readCharacteristic(commandContext, command.serviceUUID,
                        command.characteristicUUID);
                break;
            case WRITE:
                this.postCommandTimeoutTask();
                this.writeCharacteristic(commandContext, command.serviceUUID,
                        command.characteristicUUID,
                        BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT,
                        command.data);
                break;
            case WRITE_NO_RESPONSE:
                this.postCommandTimeoutTask();
                this.writeCharacteristic(commandContext, command.serviceUUID,
                        command.characteristicUUID,
                        BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE,
                        command.data);
                break;
            case ENABLE_NOTIFY:
                this.enableNotification(commandContext, command.serviceUUID,
                        command.characteristicUUID);
                break;
            case DISABLE_NOTIFY:
                this.disableNotification(commandContext, command.serviceUUID,
                        command.characteristicUUID);
                break;
        }
    }

    private void commandCompleted() {

        TelinkLog.d("commandCompleted");

        synchronized (this.mProcessLock) {
            if (this.processing.get())
                this.processing.set(false);
            this.processCommand();
        }

    }

    private void commandSuccess(CommandContext commandContext, Object data) {
        TelinkLog.d("commandSuccess");
        this.lastTime = System.currentTimeMillis();
        if (commandContext != null) {

            Command command = commandContext.command;
            Command.Callback callback = commandContext.callback;
            commandContext.clear();

            if (callback != null) {
                callback.success(this, command,
                        data);
            }
        }
    }

    private void commandSuccess(Object data) {
        CommandContext commandContext;
        commandContext = this.mOutputCommandQueue.poll();
        this.commandSuccess(commandContext, data);
    }

    private void commandError(CommandContext commandContext, String errorMsg) {
        TelinkLog.d("commandError:" + errorMsg);
        this.lastTime = System.currentTimeMillis();
        if (commandContext != null) {

            Command command = commandContext.command;
            Command.Callback callback = commandContext.callback;
            commandContext.clear();

            if (callback != null) {
                callback.error(this, command,
                        errorMsg);
            }
        }
    }

    private void commandError(String errorMsg) {
        CommandContext commandContext;
        commandContext = this.mOutputCommandQueue.poll();
        this.commandError(commandContext, errorMsg);
    }

    private boolean commandTimeout(CommandContext commandContext) {
        TelinkLog.d("commandTimeout");
        this.lastTime = System.currentTimeMillis();
        if (commandContext != null) {
            Command command = commandContext.command;
            Command.Callback callback = commandContext.callback;
            commandContext.clear();

            if (callback != null) {
                return callback.timeout(this, command);
            }
        }

        return false;
    }

    private void postCommandTimeoutTask() {

        if (this.commandTimeoutMill <= 0)
            return;

        this.mTimeoutHandler.removeCallbacksAndMessages(null);
        this.mTimeoutHandler.postDelayed(this.mCommandTimeoutRunnable, this.commandTimeoutMill);
    }

    private void cancelCommandTimeoutTask() {
        this.mTimeoutHandler.removeCallbacksAndMessages(null);
    }

    /********************************************************************************
     * Private API
     *******************************************************************************/

    private void readCharacteristic(CommandContext commandContext,
                                    UUID serviceUUID, UUID characteristicUUID) {

        boolean success = true;
        String errorMsg = "";

        BluetoothGattService service = this.gatt.getService(serviceUUID);

        if (service != null) {

            BluetoothGattCharacteristic characteristic = service
                    .getCharacteristic(characteristicUUID);

            if (characteristic != null) {

                if (!this.gatt.readCharacteristic(characteristic)) {
                    success = false;
                    errorMsg = "read characteristic error";
                }

            } else {
                success = false;
                errorMsg = "read characteristic error: characteristic-null ";
            }
        } else {
            success = false;
            errorMsg = "service is not offered by the remote device";
        }

        if (!success) {
            this.commandError(errorMsg);
            this.commandCompleted();
        }
    }

    private void writeCharacteristic(CommandContext commandContext,
                                     UUID serviceUUID, UUID characteristicUUID, int writeType,
                                     byte[] data) {

        boolean success = true;
        String errorMsg = "";

        BluetoothGattService service = this.gatt.getService(serviceUUID);

        if (service != null) {
            BluetoothGattCharacteristic characteristic = this
                    .findWritableCharacteristic(service, characteristicUUID,
                            writeType);
            if (characteristic != null) {

                characteristic.setValue(data);
                characteristic.setWriteType(writeType);

                if (!this.gatt.writeCharacteristic(characteristic)) {
                    success = false;
                    errorMsg = "write characteristic error";
                }

            } else {
                success = false;
                errorMsg = "no characteristic";
            }
        } else {
            success = false;
            errorMsg = "service is not offered by the remote device";
        }

        if (!success) {
            this.commandError(errorMsg);
            this.commandCompleted();
        }
    }

    private void enableNotification(CommandContext commandContext,
                                    UUID serviceUUID, UUID characteristicUUID) {

        boolean success = true;
        String errorMsg = "";

        BluetoothGattService service = this.gatt.getService(serviceUUID);

        if (service != null) {

            BluetoothGattCharacteristic characteristic = this
                    .findNotifyCharacteristic(service, characteristicUUID);

            if (characteristic != null) {

                if (!this.gatt.setCharacteristicNotification(characteristic,
                        true)) {
                    success = false;
                    errorMsg = "enable notification error";
                } else {
                    String key = this.generateHashKey(serviceUUID,
                            characteristic);
                    this.mNotificationCallbacks.put(key, commandContext);
                }

            } else {
                success = false;
                errorMsg = "no characteristic";
            }

        } else {
            success = false;
            errorMsg = "service is not offered by the remote device";
        }

        if (!success) {
            this.commandError(commandContext, errorMsg);
        }

        this.commandCompleted();
    }

    private void disableNotification(CommandContext commandContext,
                                     UUID serviceUUID, UUID characteristicUUID) {

        boolean success = true;
        String errorMsg = "";

        BluetoothGattService service = this.gatt.getService(serviceUUID);

        if (service != null) {

            BluetoothGattCharacteristic characteristic = this
                    .findNotifyCharacteristic(service, characteristicUUID);

            if (characteristic != null) {

                String key = this.generateHashKey(serviceUUID, characteristic);
                this.mNotificationCallbacks.remove(key);

                if (!this.gatt.setCharacteristicNotification(characteristic,
                        false)) {
                    success = false;
                    errorMsg = "disable notification error";
                }

            } else {
                success = false;
                errorMsg = "no characteristic";
            }

        } else {
            success = false;
            errorMsg = "service is not offered by the remote device";
        }

        if (!success) {
            this.commandError(commandContext, errorMsg);
        }

        this.commandCompleted();
    }

    private BluetoothGattCharacteristic findWritableCharacteristic(
            BluetoothGattService service, UUID characteristicUUID, int writeType) {

        BluetoothGattCharacteristic characteristic = null;

        int writeProperty = BluetoothGattCharacteristic.PROPERTY_WRITE;

        if (writeType == BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE) {
            writeProperty = BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE;
        }

        List<BluetoothGattCharacteristic> characteristics = service
                .getCharacteristics();

        for (BluetoothGattCharacteristic c : characteristics) {
            if ((c.getProperties() & writeProperty) != 0
                    && characteristicUUID.equals(c.getUuid())) {
                characteristic = c;
                break;
            }
        }

        return characteristic;
    }

    private BluetoothGattCharacteristic findNotifyCharacteristic(
            BluetoothGattService service, UUID characteristicUUID) {

        BluetoothGattCharacteristic characteristic = null;

        List<BluetoothGattCharacteristic> characteristics = service
                .getCharacteristics();

        for (BluetoothGattCharacteristic c : characteristics) {
            if ((c.getProperties() & BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0
                    && characteristicUUID.equals(c.getUuid())) {
                characteristic = c;
                break;
            }
        }

        if (characteristic != null)
            return characteristic;

        for (BluetoothGattCharacteristic c : characteristics) {
            if ((c.getProperties() & BluetoothGattCharacteristic.PROPERTY_INDICATE) != 0
                    && characteristicUUID.equals(c.getUuid())) {
                characteristic = c;
                break;
            }
        }

        return characteristic;
    }

    private String generateHashKey(BluetoothGattCharacteristic characteristic) {
        return this.generateHashKey(characteristic.getService().getUuid(),
                characteristic);
    }

    private String generateHashKey(UUID serviceUUID,
                                   BluetoothGattCharacteristic characteristic) {
        return String.valueOf(serviceUUID) + "|" + characteristic.getUuid()
                + "|" + characteristic.getInstanceId();
    }

    /********************************************************************************
     * Implements BluetoothGattCallback API
     *******************************************************************************/

    @Override
    public void onConnectionStateChange(BluetoothGatt gatt, int status,
                                        int newState) {
        TelinkLog.d("onConnectionStateChange  status :" + status + " state : "
                + newState);

        if (newState == BluetoothGatt.STATE_CONNECTED) {

            this.mConnState.set(CONN_STATE_CONNECTED);

            if (this.gatt == null || !this.gatt.discoverServices()) {
                TelinkLog.d("remote service discovery has been stopped status = "
                        + newState);

                this.disconnect();

            } else {
                this.onConnect();
            }

        } else {

            synchronized (this.mStateLock) {
                TelinkLog.d("Close");

                if (this.gatt != null) {
                    this.gatt.close();
                    this.mConnState.set(CONN_STATE_CLOSED);
                }

                this.clear();
                this.mConnState.set(CONN_STATE_IDLE);
                TelinkLog.d("Peripheral#onConnectionStateChange#onDisconnect");
                this.onDisconnect();
            }
        }
    }

    @Override
    public void onCharacteristicChanged(BluetoothGatt gatt,
                                        BluetoothGattCharacteristic characteristic) {
        super.onCharacteristicChanged(gatt, characteristic);

        String key = this.generateHashKey(characteristic);
        CommandContext commandContext = this.mNotificationCallbacks.get(key);

        if (commandContext != null) {

            this.onNotify(characteristic.getValue(),
                    commandContext.command.serviceUUID,
                    commandContext.command.characteristicUUID,
                    commandContext.command.tag);
        }
    }

    @Override
    public void onCharacteristicRead(BluetoothGatt gatt,
                                     BluetoothGattCharacteristic characteristic, int status) {
        super.onCharacteristicRead(gatt, characteristic, status);

        this.cancelCommandTimeoutTask();

        if (status == BluetoothGatt.GATT_SUCCESS) {
            byte[] data = characteristic.getValue();
            this.commandSuccess(data);
        } else {
            this.commandError("read characteristic failed");
        }

        this.commandCompleted();
    }

    @Override
    public void onCharacteristicWrite(BluetoothGatt gatt,
                                      BluetoothGattCharacteristic characteristic, int status) {
        super.onCharacteristicWrite(gatt, characteristic, status);

        this.cancelCommandTimeoutTask();

        if (status == BluetoothGatt.GATT_SUCCESS) {
            this.commandSuccess(null);
        } else {
            this.commandError("write characteristic fail");
        }

        TelinkLog.d("onCharacteristicWrite newStatus : " + status);

        this.commandCompleted();
    }

    @Override
    public void onDescriptorRead(BluetoothGatt gatt,
                                 BluetoothGattDescriptor descriptor, int status) {
        super.onDescriptorRead(gatt, descriptor, status);

        this.cancelCommandTimeoutTask();

        if (status == BluetoothGatt.GATT_SUCCESS) {
            byte[] data = descriptor.getValue();
            this.commandSuccess(data);
        } else {
            this.commandError("read description failed");
        }

        this.commandCompleted();
    }

    @Override
    public void onDescriptorWrite(BluetoothGatt gatt,
                                  BluetoothGattDescriptor descriptor, int status) {
        super.onDescriptorWrite(gatt, descriptor, status);

        this.cancelCommandTimeoutTask();

        if (status == BluetoothGatt.GATT_SUCCESS) {
            this.commandSuccess(null);
        } else {
            this.commandError("write description failed");
        }

        this.commandCompleted();
    }

    @Override
    public void onServicesDiscovered(BluetoothGatt gatt, int status) {
        super.onServicesDiscovered(gatt, status);

        if (status == BluetoothGatt.GATT_SUCCESS) {
            List<BluetoothGattService> services = gatt.getServices();
            this.mServices = services;
            this.onServicesDiscovered(services);
        } else {
            TelinkLog.d("Service discovery failed");
            this.disconnect();
        }
    }

    @Override
    public void onReadRemoteRssi(BluetoothGatt gatt, int rssi, int status) {
        super.onReadRemoteRssi(gatt, rssi, status);

        if (status == BluetoothGatt.GATT_SUCCESS) {

            if (rssi != this.rssi) {
                this.rssi = rssi;
                this.onRssiChanged();
            }
        }
    }

    @Override
    public void onMtuChanged(BluetoothGatt gatt, int mtu, int status) {
        super.onMtuChanged(gatt, mtu, status);

        TelinkLog.d("mtu changed : " + mtu);
    }

    @Override
    public void onReliableWriteCompleted(BluetoothGatt gatt, int status) {
        super.onReliableWriteCompleted(gatt, status);
    }

    private final class CommandContext {

        public Command command;
        public Command.Callback callback;
        public long time;

        public CommandContext(Command.Callback callback, Command command) {
            this.callback = callback;
            this.command = command;
        }

        public void clear() {
            this.command = null;
            this.callback = null;
        }
    }

    private final class RssiUpdateRunnable implements Runnable {

        @Override
        public void run() {

            if (!monitorRssi)
                return;

            if (!isConnected())
                return;

            if (gatt != null)
                gatt.readRemoteRssi();

            mRssiUpdateHandler.postDelayed(mRssiUpdateRunnable, updateIntervalMill);
        }
    }

    private final class CommandTimeoutRunnable implements Runnable {

        @Override
        public void run() {

            synchronized (mOutputCommandQueue) {

                CommandContext commandContext = mOutputCommandQueue.peek();

                if (commandContext != null) {

                    Command command = commandContext.command;
                    Command.Callback callback = commandContext.callback;

                    boolean retry = commandTimeout(commandContext);

                    if (retry) {
                        commandContext.command = command;
                        commandContext.callback = callback;
                        processCommand(commandContext);
                    } else {
                        mOutputCommandQueue.poll();
                        commandCompleted();
                    }
                }
            }
        }
    }

    private final class CommandDelayRunnable implements Runnable {

        @Override
        public void run() {
            synchronized (mOutputCommandQueue) {
                CommandContext commandContext = mOutputCommandQueue.peek();
                processCommand(commandContext);
            }
        }
    }

    private final class ConnectTimeoutRunnable implements Runnable {

        @Override
        public void run() {

        }
    }
}
