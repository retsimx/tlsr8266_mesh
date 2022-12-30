/********************************************************************************************************
 * @file AdvanceStrategy.java
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

import android.os.Handler;
import android.util.Log;

/**
 * 命令写入FIFO策略
 * <p>
 * 连续发送7条指令，
 */
public abstract class AdvanceStrategy {

    public final static byte[] DEFAULT_SAMPLE_LIST = new byte[]{(byte) 0xD0, (byte) 0xD2, (byte) 0xE2};

    private final static AdvanceStrategy DEFAULT = new DefaultAdvanceStrategy();
    private static AdvanceStrategy definition;
    protected Callback mCallback;
    protected int sampleRate = 320;
    protected byte[] sampleOpcodes;

    private static final int COMMAND_DELAY = 320;

    public static AdvanceStrategy getDefault() {
        synchronized (AdvanceStrategy.class) {
            if (definition != null)
                return definition;
        }
        return DEFAULT;
    }

    public static void setDefault(AdvanceStrategy strategy) {
        synchronized (AdvanceStrategy.class) {
            if (strategy != null)
                definition = strategy;
        }
    }

    static public boolean isExists(byte opcode, byte[] opcodeList) {
        for (byte opc : opcodeList) {
            if ((opc & 0xFF) == (opcode & 0xFF))
                return true;
        }
        return false;
    }

    final public int getSampleRate() {
        return sampleRate;
    }

    /**
     * 设置采样率,单位毫秒
     *
     * @param sampleRate
     */
    final public void setSampleRate(int sampleRate) {
        this.sampleRate = sampleRate;
    }

    /**
     * 回调接口,采样到的命令交由回调接口处理
     *
     * @param mCallback
     */
    public void setCallback(Callback mCallback) {
        this.mCallback = mCallback;
    }

    public byte[] getSampleOpcodes() {
        if (sampleOpcodes == null)
            return DEFAULT_SAMPLE_LIST;
        return sampleOpcodes;
    }

    /**
     * 设置采样的Opcode数组
     *
     * @param sampleOpcodes
     */
    public void setSampleOpcodes(byte[] sampleOpcodes) {
        this.sampleOpcodes = sampleOpcodes;
    }

    /**
     * 处理传进来的命令
     *
     * @param opcode     命令吗
     * @param address    目标地址
     * @param params     命令参数
     * @param delay      命令延时
     * @param tag        命令标签
     * @param noResponse 命令发送方式
     * @param immediate  是否立即写入底层FIFO
     * @return 命令是否成功写入
     */
    abstract public boolean postCommand(byte opcode, int address, byte[] params, int delay, Object tag, boolean noResponse, boolean immediate);

    /**
     * 启动,执行初始化
     */
    abstract public void onStart();

    /**
     * 停止，做一些清理工作
     */
    abstract public void onStop();

    public interface Callback {
        boolean onCommandSampled(byte opcode, int address, byte[] params, Object tag, int delay);
    }

    /**
     * 默认的命令发送策略
     */
    private static class DefaultAdvanceStrategy extends AdvanceStrategy {

        public final static String TAG = "AdvanceStrategy";

        private long lastSampleTime;

        // 上一个是否是采样指令
        private Handler commandSender;

        // 上一次发送指令时间
        private long lastCmdTime;

        private StrategyTask task;
//        private boolean isLastSampleCmd = false;

        private class StrategyTask implements Runnable {
            private byte opcode;
            private int address;
            private byte[] params;
            private int delay;
            private Object tag;

            public void setCommandArgs(byte opcode, int address, byte[] params, int delay, Object tag) {
                this.opcode = opcode;
                this.address = address;
                this.params = params;
                this.delay = delay;
                this.tag = tag;
            }

            @Override
            public void run() {
                Log.d(TAG, "Delay run Opcode : " + Integer.toHexString(opcode));
                lastSampleTime = System.currentTimeMillis();
                lastCmdTime = System.currentTimeMillis();
                DefaultAdvanceStrategy.this.mCallback.onCommandSampled(opcode, address, params, tag, delay);
            }
        }

        public DefaultAdvanceStrategy() {
            commandSender = new Handler();
            task = new StrategyTask();
        }


        @Override
        public void onStart() {
            this.lastSampleTime = 0;
        }

        @Override
        public void onStop() {
        }

        @Override
        public boolean postCommand(byte opcode, int address, byte[] params, int delay, Object tag, boolean noResponse, boolean immediate) {
            long currentTime = System.currentTimeMillis();
            // 是否直接发送指令
            boolean now = false;
            if (lastCmdTime == 0) {
                //第一个命令,直接写入FIFO
                now = true;
            } else if (immediate) {
                //立即发送的命令
                now = true;
            } else {
                if (isExists(opcode, this.getSampleOpcodes())) {
                    long interval = currentTime - this.lastSampleTime;
                    if (interval < 0) {
                        now = true;
                        lastSampleTime = currentTime;
                    } else if (interval >= this.getSampleRate()) {
                        now = true;
                        lastSampleTime = currentTime;
                    } else {
                        commandSender.removeCallbacks(task);
                        task.setCommandArgs(opcode, address, params, delay, tag);
                        commandSender.postDelayed(task, this.getSampleRate() - interval);
                    }
                } else {
                    now = true;
                }
            }

            if (now && this.mCallback != null) {
                if (address == 0 && opcode == (byte)0xDC) {
                    // direct command
                    Log.d(TAG, String.format("Sample - direct command  opcode : %02X delay : " + delay, opcode));
                    return this.mCallback.onCommandSampled(opcode, address, params, tag, delay);
                }
                long during = currentTime - this.lastCmdTime;
                if (during < 0) {
                    delay = COMMAND_DELAY;
                    this.lastCmdTime += COMMAND_DELAY;
                } else if (during < COMMAND_DELAY) {
                    if (delay < (COMMAND_DELAY - during)) {
                        delay = (int) (COMMAND_DELAY - during);
                    }
                    this.lastCmdTime = currentTime + delay;
                } else {
                    this.lastCmdTime = currentTime + delay;
                }
                Log.d(TAG, "Sample Opcode : " + Integer.toHexString(opcode & 0xFF) + " delay:" + delay);
/*
                long period = currentTime - this.lastCmdTime;
                if (period > 0 && period < COMMAND_DELAY) {
                    if (delay < (COMMAND_DELAY - period))
                        delay = (int) (COMMAND_DELAY - period);
                }
                lastCmdTime = System.currentTimeMillis() + delay;*/
                //所有采样到的命令立即交给回调接口处理
                return this.mCallback.onCommandSampled(opcode, address, params, tag, delay);
            }
            Log.d(TAG, "Delay Opcode : " + Integer.toHexString(opcode));
            return false;
        }
    }
}
