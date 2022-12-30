/********************************************************************************************************
 * @file     TelinkLog.java 
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

import android.os.Environment;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;

public class TelinkLog {

    public final static String TAG = "TelinkBluetoothSDK";

    public static boolean ENABLE = true;
    public static boolean LOG2FILE_ENABLE = false;
    public static String FILE_PREFIX = "TelinkBluetoothSDKLogger";
    private static BufferedWriter mWriter;


    public static boolean isLoggable(int level) {
        if (ENABLE)
            return Log.isLoggable(TAG, level);
        return false;
    }

    public static String getStackTraceString(Throwable th) {
        if (ENABLE)
            return Log.getStackTraceString(th);
        return th.getMessage();
    }

    public static int println(int level, String msg) {
        if (ENABLE)
            return Log.println(level, TAG, msg);
        return 0;
    }

    public static int v(String msg) {
        write2File(msg);
        if (ENABLE)
            return Log.v(TAG, msg);
        return 0;
    }

    public static int v(String msg, Throwable th) {
        write2File(msg);
        write2File(getStackTraceString(th));
        if (ENABLE)
            return Log.v(TAG, msg, th);
        return 0;
    }

    public static int d(String msg) {
        write2File(msg);
        if (ENABLE)
            return Log.d(TAG, msg);
        return 0;
    }

    public static int d(String msg, Throwable th) {
        write2File(msg);
        write2File(getStackTraceString(th));
        if (ENABLE)
            return Log.d(TAG, msg, th);
        return 0;
    }

    public static int i(String msg) {
        write2File(msg);
        if (ENABLE)
            return Log.i(TAG, msg);
        return 0;
    }

    public static int i(String msg, Throwable th) {
        write2File(msg);
        write2File(getStackTraceString(th));
        if (ENABLE)
            return Log.i(TAG, msg, th);
        return 0;
    }

    public static int w(String msg) {
        write2File(msg);
        if (ENABLE)
            return Log.w(TAG, msg);
        return 0;
    }

    public static int w(String msg, Throwable th) {
        write2File(msg);
        write2File(getStackTraceString(th));
        if (ENABLE)
            return Log.w(TAG, msg, th);
        return 0;
    }

    public static int w(Throwable th) {
        write2File(getStackTraceString(th));
        if (ENABLE)
            return Log.w(TAG, th);
        return 0;
    }

    public static int e(String msg) {
        write2File(msg);
        if (ENABLE)
            return Log.w(TAG, msg);
        return 0;
    }

    public static int e(String msg, Throwable th) {
        write2File(msg);
        write2File(getStackTraceString(th));
        if (ENABLE)
            return Log.e(TAG, msg, th);
        return 0;
    }

    private static void write2File(String log) {
        if (LOG2FILE_ENABLE) {
            try {
                mWriter.write(log);
                mWriter.newLine();
                mWriter.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static void onCreate(String fileName) {
        File root = Environment.getExternalStorageDirectory();
        File dir = new File(root.getPath() + "/TelinkLog");

        try {
            if (!dir.exists())
                dir.mkdir();
            File file = new File(dir, fileName);
            if (!file.exists())
                file.createNewFile();
            FileOutputStream fos = new FileOutputStream(file);
            OutputStreamWriter osw = new OutputStreamWriter(fos);
            mWriter = new BufferedWriter(osw, 1024);
            mWriter.write(TAG + " begin : ");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void onDestroy() {
        try {
            if (mWriter != null) {
                mWriter.write(TAG + " end : ");
                mWriter.close();
            }
        } catch (Exception e) {
        }
    }
}
