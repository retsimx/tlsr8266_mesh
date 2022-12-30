/********************************************************************************************************
 * @file QRCodeGenerator.java
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
package com.telink.bluetooth.light.activity.share;

import android.content.Context;
import android.graphics.Bitmap;
import android.os.AsyncTask;
import android.os.Handler;
import android.util.DisplayMetrics;
import android.view.WindowManager;

import com.google.zxing.WriterException;
import com.google.zxing.qrcode.decoder.ErrorCorrectionLevel;
import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.light.TelinkLightApplication;

import java.io.UnsupportedEncodingException;


/**
 * 二维码生成器
 */
public class QRCodeGenerator extends AsyncTask<Void, Void, Bitmap> {
    private QREncoder mEncoder;
    private Bitmap mResult;
    // 任务执行成功
    public final static int RESULT_GENERATE_SUCCESS = 1;
    // 任务执行失败
    public final static int RESULT_GENERATE_FAIL = 2;
    private Handler mHandler;
    private Context mContext;


    public QRCodeGenerator(Context context, Handler handler) {
        super();
        mContext = context;
        mResult = null;
        initEncoder();
        this.mHandler = handler;
    }

    private void initEncoder() {
        DisplayMetrics metrics = new DisplayMetrics();
        WindowManager windowManager = (WindowManager) TelinkLightApplication.getInstance().getSystemService(Context.WINDOW_SERVICE);
        windowManager.getDefaultDisplay().getMetrics(metrics);
        int size = (int) metrics.density * 350;
        QREncoder.Builder builder = new QREncoder.Builder();
        builder.setBackground(0xFFFFFFFF);
        builder.setCodeColor(0xFF000000);
        builder.setCharset(GZIP.GZIP_ENCODE);
        builder.setWidth(size);
        builder.setHeight(size);
//        builder.setPadding(2);
        builder.setLevel(ErrorCorrectionLevel.L);
        mEncoder = builder.build();

    }

    public Bitmap getResult() {
        return mResult;
    }

    public void clear() {
        this.mResult = null;
    }

    @Override
    protected Bitmap doInBackground(Void... params) {
        String src = QRCodeDataOperator.provideStr();
        if (src == null) {
            mHandler.sendEmptyMessage(RESULT_GENERATE_FAIL);
            return null;
        }
//        String src = "www.12306.com";
        TelinkLog.w("原始数据: " + src + " 共" + src.getBytes().length + "字节");
        src = GZIP.compressed(src);
        try {
            src = GZIP.bytesToHexString(src.getBytes(GZIP.GZIP_ENCODE));
            TelinkLog.w("压缩后的数据: " + src + " 共" + src.getBytes(GZIP.GZIP_ENCODE).length + "字节");
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
            mHandler.sendEmptyMessage(RESULT_GENERATE_FAIL);
            return null;
        }
//        mResult = QRCodeEncoder.syncEncodeQRCode(src, BGAQRCodeUtil.dp2px(mContext, 300));
        try {
            mResult = mEncoder.encode(src);
        } catch (WriterException e) {
            e.printStackTrace();
            mHandler.sendEmptyMessage(RESULT_GENERATE_FAIL);
        }
        return mResult;
    }


    @Override
    protected void onPostExecute(Bitmap bitmap) {
        super.onPostExecute(bitmap);
        this.mResult = bitmap;
        mHandler.sendEmptyMessage(RESULT_GENERATE_SUCCESS);
    }
}
