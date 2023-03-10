/********************************************************************************************************
 * @file ProgressViewManager.java
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
package com.telink.bluetooth.light.widget;

import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.PixelFormat;
import android.os.Build;
import android.util.DisplayMetrics;
import android.view.Gravity;
import android.view.View;
import android.view.WindowManager;

import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.util.ContextUtil;

/**
 * Created by kee on 2018/4/24.
 */

public class ProgressViewManager implements View.OnClickListener {

    private static ProgressViewManager instance;

    private ProgressWindow container;
    private WindowManager mWindowManager;
    private WindowManager.LayoutParams wmParams;
    private boolean mHasShown;

    private ProgressViewManager() {
    }


    public static ProgressViewManager getInstance() {
        if (instance == null)
            instance = new ProgressViewManager();
        return instance;
    }

    public void createFloatWindow(Context context) {
        wmParams = new WindowManager.LayoutParams();
        WindowManager windowManager = getWindowManager(context);
        container = new ProgressWindow(context);
        int sdkVersion = Build.VERSION.SDK_INT;
        if (sdkVersion >= Build.VERSION_CODES.O) {
            wmParams.type = WindowManager.LayoutParams.TYPE_APPLICATION_OVERLAY;
        } else if (sdkVersion >= 24) { /*android7.0?????????TYPE_TOAST*/
            wmParams.type = WindowManager.LayoutParams.TYPE_PHONE;
        } else { /*?????????????????????android6.0??????????????????????????????????????????????????????*/
            String packName = context.getPackageName();
            PackageManager pm = context.getPackageManager();
            boolean permission = (PackageManager.PERMISSION_GRANTED == pm.checkPermission("android.permission.SYSTEM_ALERT_WINDOW", packName));
            if (permission) {
                wmParams.type = WindowManager.LayoutParams.TYPE_PHONE;
            } else {
                wmParams.type = WindowManager.LayoutParams.TYPE_TOAST;
            }
        }

        //??????????????????????????????????????????
        wmParams.format = PixelFormat.RGBA_8888;
        //????????????????????????????????????????????????????????????????????????????????????????????????
        wmParams.flags = WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE;
        //???????????????????????????????????????????????????
        wmParams.gravity = Gravity.START | Gravity.TOP;

        DisplayMetrics dm = new DisplayMetrics();
        //??????????????????
        mWindowManager.getDefaultDisplay().getMetrics(dm);
        //???????????????
        int screenWidth = dm.widthPixels;
        //????????????
        int screenHeight = dm.heightPixels / 2;
        //????????????????????????????????????x???y?????????????????????gravity
        wmParams.x = screenWidth;
        wmParams.y = screenHeight;

        //??????????????????????????????
        wmParams.width = WindowManager.LayoutParams.WRAP_CONTENT;
        wmParams.height = WindowManager.LayoutParams.WRAP_CONTENT;
        container.setParams(wmParams);
        windowManager.addView(container, wmParams);
        mHasShown = true;
        //???????????????????????????
    }


    /**
     * ????????????????????????WindowManager???
     */
    private WindowManager getWindowManager(Context context) {
        if (mWindowManager == null) {
            mWindowManager = (WindowManager) context.getSystemService(Context.WINDOW_SERVICE);
        }
        return mWindowManager;
    }


    /**
     * ???????????????
     */
    public void removeFloatWindowManager() {
        //??????????????????
        boolean isAttach = true;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.KITKAT) {
            isAttach = container.isAttachedToWindow();
        }
        if (mHasShown && isAttach && mWindowManager != null)
            mWindowManager.removeView(container);
    }


    public void updateState(String state) {
            if (container != null) {
                container.updateState(state);
            }
    }


    public void updateProgress(int progress) {
        if (container != null) {
            container.updateProgress(progress);
        }
    }

    @Override
    public void onClick(View v) {
        if (v == container) {
            TelinkLog.e("click");
        }
    }
}
