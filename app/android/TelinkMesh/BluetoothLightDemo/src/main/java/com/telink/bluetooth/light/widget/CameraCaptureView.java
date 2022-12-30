/********************************************************************************************************
 * @file     CameraCaptureView.java 
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
package com.telink.bluetooth.light.widget;

import android.content.Context;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.Region;
import android.util.AttributeSet;
import android.view.View;

import com.telink.bluetooth.light.R;


public final class CameraCaptureView extends View {

    private final Paint mPaint;
    private final int mMaskColor;

    private Bitmap mBmpTopLeft;
    private Bitmap mBmpTopRight;
    private Bitmap mBmpBottomLeft;
    private Bitmap mBmpBottomRight;

    private int scale = 60;

    public CameraCaptureView(Context context, AttributeSet attrs) {
        super(context, attrs);
        mPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        final Resources resources = getResources();
        mMaskColor = 0xAA525252;
        // cache images
        mBmpTopLeft = BitmapFactory.decodeResource(resources, R.drawable.scan_corner_top_left);
        mBmpTopRight = BitmapFactory.decodeResource(resources, R.drawable.scan_corner_top_right);
        mBmpBottomLeft = BitmapFactory.decodeResource(resources, R.drawable.scan_corner_bottom_left);
        mBmpBottomRight = BitmapFactory.decodeResource(resources, R.drawable.scan_corner_bottom_right);
    }

    @Override
    public void onDraw(Canvas canvas) {
        final int width = canvas.getWidth();
        final int height = canvas.getHeight();
        final int wh = width > height ? height : width;
        final int boxLength = wh * scale / 100;
        final int left = (width - boxLength) / 2;
        final int top = (height - boxLength) / 2;
        final int right = left + boxLength;
        final int bottom = top + boxLength;
        final Rect frame = new Rect(left, top, right, bottom);
        canvas.save();
        canvas.clipRect(frame, Region.Op.XOR);
        canvas.drawColor(mMaskColor);
        canvas.restore();
        canvas.save();
        drawEdges(canvas, frame);
        canvas.restore();
    }

    private void drawEdges(Canvas canvas, Rect box) {
        mPaint.setColor(Color.WHITE);
        final float _x = box.right - mBmpTopRight.getWidth();
        final float _y = box.bottom - mBmpBottomLeft.getHeight();
        canvas.drawBitmap(mBmpTopLeft, box.left, box.top, mPaint);
        canvas.drawBitmap(mBmpTopRight, _x, box.top, mPaint);
        canvas.drawBitmap(mBmpBottomLeft, box.left, _y, mPaint);
        canvas.drawBitmap(mBmpBottomRight, _x, _y, mPaint);
    }
}
