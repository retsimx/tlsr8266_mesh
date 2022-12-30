/********************************************************************************************************
 * @file     ColorPicker.java 
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
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.graphics.Shader;
import android.graphics.SweepGradient;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import android.widget.RemoteViews.RemoteView;

@RemoteView
public class ColorPicker extends View {

    private static final float PI = 3.141592654F;

    private Paint mPaint;
    private Paint mCenterPaint;
    private int[] mColors;
    private OnColorChangeListener mListener;

    private int centerX = 200;
    private int centerY = 200;
    private int radius = 400;
    private int centerRadius = 16;
    private int color;

    public ColorPicker(Context context) {
        super(context, null);
    }

    public ColorPicker(Context context, AttributeSet attrs) {
        super(context, attrs);

        this.mColors = new int[]{0xFFFF0000, 0xFFFF00FF, 0xFF0000FF,
                0xFF00FFFF, 0xFF00FF00, 0xFFFFFF00, 0xFFFF0000};

        Shader s = new SweepGradient(0, 0, mColors, null);

        this.mPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        this.mPaint.setShader(s);
        this.mPaint.setStyle(Paint.Style.STROKE);

        this.mCenterPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        this.mCenterPaint.setStrokeWidth(5);
        this.mCenterPaint.setColor(0xFF000000);
    }

    public void setOnColorChangeListener(OnColorChangeListener listener) {
        this.mListener = listener;
    }

    public void setRadius(int value) {

        if (value > 0)
            this.radius = value;
    }

    public void setCenterRadius(int value) {

        if (value > 0)
            this.centerRadius = value;
    }

    public void setDefaultColor(int value) {

        this.mCenterPaint.setColor(value);
    }

    public int getColor() {
        return this.color;
    }

    @Override
    protected synchronized void onDraw(Canvas canvas) {

        float r = this.centerX - this.mPaint.getStrokeWidth() * 0.5F;
        canvas.translate(this.centerX, this.centerX);

        canvas.drawOval(new RectF(-r, -r, r, r), this.mPaint);
        canvas.drawCircle(0, 0, this.centerRadius, this.mCenterPaint);
    }

    @Override
    protected synchronized void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {

        int width = MeasureSpec.getSize(widthMeasureSpec);
        int height = MeasureSpec.getSize(heightMeasureSpec);

        int size = Math.min(width, height);

        if (size > this.radius)
            size = this.radius;

        int r = (int) (size * 0.5);

        this.centerX = r;
        this.centerY = r;
        this.mPaint.setStrokeWidth(r);

        if (this.centerRadius >= this.radius)
            this.centerRadius = 16;

        this.mCenterPaint.setStrokeWidth(this.centerRadius);

        this.setMeasuredDimension(size, size);
    }

    private int ave(int s, int d, float p) {
        return s + Math.round(p * (d - s));
    }

    private int interpColor(int colors[], float unit) {

        if (unit <= 0) {
            return colors[0];
        }
        if (unit >= 1) {
            return colors[colors.length - 1];
        }

        float p = unit * (colors.length - 1);
        int i = (int) p;
        p -= i;

        int c0 = colors[i];
        int c1 = colors[i + 1];
        int a = ave(Color.alpha(c0), Color.alpha(c1), p);
        int r = ave(Color.red(c0), Color.red(c1), p);
        int g = ave(Color.green(c0), Color.green(c1), p);
        int b = ave(Color.blue(c0), Color.blue(c1), p);

        return Color.argb(a, r, g, b);
    }

    private void fillPaint(MotionEvent event) {

        float x = event.getX() - this.centerX;
        float y = event.getY() - this.centerY;

        float angle = (float) Math.atan2(y, x);
        float unit = angle / (2 * PI);

        if (unit < 0) {
            unit += 1;
        }

        this.color = interpColor(this.mColors, unit);
        this.mCenterPaint.setColor(this.color);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {

        if (!this.isEnabled())
            return false;

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:

                this.fillPaint(event);
                this.invalidate();
                this.getParent().requestDisallowInterceptTouchEvent(true);

                if (this.mListener != null) {
                    this.mListener.onStartTrackingTouch(this);
                    this.mListener.onColorChanged(this, this.color);
                }

                break;
            case MotionEvent.ACTION_MOVE:

                this.fillPaint(event);
                this.invalidate();
                this.getParent().requestDisallowInterceptTouchEvent(true);

                if (this.mListener != null)
                    this.mListener.onColorChanged(this, this.color);

                break;
            case MotionEvent.ACTION_UP:

                this.fillPaint(event);
                this.invalidate();

                if (this.mListener != null) {
                    this.mListener.onStopTrackingTouch(this);
                    this.mListener.onColorChanged(this, this.color);
                }

                break;
            case MotionEvent.ACTION_CANCEL:
                this.getParent().requestDisallowInterceptTouchEvent(false);
                break;
        }

        return true;
    }

    public interface OnColorChangeListener {

        void onStartTrackingTouch(ColorPicker view);

        void onStopTrackingTouch(ColorPicker view);

        void onColorChanged(ColorPicker view, int color);
    }
}