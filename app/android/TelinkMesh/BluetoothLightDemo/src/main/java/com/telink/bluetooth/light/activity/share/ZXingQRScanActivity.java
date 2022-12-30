package com.telink.bluetooth.light.activity.share;

import android.content.DialogInterface;
import android.os.Bundle;
import android.os.Vibrator;
import android.util.Log;

import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;

import androidx.appcompat.app.AlertDialog;
import cn.bingoogolapple.qrcode.core.QRCodeView;
import cn.bingoogolapple.qrcode.zxing.ZXingView;

public class ZXingQRScanActivity extends TelinkBaseActivity implements QRCodeView.Delegate {
    private static final String TAG = ZXingQRScanActivity.class.getSimpleName();

    private ZXingView mZXingView;
    private AlertDialog.Builder confirmDialogBuilder;

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_qrcode_scan);
        setTitle("QRCode-Scan");
        enableBackNav(true);
        mZXingView = findViewById(R.id.zxingview);
        mZXingView.setDelegate(this);
    }

    @Override
    protected void onStart() {
        super.onStart();
        restartCamera();
    }

    private void restartCamera() {
        mZXingView.startCamera();
        mZXingView.startSpotAndShowRect();
    }


    @Override
    protected void onStop() {
        mZXingView.stopCamera(); // stop camera
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        mZXingView.onDestroy();
        super.onDestroy();
    }

    private void vibrate() {
        Vibrator vibrator = (Vibrator) getSystemService(VIBRATOR_SERVICE);
        assert vibrator != null;
        vibrator.vibrate(100);
    }

    @Override
    public void onScanQRCodeSuccess(String mResult) {
        TelinkLog.d(TAG + " result:" + mResult);
        vibrate();
        mZXingView.stopCamera();
        TelinkLog.w("Content compressed: " + mResult);
        mResult = GZIP.hexStringToBytes(mResult);
        mResult = GZIP.decompressed(mResult);
        TelinkLog.w("Content depressed: " + mResult);

        // check qrcode valid
        final QRCodeDataOperator.TmpMesh tmpMesh = QRCodeDataOperator.parseData(mResult);
        if (tmpMesh != null) {
            showConfirmDialog("QR-Code scan success, import this mesh data?", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            QRCodeDataOperator.importData(tmpMesh);
                            setResult(RESULT_OK);
                            finish();
                        }
                    },
                    new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            finish();
                        }
                    });
        } else {
            showConfirmDialog("QRCode parse error, retry?", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    restartCamera();
                }
            }, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    finish();
                }
            });
        }

    }

    @Override
    public void onCameraAmbientBrightnessChanged(boolean isDark) {

    }

    @Override
    public void onScanQRCodeOpenCameraError() {
        Log.e(TAG, "open camera error");

        showConfirmDialog("Open Camera Error!", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                finish();
            }
        }, null);
    }


    public void showConfirmDialog(String msg, DialogInterface.OnClickListener confirmClick, DialogInterface.OnClickListener onCancelClick) {
        if (confirmDialogBuilder == null) {
            confirmDialogBuilder = new AlertDialog.Builder(this);
            confirmDialogBuilder.setCancelable(false);
            confirmDialogBuilder.setTitle("Warning!");
        }
        if (onCancelClick != null) {
            confirmDialogBuilder.setNegativeButton("Cancel", onCancelClick);
        } else {
            confirmDialogBuilder.setNegativeButton(null, null);
        }
        confirmDialogBuilder.setPositiveButton("Confirm", confirmClick);
        confirmDialogBuilder.setMessage(msg);
        confirmDialogBuilder.show();
    }
}