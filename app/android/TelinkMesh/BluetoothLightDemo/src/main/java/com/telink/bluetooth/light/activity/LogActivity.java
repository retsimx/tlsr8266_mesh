package com.telink.bluetooth.light.activity;

import android.content.DialogInterface;
import android.os.Bundle;
import android.os.Handler;
import android.text.TextUtils;
import android.view.View;
import android.widget.EditText;
import android.widget.Toast;

import com.telink.bluetooth.TelinkLog;
import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;
import com.telink.bluetooth.light.TelinkLightApplication;
import com.telink.bluetooth.light.adapter.LogInfoAdapter;
import com.telink.bluetooth.light.model.LogInfo;

import java.text.SimpleDateFormat;
import java.util.Locale;

import androidx.appcompat.app.AlertDialog;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

/**
 * Created by kee on 2017/9/11.
 */

public class LogActivity extends TelinkBaseActivity {
    private AlertDialog dialog;
    private LogInfoAdapter adapter;
    private Handler mHandler = new Handler();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_log_info);
        RecyclerView recyclerView = findViewById(R.id.rv_log);
        adapter = new LogInfoAdapter(this);
        enableBackNav(true);
        setTitle("Log");
        recyclerView.setLayoutManager(new LinearLayoutManager(this));
        recyclerView.setAdapter(adapter);
    }

    public void clear(View view) {
        TelinkLightApplication.getApp().getLogInfoList().clear();
        adapter.notifyDataSetChanged();
    }

    public void refresh(View view) {
        adapter.notifyDataSetChanged();
    }


    public void save(View view) {
        if (dialog == null) {
            AlertDialog.Builder dialogBuilder = new AlertDialog.Builder(this);
            final EditText editText = new EditText(this);
            dialogBuilder.setTitle("Pls input filename(sdcard/" + "TelLog" + "/[filename].text)");
            dialogBuilder.setView(editText);
            dialogBuilder.setNegativeButton("cancel", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.dismiss();
                }
            }).setPositiveButton("confirm", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    if (TextUtils.isEmpty(editText.getText().toString())) {
                        Toast.makeText(LogActivity.this, "fileName cannot be null", Toast.LENGTH_SHORT).show();
                    } else {
                        showToast("saving......");
                        saveInFile(editText.getText().toString().trim());
                    }
                }
            });
            dialog = dialogBuilder.create();
        }
        dialog.show();
    }


    void saveInFile(final String fileName) {
        new Thread(new Runnable() {
            @Override
            public void run() {
                SimpleDateFormat mDateFormat = new SimpleDateFormat("MM-dd HH:mm:ss.SSS", Locale.getDefault());
                final StringBuilder sb = new StringBuilder("TelinkLog\n");
                for (LogInfo logInfo :
                        TelinkLightApplication.getApp().getLogInfoList()) {
                    sb.append(mDateFormat.format(logInfo.millis)).append("/").append(logInfo.tag).append(":")
                            .append(logInfo.logMessage).append("\n");
                }

                TelinkLightApplication.getApp().saveLogInFile(fileName, sb.toString());
                TelinkLog.d("logMessage saved in: " + fileName);
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(LogActivity.this, fileName + " saved", Toast.LENGTH_SHORT).show();
                    }
                });
            }
        }).start();
    }

}
