package com.telink.bluetooth.light.file;

import android.content.Context;
import android.content.SharedPreferences;

/**
 * Created by kee on 2017/12/22.
 */

public class FileSelectorCache {
    private static final String FILE_NAME = "com.telink.bluetooth.light.file.select";


    private static final String KEY_DIR_PATH = "com.telink.bluetooth.light.KEY_DIR_PATH";


    public static void saveDirPath(Context context, String path) {
        SharedPreferences sharedPreferences = context.getSharedPreferences(FILE_NAME, Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPreferences.edit();
        editor.putString(KEY_DIR_PATH, path)
                .apply();
    }

    public static String getDirPath(Context context) {
        SharedPreferences sharedPreferences = context.getSharedPreferences(FILE_NAME, Context.MODE_PRIVATE);
        return sharedPreferences.getString(KEY_DIR_PATH, null);
    }

}
