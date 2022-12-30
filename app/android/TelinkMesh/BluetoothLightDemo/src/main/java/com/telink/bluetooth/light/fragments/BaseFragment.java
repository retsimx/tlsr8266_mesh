package com.telink.bluetooth.light.fragments;

import android.view.View;
import android.widget.TextView;

import com.telink.bluetooth.light.R;
import com.telink.bluetooth.light.TelinkBaseActivity;

import androidx.fragment.app.Fragment;

/**
 * Created by kee on 2017/9/25.
 */

public class BaseFragment extends Fragment {
    public void showToast(CharSequence s) {
        ((TelinkBaseActivity) getActivity()).showToast(s);
    }

    protected void setTitle(View parent, String title) {
        TextView tv_title = parent.findViewById(R.id.tv_title);
        if (tv_title != null) {
            tv_title.setText(title);
        }
    }
}
