package com.example.kortium;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.widget.Toast;

public class MessageReceiver extends BroadcastReceiver {
    @Override
    public void onReceive(Context context, Intent intent) {
        if (intent != null && "NEW_MESSAGE".equals(intent.getAction())) {
            String message = intent.getStringExtra("message");
        }
    }
}
