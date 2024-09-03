package com.example.kortium;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.TextView;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

public class MessagesActivity extends Activity {

    private TextView messagesTextView;
    private final BroadcastReceiver messageReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (intent != null && "NEW_MESSAGE".equals(intent.getAction())) {
                String message = intent.getStringExtra("message");
                updateMessages(message);
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_messages);
        messagesTextView = findViewById(R.id.messages_text_view);

        // Убираем статус-бар и делаем фоном черный цвет
        Window window = getWindow();
        WindowManager.LayoutParams attrs = window.getAttributes();
        attrs.flags |= WindowManager.LayoutParams.FLAG_FULLSCREEN;
        window.setAttributes(attrs);

        // Убираем системные панель
        window.getDecorView().setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                        | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
        );

        // Регистрация BroadcastReceiver
        IntentFilter filter = new IntentFilter("NEW_MESSAGE");
        LocalBroadcastManager.getInstance(this).registerReceiver(messageReceiver, filter);
    }

    public void updateMessages(String message) {
        messagesTextView.append(message + "\n");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        LocalBroadcastManager.getInstance(this).unregisterReceiver(messageReceiver);

    }
}
