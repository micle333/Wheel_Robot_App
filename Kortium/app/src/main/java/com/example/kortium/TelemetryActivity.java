package com.example.kortium;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

public class TelemetryActivity extends AppCompatActivity {

    private TextView pdopTextView, satellitesTextView, snsModeTextView, humidityTextView, pressureTextView, temperatureTextView,  longetudeTextView, altitudeTextView;

    public String pdop, satellites, snsMode, altitude, longetude, humidity, pressure, temperature;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_telemetry);

        pdopTextView = findViewById(R.id.pdopTextView);
        satellitesTextView = findViewById(R.id.satellitesTextView);
        snsModeTextView = findViewById(R.id.snsModeTextView);
        altitudeTextView = findViewById(R.id.altitudeTextView);
        longetudeTextView = findViewById(R.id.longetudeTextView);
        humidityTextView = findViewById(R.id.humidityTextView);
        pressureTextView = findViewById(R.id.pressureTextView);
        temperatureTextView = findViewById(R.id.temperatureTextView);

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
    }

    private BroadcastReceiver telemetryReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String pdop = intent.getStringExtra("PDOP");
            String satellites = intent.getStringExtra("SATELLITES");
            String snsMode = intent.getStringExtra("SNS_MODE");
            String altitude = intent.getStringExtra("ALTI");
            String longitude = intent.getStringExtra("LONGE");
            String humidity = intent.getStringExtra("HUMIDITY");
            String pressure = intent.getStringExtra("PRESSURE");
            String temperature = intent.getStringExtra("TEMPERATURE");

            // Проверяем, что пришли новые данные, если нет - оставляем старое значение
            if (pdop != null) {
                updatePdop(pdop);
            }
            if (satellites != null) {
                updateSatellites(satellites);
            }
            if (snsMode != null) {
                updateSnsMode(snsMode);
            }
            if (altitude != null && longitude != null) {
                updateCoordinates(altitude, longitude);
            }
            if (humidity != null) {
                updateHumidity(humidity);
            }
            if (pressure != null) {
                updatePressure(pressure);
            }
            if (temperature != null) {
                updateTemperature(temperature);
            }
        }
    };

    @Override
    protected void onResume() {
        super.onResume();
        IntentFilter filter = new IntentFilter("TELEMETRY_UPDATE");
        registerReceiver(telemetryReceiver, filter);
    }

    @Override
    protected void onPause() {
        super.onPause();
        unregisterReceiver(telemetryReceiver);
    }

    // Методы для обновления данных в UI
    public void updatePdop(String pdop) {
        pdopTextView.setText(pdop);
    }

    public void updateSatellites(String satellites) {
        satellitesTextView.setText(satellites);
    }

    public void updateSnsMode(String snsMode) {
        snsModeTextView.setText(snsMode);
    }

    public void updateCoordinates(String alti, String longe) {
        altitudeTextView.setText(alti);
        longetudeTextView.setText(longe);
    }

    public void updateHumidity(String humidity) {
        humidityTextView.setText(humidity + "%");
    }

    public void updatePressure(String pressure) {
        pressureTextView.setText(pressure + "mm");
    }

    public void updateTemperature(String temperature) {
        temperatureTextView.setText(temperature + "C");
    }
}
