package com.example.kortium;

import android.annotation.SuppressLint;
import android.content.Context;
import android.os.Bundle;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewTreeObserver;
import android.view.Window;
import android.view.WindowManager;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupMenu;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import androidx.appcompat.widget.Toolbar;
import androidx.core.view.GravityCompat;
import androidx.drawerlayout.widget.DrawerLayout;


import com.google.android.material.navigation.NavigationView;

import jp.wasabeef.blurry.Blurry;

public class MainActivity extends AppCompatActivity {

    private EditText ipAddressField, portField, messageField;
    private TextView outputView, connecting_error;
    private Button connectButton;
    private Button sendButton;
    private Socket socket;
    private BufferedReader input;
    private PrintWriter output;
    private Thread thread;
    private ImageView black_back, error_frame;
    // Переменные для хранения состояния соединения и объектов socket
    private InputStream inputStream = null;
    private boolean isBlurred = false;

    private JoystickView joystick;

    private DrawerLayout drawerLayout;
    private NavigationView navigationView;

    private View dimView;
    private LinearLayout overlayForm;
    private FrameLayout mainFrame;


    @SuppressLint("MissingInflatedId")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Устанавливаем черный фон
        getWindow().getDecorView().setBackgroundColor(getResources().getColor(android.R.color.black));

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

        ipAddressField = findViewById(R.id.ip_address);
        portField = findViewById(R.id.port);
        outputView = findViewById(R.id.output);
        connecting_error = findViewById(R.id.connecting_error);
        messageField = findViewById(R.id.message_field);
        connectButton = findViewById(R.id.connect_button);
        sendButton = findViewById(R.id.send_button);
//        black_back = findViewById(R.id.black_back);
        error_frame = findViewById(R.id.error_frame);

//        sendButton.setVisibility(View.GONE);
        messageField.setVisibility(View.GONE);
//        outputView.setVisibility(View.GONE);
        error_frame.setVisibility(View.GONE);

        mainFrame = findViewById(R.id.background_frame);
        joystick= findViewById(R.id.joystick);



        dimView = findViewById(R.id.dim_view);
        overlayForm = findViewById(R.id.overlay_form);

        ActivateConnecting();


        InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE);
        imm.showSoftInput(ipAddressField, InputMethodManager.SHOW_IMPLICIT);
        imm.showSoftInput(portField, InputMethodManager.SHOW_IMPLICIT);


        connectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String ipAddress = ipAddressField.getText().toString().trim();
                String portText = portField.getText().toString().trim();

                if (ipAddress.isEmpty() || portText.isEmpty()) { // Показать сообщение пользователю, если поля пусты
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            connecting_error.setText("Please enter both IP address and port.");
                            error_frame.setVisibility(View.VISIBLE);
                        }
                    });
                } else {
                    try {
                        int port = Integer.parseInt(portText);
                        thread = new Thread(new ConnectRunnable(MainActivity.this, ipAddress, port));
                        thread.start();
                    } catch (NumberFormatException e) { // Показать сообщение пользователю, если порт не является числом
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                connecting_error.setText("Please enter a valid port number.");
                                error_frame.setVisibility(View.VISIBLE);
                            }
                        });
                    }
                }
            }
        });

        // Код для кнопки отправки сообщения
        sendButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (socket != null && socket.isConnected()) {
                    String message = messageField.getText().toString().trim(); // Поле для ввода сообщения

                    if (!message.isEmpty()) {
                        new Thread(new SendMessageRunnable(message)).start();
                    } else {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                outputView.setText("Please enter a message to send.");
                            }
                        });
                    }
                } else {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            outputView.setText("Not connected to the server.");
                        }
                    });
                }
            }
        });

        // Инициализация DrawerLayout и NavigationView
        drawerLayout = findViewById(R.id.drawer_layout);
        navigationView = findViewById(R.id.navigation_view);

        // Подключение меню
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main_menu, navigationView.getMenu());

        // Инициализация кнопки и установка слушателя
        ImageButton menuButton = findViewById(R.id.menu_button);
        menuButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                drawerLayout.openDrawer(GravityCompat.START); // Открываем DrawerLayout
            }
        });

        // Обработка нажатий на элементы NavigationView
        navigationView.setNavigationItemSelectedListener(new NavigationView.OnNavigationItemSelectedListener() {
            @Override
            public boolean onNavigationItemSelected(@NonNull MenuItem item) {
                // Обработка нажатий на элементы меню

                int id = item.getItemId();

                if (id == R.id.action_messages) {
                    // Обработка нажатия на Home
                    Toast.makeText(MainActivity.this, "Home clicked", Toast.LENGTH_SHORT).show();
                } else if (id == R.id.action_disconnect) {
                    // Обработка нажатия на Settings
                    Toast.makeText(MainActivity.this, "Settings clicked", Toast.LENGTH_SHORT).show();
                } else if (id == R.id.action_messages) {
                    // Обработка нажатия на Messages (если есть такой пункт)
                    Toast.makeText(MainActivity.this, "Messages clicked", Toast.LENGTH_SHORT).show();
                }
                // Закрываем меню после нажатия
                drawerLayout.closeDrawer(GravityCompat.START);
                return true;
            }
        });

    }

    // Runnable для отправки сообщения
    private class SendMessageRunnable implements Runnable {
        private String message;
        public SendMessageRunnable(String message) {
            this.message = message;
        }

        @Override
        public void run() {
            if (output != null) {
                String message = "Hello, world";
                output.println(message);
            }
        }
    }


    public void ActivateConnecting(){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                dimView.setVisibility(View.VISIBLE);
                overlayForm.setVisibility(View.VISIBLE);
//                final ViewGroup rootView = (ViewGroup) findViewById(R.id.background_frame);
//                if (rootView != null) {
//                    rootView.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
//                        @Override
//                        public void onGlobalLayout() {
//                            // Теперь можно безопасно вызывать Blurry
//                        Blurry.with(MainActivity.this).radius(25).onto(rootView);
//                        rootView.getViewTreeObserver().removeOnGlobalLayoutListener(this);
//                        }
//                    });
//                }
//                rootView.getViewTreeObserver().removeOnGlobalLayoutListener(this);
//                ipAddressField.setVisibility(View.VISIBLE);
//                portField.setVisibility(View.VISIBLE);
//                connectButton.setVisibility(View.VISIBLE);
//                connecting_error.setVisibility(View.VISIBLE);
//                final ViewGroup rootView = (ViewGroup) findViewById(R.id.background_frame);
//                Blurry.with(MainActivity.this).radius(53).onto(rootView);
            }
        });
    }

    public void DeactivateConnectingWindow(String ipAddress, int port){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                dimView.setVisibility(View.GONE);
                overlayForm.setVisibility(View.GONE);
//                ipAddressField.setVisibility(View.GONE);
//                portField.setVisibility(View.GONE);
//                connectButton.setVisibility(View.GONE);
//                connecting_error.setVisibility(View.GONE);
                error_frame.setVisibility(View.GONE);
//
//                final ViewGroup rootView = (ViewGroup) findViewById(R.id.background_frame);
//                if (rootView != null) {
//                    Blurry.delete(rootView);
//                }
//                outputView.setVisibility(View.VISIBLE);
//                outputView.setText("Connected to " + ipAddress + ":" + port);

            }
        });
    }

    public void ConnectingError(IOException e){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                error_frame.setVisibility(View.VISIBLE);
                connecting_error.setText("Connection failed: " + e.getMessage());
            }
        });
    }

    private void closeConnection() {
        Logger logger = Logger.getLogger("SocketLogger");
        if (socket != null && !socket.isClosed()) {
            try {
                logger.info("Closing connection");
                socket.close();
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        ActivateConnecting();
                        logger.info("Connection ended");
                    }
                });
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
    public void logPacket(List<Byte> packet) {
        StringBuilder sb = new StringBuilder();
        sb.append("Packet: [");
        for (int i = 0; i < packet.size(); i++) {
            sb.append(String.format("0x%02X", packet.get(i)));
            if (i < packet.size() - 1) {
                sb.append(", ");
            }
        }
        sb.append("]");
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                outputView.append(sb.toString() + "\n");
            }
        });
        Logger.getLogger("PacketLogger").info(sb.toString());
    }


    @Override
    protected void onDestroy() {
        super.onDestroy();
        closeConnection();
    }
}
