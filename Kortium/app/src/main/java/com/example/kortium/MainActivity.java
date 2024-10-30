package com.example.kortium;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
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
import android.widget.VideoView;

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


import com.google.android.exoplayer2.ExoPlayer;
import com.google.android.exoplayer2.MediaItem;
import com.google.android.exoplayer2.ui.PlayerView;
import com.google.android.material.navigation.NavigationView;

import org.videolan.libvlc.LibVLC;
import org.videolan.libvlc.Media;
import org.videolan.libvlc.MediaPlayer;
import org.videolan.libvlc.util.VLCVideoLayout;

import jp.wasabeef.blurry.Blurry;


public class MainActivity extends AppCompatActivity {

    private EditText ipAddressField, portField, messageField;
    private TextView outputView, connecting_error, speed_title, local_x, local_y, url_address;
    private Button connectButton;
    private Button sendButton;
    private Socket socket;
    private BufferedReader input;
    private PrintWriter output;
    private Thread connectRunnable;
    private ImageView black_back, error_frame;
    // Переменные для хранения состояния соединения и объектов socket
    private InputStream inputStream = null;
    private boolean isBlurred = false;

    private JoystickView joystick;

    private DrawerLayout drawerLayout;
    private NavigationView navigationView;

    private View dimView;
    private LinearLayout overlayForm;
    private RelativeLayout mainFrame;

    // Начальное изображение
    final int leftinitialImage = R.drawable.left_turn;
    // Изображение при нажатии
    final int lefttoggledImage = R.drawable.left_turn_active;

    final int rightinitialImage = R.drawable.right_turn;
    // Изображение при нажатии
    final int righttoggledImage = R.drawable.right_turn_active;

    final int lockDeactive = R.drawable.lock_deactive;
    // Изображение при нажатии
    final int lockActive = R.drawable.lock_active;

    // Переменная, чтобы отслеживать состояние кнопки
    final boolean[] leftIsToggled = {false};
    final boolean[] rightIsToggled = {false};

    final boolean[] LockIsToggled = {false};
    ViewGroup rootView;


    // Начальное изображение
    final int initialImage = R.drawable.stop_ic;
    // Изображение при удержании
    final int pressedImage = R.drawable.break_active;

    private Handler handler = new Handler();
    private long lastPacketSentTime = 0; // Время последней отправки пакета
    private static final long PACKET_SEND_DELAY_MS = 100;
    private static final String PREFS_NAME = "MyAppPrefs";
    private static final String IP_ADDRESS_KEY = "ip_address";
    private static final String PORT_KEY = "port";
    private EditText ipAddressEditText, portEditText;

    private final Handler jostick_handler = new Handler();
    private final int PACKET_SEND_INTERVAL_MS = 100; // 10 Гц = каждые 100 мс
    private double lastX = 0.0, lastY = 0.0; // Хранение последних значений
    private boolean joystickMoving = false;

    public String rtspUrl;
    private static final int REQUEST_CODE_SETTINGS = 1;
    private ExoPlayer player;
    private PlayerView playerView;
    LibVLC libVLC;
    private MediaPlayer mediaPlayer;
    private VLCVideoLayout videoView;

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

        ipAddressEditText = findViewById(R.id.ip_address);
        url_address = findViewById(R.id.rtsp_address);
        portEditText = findViewById(R.id.port);
        outputView = findViewById(R.id.output);
        connecting_error = findViewById(R.id.connecting_error);
        messageField = findViewById(R.id.message_field);
        connectButton = findViewById(R.id.connect_button);
        sendButton = findViewById(R.id.send_button);
//        black_back = findViewById(R.id.black_back);
        error_frame = findViewById(R.id.error_frame);
        speed_title = findViewById(R.id.wheel_spped);
        local_x = findViewById(R.id.local_x);
        local_y = findViewById(R.id.local_y);
//        playerView = findViewById(R.id.playerView);

        sendButton.setVisibility(View.GONE);
        messageField.setVisibility(View.GONE);

//        outputView.setVisibility(View.GONE);
        error_frame.setVisibility(View.GONE);

        mainFrame = findViewById(R.id.MainLayout);
        joystick= findViewById(R.id.joystick);
        rootView = (ViewGroup) findViewById(R.id.MainLayout);
        videoView = findViewById(R.id.videoView);
        videoView.setVisibility(View.GONE);




//      Поворотники
        ImageButton leftTurnButton = findViewById(R.id.left_turn);
        ImageButton rightTurnButton = findViewById(R.id.right_turn);
        ImageButton brackeLock = findViewById(R.id.bracke_lock);
        ImageButton stopButton = findViewById(R.id.stop);

        dimView = findViewById(R.id.dim_view);
        overlayForm = findViewById(R.id.overlay_form);

        ActivateConnecting();

        loadPreferences();
        joystick.setLocked(true);  // Блокировка джойстика
        joystick.setAlpha(0.5f);
        // Сохраняем данные при изменении текста
        ipAddressEditText.setOnFocusChangeListener((v, hasFocus) -> {
            if (!hasFocus) {
                savePreferences(); // Сохраняем при потере фокуса
            }
        });

        portEditText.setOnFocusChangeListener((v, hasFocus) -> {
            if (!hasFocus) {
                savePreferences(); // Сохраняем при потере фокуса
            }
        });


        InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE);
        imm.showSoftInput(ipAddressEditText, InputMethodManager.SHOW_IMPLICIT);
        imm.showSoftInput(portEditText, InputMethodManager.SHOW_IMPLICIT);

        String ipAddress = "";
        int port = 8080;

        ConnectRunnable connectRunnable = new ConnectRunnable(MainActivity.this, ipAddress, port);

        connectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String ipAddress = ipAddressEditText.getText().toString().trim();
                String portText = portEditText.getText().toString().trim();

                if (ipAddress.isEmpty() || portText.isEmpty()) { // Показать сообщение пользователю, если поля пусты
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            connecting_error.setText("Please enter both IP address and port.");
                            error_frame.setVisibility(View.VISIBLE);
                        }
                    });
                } else {
                    if (ipAddress.equals("enter")) {
                        Logger logger = Logger.getLogger("SocketLogger") ;
                        logger.info("Enter code input!");
                        DeactivateConnectingWindow();
                    } else {
                        try {
                            int port = Integer.parseInt(portText);
                            // Создайте экземпляр ConnectRunnable
                            connectRunnable.setIpAddressAndPort(ipAddress, port);
                            Thread thread = new Thread(connectRunnable);
                            thread.start();


                            connectRunnable.addTask(() -> {
                                connectRunnable.connectingToSocket();
                                startSendingPackets(connectRunnable);
                            });

                        } catch (
                                NumberFormatException e) { // Показать сообщение пользователю, если порт не является числом
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

        joystick.setJoystickListener(new JoystickView.JoystickListener() {
            @Override
            public void onJoystickMove(float x, float y) {
                // Преобразование координат джойстика
//                Log.d("Joystick", "Joystick moved Input: x=" + x + ", y=" + y);
                double x1 = (x - 270) / 150;
                double y1 = (y - 270) / 150 * -1;

                int x_coeff = 1;
                int y_coeff = 1;
                if (x1 < 0){ x_coeff = -1;}
                if (y1 < 0){ y_coeff = -1;}

                double a = sqrt(x1*x1 + y1*y1);

                double y2 = y1*1/a;
                double x2 = x1 * 1/a;

                double xk = 0;
                double yk = 0;
                if (abs(x1) < abs(y1)){
                    xk = x1*1/y1;
                    yk = 1;
                } else {
                    yk = y1*1/x1;
                    xk = 1;
                }


                double yt = y1* yk /y2;
                double xt = x1 * xk /x2;

                double normalizedY = abs(yt) * y_coeff;
                double normalizedX = abs(xt) * x_coeff;

//                List<Object> inputData = new ArrayList<>();
//                inputData.add("OPERATOR");
//                inputData.add("NKR");
//                List<Object> nestedList = new ArrayList<>(Arrays.asList(normalizedY, normalizedX, 0.0, 0.0));
//                inputData.add(nestedList);
//
//                connectRunnable.addTask(() -> connectRunnable.sendPacket(inputData));

                lastX = normalizedX;
                lastY = normalizedY;
                joystickMoving = true;

//
//                // Получаем текущее время
//                long currentTime = System.currentTimeMillis();
//
//                // Проверяем, прошло ли достаточно времени с последней отправки пакета
//                if (currentTime - lastPacketSentTime >= PACKET_SEND_DELAY_MS) {
//                    // Обновляем время последней отправки
//                    lastPacketSentTime = currentTime;
//
//                    List<Object> inputData = new ArrayList<>();
//                    inputData.add("OPERATOR");
//                    inputData.add("NKR");
////                    normalizedY, normalizedX
//                    List<Object> nestedList = new ArrayList<>(Arrays.asList(normalizedY, normalizedX, 0.0, 0.0));
//                    inputData.add(nestedList);
//
//                    PackerAndUnpacker packerAndUnpacker = new PackerAndUnpacker();
//                    List<Byte> packed_byte = packerAndUnpacker.pack(inputData);
//
//                    // Отправляем пакет с задержкой
//                    handler.post(() -> {
//                        connectRunnable.addTask(() -> connectRunnable.sendPacket(inputData));
//
//                        // Обработка координат джойстика
//                        Log.d("Joystick", "Joystick moved: x=" + normalizedX + ", y=" + normalizedY + ", A =" + a);
//                    });
//                }
            }
            @Override
            public void onJoystickRelease() { // код для обработки отпускания джостика
                lastX = 0.0;
                lastY = 0.0;
                joystickMoving = false;
                sendPacketWithLastData(connectRunnable);
//                List<Object> inputData = new ArrayList<>();
//                inputData.add("OPERATOR");
//                inputData.add("NKR");
//                List<Object> nestedList = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0));
//                inputData.add(nestedList);
//
//                handler.post(() -> {
//                    connectRunnable.addTask(() -> connectRunnable.sendPacket(inputData));
//                    // Обработка координат джойстика
//                    Log.d("Joystick", "Joystick moved: x=" + 0 + ", y=" + 0);
//                });

                Log.d("Joystick", "Joystick released");

            }
        });


        leftTurnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Переключение изображения
                if (leftIsToggled[0]) {
                    leftTurnButton.setImageResource(leftinitialImage);
                    connectRunnable.addTask(() -> {
                        List<Object> inputData = Arrays.asList("OPERATOR","RLI",Arrays.asList(0));
                        connectRunnable.sendPacket(inputData);
                    });

                } else {
                    leftTurnButton.setImageResource(lefttoggledImage);
                    connectRunnable.addTask(() -> {
                        List<Object> inputData = Arrays.asList("OPERATOR","RLI",Arrays.asList(1));
                        connectRunnable.sendPacket(inputData);
                    });
                }
                leftIsToggled[0] = !leftIsToggled[0]; // Смена состояния
            }
        });

        rightTurnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Переключение изображения
                if (rightIsToggled[0]) {
                    rightTurnButton.setImageResource(rightinitialImage);
                    connectRunnable.addTask(() -> {
                        List<Object> inputData = Arrays.asList("OPERATOR","LLI",Arrays.asList(0));
                        connectRunnable.sendPacket(inputData);
                    });

                } else {
                    rightTurnButton.setImageResource(righttoggledImage);
                    connectRunnable.addTask(() -> {
                        List<Object> inputData = Arrays.asList("OPERATOR","LLI",Arrays.asList(1));
                        connectRunnable.sendPacket(inputData);
                    });
                }
                rightIsToggled[0] = !rightIsToggled[0]; // Смена состояния
            }
        });

        brackeLock.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Переключение изображения
                if (LockIsToggled[0]) {
                    brackeLock.setImageResource(lockDeactive);
                       // Разблокировка джойстика
                    connectRunnable.addTask(() -> {
                        List<Object> inputData = Arrays.asList("OPERATOR","STP",Arrays.asList(0));
                        connectRunnable.sendPacket(inputData);
                    });
                    joystick.setLocked(false);
                    joystick.setAlpha(1.0f);
                } else {
                    brackeLock.setImageResource(lockActive);
                    connectRunnable.addTask(() -> {
                        List<Object> inputData = Arrays.asList("OPERATOR","STP",Arrays.asList(1));
                        connectRunnable.sendPacket(inputData);
                    });
                    joystick.setLocked(true);
                    // Блокировка джойстика
                    joystick.setAlpha(0.5f);
                }
                LockIsToggled[0] = !LockIsToggled[0]; // Смена состояния
            }
        });




        stopButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        // Меняем изображение на нажатое
                        stopButton.setImageResource(pressedImage);
                        connectRunnable.addTask(() -> {
                            List<Object> inputData = Arrays.asList("OPERATOR","SLI",Arrays.asList(1));
                            connectRunnable.sendPacket(inputData);
                        });
                        break;
                    case MotionEvent.ACTION_UP:
                    case MotionEvent.ACTION_CANCEL:
                        connectRunnable.addTask(() -> {
                            List<Object> inputData = Arrays.asList("OPERATOR","SLI",Arrays.asList(0));
                            connectRunnable.sendPacket(inputData);
                        });
                        // Возвращаем начальное изображение после отпускания
                        stopButton.setImageResource(initialImage);
                        break;
                }
                return true; // Обрабатываем событие нажатия
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
                    Intent intent = new Intent(MainActivity.this, MessagesActivity.class);
                    startActivity(intent);
                } else if (id == R.id.action_disconnect) {
                    // Обработка нажатия на Settings
                    connectRunnable.addTask(() -> {
                        connectRunnable.closeConnection();
                    });
                    Toast.makeText(MainActivity.this, "close connecting", Toast.LENGTH_SHORT).show();
                } else if (id == R.id.telemetry) {
                    Intent intent = new Intent(MainActivity.this, TelemetryActivity.class);
                    startActivity(intent);
                } else if (id == R.id.settings) {
                    Intent intent = new Intent(MainActivity.this, SettingsActivity.class);
                    startActivityForResult(intent, 1);
                }
                // Закрываем меню после нажатия
                drawerLayout.closeDrawer(GravityCompat.START);
                return true;
            }
        });

    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == 1 && resultCode == RESULT_OK) {
            rtspUrl = data.getStringExtra("RTSP_URL");
            if (rtspUrl != null) {
                initializePlayer(rtspUrl);
            }
        }
    }

    private void initializePlayer(String rtspUrl) {
        url_address.setText(rtspUrl);
        videoView.setVisibility(View.VISIBLE);

        libVLC = new LibVLC(this);
        Media media = new Media(libVLC, Uri.parse(rtspUrl));
        media.addOption("--aout=opensles");
        media.addOption("--audio-time-stretch");
        media.addOption("-vvv");

        mediaPlayer = new MediaPlayer(libVLC);
        mediaPlayer.setMedia(media);
        mediaPlayer.attachViews(videoView, null, false, false);
        mediaPlayer.play();
    }

    private void sendPacketWithLastData(ConnectRunnable connectRunnable) {

//        List<Object> inputData = new ArrayList<>();
//        inputData.add("OPERATOR");
//        inputData.add("NKR");
//        List<Object> nestedList = new ArrayList<>(Arrays.asList(normalizedY, normalizedX, 0.0, 0.0));
//        inputData.add(nestedList);
//
//        PackerAndUnpacker packerAndUnpacker = new PackerAndUnpacker();
//        List<Byte> packed_byte = packerAndUnpacker.pack(inputData);
//
//        // Отправляем пакет с задержкой
//        handler.post(() -> {
//            connectRunnable.addTask(() -> connectRunnable.sendPacket(inputData));
//
//            // Обработка координат джойстика
//            Log.d("Joystick", "Joystick moved: x=" + normalizedX + ", y=" + normalizedY + ", A =" + a);
//        });

        List<Object> inputData = new ArrayList<>();
        inputData.add("OPERATOR");
        inputData.add("NKR");

        // Используем последние значения координат
        List<Object> nestedList = new ArrayList<>(Arrays.asList(lastY,  -1 * lastX, 0.0, 0.0));
        inputData.add(nestedList);

        PackerAndUnpacker packerAndUnpacker = new PackerAndUnpacker();
        List<Byte> packed_byte = packerAndUnpacker.pack(inputData);

        // Отправляем пакет через Runnable
        connectRunnable.addTask(() -> connectRunnable.sendPacket(inputData));

        Log.d("Joystick", "Packet sent: x=" + lastX + ", y=" + lastY);
    }

    private void startSendingPackets(ConnectRunnable connectRunnable) {
        jostick_handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (joystickMoving) {
                    sendPacketWithLastData(connectRunnable); // Отправляем данные
                } jostick_handler.postDelayed(this, PACKET_SEND_INTERVAL_MS); // Повторяем через 100 мс
            }
        }, PACKET_SEND_INTERVAL_MS);
    }

    // Запускаем периодическую отправку пакетов

    private void savePreferences() {
        SharedPreferences sharedPreferences = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPreferences.edit();

        String ipAddress = ipAddressEditText.getText().toString();
        String port = portEditText.getText().toString();

        editor.putString(IP_ADDRESS_KEY, ipAddress);
        editor.putString(PORT_KEY, port);

        editor.apply(); // Сохраняем данные
    }

    private void loadPreferences() {
        SharedPreferences sharedPreferences = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);

        // Загружаем сохраненные данные или задаем значения по умолчанию
        String savedIpAddress = sharedPreferences.getString(IP_ADDRESS_KEY, "192.168.2.235");
        String savedPort = sharedPreferences.getString(PORT_KEY, "8080");

        // Заполняем EditText сохраненными данными
        ipAddressEditText.setText(savedIpAddress);
        portEditText.setText(savedPort);
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
//
                if (rootView != null) {
                    rootView.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
                        @Override
                        public void onGlobalLayout() {
                            // Теперь можно безопасно вызывать Blurry
                        Blurry.with(MainActivity.this).radius(25).onto(rootView);
                        rootView.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                        }
                    });
                }
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

    public void DeactivateConnectingWindow(){
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
//                final ViewGroup rootView = (ViewGroup) findViewById(R.id.MainLayout);
                Blurry.delete(rootView);
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

    public void ChangeSpeed(String speed){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                speed_title.setText(speed);
            }
        });
    }

    public void ChangeLocalCoordinates(String x, String y){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                local_x.setText(x);
                local_y.setText(y);
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
                logger.info("Cant close connection");
                e.printStackTrace();
            }
        } else {
            logger.info("Cant close connection");
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
//        runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                outputView.append(sb.toString() + "\n");
//            }
//        });
        Logger.getLogger("PacketLogger").info(sb.toString());
    }
    @Override
    protected void onStop() {
        super.onStop();
        if (player != null) {
            player.pause();
        }
    }
    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (player != null) {
            player.release();
            player = null;
        }
        closeConnection();
    }
}
