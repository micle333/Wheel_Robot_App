package com.example.kortium;

import android.content.Intent;
import android.view.View;
import android.view.ViewGroup;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

import jp.wasabeef.blurry.Blurry;

class ConnectRunnable implements Runnable { // Runnable для подключения к серверу
    private String ipAddress;
    private int port;
    private InputStream inputStream = null;
    private BufferedReader input;
    private PrintWriter output;
    private Socket socket;
    private MainActivity activity;
    public ConnectRunnable(MainActivity activity, String ipAddress, int port) {
        this.activity = activity;
        this.ipAddress = ipAddress;
        this.port = port;
    }

    @Override
    public void run() {
        Logger logger = Logger.getLogger("SocketLogger");

        try {
            closeConnection();
            try {
                socket = new Socket(ipAddress, port);
                logger.info("Connection started");
                inputStream = socket.getInputStream();
                output = new PrintWriter(socket.getOutputStream(), true);
                logger.info("Connection established with " + ipAddress + ":" + port);

                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        activity.DeactivateConnectingWindow();
                    }
                });
                // Запуск потока для прослушивания сообщений от сервера
                new Thread(new MessageListenerRunnable()).start();

            } catch (IOException e) {
                e.printStackTrace();
                logger.severe("Error during communication with the server: " + e.getMessage());
                activity.ConnectingError(e);

            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
    private class MessageListenerRunnable implements Runnable {
        Logger logger = Logger.getLogger("SocketLogger");
        @Override
        public void run() {
            logger.info("MessageListener Started");
            try {
                logger.info("Try to read buffer");
                BufferedInputStream in = new BufferedInputStream(inputStream);
                List<Byte> packet = new ArrayList<>();
                boolean readingPacket = false;

                int b;
                while ((b = in.read()) != -1) {
                    if (b == 0x10) {  // Начало пакета
                        packet.add((byte) b);
                        b = in.read();
                        if (b == 0x03) {
                            packet.add((byte) b);
                            logger.info("End of packet");
                            readingPacket = false;
                            activity.logPacket(packet);
                            pack(new List[]{packet});
                            DisplayPacket(packet);
                            sendPacket(packet);

                        } else {
                            logger.info("Start adding bites");
                            readingPacket = true;
                            packet.clear();
                            packet.add((byte) 0x10);
                        }
                    }
                    if (readingPacket) {
                        packet.add((byte) b);
                    }
                }
            } catch (IOException e) {
                activity.ActivateConnecting();
                logger.info("Cant Read");
                Logger.getLogger("SocketLogger").severe("Error during listening for messages: " + e.getMessage());
            } finally {
                closeConnection();
            }
        }
        private void sendBroadcastMessage(String message) {
            Intent intent = new Intent("NEW_MESSAGE");
            intent.putExtra("message", message);
            LocalBroadcastManager.getInstance(activity).sendBroadcast(intent);
        }

        public void DisplayPacket(List<Byte> packet) {
            StringBuilder sb = new StringBuilder();
            sb.append("Packet In: [");
            for (int i = 0; i < packet.size(); i++) {
                sb.append(String.format("0x%02X", packet.get(i)));
                if (i < packet.size() - 1) {
                    sb.append(", ");
                }
            }
            sb.append("]");
            sendBroadcastMessage(sb.toString());
            Logger.getLogger("PacketLogger").info(sb.toString());
        }



        public void sendPacket(List<Byte> packet) {
            if (socket != null && !socket.isClosed()) {
                try {
                    // Преобразование List<Byte> в byte[]
                    byte[] byteArray = new byte[packet.size()];
                    for (int i = 0; i < packet.size(); i++) {
                        byteArray[i] = packet.get(i);
                    }
                    // Отправка байтового массива через сокет
                    OutputStream outputStream = socket.getOutputStream();
                    outputStream.write(byteArray);
                    outputStream.flush();  // Убедитесь, что все данные отправлены

                    // Логирование отправленных данных в HEX формате
                    logBytes(byteArray);

                } catch (IOException e) {
                    Logger.getLogger("SocketLogger").severe("Error sending packet: " + e.getMessage());
                }
            }
        }

        // Метод для логирования байтов в HEX формате
        private void logBytes(byte[] byteArray) {
            StringBuilder hexString = new StringBuilder();
            for (byte b : byteArray) {
                hexString.append(String.format("%02x ", b));
            }
            Logger.getLogger("SocketLogger").info("Sent packet: " + hexString.toString().trim());
        }


    }
    private void closeConnection() {
        Logger logger = Logger.getLogger("SocketLogger");
        if (socket != null && !socket.isClosed()) {
            try {
                logger.info("Closing connection");
                socket.close();
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        activity.ActivateConnecting();
                        logger.info("Connection ended");
                    }
                });
            } catch (IOException e) {
                e.printStackTrace();
            }
        }


    }

    private String bytesToHexStr(byte[] byteData) {
        StringBuilder hexString = new StringBuilder();
        for (byte b : byteData) {
            hexString.append(String.format("%02x ", b));
        }
        return hexString.toString().trim();
    }

    // Позволяет отправить один пакет с данными и получить вывод в виде байт-строки
    public byte[] pack(Object[] inputData) {
        PacketHandler.PackagePacker packagePacker = new PacketHandler.PackagePacker();
        PacketHandler.DataPacker packer = new PacketHandler.DataPacker((String) inputData[0]);
        byte[] byteStr = packagePacker.pack(packer);

        // Вывод отправленного пакета в HEX формате
        System.out.println("Отправленный пакет: " + bytesToHexStr(byteStr));
        System.out.println("Данные: " + inputData);

        return byteStr;
    }

    // Позволяет отправить байт-последовательность и получить вывод данных из неё
    public List<Object> unpack(byte[] receivedData) {
        PacketHandler.PackageFinder finder = new PacketHandler.PackageFinder();
        List<Object> resData = new ArrayList<>();
        int counter = 0;

        for (int i = 0; i < receivedData.length; i++) {
            byte[] singleByte = new byte[] { receivedData[i] };
            Object[] result = finder.checkByte(singleByte[0]);

            if (result != null) {
                System.out.println();
                if (result[0] == null) {
                    System.out.println(result[1]);
                } else {
                    String string = (result[1] instanceof byte[])
                            ? bytesToHexStr((byte[]) result[1])
                            : result[1].toString();
                    resData.add(new PacketHandler.DataExtractor(result).extract());
                    System.out.println("TYPE: " + result[0].toString().toUpperCase() + ", CONTENT: " + string);
                    System.out.println("DATA: " + resData.get(counter));
                    counter++;
                    System.out.println();
                }
                System.out.println("-----------------------------------------------");
            }
        }

        // Возвращает список всех полученных данных из поданной байт-последовательности
        return resData;
    }
}
