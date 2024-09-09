package com.example.kortium;

import android.util.Log;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.nio.charset.StandardCharsets;


public class PacketHandler {
    public static final byte PACK_START = 0x10;
    public static final byte ESCAPE_BYTE = 0x10;
    public static final byte PACK_END = 0x03;

    // Device bytes
    public static final byte OPR_ID = (byte) 0xa6;
    public static final byte BRT_ID = (byte) 0xb0;

    // Packet types
    public static final byte LID_ID = 0x53;
    public static final byte CD_ID = 0x50;
    public static final byte BLE_ID = 0x71;
    public static final byte STS_ID = 0x61;
    public static final byte NKR_ID = 0x40;
    public static final byte ATMC_ID = (byte) 0xf1;
    public static final byte ATMI_ID = (byte) 0xf2;
    public static final byte ATMS_ID = (byte) 0xf6;
    public static final byte UTM_ID = (byte) 0xf3;
    public static final byte FTM_ID = (byte) 0xf4;
    public static final byte STM_ID = (byte) 0xf5;
    public static final byte UAV_ID = 0x44;
    public static final byte FCE_ID = 0x60;
    public static final byte FND_ID = 0x72;
    public static final byte TXT_ID = 0x62;
    public static final byte CRD_ID = 0x52;
    public static final byte PTH_ID = 0x51;
    public static final byte AUD_ID = 0x70;
    public static final byte SLI_ID = 0x41;
    public static final byte LLI_ID = 0x42;
    public static final byte RLI_ID = 0x43;
    public static final byte RTS_ID = 0x63;

    public static final Map<Byte, String> SENDERS_BY_ID = new HashMap<>();
    public static final Map<Byte, PacketFormat> PACKS_BY_ID = new HashMap<>();
    public static final Map<String, Byte> PACKS_BY_NAMES = new HashMap<>();

    static {
        SENDERS_BY_ID.put(OPR_ID, "OPERATOR");
        SENDERS_BY_ID.put(BRT_ID, "BORT");

        PACKS_BY_ID.put(LID_ID, new PacketFormat("LID", "<c", "TXT"));
        PACKS_BY_ID.put(CD_ID, new PacketFormat("CD", "<2f", "CRD"));
        PACKS_BY_ID.put(BLE_ID, new PacketFormat("BLE", "<6c", "MAC"));
        PACKS_BY_ID.put(STS_ID, new PacketFormat("STS", "<11b", "STS"));
        PACKS_BY_ID.put(NKR_ID, new PacketFormat("NKR", "<3f", "FLO"));
        PACKS_BY_ID.put(ATMC_ID, new PacketFormat("ATMC", "<5h", "SHO"));
        PACKS_BY_ID.put(ATMI_ID, new PacketFormat("ATMI", "<5h", "SHO"));
        PACKS_BY_ID.put(ATMS_ID, new PacketFormat("ATMS", "<5h", "SHO"));
        PACKS_BY_ID.put(UTM_ID, new PacketFormat("UTM", "<I", "UIN"));
        PACKS_BY_ID.put(FTM_ID, new PacketFormat("FTM", "<3f", "FLO"));
        PACKS_BY_ID.put(STM_ID, new PacketFormat("STM", "<?", "CMD"));
        PACKS_BY_ID.put(UAV_ID, new PacketFormat("UAV", "<5h", "SHO"));
        PACKS_BY_ID.put(FCE_ID, new PacketFormat("FCE", "<5h", "SHO"));
        PACKS_BY_ID.put(FND_ID, new PacketFormat("FND", "<6c", "MAC"));
        PACKS_BY_ID.put(TXT_ID, new PacketFormat("TXT", "<c", "TXT"));
        PACKS_BY_ID.put(CRD_ID, new PacketFormat("CRD", "<2f", "CRD"));
        PACKS_BY_ID.put(PTH_ID, new PacketFormat("PTH", "<2f", "CRD"));
        PACKS_BY_ID.put(AUD_ID, new PacketFormat("AUD", "<5h", "SHO"));
        PACKS_BY_ID.put(SLI_ID, new PacketFormat("SLI", "<?", "CMD"));
        PACKS_BY_ID.put(LLI_ID, new PacketFormat("LLI", "<?", "CMD"));
        PACKS_BY_ID.put(RLI_ID, new PacketFormat("RLI", "<?", "CMD"));
        PACKS_BY_ID.put(RTS_ID, new PacketFormat("RTS", "<11b", "STS"));

        for (Map.Entry<Byte, PacketFormat> entry : PACKS_BY_ID.entrySet()) {
            PACKS_BY_NAMES.put(entry.getValue().getName(), entry.getKey());
        }
    }

    public static class PacketFormat {
        private String name;
        private String format;
        private String type;

        public PacketFormat(String name, String format, String type) {
            this.name = name;
            this.format = format;
            this.type = type;
        }

        public String getName() {
            return name;
        }

        public String getFormat() {
            return format;
        }

        public String getType() {
            return type;
        }
    }
    // Special bytes
}

class PackerAndUnpacker {

    public List<Byte> pack(List<Object> inputData) {
        // Создаем объект DataPacker и упаковываем сообщение
        DataPacker packer = new DataPacker(inputData.get(0));
        byte[] packedData = packer.packMsg(inputData.subList(1, inputData.size()).toArray());


        // Используем PackagePacker для упаковки данных
        byte[] byteStr = new PackagePacker().pack(packedData);

        // Выводим отправленный пакет в HEX формате
        // System.out.println("Отправленный пакет: " + bytesToHexStr(packedData));
        // System.out.println("Данные: " + inputData);

        // Преобразуем byte[] в List<Byte>
        List<Byte> result = new ArrayList<>();
        for (byte b : byteStr) {
            result.add(b);
        }

        return result;
    }


    // Метод для преобразования List<Byte> в byte[]
    public byte[] listToByteArray(List<Byte> byteList) {
        byte[] byteArray = new byte[byteList.size()];
        for (int i = 0; i < byteList.size(); i++) {
            byteArray[i] = byteList.get(i);
        }
        return byteArray;
    }

    public List<Object> unpack(List<Byte> receivedData) {
        PackageFinder finder = new PackageFinder();
        List<Object> resData = new ArrayList<>();
        int counter = 0;

        for (int i = 0; i < receivedData.size(); i++) {
            // Преобразуем элемент списка Byte в массив Byte[], так как метод checkByte ожидает массив
            Byte[] singleByteArray = new Byte[] { receivedData.get(i) };
            Object[] result = finder.checkByte(singleByteArray);

            if (result != null) {
                System.out.println();
                if (result[0] == null) {
                    System.out.println(result[1]);
                } else {
                    String string;
                    if (result[1] instanceof byte[]) {
                        string = bytesToHexStr((byte[]) result[1]);
                    } else {
                        string = result[1].toString();
                    }
                    resData.add(result[0]);
                    resData.add(new DataExtractor(result).extract());
                    // System.out.printf("TYPE: %s, CONTENT: %s%n", result[0].toString().toUpperCase(), string);
                    // for (int l = 0; l < resData.size(); l++) {
                    //     Object element = resData.get(l);
                    //     if (element instanceof Object[]) {
                    //         // Если это массив, конвертируем его в строку
                    //         System.out.printf("DATA[%d]: %s%n", l, Arrays.toString((Object[]) element));
                    //     } else {
                    //         // Если это одиночный объект, просто выводим его
                    //         System.out.printf("DATA[%d]: %s%n", l, element.toString());
                    //     }
                    // }
                    counter++;
                    System.out.println();
                }
                System.out.println("-----------------------------------------------");
            }
        }
        // Возвращает список всех полученных данных из поданной байт-последовательности
        return resData;
    }
    private String bytesToHexStr(byte[] bytes) {
        StringBuilder sb = new StringBuilder();
        for (byte b : bytes) {
            sb.append(String.format("%02X", b));
        }
        return sb.toString();
    }
}

class PackagePacker {

    // Calculate CRC8
    private byte calculateCRC(byte[] byteString) {
        byte crc = 0x00;
        for (byte b : byteString) {
            crc ^= b;
            for (int i = 0; i < 8; i++) {
                if ((crc & 0x80) != 0) {
                    crc = (byte) ((crc << 1) ^ 0x07); // Polynomial for CRC8
                } else {
                    crc <<= 1;
                }
            }
            crc &= 0xFF; // Ensure 8-bit value
        }
        return crc;
    }

    // Pack the line
    public byte[] pack(byte[] line) {
        // Создаем новый массив для результата (добавляем место для начального байта, контрольной суммы и завершающих байтов)
        byte[] result = new byte[line.length + 4]; // 1 байт PACK_START, 1 байт CRC, 2 байта ESCAPE_BYTE и PACK_END
        byte[] crc_list = new byte[line.length + 1];
        int index = 0;

        // 1. Добавляем начальный байт в начало строки
        result[index++] = PacketHandler.PACK_START;
        crc_list[index - 1] = PacketHandler.PACK_START;

        // 2. Копируем оригинальные данные (line)
        System.arraycopy(line, 0, result, index, line.length);
        System.arraycopy(line, 0, crc_list, index, line.length);
        index += line.length;

        // 3. Рассчитываем контрольную сумму для данных без начального байта
        byte crc = calculateCRC(crc_list);

        // 4. Добавляем контрольную сумму в конец
        result[index++] = crc;

        // 5. Добавляем два завершающих байта: ESCAPE_BYTE и PACK_END
        result[index++] = PacketHandler.ESCAPE_BYTE;
        result[index] = PacketHandler.PACK_END;

        // Возвращаем собранный массив
        return result;
    }
}

class DataPacker {

    private byte senderId;
    private List<Object> data;
    private String type;

    public DataPacker(Object sender) {
        if (!(sender instanceof String)) {
            throw new IllegalArgumentException("Sender must be a String");
        }
        String senderStr = (String) sender;

        switch (senderStr) {
            case "OPERATOR":
                this.senderId = PacketHandler.OPR_ID;
                break;
            case "BORT":
                this.senderId = PacketHandler.BRT_ID;
                break;
            default:
                throw new IllegalArgumentException("Invalid sender name: " + senderStr);
        }
    }

    // Метод для упаковки сообщения
    public byte[] packMsg(Object[] dataArray) {
        if (dataArray == null || dataArray.length < 1) {
            throw new IllegalArgumentException("Input array must contain at least one element for type and data.");
        }

        // Первый элемент — это тип сообщения
        String type = (String) dataArray[0];

        // Остальные элементы — это данные
        List<Object> data = new ArrayList<>(Arrays.asList(Arrays.copyOfRange(dataArray, 1, dataArray.length)));


        // Создание строки для лога
//        StringBuilder dataString = new StringBuilder();
//        for (Object obj : data) {
//            dataString.append(obj.toString()).append(", "); // Преобразуем каждый элемент в строку
//        }
//        if (dataString.length() > 0) {
//            dataString.setLength(dataString.length() - 2);
//        }
//        Log.d("DataList", "Data List: " + dataString.toString());


        this.data = data;
        this.type = type;

        if (PacketHandler.PACKS_BY_NAMES.containsKey(type)) {
            String packetType = PacketHandler.PACKS_BY_ID.get(PacketHandler.PACKS_BY_NAMES.get(type)).getType();

            switch (packetType) {
                case "CRD":
                case "TXT":
                case "MAC":
                    return packCount();
                case "STS":
                case "FLO":
                case "SHO":
                case "CMD":
                case "UIN":
                    return packUncount();
                default:
                    throw new IllegalArgumentException("Unsupported packet type: " + packetType);
            }
        } else {
            throw new IllegalArgumentException("Invalid message type: " + type);
        }
    }


    // Преобразование DataPacker в байтовый массив
    public byte[] toByteArray() {
        ByteBuffer buffer = ByteBuffer.allocate(1024).order(java.nio.ByteOrder.LITTLE_ENDIAN);

        // Сначала добавляем senderId
        buffer.put(senderId);

        // Добавляем все элементы из data
        for (Object element : data) {
            if (element instanceof Integer) {
                buffer.putInt((Integer) element);
            } else if (element instanceof Float) {
                buffer.putFloat((Float) element);
            } else if (element instanceof Short) {
                buffer.putShort((Short) element);
            } else if (element instanceof Byte) {
                buffer.put((Byte) element);
            } else if (element instanceof String) {
                byte[] stringBytes = ((String) element).getBytes(StandardCharsets.US_ASCII);
                buffer.put(stringBytes);
            }
            // Добавьте другие типы данных по необходимости
        }

        // Возвращаем байтовый массив
        byte[] result = new byte[buffer.position()];
        buffer.flip();
        buffer.get(result);
        return result;
    }

    private byte[] packUncount() {
        String name = PacketHandler.PACKS_BY_ID.get(PacketHandler.PACKS_BY_NAMES.get(type)).getName();
        fulfill(name);
        ByteBuffer buffer = ByteBuffer.allocate(1024).order(java.nio.ByteOrder.LITTLE_ENDIAN);

        buffer.put(PacketHandler.PACKS_BY_NAMES.get(type));
        buffer.put(senderId);

        if (data.get(0) instanceof List) {
            List<?> firstElementList = (List<?>) data.get(0); // Преобразуем первый элемент в список

            for (Object element : firstElementList) {
                if (element instanceof Double) {
                    // Преобразуем Double в float для записи в 4 байта
                    float floatValue = ((Double) element).floatValue();
                    buffer.putFloat(floatValue);
                } else if (element instanceof Integer) {
                    int intValue = (Integer) element;

                    // Преобразование int в байт (для записи как 1 байт)
                    if (intValue < 0) {
                        buffer.put((byte) intValue);
                        buffer.put((byte) (0xFF));// Записываем как 1 байт
                    } else {
                        // Отрицательные числа, записанные в 1 байт, с корректным представлением
                        buffer.put((byte) (intValue));
                        // buffer.put((byte) (0x00));
                    }
                }
            }
        } else {
            throw new IllegalArgumentException("The first element is not a list");
        }

        byte[] result = new byte[buffer.position()];
        buffer.flip();
        buffer.get(result);
        return result;
    }

    private byte[] packCount() {
        String packetType = PacketHandler.PACKS_BY_ID.get(PacketHandler.PACKS_BY_NAMES.get(type)).getType();
        ByteBuffer buffer = ByteBuffer.allocate(1024).order(java.nio.ByteOrder.LITTLE_ENDIAN);

        buffer.put(PacketHandler.PACKS_BY_NAMES.get(type));
        buffer.put(senderId);

        if (packetType.equals("CRD")) {
            int counter = 2;
            int cnt = 0;
            List<?> firstElementList = (List<?>) data.get(0);
            buffer.put(binaryLength((int) firstElementList.get(0)));
            for (int i = 1; i < firstElementList.size(); i += counter) {
                buffer.putShort((Short) firstElementList.get(i));
                cnt++;
            }
            buffer.put(binaryLength(cnt));

        } else if (packetType.equals("MAC")) {
            int counter = 6;
            int cnt = 0;
            buffer.put(binaryLength(data.size() / counter));
            for (int i = 0; i < data.size(); i++) {
                buffer.put(Byte.parseByte((String) data.get(i), 16));
            }

        } else if (packetType.equals("TXT")) {
            buffer.put(binaryLength(((String) data.get(0)).length()));
            buffer.put(((String) data.get(0)).getBytes(StandardCharsets.US_ASCII));
        }

        byte[] result = new byte[buffer.position()];
        buffer.flip();
        buffer.get(result);
        return result;
    }

    private byte[] binaryLength(int integerLength) {
        return ByteBuffer.allocate(1).order(java.nio.ByteOrder.LITTLE_ENDIAN).put((byte) integerLength).array();
    }

    private void fulfill(String name) {
        if (data.get(0) instanceof List) {
            List<Object> firstList = new ArrayList<>((List<Object>) data.get(0));
            if (name.equals("AUD")) {
                firstList.add(0,0);
                firstList.add(0,0);
                firstList.add(0,0);
                firstList.add(0,0);
            } else if (name.equals("UAV") || name.equals("ATMC")) {
                firstList.add(0,0);
            } else if (name.equals("ATMI") || name.equals("ATMS")) {
                firstList.add(0,0);
                firstList.add(0,0);
            }
            // Преобразуем в изменяемый список // Добавляем ноль
            data.set(0, firstList); // Заменяем первый элемент обновленным списком
        }

    }
}
// 10 40 a6 f0 a7 82 40 25 06 fe 42 b4 f8 b0 c3 18 10 03
// Step 1: Find byte string and extract tuple: (<pack_name>, <body_byte_string>)
class PackageFinder {

    // Special bytes

    private enum Status {
        WAITING_FOR_START(0),
        COLLECTING(1),
        AFTER_ESCAPE_CHAR(2);

        private final int value;

        // Конструктор для передачи значения
        Status(int value) {
            this.value = value;
        }

        // Метод для получения значения
        public int getValue() {
            return value;
        }
    }

    private Status status;
    private ByteBuffer buffer;

    public PackageFinder() {
        this.status = Status.WAITING_FOR_START;
        this.buffer = ByteBuffer.allocate(1024);  // Adjust buffer size as necessary
    }

    private void reset() {
        this.status = Status.WAITING_FOR_START;
        this.buffer.clear();
    }

    private byte calculateCRC(byte[] byteArray) {
        int crc = 0x00;
        for (byte b : byteArray) {
            crc ^= b;
            for (int i = 0; i < 8; i++) {
                if ((crc & 0x80) != 0) {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
                crc &= 0xFF;  // Ensure this is an 8-bit value
            }
        }
        return (byte) crc;
    }

    public Object[] checkByte(Byte[] singleByteArray) {
        if (status == Status.WAITING_FOR_START) {
            if (singleByteArray[0] == PacketHandler.PACK_START) {
                buffer.put(singleByteArray[0]);
                status = Status.COLLECTING;
            }
            return null;
        }

        if (status == Status.COLLECTING) {
            if (singleByteArray[0] != PacketHandler.ESCAPE_BYTE) {
                buffer.put(singleByteArray[0]);
            } else {
                status = Status.AFTER_ESCAPE_CHAR;
            }
            return null;
        }

        if (status == Status.AFTER_ESCAPE_CHAR) {
            if (singleByteArray[0] == PacketHandler.ESCAPE_BYTE) {
                buffer.put(singleByteArray[0]);
                status = Status.COLLECTING;
                return null;
            } else if (singleByteArray[0] == PacketHandler.PACK_END) {
                byte[] packetBytes = Arrays.copyOfRange(buffer.array(), 0, buffer.position());
                byte[] dataBytes = Arrays.copyOfRange(packetBytes, 0, packetBytes.length - 1);
                byte receivedCRC = packetBytes[packetBytes.length - 1];
                byte calculatedCRC = calculateCRC(dataBytes);

                reset();

                if (receivedCRC != calculatedCRC) {
                    return new Object[]{null, "WRONG CRC"};
                }

                Byte packetId = dataBytes[1];
                dataBytes = Arrays.copyOfRange(buffer.array(),1, packetBytes.length);
                PacketHandler.PacketFormat packetFormat = PacketHandler.PACKS_BY_ID.getOrDefault(packetId, new PacketHandler.PacketFormat(null, null, null));
                return new Object[]{packetFormat.getName(), dataBytes};
            } else {
                reset();
                buffer.put(PacketHandler.ESCAPE_BYTE);
                buffer.put(singleByteArray[0]);
                status = Status.COLLECTING;
            }
        }
        return null;
    }
}


class DataExtractor {

    private PacketHandler.PacketFormat packetFormat;
    private byte[] body;

    public DataExtractor(Object[] data) {
        String packetName = (String) data[0];
        this.body = (byte[]) data[1];
        this.packetFormat = PacketHandler.PACKS_BY_ID.get(PacketHandler.PACKS_BY_NAMES.get(packetName));
    }

    public Object extract() {
        String format = packetFormat.getFormat();
        String type = packetFormat.getType();
        String name = packetFormat.getName();

        if (name.equals("CD") || name.equals("BLE") || name.equals("PTH") || name.equals("LID") ||
                name.equals("FND") || name.equals("TXT") || name.equals("CRD")) {

            if (type.equals("TXT")) {
                List<Object> tmp = new ArrayList<>();
                int length = Byte.toUnsignedInt(body[2]);
                for (int i = 0; i < length; i++) {
                    tmp.add(body[3 + i]);  // Assuming single byte for each element
                }
                return tmp.toArray();
            }

            if (type.equals("CRD")) {
                int counter = 8;
                List<Object> tmp = new ArrayList<>();
                tmp.add(Byte.toUnsignedInt(body[2]));
                int length = Byte.toUnsignedInt(body[3]);
                for (int i = 0; i < length; i++) {
                    ByteBuffer buffer = ByteBuffer.wrap(body, 4 + counter * i, counter);
                    tmp.add(buffer.getFloat());  // Adjust as needed based on the format
                }
                return tmp.toArray();
            }

            if (type.equals("MAC")) {
                int counter = 6;
                List<Object> tmp = new ArrayList<>();
                int length = Byte.toUnsignedInt(body[2]);
                for (int i = 0; i < length; i++) {
                    ByteBuffer buffer = ByteBuffer.wrap(body, 3 + counter * i, counter);
                    byte[] macBytes = new byte[counter];
                    buffer.get(macBytes);
                    tmp.add(macBytes);
                }
                return tmp.toArray();
            }

        } else {
            // Default case for other types
            ByteBuffer buffer = ByteBuffer.wrap(body, 2, body.length-2);
            return extractByFormat(buffer, format);
        }

        return null;
    }

    private Object[] extractByFormat(ByteBuffer buffer, String format) {
        // Настройка порядка байтов
        if (format.startsWith("<")) {
            buffer.order(ByteOrder.LITTLE_ENDIAN); // little-endian
            format = format.substring(1);
        } else if (format.startsWith(">")) {
            buffer.order(ByteOrder.BIG_ENDIAN); // big-endian
            format = format.substring(1);
        }

        List<Object> result = new ArrayList<>();
        int i = 0;

        while (i < format.length()) {
            char c = format.charAt(i);

            // Проверка на число перед типом данных
            int count = 1;
            if (Character.isDigit(c)) {
                int start = i;
                while (i < format.length() && Character.isDigit(format.charAt(i))) {
                    i++;
                }
                count = Integer.parseInt(format.substring(start, i));
                c = format.charAt(i);
            }

            // Распаковка данных в зависимости от типа
            for (int j = 0; j < count; j++) {
                switch (c) {
                    case 'c': // Один символ (байт)
                        result.add((char) buffer.get());
                        break;
                    case 'I': // Целое число (4 байта)
                        result.add(buffer.getInt());
                        break;
                    case 'f': // Число с плавающей запятой (4 байта)
                        result.add(buffer.getFloat());
                        break;
                    case 'b': // Байтовое значение (1 байт)
                        result.add(buffer.get());
                        break;
                    case 'h': // Короткое целое число (2 байта)
                        result.add(buffer.getShort());
                        break;
                    case '?': // Логическое значение (1 байт, true/false)
                        result.add(buffer.get() != 0);
                        break;
                    default:
                        throw new IllegalArgumentException("Unknown format: " + c);
                }
            }

            i++;
        }

        return result.toArray();
    }


}