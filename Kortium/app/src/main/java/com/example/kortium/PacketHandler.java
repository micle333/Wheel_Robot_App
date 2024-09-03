package com.example.kortium;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.List;


public class PacketHandler {

    // Special bytes
    private static final byte PACK_START = 0x10;
    private static final byte ESCAPE_BYTE = 0x10;
    private static final byte PACK_END = 0x03;

    // Device bytes
    private static final byte OPR_ID = (byte) 0xa6;
    private static final byte BRT_ID = (byte) 0xb0;

    // Packet types
    private static final byte LID_ID = 0x53;
    private static final byte CD_ID = 0x50;
    private static final byte BLE_ID = 0x71;
    private static final byte STS_ID = 0x61;
    private static final byte NKR_ID = 0x40;
    private static final byte ATMC_ID = (byte) 0xf1;
    private static final byte ATMI_ID = (byte) 0xf2;
    private static final byte ATMS_ID = (byte) 0xf6;
    private static final byte UTM_ID = (byte) 0xf3;
    private static final byte FTM_ID = (byte) 0xf4;
    private static final byte STM_ID = (byte) 0xf5;
    private static final byte UAV_ID = 0x44;
    private static final byte FCE_ID = 0x60;
    private static final byte FND_ID = 0x72;
    private static final byte TXT_ID = 0x62;
    private static final byte CRD_ID = 0x52;
    private static final byte PTH_ID = 0x51;
    private static final byte AUD_ID = 0x70;
    private static final byte SLI_ID = 0x41;
    private static final byte LLI_ID = 0x42;
    private static final byte RLI_ID = 0x43;
    private static final byte RTS_ID = 0x63;

    private static final Map<Byte, String> SENDERS_BY_ID = new HashMap<>();
    private static final Map<Byte, PacketFormat> PACKS_BY_ID = new HashMap<>();
    private static final Map<String, Byte> PACKS_BY_NAMES = new HashMap<>();

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

    // Step 1: Find byte string and extract tuple: (<pack_name>, <body_byte_string>)
    public static class PackageFinder {

        private enum Status {
            WAITING_FOR_START,
            COLLECTING,
            AFTER_ESCAPE_CHAR
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

        public Object[] checkByte(byte byteInput) {
            if (status == Status.WAITING_FOR_START) {
                if (byteInput == PACK_START) {
                    buffer.put(byteInput);
                    status = Status.COLLECTING;
                }
                return null;
            }

            if (status == Status.COLLECTING) {
                if (byteInput != ESCAPE_BYTE) {
                    buffer.put(byteInput);
                } else {
                    status = Status.AFTER_ESCAPE_CHAR;
                }
                return null;
            }

            if (status == Status.AFTER_ESCAPE_CHAR) {
                if (byteInput == ESCAPE_BYTE) {
                    buffer.put(byteInput);
                    status = Status.COLLECTING;
                    return null;
                } else if (byteInput == PACK_END) {
                    byte[] packetBytes = Arrays.copyOfRange(buffer.array(), 1, buffer.position());
                    byte[] dataBytes = Arrays.copyOfRange(packetBytes, 0, packetBytes.length - 1);
                    byte receivedCRC = packetBytes[packetBytes.length - 1];
                    byte calculatedCRC = calculateCRC(dataBytes);

                    reset();

                    if (receivedCRC != calculatedCRC) {
                        return new Object[]{null, "WRONG CRC"};
                    }

                    Byte packetId = dataBytes[0];
                    PacketFormat packetFormat = PACKS_BY_ID.getOrDefault(packetId, new PacketFormat(null, null, null));
                    return new Object[]{packetFormat.getName(), dataBytes};
                } else {
                    reset();
                    buffer.put(ESCAPE_BYTE);
                    buffer.put(byteInput);
                    status = Status.COLLECTING;
                }
            }
            return null;
        }
    }

    public class DataExtractor {

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
                ByteBuffer buffer = ByteBuffer.wrap(body, 2, body.length - 2);
                return extractByFormat(buffer, format);
            }

            return null;
        }

        private Object extractByFormat(ByteBuffer buffer, String format) {
            // Add logic to handle various format types (e.g., unpacking floats, ints, etc.)
            // Placeholder for now
            return null;
        }
    }

    public class DataPacker {

        private byte senderId;
        private List<Object> data;
        private String type;

        public DataPacker(String sender) {
            if (sender.equals("OPERATOR")) {
                this.senderId = PacketHandler.OPR_ID;
            } else if (sender.equals("BORT")) {
                this.senderId = PacketHandler.BRT_ID;
            } else {
                throw new IllegalArgumentException("Invalid sender name: " + sender);
            }
        }

        private byte[] binaryLength(int integerLength) {
            return ByteBuffer.allocate(1).order(java.nio.ByteOrder.LITTLE_ENDIAN).put((byte) integerLength).array();
        }

        public byte[] packMsg(String type, List<Object> data) {
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

        private void fulfill(String name) {
            if (name.equals("AUD")) {
                data.add(0, 0);
                data.add(0, 0);
                data.add(0, 0);
                data.add(0, 0);
            } else if (name.equals("UAV") || name.equals("ATMC")) {
                data.add(0, 0);
            } else if (name.equals("ATMI") || name.equals("ATMS")) {
                data.add(0, 0);
                data.add(0, 0);
            }
        }

        private byte[] packUncount() {
            String name = PacketHandler.PACKS_BY_ID.get(PacketHandler.PACKS_BY_NAMES.get(type)).getName();
            fulfill(name);
            ByteBuffer buffer = ByteBuffer.allocate(1024).order(java.nio.ByteOrder.LITTLE_ENDIAN);

            // Directly put the Byte value if PACKS_BY_NAMES.get(type) returns a Byte
            buffer.put(PacketHandler.PACKS_BY_NAMES.get(type));
            buffer.put(senderId);

            // Assume data contains objects that match the format in the packet.
            for (Object element : data) {
                if (element instanceof Float) {
                    buffer.putFloat((Float) element);
                } else if (element instanceof Integer) {
                    buffer.putInt((Integer) element);
                }
                // Add other types as necessary
            }

            byte[] result = new byte[buffer.position()];
            buffer.flip();
            buffer.get(result);
            return result;
        }

        private byte[] packCount() {
            String packetType = PacketHandler.PACKS_BY_ID.get(PacketHandler.PACKS_BY_NAMES.get(type)).getType();
            ByteBuffer buffer = ByteBuffer.allocate(1024).order(java.nio.ByteOrder.LITTLE_ENDIAN);

            // Use the Byte value directly without converting to bytes
            buffer.put(PacketHandler.PACKS_BY_NAMES.get(type));
            buffer.put(senderId);

            if (packetType.equals("CRD")) {
                int counter = 2;
                int cnt = 0;
                buffer.put(binaryLength((int) data.get(0)));
                for (int i = 1; i < data.size(); i += counter) {
                    buffer.putShort((Short) data.get(i));
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

    }

    public class PackagePacker {

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
            ByteBuffer buffer = ByteBuffer.allocate(1024);

            // Add start byte
            buffer.put(PACK_START);

            // Add the line and calculate the CRC
            buffer.put(line);
            buffer.put(calculateCRC(concatArrays(PACK_START, line)));

            // Handle escape bytes and add escape byte at the end
            byte[] escapedBuffer = escapeBytes(buffer.array(), buffer.position());

            // Add end byte
            buffer.clear();
            buffer.put(escapedBuffer);
            buffer.put(ESCAPE_BYTE);
            buffer.put(PACK_END);

            byte[] result = new byte[buffer.position()];
            buffer.flip();
            buffer.get(result);
            return result;
        }

        // Helper function to escape bytes
        private byte[] escapeBytes(byte[] input, int length) {
            ByteBuffer escapedBuffer = ByteBuffer.allocate(length * 2); // Allocate double size for worst case
            for (int i = 0; i < length; i++) {
                if (input[i] == ESCAPE_BYTE) {
                    escapedBuffer.put(ESCAPE_BYTE);
                    escapedBuffer.put(ESCAPE_BYTE);
                } else {
                    escapedBuffer.put(input[i]);
                }
            }
            byte[] result = new byte[escapedBuffer.position()];
            escapedBuffer.flip();
            escapedBuffer.get(result);
            return result;
        }

        // Helper function to concatenate start byte and line array
        private byte[] concatArrays(byte startByte, byte[] line) {
            byte[] combined = new byte[line.length + 1];
            combined[0] = startByte;
            System.arraycopy(line, 0, combined, 1, line.length);
            return combined;
        }
    }

}
