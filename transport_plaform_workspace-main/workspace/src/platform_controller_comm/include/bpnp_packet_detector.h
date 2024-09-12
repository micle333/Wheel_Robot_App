#pragma once
#include "bpnp_package_types.h"
#include "crc8.h"


class PacketFinder {
  public:
    enum State {
      WAIT_START,
      IN_PACKET,
      AFTER_ESCAPE,
      DONE
    };

    PacketFinder();
    void reset();
    byte processByte(byte newByte);
    byte getPacketId();
    byte getSenderId();
    byte* getPacketData();
    byte getPacketDataLength();
    std::string getPackNameByID(uint8_t packetId) const; 

  private:
    CRC8 crc;
    static const byte START_BYTE = 0x10;
    static const byte END_BYTE = 0x03;
    static const byte ESCAPE_BYTE = 0x10;

    State currentState;
    byte packetBuffer[256];
    byte packetLength;
    byte packetId;
    byte senderId;
    byte packetData[253];
    byte packetDataLength;
};

PacketFinder::PacketFinder() {
  currentState = WAIT_START;
  packetLength = 0;
  packetDataLength = 0;
}

void PacketFinder::reset() {
  crc.reset();
  packetLength = 0;
  packetDataLength = 0;
}

byte PacketFinder::processByte(byte newByte) {
  switch (currentState) {
    case WAIT_START:
      if (newByte == START_BYTE) {
        reset();
        currentState = IN_PACKET;
        packetLength = 0;
      }
      break;

    case IN_PACKET:
      if (newByte == ESCAPE_BYTE) {
        currentState = AFTER_ESCAPE;
      } else {
        packetBuffer[packetLength++] = newByte;
      }
      break;

    case AFTER_ESCAPE:
      if (newByte == END_BYTE) {
        crc.add(START_BYTE);
        for (int i=0; i<packetLength - 1; i++) {
          crc.add(packetBuffer[i]);
        }
        byte crc_res = crc.calc();
        if (crc_res == packetBuffer[packetLength - 1]) {
          packetId = packetBuffer[0];
          senderId = packetBuffer[1];
          packetDataLength = packetLength - 3; // Исключаем ID пакета и отправитель, а так же CRC
          memcpy(packetData, packetBuffer + 2, packetDataLength);
          currentState = DONE;
          return currentState;
        }
        currentState = WAIT_START;
      } else if (newByte == ESCAPE_BYTE) {
        packetBuffer[packetLength++] = newByte;
        currentState = IN_PACKET;
      } else {
        reset();
        currentState = IN_PACKET;
        packetLength = 0;
      }
      break;
  }
  return currentState;
}

byte PacketFinder::getPacketId() {
  return packetId;
}

byte PacketFinder::getSenderId() {
  return senderId;
}

byte* PacketFinder::getPacketData() {
  currentState = WAIT_START;
  return packetData;
}

byte PacketFinder::getPacketDataLength() {
  return packetDataLength;
}

std::string PacketFinder::getPackNameByID(uint8_t packetId) const {
    switch (packetId) {
        case WHU_ID: return "WHU";    // Запрос ID устройства
        case IAM_ID: return "IAM";    // ID устройства
        case ODO_ID: return "ODO";    // Датчик скорости колеса
        case MR_ID: return "MR";      // Резольвер двигателя
        case MS_ID: return "MS";      // Скорость вала двигателя
        case MM_ID: return "MM";      // Момент на двигателе (команда)
        case SM_ID: return "SM";      // Стоп сигнал вкл/выкл
        case RT_ID: return "RT";      // Правый поворотник вкл/выкл
        case LT_ID: return "LT";      // Левый поворотник вкл/выкл
        case ENC_ID: return "ENC";    // Угол поворота рулевого шагового мотора
        case ST_ID: return "ST";      // Статус
        case RC_ID: return "RC";      // Угол поворота шагового мотора (команда)
        case IMU_ID: return "IMU";    // Показания инерциального модуля
        case UZ_ID: return "UZ";      // Показания ультразвукового дальномера
        case SP_ID: return "SP";      // Гидравлический тормоз (ручник)
        default: return "Unknown";              // Неизвестный ID
    }
}