// Определение констант и структур
#ifndef PACKAGE_TYPES
#define PACKAGE_TYPES

typedef unsigned char byte;

const byte PACK_START = 0x10;    // Байт начала пакета
const byte ESCAPE_BYTE = 0x10;   // Специальный байт
const byte PACK_END = 0x03;      // Байт конца пакета

const byte BRT_ID = 0xb0;    // ID Борта
const byte FPB_ID = 0xb1;    // ID Контроллера переднего силового блока
const byte BPB_ID = 0xb2;    // ID Контроллера заднего силового блока
const byte CMC_ID = 0xb3;     // ID Общий контроллер

const byte WHU_ID = 0x01;    // Запрос ID устройства
const byte IAM_ID = 0x02;    // ID устройства
const byte ODO_ID = 0xb1;    // Датчик скорости колеса
const byte MR_ID = 0xb2;     // Резольвер двигателя
const byte MS_ID = 0xb3;     // Скорость вала двигателя
const byte MM_ID = 0xba;     // Момент на двигателе (команда)
const byte SM_ID = 0xc0;     // Стоп сигнал вкл/выкл
const byte RT_ID = 0xc1;     // Правый поворотник вкл/выкл
const byte LT_ID = 0xc2;     // Левый поворотник вкл/выкл
const byte ENC_ID = 0xe0;    // Угол поворота рулевого шагового мотора
const byte ST_ID = 0xe1;     // Статус
const byte RC_ID = 0xea;     // Угол поворота шагового мотора (команда)
const byte IMU_ID = 0xb0;    // Показания инерциального модуля
const byte UZ_ID = 0xd0;     // Показания ультразвукового дальномера
const byte SP_ID = 0xbb;     // Гидравлический тормоз (ручник)
    
struct IMUData {
    uint16_t gyro[3];
    uint16_t accel[3];
    uint16_t mag[3];
    uint32_t packetCounter;
}__attribute__((packed));

struct SensorData {
    uint8_t sensorID;
    float sensorValue;
    uint32_t packetCounter;
}__attribute__((packed));

struct WheelData {
    uint8_t wheelID;
    float angularVelocity;
    uint32_t packetCounter;
}__attribute__((packed));

struct ResolverData {
    uint8_t motorID;
    float angle;
    uint32_t packetCounter;
}__attribute__((packed));

struct MotorSpeedData {
    uint8_t motorID;
    float angularVelocity;
    uint32_t packetCounter;
}__attribute__((packed));

struct MotorTorqueData {
    uint8_t motorID;
    float torque;
}__attribute__((packed));

struct SteeringAngleData {
    uint8_t motorID;
    float angle;
    uint32_t packetCounter;
}__attribute__((packed));

struct SteeringStatusData {
    uint8_t axisID;
    uint32_t statusByte;
    uint32_t packetCounter;
}__attribute__((packed));

struct SteeringCommandData {
    uint8_t motorID;
    float angle;
}__attribute__((packed));
#endif