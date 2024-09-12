#pragma once
#include "bpnp_package_types.h"
#include "crc8.h"

CRC8 crc_pack;


// Функция для упаковки сообщений и возврата буфера
std::vector<uint8_t> packMessage(const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> buffer;
    crc_pack.reset();
    buffer.push_back(PACK_START);
    crc_pack.add(PACK_START);

    // Копируем payload в буфер
    for (uint8_t byte : payload) {
        crc_pack.add(byte);
        // Дублирование ESCAPE_BYTE
        if (byte == ESCAPE_BYTE) {
            buffer.push_back(ESCAPE_BYTE);
        }
        buffer.push_back(byte);
    }

    // Вычисление и добавление CRC
    uint8_t crc_res = crc_pack.calc();
    buffer.push_back(crc_res);
    if (crc_res == ESCAPE_BYTE) {
        buffer.push_back(ESCAPE_BYTE);
    }

    // Добавление байта конца пакета
    buffer.push_back(ESCAPE_BYTE);
    buffer.push_back(PACK_END);

    return buffer;
}

