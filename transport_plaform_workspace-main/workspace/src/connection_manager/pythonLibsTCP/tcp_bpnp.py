import struct

import crc8


# Специальные байты
PACK_START = b'\x10'    # Структурый байт: Байт начала пакета
ESCAPE_BYTE = b'\x10'   # Структурый байт: Специальный байт
PACK_END = b'\x03'      # Структурый байт: Байт конца пакета

# Устройства
OPR_ID = b'\xa6'        # Байт устройства: ПО оператора
BRT_ID = b'\xb0'        # Байт устройства: Борт

# Подготовительные пакеты
LID_ID = b'\x53'        # Байт пакета: Получение медиафайлов: ID, название
CD_ID = b'\x50'         # Байт пакета: Координаты запретных зон (N вершин)
BLE_ID = b'\x71'        # Байт пакета: Список MAC адресов для поиска BLE меток

# Регулярные пакеты
STS_ID = b'\x61'        # Байт пакета: Данные о состоянии систем
NKR_ID = b'\x40'        # Байт пакета: Управление НКР
ATMC_ID = b'\xf1'       # Байт пакета: Данные телеметрии (ATMC)
ATMI_ID = b'\xf2'       # Байт пакета: Данные телеметрии (ATMI)
ATMS_ID = b'\xf6'       # Байт пакета: Данные телеметрии (ATMS)
UTM_ID = b'\xf3'        # Байт пакета: Данные телеметрии (UTM)
FTM_ID = b'\xf4'        # Байт пакета: Данные телеметрии (FTM)
STM_ID = b'\xf5'        # Байт пакета: Состояние тормозной системы
UAV_ID = b'\x44'        # Байт пакета: Управление БЛА
FCE_ID = b'\x60'        # Байт пакета: Данные распознавания лиц
FND_ID = b'\x72'        # Байт пакета: Обнаруженные MAC адреса BLE меток
TXT_ID = b'\x62'        # Байт пакета: Текстовые сообщения
CRD_ID = b'\x52'        # Байт пакета: Координаты обнаруженных препятствий

# Единичные пакеты
PTH_ID = b'\x51'        # Байт пакета: Траектория движения НКР
AUD_ID = b'\x70'        # Байт пакета: ID аудиофайла для воспроизведения
SLI_ID = b'\x41'        # Байт пакета: Команды управления стоп сигналами
LLI_ID = b'\x42'        # Байт пакета: Команды управления левым поворотником
RLI_ID = b'\x43'        # Байт пакета: Команды управления левым поворотником
RTS_ID = b'\x63'        # Байт пакета: Перезапуск системы



# ID устройств
SENDERS_BY_ID = {
    OPR_ID: 'OPERATOR',
    BRT_ID: 'BORT'
}

# Структура пакетов
PACKS_BY_ID = {
    # Подготовительные пакеты
    LID_ID: {'name': 'LID', 'format': '<c', 'type': 'TXT'},     # count * char                          COUNT       OK
    CD_ID: {'name': 'CD', 'format': '<2f', 'type': 'CRD'},      # ID, count * 2*float                   COUNT       OK
    BLE_ID: {'name': 'BLE', 'format': '<6c', 'type': 'MAC'},    # count * 6*char                        COUNT       OK

    # Регулярные пакеты
    STS_ID: {'name': 'STS', 'format': '<11b', 'type': 'STS'},   # 11*char                                           OK
    NKR_ID: {'name': 'NKR', 'format': '<3f', 'type': 'FLO'},    # 3*float                                           OK
    ATMC_ID: {'name': 'ATMC', 'format': '<5h', 'type': 'SHO'},  # 5*short                               ISN'T WHOLE OK
    ATMI_ID: {'name': 'ATMI', 'format': '<5h', 'type': 'SHO'},  # 5*short                               ISN'T WHOLE OK
    ATMS_ID: {'name': 'ATMS', 'format': '<5h', 'type': 'SHO'},  # 5*short                               ISN'T WHOLE OK
    UTM_ID: {'name': 'UTM', 'format': '<I', 'type': 'UIN'},     # uint                                              OK
    FTM_ID: {'name': 'FTM', 'format': '<3f', 'type': 'FLO'},    # 3*float                                           OK
    STM_ID: {'name': 'STM', 'format': '<?', 'type': 'CMD'},     # bool                                              OK
    UAV_ID: {'name': 'UAV', 'format': '<5h', 'type': 'SHO'},    # 5*short                               ISN'T WHOLE OK
    FCE_ID: {'name': 'FCE', 'format': '<5h', 'type': 'SHO'},    # 5*short                                           OK
    FND_ID: {'name': 'FND', 'format': '<6c', 'type': 'MAC'},    # count * 6*char                        COUNT       OK
    TXT_ID: {'name': 'TXT', 'format': '<c', 'type': 'TXT'},     # count * char                          COUNT       OK
    CRD_ID: {'name': 'CRD', 'format': '<2f', 'type': 'CRD'},    # ID, count * 2*float                   COUNT       OK

    # Единичные пакеты
    PTH_ID: {'name': 'PTH', 'format': '<2f', 'type': 'CRD'},    # ID, count * 2*float                   COUNT       OK
    AUD_ID: {'name': 'AUD', 'format': '<5h', 'type': 'SHO'},    # 5*short                               ISN'T WHOLE OK
    SLI_ID: {'name': 'SLI', 'format': '<?', 'type': 'CMD'},     # bool                                              OK
    LLI_ID: {'name': 'LLI', 'format': '<?', 'type': 'CMD'},     # bool                                              OK
    RLI_ID: {'name': 'RLI', 'format': '<?', 'type': 'CMD'},     # bool                                              OK
    RTS_ID: {'name': 'RTS', 'format': '<11b', 'type': 'STS'}    # 11*char                                           OK
}




PACKS_BY_NAMES = dict()

for ID in PACKS_BY_ID:
    PACKS_BY_NAMES[PACKS_BY_ID[ID]['name']] = ID


# Step 1: Find byte string and extract tuple: (<pack_name>, <body_byte_string>)
class PackageFinder():
    # input: byte string like b'\xYY\xYY\...\xYY\xYY'
    # output: (<pack_name>, <body_byte_string>)
    from enum import Enum

    class Status(Enum):
        WAITING_FOR_START = 0
        COLLECTING = 1
        AFTER_ESCAPE_CHAR = 2

    def __init__(self):
        self.status = self.Status.WAITING_FOR_START
        self.string = b''

    def _reset_(self):
        self.status = self.Status.WAITING_FOR_START
        self.string = b''

    def _calculate_crc_(self, byte_string):
        hash = crc8.crc8()
        hash.update(byte_string)
        return hash.digest()

    def check_byte(self, byte):
        if self.status == self.Status.WAITING_FOR_START:
            # check if byte is equal to \x10
            if byte == PACK_START:
                self.string += byte
                self.status = self.Status.COLLECTING
            return None

        if self.status == self.Status.COLLECTING:
            # while bytes is equal to \x10 do nothing, else start collecting string
            if byte != ESCAPE_BYTE:
                self.string += byte
            else:
                self.status = self.Status.AFTER_ESCAPE_CHAR
            return None

        if self.status == self.Status.AFTER_ESCAPE_CHAR:
            # if second byte wasn't equal to \x10, start collecting string
            if byte == ESCAPE_BYTE:
                self.string += byte
                self.status = self.Status.COLLECTING
                return None
            elif byte == PACK_END:
                if len(self.string) < 4:
                    self._reset_()
                    return (None, 'TOO SHORT STRING')
                pack = self.string[1:2]
                pack = {'name': None} if pack not in PACKS_BY_ID else PACKS_BY_ID[pack]
                data = self.string[1:-1]
                crc = self._calculate_crc_(self.string[:-1])
                pack_crc = self.string[-1:]
                self._reset_()
                if crc != pack_crc:
                    return (None, 'WRONG CRC')
                return (pack['name'], data)
            else:
                self._reset_()
                self.string += ESCAPE_BYTE
                self.string += byte
                self.status = self.Status.COLLECTING




# Step 2: get value from body byte string
class DataExtracter():
    # input: (<pack_name>, <body_byte_string>)
    # output: (<package_data>) - <package_data> depends on a type of package
    def __init__(self, data):
        self.pack, self.body = data
        if self.pack in [name for name in PACKS_BY_NAMES]:
            self.pack = PACKS_BY_ID[PACKS_BY_NAMES[self.pack]]


    def extract(self):
        print(self.pack['name'])
        if self.pack['name'] in ['CD', 'BLE', 'PTH', 'LID', 'FND', 'TXT', 'CRD']:
            if self.pack['type'] == 'TXT':
                tmp = []
                for i in range(struct.unpack('<B', self.body[2:3])[0]):
                    for elem in struct.unpack(self.pack['format'], self.body[3 + i:4 + i]):
                        tmp.append(elem)
                return self.pack['name'], tuple(tmp)

            if self.pack['type'] == 'CRD':
                counter = 8
                tmp = []
                tmp.append(struct.unpack('<B', self.body[2:3])[0])
                for i in range(struct.unpack('<B', self.body[3:4])[0]):
                    for elem in struct.unpack(self.pack['format'], self.body[4 + counter * i:4 + counter + counter * i]):
                        tmp.append(elem)
                return self.pack['name'], tuple(tmp)

            if self.pack['type'] == 'MAC':
                counter = 6

            tmp = []
            for i in range(struct.unpack('<B', self.body[2:3])[0]):
                for elem in struct.unpack(self.pack['format'], self.body[3 + counter * i:3 + counter + counter * i]):
                    tmp.append(elem)
            return self.pack['name'], tuple(tmp)

        else:
            return self.pack['name'], struct.unpack(self.pack['format'], self.body[2:])


# Step 1: pack values to body
class DataPacker():
    # input: [<sender_name>, <package_name>, [<package_data>]]
    # output: body of a byte string (W/O start byte, doubled escape bytes, crc byte and end byte)

    def __init__(self, sender):
        # Sender

        if sender == 'OPERATOR':
            self.sender_id = OPR_ID
        if sender == 'BORT':
            self.sender_id = BRT_ID


    def _binary_length_(self, integer_length):
        return integer_length.to_bytes(1, byteorder='little')


    def pack_msg(self, type, data):
        self.data = data
        self.type = type
        if type in PACKS_BY_NAMES:
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'CRD':
                return self._pack_count_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'TXT':
                return self._pack_count_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'MAC':
                return self._pack_count_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'STS':
                return self._pack_uncount_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'FLO':
                return self._pack_uncount_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'SHO':
                return self._pack_uncount_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'CMD':
                return self._pack_uncount_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'UIN':
                return self._pack_uncount_()
        else:
            print(type)
            print(PACKS_BY_NAMES)
            raise Exception(f'Wrong message type. "{type}" is no valid message type.')


    def _fulfill_(self, name):
        filler = []
        if name == 'AUD':
            filler = [0, 0, 0, 0]
        if name == 'UAV' or name == 'ATMC':
            filler = [0]
        if name == 'ATMI' or name == 'ATMS':
            filler = [0, 0]
        self.data = filler + self.data

    def _pack_uncount_(self):
        name = PACKS_BY_ID[PACKS_BY_NAMES[self.type]]['name']
        self._fulfill_(name)
        return PACKS_BY_NAMES[self.type] + self.sender_id + struct.pack(PACKS_BY_ID[PACKS_BY_NAMES[self.type]]['format'], *self.data)


    def _pack_count_(self):
        type = PACKS_BY_ID[PACKS_BY_NAMES[self.type]]['type']
        if type == 'CRD':
            body = b''
            counter = 2

            cnt = 0
            for i in range(1, len(self.data), counter):
                body += struct.pack(PACKS_BY_ID[PACKS_BY_NAMES[self.type]]['format'], *self.data[i:i + counter])
                cnt += 1

            return PACKS_BY_NAMES[self.type] + self.sender_id + self._binary_length_(self.data[0]) + self._binary_length_(cnt) + body

        if type == 'MAC':
            body = b''
            counter = 6


            cnt = 0
            for i in range(0, len(self.data), counter):
                for elem in self.data[i:i + counter]:
                    body += int(elem, 16).to_bytes(1, byteorder='little')
                cnt += 1

            return PACKS_BY_NAMES[self.type] + self.sender_id + self._binary_length_(cnt) + body

        if type == 'TXT':
            return PACKS_BY_NAMES[self.type] + self.sender_id + self._binary_length_(len(self.data[0])) + (self.data[0]).encode()



# Step 2: compile a full package byte line
class PackagePacker():
    # input: body of a byte string (W/O start byte, doubled escape bytes, crc byte and end byte)
    # output: whole byte string (WITH start byte, doubled escape bytes, crc byte and end byte)

    def _calculate_crc_(self, byte_string):
        hash = crc8.crc8()
        hash.update(byte_string)
        return hash.digest()

    def pack(self, line):
        return PACK_START + (line + self._calculate_crc_(PACK_START + line)).replace(b'\x10', b'\x10\x10') + ESCAPE_BYTE + PACK_END
