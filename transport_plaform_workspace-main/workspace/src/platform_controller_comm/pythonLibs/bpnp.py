import crc8
import struct


PACK_START = b'\x10'    # Байт начала пакета
ESCAPE_BYTE = b'\x10'   # Специальный байт
PACK_END = b'\x03'      # Байт конца пакета

BRT_ID = b'\xb0'    # ID Борта
FPB_ID = b'\xb1'    # ID Контроллера переднего силового блока
BPB_ID = b'\xb2'    # ID Контроллера заднего силового блока
CMC_ID = b'\xb3'    # ID Контроллера общего назначения

WHU_ID = b'\x01'    # Запрос ID устройства
IAM_ID = b'\x02'    # ID устройства
ODO_ID = b'\xb1'    # Датчик скорости колеса
MR_ID = b'\xb2'     # Резольвер двигателя
MS_ID = b'\xb3'     # Скорость вала двигателя
MM_ID = b'\xba'     # Момент на двигателе (команда)
SM_ID = b'\xc0'     # Стоп сигнал вкл/выкл
RT_ID = b'\xc1'     # Правый поворотник вкл/выкл
LT_ID = b'\xc2'     # Левый поворотник вкл/выкл
ENC_ID = b'\xe0'    # Угол поворота рулевого шагового мотора
ST_ID = b'\xe1'     # Статус
RC_ID = b'\xea'     # Угол поворота шагового мотора (команда)
IMU_ID = b'\xb0'    # Показания инерциального модуля
UZ_ID = b'\xd0'     # Показания ультразвукового дальномера
SP_ID = b'\xbb'     # Гидравлический тормоз (ручник)

# ID сенсоров
FRONT_LEFT_WHEEL_SENSOR = 7
FRONT_RIGHT_WHEEL_SENSOR = 8
REAR_LEFT_WHEEL_SENSOR = 10
REAR_RIGHT_WHEEL_SENSOR = 11
FRONT_RESOLVER = 209
REAR_RESOLVER = 210
FRONT_MOTOR_SHAFT = 211
REAR_MOTOR_SHAFT = 212
FRONT_AXIS_ENC = 193
REAR_AXIS_ENC = 194
FRONT_AXIS_SWITCH = 195
REAR_AXIS_SWITCH = 196


SENDERS_BY_ID = {
    BRT_ID: 'BORT',
    FPB_ID: 'FPU',
    BPB_ID: 'BPU',
    CMC_ID: 'CMC'
}

PACKS_BY_ID = {
    WHU_ID: {'name': 'WHU', 'length': 0, 'format': '<', 'type': 'BLC'},
    IAM_ID: {'name': 'IAM', 'length': 0, 'format': '<', 'type': 'BLC'},
    ODO_ID: {'name': 'ODO', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    MR_ID: {'name': 'MR', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    MS_ID: {'name': 'MS', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    MM_ID: {'name': 'MM', 'length': 5, 'format': '<Bf', 'type': 'MM'},
    SM_ID: {'name': 'SM', 'length': 1, 'format': '<B', 'type': 'CMD'},
    RT_ID: {'name': 'RT', 'length': 1, 'format': '<B', 'type': 'CMD'},
    LT_ID: {'name': 'LT', 'length': 1, 'format': '<B', 'type': 'CMD'},
    ENC_ID: {'name': 'ENC', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    ST_ID: {'name': 'ST', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    RC_ID: {'name': 'RC', 'length': 5, 'format': '<Bf', 'type': 'MM'},
    IMU_ID: {'name': 'IMU', 'length': 22, 'format': '<9hI', 'type': 'IMU'},
    UZ_ID: {'name': 'UZ', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    SP_ID: {'name': 'SP', 'length': 1, 'format': '<B', 'type': 'CMD'}
}


# additional packs may be added to the dicts on the top. Further code should not be touched


PACKS_BY_NAMES = dict()

for ID in PACKS_BY_ID:
    PACKS_BY_NAMES[PACKS_BY_ID[ID]['name']] = ID

# Step 1: Find byte string and extract tuple: (<pack_name>, <body_byte_string>)
class PackageFinder():
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

    def __init__(self, data):
        self.pack, self.body = data
        if self.pack in [name for name in PACKS_BY_NAMES]:
            self.pack = PACKS_BY_ID[PACKS_BY_NAMES[self.pack]]


    def extract(self):
        if self.pack == None:
            return 'UNDEFINED PACK TYPE'
        return struct.unpack(self.pack['format'], self.body[2:])


# Step 1: pack values to body
class DataPacker():

    def __init__(self, sender):
        # Sender
        self.data = []
        if sender == 'BORT':
            self.sender_id = BRT_ID
        if sender == 'FPU':
            self.sender_id = FPB_ID
        if sender == 'BPU':
            self.sender_id = BPB_ID
        if sender == 'CMC':
            self.sender_id = CMC_ID


    def _binary_length_(self, integer_length):
        print(integer_length)
        return integer_length.to_bytes(1, byteorder='little')


    def pack_msg(self, type, data):
        self.data = data
        self.type = type
        if type in PACKS_BY_NAMES:

            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'CMD':
                return self._pack_cmd_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'BLC':
                return self._pack_blc_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'SID':
                return self._pack_sid_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'MM':
                return  self._pack_mm_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'IMU':
                return self._pack_imu_()

        else:
            raise Exception(f'Wrong message type. "{type}" is no valid message type. '
                            'Valid message types:'
                            '\nWHU'
                            '\nIAM'
                            '\nODO'
                            '\nMR'
                            '\nMS'
                            '\nMM'
                            '\nSM'
                            '\nRT'
                            '\nLT'
                            '\nENC'
                            '\nST'
                            '\nRC'
                            '\nIMU'
                            '\nUZ')


    def _pack_cmd_(self):
        return PACKS_BY_NAMES[self.type] + self.sender_id + self._binary_length_(*self.data)


    def _pack_blc_(self):
        return PACKS_BY_NAMES[self.type] + self.sender_id


    def _pack_sid_(self):
        type = PACKS_BY_NAMES[self.type]
        payload = type + self.sender_id + struct.pack(PACKS_BY_ID[type]['format'], *self.data)
        return payload


    def _pack_mm_(self):
        type = PACKS_BY_NAMES[self.type]
        payload = type + self.sender_id + struct.pack(PACKS_BY_ID[type]['format'], *self.data)
        return payload

    def _pack_imu_(self):
        type = PACKS_BY_NAMES[self.type]
        payload = type + self.sender_id + struct.pack(PACKS_BY_ID[type]['format'], *self.data)
        return payload


# Step 2: compile a full package byte line
class PackagePacker():

    def _calculate_crc_(self, byte_string):
        hash = crc8.crc8()
        hash.update(byte_string)
        return hash.digest()

    def pack(self, line):
        return PACK_START + (line + self._calculate_crc_(PACK_START + line)).replace(b'\x10', b'\x10\x10') + ESCAPE_BYTE + PACK_END



if __name__ == '__main__':

    form = lambda x: ' '.join(format(byte, '02x') for byte in x)

    inputs = [['FPU', 'SM', 1], ['FPU', 'RT', 0], ['FPU', 'LT', 1], ['FPU', 'WHU'], ['FPU', 'IAM'], ['FPU', 'ODO', 5, -0.3, 14],
              ['BPU', 'MR', 16, -123, 123], ['FPU', 'MS', 123, 9, 0], ['BORT', 'MM', 16, 16.4],
              ['FPU', 'ENC', 111, 3.5223, 7], ['FPU', 'ST', 55, 36, 33], ['FPU', 'RC', 2, 123.512],
              ['BORT', 'IMU', 3, 4, 5, 9, 16, 25, 27, 64, 125, 50], ['BPU', 'UZ', 5, 123.0, 55],
              ['BPU', 'SP', 1], ['FPU', 'MR', 16, 36, 4338], ['CMC', 'ODO', 7, 3.5223, 7],['CMC', 'ODO', 8, 3.5223, 7], ['CMC', 'ODO', 10, 3.5223, 7],['CMC', 'ODO', 11, 3.5223, 7]]

    # ['FPU', 'MR', 16, 36, 4338] - несколько \x10: \x10 в ID, \x10 в float, \x10 в Cnt и \x10 в CRC
    package_packer = PackagePacker()
    for inp in inputs:
        data_packer = DataPacker(inp[0])
        body_byte = data_packer.pack_msg(inp[1], inp[2:])
        byte_str = package_packer.pack(body_byte)

        finder = PackageFinder()
        print(form(byte_str))
        for i in range(len(byte_str)):
            result = finder.check_byte(byte_str[i:i + 1])

        extracted_data = DataExtracter(result).extract()

        sender_n_pack = f'SENDER: {SENDERS_BY_ID[result[1][1:2]]}, PACK: {PACKS_BY_ID[result[1][0:1]]["name"]}, '
        if result[0] == 'BLC':
            print(sender_n_pack[:-2])
        if result[0] == 'CMD':
            print(sender_n_pack + f'VALUE: {extracted_data[0]}')
        if result[0] == 'MM':
            print(sender_n_pack + f'ID: {extracted_data[0]}, VALUE: {extracted_data[1]}')
        if result[0] == 'SID':
            print(sender_n_pack + f'ID: {extracted_data[0]}, VALUE: {extracted_data[1]}, COUNTER: {extracted_data[2]}')
        if result[0] == 'IMU':
            print(sender_n_pack + ('\nW_X: {}, W_Y: {}, W_Z: {},\nA_X: {}, A_Y: {}, A_Z: {},\nB_X: {}, B_Y: {}, B_Z: {},\n'
                                    'COUNTER: {}'.format(*extracted_data)))

        print(f'INPUT: {", ".join(map(str, inp))}\n')
        print(f'BODY BYTE STRING: {form(body_byte)}\n')
        print(f'WHOLE BYTE STRING: {form(byte_str)}\n')
        print('PACKAGE ID: {}, BODY BYTE STRING: {}\n'.format(result[0], result[1] if type(result[1]) != type(b'') else form(result[1])))
        print(extracted_data)
        print('----------------------')


    print('\n\nCHECKING IF PACKAGE_FINDER IS WORKING FINE\n\n')

    byte_string = (b'\x10\xb1\xb2\x10\x10\x00'
                   b'\x10\xb0\xb0\x03\x00\x04\x00\x05\x00\x09\x00\x10\x10\x00\x19\x00\x1b\x00\x40\x00\x7d\x00\x32\x00\x00\x00\xc6\x10\x03'  # byte string
                   b'\x00\x10\x10\x42\xf2\x10'
                   b'\x10\x00\x00\x10\x10\x10\x03' # byte string
                   b'\x10\x10'
                   b'\x10\xb1\xb2\x10\x10\x00\x00\x10\x10\x42\xf2\x10\x10\x00\x00\x10\x10\x10\x03' # byte string
                   b'\x10\x03\x10\x03' # byte string
                   b'\x10\xbb\xb2\x01\x03\x10\x03'
                   b'\x33\x33\x22\x10'
                   b'\x10\xb2\xb1\x10\x10\x00\x00\x10\x10\x42\xef\x10\x10\x00\x00\x10\x10\x10\x03'
                   b'\x10\x10\x10\x03')

    finder = PackageFinder()
    for i in range(len(byte_string)):
        result = finder.check_byte(byte_string[i:i + 1])
        if result is not None:
            if result[0] is None:
                print(result[1])
            string = result[1] if type(result[1]) != type(b'') else form(result[1])
            print('TYPE: {}, CONTENT: {}'.format(str(result[0]).upper(), string))
            print(DataExtracter(result).extract())
