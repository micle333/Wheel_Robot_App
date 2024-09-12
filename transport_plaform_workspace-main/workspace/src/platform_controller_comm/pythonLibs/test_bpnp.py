import crc8
import struct
import serial
import time
from bpnp import DataPacker, PackagePacker, DataExtracter,  PackageFinder

# Константы протокола
PACK_START = b'\x10'    # Байт начала пакета
PACK_END = b'\x10\x03'  # Байты конца пакета
BRT_ID = b'\xb0'    # ID Борта
FPB_ID = b'\xb1'    # ID Контроллера переднего силового блока

# Функция для конвертации байтов в строку hex формата
def bytes_to_hex_str(byte_data):
    return ' '.join([f'{b:02x}' for b in byte_data])

# Открытие последовательного порта
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)
time.sleep(2)
if ser.in_waiting:
    received_data = ser.read(ser.in_waiting)


inputs = [['BORT', 'MM', 176,160.0], ['BORT', 'SP', 1], ['BORT', 'WHU']]
#['BORT', 'RC', 193, 0]]
        # , ['FPU', 'RT', 0], ['FPU', 'LT', 1], ['FPU', 'WHU'], ['FPU', 'IAM'], ['FPU', 'ODO', 5, -0.3, 14],
        #   ['BPU', 'MR', 16, -123, 123], ['FPU', 'MS', 123, 9, 0], ['BORT', 'MM', 16, 16.4],
        #   ['FPU', 'ENC', 111, 3.5223, 7], ['FPU', 'ST', 55, 36, 33], ['FPU', 'RC', 2, 123.512],
        #   ['BORT', 'IMU', 3, 4, 5, 9, 16, 25, 27, 64, 125, 50], ['BPU', 'UZ', 5, 123.0, 55],
        #   ['BPU', 'SP', 1], ['FPU', 'MR', 16, 36, 4338], ['CMC', 'ODO', 7, 3.5223, 7],['CMC', 'ODO', 8, 3.5223, 7], ['CMC', 'ODO', 10, 3.5223, 7],['CMC', 'ODO', 11, 3.5223, 7]]

# ['FPU', 'MR', 16, 36, 4338] - несколько \x10: \x10 в ID, \x10 в float, \x10 в Cnt и \x10 в CRC
finder = PackageFinder()

package_packer = PackagePacker()
# for j in range(1,10000):
for inp in inputs:
    data_packer = DataPacker(inp[0])
    body_byte = data_packer.pack_msg(inp[1], inp[2:])
    byte_str = package_packer.pack(body_byte)
    # Отправка данных
    ser.write(byte_str)
    # Вывод отправленного пакета в HEX формате
    print(f'Отправленный пакет: {bytes_to_hex_str(byte_str)}')
    print(f'Данные: {inp}')

    if ser.in_waiting:
        received_data = ser.read(ser.in_waiting)
        k = received_data.find(b'Angle: ')
        if k>0:
            print(received_data[k-20:k+10].decode("ascii", errors="ignore"))
        for i in range(len(received_data)):
            result = finder.check_byte(received_data[i:i + 1])
            print(f"{format(received_data[i], '02x')} ", end='')
            if result is not None:
                if result[0] is None:
                    print(result[1])
                else:
                    if result[0]!='IMU' and result[0]!='ODO' and result[0]!='ENC' and result[0]!='UZ':
                        string = result[1] if type(result[1]) != type(b'') else bytes_to_hex_str(result[1])
                        print('TYPE: {}, CONTENT: {}'.format(str(result[0]).upper(), string))
                        print(DataExtracter(result).extract())
                        print()
                        print('-----------------------------------------------')
    time.sleep(0.1)
    # print(f'Полученные данные: {received_data.decode("ascii", errors="ignore")}')
    # print(f'Полученные hex данные: {bytes_to_hex_str(received_data)}')

ser.close()  # Закрытие порта
