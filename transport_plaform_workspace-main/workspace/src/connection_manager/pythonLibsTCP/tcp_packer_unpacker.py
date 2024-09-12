from pythonLibsTCP.tcp_bpnp import DataExtracter, DataPacker, PackageFinder, PackagePacker
import time


class PackerAndUnpacker():

    def _bytes_to_hex_str_(self, byte_data):
        return ' '.join([f'{b:02x}' for b in byte_data])


    # Позволяет отправить один пакет с данными и получить вывод в виде байт-строки
    def pack(self, input_data):
        package_packer = PackagePacker()
        packer = DataPacker(input_data[0]).pack_msg(*input_data[1:])
        byte_str = PackagePacker().pack(packer)

        return byte_str


    # Позволяет отправить байт-последовательность и получить вывод данных из неё
    def unpack(self, received_data): # type(received_data) is <class 'bytes'>
        finder = PackageFinder()
        res_data = []
        counter = 0

        for i in range(len(received_data)):
            result = finder.check_byte(received_data[i:i + 1])
            if result is not None:
                # print()
                if result[0] is not None:
                    string = result[1] if type(result[1]) != type(b'') else self._bytes_to_hex_str_(result[1])
                    res_data.append(DataExtracter(result).extract())
                    counter += 1
        # Возвращает список всех полученных данных из поданой байт-последовательности
        return res_data
