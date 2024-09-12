#!/usr/bin/env python3
import rospy
from platform_controller_comm.msg import PacketContent
from pythonLibs.bpnp import DataExtracter
import sys
import time

class PacketFrequencyMonitor:
    def __init__(self):
        self.data_times = {}
        self.pack_names = []

        # Инициализируем узел ROS
        rospy.init_node('packet_frequency_monitor', anonymous=True)

        # Подписываемся на топики
        rospy.Subscriber('/central_hub_extracted_data', PacketContent, self.callback)
        rospy.Subscriber('/front_motor_extracted_data', PacketContent, self.callback)
        rospy.Subscriber('/rear_motor_extracted_data', PacketContent, self.callback)

        # Таймер для регулярного вывода частот
        rospy.Timer(rospy.Duration(1), self.print_frequencies)

    def callback(self, data):
        pack_name = data.pack_name
        current_time = rospy.Time.now()
        if pack_name in ['ODO', 'ENC', 'UZ', 'ST', 'MR', 'MS']:
            sensor_id, _, _ = DataExtracter([pack_name, data.data]).extract()
            pack_name += f' {sensor_id}'

        if pack_name not in self.data_times:
            self.data_times[pack_name] = []
            self.pack_names.append(pack_name)
            self.pack_names.sort()

        # Добавляем время получения сообщения
        self.data_times[pack_name].append(current_time)

        # Ограничиваем размер списка до последних 100 элементов
        if len(self.data_times[pack_name]) > 100:
            self.data_times[pack_name].pop(0)

    def print_frequencies(self, event):
        # Перемещаем курсор вверх на количество уникальных pack_name
        sys.stdout.write(f'\033[{len(self.pack_names)}A')
        for name in sorted(self.pack_names):
            if len(self.data_times[name]) >= 100:
                dt = (self.data_times[name][-1] - self.data_times[name][0]).to_sec()
                frequency = 99 / dt if dt > 0 else 0
                sys.stdout.write(f'\r{name}: {frequency:.2f} Hz        \n')
            else:
                sys.stdout.write(f'\r{name}: calculating...        \n')
        sys.stdout.flush()

def main():
    monitor = PacketFrequencyMonitor()
    rospy.spin()

if __name__ == '__main__':
    main()
