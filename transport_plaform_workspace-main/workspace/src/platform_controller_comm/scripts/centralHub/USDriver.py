#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3
from platform_controller_comm.msg import PacketContent
from pythonLibs.bpnp import DataExtracter


class USDriver:
    def __init__(self):
        rospy.init_node('IMU_driver')
        self.pub_us_0 = rospy.Publisher('ultrasound_sensor_0', Range, queue_size=10)
        self.pub_us_1 = rospy.Publisher('ultrasound_sensor_1', Range, queue_size=10)
        self.pub_us_2 = rospy.Publisher('ultrasound_sensor_2', Range, queue_size=10)
        self.pub_us_3 = rospy.Publisher('ultrasound_sensor_3', Range, queue_size=10)
        self.pub_us_4 = rospy.Publisher('ultrasound_sensor_4', Range, queue_size=10)
        self.pub_us_5 = rospy.Publisher('ultrasound_sensor_5', Range, queue_size=10)
        self.pub = {
            0: self.pub_us_0,
            1: self.pub_us_1,
            2: self.pub_us_2,
            3: self.pub_us_3,
            4: self.pub_us_4,
            5: self.pub_us_5
        }
        self.sub = rospy.Subscriber('central_hub_extracted_data', PacketContent, self.us_callback, queue_size=10)

    def us_callback(self, msg):
        # Фильтр сообщений по полю pack_name
        if msg.pack_name == 'UZ':
            sensor_id, dist, cnt = DataExtracter([msg.pack_name, msg.data]).extract()

            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = f"us_{sensor_id}_frame"
            range_msg.range = dist/100
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 1.309 # 75 градусов
            range_msg.min_range = 0.2
            range_msg.max_range = 6.0

            self.pub[sensor_id].publish(range_msg)


    def start(self):
        rospy.spin()

if __name__ == '__main__':
    ultraSoundDriver = USDriver()
    try:
        ultraSoundDriver.start()
    except rospy.ROSInterruptException:
        pass
