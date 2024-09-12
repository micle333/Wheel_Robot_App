#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from platform_controller_comm.msg import PacketContent
from pythonLibs.bpnp import DataExtracter
from math import pi

ANGLE_SCALE = 3276.7
VELOCITY_SCALE = 41753.3
TIME_SCALE = 1000.0
TEMPRATURE_SCALE = 1.27
DEG_TO_RAD = pi/180


class IMUDriver:
    def __init__(self):
        rospy.init_node('IMU_driver')
        self.pub_imu = rospy.Publisher('main_imu_data', Imu, queue_size=10)
        self.sub = rospy.Subscriber('central_hub_extracted_data', PacketContent, self.imu_callback, queue_size=10)
        self.init_cnt = 0
        self.zero_shift = 0;

    def imu_callback(self, msg):
        # Фильтр сообщений по полю pack_name
        if msg.pack_name == 'IMU':
            wx, wy, wz, ax, ay, az, dt, temperature, mz, cnt = DataExtracter([msg.pack_name, msg.data]).extract()

            # Расположение осей IMU:
            # z - вперед
            # y - вверх
            # x - влево
            # Расположение осей робота:
            # x - вперед
            # y - влево
            # z - вверх

            delta_angle_x = wz / ANGLE_SCALE
            delta_angle_y = wx / ANGLE_SCALE
            delta_angle_z = wy / ANGLE_SCALE

            delta_velocity_x = (az / VELOCITY_SCALE)
            delta_velocity_y = (ax / VELOCITY_SCALE)
            delta_velocity_z = (ay / VELOCITY_SCALE)

            dt = dt / TIME_SCALE
            temperature = temperature / TEMPRATURE_SCALE

            rate_x = (delta_angle_x / dt) * DEG_TO_RAD
            rate_y = (delta_angle_y / dt) * DEG_TO_RAD
            rate_z = (delta_angle_z / dt) * DEG_TO_RAD

            accel_x = delta_velocity_x / dt
            accel_y = delta_velocity_y / dt
            accel_z = delta_velocity_z / dt

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_frame"

            imu_msg.angular_velocity = Vector3(rate_x, rate_y, rate_z)
            imu_msg.linear_acceleration = Vector3(accel_x, accel_y, accel_z)

            if self.init_cnt < 10:
                self.zero_shift += rate_z
                self.init_cnt += 1
            else:
                rate_z = rate_z - self.zero_shift/self.init_cnt

            self.pub_imu.publish(imu_msg)


    def start(self):
        rospy.spin()

if __name__ == '__main__':
    inertialDriver = IMUDriver()
    try:
        inertialDriver.start()
    except rospy.ROSInterruptException:
        pass
