#!/usr/bin/env python3

import rospy
import csv
import random
import string
from sensor_msgs.msg import Imu, Range, NavSatFix
from platform_controller_comm.msg import DebugMsg, WheelRotation
from threading import Lock
import time
import os

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger', anonymous=True)

        self.lock = Lock()
        self.data = {
            'imu': None,
            'ultrasound': [None] * 6,
            'debug': None,
            'manual_control': None,
            'gps': None
        }

        rospy.Subscriber('/main_imu_data', Imu, self.imu_callback)
        rospy.Subscriber('/ultrasound_sensor_0', Range, self.ultrasound_callback, 0)
        rospy.Subscriber('/ultrasound_sensor_1', Range, self.ultrasound_callback, 1)
        rospy.Subscriber('/ultrasound_sensor_2', Range, self.ultrasound_callback, 2)
        rospy.Subscriber('/ultrasound_sensor_3', Range, self.ultrasound_callback, 3)
        rospy.Subscriber('/ultrasound_sensor_4', Range, self.ultrasound_callback, 4)
        rospy.Subscriber('/ultrasound_sensor_5', Range, self.ultrasound_callback, 5)
        rospy.Subscriber('/debug_data', DebugMsg, self.debug_callback)
        rospy.Subscriber('/manual_control', WheelRotation, self.manual_control_callback)
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)

        filename = self.generate_log_filename()
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        header = [
            'timestamp',
            'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
            'imu_angular_velocity_x', 'imu_angular_velocity_y', 'imu_angular_velocity_z',
            'imu_linear_acceleration_x', 'imu_linear_acceleration_y', 'imu_linear_acceleration_z',
            'ultrasound_0', 'ultrasound_1', 'ultrasound_2', 'ultrasound_3', 'ultrasound_4', 'ultrasound_5',
            'debug_frontLeftWheelRate', 'debug_frontRightWheelRate', 'debug_rearLeftWheelRate', 'debug_rearRightWheelRate',
            'debug_frontAxisTurn', 'debug_rearAxisTurn', 'debug_frontResolverVal', 'debug_rearResolverVal',
            'debug_frontShaftRate', 'debug_rearShaftRate', 'debug_frontAxisLimitRight', 'debug_rearAxisLimitRight',
            'debug_frontAxisLimitLeft', 'debug_rearAxisLimitLeft',
            'manual_control_frontLeft', 'manual_control_frontRight',
            'manual_control_rearLeft', 'manual_control_rearRight',
            'manual_control_rearAxis', 'manual_control_frontAxis',
            'gps_latitude', 'gps_longitude', 'gps_altitude', 'gps_status'
        ]

        self.csv_writer.writerow(header)
        self.rate = rospy.Rate(100)  # 100 Hz

    def generate_log_filename(self):
        current_time = time.strftime("%Y%m%d_%H%M%S")
        names = ['Alpha', 'Bravo', 'Charlie', 'Delta', 'Echo', 'Foxtrot', 'Golf', 'Hotel', 'India', 'Juliet']
        random_name = random.choice(names)
        filename = f"data_log_{current_time}_{random_name}.csv"
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        return os.path.join(log_dir, filename)

    def imu_callback(self, msg):
        with self.lock:
            self.data['imu'] = msg

    def ultrasound_callback(self, msg, index):
        with self.lock:
            self.data['ultrasound'][index] = msg

    def debug_callback(self, msg):
        with self.lock:
            self.data['debug'] = msg

    def manual_control_callback(self, msg):
        with self.lock:
            self.data['manual_control'] = msg

    def gps_callback(self, msg):
        with self.lock:
            self.data['gps'] = msg

    def log_data(self):
        while not rospy.is_shutdown():
            with self.lock:
                timestamp = time.time()
                imu = self.data['imu']
                ultrasound = self.data['ultrasound']
                debug = self.data['debug']
                manual_control = self.data['manual_control']
                gps = self.data['gps']

                row = [timestamp]

                if imu:
                    row.extend([
                        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w,
                        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
                        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z
                    ])
                else:
                    row.extend([0] * 10)

                for us in ultrasound:
                    row.append(us.range if us else 0)

                if debug:
                    row.extend([
                        debug.frontLeftWheelRate, debug.frontRightWheelRate, debug.rearLeftWheelRate, debug.rearRightWheelRate,
                        debug.frontAxisTurn, debug.rearAxisTurn, debug.frontResolverVal, debug.rearResolverVal,
                        debug.frontShaftRate, debug.rearShaftRate,
                        debug.frontAxisLimitRight, debug.rearAxisLimitRight, debug.frontAxisLimitLeft, debug.rearAxisLimitLeft
                    ])
                else:
                    row.extend([0] * 12)

                if manual_control:
                    row.extend([
                        manual_control.frontLeft, manual_control.frontRight,
                        manual_control.rearLeft, manual_control.rearRight,
                        manual_control.rearAxisTargetAngle, manual_control.frontAxisTargetAngle
                    ])
                else:
                    row.extend([0] * 6)

                if gps:
                    row.extend([gps.latitude, gps.longitude, gps.altitude, gps.status.status])
                else:
                    row.extend([0, 0, 0, 0])

                self.csv_writer.writerow(row)
                self.csv_file.flush()

            self.rate.sleep()

    def shutdown_hook(self):
        self.csv_file.close()

if __name__ == '__main__':
    logger = DataLogger()
    rospy.on_shutdown(logger.shutdown_hook)
    logger.log_data()
