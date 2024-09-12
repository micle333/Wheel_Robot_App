#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range, LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import tf


def normalize_angle(angle):
    angle = math.fmod(angle, 2 * math.pi)  # Получаем остаток от деления на 2*pi
    if angle <= -math.pi:
        angle += 2 * math.pi  # Если угол меньше -pi, добавляем 2*pi
    elif angle > math.pi:
        angle -= 2 * math.pi  # Если угол больше pi, вычитаем 2*pi
    return angle

class AngleMeasure:
    def __init__(self):
        rospy.init_node('angle_measure')


        rospy.on_shutdown(self.myhook)

        # TF listener to transform sensor data to the base_link frame
        self.tf_listener = tf.TransformListener()

        self.angle_of_interest = -90 * math.pi / 180  # Лево +, Право -
        # self.angle_of_interest = 0 * math.pi / 180  # Лево +, Право -
        # self.angle_of_interest = 30 * math.pi / 180  # Лево +, Право -
        self.range = 18 * math.pi / 180
        self.min_range = 0.0
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidarCallback)
        self.measure_timer = rospy.Timer(rospy.Duration(10), self.measureTimer)

        self.fin = True

    def lidarCallback(self, data):
        # try:
        (trans, rot) = self.tf_listener.lookupTransform('/base_link', data.header.frame_id, rospy.Time(0))
        rotation_angle = tf.transformations.euler_from_quaternion(rot)[2]  # Get yaw component
        angle = data.angle_min
        for range in data.ranges:
            if range < data.range_max:
                global_angle = normalize_angle(angle + rotation_angle)
                if -math.pi*3/5 <= global_angle <= math.pi*3/5:  # Only process the front half-circle
                    if self.angle_of_interest - self.range <= global_angle <= self.angle_of_interest + self.range:
                        if range < self.min_range or self.min_range == 0:
                            print("Min_update")
                            self.min_range = range
                        print(range)
            angle += data.angle_increment
        print("====================================")
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #     rospy.logerr("TF error in lidar_callback: %s", e)

    def publish_map(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.fin:
            rate.sleep()

    def myhook(self):
        print("====================================")
        print(self.min_range)

    def measureTimer(self, event):
        self.fin = False
        rospy.signal_shutdown("Time out")


if __name__ == '__main__':
    occupancy_map = AngleMeasure()
    occupancy_map.publish_map()
