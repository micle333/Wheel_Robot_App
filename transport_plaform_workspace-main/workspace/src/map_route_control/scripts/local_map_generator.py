#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range, LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import tf

# Parameters
map_width, map_height = 400, 400  # in cells
map_resolution = 0.1  # meters per cell
default_value = 50  # unknown space
reset_time = 1000  # Time in seconds to reset the map

def normalize_angle(angle):
    angle = math.fmod(angle, 2 * math.pi)  # Получаем остаток от деления на 2*pi
    if angle <= -math.pi:
        angle += 2 * math.pi  # Если угол меньше -pi, добавляем 2*pi
    elif angle > math.pi:
        angle -= 2 * math.pi  # Если угол больше pi, вычитаем 2*pi
    return angle

class OccupancyMap:
    def __init__(self):
        rospy.init_node('occupancy_map_builder')

        # TF listener to transform sensor data to the base_link frame
        self.tf_listener = tf.TransformListener()

        # Occupancy grid setup
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'base_link'
        self.map.info.resolution = map_resolution
        self.map.info.width = map_width
        self.map.info.height = map_height
        self.map.info.origin.position.x = -map_width * map_resolution / 2
        self.map.info.origin.position.y = -map_height * map_resolution / 2
        self.map.info.origin.orientation.w = 1.0
        self.map.data = [default_value] * (map_width * map_height)

        # Subscribers to sensor topics
        for i in range(6):
            rospy.Subscriber(f'/ultrasound_sensor_{i}', Range, self.ultrasound_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Publisher for the occupancy grid
        self.map_pub = rospy.Publisher('local_occupancy_map', OccupancyGrid, queue_size=10)
        rospy.Timer(rospy.Duration(reset_time), self.reset_map)

    def ultrasound_callback(self, data):
        return
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/base_link', data.header.frame_id, rospy.Time(0))
            angle = math.atan2(trans[1], trans[0])
            range = data.range

            if range < data.max_range:
                x = trans[0] + range * math.cos(angle)
                y = trans[1] + range * math.sin(angle)
                self.update_map(x, y, 100)  # Mark the cell as occupied
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)

    def lidar_callback(self, data):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', data.header.frame_id, rospy.Time(0))
            rotation_angle = tf.transformations.euler_from_quaternion(rot)[2]  # Get yaw component
            self.map.data = [default_value] * (map_width * map_height)
            angle = data.angle_min
            for range in data.ranges:
                if range < data.range_max:
                    global_angle = normalize_angle(angle + rotation_angle)
                    if -math.pi*3/5 <= global_angle <= math.pi*3/5:  # Only process the front half-circle
                        x = trans[0] + range * math.cos(global_angle)
                        y = -(trans[1] + range * math.sin(global_angle))
                        self.update_map(x, y, 100)  # Mark the cell as occupied
                angle += data.angle_increment
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF error in lidar_callback: %s", e)

    def update_map(self, x, y, value):
        mx = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        if 0 <= mx < self.map.info.width and 0 <= my < self.map.info.height:
            index = my * self.map.info.width + mx
            self.map.data[index] = value

    def reset_map(self, event):
        self.map.data = [default_value] * (map_width * map_height)

    def publish_map(self):
        rate = rospy.Rate(10)  # 1 Hz
        while not rospy.is_shutdown():
            self.map.header.stamp = rospy.Time.now()
            self.map_pub.publish(self.map)
            rate.sleep()

if __name__ == '__main__':
    occupancy_map = OccupancyMap()
    occupancy_map.publish_map()
