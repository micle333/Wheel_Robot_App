#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from platform_controller_comm.msg import WheelRotation, DebugMsg

class WheelRotationPublisher:
    def __init__(self):
        rospy.init_node('wheel_rotation_publisher', anonymous=True)
        self.pub = rospy.Publisher('manual_control', WheelRotation, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.max_rotation_speed = 0.5
        self.min_rotation_speed = 0.125
        self.current_rotation_speed = self.min_rotation_speed
        self.rotation_increment = 0.0625
        self.direction = 1  # 1 for increasing, -1 for decreasing

        self.front_axis_turn = 0.0
        self.rear_axis_turn = 0.0

        self.current_front_axis_turn = 0.0
        self.start_front_axis_turn = 0.0

        self.current_rear_axis_turn = 0.0
        self.start_rear_axis_turn = 0.0

        self.front_axis_limit_left = False
        self.front_axis_limit_right = False
        self.rear_axis_limit_left = False
        self.rear_axis_limit_right = False

        self.left_stop_capture = False
        self.right_stop_capture = False

        self.stop_requested = False

        rospy.Subscriber('debug_data', DebugMsg, self.debug_data_callback)

    def debug_data_callback(self, msg):
        self.front_axis_turn = msg.frontAxisTurn
        self.rear_axis_turn = msg.rearAxisTurn
        self.front_axis_limit_left = msg.frontAxisLimitLeft
        self.front_axis_limit_right = msg.frontAxisLimitRight
        self.rear_axis_limit_left = msg.rearAxisLimitLeft
        self.rear_axis_limit_right = msg.rearAxisLimitRight
        if self.rear_axis_limit_right:
            if not self.left_stop_capture:
                self.current_time = rospy.Time.now()
                self.current_rear_axis_turn = self.rear_axis_turn
                elapsed_time = (self.current_time - self.start_time).to_sec()
                passed_range = self.current_rear_axis_turn - self.start_rear_axis_turn
                print(f"{self.current_rotation_speed} - {elapsed_time} - {passed_range}")
                self.start_time = self.current_time
                self.start_rear_axis_turn = self.rear_axis_turn
                self.current_rotation_speed = self.current_rotation_speed + self.rotation_increment
                self.direction = -1
                self.left_stop_capture = True
                self.right_stop_capture = False
        if self.rear_axis_limit_left:
            if not self.right_stop_capture:
                self.current_time = rospy.Time.now()
                self.current_rear_axis_turn = self.rear_axis_turn
                elapsed_time = (self.current_time - self.start_time).to_sec()
                passed_range = self.current_rear_axis_turn - self.start_rear_axis_turn
                print(f"{self.current_rotation_speed} - {elapsed_time} - {passed_range}")
                self.start_time = self.current_time
                self.start_rear_axis_turn = self.rear_axis_turn
                self.current_rotation_speed = self.current_rotation_speed + self.rotation_increment
                self.direction = 1
                self.left_stop_capture = False
                self.right_stop_capture = True

    def request_stop(self):
        self.stop_requested = True

    def publish_rotation(self):
        self.start_time = rospy.Time.now()
        print(self.current_rotation_speed)
        while not rospy.is_shutdown() and not self.stop_requested:
            if self.current_rotation_speed > self.max_rotation_speed:
                self.current_rotation_speed = self.max_rotation_speed
                self.stop_requested = True
            elif self.current_rotation_speed < self.min_rotation_speed:
                self.current_rotation_speed = self.min_rotation_speed

            header = Header()
            header.stamp = rospy.Time.now()

            wheel_rotation_msg = WheelRotation()
            wheel_rotation_msg.header = header
            wheel_rotation_msg.frontLeft = 0.0
            wheel_rotation_msg.frontRight = 0.0
            wheel_rotation_msg.rearLeft = 0.0
            wheel_rotation_msg.rearRight = 0.0
            wheel_rotation_msg.rearAxis = self.current_rotation_speed * self.direction
            wheel_rotation_msg.frontAxis = 0.0

            self.pub.publish(wheel_rotation_msg)
            self.rate.sleep()

        # Stop all axes when exiting
        header = Header()
        header.stamp = rospy.Time.now()
        wheel_rotation_msg = WheelRotation()
        wheel_rotation_msg.header = header
        wheel_rotation_msg.frontLeft = 0.0
        wheel_rotation_msg.frontRight = 0.0
        wheel_rotation_msg.rearLeft = 0.0
        wheel_rotation_msg.rearRight = 0.0
        wheel_rotation_msg.rearAxis = 0.0
        wheel_rotation_msg.frontAxis = 0.0
        self.pub.publish(wheel_rotation_msg)

if __name__ == '__main__':
    try:
        wheel_rotation_publisher = WheelRotationPublisher()
        wheel_rotation_publisher.publish_rotation()
    except rospy.ROSInterruptException:
        pass
