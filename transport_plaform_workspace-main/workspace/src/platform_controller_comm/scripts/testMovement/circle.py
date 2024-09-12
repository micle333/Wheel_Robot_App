#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from platform_controller_comm.msg import WheelRotation

def wheel_rotation_publisher():
    rospy.init_node('wheel_rotation_publisher', anonymous=True)
    pub = rospy.Publisher('manual_control', WheelRotation, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now()
    max_speed = 3.0
    total_duration = rospy.Duration(1 * 60)  # 5 минут

    current_speed = 0.0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time

        if elapsed_time > total_duration:
            current_speed = 0
            print("Test complite")
        else:
            current_speed = 3.0

        header = Header()
        header.stamp = rospy.Time.now()

        wheel_rotation_msg = WheelRotation()
        wheel_rotation_msg.header = header
        wheel_rotation_msg.frontLeft = current_speed
        wheel_rotation_msg.frontRight = current_speed
        wheel_rotation_msg.rearLeft = current_speed
        wheel_rotation_msg.rearRight = current_speed
        wheel_rotation_msg.rearAxisTargetAngle = -20.0
        wheel_rotation_msg.frontAxisTargetAngle = 20.0

        pub.publish(wheel_rotation_msg)
        rate.sleep()
        if elapsed_time > total_duration:
            break

if __name__ == '__main__':
    try:
        wheel_rotation_publisher()
    except rospy.ROSInterruptException:
        pass
