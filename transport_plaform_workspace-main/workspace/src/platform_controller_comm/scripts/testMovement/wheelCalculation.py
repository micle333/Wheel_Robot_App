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
    total_duration = rospy.Duration(5 * 60)  # 5 минут
    step_duration = rospy.Duration(20)  # 20 секунд на каждый шаг (5 секунд нуля + 5 секунд изменения скорости)
    zero_duration = rospy.Duration(10)  # 10 секунд нулевой скорости

    current_speed = 0.0
    speed_step = max_speed / (total_duration.to_sec() / step_duration.to_sec())

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time

        if elapsed_time > total_duration:
            current_speed = 0
            print("Test complite")
        else:
            # Determine if we are in a zero period or a speed step period
            period_index = int(elapsed_time.to_sec() / step_duration.to_sec())
            period_time = elapsed_time - rospy.Duration(period_index * step_duration.to_sec())

            if period_time < zero_duration:
                current_speed = 0.0
            else:
                current_speed = (period_index + 1) * speed_step

        header = Header()
        header.stamp = rospy.Time.now()

        wheel_rotation_msg = WheelRotation()
        wheel_rotation_msg.header = header
        wheel_rotation_msg.frontLeft = current_speed
        wheel_rotation_msg.frontRight = current_speed
        wheel_rotation_msg.rearLeft = current_speed
        wheel_rotation_msg.rearRight = current_speed
        wheel_rotation_msg.rearAxis = 0.0  # Можете установить другие значения при необходимости
        wheel_rotation_msg.frontAxis = 0.0  # Можете установить другие значения при необходимости

        pub.publish(wheel_rotation_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        wheel_rotation_publisher()
    except rospy.ROSInterruptException:
        pass
