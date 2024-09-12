#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from platform_controller_comm.msg import WheelRotation

def wheel_rotation_publisher():
    rospy.init_node('wheel_rotation_publisher', anonymous=True)
    pub = rospy.Publisher('manual_control', WheelRotation, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now()
    max_speed = 6
    total_duration = rospy.Duration(1 * 60)  # 5 минут
    acceleration_duration = rospy.Duration(5)  # 5 секунд для разгона
    movement_duration = rospy.Duration(10)  # 10 секунд для стабильного движения
    braking_duration = rospy.Duration(5)  # 5 секунд для торможения

    current_speed = 0.0
    accelerating = True
    moving_forward = True

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time

        if elapsed_time > total_duration:
            current_speed = 0
            print("Test complete")
            break

        cycle_time = (acceleration_duration + movement_duration + braking_duration).to_sec()
        period_time = (elapsed_time.to_sec() % cycle_time)

        if period_time < acceleration_duration.to_sec():
            # Разгон
            current_speed = max_speed * (period_time / acceleration_duration.to_sec())
        elif period_time < (acceleration_duration + movement_duration).to_sec():
            # Стабильное движение
            current_speed = max_speed
        else:
            # Торможение
            current_speed = max_speed * (1 - ((period_time - (acceleration_duration + movement_duration).to_sec()) / braking_duration.to_sec()))

        if moving_forward:
            current_speed = current_speed
        else:
            current_speed = -current_speed  # Движение назад

        # Переключение направления после каждого цикла
        if period_time >= cycle_time - 0.1:  # Небольшой буфер для переключения
            moving_forward = not moving_forward

        header = Header()
        header.stamp = rospy.Time.now()

        wheel_rotation_msg = WheelRotation()
        wheel_rotation_msg.header = header
        wheel_rotation_msg.frontLeft = current_speed
        wheel_rotation_msg.frontRight = current_speed
        wheel_rotation_msg.rearLeft = current_speed
        wheel_rotation_msg.rearRight = current_speed
        wheel_rotation_msg.rearAxisTargetAngle = 0.0
        wheel_rotation_msg.frontAxisTargetAngle = 0.0

        pub.publish(wheel_rotation_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        wheel_rotation_publisher()
    except rospy.ROSInterruptException:
        pass
