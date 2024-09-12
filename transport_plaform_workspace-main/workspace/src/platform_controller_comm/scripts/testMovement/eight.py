#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from platform_controller_comm.msg import WheelRotation

period_time = 18.00;
shift_time = period_time/2;

def wheel_rotation_publisher():
    rospy.init_node('wheel_rotation_publisher', anonymous=True)
    pub = rospy.Publisher('manual_control', WheelRotation, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now()
    max_speed = 1.5
    total_duration = rospy.Duration(5 * 60)  # 5 минут
    turn_duration = rospy.Duration(period_time)  # Продолжительность каждого поворота (20 секунд)
    shift_duration = rospy.Duration(shift_time)

    current_speed = 0.0
    turn_angle = 20.0  # Начальный угол
    next_turn_time = shift_duration

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time
        if elapsed_time > next_turn_time:
            # Переключение угла поворота
            turn_angle = -turn_angle
            next_turn_time += turn_duration  # Обновляем время следующего переключения

        # Установка постоянной скорости после разгона
        current_speed = max_speed

        if elapsed_time > total_duration:
            current_speed = 0
            turn_angle = 0


        header = Header()
        header.stamp = rospy.Time.now()

        wheel_rotation_msg = WheelRotation()
        wheel_rotation_msg.header = header
        wheel_rotation_msg.frontLeft = current_speed
        wheel_rotation_msg.frontRight = current_speed
        wheel_rotation_msg.rearLeft = current_speed
        wheel_rotation_msg.rearRight = current_speed
        wheel_rotation_msg.rearAxisTargetAngle = turn_angle
        wheel_rotation_msg.frontAxisTargetAngle = -turn_angle

        pub.publish(wheel_rotation_msg)
        rate.sleep()
        if elapsed_time > total_duration:
            print("Test complete")
            break

if __name__ == '__main__':
    try:
        wheel_rotation_publisher()
    except rospy.ROSInterruptException:
        pass
