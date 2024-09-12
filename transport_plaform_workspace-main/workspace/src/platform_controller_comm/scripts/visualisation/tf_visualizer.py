#!/usr/bin/env python3

import rospy
import tf
import tf.transformations as tft
from platform_controller_comm.msg import DebugMsg
import math

class TfPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher')

        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(100.0)

        # Инициализация переменных
        self.front_axis_turn = 0.0
        self.rear_axis_turn = 0.0
        self.front_resolver_val = 0.0
        self.rear_resolver_val = 0.0

        # Подписка на debug_data
        rospy.Subscriber("debug_data", DebugMsg, self.debug_data_callback)

    def debug_data_callback(self, msg):
        self.front_axis_turn = msg.frontAxisTurn
        self.rear_axis_turn = msg.rearAxisTurn
        self.front_resolver_val = msg.frontResolverVal
        self.rear_resolver_val = msg.rearResolverVal

    def publish_transforms(self):
        while not rospy.is_shutdown():
            # Преобразование угловых значений в радианы для поворота осей
            front_wheels_turn = math.radians(-self.front_axis_turn)
            rear_wheels_turn = math.radians(-self.rear_axis_turn)

            # Расчет вращения колес вокруг оси
            # front_wheel_rotation = self.front_resolver_val * rospy.get_time()
            # rear_wheel_rotation = self.rear_resolver_val * rospy.get_time()

            front_wheel_rotation = 0
            rear_wheel_rotation = 0

            # Базовый поворот колес rpy="1.5708 0 0"
            base_wheel_rotation = tft.quaternion_from_euler(1.5708, 0, 0)

            # Publish transform from base_link to front_axle
            self.br.sendTransform((0.75, 0, 0.24),
                                  tft.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  "front_axle",
                                  "base_link")

            # Publish transform from base_link to rear_axle
            self.br.sendTransform((-0.75, 0, 0.24),
                                  tft.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  "rear_axle",
                                  "base_link")

            # Publish transform from front_axle to front_right_wheel
            self.br.sendTransform((0, 0.7, 0),
                                  tft.quaternion_multiply(base_wheel_rotation, tft.quaternion_from_euler(front_wheel_rotation, front_wheels_turn, 0)),
                                  rospy.Time.now(),
                                  "front_right_wheel",
                                  "front_axle")

            # Publish transform from front_axle to front_left_wheel
            self.br.sendTransform((0, -0.7, 0),
                                  tft.quaternion_multiply(base_wheel_rotation, tft.quaternion_from_euler(front_wheel_rotation, front_wheels_turn, 0)),
                                  rospy.Time.now(),
                                  "front_left_wheel",
                                  "front_axle")

            # Publish transform from rear_axle to rear_right_wheel
            self.br.sendTransform((0, 0.7, 0),
                                  tft.quaternion_multiply(base_wheel_rotation, tft.quaternion_from_euler(rear_wheel_rotation, rear_wheels_turn, 0)),
                                  rospy.Time.now(),
                                  "rear_right_wheel",
                                  "rear_axle")

            # Publish transform from rear_axle to rear_left_wheel
            self.br.sendTransform((0, -0.7, 0),
                                  tft.quaternion_multiply(base_wheel_rotation, tft.quaternion_from_euler(rear_wheel_rotation, rear_wheels_turn, 0)),
                                  rospy.Time.now(),
                                  "rear_left_wheel",
                                  "rear_axle")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        tf_publisher = TfPublisher()
        tf_publisher.publish_transforms()
    except rospy.ROSInterruptException:
        pass
