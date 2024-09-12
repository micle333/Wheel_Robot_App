#!/usr/bin/env python3
import rospy
from platform_controller_comm.msg import WheelRotation, DebugMsg  # commands on wheels
from pythonLibsNav.transport_platform_kinematic_model import evaluate_driving_kinematics
from math import cos, pi, atan, tan, copysign, pi

class SubscribeAndPublish:

    def __init__(self):
        self.lw = 1.2  # distance between wheels (const)
        self.r_roko = 0.1 # wheels radius two wheeled bot (const)

        self.odoSub = rospy.Subscriber('debug_data', DebugMsg, self.odometry_callback, queue_size=2)
        self.odoPub = rospy.Publisher('odo_converted', WheelRotation, queue_size=2)

        rospy.loginfo("[sensor_bridge] Sensor bridge is running...")


    def odometry_callback(self, msg):    
        velocity, rate = evaluate_driving_kinematics(msg.frontResolverVal, msg.rearResolverVal, msg.frontAxisTurn, msg.rearAxisTurn)

        gammaLeft = velocity / self.r_roko - self.lw * rate / self.r_roko / 2
        gammaRight = velocity / self.r_roko + self.lw * rate / self.r_roko / 2

        odo_msg = WheelRotation()
        odo_msg.header = msg.header
        odo_msg.frontLeft = gammaLeft
        odo_msg.frontRight = gammaRight
        self.odoPub.publish(odo_msg)


def main():
    rospy.init_node('sensor_bridge')
    SAP = SubscribeAndPublish()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
# End on main()


if __name__ =="__main__":
    main()
