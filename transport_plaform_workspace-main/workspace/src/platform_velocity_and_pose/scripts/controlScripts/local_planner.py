#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from platform_controller_comm.msg import WheelRotation  # commands on wheels
from pythonLibsNav.transport_platform_kinematic_model import evaluate_driving_kinematics
from math import cos, sin, pi

class TrajectoryPublisher:

    def __init__(self):
        self.lw = 1.2  # distance between wheels (const)
        self.r_roko = 0.1 # wheels radius (const)

        # Initialize subscribers and publishers
        self.odoSub = rospy.Subscriber('odom', Odometry, self.odometry_callback, queue_size=2)
        self.pathPub = rospy.Publisher('trajectory_paths', Path, queue_size=10)
        
        rospy.loginfo("[trajectory_publisher] Trajectory Publisher is running...")

    def odometry_callback(self, msg):
        # Example ranges for velocity and turn rate
        velocities = [1.5, 1.5, 1.5]  # m/s
        turn_rates = [-0.5, 0.0, 0.5]  # rad/s

        for velocity in velocities:
            for rate in turn_rates:
                self.publish_path(msg, velocity, rate)

    def publish_path(self, odom_msg, velocity, rate):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'base_link'

        # Assuming straight line movement for simplicity in example
        start_pose = PoseStamped()
        start_pose.header.stamp = rospy.Time.now()
        start_pose.header.frame_id = 'base_link'
        start_pose.pose.position.x = 0
        start_pose.pose.position.y = 0
        start_pose.pose.orientation.z = 0
        start_pose.pose.orientation.w = 0

        path.poses.append(start_pose)

        # Calculate the trajectory's next pose based on the velocity and rate
        x, y, theta = 0, 0, 0
        for i in range(1, 11):  # Generate 10 steps
            x += velocity * cos(theta) * 0.1
            y += velocity * sin(theta) * 0.1
            theta += rate * 0.1

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            path.header.frame_id = 'base_link'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = sin(theta / 2.0)
            pose.pose.orientation.w = cos(theta / 2.0)

            path.poses.append(pose)

        self.pathPub.publish(path)

def main():
    rospy.init_node('trajectory_generator')
    tp = TrajectoryPublisher()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
