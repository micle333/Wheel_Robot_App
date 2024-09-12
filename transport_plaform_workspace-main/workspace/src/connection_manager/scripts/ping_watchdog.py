#!/usr/bin/env python3

import rospy
import os
import time
from std_msgs.msg import Bool

class PingMonitor:
    def __init__(self, target_ip, check_interval=0.5):
        self.target_ips = target_ip
        self.check_interval = check_interval
        self.ping_status_publisher = rospy.Publisher('/ping_status', Bool, queue_size=10)
        self.prev_status = None

    def ping_device(self):
        for target_ip in self.target_ips:
            response = os.system(f"ping -c 1 -W 1 {target_ip} > /dev/null 2>&1")
            if response == 0:
                return True
        return False

    def run(self):
        rospy.init_node('ping_monitor', anonymous=True)
        rate = rospy.Rate(1 / self.check_interval)

        while not rospy.is_shutdown():
            current_status = self.ping_device()
            self.ping_status_publisher.publish(current_status)
            self.prev_status = current_status
            rate.sleep()

if __name__ == '__main__':
    target_ips = ["192.168.88.240", "192.168.88.239", "192.168.88.238"]
    check_interval = 0.5

    try:
        monitor = PingMonitor(target_ips, check_interval)
        monitor.run()
    except rospy.ROSInterruptException:
        pass
