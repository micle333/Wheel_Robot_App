#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from platform_controller_comm.msg import PacketContent
from platform_controller_comm.srv import LightCommand, LightCommandResponse
from pythonLibs.bpnp import DataPacker


class LightControl:
    def __init__(self):
        rospy.init_node('light_control_node')
        self.packer = DataPacker('BORT')
        self.pub = rospy.Publisher('central_hub_command_data', PacketContent, queue_size=10)
        self.service = rospy.Service('light_command', LightCommand, self.handle_light_command)

    def handle_light_command(self, req):
        print(req.cmd)
        if req.cmd == 'Stop On':
            body_byte = self.packer.pack_msg('SM', [1])
        elif req.cmd == 'Stop Off':
            body_byte = self.packer.pack_msg('SM', [0])
        elif req.cmd == 'Left turn On':
            body_byte = self.packer.pack_msg('LT', [1])
        elif req.cmd == 'Left turn Off':
            body_byte = self.packer.pack_msg('LT', [0])
        elif req.cmd == 'Right turn On':
            body_byte = self.packer.pack_msg('RT', [1])
        elif req.cmd == 'Right turn Off':
            body_byte = self.packer.pack_msg('RT', [0])

        packet = PacketContent()
        packet.header.stamp = rospy.Time.now()
        packet.pack_name = 'Light'
        packet.data = body_byte
        self.pub.publish(packet)
        return LightCommandResponse(True)

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    communication = LightControl()
    try:
        communication.start()
    except rospy.ROSInterruptException:
        pass
