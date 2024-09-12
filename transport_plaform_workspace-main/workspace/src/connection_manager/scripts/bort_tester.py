#!/usr/bin/env python3

import rospy
from threading import Thread
import time
import pythonLibsTCP.TCPServer as TCPServer
from pythonLibsTCP.tcp_packer_unpacker import PackerAndUnpacker
from platform_controller_comm.msg import WheelRotation
from platform_controller_comm.srv import LightCommand, LightCommandRequest, LightCommandResponse
from std_msgs.msg import Header
import signal

packer = PackerAndUnpacker()
SPEED_SCALE = 1.5
TURN_SCALE = 20

class SubscribeAndPublish:
    _server = 0
    packer = PackerAndUnpacker()

    def __init__(self):
        # Get path builder parameters
        rospy.init_node('remote_gamepad', anonymous=True)
        rospy.wait_for_service('light_command')
        self.light_command_client = rospy.ServiceProxy('light_command', LightCommand)
        self.wheel_data_pub = rospy.Publisher('manual_control', WheelRotation, queue_size=2)
        port_name = 8080
        print("\nTCP Server is running at port {}".format(8080))
        print("Use _port:=9050 to change it.")

        self.forward_val = 0.0
        self.turn_val = 0.0

        self._server = TCPServer.TCPServer(port_name, self.update_values)
        self._server.StartServer()

    def call_light_command_service(self, command):

        try:
            # Создаем запрос на основе структуры сообщения
            header = Header()
            header.stamp = rospy.Time.now()

            req = LightCommandRequest()
            req.header = header
            req.cmd = command

            # Отправляем запрос и получаем ответ
            resp = self.light_command_client(req)
            return resp.result
        except rospy.ServiceException as e:
            rospy.logerr("Ошибка при вызове сервиса: %s" % e)
            return False

    def update_values(self, parsed):
        print(f'Parsed data: {parsed}')
        if len(parsed)>0:
            if parsed[0][0] == 'NKR':
                forward_val, turn_val, _ = parsed[0][1]
                self.forward_val = forward_val
                self.turn_val = turn_val

                msg = WheelRotation()
                msg.header.stamp = rospy.Time.now()
                msg.frontLeft = self.forward_val * SPEED_SCALE
                msg.frontRight = self.forward_val * SPEED_SCALE
                msg.rearLeft = self.forward_val * SPEED_SCALE
                msg.rearRight = self.forward_val * SPEED_SCALE
                msg.rearAxisTargetAngle = -self.turn_val * TURN_SCALE
                msg.frontAxisTargetAngle = self.turn_val * TURN_SCALE
                self.wheel_data_pub.publish(msg)
            if parsed[0][0] == 'SLI':
                if parsed[0][1][0]:
                    self.call_light_command_service('Stop On')
                else:
                    self.call_light_command_service('Stop Off')

    def close_server(self):
        if self._server:
            self._server.StopServer()
        rospy.signal_shutdown("Closing application")


    def SendTelemetry(self):
        if self._server._remoteAddress != "":
            time.sleep(0.5)
        else:
            print("Remote is not set for telemetry ...")


def main():
    SAP = SubscribeAndPublish()  # calls __init__()
    signal.signal(signal.SIGINT, lambda sig, frame: SAP.close_server())
    time.sleep(10)
    counter = 0
    if (counter % 50 == 0):  # every 5 seconds
        counter += 1
        time.sleep(0.1)  # 10 Hz
    while not rospy.is_shutdown():
        time.sleep(0.1)
        counter += 1

if __name__ == "__main__":
    main()
