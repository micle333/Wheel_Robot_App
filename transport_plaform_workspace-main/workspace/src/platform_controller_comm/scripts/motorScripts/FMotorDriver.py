#!/usr/bin/env python3

import threading
import os
import rospy
from std_msgs.msg import Header
from platform_controller_comm.msg import PacketContent
import serial
import serial.tools.list_ports
from pythonLibs.bpnp import PackagePacker, PackageFinder, DataPacker, SENDERS_BY_ID, FPB_ID


TEST_STRING = (b'\x10\x02\xb1\x96\x10\x03'
               b'\x10\xb3\xb1\x7b\x00\x00\x10\x10\x41\x00\x00\x00\x00\xe0\x10\x03')

TARGET_ID = FPB_ID


class FMotorCommunication:
    def __init__(self, port=None, baudrate=None):
        rospy.init_node('front_motor_communication_node')
        self.port = port
        self.baudrate = baudrate
        self.data_packer = DataPacker('BORT')
        self.package_packer = PackagePacker()
        self.package_finder = PackageFinder()
        self.device_id = b'\x00'
        self.ser = None
        rospy.wait_for_message('central_hub_extracted_data', PacketContent, timeout=None)
        # rospy.wait_for_message('rear_motor_extracted_data', PacketContent, timeout=None)
        if port is not None:
            if not self._open_port_():
                self.ser = None
                rospy.logerr(f"[FrontMotorDriver] Port not found!")
        self.pub = rospy.Publisher('front_motor_extracted_data', PacketContent, queue_size=10)
        self.sub = rospy.Subscriber('front_motor_command_data', PacketContent, self.cmd_pack_callback, queue_size=10)

    def cmd_pack_callback(self, data):
        byte_str = self.package_packer.pack(data.data)
        if self.ser is not None:
            self.ser.write(byte_str)
        else:
            rospy.loginfo("[FrontMotorDriver] Packed data: %s", byte_str)

    def _open_port_(self):
        attempts = 0
        if self._attempt_open_port_():
            if self._check_port_():
                return True
        comlist = serial.tools.list_ports.comports()
        available_ports = []
        symlink_paths = [os.path.realpath(f"/dev/{link}") for link in os.listdir('/dev') if os.path.islink(f"/dev/{link}")]
        for element in comlist:
            if element.device not in symlink_paths:
                available_ports.append(element.device)
        port_index = 0
        while attempts < len(available_ports):
            self.port = available_ports[port_index]
            if self._attempt_open_port_():
                res = self._check_port_()
                if res:
                    rospy.loginfo(f"[FrontMotorDriver] Port {self.port} is open and functioning correctly.")
                    return True
            rospy.loginfo(f"[FrontMotorDriver] Port {self.port} failed. Trying next port.")
            port_index = (port_index + 1) % len(available_ports)
            attempts += 1
        return False

    def _attempt_open_port_(self):
        rospy.loginfo(f"[FrontMotorDriver] Attempting to open port {self.port}")
        def attempt():
            try:
                self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=5)
                rospy.sleep(2)
                self.success = True
            except serial.SerialException as e:
                rospy.loginfo(f"[FrontMotorDriver] Failed to open port {self.port}: {e}")
                self.success = False

        self.success = False
        thread = threading.Thread(target=attempt)
        thread.start()
        thread.join(timeout=10)  # Timeout in seconds

        if thread.is_alive():
            rospy.loginfo(f"[FrontMotorDriver] Timeout occurred while attempting to open port {self.port}")
            return False
        return self.success

    def _check_port_(self):
        def check():
            try:
                self._send_whu_request_()
                data = self._receive_data_()
                if data is not None:
                    self._process_data_(data)
                    self.check = self.device_id == TARGET_ID
            except serial.SerialException as e:
                self.check = False

        self.check = False
        thread = threading.Thread(target=check)
        thread.start()
        thread.join(timeout=5)  # Timeout in seconds

        if thread.is_alive():
            rospy.loginfo(f"[FrontMotorDriver] Timeout occurred while attempting to check open port {self.port}")
            return False
        return self.check

    def _send_whu_request_(self):
        if self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)
        whu_request = self.data_packer.pack_msg('WHU', [])
        whu_package = self.package_packer.pack(whu_request)
        if self.ser is not None:
            self.ser.write(whu_package)
        else:
            rospy.loginfo(whu_package)
        rospy.sleep(0.3)

    def _receive_data_(self):
        if self.ser is not None:
            if self.ser.in_waiting:
                return self.ser.read(self.ser.in_waiting)
            return None
        else:
            # return None
            return TEST_STRING

    def _process_data_(self, data):
        for i in range(len(data)):
            result = self.package_finder.check_byte(data[i:i+1])
            if result is not None:
                result_id = self._handle_result_(result)
                if result_id is not None:
                    self.device_id = result_id

    def _handle_result_(self, result):
        if result[0] == 'IAM':
            sender_id = result[1][1:2]
            return sender_id
        else:
            if isinstance(result[0], str):
                packet_content = PacketContent()
                packet_content.header.stamp = rospy.Time.now()
                packet_content.pack_name = result[0]
                packet_content.data = result[1]
                if hasattr(self, 'pub'):
                    self.pub.publish(packet_content)
                return None

    def run(self):
        try:
            while not rospy.is_shutdown():
                received_data = self._receive_data_()
                if received_data:
                    self._process_data_(received_data)
                rospy.sleep(0.01)
        finally:
            if self.ser is not None:
                self.ser.close()

if __name__ == '__main__':
    communication = FMotorCommunication('/dev/ttyACM1', 115200)
    try:
        communication.run()
    except rospy.ROSInterruptException:
        pass
