#!/usr/bin/env python3

import os
import threading
import rospy
from std_msgs.msg import Header
from platform_controller_comm.msg import PacketContent
import serial
import serial.tools.list_ports
from pythonLibs.bpnp import PackagePacker, PackageFinder, DataPacker, SENDERS_BY_ID, CMC_ID, FPB_ID

TEST_STRING = (b'\x10\x02\xb1\x96\x10\x03'
               b'\x10\xb2\xb1\x10\x10\x00\x00\x10\x10\x42\xef\x10\x10\x00\x00\x10\x10\x10\x03'
               b'\x10\xb0\xb0\x03\x00\x04\x00\x05\x00\x09\x00\x10\x10\x00\x19\x00\x1b\x00\x40\x00\x7d\x00\x32\x00\x00\x00\xc6\x10\x03'
               b'\x10\xd0\xb2\x05\x00\x00\xf6\x42\x37\x00\x00\x00\x3d\x10\x03'
               b'\x10\xb1\xb3\x07\x5d\x6d\x61\x40\x07\x00\x00\x00\xcc\x10\x03'
               b'\x10\xb1\xb3\x08\x5d\x6d\x61\x40\x07\x00\x00\x00\x65\x10\x03'
               b'\x10\xb1\xb3\x0a\x5d\x6d\x61\x40\x07\x00\x00\x00\x97\x10\x03'
               b'\x10\xb1\xb3\x0b\x5d\x6d\x61\x40\x07\x00\x00\x00\xee\x10\x03')

TARGET_ID = CMC_ID


class CentralHubCommunication:
    def __init__(self, port=None, baudrate=None):
        self.port = port
        self.baudrate = baudrate
        self.data_packer = DataPacker('BORT')
        self.package_packer = PackagePacker()
        self.package_finder = PackageFinder()
        self.device_id = b'\x00'
        self.ser = None
        rospy.init_node('central_hub_communication_node')
        if port is not None:
            if not self._open_port_():
                self.ser = None
                rospy.logerr(f"[CentralHub] Port not found!")
        self.pub = rospy.Publisher('central_hub_extracted_data', PacketContent, queue_size=10)
        self.sub = rospy.Subscriber('central_hub_command_data', PacketContent, self.cmd_pack_callback, queue_size=10)

    def cmd_pack_callback(self, data):
        byte_str = self.package_packer.pack(data.data)

        if self.ser is not None:
            self.ser.write(byte_str)
            # rospy.loginfo("[CentralHub] Packed data: %s", byte_str)
        else:
            pass
            # rospy.loginfo("[CentralHub] Packed data: %s", byte_str)

    def _open_port_(self):
        attempts = 0
        if self._attempt_open_port_():
            rospy.loginfo(f"[CentralHub] Checking port: {self.port}")
            if self._check_port_():
                rospy.loginfo(f"[CentralHub] Port {self.port} is open and functioning correctly.")
                return True

        # Get the list of available serial ports
        comlist = serial.tools.list_ports.comports()
        available_ports = []

        # Get the real paths of all symlinks in /dev
        symlink_paths = [os.path.realpath(f"/dev/{link}") for link in os.listdir('/dev') if os.path.islink(f"/dev/{link}")]

        # Filter out ports that have symlinks
        for element in comlist:
            if element.device not in symlink_paths:
                available_ports.append(element.device)

        port_index = 0
        while attempts < len(available_ports):
            self.port = available_ports[port_index]
            port_opened = self._attempt_open_port_()
            if port_opened:
                rospy.loginfo(f"[CentralHub] Checking port: {self.port}")
                res = self._check_port_()
                if res:
                    rospy.loginfo(f"[CentralHub] Port {self.port} is open and functioning correctly.")
                    return True
            rospy.loginfo(f"[CentralHub] Port {self.port} failed. Trying next port.")
            port_index = (port_index + 1) % len(available_ports)
            attempts += 1

        return False

    def _attempt_open_port_(self):
        rospy.loginfo(f"[CentralHub] Attempting to open port {self.port}")
        def attempt():
            try:
                self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=5)
                rospy.sleep(2)
                self.success = True
            except serial.SerialException as e:
                rospy.loginfo(f"[CentralHub] Failed to open port {self.port}: {e}")
                self.success = False

        self.success = False
        thread = threading.Thread(target=attempt)
        thread.start()
        thread.join(timeout=10)  # Timeout in seconds

        if thread.is_alive():
            rospy.loginfo(f"[CentralHub] Timeout occurred while attempting to open port {self.port}")
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
            rospy.loginfo(f"[CentralHub] Timeout occurred while attempting to check open port {self.port}")
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
                if received_data is not None:
                    self._process_data_(received_data)
                rospy.sleep(0.01)
        finally:
            if self.ser is not None:
                self.ser.close()

if __name__ == '__main__':
    communication = CentralHubCommunication('/dev/ttyACM3', 115200)
    try:
        communication.run()
    except rospy.ROSInterruptException:
        pass
