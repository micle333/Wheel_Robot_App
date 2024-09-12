#!/usr/bin/env python3
from math import copysign
from dataclasses import dataclass

import roslib
roslib.load_manifest("platform_controller_comm")
import rospy
import actionlib
from std_msgs.msg import Header, Float64, Bool
from platform_controller_comm.srv import MotorsServices, MotorsServicesResponse
from platform_controller_comm.msg import PacketContent, WheelRotation, DebugMsg
from platform_controller_comm.msg import MotorsActionAction
from pythonLibs.bpnp import DataPacker, DataExtracter, MM_ID, RC_ID, SP_ID
from pythonLibs.bpnp import (FRONT_LEFT_WHEEL_SENSOR,
                             FRONT_RIGHT_WHEEL_SENSOR,
                             REAR_LEFT_WHEEL_SENSOR,
                             REAR_RIGHT_WHEEL_SENSOR,
                             FRONT_RESOLVER,
                             REAR_RESOLVER,
                             FRONT_MOTOR_SHAFT,
                             REAR_MOTOR_SHAFT,
                             FRONT_AXIS_ENC,
                             REAR_AXIS_ENC,
                             FRONT_AXIS_SWITCH,
                             REAR_AXIS_SWITCH)

MOTOR_RATE_COEFFICIENT = 1/39.789                   # Перевод из попугаем в рад/сек оборотов вала двигателя
ABS_WHEEL_RATE_COEFFICIENT = 1/1.52                 # Перевод из отсчетов ABS в рад/сек оборотов колеса
VELOCITY_COEFFICIENT = 177.57                       # Перевод целевого значения линейной скорости м/с в команду вращения двигателя (попугаи)
AXIS_FRONT_TURN_COEFFICIENT = -800                  # Коэффициент перевода целевого значения поворота передней оси рад/сек в шаги двигателя
AXIS_REAR_TURN_COEFFICIENT = -769                   # Коэффициент перевода целевого значения поворота задней оси рад/сек в шаги двигателя
AXIS_FRONT_DEG_STEP_TO_WHEEL_TURN = 0.00125         # Обратный коэффициент перевода целевого значения поворота передней оси рад/сек в шаги двигателя
AXIS_REAR_DEG_STEP_TO_WHEEL_TURN = 0.0013           # Обратный коэффициент перевода целевого значения поворота задней оси рад/сек в шаги двигателя
AXIS_STEPPER_ANGLE_TO_WHEEL_ANGLE = 13.68586111     # Коэффициент перевода угла поворота шагового


AXIS_TURN_CALIBRATION_SPEED = 0.5   # Угловая скорость калибровочных поворотов осей рад/сек
AXIS_MAX_TURN_SPEED = 0.5           # Максимальная скорость поворота оси
AXIS_LOW_TURN_SPEED = 0.0875           # Низкая скорость поворота оси

ZERO_LINEAR_VELOCITY = 0.0          # Нулевое значение линейной скорости (использовалось в процедуре калибровки для преодоления трения)


@dataclass
class SensorsData:
    frontLeftWheelRate: float = 0.0
    frontRightWheelRate: float = 0.0
    rearLeftWheelRate: float = 0.0
    rearRightWheelRate: float = 0.0
    frontAxisTurn: float = 0.0
    rearAxisTurn: float = 0.0
    frontResolverVal: float = 0.0
    rearResolverVal: float = 0.0
    frontShaftRate: float = 0.0
    rearShaftRate: float = 0.0
    frontAxisLimitRight: bool = False
    rearAxisLimitRight: bool = False
    frontAxisLimitLeft: bool = False
    rearAxisLimitLeft: bool = False

@dataclass
class TargetValData:
    frontMotorShaftRate: float = 0.0
    rearMotorShaftRate: float = 0.0
    frontAxisTurn: float = 0.0
    rearAxisTurn: float = 0.0

    def stop(self):
        self.frontMotorShaftRate = 0.0
        self.rearMotorShaftRate = 0.0
        self.frontAxisTurn = 0.0
        self.rearAxisTurn = 0.0

    def stop_forward(self):
        self.frontMotorShaftRate = ZERO_LINEAR_VELOCITY
        self.rearMotorShaftRate = ZERO_LINEAR_VELOCITY


class MotorsDriver:
    def __init__(self):
        # Данные
        self.sensorsData = SensorsData()
        self.targetValData = TargetValData()

        self.handbrakeStatus = False;
        self.calibrationStatus = False;

        self.frontAxisRevolutes = 0
        self.rearAxisRevolutes = 0
        self.frontAxisPreviousTurn = 0.0
        self.rearAxisPreviousTurn = 0.0

        self.frontAxisTurnShift = 0.0
        self.rearAxisTurnShift = 0.0

        self.frontAxisTurnTotalVal = 0.0
        self.rearAxisTurnTotalVal = 0.0

        self.frontAxisCalibrationTurn = 0.0
        self.rearAxisCalibrationTurn = 0.0

        self.remoteControlConnection = False
        self.remoteLock = False

        self.absTotalFrontLeft = 0.0
        self.absTotalFrontRight = 0.0

        self.packer = DataPacker('BORT')

        rospy.init_node('motors_driver_node')
        self.checkRemoteWatchdog = rospy.get_param('remote_watchdog', True)

        self.remoteWatchdogSub = rospy.Subscriber('ping_status', Bool, self.remote_watchdog_callback, queue_size=10)

        # Связь с программными драйверами
        self.fCmdPub = rospy.Publisher('front_motor_command_data', PacketContent, queue_size=10)
        self.rCmdPub = rospy.Publisher('rear_motor_command_data', PacketContent, queue_size=10)
        self.centralPub = rospy.Publisher('central_hub_command_data', PacketContent, queue_size=10)
        self.debugDataPub = rospy.Publisher('debug_data', DebugMsg, queue_size=10)

        self.fRawDataSub = rospy.Subscriber('front_motor_extracted_data', PacketContent, self.front_motor_data_callback, queue_size=10)
        self.rRawDataSub = rospy.Subscriber('rear_motor_extracted_data', PacketContent, self.rear_motor_data_callback, queue_size=10)
        self.centralSub = rospy.Subscriber('central_hub_extracted_data', PacketContent, self.central_hub_callback, queue_size=10)

        # Сервис ручного тормоза
        self.handBreakService = rospy.Service('motors_services', MotorsServices, self.handle_motors_services)

        # Прием действий
        self.actionServer = actionlib.SimpleActionServer('motors_actions', MotorsActionAction, self.handle_motors_actions, False)
        self.actionServer.start()

        # Команды ручного управления
        self.manualCmdSub = rospy.Subscriber('manual_control', WheelRotation, self.cmd_callback, queue_size=10)

        # Таймер отправки команд управления
        self.sendControlTimer = rospy.Timer(rospy.Duration(0.1), self.controlSend)
        # Таймер отправки информации одометрической системы
        self.sendControlTimer = rospy.Timer(rospy.Duration(0.01), self.controlSend)


    def handle_motors_actions(self, goal):
        if self.handbrakeStatus:
            rospy.loginfo(f"[MotorsDriver] Sequnce skipped: Hand brake activated!")
            return MotorsServicesResponse(False)
        if self.calibrationStatus:
            rospy.loginfo(f"[MotorsDriver] Sequnce skipped: Calibration activated!")
            return MotorsServicesResponse(False)
        if goal.cmd == 'FRONT AXIS CALIBRATION':
            return self.calibration_sequence(goal)
        if goal.cmd == 'REAR AXIS CALIBRATION':
            return self.calibration_sequence(goal)


    def handle_motors_services(self, req):
        if req.cmd == 'BRAKE' or req.cmd == 'FREE HANDBRAKE':
            return self.handbrake_sequence(req)
        if self.handbrakeStatus:
            rospy.loginfo(f"[MotorsDriver] Sequnce skipped: Hand brake activated!")
            return MotorsServicesResponse(False)
        if self.calibrationStatus:
            rospy.loginfo(f"[MotorsDriver] Sequnce skipped: Calibration activated!")
            return MotorsServicesResponse(False)

    def calibration_sequence(self, req):
        self.calibrationStatus = True

        def abort_if_handbrake():
            if self.handbrakeStatus:
                self.actionServer.set_aborted()
                return True
            return False

        def perform_calibration(axis, direction):
            # Direction should be either 'Left' or 'Right'
            opposite_direction = 'Right' if direction == 'Left' else 'Left'
            attribute = f"{axis}AxisCalibrationTurn"
            limit_attr = f"{axis}AxisLimit{direction}"
            opposite_limit_attr = f"{axis}AxisLimit{opposite_direction}"

            # Fast turn till stopper
            while not getattr(self.sensorsData, limit_attr):
                if abort_if_handbrake():
                    break
                self.targetValData.stop_forward()
                setattr(self, attribute, (AXIS_TURN_CALIBRATION_SPEED if direction == 'Right' else -AXIS_TURN_CALIBRATION_SPEED))
                rospy.sleep(0.1)

            # Slow turn out of stopper
            while getattr(self.sensorsData, limit_attr):
                if abort_if_handbrake():
                    break
                self.targetValData.stop_forward()
                setattr(self, attribute, (-AXIS_LOW_TURN_SPEED if direction == 'Right' else AXIS_LOW_TURN_SPEED))
                rospy.sleep(0.1)

            # Slow turn back to stopper
            while not getattr(self.sensorsData, limit_attr):
                if abort_if_handbrake():
                    break
                self.targetValData.stop_forward()
                setattr(self, attribute, (AXIS_LOW_TURN_SPEED if direction == 'Right' else -AXIS_LOW_TURN_SPEED))
                rospy.sleep(0.1)

        if req.cmd == 'FRONT AXIS CALIBRATION':
            perform_calibration('front', 'Left')
            self.frontAxisTurnShift = self.sensorsData.frontAxisTurn + self.frontAxisRevolutes*36 + 264
            perform_calibration('front', 'Right')
            self.frontAxisTurnMax = self.calculate_turn_max('front')
        elif req.cmd == 'REAR AXIS CALIBRATION':
            perform_calibration('rear', 'Left')
            self.rearAxisTurnShift = self.sensorsData.rearAxisTurn + self.rearAxisRevolutes*36 + 264
            perform_calibration('rear', 'Right')
            self.rearAxisTurnMax = self.calculate_turn_max('rear')

        self.finalize_calibration()

    def calculate_turn_max(self, axis):
        axis_turn = getattr(self.sensorsData, f"{axis}AxisTurn")
        revolutes = getattr(self, f"{axis}AxisRevolutes")
        shift = getattr(self, f"{axis}AxisTurnShift")
        return axis_turn + revolutes * 36 - shift

    def finalize_calibration(self):
        self.targetValData.stop()
        self.frontAxisCalibrationTurn = 0.0
        self.rearAxisCalibrationTurn = 0.0
        self.calibrationStatus = False
        self.actionServer.set_succeeded()
        return MotorsServicesResponse(True)


    def handbrake_sequence(self, req):
        if req.cmd == 'BRAKE':
            body_byte = self.packer.pack_msg('SP', [1])
            self.targetValData.stop()
            self.handbrakeStatus = True
        if req.cmd == 'FREE HANDBRAKE':
            body_byte = self.packer.pack_msg('SP', [0])
            self.handbrakeStatus = False
        packet = PacketContent()
        packet.header.stamp = rospy.Time.now()
        packet.pack_name = 'Brake'
        packet.data = body_byte
        rospy.sleep(0.1)
        self.centralPub.publish(packet)
        print(f"Handbrake: {self.handbrakeStatus}")
        return MotorsServicesResponse(self.handbrakeStatus)


    def remote_watchdog_callback(self, msg):
        self.remoteControlConnection = msg.data

    def front_motor_data_callback(self, msg):
        if msg.pack_name == 'MR':
            sensor_id, front_motor_angle, cnt = DataExtracter([msg.pack_name, msg.data]).extract()
            if sensor_id == FRONT_RESOLVER:
                self.sensorsData.frontResolverVal = front_motor_angle*MOTOR_RATE_COEFFICIENT
        if msg.pack_name == 'MS':
            sensor_id, rate, cnt = DataExtracter([msg.pack_name, msg.data]).extract()
            if sensor_id == FRONT_MOTOR_SHAFT:
                self.sensorsData.frontShaftRate = rate

    def rear_motor_data_callback(self, msg):
        if msg.pack_name == 'MR':
            sensor_id, rear_motor_angle, cnt = DataExtracter([msg.pack_name, msg.data]).extract()
            if sensor_id == REAR_RESOLVER:
                self.sensorsData.rearResolverVal = rear_motor_angle*MOTOR_RATE_COEFFICIENT
        if msg.pack_name == 'MS':
            sensor_id, rate, cnt = DataExtracter([msg.pack_name, msg.data]).extract()
            if sensor_id == REAR_MOTOR_SHAFT:
                self.sensorsData.rearShaftRate = rate

    def central_hub_callback(self, msg):
        if msg.pack_name == 'ODO':
            sensor_id, rate, cnt = DataExtracter([msg.pack_name, msg.data]).extract()
            if sensor_id == FRONT_LEFT_WHEEL_SENSOR:
                self.absTotalFrontLeft += rate/12
                self.sensorsData.frontLeftWheelRate = rate*ABS_WHEEL_RATE_COEFFICIENT
            if sensor_id == FRONT_RIGHT_WHEEL_SENSOR:
                self.absTotalFrontRight += rate/12
                self.sensorsData.frontRightWheelRate = rate*ABS_WHEEL_RATE_COEFFICIENT
            if sensor_id == REAR_LEFT_WHEEL_SENSOR:
                self.sensorsData.rearLeftWheelRate = rate*ABS_WHEEL_RATE_COEFFICIENT
            if sensor_id == REAR_RIGHT_WHEEL_SENSOR:
                self.sensorsData.rearRightWheelRate = rate*ABS_WHEEL_RATE_COEFFICIENT
            self.publish_debug_data()
        if msg.pack_name == 'ENC':
            sensor_id, axis_angle, cnt = DataExtracter([msg.pack_name, msg.data]).extract()
            if sensor_id == FRONT_AXIS_ENC:
                self.frontAxisPreviousTurn = self.sensorsData.frontAxisTurn
                self.sensorsData.frontAxisTurn = axis_angle/10
                if self.sensorsData.frontAxisTurn-self.frontAxisPreviousTurn > 30:
                    self.frontAxisRevolutes -= 1
                if self.sensorsData.frontAxisTurn-self.frontAxisPreviousTurn < -30:
                    self.frontAxisRevolutes += 1
                self.frontAxisTurnTotalVal = -(self.sensorsData.frontAxisTurn + self.frontAxisRevolutes*36 - self.frontAxisTurnShift)/AXIS_STEPPER_ANGLE_TO_WHEEL_ANGLE
            if sensor_id == REAR_AXIS_ENC:
                self.rearAxisPreviousTurn = self.sensorsData.rearAxisTurn
                self.sensorsData.rearAxisTurn = axis_angle/10
                if self.sensorsData.rearAxisTurn-self.rearAxisPreviousTurn > 30:
                    self.rearAxisRevolutes -= 1
                if self.sensorsData.rearAxisTurn-self.rearAxisPreviousTurn < -30:
                    self.rearAxisRevolutes += 1
                self.rearAxisTurnTotalVal = -(self.sensorsData.rearAxisTurn + self.rearAxisRevolutes*36 - self.rearAxisTurnShift)/AXIS_STEPPER_ANGLE_TO_WHEEL_ANGLE
            self.publish_debug_data()

        if msg.pack_name == 'ST':
            sensor_id, binary_switch, cnt = DataExtracter([msg.pack_name, msg.data]).extract()
            if sensor_id == FRONT_AXIS_SWITCH:
                if binary_switch == 1:
                    self.sensorsData.frontAxisLimitRight = True
                if binary_switch == 10:
                    self.sensorsData.frontAxisLimitLeft = True
                if binary_switch == 0:
                    self.sensorsData.frontAxisLimitRight = False
                    self.sensorsData.frontAxisLimitLeft = False
                if binary_switch == 11:
                    self.sensorsData.frontAxisLimitRight = True
                    self.sensorsData.frontAxisLimitLeft = True
            if sensor_id == REAR_AXIS_SWITCH:
                if binary_switch == 10:
                    self.sensorsData.rearAxisLimitRight = True
                if binary_switch == 1:
                    self.sensorsData.rearAxisLimitLeft = True
                if binary_switch == 0:
                    self.sensorsData.rearAxisLimitRight = False
                    self.sensorsData.rearAxisLimitLeft = False
                if binary_switch == 11:
                    self.sensorsData.rearAxisLimitRight = True
                    self.sensorsData.rearAxisLimitLeft = True

    def cmd_callback(self, msg):
        if self.handbrakeStatus or self.calibrationStatus:
            return
        self.targetValData.frontMotorShaftRate = msg.frontLeft + msg.frontRight
        self.targetValData.rearMotorShaftRate = msg.rearLeft + msg.rearRight
        self.targetValData.frontAxisTurn = msg.frontAxisTargetAngle
        self.targetValData.rearAxisTurn = msg.rearAxisTargetAngle

        if (self.targetValData.frontMotorShaftRate == 0 and self.targetValData.rearMotorShaftRate == 0
            and self.targetValData.frontAxisTurn == 0 and self.targetValData.rearAxisTurn == 0):
            self.remoteLock = False

    def publish_debug_data(self):
        msg = DebugMsg()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "debug_frame"
        msg.frontLeftWheelRate = self.sensorsData.frontLeftWheelRate
        msg.frontRightWheelRate = self.sensorsData.frontRightWheelRate
        msg.rearLeftWheelRate = self.sensorsData.rearLeftWheelRate
        msg.rearRightWheelRate = self.sensorsData.rearRightWheelRate
        msg.frontAxisTurn = self.frontAxisTurnTotalVal
        msg.rearAxisTurn = self.rearAxisTurnTotalVal
        msg.frontResolverVal = self.sensorsData.frontResolverVal
        msg.rearResolverVal = self.sensorsData.rearResolverVal
        msg.frontShaftRate = self.absTotalFrontLeft
        msg.rearShaftRate = self.absTotalFrontRight
        msg.frontAxisLimitRight = self.sensorsData.frontAxisLimitRight
        msg.rearAxisLimitRight = self.sensorsData.rearAxisLimitRight
        msg.frontAxisLimitLeft = self.sensorsData.frontAxisLimitLeft
        msg.rearAxisLimitLeft = self.sensorsData.rearAxisLimitLeft
        self.debugDataPub.publish(msg)

    def pack_and_send(self, bodyByte, localPub):
        packet = PacketContent()
        packet.header.stamp = rospy.Time.now()
        packet.pack_name = 'Cmd'
        packet.data = bodyByte
        localPub.publish(packet)

    def controlSend(self, event):
        if (self.checkRemoteWatchdog and not self.remoteControlConnection) or self.remoteLock:
            self.remoteLock = True
            return
        if self.handbrakeStatus:
            bodyByte = self.packer.pack_msg('MM', [209, 0])
            self.pack_and_send(bodyByte, self.fCmdPub)
            bodyByte = self.packer.pack_msg('MM', [210, 0])
            self.pack_and_send(bodyByte, self.rCmdPub)
            bodyByte = self.packer.pack_msg('RC', [193, 0])
            self.pack_and_send(bodyByte, self.centralPub)
            bodyByte = self.packer.pack_msg('RC', [194, 0])
            self.pack_and_send(bodyByte, self.centralPub)
            return

        bodyByte = self.packer.pack_msg('MM', [209, self.targetValData.frontMotorShaftRate*VELOCITY_COEFFICIENT])
        self.pack_and_send(bodyByte, self.fCmdPub)
        bodyByte = self.packer.pack_msg('MM', [210, self.targetValData.rearMotorShaftRate*VELOCITY_COEFFICIENT])
        self.pack_and_send(bodyByte, self.rCmdPub)

        if not self.calibrationStatus:
            if abs(self.frontAxisTurnTotalVal - self.targetValData.frontAxisTurn) > 4:
                bodyByte = self.packer.pack_msg('RC', [193, copysign(AXIS_MAX_TURN_SPEED*AXIS_FRONT_TURN_COEFFICIENT,
                                                                      self.targetValData.frontAxisTurn - self.frontAxisTurnTotalVal)])
                self.pack_and_send(bodyByte, self.centralPub)
            elif abs(self.frontAxisTurnTotalVal - self.targetValData.frontAxisTurn) > 0.1:
                bodyByte = self.packer.pack_msg('RC', [193, copysign(AXIS_LOW_TURN_SPEED*AXIS_FRONT_TURN_COEFFICIENT,
                                                                      self.targetValData.frontAxisTurn - self.frontAxisTurnTotalVal)])
                self.pack_and_send(bodyByte, self.centralPub)

            # if abs(self.rearAxisTurnTotalVal - self.targetValData.rearAxisTurn) > 4:
            #     bodyByte = self.packer.pack_msg('RC', [194, copysign(AXIS_MAX_TURN_SPEED*AXIS_REAR_TURN_COEFFICIENT,
            #                                                          self.rearAxisTurnTotalVal - self.targetValData.rearAxisTurn)])
            #     self.pack_and_send(bodyByte, self.centralPub)
            # elif abs(self.rearAxisTurnTotalVal - self.targetValData.rearAxisTurn) > 0.1:
            #     bodyByte = self.packer.pack_msg('RC', [194, copysign(AXIS_LOW_TURN_SPEED*AXIS_REAR_TURN_COEFFICIENT,
            #                                                          self.rearAxisTurnTotalVal - self.targetValData.rearAxisTurn)])
            #     self.pack_and_send(bodyByte, self.centralPub)
            if abs(self.rearAxisTurnTotalVal - 0) > 4:
                bodyByte = self.packer.pack_msg('RC', [194, copysign(AXIS_MAX_TURN_SPEED*AXIS_REAR_TURN_COEFFICIENT,
                                                                     self.targetValData.rearAxisTurn - self.rearAxisTurnTotalVal)])
                self.pack_and_send(bodyByte, self.centralPub)
            elif abs(self.rearAxisTurnTotalVal - 0) > 0.1:
                bodyByte = self.packer.pack_msg('RC', [194, copysign(AXIS_LOW_TURN_SPEED*AXIS_REAR_TURN_COEFFICIENT,
                                                                     self.targetValData.rearAxisTurn - self.rearAxisTurnTotalVal)])
                self.pack_and_send(bodyByte, self.centralPub)

        else:
            bodyByte = self.packer.pack_msg('RC', [193, self.frontAxisCalibrationTurn*AXIS_FRONT_TURN_COEFFICIENT])
            self.pack_and_send(bodyByte, self.centralPub)
            bodyByte = self.packer.pack_msg('RC', [194, self.rearAxisCalibrationTurn*AXIS_REAR_TURN_COEFFICIENT])
            self.pack_and_send(bodyByte, self.centralPub)



    def start(self):
        rospy.spin()

if __name__ == '__main__':
    communication = MotorsDriver()
    try:
        communication.start()
    except rospy.ROSInterruptException:
        pass
