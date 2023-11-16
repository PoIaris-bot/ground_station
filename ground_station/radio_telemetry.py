#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import serial
from rclpy.node import Node
from parafoil_msgs.msg import Pose
from serial.serialutil import SerialException


class RadioTelemetry(Node):
    def __init__(self):
        super().__init__('radio_telemetry')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        try:
            self.serial = serial.Serial(self.port, self.baud_rate)
        except SerialException:
            self.serial = None

        self.publisher = self.create_publisher(Pose, 'parafoil/pose', 10)

        self.run()

    def run(self):
        buffer = ''
        while rclpy.ok():
            if self.serial is not None:
                try:
                    byte = self.serial.read().decode()
                    if '0' <= byte <= '9' or byte == '[' or byte == ']' or byte == ',' or byte == '.' or byte == '-':
                        buffer += byte
                except SerialException:
                    self.serial.close()
                    self.serial = None
            else:
                try:
                    self.serial = serial.Serial(self.port, self.baud_rate)
                except SerialException:
                    self.serial = None

            try:
                start_index = buffer.index('[')
                try:
                    end_index = buffer.index(']', start_index)
                    try:
                        data = eval(buffer[start_index:end_index + 1])
                        pose = Pose()
                        pose.timestamp = data[0]
                        pose.id = data[1]
                        pose.position = data[2:5]
                        pose.velocity = data[5:8]
                        pose.angle = data[8:11]
                        pose.quaternion = data[11:15]
                        pose.acceleration = data[15:18]
                        pose.angular_velocity = data[18:21]
                        self.publisher.publish(pose)
                    except SyntaxError:
                        pass
                    except AssertionError:
                        pass
                    buffer = buffer[end_index + 1:]
                except ValueError:
                    pass
            except ValueError:
                pass


def main(args=None):
    rclpy.init(args=args)
    radio_telemetry = RadioTelemetry()
    radio_telemetry.destroy_node()
    rclpy.shutdown()
