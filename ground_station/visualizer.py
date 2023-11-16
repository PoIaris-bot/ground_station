#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from parafoil_msgs.msg import Pose


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.declare_parameter('trajectory_length', 5000)
        self.trajectory_length = self.get_parameter('trajectory_length').get_parameter_value().integer_value

        self.cache = {'x': [], 'y': [], 'z': []}
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.subscription = self.create_subscription(Pose, 'parafoil/pose', self.callback, 10)

    def callback(self, message):
        if len(self.cache['x']) > self.trajectory_length:
            self.cache['x'].pop(0)
            self.cache['y'].pop(0)
            self.cache['z'].pop(0)
        position = message.position
        self.cache['x'].append(position[0])
        self.cache['y'].append(position[1])
        self.cache['z'].append(position[2])

        plt.cla()
        self.ax.plot3D(self.cache['x'], self.cache['y'], self.cache['z'], color='blue')
        self.ax.scatter3D(position[0], position[1], position[2], color='red', s=10)
        plt.grid(True)

        self.ax.set_xlabel('x/m')
        self.ax.set_ylabel('y/m')
        self.ax.set_zlabel('z/m')
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    visualizer = Visualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()
