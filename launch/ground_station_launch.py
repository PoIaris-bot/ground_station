from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ground_station',
            executable='radio_telemetry',
            name='radio_telemetry',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud_rate': 57600
            }]
        ),
        Node(
            package='ground_station',
            executable='visualizer',
            name='visualizer',
            parameters=[{
                'trajectory_length': 5000
            }]
        )
    ])
