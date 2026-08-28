from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_calibration',
            executable='initialization',
            prefix=['xterm -e'],
            output='screen',
        ),
        Node(
            package='test_calibration',
            executable='motor_interface',
            prefix=['xterm -e'],
            output='screen',
        ),
    ])
