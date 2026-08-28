from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_calibration',
            #namespace='init',
            executable='initialization',
            #name='idk1',
            #arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='test_calibration',
            #namespace='motor_int',
            executable='motor_interface',
            #name='idk2',
            #ros_arguments=['--log-level', 'warn']
        ),
    ])