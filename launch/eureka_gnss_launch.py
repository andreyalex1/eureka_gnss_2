from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_gnss_2',
            executable='ublox8_parser',
            name='ublox8_parser',
            shell=True,
        ),
    ])