import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ig_server',
            executable='ig_server',
            name='task_server',
            output='screen'
        )
    ])

