import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():



    return LaunchDescription([

        Node(
            package='image_proc',
            executable='image_proc',
            output='screen',
            namespace='cam0',
            )
    ])