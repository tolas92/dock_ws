#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    battery_status = Node(
        package='battery_status',
        executable='battery_status',
        output='screen'
    )
  
    ld.add_action(battery_status) 

    return ld