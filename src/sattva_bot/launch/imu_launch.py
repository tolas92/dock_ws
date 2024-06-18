#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    imu_node = Node(
        package='imu_node',
        executable='imu_node',
        output='screen'
    )
    imu_filter_node=Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        output='screen'
    )

    ld.add_action(imu_filter_node) 

    return ld