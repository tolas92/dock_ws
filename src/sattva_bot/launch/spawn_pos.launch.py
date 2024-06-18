#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# this is the function launch  system will look for


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "Prototype_1"


    entity_name = robot_base_name+"-"+str(int(random.random()*100000))

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ],
                   parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}]
    )

    

    # create and return launch description object
    return LaunchDescription(
        [ 
            use_sim_time_arg,
            spawn_robot,
        ]
    )
