from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('prototype'),'config','joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory('prototype'),'config','twist_mux.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_in','/diff_controller/cmd_vel_unstamped'
),
                        ('/cmd_vel_out','/diff_cont/cmd_vel')]
         )
    
    twist_mux_node=Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_controller/cmd_vel_unstamped')]
        )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        twist_mux_node      
    ])