import os
import time
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()
    
    
    customer_menu_node = Node(
        package='waiter_tree',
        executable='customer_menu.py',
        output='screen'
    )
    kitchen_table_node=Node(
        package='waiter_tree',
        executable='kitchen_table.py',
        output='screen'
    )

    table_nav_node=Node(
        package='navigator_planner',
        executable='table_navigation.py',
        output='screen'
    )

    waiter_node=Node(
        package='waiter_tree',
        executable='waiter.py',
        output='screen'
    )

    #launch file for the ai detected hand gesture:
    hand_gesture_package_name="hand_control"    

    # Specify the paths to the launch files
    hand_launch_file=os.path.join(get_package_share_directory(hand_gesture_package_name),"launch","hand_launch.py")

    hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            hand_launch_file
            ),
            condition=IfCondition
            (LaunchConfiguration("launch_ai", default=False))

    )
 


    ld.add_action(table_nav_node)
    ld.add_action(customer_menu_node)
    #ld.add_action(TimerAction(period=2.0,actions=[kitchen_table_node]))
    ld.add_action(kitchen_table_node)
    ld.add_action(waiter_node)
    ld.add_action(hand_launch)
    


    return ld
 