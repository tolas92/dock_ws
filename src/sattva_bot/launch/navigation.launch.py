import os
import time
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()
    map_package_name="ros2_mapping"
    nav_package_name="navigator_planner"
    joy_package_name="prototype"
    

    # Specify the paths to the launch files
    joy_launch_file=os.path.join(get_package_share_directory(joy_package_name),"launch","joystick.launch.py")

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            joy_launch_file
            ),
        condition=IfCondition(LaunchConfiguration("launch_joy", default=True))
    )

    
    #path to map launch file.
    map_launch_file=os.path.join(get_package_share_directory(map_package_name),"launch","nav2_map_server.launch.py")

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            map_launch_file
            ),
            condition=IfCondition(LaunchConfiguration("launch_map", default=True))
    )
    #path to nav launch file.
    nav_launch_file=os.path.join(get_package_share_directory(nav_package_name),"launch","navigation.launch.py")

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav_launch_file
            ),condition=IfCondition(LaunchConfiguration("launch_nav",default=True))
    )

    # Add the launch actions to the LaunchDescription
    ld.add_action(joy_launch)
    ld.add_action(map_launch)
    ld.add_action(TimerAction(period=5.0,actions=[nav_launch]))

    return ld
 