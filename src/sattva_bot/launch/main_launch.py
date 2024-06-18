import os
import time
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()
    package_name="prototype"
    

    # Specify the paths to the launch files
    
    #path to robotmodel_launch file.
    robotmodel_launch_file=os.path.join(get_package_share_directory(package_name),"launch","robotmodel_launch.py")
    robotmodel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robotmodel_launch_file)
    )
      #path to sensors_launch file.
    sensors_launch_file=os.path.join(get_package_share_directory(package_name),"launch","sensors_launch.py")
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_file)
    )
    #Specify the laser_pose file.
    robot_locate_file=os.path.join(get_package_share_directory(package_name),"launch","robot_localization_launch.py")
    robot_locate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_locate_file)
    )
    twist_locate_file=os.path.join(get_package_share_directory(package_name),"launch","twist_mux.launch.py")
    twist_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_locate_file)
    )
    # Add the launch actions to the LaunchDescription
    ld.add_action(robotmodel_launch)
    ld.add_action(TimerAction(period=15.0,actions=[sensors_launch]))
    ld.add_action(TimerAction(period=60.0,actions=[robot_locate]))
    ld.add_action(TimerAction(period=70.0,actions=[twist_launch]))

    return ld
 