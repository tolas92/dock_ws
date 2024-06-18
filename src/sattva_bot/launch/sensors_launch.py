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
    
    #path to laser_launch file.
    laser_launch_file=os.path.join(get_package_share_directory(package_name),"launch","laser_launch.py")
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(laser_launch_file)
    )
      #path to laser_launch file.
    imu_launch_file=os.path.join(get_package_share_directory(package_name),"launch","imu_launch.py")
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_file)
    )
    #Specify the motor_launch file.
    motor_launch_file=os.path.join(get_package_share_directory(package_name),"launch","motor_launch.py")
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motor_launch_file)
    )

    # Add the launch actions to the LaunchDescription
    ld.add_action(laser_launch)
    ld.add_action(TimerAction(period=30.0,actions=[motor_launch]))
    ld.add_action(TimerAction(period=60.0,actions=[imu_launch]))

    return ld
