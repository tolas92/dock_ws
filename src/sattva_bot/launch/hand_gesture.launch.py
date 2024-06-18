import os
import time
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()
    hand_gesture_package_name="hand_control"    

    # Specify the paths to the launch files
    hand_launch_file=os.path.join(get_package_share_directory(hand_gesture_package_name),"launch","hand_launch.py")

    hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            hand_launch_file
            )
    )

    # Add the launch actions to the LaunchDescription
    ld.add_action(hand_launch)

    return ld
 