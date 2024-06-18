import os 
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import IncludeLaunchDescription,TimerAction
import xacro

def generate_launch_description():
    robot_name="proto"
    robot_description=Command(['ros2 param get --hide-type /my_robot_state_publisher_node robot_description'])
    controller_config=os.path.join(
        get_package_share_directory
        ("prototype"),"config","motor_control.yaml"
    )

    controller_ros2_control= Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description':robot_description},controller_config],
        output={
        'stdout':'screen',
        'stderr':'screen'
        },
        )
    
    delayed_controller_ros2_control=TimerAction(period=3.0,actions=[controller_ros2_control])
    
    diff_drive_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"]
    )

    delayed_diff_drive_spawner=RegisterEventHandler(
        event_handler=OnProcessStart(
        target_action=controller_ros2_control,
        on_start=[diff_drive_spawner]
        )
    )
    
    imu_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster"]
    )
    
    delayed_imu_spawner=RegisterEventHandler(
        event_handler=OnProcessStart(
        target_action=controller_ros2_control,
        on_start=[imu_spawner]
        )
    )
    
    
    

    Joint_state_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )
    
    

    delayed_Joint_state_spawner=RegisterEventHandler(
        event_handler=OnProcessStart(target_action=
                                     controller_ros2_control,
                                     on_start=[Joint_state_spawner])
        )

    return LaunchDescription([
        delayed_controller_ros2_control,
        delayed_diff_drive_spawner,
       # delayed_imu_spawner,
        delayed_Joint_state_spawner
    ])