import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():



    return LaunchDescription([

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [640,480],
                'camera_frame_id': 'camera_link_optical',
                'camera_info_url':'file:///home/tolasing/main_ws/dev_ws/calibrationdata/ost.yaml',
                'publish_rate':'60.0',
                'time_per_frame':[1,10]
                }]
    )
    ])