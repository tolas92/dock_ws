import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   
   parameters = [
        {
            "frame_id": "oak",
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": True,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
        }
    ]

  
   uncompressed_node=Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='rgbd_odometry',
            parameters=parameters,
            )
  

   return LaunchDescription([uncompressed_node])
