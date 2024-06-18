#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes


depthai_prefix = get_package_share_directory("prototype")

params_file = os.path.join(depthai_prefix, 'config', 'camera.yaml')
tf_params = {'camera': {
            'i_publish_tf_from_calibration': True,
            'i_tf_tf_prefix': "oak",
            'i_tf_camera_model': "OAK-D",
            'i_tf_base_frame': "oak",
            'i_tf_parent_frame':"oak-d-base-frame" ,
            'i_tf_cam_pos_x': '0.0',
            'i_tf_cam_pos_y': '0.0',
            'i_tf_cam_pos_z': '0.0',
            'i_tf_cam_roll': '0.0',
            'i_tf_cam_pitch': '0.0',
            'i_tf_cam_yaw': '0.0',
            'i_tf_imu_from_descr': 'false',
        }
        }
def generate_launch_description():
    ld = LaunchDescription()

    cam_compose_node=ComposableNodeContainer(
            name="oak_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions= [
                ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                name="oak",
                parameters=[params_file,tf_params],
                )
            ],
            arguments=['--ros-args','--log-level','info'],
            prefix=[''],
            output="both")
    
    spatial_compose_node=LoadComposableNodes(
            target_container="oak_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_filters",
                        plugin="depthai_filters::SpatialBB",
                        remappings=[
                                    ('stereo/camera_info', 'oak/stereo/camera_info'),
                                    ('nn/spatial_detections','oak/nn/spatial_detections'),
                                    ('rgb/preview/image_raw', 'oak/rgb/preview/image_raw')]
                    ),
            ],
        )

    
            
    

    ld.add_action(cam_compose_node)
    ld.add_action(spatial_compose_node)

    return ld