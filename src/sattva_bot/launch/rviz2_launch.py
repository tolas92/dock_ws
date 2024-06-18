from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()

    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Add the RViz2 node to the launch description
    ld.add_action(rviz_node)

    return ld
