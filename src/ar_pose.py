#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Header

class PoseArrayToPoseStamped(Node):

    def __init__(self):
        super().__init__('pose_array_to_pose_stamped')
        #self.subscription = self.create_subscription(
           # PoseArray,
          #  'aruco_poses',  # Replace with your PoseArray topic name
         #   self.listener_callback,
        #    10)
        
        self.publisher = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.get_logger().info('Node is up and running, subscribing to PoseArray and publishing PoseStamped')
        self.timer=self.create_timer(0.02,self.listener_callback)

    def listener_callback(self):
        # Assume we only take the first pose from PoseArray for conversion
        #if len(msg.poses) > 0:
            pose_stamped = PoseStamped()
            pose_stamped.header= Header()
            pose_stamped.header.stamp=self.get_clock().now().to_msg()
            pose_stamped.header.frame_id="odom"

            #pose_stamped.pose = msg.poses[0]
            pose_stamped.pose.position.x=0.6
            pose_stamped.pose.position.y=0.0

            self.publisher.publish(pose_stamped)
            self.get_logger().info('Published PoseStamped message')
        #else:
            #self.get_logger().warn('Received an empty PoseArray')

def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayToPoseStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
