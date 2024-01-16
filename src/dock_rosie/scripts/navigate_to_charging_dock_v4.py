#! /usr/bin/env python3

"""
Description:
  Navigate to a charging dock once the battery gets low.
  We first navigate to a staging area in front of the docking station (~1.5 meters is good)
  We rotate around to search for the ArUco marker using the robot's front camera.
  Once the ArUco marker is detected, move towards it, making minor heading adjustments as necessary.
  Stop once the robot gets close enough to the charging dock or starts charging.
-------
Subscription Topics:
  Current battery state
  /battery_status - sensor_msgs/BatteryState
  
  A boolean variable that is True of ArUco marker detected, otherwise False
  /aruco_marker_detected – std_msgs/Bool
  
  The number of pixels offset of the ArUco marker from the center of the camera image
  /aruco_marker_offset - std_msgs/Int32
  
  LaserScan readings for object detection
  /scan - sensor_msgs/LaserScan
-------
Publishing Topics:
  Velocity command to navigate to the charging dock.
  /cmd_vel - geometry_msgs/Twist
-------
Author: Addison Sears-Collins
Website: AutomaticAddison.com
Date: January 14, 2022
"""

import math # Math library
import time  # Time library

from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data # Handle quality of service for LaserScan data
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist # Velocity command
from sensor_msgs.msg import BatteryState # Battery status
from sensor_msgs.msg import LaserScan # Handle LIDAR scans
from std_msgs.msg import Bool # Handle boolean values
from std_msgs.msg import Int32 # Handle integer values
from geometry_msgs.msg import PoseArray

# Holds the current pose of the aruco_marker
# base_link (parent frame) -> aruco_marker (child frame)
current_x = 0.0
current_y = 0.0
current_yaw_angle = 0.0

# Holds the current state of the battery
this_battery_state = BatteryState()
prev_battery_state = BatteryState()

# Flag for detecting the change in the battery state
low_battery = False
low_battery_min_threshold = 0.25

# Flag to determine if the ArUco marker has been detected (True or False)
aruco_marker_detected = False

# Store the ArUco marker center offset (in pixels)
aruco_center_offset = 0
aruco_distance=0

# Keep track of obstacles in front of the robot in meters
obstacle_distance_front = 999999.9

class ConnectToChargingDockNavigator(Node):
    """
    Navigates and connects to the charging dock
    """      
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('connect_to_charging_dock_navigator')
    
      # Create a publisher
      # This node publishes the desired linear and angular velocity of the robot
      self.publisher_cmd_vel = self.create_publisher(
        Twist,
        '/diff_controller/cmd_vel_unstamped',
        10)  
      timer_period = 0.1
      self.timer = self.create_timer(timer_period, self.navigate_to_dock_staging_area)

      # Declare linear and angular velocities
      self.linear_velocity = -0.005  # meters per second
      self.angular_velocity = 0.03 # radians per second
      self.docking_station_distance=0.4
      self.center_pid = PIDController(0.8,0.0,0.0)
      self.angular_pid=PIDController(0.7,0.0,0.0)
      self.diaganol_pid=PIDController(3.0,0.0,0.0)
      self.perpendicular_pid=PIDController(0.4,0.0,0.0)

      
      # Keep track of which goal we're headed towards
      self.goal_idx = 1
      
      # Declare obstacle tolerance 
      self.obstacle_tolerance = 0.22

      # Center offset tolerance in pixels
      self.center_offset_tolerance = 0.028
      self.orient_offset_tolerance=0.000
      self.diaganol_offset_tolerance=0.1
      
      # Undocking distance
      self.undocking_distance = 0.50
      self.perpendicular_line_x=0.0
        
    def navigate_to_dock_staging_area(self):
      """
      Navigate from somewhere in the environment to a staging area near
      the charging dock.
      """    
      self.get_logger().info('Low battery. Navigating to the charging dock...')
    
      self.connect_to_dock()
        
    def connect_to_dock(self): 
      """
      Go to the charging dock.
      """ 
        
      if (self.goal_idx == 1):
          self.navigate_to_aruco_marker()
          self.get_logger().info('Navigating to the ArUco marker...')
      else:
          # Stop the robot
          cmd_vel_msg = Twist()
          cmd_vel_msg.linear.x = 0.0
          cmd_vel_msg.angular.z = 0.0
          self.publisher_cmd_vel.publish(cmd_vel_msg)
          #self.get_logger().info('Arrived at charging dock. Robot is idle...')
        
          time.sleep(0.02)
        
          self.get_logger().info('CHARGING...')
          self.get_logger().info('Successfully connected to the charging dock!')
          cmd_vel_msg = Twist()
          cmd_vel_msg.linear.x = 0.0
          cmd_vel_msg.angular.z = 0.0
          self.publisher_cmd_vel.publish(cmd_vel_msg)
          
      # Reset the node
      #self.goal_idx = 0
    def search_for_aruco_marker(self):
      """
      Rotate around until the robot finds the charging dock
      """
      if aruco_marker_detected == False:
      
        # Create a velocity message
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = -self.angular_velocity           
      
        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg) 
      else: 
        self.goal_idx = 1
        
    def navigate_to_aruco_marker(self):
      self.get_logger().info('calling adjust_heading')

      self.adjust_heading()

    def adjust_heading(self):
      """
      Adjust heading to keep the Aruco marker centerpoint centererd.

      """
      print(aruco_center_offset)
      cmd_vel_msg = Twist()

      
      if aruco_distance > self.docking_station_distance:
          self.perpendicular_line_x=aruco_distance*0.40
          center_correction = self.center_pid.calculate(aruco_center_offset - self.center_offset_tolerance)
          angular_correction=self.angular_pid.calculate(aruco_orient_offset-self.orient_offset_tolerance)
          perpendicular_correction=self.perpendicular_pid.calculate(aruco_center_offset-self.perpendicular_line_x)
          #try to reach the perpendicular line
          cmd_vel_msg.angular.z=-perpendicular_correction
          cmd_vel_msg.linear.x=-0.03
          self.publisher_cmd_vel.publish(cmd_vel_msg)    
      elif aruco_distance<self.docking_station_distance and aruco_distance > 0.225:
          angular_correction=self.angular_pid.calculate(aruco_orient_offset-self.orient_offset_tolerance)
          center_correction = self.center_pid.calculate(aruco_center_offset - self.center_offset_tolerance)
          cmd_vel_msg.angular.z=-center_correction
          cmd_vel_msg.linear.x=-0.01
          self.publisher_cmd_vel.publish(cmd_vel_msg)

         
                   


      else:
          print("hello")
      


  
        
class ArucoMarkerSubscriber(Node):
    """
    Subscriber node to help for the ArUco marker navigation routine.
    """      
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('aruco_marker_subscriber')
    
      # Create a subscriber 
      # This node subscribes to messages of type
      # std_msgs/Bool
      self.subscription_aruco_detected = self.create_subscription(
        Bool,
        '/aruco_marker_detected', 
        self.get_aruco_detected,
        1)

      # Create a subscriber 
      # This node subscribes to messages of type
      # std_msgs/Int32
      self.subscription_center_offset = self.create_subscription(
        PoseArray,
        '/aruco_poses', 
        self.get_center_offset,
        1)
        
      # Create a subscriber 
      # This node subscribes to messages of type
      # sensor_msgs/LaserScan
      self.subscription_laser_scan = self.create_subscription(
        LaserScan,
        '/scan', 
        self.scan_callback,
        qos_profile=qos_profile_sensor_data)

    def get_aruco_detected(self, msg):
      """
      Update if the ArUco marker has been detected or not
      """
      global aruco_marker_detected 
      aruco_marker_detected = msg.data
        
    def get_center_offset(self, msg):
      """
      Update the ArUco marker center offset
      """
      global aruco_center_offset
      global aruco_orient_offset
      global aruco_distance

      for pose in msg.poses:

        aruco_center_offset=round(pose.position.x,3)
        aruco_orient_offset=round(pose.orientation.z,3)
        aruco_distance=round(pose.position.z,5)


            
    def scan_callback(self, msg):
      """
      Update obstacle distance.
      """
      global obstacle_distance_front
      obstacle_distance_front = msg.ranges[179]

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, error):
        # Proportional term
        p = self.kp * error

        # Integral term
        self.integral += error
        i = self.ki * self.integral

        # Derivative term
        derivative = error - self.prev_error
        d = self.kd * derivative

        # Update the previous error
        self.prev_error = error

        # Calculate the total correction
        correction = p + i + d

        return correction

        
def main(args=None):
  """
  Entry point for the program.
  """
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  try: 
  
    # Create the nodes
    connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
    aruco_marker_subscriber = ArucoMarkerSubscriber()
    
    # Set up mulithreading
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(connect_to_charging_dock_navigator)
    executor.add_node(aruco_marker_subscriber)
    
    try:
      # Spin the nodes to execute the callbacks
      executor.spin()
    finally:
      # Shutdown the nodes
      executor.shutdown()
      connect_to_charging_dock_navigator.destroy_node()
      aruco_marker_subscriber.destroy_node()

  finally:
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
  main()
