#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sattva_bot.sattva_docking   import DockingTester
from dock_ws.src.sattva_bot.scripts.actuator_server  import ActuatorController
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer ,GoalResponse,CancelResponse
from rclpy.action import GoalResponse
from prototype.action import GoTo
import time


class Sattva(Node):

    def __init__(self):
        super().__init__('sattva_demo')
        self.docker_=DockingTester()
        self.actuator_controller=ActuatorController()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('frame_id',rclpy.Parameter.Type.STRING),
                ('load_dock.pose',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('unload_dock.pose',rclpy.Parameter.Type.DOUBLE_ARRAY),

                ])
        #self.command_ = self.create_subscription(String, '/table_number',self.callback, 10)
        #self.navigator = BasicNavigator()
        #self.counter=0
    def callback(self,msg): 
        while self.counter <10:
            #first navigate to staging pose for loading.
            load_pose=self.get_parameter('load_dock.pose').value
            frame_id_=self.get_parameter('frame_id').value
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = frame_id_
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x =load_pose[0]
            goal_pose.pose.position.y =load_pose[1]
            goal_pose.pose.orientation.z=load_pose[2]
            goal_pose.pose.orientation.w =load_pose[3]
            self.navigator.goToPose(goal_pose)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                feedback_=GoTo.Feedback()
                feedback_.distance_left=Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            result = self.navigator.getResult()
            if result==TaskResult.SUCCEEDED:
                self.actuator_controller.lift_actuator()
                

                
                

            self.navigator.clearGlobalCostmap()
    """

def main(args=None):
    rclpy.init(args=args)
    sattva = Sattva()
    rclpy.spin(sattva)
    sattva.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()