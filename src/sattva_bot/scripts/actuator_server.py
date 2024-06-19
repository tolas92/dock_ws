#! /usr/bin/env python3
# Copyright 2024 Open Navigation LLC
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

import time
import rclpy
from rclpy.action import ActionClient,ActionServer
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Float64MultiArray
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped,PoseStamped
from prototype.action import Actuator
from rclpy.executors import MultiThreadedExecutor


actuator_pos=None
actuator_action_done=None

class ActuatorController(Node):

    def __init__(self):
        super().__init__(node_name='actuator_controller')
        self.lift_publisher=self.create_publisher(Float64MultiArray,'/lift_actuator/control',1)
        self.load_actuator = ActionServer(self, Actuator,'PalletActuator',self.move_actuator)
        global actuator_pos


    def destroy_node(self):
        super().destroy_node()

    def move_actuator(self,pos):
        global actuator_pos
        pos.succeed()
        feedback_msg=Actuator.Feedback()
        result=Actuator.Result()
        msg = Float64MultiArray()

        msg.data = [pos.request.position]
        self.lift_publisher.publish(msg)

        if(pos.request.position==1.0):
            actuator_pos='up'
        elif(pos.request.position==0.0):
            actuator_pos='down'
        timeout=0

        while timeout<30:
            feedback_msg.status="waiting"
            pos.publish_feedback(feedback_msg)
            timeout+=1
            time.sleep(1)
            if(actuator_action_done):                
                result.done=True
                return result
        pos.abort()      
                    
    
class ActuatorTransform(Node):

    def __init__(self):
        super().__init__(node_name='actuator_transform')

        self.actuator_up_=1.222 # The tf between the pallet_loader and the chassis when the actuator is in full up position
        self.actuator_down_=0.222 # The tf between the pallet_loader and the chassis when the actuator is in full down position
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.target_transform_value=1.222
        self.check_done=False
        self.check_timer=self.create_timer(1.0,self.check_actuator)
    
    def check_actuator(self):
        from_frame = 'pallet_loader'
        to_frame = 'chassis'
        try:
            # Block until the transform is available
            when = rclpy.time.Time()
            transform_stamped=self._tf_buffer.lookup_transform(
            to_frame, from_frame, when, timeout=Duration(seconds=1.0)
            )
            
            if self.transform_meets_criteria(transform_stamped):
               global actuator_action_done
               actuator_action_done=True
               
               return
            
        except LookupException:
            self.get_logger().info('transform not ready')
        
    def transform_meets_criteria(self, transform_stamped: TransformStamped) -> bool:
        global actuator_pos
        z_translation = transform_stamped.transform.translation.z
        if actuator_pos == 'up':
            if abs(z_translation - self.actuator_up_) < 0.1:
                return True
        elif actuator_pos == 'down':
            if abs(z_translation - self.actuator_down_) < 0.1:
                #self.get_logger().info(f"Actuator in full down position: z = {z_translation}")
                return True
        return False
    



def main():
    rclpy.init()
    actuator_server=ActuatorController()
    actuator_tf=ActuatorTransform()
    # Set up mulithreading
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(actuator_server)
    executor.add_node(actuator_tf)
    
    try:
      # Spin the nodes to execute the callbacks
      executor.spin()
    finally:
      # Shutdown the nodes
      executor.shutdown()
      actuator_server.destroy_node()
      actuator_tf.destroy_node()
      rclpy.shutdown()

if __name__=='__main__':
    main()