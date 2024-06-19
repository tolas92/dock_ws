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

from enum import Enum
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from opennav_docking_msgs.action import DockRobot, UndockRobot
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class DockingTester(Node):

    def __init__(self):
        super().__init__(node_name='docking_tester')
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_frame = 'map'
        self.chassis_link_frame = 'chassis_link'
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        self.docking_client = ActionClient(self, DockRobot,
                                           'dock_robot')
        self.undocking_client = ActionClient(self, UndockRobot,
                                             'undock_robot')
        self.actuator_publisher = self.create_publisher(PoseStamped, '/forward_position_controller/commands', 10)

    def timer_callback(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame=self.map_frame,
                source_frame=self.chassis_link_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0)
            )
            self.position_transform(transform_stamped)
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"Error: {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f"Error: {e}")
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f"Error: {e}")
        except tf2_ros.InvalidArgumentException as e:
            self.get_logger().error(f"Error: {e}")

    def position_transform(self, transform_stamped):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = transform_stamped.header.stamp
        pose_msg.header.stamp.sec = 0
        pose_msg.header.stamp.nanosec = 0
        # pose_msg.pose.position.x = transform_stamped.transform.translation.x
        # pose_msg.pose.position.y = transform_stamped.transform.translation.y
        try:
            transform_stamped.position.z = 1.0
            self.get_logger().info("you are in right track")
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"Error: {e} ")


        self.publisher.publish(pose_msg)

    def destroy_node(self):
        self.docking_client.destroy()
        self.undocking_client.destroy()
        super().destroy_node()

    def dockRobot(self, dock_pose, dock_type=""):
        """Send a `DockRobot` action request."""
        print("Waiting for 'DockRobot' action server")
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            print('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = False
        # goal_msg.dock_id = dock_id  # if wanting to use ID instead
        goal_msg.dock_pose = dock_pose
        goal_msg.dock_type = dock_type
        # goal_msg.navigate_to_staging_pose = True  # if want to navigate before staging

        print('Docking at pose: ' + str(dock_pose) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg,
                                                               self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Docking request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def undockRobot(self, dock_type):
        """Send a `UndockRobot` action request."""
        print("Waiting for 'UndockRobot' action server")
        while not self.undocking_client.wait_for_server(timeout_sec=1.0):
            print('"UndockRobot" action server not available, waiting...')

        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = dock_type

        print('Undocking from dock of type: ' + str(dock_type) + '...')
        send_goal_future = self.undocking_client.send_goal_async(goal_msg,
                                                                 self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Undocking request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        print('Task succeeded!')
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def cancelAction(self):
        self.goal_handle.cancel_goal()

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def startup(self, node_name='docking_server'):
        # Waits for the node within the tester namespace to become active
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def lift_actuator(self):
        print("Lifting the actuator...")
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "chassis_link"
        pose_msg.pose.position.z = 1.0  # Example height for lifting
        self.actuator_publisher.publish(pose_msg)

    def lower_actuator(self):
        print("Lowering the actuator...")
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "chassis_link"
        pose_msg.pose.position.z = 0.0  # Example height for lowering
        self.actuator_publisher.publish(pose_msg)


def main():
    rclpy.init()

    tester = DockingTester()
    tester.startup()

    time.sleep(1)

    # Example staging pose 40 cm in front of itself.
    staging_pose = PoseStamped()
    staging_pose.header.stamp = tester.get_clock().now().to_msg()
    staging_pose.header.frame_id = "camera_link"
    staging_pose.pose.position.x = 0.4
    staging_pose.pose.position.y = 0.0
    staging_pose.pose.orientation.z = 0.0
    staging_pose.pose.orientation.w = 0.99

    # Move to staging area
    tester.dockRobot(staging_pose)

    i = 0
    while not tester.isTaskComplete():
        i += 1
        if i % 5 == 0:
            print('Moving to staging area...')
        time.sleep(1)

    result = tester.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached staging area!')
        time.sleep(1)

        # Lift the actuator
        tester.lift_actuator()
        tester.timer_callback()

        # Move to docking area
        dock_pose = PoseStamped()
        dock_pose.header.stamp = tester.get_clock().now().to_msg()
        dock_pose.header.frame_id = "camera_link"
        dock_pose.pose.position.x = 1.78
        dock_pose.pose.position.y = 0.0
        dock_pose.pose.orientation.z = 0.0
        dock_pose.pose.orientation.w = 0.99
        tester.dockRobot(dock_pose)

        i = 0
        while not tester.isTaskComplete():
            i += 1
            if i % 5 == 0:
                print('Docking in progress...')
            time.sleep(1)

        result = tester.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Docking succeeded!')

            # Lower the actuator
            tester.lower_actuator()
            time.sleep(5)

            # Undock
            tester.undockRobot('nova_carter_dock')
            i = 0
            while not tester.isTaskComplete():
                i += 1
                if i % 5 == 0:
                    print('Undocking in progress...')
                time.sleep(1)

            result = tester.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Undocking succeeded!')
            elif result == TaskResult.CANCELED:
                print('Undocking canceled!')
            elif result == TaskResult.FAILED:
                print('Undocking failed!')
            else:
                print('Undocking has an invalid return status!')

        elif result == TaskResult.CANCELED:
            print('Docking canceled!')
        elif result == TaskResult.FAILED:
            print('Docking failed!')
        else:
            print('Docking has an invalid return status!')

    elif result == TaskResult.CANCELED:
        print('Staging canceled!')
    elif result == TaskResult.FAILED:
        print('Staging failed!')
    else:
        print('Staging has an invalid return status!')

    time.sleep(3)

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
