import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from prototype.msg import DistanceAngle
import math

flags = 1
PI = 3.14159265
marker=DistanceAngle()

class MarkerParameters:
    def __init__(self):
        self.distance = 0.0
        self.angle = 0.0
        self.Yaw_odom = 0.0
        self.Pitch_odom = 0.0
        self.Roll_odom = 0.0
        self.Yaw = 0.0
        self.Pitch = 0.0
        self.Roll = 0.0

    def compute_odom_angle(self, robot, i):
        q0, q1, q2, q3 = (
            robot.transforms[i].transform.rotation.x,
            robot.transforms[i].transform.rotation.y,
            robot.transforms[i].transform.rotation.z,
            robot.transforms[i].transform.rotation.w,
        )

        self.Yaw_odom = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180 / PI
        self.Pitch_odom = math.asin(2 * (q0 * q2 - q3 * q1)) * 180 / PI
        self.Roll_odom = math.atan2(2 * (q0 * q3 + q2 * q1), 1 - 2 * (q0 * q0 + q1 * q1)) * 180 / PI

    def compute_dist_angle(self, ar_marker, i):
        Yaw_odom_rad = self.Yaw_odom * PI / 180
        xp = ar_marker.transforms[i].transform.translation.z
        yp = ar_marker.transforms[i].transform.translation.x

        x = math.cos(Yaw_odom_rad) * xp + math.sin(Yaw_odom_rad) * yp
        y = -math.sin(Yaw_odom_rad) * xp + math.cos(Yaw_odom_rad) * yp

        self.distance = math.sqrt(x * x + y * y)
        self.angle = math.atan2(y, x) * 180 / PI

    def compute_rpy(self, ar_marker, i):
        q0, q1, q2, q3 = (
            ar_marker.transforms[i].transform.rotation.x,
            ar_marker.transforms[i].transform.rotation.y,
            ar_marker.transforms[i].transform.rotation.z,
            ar_marker.transforms[i].transform.rotation.w,
        )

        self.Yaw = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180 / PI
        self.Pitch = math.asin(2 * (q0 * q2 - q3 * q1)) * 180 / PI
        self.Roll = math.atan2(2 * (q0 * q3 + q2 * q1), 1 - 2 * (q0 * q0 + q1 * q1)) * 180 / PI

    def copy_to_marker(self, marker):
        marker.distance = self.distance
        marker.angle = self.angle
        marker.orientation = self.Yaw + self.Yaw_odom

ARmarker = MarkerParameters()

def odomangle_callback(robot):
    global DistanceAngle_pub, OdomAngle_pub
    odomangle = Float64()
    
    for i in range(flags):
        if (
            robot.transforms[0].header.frame_id == "odom"
            and robot.transforms[0].child_frame_id == "base_link"
        ):
            ARmarker.compute_odom_angle(robot, i)

        if (
            robot.transforms[0].header.frame_id == "camera"
            and robot.transforms[0].child_frame_id == "ar_marker"
        ):
            ARmarker.compute_dist_angle(robot, i)
            ARmarker.compute_rpy(robot, i)
            ARmarker.copy_to_marker(marker)

        print(
            f"Distance: {marker.distance}, angle: {marker.angle}, orientation: {marker.orientation}"
        )

        DistanceAngle_pub.publish(marker)

        odomangle.data = ARmarker.Yaw_odom
        OdomAngle_pub.publish(odomangle)

class DistanceAngleNode(Node):
    def __init__(self):
        super().__init__("distance_angle")
        global DistanceAngle_pub, OdomAngle_pub

        DistanceAngle_pub = self.create_publisher(DistanceAngle, "DistanceAngle", 1000)
        OdomAngle_pub = self.create_publisher(Float64, "odomangle", 1000)
        
        self.create_subscription(TFMessage, "/tf", odomangle_callback, 1)

def main(args=None):
    rclpy.init(args=args)
    node = DistanceAngleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
