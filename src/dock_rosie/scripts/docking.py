import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from prototype.msg import DistanceAngle

PI = 3.14
MAX_ANGLE = 10
MAX_ORIENT = 1
MAX_DISTANCE = 2
MAX_VEL = 0.15
MIN_VEL = 0.04
MAX_TWIST1 = 0.55
MAX_TWIST2 = 0.7
PHI_MAX = 43
PHI_MIN = 23
PHI_ANG = 37
MAX_ANG = 40
AR_DIST = 0.3

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking')
        self.velocities_pub = self.create_publisher(Twist, '/diff_controller/cmd_vel_unstamped', 10)
        self.create_subscription(DistanceAngle, 'DistanceAngle', self.docking_callback, 1)

    def velocities_function(self, station, twist):
        motor = Twist()
        motor.linear.x = -(MIN_VEL + (MAX_VEL - MIN_VEL) * (station.distance / MAX_DISTANCE))
        motor.angular.z = twist * (station.distance / MAX_DISTANCE)
       # self.get_logger().info('linear [%f], angular [%f]', motor.linear.x, motor.angular.z)
        self.velocities_pub.publish(motor)

    def docking_callback(self, station):
            phi = PHI_MIN + (PHI_MAX - PHI_MIN) * (station.distance / MAX_DISTANCE) + PHI_ANG * (abs(station.angle) / MAX_ANG)
            print(phi)
            
            if station.distance > AR_DIST:
                """
                if station.angle < -MAX_ANGLE and station.orientation < phi:
                    self.velocities_function(station, MAX_TWIST1)
                elif station.angle > MAX_ANGLE and station.orientation < -phi:
                    self.velocities_function(station, -MAX_TWIST1)
                elif station.angle > MAX_ANGLE and station.orientation > -phi:
                    self.velocities_function(station, -MAX_TWIST1)
                elif station.angle < -MAX_ANGLE and station.orientation > phi:
                    self.velocities_function(station, MAX_TWIST1)
                """
                if -MAX_ANGLE < station.angle < MAX_ANGLE:

                    if station.orientation > MAX_ORIENT:
                        self.velocities_function(station, -MAX_TWIST2)
                    elif station.orientation < -MAX_ORIENT:
                        self.velocities_function(station, MAX_TWIST2)
                    else:
                        self.velocities_function(station, 0)
            else:
                motor = Twist()
                motor.linear.x = 0.0
                motor.angular.z = 0.0
                #self.get_logger().info('linear [%f], angular [%f]', motor.linear.x, motor.angular.z)
                self.velocities_pub.publish(motor)

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
