#include "pallet_loader/comms.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"



class ImuPublisherNode : public rclcpp::Node
{
public:
  ImuPublisherNode() : Node("imu_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ImuPublisherNode::publishImuData, this));
    comms_.setup("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_75834353031351417031-if00", 115200, 10000);
    
  }

private:
  void publishImuData()
  {
    comms_.read_imu_values(comms_.accelX,comms_.accelY,comms_.accelZ,
    comms_.gyroX);
     //RCLCPP_INFO(rclcpp::get_logger("imu_node"), "Serial driver is open!");
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_msg.linear_acceleration.x =0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;
    imu_msg.angular_velocity.x =0.0;
    imu_msg.angular_velocity.y =0.0;
    imu_msg.angular_velocity.z =0.0;
    imu_msg.orientation.x=comms_.accelX;
    imu_msg.orientation.y=comms_.accelY;
    imu_msg.orientation.z=comms_.accelZ;
    imu_msg.orientation.w=comms_.gyroX;
    
    publisher_->publish(imu_msg);
  }

  Comms comms_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisherNode>());
  rclcpp::shutdown();
  return 0;
}