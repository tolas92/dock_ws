#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "dock_rosie/msg/distance_angle.hpp"

#define flags 1
#define PI 3.14

class MarkerParameters
{
public:
    float distance;
    float angle;
    float Yaw_odom;
    float Pitch_odom;
    float Roll_odom;
    float Yaw;
    float Pitch;
    float Roll;

    void ComputeOdomAngle(const tf2_msgs::msg::TFMessage &robot, int i)
    {
        float q0, q1, q2, q3;

        q0 = robot.transforms[0].transform.rotation.x;
        q1 = robot.transforms[0].transform.rotation.y;
        q2 = robot.transforms[0].transform.rotation.z;
        q3 = robot.transforms[0].transform.rotation.w;

        Yaw_odom = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180 / PI;
        Pitch_odom = asin(2 * (q0 * q2 - q3 * q1)) * 180 / PI;
        Roll_odom = atan2(2 * (q0 * q3 + q2 * q1), 1 - 2 * (q0 * q0 + q1 * q1)) * 180 / PI;
    }

    void Computedistangle(const tf2_msgs::msg::TFMessage &ar_marker, int i)
    {
        float x, y, xp, yp, Yaw_odom_rad;

        Yaw_odom_rad = Yaw_odom * PI / 180;
        xp = ar_marker.transforms[i].transform.translation.z;
        yp = ar_marker.transforms[i].transform.translation.x;

        x = cos(Yaw_odom_rad) * xp + sin(Yaw_odom_rad) * yp;
        y = -sin(Yaw_odom_rad) * xp + cos(Yaw_odom_rad) * yp;

        distance = sqrt(x * x + y * y);
        angle = atan(y / x) * 180 / PI;
    }

    void ComputeRPY(const tf2_msgs::msg::TFMessage &ar_marker, int i)
    {
        float q0, q1, q2, q3;

        q0 = ar_marker.transforms[i].transform.rotation.x;
        q1 = ar_marker.transforms[i].transform.rotation.y;
        q2 = ar_marker.transforms[i].transform.rotation.z;
        q3 = ar_marker.transforms[i].transform.rotation.w;

        Yaw = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180 / PI;
        Pitch = asin(2 * (q0 * q2 - q3 * q1)) * 180 / PI;
        Roll = atan2(2 * (q0 * q3 + q2 * q1), 1 - 2 * (q0 * q0 + q1 * q1)) * 180 / PI;
    }

    void CopytoMarker(distanceangle::msg::DistanceAngle &marker)
    {
        marker.distance = distance;
        marker.angle = angle;
        marker.orientation = Yaw + Yaw_odom;
    }
};

MarkerParameters ARmarker;

void odomangleCallback(const tf2_msgs::msg::TFMessage::SharedPtr robot)
{
    std_msgs::msg::Float64 odomangle;
    int i;

    for (i = 0; i < flags; ++i)
    {
        if ((robot->transforms[0].header.frame_id == "odom") && (robot->transforms[0].child_frame_id == "base_link"))
        {
            ARmarker.ComputeOdomAngle(*robot, i);
        }

        if ((robot->transforms[0].header.frame_id == "camera") && (robot->transforms[0].child_frame_id == "ar_marker_2"))
        {
            ARmarker.Computedistangle(*robot, i);
            ARmarker.ComputeRPY(*robot, i);
            ARmarker.CopytoMarker(marker);
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: [%f], angle: [%f], orientation: [%f]", marker.distance, marker.angle, marker.orientation);

        DistanceAngle_pub->publish(marker);

        odomangle.data = ARmarker.Yaw_odom;
        OdomAngle_pub->publish(odomangle);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("distance_angle");

    auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 1, odomangleCallback);

    OdomAngle_pub = node->create_publisher<std_msgs::msg::Float64>("odomangle", 1000);
    DistanceAngle_pub = node->create_publisher<distanceangle::msg::DistanceAngle>("DistanceAngle", 1000);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
