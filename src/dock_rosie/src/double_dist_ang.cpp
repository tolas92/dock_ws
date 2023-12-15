#include <rclcpp/rclcpp.hpp>
#include <ar_track_alvar_msgs/msg/AlvarMarkers.hpp>
#include "distance_angle/msg/DistanceAngle.hpp"
#include <cmath>

#define PI 3.14159265
#define baseline 0.12
#define flags 1

rclcpp::Publisher<distance_angle::msg::DistanceAngle>::SharedPtr DistanceAngle_pub;
distance_angle::msg::DistanceAngle marker;

class MarkerParameters
{
public:
    float distance;
    float angle;
    float Yaw;
    float Pitch;
    float Roll;

    void Computedistangle(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr ar_marker, int i)
    {
        float x, y;

        x = ar_marker->markers[i].pose.pose.position.x;
        y = ar_marker->markers[i].pose.pose.position.y;

        distance = sqrt(x * x + y * y);
        angle = atan2(y, x) * 180 / PI;
    }

    void ComputeRPY(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr ar_marker, int i)
    {
        float q0, q1, q2, q3;

        q0 = ar_marker->markers[i].pose.pose.orientation.x;
        q1 = ar_marker->markers[i].pose.pose.orientation.y;
        q2 = ar_marker->markers[i].pose.pose.orientation.z;
        q3 = ar_marker->markers[i].pose.pose.orientation.w;

        Yaw = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180 / PI;
        Pitch = asin(2 * (q0 * q2 - q3 * q1)) * 180 / PI;
        Roll = atan2(2 * (q0 * q3 + q2 * q1), 1 - 2 * (q0 * q0 + q1 * q1)) * 180 / PI;
    }

    void CopytoMarker(distance_angle::msg::DistanceAngle &marker)
    {
        marker.distance = distance;
        marker.angle = angle;
        marker.orientation = Yaw + 90; // for having 0 degree into the perpendicular line
    }
};

void distanceangleCallback(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr ar_pose_marker)
{
    int i;

    for (i = 0; i < flags; ++i)
    {
        MarkerParameters ARmarker;
        ARmarker.Computedistangle(ar_pose_marker, i);
        ARmarker.ComputeRPY(ar_pose_marker, i);
        ARmarker.CopytoMarker(marker);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: [%f], angle: [%f], orientation: [%f]", marker.distance, marker.angle, marker.orientation);

        DistanceAngle_pub->publish(marker);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("distance_angle");

    DistanceAngle_pub = node->create_publisher<distance_angle::msg::DistanceAngle>("DistanceAngle", 1000);

    auto sub = node->create_subscription<ar_track_alvar_msgs::msg::AlvarMarkers>("pose_marker", 1, distanceangleCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
