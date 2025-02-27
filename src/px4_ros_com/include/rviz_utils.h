#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace rviz_utils{
    visualization_msgs::msg::Marker createLineMarker(const std::vector<Eigen::Vector3f>& points, const std::string& frame_id);
    visualization_msgs::msg::Marker createPointMarker(const std::vector<Eigen::Vector3f>& points, const std::string& frame_id);
    visualization_msgs::msg::Marker createSquareMarker(const Eigen::Vector3f& point, const std::string& frame_id);
    visualization_msgs::msg::Marker createArrowMarker(const Eigen::Vector3f& position, float heading, const std::string& frame_id);
    visualization_msgs::msg::Marker createArrowMarker(const Eigen::Vector3f& position, float dx, float dy, const std::string& frame_id);
}
