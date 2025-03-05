#include "rviz_utils.h"
#include "px4_ros_com/frame_transforms.h"
#include <cmath>

// Helper function to create markers for RViz visualization
visualization_msgs::msg::Marker rviz_utils::createLineMarker(const std::vector<Eigen::Vector3f>& points, const std::string& frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "trajectory_test";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1; // Line width
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    for (const auto& point : points) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        marker.points.push_back(p);
    }

    return marker;
}

visualization_msgs::msg::Marker rviz_utils::createArrowMarker(const Eigen::Vector3f &position, float heading, const std::string& frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id; // Set the appropriate frame ID
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "trajectory_test";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the marker
    double arrow_length = 1.0;  // Length of the arrow
    marker.scale.x = 0.1;  // Shaft diameter
    marker.scale.y = 0.2;  // Head diameter
    marker.scale.z = 0.0;  // Not used for ARROW type

    // Set the color (purple)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Set the position of the arrow
    geometry_msgs::msg::Point start_point;
    start_point.x = position.x();
    start_point.y = position.y();
    start_point.z = position.z();

    // Set the orientation of the arrow based on the heading
    double end_x = start_point.x + arrow_length * cos(heading);
    double end_y = start_point.y + arrow_length * sin(heading);

    geometry_msgs::msg::Point end_point;
    end_point.x = end_x;
    end_point.y = end_y;
    end_point.z = position.z();

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    return marker;
}

visualization_msgs::msg::Marker rviz_utils::createArrowMarker(const Eigen::Vector3f &position, float dx, float dy, const std::string& frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id; // Set the appropriate frame ID
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "trajectory_test";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the marker
    double arrow_length = 1.0;  // Length of the arrow
    marker.scale.x = 0.1;  // Shaft diameter
    marker.scale.y = 0.2;  // Head diameter
    marker.scale.z = 0.0;  // Not used for ARROW type

    // Set the color (purple)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Set the position of the arrow
    geometry_msgs::msg::Point start_point;
    start_point.x = position.x();
    start_point.y = position.y();
    start_point.z = position.z();

    // Set the orientation of the arrow based on the heading
    double end_x = start_point.x + arrow_length * dx;
    double end_y = start_point.y + arrow_length * dy;

    geometry_msgs::msg::Point end_point;
    end_point.x = end_x;
    end_point.y = end_y;
    end_point.z = position.z();

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    return marker;
}

// Helper function to create markers for RViz visualization
visualization_msgs::msg::Marker rviz_utils::createPointMarker(const std::vector<Eigen::Vector3f>& points, const std::string& frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "trajectory_test";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    for (const auto& point : points) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        marker.points.push_back(p);
    }

    return marker;
}

visualization_msgs::msg::Marker rviz_utils::createSquareMarker(const Eigen::Vector3f& point, const std::string& frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "trajectory_test";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    marker.points.push_back(p);

    return marker;
}