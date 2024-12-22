#include "trajectory.h"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include "rviz_utils.h"

class TrajectoryTestNode : public rclcpp::Node {
public:
    TrajectoryTestNode() : Node("trajectory_test_node") {
        marker1_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
        marker2_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 10);

        // Initialize waypoints
        std::vector<Eigen::Vector3f> waypoints = {
            {0.0f, 0.0f, 0.0f},
            {1.0f, 1.0f, 0.5f},
            {2.0f, 0.0f, 1.0f},
            {3.0f, 1.0f, 0.5f}
        };

        // Initialize Trajectory
        Trajectory traj(1.0, 1.0, &waypoints); // vmax = 1.0, a = 0.5

        // Sample points along the trajectory
        std::vector<Eigen::Vector3f> trajectory_points;
        const int samples = 100;
        float total_time = traj.getTotalTime();
        for (int i = 0; i <= samples; ++i) {
            float t = static_cast<float>(i) * total_time / samples;
            trajectory_points.push_back(traj.getPosition(t));
        }

        // Create markers for visualization
        visualization_msgs::msg::Marker marker1 = rviz_utils::createLineMarker(trajectory_points, "/map");
        visualization_msgs::msg::Marker marker2 = rviz_utils::createPointMarker(waypoints, "/map");

        std::cout << marker1.points.size() << std::endl;
        std::cout << marker2.points.size() << std::endl;

        // Publish marker periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this, marker1, marker2]() {
                marker1_pub_->publish(marker1); 
                marker2_pub_->publish(marker2); 
                });
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker1_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker2_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
