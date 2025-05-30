#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "motionProfiling.h"
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

enum class DroneState { IDLE, ARM, TAKEOFF, LOITER, OFFBOARD, LAND };

struct GeoWP { double lat, lon, alt; };

using namespace Eigen;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class Mission : public rclcpp::Node {
public:
    Mission();

private:
    // ───── Parameters & State ────────────────────────────
    std::vector<GeoWP> geo_waypoints;
    std::vector<double> raw_waypoints;
    std::vector<Vector3f> segment_waypoints;
    double max_velocity, time_to_max, takeoff_alt;
    GeographicLib::LocalCartesian geodetic_proj;
    bool home_set;
    bool takeoff_offset_set;
    float tolerance;
    DroneState drone_state;
    std::shared_ptr<MotionProfiling> curr_traj;
    uint8_t nav_status = VehicleStatus::NAVIGATION_STATE_MAX;
    uint8_t arm_status = VehicleStatus::NAVIGATION_STATE_MAX;
    uint8_t failsafe = false;
    uint8_t flight_check = VehicleStatus::NAVIGATION_STATE_MAX;
    Vector3f takeoff_offset;
    Vector3f target_pos;
    int offboard_counter;
    std::vector<Eigen::Vector3f> local_wps;
    Vector3f home_gps;
    int current_waypoint;

    // ───── Publishers & Subscribers ──────────────────────
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_setpoint;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_command;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_local;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_marker;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_traj_marker;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_wp_marker;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_heading_marker;

    rclcpp::TimerBase::SharedPtr timer;

    // ───── Callbacks & Helpers ───────────────────────────
    void declare_parameters();
    void setup_publishers();
    void setup_subscribers();

    void convert_waypoints();
    void main_loop();
    void offboard();

    void vehicle_local_position_callback(const VehicleOdometry::SharedPtr msg);
    void vehicle_global_position_callback(const VehicleGlobalPosition::SharedPtr msg);
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(double t);
    void publish_drone_marker(Vector3f pose);
    void publish_vehicle_command(unsigned command, float p1=0, float p2=0, float p3=0,
                            float p4=0, float p5=0, float p6=0, float p7=0);
    void print_parameters();

    void disarm();
    void arm();
    void takeoff();
};
