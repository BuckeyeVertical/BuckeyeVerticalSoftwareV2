#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "motionProfiling.h"

enum class DroneState { TAKEOFF, FOLLOW, LOITER, LAND };

struct GeoWP { double lat, lon, alt; };

class Mission : public rclcpp::Node {
public:
    Mission();

private:
    // ───── Parameters & State ────────────────────────────
    std::vector<GeoWP>     geo_waypoints_;
    double                  max_velocity_, time_to_max_, takeoff_alt_;
    GeographicLib::LocalCartesian geodetic_proj_;
    bool                    home_set_;
    DroneState              drone_state_;
    std::shared_ptr<MotionProfiling> currTraj_;

    // ───── Publishers & Subscribers ──────────────────────
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_setpoint_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_command_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ───── Callbacks & Helpers ───────────────────────────
    void declareParameters();
    void setupPublishers();
    void setupSubscribers();

    void globalPositionCb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void loadAndConvertWaypoints();
    void mainLoop();

    void publishOffboardControlMode();
    void publishTrajectorySetpoint(double t);
    void sendVehicleCommand(unsigned command, float p1=0, float p2=0, float p3=0,
                            float p4=0, float p5=0, float p6=0, float p7=0);

    void OffboardControl::disarm()
};
