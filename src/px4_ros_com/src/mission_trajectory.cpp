#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <cmath>
#include <memory>
#include <vector>
#include <atomic>

// PX4 msgs
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

// RViz markers
#include <visualization_msgs/msg/marker.hpp>

// std_msgs for the scan trigger
#include <std_msgs/msg/empty.hpp>

// ** Replace this with actual detection-msg path **
#include <your_detection_msgs/msg/detections.hpp>

#include <eigen3/Eigen/Dense>
#include "motionProfiling.h"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum State {
    TAKEOFF,
    FIELD_LAP,      // fly the perimeter
    WAIT_DETECTIONS,// waiting for detection callback
    GOTO_DROP,      // fly to next drop waypoint
    DROP,           // drop payload
    LAND,
    COMPLETE
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl()
    : Node("mission_trajectory"),
      drone_state_(TAKEOFF),
      field_wp_idx_(0),
      current_drop_idx_(0),
      reset_time_(true),
      armed_(false),
      last_arming_state_(255)
    {
        // Publishers
        offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_   = create_publisher<TrajectorySetpoint>    ("/fmu/in/trajectory_setpoint",    10);
        vehicle_command_pub_       = create_publisher<VehicleCommand>        ("/fmu/in/vehicle_command",        10);
        marker_traj_pub_           = create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
        marker_wp_pub_             = create_publisher<visualization_msgs::msg::Marker>("waypoint_marker",   10);
        marker_drone_pub_          = create_publisher<visualization_msgs::msg::Marker>("drone_marker",      10);
        marker_heading_pub_        = create_publisher<visualization_msgs::msg::Marker>("heading_marker",    10);

        // Scanning trigger
        scanning_pub_ = create_publisher<std_msgs::msg::Empty>("Scanning", 1);
        // Detection results
        detection_sub_ = create_subscription<your_detection_msgs::msg::Detections>(
            "detections", 10,
            std::bind(&OffboardControl::detections_callback, this, std::placeholders::_1));

        // PX4 telemetry
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
        vehicle_local_position_sub_ = create_subscription<VehicleLocalPosition>(
            "fmu/out/vehicle_local_position", qos,
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));
        vehicle_status_sub_ = create_subscription<VehicleStatus>(
            "fmu/out/vehicle_status", qos,
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

        prev_time_ = now();

        // Define the field perimeter waypoints (EX: 10m x 10m Box)
        field_waypoints_ = {
            {  0.f,  0.f, takeoff_alt_},
            { 10.f,  0.f, takeoff_alt_},
            { 10.f, 10.f, takeoff_alt_},
            {  0.f, 10.f, takeoff_alt_},
            {  0.f,  0.f, takeoff_alt_}  // close the loop/Repeat starting waypoint
        };

        // Start with takeoff up to first field_waypoint (EX: (0,0) of Box)
        target_pos_ = field_waypoints_[0];
        segment_waypoints_ = {Eigen::Vector3f{0,0,0}, target_pos_};
        curr_traj_ = std::make_shared<MotionProfiling>(max_vel_, time_to_max_, &segment_waypoints_);

        // Timer for setpoints and visualization
        timer_ = create_wall_timer(100ms, [this]() {
            if (reset_time_) {
                start_time_ = now();
                reset_time_ = false;
            }
            double t = (now() - start_time_).seconds();
            curr_traj_->sendVisualizeMsg(marker_traj_pub_, marker_wp_pub_);
            publish_offboard_control_mode();
            publish_trajectory_setpoint(t);
        });
    }

private:
    // ROS interfaces
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_traj_pub_, marker_wp_pub_, marker_drone_pub_, marker_heading_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr scanning_pub_;
    rclcpp::Subscription<your_detection_msgs::msg::Detections>::SharedPtr detection_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Finite State machine
    State drone_state_;
    std::vector<Eigen::Vector3f> field_waypoints_;
    size_t field_wp_idx_;

    std::vector<Eigen::Vector3f> pending_drops_;
    std::vector<std::string> pending_labels_;
    size_t current_drop_idx_;

    // Trajectory & timing
    std::shared_ptr<MotionProfiling> curr_traj_;
    std::vector<Eigen::Vector3f> segment_waypoints_;
    Eigen::Vector3f target_pos_;
    Eigen::Vector3f current_pos_;
    bool reset_time_;
    rclcpp::Time start_time_;
    rclcpp::Time prev_time_;

    // Drone / PX4 state and Parameters
    bool armed_;
    uint8_t last_arming_state_;
    static constexpr float takeoff_alt_ = 10.0f;
    float max_vel_     = 3.0f;    // m/s
    float time_to_max_ = 2.0f;    // s
    const float tolerance = 0.5f; // m

    // Callbacks Funtions
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg) {
        if (msg->arming_state != last_arming_state_) {
            if (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED) {
                RCLCPP_INFO(get_logger(), "ARMED");
                armed_ = true;
            } else {
                RCLCPP_INFO(get_logger(), "DISARMED");
                armed_ = false;
            }
            if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                RCLCPP_INFO(get_logger(), "OFFBOARD");
            }
            last_arming_state_ = msg->arming_state;
        }
    }

    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg) {
        if (!msg->xy_valid || !msg->z_valid) return;

        // update current_pos_ (in real/world frame)
        current_pos_ = { -msg->x, msg->y, -msg->z };
        marker_drone_pub_->publish(
            rviz_utils::createSquareMarker(current_pos_, "/map"));

        float dx = current_pos_.x() - target_pos_.x();
        float dy = current_pos_.y() - target_pos_.y();
        float dz = current_pos_.z() - target_pos_.z();
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < tolerance) {
            // reached current target -> advance state
            reset_time_ = true;
            switch (drone_state_) {
                case TAKEOFF:
                    RCLCPP_INFO(get_logger(), "Takeoff done: start FIELD_LAP");
                    process_field_lap_step();
                    drone_state_ = FIELD_LAP;
                    break;

                case FIELD_LAP:
                    process_field_lap_step();
                    break;

                case GOTO_DROP:
                    RCLCPP_INFO(get_logger(), "Reached drop waypoint: DROP");
                    drone_state_ = DROP;
                    break;

                case DROP:
                    execute_drop();
                    break;

                default:
                    break;
            }
        }
    }

    // Detection callback function that needs to be fixed
    void detections_callback(const your_detection_msgs::msg::Detections::SharedPtr msg) {
        pending_drops_.clear();
        pending_labels_.clear();
        for (auto &d : msg->detections) {
            pending_drops_.push_back({ d.x, d.y, d.z });
            pending_labels_.push_back(d.label);
        }
        if (pending_drops_.empty()) {
            RCLCPP_WARN(get_logger(), "No detections: re-lap");
            field_wp_idx_ = 0;
            process_field_lap_step();
            drone_state_ = FIELD_LAP;
        } else {
            RCLCPP_INFO(get_logger(), "Got %zu drop-points", pending_drops_.size());
            current_drop_idx_ = 0;
            setup_flight_to(pending_drops_[0]);
            drone_state_ = GOTO_DROP;
        }
        reset_time_ = true;
    }


    // State helper functions


    void process_field_lap_step() {
        // if we've finished all perimeter points, trigger scan
        if (field_wp_idx_ >= field_waypoints_.size()) {
            RCLCPP_INFO(get_logger(), "Field lap done: trigger SCAN");
            scanning_pub_->publish(std_msgs::msg::Empty{});
            drone_state_ = WAIT_DETECTIONS;
            return;
        }
        // fly to next perimeter corner
        setup_flight_to(field_waypoints_[field_wp_idx_++]);
    }

    void setup_flight_to(const Eigen::Vector3f &pt) {
        segment_waypoints_.clear();
        segment_waypoints_.push_back(current_pos_);
        segment_waypoints_.push_back(pt);
        curr_traj_ = std::make_shared<MotionProfiling>(max_vel_, time_to_max_, &segment_waypoints_);
        target_pos_ = pt;
    }

    void execute_drop() {
        RCLCPP_INFO(get_logger(), "Dropping payload at \"%s\"",
                    pending_labels_[current_drop_idx_].c_str());

        // TODO: send servo / actuator command here
        // e.g. publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_SERVO, servo_ch, drop_pw);

        // move to next drop or finish
        if (++current_drop_idx_ < pending_drops_.size()) {
            setup_flight_to(pending_drops_[current_drop_idx_]);
            drone_state_ = GOTO_DROP;
        } else {
            RCLCPP_INFO(get_logger(), "All payloads dropped → LAND");
            drone_state_ = LAND;
            setup_flight_to({0,0,0});  // land at home
        }
    }

    // Publishers
    void publish_offboard_control_mode() {
        OffboardControlMode m{};
        m.position     = true;
        m.velocity     = true;
        m.acceleration = false;
        m.attitude     = false;
        m.body_rate    = false;
        m.timestamp    = get_clock()->now().nanoseconds() / 1'000;
        offboard_control_mode_pub_->publish(m);
    }

    void publish_trajectory_setpoint(double t) {
        TrajectorySetpoint m{};
        Eigen::Vector3f vel = curr_traj_->getVelocity(t);
        float yaw;

        switch (drone_state_) {
        case TAKEOFF:
        case FIELD_LAP:
        case GOTO_DROP:
            // for these, we actually follow the trajectory
            {
                Eigen::Vector3f pos = curr_traj_->getPosition(t, yaw);
                m.position = { -pos.x(), pos.y(), -pos.z() };
                m.velocity = { -vel.x(), vel.y(), -vel.z() };
                m.yaw      = yaw;
            }
            break;

        case DROP:
        case LAND:
        case COMPLETE:
            {
                // just hold position
                m.position = { -target_pos_.x(), target_pos_.y(), -target_pos_.z() };
                m.velocity = { 0,0,0 };
                m.yaw      = 0;
            }
            break;
        }

        m.timestamp = get_clock()->now().nanoseconds() / 1'000;
        trajectory_setpoint_pub_->publish(m);

        // heading arrow
        marker_heading_pub_->publish(
            rviz_utils::createArrowMarker(target_pos_, m.yaw, "/map"));
    }

    void publish_vehicle_command(uint16_t cmd, float p1=0, float p2=0) {
        VehicleCommand m{};
        m.command          = cmd;
        m.param1           = p1;
        m.param2           = p2;
        m.target_system    = 1;
        m.target_component = 1;
        m.source_system    = 1;
        m.source_component = 1;
        m.from_external    = true;
        m.timestamp        = get_clock()->now().nanoseconds()/1'000;
        vehicle_command_pub_->publish(m);
    }

    void arm()    { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f); }
    void disarm() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f); }

}; // class OffboardControl

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();
    RCLCPP_INFO(node->get_logger(), "Starting offboard control node...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
