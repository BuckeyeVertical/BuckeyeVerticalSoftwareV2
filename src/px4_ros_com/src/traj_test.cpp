#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <eigen3/Eigen/Dense>
#include "trajectory.h"

#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum State {
	TAKEOFF,
	FOLLOW_TRAJECTORY,
	LOITER,
	LAND
};

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("traj_test")
	{
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		marker_traj_pub = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
		marker_wp_pub = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 10);
		marker_drone_pub = this->create_publisher<visualization_msgs::msg::Marker>("drone_marker", 10);
		marker_heading_pub = this->create_publisher<visualization_msgs::msg::Marker>("heading_marker", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_local_position_ = this->create_subscription<VehicleLocalPosition>("fmu/out/vehicle_local_position", qos,
															std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

        vehicle_status_ = this->create_subscription<VehicleStatus>("fmu/out/vehicle_status", qos,
															std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

		prev_time = this->now();

		waypoints.push_back(Eigen::Vector3f(0.0, 0.0, 10.0));
		waypoints.push_back(Eigen::Vector3f(0.0, 10.0, 10.0));
        waypoints.push_back(Eigen::Vector3f(10.0, 10.0, 10.0));
		waypoints.push_back(Eigen::Vector3f(10.0, 0.0, 10.0));
		// waypoints.push_back(Eigen::Vector3f(0.0, 0.0, 10.0)); FIX THIS EDGE CASE

		currTraj = std::make_shared<Trajectory>(3.0, 2.5, &waypoints);

        offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
            if (reset_time){
                std::cout << "Starting loop" << std::endl;
                reset_time = false;
                start_time = this->now();
            }

			if (offboard_setpoint_counter_ == 10){
				std::cout << "Publishing offboard" << std::endl;
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				rclcpp::sleep_for(1s);

				start_time = this->now();

				this->arm();
			}
            
            if (armed){
                double elapsed_time = (this->now() - start_time).seconds();

				// std::cout << "Publishing trajectory setpoint" << std::endl;

                // offboard_control_mode needs to be paired with trajectory_setpoint
                publish_offboard_control_mode();
                publish_trajectory_setpoint(elapsed_time);
            }

            // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				currTraj->sendVisualizeMsg(marker_traj_pub, marker_wp_pub);
                publish_offboard_control_mode();
                publish_trajectory_setpoint(0);

				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	// Parameters for trajectory smoothing
	float max_velocity = 1.0;  // m/s
	float max_acceleration = 0.5;  // m/s^2
	float smoothing_factor = 0.15;  // Lower value = smoother but slower
    float SIGMOID_STEEPNESS = 1.0;
	State drone_state = TAKEOFF;

	float takeoff_altitude = 10.0;

    bool reset_time = true;
    rclcpp::Time start_time;
    bool armed = false;

	const float tolerance = 0.5;
	std::vector<Eigen::Vector3f> waypoints;
	Eigen::Vector3f target_pos{0.0, 0.0, takeoff_altitude};
	std::shared_ptr<Trajectory> currTraj;

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_traj_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_wp_pub;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_drone_pub;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_heading_pub;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    uint8_t last_arming_state = 255;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float t);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);

	// TODO: TEMPORARY REMOVE!
	float prevDist = 0.0;
	rclcpp::Time prev_time;
};

void OffboardControl::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
    if (msg->arming_state == last_arming_state){
        return;
    }

    // Check arming state
    if (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED) {
        RCLCPP_INFO(this->get_logger(), "Vehicle is ARMED");
        armed = true;
    } else if (msg->arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
        RCLCPP_INFO(this->get_logger(), "Vehicle is DISARMED");
        armed = false;
    }

    // You can also check nav_state
    if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        RCLCPP_INFO(this->get_logger(), "Vehicle is in OFFBOARD mode");
    }

    last_arming_state = msg->arming_state;
}

void OffboardControl::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg){
	if(!msg->xy_valid || !msg->z_valid){
		std::cout << "Message invalid!" << std::endl;
		return;
	}

	float dx = msg->x + target_pos.x();
	float dy = msg->y - target_pos.y();
	float dz = msg->z + target_pos.z();
	float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

	visualization_msgs::msg::Marker drone_marker = rviz_utils::createSquareMarker(Eigen::Vector3f{-msg->x, msg->y, -msg->z}, "/map");
	marker_drone_pub->publish(drone_marker);

	// if (drone_state == FOLLOW_TRAJECTORY){
	// 	// TEMP
	// 	// Calculate velocity by dividing the distance difference by the time difference
	// 	rclcpp::Time current_time = this->now(); // Get the current time
	// 	float dt = (current_time - prev_time).seconds(); // Time difference in seconds

	// 	if (dt > 0) { // To prevent division by zero
	// 		float velocity = (distance - prevDist) / dt;  // Calculate velocity
	// 		std::cout << "Velocity: " << velocity << " m/s" << std::endl;
	// 	} else {
	// 		std::cout << "Time difference is too small. Velocity calculation skipped." << std::endl;
	// 	}

	// 	// Update previous distance and time
	// 	prevDist = distance;
	// 	prev_time = current_time;
	// 	//TEMP
	// }

	if(distance < tolerance){
		//std::cout << "Reached position setpoint with distance " << distance << std::endl;
        reset_time = true;
        switch (drone_state){
			case TAKEOFF:
				target_pos = waypoints.back();
				std::cout << "target position: " << target_pos << std::endl;
				// std::cout << "Completed TAKEOFF... moving into FOLLOW_TRAJECTORY.";
				drone_state = FOLLOW_TRAJECTORY;
				break;
			case FOLLOW_TRAJECTORY:
				std::cout << "Completed FOLLOW_TRAJECTORY... moving into LAND.";
				target_pos = waypoints.back();
				drone_state = LOITER;
				break;
			case LOITER:
				//std::cout << "Loitering" << std::endl;
				break;
			case LAND:
				std::cout << "Completed LAND" << std::endl;
				break;
		}
	}
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float t)
{
	TrajectorySetpoint msg{};
	Eigen::Vector3f pos;

	switch (drone_state){
		case TAKEOFF:
			pos = target_pos;
			msg.yaw = -atan2(currTraj->getSpline().derivatives(0.0, 1)(1), currTraj->getSpline().derivatives(0.0, 1)(0));
			break;
		case FOLLOW_TRAJECTORY:
			pos = currTraj->getPosition(t, msg.yaw);
			break;
		case LOITER:
		case LAND:
			pos = target_pos;
			break;
	}

	std::cout << "Heading: " << msg.yaw << std::endl;

    std::cout << "Pos: " << pos << std::endl;
    
	msg.position = {-pos.x(), pos.y(), -pos.z()};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

	visualization_msgs::msg::Marker heading_marker = rviz_utils::createArrowMarker(Eigen::Vector3f{pos.x(), pos.y(), pos.z()}, msg.yaw, "/map");
	marker_heading_pub->publish(heading_marker);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}