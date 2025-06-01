#include "mission.hpp"

Mission::Mission() : Node("bv_mission"),
    max_velocity(5.0),
    time_to_max(1.0),
    takeoff_alt(10.0),
    geodetic_proj(0,0,0),
    takeoff_offset_set(false),
    home_set(false),
    drone_state(DroneState::IDLE),
    offboard_counter(0),
    tolerance(1.0),
    current_waypoint(0)
{
    declare_parameters();
    setup_publishers();
    setup_subscribers();

    timer = this->create_wall_timer(100ms, std::bind(&Mission::main_loop, this));
}

void Mission::declare_parameters() {
    max_velocity   = this->declare_parameter("max_velocity",   5.0);
    time_to_max    = this->declare_parameter("time_to_max",     1.0);
    takeoff_alt    = this->declare_parameter("takeoff_alt",    10.0);
    raw_waypoints = this->declare_parameter<std::vector<double>>("mission_waypoints", std::vector<double>{});
    tolerance = this->declare_parameter("waypoint_tolerance", 1.0);

    geo_waypoints.clear();
    for (size_t i = 0; i + 2 < raw_waypoints.size(); i += 3) {
    geo_waypoints.push_back( GeoWP{
        raw_waypoints[i + 0],
        raw_waypoints[i + 1],
        raw_waypoints[i + 2]
    });
    }
    if (raw_waypoints.size() % 3 != 0) {
        RCLCPP_WARN(get_logger(), "mission_waypoints size (%zu) not a multiple of 3", raw_waypoints.size());
    }

    print_parameters();
}

void Mission::print_parameters() {
    std::stringstream ss;
    ss << "\n=== Loaded Parameters ===\n"
        << "  max_velocity:      " << max_velocity   << "\n"
        << "  time_to_max:       " << time_to_max    << "\n"
        << "  takeoff_alt:       " << takeoff_alt    << "\n"
        << "  waypoint_tolerance:" << tolerance       << "\n"
        << "  mission_waypoints:\n";
    for (size_t i = 0; i < geo_waypoints.size(); ++i) {
        const auto &wp = geo_waypoints[i];
        ss << "    [" << i << "] lat: " << wp.lat
        << ", lon: " << wp.lon
        << ", alt: " << wp.alt << "\n";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void Mission::setup_publishers() {
    pub_offboard = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    pub_setpoint = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    pub_command = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    pub_traj_marker = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
    pub_wp_marker = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 10);
    pub_drone_marker = this->create_publisher<visualization_msgs::msg::Marker>("drone_marker", 10);
    pub_heading_marker = this->create_publisher<visualization_msgs::msg::Marker>("heading_marker", 10);
}

void Mission::setup_subscribers() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub_local = this->create_subscription<VehicleOdometry>("fmu/out/vehicle_odometry", qos,
                                                            std::bind(&Mission::vehicle_local_position_callback, this, std::placeholders::_1));

    sub_global = this->create_subscription<VehicleGlobalPosition>("fmu/out/vehicle_global_position", qos,
                                                            std::bind(&Mission::vehicle_global_position_callback, this, std::placeholders::_1));

    sub_status = this->create_subscription<VehicleStatus>("fmu/out/vehicle_status_v1", qos,
                                                            std::bind(&Mission::vehicle_status_callback, this, std::placeholders::_1));
}

void Mission::vehicle_local_position_callback(const VehicleOdometry::SharedPtr msg) {
    if (local_wps.size() == 0) return;

    float x = msg->position[0];
    float y = msg->position[1];
    float z = msg->position[2];

    if (!takeoff_offset_set) {
        takeoff_offset = Vector3f(x, y, z);
        segment_waypoints.push_back(Eigen::Vector3f(takeoff_offset.x(), takeoff_offset.y(), takeoff_alt));
        segment_waypoints.push_back(local_wps[current_waypoint]);
        target_pos = local_wps[current_waypoint];
        takeoff_offset_set = true;
    }

    publish_drone_marker(Vector3f{-x, y, -z});

    float dx = x + target_pos.x();
    float dy = y  - target_pos.y();
    float dz = z  + target_pos.z();
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);


    if(distance < tolerance){
        std::cout << "Reached position setpoint with l2 error " << distance << std::endl;
        rclcpp::sleep_for(1s);

        segment_waypoints.clear();
        segment_waypoints.push_back(local_wps[current_waypoint]);
        segment_waypoints.push_back(local_wps[current_waypoint + 1]);
        curr_traj = std::make_shared<MotionProfiling>(max_velocity, time_to_max, &segment_waypoints, now());
        target_pos = local_wps[current_waypoint + 1];
        current_waypoint++;
    }
}

void Mission::publish_drone_marker(Vector3f pose) {
    visualization_msgs::msg::Marker drone_marker = rviz_utils::createSquareMarker(pose, "/map");
    pub_drone_marker->publish(drone_marker);
}

void Mission::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
    if (msg->nav_state != this->nav_status){
        RCLCPP_INFO(this->get_logger(), "NAV STATUS: %d", msg->nav_state);
    }

    if (msg->arming_state != this->arm_status) {
        RCLCPP_INFO(this->get_logger(), "ARM STATUS: %d", msg->arming_state);
    }

    if (msg->failsafe != this->failsafe) {
        RCLCPP_INFO(this->get_logger(), "FAILSAFE STATUS: %d", msg->failsafe);
    }

    if (msg->pre_flight_checks_pass != this->flight_check) {
        RCLCPP_INFO(this->get_logger(), "PREFLIGHT STATUS: %d", msg->pre_flight_checks_pass);
    }

    this->nav_status = msg->nav_state;
    this->arm_status = msg->arming_state;
    this->failsafe = msg->failsafe;
    this->flight_check = msg->pre_flight_checks_pass;
}

void Mission::vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
  if (home_set) return;
  double lat = msg->lat;
  double lon = msg->lon;
  double alt = msg->alt;
  home_gps = Vector3f(lat, lon, alt);
  geodetic_proj.Reset(lat, lon, alt);
  home_set = true;
  RCLCPP_INFO(get_logger(), "Home set: %.6f, %.6f, %.2f", lat, lon, alt);
  convert_waypoints();
}

void Mission::convert_waypoints() {
    // build the local waypoints
    local_wps.clear();
    for (auto &g : geo_waypoints) {
        double x, y, z;
        geodetic_proj.Forward(g.lat, g.lon, g.alt, x, y, z);
        local_wps.emplace_back(
            x + takeoff_offset.x(),
            y + takeoff_offset.y(),
            z + takeoff_offset.z()
        );
    }

    // print them out
    std::stringstream ss;
    ss << "\n=== Local Waypoints ===\n";
    for (size_t i = 0; i < local_wps.size(); ++i) {
        const auto &wp = local_wps[i];
        ss << "  [" << i << "] x: " << wp.x()
           << ", y: " << wp.y()
           << ", z: " << wp.z() << "\n";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}


void Mission::main_loop() {
    if (!home_set && !takeoff_offset_set) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Waiting for home position... %d %d", home_set, takeoff_offset_set);
        return;
    }

    switch (drone_state) {
        case DroneState::IDLE: {
            if (flight_check){
                RCLCPP_INFO(this->get_logger(), "Arming");
                drone_state = DroneState::ARM;
            }
            break;
        }
        case DroneState::ARM: {
            if (!flight_check) {
                drone_state = DroneState::IDLE;
                RCLCPP_INFO(this->get_logger(), "Flight check failed... returning back to idle.");
            } else if (arm_status == VehicleStatus::ARMING_STATE_ARMED){
                drone_state = DroneState::TAKEOFF;
                RCLCPP_INFO(this->get_logger(), "Armed -> Taking Off");
            }
            arm();
            break;
        }
        case DroneState::TAKEOFF: {
            if (!flight_check) {
                drone_state = DroneState::IDLE;
            } else if (nav_status == VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF) {
                drone_state = DroneState::LOITER;
                RCLCPP_INFO(this->get_logger(), "Takeoff -> Loiter");
            }
            arm();
            takeoff();
            break;
        }
        case DroneState::LOITER: {
            if (!flight_check) {
                drone_state = DroneState::IDLE;
            } else if (nav_status == VehicleStatus::NAVIGATION_STATE_AUTO_LOITER){
                drone_state = DroneState::OFFBOARD;
                RCLCPP_INFO(this->get_logger(), "Loitering -> Offboard");
            }
            rclcpp::sleep_for(1s);
            arm();
            break;
        }
        case DroneState::OFFBOARD: {
            if (!flight_check || arm_status != VehicleStatus::ARMING_STATE_ARMED || failsafe){
                drone_state = DroneState::IDLE;
                RCLCPP_INFO(this->get_logger(), "OFFBOARD");
            }

            if (offboard_counter == 10) {
                curr_traj = std::make_shared<MotionProfiling>(max_velocity, time_to_max, &segment_waypoints, now());
                offboard();
            }

            double t = 0.0;

            if (offboard_counter < 11) {
                offboard_counter++;
            } else {
                t = (now() - rclcpp::Time(curr_traj->getStartTime())).seconds();
            }
            
            publish_offboard_control_mode();

            if (curr_traj != NULL && segment_waypoints.size() >= 2) {
                publish_trajectory_setpoint(t);
                curr_traj->sendVisualizeMsg(pub_traj_marker, pub_wp_marker);
            }
            
            break;
        }
        case DroneState::LAND: {
            // TODO: Implement this
            RCLCPP_INFO(this->get_logger(), "LANDING");
            break;
        }
    }
}

void Mission::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode m{};
    m.position = true; m.velocity = true;
    m.timestamp = now().nanoseconds()/1000;
    pub_offboard->publish(m);
}

void Mission::publish_trajectory_setpoint(double t) {
    auto vel = curr_traj->getVelocity(t);
    px4_msgs::msg::TrajectorySetpoint msg{};
    auto pos = curr_traj->getPosition(t, msg.yaw);
    msg.position = {-pos.x(), pos.y(), -pos.z()};
    msg.velocity = {-vel.x(), vel.y(), -vel.z()};
    msg.timestamp = now().nanoseconds()/1000;
    pub_setpoint->publish(msg);

    visualization_msgs::msg::Marker heading_marker = rviz_utils::createArrowMarker(Eigen::Vector3f{pos.x(), pos.y(), pos.z()}, msg.yaw, "/map");
    pub_heading_marker->publish(heading_marker);
}

void Mission::publish_vehicle_command(unsigned int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub_command->publish(msg);
}

void Mission::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void Mission::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void Mission::offboard()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    RCLCPP_INFO(this->get_logger(), "Offboard command send");
}

void Mission::takeoff()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 
        1.0,
        0.0,
        0.0,
        NAN,
        home_gps.x(),
        home_gps.y(),
        10.0
    );
    RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mission>());

    rclcpp::shutdown();
    return 0;
}