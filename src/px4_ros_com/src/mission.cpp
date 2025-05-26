#include "mission.hpp"

Mission::Mission() : Node("bv_mission"),
    max_velocity_(5.0),
    time_to_max_(1.0),
    takeoff_alt_(10.0),
    geodetic_proj_(0,0,0),
    home_set_(false),
    drone_state_(DroneState::TAKEOFF)
{
  declareParameters();
  setupPublishers();
  setupSubscribers();

  timer_ = this->create_wall_timer(
    100ms, std::bind(&OffboardControl::mainLoop, this));
}

void OffboardControl::declareParameters() {
    this->declare_parameter("mission_waypoints", std::vector<GeoWP>{});
    this->declare_parameter("max_velocity", max_velocity_);
    this->declare_parameter("time_to_max", time_to_max_);
    this->declare_parameter("takeoff_alt", takeoff_alt_);
    this->get_parameter("mission_waypoints", geo_waypoints_);
}

void OffboardControl::setupPublishers() {
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    marker_traj_pub = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
    marker_wp_pub = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 10);
    marker_drone_pub = this->create_publisher<visualization_msgs::msg::Marker>("drone_marker", 10);
    marker_heading_pub = this->create_publisher<visualization_msgs::msg::Marker>("heading_marker", 10);
}

void OffboardControl::setupSubscribers() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    vehicle_local_position_ = this->create_subscription<VehicleOdometry>("fmu/out/vehicle_odometry", qos,
                                                            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

    vehicle_status_ = this->create_subscription<VehicleStatus>("fmu/out/vehicle_status", qos,
                                                            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
}

void OffboardControl::globalPositionCb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
  if (home_set_) return;
  double lat = msg->lat * 1e-7;
  double lon = msg->lon * 1e-7;
  double alt = msg->alt * 1e-3;
  geodetic_proj_.Reset(lat, lon, alt);
  home_set_ = true;
  RCLCPP_INFO(get_logger(), "Home set: %.6f, %.6f, %.2f", lat, lon, alt);
  loadAndConvertWaypoints();
}

void OffboardControl::loadAndConvertWaypoints() {
    std::vector<Eigen::Vector3f> local_wps;
    for (auto &g : geo_waypoints_) {
        double x,y,z;
        geodetic_proj_.Forward(g.lat, g.lon, g.alt, x,y,z);
        local_wps.emplace_back(x, y, -z);
    }
    currTraj = std::make_shared<MotionProfiling>(
        max_velocity_, time_to_max_, &local_wps);
}

void OffboardControl::mainLoop() {
    if (!home_set_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Waiting for home position...");
        return;
    }
    double t = (now() - rclcpp::Time(currTraj->startTime())).seconds();
    publishOffboardControlMode();
    publishTrajectorySetpoint(t);
    // … state machine transitions …
}

void OffboardControl::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode m{};
    m.position = true; m.velocity = true;
    m.timestamp = now().nanoseconds()/1000;
    pub_offboard_->publish(m);
}

void OffboardControl::publishTrajectorySetpoint(double t) {
    auto vel = currTraj->getVelocity(t);
    auto pos = currTraj->getPosition(t, /*&out_yaw*/);
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {-pos.x(), pos.y(), -pos.z()};
    msg.velocity = {-vel.x(), vel.y(), -vel.z()};
    msg.timestamp = now().nanoseconds()/1000;
    pub_setpoint_->publish(msg);
}

void OffboardControl::publish_vehicle_command(unsigned int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
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
    vehicle_command_publisher_->publish(msg);
}

void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
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