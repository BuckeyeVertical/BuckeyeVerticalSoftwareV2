#include <vector>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "rviz_utils.h"


class MotionProfiling {
private:
    const std::vector<Eigen::Vector3f> *waypoints;

    float vmax, timeToMaxV = 0;

    float getTimeScaledParameter(float t) const;
    float calculateLineLength() const;
    float generateTrajectory();
    float lineLength;
    float totalTime;

public:
    MotionProfiling(double vmax, double timeToMaxV, const std::vector<Eigen::Vector3f> *waypoints){
        this->waypoints = waypoints;
        this->vmax = vmax;
        this->timeToMaxV = timeToMaxV;
        this->lineLength = 0;
        this->totalTime = 0.0;

        lineLength = generateTrajectory();
    }

    Eigen::Vector3f getPosition(float t, float &heading);
    Eigen::Vector3f getPosition(float t);

    const std::vector<Eigen::Vector3f> getWaypoints();
    float getVMax();
    float getTimeToMaxV();
    float getTotalTime();
    void setVMax(float vmax);
    void setTimeToMaxV(float timeToMaxV);
    void sendVisualizeMsg(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker1_pub, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker2_pub);
};