#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>
#include <cassert>
#include <iostream>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "rviz_utils.h"
#include <rclcpp/rclcpp.hpp>


class MotionProfiling {
private:
    const std::vector<Eigen::Vector3f> *waypoints;

    float vmax, timeToMaxV = 0;

    float getTimeScaledParameter(float t);
    float calculateLineLength() const;
    float generateTrajectory();
    float lineLength;
    float totalTime;
    float vScale;


public:
    MotionProfiling(double vmax, double timeToMaxV, const std::vector<Eigen::Vector3f> *waypoints){
        this->waypoints = waypoints;
        this->vmax = vmax;
        this->timeToMaxV = timeToMaxV;
        this->lineLength = 0;
        this->totalTime = 0.0;
        this->vScale = 0.0;

        lineLength = generateTrajectory();
    }

    Eigen::Vector3f getPosition(float t, float &heading);
    Eigen::Vector3f getPosition(float t);

    Eigen::Vector3f getVelocity();

    const std::vector<Eigen::Vector3f> getWaypoints();
    float getVMax();
    float getTimeToMaxV();
    float getTotalTime();
    float getvScale();
    void setVMax(float vmax);
    void setTimeToMaxV(float timeToMaxV);
    void sendVisualizeMsg(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker1_pub, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker2_pub);
};