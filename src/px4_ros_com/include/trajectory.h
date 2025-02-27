#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>
#include <cassert>
#include <iostream>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "rviz_utils.h"
#include <rclcpp/rclcpp.hpp>

typedef Eigen::Spline<float, 3> Spline3f;

class Trajectory {
private:
    const std::vector<Eigen::Vector3f> *waypoints;

    float vmax, timeToMaxV = 0;

    float getTimeScaledParameter(float t) const;
    float calculateSplineLength() const;
    float generateTrajectory();
    Spline3f spline;
    bool splineInitialized;
    float splineLength;
    float totalTime;
    

public:
    Trajectory(double vmax, double timeToMaxV, const std::vector<Eigen::Vector3f> *waypoints){
        this->waypoints = waypoints;
        this->vmax = vmax;
        this->timeToMaxV = timeToMaxV;
        this->splineInitialized = false;
        this->splineLength = 0;
        this->totalTime = 0.0;

        splineLength = generateTrajectory();
    }

    Eigen::Vector3f getPosition(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_heading_pub, float t, float &heading);
    Eigen::Vector3f getPosition(float t);

    Spline3f getSpline();
    float getVMax();
    float getTimeToMaxV();
    float getTotalTime();
    void setVMax(float vmax);
    void setTimeToMaxV(float timeToMaxV);
    void sendVisualizeMsg(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker1_pub, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker2_pub);
};