#include "trajectory.h"

/**
 * @brief Calculate normalized position along a trajectory with acceleration and deceleration phases
 * @param vmax Maximum velocity
 * @param a Acceleration
 * @param t Current time
 * @param totalDist Total distance to travel
 * @return Normalized position (0 to 1) along the trajectory
 */
float Trajectory::getTimeScaledParameter(float t) const {
    // Handle edge cases
    if (vmax <= 0.0 || timeToMaxV <= 0.0 || splineLength <= 0.0) {
        std::cout << "Returning 0" << std::endl;
        return 0.0;
    }

    float a = vmax / timeToMaxV;
    float x;

    // Acceleration phase
    if (t >= 0.0 && t <= timeToMaxV) {
        // std::cout << "Acceleration phase!" << std::endl;
        x = 0.5 * a * t * t;
    }
    // Constant velocity phase
    else if (t > timeToMaxV && t <= (totalTime - timeToMaxV)) {
        // std::cout << "Constant velocity phase!" << std::endl;
        x = 0.5 * a * timeToMaxV * timeToMaxV + vmax * (t - timeToMaxV);
    }
    // Deceleration phase
    else if (t > (totalTime - timeToMaxV) && t <= totalTime) {
        // std::cout << "Deceleration phase!" << std::endl;
        x = splineLength - 0.5 * a * (totalTime - t) * (totalTime - t);
    }
    else {
        std::cout << "Returning 1" << std::endl;
        return 1.0;  // Beyond total time
    }

    return x / splineLength;
}

// Only call this function once. Loops through all points and sends then to visualizer.
void Trajectory::sendVisualizeMsg(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker1_pub, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker2_pub){
    // Sample points along the trajectory
    std::vector<Eigen::Vector3f> trajectory_points;
    const int samples = 100;
    float total_time = totalTime;
    for (int i = 0; i <= samples; ++i) {
        float t = static_cast<float>(i) * total_time / samples;
        trajectory_points.push_back(getPosition(t));
    }

    // Create markers for visualization
    visualization_msgs::msg::Marker marker1 = rviz_utils::createLineMarker(trajectory_points, "/map");
    visualization_msgs::msg::Marker marker2 = rviz_utils::createPointMarker(*waypoints, "/map");
    
    marker1_pub->publish(marker1); 
    marker2_pub->publish(marker2); 
}

float Trajectory::getTotalTime(){
    return totalTime;
}

float Trajectory::generateTrajectory(){
    if (waypoints->size() < 2) {
        std::cerr << "Less than 2 waypoints. Returning 0." << std::endl;
        return 0.0f;
    }

    // Build the spline knots matrix
    // Each row represents [x, y, z] for a waypoint
    Eigen::MatrixXf points(3, waypoints->size());
    for(size_t i = 0; i < waypoints->size(); i++) {
        points(0, i) = waypoints->at(i).x();
        points(1, i) = waypoints->at(i).y();
        points(2, i) = waypoints->at(i).z();
    }

    // The degree of the interpolating spline should be one less than
    // the number of points for interpolation
    spline = Eigen::SplineFitting<Spline3f>::Interpolate(points, std::min<int>(waypoints->size() - 1, 3));
    
    splineInitialized = true;
    float length = calculateSplineLength();
    totalTime = length / vmax + timeToMaxV;

    float a = vmax / timeToMaxV;

    // If profile is too short
    if (vmax * timeToMaxV > length) {
        timeToMaxV = sqrt(length / a);
        totalTime = 2.0 * timeToMaxV;
        vmax = a * timeToMaxV;

        std::cout << "Profile too short. Updating values..." << std::endl;
        std::cout << "timeToMaxV = " << timeToMaxV << ", totalTime = " << totalTime << ", vmax = " << vmax << std::endl;
    }

    std::cout << "Generated trajectory with length " << length << " and total time " << totalTime << " seconds." << std::endl;
    
    return length;
}

float Trajectory::calculateSplineLength() const {
    assert(splineInitialized && "Spline not initialized!");

    // Approximate length by sampling points
    const int samples = 1000;
    float length = 0.0f;
    Eigen::Vector3f prevPoint = spline(0.0f);
    
    for (int i = 1; i <= samples; ++i) {
        float t = static_cast<float>(i) / samples;
        Eigen::Vector3f currentPoint = spline(t);
        length += (currentPoint - prevPoint).norm();
        prevPoint = currentPoint;
    }
    
    return length;
}

Spline3f Trajectory::getSpline(){
    return spline;
}

Eigen::Vector3f Trajectory::getPosition(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_heading_pub, float t, float &heading) {
    assert(splineInitialized && "Spline not initialized!");

    // Get time-scaled parameter
    float u = getTimeScaledParameter(t);

    float dx = spline.derivatives(u, 1)(0);
    float dy = spline.derivatives(u, 1)(1);

    heading = -atan2(dy, dx);
    
    Eigen::Vector3f pos = spline(u);

    visualization_msgs::msg::Marker dMarker = rviz_utils::createArrowMarker(pos, dx, dy, "/map");

    marker_heading_pub->publish(dMarker);

    // Get position from spline
    return pos;
}

Eigen::Vector3f Trajectory::getPosition(float t) {
    assert(splineInitialized && "Spline not initialized!");

    // Get time-scaled parameter
    float u = getTimeScaledParameter(t);
    
    // Get position from spline
    return spline(u);
}

float Trajectory::getVMax(){
    return vmax;
}

float Trajectory::getTimeToMaxV(){
    return timeToMaxV;
}

void Trajectory::setVMax(float vmax){
    this->vmax = vmax;
}

void Trajectory::setTimeToMaxV(float a){
    this->timeToMaxV = a;
}