#include "motionProfiling.h"

/**
 * @brief Calculate normalized position along a trajectory with acceleration and deceleration phases
 * @param vmax Maximum velocity
 * @param a Acceleration
 * @param t Current time
 * @param totalDist Total distance to travel
 * @return Normalized position (0 to 1) along the trajectory
 */
float MotionProfiling::getTimeScaledParameter(float t){
    // Handle edge cases
    if (vmax <= 0.0 || timeToMaxV <= 0.0 || lineLength <= 0.0) {
        std::cout << "Returning 0" << std::endl;
        return 0.0;
    }

    float a = vmax / timeToMaxV;
    float x = 0.0;

    // Acceleration phase
    if (t >= 0.0 && t <= timeToMaxV) {

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
        x = lineLength - 0.5 * a * (totalTime - t) * (totalTime - t);
    }
    else {
        std::cout << "Returning 1" << std::endl;
        return 1.0;  // Beyond total time
    }

    return lineLength > 0 ? (x / lineLength) : 0.0;
}


float MotionProfiling::getvScale(float t){

    // Handle edge cases
    if (vmax <= 0.0 || timeToMaxV <= 0.0 || lineLength <= 0.0) {
        std::cout << "Returning 0" << std::endl;
        return 0.0;
    }

    float a = vmax / timeToMaxV;
    float vScale = 0;

    // Acceleration phase
    if (t >= 0.0 && t <= timeToMaxV) {

        vScale = a * t;
        
    }
    // Constant velocity phase
    else if (t > timeToMaxV && t <= (totalTime - timeToMaxV)) {

        vScale = vmax;
    }
    // Deceleration phase
    else if (t > (totalTime - timeToMaxV) && t <= totalTime) {

        vScale = a * (totalTime - t);

    }
    else {
        std::cout << "Returning 1" << std::endl;
        return 1.0;  // Beyond total time
    }

    vScale = vScale / vmax;

    std::cout << "VScale in timeScaledFunction: " << vScale << std::endl;

    return vScale;
}

// TODO: Add time as param
Eigen::Vector3f MotionProfiling::getVelocity(float t){

    //Eigen::Vector3f diff = (waypoints->at(1) - waypoints->at(0));
    // std::cout << "Waypoint difference: " << diff.x() << diff.y() << diff.z() << std::endl;
    // std::cout << "Line length: " << calculateLineLength() << std::endl;
    //Eigen::Vector3f afterLineLegnth = (waypoints->at(1) - waypoints->at(0)) / 10;
    // std::cout << "After division " << afterLineLegnth.x() << afterLineLegnth.y() << afterLineLegnth.z() << std::endl;
    // std::cout << "vScale in getVelocity() " << vScale << std::endl;
    // std::cout << "vmax " << vmax << std::endl;
    //Eigen::Vector3f retVector = (((waypoints->at(1) - waypoints->at(0))/calculateLineLength()) * vScale) * vmax;
    // std::cout << "Final return difference: " << retVector.x() << retVector.y() << retVector.z() << std::endl;

    

    return (((waypoints->at(1) - waypoints->at(0))/calculateLineLength()) * getvScale(t)) * vmax;

}


// Only call this function once. Loops through all points and sends then to visualizer.
void MotionProfiling::sendVisualizeMsg(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker1_pub, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker2_pub){
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

float MotionProfiling::getTotalTime(){
    return totalTime;
}

float MotionProfiling::generateTrajectory(){
    if (waypoints->size() < 2) {
        std::cerr << "Less than 2 waypoints. Returning 0." << std::endl;
        return 0.0f;
    }

    float a = vmax / timeToMaxV;

    float trajectoryLength = calculateLineLength();
    totalTime = trajectoryLength / vmax + timeToMaxV;

    // Acceleration
    

    // If profile is too short
    if (vmax * timeToMaxV > trajectoryLength) {
        timeToMaxV = sqrt(trajectoryLength / a);
        totalTime = 2.0 * timeToMaxV;
        vmax = a * timeToMaxV;

        std::cout << "Profile too short. Updating values..." << std::endl;
        std::cout << "timeToMaxV = " << timeToMaxV << ", totalTime = " << totalTime << ", vmax = " << vmax << std::endl;
    }

    //std::cout << "Generated trajectory with length " << trajectoryLength << " and total time " << totalTime << " seconds." << std::endl;
   
    return trajectoryLength;
}

float MotionProfiling::calculateLineLength() const {
   
    // Starting waypoint of line
    Eigen::Vector3f startPoint = waypoints->at(0);
    // Ending waypoint of line
    Eigen::Vector3f endPoint = waypoints->at(1);

    // Calculates the 3D distance between start and end points
    return sqrt(pow(startPoint.x() - endPoint.x(), 2) + pow(startPoint.y() - endPoint.y(), 2) + pow(startPoint.z() - endPoint.z(), 2));
}

Eigen::Vector3f MotionProfiling::getPosition(float t) {

    // Get time-scaled parameter
    float u = getTimeScaledParameter(t);

    // Starting waypoint of line
    Eigen::Vector3f startPoint = waypoints->at(0);
    // Ending waypoint of line
    Eigen::Vector3f endPoint = waypoints->at(1);
   
    // Get position
    return startPoint + u * (endPoint - startPoint);
}

Eigen::Vector3f MotionProfiling::getPosition(float t, float &heading) {

    // Get time-scaled parameter
    float u = getTimeScaledParameter(t);
    

    // Starting waypoint of line
    Eigen::Vector3f startPoint = waypoints->at(0);
    // Ending waypoint of line
    Eigen::Vector3f endPoint = waypoints->at(1);

    heading = -atan2(endPoint.y() - startPoint.y(), endPoint.x() - startPoint.x());
   
    // Get position
    return startPoint + u * (endPoint - startPoint);
}

const std::vector<Eigen::Vector3f> MotionProfiling::getWaypoints(){
    return *waypoints;
}


float MotionProfiling::getVMax(){
    return vmax;
}

float MotionProfiling::getTimeToMaxV(){
    return timeToMaxV;
}

void MotionProfiling::setVMax(float vmax){
    this->vmax = vmax;
}

void MotionProfiling::setTimeToMaxV(float a){
    this->timeToMaxV = a;
}
