#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <queue>
#include <cmath>
#include <algorithm>

// ROS2 includes.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

// OpenCV includes.
#include "opencv2/opencv.hpp"

// ------------------------------
// Hyperparameters (Tuning Parameters)
// ------------------------------
namespace hyper {
  // Detection and accumulation parameters.
  const int MIN_CONTOUR_AREA = 500;             // Minimum area (in pixels) for a contour to be considered.
  const int ACCUMULATION_THRESHOLD = 10;        // Number of detections to accumulate before processing.

  // DBSCAN clustering parameters.
  const double DBSCAN_EPS = 0.0001;             // Maximum distance (in geo-units) to consider points as neighbors.
  const int DBSCAN_MIN_POINTS = 3;              // Minimum points to form a cluster.

  // Intrinsic parameters (from the provided intrinsic matrix).
  // K = [ [fx,   0, cx],
  //       [  0, fy, cy],
  //       [  0,  0,  1] ]
  const double fx = 9.31696166e+03;
  const double fy = 1.02161568e+04;
  const double cx = 9.42461549e+02;
  const double cy = 9.53746283e+02;

  // Camera and geographic parameters.
  const double CAMERA_HEIGHT = 10.0;            // Camera height above ground in meters.
  const double GEO_ORIGIN_LAT = 40.0;           // Latitude corresponding to the camera's origin.
  const double GEO_ORIGIN_LON = -74.0;          // Longitude corresponding to the camera's origin.

  // Conversion factors (approximate, valid for small distances).
  // One degree of latitude is roughly 111111 meters.
  const double METERS_PER_DEGREE_LAT = 111111.0;
  // One degree of longitude in meters, depends on latitude.
  const double METERS_PER_DEGREE_LON = 111111.0 * std::cos(GEO_ORIGIN_LAT * M_PI / 180.0);
}

// ------------------------------
// Helper Functions
// ------------------------------

// Compute the centroid of a bounding box.
cv::Point2f computeCentroid(const cv::Rect &bbox) {
  return cv::Point2f(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
}

// Convert a pixel coordinate (detection centroid) into geographic coordinates (longitude, latitude)
// using the intrinsic matrix, an assumed camera height, and a flat-ground model.
// The conversion process is as follows:
//   1. Convert the pixel coordinate (u,v) to a normalized camera coordinate:
//         x_norm = (u - cx) / fx,   y_norm = (v - cy) / fy
//   2. Assuming the object is on the ground (Z = 0) and the camera is mounted at a known height,
//      scale the normalized coordinates with the camera height to get an offset (in meters).
//   3. Convert the meter offsets into latitude and longitude offsets using approximate conversion factors.
std::pair<double, double> convertToGeoCoordinates(const cv::Point2f &point) {
  // Extract pixel coordinates.
  double u = static_cast<double>(point.x);
  double v = static_cast<double>(point.y);

  // Convert pixel coordinates to normalized camera coordinates.
  double x_norm = (u - hyper::cx) / hyper::fx;
  double y_norm = (v - hyper::cy) / hyper::fy;

  // Back-project to the ground plane (assuming camera is pointed downward).
  // Here, we assume that the detection lies on the ground (Z = 0). Given the camera height,
  // the ground offset in meters is approximated as:
  double X_m = hyper::CAMERA_HEIGHT * x_norm;   // Offset in meters along east-west.
  double Y_m = hyper::CAMERA_HEIGHT * y_norm;     // Offset in meters along north-south.

  // Convert meter offsets into geo-coordinate offsets.
  // Note: A positive X_m increases longitude; a positive Y_m increases latitude (if north is up).
  // Adjust the sign of Y_m if your camera coordinate system differs.
  double delta_lon = X_m / hyper::METERS_PER_DEGREE_LON;
  double delta_lat = Y_m / hyper::METERS_PER_DEGREE_LAT;

  // For this example, we assume that moving downward in the image (increasing v) corresponds to moving south.
  // Thus, we subtract the latitude offset.
  double lon = hyper::GEO_ORIGIN_LON + delta_lon;
  double lat = hyper::GEO_ORIGIN_LAT - delta_lat;

  return {lon, lat};
}

// Compute the intersection of a vector of bounding boxes.
cv::Rect computeIntersection(const std::vector<cv::Rect>& boxes) {
  if (boxes.empty())
    return cv::Rect();
  
  cv::Rect intersection = boxes[0];
  for (size_t i = 1; i < boxes.size(); i++) {
    int x1 = std::max(intersection.x, boxes[i].x);
    int y1 = std::max(intersection.y, boxes[i].y);
    int x2 = std::min(intersection.x + intersection.width, boxes[i].x + boxes[i].width);
    int y2 = std::min(intersection.y + intersection.height, boxes[i].y + boxes[i].height);
    
    if (x2 <= x1 || y2 <= y1) {
      // No intersection exists.
      return cv::Rect();
    }
    intersection = cv::Rect(x1, y1, x2 - x1, y2 - y1);
  }
  return intersection;
}

// Compute Euclidean distance between two geographic points.
double geoDistance(const std::pair<double, double> &a, const std::pair<double, double> &b) {
  return std::sqrt((a.first - b.first) * (a.first - b.first) +
                   (a.second - b.second) * (a.second - b.second));
}

// ------------------------------
// Simple DBSCAN Implementation
// ------------------------------

// For a given point at index 'idx', return indices of all points within 'eps' distance.
std::vector<int> regionQuery(const std::vector<std::pair<double, double>> &points, int idx, double eps) {
  std::vector<int> neighbors;
  for (size_t i = 0; i < points.size(); ++i) {
    if (geoDistance(points[idx], points[i]) <= eps) {
      neighbors.push_back(i);
    }
  }
  return neighbors;
}

// Expand the cluster starting from the given point.
void expandCluster(const std::vector<std::pair<double, double>> &points,
                   std::vector<int> &labels,
                   int idx,
                   std::vector<int> &neighbors,
                   int clusterId,
                   double eps,
                   int minPoints) {
  labels[idx] = clusterId;
  
  // Process all neighbors.
  for (size_t i = 0; i < neighbors.size(); i++) {
    int neighborIdx = neighbors[i];
    
    if (labels[neighborIdx] == -1) {
      // Previously marked as noise, now included.
      labels[neighborIdx] = clusterId;
    }
    
    if (labels[neighborIdx] != 0)
      continue;
    
    labels[neighborIdx] = clusterId;
    std::vector<int> neighborNeighbors = regionQuery(points, neighborIdx, eps);
    if (neighborNeighbors.size() >= static_cast<size_t>(minPoints)) {
      // Append these new neighbors to the current neighbor list.
      neighbors.insert(neighbors.end(), neighborNeighbors.begin(), neighborNeighbors.end());
    }
  }
}

// Run DBSCAN clustering on geographic points.
// Returns a vector of cluster IDs for each point:
//    0: unvisited, -1: noise, positive numbers: cluster IDs.
std::vector<int> clusterGeoPoints(const std::vector<std::pair<double, double>> &points,
                                  double eps, int minPoints) {
  std::vector<int> labels(points.size(), 0);
  int clusterId = 0;
  
  for (size_t i = 0; i < points.size(); i++) {
    if (labels[i] != 0)
      continue;
    
    std::vector<int> neighbors = regionQuery(points, i, eps);
    if (neighbors.size() < static_cast<size_t>(minPoints)) {
      labels[i] = -1;  // Mark as noise.
    } else {
      clusterId++;
      expandCluster(points, labels, i, neighbors, clusterId, eps, minPoints);
    }
  }
  
  return labels;
}

// ------------------------------
// Detector Interface
// ------------------------------
class Detector {
public:
  struct DetectionResult {
    cv::Rect bounding_box;
    std::string label;
  };

  virtual ~Detector() = default;
  virtual std::vector<DetectionResult> detect(const cv::Mat & image) = 0;
};

// ------------------------------
// MultiColorDetector Implementation (Detects Different Colors)
// ------------------------------
class MultiColorDetector : public Detector {
public:
  struct ColorThreshold {
    std::string color_name;
    cv::Scalar lower;
    cv::Scalar upper;
  };

  std::vector<ColorThreshold> thresholds;

  MultiColorDetector() {
    // Thresholds for red, green, blue. Add additional colors as needed.
    thresholds.push_back({"Red Object", cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)});
    thresholds.push_back({"Green Object", cv::Scalar(36, 50, 70), cv::Scalar(89, 255, 255)});
    thresholds.push_back({"Blue Object", cv::Scalar(90, 50, 70), cv::Scalar(128, 255, 255)});
  }

  std::vector<DetectionResult> detect(const cv::Mat & image) override {
    std::vector<DetectionResult> detections;
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Process each color threshold.
    for (const auto & thr : thresholds) {
      cv::Mat mask;
      cv::inRange(hsv, thr.lower, thr.upper, mask);

      // Clean the mask using morphological operations.
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel);

      // Find contours corresponding to the current color.
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      for (const auto & contour : contours) {
        double area = cv::contourArea(contour);
        if (area > hyper::MIN_CONTOUR_AREA) {
          cv::Rect bbox = cv::boundingRect(contour);
          DetectionResult res;
          res.bounding_box = bbox;
          res.label = thr.color_name;
          detections.push_back(res);
        }
      }
    }
    return detections;
  }
};

// ------------------------------
// Image Listener Node (ROS2 Node)
// ------------------------------
class ImageListenerNode : public rclcpp::Node {
public:
  ImageListenerNode(std::shared_ptr<Detector> detector)
    : Node("image_listener_node"), detector_(detector) {
    // Subscribe to the image topic.
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image", 10,
      std::bind(&ImageListenerNode::image_callback, this, std::placeholders::_1));
  }

private:
  // Callback to receive and process each frame as it arrives.
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    // Clone the frame for processing.
    latest_frame_ = cv_ptr->image.clone();

    // Run detection on the latest frame immediately.
    auto results = detector_->detect(latest_frame_);

    // Append new detections to the vectors and annotate the frame.
    for (const auto & detection : results) {
      detection_labels_.push_back(detection.label);
      detection_coordinates_.push_back(detection.bounding_box);

      // Annotate the frame.
      cv::rectangle(latest_frame_, detection.bounding_box, cv::Scalar(0, 255, 0), 2);
      cv::putText(latest_frame_, detection.label,
                  cv::Point(detection.bounding_box.x, detection.bounding_box.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    // Log the detection labels.
    std::stringstream label_ss;
    label_ss << "Detected colors: ";
    for (const auto & label : detection_labels_) {
      label_ss << label << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", label_ss.str().c_str());

    // Log the detection coordinates.
    std::stringstream coord_ss;
    coord_ss << "Detection coordinates (x, y, w, h): ";
    for (const auto & rect : detection_coordinates_) {
      coord_ss << "(" << rect.x << ", " << rect.y << ", "
               << rect.width << ", " << rect.height << ") ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", coord_ss.str().c_str());

    // Display the annotated frame.
    cv::imshow("Final Detections", latest_frame_);
    cv::waitKey(1);

    // ----- Filtering Flickering Detections with Clustering -----
    if (detection_coordinates_.size() >= static_cast<size_t>(hyper::ACCUMULATION_THRESHOLD)) {
      // 1. Convert each detection’s centroid into geographic coordinates.
      std::vector<std::pair<double, double>> geoPoints;
      std::vector<cv::Rect> boxes;
      std::vector<std::string> labels;
      for (size_t i = 0; i < detection_coordinates_.size(); i++) {
        cv::Point2f center = computeCentroid(detection_coordinates_[i]);
        auto geoPt = convertToGeoCoordinates(center);
        geoPoints.push_back(geoPt);
        boxes.push_back(detection_coordinates_[i]);
        labels.push_back(detection_labels_[i]);
      }

      // 2. Run DBSCAN clustering on the geographic points.
      std::vector<int> clusterIds = clusterGeoPoints(geoPoints,
                                                     hyper::DBSCAN_EPS,
                                                     hyper::DBSCAN_MIN_POINTS);

      // 3. Group detections by cluster.
      std::map<int, std::vector<cv::Rect>> clusterBoxes;
      std::map<int, std::vector<std::string>> clusterLabelsMap;
      for (size_t i = 0; i < clusterIds.size(); i++) {
        int clusterId = clusterIds[i];
        if (clusterId == -1)
          continue; // Skip noise.
        clusterBoxes[clusterId].push_back(boxes[i]);
        clusterLabelsMap[clusterId].push_back(labels[i]);
      }

      // 4. For each valid cluster, compute the intersection of the bounding boxes.
      for (const auto & cluster : clusterBoxes) {
        int cid = cluster.first;
        const auto & clusterRects = cluster.second;
        if (clusterRects.size() >= static_cast<size_t>(hyper::DBSCAN_MIN_POINTS)) {
          cv::Rect stableDetection = computeIntersection(clusterRects);
          std::stringstream ss;
          ss << "Stable detection for cluster " << cid << ": ("
             << stableDetection.x << ", " << stableDetection.y << ", "
             << stableDetection.width << ", " << stableDetection.height << ")";
          RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
      }
      
      // Clear the detection vectors after processing.
      detection_coordinates_.clear();
      detection_labels_.clear();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  cv::Mat latest_frame_;
  std::shared_ptr<Detector> detector_;
  // Vectors to store detection labels and bounding boxes.
  std::vector<std::string> detection_labels_;
  std::vector<cv::Rect> detection_coordinates_;
};

// ------------------------------
// Main Function
// ------------------------------
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // Create an instance of the multi-color detector.
  std::shared_ptr<Detector> detector = std::make_shared<MultiColorDetector>();

  // Create and spin the image listener node.
  auto node = std::make_shared<ImageListenerNode>(detector);
  rclcpp::spin(node);

  cv::destroyAllWindows();  // Cleanup OpenCV windows.
  rclcpp::shutdown();
  return 0;
}
