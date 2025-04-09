#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

// OpenCV
#include "opencv2/opencv.hpp"

// ------------------------------
// Detector Interface
// ------------------------------
class Detector
{
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
class MultiColorDetector : public Detector
{
public:
  struct ColorThreshold {
    std::string color_name;
    cv::Scalar lower;
    cv::Scalar upper;
  };

  std::vector<ColorThreshold> thresholds;

  MultiColorDetector()
  {
    // Thresholds for red, green, blue. Add additional colors as needed.
    thresholds.push_back({"Red Object", cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)});
    thresholds.push_back({"Green Object", cv::Scalar(36, 50, 70), cv::Scalar(89, 255, 255)});
    thresholds.push_back({"Blue Object", cv::Scalar(90, 50, 70), cv::Scalar(128, 255, 255)});
  }

  std::vector<DetectionResult> detect(const cv::Mat & image) override
  {
    std::vector<DetectionResult> detections;
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Process each color threshold.
    for (const auto & thr : thresholds)
    {
      cv::Mat mask;
      cv::inRange(hsv, thr.lower, thr.upper, mask);

      // Clean the mask using morphological operations.
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel);


      // Find contours corresponding to the current color.
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      for (const auto & contour : contours)
      {
        double area = cv::contourArea(contour);
        if (area > 500)  // Tune this threshold as needed.
        {
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
// Image Listener Node (ROS2 Node) with Timer-based Detection
// ------------------------------
class ImageListenerNode : public rclcpp::Node
{
public:
  ImageListenerNode(std::shared_ptr<Detector> detector)
    : Node("image_listener_node"), detector_(detector)
  {
    // Subscribe to the image topic.
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image", 10,
      std::bind(&ImageListenerNode::image_callback, this, std::placeholders::_1));

    // Create a timer that triggers detection every 5 seconds.
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&ImageListenerNode::timer_callback, this));
  }

private:
  // Callback to continuously receive and store the latest frame.
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    // Clone and store the frame for timed processing.
    latest_frame_ = cv_ptr->image.clone();
    cv::waitKey(1);
  }

  // Timer callback that runs every 5 seconds to perform detection.
  void timer_callback()
  {
    if (latest_frame_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No frame available for detection.");
      return;
    }

    // Run detection on the latest frame.
    auto results = detector_->detect(latest_frame_);

    // Clear the data structures before processing.
    detection_labels_.clear();
    detection_coordinates_.clear();

    // Process each detection.
    for (const auto & detection : results)
    {
      detection_labels_.push_back(detection.label);
      detection_coordinates_.push_back(detection.bounding_box);
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

    
    // Log and print detection coordinates.
    std::stringstream coord_ss;
    coord_ss << "Detection coordinates (x, y, w, h): ";
    for (const auto & rect : detection_coordinates_) {
      coord_ss << "(" << rect.x << ", " << rect.y << ", " << rect.width << ", " << rect.height << ") ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", coord_ss.str().c_str());

    // Display the final annotated frame.
    cv::imshow("Final Detections", latest_frame_);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat latest_frame_;
  std::shared_ptr<Detector> detector_;
  // Data structure to store detection labels.
  std::vector<std::string> detection_labels_;
  // Data structure to store detection coordinates (bounding boxes).
  std::vector<cv::Rect> detection_coordinates_;
};

// ------------------------------
// Main Function
// ------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create an instance of the multi-color detector.
  std::shared_ptr<Detector> detector = std::make_shared<MultiColorDetector>();

  // Create and spin the image listener node with the detector.
  auto node = std::make_shared<ImageListenerNode>(detector);
  rclcpp::spin(node);

  cv::destroyAllWindows();  // Cleanup OpenCV windows.
  rclcpp::shutdown();
  return 0;
}
