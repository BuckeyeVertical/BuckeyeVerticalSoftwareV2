#include <memory>
#include <vector>
#include <string>
#include <sstream>

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
// RedDetector Implementation (Detects Any Red Object)
// ------------------------------
class RedDetector : public Detector
{
public:
  std::vector<DetectionResult> detect(const cv::Mat & image) override
  {
    std::vector<DetectionResult> detections;

    RCLCPP_INFO(rclcpp::get_logger("RedDetector"), "Starting red detection process...");

    // 1. Convert from BGR to HSV color space for easier thresholding.
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    RCLCPP_DEBUG(rclcpp::get_logger("RedDetector"), "Converted image to HSV.");
  

    // 2. Threshold HSV image to get only red colors.
    cv::Mat mask1, mask2;
    cv::inRange(hsv, cv::Scalar(0, 120, 70),  cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    cv::Mat mask = mask1 | mask2;
    RCLCPP_DEBUG(rclcpp::get_logger("RedDetector"), "Created raw red mask.");
  

    // 3. Clean the mask using morphological operations.
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel);
    RCLCPP_DEBUG(rclcpp::get_logger("RedDetector"), "Cleaned the red mask.");

    // 4. Detect contours in the cleaned mask.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_DEBUG(rclcpp::get_logger("RedDetector"), "Found %zu contours.", contours.size());

    // 5. Process each contour: if it is large enough, consider it a red object.
    for (const auto & contour : contours)
    {
      double area = cv::contourArea(contour);
      if (area > 500)  // Tune this threshold as necessary.
      {
        cv::Rect bbox = cv::boundingRect(contour);
        DetectionResult result;
        result.bounding_box = bbox;
        result.label = "Red Object";
        detections.push_back(result);

        // Log the detection details.
        std::stringstream ss;
        ss << "Red object detected at (x=" << bbox.x << ", y=" << bbox.y 
           << ", w=" << bbox.width << ", h=" << bbox.height << ")";
        RCLCPP_INFO(rclcpp::get_logger("RedDetector"), "%s", ss.str().c_str());
      }
    }

    return detections;
  }
};

// ------------------------------
// Image Listener Node (ROS2 Node)
// ------------------------------
class ImageListenerNode : public rclcpp::Node
{
public:
  ImageListenerNode(std::shared_ptr<Detector> detector)
  : Node("image_listener_node"), detector_(detector)
  {
    // Subscribe to the image topic (update the topic name if needed).
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image", 10,
      std::bind(&ImageListenerNode::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received new image frame.");

    // Convert the ROS image message to an OpenCV image.
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat frame = cv_ptr->image;

    // Run detection.
    std::vector<Detector::DetectionResult> results = detector_->detect(frame);

    // Draw bounding boxes and labels on the image.
    for (const auto & detection : results)
    {
      cv::rectangle(frame, detection.bounding_box, cv::Scalar(0, 255, 0), 2);
      cv::putText(frame, detection.label,
                  cv::Point(detection.bounding_box.x, detection.bounding_box.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    // Show the final image with detections.
    cv::imshow("Final Detections", frame);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::shared_ptr<Detector> detector_;
};

// ------------------------------
// Main Function
// ------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create an instance of the red detector.
  std::shared_ptr<Detector> detector = std::make_shared<RedDetector>();

  // Create and spin the image listener node with the detector.
  auto node = std::make_shared<ImageListenerNode>(detector);
  rclcpp::spin(node);

  cv::destroyAllWindows();  // Cleanup OpenCV windows.
  rclcpp::shutdown();
  return 0;
}
