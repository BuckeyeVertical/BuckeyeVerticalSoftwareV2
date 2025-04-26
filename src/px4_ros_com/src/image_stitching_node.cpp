// ----------------------------------------------------
// Image Stitcher Node (ROS2 + OpenCV)
// ----------------------------------------------------
// - Subscribes to a ROS2 image topic
// - Buffers incoming images
// - Every N seconds, attempts to stitch them together
// - Publishes and saves the stitched result
// ----------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <vector>
#include <string>
#include <filesystem>
#include <chrono>
#include <iomanip>

// ------------------------------
// Image Stitcher Node Definition
// ------------------------------
class ImageStitcherNode : public rclcpp::Node {
public:
  ImageStitcherNode()
      : Node("image_stitcher_node") {
    
    // Declare and load ROS parameters.
    this->declare_parameter<std::string>("output_path", "/tmp/stitched.jpg");
    this->declare_parameter<bool>("crop", true);
    this->declare_parameter<bool>("preprocessing", false);
    this->declare_parameter<double>("stitch_interval_sec", 10.0);  // How often to stitch (in seconds)

    this->get_parameter("output_path", output_path_);
    this->get_parameter("crop", crop_);
    this->get_parameter("preprocessing", preprocessing_);
    this->get_parameter("stitch_interval_sec", stitch_interval_sec_);

    // Publisher to output stitched image
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stitched_image", 10);

    // Subscriber to image feed
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image", 
      10,
      std::bind(&ImageStitcherNode::image_callback, this, std::placeholders::_1)
    );

    // Timer to stitch every N seconds
    stitch_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(stitch_interval_sec_),
      std::bind(&ImageStitcherNode::timer_callback, this)
    );
  }

private:
  // Parameters
  std::string output_path_;
  bool crop_;
  bool preprocessing_;
  double stitch_interval_sec_;

  // ROS Interfaces
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr stitch_timer_;

  // Image buffer (stores images between stitching intervals)
  std::vector<cv::Mat> received_images_;

  // ------------------------------
  // Image Callback (Triggered on Every Frame)
  // ------------------------------
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Store the incoming image
    received_images_.push_back(cv_ptr->image.clone());
    RCLCPP_INFO(this->get_logger(), "Received image. Buffer size: %zu", received_images_.size());
  }

  // ------------------------------
  // Timer Callback (Triggered Every N Seconds)
  // ------------------------------
  void timer_callback() {
    if (received_images_.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Not enough images to stitch yet.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Attempting to stitch %zu images...", received_images_.size());
    stitch_and_publish(received_images_);
    received_images_.clear();  // Clear buffer after stitching
  }

  // ------------------------------
  // Core Stitching and Publishing Logic
  // ------------------------------
  void stitch_and_publish(const std::vector<cv::Mat>& input_images) {
    auto resized = resize_images(input_images);
    auto preprocessed = preprocess_images(resized);
    cv::Mat stitched;
    auto status = stitch_images(preprocessed, stitched);

    if (status == cv::Stitcher::OK) {
      RCLCPP_INFO(this->get_logger(), "Image stitching successful.");
      crop_image(stitched);  // Optional post-processing crop

      // Generate timestamped filename
      auto now = std::chrono::system_clock::now();
      auto time = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time), "stitched_%Y%m%d_%H%M%S.jpg");

      std::filesystem::path output_path_base(output_path_);
      std::string filename = (output_path_base.parent_path() / ss.str()).string();

      // Save to disk
      if (!cv::imwrite(filename, stitched)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save stitched image to %s", filename.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Saved stitched image to %s", filename.c_str());
      }

      // Publish over ROS
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", stitched).toImageMsg();
      image_pub_->publish(*msg);
      RCLCPP_INFO(this->get_logger(), "Published stitched image to topic: /stitched_image");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Image stitching failed with status code: %d", static_cast<int>(status));
    }
  }

  // ------------------------------
  // Image Preprocessing Helpers
  // ------------------------------

  // Resize images to a consistent width if needed
  std::vector<cv::Mat> resize_images(const std::vector<cv::Mat>& images, int widthThreshold = 1500) {
    std::vector<cv::Mat> resized_images;
    for (const auto& image : images) {
      if (image.cols > widthThreshold) {
        double ratio = static_cast<double>(widthThreshold) / image.cols;
        cv::Size dim(widthThreshold, static_cast<int>(image.rows * ratio));
        cv::Mat resized;
        cv::resize(image, resized, dim);
        resized_images.push_back(resized);
      } else {
        resized_images.push_back(image);
      }
    }
    return resized_images;
  }

  // We can get rid of this, not sure if its better to have or leave out
  // Contrast enhancement (CLAHE)
  std::vector<cv::Mat> preprocess_images(std::vector<cv::Mat>& images) {
    if (!preprocessing_) return images;

    RCLCPP_INFO(this->get_logger(), "Preprocessing images...");
    for (auto& image : images) {
      cv::Mat lab_image;
      cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
      std::vector<cv::Mat> planes(3);
      cv::split(lab_image, planes);
      auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
      clahe->apply(planes[0], planes[0]);
      cv::merge(planes, lab_image);
      cv::cvtColor(lab_image, image, cv::COLOR_Lab2BGR);
    }
    return images;
  }

  // Run OpenCV stitching pipeline
  cv::Stitcher::Status stitch_images(const std::vector<cv::Mat>& images, cv::Mat& stitched) {
    RCLCPP_INFO(this->get_logger(), "Stitching images...");
    cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::SCANS);
    return stitcher->stitch(images, stitched);
  }

  // Auto-crop black borders from the stitched result
  void crop_image(cv::Mat& image) {
    if (!crop_) return;

    RCLCPP_INFO(this->get_logger(), "Cropping stitched image...");
    cv::copyMakeBorder(image, image, 10, 10, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    cv::Mat gray, thresh;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      size_t maxIdx = 0;
      double maxArea = 0;
      for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
          maxArea = area;
          maxIdx = i;
        }
      }

      cv::Rect bounding_rect = cv::boundingRect(contours[maxIdx]);
      image = image(bounding_rect).clone();
    }
  }
};

// ------------------------------
// Main Function
// ------------------------------
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageStitcherNode>());
  rclcpp::shutdown();
  return 0;
}
