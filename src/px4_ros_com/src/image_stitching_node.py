#!/usr/bin/env python3

# ----------------------------------------------------
# Image Stitcher Node (ROS2 + OpenCV in Python)
# ----------------------------------------------------
# - Subscribes to a ROS2 image topic
# - Buffers incoming images
# - Every N seconds, stitches them
# - Saves and displays the result (no publishing)
# ----------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime

class ImageStitcherNode(Node):
    def __init__(self):
        super().__init__('image_stitcher_node')

        # Parameters
        self.declare_parameter('output_path', '/tmp/stitched.jpg')
        self.declare_parameter('crop', True)
        self.declare_parameter('preprocessing', False)
        self.declare_parameter('stitch_interval_sec', 3.0)

        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.crop = self.get_parameter('crop').get_parameter_value().bool_value
        self.preprocessing = self.get_parameter('preprocessing').get_parameter_value().bool_value
        self.stitch_interval_sec = self.get_parameter('stitch_interval_sec').get_parameter_value().double_value

        self.image_sub = self.create_subscription(
            Image,
            '/world/baylands/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image',
            self.image_callback,
            10
        )

        self.stitch_timer = self.create_timer(self.stitch_interval_sec, self.timer_callback)

        self.bridge = CvBridge()
        self.received_images = []

        self.get_logger().info("Image Stitcher Node Initialized.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.received_images.append(cv_image)
            self.get_logger().info(f"Received image. Buffer size: {len(self.received_images)}")
        except Exception as e:
            self.get_logger().error(f"cv_bridge exception: {str(e)}")


    def timer_callback(self):
        if len(self.received_images) < 2:
            self.get_logger().warn("Not enough images to stitch yet.")

            return

        self.get_logger().info(f"Attempting to stitch {len(self.received_images)} images...")

        status, stitched = self.stitch_images(self.received_images)
        self.received_images.clear()

        if status == cv2.Stitcher_OK:
            self.get_logger().info("Image stitching successful.")

            if self.crop:
                stitched = self.crop_image(stitched)

            # Save stitched image
            now = datetime.now()
            timestamp = now.strftime("%Y%m%d_%H%M%S")
            output_file = os.path.join(os.path.dirname(self.output_path), f"stitched_{timestamp}.jpg")
            cv2.imwrite(output_file, stitched)
            self.get_logger().info(f"Stitched image saved to {output_file}")

            # Display stitched image
            cv2.imshow("Stitched Image", stitched)
            cv2.waitKey(1)  # 1ms non-blocking wait
        else:
            self.get_logger().error(f"Image stitching failed with status code: {status}")
            if status == cv2.Stitcher_ERR_NEED_MORE_IMGS:
                self.get_logger().error("Need more images to perform stitching.")
            elif status == cv2.Stitcher_ERR_HOMOGRAPHY_EST_FAIL:
                self.get_logger().error("Homography estimation failed. Improve overlap or reduce blur.")
            elif status == cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL:
                self.get_logger().error("Camera parameter adjustment failed. Check capture settings.")

    def resize_images(self, images, widthThreshold=1500):
        resized_images = []
        for image in images:
            if image.shape[1] > widthThreshold:
                ratio = widthThreshold / image.shape[1]
                dim = (widthThreshold, int(image.shape[0] * ratio))
                resized_images.append(cv2.resize(image, dim))
            else:
                resized_images.append(image)
        return resized_images

    def preprocess_images(self, images):
        if not self.preprocessing:
            return images

        self.get_logger().info("[INFO] Preprocessing images to improve clarity...")
        for i in range(len(images)):
            lab = cv2.cvtColor(images[i], cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l = clahe.apply(l)
            lab = cv2.merge((l, a, b))
            images[i] = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return images

    def stitch_images(self, images):
        images = self.resize_images(images)
        images = self.preprocess_images(images)
        stitcher = cv2.Stitcher.create(cv2.Stitcher_SCANS)
        return stitcher.stitch(images)

    def crop_image(self, stitched):
        self.get_logger().info("[INFO] Cropping stitched image...")
        stitched = cv2.copyMakeBorder(stitched, 10, 10, 10, 10,
                                      cv2.BORDER_CONSTANT, value=(0, 0, 0))
        gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            stitched = stitched[y:y+h, x:x+w]
        return stitched


def main(args=None):
    rclpy.init(args=args)
    node = ImageStitcherNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
