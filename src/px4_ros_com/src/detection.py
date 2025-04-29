#!/usr/bin/env python3
"""
ROS2 Detection Node

Key Behaviors:
  1. Modularity:
       - Swap out `MultiColorDetector` for any other `Detector` implementation
         (e.g. a deep-learning model) without changing the node logic.
  2. Timed Execution:
       - Detection runs only once every 5 seconds, regardless of camera frame rate.
  3. Data Structures:
       - `detection_labels_` holds the list of labels detected.
       - `detection_coordinates_` holds the corresponding bounding boxes.
  4. Logging & Visualization:
       - All detections are logged via ROS2 logger.
       - Results are drawn on the frame and shown in an OpenCV window.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
from abc import ABC, abstractmethod
from typing import List, Tuple


class DetectionResult:
    """Holds one detection’s bounding box and label."""
    def __init__(self, bbox: Tuple[int, int, int, int], label: str):
        self.bounding_box = bbox  # (x, y, w, h)
        self.label = label


class Detector(ABC):
    """Abstract detector interface."""
    @abstractmethod
    def detect(self, image: np.ndarray) -> List[DetectionResult]:
        ...

class MultiColorDetector(Detector):
    """Detects red, green, blue objects via HSV thresholding."""
    def __init__(self):
        # (name, lower HSV, upper HSV)
        self.thresholds = [
            ("Red Object",   np.array([0, 120,  70]),  np.array([10, 255, 255])),
            ("Green Object", np.array([36,  50,  70]),  np.array([89, 255, 255])),
            ("Blue Object",  np.array([90,  50,  70]),  np.array([128, 255, 255])),
        ]
        # 3×3 rectangular kernel
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def detect(self, image: np.ndarray) -> List[DetectionResult]:
        detections: List[DetectionResult] = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for name, lower, upper in self.thresholds:
            mask = cv2.inRange(hsv, lower, upper)
            # clean up
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, self.kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    detections.append(DetectionResult((x, y, w, h), name))

        return detections


class ImageListenerNode(Node):
    """ROS2 node that subscribes to Image, runs detection every 5 seconds."""
    def __init__(self, detector: Detector):
        super().__init__('image_listener_node')
        self.bridge = CvBridge()
        self.detector = detector
        self.latest_frame = None

        self.detection_labels_: List[str] = []
        self.detection_coordinates_: List[Tuple[int,int,int,int]] = []

        # subscribe to camera topic
        self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )

        # timer fires every 5 seconds, regardless of frame rate
        self.create_timer(2.0, self.timer_callback)

    def image_callback(self, msg: Image):
        """Save the latest frame for timed processing."""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        self.latest_frame = cv_img.copy()
        cv2.waitKey(1)

    def timer_callback(self):
        """Called once every 5 seconds to run detection + logging + display."""
        if self.latest_frame is None:
            self.get_logger().warn('No frame available for detection.')
            return

        # run detector
        results = self.detector.detect(self.latest_frame)


        # annotate results
        for det in results:
            x, y, w, h = det.bounding_box
            self.detection_labels_.append(det.label)
            self.detection_coordinates_.append(det.bounding_box)

            cv2.rectangle(self.latest_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(
                self.latest_frame, det.label,
                (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2
            )
            self.get_logger().info(
                f"After detection {det.label}: "
                f"labels={self.detection_labels_}, "
                f"boxes={self.detection_coordinates_}"
            )

        # log labels
        label_str = ', '.join(self.detection_labels_) or 'None'
        self.get_logger().info(f'Detected labels: {label_str}')

        # log coordinates
        coord_str = ', '.join(f'({x},{y},{w},{h})' for x,y,w,h in self.detection_coordinates_) or 'None'
        self.get_logger().info(f'Coordinates: {coord_str}')

        # display
        cv2.imshow('Detections', self.latest_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detector = MultiColorDetector()
    node = ImageListenerNode(detector)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
