#!/usr/bin/env python3
"""
ROS2 Detection Node with RT-DETR

Key Behaviors:
  1. Modularity:
       - Uses RTDETRDetector, swappable with any other `Detector` implementation.
  2. Timed Execution:
       - Detection runs only once every 5 seconds, regardless of camera frame rate.
  3. Data Structures:
       - `detection_labels_` holds the list of labels detected.
       - `detection_coordinates_` holds the corresponding bounding boxes.
       - `detection_drone_positions_` holds the drone's local position at the time of each detection.
  4. Logging & Visualization:
       - Detections are logged via ROS2 logger.
       - Results are drawn on the frame and shown in an OpenCV window.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError


from px4_msgs.msg import VehicleLocalPosition



import cv2
import numpy as np
from abc import ABC, abstractmethod
from typing import List, Tuple, Any # Added Any for drone position type flexibility


from PIL import Image as PILImage
import supervision as sv
from rfdetr import RFDETRLarge
from rfdetr.util.coco_classes import COCO_CLASSES
from tqdm import tqdm


class DetectionResult:
    """Holds one detection’s bounding box and label."""
    def __init__(self, bbox: Tuple[int, int, int, int], label: str):
        self.bounding_box = bbox
        self.label = label


class Detector(ABC):
    """Abstract detector interface."""
    @abstractmethod
    def detect(self, image: np.ndarray) -> List[DetectionResult]:
        ...

class RTDETRDetector(Detector):
    """Detector using RT-DETR model with tiling and specific class filtering."""
    def __init__(self, tile_size: int = 728, overlap: int = 100, threshold: float = 0.5):
        super().__init__()
        self.tile_size = tile_size
        self.overlap = overlap
        self.threshold = threshold
        
        print(f"Initializing RFDETRLarge model (resolution: {self.tile_size}). This may take a moment...")
        self.model = RFDETRLarge(resolution=self.tile_size)
        print("RFDETRLarge model initialized.")
        
        self.target_object_names = [
            "person",       
            "car",          
            "motorcycle",   
            "airplane",     
            "bus",          
            "boat",         
            "stop sign",    
            "snowboard",    
            "umbrella",     
            "sports ball",  
            "baseball bat", 
            "bed",          
            "tennis racket",
            "suitcase",     
            "skis"         
        ]
        self.coco_class_names = COCO_CLASSES

    def _create_tiles(self, image: PILImage.Image) -> List[Tuple[PILImage.Image, Tuple[int, int]]]:
        width, height = image.size
        tiles = []
        tile_s = self.tile_size
        ovlp = self.overlap
        x_pos_temp = list(range(0, width - ovlp, tile_s - ovlp))
        if width not in x_pos_temp and x_pos_temp:
            x_pos_temp.append(max(0, width - tile_s))
        if not x_pos_temp:
            x_pos_temp = [0]
        x_positions = sorted(list(set(x_pos_temp)))
        y_pos_temp = list(range(0, height - ovlp, tile_s - ovlp))
        if height not in y_pos_temp and y_pos_temp:
            y_pos_temp.append(max(0, height - tile_s))
        if not y_pos_temp:
            y_pos_temp = [0]
        y_positions = sorted(list(set(y_pos_temp)))
        for y_anchor in y_positions:
            for x_anchor in x_positions:
                x_end_crop = min(x_anchor + tile_s, width)
                y_end_crop = min(y_anchor + tile_s, height)
                x_start_crop = max(0, x_end_crop - tile_s)
                y_start_crop = max(0, y_end_crop - tile_s)
                tile_image = image.crop((x_start_crop, y_start_crop, x_end_crop, y_end_crop))
                tiles.append((tile_image, (x_start_crop, y_start_crop)))
        return tiles

    def _process_image_with_tiles(self, image: PILImage.Image) -> sv.Detections:
        tiles_with_positions = self._create_tiles(image)
        all_detections_list: List[sv.Detections] = []
        for tile, (x_offset, y_offset) in tqdm(tiles_with_positions, desc="Processing tiles", leave=False, unit="tile"):
            tile_sv_detections = self.model.predict(tile, threshold=self.threshold)
            if len(tile_sv_detections.xyxy) > 0:
                adjusted_boxes = tile_sv_detections.xyxy.copy()
                adjusted_boxes[:, 0] += x_offset
                adjusted_boxes[:, 1] += y_offset
                adjusted_boxes[:, 2] += x_offset
                adjusted_boxes[:, 3] += y_offset
                all_detections_list.append(sv.Detections(
                    xyxy=adjusted_boxes,
                    confidence=tile_sv_detections.confidence,
                    class_id=tile_sv_detections.class_id
                ))
        if not all_detections_list: return sv.Detections.empty()
        combined_detections = sv.Detections.merge(detections_list=all_detections_list)
        if len(combined_detections.xyxy) > 0:
            final_detections = combined_detections.with_nms(threshold=0.45)
            return final_detections
        else: return sv.Detections.empty()

    def detect(self, image: np.ndarray) -> List[DetectionResult]:
        pil_rgb_image = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        processed_detections_sv = self._process_image_with_tiles(pil_rgb_image)
        detection_results: List[DetectionResult] = []
        if len(processed_detections_sv.xyxy) > 0:
            for i in range(len(processed_detections_sv.xyxy)):
                class_id = processed_detections_sv.class_id[i]
                if 0 <= class_id < len(self.coco_class_names):
                    class_name = self.coco_class_names[class_id]
                    if class_name in self.target_object_names:
                        xyxy = processed_detections_sv.xyxy[i]
                        x_min, y_min, x_max, y_max = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                        width, height = x_max - x_min, y_max - y_min
                        bbox = (x_min, y_min, width, height)
                        detection_results.append(DetectionResult(bbox=bbox, label=class_name))
        return detection_results


class ImageListenerNode(Node):
    """ROS2 node subscribing to Image, running RT-DETR detection periodically."""
    def __init__(self, detector: Detector):
        super().__init__('image_listener_rtdetr_node')
        self.bridge = CvBridge()
        self.detector = detector
        self.latest_frame: np.ndarray = None
        self.latest_drone_position: Any = None 

        self.detection_labels_: List[str] = []
        self.detection_coordinates_: List[Tuple[int,int,int,int]] = []
        self.detection_drone_positions_: List[Any] = [] 

        # Image subscriber
        self.create_subscription(
            RosImage,
            '/world/bv_mission/model/x500_gimbal_0/link/camera_link/sensor/camera/image',
            self.image_callback,
            10
        )


        self.vehicle_position_topic = '/fmu/out/vehicle_local_position'
        self.create_subscription(
            VehicleLocalPosition,
            self.vehicle_position_topic,
            self.vehicle_position_callback,
            10
        )



        self.timer_duration = 5.0
        self.create_timer(self.timer_duration, self.timer_callback)
        self.get_logger().info(
            f"{self.get_name()} initialized. "
            f"Detection timer: {self.timer_duration}s. "
            f"Detecting: {getattr(self.detector, 'target_object_names', 'all COCO classes')}. "
            f"Subscribed to drone position on: {self.vehicle_position_topic}"
        )


    def image_callback(self, msg: RosImage):
        """Callback for incoming images. Stores the latest frame."""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_frame = cv_img.copy()
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')

    def vehicle_position_callback(self, msg: VehicleLocalPosition):
        """Callback for VehicleLocalPosition. Stores the latest X, Y, Z position."""

        self.latest_drone_position = (msg.x, msg.y, msg.z)




    def timer_callback(self):
        """Periodically run detection on the latest frame."""
        if self.latest_frame is None:
            self.get_logger().warn('No frame available for detection.')
            return
        
   
        if self.latest_drone_position is None:
            self.get_logger().warn('No drone position available yet for association.')

        self.get_logger().info('Starting detection cycle...')
        frame_to_process = self.latest_frame.copy()

        results = self.detector.detect(frame_to_process)
        self.get_logger().info(f'Detection cycle complete. Found {len(results)} target objects in current frame.')

        display_frame = frame_to_process
        
        current_drone_pos_for_this_cycle = self.latest_drone_position # Capture position for this cycle's detections

        if results:
            for idx, det_result in enumerate(results):
                x, y, w, h = det_result.bounding_box
                self.detection_labels_.append(det_result.label)
                self.detection_coordinates_.append(det_result.bounding_box)
                # Append the drone's position at the time of this detection batch
                self.detection_drone_positions_.append(current_drone_pos_for_this_cycle)


                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                label_y_pos = y - 10 if y - 10 > 10 else y + h + 20
                cv2.putText(display_frame, det_result.label, (x, label_y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Log the associated drone position for this specific detection (optional)
                pos_str = f"({current_drone_pos_for_this_cycle[0]:.2f}, {current_drone_pos_for_this_cycle[1]:.2f}, {current_drone_pos_for_this_cycle[2]:.2f})" if current_drone_pos_for_this_cycle else "N/A"
                self.get_logger().debug(f"Detected: {det_result.label} at ({x},{y},{w},{h}), Drone Pos: {pos_str}")
        
        final_label_str = ', '.join(self.detection_labels_) if self.detection_labels_ else 'None'
        self.get_logger().info(f'Accumulated Labels: {final_label_str}')

        # Log accumulated drone positions (can be very verbose)
        # Example: just log the count or the last few
        num_pos_records = len(self.detection_drone_positions_)
        self.get_logger().info(f'Accumulated Drone Position Records: {num_pos_records}')
        if num_pos_records > 0 and self.detection_drone_positions_[-1] is not None:
             last_pos = self.detection_drone_positions_[-1]
             self.get_logger().debug(f'Last recorded drone position for a detection: ({last_pos[0]:.2f}, {last_pos[1]:.2f}, {last_pos[2]:.2f})')
        

        cv2.imshow('RT-DETR Detections', display_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    rtdetr_detector = RTDETRDetector(tile_size=728, overlap=200, threshold=0.5)
    node = ImageListenerNode(detector=rtdetr_detector)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        node.get_logger().info('Cleaning up before exit.')
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()