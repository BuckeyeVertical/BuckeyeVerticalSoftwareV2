#!/usr/bin/env python3
"""
object_coordinate_estimator.py

Given:
  - detection_coords: List[Tuple[x, y, w, h]]
  - detection_labels: List[str]
  - K (3×3 intrinsics)

This node:
  1. Subscribes to /fmu/out/vehicle_local_position to get the drone's local position,
     matching the publisher's Best-Effort & Transient-Local QoS.
  2. Computes pixel centers of each bbox.
  3. Converts to normalized camera coords via inv(K).
  4. Adds the drone's position to each point, yielding world coordinates.
  5. Runs DBSCAN (per label) to cluster points & remove outliers.
  6. Outputs the mean coordinate of each cluster as the object's final location.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
import numpy as np
from sklearn.cluster import DBSCAN
from collections import defaultdict

class ObjectCoordinateEstimator(Node):
    def __init__(self):
        super().__init__('object_coordinate_estimator')
        self.vehicle_local_position = None

        # Match publisher's QoS: Best-Effort reliability, Transient-Local durability
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos
        )

        self.K = np.array([
            [539.93633270263672, 0.0,                640.0],
            [0.0,                539.93637084960938, 480.0],
            [0.0,                0.0,                1.0]
        ], dtype=np.float32)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def pixel_centers(self, bboxes):
        return np.array([[x + w/2.0, y + h/2.0] for x, y, w, h in bboxes], dtype=np.float32)

    def pixel_to_norm_coords(self, uv_points: np.ndarray) -> np.ndarray:
        inv_K = np.linalg.inv(self.K)
        uv1 = np.hstack([uv_points, np.ones((uv_points.shape[0], 1), dtype=np.float32)])
        return (inv_K @ uv1.T).T

    def cluster_and_average(self, points: np.ndarray,
                            eps: float = 0.05,
                            min_samples: int = 3) -> dict:
        if points.size == 0:
            return {}
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(points)
        return {lbl: points[labels == lbl].mean(axis=0)
                for lbl in set(labels) if lbl != -1}

    def estimate_object_locations(self, bboxes, labels,
                                  eps=0.05, min_samples=3):
        grouped = defaultdict(list)
        for bbox, lbl in zip(bboxes, labels):
            grouped[lbl].append(bbox)

        results = {}
        for lbl, boxes in grouped.items():
            uv = self.pixel_centers(boxes)
            cam_pts = self.pixel_to_norm_coords(uv)
            if self.vehicle_local_position:
                drone_pos = np.array([
                    self.vehicle_local_position.x,
                    self.vehicle_local_position.y,
                    self.vehicle_local_position.z
                ], dtype=np.float32)
                cam_pts += drone_pos
            clusters = self.cluster_and_average(cam_pts, eps, min_samples)
            results[lbl] = list(clusters.values())
        return results


def main(args=None):
    rclpy.init(args=args)
    node = ObjectCoordinateEstimator()

    # full detection history test data
    all_labels = [
        'Red Object', 'Red Object', 'Red Object', 'Red Object',
        'Red Object', 'Green Object', 'Red Object', 'Green Object',
        'Red Object', 'Green Object', 'Red Object', 'Green Object',
        'Blue Object', 'Red Object', 'Green Object', 'Blue Object',
        'Red Object', 'Green Object', 'Blue Object', 'Red Object',
        'Green Object', 'Blue Object', 'Red Object', 'Green Object',
        'Blue Object', 'Red Object', 'Green Object', 'Blue Object',
        'Red Object', 'Green Object', 'Blue Object', 'Red Object',
        'Green Object', 'Blue Object', 'Red Object', 'Green Object',
        'Blue Object', 'Red Object', 'Green Object', 'Blue Object',
        'Red Object', 'Green Object', 'Blue Object', 'Red Object',
        'Green Object', 'Blue Object', 'Red Object', 'Green Object',
        'Blue Object', 'Red Object', 'Green Object', 'Blue Object'
    ]
    all_boxes = [
        (543, 580, 628, 380), (510, 99, 360, 374), (536, 0,   253, 258),
        (569,   0, 188, 200), (594,   0, 151, 204), (0,   586,  98, 152),
        (615,  63, 120, 167), (0,   556, 186, 131), (631, 126, 100, 134),
        (98,  544, 156, 108), (640, 174,  86, 112), (176, 536, 129,  92),
        (448, 923,  81,  37), (642, 205,  76,  98), (222, 530, 113,  82),
        (445, 888,  94,  72), (640, 226,  69,  88), (256, 523, 102,  73),
        (445, 854, 100, 106), (637, 233,  66,  83), (272, 515,  96,  69),
        (451, 831,  96, 124), (632, 235,  63,  81), (277, 509,  92,  66),
        (451, 816,  93, 118), (627, 236,  62,  79), (279, 504,  91,  64),
        (450, 806,  91, 115), (625, 236,  61,  78), (281, 500,  89,  63),
        (450, 799,  90, 113), (623, 234,  60,  78), (282, 496,  89,  62),
        (449, 793,  89, 110), (620, 231,  60,  78), (281, 492,  88,  61),
        (447, 787,  89, 109), (619, 227,  60,  78), (282, 487,  88,  61),
        (447, 780,  89, 108), (618, 222,  60,  79), (282, 482,  87,  60),
        (447, 774,  89, 107), (618, 221,  60,  78), (283, 480,  87,  59),
        (448, 771,  87, 106), (618, 221,  59,  78), (284, 479,  86,  60),
        (447, 770,  88, 106), (618, 222,  59,  78), (285, 479,  87,  59),
        (448, 769,  87, 106)
    ]

    # pick only the last 10 detections
    detection_labels = all_labels[-10:]
    detection_coords = all_boxes[-10:]

    final_locations = node.estimate_object_locations(
        detection_coords,
        detection_labels,
        eps=0.05,
        min_samples=2
    )

    # Print out the results:
    for label, coords in final_locations.items():
        print(f"\nLabel: {label}")
        for i, (x, y, z) in enumerate(coords, 1):
            print(f"  Cluster {i}:  x={x:.4f}, y={y:.4f}, z={z:.4f}")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()