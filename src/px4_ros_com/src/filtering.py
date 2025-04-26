#!/usr/bin/env python3
"""
object_coordinate_estimator.py

Given:
  - detection_coords: List[Tuple[x, y, w, h]]
  - detection_labels: List[str]
  - K (3×3 intrinsics)

This script:
  1. Computes pixel centers of each bbox.
  2. Converts to normalized camera coords via inv(K).
  3. Runs DBSCAN (per label) to cluster points & remove outliers.
  4. Outputs the mean coordinate of each cluster as the object's final location.
"""

import numpy as np
from sklearn.cluster import DBSCAN
from collections import defaultdict

def pixel_centers(bboxes):
    """Compute (u, v) center of each bounding box."""
    centers = []
    for x, y, w, h in bboxes:
        centers.append([x + w / 2.0, y + h / 2.0])
    return np.array(centers, dtype=np.float32)

def pixel_to_norm_coords(uv_points: np.ndarray, K: np.ndarray) -> np.ndarray:
    """
    Convert pixel coords (u,v) to normalized camera coords (x, y, 1)
    via inv(K) * [u, v, 1]^T. Returns Nx3 array.
    """
    inv_K = np.linalg.inv(K)
    # make homogeneous
    uv1 = np.hstack([uv_points, np.ones((uv_points.shape[0], 1), dtype=np.float32)])
    cam_pts = (inv_K @ uv1.T).T
    return cam_pts  # shape (N, 3)

def cluster_and_average(points: np.ndarray,
                        eps: float = 0.05,
                        min_samples: int = 3) -> dict:
    """
    Run DBSCAN on points (Nx3), return dict:
      cluster_label -> mean 3D coordinate of that cluster.
    Outliers (cluster -1) are dropped.
    """
    if len(points) == 0:
        return {}
    db = DBSCAN(eps=eps, min_samples=min_samples)
    labels = db.fit_predict(points)
    clusters = {}
    for lbl in set(labels):
        if lbl == -1:
            continue
        mask = labels == lbl
        clusters[lbl] = points[mask].mean(axis=0)
    return clusters

def estimate_object_locations(bboxes, labels, K,
                              eps=0.05, min_samples=3):
    """
    For each unique label:
      1. Get all centers of that label.
      2. Convert to normalized coords.
      3. Cluster & average.
    Returns:
      dict[label] = list of cluster-mean coordinates (as 3‐vectors)
    """
    # group bboxes by label
    grouped = defaultdict(list)
    for bbox, lbl in zip(bboxes, labels):
        grouped[lbl].append(bbox)

    results = {}
    for lbl, boxes in grouped.items():
        uv = pixel_centers(boxes)
        cam_pts = pixel_to_norm_coords(uv, K)
        clusters = cluster_and_average(cam_pts, eps=eps, min_samples=min_samples)
        # keep only the mean coords
        results[lbl] = list(clusters.values())

    return results

if __name__ == "__main__":
    # --- Intrinsics (unchanged) ---
    K = np.array([
        [9.31696166e+03, 0.0,            9.42461549e+02],
        [0.0,            1.02161568e+04, 9.53746283e+02],
        [0.0,            0.0,            1.0]
    ], dtype=np.float32)

    # your full history
    all_labels = [
        'Blue Object', 'Red Object', 'Red Object', 'Red Object', 'Red Object',
        'Red Object', 'Green Object', 'Red Object', 'Green Object', 'Blue Object',
        'Red Object', 'Green Object', 'Blue Object', 'Red Object', 'Green Object',
        'Blue Object', 'Red Object', 'Green Object', 'Blue Object', 'Red Object',
        'Green Object', 'Blue Object', 'Red Object', 'Green Object', 'Blue Object',
        'Red Object', 'Green Object', 'Blue Object', 'Red Object', 'Green Object',
        'Blue Object', 'Red Object', 'Green Object', 'Blue Object', 'Red Object',
        'Green Object', 'Blue Object', 'Red Object', 'Green Object', 'Blue Object',
        'Red Object', 'Green Object', 'Blue Object', 'Red Object', 'Green Object',
        'Blue Object'
    ]
    all_boxes = [
        (456, 799, 422, 161), (764, 0,   516, 242), (279, 833, 546, 127),
        (473,   0, 512, 251), (0,   389, 413, 486), (601, 350, 291, 288),
        (624, 709, 297, 251), (214, 291, 260, 215), (537, 482, 177, 204),
        (813, 267, 234, 182), (217, 557, 243, 227), (539, 623, 190, 221),
        (721, 367, 164, 130), (380, 529, 175, 172), (620, 581, 159, 177),
        (775, 343, 154, 125), (372, 488, 138, 124), (543, 596, 117, 139),
        (742, 455, 137, 115), (394, 396, 129, 115), (432, 563, 123, 126),
        (668, 578, 122, 129), (484, 317,  98, 100), (393, 447, 110,  93),
        (533, 599,  88, 103), (518, 292,  75,  84), (397, 373,  89,  75),
        (449, 540,  99, 101), (502, 313,  70,  76), (389, 394,  83,  68),
        (445, 549,  94,  96), (474, 358,  79,  78), (385, 458,  90,  75),
        (486, 593,  81,  89), (480, 383,  83,  80), (411, 492,  91,  79),
        (534, 613,  70,  83), (514, 389,  79,  78), (447, 495,  86,  77),
        (565, 614,  68,  82), (549, 388,  72,  73), (472, 485,  80,  71),
        (568, 610,  73,  85), (566, 387,  68,  70), (483, 477,  76,  66),
        (564, 605,  75,  87)
    ]

    # pick only the last 5 detections
    detection_labels = all_labels[-10:]
    detection_coords = all_boxes[-10:]

    final_locations = estimate_object_locations(
        detection_coords,
        detection_labels,
        K,
        eps=0.05,
        min_samples=2
    )

    # Print out the results:
    for label, coords in final_locations.items():
        print(f"\nLabel: {label}")
        for i, (x, y, z) in enumerate(coords, 1):
            print(f"  Cluster {i}:  x={x:.4f}, y={y:.4f}, z={z:.4f}")
