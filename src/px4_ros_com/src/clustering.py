#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN
from oa_msgs.msg import NearestObstacleState
# Constants
DBSCAN_EPS = 0.5  # Maximum distance between two samples for them to be considered as in the same neighborhood
DBSCAN_MIN_SAMPLES = 3  # Minimum number of points to form a cluster
TRACKING_DISTANCE_THRESHOLD = 2  # Maximum distance to associate an object between frames
MINIMUM_KEEP_OUT_DISTANCE = 2  # Minimum distance to consider an object (ignore closer objects)
class LidarTracker(Node):
    def __init__(self):
        super().__init__('lidar_tracker')
        # Configure QoS profile for LiDAR data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Subscriber for LiDAR data
        self.lidar_subscriber = self.create_subscription(
            PointCloud2, '/lidar/pointcloud', self.lidar_callback, qos_profile
        )
        # Publisher for nearest obstacle state
        self.nearest_obstacle_state_publisher = self.create_publisher(
            NearestObstacleState, 'nearest_obstacle_state', qos_profile
        )
        # Tracked objects
        self.tracked_objects = []
        self.current_time = 0.0  # Simulated time (replace with actual timestamp if available)
        # Placeholder for the nearest obstacle state
        self.nearest_obstacle_state = NearestObstacleState()
    def lidar_callback(self, msg):
        """Callback function for LiDAR data."""
        # Convert PointCloud2 message to numpy array
        self.get_logger().info("LiDAR callback triggered!")
        lidar_data = self.pointcloud2_to_array(msg)
        self.get_logger().info(f"Raw data shape: {lidar_data.shape}")
        # First apply DBSCAN clustering to all points
        dbscan = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES)
        labels = dbscan.fit_predict(lidar_data)
        # Separate clusters and calculate their centroids
        unique_labels = set(labels)
        unique_labels.discard(-1)  # Remove noise points
        clusters = [lidar_data[labels == label] for label in unique_labels]
        centroids = [np.mean(cluster, axis=0) for cluster in clusters]
        # Filter clusters based on centroid distance
        valid_clusters = []
        for cluster, centroid in zip(clusters, centroids):
            centroid_distance = np.linalg.norm(centroid)
            if centroid_distance > MINIMUM_KEEP_OUT_DISTANCE:
                valid_clusters.append(cluster)
                self.get_logger().info(f"Keeping cluster with centroid at {centroid} (distance: {centroid_distance:.2f}m)")
            else:
                self.get_logger().info(f"Ignoring cluster with centroid at {centroid} (distance: {centroid_distance:.2f}m)")
        if len(valid_clusters) == 0:
            self.get_logger().info("No valid clusters beyond keep-out distance")
            self.tracked_objects = []  # Clear tracked objects if nothing is detected
            return
        # Track objects across frames
        self.tracked_objects = self.track_objects(self.tracked_objects, valid_clusters)
        # Find and update the nearest object's state
        self.update_nearest_obstacle_state()
        # Log tracked objects
        self.get_logger().info(f"Tracked objects: {len(self.tracked_objects)}")
        for obj in self.tracked_objects:
            self.get_logger().info(
                f"Object ID: {obj.id}, Position: {obj.position}, Velocity: {obj.velocity}"
            )
        # Increment time (simulated)
        self.current_time += 0.1
    def pointcloud2_to_array(self, msg):
        """Convert a PointCloud2 message to a numpy array."""
        # Extract points from the PointCloud2 message
        points = np.frombuffer(msg.data, dtype=np.float32)
        points = points.reshape(-1, 4)  # Assuming x, y, z, intensity
        return points[:, :3]  # Return only x, y, z
    def track_objects(self, previous_tracked_objects, current_clusters):
        """Track objects using linear velocity prediction and nearest neighbor matching."""
        if not previous_tracked_objects:
            return [TrackedObject(cluster, i) for i, cluster in enumerate(current_clusters)]
        current_tracked_objects = []
        used_clusters = set()
        # Predict positions for existing objects
        for prev_obj in previous_tracked_objects:
            prev_obj.predict()
        # Match objects
        for prev_obj in previous_tracked_objects:
            best_match = None
            min_distance = float('inf')
            for j, current_cluster in enumerate(current_clusters):
                if j in used_clusters:
                    continue
                # Calculate centroid distance
                current_centroid = np.mean(current_cluster, axis=0)
                centroid_distance = np.linalg.norm(current_centroid - prev_obj.position)
                # Match if within threshold
                if centroid_distance < TRACKING_DISTANCE_THRESHOLD:
                    if centroid_distance < min_distance:
                        best_match = (j, current_cluster)
                        min_distance = centroid_distance
            if best_match is not None:
                # Update existing object
                prev_obj.update(best_match[1])
                current_tracked_objects.append(prev_obj)
                used_clusters.add(best_match[0])
        # Add remaining unmatched clusters as new objects
        next_id = max([obj.id for obj in current_tracked_objects], default=-1) + 1
        for j, cluster in enumerate(current_clusters):
            if j not in used_clusters:
                new_obj = TrackedObject(cluster, next_id)
                current_tracked_objects.append(new_obj)
                next_id += 1
        return current_tracked_objects
    def update_nearest_obstacle_state(self):
        """Update the nearest obstacle state."""
        if not self.tracked_objects:
            # If no tracked objects, publish a default state or don't publish
            self.nearest_obstacle_state.r_pz = 0.0
            self.nearest_obstacle_state.x_io = [0.0, 0.0, 0.0]
            self.nearest_obstacle_state.v_io = [0.0, 0.0, 0.0]
            self.nearest_obstacle_state_publisher.publish(self.nearest_obstacle_state)
            self.get_logger().info("No tracked objects beyond keep-out distance")
            return
        # Find the nearest object beyond the keep-out distance
        nearest_object = min(self.tracked_objects, key=lambda obj: np.linalg.norm(obj.position))
        # Update the nearest obstacle state
        self.nearest_obstacle_state.r_pz = float(nearest_object.position[2])
        self.nearest_obstacle_state.x_io = [float(val) for val in nearest_object.position.tolist()]
        self.nearest_obstacle_state.v_io = [float(val) for val in nearest_object.velocity.tolist()]
        self.nearest_obstacle_state_publisher.publish(self.nearest_obstacle_state)
        self.get_logger().info(f"Nearest Obstacle State - r_pz: {self.nearest_obstacle_state.r_pz}, x_io: {self.nearest_obstacle_state.x_io}, v_io: {self.nearest_obstacle_state.v_io}")
class TrackedObject:
    def __init__(self, initial_cluster_points, object_id):
        self.id = object_id
        self.position = np.mean(initial_cluster_points, axis=0)  # Initial position
        self.velocity = np.zeros(3)  # Initial velocity
        self.points = initial_cluster_points
    def predict(self):
        """Predict next position using linear velocity."""
        self.position += self.velocity * 0.1  # Assuming 0.1s time step
    def update(self, new_cluster_points):
        """Update object state with new measurement."""
        new_centroid = np.mean(new_cluster_points, axis=0)
        self.velocity = (new_centroid - self.position) / 0.1  # Update velocity
        self.position = new_centroid
        self.points = new_cluster_points
def main(args=None):
    rclpy.init(args=args)
    lidar_tracker = LidarTracker()
    rclpy.spin(lidar_tracker)
    lidar_tracker.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()