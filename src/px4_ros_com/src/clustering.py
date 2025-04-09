#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
import numpy as np
from oa_msgs.msg import NearestObstacleState, CollisionData

class Clustering(Node):
    
    def __init__(self):
        super().__init__('clustering')
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.vehicle_local_position_subscriber = self.create_subscription(
                    VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        self.nearest_obstacle_state_publisher = self.create_publisher(
            NearestObstacleState, 'nearest_obstacle_state', qos_profile
        )
        
        # Initialize variables
        self.nearest_obstacle_state = NearestObstacleState()
        self.vehicle_local_position = VehicleLocalPosition()
        
        # create timer 
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    
        
    
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        t = self.vehicle_local_position.timestamp
        Xo_x = self.vehicle_local_position.x
        Xo_y = self.vehicle_local_position.y
        Xo_z = self.vehicle_local_position.z
        Vo_x = self.vehicle_local_position.vx
        Vo_y = self.vehicle_local_position.vy
        Vo_z = self.vehicle_local_position.vz
        X_i = [-35.5,0,-10]
        V_i = [0,0,0]
        
        r_pz = 10.0
        X_io_x = X_i[0] - Xo_x
        X_io_y = X_i[1] - Xo_y
        X_io_z = X_i[2] - Xo_z
        V_io_x = V_i[0] - Vo_x
        V_io_y = V_i[1] - Vo_y
        V_io_z = V_i[2] - Vo_z
        X_io = [X_io_x, X_io_y, X_io_z]
        V_io = [V_io_x, V_io_y, V_io_z]
        
        self.nearest_obstacle_state.r_pz = float(r_pz)
        self.nearest_obstacle_state.x_io = [float(val) for val in X_io]  # Ensure all values are floats
        self.nearest_obstacle_state.v_io = [float(val) for val in V_io]  # Ensure all values are floats
        
        
        self.nearest_obstacle_state_publisher.publish(self.nearest_obstacle_state)
        # self.get_logger().info(f"{self.nearest_obstacle_state.x_io}")
    
        
def main(args=None) -> None:
    print('Starting clustering node...')
    rclpy.init(args=args)
    cluster_node = Clustering()
    rclpy.spin(cluster_node)
    cluster_node.destroy_node()
    rclpy.shutdown()
    print('CLUSTERING QUIT')


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)