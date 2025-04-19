#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
import numpy as np
from oa_msgs.msg import NearestObstacleState, CollisionData
import math

class Avoidance(Node):
    
    def __init__(self):
        super().__init__('obstacle_avoidan')
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # create subscribers from built-ins
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.nearest_obstacle_state_subscriber = self.create_subscription(
            NearestObstacleState, 'nearest_obstacle_state', self.nearest_obstacle_state_callback, qos_profile)
        self.trajectory_setpoint_subscriber = self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.trajectory_setpoint_callback, qos_profile)
        self.offboard_control_mode_subscriber = self.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.offboard_control_mode_callback, qos_profile)
        
        
        # create publishers/new topics
        self.collision_data_publisher = self.create_publisher(
            CollisionData, 'collision_data', qos_profile
        )
        
        
        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_attitude = VehicleAttitude()
        self.nearest_obstacle_state = NearestObstacleState()
        self.collision_data = CollisionData()
        self.trajectory_setpoint = TrajectorySetpoint()
        self.V_avoid = np.array([])
        
        # create timer 
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status 
        
    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_attitude = vehicle_attitude
        
    def nearest_obstacle_state_callback(self, nearest_obstacle_state):
        self.nearest_obstacle_state = nearest_obstacle_state
        
    def trajectory_setpoint_callback(self, trajectory_setpoint):
        self.trajectory_setpoint = trajectory_setpoint
        self.next_pos_sp = trajectory_setpoint.position
        # self.get_logger().info(f'{self.next_pos_sp}')
        
    def offboard_control_mode_callback(self, offboard_control_mode):
        self.offboard_control_mode = offboard_control_mode
        self.p_controller_on = offboard_control_mode.position
        
    def quaternion_to_dcm(self, q):
        # q comes as (w,x,y,z)
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]
        q4 = q[0]
        
        a11 = q1**2 - q2**2 - q3**2 + q4**2
        a12 = 2 * ( q1*q2 + q3*q4 )
        a13 = 2 * ( q1*q3 - q2*q4 )
        a21 = 2 * ( q1*q2 - q3*q4 )
        a22 = q2**2 - q1**2 - q3**2 + q4**2
        a23 = 2 * ( q2*q3 + q1*q4 )
        a31 = 2 * ( q1*q3 + q2*q4 )
        a32 = 2 * ( q2*q3 - q1*q4 )
        a33 = q3**2 - q1**2 - q2**2 + q4**2
        dcm = np.array([[a11, a12, a13],
                        [a21, a22, a23],
                        [a31, a32, a33]])
        return dcm
            
    def Obstacle_Avoidance(
        self,
        r_pz,
        X_io,
        V_io,
        Xo,
        Vo,
        DCM
        ):
        
        collision_imminent = False
        LOS_clear = True
        V_avoid = np.array([0,0,0])
        # self.get_logger().info(f'{DCM}')
        
        if (np.linalg.norm(X_io) != 0 and
            np.linalg.norm(X_io) < 15 and 
            all(item is not None and (not isinstance(item, float) or not math.isnan(item)) for item in X_io) and
            all(item is not None and (not isinstance(item, float) or not math.isnan(item)) for item in V_io)): 
            
           
            Vo_mag = np.linalg.norm(Vo)
            r_pz = 2*r_pz
            # self.get_logger().info(f'{r_pz}')
            # self.get_logger().info(f'PRE DCM: X_io: {X_io}, V_io: {V_io} V_io_mag: {np.linalg.norm(V_io)}')
            # self.get_logger().info(f'Xo: {Xo} Vo: {Vo}')
            # self.get_logger().info(f'{DCM}')
            # Apply the DCM to X_io and V_io in homogeneous coordinates
            X_io = np.matmul(DCM, X_io)
            # X_io = [-1*X_io[0], X_io[1], X_io[2]]
            # V_io = np.matmul(DCM, V_io)
            V_io = -1*Vo
            # self.get_logger().info(f'POST DCM: X_io: {X_io}, V_io: {V_io}')
            
            Vi = V_io + Vo
            # self.get_logger().info(f'PRE DCM: X_io: {X_io}, V_io: {V_io} V_io_mag: {np.linalg.norm(V_io)}')

            
            V_rel = -1*V_io
            
            V_rel_mag = np.linalg.norm(V_rel)
            
            # here "oi" is copied from the article and means "o to i"
            D_oi = X_io
            d_oi = np.linalg.norm(D_oi)
            # print(d_oi)
            
            if d_oi < r_pz:
                r_pz = 0.95 * d_oi
            
            # radis of B_vo: the base of the VO cone
            r_vo = (r_pz/d_oi)*np.sqrt(d_oi**2 - r_pz**2)
            
            # distance from cone apex to base
            d_vo = (d_oi**2 - r_pz**2)/d_oi
            
            
            # cone frame creation
            X_cone = X_io / np.linalg.norm(X_io)
            z_inertial = np.array([0,0,1])
            Y_cone = np.cross(z_inertial,X_cone)
            Y_cone = Y_cone / np.linalg.norm(Y_cone)
            Z_cone = np.cross(X_cone, Y_cone)    
            
            
            R = np.array([X_cone,
                        Y_cone,
                        Z_cone])
            
            # V_REL CALCULATIONS
            V_rel_cone = np.matmul(R,V_rel)
            V_rel_cone_x = V_rel_cone[0]
            V_rel_cone_y = V_rel_cone[1]
            V_rel_cone_z = V_rel_cone[2]
            if V_rel_cone_x > 0 and (1/V_rel_cone_x)*np.sqrt(V_rel_cone_y**2 + V_rel_cone_z**2) < (r_vo/d_vo):
                collision_imminent = True
                # self.get_logger().info('AVOID AVOID AVOID AVOID')
                theta_yz = np.arctan2(V_rel_cone_z, V_rel_cone_y)
                cone_r_local = V_rel_cone_x*(r_vo/d_vo)
                V_rel_cone_z_new = cone_r_local * np.sin(theta_yz)
                V_rel_cone_y_new = cone_r_local * np.cos(theta_yz)
                V_rel_cone_new = np.array([V_rel_cone_x, V_rel_cone_y_new, V_rel_cone_z_new])
                V_rel_new = np.linalg.solve(R,V_rel_cone_new)
                V_avoid = V_rel_new + Vi
                self.get_logger().info(f'{V_avoid}')
               
               
            # NEXT POSITION SETPOINT LINE OF SIGHT CALCULATIONS
            if  all(item is not None and (not isinstance(item, float) or not math.isnan(item)) for item in self.next_pos_sp):
                next_pos = np.array(self.next_pos_sp)
                next_pos_rel = next_pos - Xo
                next_pos_rel_cone = np.matmul(R,next_pos_rel)
                next_pos_rel_cone_x = next_pos_rel_cone[0]
                next_pos_rel_cone_y = next_pos_rel_cone[1]
                next_pos_rel_cone_z = next_pos_rel_cone[2]
                if next_pos_rel_cone_x > 0 and (1/next_pos_rel_cone_x)*np.sqrt(next_pos_rel_cone_y**2 + next_pos_rel_cone_z**2) < (r_vo/d_vo):
                    LOS_clear = False
                    self.get_logger().info('NOT CLEAR')
                else:
                    self.get_logger().info('CLEAR')
            

        return collision_imminent, V_avoid, LOS_clear
    
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # velocities in NED
        Vox = self.vehicle_local_position.vx
        Voy = self.vehicle_local_position.vy
        Voz = self.vehicle_local_position.vz
        Xox = self.vehicle_local_position.x
        Xoy = self.vehicle_local_position.y
        Xoz = self.vehicle_local_position.z
        
        Vo = np.array([Vox, Voy, Voz])
        Xo = np.array([Xox, Xoy, Xoz])
        
        #get X_io, V_io, r_pz of nearest obstacle from Lidar/clustering node
        # note becuase these are raw from lidar, they will be in body frame
        # will convert to NED for apples to apples
        X_io = self.nearest_obstacle_state.x_io
        V_io = self.nearest_obstacle_state.v_io
        r_pz = self.nearest_obstacle_state.r_pz
        
        quat = self.vehicle_attitude.q
        DCM = self.quaternion_to_dcm(quat)
        collision_immient, V_avoid, los_clear =  self.Obstacle_Avoidance(
            r_pz,
            X_io,
            V_io,
            Xo,
            Vo,
            DCM
        )
        
        #publish collision_imminent, V_avoid
        self.collision_data.collision_imminent = collision_immient
        self.collision_data.v_avoid = [float(V_avoid[0]), float(V_avoid[1]), float(V_avoid[2])]
        self.collision_data.los_clear = los_clear
        self.collision_data_publisher.publish(self.collision_data)
        # if self.collision_data.collision_imminent:
        #     self.get_logger().info('COLLISION IMMINENT')
        # else:
        #     self.get_logget().info('-------')
        
def main(args=None) -> None:
    # print('Starting avoidance node...')
    rclpy.init(args=args)
    offboard_control = Avoidance()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()
    # print('AVOIDANCE QUIT')


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)