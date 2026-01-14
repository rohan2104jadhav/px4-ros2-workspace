#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 Message Imports
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition


class OffboardMission(Node):

    def __init__(self):
        super().__init__('offboard_mission_node')

        # --- QoS Configuration ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', qos_profile)
        
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_profile)
        
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', qos_profile)

        # --- Subscribers ---
        self.vehicle_status_subscriber_ = self.create_subscription(
            VehicleStatus, 'fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Added: Local Position to check where we are (for state switching)
        self.vehicle_local_position_subscriber_ = self.create_subscription(
            VehicleLocalPosition, 'fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # --- Mission Constants ---
        self.TAKEOFF_ALTITUDE = -20.0  # Meters (Negative Z is Up)
        self.CIRCLE_RADIUS = 15.0      # Meters
        self.CIRCLE_SPEED = 0.05       # Radians per tick (Speed of orbit)
        self.ACCEPTANCE_RADIUS = 1.0   # How close we need to be to a waypoint to switch states

        # --- Variables ---
        self.offboard_setpoint_counter_ = 0
        self.vehicle_nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.vehicle_arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Current Position
        self.current_position = [0.0, 0.0, 0.0] # X, Y, Z

        # State Machine Variables
        self.mission_state = "WARMUP" 
        self.theta = 0.0 # Angle for circle trajectory

        # --- Timer ---
        self.timer_period = 0.02 # 50Hz
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)


    # --- Callbacks ---

    def vehicle_status_callback(self, msg):
        self.vehicle_nav_state = msg.nav_state
        self.vehicle_arming_state = msg.arming_state

    def vehicle_local_position_callback(self, msg):
        # Update current position (NED Frame)
        self.current_position = [msg.x, msg.y, msg.z]


    # --- Helper Functions ---

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=float('nan')):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.trajectory_setpoint_publisher_.publish(msg)

    def distance_to_target(self, target_pos):
        # Calculate Euclidean distance between current position and target (3D)
        dx = target_pos[0] - self.current_position[0]
        dy = target_pos[1] - self.current_position[1]
        dz = target_pos[2] - self.current_position[2]
        return np.sqrt(dx**2 + dy**2 + dz**2)


    # --- Main Loop ---

    def timer_callback(self):
        # 1. Always publish Heartbeat
        self.publish_offboard_control_mode()

        # 2. Main Mission Logic
        if self.mission_state == "WARMUP":
            # Send initial setpoints to prepare PX4 for Offboard
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            
            self.offboard_setpoint_counter_ += 1
            
            if self.offboard_setpoint_counter_ == 10:
                # Switch to Offboard
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.get_logger().info("Mode: OFFBOARD")

            if self.offboard_setpoint_counter_ == 20:
                # Arm
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.get_logger().info("Status: ARMED")
                self.mission_state = "TAKEOFF"

        elif self.mission_state == "TAKEOFF":
            # Ascend to 20m
            target = [0.0, 0.0, self.TAKEOFF_ALTITUDE]
            self.publish_trajectory_setpoint(target[0], target[1], target[2])

            if self.distance_to_target(target) < self.ACCEPTANCE_RADIUS:
                self.get_logger().info("Altitude Reached. Moving to Start of Circle.")
                self.mission_state = "MOVE_TO_CIRCLE_START"

        elif self.mission_state == "MOVE_TO_CIRCLE_START":
            # Move to (radius, 0, -20) to start the circle smoothly
            target = [self.CIRCLE_RADIUS, 0.0, self.TAKEOFF_ALTITUDE]
            self.publish_trajectory_setpoint(target[0], target[1], target[2])

            if self.distance_to_target(target) < self.ACCEPTANCE_RADIUS:
                self.get_logger().info("Starting Circular Trajectory.")
                self.mission_state = "CIRCLE"
                self.theta = 0.0

        elif self.mission_state == "CIRCLE":
            # Calculate new X, Y based on Theta
            # Using Parametric equation of circle: x = r*cos(t), y = r*sin(t)
            x = self.CIRCLE_RADIUS * np.cos(self.theta)
            y = self.CIRCLE_RADIUS * np.sin(self.theta)
            z = self.TAKEOFF_ALTITUDE

            # Yaw towards the direction of travel (tangent to circle)
            # Yaw = theta + 90 degrees (pi/2)
            yaw = self.theta + (np.pi / 2)

            self.publish_trajectory_setpoint(x, y, z, yaw)

            # Increment angle
            self.theta += self.CIRCLE_SPEED

            # Check if circle complete (2*PI)
            if self.theta >= 2 * np.pi:
                self.get_logger().info("Circle Complete. Returning Home.")
                self.mission_state = "RETURN_HOME"

        elif self.mission_state == "RETURN_HOME":
            # Fly back to (0, 0, -20)
            target = [0.0, 0.0, self.TAKEOFF_ALTITUDE]
            self.publish_trajectory_setpoint(target[0], target[1], target[2])

            if self.distance_to_target(target) < self.ACCEPTANCE_RADIUS:
                self.get_logger().info("Home Reached. Landing.")
                self.mission_state = "LAND"

        elif self.mission_state == "LAND":
            # Send Land Command
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
            # Stop publishing setpoints, but keep Heartbeat alive until landed
            # Once landed, PX4 will auto-disarm.
            if self.vehicle_arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info("Vehicle Disarmed. Mission Complete.")
                self.destroy_node()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    offboard_mission = OffboardMission()

    try:
        rclpy.spin(offboard_mission)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure clean shutdown
        if rclpy.ok():
            offboard_mission.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()