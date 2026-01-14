#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 Message Imports
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand


class OffboardTakeoff(Node):

    def __init__(self):
        super().__init__('offboard_takeoff')

        # --- QoS Configuration ---
        # Matches the "Best Effort" reliability used by PX4 uXRCE-DDS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---
        # 1. Heartbeat to keep Offboard mode active
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', qos_profile)
        
        # 2. Position Setpoints (Where to go)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_profile)
        
        # 3. Vehicle Commands (To Arm and Change Mode)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', qos_profile)

        # --- Subscribers ---
        # Monitor the drone status (Armed? Offboard?)
        self.vehicle_status_subscriber_ = self.create_subscription(
            VehicleStatus, 'fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # --- Variables ---
        self.offboard_setpoint_counter_ = 0
        self.vehicle_nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.vehicle_arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Target Altitude (NED Frame: Negative Z is Up)
        self.takeoff_height = -10.0 

        # --- Timer ---
        # Run at 50Hz (0.02s)
        self.timer_period = 0.02
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)


    def vehicle_status_callback(self, msg):
        self.vehicle_nav_state = msg.nav_state
        self.vehicle_arming_state = msg.arming_state


    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Helper function to send commands (like ARM or change MODE)"""
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


    def timer_callback(self):
        # 1. Publish Offboard Control Mode Heartbeat
        # We must send this continuously (at least 2Hz) or PX4 triggers failsafe.
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.offboard_control_mode_publisher_.publish(offboard_msg)


        # 2. Publish Trajectory Setpoint (Hold Position)
        # We publish this *always*, even before arming, so the drone knows where to go immediately.
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # NED Coordinates: (North, East, Down)
        # x=0, y=0, z=-10.0 (10 meters up)
        trajectory_msg.position[0] = 0.0
        trajectory_msg.position[1] = 0.0
        trajectory_msg.position[2] = self.takeoff_height 
        trajectory_msg.yaw = float('nan') # Keep current heading

        self.trajectory_setpoint_publisher_.publish(trajectory_msg)


        # 3. State Machine (Switch to Offboard -> Arm)
        # We use a counter to ensure we have sent enough setpoints before asking to switch modes.
        if self.offboard_setpoint_counter_ == 10:
            # Step 1: Switch to Offboard Mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Switching to OFFBOARD mode...")

        if self.offboard_setpoint_counter_ == 20:
            # Step 2: Arm the drone
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Arming vehicle...")

        # Increment counter (stops at 21 to prevent overflow, logic is done)
        if self.offboard_setpoint_counter_ < 21:
            self.offboard_setpoint_counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    offboard_takeoff = OffboardTakeoff()

    try:
        rclpy.spin(offboard_takeoff)
    except KeyboardInterrupt:
        pass
    finally:
        offboard_takeoff.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()