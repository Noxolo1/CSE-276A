#!/usr/bin/env python3
"""
Standalone waypoint follower node for HW4.

This node:
1. Loads waypoints from a JSON file
2. Uses AprilTag localization for positioning
3. Implements a simple controller to follow waypoints
4. Publishes Twist messages to /cmd_vel
5. Works with your existing velocity_mapping and motor_control nodes

Usage:
    ros2 run hw4_planning waypoint_follower_node --ros-args -p waypoints_file:=path/to/waypoints.json
"""

import json
import math
from typing import List, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import String


class WaypointFollowerNode(Node):
    """
    Follows a pre-planned path from a JSON waypoint file.
    
    Expected JSON format:
    [
        {"index": 0, "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"index": 1, "x": 0.5, "y": 0.5, "yaw": 1.57},
        ...
    ]
    """

    def __init__(self):
        super().__init__('waypoint_follower_node')
        
        # Parameters
        self.declare_parameter('waypoints_file', 'hw4_waypoints_safety.json')
        self.declare_parameter('position_tolerance', 0.05)  # meters
        self.declare_parameter('angle_tolerance', 0.15)     # radians (~8.6 degrees)
        self.declare_parameter('max_linear_velocity', 0.2)  # m/s
        self.declare_parameter('max_angular_velocity', 0.8) # rad/s
        self.declare_parameter('lookahead_distance', 0.15)  # meters (for pure pursuit)
        
        waypoints_file = self.get_parameter('waypoints_file').value
        self.position_tol = self.get_parameter('position_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        
        # Control gains
        self.k_linear = 1.0      # Proportional gain for linear velocity
        self.k_angular = 2.0     # Proportional gain for angular velocity
        
        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.localized = False
        
        self.waypoints: List[Tuple[float, float, float]] = []  # (x, y, yaw)
        self.current_waypoint_idx = 0
        self.stage = "rotate_to_goal"  # "rotate_to_goal", "drive", "rotate_at_goal", "done"
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/waypoint_follower/status', 10)
        
        # Subscribe to localization (assumes you have a localization node publishing PoseArray)
        # Adjust topic name if needed
        self.create_subscription(
            PoseArray,
            '/apriltag_poses',
            self.localization_callback,
            10
        )
        
        # Control timer (20 Hz)
        self.create_timer(0.05, self.control_loop)
        
        # Load waypoints
        self.load_waypoints(waypoints_file)
        
        self.get_logger().info(f'Waypoint follower started with {len(self.waypoints)} waypoints')
        self.get_logger().info(f'Tolerances: pos={self.position_tol}m, angle={self.angle_tol}rad')

    def load_waypoints(self, filename: str) -> None:
        """Load waypoints from JSON file."""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.waypoints = [(wp['x'], wp['y'], wp['yaw']) for wp in data]
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            self.waypoints = []

    def localization_callback(self, msg: PoseArray) -> None:
        """
        Update robot position from localization.
        
        This assumes your localization publishes the robot pose in the map frame.
        Adjust this callback based on your actual localization topic/message type.
        """
        if len(msg.poses) == 0:
            return
        
        # Use the first pose (assumes single robot)
        pose = msg.poses[0]
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y
        
        # Convert quaternion to yaw
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        self.robot_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy**2 + qz**2)
        )
        
        self.localized = True

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_target_waypoint(self) -> Tuple[float, float, float]:
        """Get the current target waypoint."""
        if self.current_waypoint_idx >= len(self.waypoints):
            return self.waypoints[-1] if self.waypoints else (0, 0, 0)
        return self.waypoints[self.current_waypoint_idx]

    def distance_to_waypoint(self, target_x: float, target_y: float) -> float:
        """Calculate Euclidean distance to target."""
        return math.hypot(target_x - self.robot_x, target_y - self.robot_y)

    def angle_to_waypoint(self, target_x: float, target_y: float) -> float:
        """Calculate angle to target waypoint."""
        return math.atan2(target_y - self.robot_y, target_x - self.robot_x)

    def control_loop(self) -> None:
        """Main control loop - called at 20 Hz."""
        # Wait for localization
        if not self.localized or len(self.waypoints) == 0:
            return
        
        # Check if we're done
        if self.stage == "done":
            self.stop_robot()
            return
        
        # Get current target
        target_x, target_y, target_yaw = self.get_target_waypoint()
        
        # Calculate errors
        distance = self.distance_to_waypoint(target_x, target_y)
        angle_to_target = self.angle_to_waypoint(target_x, target_y)
        heading_error = self.normalize_angle(angle_to_target - self.robot_yaw)
        final_heading_error = self.normalize_angle(target_yaw - self.robot_yaw)
        
        # State machine for waypoint following
        if self.stage == "rotate_to_goal":
            # Rotate to face the target waypoint
            if abs(heading_error) < self.angle_tol:
                self.stage = "drive"
                self.get_logger().info(f'Waypoint {self.current_waypoint_idx}: Aligned, starting drive')
                self.stop_robot()
            else:
                # Rotate in place
                angular_vel = np.clip(
                    self.k_angular * heading_error,
                    -self.max_angular_vel,
                    self.max_angular_vel
                )
                self.publish_velocity(0.0, angular_vel)
        
        elif self.stage == "drive":
            # Drive toward the waypoint while maintaining heading
            if distance < self.position_tol:
                # Reached waypoint
                self.stage = "rotate_at_goal"
                self.get_logger().info(f'Waypoint {self.current_waypoint_idx}: Reached position')
                self.stop_robot()
            else:
                # Pure pursuit: adjust heading while driving
                linear_vel = np.clip(
                    self.k_linear * distance,
                    0.0,
                    self.max_linear_vel
                )
                angular_vel = np.clip(
                    self.k_angular * heading_error,
                    -self.max_angular_vel,
                    self.max_angular_vel
                )
                self.publish_velocity(linear_vel, angular_vel)
        
        elif self.stage == "rotate_at_goal":
            # Rotate to final orientation at waypoint
            if abs(final_heading_error) < self.angle_tol:
                # Move to next waypoint
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx >= len(self.waypoints):
                    self.stage = "done"
                    self.get_logger().info('All waypoints reached! Mission complete.')
                    self.publish_status('COMPLETE')
                    self.stop_robot()
                else:
                    self.stage = "rotate_to_goal"
                    self.get_logger().info(
                        f'Waypoint {self.current_waypoint_idx - 1}: Complete. '
                        f'Moving to waypoint {self.current_waypoint_idx}'
                    )
            else:
                # Rotate to final heading
                angular_vel = np.clip(
                    self.k_angular * final_heading_error,
                    -self.max_angular_vel,
                    self.max_angular_vel
                )
                self.publish_velocity(0.0, angular_vel)

    def publish_velocity(self, linear: float, angular: float) -> None:
        """Publish velocity command."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self) -> None:
        """Send zero velocity command."""
        self.publish_velocity(0.0, 0.0)

    def publish_status(self, status: str) -> None:
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Waypoint follower stopped by keyboard interrupt')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
