#!/usr/bin/env python3
"""
HW5 pose logger node.

Subscribes to a PoseStamped topic (e.g., /slam_pose from HW3 EKF) and
logs the time, (x, y), and yaw into a JSON file:

  hw5_slam_trajectory_YYYYMMDD-HHMMSS.json

in a configurable log directory (default: current working directory or
as set by the 'log_dir' parameter).

Usage (via launch):
  - set parameters:
      pose_topic: which PoseStamped topic to listen to (default: /slam_pose)
      log_dir: directory where the JSON file will be saved
"""

import json
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def quat_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw (Z yaw in radians)."""
    # standard z-yaw extraction
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class Hw5PoseLoggerNode(Node):
    def __init__(self):
        super().__init__("hw5_pose_logger")

        # Parameters
        self.declare_parameter("pose_topic", "/slam_pose")
        self.declare_parameter("log_dir", ".")

        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.log_dir = self.get_parameter("log_dir").get_parameter_value().string_value

        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir, exist_ok=True)

        self.get_logger().info(
            f"HW5 pose logger listening to '{pose_topic}', "
            f"logging to '{self.log_dir}'"
        )

        # Internal buffer of samples
        self.trajectory = []

        # Subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            10,
        )

    def pose_callback(self, msg: PoseStamped):
        """Store each incoming pose sample."""
        # Use header timestamp as the time
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        x = msg.pose.position.x
        y = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = quat_to_yaw(qx, qy, qz, qw)

        self.trajectory.append(
            {
                "time": float(t),
                "x": float(x),
                "y": float(y),
                "theta": float(yaw),
                "frame_id": msg.header.frame_id,
            }
        )

    def save_trajectory(self):
        """Write the collected samples to a JSON file on shutdown."""
        if not self.trajectory:
            self.get_logger().warn("No pose samples collected; not writing log.")
            return

        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = f"hw5_slam_trajectory_{ts}.json"
        out_path = os.path.join(self.log_dir, filename)

        try:
            with open(out_path, "w") as f:
                json.dump(self.trajectory, f, indent=2)
            self.get_logger().info(
                f"Saved {len(self.trajectory)} pose samples to {out_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to save pose log to {out_path}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Hw5PoseLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_trajectory()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
