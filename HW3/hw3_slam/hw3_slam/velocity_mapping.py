#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np


class VelocityToMotorNode(Node):
    def __init__(self):
        super().__init__('hw3_velocity_mapping')

        # Robot geometry
        self.wheel_base = 0.127  # [m]

        # Max command (not enforced in original hw2; kept for compatibility)
        self.cmd_max = 1.5

        # ---------- Linear mapping ----------
        # These are from the instructor example. Tune if needed.
        self.left_linear_deadzone = 0.09
        self.left_linear_slope = 2.5
        self.right_linear_deadzone = 0.09
        self.right_linear_slope = 2.5

        # ---------- Angular mapping ----------
        self.left_angular_deadzone = 0.245
        self.left_angular_slope = 16.0
        self.right_angular_deadzone = 0.24
        self.right_angular_slope = 16.0

        # Subscriptions / publishers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_commands', 10)

        self.get_logger().info('HW3 velocity_mapping node started (HW2-style mapping, no min-power clamp)')

    def _wheel_velocities(self, linear: float, angular: float) -> tuple[float, float]:
        """Compute left/right wheel linear velocities from (v, w)."""
        half_b = 0.5 * self.wheel_base
        v_left = linear - half_b * angular
        v_right = linear + half_b * angular
        return v_left, v_right

    def _map_with_deadzone(self, value: float, deadzone: float, slope: float) -> float:
        """
        Map desired wheel velocity to motor command with deadzone:
            cmd = sign(v) * (deadzone + |v| / slope)
        This matches the instructor's HW2 implementation.
        """
        if abs(value) < 1e-9:
            return 0.0
        cmd_mag = deadzone + abs(value) / max(slope, 1e-9)
        return float(np.copysign(cmd_mag, value))

    def cmd_vel_callback(self, msg: Twist):
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)

        v_left, v_right = self._wheel_velocities(lin, ang)

        if lin != 0.0:
            # Use linear mapping when there's a linear component
            left_cmd = self._map_with_deadzone(v_left, self.left_linear_deadzone, self.left_linear_slope)
            right_cmd = self._map_with_deadzone(v_right, self.right_linear_deadzone, self.right_linear_slope)
        elif ang != 0.0:
            # Use angular mapping for pure rotation
            left_cmd = self._map_with_deadzone(v_left, self.left_angular_deadzone, self.left_angular_slope)
            right_cmd = self._map_with_deadzone(v_right, self.right_angular_deadzone, self.right_angular_slope)
        else:
            left_cmd = 0.0
            right_cmd = 0.0

        out = Float32MultiArray()
        out.data = [left_cmd, right_cmd]
        self.motor_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityToMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HW3 velocity_mapping node stopped by keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
