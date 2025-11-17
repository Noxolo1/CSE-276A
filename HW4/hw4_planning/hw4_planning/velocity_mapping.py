# hw4_planning/hw4_planning/velocity_mapping.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np


class VelocityToMotorNode(Node):
    """
    HW4 velocity mapping node.

    Reuses the HW2 velocity mapping structure, but with the tuned deadzone and
    slope values you arrived at in HW3 for *your* robot (not the instructor's).
    """

    def __init__(self):
        super().__init__('hw4_velocity_mapping')

        self.wheel_base = 0.127  # [m]

        # Tuned maximum command magnitude
        self.cmd_max = 1.5

        # Tuned linear mapping parameters
        self.left_linear_deadzone = 0.09
        self.left_linear_slope = 2.5
        self.right_linear_deadzone = 0.09
        self.right_linear_slope = 2.5

        # Tuned angular mapping parameters (from your HW3 mapping)
        self.left_angular_deadzone = 0.31
        self.left_angular_slope = 14.0
        self.right_angular_deadzone = 0.31
        self.right_angular_slope = 14.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray,
                                               '/motor_commands', 10)

        self.get_logger().info('HW4 velocity_mapping node started')

    def _wheel_velocities(self, linear: float, angular: float) -> tuple[float, float]:
        """Convert (v, w) to wheel linear velocities (v_left, v_right)."""
        half_b = 0.5 * self.wheel_base
        v_left = linear - half_b * angular
        v_right = linear + half_b * angular
        return v_left, v_right

    def _map_with_deadzone(self, value: float, deadzone: float, slope: float) -> float:
        """
        Map a wheel velocity value to a motor command with deadzone and slope.

        The mapping is:
          cmd = sign(value) * (deadzone + |value| / slope)
        """
        if abs(value) < 1e-9:
            return 0.0
        cmd_mag = deadzone + abs(value) / max(slope, 1e-9)
        cmd_mag = min(cmd_mag, self.cmd_max)
        return float(np.copysign(cmd_mag, value))

    def cmd_vel_callback(self, msg: Twist):
        """Callback: convert /cmd_vel to two motor commands."""
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)

        v_left, v_right = self._wheel_velocities(lin, ang)

        if lin != 0.0:
            left_cmd = self._map_with_deadzone(
                v_left, self.left_linear_deadzone, self.left_linear_slope
            )
            right_cmd = self._map_with_deadzone(
                v_right, self.right_linear_deadzone, self.right_linear_slope
            )
        elif ang != 0.0:
            left_cmd = self._map_with_deadzone(
                v_left, self.left_angular_deadzone, self.left_angular_slope
            )
            right_cmd = self._map_with_deadzone(
                v_right, self.right_angular_deadzone, self.right_angular_slope
            )
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
        node.get_logger().info('Stopped by keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
