#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np


class Hw5VelocityToMotorNode(Node):
    """
    Velocity-to-motor command mapping for HW5.

    Reuses the HW2/HW4 mapping logic:

        1) Convert (v, w) to (v_left, v_right) using wheel_base.
        2) Map wheel velocities to motor commands with:
               cmd = deadzone + |v| / slope
           saturated by cmd_max and signed appropriately.
        3) Use separate deadzones/slopes for linear vs angular motions.

    All parameters are exposed as ROS parameters so you can tune them
    per homework without touching the code.
    """

    def __init__(self) -> None:
        super().__init__('hw5_velocity_mapping')

        # ---------------- Parameters ----------------
        # Wheel base and command saturation
        self.declare_parameter('wheel_base', 0.127)      # [m]
        self.declare_parameter('cmd_max', 1.5)

        # Linear mapping parameters
        self.declare_parameter('left_linear_deadzone', 0.13)
        self.declare_parameter('left_linear_slope', 3.5)
        self.declare_parameter('right_linear_deadzone', 0.11)
        self.declare_parameter('right_linear_slope', 3.5)

        # Angular mapping parameters
        self.declare_parameter('left_angular_deadzone', 0.26)
        self.declare_parameter('left_angular_slope', 12.0)
        self.declare_parameter('right_angular_deadzone', 0.27)
        self.declare_parameter('right_angular_slope', 12.0)

        # Threshold for deciding "linear vs angular mode"
        # (so tiny numerical noise in cmd_vel doesn't flip modes)
        self.declare_parameter('mode_threshold', 1e-3)

        # Load values into member variables
        self._update_from_parameters()

        # ---------------- ROS I/O ----------------
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.motor_pub = self.create_publisher(
            Float32MultiArray, '/motor_commands', 10
        )

        # Optional: allow dynamic parameter updates (re-tune at runtime)
        self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info('HW5 velocity mapping node started')

    # --------------------------------------------------------
    # Parameter helpers
    # --------------------------------------------------------

    def _update_from_parameters(self) -> None:
        """Read ROS parameters into member variables."""
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.cmd_max = float(self.get_parameter('cmd_max').value)

        self.left_linear_deadzone = float(
            self.get_parameter('left_linear_deadzone').value
        )
        self.left_linear_slope = float(
            self.get_parameter('left_linear_slope').value
        )
        self.right_linear_deadzone = float(
            self.get_parameter('right_linear_deadzone').value
        )
        self.right_linear_slope = float(
            self.get_parameter('right_linear_slope').value
        )

        self.left_angular_deadzone = float(
            self.get_parameter('left_angular_deadzone').value
        )
        self.left_angular_slope = float(
            self.get_parameter('left_angular_slope').value
        )
        self.right_angular_deadzone = float(
            self.get_parameter('right_angular_deadzone').value
        )
        self.right_angular_slope = float(
            self.get_parameter('right_angular_slope').value
        )

        self.mode_threshold = float(
            self.get_parameter('mode_threshold').value
        )

        self.get_logger().info(
            f'Params: wheel_base={self.wheel_base:.3f}, cmd_max={self.cmd_max:.2f}; '
            f'lin L(dead={self.left_linear_deadzone:.2f}, k={self.left_linear_slope:.2f}) '
            f'R(dead={self.right_linear_deadzone:.2f}, k={self.right_linear_slope:.2f}); '
            f'ang L(dead={self.left_angular_deadzone:.2f}, k={self.left_angular_slope:.2f}) '
            f'R(dead={self.right_angular_deadzone:.2f}, k={self.right_angular_slope:.2f})'
        )

    def _on_param_update(self, params):
        """Callback when parameters are changed at runtime."""
        # Just re-read everything; we don't need to inspect which changed.
        self._update_from_parameters()
        return rclpy.parameter.SetParametersResult(successful=True)

    # --------------------------------------------------------
    # Core mapping logic (same as HW2/HW4)
    # --------------------------------------------------------

    def _wheel_velocities(self, linear: float, angular: float) -> tuple[float, float]:
        """
        Convert (v, w) to (v_left, v_right).
        """
        half_b = 0.5 * self.wheel_base
        v_left = linear - half_b * angular
        v_right = linear + half_b * angular
        return v_left, v_right

    def _map_with_deadzone(self, value: float, deadzone: float, slope: float) -> float:
        """
        Map wheel velocity [m/s] to motor command in [-cmd_max, cmd_max]
        using deadzone + slope model from HW2/HW4.
        """
        if abs(value) < 1e-9:
            return 0.0

        cmd_mag = deadzone + abs(value) / max(slope, 1e-9)
        cmd_mag = min(cmd_mag, self.cmd_max)

        return float(np.copysign(cmd_mag, value))

    # --------------------------------------------------------
    # ROS callback
    # --------------------------------------------------------

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Map incoming /cmd_vel (v, w) to /motor_commands [left, right].
        """
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)

        v_left, v_right = self._wheel_velocities(lin, ang)

        # Decide mode: linear vs angular vs stop
        if abs(lin) > self.mode_threshold:
            left_cmd = self._map_with_deadzone(
                v_left, self.left_linear_deadzone, self.left_linear_slope
            )
            right_cmd = self._map_with_deadzone(
                v_right, self.right_linear_deadzone, self.right_linear_slope
            )
        elif abs(ang) > self.mode_threshold:
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
    node = Hw5VelocityToMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HW5 velocity mapping node stopped by keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
