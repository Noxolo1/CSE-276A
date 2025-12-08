import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np


class Hw5VelocityToMotorNode(Node):
    # hw5 velocity mapping file, parameterized this time
    # for easier command line tuning 

    def __init__(self) -> None:
        super().__init__('hw5_velocity_mapping')

        self.declare_parameter('wheel_base', 0.127)
        self.declare_parameter('cmd_max', 1.5)

        self.declare_parameter('left_linear_deadzone', 0.13)
        self.declare_parameter('left_linear_slope', 3.5)
        self.declare_parameter('right_linear_deadzone', 0.11)
        self.declare_parameter('right_linear_slope', 3.5)
        self.declare_parameter('left_angular_deadzone', 0.23)
        self.declare_parameter('left_angular_slope', 20.0)
        self.declare_parameter('right_angular_deadzone', 0.23)
        self.declare_parameter('right_angular_slope', 20.0)

        self.declare_parameter('mode_threshold', 1e-3)

        self._update_from_parameters()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.motor_pub = self.create_publisher(
            Float32MultiArray, '/motor_commands', 10
        )

        self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info('HW5 velocity mapping node started')

   
    def _update_from_parameters(self) -> None:
        
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
        self._update_from_parameters()
        return rclpy.parameter.SetParametersResult(successful=True)

    # mapping code from HW2 velcoity mapping
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
    
    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Map incoming /cmd_vel (v, w) to /motor_commands [left, right].
        """
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)

        v_left, v_right = self._wheel_velocities(lin, ang)

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
