#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math

# PID controller for differential drive robot (a lot of code reused from HW2) 
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0, 0.0])
        self.lastError = np.array([0.0, 0.0])
        self.timestep = 0.1
        self.maximumValue = 0.2

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0, 0.0])
        self.lastError = np.array([0.0, 0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState, drive_backwards):
        """
        return the error between current and target state
        for differential drive: distance error and heading error
        """
        delta_x = targetState[0] - currentState[0]
        delta_y = targetState[1] - currentState[1]

        distance = np.sqrt(delta_x**2 + delta_y**2)
        angle_to_target = np.arctan2(delta_y, delta_x)

        if drive_backwards:
            desired_heading = angle_to_target + np.pi
            desired_heading = (desired_heading + np.pi) % (2 * np.pi) - np.pi
            distance = -distance
        else:
            desired_heading = angle_to_target

        heading_error = desired_heading - currentState[2]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        if abs(distance) < 0.05:
            heading_error = targetState[2] - currentState[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            distance = 0.0

        return np.array([distance, heading_error])

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState, drive_backwards):
        """
        calculate the update value based on PID control
        Returns: [linear_velocity, angular_velocity]
        """
        e = self.getError(currentState, self.target, drive_backwards)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep
        I = self.I
        D = self.Kd * (e - self.lastError)

        result = P + I + D
        self.lastError = e

        if abs(result[0]) > self.maximumValue:
            result[0] = np.sign(result[0]) * self.maximumValue

        max_angular = 1.5  # rad/s
        if abs(result[1]) > max_angular:
            result[1] = np.sign(result[1]) * max_angular

        if abs(e[0]) < 0.05:
            result[0] = 0.0

        return result


class Hw3ControllerNode(Node):
    
    # controller node for HW3 that uses SLAM pose estimate
    # subscribes to /slam_pose from ekf_slam.py and publishes /cmd_vel
    # to follow trajectories
    def __init__(self):
        super().__init__('hw3_controller_node')

        # publish and subscribe
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/slam_pose',
            self.pose_callback,
            10
        )

        # debug logging
        self.log_period = 2.5  
        self.last_log_time = self.get_clock().now()

        # define trajectories
        # trajectory: square or octagon
        self.declare_parameter('trajectory', 'square')

        # square side length in meters
        self.declare_parameter('square_side', 1.5)

        # octagon radius from center to vertices
        self.declare_parameter('octagon_radius', 1.2)

        trajectory_type = self.get_parameter('trajectory').value

        if trajectory_type == 'square':
            side_length = float(self.get_parameter('square_side').value)
            self.waypoints = self.generate_square_waypoints(side_length)
            self.get_logger().info(f'Square trajectory: side={side_length} m')

        elif trajectory_type == 'octagon':
            radius = float(self.get_parameter('octagon_radius').value)
            self.waypoints = self.generate_octagon_waypoints(radius)
            self.get_logger().info(f'Octagon trajectory: radius={radius} m')

        else:
            self.get_logger().error(f'Unknown trajectory type: {trajectory_type}')
            raise ValueError(f'Unknown trajectory type: {trajectory_type}')

        # PID parameters
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.005)
        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        self.pid = PIDcontroller(kp, ki, kd)

        # state from SLAM
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.pose_received = False

        # waypoint tracking
        self.current_waypoint_idx = 0
        self.waypoint_reached = False

        # tolerances
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('angle_tolerance', 0.1)
        self.tolerance = float(self.get_parameter('position_tolerance').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)

        self.drive_backwards = False

        self.declare_parameter('control_dt', 0.1)
        self.dt = float(self.get_parameter('control_dt').value)
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.declare_parameter('rotation_speed', 0.785)  # rad/s
        self.fixed_rotation_vel = float(self.get_parameter('rotation_speed').value)

        self.stage = 'rotate_to_goal'

        self.get_logger().info('HW3 controller node initialized')
        self.get_logger().info(f'Waypoints: {len(self.waypoints)}')
        self.get_logger().info('Waiting for SLAM pose...')

    # functions to generate waypoints

    def generate_square_waypoints(self, side_length):
        return np.array([
            [side_length, 0.0,          0.0],        # +x
            [side_length, side_length,  math.pi/2],  # +y
            [0.0,        side_length,   math.pi],    # -x
            [0.0,        0.0,          -math.pi/2],  # -y back to origin
        ])

    def generate_octagon_waypoints(self, radius):
        waypoints = []
        n_sides = 8
        angle_step = 2 * np.pi / n_sides

        for i in range(n_sides):
            angle = i * angle_step
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            heading = angle + angle_step / 2.0  # roughly tangent
            waypoints.append([x, y, heading])

        waypoints.append([waypoints[0][0], waypoints[0][1], waypoints[0][2]])

        return np.array(waypoints)

    def pose_callback(self, msg: PoseStamped):
        # get pose estimate from EKF SLAM 
        self.current_state[0] = msg.pose.position.x
        self.current_state[1] = msg.pose.position.y

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_state[2] = math.atan2(siny_cosp, cosy_cosp)

        if not self.pose_received:
            self.get_logger().info('SLAM pose received, starting navigation.')
            self.pose_received = True

    def control_loop(self):
        if not self.pose_received:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info(
                'All waypoints reached.',
                throttle_duration_sec=5.0
            )
            self.stop_robot()
            return

        current_wp = self.waypoints[self.current_waypoint_idx]

        # initialize new waypoint
        if not self.waypoint_reached:
            self.pid.setTarget(current_wp)
            self.drive_backwards = self.should_drive_backwards(current_wp)
            self.waypoint_reached = True
            self.stage = 'rotate_to_goal'
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
                f'({current_wp[0]:.2f}, {current_wp[1]:.2f}, {current_wp[2]:.2f}rad)'
            )

        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        position_error = math.hypot(delta_x, delta_y)

        twist_msg = Twist()

        # Stage 1: Rotate to face the goal
        if self.stage == 'rotate_to_goal':
            desired_heading = self.get_desired_heading_to_goal(current_wp, self.drive_backwards)
            heading_error = desired_heading - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

            if abs(heading_error) < 0.05:
                self.stage = 'drive'
                self.get_logger().info('Aligned with goal, starting drive')
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))

        # Stage 2: Drive towards/away from the goal
        elif self.stage == 'drive':
            if position_error < self.tolerance:
                self.stage = 'rotate_to_orient'
                self.get_logger().info('Reached position, rotating to final orientation')
                twist_msg.linear.x = 0.0
            else:
                desired_heading = self.get_desired_heading_to_goal(current_wp, self.drive_backwards)
                heading_error = desired_heading - self.current_state[2]
                heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

                if abs(heading_error) > 0.2:
                    self.stage = 'rotate_to_goal'
                    self.get_logger().info('Lost alignment, re-rotating')
                    twist_msg.linear.x = 0.0
                else:
                    update_value = self.pid.update(self.current_state, self.drive_backwards)
                    twist_msg.linear.x = float(update_value[0])

        # Stage 3: Rotate to target orientation
        elif self.stage == 'rotate_to_orient':
            heading_error = current_wp[2] - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

            if abs(heading_error) < 0.1 :
                self.get_logger().info(f'Waypoint {self.current_waypoint_idx + 1} complete!')
                self.current_waypoint_idx += 1
                self.waypoint_reached = False
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))

        # debugging log
        now = self.get_clock().now()
        dt = (now - self.last_log_time).nanoseconds / 1e9
        if dt >= self.log_period:
            self.last_log_time = now

            if self.stage == 'rotate_to_goal':
                state_str = 'ROTATE_TO_GOAL'
            elif self.stage == 'drive':
                state_str = 'DRIVE'
            elif self.stage == 'rotate_to_orient':
                state_str = 'ROTATE_TO_FINAL'
            else:
                state_str = str(self.stage)

            gx, gy, gtheta = current_wp
            x, y, theta = self.current_state

            self.get_logger().info(
                f"STATE={state_str} | "
                f"TARGET={gx:.2f}, {gy:.2f}, {math.degrees(gtheta):.1f}deg | "
                f"CMD=v={twist_msg.linear.x:.3f}, w={twist_msg.angular.z:.3f} | "
                f"POSE={x:.2f}, {y:.2f}, {math.degrees(theta):.1f}deg"
            )

        self.cmd_vel_pub.publish(twist_msg)

    def should_drive_backwards(self, current_wp):
        return False 

    def get_desired_heading_to_goal(self, current_wp, drive_backwards):
        """
        Get the desired heading to face towards (or away from) the goal
        """
        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        angle_to_target = math.atan2(delta_y, delta_x)

        if drive_backwards:
            desired_heading = angle_to_target + np.pi
            desired_heading = (desired_heading + np.pi) % (2 * np.pi) - np.pi
        else:
            desired_heading = angle_to_target

        return desired_heading

    def get_rotation_direction(self, heading_error):
        """
        Determine rotation direction based on heading error.
        Returns: angular velocity with fixed magnitude but correct direction
        """
        if heading_error > 0:
            return self.fixed_rotation_vel
        else:
            return -self.fixed_rotation_vel

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Hw3ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by keyboard interrupt')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()