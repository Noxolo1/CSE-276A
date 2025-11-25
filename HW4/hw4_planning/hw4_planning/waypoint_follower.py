#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from math import sin, cos
import json
import os

"""
The class of the pid controller for differential drive robot.
Reused from hw1_solution.py
"""
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

    def getError(self, currentState, targetState):
        """
        return the error between current and target state
        for differential drive: distance error and heading error
        """
        delta_x = targetState[0] - currentState[0]
        delta_y = targetState[1] - currentState[1]
        
        distance = np.sqrt(delta_x**2 + delta_y**2)
        
        angle_to_target = np.arctan2(delta_y, delta_x)
        desired_heading = angle_to_target
        
        heading_error = desired_heading - currentState[2]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        
        # When close to target, switch to orienting to final yaw
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

    def update(self, currentState):
        """
        calculate the update value based on PID control
        Returns: [linear_velocity, angular_velocity]
        """
        e = self.getError(currentState, self.target)

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


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')
        
        # Declare parameter for waypoint file path
        self.declare_parameter('waypoint_file', 'hw4_waypoints_safety.json')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        
        # Load waypoints from JSON file
        self.waypoints = self.load_waypoints()
        
        if len(self.waypoints) == 0:
            self.get_logger().error('No waypoints loaded! Exiting.')
            return
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
        self.pid = PIDcontroller(0.5, 0.01, 0.005)
        
        # Initialize at first waypoint position
        self.current_state = np.array([
            self.waypoints[0][0],  # x
            self.waypoints[0][1],  # y  
            self.waypoints[0][2]   # yaw
        ])
        
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.tolerance = 0.05  # position tolerance (meters)
        self.angle_tolerance = 0.2  # angle tolerance (radians)
        
        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Control stages: 'rotate_to_goal', 'drive', 'rotate_to_orient'
        self.stage = 'rotate_to_goal'
        self.fixed_rotation_vel = 0.785  # rad/s
        
        self.get_logger().info('Waypoint Follower Node started')

        # Orientation logging: actual vs desired
        self.orientation_log = []
        # Where to save the log on the Pi
        self.orientation_log_path = os.path.expanduser(
            "~/ros2_ws/rubikpi_ros2/hw4_planning/hw4_orientation_log.json"
        )

        
    def load_waypoints(self):
        """
        Load waypoints from JSON file.
        Returns: numpy array of waypoints [[x, y, yaw], ...]
        """
        waypoint_file = self.get_parameter('waypoint_file').value
        
        # Try to find the file
        if not os.path.exists(waypoint_file):
            # Try in common locations
            possible_paths = [
                waypoint_file,
                os.path.join(os.getcwd(), waypoint_file),
                os.path.join(os.path.expanduser('~'), 'ros2_ws', waypoint_file),
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    waypoint_file = path
                    break
            else:
                self.get_logger().error(f'Could not find waypoint file: {waypoint_file}')
                return np.array([])
        
        self.get_logger().info(f'Loading waypoints from: {waypoint_file}')
        
        try:
            with open(waypoint_file, 'r') as f:
                data = json.load(f)
            
            waypoints = []
            for wp in data:
                x = float(wp['x'])
                y = float(wp['y'])
                yaw = float(wp['yaw'])
                waypoints.append([x, y, yaw])
            
            return np.array(waypoints)
            
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {str(e)}')
            return np.array([])
        
    def update_dead_reckoning(self, linear_vel, angular_vel):
        """
        Update robot pose using dead reckoning
        """
        self.current_state[0] += linear_vel * np.cos(self.current_state[2]) * self.dt
        self.current_state[1] += linear_vel * np.sin(self.current_state[2]) * self.dt
        self.current_state[2] += angular_vel * self.dt
        self.current_state[2] = (self.current_state[2] + np.pi) % (2 * np.pi) - np.pi
        
    def broadcast_tf(self):
        """
        Broadcast TF transform from odom to base_link
        """
        current_time = self.get_clock().now()
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.current_state[0]
        t.transform.translation.y = self.current_state[1]
        t.transform.translation.z = 0.0
        
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.current_state[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def get_desired_heading_to_goal(self, current_wp):
        """
        Get the desired heading to face towards the goal
        """
        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        angle_to_target = np.arctan2(delta_y, delta_x)
        return angle_to_target
    
    def get_rotation_direction(self, heading_error):
        """
        Determine rotation direction based on heading error.
        Returns: angular velocity with fixed magnitude but correct direction
        """
        if heading_error > 0:
            return self.fixed_rotation_vel
        else:
            return -self.fixed_rotation_vel
        
    
    def log_orientation_sample(self, stage, current_wp, desired_heading):
        """
        Store a single orientation sample:
        - actual theta from dead-reckoning
        - desired heading towards waypoint position
        - desired yaw from the waypoint's planned orientation
        """
        # ROS time in seconds
        t = self.get_clock().now().nanoseconds / 1e9

        entry = {
            "time": float(t),
            "waypoint_idx": int(self.current_waypoint_idx),
            "stage": stage,
            "theta_actual": float(self.current_state[2]),
            "theta_desired_heading": float(desired_heading)
                if desired_heading is not None else None,
            "theta_desired_yaw": float(current_wp[2]),
        }
        self.orientation_log.append(entry)

    def save_orientation_log(self):
        """
        Write the collected orientation samples to a JSON file.
        """
        if not self.orientation_log:
            return

        try:
            with open(self.orientation_log_path, "w") as f:
                json.dump(self.orientation_log, f, indent=2)
            self.get_logger().info(
                f"Saved {len(self.orientation_log)} orientation samples to "
                f"'{self.orientation_log_path}'."
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to save orientation log: {e!r}")

        
    # def control_loop(self):
    #     """
    #     Main control loop with three stages: rotate to goal, drive, rotate to orientation
    #     """
    #     if self.current_waypoint_idx >= len(self.waypoints):
    #         self.get_logger().info('All waypoints reached! Stopping robot.')
    #         self.stop_robot()
    #         self.broadcast_tf()
    #         return

    #     current_wp = self.waypoints[self.current_waypoint_idx]
        
    #     # Initialize target on new waypoint
    #     if not self.waypoint_reached:
    #         self.pid.setTarget(current_wp)
    #         self.waypoint_reached = True
    #         self.stage = 'rotate_to_goal'

    #     # Calculate position error
    #     delta_x = current_wp[0] - self.current_state[0]
    #     delta_y = current_wp[1] - self.current_state[1]
    #     position_error = np.sqrt(delta_x**2 + delta_y**2)
        
    #     twist_msg = Twist()
        
    #     # Stage 1: Rotate to face the goal
    #     if self.stage == 'rotate_to_goal':
    #         desired_heading = self.get_desired_heading_to_goal(current_wp)
    #         heading_error = desired_heading - self.current_state[2]
    #         heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            
    #         if abs(heading_error) < 0.05:
    #             self.stage = 'drive'
    #             twist_msg.angular.z = 0.0
    #         else:
    #             twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
    #     # Stage 2: Drive towards the goal
    #     elif self.stage == 'drive':
    #         if position_error < self.tolerance:
    #             self.stage = 'rotate_to_orient'
    #             twist_msg.linear.x = 0.0
    #         else:
    #             update_value = self.pid.update(self.current_state)
    #             twist_msg.linear.x = float(update_value[0])
        
    #     # Stage 3: Rotate to target orientation
    #     elif self.stage == 'rotate_to_orient':
    #         heading_error = current_wp[2] - self.current_state[2]
    #         heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            
    #         if abs(heading_error) < self.angle_tolerance:
    #             # Move to next waypoint
    #             self.get_logger().info(
    #                 f'Reached waypoint {self.current_waypoint_idx} at '
    #                 f'({current_wp[0]:.3f}, {current_wp[1]:.3f})'
    #             )
    #             self.current_waypoint_idx += 1
    #             self.waypoint_reached = False
    #             twist_msg.angular.z = 0.0
    #         else:
    #             twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
    #     # Update pose and publish
    #     self.update_dead_reckoning(twist_msg.linear.x, twist_msg.angular.z)
    #     self.broadcast_tf()
    #     self.cmd_vel_pub.publish(twist_msg)

    def control_loop(self):
        """
        Main control loop with three stages: rotate to goal, drive, rotate to orientation
        """
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached! Stopping robot.')
            self.stop_robot()
            self.broadcast_tf()
            self.save_orientation_log()  # <-- save when done
            return

        current_wp = self.waypoints[self.current_waypoint_idx]
        
        # Initialize target on new waypoint
        if not self.waypoint_reached:
            self.pid.setTarget(current_wp)
            self.waypoint_reached = True
            self.stage = 'rotate_to_goal'

        # Calculate position error
        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        position_error = np.sqrt(delta_x**2 + delta_y**2)
        
        twist_msg = Twist()
        
        # Stage 1: Rotate to face the goal
        if self.stage == 'rotate_to_goal':
            desired_heading = self.get_desired_heading_to_goal(current_wp)
            heading_error = desired_heading - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

            # Log actual vs desired orientation
            self.log_orientation_sample(self.stage, current_wp, desired_heading)
            
            if abs(heading_error) < 0.05:
                self.stage = 'drive'
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
        # Stage 2: Drive towards the goal
        elif self.stage == 'drive':
            # Desired heading is still "point at the waypoint position"
            desired_heading = self.get_desired_heading_to_goal(current_wp)

            # Log actual vs desired orientation while driving
            self.log_orientation_sample(self.stage, current_wp, desired_heading)

            if position_error < self.tolerance:
                self.stage = 'rotate_to_orient'
                twist_msg.linear.x = 0.0
            else:
                update_value = self.pid.update(self.current_state)
                twist_msg.linear.x = float(update_value[0])
        
        # Stage 3: Rotate to target orientation
        elif self.stage == 'rotate_to_orient':
            # Here, the "desired" orientation is the waypoint's planned yaw
            desired_heading = self.get_desired_heading_to_goal(current_wp)
            heading_error = current_wp[2] - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

            # Log actual vs both: heading-to-waypoint and final yaw
            self.log_orientation_sample(self.stage, current_wp, desired_heading)
            
            if abs(heading_error) < self.angle_tolerance:
                # Move to next waypoint
                self.get_logger().info(
                    f'Reached waypoint {self.current_waypoint_idx} at '
                    f'({current_wp[0]:.3f}, {current_wp[1]:.3f})'
                )
                self.current_waypoint_idx += 1
                self.waypoint_reached = False
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
        # Update pose and publish
        self.update_dead_reckoning(twist_msg.linear.x, twist_msg.angular.z)
        self.broadcast_tf()
        self.cmd_vel_pub.publish(twist_msg)

    
    def stop_robot(self):
        """
        Stop the robot by publishing zero velocities
        """
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = WaypointFollowerNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Stopped by keyboard interrupt')
#     finally:
#         node.stop_robot()
#         node.save_orientation_log()
#         node.destroy_node()
#         rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by keyboard interrupt')
    finally:
        # 1) Save log first – this does not depend on ROS being alive
        try:
            node.save_orientation_log()
        except Exception as e:
            node.get_logger().warn(f"Failed to save orientation log in finally: {e!r}")

        # 2) Now *try* to stop the robot, but don't crash if ROS is already shutting down
        try:
            node.stop_robot()
        except Exception as e:
            node.get_logger().warn(f"Failed to stop robot cleanly: {e!r}")

        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
