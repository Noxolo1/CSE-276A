#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException
import numpy as np
from math import sin, cos
import json
import os
from datetime import datetime
import yaml
from scipy.spatial.transform import Rotation


# PID controller (same as HW4s)
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
        self.I = np.array([0.0, 0.0])
        self.lastError = np.array([0.0, 0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        delta_x = targetState[0] - currentState[0]
        delta_y = targetState[1] - currentState[1]

        distance = np.sqrt(delta_x**2 + delta_y**2)

        angle_to_target = np.arctan2(delta_y, delta_x)
        desired_heading = angle_to_target

        heading_error = desired_heading - currentState[2]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        if abs(distance) < 0.05:
            heading_error = targetState[2] - currentState[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            distance = 0.0

        return np.array([distance, heading_error])

    def setMaximumUpdate(self, mv):
        self.maximumValue = mv

    def update(self, currentState):
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        if abs(result[0]) > self.maximumValue:
            result[0] = np.sign(result[0]) * self.maximumValue

        max_angular = 1.5 
        if abs(result[1]) > max_angular:
            result[1] = np.sign(result[1]) * max_angular

        if abs(e[0]) < 0.05:
            result[0] = 0.0
        return result


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__("waypoint_follower_node")

        # parameter for waypoint data
        self.declare_parameter("waypoint_file", "hw5_waypoints_lawnmower.json")
        self.declare_parameter("tag_yaml_file", "apriltags_position.yaml")
        self.declare_parameter("enable_apriltag_corrections", True)
        self.declare_parameter("correction_max_age", 1.0)  # sec

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_frame = "odom"
        self.base_frame = "base_link"
        self.camera_frame = "camera_frame"

        # path to waypoint file
        self.waypoint_path = None

        # load in apriltag map
        self.tag_data = self.load_apriltag_map()
        self.enable_corrections = self.get_parameter("enable_apriltag_corrections").value
        self.correction_max_age = self.get_parameter("correction_max_age").value
        self.last_correction_time = 0.0

        # load waypoints
        self.waypoints = self.load_waypoints()

        if len(self.waypoints) == 0:
            self.get_logger().error("No waypoints loaded")
            return

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        if self.enable_corrections:
            self.get_logger().info(f"AprilTag corrections enabled (loaded {len(self.tag_data)} tags)")
        else:
            self.get_logger().info("AprilTag corrections disabled (pure dead reckoning)")

        self.pid = PIDcontroller(0.5, 0.01, 0.005)

        # init robot state at first waypoint (we assume robot starts there)
        self.current_state = np.array(
            [
                self.waypoints[0][0],  # x
                self.waypoints[0][1],  # y
                self.waypoints[0][2],  # orientation (yaw)
            ]
        )

        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.tolerance = 0.05  # position tolerance (m)
        self.angle_tolerance = 0.2  # angle tolerance (rad)

        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # timer to consider apriltag correction
        if self.enable_corrections:
            self.correction_timer = self.create_timer(0.1, self.apriltag_correction)

        self.stage = "rotate_to_goal"
        self.fixed_rotation_vel = 0.785 

        # logging setup
        if self.waypoint_path is not None:
            base_dir = os.path.dirname(self.waypoint_path)
        else:
            base_dir = os.getcwd()

        # orientation log
        self.orientation_log = []
        self.orientation_log_path = os.path.join(base_dir, "hw5_orientation_log.json")

        # coverage trajectory log
        self.trajectory_log = []
        self.trajectory_log_dir = base_dir

        self.get_logger().info("hw5 waypoint follower node started")


    def load_apriltag_map(self):
        yaml_file = self.get_parameter("tag_yaml_file").value
        
        if not yaml_file:
            self.get_logger().warn("No AprilTag map file specified")
            return []
        
        possible_paths = [
            yaml_file,
            os.path.join(os.getcwd(), yaml_file),
            os.path.join(os.path.expanduser("~"), "ros2_ws", yaml_file),
            os.path.join(os.path.expanduser("~"), "ros2_ws", "src", "hw5_coverage", "configs", yaml_file),
        ]
        
        yaml_path = None
        for path in possible_paths:
            if os.path.exists(path):
                yaml_path = path
                break
        
        if yaml_path is None:
            self.get_logger().warn(f"Could not find AprilTag map: {yaml_file}")
            return []
        
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            tags = data.get('apriltags', [])
            self.get_logger().info(f"Loaded {len(tags)} AprilTags from {yaml_path}")
            return tags
        except Exception as e:
            self.get_logger().error(f"Error loading AprilTag map: {e}")
            return []


    def load_waypoints(self):
        waypoint_file = self.get_parameter("waypoint_file").value

        if not os.path.exists(waypoint_file):
            possible_paths = [
                waypoint_file,
                os.path.join(os.getcwd(), waypoint_file),
                os.path.join(os.path.expanduser("~"), "ros2_ws", waypoint_file),
            ]

            for path in possible_paths:
                if os.path.exists(path):
                    waypoint_file = path
                    break
            else:
                self.get_logger().error(f"could not find waypoint file: {waypoint_file}")
                return np.array([])

        self.get_logger().info(f"loading waypoints from: {waypoint_file}")
        self.waypoint_path = waypoint_file

        try:
            with open(waypoint_file, "r") as f:
                data = json.load(f)

            waypoints = []
            for wp in data:
                x = float(wp["x"])
                y = float(wp["y"])
                yaw = float(wp.get("yaw", wp.get("theta", 0.0)))
                waypoints.append([x, y, yaw])

            return np.array(waypoints)

        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {str(e)}")
            return np.array([])

    # apriltag localization (reused from hw2)
    def compute_robot_pose_from_tag(self, tag_id, observation):
        """Compute robot pose from AprilTag observation"""
        tag_data = None
        for tag in self.tag_data:
            if tag.get('id') == tag_id:
                tag_data = tag
                break
        
        if tag_data is None:
            return None
        
        tag_map_pos = np.array([tag_data['x'], tag_data['y'], tag_data['z']])
        tag_map_rot = Rotation.from_quat([
            tag_data['qx'], tag_data['qy'], 
            tag_data['qz'], tag_data['qw']
        ])
        
        obs_pos = np.array([
            observation.transform.translation.x,
            observation.transform.translation.y,
            observation.transform.translation.z
        ])
        obs_rot = Rotation.from_quat([
            observation.transform.rotation.x,
            observation.transform.rotation.y,
            observation.transform.rotation.z,
            observation.transform.rotation.w
        ])
        
        tag_to_camera_rot = obs_rot.inv()
        tag_to_camera_pos = -tag_to_camera_rot.apply(obs_pos)
        
        camera_map_rot = tag_map_rot * tag_to_camera_rot
        camera_map_pos = tag_map_pos + tag_map_rot.apply(tag_to_camera_pos)
        
        camera_to_base_pos = np.array([-0.0675, 0.0, -0.035])
        camera_to_base_rot = Rotation.from_quat([-0.5, 0.5, -0.5, 0.5])
        
        robot_map_rot = camera_map_rot * camera_to_base_rot
        robot_map_pos = camera_map_pos + camera_map_rot.apply(camera_to_base_pos)
        
        yaw = robot_map_rot.as_euler('xyz')[2]
        
        return (float(robot_map_pos[0]), float(robot_map_pos[1]), float(yaw))

    def apriltag_correction(self):
        # applies apriltag correction to dead reckoning
        if not self.enable_corrections or len(self.tag_data) == 0:
            return
        
        # only correct during driving stage
        if self.stage != 'drive':
            return
        
        now = self.get_clock().now().nanoseconds / 1e9
        
        # get closest visible tag 
        best_tag_id = None
        best_distance = float('inf')
        best_observation = None
        
        for tag in self.tag_data:
            tag_id = tag.get('id')
            if tag_id is None:
                continue

            try:
                obs = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    f'tag_{tag_id}',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.01)
                )
                
                # check obs age
                obs_time = rclpy.time.Time.from_msg(obs.header.stamp)
                time_diff = (self.get_clock().now() - obs_time).nanoseconds / 1e9
                
                if time_diff > 0.5:  # threshold for being too old
                    continue
                
                dx = obs.transform.translation.x
                dy = obs.transform.translation.y
                dz = obs.transform.translation.z
                distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance < best_distance:
                    best_distance = distance
                    best_tag_id = tag_id
                    best_observation = obs
                    
            except TransformException:
                continue
        
        # apply correction
        if best_observation is not None:
            pose = self.compute_robot_pose_from_tag(best_tag_id, best_observation)
            
            if pose is not None:
                age = now - self.last_correction_time
                
                if age <= self.correction_max_age or self.last_correction_time == 0.0:
                    self.current_state = np.array(pose)
                    self.last_correction_time = now
                    
                    self.get_logger().info(
                        f"AprilTag correction from tag {best_tag_id}: "
                        f"({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})",
                        throttle_duration_sec=2.0
                    )


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
        Broadcast TF transform from odom to base_link (purely internal DR frame)
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
        Returns angular velocity with fixed magnitude but correct direction.
        """
        if heading_error > 0:
            return self.fixed_rotation_vel
        else:
            return -self.fixed_rotation_vel

    def log_orientation_sample(self, stage, current_wp, desired_heading):
        t = self.get_clock().now().nanoseconds / 1e9

        entry = {
            "time": float(t),
            "waypoint_idx": int(self.current_waypoint_idx),
            "stage": stage,
            "theta_actual": float(self.current_state[2]),
            "theta_desired_heading": float(desired_heading)
            if desired_heading is not None
            else None,
            "theta_desired_yaw": float(current_wp[2]),
        }
        self.orientation_log.append(entry)

    def log_trajectory_sample(self):
        t = self.get_clock().now().nanoseconds / 1e9
        entry = {
            "time": float(t),
            "x": float(self.current_state[0]),
            "y": float(self.current_state[1]),
            "theta": float(self.current_state[2]),
            "waypoint_idx": int(self.current_waypoint_idx),
            "stage": self.stage,
        }
        self.trajectory_log.append(entry)

    def control_loop(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached! Stopping robot.")
            self.stop_robot()
            self.broadcast_tf()
            self.log_trajectory_sample()
            self.save_orientation_log()
            return

        current_wp = self.waypoints[self.current_waypoint_idx]

        if not self.waypoint_reached:
            self.pid.setTarget(current_wp)
            self.waypoint_reached = True
            self.stage = "rotate_to_goal"

        # calculate position error
        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        position_error = np.sqrt(delta_x**2 + delta_y**2)

        twist_msg = Twist()

        # Stage 1: Rotate to face the goal
        if self.stage == "rotate_to_goal":
            desired_heading = self.get_desired_heading_to_goal(current_wp)
            heading_error = desired_heading - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

            self.log_orientation_sample(self.stage, current_wp, desired_heading)

            if abs(heading_error) < 0.05:
                self.stage = "drive"
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))

        # Stage 2: Drive towards the goal (we correct w/ apriltag here if needed)
        elif self.stage == "drive":
            desired_heading = self.get_desired_heading_to_goal(current_wp)

            self.log_orientation_sample(self.stage, current_wp, desired_heading)

            if position_error < self.tolerance:
                self.stage = "rotate_to_orient"
                twist_msg.linear.x = 0.0
            else:
                update_value = self.pid.update(self.current_state)
                twist_msg.linear.x = float(update_value[0])

        # Stage 3: Rotate to target orientation at the waypoint
        elif self.stage == "rotate_to_orient":
            desired_heading = self.get_desired_heading_to_goal(current_wp)
            heading_error = current_wp[2] - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

            self.log_orientation_sample(self.stage, current_wp, desired_heading)

            if abs(heading_error) < self.angle_tolerance:
                self.get_logger().info(
                    f"Reached waypoint {self.current_waypoint_idx} at "
                    f"({current_wp[0]:.3f}, {current_wp[1]:.3f})"
                )
                self.current_waypoint_idx += 1
                self.waypoint_reached = False
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))

        self.update_dead_reckoning(twist_msg.linear.x, twist_msg.angular.z)
        self.broadcast_tf()
        self.cmd_vel_pub.publish(twist_msg)
        self.log_trajectory_sample()

    def save_orientation_log(self):
        if not self.orientation_log:
            return

        try:
            with open(self.orientation_log_path, "w") as f:
                json.dump(self.orientation_log, f, indent=2)
            self.get_logger().info(
                f"saved {len(self.orientation_log)} orientation samples to "
                f"'{self.orientation_log_path}'"
            )
        except Exception as e:
            self.get_logger().warn(f"failed to save orientation log: {e!r}")

    def save_trajectory_log(self):
        if not self.trajectory_log:
            return

        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = f"hw5_coverage_trajectory_{ts}.json"
        out_path = os.path.join(self.trajectory_log_dir, filename)

        try:
            with open(out_path, "w") as f:
                json.dump(self.trajectory_log, f, indent=2)
            self.get_logger().info(
                f"saved {len(self.trajectory_log)} trajectory samples to "
                f"'{out_path}'"
            )
        except Exception as e:
            self.get_logger().warn(f"failed to save trajectory log: {e!r}")

    def stop_robot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by keyboard interrupt")
    finally:
        try:
            node.save_orientation_log()
        except Exception as e:
            node.get_logger().warn(f"Failed to save orientation log: {e!r}")

        try:
            node.save_trajectory_log()
        except Exception as e:
            node.get_logger().warn(f"Failed to save trajectory log: {e!r}")

        try:
            node.stop_robot()
        except Exception as e:
            node.get_logger().warn(f"Failed to stop robot: {e!r}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

