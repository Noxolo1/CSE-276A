#!/usr/bin/env python3
"""
HW5 coverage controller node.

This node reuses the HW2 reference localization pipeline:

- Use AprilTag ground-truth poses from apriltags_position.yaml (map/world frame).
- Use TF to get base_link -> tag_i transforms (via camera_tf + apriltag_ros).
- Fuse those to compute the robot pose (x, y, yaw) in the map/world frame.

On top of that, it implements a simple randomized waypoint-coverage strategy:
- The workspace bounds come from tags 0–7.
- Tags 8–11 (if present) define an internal obstacle region.
- The node repeatedly samples random waypoints inside the workspace but
  outside the obstacle box and drives the robot toward them using a simple
  proportional controller on distance and heading.

No EKF/SLAM is used here (that is only needed for extra credit).
"""

import math
import os
import json
import time
from typing import Dict, List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import yaml


class Hw5CoverageNode(Node):
    def __init__(self) -> None:
        super().__init__("hw5_coverage_node")

        # --- Publishers / TF ---
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Frames (matching HW2/3 style)
        self.odom_frame = "odom"       # world / map frame
        self.base_frame = "base_link"

        # --- Parameters ---
        self.declare_parameter("apriltag_map_file", "apriltags_position.yaml")
        self.declare_parameter("trajectory_log_file", "hw5_coverage_trajectory.json")

        # coverage control parameters
        self.declare_parameter("explore_speed", 0.18)          # max forward speed [m/s]
        self.declare_parameter("max_angular_speed", 0.9)       # [rad/s]
        self.declare_parameter("boundary_margin", 0.15)        # [m]
        self.declare_parameter("obstacle_margin", 0.10)        # [m]
        self.declare_parameter("waypoint_radius", 0.10)        # [m] distance to consider waypoint reached
        self.declare_parameter("lost_pose_timeout", 2.0)       # [s] before we start searching
        self.declare_parameter("tag_stale_threshold", 0.25)    # [s] max allowed transform age
        self.declare_parameter("k_v", 0.5)                     # [1/s] gain for linear velocity
        self.declare_parameter("k_w", 1.5)                     # [1/s] gain for angular velocity

        # read parameter values
        self.explore_speed = float(self.get_parameter("explore_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.boundary_margin = float(self.get_parameter("boundary_margin").value)
        self.obstacle_margin = float(self.get_parameter("obstacle_margin").value)
        self.waypoint_radius = float(self.get_parameter("waypoint_radius").value)
        self.lost_pose_timeout = float(self.get_parameter("lost_pose_timeout").value)
        self.tag_stale_threshold = float(self.get_parameter("tag_stale_threshold").value)
        self.k_v = float(self.get_parameter("k_v").value)
        self.k_w = float(self.get_parameter("k_w").value)

        # --- AprilTag map and workspace geometry ---
        self.tag_positions: Dict[int, Dict] = {}
        self.tag_ids: List[int] = []
        self._load_tag_configurations()
        self._compute_workspace_and_obstacle_bounds()

        # --- Robot pose state (odom/map frame) ---
        # current_state = [x, y, yaw]
        self.current_state = np.array([0.0, 0.0, 0.0], dtype=float)
        self.has_pose: bool = False
        self.last_tag_detection_time: float = 0.0

        # --- Coverage / waypoint state ---
        self.current_waypoint: Optional[np.ndarray] = None  # [x, y, yaw_goal]
        self.rng = np.random.default_rng()

        # --- Trajectory logging ---
        # We want a unique file per run: hw5_coverage_trajectory_TIMESTAMP.json
        traj_base = self.get_parameter("trajectory_log_file").value
        # Allow the parameter to be given with or without ".json"
        if traj_base.endswith(".json"):
            traj_base = traj_base[:-5]

        # e.g. 20251206-113045
        self.run_timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"{traj_base}_{self.run_timestamp}.json"

        self.trajectory_log_path = os.path.join(
            os.path.expanduser("~"), "ros2_ws", filename
        )
        self.trajectory_log: List[Dict] = []

        self.get_logger().info(
            f"HW5 trajectory log will be saved to: {self.trajectory_log_path}"
        )


        # --- Timers ---
        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.localization_timer = self.create_timer(0.1, self.localization_update)

        self.get_logger().info("HW5 Coverage Node initialized")

    # ------------------------------------------------------------------
    # AprilTag map loading & workspace geometry (reusing HW2 patterns)
    # ------------------------------------------------------------------
    def _resolve_apriltag_map_path(self) -> str:
        filename = self.get_parameter("apriltag_map_file").value
        if os.path.isabs(filename) and os.path.exists(filename):
            return filename

        try:
            pkg_share = get_package_share_directory("hw5_coverage")
            candidate = os.path.join(pkg_share, "configs", filename)
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass

        return os.path.join(os.getcwd(), filename)

    def _load_tag_configurations(self) -> None:
        """Load AprilTag positions / orientations from YAML (like HW2)."""
        yaml_path = self._resolve_apriltag_map_path()
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Could not find AprilTag map file: {yaml_path}")
            return

        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)

        tags_data = data.get("apriltags", [])
        for tag in tags_data:
            tag_id = tag.get("id")
            if tag_id is None:
                continue

            self.tag_positions[int(tag_id)] = {
                "x": float(tag["x"]),
                "y": float(tag["y"]),
                "z": float(tag.get("z", 0.0)),
                "qx": float(tag.get("qx", 0.0)),
                "qy": float(tag.get("qy", 0.0)),
                "qz": float(tag.get("qz", 0.0)),
                "qw": float(tag.get("qw", 1.0)),
            }

        self.tag_ids = sorted(self.tag_positions.keys())
        self.get_logger().info(f"Loaded {len(self.tag_ids)} AprilTags from {yaml_path}")

    def _compute_workspace_and_obstacle_bounds(self) -> None:
        """
        HW5: no internal obstacle. All tags (0–11) are on the outer boundary
        of the 8x8 ft workspace. We just use ALL tags to define the workspace,
        and we disable obstacle handling.
        """
        if not self.tag_positions:
            self.workspace_min_x = -1.0
            self.workspace_max_x = 1.0
            self.workspace_min_y = -1.0
            self.workspace_max_y = 1.0
            self.workspace_center_x = 0.0
            self.workspace_center_y = 0.0
            self.has_obstacle = False
            self.get_logger().warn(
                "No tags in map; using dummy workspace [-1,1] x [-1,1] and no obstacle."
            )
            return

        # Use ALL tags to estimate workspace bounds (0–11 are all boundary tags)
        xs = [t["x"] for t in self.tag_positions.values()]
        ys = [t["y"] for t in self.tag_positions.values()]

        self.workspace_min_x = min(xs)
        self.workspace_max_x = max(xs)
        self.workspace_min_y = min(ys)
        self.workspace_max_y = max(ys)
        self.workspace_center_x = 0.5 * (self.workspace_min_x + self.workspace_max_x)
        self.workspace_center_y = 0.5 * (self.workspace_min_y + self.workspace_max_y)

        self.get_logger().info(
            f"Workspace bounds (no obstacle): "
            f"x=[{self.workspace_min_x:.3f}, {self.workspace_max_x:.3f}], "
            f"y=[{self.workspace_min_y:.3f}, {self.workspace_max_y:.3f}]"
        )

        # No internal obstacle for HW5
        self.has_obstacle = False
        self.get_logger().info("HW5: treating all tags as boundary; no internal obstacle.")


    # ------------------------------------------------------------------
    # Localization (reusing HW2: base_link -> tag_i + YAML)
    # ------------------------------------------------------------------
    def _update_pose_from_tag(self, tag_id: int, observation: TransformStamped) -> None:
        """
        Compute robot pose in map/odom frame from:
          - map->tag_i from YAML (self.tag_positions)
          - base_link->tag_i from TF (observation)
        and update self.current_state = [x, y, yaw].
        """
        tag_map = self.tag_positions[tag_id]

        tag_map_pos = np.array([tag_map["x"], tag_map["y"], tag_map["z"]])
        tag_map_rot = Rotation.from_quat(
            [tag_map["qx"], tag_map["qy"], tag_map["qz"], tag_map["qw"]]
        )

        # observation: transform from base_link to tag_i
        obs_pos = np.array(
            [
                observation.transform.translation.x,
                observation.transform.translation.y,
                observation.transform.translation.z,
            ]
        )
        obs_rot = Rotation.from_quat(
            [
                observation.transform.rotation.x,
                observation.transform.rotation.y,
                observation.transform.rotation.z,
                observation.transform.rotation.w,
            ]
        )

        # base_link -> tag ; we want tag -> base_link
        tag_to_robot_rot = obs_rot.inv()
        tag_to_robot_pos = -tag_to_robot_rot.apply(obs_pos)

        # Compose: map -> base_link = map->tag * tag->base_link
        robot_map_rot = tag_map_rot * tag_to_robot_rot
        robot_map_pos = tag_map_pos + tag_map_rot.apply(tag_to_robot_pos)

        yaw = robot_map_rot.as_euler("xyz")[2]
        self.current_state = np.array([robot_map_pos[0], robot_map_pos[1], yaw], dtype=float)
        self.has_pose = True
        self.last_tag_detection_time = time.time()

        # Log
        self.trajectory_log.append(
            {
                "time": self.last_tag_detection_time,
                "x": float(robot_map_pos[0]),
                "y": float(robot_map_pos[1]),
                "theta": float(yaw),
                "tag_id": int(tag_id),
            }
        )

        # Optionally broadcast odom -> base_link transform for RViz debugging
        self._broadcast_tf()

    def _broadcast_tf(self) -> None:
        """Broadcast TF transform from odom to base_link, like in HW2."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = float(self.current_state[0])
        t.transform.translation.y = float(self.current_state[1])
        t.transform.translation.z = 0.0

        yaw = float(self.current_state[2])
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(0.0)
        sp = math.sin(0.0)
        cr = math.cos(0.0)
        sr = math.sin(0.0)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def localization_update(self) -> None:
        """
        Main localization update:
          - Look for the closest *fresh* tag in TF (base_link -> tag_i).
          - If found, use it to update the robot pose.
          - Otherwise, keep last pose but mark pose as stale.
        """
        if not self.tag_positions:
            return

        now_ros = self.get_clock().now()
        closest_tag_id = None
        closest_observation: Optional[TransformStamped] = None
        closest_distance = float("inf")

        for tag_id in self.tag_positions.keys():
            tag_frame = f"tag_{tag_id}"
            try:
                obs = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    tag_frame,
                    rclpy.time.Time()
                )
            except Exception:
                continue

            # Discard stale detections
            transform_time = rclpy.time.Time.from_msg(obs.header.stamp)
            time_diff = (now_ros - transform_time).nanoseconds / 1e9
            if time_diff > self.tag_stale_threshold:
                continue

            dx = obs.transform.translation.x
            dy = obs.transform.translation.y
            dz = obs.transform.translation.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if distance < closest_distance:
                closest_distance = distance
                closest_tag_id = tag_id
                closest_observation = obs

        if closest_observation is not None and closest_tag_id is not None:
            self._update_pose_from_tag(closest_tag_id, closest_observation)
        # else: no update; pose may become stale and control_loop will handle it

    # ------------------------------------------------------------------
    # Coverage control: random waypoint coverage in workspace
    # ------------------------------------------------------------------
    def _sample_random_waypoint(self) -> np.ndarray:
        """
        Sample a random waypoint [x, y, yaw] inside the workspace bounds,
        but outside the obstacle box (with margins).
        """
        # workspace with a small margin from the walls
        margin = self.boundary_margin
        xmin = self.workspace_min_x + margin
        xmax = self.workspace_max_x - margin
        ymin = self.workspace_min_y + margin
        ymax = self.workspace_max_y - margin

        for _ in range(1000):
            gx = float(self.rng.uniform(xmin, xmax))
            gy = float(self.rng.uniform(ymin, ymax))

            # reject if inside obstacle (with margin)
            if self.has_obstacle:
                if (
                    self.obstacle_min_x - self.obstacle_margin <= gx <= self.obstacle_max_x + self.obstacle_margin
                    and self.obstacle_min_y - self.obstacle_margin <= gy <= self.obstacle_max_y + self.obstacle_margin
                ):
                    continue

            # random desired orientation
            gyaw = float(self.rng.uniform(-math.pi, math.pi))
            return np.array([gx, gy, gyaw], dtype=float)

        # fallback (should not happen)
        return np.array(
            [self.workspace_center_x, self.workspace_center_y, 0.0],
            dtype=float,
        )

    def _compute_control_to_waypoint(self, wp: np.ndarray) -> Tuple[float, float]:
        """
        Simple P controller:
          - v proportional to distance to waypoint (capped at explore_speed)
          - w proportional to heading error to waypoint
        """
        x, y, yaw = self.current_state
        gx, gy, gyaw = wp

        dx = gx - x
        dy = gy - y
        distance = math.sqrt(dx * dx + dy * dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - yaw
        # wrap to [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # linear velocity
        v = self.k_v * distance
        v = min(v, self.explore_speed)

        # angular velocity
        w = self.k_w * heading_error
        w = max(-self.max_angular_speed, min(self.max_angular_speed, w))

        # If very close, focus on final orientation
        if distance < self.waypoint_radius:
            # orient toward gyaw
            yaw_err = gyaw - yaw
            yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi
            v = 0.0
            w = self.k_w * yaw_err
            w = max(-self.max_angular_speed, min(self.max_angular_speed, w))

        return float(v), float(w)

    def _waypoint_reached(self, wp: np.ndarray) -> bool:
        x, y, yaw = self.current_state
        gx, gy, gyaw = wp
        dx = gx - x
        dy = gy - y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance < self.waypoint_radius

    def control_loop(self) -> None:
        """
        Main coverage behavior:
          1. If pose is stale or unavailable -> spin in place to reacquire tags.
          2. Else, ensure we have a current waypoint; if not, sample a new one.
          3. Drive toward the waypoint using a simple P controller.
          4. When the waypoint is reached, sample a new one.
        """
        twist = Twist()
        now = time.time()

        # use for debugging 
        age = now - self.last_tag_detection_time if self.last_tag_detection_time > 0.0 else -1.0
        self.get_logger().info(
            f"[control_loop] has_pose={self.has_pose}, tag_age={age:.2f}s, wp={self.current_waypoint}"
        )

        if (not self.has_pose) or (now - self.last_tag_detection_time > self.lost_pose_timeout):
            # No reliable pose: search for tags
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular_speed * 0.7
            self.cmd_vel_pub.publish(twist)
            return

        # We have a pose: pick waypoint if needed
        if self.current_waypoint is None or self._waypoint_reached(self.current_waypoint):
            self.current_waypoint = self._sample_random_waypoint()
            gx, gy, gyaw = self.current_waypoint
            self.get_logger().info(
                f"New coverage waypoint: x={gx:.2f}, y={gy:.2f}, yaw={gyaw:.2f}"
            )

        v, w = self._compute_control_to_waypoint(self.current_waypoint)
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Trajectory logging
    # ------------------------------------------------------------------
    def save_trajectory_log(self) -> None:
        if not self.trajectory_log:
            return
        try:
            os.makedirs(os.path.dirname(self.trajectory_log_path), exist_ok=True)
            with open(self.trajectory_log_path, "w") as f:
                json.dump(self.trajectory_log, f, indent=2)
            self.get_logger().info(
                f"Saved {len(self.trajectory_log)} trajectory samples to {self.trajectory_log_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to save trajectory log: {e}")

    def destroy_node(self) -> bool:
        self.save_trajectory_log()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Hw5CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("HW5 Coverage Node stopped by keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
