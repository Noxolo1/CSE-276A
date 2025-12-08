# #!/usr/bin/env python3
# """
# HW5 coverage controller node.

# This node reuses the HW2 reference localization pipeline:

# - Use AprilTag ground-truth poses from apriltags_position.yaml (map/world frame).
# - Use TF to get base_link -> tag_i transforms (via camera_tf + apriltag_ros).
# - Fuse those to compute the robot pose (x, y, yaw) in the map/world frame.

# On top of that, it implements a simple randomized waypoint-coverage strategy:
# - The workspace bounds come from tags 0–7.
# - Tags 8–11 (if present) define an internal obstacle region.
# - The node repeatedly samples random waypoints inside the workspace but
#   outside the obstacle box and drives the robot toward them using a simple
#   proportional controller on distance and heading.

# No EKF/SLAM is used here (that is only needed for extra credit).
# """

# import math
# import os
# import json
# import time
# from typing import Dict, List, Tuple, Optional

# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, TransformStamped
# from tf2_ros import Buffer, TransformListener, TransformBroadcaster
# from ament_index_python.packages import get_package_share_directory
# from scipy.spatial.transform import Rotation
# import yaml


# class Hw5CoverageNode(Node):
#     def __init__(self) -> None:
#         super().__init__("hw5_coverage_node")

#         # --- Publishers / TF ---
#         self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Frames (matching HW2/3 style)
#         self.odom_frame = "odom"       # world / map frame
#         self.base_frame = "base_link"

#         # --- Parameters ---
#         self.declare_parameter("apriltag_map_file", "apriltags_position.yaml")
#         self.declare_parameter("trajectory_log_file", "hw5_coverage_trajectory.json")

#         # coverage control parameters
#         self.declare_parameter("explore_speed", 0.18)          # max forward speed [m/s]
#         self.declare_parameter("max_angular_speed", 0.9)       # [rad/s]
#         self.declare_parameter("boundary_margin", 0.15)        # [m]
#         self.declare_parameter("obstacle_margin", 0.10)        # [m]
#         self.declare_parameter("waypoint_radius", 0.10)        # [m] distance to consider waypoint reached
#         self.declare_parameter("lost_pose_timeout", 2.0)       # [s] before we start searching
#         self.declare_parameter("tag_stale_threshold", 0.25)    # [s] max allowed transform age
#         self.declare_parameter("k_v", 0.5)                     # [1/s] gain for linear velocity
#         self.declare_parameter("k_w", 1.5)                     # [1/s] gain for angular velocity

#         # read parameter values
#         self.explore_speed = float(self.get_parameter("explore_speed").value)
#         self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
#         self.boundary_margin = float(self.get_parameter("boundary_margin").value)
#         self.obstacle_margin = float(self.get_parameter("obstacle_margin").value)
#         self.waypoint_radius = float(self.get_parameter("waypoint_radius").value)
#         self.lost_pose_timeout = float(self.get_parameter("lost_pose_timeout").value)
#         self.tag_stale_threshold = float(self.get_parameter("tag_stale_threshold").value)
#         self.k_v = float(self.get_parameter("k_v").value)
#         self.k_w = float(self.get_parameter("k_w").value)

#         # --- AprilTag map and workspace geometry ---
#         self.tag_positions: Dict[int, Dict] = {}
#         self.tag_ids: List[int] = []
#         self._load_tag_configurations()
#         self._compute_workspace_and_obstacle_bounds()

#         # --- Robot pose state (odom/map frame) ---
#         # current_state = [x, y, yaw]
#         self.current_state = np.array([0.0, 0.0, 0.0], dtype=float)
#         self.has_pose: bool = False
#         self.last_tag_detection_time: float = 0.0

#         # --- Coverage / waypoint state ---
#         self.current_waypoint: Optional[np.ndarray] = None  # [x, y, yaw_goal]
#         self.rng = np.random.default_rng()

#         # --- Trajectory logging ---
#         # We want a unique file per run: hw5_coverage_trajectory_TIMESTAMP.json
#         traj_base = self.get_parameter("trajectory_log_file").value
#         # Allow the parameter to be given with or without ".json"
#         if traj_base.endswith(".json"):
#             traj_base = traj_base[:-5]

#         # e.g. 20251206-113045
#         self.run_timestamp = time.strftime("%Y%m%d-%H%M%S")
#         filename = f"{traj_base}_{self.run_timestamp}.json"

#         self.trajectory_log_path = os.path.join(
#             os.path.expanduser("~"), "ros2_ws", filename
#         )
#         self.trajectory_log: List[Dict] = []

#         self.get_logger().info(
#             f"HW5 trajectory log will be saved to: {self.trajectory_log_path}"
#         )


#         # --- Timers ---
#         self.dt = 0.1
#         self.control_timer = self.create_timer(self.dt, self.control_loop)
#         self.localization_timer = self.create_timer(0.1, self.localization_update)

#         self.get_logger().info("HW5 Coverage Node initialized")

#     # ------------------------------------------------------------------
#     # AprilTag map loading & workspace geometry (reusing HW2 patterns)
#     # ------------------------------------------------------------------
#     def _resolve_apriltag_map_path(self) -> str:
#         filename = self.get_parameter("apriltag_map_file").value
#         if os.path.isabs(filename) and os.path.exists(filename):
#             return filename

#         try:
#             pkg_share = get_package_share_directory("hw5_coverage")
#             candidate = os.path.join(pkg_share, "configs", filename)
#             if os.path.exists(candidate):
#                 return candidate
#         except Exception:
#             pass

#         return os.path.join(os.getcwd(), filename)

#     def _load_tag_configurations(self) -> None:
#         """Load AprilTag positions / orientations from YAML (like HW2)."""
#         yaml_path = self._resolve_apriltag_map_path()
#         if not os.path.exists(yaml_path):
#             self.get_logger().error(f"Could not find AprilTag map file: {yaml_path}")
#             return

#         with open(yaml_path, "r") as file:
#             data = yaml.safe_load(file)

#         tags_data = data.get("apriltags", [])
#         for tag in tags_data:
#             tag_id = tag.get("id")
#             if tag_id is None:
#                 continue

#             self.tag_positions[int(tag_id)] = {
#                 "x": float(tag["x"]),
#                 "y": float(tag["y"]),
#                 "z": float(tag.get("z", 0.0)),
#                 "qx": float(tag.get("qx", 0.0)),
#                 "qy": float(tag.get("qy", 0.0)),
#                 "qz": float(tag.get("qz", 0.0)),
#                 "qw": float(tag.get("qw", 1.0)),
#             }

#         self.tag_ids = sorted(self.tag_positions.keys())
#         self.get_logger().info(f"Loaded {len(self.tag_ids)} AprilTags from {yaml_path}")

#     def _compute_workspace_and_obstacle_bounds(self) -> None:
#         """
#         HW5: no internal obstacle. All tags (0–11) are on the outer boundary
#         of the 8x8 ft workspace. We just use ALL tags to define the workspace,
#         and we disable obstacle handling.
#         """
#         if not self.tag_positions:
#             self.workspace_min_x = -1.0
#             self.workspace_max_x = 1.0
#             self.workspace_min_y = -1.0
#             self.workspace_max_y = 1.0
#             self.workspace_center_x = 0.0
#             self.workspace_center_y = 0.0
#             self.has_obstacle = False
#             self.get_logger().warn(
#                 "No tags in map; using dummy workspace [-1,1] x [-1,1] and no obstacle."
#             )
#             return

#         # Use ALL tags to estimate workspace bounds (0–11 are all boundary tags)
#         xs = [t["x"] for t in self.tag_positions.values()]
#         ys = [t["y"] for t in self.tag_positions.values()]

#         self.workspace_min_x = min(xs)
#         self.workspace_max_x = max(xs)
#         self.workspace_min_y = min(ys)
#         self.workspace_max_y = max(ys)
#         self.workspace_center_x = 0.5 * (self.workspace_min_x + self.workspace_max_x)
#         self.workspace_center_y = 0.5 * (self.workspace_min_y + self.workspace_max_y)

#         self.get_logger().info(
#             f"Workspace bounds (no obstacle): "
#             f"x=[{self.workspace_min_x:.3f}, {self.workspace_max_x:.3f}], "
#             f"y=[{self.workspace_min_y:.3f}, {self.workspace_max_y:.3f}]"
#         )

#         # No internal obstacle for HW5
#         self.has_obstacle = False
#         self.get_logger().info("HW5: treating all tags as boundary; no internal obstacle.")


#     # ------------------------------------------------------------------
#     # Localization (reusing HW2: base_link -> tag_i + YAML)
#     # ------------------------------------------------------------------
#     def _update_pose_from_tag(self, tag_id: int, observation: TransformStamped) -> None:
#         """
#         Compute robot pose in map/odom frame from:
#           - map->tag_i from YAML (self.tag_positions)
#           - base_link->tag_i from TF (observation)
#         and update self.current_state = [x, y, yaw].
#         """
#         tag_map = self.tag_positions[tag_id]

#         tag_map_pos = np.array([tag_map["x"], tag_map["y"], tag_map["z"]])
#         tag_map_rot = Rotation.from_quat(
#             [tag_map["qx"], tag_map["qy"], tag_map["qz"], tag_map["qw"]]
#         )

#         # observation: transform from base_link to tag_i
#         obs_pos = np.array(
#             [
#                 observation.transform.translation.x,
#                 observation.transform.translation.y,
#                 observation.transform.translation.z,
#             ]
#         )
#         obs_rot = Rotation.from_quat(
#             [
#                 observation.transform.rotation.x,
#                 observation.transform.rotation.y,
#                 observation.transform.rotation.z,
#                 observation.transform.rotation.w,
#             ]
#         )

#         # base_link -> tag ; we want tag -> base_link
#         tag_to_robot_rot = obs_rot.inv()
#         tag_to_robot_pos = -tag_to_robot_rot.apply(obs_pos)

#         # Compose: map -> base_link = map->tag * tag->base_link
#         robot_map_rot = tag_map_rot * tag_to_robot_rot
#         robot_map_pos = tag_map_pos + tag_map_rot.apply(tag_to_robot_pos)

#         yaw = robot_map_rot.as_euler("xyz")[2]
#         self.current_state = np.array([robot_map_pos[0], robot_map_pos[1], yaw], dtype=float)
#         self.has_pose = True
#         self.last_tag_detection_time = time.time()

#         # Log
#         self.trajectory_log.append(
#             {
#                 "time": self.last_tag_detection_time,
#                 "x": float(robot_map_pos[0]),
#                 "y": float(robot_map_pos[1]),
#                 "theta": float(yaw),
#                 "tag_id": int(tag_id),
#             }
#         )

#         # Optionally broadcast odom -> base_link transform for RViz debugging
#         self._broadcast_tf()

#     def _broadcast_tf(self) -> None:
#         """Broadcast TF transform from odom to base_link, like in HW2."""
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.odom_frame
#         t.child_frame_id = self.base_frame

#         t.transform.translation.x = float(self.current_state[0])
#         t.transform.translation.y = float(self.current_state[1])
#         t.transform.translation.z = 0.0

#         yaw = float(self.current_state[2])
#         cy = math.cos(yaw * 0.5)
#         sy = math.sin(yaw * 0.5)
#         cp = math.cos(0.0)
#         sp = math.sin(0.0)
#         cr = math.cos(0.0)
#         sr = math.sin(0.0)

#         qw = cr * cp * cy + sr * sp * sy
#         qx = sr * cp * cy - cr * sp * sy
#         qy = cr * sp * cy + sr * cp * sy
#         qz = cr * cp * sy - sr * sp * cy

#         t.transform.rotation.x = qx
#         t.transform.rotation.y = qy
#         t.transform.rotation.z = qz
#         t.transform.rotation.w = qw

#         self.tf_broadcaster.sendTransform(t)

#     def localization_update(self) -> None:
#         """
#         Main localization update:
#           - Look for the closest *fresh* tag in TF (base_link -> tag_i).
#           - If found, use it to update the robot pose.
#           - Otherwise, keep last pose but mark pose as stale.
#         """
#         if not self.tag_positions:
#             return

#         now_ros = self.get_clock().now()
#         closest_tag_id = None
#         closest_observation: Optional[TransformStamped] = None
#         closest_distance = float("inf")

#         for tag_id in self.tag_positions.keys():
#             tag_frame = f"tag_{tag_id}"
#             try:
#                 obs = self.tf_buffer.lookup_transform(
#                     self.base_frame,
#                     tag_frame,
#                     rclpy.time.Time()
#                 )
#             except Exception:
#                 continue

#             # Discard stale detections
#             transform_time = rclpy.time.Time.from_msg(obs.header.stamp)
#             time_diff = (now_ros - transform_time).nanoseconds / 1e9
#             if time_diff > self.tag_stale_threshold:
#                 continue

#             dx = obs.transform.translation.x
#             dy = obs.transform.translation.y
#             dz = obs.transform.translation.z
#             distance = math.sqrt(dx * dx + dy * dy + dz * dz)

#             if distance < closest_distance:
#                 closest_distance = distance
#                 closest_tag_id = tag_id
#                 closest_observation = obs

#         if closest_observation is not None and closest_tag_id is not None:
#             self._update_pose_from_tag(closest_tag_id, closest_observation)
#         # else: no update; pose may become stale and control_loop will handle it

#     # ------------------------------------------------------------------
#     # Coverage control: random waypoint coverage in workspace
#     # ------------------------------------------------------------------
#     def _sample_random_waypoint(self) -> np.ndarray:
#         """
#         Sample a random waypoint [x, y, yaw] inside the workspace bounds,
#         but outside the obstacle box (with margins).
#         """
#         # workspace with a small margin from the walls
#         margin = self.boundary_margin
#         xmin = self.workspace_min_x + margin
#         xmax = self.workspace_max_x - margin
#         ymin = self.workspace_min_y + margin
#         ymax = self.workspace_max_y - margin

#         for _ in range(1000):
#             gx = float(self.rng.uniform(xmin, xmax))
#             gy = float(self.rng.uniform(ymin, ymax))

#             # reject if inside obstacle (with margin)
#             if self.has_obstacle:
#                 if (
#                     self.obstacle_min_x - self.obstacle_margin <= gx <= self.obstacle_max_x + self.obstacle_margin
#                     and self.obstacle_min_y - self.obstacle_margin <= gy <= self.obstacle_max_y + self.obstacle_margin
#                 ):
#                     continue

#             # random desired orientation
#             gyaw = float(self.rng.uniform(-math.pi, math.pi))
#             return np.array([gx, gy, gyaw], dtype=float)

#         # fallback (should not happen)
#         return np.array(
#             [self.workspace_center_x, self.workspace_center_y, 0.0],
#             dtype=float,
#         )

#     def _compute_control_to_waypoint(self, wp: np.ndarray) -> Tuple[float, float]:
#         """
#         Simple P controller:
#           - v proportional to distance to waypoint (capped at explore_speed)
#           - w proportional to heading error to waypoint
#         """
#         x, y, yaw = self.current_state
#         gx, gy, gyaw = wp

#         dx = gx - x
#         dy = gy - y
#         distance = math.sqrt(dx * dx + dy * dy)

#         desired_heading = math.atan2(dy, dx)
#         heading_error = desired_heading - yaw
#         # wrap to [-pi, pi]
#         heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

#         # linear velocity
#         v = self.k_v * distance
#         v = min(v, self.explore_speed)

#         # angular velocity
#         w = self.k_w * heading_error
#         w = max(-self.max_angular_speed, min(self.max_angular_speed, w))

#         # If very close, focus on final orientation
#         if distance < self.waypoint_radius:
#             # orient toward gyaw
#             yaw_err = gyaw - yaw
#             yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi
#             v = 0.0
#             w = self.k_w * yaw_err
#             w = max(-self.max_angular_speed, min(self.max_angular_speed, w))

#         return float(v), float(w)

#     def _waypoint_reached(self, wp: np.ndarray) -> bool:
#         x, y, yaw = self.current_state
#         gx, gy, gyaw = wp
#         dx = gx - x
#         dy = gy - y
#         distance = math.sqrt(dx * dx + dy * dy)
#         return distance < self.waypoint_radius

#     def control_loop(self) -> None:
#         """
#         Main coverage behavior:
#           1. If pose is stale or unavailable -> spin in place to reacquire tags.
#           2. Else, ensure we have a current waypoint; if not, sample a new one.
#           3. Drive toward the waypoint using a simple P controller.
#           4. When the waypoint is reached, sample a new one.
#         """
#         twist = Twist()
#         now = time.time()

#         # use for debugging 
#         age = now - self.last_tag_detection_time if self.last_tag_detection_time > 0.0 else -1.0
#         self.get_logger().info(
#             f"[control_loop] has_pose={self.has_pose}, tag_age={age:.2f}s, wp={self.current_waypoint}"
#         )

#         if (not self.has_pose) or (now - self.last_tag_detection_time > self.lost_pose_timeout):
#             # No reliable pose: search for tags
#             twist.linear.x = 0.0
#             twist.angular.z = self.max_angular_speed * 0.7
#             self.cmd_vel_pub.publish(twist)
#             return

#         # We have a pose: pick waypoint if needed
#         if self.current_waypoint is None or self._waypoint_reached(self.current_waypoint):
#             self.current_waypoint = self._sample_random_waypoint()
#             gx, gy, gyaw = self.current_waypoint
#             self.get_logger().info(
#                 f"New coverage waypoint: x={gx:.2f}, y={gy:.2f}, yaw={gyaw:.2f}"
#             )

#         v, w = self._compute_control_to_waypoint(self.current_waypoint)
#         twist.linear.x = v
#         twist.angular.z = w
#         self.cmd_vel_pub.publish(twist)

#     # ------------------------------------------------------------------
#     # Trajectory logging
#     # ------------------------------------------------------------------
#     def save_trajectory_log(self) -> None:
#         if not self.trajectory_log:
#             return
#         try:
#             os.makedirs(os.path.dirname(self.trajectory_log_path), exist_ok=True)
#             with open(self.trajectory_log_path, "w") as f:
#                 json.dump(self.trajectory_log, f, indent=2)
#             self.get_logger().info(
#                 f"Saved {len(self.trajectory_log)} trajectory samples to {self.trajectory_log_path}"
#             )
#         except Exception as e:
#             self.get_logger().error(f"Failed to save trajectory log: {e}")

#     def destroy_node(self) -> bool:
#         self.save_trajectory_log()
#         return super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = Hw5CoverageNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("HW5 Coverage Node stopped by keyboard interrupt")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()



#!/usr/bin/env python3
"""
HW5 Coverage Node - Version 1: Lawnmower Pattern with Filtered Localization

Features:
- Filtered AprilTag localization (exponential smoothing)
- Lawnmower coverage pattern option
- Wandering coverage option
- No EKF required
"""
import math
import os
import json
from typing import List, Dict, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
    StaticTransformBroadcaster,
    TransformBroadcaster,
)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

import yaml
import numpy as np
from scipy.spatial.transform import Rotation


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PoseFilter:
    """Exponential moving average filter for pose smoothing"""
    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha  # Smoothing factor (0=no update, 1=no smoothing)
        self.filtered_pose: Optional[np.ndarray] = None
        
    def update(self, pose: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Apply exponential smoothing to pose"""
        if self.filtered_pose is None:
            self.filtered_pose = np.array(pose)
            return pose
        
        # Smooth x, y
        self.filtered_pose[0] = self.alpha * pose[0] + (1 - self.alpha) * self.filtered_pose[0]
        self.filtered_pose[1] = self.alpha * pose[1] + (1 - self.alpha) * self.filtered_pose[1]
        
        # Smooth yaw (handle wraparound)
        yaw_diff = wrap_angle(pose[2] - self.filtered_pose[2])
        self.filtered_pose[2] = wrap_angle(self.filtered_pose[2] + self.alpha * yaw_diff)
        
        return tuple(self.filtered_pose)


class Hw5CoverageNode(Node):
    """HW5 coverage controller with lawnmower pattern and filtered localization"""

    def __init__(self) -> None:
        super().__init__('hw5_coverage_node')

        # Frames
        self.map_frame = 'map'
        self.base_frame = 'base_link'
        self.camera_frame = 'camera_frame'

        # Parameters
        self.declare_parameter('apriltag_map_file', 'apriltags_position.yaml')
        self.declare_parameter('trajectory_log_file', 'hw5_coverage_trajectory.json')
        self.declare_parameter('coverage_mode', 'lawnmower')  # 'lawnmower' or 'wander'
        self.declare_parameter('explore_speed', 0.12)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('boundary_margin', 0.20)
        self.declare_parameter('obstacle_margin', 0.15)
        self.declare_parameter('lost_pose_timeout', 2.0)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('angle_tolerance', 0.15)
        self.declare_parameter('lawnmower_spacing', 0.3)
        self.declare_parameter('pose_filter_alpha', 0.4)

        self.coverage_mode = str(self.get_parameter('coverage_mode').value)
        self.explore_speed = float(self.get_parameter('explore_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.boundary_margin = float(self.get_parameter('boundary_margin').value)
        self.obstacle_margin = float(self.get_parameter('obstacle_margin').value)
        self.lost_pose_timeout = float(self.get_parameter('lost_pose_timeout').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.lawnmower_spacing = float(self.get_parameter('lawnmower_spacing').value)

        # Velocity limits
        self.v_max = 0.2
        self.w_max = 1.0
        self.search_angular_speed = 0.6

        # Publishers and TF
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Load tags and compute bounds
        self.tag_data = self._load_tag_map()
        self._compute_workspace_and_obstacle_bounds(self.tag_data)
        self._publish_static_transforms(self.tag_data)

        # Pose filtering
        filter_alpha = float(self.get_parameter('pose_filter_alpha').value)
        self.pose_filter = PoseFilter(alpha=filter_alpha)

        # State
        self.current_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.has_pose: bool = False
        self.last_pose_time: float = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Waypoints
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_waypoint_idx: int = 0
        
        if self.coverage_mode == 'lawnmower':
            self._generate_lawnmower_waypoints()
        
        self.stage = 'tag_search'

        # Trajectory logging
        log_file_name = self.get_parameter('trajectory_log_file').value
        self.trajectory_log_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', log_file_name)
        self.trajectory_log: List[Dict] = []

        # Timers
        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.localization_timer = self.create_timer(0.1, self.update_pose_from_tags)

        self.get_logger().info(f'HW5 Coverage Node - Mode: {self.coverage_mode}')
        if self.coverage_mode == 'lawnmower':
            self.get_logger().info(f'Generated {len(self.waypoints)} lawnmower waypoints')

    def _resolve_apriltag_map_path(self) -> str:
        filename = self.get_parameter('apriltag_map_file').value
        if os.path.isabs(filename) and os.path.exists(filename):
            return filename
        if get_package_share_directory is not None:
            try:
                pkg_share = get_package_share_directory('hw5_coverage')
                candidate = os.path.join(pkg_share, 'configs', filename)
                if os.path.exists(candidate):
                    return candidate
            except Exception:
                pass
        return os.path.join(os.getcwd(), filename)

    def _load_tag_map(self) -> List[Dict]:
        yaml_path = self._resolve_apriltag_map_path()
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Could not find AprilTag map: {yaml_path}")
            return []
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        tags = data.get('apriltags', [])
        self.get_logger().info(f"Loaded {len(tags)} tags")
        return tags

    def _compute_workspace_and_obstacle_bounds(self, tags: List[Dict]) -> None:
        if not tags:
            self.workspace_min_x = -1.0
            self.workspace_max_x = 1.0
            self.workspace_min_y = -1.0
            self.workspace_max_y = 1.0
            self.has_obstacle = False
            return

        obstacle_ids = {8, 9, 10, 11}
        workspace_tags = [t for t in tags if t.get('id') not in obstacle_ids]
        if not workspace_tags:
            workspace_tags = tags

        xs = [float(t['x']) for t in workspace_tags]
        ys = [float(t['y']) for t in workspace_tags]

        self.workspace_min_x = min(xs)
        self.workspace_max_x = max(xs)
        self.workspace_min_y = min(ys)
        self.workspace_max_y = max(ys)
        self.workspace_center_x = 0.5 * (self.workspace_min_x + self.workspace_max_x)
        self.workspace_center_y = 0.5 * (self.workspace_min_y + self.workspace_max_y)

        obstacle_tags = [t for t in tags if t.get('id') in obstacle_ids]
        if obstacle_tags:
            ox = [float(t['x']) for t in obstacle_tags]
            oy = [float(t['y']) for t in obstacle_tags]
            self.obstacle_min_x = min(ox)
            self.obstacle_max_x = max(ox)
            self.obstacle_min_y = min(oy)
            self.obstacle_max_y = max(oy)
            self.obstacle_center_x = 0.5 * (self.obstacle_min_x + self.obstacle_max_x)
            self.obstacle_center_y = 0.5 * (self.obstacle_min_y + self.obstacle_max_y)
            self.has_obstacle = True
        else:
            self.has_obstacle = False

    def _publish_static_transforms(self, tags: List[Dict]) -> None:
        """Only publish map->tag transforms (camera_tf node handles base_link->camera_frame)"""
        transforms: List[TransformStamped] = []
        stamp = self.get_clock().now().to_msg()

        for tag in tags:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = f"tag_{int(tag['id'])}"
            t.transform.translation.x = float(tag['x'])
            t.transform.translation.y = float(tag['y'])
            t.transform.translation.z = float(tag.get('z', 0.0))
            t.transform.rotation.x = float(tag.get('qx', 0.0))
            t.transform.rotation.y = float(tag.get('qy', 0.0))
            t.transform.rotation.z = float(tag.get('qz', 0.0))
            t.transform.rotation.w = float(tag.get('qw', 1.0))
            transforms.append(t)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Published {len(transforms)} map->tag transforms")

    def _generate_lawnmower_waypoints(self) -> None:
        """Generate lawnmower pattern"""
        x_min = self.workspace_min_x + self.boundary_margin
        x_max = self.workspace_max_x - self.boundary_margin
        y_min = self.workspace_min_y + self.boundary_margin
        y_max = self.workspace_max_y - self.boundary_margin

        y_current = y_min
        going_right = True
        
        while y_current <= y_max:
            if going_right:
                self.waypoints.append((x_min, y_current, 0.0))
                self.waypoints.append((x_max, y_current, 0.0))
            else:
                self.waypoints.append((x_max, y_current, math.pi))
                self.waypoints.append((x_min, y_current, math.pi))
            
            y_current += self.lawnmower_spacing
            going_right = not going_right

        self.waypoints.append((0.0, 0.0, 0.0))  # Return home

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def broadcast_robot_pose(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.current_pose[0])
        t.transform.translation.y = float(self.current_pose[1])
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, float(self.current_pose[2]))
        t.transform.rotation.x = qx; t.transform.rotation.y = qy
        t.transform.rotation.z = qz; t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def compute_robot_pose_from_tag(self, tag_id: int, obs: TransformStamped) -> Optional[Tuple[float, float, float]]:
        tag_data = None
        for tag in self.tag_data:
            if tag.get('id') == tag_id:
                tag_data = tag
                break
        if tag_data is None:
            return None
        
        tag_map_pos = np.array([tag_data['x'], tag_data['y'], tag_data['z']])
        tag_map_rot = Rotation.from_quat([tag_data['qx'], tag_data['qy'], tag_data['qz'], tag_data['qw']])
        
        obs_pos = np.array([obs.transform.translation.x, obs.transform.translation.y, obs.transform.translation.z])
        obs_rot = Rotation.from_quat([obs.transform.rotation.x, obs.transform.rotation.y, 
                                       obs.transform.rotation.z, obs.transform.rotation.w])
        
        tag_to_camera_rot = obs_rot.inv()
        tag_to_camera_pos = -tag_to_camera_rot.apply(obs_pos)
        camera_map_rot = tag_map_rot * tag_to_camera_rot
        camera_map_pos = tag_map_pos + tag_map_rot.apply(tag_to_camera_pos)
        
        # Camera to base (from camera_tf.py)
        camera_to_base_pos = np.array([-0.0675, 0.0, -0.035])
        camera_to_base_rot = Rotation.from_quat([-0.5, 0.5, -0.5, 0.5])
        
        robot_map_rot = camera_map_rot * camera_to_base_rot
        robot_map_pos = camera_map_pos + camera_map_rot.apply(camera_to_base_pos)
        yaw = robot_map_rot.as_euler('xyz')[2]
        
        return (float(robot_map_pos[0]), float(robot_map_pos[1]), float(yaw))

    def update_pose_from_tags(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        
        best_tag_id = None
        best_distance = float('inf')
        best_observation = None
        
        for tag in self.tag_data:
            tag_id = tag.get('id')
            if tag_id is None:
                continue
            try:
                obs = self.tf_buffer.lookup_transform(
                    self.camera_frame, f'tag_{tag_id}', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.01)
                )
                obs_time = rclpy.time.Time.from_msg(obs.header.stamp)
                if ((self.get_clock().now() - obs_time).nanoseconds / 1e9) > 0.5:
                    continue
                dx = obs.transform.translation.x
                dy = obs.transform.translation.y
                dz = obs.transform.translation.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                if distance < best_distance:
                    best_distance = distance
                    best_tag_id = tag_id
                    best_observation = obs
            except TransformException:
                continue
        
        if best_observation is not None:
            raw_pose = self.compute_robot_pose_from_tag(best_tag_id, best_observation)
            if raw_pose is not None:
                filtered_pose = self.pose_filter.update(raw_pose)
                self.current_pose = filtered_pose
                self.has_pose = True
                self.last_pose_time = now
                self.trajectory_log.append({
                    "time": now, "x": filtered_pose[0], "y": filtered_pose[1],
                    "theta": filtered_pose[2], "tag_id": best_tag_id
                })
                self.get_logger().info(
                    f"Tag{best_tag_id}: ({filtered_pose[0]:.2f},{filtered_pose[1]:.2f},{filtered_pose[2]:.2f})",
                    throttle_duration_sec=2.0
                )
        
        if self.has_pose:
            self.broadcast_robot_pose()

    def control_loop(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        twist = Twist()

        if (not self.has_pose) or (now - self.last_pose_time > self.lost_pose_timeout):
            twist.angular.z = self.search_angular_speed
            self.cmd_vel_pub.publish(twist)
            return

        x, y, yaw = self.current_pose

        if self.coverage_mode == 'lawnmower' and len(self.waypoints) > 0:
            self._lawnmower_control(twist, x, y, yaw)
        else:
            self._wandering_control(twist, x, y)

    def _lawnmower_control(self, twist: Twist, x: float, y: float, yaw: float) -> None:
        if self.current_waypoint_idx >= len(self.waypoints):
            self.cmd_vel_pub.publish(twist)
            return
        
        wp = self.waypoints[self.current_waypoint_idx]
        dx = wp[0] - x
        dy = wp[1] - y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if self.stage == 'tag_search':
            self.stage = 'rotate_to_goal'
        
        if self.stage == 'rotate_to_goal':
            desired = math.atan2(dy, dx)
            herr = wrap_angle(desired - yaw)
            if abs(herr) < self.angle_tolerance:
                self.stage = 'drive'
            else:
                twist.angular.z = self.turn_speed if herr > 0 else -self.turn_speed
        
        elif self.stage == 'drive':
            if distance < self.waypoint_tolerance:
                self.stage = 'rotate_to_orient'
            else:
                desired = math.atan2(dy, dx)
                herr = wrap_angle(desired - yaw)
                if abs(herr) > 0.3:
                    self.stage = 'rotate_to_goal'
                else:
                    twist.linear.x = self.explore_speed
                    twist.angular.z = 0.5 * herr
        
        elif self.stage == 'rotate_to_orient':
            herr = wrap_angle(wp[2] - yaw)
            if abs(herr) < self.angle_tolerance:
                self.current_waypoint_idx += 1
                self.stage = 'rotate_to_goal'
                self.get_logger().info(f"Waypoint {self.current_waypoint_idx}/{len(self.waypoints)}")
            else:
                twist.angular.z = self.turn_speed if herr > 0 else -self.turn_speed
        
        twist.linear.x = max(-self.v_max, min(self.v_max, twist.linear.x))
        twist.angular.z = max(-self.w_max, min(self.w_max, twist.angular.z))
        self.cmd_vel_pub.publish(twist)

    def _wandering_control(self, twist: Twist, x: float, y: float) -> None:
        # Simple wandering (not implemented fully - use lawnmower mode)
        twist.linear.x = self.explore_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def save_trajectory_log(self) -> None:
        if not self.trajectory_log:
            return
        try:
            os.makedirs(os.path.dirname(self.trajectory_log_path), exist_ok=True)
            with open(self.trajectory_log_path, 'w') as f:
                json.dump(self.trajectory_log, f, indent=2)
            self.get_logger().info(f"Saved {len(self.trajectory_log)} points")
        except Exception as e:
            self.get_logger().error(f"Failed to save log: {e}")

    def destroy_node(self) -> bool:
        self.save_trajectory_log()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Hw5CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()