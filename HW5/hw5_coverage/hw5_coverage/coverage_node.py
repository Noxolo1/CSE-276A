#!/usr/bin/env python3
import math
import os
import json
from typing import List, Dict, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
    StaticTransformBroadcaster,
)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

import yaml


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw from quaternion (assuming ROS REP-103 convention)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class Hw5CoverageNode(Node):
    """
    HW5 coverage / Roomba-style controller.

    - Uses AprilTags for localization (map -> tag_i, camera_frame -> tag_i, base_link -> camera_frame).
    - Keeps the robot inside a virtual 8x8ft workspace.
    - Treats tags 8-11 as an internal obstacle region.
    - Simple subsumption-style behaviors:
      * tag_search > obstacle_avoidance > boundary_avoidance > wander
    """

    def __init__(self) -> None:
        super().__init__('hw5_coverage_node')

        # Frames
        self.map_frame = 'map'
        self.base_frame = 'base_link'
        self.camera_frame = 'camera_frame'

        # Parameters
        self.declare_parameter('apriltag_map_file', 'apriltags_position.yaml')
        self.declare_parameter('trajectory_log_file', 'hw5_coverage_trajectory.json')
        self.declare_parameter('explore_speed', 0.15)      # m/s
        self.declare_parameter('boundary_margin', 0.15)    # m (virtual wall thickness)
        self.declare_parameter('obstacle_margin', 0.10)    # m around obstacle
        self.declare_parameter('lost_pose_timeout', 2.0)   # s before we start spinning to reacquire tags

        self.explore_speed = float(self.get_parameter('explore_speed').value)
        self.boundary_margin = float(self.get_parameter('boundary_margin').value)
        self.obstacle_margin = float(self.get_parameter('obstacle_margin').value)
        self.lost_pose_timeout = float(self.get_parameter('lost_pose_timeout').value)

        # Velocity limits and turning gains
        self.v_max = 0.25
        self.w_max = 1.0
        self.turn_gain = 1.5      # for boundary / obstacle corrections
        self.search_angular_speed = 0.6  # spinning when lost
        self.wander_omega_amp = 0.3      # sinusoidal wandering
        self.wander_omega_freq = 0.2     # rad/s in sin()

        # Publishers and TF
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Load tag map and publish static transforms
        self.tag_data = self._load_tag_map()
        self._compute_workspace_and_obstacle_bounds(self.tag_data)
        self._publish_static_transforms(self.tag_data)

        # Internal state
        self.current_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.has_pose: bool = False
        self.last_pose_time: float = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Trajectory logging
        log_file_name = self.get_parameter('trajectory_log_file').value
        self.trajectory_log_path = os.path.join(
            os.path.expanduser('~'),
            'ros2_ws',
            log_file_name,
        )
        self.trajectory_log: List[Dict] = []

        # Timers
        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.localization_timer = self.create_timer(0.1, self.update_pose_from_tf)

        self.get_logger().info('HW5 Coverage Node initialized')

    # --------- Map / Tag loading and static TF publishing ---------

    def _resolve_apriltag_map_path(self) -> str:
        filename = self.get_parameter('apriltag_map_file').value
        # absolute path
        if os.path.isabs(filename) and os.path.exists(filename):
            return filename

        # Try package share
        if get_package_share_directory is not None:
            try:
                pkg_share = get_package_share_directory('hw5_coverage')
                candidate = os.path.join(pkg_share, 'configs', filename)
                if os.path.exists(candidate):
                    return candidate
            except Exception:
                pass

        # fallback: CWD
        candidate = os.path.join(os.getcwd(), filename)
        return candidate

    def _load_tag_map(self) -> List[Dict]:
        yaml_path = self._resolve_apriltag_map_path()
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Could not find AprilTag map file: {yaml_path}")
            return []

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        tags = data.get('apriltags', [])
        self.get_logger().info(f"Loaded {len(tags)} tags from {yaml_path}")
        return tags

    def _compute_workspace_and_obstacle_bounds(self, tags: List[Dict]) -> None:
        """Compute workspace bounding box from tags 0–7 and obstacle from tags 8–11."""
        if not tags:
            self.workspace_min_x = -1.0
            self.workspace_max_x = 1.0
            self.workspace_min_y = -1.0
            self.workspace_max_y = 1.0
            self.has_obstacle = False
            self.get_logger().warn("No tags loaded; using dummy workspace bounds [-1,1] x [-1,1]")
            return

        obstacle_ids = {8, 9, 10, 11}

        workspace_tags = [t for t in tags if t.get('id') not in obstacle_ids]
        obstacle_tags = [t for t in tags if t.get('id') in obstacle_ids]

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

        self.get_logger().info(
            f"Workspace bounds: x=[{self.workspace_min_x:.3f}, {self.workspace_max_x:.3f}], "
            f"y=[{self.workspace_min_y:.3f}, {self.workspace_max_y:.3f}]"
        )

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

            self.get_logger().info(
                f"Obstacle bounds: x=[{self.obstacle_min_x:.3f}, {self.obstacle_max_x:.3f}], "
                f"y=[{self.obstacle_min_y:.3f}, {self.obstacle_max_y:.3f}]"
            )
        else:
            self.has_obstacle = False
            self.get_logger().info("No obstacle tags found; running with no internal obstacle.")

    def _publish_static_transforms(self, tags: List[Dict]) -> None:
        """Publish map->tag_i and base_link->camera_frame as static TF."""
        transforms: List[TransformStamped] = []
        stamp = self.get_clock().now().to_msg()

        # map -> tag_i from the YAML
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

        # base_link -> camera_frame (same transform you used in previous HWs)
        cam = TransformStamped()
        cam.header.stamp = stamp
        cam.header.frame_id = self.base_frame
        cam.child_frame_id = self.camera_frame

        cam.transform.translation.x = 0.0675
        cam.transform.translation.y = 0.0
        cam.transform.translation.z = 0.035

        # from your existing camera_tf.py
        cam.transform.rotation.w = 0.5
        cam.transform.rotation.x = -0.5
        cam.transform.rotation.y = 0.5
        cam.transform.rotation.z = -0.5

        transforms.append(cam)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f"Published {len(transforms)} static transforms (map->tags + base_link->camera_frame)"
        )

    # --------- Localization via TF ---------

    def update_pose_from_tf(self) -> None:
        """Use TF tree to get map -> base_link pose."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            # If we have never had a pose, log occasionally. Otherwise rely on timeout in control_loop.
            if not self.has_pose:
                self.get_logger().warn(f"TF lookup (map -> base_link) failed: {ex}")
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        x = float(t.x)
        y = float(t.y)
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.current_pose = (x, y, yaw)
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_pose_time = now
        self.has_pose = True

        # Log trajectory for the report
        self.trajectory_log.append({
            "time": now,
            "x": x,
            "y": y,
            "theta": yaw,
        })

    # --------- Behavior helpers ---------

    def _is_near_boundary(self, x: float, y: float) -> bool:
        dx_left = x - self.workspace_min_x
        dx_right = self.workspace_max_x - x
        dy_bottom = y - self.workspace_min_y
        dy_top = self.workspace_max_y - y
        min_dist = min(dx_left, dx_right, dy_bottom, dy_top)
        return min_dist < self.boundary_margin

    def _is_inside_obstacle(self, x: float, y: float) -> bool:
        if not self.has_obstacle:
            return False
        return (
            self.obstacle_min_x - self.obstacle_margin <= x <= self.obstacle_max_x + self.obstacle_margin and
            self.obstacle_min_y - self.obstacle_margin <= y <= self.obstacle_max_y + self.obstacle_margin
        )

    # --------- Control loop / behaviors ---------

    def control_loop(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        twist = Twist()

        # Behavior 1: search for tags if localization is lost
        if (not self.has_pose) or (
            self.last_pose_time is not None and
            now - self.last_pose_time > self.lost_pose_timeout
        ):
            twist.linear.x = 0.0
            twist.angular.z = self.search_angular_speed
            self.cmd_vel_pub.publish(twist)
            return

        x, y, yaw = self.current_pose

        v = 0.0
        w = 0.0

        # Behavior 2: obstacle avoidance (higher priority than boundary)
        if self._is_inside_obstacle(x, y):
            # Turn away from obstacle center
            desired_heading = math.atan2(
                y - self.obstacle_center_y,
                x - self.obstacle_center_x
            )
            heading_error = wrap_angle(desired_heading - yaw)
            v = 0.0
            w = self.turn_gain * heading_error

        # Behavior 3: boundary avoidance
        elif self._is_near_boundary(x, y):
            # Turn toward the workspace center
            desired_heading = math.atan2(
                self.workspace_center_y - y,
                self.workspace_center_x - x
            )
            heading_error = wrap_angle(desired_heading - yaw)
            v = 0.0
            w = self.turn_gain * heading_error

        # Behavior 4: wandering coverage
        else:
            v = self.explore_speed
            # simple sinusoidal steering for sweeping behaviour
            w = self.wander_omega_amp * math.sin(self.wander_omega_freq * now)

        # Saturate velocities
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)

    # --------- Trajectory logging ---------

    def save_trajectory_log(self) -> None:
        if not self.trajectory_log:
            return
        try:
            os.makedirs(os.path.dirname(self.trajectory_log_path), exist_ok=True)
            with open(self.trajectory_log_path, 'w') as f:
                json.dump(self.trajectory_log, f, indent=2)
            self.get_logger().info(
                f"Saved {len(self.trajectory_log)} trajectory points to {self.trajectory_log_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to save trajectory log: {e}")

    def destroy_node(self) -> bool:
        # dump trajectory on shutdown (CTRL-C)
        self.save_trajectory_log()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Hw5CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HW5 Coverage Node stopped by keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
