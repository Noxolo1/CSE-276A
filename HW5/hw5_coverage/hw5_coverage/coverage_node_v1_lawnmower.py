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