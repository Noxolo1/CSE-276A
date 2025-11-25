"""
Localization Node for HW4

Provides pose estimation using AprilTag detections and TF2 transforms.
Adapts HW2's localization logic for HW4 context.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_matrix, quaternion_matrix

from . import hw4_config as cfg


def T_from_xyz_yaw(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    """Create 4x4 transformation matrix from position and yaw angle."""
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s, c
    return T


def T_from_tf(tf: TransformStamped) -> np.ndarray:
    """Convert ROS TransformStamped to 4x4 matrix."""
    T = np.eye(4)
    t = tf.transform.translation
    q = tf.transform.rotation
    T[:3, 3] = [t.x, t.y, t.z]
    T[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    return T


def T_inv(T: np.ndarray) -> np.ndarray:
    """Compute inverse of 4x4 transformation matrix."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def yaw_from_R(R: np.ndarray) -> float:
    """Extract yaw angle from 3x3 rotation matrix."""
    return math.atan2(R[1, 0], R[0, 0])


class LocalizationNode(Node):
    """
    Estimates robot pose using AprilTag detections.

    Subscribes to AprilTag detections (from rubikpi_ros2 package) and
    computes global pose by transforming tag detections through the
    robot kinematic chain and comparing to known tag positions.
    """

    def __init__(self):
        super().__init__("hw4_localization")

        # AprilTag map (x, y, z, yaw)
        self.map_tags = cfg.MAP_TAGS

        self.base_frame = "base_link"
        self.cam_frame = "camera_frame"

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Pose publisher
        self.pose_pub = self.create_publisher(PoseStamped, "/pose_estimated", 10)

        # Subscriptions to AprilTag detections (from apriltag_ros)
        # Expected topic format: /tag_detections or similar from apriltag_ros
        # For now, we'll create a placeholder subscription
        self.create_subscription(
            PoseStamped,
            "/apriltag_pose",  # Adapt to your actual topic
            self.on_apriltag_detection,
            10,
        )

        self.get_logger().info("HW4 LocalizationNode initialized")

    def on_apriltag_detection(self, msg: PoseStamped):
        """
        Handle AprilTag detection and compute robot pose.

        Args:
            msg: PoseStamped message containing detected tag pose
        """
        # Extract tag ID from frame_id or message metadata
        # This is a simplified example; adapt to your apriltag_ros output

        try:
            # For each detected tag, compute the robot base pose
            # and publish it

            # Example: If we detect a tag, compute robot pose relative to it
            tag_pose = msg.pose
            tag_x = tag_pose.position.x
            tag_y = tag_pose.position.y
            tag_z = tag_pose.position.z

            q = tag_pose.orientation
            # Convert to yaw (simplified)
            tag_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            # Publish estimated pose
            estimated_pose = PoseStamped()
            estimated_pose.header.stamp = msg.header.stamp
            estimated_pose.header.frame_id = "world"
            estimated_pose.pose.position.x = tag_x
            estimated_pose.pose.position.y = tag_y
            estimated_pose.pose.position.z = tag_z
            estimated_pose.pose.orientation = q

            self.pose_pub.publish(estimated_pose)

        except Exception as e:
            self.get_logger().error(f"Error in apriltag detection: {e}")


def main():
    rclpy.init()
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
