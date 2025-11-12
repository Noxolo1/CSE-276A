#!/usr/bin/env python3
import math
import numpy as np
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray

from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix


class Hw3SlamNode(Node):
   
    # EKF SLAM node for HW3.
    #resuses hw2's static TF from hw_2_solution.camera_tf (base_link -> camera_frame)

    def __init__(self):
        super().__init__('hw3_slam_node')

        self.declare_parameter('dt', 0.05)
        self.dt = float(self.get_parameter('dt').value)

        # log timing
        self.ekf_log_period = 5  # seconds
        self.last_ekf_log_time = self.get_clock().now()

        self.wheel_base = 0.127 # m

        self.left_linear_deadzone = 0.09
        self.left_linear_slope = 2.5
        self.right_linear_deadzone = 0.09
        self.right_linear_slope = 2.5

        self.left_angular_deadzone = 0.31
        self.left_angular_slope = 14.0
        self.right_angular_deadzone = 0.31
        self.right_angular_slope = 14.0

        # Q initial params
        # sigma_v = q_v0 + q_v1 * |v|
        # sigma_w = q_w0 + q_w1 * |w|
        self.declare_parameter('q_v0', 0.01) 
        self.declare_parameter('q_v1', 0.05) 
        self.declare_parameter('q_w0', 0.008) 
        self.declare_parameter('q_w1', 0.04) 
        self.q_v0 = float(self.get_parameter('q_v0').value)
        self.q_v1 = float(self.get_parameter('q_v1').value)
        self.q_w0 = float(self.get_parameter('q_w0').value)
        self.q_w1 = float(self.get_parameter('q_w1').value)

        # R initial params
        # sigma_r = r0 + r1 * r
        # sigma_b = b0 + b1 * r
        self.declare_parameter('r0', 0.03) 
        self.declare_parameter('r1', 0.05)
        self.declare_parameter('b0', 0.02) 
        self.declare_parameter('b1', 0.01)
        self.r0 = float(self.get_parameter('r0').value)
        self.r1 = float(self.get_parameter('r1').value)
        self.b0 = float(self.get_parameter('b0').value)


        # data logging / debugging 
        self.get_logger().info(
            f"Q params: q_v0={self.q_v0}, q_v1={self.q_v1}, q_w0={self.q_w0}, q_w1={self.q_w1}"
        )
        self.get_logger().info(
            f"R params: r0={self.r0}, r1={self.r1}, b0={self.b0}, b1={self.b1}"
        )

        self.declare_parameter('log_data', True)
        self.declare_parameter('log_dir', '/tmp/hw3_slam_logs')
        self.log_data = self.get_parameter('log_data').value
        self.log_dir = self.get_parameter('log_dir').value

        if self.log_data:
            os.makedirs(self.log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.log_file = os.path.join(self.log_dir, f'slam_log_{timestamp}.json')
            self.trajectory_log = []
            self.landmark_log = {}

        # camera_tf node publishes base_link -> camera_frame 
        # apriltag_ros detections are in camera frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # EKF state vector
        # x = [x_r, y_r, theta_r, x_L1, y_L1, x_L2, y_L2, ...]^T
        # large initial covariance for unknown start pose
        self.x = np.zeros((3, 1))
        self.P = np.diag([10.0, 10.0, (2 * math.pi) ** 2]) 

        # tag_id -> index for state vector
        self.landmark_index = {}

        # latest control [v, w]
        self.u = np.zeros((2, 1))

        # publish and subscribe 
        self.pose_pub = self.create_publisher(PoseStamped, '/slam_pose', 10)
        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/motor_commands',
            self.motor_cmd_callback,
            10
        )

        self.det_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10
        )

        # prediction loop
        self.timer = self.create_timer(self.dt, self.predict_step)

        self.get_logger().info('=' * 60)
        self.get_logger().info('HW3 SLAM Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Initial position uncertainty: ±{math.sqrt(self.P[0,0]):.2f}m')
        self.get_logger().info(f'Initial orientation uncertainty: ±{math.sqrt(self.P[2,2]):.2f}rad')
        if self.log_data:
            self.get_logger().info(f'Logging to: {self.log_file}')
        self.get_logger().info('=' * 60)

    # /motor_commands -> [v, w]  (inverse mapping)
    def motor_cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return

        L = float(msg.data[0])
        R = float(msg.data[1])

        # idle
        if abs(L) < 1e-4 and abs(R) < 1e-4:
            self.u[:, 0] = 0.0
            return
        
        same_sign = (L >= 0 and R >= 0) or (L <= 0 and R <= 0)

        if same_sign:
            vL = self._invert_deadzone(L, self.left_linear_deadzone, self.left_linear_slope)
            vR = self._invert_deadzone(R, self.right_linear_deadzone, self.right_linear_slope)
        else:
            vL = self._invert_deadzone(L, self.left_angular_deadzone, self.left_angular_slope)
            vR = self._invert_deadzone(R, self.right_angular_deadzone, self.right_angular_slope)

        # wheel -> body
        lin = 0.5 * (vL + vR)
        ang = (vR - vL) / max(self.wheel_base, 1e-9)

        self.u[0, 0] = lin
        self.u[1, 0] = ang

    def _invert_deadzone(self, cmd: float, deadzone: float, slope: float) -> float:
        
        # invert HW2 map_with_deadzone from velocity mapping
        # v = sign(cmd) * max(0, (|cmd| - deadzone) * slope)
        
        if abs(cmd) <= deadzone:
            return 0.0
        return math.copysign((abs(cmd) - deadzone) * slope, cmd)

    # EKF predict
    def predict_step(self):
        v = float(self.u[0, 0])
        w = float(self.u[1, 0])
        dt = self.dt

        x_r = float(self.x[0, 0])
        y_r = float(self.x[1, 0])
        th = float(self.x[2, 0])

        # unicycle model
        x_r_new = x_r + v * dt * math.cos(th)
        y_r_new = y_r + v * dt * math.sin(th)
        th_new = th + w * dt

        self.x[0, 0] = x_r_new
        self.x[1, 0] = y_r_new
        self.x[2, 0] = self.normalize_angle(th_new)

        n = self.x.shape[0]

        # F = df/dx
        F = np.eye(n)
        F[0, 2] = -v * dt * math.sin(th)
        F[1, 2] =  v * dt * math.cos(th)

        # G = df/d[n_v, n_w]  (noise on v, w)
        G = np.zeros((n, 2))
        G[0, 0] = dt * math.cos(th)
        G[1, 0] = dt * math.sin(th)
        G[2, 1] = dt

        # motion dependent Q in control space
        sigma_v = self.q_v0 + self.q_v1 * abs(v)
        sigma_w = self.q_w0 + self.q_w1 * abs(w)
        Q_u = np.diag([sigma_v ** 2, sigma_w ** 2])

        self.P = F @ self.P @ F.T + G @ Q_u @ G.T

        # publish updated pose
        self.publish_state()

    # EKF update: AprilTag detections -> range/bearing in base link
    def detections_callback(self, msg: AprilTagDetectionArray):
        for det in msg.detections:

            tag_id = det.id
            tag_frame = f'tag_{tag_id}'

            try:
                tag_tf = self.tf_buffer.lookup_transform(
                    'camera_frame',
                    tag_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except Exception:
                continue

            try:
                camera_to_base_tf = self.tf_buffer.lookup_transform(
                    'base_link',
                    'camera_frame',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except Exception:
                continue

            tag_pos_camera = tag_tf.transform.translation

            # transform to base link frame
            x_b, y_b = self._transform_point(tag_pos_camera, camera_to_base_tf)

            r = math.sqrt(x_b * x_b + y_b * y_b)
            if r < 1e-6:
                continue

            bearing = math.atan2(y_b, x_b)  # in base link frame

            z = np.array([[r],
                          [bearing]])

            if tag_id not in self.landmark_index:
                self.initialize_landmark(tag_id, z)
            else:
                self.ekf_update_landmark(tag_id, z)

    def _transform_point(self, p, tf):

        # uses tf_transformations.quaternion_matrix for rotation.
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        T = quaternion_matrix([qx, qy, qz, qw])
        T[0, 3] = tx
        T[1, 3] = ty
        T[2, 3] = tz

        v = np.array([p.x, p.y, p.z, 1.0])
        v_b = T @ v

        return float(v_b[0]), float(v_b[1])  # (x, y) in base link

    # landmark initialization (augments state vector) 
    def initialize_landmark(self, tag_id, z):
        r = float(z[0, 0])
        b = float(z[1, 0])

        x_r = float(self.x[0, 0])
        y_r = float(self.x[1, 0])
        th = float(self.x[2, 0])

        xL = x_r + r * math.cos(th + b)
        yL = y_r + r * math.sin(th + b)

        # augment state vector with [xL, yL]
        self.x = np.vstack([self.x, [[xL], [yL]]])

        n_old = self.P.shape[0]
        P_new = np.zeros((n_old + 2, n_old + 2))
        P_new[:n_old, :n_old] = self.P

        # initial landmark covariance based on measurement uncertainty
        sigma_r = self.r0 + self.r1 * r
        sigma_b = self.b0 + self.b1 * r

        # propagate measurement uncertainty to landmark position
        landmark_var = max((sigma_r ** 2) + (r * sigma_b) ** 2, 0.1)
        P_new[n_old, n_old] = landmark_var  # x uncertainty
        P_new[n_old + 1, n_old + 1] = landmark_var  # y uncertainty

        self.P = P_new
        self.landmark_index[tag_id] = n_old

        self.get_logger().info(
            f'initialized landmark {tag_id} at ({xL:.2f}, {yL:.2f}), '
            f'sigma={math.sqrt(landmark_var):.3f}m'
        )

    # EKF update for existing landmark
    def ekf_update_landmark(self, tag_id, z):
        idx = self.landmark_index[tag_id]

        x_r = float(self.x[0, 0])
        y_r = float(self.x[1, 0])
        th = float(self.x[2, 0])

        xL = float(self.x[idx, 0])
        yL = float(self.x[idx + 1, 0])

        dx = xL - x_r
        dy = yL - y_r
        q = dx * dx + dy * dy
        if q < 1e-9:
            return

        r_pred = math.sqrt(q)
        b_pred = math.atan2(dy, dx) - th
        z_pred = np.array([[r_pred],
                           [self.normalize_angle(b_pred)]])

        # innovation
        y = z - z_pred
        y[1, 0] = self.normalize_angle(y[1, 0])

        n = self.x.shape[0]
        H = np.zeros((2, n))

        # robot pose part
        H[0, 0] = -dx / r_pred
        H[0, 1] = -dy / r_pred
        H[0, 2] = 0.0

        H[1, 0] =  dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0

        # landmark part
        H[0, idx] = dx / r_pred
        H[0, idx + 1] = dy / r_pred
        H[1, idx] = -dy / q
        H[1, idx + 1] =  dx / q

        # distance dependent R
        r_meas = float(z[0, 0])
        sigma_r = self.r0 + self.r1 * r_meas
        sigma_b = self.b0 + self.b1 * r_meas
        R = np.diag([sigma_r ** 2, sigma_b ** 2])

        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn('S not invertible, skipping update')
            return

        # covariance check for debugging
        #self.get_logger().info(
        #    f"[BEFORE] tag {tag_id}: cov_xx={self.P[idx, idx]:.4f}, cov_yy={self.P[idx+1, idx+1]:.4f}"
        #)

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.x[2, 0] = self.normalize_angle(float(self.x[2, 0]))

        I = np.eye(n)
        self.P = (I - K @ H) @ self.P

        #self.get_logger().info(
        #    f"[AFTER]  tag {tag_id}: cov_xx={self.P[idx, idx]:.4f}, cov_yy={self.P[idx+1, idx+1]:.4f}"
        #)

        # publish updated pose
        self.publish_state()

    # pose publishing and logging
    def publish_state(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'

        pose_msg.pose.position.x = float(self.x[0, 0])
        pose_msg.pose.position.y = float(self.x[1, 0])
        pose_msg.pose.position.z = 0.0

        # convert theta to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, float(self.x[2, 0]))
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        # EKF pose printing for debugging
        now = self.get_clock().now()
        dt = (now - self.last_ekf_log_time).nanoseconds / 1e9
        if dt >= self.ekf_log_period:
            self.last_ekf_log_time = now
            self.get_logger().info(
                f"EKF_POSE=({pose_msg.pose.position.x:.2f}, "
                f"{pose_msg.pose.position.y:.2f}, "
                f"{math.degrees(float(self.x[2, 0])):.1f}deg)"
            )

        self.pose_pub.publish(pose_msg)

        # log trajectory
        if self.log_data:
            self.log_trajectory_point()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def log_trajectory_point(self):
        # log current pose and covariance
        self.trajectory_log.append({
            'time': self.get_clock().now().nanoseconds / 1e9,
            'x': float(self.x[0, 0]),
            'y': float(self.x[1, 0]),
            'theta': float(self.x[2, 0]),
            'cov_xx': float(self.P[0, 0]),
            'cov_yy': float(self.P[1, 1]),
            'cov_tt': float(self.P[2, 2]),
            'cov_xy': float(self.P[0, 1])
        })

    def save_logs(self):
        # save trajectory and landmark data 
        if not self.log_data:
            return

        # update landmark log with final estimates
        for tag_id, idx in self.landmark_index.items():
            self.landmark_log[f'tag_{tag_id}'] = {
                'x': float(self.x[idx, 0]),
                'y': float(self.x[idx + 1, 0]),
                'cov_xx': float(self.P[idx, idx]),
                'cov_yy': float(self.P[idx + 1, idx + 1]),
                'cov_xy': float(self.P[idx, idx + 1])
            }

        data = {
            'trajectory': self.trajectory_log,
            'landmarks': self.landmark_log,
            'parameters': {
                'q_v0': self.q_v0,
                'q_v1': self.q_v1,
                'q_w0': self.q_w0,
                'q_w1': self.q_w1,
                'r0': self.r0,
                'r1': self.r1,
                'b0': self.b0,
                'b1': self.b1
            }
        }

        with open(self.log_file, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info('=' * 60)
        self.get_logger().info('SLAM data saved')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Log file: {self.log_file}')
        self.get_logger().info(f'Trajectory points: {len(self.trajectory_log)}')
        self.get_logger().info(f'Landmarks detected: {len(self.landmark_log)}')
        self.get_logger().info('=' * 60)


    @staticmethod
    def normalize_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def destroy_node(self):
        if self.log_data:
            self.save_logs()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Hw3SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
