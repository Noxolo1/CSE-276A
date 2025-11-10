import math
import numpy as np
import rclpy 
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_matrix, quaternion_matrix

# measurements of april tags from world origin, starting point (0,0,0) in (x, y, z, yaw in rads)
# z is height of the middle of the black square on printed april tags
# yaw is tag rotation with respect to world origin
MAP_TAGS = {
    'tag_0': (0.73330, -0.67310, 0.1645, math.radians(90.0)),  
    'tag_1': (1.29210,  0.00000, 0.1645, math.radians(180.0)), 
    'tag_2': (1.72390,  1.69520, 0.1695, math.radians(180.0)), 
    'tag_3': (1.00000,  2.68580, 0.1645, math.radians(270.0)),
    'tag_4': (-0.19380, 1.61900, 0.1645, math.radians(0.0)), 
}

BASE_FRAME  = 'base_link'
CAM_FRAME   = 'camera_frame'

# helper functions to compute transforms
def T_from_xyz_yaw(x, y, z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[0,0], T[0,1] = c, -s
    T[1,0], T[1,1] = s,  c
    return T

def T_from_tf(tf: TransformStamped):
    T = np.eye(4)
    t = tf.transform.translation
    q = tf.transform.rotation
    T[:3, 3] = [t.x, t.y, t.z]
    T[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    return T

def T_inv(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

def yaw_from_R(R):
    return math.atan2(R[1,0], R[0,0])

def R_from_yaw(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.eye(3)
    R[0,0], R[0,1] = c, -s
    R[1,0], R[1,1] = s,  c
    return R

# lcoalization node
class Localizer(Node):
    def __init__(self):
        super().__init__('localizer_numeric')

        # params
        self.declare_parameter('tick_period', 0.3) # seconds between estimates
        self.declare_parameter('range_scale', 1.0) # range scale
        self.declare_parameter('fixed_z', 0.0) # published base_link z in map

        tick_period = float(self.get_parameter('tick_period').value)
        self.range_scale = float(self.get_parameter('range_scale').value)
        self.fixed_z = float(self.get_parameter('fixed_z').value)

        self.pub = self.create_publisher(PoseStamped, '/pose_estimated', 10)
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        # precompute map to tag_j transforms
        self.T_map_tag = {k: T_from_xyz_yaw(*v) for k, v in MAP_TAGS.items()}

        # lookup base to cam once 
        self.T_base_cam = None

        # timer for periodic estimation
        self.timer = self.create_timer(tick_period, self.tick)

    def get_T_base_cam(self):
        if self.T_base_cam is not None:
            return self.T_base_cam
        try:
            tf = self.buffer.lookup_transform(BASE_FRAME, CAM_FRAME, rclpy.time.Time())
            self.T_base_cam = T_from_tf(tf)
            self.get_logger().info('got static T_base_cam')
        except Exception:
            pass
        return self.T_base_cam

    def tick(self):
        # need static T_base_cam
        Tbc = self.get_T_base_cam()
        if Tbc is None:
            return

        T_map_base_list = []

        # try visible tags (iterating by key order)
        for tag in MAP_TAGS.keys():
            try:
                # camera_frame to tag_j (from apriltag_ros)
                tf_cam_tag = self.buffer.lookup_transform(CAM_FRAME, tag, rclpy.time.Time())
                T_cam_tag = T_from_tf(tf_cam_tag)

                # scalar range correction
                if abs(self.range_scale - 1.0) > 1e-6:
                    t = T_cam_tag[:3,3]
                    T_cam_tag[:3,3] = t * self.range_scale

                # map to tag_j (constant from measurements)
                T_map_tag = self.T_map_tag[tag]

                # compose 
                T_map_base = T_map_tag @ T_inv(T_cam_tag) @ T_inv(Tbc)
                T_map_base_list.append(T_map_base)

            except Exception:
                continue

        if not T_map_base_list:
            return

        # averaging translation
        avg_t = np.mean([T[:3, 3] for T in T_map_base_list], axis=0)

        # average orientation 
        quats = [quaternion_from_matrix(T) for T in T_map_base_list]
        qsum = np.sum(quats, axis=0)
        avg_q = qsum / np.linalg.norm(qsum)

        # build averaged transform
        T_map_base_avg = np.eye(4)
        T_map_base_avg[:3, 3] = avg_t
        T_map_base_avg[:3, :3] = quaternion_matrix(avg_q)[:3, :3]

        # planar projection: keep x,y,yaw , fix z (on ground), zero roll/pitch 
        R = T_map_base_avg[:3, :3]
        yaw = yaw_from_R(R)
        T_map_base_avg[:3, :3] = R_from_yaw(yaw)
        T_map_base_avg[2, 3] = self.fixed_z  # publish on ground plane 
        avg_q_planar = quaternion_from_matrix(T_map_base_avg)

        # publish PoseStamped in map
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = T_map_base_avg[0, 3]
        ps.pose.position.y = T_map_base_avg[1, 3]
        ps.pose.position.z = T_map_base_avg[2, 3]
        ps.pose.orientation.x = avg_q_planar[0]
        ps.pose.orientation.y = avg_q_planar[1]
        ps.pose.orientation.z = avg_q_planar[2]
        ps.pose.orientation.w = avg_q_planar[3]
        self.pub.publish(ps)

def main():
    rclpy.init()
    node = Localizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
