import math, time, csv, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

# waypoints
WAYPOINTS = [(0.0,0.0,0.0), (1.0,0.0,0.0), (1.0,2.0,math.pi), (0.0,0.0,0.0)]

POS_TOL        = 0.12 # m
YAW_TOL        = 10*math.pi/180 # rad
POSE_STALE_S   = 0.50 # if pose older than this than its stale
TIMER_PERIOD_S = 0.05 # control loop at 20hz

MAX_V   = 0.2 # m/s
MAX_W   = 0.80 # rad/s
Kv      = 0.6 # P gain for distance
Kw      = 1.2 # P gain for heading
BASELINE= 0.127 # m wheel track 
K_WHEEL = 1.0 # scaling factor (tried multiple values but found 1 to be the best)

# for when pose can't be estimated (apriltag cant be seen)
COAST_TIME_S      = 0.20 # grace stop before scanning
SEARCH_OMEGA      = 0.60 # rad/s scan speed
REACQUIRE_HOLD_S  = 0.15 # require fresh poses for this long (sec)
MAX_SEARCH_TIME_S = 8.0 # sec

def angdiff(a, b):
    d = (a - b + math.pi) % (2*math.pi) - math.pi
    return d

def passed_segment(px, py, gx, gy, x, y):
    # progress along the segment from P(prev) to G(goal)
    ux, uy = (gx - px), (gy - py)
    seg2 = ux*ux + uy*uy
    if seg2 < 1e-6:
        return False
    sx, sy = (x - px), (y - py)
    s = (sx*ux + sy*uy) / seg2  # 0 at P, 1 at G
    return s > 1.05  # a tiny cushion beyond the goal


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.wpt_i = 0
        self.pose = None
        self.prev_wpt = WAYPOINTS[0]
        self.pose_time = 0.0

        # state machine
        self.state = "TRACKING"
        self.state_enter_t = time.time()
        self.had_pose_recently = False
        self.hold_started = None
        self.search_dir = 1.0 

        # publish adn subscribe
        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.sub = self.create_subscription(PoseStamped, '/pose_estimated', self.on_pose, 10)
        self.timer = self.create_timer(TIMER_PERIOD_S, self.tick)

        # logging for report
        self.log_path = os.path.expanduser('~/hw2_log.csv')
        self.logf = open(self.log_path, 'w', newline='')
        self.w = csv.writer(self.logf)
        self.w.writerow(['t','state','x','y','yaw','gx','gy','gyaw','rho','eyaw','L','R'])

    # some utility functions for easy use
    def on_pose(self, msg: PoseStamped):
        self.pose = msg
        self.pose_time = time.time()

    def send_motor(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.pub.publish(msg)

    def stop(self):
        self.send_motor(0.0, 0.0)

    def state_now(self):
        return time.time() - self.state_enter_t

    def enter(self, new_state):
        self.state = new_state
        self.state_enter_t = time.time()
        # reset state helpers
        if new_state == "SEARCH":
            self.had_pose_recently = False
            self.hold_started = None
            self.search_dir = 1.0

    def compute_tracking_cmd(self, x, y, yaw, gx, gy, gyaw):
        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)
        goal_bearing = math.atan2(dy, dx)
        e_yaw_to_goal = angdiff(goal_bearing, yaw)

        v = Kv * rho
        w = Kw * e_yaw_to_goal
        v = max(-MAX_V, min(MAX_V, v))
        w = max(-MAX_W, min(MAX_W, w))

        # wheel velocities
        vL = v - 0.5*w*BASELINE
        vR = v + 0.5*w*BASELINE
        L = K_WHEEL * vL
        R = K_WHEEL * vR

        # align final heading at the goal
        if rho < POS_TOL:
            eh = angdiff(gyaw, yaw)
            w_close = max(-MAX_W, min(MAX_W, Kw * eh))
            vL = -0.5 * BASELINE * w_close
            vR = +0.5 * BASELINE * w_close
            L, R = K_WHEEL*vL, K_WHEEL*vR
            done_here = abs(eh) < YAW_TOL
            return L, R, rho, e_yaw_to_goal, done_here

        return L, R, rho, e_yaw_to_goal, False

    def log_row(self, state, x, y, yaw, gx, gy, gyaw, rho, eyaw, L, R):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.w.writerow([t, state, x, y, yaw, gx, gy, gyaw, rho, eyaw, L, R])

    # main loop
    def tick(self):
        now = time.time()
        pose_fresh = (self.pose is not None) and ((now - self.pose_time) < POSE_STALE_S)

        # default log placeholders
        x=y=yaw=gx=gy=gyaw=rho=eyaw=L=R=0.0

        # tracking normal waypoint control when poses are "fresh"
        if self.state == "TRACKING":
            if not pose_fresh:
                self.stop()
                self.enter("COAST")
            else:
                # read pose
                x = self.pose.pose.position.x
                y = self.pose.pose.position.y
                q = self.pose.pose.orientation
                (_,_,yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

                gx, gy, gyaw = WAYPOINTS[self.wpt_i]
                
                L, R, rho, eyaw, done_here = self.compute_tracking_cmd(x, y, yaw, gx, gy, gyaw)

                if done_here:
                    self.stop()
                    self.get_logger().info(f"Reached waypoint {self.wpt_i}: ({gx:.2f},{gy:.2f},{gyaw:.2f})")
                    self.wpt_i = min(self.wpt_i + 1, len(WAYPOINTS) - 1)
                    time.sleep(0.4)
                else:
                    self.send_motor(L, R)

        # coast is brief stop window before scanning
        elif self.state == "COAST":
            self.stop()
            if pose_fresh:
                self.enter("TRACKING")
            elif self.state_now() >= COAST_TIME_S:
                # start in place scan
                self.send_motor(-0.5*BASELINE*SEARCH_OMEGA, +0.5*BASELINE*SEARCH_OMEGA)
                self.enter("SEARCH")

        # search rotates in place until poses are fresh for REACQUIRE_HOLD_S
        elif self.state == "SEARCH":
            elapsed = self.state_now()

            if pose_fresh:
                if not self.had_pose_recently:
                    self.had_pose_recently = True
                    self.hold_started = now
                elif (now - self.hold_started) >= REACQUIRE_HOLD_S:
                    self.had_pose_recently = False
                    self.enter("TRACKING")
            else:
                self.had_pose_recently = False

            # flip direction halfway to avoid possible bias
            if elapsed > MAX_SEARCH_TIME_S / 2 and self.search_dir > 0:
                self.search_dir = -1.0

            # keep rotating
            omega = self.search_dir * SEARCH_OMEGA
            L = -0.5 * BASELINE * omega
            R = +0.5 * BASELINE * omega
            self.send_motor(L, R)

            if elapsed > MAX_SEARCH_TIME_S:
                self.stop()
                self.enter("FAILSAFE")

        # failsafe to stay stopped and resume if pose becomes fresh (shouldnt happen because we scan with rotation)
        elif self.state == "FAILSAFE":
            self.stop()
            if pose_fresh:
                self.enter("TRACKING")

        # logging
        if self.pose is not None:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            q = self.pose.pose.orientation
            (_,_,yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        gx, gy, gyaw = WAYPOINTS[self.wpt_i]
        self.log_row(self.state, x, y, yaw, gx, gy, gyaw, rho, eyaw, L, R)

    # shutdown
    def destroy_node(self):
        try:
            self.stop()
            self.logf.close()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
