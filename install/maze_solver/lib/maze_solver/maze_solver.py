#!/usr/bin/env python3
"""
maze_solver.py  —  BUG2 + SLAM  (fixed)
========================================

Fixes vs previous version:
  1. GO_TO_GOAL now drives in TWO clear sub-steps:
       a. ROTATE in place until aligned with goal (|err| < 0.1 rad)
       b. DRIVE straight — no curved steering that confuses wall detection
  2. Wall following is RIGHT-hand (wall on right) — more natural for
     typical maze layouts where entrance is top-left, exit bottom-right.
  3. M-line leave condition loosened: tolerance 0.25m, distance gain 0.3m.
  4. Added a minimum wall-follow distance (MIN_FOLLOW_DIST) so the robot
     doesn't try to leave the obstacle 1cm after hitting it.
  5. SlamMapper unchanged — it was working fine.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped

import math
import numpy as np
import threading


# ═══════════════════════════════════════════════════════════════════════
# TUNABLES  —  edit these to match YOUR maze
# ═══════════════════════════════════════════════════════════════════════

GOAL_X = 3.0          # world X of goal (match your SDF goal pose)
GOAL_Y = -3.0         # world Y of goal
GOAL_RADIUS = 0.4     # m — stop when this close

LINEAR_SPEED  = 0.20  # m/s
TURN_SPEED    = 0.50  # rad/s

# BUG2 thresholds
OBSTACLE_DIST    = 0.60   # m — obstacle too close ahead in GO_TO_GOAL
WALL_FOLLOW_DIST = 0.50   # m — desired distance from right wall
WALL_TOL         = 0.10   # m — tolerance band

MLINE_TOL        = 0.25   # m — distance to M-line counts as "on it"
LEAVE_GAIN       = 0.30   # m — must be this much closer to goal than hit-point
MIN_FOLLOW_DIST  = 1.20   # m — minimum travel in WALL_FOLLOWING before leaving

# Sensor arcs (degrees, robot frame: 0=front, 90=left, -90=right)
FRONT_ARC = 25
RIGHT_ARC = 30
FL_ARC    = 15   # front-right diagonal

# SLAM map
MAP_RES    = 0.10
MAP_W      = 20.0
MAP_H      = 20.0
MAP_OX     = -10.0
MAP_OY     = -10.0
HIT_INC    = 15
MISS_DEC   =  5
MAX_LOG    = 100
OCC_THRESH =  50
MAP_HZ     =  2.0

# BUG2 sub-states
ROTATING  = "ROTATING"   # aligning to goal before driving
DRIVING   = "DRIVING"    # driving straight toward goal
FOLLOWING = "FOLLOWING"  # wall following
DONE      = "DONE"


# ═══════════════════════════════════════════════════════════════════════
# SLAM MAPPER  (unchanged — was working correctly)
# ═══════════════════════════════════════════════════════════════════════

class SlamMapper(Node):
    def __init__(self):
        super().__init__('slam_mapper')
        self.cols = int(MAP_W / MAP_RES)
        self.rows = int(MAP_H / MAP_RES)
        self.log_odds = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.rx = 0.0; self.ry = 0.0; self.rth = 0.0
        self._lock = threading.Lock()

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        self.map_pub  = self.create_publisher(OccupancyGrid, '/map',       10)
        self.pose_pub = self.create_publisher(PoseStamped,   '/slam_pose', 10)
        self.create_timer(1.0 / MAP_HZ, self._pub_map)
        self.get_logger().info("SlamMapper ready")

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        with self._lock:
            self.rx = p.x; self.ry = p.y; self.rth = yaw
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose = msg.pose.pose
        self.pose_pub.publish(ps)

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            rx, ry, rth = self.rx, self.ry, self.rth
        ang = msg.angle_min
        for r in msg.ranges:
            ang += msg.angle_increment
            if not math.isfinite(r) or r < msg.range_min:
                continue
            hit  = r < msg.range_max - 0.05
            dist = min(r, msg.range_max)
            wx = rx + dist * math.cos(rth + ang)
            wy = ry + dist * math.sin(rth + ang)
            c0, r0 = self._w2c(rx, ry)
            c1, r1 = self._w2c(wx, wy)
            for cc, rc in self._bres(c0, r0, c1, r1):
                if self._ok(cc, rc):
                    self.log_odds[rc, cc] = max(
                        -MAX_LOG, self.log_odds[rc, cc] - MISS_DEC)
            if hit and self._ok(c1, r1):
                self.log_odds[r1, c1] = min(
                    MAX_LOG, self.log_odds[r1, c1] + HIT_INC)

    def _pub_map(self):
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'map'
        og.info.resolution = MAP_RES
        og.info.width  = self.cols
        og.info.height = self.rows
        og.info.origin.position.x = MAP_OX
        og.info.origin.position.y = MAP_OY
        flat = self.log_odds.flatten()
        data = []
        for v in flat:
            if v == 0:          data.append(-1)
            elif v >= OCC_THRESH: data.append(100)
            elif v <= -OCC_THRESH: data.append(0)
            else:               data.append(-1)
        og.data = data
        self.map_pub.publish(og)

    def _w2c(self, wx, wy):
        return int((wx - MAP_OX)/MAP_RES), int((wy - MAP_OY)/MAP_RES)

    def _ok(self, c, r):
        return 0 <= c < self.cols and 0 <= r < self.rows

    @staticmethod
    def _bres(c0, r0, c1, r1):
        cells = []
        dc=abs(c1-c0); dr=abs(r1-r0)
        sc=1 if c1>c0 else -1; sr=1 if r1>r0 else -1
        err=dc-dr; c,r=c0,r0
        while True:
            cells.append((c,r))
            if c==c1 and r==r1: break
            e2=2*err
            if e2>-dr: err-=dr; c+=sc
            if e2< dc: err+=dc; r+=sr
        return cells

    def get_pose(self):
        with self._lock:
            return self.rx, self.ry, self.rth


# ═══════════════════════════════════════════════════════════════════════
# BUG2 NAVIGATOR
# ═══════════════════════════════════════════════════════════════════════

class Bug2Navigator(Node):

    def __init__(self, mapper: SlamMapper):
        super().__init__('bug2_navigator')
        self.mapper = mapper

        # BUG2 state
        self.state = ROTATING     # start by aligning to goal

        # M-line (start → goal straight line)
        self.start_x: float | None = None
        self.start_y: float | None = None

        # Hit-point info
        self.hit_x    = 0.0
        self.hit_y    = 0.0
        self.hit_dist = float('inf')   # dist-to-goal at hit point
        self.follow_travel = 0.0       # metres travelled while wall-following
        self._last_fx = 0.0
        self._last_fy = 0.0

        # Scan data
        self.ranges    = []
        self.num_rays  = 0
        self.angle_min = 0.0
        self.angle_inc = 0.0

        # Debounce
        self._front_ct = 0
        FRONT_DB = 2
        self._FRONT_DB = FRONT_DB

        # Path for RViz
        self.path_poses = []

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel',   10)
        self.path_pub = self.create_publisher(Path,  '/bug2_path', 10)

        self.create_timer(0.1, self._loop)
        self.get_logger().info(
            f"Bug2Navigator ready  goal=({GOAL_X},{GOAL_Y})")

    # ── callbacks ─────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self.ranges    = list(msg.ranges)
        self.num_rays  = len(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def _odom_cb(self, msg: Odometry):
        if self.start_x is None:
            p = msg.pose.pose.position
            self.start_x = p.x
            self.start_y = p.y
            self._last_fx = p.x
            self._last_fy = p.y
            self.get_logger().info(
                f"M-line start: ({self.start_x:.2f},{self.start_y:.2f})")

    # ── sensor helpers ────────────────────────────────────────────────
    def _sec(self, deg: float, arc: float) -> float:
        if not self.ranges: return float('inf')
        cr = math.radians(deg)
        hr = math.radians(arc)
        ci = int((cr - self.angle_min) / self.angle_inc)
        hi = int(hr / self.angle_inc)
        lo = max(0, ci-hi); hi2 = min(self.num_rays-1, ci+hi)
        vals = [r for r in self.ranges[lo:hi2+1]
                if math.isfinite(r) and r > 0.01]
        return min(vals) if vals else float('inf')

    def F(self)  -> float: return self._sec(  0.0, FRONT_ARC)
    def R(self)  -> float: return self._sec(-90.0, RIGHT_ARC)
    def FR(self) -> float: return self._sec(-45.0, FL_ARC)

    # ── drive helpers ─────────────────────────────────────────────────
    def _cmd(self, v: float, w: float):
        m = Twist(); m.linear.x = v; m.angular.z = w
        self.cmd_pub.publish(m)

    def _stop(self): self._cmd(0.0, 0.0)

    def _go(self, s: str):
        if s != self.state:
            rx,ry,_ = self.mapper.get_pose()
            self.get_logger().info(
                f"BUG2 {self.state}→{s}  "
                f"pos=({rx:.2f},{ry:.2f})  "
                f"F={self.F():.2f} R={self.R():.2f}")
            self.state = s

    # ── geometry ──────────────────────────────────────────────────────
    def _d_goal(self, x, y):
        return math.hypot(GOAL_X-x, GOAL_Y-y)

    def _ang_goal(self, x, y, th):
        desired = math.atan2(GOAL_Y-y, GOAL_X-x)
        e = desired - th
        while e >  math.pi: e -= 2*math.pi
        while e < -math.pi: e += 2*math.pi
        return e

    def _d_mline(self, x, y):
        if self.start_x is None: return float('inf')
        sx,sy = self.start_x, self.start_y
        gx,gy = GOAL_X, GOAL_Y
        num = abs((gy-sy)*x - (gx-sx)*y + (gx-sx)*sy - (gy-sy)*sx)
        den = math.hypot(gx-sx, gy-sy)
        return num/den if den > 1e-6 else float('inf')

    # ── path pub ──────────────────────────────────────────────────────
    def _pub_path(self, x, y):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x; ps.pose.position.y = y
        self.path_poses.append(ps)
        if len(self.path_poses) > 3000:
            self.path_poses = self.path_poses[-3000:]
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.poses = self.path_poses
        self.path_pub.publish(p)

    # ══════════════════════════════════════════════════════════════════
    # CONTROL LOOP
    # ══════════════════════════════════════════════════════════════════
    def _loop(self):
        if not self.ranges or self.start_x is None:
            return

        rx, ry, rth = self.mapper.get_pose()
        self._pub_path(rx, ry)

        dg = self._d_goal(rx, ry)

        # ── DONE ──────────────────────────────────────────────────────
        if self.state == DONE:
            self._stop(); return

        if dg < GOAL_RADIUS:
            self._stop()
            self._go(DONE)
            self.get_logger().info(f"★ GOAL REACHED at ({rx:.2f},{ry:.2f}) ★")
            return

        f  = self.F()
        r  = self.R()
        fr = self.FR()

        # ══════════════════════════════════════════════════════════════
        # ROTATING — spin in place until facing goal (|err| < 0.08 rad)
        # ══════════════════════════════════════════════════════════════
        if self.state == ROTATING:
            ae = self._ang_goal(rx, ry, rth)
            if abs(ae) < 0.08:
                self._stop()
                self._go(DRIVING)
            else:
                # rotate toward goal
                w = TURN_SPEED if ae > 0 else -TURN_SPEED
                self._cmd(0.0, w)

        # ══════════════════════════════════════════════════════════════
        # DRIVING — go straight toward goal, watch for obstacles
        # ══════════════════════════════════════════════════════════════
        elif self.state == DRIVING:

            # Obstacle detection (debounced)
            if f < OBSTACLE_DIST:
                self._front_ct += 1
            else:
                self._front_ct = 0

            if self._front_ct >= self._FRONT_DB:
                # Hit! Record hit-point, enter wall following
                self._front_ct = 0
                self.hit_x = rx; self.hit_y = ry
                self.hit_dist = dg
                self.follow_travel = 0.0
                self._last_fx = rx; self._last_fy = ry
                self._stop()
                self._go(FOLLOWING)
                return

            # Keep correcting heading while driving (small correction only)
            ae = self._ang_goal(rx, ry, rth)
            if abs(ae) > 0.25:
                # Heading drifted too much — stop and re-rotate
                self._stop()
                self._go(ROTATING)
            else:
                # Drive straight with tiny heading correction
                self._cmd(LINEAR_SPEED, ae * 0.5)

        # ══════════════════════════════════════════════════════════════
        # FOLLOWING — right-hand wall follower
        # ══════════════════════════════════════════════════════════════
        elif self.state == FOLLOWING:

            # Accumulate travel distance
            dx = rx - self._last_fx; dy = ry - self._last_fy
            self.follow_travel += math.hypot(dx, dy)
            self._last_fx = rx; self._last_fy = ry

            # ── BUG2 leave condition ───────────────────────────────────
            on_mline   = self._d_mline(rx, ry) < MLINE_TOL
            closer     = dg < self.hit_dist - LEAVE_GAIN
            travelled  = self.follow_travel > MIN_FOLLOW_DIST
            not_at_hit = math.hypot(rx-self.hit_x, ry-self.hit_y) > 0.5

            if on_mline and closer and travelled and not_at_hit:
                self._stop()
                # Re-align to goal before driving
                self._go(ROTATING)
                return

            # ── Right-hand wall following ──────────────────────────────
            if f < OBSTACLE_DIST:
                # Wall ahead → turn LEFT (away from wall on right)
                self._cmd(0.0, TURN_SPEED)

            elif r > WALL_FOLLOW_DIST + 0.30 and fr > WALL_FOLLOW_DIST + 0.30:
                # Right wall disappeared → turn RIGHT into opening
                self._cmd(LINEAR_SPEED * 0.3, -TURN_SPEED)

            elif r < WALL_FOLLOW_DIST - WALL_TOL:
                # Too close → nudge left
                self._cmd(LINEAR_SPEED, TURN_SPEED * 0.30)

            elif r > WALL_FOLLOW_DIST + WALL_TOL:
                # Drifting away → nudge right
                self._cmd(LINEAR_SPEED, -TURN_SPEED * 0.22)

            else:
                self._cmd(LINEAR_SPEED, 0.0)

    def destroy_node(self):
        self._stop()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    mapper    = SlamMapper()
    navigator = Bug2Navigator(mapper)
    executor  = MultiThreadedExecutor(num_threads=4)
    executor.add_node(mapper)
    executor.add_node(navigator)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()