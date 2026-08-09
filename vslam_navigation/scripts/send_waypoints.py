#!/usr/bin/env python3
"""Drive the robot through a sequence of points, purely from code.

The RViz "Waypoint / Nav Through Poses Mode" panel button is broken here -
see README.md "Use '2D Goal Pose', not 'Nav2 Goal'" - it shares the same
wall-clock-stamped internal node as the single-goal button, with no
/goal_pose-style topic available to relay from. This is the code-driven
replacement: send each point as its own NavigateToPose goal, wait for it to
finish, then send the next. No clicking required.

  ros2 run vslam_navigation send_waypoints.py
  ros2 run vslam_navigation send_waypoints.py --waypoints "0.9,0.4 0.2,-0.8 -0.5,0.6"
  ros2 run vslam_navigation send_waypoints.py --preset textures
  ros2 run vslam_navigation send_waypoints.py --preset loop --interior 6 --loop

FRAMES - read this before adding waypoints.

Every waypoint here (presets, --waypoints, --interior) is a real Gazebo-world
coordinate - the same numbers you'd read out of
vslam_simulator/worlds/textured_tb3_world.sdf for a pillar or wall. NavigateToPose
goals, however, are sent in the "map" frame: ORB-SLAM3's own frame, whose origin
is wherever the camera happened to be on the first frame it successfully tracked
(near the spawn pose, but offset further and arbitrarily rotated). map is NOT
the same frame as the Gazebo world the SDF describes, and the offset between
them is not fixed - it depends on exactly when and how tracking initialised, and
shifts again on every tracking-loss re-anchor.

world_to_map() converts world -> map at send time, using the live map->odom
correction (already being broadcast by mono_slam_node) composed with the fixed,
known spawn offset (SPAWN_X/Y/YAW below, matching sim.launch.py's defaults - pass
--spawn-x/--spawn-y/--spawn-yaw if you launched with a different pose). This is
recomputed before every single goal, not once, so it keeps tracking correctly as
SLAM drifts or re-anchors mid-tour.
"""

import argparse
import math
import random
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Must match sim.launch.py's x_pose/y_pose/yaw defaults - the fixed,
# known offset of the odom frame's origin from the real Gazebo world origin.
SPAWN_X, SPAWN_Y, SPAWN_YAW = -2.00, -0.50, 0.00

# Stuck recovery. STUCK_TIMEOUT is set just above nav2_params.yaml's
# progress_checker.movement_time_allowance (40s) - if Nav2's own progress
# check would have aborted by now anyway, no point waiting longer for it.
# Below that, this also catches the case that motivated it: a goal that
# never resolves at all (accepted, then silence forever) - previously
# send_one() waited on the result with no timeout, so a stuck goal hung the
# whole script with no way out.
STUCK_TIMEOUT_SEC = 45.0
REVERSE_SPEED = -0.08    # gentle - same reasoning as the capped Nav2 speeds
REVERSE_DURATION_SEC = 1.5
MAX_RETRIES = 2


def parse_waypoints(text):
    """'x1,y1 x2,y2 ...' -> [(x1, y1), (x2, y2), ...]. Pure, no ROS - see demo()."""
    points = []
    for token in text.split():
        x_str, y_str = token.split(',')
        points.append((float(x_str), float(y_str)))
    return points


# Default tour: spread across the 5x5m arena, threading between pillars
# (see vslam_simulator/worlds/textured_tb3_world.sdf for the 1.1m pillar grid),
# well clear of the walls.
DEFAULT_WAYPOINTS = '0.9,0.4 0.2,-0.9 -0.9,-0.4 -0.2,0.9 0.9,-0.9'

# "textures" tour: stops in front of all 9 uniquely-textured pillars (see
# vslam_simulator/scripts/make_textures.py - every pillar gets a DIFFERENT
# pattern) plus a look at each of the 4 differently-textured walls.
#
# Each pillar stop is computed, not guessed: pillars sit on the world's
# +/-1.1m grid with radius 0.15m; the approach point is 0.6m from the pillar
# centre (clears the 0.18m inflation_radius + 0.11m robot_radius with margin,
# close enough to frame the texture) along the line toward the arena centre.
# The centre pillar (0,0) has no such direction, so it gets a fixed stop.
#
# Visit order is a serpentine sweep of the 3x3 pillar grid (right, up, left,
# up, right) to minimise backtracking, then the 4 wall midpoints.
TEXTURE_TOUR_WAYPOINTS = (
    '-0.676,-0.676  0.0,-0.5  0.676,-0.676  '   # bottom row: pillars 1, 4, 7
    '0.5,0.0  '                                  # right-middle: pillar 8
    '0.0,0.65  '                                 # centre: pillar 5
    '-0.5,0.0  '                                 # left-middle: pillar 2
    '-0.676,0.676  0.0,0.5  0.676,0.676  '       # top row: pillars 3, 6, 9
    '2.0,0.0  -2.0,0.0  0.0,2.0  0.0,-2.0'       # the 4 walls, 0.5m off each
)

# "loop" tour: a big square sweep near the walls, staying clear of the entire
# pillar grid (which only extends to +/-1.1) rather than threading through it.
# For "drive all around the map" - use with --loop for continuous travel.
# Corners are 1.8m out: 0.65m clear of the walls, 0.99m clear of the nearest
# pillar - wide margins on both sides.
LOOP_WAYPOINTS = '1.8,1.8 -1.8,1.8 -1.8,-1.8 1.8,-1.8'

PRESETS = {
    'default': DEFAULT_WAYPOINTS,
    'textures': TEXTURE_TOUR_WAYPOINTS,
    'loop': LOOP_WAYPOINTS,
}

PILLARS = [(x, y) for x in (-1.1, 0, 1.1) for y in (-1.1, 0, 1.1)]
# pillar radius (0.15) + inflation_radius (0.18), from nav2_params.yaml, plus
# a small margin so sampled points don't sit right on the boundary
MIN_PILLAR_CLEARANCE = 0.15 + 0.18 + 0.05
ARENA_HALF_EXTENT = 1.9  # comfortably inside the walls (inner face at 2.45)


def world_to_map_xy(x_w, y_w, spawn_x, spawn_y, spawn_yaw, map_odom_x, map_odom_y, map_odom_yaw):
    """A real Gazebo-world point -> the equivalent point in ORB-SLAM3's "map"
    frame, via odom. Pure function - no ROS, no live TF - see demo().

        world -> odom:  translate by -spawn, rotate by -spawn_yaw
        odom  -> map:   rotate by map_odom_yaw, translate by map_odom_(x,y)

    (the live map->odom transform, looked up fresh for every goal - see
    WaypointTour.world_to_map)."""
    dx, dy = x_w - spawn_x, y_w - spawn_y
    c, s = math.cos(-spawn_yaw), math.sin(-spawn_yaw)
    x_o = dx * c - dy * s
    y_o = dx * s + dy * c

    c, s = math.cos(map_odom_yaw), math.sin(map_odom_yaw)
    x_m = map_odom_x + x_o * c - y_o * s
    y_m = map_odom_y + x_o * s + y_o * c
    return x_m, y_m


def should_reverse_and_retry(result, attempt, max_retries):
    """Decision only - no ROS, no timing, so it's directly testable (see demo()).
    REJECTED means the action server itself is unavailable; reversing a robot
    that isn't even talking to Nav2 accomplishes nothing, so that one fails fast."""
    return result in ('ABORTED', 'TIMEOUT') and attempt < max_retries


def random_interior_waypoints(n, rng):
    """n random points weaving between the pillars, each clearing all of them
    by MIN_PILLAR_CLEARANCE. Rejection sampling - the arena is mostly open
    space (9 small pillars in 25 sq m), so this converges in a handful of
    tries per point. Pure function, no ROS - see demo()."""
    pts = []
    while len(pts) < n:
        x = rng.uniform(-ARENA_HALF_EXTENT, ARENA_HALF_EXTENT)
        y = rng.uniform(-ARENA_HALF_EXTENT, ARENA_HALF_EXTENT)
        if min(math.dist((x, y), p) for p in PILLARS) > MIN_PILLAR_CLEARANCE:
            pts.append((round(x, 3), round(y, 3)))
    return pts


class WaypointTour(Node):

    def __init__(self, waypoints, loop, frame='world',
                 spawn_x=SPAWN_X, spawn_y=SPAWN_Y, spawn_yaw=SPAWN_YAW):
        super().__init__('send_waypoints')
        self.waypoints = waypoints
        self.loop = loop
        self.frame = frame
        self.spawn_x, self.spawn_y, self.spawn_yaw = spawn_x, spawn_y, spawn_yaw
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def world_to_map(self, x_w, y_w):
        """Live-look-up version of world_to_map_xy(). Returns None if
        map->odom isn't available yet (SLAM hasn't started publishing it -
        see README "Drive forward first")."""
        for _ in range(50):  # ~5s: give TF a chance to populate on first call
            try:
                t = self.tf_buffer.lookup_transform('map', 'odom', Time())
                break
            except (LookupException, ConnectivityException, ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.1)
        else:
            return None

        tx, ty = t.transform.translation.x, t.transform.translation.y
        qz, qw = t.transform.rotation.z, t.transform.rotation.w
        yaw = 2 * math.atan2(qz, qw)
        return world_to_map_xy(
            x_w, y_w, self.spawn_x, self.spawn_y, self.spawn_yaw, tx, ty, yaw)

    def send_one(self, x, y):
        """Blocking: send one goal, wait for the result, return True on SUCCEEDED."""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return False

        if self.frame == 'world':
            converted = self.world_to_map(x, y)
            if converted is None:
                self.get_logger().error(
                    'map->odom not available - is SLAM tracking? '
                    '(drive forward first; see README)')
                return False
            x_map, y_map = converted
            self.get_logger().info(
                f'-> world ({x:.2f}, {y:.2f}) = map ({x_map:.2f}, {y_map:.2f})')
        else:
            x_map, y_map = x, y
            self.get_logger().info(f'-> map ({x_map:.2f}, {y_map:.2f})')

        for attempt in range(MAX_RETRIES + 1):
            result = self._attempt(x_map, y_map)
            if result == 'SUCCEEDED':
                return True
            if not should_reverse_and_retry(result, attempt, MAX_RETRIES):
                return False
            self.get_logger().warn(
                f'   {result} - reversing {REVERSE_DURATION_SEC:.1f}s and retrying '
                f'({attempt + 1}/{MAX_RETRIES})')
            self._reverse()
        return False

    def _attempt(self, x_map, y_map):
        """One NavigateToPose goal, start to finish. Returns 'SUCCEEDED',
        'CANCELED', 'ABORTED', 'REJECTED', or 'TIMEOUT' (no result within
        STUCK_TIMEOUT_SEC - this is what used to hang the whole script
        forever with zero output)."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # Zero stamp = "latest" to tf2. A real stamp pins the lookup to one
        # instant; once the TF buffer moves past it every lookup fails and
        # the goal aborts - see README "The zero stamp matters".
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = x_map
        goal.pose.pose.position.y = y_map
        goal.pose.pose.orientation.w = 1.0

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if not handle.accepted:
            self.get_logger().warn('   rejected')
            return 'REJECTED'

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=STUCK_TIMEOUT_SEC)

        if not result_future.done():
            self.get_logger().warn(f'   no result after {STUCK_TIMEOUT_SEC:.0f}s')
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return 'TIMEOUT'

        # GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        label = names.get(result_future.result().status, str(result_future.result().status))
        self.get_logger().info(f'   {label}')
        return label

    def _reverse(self):
        """Drive straight back for REVERSE_DURATION_SEC, then stop. Backs the
        robot off whatever it's stuck against, and the translation itself
        helps monocular SLAM regain parallax if tracking was the culprit."""
        twist = Twist()
        twist.linear.x = REVERSE_SPEED
        steps = max(1, int(REVERSE_DURATION_SEC / 0.1))
        for _ in range(steps):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.cmd_vel_pub.publish(Twist())  # stop

    def run(self):
        ok = total = 0
        while True:
            for x, y in self.waypoints:
                total += 1
                if self.send_one(x, y):
                    ok += 1
            if not self.loop:
                break
        self.get_logger().info(f'{ok}/{total} waypoints reached')
        return ok == total


def demo():
    """Self-check: waypoint string parsing, and that every generated stop
    actually clears the pillars it's near."""
    assert parse_waypoints('0.9,0.4 0.2,-0.9') == [(0.9, 0.4), (0.2, -0.9)]
    assert parse_waypoints('1,2') == [(1.0, 2.0)]
    assert parse_waypoints(DEFAULT_WAYPOINTS) == [
        (0.9, 0.4), (0.2, -0.9), (-0.9, -0.4), (-0.2, 0.9), (0.9, -0.9)]

    tour = parse_waypoints(TEXTURE_TOUR_WAYPOINTS)
    assert len(tour) == 9 + 4, f'expected 9 pillars + 4 walls, got {len(tour)}'

    pillar_stops, wall_stops = tour[:9], tour[9:]
    for px, py in PILLARS:
        nearest = min(math.dist((px, py), s) for s in pillar_stops)
        assert nearest < 1.0, f'no tour stop near pillar ({px},{py})'
    for x, y in pillar_stops:
        # every stop must clear EVERY pillar, not just the one it targets
        closest_pillar = min(math.dist((x, y), p) for p in PILLARS)
        assert closest_pillar > MIN_PILLAR_CLEARANCE - 0.05, (  # -0.05: no margin req. here
            f'stop ({x},{y}) only {closest_pillar:.3f}m from a pillar')

    assert len(wall_stops) == 4
    for x, y in wall_stops:
        assert abs(abs(x) - 2.0) < 1e-6 or abs(abs(y) - 2.0) < 1e-6, \
            f'wall stop {(x, y)} not 0.5m off a wall'

    loop = parse_waypoints(LOOP_WAYPOINTS)
    assert len(loop) == 4
    wall_inner = 2.5 - 0.05
    for x, y in loop:
        assert max(abs(x), abs(y)) < wall_inner, f'loop point {(x,y)} too close to a wall'
        assert min(math.dist((x, y), p) for p in PILLARS) > MIN_PILLAR_CLEARANCE, \
            f'loop point {(x,y)} too close to a pillar'

    # random interior sampling: clearance, in-bounds, and seed reproducibility
    rng_a, rng_b = random.Random(7), random.Random(7)
    pts_a = random_interior_waypoints(10, rng_a)
    pts_b = random_interior_waypoints(10, rng_b)
    assert pts_a == pts_b, 'same seed must reproduce the same points'
    assert random_interior_waypoints(10, random.Random(8)) != pts_a
    for x, y in pts_a:
        assert max(abs(x), abs(y)) <= ARENA_HALF_EXTENT
        assert min(math.dist((x, y), p) for p in PILLARS) > MIN_PILLAR_CLEARANCE

    # world_to_map_xy: identity transform (no spawn offset, no drift) is a no-op
    x, y = world_to_map_xy(1.5, -0.7, 0, 0, 0, 0, 0, 0)
    assert (round(x, 6), round(y, 6)) == (1.5, -0.7)

    # spawn offset only (the common case: SLAM just initialised, no drift yet -
    # map->odom is identity, so world and odom coincide after the spawn shift)
    x, y = world_to_map_xy(0.0, 0.0, SPAWN_X, SPAWN_Y, 0.0, 0.0, 0.0, 0.0)
    assert (round(x, 6), round(y, 6)) == (round(-SPAWN_X, 6), round(-SPAWN_Y, 6))

    # pure map->odom rotation: a point 1m ahead in odom, with map rotated 90 deg,
    # should land 1m to the side in map
    x, y = world_to_map_xy(1.0, 0.0, 0, 0, 0, 0, 0, math.pi / 2)
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9, (x, y)

    # stuck-recovery decision: retry ABORTED/TIMEOUT within budget, never REJECTED
    # or SUCCEEDED, never past max_retries
    assert should_reverse_and_retry('ABORTED', 0, 2) is True
    assert should_reverse_and_retry('TIMEOUT', 1, 2) is True
    assert should_reverse_and_retry('ABORTED', 2, 2) is False   # exhausted
    assert should_reverse_and_retry('SUCCEEDED', 0, 2) is False
    assert should_reverse_and_retry('REJECTED', 0, 2) is False
    assert should_reverse_and_retry('CANCELED', 0, 2) is False

    print('demo ok')


def main():
    if len(sys.argv) == 2 and sys.argv[1] == 'demo':
        demo()
        return

    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoints', default=None,
                         help="'x1,y1 x2,y2 ...' in the map frame; "
                              "overrides --preset if given")
    parser.add_argument('--preset', default='default', choices=sorted(PRESETS),
                         help="'textures' visits every uniquely-textured "
                              "pillar and wall")
    parser.add_argument('--loop', action='store_true',
                         help='repeat the tour indefinitely')
    parser.add_argument('--interior', type=int, default=0,
                         help='append N random stops weaving between the '
                              'pillars, after the preset/waypoints tour')
    parser.add_argument('--seed', type=int, default=None,
                         help='seed for --interior, for a reproducible tour')
    parser.add_argument('--frame', default='world', choices=('world', 'map'),
                         help="'world' (default): coordinates are real Gazebo "
                              "positions, converted to map frame at send time. "
                              "'map': coordinates are already in map frame "
                              "(e.g. copied from a working RViz click) - sent "
                              "as-is, no conversion.")
    parser.add_argument('--spawn-x', type=float, default=SPAWN_X)
    parser.add_argument('--spawn-y', type=float, default=SPAWN_Y)
    parser.add_argument('--spawn-yaw', type=float, default=SPAWN_YAW,
                         help='must match the x_pose/y_pose/yaw the sim was '
                              'launched with, if not the sim.launch.py defaults')
    parser.add_argument('--retries', type=int, default=MAX_RETRIES,
                         help='reverse-and-retry attempts before giving up on '
                              'a stuck/aborted goal and moving to the next')
    parser.add_argument('--stuck-timeout', type=float, default=STUCK_TIMEOUT_SEC,
                         help='seconds with no result before treating a goal as stuck')
    args, ros_args = parser.parse_known_args()

    global MAX_RETRIES, STUCK_TIMEOUT_SEC
    MAX_RETRIES = args.retries
    STUCK_TIMEOUT_SEC = args.stuck_timeout

    waypoints_str = args.waypoints if args.waypoints is not None else PRESETS[args.preset]
    waypoints = parse_waypoints(waypoints_str)

    if args.interior > 0:
        waypoints += random_interior_waypoints(args.interior, random.Random(args.seed))

    rclpy.init(args=ros_args)
    node = WaypointTour(waypoints, args.loop, args.frame,
                         args.spawn_x, args.spawn_y, args.spawn_yaw)
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
