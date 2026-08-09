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
"""

import argparse
import math
import random
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


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

    def __init__(self, waypoints, loop):
        super().__init__('send_waypoints')
        self.waypoints = waypoints
        self.loop = loop
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_one(self, x, y):
        """Blocking: send one goal, wait for the result, return True on SUCCEEDED."""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # Zero stamp = "latest" to tf2. A real stamp pins the lookup to one
        # instant; once the TF buffer moves past it every lookup fails and
        # the goal aborts - see README "The zero stamp matters".
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'-> ({x:.2f}, {y:.2f})')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if not handle.accepted:
            self.get_logger().warn('   rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        # GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        label = names.get(status, str(status))
        self.get_logger().info(f'   {label}')
        return status == 4

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
    args, ros_args = parser.parse_known_args()

    waypoints_str = args.waypoints if args.waypoints is not None else PRESETS[args.preset]
    waypoints = parse_waypoints(waypoints_str)

    if args.interior > 0:
        waypoints += random_interior_waypoints(args.interior, random.Random(args.seed))

    rclpy.init(args=ros_args)
    node = WaypointTour(waypoints, args.loop)
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
