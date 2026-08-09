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
  ros2 run vslam_navigation send_waypoints.py --loop
"""

import argparse
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
    """Self-check: waypoint string parsing."""
    assert parse_waypoints('0.9,0.4 0.2,-0.9') == [(0.9, 0.4), (0.2, -0.9)]
    assert parse_waypoints('1,2') == [(1.0, 2.0)]
    assert parse_waypoints(DEFAULT_WAYPOINTS) == [
        (0.9, 0.4), (0.2, -0.9), (-0.9, -0.4), (-0.2, 0.9), (0.9, -0.9)]
    print('demo ok')


def main():
    if len(sys.argv) == 2 and sys.argv[1] == 'demo':
        demo()
        return

    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoints', default=DEFAULT_WAYPOINTS,
                         help="'x1,y1 x2,y2 ...' in the map frame")
    parser.add_argument('--loop', action='store_true',
                         help='repeat the tour indefinitely')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = WaypointTour(parse_waypoints(args.waypoints), args.loop)
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
