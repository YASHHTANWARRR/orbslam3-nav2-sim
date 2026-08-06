#!/usr/bin/env python3
"""Measure the monocular scale factor against wheel odometry.

Monocular SLAM has no metric scale: ORB-SLAM3's units are arbitrary and fixed
at map initialisation. This compares path length on /orbslam3/pose against
/odom and prints the ratio to pass as pose_scale.

Run it with SLAM already tracking, then drive the robot in a STRAIGHT line
while it samples:

  ros2 run vslam_slam calibrate_scale.py
  # in another terminal, within the sampling window:
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.09}}'

Then:
  ros2 launch vslam_slam slam.launch.py pose_scale:=<value>

The number is only valid for the map that was live when it was measured. Any
tracking loss re-initialises the map with a DIFFERENT arbitrary scale, so it
must be re-measured. That is inherent to monocular SLAM, not a bug.
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class ScaleCalibrator(Node):

    def __init__(self):
        super().__init__('calibrate_scale')
        self.odom = []
        self.slam = []
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(Odometry, '/orbslam3/pose', self._slam, 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom.append((p.x, p.y))

    def _slam(self, m):
        p = m.pose.pose.position
        self.slam.append((p.x, p.y))


def path_length(pts):
    """Sum of segment lengths - not endpoint distance, which underestimates
    any curved path."""
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

    rclpy.init()
    node = ScaleCalibrator()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f'sampling for {duration:.0f}s - drive the robot straight now')
    node.odom.clear()
    node.slam.clear()
    time.sleep(duration)

    odom_pts = list(node.odom)
    slam_pts = list(node.slam)
    node.destroy_node()
    rclpy.shutdown()

    print(f'odom samples {len(odom_pts)}, slam samples {len(slam_pts)}')
    if len(odom_pts) < 2:
        print('no /odom - is the sim running?')
        return 1
    if len(slam_pts) < 2:
        print('no /orbslam3/pose - SLAM is not tracking. '
              'Drive forward slowly to initialise the map first.')
        return 1

    od = path_length(odom_pts)
    sd = path_length(slam_pts)
    print(f'odom travelled : {od:.4f} m')
    print(f'slam travelled : {sd:.4f} units')

    if sd < 1e-6:
        print('slam did not move - drive further')
        return 1

    print(f'\n  pose_scale = {od / sd:.4f}\n')
    print(f'ros2 launch vslam_slam slam.launch.py pose_scale:={od / sd:.4f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
