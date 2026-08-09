#!/usr/bin/env python3
"""Drive a randomised forward/turn path for a few seconds after startup.

Nothing in this stack moves the robot on its own - SLAM cannot initialise its
map without translation, and Nav2 cannot localise without a map. Previously
that meant manually publishing /cmd_vel after every launch just to bootstrap
tracking. This does it automatically, then gets out of the way.

Waits for /odom (proof Gazebo has actually spawned the robot and the bridge is
up) rather than a fixed delay, since startup time varies a lot between GUI and
headless runs. The path is a randomised sequence of forward/turn segments built
once at start - see build_random_path() - still forward-heavy with only short,
gentle turns. See vslam_navigation/config/nav2_params.yaml and the README for
why: rotation is the primary cause of monocular tracking loss.

  ros2 run vslam_slam wander_node.py
  ros2 run vslam_slam wander_node.py --ros-args -p duration:=15.0 -p seed:=42
"""

import random
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def build_random_path(duration, rng, linear, angular,
                       forward_range=(1.5, 4.0), turn_range=(0.5, 1.5)):
    """A random sequence of (linear.x, angular.z, seg_duration) segments
    covering `duration` seconds.

    Alternates forward legs of random length with turns of random length and
    random direction, so the shape of the path differs run to run - useful for
    exercising SLAM against more than one fixed trajectory. Kept forward-heavy
    on purpose: turns are short relative to forward legs and gentle in speed,
    since rotation is what kills monocular tracking.

    Pure function, isolated from ROS/rclpy and from wall-clock time so it can
    be checked directly - see demo().
    """
    segments = []
    remaining = duration
    forward = True
    while remaining > 1e-6:
        lo, hi = forward_range if forward else turn_range
        seg = min(rng.uniform(lo, hi), remaining)
        if forward:
            segments.append((linear, 0.0, seg))
        else:
            direction = rng.choice((-1.0, 1.0))
            segments.append((0.0, direction * angular, seg))
        remaining -= seg
        forward = not forward
    return segments


def pattern_at(elapsed, segments):
    """elapsed seconds -> (linear.x, angular.z), looked up in a segment list
    built by build_random_path(). Elapsed past the end holds the last segment's
    velocity - the caller is expected to stop the robot itself once done."""
    t = 0.0
    for lin, ang, seg_dur in segments:
        if elapsed < t + seg_dur:
            return lin, ang
        t += seg_dur
    return segments[-1][0], segments[-1][1] if segments else (0.0, 0.0)


class WanderNode(Node):

    def __init__(self):
        super().__init__('wander_node')

        self.duration = self.declare_parameter('duration', 10.0).value
        self.linear = self.declare_parameter('linear_speed', 0.07).value
        self.angular = self.declare_parameter('angular_speed', 0.15).value
        # -1 means unseeded: a different random path every launch. Pin a seed
        # to reproduce a specific run.
        seed = self.declare_parameter('seed', -1).value
        self.rng = random.Random(None if seed < 0 else seed)
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.start_time = None
        self.finished = False
        self.segments = None

        # Waiting for real odometry, not a fixed sleep, means this works the
        # same whether Gazebo takes 10s or 25s to spawn the robot.
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, 1)

        self.get_logger().info(
            f'waiting for {odom_topic} before wandering '
            f'({self.duration:.0f}s random path, then stopping)')

    def on_odom(self, _msg):
        if self.start_time is not None:
            return
        self.segments = build_random_path(
            self.duration, self.rng, self.linear, self.angular)
        self.start_time = self.get_clock().now()
        self.destroy_subscription(self.odom_sub)
        self.get_logger().info(
            f'robot alive - driving a random path of {len(self.segments)} segments')
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed >= self.duration:
            if not self.finished:
                self.pub.publish(Twist())
                self.finished = True
                self.timer.cancel()
                self.get_logger().info('bootstrap drive complete, handing back control')
            return

        lin, ang = pattern_at(elapsed, self.segments)
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)


def demo():
    """Self-check: path covers the requested duration, is forward-heavy, and
    two different seeds produce two different paths."""
    duration, lin, ang = 10.0, 0.07, 0.15

    path_a = build_random_path(duration, random.Random(1), lin, ang)
    total = sum(seg[2] for seg in path_a)
    assert abs(total - duration) < 1e-6, f'segments should cover duration: {total}'

    forward_secs = sum(seg[2] for seg in path_a if seg[0] != 0.0)
    assert forward_secs / duration > 0.5, 'path should be forward-heavy'

    for lin_v, ang_v, _ in path_a:
        assert (lin_v == 0.0) != (ang_v == 0.0) or (lin_v == 0.0 and ang_v == 0.0), \
            'a segment should not drive forward and turn at once'
        assert abs(ang_v) in (0.0, ang), f'turn speed should be +-{ang}: got {ang_v}'

    path_b = build_random_path(duration, random.Random(2), lin, ang)
    assert path_a != path_b, 'different seeds should produce different paths'

    same_seed = build_random_path(duration, random.Random(1), lin, ang)
    assert path_a == same_seed, 'same seed should reproduce the same path'

    assert pattern_at(0.0, path_a) == (path_a[0][0], path_a[0][1])
    print('demo ok')


def main(args=None):
    rclpy.init(args=args)
    node = WanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == 'demo':
        demo()
    else:
        main()
