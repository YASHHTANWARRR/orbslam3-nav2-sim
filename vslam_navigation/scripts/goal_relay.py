#!/usr/bin/env python3
"""Relay plain RViz "2D Goal Pose" clicks into a NavigateToPose action call.

Exists because of a real bug in nav2_rviz_plugins: the toolbar's "Nav2 Goal"
button and the Navigation 2 panel send their action goal from a SEPARATE
internal ROS node (nav2_panel.hpp's client_node_), created without
use_sim_time. It stamps every goal with WALL-clock time while the rest of the
simulation runs on SIM time, so every TF lookup for that goal fails and
Nav2 aborts - every time, regardless of where you clicked. Confirmed by
inspecting the installed nav2_rviz_plugins headers.

The plain "2D Goal Pose" tool (rviz_default_plugins, NOT nav2_rviz_plugins) is
a different code path: it publishes PoseStamped on /goal_pose using RViz's
main node clock, which correctly follows use_sim_time because it IS the node
we set that parameter on. This node picks that message up and drives the
action client itself, on a node that also has use_sim_time set - so the
stamps line up with the rest of the system.

Use the "2D Goal Pose" tool, not "Nav2 Goal", to send goals reliably.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalRelay(Node):

    def __init__(self):
        super().__init__('goal_relay')

        topic = self.declare_parameter('goal_topic', '/goal_pose').value
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, topic, self.on_goal, 10)
        self._goal_handle = None

        self.get_logger().info(f'relaying {topic} clicks to navigate_to_pose')

    def on_goal(self, msg):
        # A zero stamp means "latest" to tf2, sidestepping any residual clock
        # skew - the same trick the CLI workaround in the README relies on.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return

        if self._goal_handle is not None:
            self.get_logger().info('new goal received - cancelling the previous one')
            self._goal_handle.cancel_goal_async()

        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            f'sending goal x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}')

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('goal rejected')
            self._goal_handle = None
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        # GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        self.get_logger().info(f'goal finished: {names.get(status, status)}')
        self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
