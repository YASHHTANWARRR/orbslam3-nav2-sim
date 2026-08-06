#!/usr/bin/env python3
"""Feed the simulated Gazebo camera into ORB-SLAM3's mono_node_cpp.

Replaces ros2_orb_slam3's mono_driver_node.py, which reads EuRoC images off
disk. The wire protocol is identical, because mono_node_cpp is not a plain
image subscriber - it ignores everything until a handshake completes:

  1. publish the settings name ONCE on /mono_py_driver/experiment_settings,
     then wait for "ACK" on /mono_py_driver/exp_settings_ack
  2. only then, per frame: Float64 timestep FIRST, Image SECOND

Exactly once matters. experimentSetting_callback in common.cpp has no re-entry
guard: it calls initializeVSLAM on every message, and initializeVSLAM does
    settingsFilePath.append(configString).append(".yaml")
on a member. A second config message therefore builds "tb3.yamltb3.yaml",
fails to open it, and segfaults. Upstream's driver publishes in a loop and
survives only by luck.

Instead of retrying, this waits for the subscriber to appear before publishing,
which removes the reason to retry at all.

The timestep-then-image ordering is not stylistic either. mono_node_cpp stores
the timestep in a member from one callback and consumes it in the other, so an
image arriving before its timestep is tracked against the previous frame's time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String


class GzCameraDriver(Node):

    def __init__(self):
        super().__init__('gz_camera_driver')

        self.declare_parameter('settings_name', 'tb3')
        self.declare_parameter('image_topic', '/camera/image')

        self.settings_name = self.get_parameter('settings_name').value
        image_topic = self.get_parameter('image_topic').value

        self.config_sent = False
        self.handshake_done = False
        self.frames_forwarded = 0
        self.frames_dropped = 0

        self.pub_config = self.create_publisher(
            String, '/mono_py_driver/experiment_settings', 1)
        self.pub_img = self.create_publisher(
            Image, '/mono_py_driver/img_msg', 1)
        self.pub_timestep = self.create_publisher(
            Float64, '/mono_py_driver/timestep_msg', 1)

        self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack', self.on_ack, 10)

        # Best-effort matches the bridge and tolerates a reliable publisher.
        self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data)

        self.handshake_timer = self.create_timer(0.2, self.send_handshake)

        self.get_logger().info(
            f"waiting for mono_node_cpp (settings '{self.settings_name}', "
            f"images from '{image_topic}')")

    def send_handshake(self):
        """Publish the config exactly once, as soon as anyone is listening."""
        if self.config_sent:
            return
        if self.pub_config.get_subscription_count() == 0:
            return  # mono_node_cpp not up yet; a message now would be dropped
        self.pub_config.publish(String(data=self.settings_name))
        self.config_sent = True
        self.handshake_timer.cancel()
        self.get_logger().info(f"sent settings '{self.settings_name}', awaiting ACK")

    def on_ack(self, msg):
        if self.handshake_done or msg.data != 'ACK':
            return
        self.handshake_done = True
        self.get_logger().info('handshake complete - forwarding frames')

    def on_image(self, msg):
        # Frames before the handshake would be silently discarded by the C++
        # node; drop them here so the count is visible.
        if not self.handshake_done:
            self.frames_dropped += 1
            return

        stamp = msg.header.stamp
        timestep = stamp.sec + stamp.nanosec * 1e-9

        # order matters - see module docstring
        self.pub_timestep.publish(Float64(data=timestep))
        self.pub_img.publish(msg)

        self.frames_forwarded += 1
        if self.frames_forwarded % 300 == 0:
            self.get_logger().info(f'{self.frames_forwarded} frames forwarded')


def main(args=None):
    rclpy.init(args=args)
    node = GzCameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'forwarded {node.frames_forwarded}, '
            f'dropped {node.frames_dropped} pre-handshake')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
