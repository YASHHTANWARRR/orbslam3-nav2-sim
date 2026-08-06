# ORB-SLAM3 monocular SLAM against the simulated camera.
#
# Runs mono_node_cpp plus the driver that performs its handshake. Expects
# /camera/image to already be bridged - see vslam_simulator/launch/sim.launch.py.
#
#   ros2 launch vslam_slam slam.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('vslam_slam')

    # The wrapper hardcodes this path in src/common.cpp
    # (packagePath = "ros2_ws/src/ros2_orb_slam3/"), so the vocabulary lives
    # there regardless of where our packages sit.
    voc_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin')

    # Trailing slash is required: mono_node_cpp builds the path as
    # settings_file_path + settings_name + ".yaml"
    settings_path = os.path.join(slam_share, 'config') + '/'

    settings_name = LaunchConfiguration('settings_name')
    image_topic = LaunchConfiguration('image_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_delay = LaunchConfiguration('start_delay')

    args = [
        DeclareLaunchArgument('settings_name', default_value='tb3'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'start_delay', default_value='6.0',
            description='Seconds to wait for Gazebo before handshaking'),
    ]

    # voc_file_arg AND settings_file_path_arg must BOTH be set. common.cpp
    # falls back to its hardcoded paths if EITHER is left at its default,
    # so setting only one silently ignores it.
    slam_node = Node(
        package='ros2_orb_slam3',
        executable='mono_node_cpp',
        output='screen',
        parameters=[{
            'node_name_arg': 'mono_slam_cpp',
            'voc_file_arg': voc_file,
            'settings_file_path_arg': settings_path,
            'use_sim_time': use_sim_time,
        }],
    )

    driver_node = Node(
        package='vslam_slam',
        executable='gz_camera_driver.py',
        output='screen',
        parameters=[{
            'settings_name': settings_name,
            'image_topic': image_topic,
            'use_sim_time': use_sim_time,
        }],
    )

    # Pangolin is installed to ~/.local with no RPATH baked in, so without this
    # mono_node_cpp dies with exit 127 (libpango_display.so.0 not found) in any
    # shell that has not sourced ~/.bashrc.
    pangolin_libs = AppendEnvironmentVariable(
        'LD_LIBRARY_PATH', os.path.join(os.path.expanduser('~'), '.local', 'lib'))

    # Delayed so the handshake does not fire into a Gazebo that has not
    # finished starting.
    return LaunchDescription(
        args + [pangolin_libs,
                TimerAction(period=start_delay, actions=[slam_node, driver_node])])
