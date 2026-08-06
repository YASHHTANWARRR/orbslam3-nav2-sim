# Monocular ORB-SLAM3 against the simulated camera.
#
# Expects /camera/image to already be bridged - see
# vslam_simulator/launch/sim.launch.py.
#
#   ros2 launch vslam_slam slam.launch.py
#   ros2 launch vslam_slam slam.launch.py pose_scale:=2.35

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('vslam_slam')

    # The vocabulary ships with the wrapper, whose location is fixed by its own
    # hardcoded packagePath.
    voc_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin')

    # tb3_imu.yaml carries the IMU block; tb3.yaml is monocular-only.
    settings_file = PythonExpression([
        "'", os.path.join(slam_share, 'config', 'tb3_imu.yaml'), "'",
        " if '", LaunchConfiguration('use_imu'), "' == 'true' else ",
        "'", os.path.join(slam_share, 'config', 'tb3.yaml'), "'"])

    args = [
        DeclareLaunchArgument('image_topic', default_value='/camera/image'),
        DeclareLaunchArgument(
            'use_imu', default_value='false',
            description='Visual-inertial mode (IMU_MONOCULAR). Metrically '
                        'scaled, so pose_scale is ignored.'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'pose_scale', default_value='3.2068',
            description='Monocular scale factor, measured in textured_tb3_world '
                        'with calibrate_scale.py. Only valid for the map it was '
                        'measured on - any tracking loss re-initialises with a '
                        'different arbitrary scale, so re-measure.'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('camera_frame', default_value='orbslam3_camera'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('show_viewer', default_value='true'),
        DeclareLaunchArgument(
            'start_delay', default_value='5.0',
            description='Seconds to wait for Gazebo before starting SLAM'),
    ]

    # Pangolin lives in ~/.local with no RPATH baked in, so without this the
    # node dies with exit 127 in any shell that has not sourced ~/.bashrc.
    pangolin_libs = AppendEnvironmentVariable(
        'LD_LIBRARY_PATH', os.path.join(os.path.expanduser('~'), '.local', 'lib'))

    slam_node = Node(
        package='vslam_slam',
        executable='mono_slam_node',
        name='vslam_mono',
        output='screen',
        parameters=[{
            'voc_file': voc_file,
            'settings_file': settings_file,
            'image_topic': LaunchConfiguration('image_topic'),
            'use_imu': LaunchConfiguration('use_imu'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'pose_scale': LaunchConfiguration('pose_scale'),
            'map_frame': LaunchConfiguration('map_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'show_viewer': LaunchConfiguration('show_viewer'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(
        args + [pangolin_libs,
                TimerAction(period=LaunchConfiguration('start_delay'),
                            actions=[slam_node])])
