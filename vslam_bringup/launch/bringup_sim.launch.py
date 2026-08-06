# The only launch file you need to run.
#
# Composes the simulation (Gazebo + robot + bridge) with monocular ORB-SLAM3
# and RViz. Per the glitch-amr convention, this includes the per-package launch
# files rather than declaring nodes itself.
#
#   ros2 launch vslam_bringup bringup_sim.launch.py
#   ros2 launch vslam_bringup bringup_sim.launch.py rviz:=false gui:=false
#   ros2 launch vslam_bringup bringup_sim.launch.py pose_scale:=3.2068

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('vslam_simulator')
    slam_share = get_package_share_directory('vslam_slam')
    desc_share = get_package_share_directory('vslam_description')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    slam = LaunchConfiguration('slam')
    pose_scale = LaunchConfiguration('pose_scale')
    show_viewer = LaunchConfiguration('show_viewer')

    args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='Start ORB-SLAM3'),
        DeclareLaunchArgument('show_viewer', default_value='true',
                              description="ORB-SLAM3's own Pangolin window"),
        DeclareLaunchArgument('pose_scale', default_value='3.2068',
                              description='Monocular scale; see calibrate_scale.py'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui}.items(),
    )

    slam_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'slam.launch.py')),
        launch_arguments={
            'pose_scale': pose_scale,
            'show_viewer': show_viewer,
        }.items(),
        condition=IfCondition(slam),
    )

    # VISUALISATION PLACEHOLDER, not a localisation estimate.
    #
    # The TF tree is two disconnected branches:
    #     odom -> base_footprint   (DiffDrive, via the bridge)
    #     map  -> orbslam3_camera  (our SLAM node)
    # RViz needs one connected tree to show both under a single fixed frame, so
    # this pins map to odom with identity. It is only approximately true - it
    # holds because ORB-SLAM3's map origin is wherever it initialised, which is
    # near the spawn pose.
    #
    # Nav2 will need this replaced by a real map -> odom correction published by
    # the SLAM node. Delete this static publisher at that point.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_placeholder',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(desc_share, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(args + [simulation, slam_stack, map_to_odom, rviz_node])
