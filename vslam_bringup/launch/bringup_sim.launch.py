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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

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
        DeclareLaunchArgument('nav', default_value='false',
                              description='Start Nav2. Needs SLAM tracking first, '
                                          'since map->odom comes from it.'),
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

    # Fallback only. With SLAM running, the SLAM node publishes the real
    # map -> odom correction and this must NOT run, or base_footprint would
    # effectively have two conflicting parents. Active only when slam:=false,
    # so the sim still has a connected TF tree for RViz.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_placeholder',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(slam),
    )

    # Two configs: nav.rviz adds costmaps, global/local plans and the goal tool,
    # which only exist once Nav2 is running.
    def rviz_with(config, cond):
        return Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(desc_share, 'rviz', config)],
            parameters=[{'use_sim_time': True}],
            condition=cond,
        )

    rviz_slam = rviz_with(
        'slam.rviz',
        IfCondition(PythonExpression(["'", rviz, "' == 'true' and '",
                                      LaunchConfiguration('nav'), "' != 'true'"])))
    rviz_nav = rviz_with(
        'nav.rviz',
        IfCondition(PythonExpression(["'", rviz, "' == 'true' and '",
                                      LaunchConfiguration('nav'), "' == 'true'"])))

    # Delayed: Nav2's costmaps need map -> odom to exist, which only happens
    # once ORB-SLAM3 has initialised its map. Drive the robot forward after
    # startup to bootstrap tracking.
    navigation = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('vslam_navigation'),
                    'launch', 'navigation.launch.py')),
        )],
        condition=IfCondition(LaunchConfiguration('nav')),
    )

    return LaunchDescription(
        args + [simulation, slam_stack, map_to_odom, navigation, rviz_slam, rviz_nav])
