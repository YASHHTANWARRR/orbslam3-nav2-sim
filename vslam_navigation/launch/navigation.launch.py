# Nav2 for the slim TurtleBot3, localised by ORB-SLAM3.
#
# Deliberately does NOT start map_server or AMCL: localisation is map -> odom
# from vslam_slam, and both costmaps are rolling windows fed by the lidar, so
# there is no prebuilt occupancy grid to serve.
#
#   ros2 launch vslam_navigation navigation.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    nav_share = get_package_share_directory('vslam_navigation')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]

    # No map_server, no amcl - see module docstring.
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    nav2_nodes = GroupAction([
        SetParameter('use_sim_time', use_sim_time),

        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}]),

        # See goal_relay.py: the RViz "Nav2 Goal" toolbar button and the
        # Navigation 2 panel stamp goals with WALL time from an internal node
        # nav2_rviz_plugins creates without use_sim_time, so every goal sent
        # that way aborts immediately regardless of where you clicked. This
        # node relays the plain "2D Goal Pose" tool instead, which correctly
        # respects use_sim_time because it runs on RViz's main node.
        Node(package='vslam_navigation', executable='goal_relay.py',
             name='goal_relay', output='screen'),
    ])

    return LaunchDescription(args + [nav2_nodes])
