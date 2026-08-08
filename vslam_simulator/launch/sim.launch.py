# Brings up Gazebo Harmonic with the textured SLAM world, spawns the robot,
# and bridges Gazebo topics into ROS 2.
#
# This is the simulation layer only - no SLAM. vslam_bringup composes this
# with the SLAM nodes.
#
#   ros2 launch vslam_simulator sim.launch.py
#   ros2 launch vslam_simulator sim.launch.py gui:=false rviz:=true

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('vslam_simulator')
    desc_share = get_package_share_directory('vslam_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.02'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    args = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_share, 'worlds', 'textured_tb3_world.sdf'),
            description='World SDF to load'),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(desc_share, 'urdf', 'gz_slim_tb3.sdf.xacro'),
            description='Robot xacro to spawn'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run the Gazebo GUI; false runs headless'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock from Gazebo'),
    ]

    # Meshes resolve through these. Without them the robot renders invisible:
    # burger_deck1.stl lives in vslam_description, the tires in turtlebot3_gazebo.
    resources = [
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(os.path.dirname(desc_share))),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(gui),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'slim_tb3',
            '-string', Command([FindExecutable(name='xacro'), ' namespace:= ', robot_sdf]),
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'], '-Y', pose['Y'],
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(sim_share, 'config', 'bridge.yaml'),
            'expand_gz_topic_names': True,
            'use_sim_time': use_sim_time,
        }],
    )

    # No robot_state_publisher: it requires URDF and this robot is described in
    # SDF. odom -> base_footprint comes from the DiffDrive plugin via the
    # bridge, but the links below it do not exist in TF at all, and Nav2 needs
    # them to place the laser. These mirror the joint poses in
    # gz_slim_tb3.sdf.xacro and must be kept in sync with it.
    # args are: x y z yaw pitch roll parent child
    def static_tf(name, xyz, rpy, parent, child):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=[*xyz, *rpy, parent, child],
            parameters=[{'use_sim_time': use_sim_time}],
        )

    frames = [
        static_tf('tf_base_link', ['0', '0', '0.010'], ['0', '0', '0'],
                  'base_footprint', 'base_link'),
        static_tf('tf_base_scan', ['-0.032', '0', '0.150'], ['0', '0', '0'],
                  'base_link', 'base_scan'),
        # pitch -0.0873 = 5 deg nose-up, matching cam_pitch in the xacro.
        # If it disagrees with the xacro, map->odom inherits the error.
        static_tf('tf_camera_link', ['0.032', '0', '0.250'], ['0', '-0.0873', '0'],
                  'base_link', 'camera_link'),
    ]

    return LaunchDescription(
        args + resources + [gazebo, gazebo_headless, spawn, bridge] + frames)
