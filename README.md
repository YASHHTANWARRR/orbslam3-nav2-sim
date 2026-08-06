# orbslam3-nav2-sim

Monocular ORB-SLAM3 on a slim TurtleBot3, in Gazebo Harmonic, visualised in RViz.

ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8) · Ubuntu 24.04

## Packages

| Package | Role |
|---|---|
| `vslam_description` | Robot xacro, cut Burger mesh, RViz config |
| `vslam_simulator` | Textured world, Gazebo launch, `ros_gz_bridge` config |
| `vslam_slam` | ORB-SLAM3 node, camera settings, scale calibration |
| `vslam_navigation` | Nav2 params and launch |
| `vslam_bringup` | Top-level launch composing everything |

## Quick start

```bash
ros2 launch vslam_bringup bringup_sim.launch.py
```

Then drive it. **Go straight first** — monocular SLAM initialises from
translation, and pure rotation will never converge:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.08}}'
```

Useful arguments:

```bash
ros2 launch vslam_bringup bringup_sim.launch.py gui:=false rviz:=false
ros2 launch vslam_bringup bringup_sim.launch.py slam:=false
ros2 launch vslam_bringup bringup_sim.launch.py pose_scale:=3.2068
```

## Setup

ORB-SLAM3 comes from [`Mechazo11/ros2_orb_slam3`](https://github.com/Mechazo11/ros2_orb_slam3)
(**`jazzy` branch**), which must live at `~/ros2_ws/src/ros2_orb_slam3` — its
`common.cpp` hardcodes that path. Pangolin is not packaged for 24.04; v0.9.5
built from source into `~/.local` is verified working.

```bash
sudo apt install -y wayland-protocols libegl1-mesa-dev libc++-dev libepoxy-dev \
                    ros-jazzy-turtlebot3-gazebo python3-natsort
```

```bash
cd ~/src && git clone --branch v0.9.5 --depth 1 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
cmake --build build -j12 && cmake --install build
```

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --branch jazzy https://github.com/Mechazo11/ros2_orb_slam3.git
ln -s /path/to/orbslam3-nav2-sim ~/ros2_ws/src/orbslam3-nav2-sim
cd ~/ros2_ws && colcon build --symlink-install \
  --cmake-args -DCMAKE_PREFIX_PATH=$HOME/.local -DCMAKE_BUILD_TYPE=Release
```

## The robot

TurtleBot3 Burger with the upper two decks cut off and a forward monocular
camera on an L-bracket. Geometry follows the official ROBOTIS burger model.

| | |
|---|---|
| Body | Burger base plate only, mesh cut at z=62mm |
| Wheels | `y = ±0.080`, `wheel_separation 0.160`, radius `0.033` |
| Camera | 480×480 @ 30 Hz at `z=0.115`, `hfov 1.047` |
| Intrinsics | `fx = fy = 415.787`, `cx = cy = 240`, no distortion |
| Lidar | removed |

Intrinsics are measured from Gazebo's `/camera/camera_info`, not computed from
the FOV formula. They must stay in sync between `gz_slim_tb3.sdf.xacro` and
`vslam_slam/config/tb3.yaml`.

## Topics

| Topic | Type | From |
|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | Gazebo, via bridge |
| `/odom` | `nav_msgs/Odometry` | DiffDrive plugin (ground truth) |
| `/cmd_vel` | `geometry_msgs/Twist` | → Gazebo |
| `/orbslam3/pose` | `nav_msgs/Odometry` | ORB-SLAM3 |
| `/orbslam3/path` | `nav_msgs/Path` | ORB-SLAM3 |

## Monocular scale

Monocular SLAM has no metric scale — ORB-SLAM3's units are arbitrary and fixed
when the map initialises. Measure the factor against wheel odometry:

```bash
ros2 run vslam_slam calibrate_scale.py
# drive straight while it samples, then pass the result:
ros2 launch vslam_bringup bringup_sim.launch.py pose_scale:=<value>
```

Measured at **3.2068** in `textured_tb3_world`. **It is only valid for the map it
was measured on.** Any tracking loss re-initialises with a different arbitrary
scale. This is inherent to monocular SLAM; feeding the robot's IMU into
`IMU_MONOCULAR` mode would make the output metric and remove the problem.

## Notes

`vslam_slam` builds its own node rather than using upstream's `mono_node_cpp`,
which is driven by a Python String handshake and publishes no pose. Nothing in
`ros2_orb_slam3` is modified. The full SLAM stack is active — tracking, local
mapping, and loop closure with Sim3 scale correction.

Pillars in the world are each textured differently on purpose. Nine identical
pillars cause perceptual aliasing, where place recognition matches the wrong one
and corrupts the map.

## Nav2

```bash
ros2 launch vslam_bringup bringup_sim.launch.py nav:=true
```

**Drive forward first.** Nav2's costmaps need `map -> odom`, which only exists
once ORB-SLAM3 has initialised its map:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.06}}'
```

Then send goals from RViz's "2D Goal Pose", or:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: map}, pose: {position: {x: 0.8, y: 0.0}, orientation: {w: 1.0}}}}"
```

The **zero stamp matters**. A stamped goal pins the planner to one instant, and
once the TF buffer moves past it every lookup fails with "extrapolation into the
past" and the goal aborts. Zero means "latest".

Two deliberate differences from a stock TurtleBot3 Nav2 config:

- **No AMCL, no map_server.** ORB-SLAM3 publishes `map -> odom`, so visual SLAM
  replaces particle-filter-on-a-prebuilt-map. That is the point of the project.
- **No static layer.** There is no prebuilt occupancy grid, so both costmaps are
  rolling windows populated purely by the lidar.

Verified: two `NavigateToPose` goals SUCCEEDED, reaching within 0.13 m and
0.23 m of target.

## Known limits

Monocular tracking is the weak link. It drops on fast rotation, on featureless
views, and near walls, and every recovery re-initialises the map with a new
arbitrary scale. Drive gently. Feeding the robot's IMU into `IMU_MONOCULAR`
would make the output metric and remove the scale problem entirely — the IMU is
already simulated and bridged, just unused.

`map -> odom` falls back to a **static identity placeholder** when `slam:=false`,
purely so RViz has a connected TF tree. It is not a localisation estimate.
