---
name: vslam-sim
user-invocable: true
allowed-tools: Read, Write, Edit, Bash, Glob, Grep
description: Conventions and hard-won facts for the monocular ORB-SLAM3 TurtleBot3 simulation (ROS 2 Jazzy + Gazebo Harmonic). Read before touching any xacro, world, launch file, or the ORB-SLAM3 wrapper.
---

# vslam-sim

Monocular ORB-SLAM3 on a slim TurtleBot3, in Gazebo Harmonic, rendered in RViz.
ROS 2 Jazzy, Ubuntu 24.04.

## Package layout

Prefix is `vslam`.

| Package | Owns |
|---|---|
| `vslam_description` | robot xacro, rviz configs |
| `vslam_simulator` | worlds, gz launch, ros_gz bridge yaml |
| `vslam_slam` | camera driver node, ORB-SLAM3 settings, upstream patch |
| `vslam_bringup` | `bringup_sim.launch.py` |

Do not create `vslam_navigation` until Nav2 actually lands. An empty package is a
lie about what the repo does.

## Naming conventions

- Launch files: snake_case, `_sim` / `_real` suffix where the distinction exists.
- Config yaml is named for its domain (`bridge.yaml`, `tb3.yaml`), not its package.
- **`bringup_*.launch.py` is the only entry point a user runs.** It `IncludeLaunchDescription`s
  the per-package launch files; it does not declare `Node` actions itself.
- Sentence-case comments, no decorative banners.

## Environment facts

Three things that look broken but are not:

1. **`gz` is not on PATH until Jazzy is sourced.** It lives at
   `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`. A bare `gz sim --version` failing
   with a `GZ_CONFIG_PATH` complaint is expected, not a missing install.
   Gazebo Harmonic (`gz-sim8` 8.14.0) and `ros_gz` are installed.
2. **Pangolin is NOT packaged for 24.04** — build from source. Verified working:
   **v0.9.5**, built Release/Ninja, installed to `~/.local` (avoids sudo).
   `orb_slam3/Thirdparty/Pangolin` in the repo is an empty placeholder the build
   never references — Pangolin is found via `find_package`, not vendored.
   **Because it is in `~/.local` and nothing bakes an RPATH, `~/.local/lib` must
   be on `LD_LIBRARY_PATH`** or `mono_node_cpp` fails at load with
   `libpango_display.so.0 => not found`. This is exported from `~/.bashrc`.
   Build the wrapper with `--cmake-args -DCMAKE_PREFIX_PATH=$HOME/.local`.
   OpenCV 4.6 and Eigen3 are already present.
3. **`ros2_orb_slam3` hardcodes its own path.** `src/common.cpp` sets
   `packagePath = "ros2_ws/src/ros2_orb_slam3/"` relative to `$HOME` and derives
   the vocabulary and settings paths from it. The clone must live at
   `~/ros2_ws/src/ros2_orb_slam3`. Symlink this project into `~/ros2_ws/src/`
   rather than trying to relocate the wrapper.

Ignore the `Cannot locate rosdep definition for [libcrypto]` error — the key is
bogus in the wrapper's `package.xml`, and `libssl-dev` / `libcrypto.so.3` are
present. `python3-natsort` is a real dependency of `mono_driver_node.py`.

`nav2_minimal_tb3_sim` is installed and is the fork source for both the robot
(`urdf/gz_waffle.sdf.xacro`) and the world (`models/turtlebot3_world/model.sdf`).
Fork and trim; do not model from scratch.

## The `mono_node_cpp` contract

The `jazzy` branch is **monocular only** — stereo and RGBD are "TODO next version".

`mono_node_cpp` is **not** a plain image subscriber. It ignores all input until a
handshake completes, and it fails silently if you skip it.

| Topic | Type | Direction |
|---|---|---|
| `/mono_py_driver/experiment_settings` | `std_msgs/String` | → SLAM |
| `/mono_py_driver/exp_settings_ack` | `std_msgs/String` (`"ACK"`) | ← SLAM |
| `/mono_py_driver/timestep_msg` | `std_msgs/Float64` | → SLAM |
| `/mono_py_driver/img_msg` | `sensor_msgs/Image` | → SLAM |

Required order:

1. Publish the settings name repeatedly on `experiment_settings` until `"ACK"`
   arrives on `exp_settings_ack`, then stop publishing it.
2. Only then, per frame: **`Float64` timestep first, `Image` second.** That order
   is what the C++ side expects.

The settings name resolves to
`~/ros2_ws/src/ros2_orb_slam3/orb_slam3/config/Monocular/<name>.yaml`.

`gz_camera_driver.py` in `vslam_slam` replaces upstream's `mono_driver_node.py`,
which reads EuRoC images off disk.

When adding `tb3.yaml`, **copy the repo's existing `EuRoC.yaml` and edit values
into it.** Do not hand-write it — the key schema differs between ORB-SLAM2 and
ORB-SLAM3 and across versions, and most tutorials online show the ORB-SLAM2 form.

## Robot geometry

Slim chassis: burger width, waffle-pi height. Primitive boxes, no mesh.

| | Value |
|---|---|
| Body (collision + visual box) | `0.138 × 0.138 × 0.141` |
| Wheel poses | `y = ±0.080` |
| `<wheel_separation>` | `0.160` |
| `<wheel_radius>` | `0.033` |
| Camera joint | `0.064 0 0.115` (centred) |
| Lidar | `base_scan` link and `lidar_joint` deleted |

**`<wheel_separation>` must match the wheel joint poses.** If they disagree,
odometry silently reports the wrong distance and everything downstream inherits
the error with no visible symptom. The stock waffle values are `±0.144` / `0.287`
— both must change together. Verified: 1.54% yaw error vs ground truth over a
200° turn (slip). A wrong separation would show as ~1.79x, not 1.5%.

Geometry follows the **official ROBOTIS burger** at
`/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`,
not re-derived values. Single rear caster at `x=-0.081` — improvising two
casters caused spontaneous yaw drift.

Body visual is `meshes/burger_deck1.stl`: the burger base with its upper two
decks cut off at z=62mm (see `meshes/README.md`). The cut leaves an open top,
capped by the `deck_plate_visual` box, which the camera mast mounts to.

**Collision box bottom must sit at z=0.000, not lower.** Ground contact is at
z=-0.010; a box reaching that far scrapes the floor.

**Two paths are required on `GZ_SIM_RESOURCE_PATH`** or meshes silently fail to
resolve and the robot renders invisible:

```
export GZ_SIM_RESOURCE_PATH=<repo root>:/opt/ros/jazzy/share/turtlebot3_gazebo/models
```

**Kill stale `gz sim` servers before testing** (`pkill -f 'gz sim'`). A running
server holds the world name, so `gz service .../create` silently targets the old
server with the old model — which looks exactly like a physics bug.

## Camera intrinsics

The stock waffle camera is a `type="depth"` sensor at **320×240 @ 5 Hz**. 5 Hz is
far too slow for ORB-SLAM3 to track. Change it to `type="camera"`, drop the
`<depth_camera>` block, and set:

```
update_rate     30
horizontal_fov  1.047
width/height   480x480
topic           camera/image
```

Intrinsics — **take these from Gazebo's own `/camera/camera_info`, not from the
hand formula.** `fx = (width/2)/tan(hfov/2)` gives 415.69, but Gazebo actually
reports 415.787; its internal conversion differs slightly. Measured values:

```
fx = fy = 415.787    cx = 240     cy = 240
all distortion = 0.0    (ideal sim camera, no calibration needed)
```

To re-measure after any camera change:
`gz topic -e -t /camera/camera_info -n 1`

**These live in two places and must stay in sync:** the sensor block in
`vslam_description/urdf/gz_slim_tb3.sdf.xacro` and
`vslam_slam/config/tb3.yaml`. Changing resolution or FOV in one without the other
produces tracking that degrades subtly rather than failing loudly. If you touch
either, recompute `fx` and update both.

## World texture

`turtlebot3_world/model.sdf` uses **Gazebo-classic `<script>` materials**
(`Gazebo/White`) on all 9 cylinders. Harmonic does not resolve these, so the
scene renders uniformly white. Monocular ORB-SLAM3 cannot initialize against
grey-on-grey — there are no corners to detect.

Replace every `<material><script>…</script></material>` with PBR materials:
distinct `<diffuse>` per cylinder, `<albedo_map>` on walls and ground. Keep the
layout so the spawn pose `(-2.0, -0.5)` stays valid.

## Frames and scale

Two failure modes that look like bugs elsewhere:

**Optical → ROS rotation.** ORB-SLAM3 returns `Tcw` (world-to-camera) as a
`Sophus::SE3f` in the camera-optical frame: Z-forward, X-right, Y-down. REP-103
wants X-forward, Z-up. Order of operations when publishing pose:

1. Invert `Tcw` → `Twc`.
2. Scale the translation.
3. Apply the optical→ROS rotation.

> If the trajectory in RViz drives sideways or into the floor, it is step 3.
> This is the single most common bug in this pipeline.

**Monocular scale.** There is none — mono SLAM is scale-free. Do not try to solve
this analytically. Expose a `scale` parameter (default `1.0`), calibrate once by
driving a measured straight line and comparing SLAM path length to `/odom` path
length, then set `scale = odom_dist / slam_dist`.

Scale drifts over long runs and **resets to a different value after any tracking
loss**. That is inherent to monocular SLAM, not a bug to chase. Upgrading to RGBD
is not currently an option — the `jazzy` branch does not implement it.

Guard pose publishing on tracking state. Publish nothing while tracking is lost,
rather than emitting identity poses that look like the robot teleported home.

## Launch composition

Reuse from `nav2_minimal_tb3_sim/launch/spawn_tb3.launch.py`:

- the xacro-via-`Command([FindExecutable(name='xacro'), …])` pattern for `-string`
- both `AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', …)` actions — without
  them, meshes and worlds will not resolve

Set `use_sim_time: true` on every node. Delay `mono_node_cpp` and the driver by
4–6 s so the handshake does not fire into a Gazebo that has not come up yet.

`vslam_simulator/launch/sim.launch.py` brings up world + robot + bridge.
Two things that bite:

- **No `robot_state_publisher`.** It requires URDF; this robot is described in
  SDF, so it fails at startup. `odom -> base_footprint` TF comes from the
  DiffDrive plugin through the bridge instead.
- **Use `UnlessCondition(x)`, never `IfCondition(['not ', x])`** — the latter
  raises `invalid condition expression ... got 'not false'`.

The launch file sets `GZ_SIM_RESOURCE_PATH` itself, so no manual export is
needed when launching this way.

## Verification discipline

Gate each layer before moving to the next. Never debug two layers at once.

| Gate | Check |
|---|---|
| ORB-SLAM3 builds | upstream EuRoC demo tracks in Pangolin |
| Sim | `gz topic -l` shows `/camera/image` @ 30 Hz; `/cmd_vel` drives |
| Odometry | `/odom` distance matches a measured floor distance |
| World | camera view shows textured, high-contrast scene |
| Handshake | `ros2 topic echo /mono_py_driver/exp_settings_ack` yields `"ACK"` |
| Tracking | Pangolin builds a map from the sim feed |
| Scale | SLAM path length ≈ `/odom` path length over a straight drive |
| End to end | one `ros2 launch`; `/orbslam3/path` tracks `/odom` in RViz |

Drive slowly during monocular initialization. Fast rotation kills it.
