# orbslam3-nav2-sim

Autonomous navigation driven by **monocular visual SLAM**. A slim TurtleBot3
localises itself with ORB-SLAM3 from a single RGB camera — no AMCL, no prebuilt
map — and Nav2 plans and drives on top of that estimate.

**ROS 2 Jazzy** · **Gazebo Harmonic** (gz-sim 8.14) · **Ubuntu 24.04**

---

## Architecture

```mermaid
flowchart TD
    subgraph GZ["Gazebo Harmonic"]
        WORLD["textured_tb3_world<br/>9 uniquely textured pillars"]
        ROBOT["slim_tb3<br/>camera + lidar + diff drive"]
    end

    BRIDGE["ros_gz_bridge<br/>gz msgs to ROS 2"]

    subgraph SLAM["vslam_slam"]
        ORB["mono_slam_node<br/>ORB-SLAM3 MONOCULAR"]
        PANG["Pangolin viewer"]
    end

    subgraph NAV["vslam_navigation - Nav2"]
        COST["costmaps<br/>rolling, lidar only"]
        PLAN["planner_server<br/>NavFn"]
        CTRL["controller_server<br/>DWB"]
        BT["bt_navigator"]
    end

    RVIZ["RViz"]

    WORLD --> ROBOT
    ROBOT -->|"/camera/image"| BRIDGE
    ROBOT -->|"/scan"| BRIDGE
    ROBOT -->|"/odom, /tf"| BRIDGE

    BRIDGE -->|"/camera/image"| ORB
    ORB --> PANG
    ORB -->|"map to odom TF"| NAV
    ORB -->|"/orbslam3/path"| RVIZ

    BRIDGE -->|"/scan"| COST
    COST --> PLAN --> CTRL
    BT --> PLAN
    BT --> CTRL
    CTRL -->|"/cmd_vel"| BRIDGE
    BRIDGE -->|"gz.msgs.Twist"| ROBOT

    BRIDGE -->|"/odom, /scan"| RVIZ

    classDef sim fill:#E1F5EE,stroke:#0F6E56,color:#04342C
    classDef ours fill:#EEEDFE,stroke:#534AB7,color:#26215C
    classDef nav fill:#FAECE7,stroke:#993C1D,color:#4A1B0C
    classDef view fill:#F1EFE8,stroke:#5F5E5A,color:#2C2C2A
    class WORLD,ROBOT,BRIDGE sim
    class ORB ours
    class COST,PLAN,CTRL,BT nav
    class RVIZ,PANG view
```

The loop that matters: **camera → ORB-SLAM3 → `map→odom` → Nav2 → `/cmd_vel` → robot**.
The lidar never touches SLAM; it only fills obstacle costmaps. Vision alone
answers "where am I".

---

## Packages

| Package | Role | Mirrors `glitch-amr` |
|---|---|---|
| `vslam_description` | Robot xacro, cut Burger mesh, RViz config | `glitch_description` |
| `vslam_simulator` | Textured world, Gazebo launch, bridge config, static TFs | `glitch_simulator` |
| `vslam_slam` | ORB-SLAM3 node, camera settings, scale calibration | `glitch_nodes` |
| `vslam_navigation` | Nav2 params and launch | `glitch_navigation` |
| `vslam_bringup` | Top-level launch composing everything | `glitch_bringup` |

```mermaid
flowchart LR
    BRINGUP["vslam_bringup"] --> SIM["vslam_simulator"]
    BRINGUP --> SLAM["vslam_slam"]
    BRINGUP --> NAV["vslam_navigation"]
    SIM --> DESC["vslam_description"]
    SLAM --> EXT["ros2_orb_slam3<br/>external, unmodified"]

    classDef ours fill:#EEEDFE,stroke:#534AB7,color:#26215C
    classDef ext fill:#F1EFE8,stroke:#5F5E5A,color:#2C2C2A
    class BRINGUP,SIM,SLAM,NAV,DESC ours
    class EXT ext
```

Only `bringup_sim.launch.py` is meant to be run directly. It includes the
per-package launch files rather than declaring nodes itself.

---

## Quick start

```bash
ros2 launch vslam_bringup bringup_sim.launch.py
```

The robot then **drives itself** for ~10 s to bootstrap SLAM — no manual
`/cmd_vel` needed. `wander_node` waits for `/odom` (proof the robot has actually
spawned) rather than a fixed delay, then drives a **randomised** path — a
different sequence of forward/turn segments every launch (`build_random_path()`
in `wander_node.py`), forward-heavy with short gentle turns — and hands control
back. Pin `seed` for a reproducible path, or disable it entirely with
`wander:=false` and drive manually yourself:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.08}}'
```

Monocular SLAM initialises from *translation* — pure rotation will never
converge, which is why the wander pattern is forward-heavy with only short
gentle turns.

With autonomous navigation:

```bash
ros2 launch vslam_bringup bringup_sim.launch.py nav:=true
```

### Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `gui` | `true` | Gazebo GUI; `false` runs headless |
| `rviz` | `true` | Start RViz with the SLAM config |
| `slam` | `true` | Start ORB-SLAM3 |
| `nav` | `false` | Start Nav2 (delayed 12 s) |
| `show_viewer` | `true` | ORB-SLAM3's own Pangolin window |
| `pose_scale` | `3.2068` | Monocular scale factor |
| `wander` | `true` | Auto-drive to bootstrap SLAM; `false` for manual control only |
| `wander_duration` | `10.0` | Seconds to auto-drive before handing back control |

---

## Setup

### Dependencies

```bash
sudo apt install -y wayland-protocols libegl1-mesa-dev libc++-dev libepoxy-dev \
                    ros-jazzy-turtlebot3-gazebo ros-jazzy-nav2-bringup
```

### Pangolin

Not packaged for 24.04. **v0.9.5 is verified working** against this ORB-SLAM3 —
no source patches needed.

```bash
cd ~/src && git clone --branch v0.9.5 --depth 1 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
cmake --build build -j12 && cmake --install build
```

Installing to `~/.local` avoids sudo but bakes no RPATH, so `~/.local/lib` must
be on `LD_LIBRARY_PATH`. `slam.launch.py` sets this itself; without it the node
dies with **exit 127** (`libpango_display.so.0` not found). To remove the
workaround entirely:

```bash
echo "$HOME/.local/lib" | sudo tee /etc/ld.so.conf.d/pangolin.conf && sudo ldconfig
```

### ORB-SLAM3

From [`Mechazo11/ros2_orb_slam3`](https://github.com/Mechazo11/ros2_orb_slam3),
**`jazzy` branch**. It must live at `~/ros2_ws/src/ros2_orb_slam3` — its
`common.cpp` hardcodes `packagePath = "ros2_ws/src/ros2_orb_slam3/"`.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --branch jazzy https://github.com/Mechazo11/ros2_orb_slam3.git
ln -s /path/to/orbslam3-nav2-sim ~/ros2_ws/src/orbslam3-nav2-sim
cd ~/ros2_ws && colcon build --symlink-install \
  --cmake-args -DCMAKE_PREFIX_PATH=$HOME/.local -DCMAKE_BUILD_TYPE=Release
```

**Nothing in `ros2_orb_slam3` is modified.** `vslam_slam` links its
`liborb_slam3_lib.so` and drives ORB-SLAM3 directly. Upstream's `mono_node_cpp`
is fed by a Python String handshake and publishes no pose at all, so it was
replaced rather than patched.

---

## The robot

TurtleBot3 Burger with the upper two decks cut off, the LDS lidar mounted
flush on the deck, and a forward monocular camera on its own mast offset
behind it, tilted 5° up. Geometry follows the official ROBOTIS burger model
rather than being re-derived.

Lidar and camera are **separate mounts, not stacked on one post.** An earlier
version ran a single rod through the lidar's own body to reach the camera
above it — not something buildable in real hardware. The camera's mast is now
offset sideways (`camera_post_x = -0.10` vs the lidar's `-0.032`) by enough to
clear the lidar's 0.0508 m radius with 7 mm to spare.

```
   camera (own mast)                    lidar (flush on deck)
       ▲                                     ▲
       │  z = 0.250, tilted 5° up            │  z = 0.0875
       │  x = -0.10                          │  x = -0.032
       │                                ══════╧══════
   ════╧════                            │  burger base │
   deck z=0.060                         └──○────────○──┘   wheels
                                              caster       x = -0.081
```

| | Value | Notes |
|---|---|---|
| Body | `0.138 × 0.138 × 0.066` | Burger base plate, mesh cut at z=62 mm |
| Wheels | `y = ±0.080`, radius `0.033` | matches real Burger |
| `wheel_separation` | `0.160` | **must** equal `2 × wheel_y` |
| Max speed | `0.22 m/s`, `2.84 rad/s` | real Burger limits |
| Camera | 480×480 @ 30 Hz, `hfov 1.047` | own mast, `x=-0.10, z=0.250`, 5° up |
| Lidar | 360 samples, 0.12–3.5 m, 10 Hz | flush on deck, `x=-0.032, z=0.0875` |
| Intrinsics | `fx = fy = 415.787`, `cx = cy = 240` | zero distortion |

**Two constraints that cause silent failures:**

`wheel_separation` must match the wheel joint poses. If they disagree, odometry
reports the wrong rotation with no visible symptom. Verified: 1.54 % yaw error
against ground truth over a 200° turn — a wrong separation would show ~79 %.

Intrinsics are **measured** from `/camera/camera_info`, not computed from the FOV
formula (which gives 415.69, off by 0.1). They live in two files and must stay in
sync: `gz_slim_tb3.sdf.xacro` and `vslam_slam/config/tb3.yaml`.

Lidar and camera are on separate mounts (see above) so neither's geometry
overlaps the other's — the camera's mast does clip a narrow arc of the lidar's
scan directly behind the robot, a small blind spot from the mount itself,
same as real hardware.

---

## TF tree

```mermaid
flowchart TD
    MAP["map"] -->|"ORB-SLAM3 correction<br/>30 Hz, post-dated"| ODOM["odom"]
    ODOM -->|"DiffDrive plugin"| BF["base_footprint"]
    BF -->|"static"| BL["base_link"]
    BL -->|"static"| SCAN["base_scan"]
    BL -->|"static"| CAM["camera_link"]
    MAP -->|"visualisation only"| OC["orbslam3_camera"]

    classDef slam fill:#EEEDFE,stroke:#534AB7,color:#26215C
    classDef sim fill:#E1F5EE,stroke:#0F6E56,color:#04342C
    class MAP,ODOM,OC slam
    class BF,BL,SCAN,CAM sim
```

There is **no `robot_state_publisher`** — it requires URDF and this robot is
described in SDF. The `base_footprint → base_link → base_scan / camera_link`
transforms are published as static transforms in `sim.launch.py` and must be kept
in sync with the xacro joint poses.

`map → odom` is a *correction* on wheel odometry, not the robot pose:

```
map→odom = map→camera × (base→camera)⁻¹ × (odom→base)⁻¹
```

Publishing `map → base_footprint` directly would give `base_footprint` two
parents and fight the DiffDrive plugin.

---

## Startup sequence

```mermaid
sequenceDiagram
    participant U as User
    participant G as Gazebo
    participant B as Bridge
    participant S as ORB-SLAM3
    participant N as Nav2

    U->>G: launch bringup
    G->>G: load world, spawn robot
    B->>B: bridge topics
    Note over S: +5 s delay
    S->>S: load 48 MB vocabulary
    S->>B: subscribe /camera/image
    Note over S: tracking state LOST
    U->>B: /cmd_vel forward
    S->>S: map initialises
    Note over S: tracking state OK
    S->>N: map to odom now published
    Note over N: +12 s delay
    N->>N: costmaps activate
    U->>N: NavigateToPose goal
    N->>B: /cmd_vel
```

The delays are load-bearing. SLAM waits for Gazebo; Nav2 waits for SLAM. **Nav2
cannot localise until you drive forward and the map initialises.**

---

## Topics

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | Gazebo → ROS | 30 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS | 30 Hz |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS | 10 Hz |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS | 30 Hz |
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS | 200 Hz |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS | 30 Hz |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | — |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | — |
| `/orbslam3/pose` | `nav_msgs/Odometry` | ORB-SLAM3 | on tracked frames |
| `/orbslam3/path` | `nav_msgs/Path` | ORB-SLAM3 | on tracked frames |

`/odom` is simulator ground truth — used for scale calibration and as an RViz
comparison, never as Nav2's localisation source.

---

## Monocular scale

Monocular SLAM has **no metric scale**. ORB-SLAM3's units are arbitrary and fixed
at map initialisation.

```mermaid
flowchart TD
    A["launch, drive forward"] --> B["map initialises<br/>arbitrary scale locked"]
    B --> C["run calibrate_scale.py<br/>drive straight"]
    C --> D["scale = odom path / slam path"]
    D --> E["relaunch with pose_scale"]
    E --> F["metric pose output"]
    F -.->|"tracking lost"| G["map re-initialises<br/>NEW arbitrary scale"]
    G -.->|"must re-measure"| C

    classDef warn fill:#FCEBEB,stroke:#A32D2D,color:#501313
    class G warn
```

```bash
ros2 run vslam_slam calibrate_scale.py
# drive straight in another terminal while it samples:
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.09}}'
```

Measured at **3.2068** in `textured_tb3_world` (odom 1.1343 m vs SLAM 0.3537
units). The script sums path segments rather than endpoint distance — measuring
endpoints underestimates any curved path.

**The value is only valid for the map it was measured on.** The dashed loop above
is the real operational cost of monocular SLAM.

---

## Nav2

```bash
ros2 launch vslam_bringup bringup_sim.launch.py nav:=true
```

Drive forward to initialise SLAM, then send goals from RViz's **"2D Goal Pose"**
toolbar tool, or:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: map}, pose: {position: {x: 0.8, y: 0.0}, orientation: {w: 1.0}}}}"
```

```mermaid
flowchart LR
    GOAL["NavigateToPose"] --> BT["bt_navigator"]
    BT --> PL["planner_server<br/>NavFn"]
    PL --> GC["global costmap<br/>12x12 rolling"]
    BT --> CT["controller_server<br/>DWB"]
    CT --> LC["local costmap<br/>3x3 rolling"]
    SCAN["/scan"] --> GC
    SCAN --> LC
    CT --> VS["velocity_smoother"]
    VS --> CMD["/cmd_vel"]

    classDef nav fill:#FAECE7,stroke:#993C1D,color:#4A1B0C
    class BT,PL,CT,VS,GC,LC nav
```

The **zero stamp matters**. A stamped goal pins the planner to one instant; once
the TF buffer moves past it, every lookup fails with *"extrapolation into the
past"* and the goal ABORTS. Zero means "latest".

### Use "2D Goal Pose", not "Nav2 Goal"

RViz has **two different click-to-navigate tools** and only one of them works
here. This cost real debugging time, so it's worth being explicit:

| Tool | Result |
|---|---|
| **"2D Goal Pose"** (plain toolbar icon) | Works |
| **"Nav2 Goal"** (toolbar icon, or the Navigation 2 panel) | **Always aborts** |

The cause is a real bug, confirmed by reading the installed `nav2_rviz_plugins`
headers: `nav2_panel.hpp` sends its action goal from its **own internal ROS
node** (`client_node_`), created without `use_sim_time`. That node stamps every
goal with **wall-clock time**, while the entire rest of the simulation — TF,
costmaps, SLAM — runs on **sim time**. Every lookup for that goal therefore
fails "extrapolation into the future", and the goal aborts immediately,
regardless of where you clicked or what the map looks like. It is not a
tuning problem; no parameter fixes a wall-time-vs-sim-time mismatch of that
size.

`goal_relay.py` works around it: the plain "2D Goal Pose" tool publishes to
`/goal_pose` from RViz's *main* node — the one `use_sim_time: true` was
actually set on — so its stamps are correct. The relay picks that topic up and
drives the `NavigateToPose` action itself, zeroing the stamp for good measure.
It runs automatically with `nav:=true`. Verified: a goal published to
`/goal_pose` → **SUCCEEDED**.

**"Waypoint / Nav Through Poses Mode" (in the same panel) has the identical
bug — don't use it either.** Confirmed by reading `nav2_panel.hpp`: it declares
a *single* `client_node_` shared by all three action clients — `NavigateToPose`,
`FollowWaypoints`, and `NavigateThroughPoses`. Clicks are collected via
`GoalPoseUpdater`, a Qt signal carrying raw `x, y, theta` doubles (not a
stamped ROS message), and the panel stamps the final action goal itself using
that same broken node. Same wall-clock-vs-sim-clock mismatch, same guaranteed
abort — nothing route-specific to single goals about the fix.

No `goal_relay`-style fix exists for this one: there's no plain-RViz tool that
collects multiple waypoints the way "2D Goal Pose" collects one, so a fix here
means building a whole collection UI. **For a multi-stop demo, just send single
goals back-to-back via "2D Goal Pose"** — each one already works reliably, and
the visual result (robot drives to A, then B, then C) is identical.

### Sending a tour of waypoints from code

`send_waypoints.py` is that back-to-back pattern, scripted — no clicking:

```bash
ros2 run vslam_navigation send_waypoints.py
ros2 run vslam_navigation send_waypoints.py --waypoints "0.9,0.4 0.2,-0.9 -0.9,-0.4"
ros2 run vslam_navigation send_waypoints.py --loop
```

Each point is sent as its own `NavigateToPose` goal (zero-stamped, same as
`goal_relay`) and the script blocks until it finishes before sending the next —
so it's not a real `FollowWaypoints` action, just the reliable single-goal path
called in sequence.

Verified: sent, `SUCCEEDED`. Also observed a case where a leg stalled for
minutes — a tracking-loss + re-anchor happened right as that goal was sent.
That's the project's known monocular fragility (see "Tracking stability"
above), not a bug in the script; it correctly kept waiting on the real result
rather than reporting false success.

**`--preset textures`** is a ready-made tour of every uniquely-textured
surface in the world — all 9 pillars (each running a different pattern, see
`make_textures.py`) plus all 4 differently-textured walls:

```bash
ros2 run vslam_navigation send_waypoints.py --preset textures
```

Every pillar stop is **computed**, not eyeballed: 0.6 m from the pillar centre
along the line toward the arena's centre — clears the 0.18 m inflation layer
plus the 0.11 m robot radius with margin, and close enough to frame the
texture. The self-check (`send_waypoints.py demo`) verifies every stop clears
*every* pillar, not just the one it targets, since some approach points sit
close to a neighbour in the grid. Visit order is a serpentine sweep of the 3×3
grid, then the 4 walls, to minimise backtracking.

Verified live: first leg (pillar_1, the `(-1.1,-1.1)` corner) — `SUCCEEDED`
in ~21s.

### Seeing the planning

With `nav:=true`, RViz loads `nav.rviz` instead of `slam.rviz`, adding everything
the planner produces:

| Display | Topic | Colour |
|---|---|---|
| Global plan | `/plan` | green |
| Local plan (DWB trajectory) | `/local_plan` | yellow |
| Global costmap | `/global_costmap/costmap` | costmap scheme |
| Local costmap | `/local_costmap/costmap` | costmap scheme |
| Footprint | `/local_costmap/published_footprint` | cyan |
| Lidar | `/scan` | white points |
| SLAM trajectory | `/orbslam3/path` | violet |
| Ground truth | `/odom` | orange arrows |

It also loads the **Navigation 2 panel** and the **GoalTool**, so you can click
"Nav2 Goal" in the toolbar and set targets directly in the 3D view.

Measured on a live goal: `/plan` carried **60 poses**, `/local_plan` **6**.

Inspect the plans from the terminal instead:

```bash
ros2 topic echo /plan --once            # global NavFn path
ros2 topic echo /local_plan --once      # DWB rollout
ros2 topic hz /global_costmap/costmap
```

### Swapping the planner

`NavFn` (Dijkstra/A*) is the default. Also installed: `nav2_smac_planner`
(2D, hybrid-A*, lattice) and `nav2_theta_star_planner`. Change
`planner_server.GridBased.plugin` in `vslam_navigation/config/nav2_params.yaml`:

```yaml
GridBased:
  plugin: "nav2_smac_planner::SmacPlannerHybrid"   # feasible curved paths
  # plugin: "nav2_theta_star_planner::ThetaStarPlanner"  # any-angle, fewer waypoints
```

`NavfnPlanner` ignores robot kinematics and produces grid-aligned paths;
Hybrid-A* respects the turning radius, which matters more on larger robots than
this one.

Two deliberate differences from a stock TurtleBot3 Nav2 config:

- **No AMCL, no map_server.** ORB-SLAM3 publishes `map → odom`, so visual SLAM
  replaces particle-filter-on-a-prebuilt-map. That is the point of the project.
- **No static layer.** There is no prebuilt occupancy grid, so both costmaps are
  rolling windows populated purely by the lidar.

Footprint is `robot_radius: 0.11` — the body is 0.138 wide, but the wheels reach
`y = ±0.089`.

**Verified:** two `NavigateToPose` goals SUCCEEDED, reaching within 0.13 m and
0.23 m of target.

---

## The world

`textured_tb3_world.sdf` keeps the TurtleBot3 layout — 9 pillars on a ±1.1 m
grid — inside a 5×5 m walled arena.

Two departures from the stock world, both deliberate:

**Pillars are cylinders, not `hexagon.dae`.** The mesh has no reliable UVs, so
albedo maps would not apply to it.

**Every pillar is textured differently.** Nine identical pillars cause perceptual
aliasing: DBoW2 place recognition matches the wrong one and corrupts the map.
Each texture carries 679–2157 ORB features; the rendered camera view measures
**1682 features per frame**, against ORB-SLAM3's 1000/frame default.

Regenerate textures with:

```bash
python3 vslam_simulator/scripts/make_textures.py vslam_simulator/materials/textures
python3 vslam_simulator/scripts/make_textures.py demo   # self-check
```

The self-check asserts on **ORB feature count**, not pixel variance — a texture
can look busy to a human and still be nearly invisible to the detector.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `exit code 127`, `libpango_display.so.0 not found` | Pangolin in `~/.local`, no RPATH | Launch file sets `LD_LIBRARY_PATH`; or run `ldconfig` step above |
| Robot renders invisible | `meshes/` not installed | `install(DIRECTORY urdf meshes rviz ...)` in CMakeLists |
| Goal ABORTS, `extrapolation into the past` | Goal carried a real timestamp | Send with `stamp: {sec: 0, nanosec: 0}` |
| SLAM never initialises | Rotating, not translating | Drive **straight** first |
| `tracking lost` near walls | Too few features in view | Drive gently, stay off walls |
| Nav2 costmaps empty | `map → odom` missing | Drive forward until SLAM tracks |
| Trajectory drives sideways / into floor | Optical→ROS rotation skipped | See `publishPose()` |
| Odometry distance wrong, no other symptom | `wheel_separation` ≠ 2 × wheel_y | Change both together |
| Stale sim behaviour after edits | Old `gz sim` server still holding the world | `pkill -f 'gz[ ]sim'` |

---

## Tracking stability

Monocular tracking used to collapse under Nav2: the Atlas spawned a **new map
every few seconds**, each with its own origin and scale, so `map → odom` jumped
and goals aborted. Two changes fixed it.

**Rotation is the primary killer.** Pure rotation gives monocular SLAM no
parallax. Nav2's defaults rotate hard and DWB spins in place to align with the
path, so the angular limits are capped well below what the robot can do:

| Parameter | Was | Now |
|---|---|---|
| `max_vel_theta` (DWB) | 2.0 | **0.6** |
| `acc_lim_theta` | 3.2 | **1.2** |
| `RotateToGoal.scale` | 32.0 | **16.0** |
| `PathAlign.scale` | 32.0 | **40.0** |
| `max_rotational_vel` (recovery) | 1.0 | **0.6** |
| velocity smoother θ | 2.0 | **0.6** |

The DiffDrive plugin still permits 2.84 rad/s; Nav2 deliberately uses a fraction.

**More features survive sparse views** (`tb3.yaml`):

| Parameter | Was | Now |
|---|---|---|
| `ORBextractor.nFeatures` | 1000 | **1500** |
| `iniThFAST` | 20 | **12** |
| `minThFAST` | 7 | **4** |

**Result:** a full navigation run with **zero tracking losses** and a single map,
versus constant re-initialisation before, on that test run. Under sustained
multi-goal navigation, losses still occur — see "Map continuity" and "Camera
geometry" below for what further reduced them, and "Tracking stability" for
what remains.

## Map continuity across tracking losses

Even with the above, ORB-SLAM3 spawns a **new map with a new origin** every time
tracking recovers from a loss. Publishing that origin raw made `map` teleport,
which broke every outstanding Nav2 goal — the planner would return
`Resulting plan has 0 poses in it` and abort, because the goal (expressed in
`map`) no longer meant anything.

`mono_slam_node` now **re-anchors** each new map onto the previous one at the
moment tracking returns, using odometry to bridge the gap — the same idea AMCL
uses when it loses confidence. `map` stays continuous through a reset; only
metric accuracy drifts, not goal validity.

**A follow-on bug from that fix**, since caught: the re-anchor was applied to
the `map -> odom` TF only. `/orbslam3/pose` and `/orbslam3/path` kept
publishing raw coordinates from whichever map happened to be active — so every
reset still made the visualised path (and the `orbslam3_camera` TF) teleport
to the new map's own origin, even though `map -> odom` was already correct.
In RViz this showed up as the SLAM trajectory shooting off in a random
direction and zigzagging after every tracking loss.

Fixed by computing the anchor once and applying it everywhere pose data is
published — `correctAndAnchor()` is now the single place `/orbslam3/pose`,
`/orbslam3/path`, the camera TF, and `map -> odom` all get corrected from, so
they cannot disagree about which map they're anchored to.

**Verified with 3 forced tracking losses** (aggressive rotation) — 3930 path
points sampled, mean point-to-point distance 0.001 m, and the only jumps above
0.15 m were the 3 reset events themselves, capped at 0.33 m (ordinary
dead-reckoning drift during the blind window, not a teleport).

Also fixed: the inflation radius (0.35 m) very nearly sealed the gaps between
pillars (0.30 m diameter on a 1.1 m grid, leaving a 0.10 m corridor against a
0.22 m-wide robot) so the planner would refuse to thread the grid at all.
Lowered to 0.18 m, corridor width 0.44 m.

And the progress checker (0.5 m / 15 s) was incompatible with the capped
rotation speed — a slow in-place turn could exceed 15 s without moving 0.5 m
and get aborted as "Failed to make progress" even when navigating correctly.
Relaxed to 0.2 m / 40 s.

## Camera geometry

The camera sat at `z=0.115`, horizontal. From ~0.2 m out, the **bottom half of
every frame was floor** — close, low-parallax, and repetitively textured, a poor
diet for a feature tracker. Raised to `z=0.250` with a 5° upward tilt; floor now
enters the frame at ~0.54 m and pillars/walls dominate. The static
`camera_link` TF in `sim.launch.py` was updated to match — if it disagrees with
the xacro, `map -> odom` silently inherits the mounting error.

The camera was later moved from its own forward bracket onto the lidar's post
(`x=-0.032`, same as the lidar), directly above it — one sensor mast instead of
two separate mounts. Both static TFs and the xacro were updated together.

The world was also enriched to give the raised view something to track: walls
raised from 1.0 m to 1.8 m with **four distinct textures** (four identical walls
would alias the same way nine identical pillars did), plus five boxes at varying
heights and positions.

**Measured across a five-goal run:** tracking losses 11 → 8, planner `0 poses`
failures 2 → 0, goals succeeded 3/5 → 4/5. The fifth goal in that run did not
ABORT — it was still recovering from consecutive tracking losses when the test's
90 s timeout expired. Real, if milder, instance of the same weakness.

## Visual-inertial mode (does not currently work)

`IMU_MONOCULAR` is implemented and selectable:

```bash
ros2 launch vslam_bringup bringup_sim.launch.py use_imu:=true
```

It loads `config/tb3_imu.yaml` with correct camera-IMU extrinsics, subscribes
`/imu` (198 Hz measured), and buffers frames until the IMU stream covers each
timestamp before tracking.

**It does not initialise on this robot.** ORB-SLAM3 reports `scale too small`
and `IMU is not or recently initialized. Reseting active map...` in a loop.

The reason is fundamental, not a tuning problem: **visual-inertial scale is only
observable under acceleration.** A differential-drive robot cruising at near
constant velocity in a plane is a degenerate case — the accelerometer sees little
beyond gravity, so the scale estimate never converges. Aggressive
speed-up/slow-down cycles were not enough.

Left in place because it is correct and would work on a platform with real
excitation (a drone, or a handheld camera). **Default is `use_imu:=false`.**

## Known limits

Monocular scale is still calibrated rather than metric, and `pose_scale` remains
valid only for the map it was measured on. Tracking is now stable enough that
re-initialisation is rare, but it is not impossible.

**No map persistence.** `System.LoadAtlasFromFile` / `SaveAtlasToFile` are
commented out, so every run starts from scratch.

**Map points are not published.** ORB-SLAM3's point cloud exists internally but
only pose is exposed to ROS.

`map → odom` falls back to a **static identity placeholder** when `slam:=false`,
purely so RViz has a connected TF tree. It is not a localisation estimate.

---

## Repository layout

```
vslam_description/
  urdf/gz_slim_tb3.sdf.xacro     robot: body, camera, lidar, diff drive
  meshes/burger_deck1.stl        Burger base, upper decks cut at z=62mm
  scripts/cut_stl.py             regenerates the cut mesh
  rviz/slam.rviz                 path vs odom, camera, TF
vslam_simulator/
  worlds/textured_tb3_world.sdf  9 uniquely textured pillars, walled arena
  materials/textures/            generated, 512x512
  scripts/make_textures.py       texture generator + ORB self-check
  config/bridge.yaml             ros_gz_bridge topic map
  launch/sim.launch.py           Gazebo + spawn + bridge + static TFs
vslam_slam/
  src/mono_slam_node.cpp         ORB-SLAM3 driver, pose + map->odom
  config/tb3.yaml                camera intrinsics, ORB parameters
  scripts/calibrate_scale.py     monocular scale measurement
  launch/slam.launch.py
vslam_navigation/
  config/nav2_params.yaml        no AMCL, no static layer
  launch/navigation.launch.py
vslam_bringup/
  launch/bringup_sim.launch.py   the only entry point
```

## License

Apache-2.0. TurtleBot3 meshes are Apache-2.0 from ROBOTIS.
