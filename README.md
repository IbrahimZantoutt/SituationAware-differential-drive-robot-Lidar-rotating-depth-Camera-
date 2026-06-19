# R2D2 — A Situation-Aware Differential-Drive Robot

> A ROS 2 robot that drops into an unknown apartment, builds a map of it from
> scratch, and explores every reachable room on its own — all in simulation,
> with a lidar, a rotating depth camera, and a custom C++ navigation brain.

This project is a from-scratch **autonomous mobile robot** built on ROS 2
Humble and Gazebo. You can drive it to any point you click, or flip a single
switch and watch it map an entire apartment by itself using frontier-based
exploration. The whole navigation layer — including a hand-ported port of the
`explore_lite` frontier search — lives in a single, heavily-commented C++ node.

---

## Table of contents

- [What it does](#what-it-does)
- [The robot](#the-robot)
- [Architecture at a glance](#architecture-at-a-glance)
- [Feature deep-dive](#feature-deep-dive)
  - [1. SLAM — mapping the world live](#1-slam--mapping-the-world-live)
  - [2. Nav2 — planning and obstacle avoidance](#2-nav2--planning-and-obstacle-avoidance)
  - [3. The custom NavigationServer](#3-the-custom-navigationserver)
  - [4. Autonomous frontier exploration](#4-autonomous-frontier-exploration)
  - [5. The rotating camera](#5-the-rotating-camera)
- [Tech stack](#tech-stack)
- [Repository layout](#repository-layout)
- [Build it yourself](#build-it-yourself)
- [Run it](#run-it)
- [Tuning knobs](#tuning-knobs)
- [Design notes & gotchas](#design-notes--gotchas)

---

## What it does

- 🗺️ **Maps an unknown environment in real time** using a 360° lidar and
  `slam_toolbox`.
- 🧭 **Navigates to any goal you give it**, planning a global path and steering
  around obstacles it discovers along the way (Nav2).
- 🤖 **Explores entirely on its own.** Publish `true` to one topic and the robot
  repeatedly finds the most promising unexplored boundary ("frontier"), drives
  to it, and repeats until the whole reachable space is mapped.
- 🎥 Carries a **rotating depth camera** on a controllable joint, driven by a
  dedicated ROS node.
- 🏠 Ships with a hand-built **apartment world** (`world2.sdf`) with rooms and
  walls to explore.

Here's the headline feature — autonomous exploration — in one command:

```bash
ros2 topic pub --once /explore_all std_msgs/msg/Bool "{data: true}"
```

…then sit back and watch it map the apartment.

---

## The robot

The robot is described in plain URDF ([robot.urdf](src/r2d2_main/urdf/robot.urdf)):

| Part | Detail |
|------|--------|
| **Chassis** | 0.28 × 0.28 × 0.35 m box, ~3 kg, scaled to 70% of the original design |
| **Drive** | 4 wheels (two pairs), differential drive via `libgazebo_ros_diff_drive` |
| **Lidar** | 360 samples over a full circle, 0.15–12 m range, 10 Hz, mounted on top |
| **Camera** | 640×480, ~80° FOV, 30 Hz, mounted on a **rotating joint** |
| **Frames** | `map → odom → robot_base → {lidar_link, camera_rotator_link → camera_link}` |

The differential-drive plugin publishes `/odom` and the `odom → robot_base`
transform, and subscribes to `/cmd_vel`. The lidar publishes `/scan`. The
camera publishes `/camera/image_raw`.

> **Note on the camera control:** the project deliberately avoids
> `ros2_control` / `controller_manager`. The available `gazebo_ros2_control`
> (0.4.x) is incompatible with the newer `ros2_control` parser, so the rotator
> joint is driven instead by the standalone `gazebo_ros_joint_pose_trajectory`
> plugin and read back via `gazebo_ros_joint_state_publisher`. This was a
> pragmatic choice to keep the joint commandable without fighting the stack.

---

## Architecture at a glance

```
                 ┌────────────────────────────────────────────────────┐
                 │                     Gazebo                          │
                 │   world2.sdf  +  R2D2 (diff-drive, lidar, camera)   │
                 └──────┬───────────────┬──────────────┬──────────────┘
                /scan   │        /odom  │     /clock,   │ /joint_states
                        │        + TF   │  /camera/...  │
                        ▼               ▼               ▼
                 ┌─────────────┐   ┌─────────┐    ┌──────────────┐
                 │ slam_toolbox│   │  Nav2   │    │ CameraRotator│
                 │  (mapping)  │   │ planner │    │   (node)     │
                 │ map + map→  │   │ +ctrl + │    └──────────────┘
                 │ odom TF     │   │costmaps │
                 └──────┬──────┘   └────┬────┘
                        │  /map         │ navigate_to_pose (action)
                        │  global_costmap/costmap
                        ▼               ▲
                 ┌────────────────────────────────────┐
                 │        NavigationServer (C++)       │
                 │  • custom `navigate` action         │
                 │  • frontier search (explore_lite)   │
                 │  • /explore_all autonomous loop     │
                 └────────────────────────────────────┘
                        ▲
                        │ Nav action (target_x, target_y)
                 ┌──────┴───────┐
                 │NavigationClient (CLI: type X,Y)     │
                 └──────────────────────────────────────┘
```

Everything is brought up by a single launch file. The pieces talk over standard
ROS 2 topics, actions and TF.

---

## Feature deep-dive

### 1. SLAM — mapping the world live

Mapping is handled by `slam_toolbox` running in **async mapping mode**
([mapper_params.yaml](src/r2d2_main/config/mapper_params.yaml)). It consumes
`/scan` and odometry, runs Ceres-based scan matching, and publishes:

- the live occupancy grid on `/map` (5 cm resolution), and
- the crucial `map → odom` transform.

There's **no AMCL and no pre-saved map** in the live pipeline — the robot starts
knowing nothing and the map grows as it drives.

### 2. Nav2 — planning and obstacle avoidance

Navigation uses the standard **Nav2** stack
([nav2_params.yaml](src/r2d2_main/config/nav2_params.yaml)), tuned for this
small differential robot:

- **Global planner:** NavFn (Dijkstra) with `allow_unknown: true`, so it can
  plan *into* unexplored space — essential for exploration.
- **Local planner / controller:** DWB, tuned with a 0.26 m/s top speed and a
  critic set balancing path-following and obstacle clearance.
- **Costmaps:** a rolling local costmap (voxel + inflation, fed live by the
  lidar) for reactive avoidance, and a global costmap with
  `track_unknown_space: true` so unknown cells stay genuinely "unknown" — which
  the exploration logic depends on.
- **Recovery behaviors:** spin, backup, drive-on-heading, wait, assisted teleop.

Because slam_toolbox already provides `map → odom`, Nav2 runs **without**
`map_server` or AMCL.

### 3. The custom NavigationServer

[NavigationServer.cpp](src/r2d2_main/src/NavigationServer.cpp) is the heart of
the project (~750 lines, heavily commented). It does two jobs:

**(a) A simple goal interface.** It exposes a custom `Nav` action
([Nav.action](src/action_interfaces/action/Nav.action)) that takes just a
`(target_x, target_y)` and forwards it to Nav2's `navigate_to_pose`, relaying
feedback (current position, distance remaining) back to the caller. The matching
[NavigationClient.cpp](src/r2d2_main/src/NavigationClient.cpp) lets you simply
type X and Y coordinates in a terminal to send the robot somewhere.

**(b) The autonomous exploration brain** — see below.

### 4. Autonomous frontier exploration

This is the most interesting part. The server contains a **from-scratch C++ port
of `m-explore-ros2`'s `explore_lite`** frontier search, adapted to operate on
Nav2's global costmap instead of a raw costmap_2d.

**What's a frontier?** A frontier is a cluster of *unknown* cells that border
*free* space. Driving to one is guaranteed to reveal new map. The algorithm:

1. **BFS over free space** outward from the robot's current cell.
2. Whenever it touches an unknown cell that borders free space, it **grows the
   whole frontier cluster** (8-connected) and records its size, centroid, and the
   point closest to the robot.
3. Each frontier is **scored**: `cost = potential_scale · distance − gain_scale ·
   size`. Lower is better — so it prefers large, nearby frontiers.
4. It sends the lowest-cost reachable frontier's centroid to Nav2 as a goal.

The loop runs on a timer (`planner_frequency`, default 0.33 Hz) and includes the
robustness tricks that make `explore_lite` actually work:

- **Frontier blacklisting** — if the robot can't make progress toward a frontier
  (no real motion within `progress_timeout`), or Nav2 aborts it, that frontier
  goes on a blacklist (with a 5-cell tolerance) so it's never retried.
- **No goal thrashing** — once it commits to a frontier it keeps driving there
  rather than re-picking a near-equal frontier every cycle (which would make the
  robot dither and falsely blacklist good frontiers).
- **Progress = real motion**, not straight-line distance to the goal — so a path
  that winds around walls doesn't get flagged as "stuck."
- **Traversing inflated corridors** — the search treats any known cell below a
  cost threshold as navigable, so it can reach pockets behind inflated doorways.

It also publishes the frontiers as an RViz `MarkerArray` on `/explore/frontiers`
(blue points for cells, green/red spheres for live/blacklisted centroids), so you
can *watch it think*.

Start and stop it with one topic:

```bash
ros2 topic pub --once /explore_all std_msgs/msg/Bool "{data: true}"   # start
ros2 topic pub --once /explore_all std_msgs/msg/Bool "{data: false}"  # stop
```

When no frontiers remain, exploration declares itself complete and stops.

### 5. The rotating camera

[CameraRotator.cpp](src/r2d2_main/src/CameraRotator.cpp) drives the camera's
`rotator_joint`. It listens on `/camera_angle_target` (a latched `Float64`),
reads the current angle back from `/joint_states`, computes the shortest angular
path, and publishes a single-point `JointTrajectory` to the Gazebo plugin. (The
server currently parks it at 0.0 / straight ahead, but the plumbing for a
steerable depth camera is all there.)

> A subtle bug fixed here: the trajectory's `header.stamp` is left at **0** on
> purpose. The plugin schedules points against *sim* time while the node runs on
> *wall* time — a wall-clock stamp would push the point ~1.7×10⁹ s into the sim
> future and it would never execute. Stamp 0 means "apply immediately."

---

## Tech stack

- **ROS 2 Humble** — middleware, actions, TF2
- **Gazebo Classic 11** — physics simulation
- **C++17** — NavigationServer, NavigationClient, CameraRotator
- **Nav2** — planning, control, costmaps, recovery behaviors
- **slam_toolbox** — online async SLAM (Ceres solver)
- **`gazebo_ros` plugins** — diff drive, lidar (ray), camera, joint pose
  trajectory, joint state publisher
- **`ament_cmake`** build system, with a custom `rosidl` action package

---

## Repository layout

```
R2D2/
├── src/
│   ├── r2d2_main/                  # main package
│   │   ├── src/
│   │   │   ├── NavigationServer.cpp   # ⭐ Nav2 bridge + frontier exploration
│   │   │   ├── NavigationClient.cpp   # CLI: type X,Y to send a goal
│   │   │   ├── CameraRotator.cpp      # rotating-camera controller
│   │   │   └── LidarDetect.cpp        # placeholder
│   │   ├── launch/
│   │   │   ├── bringup.launch.py      # ⭐ everything: gazebo + slam + rviz + nav2
│   │   │   ├── gazebo.launch.py       # gazebo + world + robot + slam_toolbox
│   │   │   ├── nav2.launch.py         # Nav2 stack
│   │   │   ├── slam_rviz.launch.py    # RViz only
│   │   │   └── display.launch.py      # URDF viewer (no sim)
│   │   ├── config/
│   │   │   ├── nav2_params.yaml       # Nav2 tuning
│   │   │   └── mapper_params.yaml     # slam_toolbox tuning
│   │   ├── urdf/robot.urdf            # the robot
│   │   ├── worlds/{world.sdf,world2.sdf}   # world2 = the apartment
│   │   ├── rviz/                      # RViz configs
│   │   └── maps/                      # saved map(s)
│   └── action_interfaces/            # custom Nav.action definition
├── build/  install/  log/            # colcon output (generated)
└── README.md
```

---

## Build it yourself

### Prerequisites

- **Ubuntu 22.04** with **ROS 2 Humble** ([install guide](https://docs.ros.org/en/humble/Installation.html))
- **Gazebo Classic 11** and the ROS bridge packages

Install the dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro
```

### Clone and build

```bash
# clone into a workspace (the repo root is the workspace)
git clone https://github.com/IbrahimZantoutt/SituationAware-differential-drive-robot-Lidar-rotating-depth-Camera-.git R2D2
cd R2D2

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> Build the `action_interfaces` package first if colcon doesn't resolve the
> order automatically: `colcon build --packages-select action_interfaces` then
> `colcon build`.

---

## Run it

**Terminal 1 — bring up the whole simulation** (Gazebo + the apartment + SLAM +
RViz, with Nav2 starting after an 8 s delay so its costmaps initialize cleanly):

```bash
source install/setup.bash
ros2 launch r2d2_main bringup.launch.py
```

**Terminal 2 — start the navigation brain:**

```bash
source install/setup.bash
ros2 run r2d2_main NavigationServer
```

Now pick one of two modes:

**Drive it manually** — either use RViz's *2D Goal Pose* tool, or run the CLI
client and type coordinates:

```bash
source install/setup.bash
ros2 run r2d2_main NavigationClient
# Enter target X: 3.0
# Enter target Y: 1.5
```

**Let it explore on its own:**

```bash
ros2 topic pub --once /explore_all std_msgs/msg/Bool "{data: true}"
```

Add the `/explore/frontiers` MarkerArray display in RViz to watch the frontiers
it's targeting. Stop any time with `{data: false}`.

**Optional — drive the camera node:**

```bash
ros2 run r2d2_main CameraRotator
```

> **Switching worlds:** edit the one line `world_file = 'world2.sdf'` near the
> top of [gazebo.launch.py](src/r2d2_main/launch/gazebo.launch.py).

---

## Tuning knobs

The exploration server exposes the same parameters as `explore_lite`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `potential_scale` | 3.0 | Penalty on distance to a frontier (higher → prefer closer) |
| `gain_scale` | 1.0 | Reward for frontier size (higher → prefer bigger) |
| `min_frontier_size` | 0.5 m | Ignore frontiers smaller than this |
| `planner_frequency` | 0.33 Hz | How often the explore loop runs |
| `progress_timeout` | 30 s | Blacklist a frontier if no real motion for this long |
| `frontier_occupied_threshold` | 90 | Costmap cost (0–100) at/above which a cell blocks the search |

Pass them at launch, e.g.:

```bash
ros2 run r2d2_main NavigationServer --ros-args -p gain_scale:=2.0 -p progress_timeout:=45.0
```

Robot speed, costmap inflation, and goal tolerances live in
[nav2_params.yaml](src/r2d2_main/config/nav2_params.yaml).

---

## Design notes & gotchas

A few decisions worth knowing about, learned the hard way:

- **Exploration runs on Nav2's global costmap, not the raw SLAM map.** The
  costmap's inflation gives frontier targets clearance from walls, and walls
  block connectivity so the robot never tries to reach unknown space sealed
  behind them. This requires `track_unknown_space: true` (already set).
- **Gazebo's online model DB is dead.** The launch file clears
  `GAZEBO_MODEL_DATABASE_URI` and points at local models — otherwise Gazebo
  hangs ~30 s on startup trying to reach `models.gazebosim.org`, which makes
  `spawn_entity` time out.
- **Nav2 is delayed 8 s** behind Gazebo + SLAM so TF, `/scan` and `/odom` are
  flowing before the costmaps come up.
- **No `ros2_control`.** See [the camera note](#the-robot) above — the rotator
  joint uses standalone `gazebo_ros` plugins instead.

---

*Built with ROS 2 Humble, Gazebo, and a lot of frontier BFS. Contributions and
questions welcome.*
