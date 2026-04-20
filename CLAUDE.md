# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace layout

Standard ROS 2 colcon workspace. Only one package: `src/yale_delivery_robot`. It is an **assets-only `ament_cmake` package** — no C++/Python nodes are compiled. `CMakeLists.txt` just installs `worlds/`, `urdf/`, `launch/`, `config/`, `maps/` into the package share directory. Any changes to those asset directories require a rebuild+resource before they are visible to `ros2 launch`.

## Target stack

- ROS 2 **Jazzy**
- Gazebo **Harmonic** (new `gz sim`, launched via `ros_gz_sim`; bridged to ROS 2 via `ros_gz_bridge parameter_bridge`)
- `slam_toolbox` (async) for mapping
- `nav2_bringup` for localization + navigation

If any launch file is rewritten for classic Gazebo (`gazebo_ros` `spawn_entity.py`, `/gazebo/` services, etc.), that is a regression — the URDF uses `gz-sim-*` plugins and expects the new stack.

## Build / run

```bash
# from workspace root
colcon build --symlink-install
source install/setup.bash
```

Use `--symlink-install` so edits to launch files, URDF, worlds, and YAML under `src/` are picked up without a rebuild. Asset **additions** (new files) still need a rebuild so colcon re-runs the install step.

Typical three-terminal workflow (each terminal must `source install/setup.bash`):

```bash
# 1. sim + robot + ros<->gz bridge
ros2 launch yale_delivery_robot yale_campus.launch.py

# 2a. first-time mapping
ros2 launch yale_delivery_robot slam.launch.py
# drive around with teleop, then save:
ros2 run nav2_map_server map_saver_cli -f src/yale_delivery_robot/maps/yale_campus_map
# rebuild so the new map is installed into share/

# 2b. after a map exists: navigation
ros2 launch yale_delivery_robot navigation.launch.py
```

`navigation.launch.py` hard-defaults `map` to `<pkg_share>/maps/yale_campus_map.yaml`. That file is **not checked in** — `src/yale_delivery_robot/maps/` is intentionally empty and must be populated by running SLAM first, or by passing `map:=<path>` on the command line.

No tests, linters, or CI are configured. `colcon test` currently runs nothing meaningful.

## Architecture

### Frames and topics (set by the URDF Gazebo plugins and the bridge)

- TF tree: `map` → `odom` (published by AMCL when navigating, by `slam_toolbox` when mapping) → `base_footprint` → `base_link` → wheels / sensors. `nav2_params.yaml` uses `base_footprint` for AMCL and `base_link` for the BT navigator — keep that split; changing it will break costmap transforms.
- Bridged topics (see `launch/yale_campus.launch.py`): `/clock`, `/scan`, `/odom`, `/cmd_vel`, `/imu`, `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`, `/tf`. `/cmd_vel` is the only ROS→GZ direction (`]` in the bridge arg); everything else is GZ→ROS (`[`).
- Lidar: 360 samples, 0.15–25 m range, 10 Hz. AMCL's `laser_max_range` in `nav2_params.yaml` is set to match (25.0). Changing the URDF lidar range without updating AMCL/costmap params will silently degrade localization.
- Depth camera publishes under `/camera/*` after bridging; the GZ sensor topic is `camera`.

### URDF

`urdf/yale_delivery_robot.urdf` is a single static URDF (not xacro — the `xmlns:xacro` is vestigial, the `xacro:property` lines are inert). Mesh files are loaded from the **installed** `turtlebot3_description` package via `package://turtlebot3_description/...`, so that package must be on the ROS 2 install path at runtime. The robot is a Turtlebot3 Waffle clone with added depth camera + IMU.

### Gazebo world

`worlds/yale_campus.world` is a hand-authored SDF world simulating a campus (roads, sidewalks, buildings). It is large (~300 lines of static models) — prefer surgical edits to regenerating. Physics is ODE at 1 kHz.

### Nav2 configuration

`config/nav2_params.yaml` configures the full Nav2 stack (AMCL, BT navigator, controller, planner, both costmaps, behavior server, smoother, velocity smoother). When tuning:
- Global/local costmap `robot_radius`, `inflation_radius`, and the obstacle layer's `obstacle_max_range` must stay consistent with the lidar range and the robot's actual footprint.
- `use_sim_time: True` is set throughout — do not flip individual nodes to `False` while running against Gazebo.
