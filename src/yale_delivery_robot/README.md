# yale_delivery_robot

Simulation of an autonomous campus delivery robot for Yale, built on a Turtlebot3 Waffle base. Runs on **Ubuntu 24.04** with **ROS 2 Jazzy** + **Gazebo Harmonic**, with `cartographer_ros` for mapping, **Nav2** for navigation, and a YOLO + depth-camera pedestrian-safety override.

## Stack

| Layer             | Tool                                                    |
| ----------------- | ------------------------------------------------------- |
| OS                | Ubuntu 24.04                                            |
| ROS               | ROS 2 Jazzy                                             |
| Simulator         | Gazebo Harmonic (`gz sim`)                              |
| ROS↔GZ bridge     | `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`           |
| Mapping           | `cartographer_ros`                                      |
| Navigation        | `nav2_bringup` (AMCL, planner, controller, BT, costmaps)|
| Cmd_vel mux       | `twist_mux`                                             |
| Robot description | `robot_state_publisher` + `turtlebot3_description` mesh |
| Visualization     | `rviz2`                                                 |
| Perception        | YOLOv8n (Ultralytics) + OpenCV + `cv_bridge`            |

## Layout

```
src/yale_delivery_robot/
├── CMakeLists.txt          # installs assets + python scripts
├── package.xml
├── config/                 # nav2, twist_mux, speed zones, waypoints, bridge yaml
├── launch/                 # sim, slam, nav, rsp, spawn
├── maps/                   # populated after first SLAM run (not in git)
├── models/                 # turtlebot3_waffle GZ models
├── scripts/                # python nodes (installed to lib/)
│   ├── pedestrian_safety_node.py
│   ├── waypoint_ui_node.py
│   ├── set_initial_pose.py
│   ├── test_yolo.py
│   └── test_depth.py
├── urdf/                   # yale_delivery_robot.urdf (waffle + depth cam + IMU)
└── worlds/                 # yale_campus.world (hand-authored SDF)
```

## Build

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Typical workflow

Each terminal must `source install/setup.bash` first.

### 1. Bring up the simulation

```bash
ros2 launch yale_delivery_robot yale_campus.launch.py
```

This starts `gz sim`, spawns the robot, and runs `ros_gz_bridge` for `/clock`, `/scan`, `/odom`, `/cmd_vel`, `/imu`, `/camera/*`, `/tf`.

### 2a. First-time mapping

Mapping is done with the standalone `cartographer_ros` package (run separately, not via a launch file in this package). Drive the robot around with teleop while Cartographer builds the map, then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f src/yale_delivery_robot/maps/yale_campus_map
colcon build --symlink-install   # so the new map is installed into share/
```

### 2b. Navigation (after a map exists)

```bash
ros2 launch yale_delivery_robot navigation.launch.py
```

Defaults to `<pkg_share>/maps/yale_campus_map.yaml`. Override with `map:=<path>` if needed.

### 3. Optional perception + UI nodes

```bash
ros2 run yale_delivery_robot pedestrian_safety_node.py
ros2 run yale_delivery_robot waypoint_ui_node.py   # then open http://localhost:8765
```

## Frames and topics

- TF: `map` → `odom` (AMCL when navigating, `cartographer_ros` when mapping) → `base_footprint` → `base_link` → wheels / sensors. Nav2 uses `base_footprint` for AMCL and `base_link` for the BT navigator — keep that split.
- Lidar: 360 samples, 0.15–25 m range, 10 Hz. AMCL `laser_max_range` matches at 25.0.
- Depth camera publishes under `/camera/*` after bridging.
- `/cmd_vel` is the only ROS→GZ topic; everything else is GZ→ROS.

## Pedestrian safety node

`scripts/pedestrian_safety_node.py` runs YOLOv8n on `/camera/image_raw/compressed`, projects each person bounding-box centre into 3D using `/camera/depth/image_raw` + `/camera/camera_info`, and tracks each person with a Kalman filter. If a tracked person is close and moving toward the robot, it publishes a zero-velocity `TwistStamped` on `/cmd_vel_safety`. `twist_mux` gives this topic priority over Nav2's `/cmd_vel_nav`, freezing the robot.

Debug output: annotated RGB on `/pedestrian_debug_image`, RViz markers on `/pedestrian_markers`.

## Waypoint UI node

`scripts/waypoint_ui_node.py` serves a browser UI on `http://localhost:8765` for sending single goals or sequential routes to Nav2, watching `/behavior_tree_log` for status, cancelling via `/navigate_to_pose/_action/cancel_goal`, and streaming the YOLO debug image as MJPEG. Waypoints are loaded from `config/waypoints.yaml`.

## Configuration files

| File                          | Purpose                                                       |
| ----------------------------- | ------------------------------------------------------------- |
| `config/nav2_params.yaml`     | full Nav2 stack (AMCL, BT, controller, planner, costmaps, …) |
| `config/twist_mux.yaml`       | `/cmd_vel` priorities (safety > teleop > nav)                |
| `config/speed_zones.yaml`     | per-zone max linear/angular velocity + acceleration          |
| `config/waypoints.yaml`       | named goal poses for the UI                                  |
| `config/turtlebot3_waffle_bridge.yaml` | ros_gz_bridge topic mapping                         |

When tuning Nav2: keep global/local costmap `robot_radius`, `inflation_radius`, and the obstacle layer's `obstacle_max_range` consistent with the lidar range and the actual footprint. `use_sim_time: True` is set throughout — do not flip individual nodes to `False` while running against Gazebo.

## Dependencies

Declared in `package.xml`: `rclpy`, `robot_state_publisher`, `joint_state_publisher`, `gazebo_ros`, `nav2_bringup`, `tf2_ros`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`.

Also required at runtime (apt or workspace): `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-image`, `ros-jazzy-twist-mux`, `ros-jazzy-rviz2`, `ros-jazzy-turtlebot3-description`, `ros-jazzy-cartographer-ros` (for mapping).

Python (pip): `ultralytics`, `opencv-python`, `numpy`, `pyyaml`.

## Notes

- The URDF loads meshes via `package://turtlebot3_description/...` — that package must be on the ROS 2 install path at runtime.
- `urdf/yale_delivery_robot.urdf` is plain URDF, not xacro (the `xmlns:xacro` namespace is vestigial).
