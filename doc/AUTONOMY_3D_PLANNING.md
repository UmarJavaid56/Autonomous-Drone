# 3D Autonomy Stack: RTAB-Map → ESDF → RRT* → Path Executor

## Overview

Fully autonomous 3D navigation for the indoor quadrotor using:

- **RTAB-Map**: SLAM from RGB-D point clouds (map, odom, base_link).
- **ESDF server**: Volumetric collision representation; answers `GetDistance(x,y,z)` (OctoMap fallback; NVBlox/Voxblox can be swapped via same interface).
- **RRT* planner**: OMPL RRT* in state space (x, y, z, yaw); collision via ESDF; goal bias; runs at ~1–3 Hz asynchronously.
- **Path executor**: Follows `nav_msgs/Path` with velocity setpoints; hover/abort when path is empty or planning fails.

Design: **No Nav2**, **no 2D costmaps**, **no LiDAR**. Planning is global only; PX4 (or sim velocity controller) handles low-level control.

## ROS 2 Graph

```
Gazebo (depth + RGB) → ros_gz_bridge → /oak_d_lite/depth/points, /tf
       ↓
RTAB-Map → map, odom, /rtabmap/odom_path, /map
       ↓
ESDF server ← /oak_d_lite/depth/points (in map frame)  →  get_distance (srv)
       ↓
RRT* planner ← goal_pose (PoseStamped), TF map→base_link, get_distance (srv)
       ↓
       → path (nav_msgs/Path), path_markers (MarkerArray), planning_active (Bool)
       ↓
Path executor ← path, planning_active, TF map→base_link
       ↓
       → /x500_depth/cmd_vel (Twist), /x500_depth/enable (Bool)
```

## Packages

| Package       | Role |
|---------------|------|
| `esdf_msgs`   | Service `GetDistance.srv` (ESDF query interface). |
| `esdf_server` | Builds volumetric map from PointCloud2; answers GetDistance (current backend: voxelized point cloud + KdTree; OctoMap/NVBlox/Voxblox can be added with same interface). |
| `rrt_star_planner` | OMPL RRT* (x,y,z,yaw); collision from ESDF service; publishes Path + markers; multi-threaded executor so service calls don’t deadlock. |
| `path_executor` | Subscribes to path and planning_active; publishes cmd_vel and enable; hovers when path empty or planning active. |

## Launch

Full pipeline (Gazebo + RTAB-Map + ESDF + RRT* + path executor + RViz):

```bash
cd ~/Autonomous-Drone/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch x500_rtabmap_slam autonomy_full.launch.py
```

- **Goal in RViz**: Use **"Publish Point"** to click a 3D point (converted to `/goal_pose` by `goal_from_rviz`), or **"2D Goal Pose"** to set (x, y, yaw). Or publish `geometry_msgs/PoseStamped` to `/goal_pose` (frame_id = `map`), or use RViz “2D Goal Pose” if a tool publishes to that topic.
- **Simulation time**: All nodes use `use_sim_time:=true` and `/clock`.

## Parameters (high level)

- **ESDF server**: `esdf_server/config/esdf_params.yaml` — voxel_size, map bounds (x_min/max, y_min/max, z_min/max), point_cloud_topic.
- **RRT* planner**: drone_radius, safety_margin, goal_bias, max_planning_time, replan_rate (e.g. 2 Hz).
- **Path executor**: waypoint_tolerance, max_linear_speed, cmd_vel_topic, enable_topic.

## Safety

- Path executor sends **hover** (zero velocity) when: path is empty, planning_active is true, or TF map→base_link is unavailable.
- Planner does not block PX4 offboard: it runs in a timer; path executor runs in a separate node and keeps publishing cmd_vel/enable.

## RViz

- **Set goal**: Use toolbar **Publish Point** (click a 3D point) or **2D Goal Pose** (click-drag); both feed `/goal_pose`.
- **Path**: Add display “Path”, topic `/path` (green line).
- **Path markers**: Add “MarkerArray”, topic `/path_markers`.
- **ESDF slice** (optional): Add “MarkerArray”, topic `/esdf_slice` (if esdf_server publishes it).
- Fixed frame: `map`.

## Dependencies

- OMPL: `libompl-dev` (added in `scripts/install_dependencies.sh`).
- PCL, tf2, sensor_msgs, nav_msgs, etc. (from existing RTAB-Map/sim setup).

## Build

From workspace root:

```bash
cd ~/Autonomous-Drone/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to esdf_msgs esdf_server rrt_star_planner path_executor
source install/setup.bash
```

If you see `pkgutil.ImpImporter` or setuptools errors (common with Python 3.12 and old `~/.local` setuptools), fix the environment first, e.g.:

```bash
pip3 install --upgrade setuptools
# or use the project venv: source .venv/bin/activate then colcon build ...
```

## Swapping ESDF Backend

The planner depends only on the `esdf_msgs/GetDistance` service. To use NVBlox or Voxblox:

1. Run an ESDF node that builds TSDF/ESDF from the same point cloud (and optionally depth images) and advertises the same service (or a wrapper that translates to GetDistance).
2. Keep the same service name `get_distance` so the RRT* planner does not need changes.

## Simulation-to-hardware

- In sim: path executor publishes to `/x500_depth/cmd_vel` and `/x500_depth/enable` (Gazebo velocity controller).
- On hardware: point path executor’s cmd_vel/enable topics to the node that sends PX4 Offboard position or velocity setpoints (e.g. px4_ros_com or a custom bridge). Path executor output remains `Twist` + `Bool`; the hardware bridge can convert to setpoint type as needed.
