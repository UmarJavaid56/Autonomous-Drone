# OAK-D Camera Simulation in Gazebo Harmonic - Implementation Summary

## Overview
Successfully set up OAK-D S2 camera simulation in Gazebo Harmonic (ROS 2 Jazzy) with bridging to ROS 2 topics.

## Architecture
- **Simulation Engine**: Gazebo Sim Harmonic (gz-sim v8.x)
- **ROS Version**: ROS 2 Jazzy
- **Bridge Strategy**: ros_gz_bridge for topic translation between Gazebo and ROS 2
- **Launch Mode**: Server-only (headless) with programmatic camera spawning
- **Rendering Engine**: Ogre2 (required for depth cameras)

## Key Components

### 1. Camera Models (SDF 1.9)
Two camera models defined:

#### Depth Camera: `oak_depth_camera.sdf`
- Sensor Type: `depth_camera`
- Resolution: 640x480
- Update Rate: 30 Hz
- Depth Range: 0.05m - 50m
- Frame: `oak_depth_optical_frame`
- ROS Bridge Topic: `/oak_depth_camera` and `/oak_depth_camera_info`

#### RGB Camera: `oak_rgb_camera.sdf`
- Sensor Type: `camera`
- Resolution: 640x480
- Update Rate: 30 Hz
- Frame: `oak_rgb_optical_frame`
- ROS Bridge Topic: `/oak_rgb_camera` and `/oak_rgb_camera_info`

### 2. Launch Configuration
File: `depthai_cam_sim_bridged.launch.py`

**Process Flow**:
1. Start Ignition Gazebo server (headless, -s flag)
2. Wait 2s for world initialization
3. Spawn depth camera model via `/world/depthai_test_world/create` service
4. Wait 4s total
5. Spawn RGB camera model via `/world/depthai_test_world/create` service
6. Start ros_gz_bridge nodes for bidirectional topic bridging

**ROS 2 Topics Published**:
- `/oak_depth_camera` (sensor_msgs/Image)
- `/oak_depth_camera_info` (sensor_msgs/CameraInfo)
- `/oak_rgb_camera` (sensor_msgs/Image)
- `/oak_rgb_camera_info` (sensor_msgs/CameraInfo)

### 3. Topic Bridging
Two independent `ros_gz_bridge` nodes:
- **Depth Bridge**: Bridges `/oak_depth_camera` and `/oak_depth_camera_info` between Ignition and ROS 2
- **RGB Bridge**: Bridges `/oak_rgb_camera` and `/oak_rgb_camera_info` between Ignition and ROS 2

## Solution Rationale

### Why Remove Direct ROS Plugins?
Initial approach used embedded ROS plugins in SDF (`libgz-ros2-camera-system.so`, `libgz-ros2-depth-camera-system.so`). These plugins were not available in the installed packages, causing load failures.

**Error Encountered**:
```
[Err] [SystemLoader.cc:94] Failed to load system plugin [libgz-ros2-depth-camera-system.so] : couldn't find shared library.
```

**Resolution**: Removed embedded ROS plugins from SDFs and use external bridging instead. This approach:
1. Decouples Gazebo simulation from ROS integration
2. Uses standard `ros_gz_bridge` which is reliable and well-supported
3. Keeps SDFs minimal and focused on physics/sensing

## How to Run

### Basic Launch (Bridged Cameras)
```bash
cd ~/Autonomous-Drone/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch depthai_cam depthai_cam_sim_bridged.launch.py
```

### Verify Camera Topics
In another terminal:
```bash
cd ~/Autonomous-Drone/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# List topics
ros2 topic list | grep oak

# Check topic info
ros2 topic info /oak_depth_camera

# Echo data
ros2 topic echo /oak_depth_camera --no-arr
```

## File Structure
```
depthai_cam/
├── launch/
│   ├── depthai_cam_sim.launch.py         (original, no longer used)
│   └── depthai_cam_sim_bridged.launch.py (NEW - recommended)
├── models/
│   ├── oak_depth_camera.sdf              (updated - no embedded ROS plugin)
│   └── oak_rgb_camera.sdf                (updated - no embedded ROS plugin)
├── worlds/
│   └── depthai_test.world                (SDF 1.9, inlined models)
└── ...
```

## Known Limitations
1. **Camera Data Quality**: Simulated depth/RGB images are synthetic - not realistic OAK-D output
2. **No Fisheye Distortion**: Current model uses simple perspective camera, OAK-D typically has wide FOV with distortion
3. **No IMU/Stereo Depth**: Only monocular RGB and basic depth simulation
4. **Single Instance**: Current spawn script spawns one instance of each camera at fixed pose

## Future Enhancements
1. **Multiple Camera Instances**: Modify spawn logic to support multiple cameras
2. **Dynamic Pose**: Add services to move cameras during simulation
3. **Realistic Distortion**: Implement OAK-D lens distortion model in camera sensor
4. **Stereo Rectification**: Add dual stereo pair cameras with proper stereo rectification
5. **ROS Driver Integration**: Map simulated topics to match real depthai_ros driver output

## Troubleshooting

### Topics Not Appearing
- Check bridge processes are running: `ros2 node list | grep bridge`
- Verify Ignition server is running: `ign topic -l` should list `/oak_depth_camera` etc.
- Increase spawn delay in launch file if world not fully loaded

### Camera Not Visible in Ignition GUI
- World loads in server-only mode (no GUI visualization)
- Use `gz sim` (non-server) for GUI: manually add `-s` flag to toggle

### Bridge Connection Issues
- Ensure `ros_jazzy_ros_gz_bridge` package is installed
- Check ROS 2 and Gazebo can discover each other via `gz transport`

## Environment Setup Verification
```bash
# Verify ROS 2 Jazzy
ros2 --version  # jazzy (or similar)

# Verify Gazebo Harmonic
gz sim --version  # should show 8.x

# Verify bridge package
ros2 pkg list | grep gz_bridge
```

## Installation Notes for Ubuntu 24.04

### Known Issues & Solutions

#### DART Library Conflict
On Ubuntu 24.04, the system DART physics libraries conflict with the OSRF Gazebo versions. The `scripts/install_dependencies.sh` script **automatically handles this** by:

1. Removing conflicting Ubuntu packages: `libdart6.13`, `libdart-collision-bullet6.13`, `libdart-collision-ode6.13`, `libdart-external-odelcpsolver6.13`
2. Installing OSRF's unified package: `libdart-core+collisions+odelcpsolver6.13`
3. Cleaning up stray dev packages and dependencies

**If you see "trying to overwrite libdart-*.so" errors**, use the manual fix from the README Troubleshooting section.

#### Python argcomplete Module
Some ROS 2 startup scripts reference an old `register-python-argcomplete` script that may not exist. The install script removes this stale reference to prevent import warnings.

