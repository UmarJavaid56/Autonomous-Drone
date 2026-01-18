# RTAB-Map SLAM Integration Guide

## Overview

This guide explains how RTAB-Map SLAM is integrated with the x500_depth drone and OAK-D camera system.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Gazebo Simulation                             │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  x500_depth Drone Model with OAK-D Camera                      │ │
│  │  - RGB Camera (640x480 @ 30Hz)                                 │ │
│  │  - Depth Camera (640x480 @ 30Hz)                               │ │
│  │  - Point Cloud Generator                                       │ │
│  └────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
                     ros_gz_bridge
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Topics                                 │
│  - /oak_d_lite/rgb/image_raw        (sensor_msgs/Image)            │
│  - /oak_d_lite/rgb/camera_info      (sensor_msgs/CameraInfo)       │
│  - /oak_d_lite/depth/image_raw      (sensor_msgs/Image)            │
│  - /oak_d_lite/depth/camera_info    (sensor_msgs/CameraInfo)       │
│  - /oak_d_lite/depth/points         (sensor_msgs/PointCloud2)      │
│  - /tf                               (tf2_msgs/TFMessage)           │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                         RTAB-Map SLAM                                │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  Visual Odometry                                             │  │
│  │  - RGB-D feature tracking                                    │  │
│  │  - ICP point cloud alignment                                 │  │
│  │  - Pose estimation                                           │  │
│  └──────────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  SLAM Mapping                                                │  │
│  │  - 3D point cloud map assembly                               │  │
│  │  - Loop closure detection                                    │  │
│  │  - Graph optimization                                        │  │
│  │  - 2D occupancy grid generation                              │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                    RTAB-Map Output Topics                            │
│  - /rtabmap/odom                    (nav_msgs/Odometry)            │
│  - /rtabmap/cloud_map               (sensor_msgs/PointCloud2)      │
│  - /rtabmap/grid_map                (nav_msgs/OccupancyGrid)       │
│  - /rtabmap/mapData                 (rtabmap_msgs/MapData)         │
│  - /rtabmap/localization_pose       (geometry_msgs/PoseWith...)    │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
                          RViz2 / Navigation Stack
```

## Key Features

### 1. RGB-D SLAM
RTAB-Map uses both RGB images and depth point clouds from the OAK-D camera to:
- Extract visual features for tracking
- Align point clouds using ICP (Iterative Closest Point)
- Build consistent 3D maps

### 2. Visual Odometry
- Tracks drone motion using RGB-D visual features
- Provides pose estimates at ~2 Hz
- Uses ICP for robust registration in texture-poor environments

### 3. Loop Closure Detection
- Detects when drone revisits previously mapped areas
- Corrects accumulated drift through graph optimization
- Ensures globally consistent maps

### 4. 3D Mapping
- Assembles point clouds into coherent 3D maps
- Filters noise and outliers
- Generates 2D occupancy grids for navigation

## Configuration

### RTAB-Map Parameters

The configuration is optimized for drone navigation with the following key parameters:

#### Registration Strategy
- **Reg/Strategy**: `1` (ICP) - Uses point cloud alignment
- Suitable for environments with varying lighting
- More robust than pure visual features for drones

#### Grid Mapping
- **Grid/CellSize**: `0.05` m (5cm resolution)
- **Grid/RangeMax**: `5.0` m (maximum sensor range)
- **Grid/RangeMin**: `0.4` m (minimum sensor range)
- **Grid/3D**: `true` (generates 3D occupancy grid)

#### ICP Parameters
- **Icp/VoxelSize**: `0.05` m (downsampling resolution)
- **Icp/MaxCorrespondenceDistance**: `0.15` m
- **Icp/Iterations**: `30` (registration iterations)
- **Icp/PointToPlane**: `true` (use point-to-plane ICP)

#### Detection Rate
- **Rtabmap/DetectionRate**: `2.0` Hz
- Processes 2 frames per second to balance accuracy and performance

#### Movement Thresholds
- **RGBD/LinearUpdate**: `0.05` m (5cm movement to create new node)
- **RGBD/AngularUpdate**: `0.05` rad (~3 degrees rotation to create new node)

## Usage Modes

### 1. Mapping Mode (Default)
Creates a new map from scratch:
```bash
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py
```

The map is saved to `~/.ros/rtabmap.db`

### 2. Localization Mode
Localizes against an existing map:
```bash
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py localization:=true
```

Requires an existing map database at `~/.ros/rtabmap.db`

## Integration with Navigation Stack

RTAB-Map outputs can be used with ROS 2 Navigation Stack (Nav2):

1. **Occupancy Grid**: `/rtabmap/grid_map` → Nav2 costmap
2. **Odometry**: `/rtabmap/odom` → Nav2 localization
3. **TF Tree**: `map` → `odom` → `base_link` transforms

## Performance Considerations

### CPU Usage
- RTAB-Map is computationally intensive
- Adjust `Rtabmap/DetectionRate` to balance performance
- Lower values (1.0 Hz) reduce CPU load but may miss features

### Memory Usage
- Maps are stored in RAM during operation
- Large maps can consume significant memory
- Use `Mem/MemoryThr` to limit memory usage

### Update Rate
- Visual odometry: ~2 Hz
- Map updates: On new node creation (based on movement thresholds)
- Loop closure: When revisiting areas

## Comparison with ORB-SLAM3

| Feature | ORB-SLAM3 | RTAB-Map |
|---------|-----------|----------|
| Input | Monocular/Stereo RGB | RGB-D |
| Map Type | Sparse feature map | Dense point cloud |
| Output | Camera poses | Poses + 3D map + occupancy grid |
| Loop Closure | Yes | Yes |
| Navigation | Requires separate mapping | Built-in occupancy grid |
| CPU Usage | Lower | Higher |
| Use Case | Visual odometry | Full SLAM with mapping |

## Troubleshooting

### No Map Building
- Ensure depth camera is publishing: `ros2 topic hz /oak_d_lite/depth/points`
- Check TF tree is complete: `ros2 run tf2_tools view_frames`
- Verify RTAB-Map is receiving data: `ros2 topic echo /rtabmap/info`

### Poor Map Quality
- Increase lighting in simulation
- Move drone slowly for better feature tracking
- Adjust ICP parameters for better point cloud alignment

### High CPU Usage
- Decrease `Rtabmap/DetectionRate` from 2.0 to 1.0
- Increase movement thresholds (`RGBD/LinearUpdate`, `RGBD/AngularUpdate`)
- Reduce `Icp/Iterations` from 30 to 20

## References

- [RTAB-Map Documentation](https://github.com/introlab/rtabmap/wiki)
- [RTAB-Map ROS 2 Wiki](http://wiki.ros.org/rtabmap_ros)
- [RTAB-Map Paper](http://introlab.github.io/rtabmap/)
