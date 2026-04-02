# RTAB-Map SLAM Package

This package provides RTAB-Map SLAM integration for the x500_depth drone equipped with an OAK-D depth camera.

## Overview

RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D SLAM approach that uses visual features and depth information to create 3D maps and localize the robot. This package configures RTAB-Map to work with the point clouds generated from the OAK-D camera's depth images.

## Features

- **3D SLAM Mapping**: Creates detailed 3D point cloud maps using depth camera data
- **Visual Odometry**: Estimates drone pose using RGB-D visual features
- **Loop Closure Detection**: Detects when the drone revisits previously mapped areas
- **Occupancy Grid**: Generates 2D occupancy grid maps for navigation
- **Localization Mode**: Can localize against previously created maps
- **Map-Driven Maze Mission**: Accepts one final goal and generates intermediate subgoals from `/map` (A* + frontier fallback). Out-of-bounds goals trigger frontier/reachable in-map expansion before any direct-goal fallback.

## Launch Files

### rtabmap_slam.launch.py

Launches RTAB-Map SLAM node only (requires camera topics to be running separately).

**Usage:**
```bash
ros2 launch rtabmap_slam rtabmap_slam.launch.py
```

**Parameters:**
- `use_sim_time` (default: true) - Use simulation time
- `localization` (default: false) - Enable localization mode instead of mapping

### x500_rtabmap_slam.launch.py

Complete launch file that includes:
- x500_depth Gazebo simulation with OAK-D camera
- RTAB-Map SLAM
- RViz2 visualization

**Usage:**
```bash
# Mapping mode (default)
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py

# Localization mode (requires existing map)
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py localization:=true

# Without RViz
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py rviz:=false
```

### autonomy_full.launch.py

Full autonomy stack (Gazebo + RTAB-Map + ESDF + RRT* + path executor):

```bash
ros2 launch x500_rtabmap_slam autonomy_full.launch.py rviz:=false
```

Enable map-driven maze autonomy (recommended for end-to-end maze runs):

```bash
ros2 launch x500_rtabmap_slam autonomy_full.launch.py \
  rviz:=false auto_takeoff:=true enable_waypoint_mission:=true
```

Then send one final goal in map frame (the mission node will handle intermediate subgoals):

```bash
ros2 topic pub --once /maze_final_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 11.1, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Useful mission tuning arguments:
- `mission_start_delay_sec` (default: `0.0`)
- `mission_final_goal_topic` (default: `/maze_final_goal`)
- `mission_final_goal_tolerance` (default: `0.60`)
- `mission_subgoal_lookahead_m` (default: `1.8`)
- `mission_inflation_radius_m` (default: `0.12`)
- `mission_unknown_is_blocked` (default: `true`)
- `drone_radius` (default: `0.25`)

## Topics

### Subscribed Topics

- `/oak_d_lite/rgb/image_raw` (sensor_msgs/Image) - RGB camera image
- `/oak_d_lite/rgb/camera_info` (sensor_msgs/CameraInfo) - RGB camera intrinsics
- `/oak_d_lite/depth/image_raw` (sensor_msgs/Image) - Depth image
- `/oak_d_lite/depth/camera_info` (sensor_msgs/CameraInfo) - Depth camera intrinsics
- `/oak_d_lite/depth/points` (sensor_msgs/PointCloud2) - Point cloud from depth camera
- `/tf` (tf2_msgs/TFMessage) - Transform tree

### Published Topics

- `/rtabmap/map` - 3D occupancy grid map
- `/rtabmap/mapData` - RTAB-Map internal map data
- `/rtabmap/grid_map` (nav_msgs/OccupancyGrid) - 2D occupancy grid for navigation
- `/rtabmap/odom` (nav_msgs/Odometry) - Visual odometry estimate
- `/rtabmap/odom_path` (nav_msgs/Path) - Odometry path
- `/rtabmap/cloud_map` (sensor_msgs/PointCloud2) - Assembled point cloud map
- `/rtabmap/localization_pose` (geometry_msgs/PoseWithCovarianceStamped) - Current localization pose

## Configuration

RTAB-Map is configured with the following key parameters:

- **Registration Strategy**: ICP (Iterative Closest Point) for point cloud alignment
- **Detection Rate**: 2 Hz for efficient processing
- **Grid Resolution**: 5cm cell size for detailed mapping
- **Max Range**: 5m maximum sensor range
- **Min Range**: 40cm minimum sensor range
- **Voxel Size**: 5cm for point cloud downsampling

## Database

RTAB-Map stores its map database at: `~/.ros/rtabmap.db`

To reset the map:
```bash
rm ~/.ros/rtabmap.db
```

## Visualization

The RViz configuration includes:

- RGB camera image display
- Live point cloud from depth camera
- Assembled map point cloud
- 2D occupancy grid
- Odometry path visualization
- Current pose with covariance
- TF frames visualization

## Requirements

This package requires the following ROS 2 packages:

- `rtabmap_ros` - RTAB-Map ROS 2 wrapper
- `rtabmap_slam` - RTAB-Map SLAM executable
- `rtabmap_odom` - RTAB-Map odometry
- `rtabmap_viz` - RTAB-Map visualization tools
- `rtabmap_util` - RTAB-Map utilities

Install with:
```bash
sudo apt install ros-jazzy-rtabmap-ros
```

## Troubleshooting

### No map being built

**Issue**: RTAB-Map is running but no map appears in RViz.

**Solutions**:
1. Check that camera topics are being published:
   ```bash
   ros2 topic hz /oak_d_lite/depth/points
   ros2 topic hz /oak_d_lite/rgb/image_raw
   ```

2. Verify TF tree is complete:
   ```bash
   ros2 run tf2_tools view_frames
   ```

3. Check RTAB-Map is receiving data:
   ```bash
   ros2 topic hz /rtabmap/cloud_map
   ```

### Database errors

**Issue**: RTAB-Map fails to start with database errors.

**Solution**: Delete the existing database:
```bash
rm ~/.ros/rtabmap.db
```

### Poor map quality

**Issue**: Map has many artifacts or is not accurate.

**Solutions**:
1. Ensure good lighting in simulation environment
2. Move drone slowly to allow better feature tracking
3. Adjust ICP parameters in launch file for better registration
4. Increase `Rtabmap/DetectionRate` for more frequent updates

### High CPU usage

**Issue**: RTAB-Map uses too much CPU.

**Solutions**:
1. Decrease `Rtabmap/DetectionRate` (e.g., from 2.0 to 1.0)
2. Increase `RGBD/LinearUpdate` and `RGBD/AngularUpdate` thresholds
3. Reduce `Icp/Iterations` (e.g., from 30 to 20)
4. Enable `Mem/MemoryThr` to limit memory usage

## References

- [RTAB-Map ROS Wiki](http://wiki.ros.org/rtabmap_ros)
- [RTAB-Map GitHub](https://github.com/introlab/rtabmap)
- [RTAB-Map Parameters](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)
