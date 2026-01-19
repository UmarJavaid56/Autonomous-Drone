# RTAB-Map SLAM Quick Start Guide

This guide will get you started with RTAB-Map SLAM on the x500_depth drone in just a few steps.

## Prerequisites

1. Complete the main setup:
```bash
cd Autonomous-Drone
./scripts/setup_all.sh
```

2. Install RTAB-Map dependencies:
```bash
./scripts/install_rtabmap.sh
```

## Quick Start

### Step 1: Source ROS 2 Environment
```bash
source /opt/ros/jazzy/setup.bash
cd ~/Autonomous-Drone/ros2_ws
source install/setup.bash
```

### Step 2: Launch SLAM System

**Option A: All-in-One Launch (Recommended for Beginners)**
```bash
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py
```

This single command will:
- Start Gazebo simulation with x500_depth drone
- Launch RTAB-Map SLAM
- Open RViz2 for visualization

**Option B: Separate Components (Advanced)**

Terminal 1 - Simulation:
```bash
ros2 launch depthai_cam depthai_cam_sim.launch.py
```

Terminal 2 - RTAB-Map:
```bash
ros2 launch rtabmap_slam rtabmap_slam.launch.py
```

### Step 3: Control the Drone

In a new terminal:
```bash
# Source environment
source /opt/ros/jazzy/setup.bash
cd ~/Autonomous-Drone/ros2_ws
source install/setup.bash

# Run keyboard teleop
ros2 run depthai_cam keyboard_teleop
```

### Step 4: Observe the Map Building

In RViz2, you should see:
- **RGB Image**: Live camera feed (bottom panel)
- **RGB PointCloud**: Current point cloud from depth camera
- **Map Cloud**: Accumulated 3D map (colored points)
- **Occupancy Grid**: 2D navigation map (gray/black/white grid)
- **Odometry Path**: Green path showing drone trajectory
- **Current Pose**: Red arrow showing drone position

### Step 5: Move the Drone to Build Map

Use keyboard teleop to move the drone:
- `w` - Move forward
- `s` - Move backward
- `a` - Move left
- `d` - Move right
- `q` - Rotate left
- `e` - Rotate right
- `r` - Move up
- `f` - Move down
- `Space` - Enable/disable motors
- `Esc` - Exit

**Tips for Good Maps:**
1. Move slowly to allow feature tracking
2. Avoid pure rotation (combine rotation with translation)
3. Ensure good overlap between consecutive views
4. Revisit areas to trigger loop closures

## What to Expect

### Initial Startup (~30 seconds)
- Gazebo simulation loads
- RTAB-Map initializes
- RViz2 opens with configuration

### During Mapping
- Point clouds appear in RViz2
- Map builds incrementally as drone moves
- Green path shows drone trajectory
- Occupancy grid updates in real-time

### Performance
- RTAB-Map processes at ~2 Hz
- Map updates when drone moves > 5cm or rotates > 3°
- CPU usage: Moderate (one core at ~60-80%)

## Checking Map Quality

### Monitor RTAB-Map Status
```bash
# In a new terminal
ros2 topic echo /rtabmap/info
```

Look for:
- `Loop/Accepted: X` - Number of successful loop closures
- `Odometry/Features: X` - Number of tracked features
- `Memory/Working_memory_size: X` - Number of nodes in map

### Visualize TF Tree
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

Should show: `map` → `odom` → `base_link` → `camera_link`

## Saving and Loading Maps

### Map Location
Maps are automatically saved to: `~/.ros/rtabmap.db`

### Reset Map (Start Fresh)
```bash
rm ~/.ros/rtabmap.db
```

### Use Existing Map (Localization Mode)
```bash
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py localization:=true
```

## Troubleshooting

### "No map appearing in RViz"
1. Check camera topics:
   ```bash
   ros2 topic hz /oak_d_lite/depth/points
   ```
   Should show ~30 Hz

2. Check RTAB-Map output:
   ```bash
   ros2 topic hz /rtabmap/cloud_map
   ```
   Should show ~2 Hz

3. Move the drone - map only updates when moving

### "CPU usage too high"
Edit launch file and decrease `Rtabmap/DetectionRate` from 2.0 to 1.0

### "Map has many artifacts"
1. Move drone more slowly
2. Ensure good lighting in environment
3. Check camera is focused properly

### "Database errors on startup"
```bash
rm ~/.ros/rtabmap.db
```

## Next Steps

1. **Read the documentation:**
   - [RTAB-Map Package README](../ros2_ws/src/rtabmap_slam/README.md)
   - [Integration Guide](../doc/RTABMAP_INTEGRATION.md)

2. **Tune parameters:**
   - See `ros2_ws/src/rtabmap_slam/config/rtabmap_params.yaml`
   - Adjust for your specific use case

3. **Integrate with navigation:**
   - Use `/rtabmap/grid_map` for Nav2
   - Use `/rtabmap/odom` for localization

4. **Export maps:**
   ```bash
   # Export point cloud
   ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap
   
   # Save to PCD file
   ros2 run pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map
   ```

## Getting Help

- **GitHub Issues**: https://github.com/UmarJavaid56/Autonomous-Drone/issues
- **RTAB-Map Forum**: https://github.com/introlab/rtabmap/discussions
- **ROS Answers**: https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:rtabmap/

## Summary

You now have a working RTAB-Map SLAM system! The key points:

✅ Single command launch: `ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py`  
✅ Keyboard control: `ros2 run depthai_cam keyboard_teleop`  
✅ Map location: `~/.ros/rtabmap.db`  
✅ Visualization: RViz2 (auto-launched)  

Happy mapping! 🚁🗺️
