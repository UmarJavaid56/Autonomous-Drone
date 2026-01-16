# depthai_cam

ROS 2 package for OAK-D Lite camera integration and Gazebo simulation of x500 drone with depth camera.

## Overview

This package provides:
- **Gazebo Simulation**: x500 quadcopter model with integrated OAK-D Lite depth camera
- **Hardware Integration**: ROS 2 node for real OAK-D Lite camera streaming
- **Keyboard Teleop**: Manual velocity control for the simulated drone
- **Visualization**: Pre-configured RViz2 setup for camera streams and robot state

## Package Structure

```
depthai_cam/
├── depthai_cam/          # Python nodes
│   ├── keyboard_teleop.py    # Keyboard control for drone
│   ├── oak_publisher.py      # Real hardware camera publisher
│   └── pose_to_tf.py         # Gazebo pose to TF converter (deprecated)
├── launch/               # Launch files
│   └── depthai_cam_sim.launch.py  # Main simulation launch
├── models/              # Gazebo models
│   ├── OakD-Lite/       # OAK-D Lite camera model
│   ├── x500/            # Base x500 drone model
│   ├── x500_base/       # x500 airframe components
│   └── x500_depth/      # x500 with OAK-D Lite
├── worlds/              # Gazebo worlds
│   └── depthai_test.world    # Test world with colored objects
└── config/              # Configuration files
    └── x500_depth.rviz       # RViz visualization config
```

## Usage

### Simulation

Launch the complete simulation environment:

```bash
ros2 launch depthai_cam depthai_cam_sim.launch.py
```

This will start:
- Gazebo Harmonic with the x500_depth drone
- RViz2 with camera visualization
- Topic bridges for camera streams and control

### Keyboard Control

In a new terminal, launch the keyboard teleop:

```bash
ros2 run depthai_cam keyboard_teleop
```

**Controls:**
- `W/S`: Forward/Backward
- `A/D`: Strafe Left/Right  
- `Q/E`: Rotate CCW/CW (yaw)
- `R/F`: Up/Down (altitude)
- `Space`: Stop all motion
- `T`: Toggle controller enable/disable
- `I/K`: Increase/Decrease linear speed
- `O/L`: Increase/Decrease angular speed
- `Esc`: Exit

### Camera Topics

The simulation publishes the following topics:

**RGB Camera:**
- `/oak_d_lite/rgb/image_raw` - RGB image stream
- `/oak_d_lite/rgb/camera_info` - Camera calibration

**Depth Camera:**
- `/oak_d_lite/depth/image_raw` - Depth image
- `/oak_d_lite/depth/camera_info` - Depth camera calibration
- `/oak_d_lite/depth/points` - Point cloud

**Control:**
- `/x500_depth/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/x500_depth/enable` - Enable velocity controller (std_msgs/Bool)

## Hardware Integration

To use with real OAK-D Lite hardware:

```bash
ros2 run depthai_cam oak_publisher
```

**Parameters:**
- `width`: Image width (default: 640)
- `height`: Image height (default: 480)
- `fps`: Frame rate (default: 30)
- `topic`: Output topic name (default: camera/image_raw)

## Dependencies

### ROS 2 Packages
- `rclpy` - Python ROS 2 client library
- `sensor_msgs` - Sensor message types
- `geometry_msgs` - Geometry message types
- `std_msgs` - Standard message types
- `tf2_ros` - TF2 transform library
- `cv_bridge` - OpenCV-ROS bridge
- `ros_gz_bridge` - Gazebo-ROS bridge
- `robot_state_publisher` - Robot model TF publisher
- `rviz2` - 3D visualization

### External Dependencies
- Gazebo Harmonic (gz-sim8)
- DepthAI SDK (for hardware integration)
- OpenCV

## License


## Authors


## Contributing

This package is part of the Autonomous-Drone project. For issues and contributions,
please refer to the main project repository.
