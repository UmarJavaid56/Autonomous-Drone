# Autonomous-Drone

A ROS 2-based autonomous drone system with OAK-D camera integration, RTAB-Map SLAM (3D mapping), and Gazebo simulation.

## 🚀 Quick Start

### Prerequisites
- Ubuntu 24.04 LTS
- At least 8GB RAM, 4GB free disk space
- Internet connection for downloads

### One-Command Setup
```bash
git clone https://github.com/UmarJavaid56/Autonomous-Drone.git
cd Autonomous-Drone
./scripts/setup_all.sh
```

This will:
- Install ROS 2 Jazzy
- Install Gazebo Harmonic
- Install all dependencies
- Create a Python virtual environment (`.venv/` in project root)
- Build the workspace

### Manual Setup (Alternative)

If you prefer step-by-step control:

```bash
# 1. Install system dependencies
./scripts/install_dependencies.sh

# 2. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 3. Setup and build workspace
./scripts/setup_workspace.sh
```

## 🏗️ Architecture

- **ROS 2 Jazzy**: Latest LTS robotics framework
- **Gazebo Harmonic**: Modern simulation with Ogre2 rendering
- **OAK-D Camera**: DepthAI-powered stereo vision
- **RTAB-Map**: RGB-D SLAM for 3D mapping with point clouds and visual odometry
- **PX4 Integration**: MAVLink-based flight control

## 📁 Project Structure

```
Autonomous-Drone/
├── ros2_ws/                    # ROS 2 workspace
│   ├── src/
│   │   ├── depthai_cam/        # OAK-D camera ROS 2 driver
│   │   ├── x500_rtabmap_slam/   # RTAB-Map SLAM + autonomy launch
│   │   ├── vio_perception/     # VIO perception pipeline
│   │   ├── vio_mavlink_bridge/ # MAVLink VIO bridge
│   │   └── vio_bringup/        # Launch configurations
│   └── ...
├── scripts/                    # Setup and utility scripts
├── config/                     # Configuration files
├── doc/                        # Documentation
└── SIMULATION_SETUP.md         # Simulation guide
```

## 🎯 Usage

### Launch Simulation
```bash
# Terminal 1: Launch Gazebo simulation with OAK-D camera
# Includes ROS 2 - Gazebo bridges for camera topics
ros2 launch depthai_cam depthai_cam_sim.launch.py

# Terminal 2: Launch MAVROS SITL (subscribes to vision pose, e.g. from RTAB-Map adapter)
ros2 launch vio_bringup vio_sitl_mavros.launch.py
```

**Note**: The simulation uses `ros_gz_bridge` to bridge Gazebo camera topics to ROS 2 topics. Topics published:
- `/oak_d_lite/rgb/image_raw` - RGB camera
- `/oak_d_lite/depth/image_raw` - Depth image
- `/oak_d_lite/depth/points` - Depth point cloud
- `/tf` - Transform frames

### RTAB-Map SLAM (3D Mapping)
```bash
# Launch x500_depth simulation with RTAB-Map SLAM
# This includes Gazebo simulation, RTAB-Map, and RViz visualization
ros2 launch x500_rtabmap_slam x500_rtabmap_slam.launch.py

# For localization mode (using existing map)
ros2 launch x500_rtabmap_slam x500_rtabmap_slam.launch.py localization:=true
```

RTAB-Map creates 3D point cloud maps and provides:
- Real-time 3D mapping using depth camera point clouds
- Visual odometry and pose estimation
- Loop closure detection for accurate mapping
- 2D occupancy grid for navigation planning

**Note**: RTAB-Map and its ROS 2 packages are installed as part of the main dependency script:
```bash
./scripts/install_dependencies.sh
```

See [x500_rtabmap_slam/README.md](ros2_ws/src/x500_rtabmap_slam/README.md) for detailed usage and [RTAB-Map Quick Start](doc/RTABMAP_QUICKSTART.md) for step-by-step guide.

### Real Hardware
```bash
# Connect OAK-D and run RTAB-Map SLAM; use vio_sitl_mavros with pose_topic pointing to your vision pose
ros2 launch x500_rtabmap_slam x500_rtabmap_slam.launch.py
ros2 launch vio_bringup vio_sitl_mavros.launch.py pose_topic:=/vision_pose/pose
```

## 🔧 Development

### Python Virtual Environment

This project uses a Python virtual environment to ensure compatibility across different machines and avoid system Python conflicts. The venv is created in the project root (`.venv/`) and is portable - it works when the repo is cloned to any machine.

**Setup venv manually:**
```bash
./scripts/setup_venv.sh
```

**Activate venv and workspace:**
```bash
# Option 1: Use the workspace setup script (recommended)
source ros2_ws/setup.sh

# Option 2: Manual activation
source .venv/bin/activate
source ros2_ws/install/setup.bash
```

The `ros2_ws/setup.sh` script automatically activates the venv and sources the workspace, making it easy to get started.

### Building
```bash
# The setup_workspace.sh script automatically uses the venv
./scripts/setup_workspace.sh

# Or manually with venv activated
source .venv/bin/activate
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Testing
```bash
# Run all tests
colcon test
colcon test-result --verbose
```

## ⚠️ Troubleshooting

### Missing ros_gz_bridge Package

**Problem**: Launch fails with "package 'ros_gz_bridge' not found"

**Cause**: `ros_gz_bridge` is required to bridge Gazebo camera topics to ROS 2 but wasn't installed

**Solution**: Install the bridge package and dependencies
```bash
sudo apt install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-actuator-msgs \
    ros-jazzy-gps-msgs \
    ros-jazzy-vision-msgs
```

Or run the updated installation script:
```bash
./scripts/install_dependencies.sh
```

### DART Library Conflict during Gazebo Installation

**Problem**: Installation fails with "trying to overwrite libdart-*.so.6.13.2" error

**Cause**: Ubuntu 24.04 provides conflicting DART physics library packages that conflict with OSRF's versions.

**Solution**: The updated installation scripts (`install_dependencies.sh`) automatically resolve this conflict by:
1. Removing conflicting Ubuntu DART packages
2. Installing OSRF's DART libraries
3. Cleaning up stray dependencies

If you encounter this issue after running the old scripts:
```bash
# Force remove conflicting packages
sudo dpkg --remove --force-all \
  libdart6.13:amd64 libdart-external-odelcpsolver6.13:amd64 \
  libdart-collision-bullet6.13:amd64 libdart-collision-ode6.13:amd64

# Install OSRF DART
sudo apt install -y libdart-core+collisions+odelcpsolver6.13

# Clean up
sudo dpkg --configure -a
sudo apt autoremove -y
```

### Gazebo `gz sim` Command Not Recognized

**Problem**: Running `gz sim` fails with "no such command" error

**Cause**: The `gz` command-line tool requires the `GZ_CONFIG_PATH` environment variable to locate plugin configurations.

**Solution**: Set the environment variable:
```bash
export GZ_CONFIG_PATH=/usr/share/gz
gz sim --version  # should now work
```

**Note**: This is automatically configured in ROS 2 launch files via `depthai_cam_sim.launch.py`, so you don't need to set it manually when launching through ROS 2.

### Python Setuptools Compatibility Issues

**Problem**: Build fails with "AttributeError: module 'pkgutil' has no attribute 'ImpImporter'"

**Cause**: Python 3.12 removed `pkgutil.ImpImporter`, but older setuptools versions still reference it.

**Solution**: The setup scripts automatically create a Python virtual environment with updated setuptools. If you encounter this issue:

1. Ensure the venv is created and activated:
   ```bash
   ./scripts/setup_venv.sh
   source .venv/bin/activate
   ```

2. Rebuild the workspace:
   ```bash
   ./scripts/setup_workspace.sh
   ```

The venv ensures compatibility across different Python versions and system configurations.

### ROS 2 Shell Completion Warnings

**Problem**: "ModuleNotFoundError: No module named 'argcomplete.scripts'" on terminal startup

**Solution**: Remove the stale script:
```bash
rm ~/.local/bin/register-python-argcomplete
```

## 📚 Documentation

- [Simulation Setup Guide](SIMULATION_SETUP.md)
- [Camera Configuration](config/oakd_mono.yaml)
- [RTAB-Map Quick Start](doc/RTABMAP_QUICKSTART.md)
- [RTAB-Map Integration Guide](doc/RTABMAP_INTEGRATION.md)
- [API Documentation](doc/)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests
5. Submit a pull request

## 📄 License

This project is licensed under the BSD-3-Clause License - see the package.xml files for details.

## 🙏 Acknowledgments

- [ROS 2](https://docs.ros.org/en/jazzy/)
- [Gazebo Sim](https://gazebosim.org/docs/harmonic/)
- [DepthAI](https://docs.luxonis.com/)
- [RTAB-Map](https://github.com/introlab/rtabmap)