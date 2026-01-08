# Autonomous-Drone

A ROS 2-based autonomous drone system with OAK-D camera integration, ORB-SLAM3 visual odometry, and Gazebo simulation.

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
- **ORB-SLAM3**: Real-time visual SLAM
- **PX4 Integration**: MAVLink-based flight control

## 📁 Project Structure

```
Autonomous-Drone/
├── ros2_ws/                    # ROS 2 workspace
│   ├── src/
│   │   ├── depthai_cam/        # OAK-D camera ROS 2 driver
│   │   ├── orbslam3_bridge/    # ORB-SLAM3 ROS 2 bridge
│   │   ├── vio_perception/     # VIO perception pipeline
│   │   ├── vio_mavlink_bridge/ # MAVLink VIO bridge
│   │   └── vio_bringup/        # Launch configurations
│   └── external/               # External dependencies (ORB-SLAM3)
├── scripts/                    # Setup and utility scripts
├── config/                     # Configuration files
├── doc/                        # Documentation
└── SIMULATION_SETUP.md         # Simulation guide
```

## 🎯 Usage

### Launch Simulation
```bash
# Terminal 1: Launch Gazebo simulation with OAK-D camera
ros2 launch depthai_cam depthai_cam_sim.launch.py

# Terminal 2: Launch ORB-SLAM3 with camera feed
ros2 launch vio_bringup vio_oakd_orbslam3.launch.py

# Terminal 3: Launch MAVROS SITL integration
ros2 launch vio_bringup vio_sitl_mavros.launch.py
```

### Real Hardware
```bash
# Connect OAK-D camera and launch real-time VIO
ros2 launch vio_bringup vio_oakd_orbslam3.launch.py
```

## 🔧 Development

### Building
```bash
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

### ROS 2 Shell Completion Warnings

**Problem**: "ModuleNotFoundError: No module named 'argcomplete.scripts'" on terminal startup

**Solution**: Remove the stale script:
```bash
rm ~/.local/bin/register-python-argcomplete
```

## 📚 Documentation

- [Simulation Setup Guide](SIMULATION_SETUP.md)
- [Camera Configuration](config/oakd_mono.yaml)
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
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)