"""
Enhanced Realistic SLAM Simulation Launch File (Complete VIO Pipeline)

This launch file starts the complete ROS2 VIO pipeline with highly realistic synthetic SLAM:
1. Enhanced SLAM Simulator - generates motion-aware poses with realistic SLAM errors
2. ORB-SLAM3 Bridge - processes poses (placeholder for NN)
3. VIO Direct MAVLink Bridge - converts ENU→NED and sends MAVLink directly to PX4

Architecture:
Motion-Aware SLAM Simulator → ORB-SLAM3 Bridge → /orbslam3/pose_processed (ENU)
    ↓
VIO Direct Bridge → MAVLink VISION_POSITION_ESTIMATE → PX4 SITL

Enhanced SLAM Characteristics:
- Motion-coupled pose estimation (takeoff → cruise → landing)
- Realistic position noise (±2cm), scale drift (0.1%)
- Random tracking failures and recoveries (1-3 seconds)
- Bias drift accumulation over time
- Flight-phase aware behavior

IMPORTANT:
- Use camera-enabled PX4 model: make px4_sitl gz_x500_mono_cam (for consistency)
- Run with consistent ROS_DOMAIN_ID:
  Terminal 1 (PX4): export ROS_DOMAIN_ID=0
  Terminal 2 (ROS2): export ROS_DOMAIN_ID=1 && export ROS_LOCALHOST_ONLY=1
"""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set DDS transport to UDP (fixes SHM transport issues)
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', '/home/umarj/.ros/fastdds.xml'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHARED_MEMORY', '0'),

        # Gazebo-Style Camera + IMU Simulator (generates synchronized camera images and IMU data)
        Node(
            package='vio_bringup',
            executable='camera_simulator.py',
            name='gazebo_camera_imu_simulator',
            output='screen'
        ),

        # ORB-SLAM3 Bridge - receives poses and can add NN processing
        # Note: For real camera SLAM, this node receives poses from orbslam3_mono_node
        Node(
            package='vio_mavlink_bridge',
            executable='orb_slam3_bridge_node',
            name='orb_slam3_bridge',
            output='screen'
        ),

        # VIO Direct MAVLink Bridge - Direct MAVLink communication to PX4 (no MAVROS needed)
        Node(
            package='vio_mavlink_bridge',
            executable='vio_mavlink_bridge_direct_node',
            name='vio_mavlink_bridge_direct',
            output='screen',
            parameters=[{
                'pose_topic': '/orbslam3/pose_processed',
                'mavlink_port': 14580,  # PX4 SITL onboard MAVLink port
                'mavlink_host': '127.0.0.1',
                'system_id': 1,
                'component_id': 1  # Camera component
            }]
        )

        # NOTE: Using direct MAVLink bridge instead of MAVROS to avoid conflicts.
        # MAVROS can be run separately in another terminal if needed for other functionality.
    ])
