"""
Camera Simulation Launch File (Complete VIO Pipeline for PX4 SITL)

This launch file starts the complete ROS2 VIO pipeline optimized for SITL:
1. Mock Pose Publisher - generates circular trajectory
2. ORB-SLAM3 Bridge - processes poses (placeholder for NN)
3. VIO MAVLink Direct Bridge - ENU→NED + Direct MAVLink to PX4 SITL

Architecture (SITL Optimized):
Mock → ORB-SLAM3 Bridge → /orbslam3/pose_processed (ENU)
    ↓
VIO Direct Bridge → MAVLink VISION_POSITION_ESTIMATE → PX4 SITL

Direct MAVLink communication (no ROS2 middleware conflicts)
Reliable for SITL testing (MAVROS has parameter service conflicts)
Clean single-launch design
PX4 receives VISION_POSITION_ESTIMATE messages directly
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Mock pose publisher (simulating SLAM output from camera)
        Node(
            package='vio_mavlink_bridge',
            executable='mock_pose_publisher.py',
            name='mock_pose_publisher',
            output='screen'
        ),

        # ORB-SLAM3 Bridge - receives poses and can add NN processing
        Node(
            package='vio_mavlink_bridge',
            executable='orb_slam3_bridge_node',
            name='orb_slam3_bridge',
            output='screen'
        ),

        # VIO MAVLink Direct Bridge - ENU to NED + Direct MAVLink for PX4 SITL
        # This is BETTER than MAVROS for SITL testing (no conflicts, direct communication)
        Node(
            package='vio_mavlink_bridge',
            executable='vio_mavlink_bridge_direct_node',
            name='vio_mavlink_direct_bridge',
            output='screen',
            parameters=[{
                'pose_topic': '/orbslam3/pose_processed',
                'mavlink_port': 14580,
                'mavlink_host': '127.0.0.1',
                'system_id': 1,
                'component_id': 0  # MAV_COMP_ID_ALL or use camera component
            }]
        ),

    ])
