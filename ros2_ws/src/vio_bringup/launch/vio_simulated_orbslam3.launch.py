"""
VIO System with Simulated Camera + Real ORB-SLAM3 Processing

This launch file runs the complete VIO pipeline with:
1. Camera Simulator (RGB + Depth images)
2. Real ORB-SLAM3 processing the simulated images
3. ORB-SLAM3 Bridge for pose processing
4. VIO MAVLink Bridge to PX4

This lets you test real SLAM algorithms with synthetic camera data!
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set DDS transport to UDP (fixes SHM transport issues)
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', '/home/umarj/.ros/fastdds.xml'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHARED_MEMORY', '0'),

        # Simulated Camera + IMU (replaces hardware camera)
        Node(
            package='vio_bringup',
            executable='camera_simulator.py',
            name='camera_simulator',
            output='screen'
        ),

        # ORB-SLAM3 Mono Node - processes simulated camera images
        # Note: This assumes ORB-SLAM3 is built and configured
        Node(
            package='orbslam3_bridge',
            executable='orbslam3_mono_node',
            name='orbslam3_mono',
            output='screen',
            parameters=[
                {
                    'voc_file': '/home/umarj/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                    'settings_file': '/home/umarj/Capstone/Autonomous-Drone/config/oakd_mono.yaml'
                }
            ]
        ),

        # ORB-SLAM3 Bridge - receives poses from ORB-SLAM3 and can add processing
        Node(
            package='vio_mavlink_bridge',
            executable='orb_slam3_bridge_node',
            name='orb_slam3_bridge',
            output='screen'
        ),

        # VIO Direct MAVLink Bridge - sends poses to PX4 SITL
        Node(
            package='vio_mavlink_bridge',
            executable='vio_mavlink_bridge_direct_node',
            name='vio_mavlink_bridge_direct',
            output='screen',
            parameters=[{
                'pose_topic': '/orbslam3/pose_processed',
                'mavlink_port': 14580,
                'mavlink_host': '127.0.0.1',
                'system_id': 1,
                'component_id': 1
            }]
        )
    ])
