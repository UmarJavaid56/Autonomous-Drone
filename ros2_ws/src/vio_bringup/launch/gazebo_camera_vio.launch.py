"""
Gazebo Camera VIO Launch File (Real Camera Data from Simulation)

This launch file uses real camera data from Gazebo simulation instead of synthetic data:
1. Gazebo camera plugins publish /camera/image_raw and /camera/depth/image_raw
2. ORB-SLAM3 processes the real camera images
3. VIO Direct MAVLink Bridge sends poses to PX4

Prerequisites:
- Gazebo must be running with x500_camera_depth model
- PX4 SITL must be connected to Gazebo
- Run with: ros2 launch vio_bringup gazebo_camera_vio.launch.py

Architecture:
Gazebo Camera → ORB-SLAM3 → /orbslam3/pose_processed (ENU)
    ↓
VIO Direct Bridge → MAVLink VISION_POSITION_ESTIMATE → PX4 SITL
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

        # GZ-ROS Bridge for real Gazebo camera and IMU data
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=[
                # DISABLED: Gazebo IMU causes timestamp errors - PX4 will use simulated IMU
                # '/world/default/model/x500_depth/link/base_link/sensor/imu_sensor/imu@gz.msgs.IMU@sensor_msgs/msg/Imu@/imu/data_raw',
                # RGB camera image from Gazebo to ROS
                '/world/default/model/x500_depth/link/camera_link/sensor/IMX214/image@gz.msgs.Image@sensor_msgs/msg/Image@/camera/image_raw',
                # Depth camera image from Gazebo to ROS
                '/world/default/model/x500_depth/link/camera_link/sensor/StereoOV7251/depth_camera@gz.msgs.Image@sensor_msgs/msg/Image@/camera/depth/image_raw'
            ],
            parameters=[{'use_sim_time': True}]
        ),

        # ORB-SLAM3 Mono Node - processes real camera images from Gazebo
        Node(
            package='orbslam3_bridge',
            executable='orbslam3_mono_node',
            name='orbslam3_mono',
            output='screen',
            parameters=[{
                'image_topic': '/world/default/model/x500_depth/link/camera_link/sensor/IMX214/image',
                'voc_file': '/home/umarj/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                'settings_file': '/home/umarj/Capstone/Autonomous-Drone/config/oakd_mono.yaml'
            }]
        ),

        # ORB-SLAM3 Bridge - receives poses and can add NN processing
        Node(
            package='vio_mavlink_bridge',
            executable='orb_slam3_bridge_node',
            name='orb_slam3_bridge',
            output='screen'
        ),

        # VIO Direct MAVLink Bridge - Direct MAVLink communication to PX4
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
        ),
    ])
