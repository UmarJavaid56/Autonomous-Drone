from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://:14540@localhost:14541',
            description='MAVROS connection URL for PX4 SITL'
        ),

        # Camera publisher (OAK-D) - feeds images to external ORB-SLAM3
        Node(
            package='depthai_cam',
            executable='oak_publisher',
            name='oakd_camera',
            output='screen'
        ),

        # ORB-SLAM3 Bridge - receives poses from external ORB-SLAM3 process
        # Can add NN processing, validation, and multi-source fusion here
        # Note: ORB-SLAM3 runs as separate process outside of ROS2
        Node(
            package='vio_mavlink_bridge',
            executable='orb_slam3_bridge_node',
            name='orb_slam3_bridge',
            output='screen'
        ),

        # VIO MAVLink Bridge - ENU to NED conversion + MAVLink
        Node(
            package='vio_mavlink_bridge',
            executable='vio_mavlink_bridge_node',
            name='vio_mavlink_bridge',
            output='screen',
            parameters=[{
                'pose_topic': '/orbslam3/pose_processed'  # Subscribe to processed poses
            }]
        ),

        # MAVROS Node for SITL
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'system_id': 1,
                'component_id': 240,
                'plugin_blacklist': '',
            }]
        ),
    ])
