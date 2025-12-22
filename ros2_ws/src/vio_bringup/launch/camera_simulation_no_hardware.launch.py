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

        # VIO MAVLink Bridge - ENU to NED conversion + MAVLink
        Node(
            package='vio_mavlink_bridge',
            executable='vio_mavlink_bridge_node',
            name='vio_mavlink_bridge',
            output='screen',
            parameters=[{
                'pose_topic': '/orbslam3/pose_processed'  
            }]
        ),

        # MAVROS Node for SITL - Run separately to avoid conflicts
        # Start with: ros2 run mavros mavros_node --ros-args -r __node:=mavros_vio -p fcu_url:=udp://:14540@localhost:14541
        # Only after PX4 SITL is running
    ])
