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

        # Camera publisher (OAK-D)
        Node(
            package='depthai_cam',
            executable='oak_publisher',
            name='oakd_camera',
            output='screen'
        ),

        # Mock pose publisher (simulating SLAM output)
        Node(
            package='vio_mavlink_bridge',
            executable='mock_pose_publisher.py',
            name='mock_pose_publisher',
            output='screen'
        ),

        # VIO MAVLink Bridge Node
        Node(
            package='vio_mavlink_bridge',
            executable='vio_mavlink_bridge_node',
            name='vio_mavlink_bridge',
            output='screen',
            parameters=[{
                'pose_topic': '/orbslam3/pose'
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
