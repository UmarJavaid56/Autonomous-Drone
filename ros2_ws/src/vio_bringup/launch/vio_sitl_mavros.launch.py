from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    pose_topic = LaunchConfiguration('pose_topic')
    fcu_url = LaunchConfiguration('fcu_url')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/orbslam3/pose',
            description='Topic to subscribe to for pose data'
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://:14540@localhost:14541',
            description='MAVROS connection URL for PX4 SITL'
        ),

        # VIO MAVLink Bridge Node
        Node(
            package='vio_mavlink_bridge',
            executable='vio_mavlink_bridge_node',
            name='vio_mavlink_bridge',
            output='screen',
            parameters=[{
                'pose_topic': pose_topic,
            }]
        ),

        # MAVROS Node for SITL
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                'system_id': 1,
                'component_id': 240,
                'plugin_blacklist': [],  # Enable all plugins including vision_pose_estimate
            }]
        ),
    ])

