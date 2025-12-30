from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    voc_file = LaunchConfiguration('voc_file')
    settings_file = LaunchConfiguration('settings_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'voc_file',
            default_value='/home/umarj/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            description='Path to ORB-SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'settings_file',
            default_value='/home/umarj/Capstone/Autonomous-Drone/config/oakd_mono.yaml',
            description='Path to ORB-SLAM3 camera settings file'
        ),

        Node(
            package='depthai_cam',
            executable='oak_publisher',  
            name='oakd_mono',
            output='screen'
        ),

        Node(
            package='orbslam3_bridge',
            executable='orbslam3_mono_node',
            name='orbslam3_mono',
            output='screen',
            parameters=[
                {
                    'voc_file': voc_file,
                    'settings_file': settings_file
                }
            ]
        ),
    ])

