"""
Camera Visualization Launch File

This launch file starts the camera simulator and RViz for visualizing
both RGB and depth camera data from the simulated drone camera.

Uses TCP transport instead of shared memory to avoid DDS transport issues.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set DDS transport to UDP (fixes SHM transport issues)
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', '/home/umarj/.ros/fastdds.xml'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHARED_MEMORY', '0'),

        # Camera simulator (generates RGB + Depth + IMU data)
        Node(
            package='vio_bringup',
            executable='camera_simulator.py',
            name='gazebo_camera_simulator',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/umarj/Capstone/Autonomous-Drone/config/camera_visualization.rviz'],
            parameters=[{'use_sim_time': False}]
        ),
    ])
