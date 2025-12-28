#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('depthai_cam')
    world_path = os.path.join(pkg_share, 'worlds', 'depthai_test.world')
    x500_depth_model_path = os.path.join(pkg_share, 'models', 'x500_depth', 'model.sdf')

    # Ensure Fortress can find default models (ground_plane, sun), our models, and PX4 models
    ign_resource_path = os.pathsep.join([
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        '/usr/share/ignition/gazebo-6',
        '/usr/share/ignition/fuel_tools',
        os.path.join(pkg_share, 'models'),  # Our local models (x500, x500_base, OakD-Lite, etc.)
    ])

    # Launch Ignition Gazebo Fortress
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-v', '3', '--render-engine', 'ogre'],
        output='screen'
    )

    # Spawn X500 drone with depth camera using Ignition service (delay to allow world to load)
    spawn_x500_depth = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ign', 'service', '-s', '/world/depthai_test_world/create',
                    '--reqtype', 'ignition.msgs.EntityFactory',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '10000',
                    '--req', f'sdf_filename: "{x500_depth_model_path}"',
                ],
                output='screen'
            )
        ]
    )

    # OakD-Lite depth camera topics from X500 model
    depth_image_topic = '/world/depthai_test_world/model/x500_depth/link/camera_link/sensor/StereoOV7251/image'
    depth_info_topic = '/world/depthai_test_world/model/x500_depth/link/camera_link/sensor/StereoOV7251/camera_info'
    depth_points_topic = '/world/depthai_test_world/model/x500_depth/link/camera_link/sensor/StereoOV7251/points'
    rgb_image_topic = '/world/depthai_test_world/model/x500_depth/link/camera_link/sensor/IMX214/image'
    rgb_info_topic = '/world/depthai_test_world/model/x500_depth/link/camera_link/sensor/IMX214/camera_info'

    bridge_args = [
        # Depth camera bridge
        f'{depth_image_topic}@sensor_msgs/msg/Image@ignition.msgs.Image',
        f'{depth_info_topic}@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        f'{depth_points_topic}@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        # RGB camera bridge
        f'{rgb_image_topic}@sensor_msgs/msg/Image@ignition.msgs.Image',
        f'{rgb_info_topic}@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        '--ros-args',
        # Remap depth topics to match OAK-D naming
        '-r', f'{depth_image_topic}:=/oak_d_lite/depth/image_raw',
        '-r', f'{depth_info_topic}:=/oak_d_lite/depth/camera_info',
        '-r', f'{depth_points_topic}:=/oak_d_lite/depth/points',
        # Remap RGB topics
        '-r', f'{rgb_image_topic}:=/oak_d_lite/rgb/image_raw',
        '-r', f'{rgb_info_topic}:=/oak_d_lite/rgb/camera_info',
    ]

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='oak_s2_bridge',
        output='screen',
        arguments=bridge_args,
    )

    # Launch RViz2 with configuration
    rviz_config_path = os.path.join(pkg_share, 'config', 'x500_depth.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        ign_gazebo,
        spawn_x500_depth,
        ros_gz_bridge,
        rviz2,
    ])
