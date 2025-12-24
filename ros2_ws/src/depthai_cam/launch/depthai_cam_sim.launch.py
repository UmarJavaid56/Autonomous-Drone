#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('depthai_cam')
    world_path = os.path.join(pkg_share, 'worlds', 'depthai_test.world')
    camera_model_path = os.path.join(pkg_share, 'models', 'oak_rgb_camera.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'oak_rgb_camera', '-file', camera_model_path, '-x', '0', '-y', '0', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_camera,
    ])
