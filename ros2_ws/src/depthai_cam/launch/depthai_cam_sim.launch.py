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

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': open(x500_depth_model_path, 'r').read()},
        ]
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

    launch_items = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        ign_gazebo,
        spawn_x500_depth,
        rviz2,
        joint_state_publisher,
        robot_state_publisher,
    ]
    
    return LaunchDescription(launch_items)
