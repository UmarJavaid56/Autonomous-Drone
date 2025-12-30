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

    # Launch RViz2 with configuration
    rviz_config_path = os.path.join(pkg_share, 'config', 'x500_depth.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Static TF publisher for base transforms
    static_tf_publisher = Node(
        package='depthai_cam',
        executable='static_tf_publisher',
        name='static_tf_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Bridges for TF and OakD-Lite camera topics (RGB + Depth + CameraInfo + PointCloud)
    # Ignition topic roots
    world_name = 'depthai_test_world'
    model_name = 'x500_depth'
    link_name = 'camera_link'

    ignition_root = f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'

    bridge_args = [
        # Bridge simulation clock so ROS nodes can use sim time
        f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        # TF frames (Pose_V -> TFMessage) using full world-scoped topics
        f'/world/{world_name}/model/world_frame/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        f'/world/{world_name}/model/odom_frame/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        f'/world/{world_name}/model/{model_name}/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',

        # RGB camera (IMX214)
        f'{ignition_root}/IMX214/image@sensor_msgs/msg/Image@ignition.msgs.Image',
        f'{ignition_root}/IMX214/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',

        # Depth camera (StereoOV7251)
        f'{ignition_root}/StereoOV7251/image@sensor_msgs/msg/Image@ignition.msgs.Image',
        f'{ignition_root}/StereoOV7251/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        f'{ignition_root}/StereoOV7251/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        # Some PX4 OakD-Lite models set a custom sensor <topic>; bridge it too if present
        f'{ignition_root}/StereoOV7251/depth_camera@sensor_msgs/msg/Image@ignition.msgs.Image',

        '--ros-args',
        # Remap TF frames into /tf
        '-r', f'/world/{world_name}/model/world_frame/pose:=/tf',
        '-r', f'/world/{world_name}/model/odom_frame/pose:=/tf',
        '-r', f'/world/{world_name}/model/{model_name}/pose:=/tf',

        # Remap RGB topics to stable ROS names
        '-r', f'{ignition_root}/IMX214/image:=/oak_d_lite/rgb/image_raw',
        '-r', f'{ignition_root}/IMX214/camera_info:=/oak_d_lite/rgb/camera_info',

        # Remap Depth topics to stable ROS names
        '-r', f'{ignition_root}/StereoOV7251/image:=/oak_d_lite/depth/image_raw',
        '-r', f'{ignition_root}/StereoOV7251/camera_info:=/oak_d_lite/depth/camera_info',
        '-r', f'{ignition_root}/StereoOV7251/points:=/oak_d_lite/depth/points',
        '-r', f'{ignition_root}/StereoOV7251/depth_camera:=/oak_d_lite/depth/image_raw',
    ]

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        output='screen',
        arguments=bridge_args,
        parameters=[
            {'use_sim_time': True},
        ]
    )

    launch_items = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        ign_gazebo,
        spawn_x500_depth,
        static_tf_publisher,
        rviz2,
        ros_gz_bridge,
        joint_state_publisher,
    ]
    
    return LaunchDescription(launch_items)
