#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('depthai_cam')
    world_path = os.path.join(pkg_share, 'worlds', 'depthai_test.world')
    robot_description_path = os.path.join(pkg_share, 'models', 'x500_depth', 'model_description.urdf')


    # Ensure Gazebo can find default models (ground_plane, sun), our models, and PX4 models
    ign_resource_path = os.pathsep.join([
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        '/usr/share/gz/gz-sim8',
        '/usr/share/gz/gz-fuel-tools',
        os.path.join(pkg_share, 'models'),  # Our local models (x500, x500_base, OakD-Lite, etc.)
    ])

    # Launch Gazebo Sim with GUI
    # Using ogre2 render engine (required for depth cameras)
    # Set GZ_CONFIG_PATH to use system gz configs (includes sim8.yaml)
    # -r flag starts the simulation automatically (not paused)
    ign_gazebo = ExecuteProcess(
        cmd=['/usr/bin/gz', 'sim', '--force-version', '8', world_path, 
             '-v', '4', '-r', '--render-engine', 'ogre2'],
        output='screen',
        additional_env={
            'GZ_CONFIG_PATH': '/usr/share/gz',
            'GZ_SIM_RESOURCE_PATH': ign_resource_path
        }
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

    # Robot State Publisher - publishes robot model transforms from simplified SDF
    # Uses model_description.sdf (not the full Gazebo SDF) which is cleaner for visualization
    # robot_state_publisher handles ALL transforms:
    # - Fixed joints (camera_joint) → published automatically as static TF
    # - Revolute joints (rotors) → computed from /joint_states (bridged from Gazebo)
    with open(robot_description_path, 'r') as sdf_file:
        robot_description = sdf_file.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ]
    )

    # Static TF for world -> odom (identity transform)
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_odom',
        arguments=['--frame-id', 'world', '--child-frame-id', 'odom'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Pose to TF node - converts model pose from Gazebo to odom->base_link TF
    # Gazebo publishes gz.msgs.Pose_V which bridges to PoseArray
    pose_to_tf = Node(
        package='depthai_cam',
        executable='pose_to_tf',
        name='pose_to_tf',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'parent_frame': 'odom'},
            {'child_frame': 'base_link'},
            {'pose_topic': '/model/x500_depth/pose'},  # Direct Gazebo topic
            {'use_pose_array': True},
        ],
    )


    # Bridges for TF and OakD-Lite camera topics (RGB + Depth + CameraInfo + PointCloud)
    # Ignition topic roots
    world_name = 'depthai_test_world'
    model_name = 'x500_depth'
    link_name = 'camera_link'

    ignition_root = f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'

    bridge_args = [
        # Bridge simulation clock to /clock so ROS nodes can use sim time
        f'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        
        # Model pose - Gazebo publishes Pose_V (vector of poses) -> PoseArray
        f'/model/{model_name}/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        
        # Joint states - allows rotor visualization to update in real-time
        f'/world/{world_name}/model/{model_name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',

        # Velocity control command - ROS Twist -> Gazebo Twist (for keyboard teleop)
        # Note: MulticopterVelocityControl creates topics at /{robotNamespace}/cmd_vel
        f'/{model_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        
        # Enable topic for velocity controller - ROS Bool -> Gazebo Boolean
        f'/{model_name}/enable@std_msgs/msg/Bool]gz.msgs.Boolean',

        # RGB camera (IMX214)
        f'{ignition_root}/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{ignition_root}/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        # Depth camera (StereoOV7251) - RGBD camera sensor
        f'{ignition_root}/StereoOV7251/image@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{ignition_root}/StereoOV7251/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{ignition_root}/StereoOV7251/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'{ignition_root}/StereoOV7251/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

        '--ros-args',
        
        # Remap joint states from Gazebo to /joint_states
        '-r', f'/world/{world_name}/model/{model_name}/joint_state:=/joint_states',

        # Remap RGB topics to stable ROS names
        '-r', f'{ignition_root}/IMX214/image:=/oak_d_lite/rgb/image_raw',
        '-r', f'{ignition_root}/IMX214/camera_info:=/oak_d_lite/rgb/camera_info',

        # Remap Depth topics to stable ROS names
        '-r', f'{ignition_root}/StereoOV7251/image:=/oak_d_lite/stereo/image_raw',
        '-r', f'{ignition_root}/StereoOV7251/depth_image:=/oak_d_lite/depth/image_raw',
        '-r', f'{ignition_root}/StereoOV7251/camera_info:=/oak_d_lite/depth/camera_info',
        '-r', f'{ignition_root}/StereoOV7251/points:=/oak_d_lite/depth/points',

        # Remap velocity control topics for keyboard teleop
        # The Gazebo topics /{robotNamespace}/cmd_vel map to /x500_depth/cmd_vel in ROS
        '-r', f'/{model_name}/cmd_vel:=/x500_depth/cmd_vel',
        '-r', f'/{model_name}/enable:=/x500_depth/enable',
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
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ign_resource_path),
        ign_gazebo,
        static_tf_world_odom,
        pose_to_tf,
        robot_state_publisher,
        rviz2,
        ros_gz_bridge,
    ]
    
    return LaunchDescription(launch_items)
