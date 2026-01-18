#!/usr/bin/env python3
"""Launch file for x500_depth simulation with RTAB-Map SLAM.

This launch file combines:
- x500_depth Gazebo simulation with OAK-D camera
- RTAB-Map SLAM for 3D mapping using depth camera point clouds
- RViz2 visualization with SLAM data

Usage:
    ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py

For localization mode (using existing map):
    ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py localization:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    # Get package directories
    rtabmap_slam_dir = get_package_share_directory('rtabmap_slam')
    depthai_cam_dir = get_package_share_directory('depthai_cam')
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    
    # Include x500_depth simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_cam_dir, 'launch', 'depthai_cam_sim.launch.py')
        ),
    )
    
    # RTAB-Map parameters optimized for drone navigation
    rtabmap_params = {
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan_cloud': True,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        'wait_for_transform': 0.5,
        
        # Core SLAM parameters
        'queue_size': 30,
        'Rtabmap/DetectionRate': '2.0',  # Hz - process 2 frames per second
        'Rtabmap/TimeThr': '0',  # No time limit for map update
        'Rtabmap/MemoryThr': '0',  # Disable memory management threshold
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        
        # Odometry from visual features
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',  # rad - create new node if rotated > 3 degrees
        'RGBD/LinearUpdate': '0.05',   # m - create new node if moved > 5cm
        'RGBD/OptimizeMaxError': '1.0',
        
        # Registration strategy: ICP with visual features
        'Reg/Strategy': '1',  # 0=Visual, 1=ICP, 2=Visual+ICP
        'Reg/Force3DoF': 'false',  # Allow full 6DOF for drone
        
        # ICP parameters for point cloud alignment
        'Icp/VoxelSize': '0.05',  # 5cm voxel size
        'Icp/MaxCorrespondenceDistance': '0.15',  # 15cm max correspondence
        'Icp/PointToPlaneK': '5',
        'Icp/PointToPlaneRadius': '0.0',
        'Icp/MaxTranslation': '2.0',  # Max 2m translation between frames
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '30',
        'Icp/PM': 'true',  # Use libpointmatcher
        'Icp/PMOutlierRatio': '0.7',
        
        # Grid map parameters for 3D mapping
        'Grid/FromDepth': 'true',
        'Grid/3D': 'true',
        'Grid/RangeMax': '5.0',  # 5m max range
        'Grid/RangeMin': '0.4',  # 40cm min range
        'Grid/CellSize': '0.05',  # 5cm cell size
        'Grid/ClusterRadius': '0.1',
        'Grid/GroundIsObstacle': 'false',
        'Grid/MaxGroundHeight': '0.0',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'Grid/NormalsSegmentation': 'false',
        
        # Loop closure detection
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Kp/MaxFeatures': '400',
        'Kp/DetectorStrategy': '6',  # GFTT/BRIEF
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.1',
        
        # Optimization
        'RGBD/OptimizeStrategy': '1',  # g2o
        'Optimizer/Strategy': '1',  # g2o
        'Optimizer/Iterations': '20',
        'Optimizer/Epsilon': '0.00001',
    }
    
    # Update parameters for localization mode
    localization_params = rtabmap_params.copy()
    localization_params['Mem/IncrementalMemory'] = 'false'
    localization_params['Mem/InitWMWithAllNodes'] = 'true'
    
    # RTAB-Map SLAM node
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=[
            ('rgb/image', '/oak_d_lite/rgb/image_raw'),
            ('rgb/camera_info', '/oak_d_lite/rgb/camera_info'),
            ('depth/image', '/oak_d_lite/depth/image_raw'),
            ('scan_cloud', '/oak_d_lite/depth/points'),
        ],
        arguments=['--delete_db_on_start'],
    )
    
    # RViz2 with RTAB-Map visualization
    rviz_config_path = os.path.join(rtabmap_slam_dir, 'config', 'rtabmap_slam.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_rtabmap',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('rviz', default='true')
        ),
    )
    
    # Map server for publishing the map
    map_server_node = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_clouds': 10,
            'fixed_frame_id': 'map',
        }],
        remappings=[
            ('cloud', '/oak_d_lite/depth/points'),
        ],
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch in localization mode (vs mapping mode)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 visualization'
        ),
        
        # Launch simulation
        simulation_launch,
        
        # Launch RTAB-Map SLAM
        rtabmap_slam_node,
        
        # Launch utilities
        map_server_node,
        rviz2_node,
    ])
