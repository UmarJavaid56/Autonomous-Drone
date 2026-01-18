#!/usr/bin/env python3
"""Launch file for RTAB-Map SLAM with x500_depth drone and OAK-D camera.

This launch file:
- Launches RTAB-Map SLAM node configured for depth camera
- Subscribes to OAK-D camera topics (RGB, Depth, PointCloud2)
- Publishes map, occupancy grid, and odometry
- Provides 3D SLAM mapping using point clouds from depth camera

Topics Subscribed:
- /oak_d_lite/rgb/image_raw - RGB camera image
- /oak_d_lite/rgb/camera_info - RGB camera intrinsics
- /oak_d_lite/depth/image_raw - Depth image
- /oak_d_lite/depth/camera_info - Depth camera intrinsics
- /oak_d_lite/depth/points - Point cloud from depth camera
- /tf - Transform tree

Topics Published:
- /rtabmap/map - 3D occupancy grid map
- /rtabmap/mapData - RTAB-Map internal data
- /rtabmap/grid_map - 2D occupancy grid for navigation
- /rtabmap/odom - Visual odometry estimate
- /rtabmap/cloud_map - Point cloud map
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RTAB-Map SLAM."""
    pkg_share = get_package_share_directory('rtabmap_slam')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    
    # RTAB-Map parameters
    rtabmap_params = {
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan_cloud': True,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        'wait_for_transform': 0.2,
        
        # RTAB-Map specific parameters
        'Rtabmap/DetectionRate': '1.0',  # Process every frame
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.01',
        'RGBD/LinearUpdate': '0.01',
        'RGBD/OptimizeMaxError': '0.0',
        'Reg/Strategy': '1',  # 0=Vis, 1=ICP, 2=Vis+ICP
        'Reg/Force3DoF': 'false',
        'Grid/FromDepth': 'true',
        'Grid/3D': 'true',
        'Grid/RangeMax': '5.0',
        'Grid/CellSize': '0.05',
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        
        # ICP parameters for point cloud registration
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PointToPlaneK': '5',
        'Icp/PointToPlaneRadius': '0.0',
        'Icp/MaxTranslation': '3.0',
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '30',
        'Icp/PM': 'true',
        
        # Visualization
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '5',
    }
    
    # Add localization mode parameter
    if localization:
        rtabmap_params['Mem/IncrementalMemory'] = 'false'
        rtabmap_params['Mem/InitWMWithAllNodes'] = 'true'
    
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
        arguments=['--delete_db_on_start'] if not localization else [],
    )
    
    # RTAB-Map visualization node
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'wait_for_transform': 0.2,
        }],
        remappings=[
            ('rgb/image', '/oak_d_lite/rgb/image_raw'),
            ('rgb/camera_info', '/oak_d_lite/rgb/camera_info'),
            ('depth/image', '/oak_d_lite/depth/image_raw'),
            ('scan_cloud', '/oak_d_lite/depth/points'),
        ],
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Set to true for localization mode (vs mapping mode)'
        ),
        
        # Set RTAB-Map database location
        SetEnvironmentVariable('RTABMAP_DATABASE_PATH', 
                             os.path.join(os.path.expanduser('~'), '.ros/rtabmap.db')),
        
        # Launch nodes
        rtabmap_slam_node,
        rtabmap_viz_node,
    ])
