#!/usr/bin/env python3
"""Full autonomy launch: Gazebo + RTAB-Map + ESDF + RRT* + path executor + RViz.

Pipeline: RTAB-Map (point clouds) -> ESDF server (GetDistance) -> RRT* planner (path)
-> path executor (cmd_vel). Uses simulation time (/clock).

Usage:
    ros2 launch x500_rtabmap_slam autonomy_full.launch.py

Set goal in RViz: use "Publish Point" (click a 3D point) or "2D Goal Pose" (click-drag); or publish PoseStamped to /goal_pose.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    x500_rtabmap_slam_dir = get_package_share_directory("x500_rtabmap_slam")
    depthai_cam_dir = get_package_share_directory("depthai_cam")
    esdf_server_dir = get_package_share_directory("esdf_server")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    rviz = LaunchConfiguration("rviz", default="true")
    takeoff_speed = 0.6
    takeoff_duration = 2.5
    takeoff_delay = 10.0  # Delay before takeoff to allow sim and TF to start up
    takeoff_rate = 20
    takeoff_times = max(1, int(takeoff_duration * takeoff_rate))

    # 1) Simulation (Gazebo + bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_cam_dir, "launch", "depthai_cam_sim.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 1b) Point cloud filter: strip NaN/Inf and ground points to avoid RTAB-Map "invalid normal" and topic errors
    point_cloud_filter_node = Node(
        package="esdf_server",
        executable="point_cloud_filter_node",
        name="point_cloud_filter",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/oak_d_lite/depth/points"},
            {"output_topic": "/oak_d_lite/depth/points_filtered"},
            {"min_z_ground": 0.02},
            {"filter_ground": True},
        ],
    )

    # 2) RTAB-Map SLAM (from existing launch, inlined key nodes)
    rtabmap_params = {
        "use_sim_time": use_sim_time,
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "subscribe_scan_cloud": True,
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "publish_tf": True,
        "wait_for_transform": 0.5,
        "Rtabmap/DetectionRate": "2.0",
        "Reg/Strategy": "1",
        "Grid/FromDepth": True,
        "Grid/3D": "true",
        "Grid/RangeMax": "5.0",
        "Grid/CellSize": "0.05",
        "Icp/VoxelSize": "0.05",
        "Icp/MaxCorrespondenceDistance": "0.15",
        "Icp/Iterations": "30",
        "Mem/IncrementalMemory": "true",
    }
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        remappings=[
            ("rgb/image", "/oak_d_lite/rgb/image_raw"),
            ("rgb/camera_info", "/oak_d_lite/rgb/camera_info"),
            ("depth/image", "/oak_d_lite/depth/image_raw"),
            ("scan_cloud", "/oak_d_lite/depth/points_filtered"),
            ("grid_map", "/map"),
            ("grid_map_updates", "/map_updates"),
        ],
        arguments=["--delete_db_on_start"],
    )
    # Start RTAB-Map after sim and TF are up (odom->base_link from gazebo_odom_to_tf node)
    rtabmap_delayed = TimerAction(period=6.0, actions=[rtabmap_node])
    static_tf_world_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_map",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    static_tf_odom_gazebo = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom_gazebo_odom",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "gazebo_odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    # 3) ESDF server (OctoMap fallback: point cloud -> GetDistance)
    esdf_params = os.path.join(esdf_server_dir, "config", "esdf_params.yaml")
    esdf_node = Node(
        package="esdf_server",
        executable="esdf_server_node",
        name="esdf_server",
        output="screen",
        parameters=[
            esdf_params,
            {"use_sim_time": use_sim_time},
            {"point_cloud_topic": "/oak_d_lite/depth/points_filtered"},
        ],
        remappings=[("get_distance", "get_distance")],
    )

    # 4) RRT* planner (goal_pose -> path)
    rrt_node = Node(
        package="rrt_star_planner",
        executable="rrt_star_planner_node",
        name="rrt_star_planner",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_frame_id": "map"},
            {"base_frame_id": "base_link"},
            {"drone_radius": 0.25},
            {"safety_margin": 0.15},
            {"replan_rate": 2.0},
            {"max_planning_time": 0.5},
            {"goal_bias": 0.15},
        ],
    )

    # 5) Path executor (path -> cmd_vel; hover on empty/failure)
    path_exec_node = Node(
        package="path_executor",
        executable="path_executor_node",
        name="path_executor",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"path_topic": "path"},
            {"cmd_vel_topic": "/x500_depth/cmd_vel"},
            {"enable_topic": "/x500_depth/enable"},
            {"waypoint_tolerance": 0.25},
            {"max_linear_speed": 0.8},
        ],
    )

    # 5) Goal from RViz: click a 3D point -> /goal_pose (for RRT* planner)
    goal_from_rviz_node = Node(
        package="x500_rtabmap_slam",
        executable="goal_from_rviz_node",
        name="goal_from_rviz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 6) RViz
    rviz_config = os.path.join(x500_rtabmap_slam_dir, "config", "rtabmap_slam.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_autonomy",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    # 7) Startup hover: enable controller and command a short takeoff, then hover
    enable_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/x500_depth/enable",
            "std_msgs/msg/Bool",
            "{data: true}",
        ],
        output="screen",
    )
    takeoff_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--rate",
            str(takeoff_rate),
            "--times",
            str(takeoff_times),
            "/x500_depth/cmd_vel",
            "geometry_msgs/msg/Twist",
            f"{{linear: {{x: 0.0, y: 0.0, z: {takeoff_speed}}}, angular: {{z: 0.0}}}}",
        ],
        output="screen",
    )
    hover_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/x500_depth/cmd_vel",
            "geometry_msgs/msg/Twist",
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}",
        ],
        output="screen",
    )
    takeoff_sequence = TimerAction(
        period=takeoff_delay,
        actions=[enable_controller, takeoff_cmd],
    )
    hover_sequence = TimerAction(
        period=takeoff_delay + takeoff_duration,
        actions=[hover_cmd],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use /clock"),
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
        sim_launch,
        point_cloud_filter_node,
        static_tf_world_map,
        static_tf_odom_gazebo,
        rtabmap_delayed,
        esdf_node,
        rrt_node,
        path_exec_node,
        goal_from_rviz_node,
        rviz_node,
        takeoff_sequence,
        hover_sequence,
    ])
