#!/usr/bin/env python3
"""Full autonomy launch: Gazebo + RTAB-Map + ESDF + RRT* + path executor + RViz.

Pipeline: RTAB-Map (point clouds) -> ESDF server (GetDistance) -> RRT* planner (path)
-> path executor (cmd_vel). Uses simulation time (/clock).

Usage:
    ros2 launch x500_rtabmap_slam autonomy_full.launch.py
    # Mapping-only (manual flight): disable planner/executor stack
    ros2 launch x500_rtabmap_slam autonomy_full.launch.py enable_autonomy:=false
    # Safe mode (default): unknown space treated as occupied
    ros2 launch x500_rtabmap_slam autonomy_full.launch.py unknown_is_occupied:=true
    # Explore mode: unknown space treated as free (legacy behavior)
    ros2 launch x500_rtabmap_slam autonomy_full.launch.py unknown_is_occupied:=false
    # Approximate solution rejection threshold (meters); <=0 disables rejection
    ros2 launch x500_rtabmap_slam autonomy_full.launch.py max_approx_goal_distance:=0.8

Set goal in RViz: use "Publish Point" (click a 3D point) or "2D Goal Pose" (click-drag); or publish PoseStamped to /goal_pose.
Map-driven mission mode: publish a final PoseStamped goal to /maze_final_goal and the node will generate adaptive subgoals.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    x500_rtabmap_slam_dir = get_package_share_directory("x500_rtabmap_slam")
    depthai_cam_dir = get_package_share_directory("depthai_cam")
    esdf_server_dir = get_package_share_directory("esdf_server")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    rviz = LaunchConfiguration("rviz", default="true")
    enable_autonomy = LaunchConfiguration("enable_autonomy", default="true")
    enable_waypoint_mission = LaunchConfiguration("enable_waypoint_mission", default="false")
    mission_start_delay_sec = LaunchConfiguration("mission_start_delay_sec", default="0.0")
    mission_final_goal_topic = LaunchConfiguration("mission_final_goal_topic", default="/maze_final_goal")
    mission_final_goal_tolerance = LaunchConfiguration("mission_final_goal_tolerance", default="0.60")
    mission_subgoal_lookahead_m = LaunchConfiguration("mission_subgoal_lookahead_m", default="1.8")
    mission_inflation_radius_m = LaunchConfiguration("mission_inflation_radius_m", default="0.12")
    mission_unknown_is_blocked = LaunchConfiguration("mission_unknown_is_blocked", default="true")
    mission_recovery_empty_path_streak = LaunchConfiguration(
        "mission_recovery_empty_path_streak", default="10"
    )
    mission_recovery_stuck_timeout_sec = LaunchConfiguration(
        "mission_recovery_stuck_timeout_sec", default="8.0"
    )
    mission_recovery_progress_epsilon_m = LaunchConfiguration(
        "mission_recovery_progress_epsilon_m", default="0.12"
    )
    mission_recovery_min_target_distance_m = LaunchConfiguration(
        "mission_recovery_min_target_distance_m", default="0.8"
    )
    mission_recovery_retarget_cooldown_sec = LaunchConfiguration(
        "mission_recovery_retarget_cooldown_sec", default="2.0"
    )
    mission_enable_progress_subgoal = LaunchConfiguration(
        "mission_enable_progress_subgoal", default="false"
    )
    mission_progress_min_improvement_m = LaunchConfiguration(
        "mission_progress_min_improvement_m", default="0.20"
    )
    mission_progress_min_target_distance_m = LaunchConfiguration(
        "mission_progress_min_target_distance_m", default="0.70"
    )
    mission_progress_max_target_distance_m = LaunchConfiguration(
        "mission_progress_max_target_distance_m", default="1.60"
    )
    mission_progress_distance_penalty = LaunchConfiguration(
        "mission_progress_distance_penalty", default="0.10"
    )
    mission_recovery_progress_min_improvement_m = LaunchConfiguration(
        "mission_recovery_progress_min_improvement_m", default="0.10"
    )
    mission_recovery_progress_max_target_distance_m = LaunchConfiguration(
        "mission_recovery_progress_max_target_distance_m", default="1.80"
    )
    auto_takeoff = LaunchConfiguration("auto_takeoff", default="false")
    drone_radius = LaunchConfiguration("drone_radius", default="0.25")
    unknown_is_occupied = LaunchConfiguration("unknown_is_occupied", default="false")
    max_approx_goal_distance = LaunchConfiguration("max_approx_goal_distance", default="0.8")
    safety_margin = LaunchConfiguration("safety_margin", default="0.32")
    adaptive_safety_margin = LaunchConfiguration("adaptive_safety_margin", default="true")
    min_safety_margin = LaunchConfiguration("min_safety_margin", default="0.08")
    safety_margin_relax_step = LaunchConfiguration("safety_margin_relax_step", default="0.02")
    no_solution_before_relax = LaunchConfiguration("no_solution_before_relax", default="4")
    success_before_tighten = LaunchConfiguration("success_before_tighten", default="4")
    enable_dense_path_validation = LaunchConfiguration("enable_dense_path_validation", default="true")
    collision_check_resolution_m = LaunchConfiguration("collision_check_resolution_m", default="0.05")
    dense_check_safety_scale = LaunchConfiguration("dense_check_safety_scale", default="0.70")
    postcheck_relax_step = LaunchConfiguration("postcheck_relax_step", default="0.02")
    start_exempt_radius = LaunchConfiguration("start_exempt_radius", default="0.12")
    allow_progressive_approximate = LaunchConfiguration("allow_progressive_approximate", default="true")
    min_progress_toward_goal_m = LaunchConfiguration("min_progress_toward_goal_m", default="0.25")
    allow_safe_prefix_fallback = LaunchConfiguration("allow_safe_prefix_fallback", default="true")
    min_progress_path_length_m = LaunchConfiguration("min_progress_path_length_m", default="0.20")
    allow_regressive_safe_prefix = LaunchConfiguration("allow_regressive_safe_prefix", default="true")
    max_safe_prefix_regression_m = LaunchConfiguration("max_safe_prefix_regression_m", default="1.20")
    consecutive_failures_before_hover = LaunchConfiguration("consecutive_failures_before_hover", default="8")
    hover_on_planning_failure = LaunchConfiguration("hover_on_planning_failure", default="true")
    planner_z_min = LaunchConfiguration("planner_z_min", default="-0.2")
    planner_z_max = LaunchConfiguration("planner_z_max", default="1.25")
    map_x_min = LaunchConfiguration("map_x_min", default="-2.0")
    map_x_max = LaunchConfiguration("map_x_max", default="12.5")
    map_y_min = LaunchConfiguration("map_y_min", default="-6.0")
    map_y_max = LaunchConfiguration("map_y_max", default="6.0")
    min_altitude_for_xy_motion = LaunchConfiguration("min_altitude_for_xy_motion", default="0.85")
    hover_reference_frame_id = LaunchConfiguration("hover_reference_frame_id", default="odom")
    cmd_vel_body_x_gain = LaunchConfiguration("cmd_vel_body_x_gain", default="1.0")
    cmd_vel_body_y_gain = LaunchConfiguration("cmd_vel_body_y_gain", default="1.0")
    max_path_age_sec = LaunchConfiguration("max_path_age_sec", default="1.5")
    min_consecutive_nonempty_paths = LaunchConfiguration("min_consecutive_nonempty_paths", default="3")
    hard_stop_on_no_path = LaunchConfiguration("hard_stop_on_no_path", default="true")
    hard_stop_drift_guard = LaunchConfiguration("hard_stop_drift_guard", default="true")
    hard_stop_drift_guard_pos_threshold = LaunchConfiguration("hard_stop_drift_guard_pos_threshold", default="0.02")
    hard_stop_drift_guard_vel_threshold = LaunchConfiguration("hard_stop_drift_guard_vel_threshold", default="0.02")
    hard_stop_drift_guard_hold_kp = LaunchConfiguration("hard_stop_drift_guard_hold_kp", default="0.6")
    hard_stop_drift_guard_brake_kp = LaunchConfiguration("hard_stop_drift_guard_brake_kp", default="1.2")
    hard_stop_drift_guard_max_speed = LaunchConfiguration("hard_stop_drift_guard_max_speed", default="0.20")
    hover_brake_kp = LaunchConfiguration("hover_brake_kp", default="4.0")
    hover_brake_max_speed = LaunchConfiguration("hover_brake_max_speed", default="1.5")
    hover_hold_kp = LaunchConfiguration("hover_hold_kp", default="2.0")
    hover_hold_max_speed = LaunchConfiguration("hover_hold_max_speed", default="1.5")
    hover_total_max_speed = LaunchConfiguration("hover_total_max_speed", default="1.5")
    takeoff_speed = 0.6
    takeoff_duration = 3.0  # sim-time seconds of ascent
    takeoff_delay = 10.0  # wall-time delay before takeoff to allow sim and TF to start up

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
            {"filter_ground": False},
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
            {"x_min": map_x_min},
            {"x_max": map_x_max},
            {"y_min": map_y_min},
            {"y_max": map_y_max},
            {"max_map_age_sec": 2.0},
            {"unknown_is_occupied": unknown_is_occupied},
        ],
        remappings=[("get_distance", "get_distance")],
    )

    # 4) RRT* planner (goal_pose_planned -> path)
    rrt_node = Node(
        package="rrt_star_planner",
        executable="rrt_star_planner_node",
        name="rrt_star_planner",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_frame_id": "map"},
            {"base_frame_id": "base_link"},
            {"drone_radius": ParameterValue(drone_radius, value_type=float)},
            {"safety_margin": safety_margin},
            {"adaptive_safety_margin": adaptive_safety_margin},
            {"min_safety_margin": min_safety_margin},
            {"safety_margin_relax_step": safety_margin_relax_step},
            {"no_solution_before_relax": no_solution_before_relax},
            {"success_before_tighten": success_before_tighten},
            {"enable_dense_path_validation": enable_dense_path_validation},
            {"collision_check_resolution_m": collision_check_resolution_m},
            {"dense_check_safety_scale": dense_check_safety_scale},
            {"postcheck_relax_step": postcheck_relax_step},
            {"start_exempt_radius": start_exempt_radius},
            {"allow_progressive_approximate": allow_progressive_approximate},
            {"min_progress_toward_goal_m": min_progress_toward_goal_m},
            {"allow_safe_prefix_fallback": allow_safe_prefix_fallback},
            {"min_progress_path_length_m": min_progress_path_length_m},
            {"allow_regressive_safe_prefix": allow_regressive_safe_prefix},
            {"max_safe_prefix_regression_m": max_safe_prefix_regression_m},
            {"consecutive_failures_before_hover": consecutive_failures_before_hover},
            {"hover_on_planning_failure": hover_on_planning_failure},
            {"x_min": map_x_min},
            {"x_max": map_x_max},
            {"y_min": map_y_min},
            {"y_max": map_y_max},
            {"z_min": planner_z_min},
            {"z_max": planner_z_max},
            {"replan_rate": 2.0},
            {"max_planning_time": 1.0},
            {"goal_bias": 0.15},
            {"path_samples": 120},
            {"max_approx_goal_distance": ParameterValue(max_approx_goal_distance, value_type=float)},
        ],
        remappings=[
            ("goal_pose", "/goal_pose_planned"),
        ],
        condition=IfCondition(enable_autonomy),
    )

    # 5) Path executor (path -> cmd_vel; hover on empty/failure)
    path_exec_node = Node(
        package="path_executor",
        executable="path_executor_node",
        name="path_executor",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"hover_reference_frame_id": hover_reference_frame_id},
            {"path_topic": "path"},
            {"cmd_vel_topic": "/x500_depth/cmd_vel"},
            {"enable_topic": "/x500_depth/enable"},
            {"waypoint_tolerance": 0.15},
            {"max_linear_speed": 1.2},
            {"max_angular_speed": 1.2},
            {"linear_kp": 1.6},
            {"min_linear_speed": 0.20},
            {"rotate_to_heading_before_move": True},
            {"heading_align_threshold_rad": 0.8},
            {"heading_hard_stop_threshold_rad": 2.6},
            {"min_heading_speed_factor": 0.45},
            {"min_xy_dist_for_heading_align": 0.4},
            {"cmd_vel_is_body_frame": True},
            {"cmd_vel_body_x_gain": cmd_vel_body_x_gain},
            {"cmd_vel_body_y_gain": cmd_vel_body_y_gain},
            {"max_path_age_sec": max_path_age_sec},
            {"min_consecutive_nonempty_paths": min_consecutive_nonempty_paths},
            {"hard_stop_on_no_path": hard_stop_on_no_path},
            {"hard_stop_drift_guard": hard_stop_drift_guard},
            {"hard_stop_drift_guard_pos_threshold": hard_stop_drift_guard_pos_threshold},
            {"hard_stop_drift_guard_vel_threshold": hard_stop_drift_guard_vel_threshold},
            {"hard_stop_drift_guard_hold_kp": hard_stop_drift_guard_hold_kp},
            {"hard_stop_drift_guard_brake_kp": hard_stop_drift_guard_brake_kp},
            {"hard_stop_drift_guard_max_speed": hard_stop_drift_guard_max_speed},
            {"hover_brake_kp": hover_brake_kp},
            {"hover_brake_max_speed": hover_brake_max_speed},
            {"hover_hold_kp": hover_hold_kp},
            {"hover_hold_max_speed": hover_hold_max_speed},
            {"hover_total_max_speed": hover_total_max_speed},
            {"enforce_takeoff_before_xy": True},
            {"min_altitude_for_xy_motion": min_altitude_for_xy_motion},
            {"enforce_min_target_altitude": True},
            {"min_target_altitude": min_altitude_for_xy_motion},
            {"takeoff_altitude_tolerance": 0.12},
            {"takeoff_vertical_speed": 0.6},
            {"takeoff_vertical_kp": 1.2},
        ],
        condition=IfCondition(enable_autonomy),
    )

    # 6) Goal from RViz: /goal_pose (input) -> /goal_pose_planned (output, z-corrected)
    goal_from_rviz_node = Node(
        package="x500_rtabmap_slam",
        executable="goal_from_rviz_node",
        name="goal_from_rviz",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"goal_z_min": planner_z_min},
            {"goal_z_max": planner_z_max},
            {"clamp_goal_z": True},
            {"min_2d_goal_z": min_altitude_for_xy_motion},
        ],
        condition=IfCondition(enable_autonomy),
    )

    # 6b) Optional map-driven maze mission: final goal -> adaptive subgoals on /goal_pose
    waypoint_mission_node = Node(
        package="x500_rtabmap_slam",
        executable="waypoint_mission_node",
        name="waypoint_mission",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"start_delay_sec": ParameterValue(mission_start_delay_sec, value_type=float)},
            {"final_goal_topic": mission_final_goal_topic},
            {"goal_z": ParameterValue(min_altitude_for_xy_motion, value_type=float)},
            {"final_goal_reach_tolerance": ParameterValue(mission_final_goal_tolerance, value_type=float)},
            {"subgoal_lookahead_m": ParameterValue(mission_subgoal_lookahead_m, value_type=float)},
            {"inflation_radius_m": ParameterValue(mission_inflation_radius_m, value_type=float)},
            {"unknown_is_blocked": ParameterValue(mission_unknown_is_blocked, value_type=bool)},
            {"recovery_empty_path_streak": ParameterValue(mission_recovery_empty_path_streak, value_type=int)},
            {"recovery_stuck_timeout_sec": ParameterValue(mission_recovery_stuck_timeout_sec, value_type=float)},
            {"recovery_progress_epsilon_m": ParameterValue(mission_recovery_progress_epsilon_m, value_type=float)},
            {"recovery_min_target_distance_m": ParameterValue(mission_recovery_min_target_distance_m, value_type=float)},
            {"recovery_retarget_cooldown_sec": ParameterValue(mission_recovery_retarget_cooldown_sec, value_type=float)},
            {"enable_progress_subgoal": ParameterValue(mission_enable_progress_subgoal, value_type=bool)},
            {"progress_min_improvement_m": ParameterValue(mission_progress_min_improvement_m, value_type=float)},
            {"progress_min_target_distance_m": ParameterValue(mission_progress_min_target_distance_m, value_type=float)},
            {"progress_max_target_distance_m": ParameterValue(mission_progress_max_target_distance_m, value_type=float)},
            {"progress_distance_penalty": ParameterValue(mission_progress_distance_penalty, value_type=float)},
            {"recovery_progress_min_improvement_m": ParameterValue(mission_recovery_progress_min_improvement_m, value_type=float)},
            {"recovery_progress_max_target_distance_m": ParameterValue(mission_recovery_progress_max_target_distance_m, value_type=float)},
        ],
        condition=IfCondition(enable_waypoint_mission),
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

    # 7) Startup takeoff: sim-time-aware node that enables the controller,
    #    ascends for takeoff_duration sim seconds, then holds hover.
    takeoff_node = Node(
        package="x500_rtabmap_slam",
        executable="takeoff_node",
        name="takeoff",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"takeoff_speed": takeoff_speed},
            {"takeoff_duration": takeoff_duration},
            {"cmd_vel_topic": "/x500_depth/cmd_vel"},
            {"enable_topic": "/x500_depth/enable"},
        ],
        condition=IfCondition(enable_autonomy),
    )
    takeoff_delayed = TimerAction(
        period=takeoff_delay,
        actions=[takeoff_node],
        condition=IfCondition(auto_takeoff),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use /clock"),
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
        DeclareLaunchArgument(
            "enable_autonomy",
            default_value="true",
            description="If false, run mapping/simulation stack only (no planner/path executor/goal bridge).",
        ),
        DeclareLaunchArgument(
            "enable_waypoint_mission",
            default_value="false",
            description="If true, run map-driven maze mission node (single final goal -> adaptive subgoals).",
        ),
        DeclareLaunchArgument(
            "mission_start_delay_sec",
            default_value="0.0",
            description="Delay (wall-time seconds) before map-driven mission starts.",
        ),
        DeclareLaunchArgument(
            "mission_final_goal_topic",
            default_value="/maze_final_goal",
            description="PoseStamped topic used to provide the final maze goal (map frame).",
        ),
        DeclareLaunchArgument(
            "mission_final_goal_tolerance",
            default_value="0.60",
            description="XY distance threshold (meters) to consider the final maze goal reached.",
        ),
        DeclareLaunchArgument(
            "mission_subgoal_lookahead_m",
            default_value="1.8",
            description="A* path lookahead distance (meters) used to choose intermediate subgoals.",
        ),
        DeclareLaunchArgument(
            "mission_inflation_radius_m",
            default_value="0.12",
            description="Obstacle inflation radius for map-driven subgoal generation.",
        ),
        DeclareLaunchArgument(
            "mission_unknown_is_blocked",
            default_value="true",
            description="If true, unknown occupancy cells are blocked in mission-level path search.",
        ),
        DeclareLaunchArgument(
            "mission_recovery_empty_path_streak",
            default_value="10",
            description="Consecutive empty planner paths before mission forces an open-space recovery target.",
        ),
        DeclareLaunchArgument(
            "mission_recovery_stuck_timeout_sec",
            default_value="8.0",
            description="No-progress timeout before mission forces a recovery target.",
        ),
        DeclareLaunchArgument(
            "mission_recovery_progress_epsilon_m",
            default_value="0.12",
            description="Minimum target-distance improvement to count as mission progress.",
        ),
        DeclareLaunchArgument(
            "mission_recovery_min_target_distance_m",
            default_value="0.8",
            description="Minimum distance for recovery-selected target from current position.",
        ),
        DeclareLaunchArgument(
            "mission_recovery_retarget_cooldown_sec",
            default_value="2.0",
            description="Cooldown between forced mission recovery retarget attempts.",
        ),
        DeclareLaunchArgument(
            "mission_enable_progress_subgoal",
            default_value="false",
            description="If true, mission selects dynamic safe progress points when direct map path is unavailable.",
        ),
        DeclareLaunchArgument(
            "mission_progress_min_improvement_m",
            default_value="0.20",
            description="Minimum reduction in final-goal distance (meters) required for a safe progress target.",
        ),
        DeclareLaunchArgument(
            "mission_progress_min_target_distance_m",
            default_value="0.70",
            description="Minimum separation from current position (meters) for normal safe progress targets.",
        ),
        DeclareLaunchArgument(
            "mission_progress_max_target_distance_m",
            default_value="1.60",
            description="Maximum separation from current position (meters) for normal safe progress targets.",
        ),
        DeclareLaunchArgument(
            "mission_progress_distance_penalty",
            default_value="0.10",
            description="Penalty on target distance when scoring safe progress candidates.",
        ),
        DeclareLaunchArgument(
            "mission_recovery_progress_min_improvement_m",
            default_value="0.10",
            description="Minimum reduction in final-goal distance (meters) required for recovery safe progress targets.",
        ),
        DeclareLaunchArgument(
            "mission_recovery_progress_max_target_distance_m",
            default_value="1.80",
            description="Maximum separation from current position (meters) for recovery safe progress targets.",
        ),
        DeclareLaunchArgument(
            "planner_z_min",
            default_value="-0.2",
            description="Planner lower altitude bound in map frame (meters).",
        ),
        DeclareLaunchArgument(
            "planner_z_max",
            default_value="1.25",
            description="Planner upper altitude bound in map frame (meters).",
        ),
        DeclareLaunchArgument(
            "map_x_min",
            default_value="-2.0",
            description="Shared ESDF/RRT lower X bound in map frame (meters).",
        ),
        DeclareLaunchArgument(
            "map_x_max",
            default_value="12.5",
            description="Shared ESDF/RRT upper X bound in map frame (meters).",
        ),
        DeclareLaunchArgument(
            "map_y_min",
            default_value="-6.0",
            description="Shared ESDF/RRT lower Y bound in map frame (meters).",
        ),
        DeclareLaunchArgument(
            "map_y_max",
            default_value="6.0",
            description="Shared ESDF/RRT upper Y bound in map frame (meters).",
        ),
        DeclareLaunchArgument(
            "min_altitude_for_xy_motion",
            default_value="0.85",
            description="Path executor safety gate: climb to this altitude before XY translation.",
        ),
        DeclareLaunchArgument(
            "hover_reference_frame_id",
            default_value="odom",
            description="Frame used by hover safety controller (should be stable even if map drifts).",
        ),
        DeclareLaunchArgument(
            "cmd_vel_body_x_gain",
            default_value="1.0",
            description="Scale/sign for body-frame cmd_vel x sent to Gazebo velocity controller.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_body_y_gain",
            default_value="1.0",
            description="Scale/sign for body-frame cmd_vel y sent to Gazebo velocity controller.",
        ),
        DeclareLaunchArgument(
            "max_path_age_sec",
            default_value="1.5",
            description="Maximum path age before executor forces hover.",
        ),
        DeclareLaunchArgument(
            "min_consecutive_nonempty_paths",
            default_value="3",
            description="Require this many consecutive non-empty path messages before allowing XY motion.",
        ),
        DeclareLaunchArgument(
            "hard_stop_on_no_path",
            default_value="true",
            description="If true, executor does not track path on empty/stale input; optional drift guard may still apply.",
        ),
        DeclareLaunchArgument(
            "hard_stop_drift_guard",
            default_value="true",
            description="In hard-stop mode, apply bounded anti-drift correction when displacement/velocity exceeds thresholds.",
        ),
        DeclareLaunchArgument(
            "hard_stop_drift_guard_pos_threshold",
            default_value="0.02",
            description="Hard-stop drift guard activation threshold for XY displacement (meters).",
        ),
        DeclareLaunchArgument(
            "hard_stop_drift_guard_vel_threshold",
            default_value="0.02",
            description="Hard-stop drift guard activation threshold for XY speed (m/s).",
        ),
        DeclareLaunchArgument(
            "hard_stop_drift_guard_hold_kp",
            default_value="0.6",
            description="Hard-stop drift guard XY position gain.",
        ),
        DeclareLaunchArgument(
            "hard_stop_drift_guard_brake_kp",
            default_value="1.2",
            description="Hard-stop drift guard XY velocity damping gain.",
        ),
        DeclareLaunchArgument(
            "hard_stop_drift_guard_max_speed",
            default_value="0.20",
            description="Hard-stop drift guard max XY correction speed (m/s).",
        ),
        DeclareLaunchArgument(
            "hover_brake_kp",
            default_value="4.0",
            description="Velocity damping gain used by hover safety mode.",
        ),
        DeclareLaunchArgument(
            "hover_brake_max_speed",
            default_value="1.5",
            description="Max XY damping command speed in hover safety mode.",
        ),
        DeclareLaunchArgument(
            "hover_hold_kp",
            default_value="2.0",
            description="XY position-hold gain toward hover anchor when no path is valid.",
        ),
        DeclareLaunchArgument(
            "hover_hold_max_speed",
            default_value="1.5",
            description="Max XY hold speed toward hover anchor when no path is valid.",
        ),
        DeclareLaunchArgument(
            "hover_total_max_speed",
            default_value="1.5",
            description="Final max XY speed after combining hover hold + damping commands.",
        ),
        DeclareLaunchArgument(
            "safety_margin",
            default_value="0.32",
            description="Nominal ESDF clearance added to drone_radius (meters).",
        ),
        DeclareLaunchArgument(
            "drone_radius",
            default_value="0.30",
            description="Planner collision radius used for ESDF clearance checks (meters).",
        ),
        DeclareLaunchArgument(
            "adaptive_safety_margin",
            default_value="true",
            description="If true, reduce safety_margin after repeated planning failures and restore on success.",
        ),
        DeclareLaunchArgument(
            "min_safety_margin",
            default_value="0.08",
            description="Lower bound for adaptive safety margin relaxation.",
        ),
        DeclareLaunchArgument(
            "safety_margin_relax_step",
            default_value="0.02",
            description="Adaptive safety margin step size (meters).",
        ),
        DeclareLaunchArgument(
            "no_solution_before_relax",
            default_value="4",
            description="Consecutive planning failures before relaxing safety margin.",
        ),
        DeclareLaunchArgument(
            "success_before_tighten",
            default_value="4",
            description="Consecutive successful replans before tightening safety margin.",
        ),
        DeclareLaunchArgument(
            "enable_dense_path_validation",
            default_value="true",
            description="If true, run a dense ESDF collision validation pass on solved paths before publishing.",
        ),
        DeclareLaunchArgument(
            "collision_check_resolution_m",
            default_value="0.05",
            description="Dense path collision sampling spacing in meters.",
        ),
        DeclareLaunchArgument(
            "dense_check_safety_scale",
            default_value="0.70",
            description="Scale [0..1] applied to safety_margin during dense path validation.",
        ),
        DeclareLaunchArgument(
            "postcheck_relax_step",
            default_value="0.02",
            description="On dense-check failure, reduce safety margin by this step (down to min_safety_margin).",
        ),
        DeclareLaunchArgument(
            "start_exempt_radius",
            default_value="0.12",
            description="Collision-check exemption radius around the drone to reduce startup self-map lock-in.",
        ),
        DeclareLaunchArgument(
            "allow_progressive_approximate",
            default_value="true",
            description="If true, allow approximate solutions that still make minimum progress to goal.",
        ),
        DeclareLaunchArgument(
            "min_progress_toward_goal_m",
            default_value="0.25",
            description="Minimum goal-distance reduction required for progress-only approximate/safe-prefix acceptance.",
        ),
        DeclareLaunchArgument(
            "allow_safe_prefix_fallback",
            default_value="true",
            description="If true, publish a collision-safe path prefix when dense-check rejects full path.",
        ),
        DeclareLaunchArgument(
            "min_progress_path_length_m",
            default_value="0.20",
            description="Minimum prefix path length required to publish safe-prefix fallback.",
        ),
        DeclareLaunchArgument(
            "allow_regressive_safe_prefix",
            default_value="true",
            description="If true, allow bounded temporary regression for safe-prefix fallback (maze detours).",
        ),
        DeclareLaunchArgument(
            "max_safe_prefix_regression_m",
            default_value="1.20",
            description="Maximum temporary goal-distance regression allowed for regressive safe-prefix fallback.",
        ),
        DeclareLaunchArgument(
            "consecutive_failures_before_hover",
            default_value="8",
            description="Number of consecutive planning failures before publishing empty path (hover).",
        ),
        DeclareLaunchArgument(
            "hover_on_planning_failure",
            default_value="true",
            description="If true, publish empty path immediately on any planning failure to avoid following stale paths.",
        ),
        DeclareLaunchArgument(
            "unknown_is_occupied",
            default_value="true",
            description="If true, planner treats unknown space as occupied (safe mapped mode). If false, unknown is allowed (explore mode).",
        ),
        DeclareLaunchArgument(
            "max_approx_goal_distance",
            default_value="0.8",
            description="Approximate solution goal error threshold in meters; <=0 accepts all approximate solutions.",
        ),
        DeclareLaunchArgument(
            "auto_takeoff",
            default_value="false",
            description="If true, run startup takeoff sequence before hovering",
        ),
        sim_launch,
        point_cloud_filter_node,
        static_tf_world_map,
        static_tf_odom_gazebo,
        rtabmap_delayed,
        esdf_node,
        rrt_node,
        path_exec_node,
        goal_from_rviz_node,
        waypoint_mission_node,
        rviz_node,
        takeoff_delayed,
    ])
