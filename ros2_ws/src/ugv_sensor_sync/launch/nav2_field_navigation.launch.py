from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _workspace_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "src" / "ugv_nav" / "ugv_nav2_adapter.py").exists():
            return parent
    raise RuntimeError("Could not find workspace root containing src/ugv_nav/ugv_nav2_adapter.py")


def generate_launch_description():
    workspace_root = _workspace_root()
    bringup_launch = workspace_root / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py"
    nav2_adapter = workspace_root / "src" / "ugv_nav" / "ugv_nav2_adapter.py"
    field_odom = workspace_root / "src" / "ugv_nav" / "ugv_field_odom_node.py"
    field_map = workspace_root / "src" / "ugv_nav" / "ugv_field_map_node.py"
    target_receiver = workspace_root / "src" / "ugv_sensor_sync" / "ugv_sensor_sync_nodes" / "uwb_node.py"
    goal_bridge = workspace_root / "src" / "ugv_nav" / "ugv_uav_goal_bridge.py"
    mission_supervisor = workspace_root / "src" / "ugv_nav" / "ugv_operation_touchdown_mission.py"
    posearray_to_cloud = workspace_root / "src" / "ugv_nav" / "ugv_posearray_to_cloud.py"
    aruco_marker = workspace_root / "src" / "ugv_perception" / "ugv_perception" / "aruco_marker_node.py"
    default_nav2_params = workspace_root / "src" / "ugv_nav" / "config" / "nav2_field_params.yaml"
    default_collision_params = workspace_root / "src" / "ugv_nav" / "config" / "collision_monitor_params.yaml"

    start_motor_controller = LaunchConfiguration("start_motor_controller")
    start_sensor_sync = LaunchConfiguration("start_sensor_sync")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_nav2_adapter = LaunchConfiguration("start_nav2_adapter")
    start_field_odom = LaunchConfiguration("start_field_odom")
    start_field_map = LaunchConfiguration("start_field_map")
    start_uav_target_receiver = LaunchConfiguration("start_uav_target_receiver")
    start_goal_bridge = LaunchConfiguration("start_goal_bridge")
    start_mission_supervisor = LaunchConfiguration("start_mission_supervisor")
    start_aruco_marker = LaunchConfiguration("start_aruco_marker")
    start_posearray_cloud = LaunchConfiguration("start_posearray_cloud")
    start_collision_monitor = LaunchConfiguration("start_collision_monitor")
    mission_cmd_vel_topic = PythonExpression(
        ["'/cmd_vel_raw' if '", start_collision_monitor, "'.lower() in ['true', '1', 'yes'] else '/cmd_vel'"]
    )

    nav2_params_file = LaunchConfiguration("nav2_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    motor_port = LaunchConfiguration("motor_port")
    motor_dry_run = LaunchConfiguration("motor_dry_run")
    lidar_port = LaunchConfiguration("lidar_port")
    zed_publish_image = LaunchConfiguration("zed_publish_image")
    zed_image_downsample_factor = LaunchConfiguration("zed_image_downsample_factor")
    field_width_m = LaunchConfiguration("field_width_m")
    field_height_m = LaunchConfiguration("field_height_m")
    field_margin_m = LaunchConfiguration("field_margin_m")
    field_map_resolution_m = LaunchConfiguration("field_map_resolution_m")
    initial_x_m = LaunchConfiguration("initial_x_m")
    initial_y_m = LaunchConfiguration("initial_y_m")
    initial_yaw_deg = LaunchConfiguration("initial_yaw_deg")
    field_odom_imu_yaw_axis = LaunchConfiguration("field_odom_imu_yaw_axis")
    field_odom_imu_yaw_sign = LaunchConfiguration("field_odom_imu_yaw_sign")
    competition_min_speed_mps = LaunchConfiguration("competition_min_speed_mps")
    competition_moving_target_speed_mps = LaunchConfiguration("competition_moving_target_speed_mps")
    competition_continuous_motion_enabled = LaunchConfiguration("competition_continuous_motion_enabled")
    competition_motion_phase_topic = LaunchConfiguration("competition_motion_phase_topic")
    allow_reverse = LaunchConfiguration("allow_reverse")
    track_width_m = LaunchConfiguration("track_width_m")
    arc_min_turn_radius_m = LaunchConfiguration("arc_min_turn_radius_m")
    arc_max_omega_radps = LaunchConfiguration("arc_max_omega_radps")
    allow_side_reverse = LaunchConfiguration("allow_side_reverse")
    field_boundary_prediction_time_s = LaunchConfiguration("field_boundary_prediction_time_s")
    uav_target_input_mode = LaunchConfiguration("uav_target_input_mode")
    uav_esp_serial_port = LaunchConfiguration("uav_esp_serial_port")
    uav_esp_serial_baud = LaunchConfiguration("uav_esp_serial_baud")
    uav_esp_serial_protocol = LaunchConfiguration("uav_esp_serial_protocol")
    uav_esp_require_checksum = LaunchConfiguration("uav_esp_require_checksum")
    uav_target_units = LaunchConfiguration("uav_target_units")
    uav_require_target_frame = LaunchConfiguration("uav_require_target_frame")
    uav_required_target_frame = LaunchConfiguration("uav_required_target_frame")
    uav_allow_boundary_projection = LaunchConfiguration("uav_allow_boundary_projection")
    uav_require_costmap_for_goal = LaunchConfiguration("uav_require_costmap_for_goal")
    uav_unknown_cost_is_blocked = LaunchConfiguration("uav_unknown_cost_is_blocked")
    aruco_allowed_marker_ids = LaunchConfiguration("aruco_allowed_marker_ids")
    aruco_dictionary = LaunchConfiguration("aruco_dictionary")
    aruco_marker_size_m = LaunchConfiguration("aruco_marker_size_m")
    destination_radius_m = LaunchConfiguration("destination_radius_m")
    terminal_stop_distance_m = LaunchConfiguration("terminal_stop_distance_m")
    terminal_forward_speed_mps = LaunchConfiguration("terminal_forward_speed_mps")
    marker_target_gate_radius_m = LaunchConfiguration("marker_target_gate_radius_m")
    lidar_x_m = LaunchConfiguration("lidar_x_m")
    lidar_y_m = LaunchConfiguration("lidar_y_m")
    lidar_z_m = LaunchConfiguration("lidar_z_m")
    lidar_roll_rad = LaunchConfiguration("lidar_roll_rad")
    lidar_pitch_rad = LaunchConfiguration("lidar_pitch_rad")
    lidar_yaw_rad = LaunchConfiguration("lidar_yaw_rad")
    zed_x_m = LaunchConfiguration("zed_x_m")
    zed_y_m = LaunchConfiguration("zed_y_m")
    zed_z_m = LaunchConfiguration("zed_z_m")
    zed_roll_rad = LaunchConfiguration("zed_roll_rad")
    zed_pitch_rad = LaunchConfiguration("zed_pitch_rad")
    zed_yaw_rad = LaunchConfiguration("zed_yaw_rad")

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={
            "global_costmap.global_costmap.ros__parameters.width": field_width_m,
            "global_costmap.global_costmap.ros__parameters.height": field_height_m,
            "global_costmap.global_costmap.ros__parameters.origin_x": "0.0",
            "global_costmap.global_costmap.ros__parameters.origin_y": "0.0",
            "global_costmap.global_costmap.ros__parameters.resolution": field_map_resolution_m,
        },
        convert_types=True,
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_launch)),
        launch_arguments={
            "start_motor_controller": start_motor_controller,
            "start_sensor_sync": start_sensor_sync,
            "start_nav": "false",
            "motor_port": motor_port,
            "motor_dry_run": motor_dry_run,
            "lidar_port": lidar_port,
            "zed_publish_image": zed_publish_image,
            "zed_image_downsample_factor": zed_image_downsample_factor,
        }.items(),
    )

    nav2_without_collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_nav2_params,
            "autostart": autostart,
        }.items(),
        condition=UnlessCondition(start_collision_monitor),
    )

    nav2_with_collision_monitor = GroupAction(
        actions=[
            # When Collision Monitor is enabled, Nav2 publishes raw velocity to
            # /cmd_vel_raw. Collision Monitor is the only publisher to /cmd_vel,
            # and ugv_nav2_adapter remains the only bridge to /ugv_nav_cmd.
            SetRemap(src="cmd_vel", dst="cmd_vel_raw"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": configured_nav2_params,
                    "autostart": autostart,
                }.items(),
            ),
        ],
        condition=IfCondition(start_collision_monitor),
    )

    field_odom_node = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(field_odom),
            "--ros-args",
            "-p",
            ["initial_x_m:=", initial_x_m],
            "-p",
            ["initial_y_m:=", initial_y_m],
            "-p",
            ["initial_yaw_deg:=", initial_yaw_deg],
            "-p",
            ["imu_yaw_axis:=", field_odom_imu_yaw_axis],
            "-p",
            ["imu_yaw_sign:=", field_odom_imu_yaw_sign],
        ],
        output="screen",
        condition=IfCondition(start_field_odom),
    )

    field_map_node = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(field_map),
            "--ros-args",
            "-p",
            ["field_width_m:=", field_width_m],
            "-p",
            ["field_height_m:=", field_height_m],
            "-p",
            ["field_margin_m:=", field_margin_m],
            "-p",
            ["resolution_m:=", field_map_resolution_m],
        ],
        output="screen",
        condition=IfCondition(start_field_map),
    )

    posearray_cloud_node = ExecuteProcess(
        cmd=[FindExecutable(name="python3"), str(posearray_to_cloud)],
        output="screen",
        condition=IfCondition(start_posearray_cloud),
    )

    nav2_adapter_node = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(nav2_adapter),
            "--ros-args",
            "-p",
            ["competition_min_speed_mps:=", competition_min_speed_mps],
            "-p",
            ["competition_moving_target_speed_mps:=", competition_moving_target_speed_mps],
            "-p",
            ["competition_continuous_motion_enabled:=", competition_continuous_motion_enabled],
            "-p",
            ["competition_motion_phase_topic:=", competition_motion_phase_topic],
            "-p",
            ["raw_cmd_vel_topic:=/cmd_vel_raw"],
            "-p",
            ["allow_reverse:=", allow_reverse],
            "-p",
            ["track_width_m:=", track_width_m],
            "-p",
            ["arc_min_turn_radius_m:=", arc_min_turn_radius_m],
            "-p",
            ["arc_max_omega_radps:=", arc_max_omega_radps],
            "-p",
            ["allow_side_reverse:=", allow_side_reverse],
            "-p",
            ["field_width_m:=", field_width_m],
            "-p",
            ["field_height_m:=", field_height_m],
            "-p",
            ["field_boundary_margin_m:=", field_margin_m],
            "-p",
            ["field_boundary_prediction_time_s:=", field_boundary_prediction_time_s],
        ],
        output="screen",
        condition=IfCondition(start_nav2_adapter),
    )

    target_receiver_node = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(target_receiver),
            "--ros-args",
            "-p",
            ["input_mode:=", uav_target_input_mode],
            "-p",
            ["target_units:=", uav_target_units],
            "-p",
            ["serial_port:=", uav_esp_serial_port],
            "-p",
            ["serial_baud:=", uav_esp_serial_baud],
            "-p",
            ["serial_protocol:=", uav_esp_serial_protocol],
            "-p",
            ["require_checksum:=", uav_esp_require_checksum],
        ],
        output="screen",
        condition=IfCondition(start_uav_target_receiver),
    )

    goal_bridge_node = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(goal_bridge),
            "--ros-args",
            "-p",
            ["field_width_m:=", field_width_m],
            "-p",
            ["field_height_m:=", field_height_m],
            "-p",
            ["field_margin_m:=", field_margin_m],
            "-p",
            ["target_units:=", uav_target_units],
            "-p",
            ["require_target_frame:=", uav_require_target_frame],
            "-p",
            ["required_target_frame:=", uav_required_target_frame],
            "-p",
            ["allow_boundary_projection:=", uav_allow_boundary_projection],
            "-p",
            ["require_costmap_for_goal:=", uav_require_costmap_for_goal],
            "-p",
            ["unknown_cost_is_blocked:=", uav_unknown_cost_is_blocked],
        ],
        output="screen",
        condition=IfCondition(start_goal_bridge),
    )

    mission_supervisor_node = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(mission_supervisor),
            "--ros-args",
            "-p",
            ["field_width_m:=", field_width_m],
            "-p",
            ["field_height_m:=", field_height_m],
            "-p",
            ["field_margin_m:=", field_margin_m],
            "-p",
            ["target_units:=", uav_target_units],
            "-p",
            ["require_target_frame:=", uav_require_target_frame],
            "-p",
            ["required_target_frame:=", uav_required_target_frame],
            "-p",
            ["require_costmap_for_goal:=", uav_require_costmap_for_goal],
            "-p",
            ["unknown_cost_is_blocked:=", uav_unknown_cost_is_blocked],
            "-p",
            ["destination_radius_m:=", destination_radius_m],
            "-p",
            ["terminal_stop_distance_m:=", terminal_stop_distance_m],
            "-p",
            ["terminal_forward_speed_mps:=", terminal_forward_speed_mps],
            "-p",
            ["competition_moving_target_speed_mps:=", competition_moving_target_speed_mps],
            "-p",
            ["competition_motion_phase_topic:=", competition_motion_phase_topic],
            "-p",
            ["marker_target_gate_radius_m:=", marker_target_gate_radius_m],
            "-p",
            ["cmd_vel_topic:=", mission_cmd_vel_topic],
        ],
        output="screen",
        condition=IfCondition(start_mission_supervisor),
    )

    aruco_marker_node = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(aruco_marker),
            "--ros-args",
            "-p",
            ["dictionary:=", aruco_dictionary],
            "-p",
            ["allowed_marker_ids:=", aruco_allowed_marker_ids],
            "-p",
            ["marker_size_m:=", aruco_marker_size_m],
            "-p",
            ["camera_x_m:=", zed_x_m],
            "-p",
            ["camera_y_m:=", zed_y_m],
            "-p",
            ["camera_yaw_offset_rad:=", zed_yaw_rad],
        ],
        output="screen",
        condition=IfCondition(start_aruco_marker),
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[collision_monitor_params_file],
        condition=IfCondition(start_collision_monitor),
    )

    collision_monitor_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["collision_monitor"],
            }
        ],
        condition=IfCondition(start_collision_monitor),
    )

    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        arguments=[
            "--x",
            lidar_x_m,
            "--y",
            lidar_y_m,
            "--z",
            lidar_z_m,
            "--roll",
            lidar_roll_rad,
            "--pitch",
            lidar_pitch_rad,
            "--yaw",
            lidar_yaw_rad,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar",
        ],
    )
    zed_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed_imu_tf",
        arguments=[
            "--x",
            zed_x_m,
            "--y",
            zed_y_m,
            "--z",
            zed_z_m,
            "--roll",
            zed_roll_rad,
            "--pitch",
            zed_pitch_rad,
            "--yaw",
            zed_yaw_rad,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "zed_imu",
        ],
    )
    zed_depth_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed_depth_tf",
        arguments=[
            "--x",
            zed_x_m,
            "--y",
            zed_y_m,
            "--z",
            zed_z_m,
            "--roll",
            zed_roll_rad,
            "--pitch",
            zed_pitch_rad,
            "--yaw",
            zed_yaw_rad,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "zed_depth",
        ],
    )
    zed_left_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed_left_tf",
        arguments=[
            "--x",
            zed_x_m,
            "--y",
            zed_y_m,
            "--z",
            zed_z_m,
            "--roll",
            zed_roll_rad,
            "--pitch",
            zed_pitch_rad,
            "--yaw",
            zed_yaw_rad,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "zed_left",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_motor_controller", default_value="true"),
            DeclareLaunchArgument("start_sensor_sync", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_nav2_adapter", default_value="true"),
            DeclareLaunchArgument("start_field_odom", default_value="true"),
            DeclareLaunchArgument("start_field_map", default_value="true"),
            DeclareLaunchArgument("start_uav_target_receiver", default_value="false"),
            DeclareLaunchArgument("start_goal_bridge", default_value="false"),
            DeclareLaunchArgument("start_mission_supervisor", default_value="true"),
            DeclareLaunchArgument("start_aruco_marker", default_value="true"),
            DeclareLaunchArgument("start_posearray_cloud", default_value="true"),
            DeclareLaunchArgument("start_collision_monitor", default_value="true"),
            DeclareLaunchArgument("nav2_params_file", default_value=str(default_nav2_params)),
            DeclareLaunchArgument("collision_monitor_params_file", default_value=str(default_collision_params)),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("motor_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("motor_dry_run", default_value="false"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("zed_publish_image", default_value="true"),
            DeclareLaunchArgument("zed_image_downsample_factor", default_value="1"),
            DeclareLaunchArgument("field_width_m", default_value="13.716"),
            DeclareLaunchArgument("field_height_m", default_value="13.716"),
            DeclareLaunchArgument("field_margin_m", default_value="0.45"),
            DeclareLaunchArgument("field_map_resolution_m", default_value="0.05"),
            DeclareLaunchArgument("initial_x_m", default_value="0.0"),
            DeclareLaunchArgument("initial_y_m", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("field_odom_imu_yaw_axis", default_value="y"),
            DeclareLaunchArgument("field_odom_imu_yaw_sign", default_value="-1.0"),
            DeclareLaunchArgument("competition_min_speed_mps", default_value="0.0894"),
            DeclareLaunchArgument("competition_moving_target_speed_mps", default_value="0.12"),
            DeclareLaunchArgument("competition_continuous_motion_enabled", default_value="true"),
            DeclareLaunchArgument("competition_motion_phase_topic", default_value="/ugv/competition_motion_phase"),
            DeclareLaunchArgument("allow_reverse", default_value="false"),
            DeclareLaunchArgument("track_width_m", default_value="0.416"),
            DeclareLaunchArgument("arc_min_turn_radius_m", default_value="0.75"),
            DeclareLaunchArgument("arc_max_omega_radps", default_value="0.45"),
            DeclareLaunchArgument("allow_side_reverse", default_value="false"),
            DeclareLaunchArgument("field_boundary_prediction_time_s", default_value="0.75"),
            DeclareLaunchArgument("uav_target_input_mode", default_value="serial"),
            DeclareLaunchArgument("uav_esp_serial_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("uav_esp_serial_baud", default_value="115200"),
            DeclareLaunchArgument("uav_esp_serial_protocol", default_value="binary14"),
            DeclareLaunchArgument("uav_esp_require_checksum", default_value="false"),
            DeclareLaunchArgument("uav_target_units", default_value="meters"),
            DeclareLaunchArgument("uav_require_target_frame", default_value="true"),
            DeclareLaunchArgument("uav_required_target_frame", default_value="map"),
            DeclareLaunchArgument("uav_allow_boundary_projection", default_value="false"),
            DeclareLaunchArgument("uav_require_costmap_for_goal", default_value="true"),
            DeclareLaunchArgument("uav_unknown_cost_is_blocked", default_value="true"),
            DeclareLaunchArgument("aruco_dictionary", default_value="DICT_6X6_250"),
            DeclareLaunchArgument("aruco_allowed_marker_ids", default_value="0,1,2,3,4"),
            DeclareLaunchArgument("aruco_marker_size_m", default_value="0.3048"),
            DeclareLaunchArgument("destination_radius_m", default_value="1.524"),
            DeclareLaunchArgument("terminal_stop_distance_m", default_value="1.0"),
            DeclareLaunchArgument("terminal_forward_speed_mps", default_value="0.12"),
            DeclareLaunchArgument("marker_target_gate_radius_m", default_value="2.274"),
            DeclareLaunchArgument("lidar_x_m", default_value="0.0"),
            DeclareLaunchArgument("lidar_y_m", default_value="0.0"),
            DeclareLaunchArgument("lidar_z_m", default_value="0.18"),
            DeclareLaunchArgument("lidar_roll_rad", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch_rad", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw_rad", default_value="0.0"),
            DeclareLaunchArgument("zed_x_m", default_value="0.0"),
            DeclareLaunchArgument("zed_y_m", default_value="0.0"),
            DeclareLaunchArgument("zed_z_m", default_value="0.35"),
            DeclareLaunchArgument("zed_roll_rad", default_value="0.0"),
            DeclareLaunchArgument("zed_pitch_rad", default_value="0.0"),
            DeclareLaunchArgument("zed_yaw_rad", default_value="0.0"),
            LogInfo(
                msg=(
                    "[WARN] Verify sensor extrinsics before autonomous Nav2 ground tests; "
                    "default LiDAR/ZED x/y/yaw values are placeholders until measured on the robot."
                )
            ),
            bringup,
            field_odom_node,
            field_map_node,
            posearray_cloud_node,
            lidar_tf,
            zed_imu_tf,
            zed_depth_tf,
            zed_left_tf,
            GroupAction(
                actions=[nav2_without_collision_monitor],
                condition=IfCondition(start_nav2),
            ),
            GroupAction(
                actions=[nav2_with_collision_monitor],
                condition=IfCondition(start_nav2),
            ),
            collision_monitor,
            collision_monitor_lifecycle,
            nav2_adapter_node,
            target_receiver_node,
            aruco_marker_node,
            mission_supervisor_node,
            goal_bridge_node,
        ]
    )
