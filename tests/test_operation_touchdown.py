import math
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.nav2_bridge import FieldBounds, OccupancyGridSpec, Transform2D  # noqa: E402
from ugv_nav_core.operation_touchdown import (  # noqa: E402
    ARUCO_MARKER_SIDE_M,
    DEFAULT_ALLOWED_ARUCO_IDS,
    DESTINATION_RADIUS_M,
    choose_marker_staging_pose,
    coordinate_arrival_decision,
    destination_reached_by_coordinate,
    footprint_allows_pose,
    parse_allowed_marker_ids,
    terminal_approach_command,
)


def test_operation_touchdown_rule_constants_match_rulebook():
    assert ARUCO_MARKER_SIDE_M == pytest.approx(0.3048)
    assert DESTINATION_RADIUS_M == pytest.approx(1.524)
    assert DEFAULT_ALLOWED_ARUCO_IDS == (0, 1, 2, 3, 4)


def test_parse_allowed_marker_ids_keeps_competition_defaults_and_rejects_noise():
    assert parse_allowed_marker_ids(None) == (0, 1, 2, 3, 4)
    assert parse_allowed_marker_ids("4,2,2,bad,0") == (0, 2, 4)
    assert parse_allowed_marker_ids("") == (0, 1, 2, 3, 4)


def test_destination_reached_by_coordinate_uses_five_foot_radius_with_buffer():
    assert destination_reached_by_coordinate(
        robot_x_m=1.0,
        robot_y_m=1.0,
        target_x_m=2.0,
        target_y_m=1.0,
        destination_radius_m=DESTINATION_RADIUS_M,
        buffer_m=0.20,
    )
    assert not destination_reached_by_coordinate(
        robot_x_m=0.0,
        robot_y_m=0.0,
        target_x_m=1.4,
        target_y_m=0.0,
        destination_radius_m=DESTINATION_RADIUS_M,
        buffer_m=0.20,
    )


def test_coordinate_arrival_decision_stops_by_coordinate_before_visual_refinement():
    no_visual = coordinate_arrival_decision(
        robot_pose=Transform2D(0.0, 0.0, 0.0),
        target_x_m=1.0,
        target_y_m=0.0,
        marker_confirmed=False,
        destination_radius_m=DESTINATION_RADIUS_M,
        buffer_m=0.20,
    )
    assert no_visual.destination_reached
    assert no_visual.reason == "destination_reached_by_coordinate_no_marker_visual"
    assert no_visual.distance_to_target_m == pytest.approx(1.0)

    confirmed = coordinate_arrival_decision(
        robot_pose=Transform2D(0.0, 0.0, 0.0),
        target_x_m=1.0,
        target_y_m=0.0,
        marker_confirmed=True,
        destination_radius_m=DESTINATION_RADIUS_M,
        buffer_m=0.20,
    )
    assert confirmed.destination_reached
    assert confirmed.reason == "destination_reached_coordinate_marker_confirmed"


def test_choose_marker_staging_pose_is_inside_destination_circle_and_faces_marker():
    bounds = FieldBounds(width_m=13.716, height_m=13.716, margin_m=0.45)
    robot = Transform2D(2.0, 2.0, 0.0)
    decision = choose_marker_staging_pose(
        robot_pose=robot,
        marker_x_m=7.0,
        marker_y_m=7.0,
        bounds=bounds,
        occupancy_grid=None,
        destination_radius_m=DESTINATION_RADIUS_M,
    )
    assert decision.accepted
    assert decision.pose is not None
    assert math.hypot(decision.pose.x_m - 7.0, decision.pose.y_m - 7.0) <= DESTINATION_RADIUS_M
    desired_yaw = math.atan2(7.0 - decision.pose.y_m, 7.0 - decision.pose.x_m)
    assert decision.pose.yaw_rad == pytest.approx(desired_yaw)


def test_choose_marker_staging_pose_rejects_when_candidates_are_occupied():
    # A 3x3 grid around the marker with all cells occupied leaves no legal staging pose.
    grid = OccupancyGridSpec(
        width=3,
        height=3,
        resolution_m=1.0,
        origin_x_m=5.5,
        origin_y_m=5.5,
        data=[100] * 9,
    )
    bounds = FieldBounds(width_m=13.716, height_m=13.716, margin_m=0.45)
    decision = choose_marker_staging_pose(
        robot_pose=Transform2D(2.0, 2.0, 0.0),
        marker_x_m=7.0,
        marker_y_m=7.0,
        bounds=bounds,
        occupancy_grid=grid,
    )
    assert not decision.accepted
    assert decision.reason == "no_safe_marker_staging_pose"


def test_footprint_pose_check_rejects_obstacle_inside_robot_radius():
    grid = OccupancyGridSpec(
        width=5,
        height=5,
        resolution_m=0.25,
        origin_x_m=0.0,
        origin_y_m=0.0,
        data=[0] * 25,
    )
    data = list(grid.data)
    data[2 * grid.width + 4] = 100
    blocked = OccupancyGridSpec(
        width=grid.width,
        height=grid.height,
        resolution_m=grid.resolution_m,
        origin_x_m=grid.origin_x_m,
        origin_y_m=grid.origin_y_m,
        data=data,
    )
    assert not footprint_allows_pose(0.55, 0.55, blocked, radius_m=0.45, samples=8)


def test_terminal_approach_command_never_emits_sub_min_forward_speed():
    decision = terminal_approach_command(
        robot_pose=Transform2D(0.0, 0.0, 0.0),
        marker_x_m=2.0,
        marker_y_m=0.05,
        marker_fresh=True,
        forward_speed_mps=0.01,
    )
    assert decision.command.command_type == "velocity"
    assert decision.command.v_mps == pytest.approx(0.12)
    assert decision.command.motion_rule_ok


def test_terminal_approach_uses_arc_crawl_for_large_heading_error_and_coordinate_fallback_when_lost():
    hold = terminal_approach_command(
        robot_pose=Transform2D(0.0, 0.0, 0.0),
        marker_x_m=0.0,
        marker_y_m=2.0,
        marker_fresh=True,
    )
    assert hold.command.command_type == "velocity"
    assert hold.command.v_mps == pytest.approx(0.12)
    assert abs(hold.command.omega_radps) > 0.0
    assert hold.reason == "terminal_heading_arc_crawl"

    lost = terminal_approach_command(
        robot_pose=Transform2D(0.0, 0.0, 0.0),
        marker_x_m=3.0,
        marker_y_m=0.0,
        marker_fresh=False,
    )
    assert lost.command.command_type == "velocity"
    assert lost.command.v_mps == pytest.approx(0.12)
    assert lost.reason == "terminal_approach_coordinate_fallback"


def test_terminal_approach_stops_inside_destination_radius():
    decision = terminal_approach_command(
        robot_pose=Transform2D(0.0, 0.0, 0.0),
        marker_x_m=1.0,
        marker_y_m=0.0,
        marker_fresh=False,
        destination_radius_m=DESTINATION_RADIUS_M,
        terminal_arrival_buffer_m=0.20,
    )
    assert decision.destination_reached
    assert decision.command.command_type == "stop"
    assert decision.reason == "destination_reached"


def test_nav2_launch_defaults_to_mission_supervisor_and_aruco_not_direct_goal_bridge():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()
    assert 'DeclareLaunchArgument("start_mission_supervisor", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("start_aruco_marker", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("start_goal_bridge", default_value="false")' in launch_text
    assert "ugv_operation_touchdown_mission.py" in launch_text
    assert "aruco_marker_node.py" in launch_text
    assert 'DeclareLaunchArgument("zed_publish_image", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("marker_target_gate_radius_m", default_value="2.274")' in launch_text
    assert "coordinate_arrival_marker_search_s" not in launch_text
    assert "terminal_replan_max_attempts" not in launch_text
    assert 'DeclareLaunchArgument("competition_motion_phase_topic", default_value="/ugv/competition_motion_phase")' in launch_text
    assert 'DeclareLaunchArgument("competition_moving_target_speed_mps", default_value="0.12")' in launch_text


def test_mission_supervisor_does_not_publish_stop_while_nav2_owns_cmd_vel_raw():
    mission_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_operation_touchdown_mission.py").read_text()
    timer_text = mission_text[mission_text.index("    def timer_callback") : mission_text.index("    def _coordinate_terminal_gate")]

    assert 'elif self.state in {"destination_stop", "mission_complete", "abort", "wait_for_uav_target"}' in timer_text
    assert '"nav2_goal_pending"' not in timer_text
    assert '"nav2_to_staging"' not in timer_text
    assert "cancel_goal_async" in mission_text
    assert "goal_sequence" in mission_text


def test_mission_supervisor_cmd_vel_topic_matches_collision_monitor_chain():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()
    assert "PythonExpression" in launch_text
    assert "'/cmd_vel_raw' if" in launch_text
    assert "else '/cmd_vel'" in launch_text
    assert '["cmd_vel_topic:=", mission_cmd_vel_topic]' in launch_text


def test_mission_supervisor_gates_marker_detections_against_uav_target():
    mission_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_operation_touchdown_mission.py").read_text()
    assert 'self.declare_parameter("marker_target_gate_radius_m", 2.274)' in mission_text
    assert '"marker_target_disagreement"' in mission_text
    assert '"marker_without_uav_target"' in mission_text


def test_zed_sync_publishes_camera_info_for_aruco_pnp():
    zed_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "ugv_sensor_sync_nodes" / "zed_sync_node.py").read_text()
    sensor_launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "sensor_sync_launch.py").read_text()
    nav2_launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()

    assert "CameraInfo" in zed_text
    assert "camera_info_topic" in zed_text
    assert "/zed/left/camera_info" in zed_text
    assert "_camera_info_for_image" in zed_text
    assert "image_downsample_factor" in zed_text
    assert "self.image_downsample_factor" in zed_text
    assert "self.depth_downsample_factor" in zed_text
    assert "if depth_np.ndim == 3:\n            depth_np = depth_np[:, :, 0]\n        if self.depth_downsample_factor > 1:" in zed_text
    assert "left_np[::self.image_downsample_factor, ::self.image_downsample_factor]" in zed_text
    assert "def poll_camera" in zed_text
    assert "self.zed.grab(self.runtime)" in zed_text
    assert zed_text.index("self.zed.grab(self.runtime)") < zed_text.index("self.publish_imu()")
    assert "self.publish_imu()" in zed_text
    assert "self.grab_frame()" in zed_text
    assert "_should_publish_depth_or_image" in zed_text
    assert "publish_depth_without_subscribers" in zed_text
    assert "camera_grab_rate_hz" in zed_text
    assert "MultiThreadedExecutor" not in zed_text
    assert "MutuallyExclusiveCallbackGroup" not in zed_text
    assert "acquire(blocking=False)" not in zed_text
    assert "math.radians(value)" in zed_text
    assert "last_imu_ang_degps" in zed_text
    assert "imu_rate_hz" in zed_text
    assert "imu_age_s" in zed_text
    assert "imu_busy_skips" in zed_text
    assert "last_imu_publish_s" in zed_text
    assert "zed_image_downsample_factor" in sensor_launch_text
    assert "start_debug_status" in sensor_launch_text
    assert "debug_status_node.py" in sensor_launch_text
    assert 'DeclareLaunchArgument("zed_image_downsample_factor", default_value="1")' in nav2_launch_text


def test_fusion_depth_blind_hazard_does_not_override_clear_lidar():
    fusion_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "ugv_sensor_sync_nodes" / "fusion_node.py").read_text()
    nav_frame_msg = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "msg" / "NavSensorFrame.msg").read_text()

    assert "lidar_front_clear" in fusion_text
    assert "depth_blind_hazard_active = depth_blind_hazard and not lidar_front_clear" in fusion_text
    assert "near_obstacle = depth_blind_hazard_active or (" in fusion_text
    assert "bool depth_blind_hazard" in nav_frame_msg
    assert "string front_clearance_source" in nav_frame_msg
    assert "float32 front_lidar_range_m" in nav_frame_msg


def test_fusion_summary_exposes_stop_first_obstacle_debug_fields():
    fusion_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "ugv_sensor_sync_nodes" / "fusion_node.py").read_text()

    assert "FRONT_OBSTACLE_DEBUG_THRESHOLD_M = 2.0" in fusion_text
    assert "FRONT_STOP_DEBUG_CLEARANCE_M = 1.0" in fusion_text
    assert "'front_obstacle_within_2m': bool(front_obstacle_within_2m)" in fusion_text
    assert "'front_stop_required_1m': bool(front_stop_required_1m)" in fusion_text
    assert "'front_lidar_fov_deg': round(float(self.lidar_front_fov_deg), 3)" in fusion_text
    assert "'front_lidar_min_cluster_points': int(self.lidar_front_min_cluster_points)" in fusion_text
    assert "'front_lidar_cluster_max_gap_m': round(float(self.lidar_front_cluster_max_gap_m), 3)" in fusion_text
    assert "'depth_corridor_half_width_m': round(float(self.depth_front_corridor_half_width_m), 3)" in fusion_text
    assert "'lidar_any_min_range_m': self._finite_or_none(lidar_min_range_m)" in fusion_text
    assert "'front_sensor_health': front_sensor_health" in fusion_text


def test_aruco_detector_uses_full_tf_projection_not_yaw_only_projection_by_default():
    aruco_text = (ROOT / "ros2_ws" / "src" / "ugv_perception" / "ugv_perception" / "aruco_marker_node.py").read_text()
    assert "TransformListener" in aruco_text
    assert "lookup_transform" in aruco_text
    assert 'self.declare_parameter("allow_planar_projection_fallback", False)' in aruco_text
    assert 'self.declare_parameter("pnp_tvec_frame", "optical")' in aruco_text
    assert "camera_forward_m = z_opt" in aruco_text
    assert "camera_left_m = -x_opt" in aruco_text
    assert "camera_up_m = -y_opt" in aruco_text
    assert "_transform_point" in aruco_text
    assert "msg_out.header.frame_id = self.map_frame" in aruco_text


def test_mission_supervisor_global_coordinate_gate_stops_even_during_nav2_stage():
    mission_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_operation_touchdown_mission.py").read_text()

    assert "def _coordinate_terminal_gate" in mission_text
    gate_text = mission_text[
        mission_text.index("    def _coordinate_terminal_gate") : mission_text.index("    def _plan_or_wait_for_staging")
    ]
    assert '"nav2_to_staging"' in gate_text
    assert "_cancel_active_goal(decision.reason)" in gate_text
    assert "_set_state(\"destination_stop\", decision.reason)" in gate_text
    assert "_publish_stop(decision.reason)" in gate_text
    assert '"marker_search_coordinate_hold"' not in mission_text


def test_mission_supervisor_makes_coordinate_radius_a_hard_terminal_stop():
    mission_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_operation_touchdown_mission.py").read_text()
    timer_text = mission_text[mission_text.index("    def timer_callback") : mission_text.index("    def _coordinate_terminal_gate")]
    gate_text = mission_text[
        mission_text.index("    def _coordinate_terminal_gate") : mission_text.index("    def _plan_or_wait_for_staging")
    ]
    terminal_step = mission_text[mission_text.index("    def _terminal_step") : mission_text.index("    def _marker_fresh")]

    assert "if self._coordinate_terminal_gate(now_s):" in timer_text
    assert "coordinate_arrival_decision(" in gate_text
    assert "if not decision.destination_reached:" in gate_text
    assert "coordinate_arrival_decision(" in terminal_step
    assert "if coordinate_decision.destination_reached:" in terminal_step
    assert "terminal_approach_command(" in terminal_step
    assert terminal_step.index("if coordinate_decision.destination_reached:") < terminal_step.index(
        "terminal_approach_command("
    )
    assert "_request_terminal_replan" not in mission_text
    assert "terminal_replan_limit_exceeded" not in mission_text
    assert '"coordinate_distance_to_target_m"' in mission_text
    assert '"coordinate_destination_reached"' in mission_text
    assert '"visual_marker_used_for_motion"' in mission_text
    assert '"marker_target_disagreement_m"' in mission_text


def test_mission_supervisor_publishes_motion_phase_and_crawls_during_active_terminal_states():
    mission_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_operation_touchdown_mission.py").read_text()

    assert 'self.declare_parameter("competition_motion_phase_topic", "/ugv/competition_motion_phase")' in mission_text
    assert "self.phase_pub = self.create_publisher" in mission_text
    assert '"path_following"' in mission_text
    assert '"marker_search"' in mission_text
    assert '"terminal_approach"' in mission_text
    assert "_publish_coordinate_crawl" in mission_text
    assert "marker_search_coordinate_crawl" in mission_text
    assert "marker_search_timeout_coordinate_fallback" in mission_text
    assert '"destination_reached"' in mission_text


def test_mission_supervisor_does_not_start_active_motion_until_nav2_goal_is_accepted():
    mission_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_operation_touchdown_mission.py").read_text()
    send_goal = mission_text[mission_text.index("    def _send_nav2_goal") : mission_text.index("    def _goal_response_callback")]
    goal_response = mission_text[
        mission_text.index("    def _goal_response_callback") : mission_text.index("    def _goal_result_callback")
    ]
    phase_update = mission_text[
        mission_text.index("    def _update_motion_phase_for_state") : mission_text.index("    def _publish_motion_phase")
    ]

    assert 'self._set_state("nav2_goal_pending", "nav2_goal_sent")' in send_goal
    assert "_mark_run_started()" not in send_goal
    assert "if not handle.accepted:" in goal_response
    assert goal_response.index("if not handle.accepted:") < goal_response.index("self._mark_run_started()")
    assert goal_response.index("self._mark_run_started()") < goal_response.index('self._set_state("nav2_to_staging", "nav2_goal_accepted")')
    assert '"nav2_goal_pending"} and self.ugv_start_time_s is None' in phase_update


def test_nav2_adapter_waits_for_first_active_cmd_vel_before_timeout_crawl():
    adapter_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav2_adapter.py").read_text()
    timer_text = adapter_text[adapter_text.index("    def timer_callback") : adapter_text.index("    def _measured_speed_mps")]

    assert "self.active_cmd_vel_seen = False" in adapter_text
    assert "self.first_active_cmd_vel_s" in adapter_text
    assert "and self.active_cmd_vel_seen" in timer_text


def test_nav2_adapter_has_explicit_kill_switch_hard_stop():
    adapter_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav2_adapter.py").read_text()
    assert 'self.declare_parameter("kill_switch_topic", "/ugv/kill_switch")' in adapter_text
    assert "kill_switch_active" in adapter_text
    assert 'return "kill_switch"' in adapter_text
