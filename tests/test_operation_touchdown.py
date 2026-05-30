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
    assert decision.command.v_mps == pytest.approx(0.089408)
    assert decision.command.motion_rule_ok


def test_terminal_approach_pivots_for_large_heading_error_and_stops_when_lost():
    pivot = terminal_approach_command(
        robot_pose=Transform2D(0.0, 0.0, 0.0),
        marker_x_m=0.0,
        marker_y_m=2.0,
        marker_fresh=True,
    )
    assert pivot.command.command_type == "velocity"
    assert pivot.command.v_mps == pytest.approx(0.0)
    assert abs(pivot.command.omega_radps) > 0.0
    assert pivot.reason == "terminal_align_to_marker"

    lost = terminal_approach_command(
        robot_pose=Transform2D(0.0, 0.0, 0.0),
        marker_x_m=3.0,
        marker_y_m=0.0,
        marker_fresh=False,
    )
    assert lost.command.command_type == "stop"
    assert lost.reason == "marker_lost"


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


def test_mission_supervisor_does_not_publish_stop_while_nav2_owns_cmd_vel_raw():
    mission_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_operation_touchdown_mission.py").read_text()
    assert 'elif self.state in {"destination_stop", "mission_complete", "abort", "wait_for_uav_target"}' in mission_text
    assert '"nav2_to_staging"}:' not in mission_text
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
    assert "CameraInfo" in zed_text
    assert "camera_info_topic" in zed_text
    assert "/zed/left/camera_info" in zed_text
    assert "_camera_info_for_image" in zed_text


def test_aruco_detector_uses_full_tf_projection_not_yaw_only_projection_by_default():
    aruco_text = (ROOT / "ros2_ws" / "src" / "ugv_perception" / "ugv_perception" / "aruco_marker_node.py").read_text()
    assert "TransformListener" in aruco_text
    assert "lookup_transform" in aruco_text
    assert 'self.declare_parameter("allow_planar_projection_fallback", False)' in aruco_text
    assert "_transform_point" in aruco_text
    assert "msg_out.header.frame_id = self.map_frame" in aruco_text


def test_nav2_adapter_has_explicit_kill_switch_hard_stop():
    adapter_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav2_adapter.py").read_text()
    assert 'self.declare_parameter("kill_switch_topic", "/ugv/kill_switch")' in adapter_text
    assert "kill_switch_active" in adapter_text
    assert 'return "kill_switch"' in adapter_text
