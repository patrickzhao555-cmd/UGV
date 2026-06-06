import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
UGV_NAV_SRC = ROOT / "ros2_ws" / "src" / "ugv_nav"
if str(UGV_NAV_SRC) not in sys.path:
    sys.path.insert(0, str(UGV_NAV_SRC))

from ugv_nav_core.challenge3_corridor import (  # noqa: E402
    CHALLENGE3_DESTINATION_RADIUS_M,
    CHALLENGE3_TURN_ENVELOPE_OMEGA_RADPS,
    CHALLENGE3_TURN_ENVELOPE_SPEED_MPS,
    Challenge3CorridorConfig,
    Challenge3CorridorState,
    FieldPose,
    ObstacleMemoryPoint,
    RouteObservation,
    centerline_obstacles_passed,
    forward_arc_command,
    lane_offset_at_u,
    make_route_frame,
    project_field_to_route,
    robot_point_to_route,
    scan_ranges_to_route_observations,
    select_lane_offset,
    step_corridor_controller,
    update_obstacle_memory,
)


def _route():
    return make_route_frame(start_x_m=1.0, start_y_m=2.0, target_x_m=5.0, target_y_m=2.0)


def _memory(u_m, v_m, *, now_s=0.0):
    return ObstacleMemoryPoint(
        u_m=float(u_m),
        v_m=float(v_m),
        field_x_m=float(u_m),
        field_y_m=float(v_m),
        first_seen_s=float(now_s),
        last_seen_s=float(now_s),
    )


def test_route_projection_uses_start_target_baseline():
    point = project_field_to_route(_route(), x_m=3.0, y_m=3.0)

    assert point.u_m == pytest.approx(2.0)
    assert point.v_m == pytest.approx(1.0)


def test_robot_lidar_point_transforms_to_route_frame():
    obs = robot_point_to_route(
        FieldPose(x_m=1.0, y_m=2.0, yaw_rad=0.0),
        _route(),
        forward_m=2.0,
        left_m=1.0,
    )

    assert obs.u_m == pytest.approx(2.0)
    assert obs.v_m == pytest.approx(1.0)
    assert obs.field_x_m == pytest.approx(3.0)
    assert obs.field_y_m == pytest.approx(3.0)


def test_scan_observations_ignore_isolated_lidar_return_by_default():
    observations = scan_ranges_to_route_observations(
        ranges=[float("inf"), 1.0, float("inf")],
        angle_min_rad=math.radians(-1.0),
        angle_increment_rad=math.radians(1.0),
        range_min_m=0.05,
        range_max_m=8.0,
        pose=FieldPose(x_m=1.0, y_m=2.0, yaw_rad=0.0),
        route=_route(),
        max_range_m=5.5,
    )

    assert observations == []


def test_scan_observations_accept_supported_lidar_cluster():
    observations = scan_ranges_to_route_observations(
        ranges=[float("inf"), 1.10, 1.00, 1.05, float("inf")],
        angle_min_rad=math.radians(-2.0),
        angle_increment_rad=math.radians(1.0),
        range_min_m=0.05,
        range_max_m=8.0,
        pose=FieldPose(x_m=1.0, y_m=2.0, yaw_rad=0.0),
        route=_route(),
        max_range_m=5.5,
        min_cluster_points=3,
        cluster_max_gap_m=0.35,
    )

    assert len(observations) == 3
    assert min(obs.range_m for obs in observations) == pytest.approx(1.0)


def test_centerline_obstacle_triggers_lane_change():
    route = make_route_frame(start_x_m=1.0, start_y_m=4.0, target_x_m=11.0, target_y_m=4.0)
    state = Challenge3CorridorState(mode="TRACK_BASELINE")
    obs = RouteObservation(
        u_m=3.0,
        v_m=0.0,
        field_x_m=3.0,
        field_y_m=4.0,
        robot_forward_m=3.0,
        robot_left_m=0.0,
        range_m=3.0,
    )

    step = step_corridor_controller(
        state,
        route=route,
        pose=FieldPose(x_m=1.0, y_m=4.0, yaw_rad=0.0),
        now_s=1.0,
        observations=[obs],
        config=Challenge3CorridorConfig(),
    )

    assert step.command.command_type == "velocity"
    assert state.mode == "ENTER_LANE"
    assert abs(state.desired_lane_offset_m) == pytest.approx(1.6)


def test_lane_selection_prefers_side_with_better_clearance():
    route = make_route_frame(start_x_m=1.0, start_y_m=4.0, target_x_m=11.0, target_y_m=4.0)
    obstacles = [_memory(3.0, 0.0), _memory(3.2, -1.6)]

    selection = select_lane_offset(
        route,
        current_u_m=0.0,
        current_offset_m=0.0,
        obstacles=obstacles,
        config=Challenge3CorridorConfig(),
    )

    assert selection.offset_m == pytest.approx(1.6)
    assert selection.reason == "lane_change_selected"


def test_lane_selection_avoids_field_boundary():
    route = make_route_frame(start_x_m=1.0, start_y_m=2.0, target_x_m=11.0, target_y_m=2.0)
    obstacles = [_memory(3.0, 0.0)]
    config = Challenge3CorridorConfig(field_width_m=12.0, field_height_m=3.5, field_margin_m=0.2)

    selection = select_lane_offset(
        route,
        current_u_m=0.0,
        current_offset_m=0.0,
        obstacles=obstacles,
        config=config,
    )

    assert selection.offset_m < 0.0
    assert selection.offset_m == pytest.approx(-1.6)


def test_obstacle_memory_persists_until_confirmed_passed():
    state = Challenge3CorridorState()
    config = Challenge3CorridorConfig(obstacle_memory_ttl_s=0.5)
    obs = RouteObservation(
        u_m=3.0,
        v_m=0.0,
        field_x_m=3.0,
        field_y_m=0.0,
        robot_forward_m=3.0,
        robot_left_m=0.0,
        range_m=3.0,
    )

    update_obstacle_memory(state, observations=[obs], current_u_m=0.0, now_s=0.0, config=config)
    update_obstacle_memory(state, observations=[], current_u_m=1.0, now_s=10.0, config=config)

    assert len(state.obstacles) == 1
    assert not centerline_obstacles_passed(current_u_m=1.0, obstacles=state.obstacles, config=config)

    update_obstacle_memory(state, observations=[], current_u_m=5.0, now_s=11.0, config=config)

    assert state.obstacles == []


def test_rejoin_begins_only_after_obstacle_is_behind():
    route = make_route_frame(start_x_m=1.0, start_y_m=4.0, target_x_m=11.0, target_y_m=4.0)
    state = Challenge3CorridorState(
        mode="HOLD_LANE",
        desired_lane_offset_m=1.6,
        lane_change_start_u_m=0.0,
        lane_change_start_offset_m=1.6,
        lane_change_distance_m=5.0,
        obstacles=[_memory(2.0, 0.0)],
    )
    config = Challenge3CorridorConfig()

    step = step_corridor_controller(
        state,
        route=route,
        pose=FieldPose(x_m=5.0, y_m=5.6, yaw_rad=0.0),
        now_s=2.0,
        observations=[],
        config=config,
    )

    assert step.command.command_type == "velocity"
    assert state.mode == "REJOIN_BASELINE"
    assert state.desired_lane_offset_m == pytest.approx(0.0)
    assert lane_offset_at_u(state, 4.0) == pytest.approx(1.6)


def test_destination_circle_stops_without_aruco_dependency():
    route = make_route_frame(start_x_m=1.0, start_y_m=1.0, target_x_m=10.0, target_y_m=1.0)
    state = Challenge3CorridorState(mode="TRACK_BASELINE")

    step = step_corridor_controller(
        state,
        route=route,
        pose=FieldPose(x_m=10.0 - (CHALLENGE3_DESTINATION_RADIUS_M - 0.25), y_m=1.0, yaw_rad=0.0),
        now_s=1.0,
        observations=[],
        config=Challenge3CorridorConfig(),
    )

    assert step.command.command_type == "stop"
    assert state.mode == "DESTINATION_STOP"
    assert step.reason == "challenge3_destination_reached"


def test_emergency_stop_only_for_close_obstacle_inside_current_corridor():
    route = make_route_frame(start_x_m=1.0, start_y_m=4.0, target_x_m=11.0, target_y_m=4.0)
    state = Challenge3CorridorState(mode="TRACK_BASELINE")
    side_obs = RouteObservation(
        u_m=0.4,
        v_m=2.0,
        field_x_m=0.4,
        field_y_m=6.0,
        robot_forward_m=0.4,
        robot_left_m=2.0,
        range_m=0.45,
    )

    side_step = step_corridor_controller(
        state,
        route=route,
        pose=FieldPose(x_m=1.0, y_m=4.0, yaw_rad=0.0),
        now_s=1.0,
        observations=[side_obs],
        config=Challenge3CorridorConfig(),
    )

    assert side_step.command.command_type == "velocity"

    center_step = step_corridor_controller(
        Challenge3CorridorState(mode="TRACK_BASELINE"),
        route=route,
        pose=FieldPose(x_m=1.0, y_m=4.0, yaw_rad=0.0),
        now_s=1.0,
        observations=[
            RouteObservation(
                u_m=0.4,
                v_m=0.0,
                field_x_m=0.4,
                field_y_m=4.0,
                robot_forward_m=0.4,
                robot_left_m=0.0,
                range_m=0.4,
            )
        ],
        config=Challenge3CorridorConfig(),
    )

    assert center_step.command.command_type == "stop"
    assert center_step.reason == "challenge3_obstacle_emergency_stop"


def test_forward_arc_command_uses_c2_envelope_and_never_reverses_side_speed():
    config = Challenge3CorridorConfig()

    command, left_mps, right_mps = forward_arc_command(heading_error_rad=1.0, config=config)

    assert command.command_type == "velocity"
    assert command.v_mps == pytest.approx(CHALLENGE3_TURN_ENVELOPE_SPEED_MPS)
    assert abs(command.omega_radps) <= CHALLENGE3_TURN_ENVELOPE_OMEGA_RADPS
    assert left_mps >= -1e-9
    assert right_mps >= -1e-9
