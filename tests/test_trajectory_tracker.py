import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
NAV_SRC = ROOT / "ros2_ws" / "src" / "ugv_nav"
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from ugv_nav_core.trajectory_tracker import (  # noqa: E402
    Point2D,
    TrackerConfig,
    TrackerPose,
    TrackerState,
    project_pose_to_path,
    start_tracker_goal,
    step_tracker,
    update_tracker_odometry,
)


def test_project_pose_to_path_reports_signed_cross_track_error():
    path = [Point2D(0.0, 0.0), Point2D(10.0, 0.0)]
    projection = project_pose_to_path(path, TrackerPose(x=2.0, y=0.5, yaw=0.0))

    assert projection.s_m == pytest.approx(2.0)
    assert projection.cross_track_m == pytest.approx(0.5)


def test_pure_pursuit_tracks_straight_line_forward():
    state = TrackerState(
        target=Point2D(10.0, 0.0),
        base_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        active_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        state="TRACK_PATH",
    )
    step = step_tracker(state, config=TrackerConfig())

    assert step.command_type == "velocity"
    assert step.v_mps > 0.0
    assert abs(step.omega_radps) < 1e-6
    assert state.remaining_m == pytest.approx(10.0)


def test_heading_drift_produces_corrective_omega():
    state = TrackerState(
        target=Point2D(10.0, 0.0),
        base_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        active_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        pose=TrackerPose(x=1.0, y=0.0, yaw=math.radians(10.0)),
        state="TRACK_PATH",
    )
    step = step_tracker(state, config=TrackerConfig(heading_kp=0.85))

    assert step.omega_radps < 0.0
    assert state.heading_error_rad < 0.0


def test_cross_track_error_returns_to_path():
    state = TrackerState(
        target=Point2D(10.0, 0.0),
        base_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        active_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        pose=TrackerPose(x=2.0, y=0.5, yaw=0.0),
        state="TRACK_PATH",
    )
    step = step_tracker(state, config=TrackerConfig(cross_track_kp=0.75))

    assert step.omega_radps < 0.0
    assert state.cross_track_error_m == pytest.approx(0.5)


def test_goal_stop_triggers_within_stop_radius():
    state = TrackerState(
        target=Point2D(10.0, 0.0),
        base_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        active_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        pose=TrackerPose(x=9.4, y=0.0, yaw=0.0),
        state="TRACK_PATH",
    )
    step = step_tracker(state, config=TrackerConfig(target_stop_radius_m=0.75))

    assert step.command_type == "stop"
    assert step.reason == "goal_reached"
    assert state.state == "GOAL_REACHED"


def test_obstacle_ahead_generates_bypass_waypoints_and_chooses_clearer_side():
    state = TrackerState(
        target=Point2D(10.0, 0.0),
        base_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        active_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        pose=TrackerPose(x=1.0, y=0.0, yaw=0.0),
        state="TRACK_PATH",
    )
    step = step_tracker(
        state,
        config=TrackerConfig(obstacle_warn_m=2.0, obstacle_stop_m=1.0, bypass_offset_m=1.1),
        front_clearance_m=1.5,
        left_clearance_m=2.4,
        right_clearance_m=1.1,
    )

    assert step.command_type == "velocity"
    assert state.state == "BYPASS_TRACK"
    assert state.bypass_side == "left"
    assert len(state.active_path) >= 5


def test_obstacle_with_no_clear_side_stops():
    state = TrackerState(
        target=Point2D(10.0, 0.0),
        base_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        active_path=[Point2D(0.0, 0.0), Point2D(10.0, 0.0)],
        pose=TrackerPose(x=1.0, y=0.0, yaw=0.0),
        state="TRACK_PATH",
    )
    step = step_tracker(
        state,
        config=TrackerConfig(obstacle_warn_m=2.0, obstacle_stop_m=1.0, side_clearance_min_m=0.7),
        front_clearance_m=1.5,
        left_clearance_m=0.4,
        right_clearance_m=0.5,
    )

    assert step.command_type == "stop"
    assert step.reason == "obstacle_no_clear_side"
    assert state.fault_reason == "obstacle_no_clear_side"


def test_tracker_odometry_uses_encoder_distance_and_imu_heading():
    state = TrackerState()
    start_tracker_goal(
        state,
        target_x_m=5.0,
        target_y_m=0.0,
        left_ticks=0,
        right_ticks=0,
        heading_rad=0.0,
    )
    config = TrackerConfig(wheel_radius_m=0.0825, ticks_per_rev=3200.0)
    ticks_for_one_meter = int(round(3200.0 / (2.0 * math.pi * 0.0825)))

    update_tracker_odometry(
        state,
        left_ticks=ticks_for_one_meter,
        right_ticks=ticks_for_one_meter,
        heading_rad=0.0,
        config=config,
    )

    assert state.pose.x == pytest.approx(1.0, abs=0.01)
    assert state.pose.y == pytest.approx(0.0, abs=0.01)
