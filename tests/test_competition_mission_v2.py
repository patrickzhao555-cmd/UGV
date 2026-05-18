import math
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.competition_mission import (  # noqa: E402
    CELL_BLOCKED,
    CompetitionMissionConfig,
    CompetitionMissionV2,
    NavigationFeedback,
    SweepGrid,
)
from ugv_nav_dual_mode import normalize_mission_mode  # noqa: E402


def small_config(**overrides):
    values = {
        "field_width_m": 2.0,
        "field_height_m": 1.0,
        "sweep_cell_size_m": 0.5,
        "sweep_lane_spacing_m": 0.5,
        "sweep_boundary_margin_m": 0.25,
        "sweep_coverage_radius_m": 0.20,
        "sweep_fail_limit": 2,
        "sweep_goal_timeout_s": 1.0,
        "target_accept_radius_m": 0.25,
        "straight_distance_m": 2.0,
        "start_x_m": 0.0,
        "start_y_m": 0.0,
        "start_yaw_deg": 0.0,
    }
    values.update(overrides)
    return CompetitionMissionConfig(**values)


def test_normalize_mission_mode_accepts_competition_rounds():
    assert normalize_mission_mode("round1") == "round1"
    assert normalize_mission_mode("round2") == "round2"
    assert normalize_mission_mode("round3") == "round3"


def test_sweep_grid_cell_count_for_small_field():
    grid = SweepGrid(2.0, 1.0, 0.5, 0.5, 0.25)
    assert grid.total_cells == 8


def test_sweep_grid_boustrophedon_order_alternates_row_direction():
    grid = SweepGrid(2.0, 1.0, 0.5, 0.5, 0.25)
    row0 = [cell.x for cell in grid.cells if cell.row == 0]
    row1 = [cell.x for cell in grid.cells if cell.row == 1]
    assert row0 == [0.25, 0.75, 1.25, 1.75]
    assert row1 == [1.75, 1.25, 0.75, 0.25]


def test_cell_becomes_visited_when_pose_is_within_coverage_radius():
    mission = CompetitionMissionV2("round2", small_config())
    update = mission.update((0.25, 0.25, 0.0), 0.0)
    assert update.status["visited_count"] == 1


def test_next_cell_advances_after_visited():
    mission = CompetitionMissionV2("round2", small_config())
    update = mission.update((0.25, 0.25, 0.0), 0.0)
    assert update.status["active_cell"] == 1
    assert update.active_goal_m == (1.25, 0.25)


def test_sweep_search_with_lateral_error_within_tolerance_uses_lane_lookahead():
    mission = CompetitionMissionV2(
        "round2",
        small_config(
            field_width_m=5.0,
            field_height_m=2.0,
            sweep_cell_size_m=0.75,
            sweep_lane_spacing_m=0.75,
            sweep_boundary_margin_m=0.45,
            sweep_coverage_radius_m=0.55,
            sweep_lane_tolerance_m=0.30,
        ),
    )
    update = mission.update((2.38, 0.637, math.radians(-26.0)), 0.0)
    assert update.phase == "sweep_search"
    assert update.active_goal_m[0] >= 3.35
    assert abs(update.active_goal_m[1] - 0.45) <= 1e-6
    assert math.hypot(update.active_goal_m[0] - 2.38, update.active_goal_m[1] - 0.637) >= 1.0
    assert update.status["sweep_lane_tolerance_m"] == 0.3


def test_after_coverage_threshold_uses_forward_patrol_goal_not_tiny_turn_goal():
    mission = CompetitionMissionV2("round2", small_config(sweep_coverage_threshold=0.1))
    pose = (0.25, 0.25, 0.0)
    update = mission.update(pose, 0.0)
    dx = update.active_goal_m[0] - pose[0]
    dy = update.active_goal_m[1] - pose[1]
    assert update.phase == "sweep_search"
    assert update.reason in {
        "sweep_cell_visited_advanced",
        "coverage_threshold_reached_continuing_patrol",
    }
    assert math.hypot(dx, dy) >= 0.44
    assert dx > 0.0


def test_blocked_cell_is_skipped_by_sweep_grid():
    grid = SweepGrid(2.0, 1.0, 0.5, 0.5, 0.25)
    first = grid.ensure_active(0.0)
    grid.mark_active_blocked("test", 0.1)
    next_cell = grid.select_next(0.2)
    assert first.state == CELL_BLOCKED
    assert next_cell.index != first.index
    assert grid.blocked_count == 1


def test_round1_extends_straight_goal_instead_of_stopping_before_uav_landed():
    mission = CompetitionMissionV2("round1", small_config(sweep_coverage_radius_m=0.1))
    first = mission.update((0.0, 0.0, 0.0), 0.0)
    second = mission.update((1.6, 0.0, 0.0), 1.0, mission_flag_state="landing")
    assert first.active_goal_m == (2.0, 0.0)
    assert second.active_goal_m[0] > first.active_goal_m[0]
    assert second.phase == "round1_straight"
    assert not second.stop_requested


def test_round1_stops_on_landed_or_complete_flag():
    mission = CompetitionMissionV2("round1", small_config())
    landed = mission.update((0.4, 0.0, 0.0), 1.0, mission_flag_state="landed")
    complete = mission.update((0.4, 0.0, 0.0), 2.0, mission_flag_state="round_complete")
    assert landed.stop_requested
    assert complete.stop_requested
    assert complete.phase == "complete"


def test_round2_selects_sweep_goal_when_no_target_is_known():
    mission = CompetitionMissionV2("round2", small_config())
    update = mission.update((0.0, 0.0, 0.0), 0.0)
    assert update.phase == "sweep_search"
    assert update.active_goal_m == (1.0, 0.25)
    assert not update.target_known


def test_round2_switches_to_target_nav_when_target_is_known():
    mission = CompetitionMissionV2("round2", small_config())
    update = mission.update((0.0, 0.0, 0.0), 0.0, field_map_goal=(1.5, 0.75), field_map_source="uav")
    assert update.phase == "target_nav"
    assert update.active_goal_m == (1.5, 0.75)
    assert update.target_known
    assert update.target_source == "uav"


def test_round2_target_reached_without_landed_flag_keeps_moving():
    mission = CompetitionMissionV2("round2", small_config(stop_on_marker_reached=True))
    update = mission.update((1.5, 0.75, 0.0), 0.0, field_map_goal=(1.5, 0.75), field_map_source="uav")
    assert update.phase == "target_loiter_moving"
    assert update.status["phase"] == "target_loiter_moving"
    assert not update.stop_requested
    assert update.active_goal_m != (1.5, 0.75)


def test_round3_target_reached_without_landed_flag_keeps_moving():
    mission = CompetitionMissionV2("round3", small_config(stop_on_marker_reached=True))
    update = mission.update((1.5, 0.75, 0.0), 0.0, field_map_goal=(1.5, 0.75), field_map_source="uav")
    assert update.phase == "target_loiter_moving"
    assert not update.stop_requested
    assert update.active_goal_m != (1.5, 0.75)


def test_target_loiter_moving_produces_forward_moving_goal_around_target():
    mission = CompetitionMissionV2("round2", small_config())
    pose = (1.0, 0.5, 0.0)
    update = mission.update(pose, 0.0, field_map_goal=(1.0, 0.5), field_map_source="uav")
    dx = update.active_goal_m[0] - pose[0]
    dy = update.active_goal_m[1] - pose[1]
    assert update.phase == "target_loiter_moving"
    assert math.hypot(dx, dy) >= 0.44
    assert dx > 0.0
    assert not update.stop_requested


def test_round2_target_reached_with_landed_flag_stops():
    mission = CompetitionMissionV2("round2", small_config())
    update = mission.update(
        (1.5, 0.75, 0.0),
        0.0,
        field_map_goal=(1.5, 0.75),
        field_map_source="uav",
        mission_flag_state="landed",
    )
    assert update.phase == "complete"
    assert update.stop_requested
    assert update.reason == "mission_flag_landed"


def test_round3_target_reached_with_round_complete_flag_stops():
    mission = CompetitionMissionV2("round3", small_config())
    update = mission.update(
        (1.5, 0.75, 0.0),
        0.0,
        field_map_goal=(1.5, 0.75),
        field_map_source="uav",
        mission_flag_state="round_complete",
    )
    assert update.phase == "complete"
    assert update.stop_requested
    assert update.reason == "mission_flag_round_complete"


def test_round3_marks_cell_blocked_after_repeated_failure_and_chooses_another():
    mission = CompetitionMissionV2("round3", small_config())
    update = mission.update((0.0, 0.0, 0.0), 0.0)
    first_cell = update.status["active_cell"]
    changed1 = mission.observe_navigation_feedback(NavigationFeedback(0.1, no_safe_trajectory=True))
    changed2 = mission.observe_navigation_feedback(NavigationFeedback(0.2, local_planner_failed=True))
    status = mission.status_dict()
    assert not changed1
    assert changed2
    assert status["blocked_count"] == 1
    assert status["active_cell"] != first_cell


def test_round2_skips_active_cell_on_physical_stall_instead_of_repeating_turn():
    mission = CompetitionMissionV2("round2", small_config())
    update = mission.update((0.0, 0.0, 0.0), 0.0)
    first_cell = update.status["active_cell"]
    changed = mission.observe_navigation_feedback(
        NavigationFeedback(
            1.6,
            physical_stall_detected=True,
            physical_stall_steps=16,
            physical_stall_reason="active_command_zero_odom",
        )
    )
    status = mission.status_dict()
    assert changed
    assert status["skipped_count"] == 1
    assert status["active_cell"] != first_cell
    assert not status["stop_requested"]


def test_round3_skips_active_cell_on_physical_stall():
    mission = CompetitionMissionV2("round3", small_config())
    update = mission.update((0.0, 0.0, 0.0), 0.0)
    first_cell = update.status["active_cell"]
    changed = mission.observe_navigation_feedback(
        NavigationFeedback(
            1.6,
            physical_stall_detected=True,
            physical_stall_steps=16,
            physical_stall_reason="active_command_zero_odom",
        )
    )
    status = mission.status_dict()
    assert changed
    assert status["skipped_count"] == 1
    assert status["active_cell"] != first_cell


def test_competition_v2_status_contains_coverage_fields():
    mission = CompetitionMissionV2("round2", small_config())
    update = mission.update((0.25, 0.25, 0.0), 0.0)
    for key in [
        "visited_count",
        "blocked_count",
        "skipped_count",
        "total_cells",
        "coverage_fraction",
        "coverage_threshold",
        "minimum_speed_mps",
        "physical_stall_detected",
        "physical_stall_steps",
        "physical_stall_reason",
    ]:
        assert key in update.status


def test_competition_v2_cli_launch_and_env_wiring_exists():
    nav_script = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").read_text()
    launch_file = (
        ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py"
    ).read_text()
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()

    for token in [
        "--competition-mission-v2-enabled",
        "--sweep-cell-size-m",
        "--sweep-lane-spacing-m",
        "--sweep-coverage-radius-m",
        "--sweep-coverage-threshold",
        "--sweep-goal-timeout-s",
        "--sweep-fail-limit",
        "--sweep-lane-tolerance-m",
        "--sweep-heading-tolerance-deg",
        "--sweep-allow-pure-turn",
        "--sweep-stall-action",
        "--min-competition-speed-mps",
        "--recovery-turn-raw",
    ]:
        assert token in nav_script
        assert token in launch_file

    for token in [
        "COMPETITION_MISSION_V2_ENABLED",
        "SWEEP_CELL_SIZE_M",
        "SWEEP_LANE_SPACING_M",
        "SWEEP_COVERAGE_RADIUS_M",
        "SWEEP_COVERAGE_THRESHOLD",
        "SWEEP_GOAL_TIMEOUT_S",
        "SWEEP_FAIL_LIMIT",
        "SWEEP_LANE_TOLERANCE_M",
        "SWEEP_HEADING_TOLERANCE_DEG",
        "SWEEP_ALLOW_PURE_TURN",
        "SWEEP_STALL_ACTION",
        "MIN_COMPETITION_SPEED_MPS",
        "NAV_RECOVERY_TURN_RAW",
    ]:
        assert token in bringup
