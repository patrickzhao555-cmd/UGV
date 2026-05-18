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
from ugv_nav_core.row_transition_controller import RowTransitionController, RowTransitionRequest  # noqa: E402
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


def test_round2_does_not_repeatedly_advance_cells_while_odom_stays_zero():
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
    after_first = mission.status_dict()["active_cell"]
    changed_again = mission.observe_navigation_feedback(
        NavigationFeedback(
            1.7,
            physical_stall_detected=True,
            physical_stall_steps=17,
            physical_stall_reason="active_command_zero_odom",
        )
    )
    after_second = mission.status_dict()["active_cell"]

    assert changed
    assert after_first != first_cell
    assert not changed_again
    assert after_second == after_first


def test_sweep_divergence_holds_active_cell_even_when_pose_reaches_cell():
    mission = CompetitionMissionV2("round2", small_config())
    update = mission.update((0.0, 0.0, 0.0), 0.0)
    active_cell = update.status["active_cell"]
    active_xy = update.status["active_cell_xy_m"]
    changed = mission.observe_navigation_feedback(
        NavigationFeedback(
            0.1,
            closed_loop_active=True,
            closed_loop_diverging=True,
            closed_loop_reason="cross_track_error_growing",
        )
    )
    update2 = mission.update((active_xy[0], active_xy[1], 0.0), 0.2)

    assert not changed
    assert update2.status["active_cell"] == active_cell
    assert update2.status["visited_count"] == 0
    assert update2.status["closed_loop_unhealthy"]


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
        "active_cell_row",
        "active_cell_col",
        "sweep_row_direction",
        "sweep_target_yaw_deg",
        "physical_stall_detected",
        "physical_stall_steps",
        "physical_stall_reason",
    ]:
        assert key in update.status


def row_transition_config(**overrides):
    return small_config(
        field_width_m=5.0,
        field_height_m=2.6,
        sweep_row_length_m=4.0,
        sweep_boundary_margin_m=0.45,
        sweep_headland_margin_m=0.75,
        sweep_turn_radius_m=1.0,
        sweep_row_transition_enabled=True,
        sweep_max_rows=3,
        sweep_row_end_tolerance_m=0.35,
        sweep_lane_capture_tolerance_m=0.25,
        sweep_yaw_capture_tolerance_deg=12.0,
        **overrides,
    )


def test_row_end_prep_triggers_before_row_boundary():
    mission = CompetitionMissionV2("round2", row_transition_config())
    update = mission.update((3.75, 0.45, 0.0), 0.0)

    assert update.phase == "sweep_search"
    assert update.status["sweep_subphase"] == "row_end_prep"
    assert update.status["row_end_x_m"] == 4.45
    assert update.reason == "row_end_prep_within_headland_margin"


def test_headland_turn_selects_next_lane():
    mission = CompetitionMissionV2("round2", row_transition_config())
    mission.update((3.75, 0.45, 0.0), 0.0)
    update = mission.update((4.42, 0.45, 0.0), 0.1)

    assert update.status["sweep_subphase"] == "headland_turn"
    assert update.status["row_transition_active"]
    assert update.status["current_lane_y_m"] == 0.45
    assert update.status["next_lane_y_m"] == 0.95
    assert update.status["turn_side"] == "left"


def test_transition_does_not_complete_until_lane_and_yaw_are_captured():
    mission = CompetitionMissionV2("round2", row_transition_config())
    mission.update((3.75, 0.45, 0.0), 0.0)
    mission.update((4.42, 0.45, 0.0), 0.1)

    not_done = mission.update((4.0, 0.95, 0.0), 0.2)
    assert not_done.status["sweep_subphase"] in {"headland_turn", "acquire_next_row"}
    assert not_done.status["row_transition_active"]
    assert not not_done.status["row_transition_done"]
    assert not_done.status["row_index"] == 0

    captured = mission.update((4.0, 0.95, math.pi), 0.3)
    assert captured.status["sweep_subphase"] == "follow_row"
    assert captured.status["row_index"] == 1
    assert captured.status["row_direction"] == -1.0


def test_wide_forward_arc_transition_keeps_wheel_targets_nonnegative():
    controller = RowTransitionController()
    cmd = controller.compute(
        RowTransitionRequest(
            pose_x_m=4.40,
            pose_y_m=0.45,
            pose_yaw_rad=0.0,
            row_direction=1.0,
            current_lane_y_m=0.45,
            next_lane_y_m=0.95,
            row_end_x_m=4.45,
            turn_radius_m=1.0,
            track_width_m=0.6096,
            min_v_mps=0.12,
            max_v_mps=0.36,
            forward_arc_only_enabled=True,
            forward_arc_margin=0.60,
        )
    )

    assert cmd.desired_v_mps > 0.0
    assert cmd.debug["desired_left_target_mps"] >= -1e-9
    assert cmd.debug["desired_right_target_mps"] >= -1e-9
    assert not cmd.transition_done


def test_marker_auto_lane_spacing_recommends_but_manual_override_wins():
    auto = CompetitionMissionConfig(
        marker_auto_lane_spacing_enabled=True,
        marker_reliable_detection_range_m=2.0,
        marker_camera_fov_deg=120.0,
        marker_coverage_overlap_ratio=0.5,
        sweep_lane_spacing_m=0.75,
        sweep_lane_spacing_manual_override=False,
    ).normalized_for_round("round2")
    manual = CompetitionMissionConfig(
        marker_auto_lane_spacing_enabled=True,
        marker_reliable_detection_range_m=2.0,
        marker_camera_fov_deg=120.0,
        marker_coverage_overlap_ratio=0.5,
        sweep_lane_spacing_m=0.75,
        sweep_lane_spacing_manual_override=True,
    ).normalized_for_round("round2")

    assert math.isclose(auto.recommended_marker_lane_spacing_m(), 2.0 * 2.0 * math.tan(math.radians(60.0)) * 0.5)
    assert math.isclose(auto.sweep_lane_spacing_m, auto.recommended_marker_lane_spacing_m())
    assert math.isclose(manual.sweep_lane_spacing_m, 0.75)


def test_competition_v2_cli_launch_and_env_wiring_exists():
    nav_script = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").read_text()
    launch_file = (
        ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py"
    ).read_text()
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()

    for token in [
        "--competition-mission-v2-enabled",
        "--sweep-field-width-m",
        "--sweep-field-height-m",
        "--sweep-row-length-m",
        "--sweep-headland-margin-m",
        "--sweep-boundary-margin-m",
        "--sweep-turn-radius-m",
        "--sweep-row-transition-enabled",
        "--sweep-max-rows",
        "--sweep-row-end-tolerance-m",
        "--sweep-lane-capture-tolerance-m",
        "--sweep-yaw-capture-tolerance-deg",
        "--sweep-cell-size-m",
        "--sweep-lane-spacing-m",
        "--sweep-lane-spacing-manual-override",
        "--sweep-coverage-radius-m",
        "--sweep-coverage-threshold",
        "--sweep-goal-timeout-s",
        "--sweep-fail-limit",
        "--sweep-lane-tolerance-m",
        "--sweep-heading-tolerance-deg",
        "--sweep-allow-pure-turn",
        "--sweep-stall-action",
        "--marker-camera-fov-deg",
        "--marker-reliable-detection-range-m",
        "--marker-coverage-overlap-ratio",
        "--marker-auto-lane-spacing-enabled",
        "--min-competition-speed-mps",
        "--recovery-turn-raw",
        "--competition-closed-loop-enabled",
        "--heading-hold-enabled",
        "--lane-follow-enabled",
        "--heading-hold-kp",
        "--heading-hold-kd",
        "--heading-hold-deadband-deg",
        "--heading-hold-max-omega-rps",
        "--lane-follow-kp-heading",
        "--lane-follow-kp-omega",
        "--lane-follow-deadband-m",
        "--lane-follow-max-heading-deg",
        "--lane-follow-max-omega-rps",
        "--row-follower-speed-schedule-enabled",
        "--row-follower-low-speed-mps",
        "--row-follower-high-speed-mps",
        "--row-follower-low-speed-lane-kp",
        "--row-follower-high-speed-lane-kp",
        "--row-follower-low-speed-heading-kp",
        "--row-follower-high-speed-heading-kp",
        "--row-follower-omega-low-pass-alpha",
        "--row-follower-omega-rate-limit-rps2",
        "--row-follower-min-correction-interval-s",
        "--sweep-planner-omega-weight-round2",
        "--sweep-planner-omega-weight-round3",
        "--sweep-metrics-log-enabled",
        "--sweep-metrics-log-dir",
        "--forward-arc-only-enabled",
        "--forward-arc-margin",
        "--min-sweep-v-mps",
        "--omega-command-sign",
        "--heading-error-sign",
        "--lane-error-sign",
        "--lane-correction-sign",
        "--closed-loop-health-enabled",
        "--closed-loop-divergence-window",
        "--closed-loop-divergence-min-error-m",
        "--closed-loop-divergence-max-growth-m",
        "--closed-loop-divergence-action",
    ]:
        assert token in nav_script
        assert token in launch_file

    for token in [
        "COMPETITION_MISSION_V2_ENABLED",
        "SWEEP_FIELD_WIDTH_M",
        "SWEEP_FIELD_HEIGHT_M",
        "SWEEP_ROW_LENGTH_M",
        "SWEEP_HEADLAND_MARGIN_M",
        "SWEEP_BOUNDARY_MARGIN_M",
        "SWEEP_TURN_RADIUS_M",
        "SWEEP_ROW_TRANSITION_ENABLED",
        "SWEEP_MAX_ROWS",
        "SWEEP_ROW_END_TOLERANCE_M",
        "SWEEP_LANE_CAPTURE_TOLERANCE_M",
        "SWEEP_YAW_CAPTURE_TOLERANCE_DEG",
        "SWEEP_CELL_SIZE_M",
        "SWEEP_LANE_SPACING_M",
        "SWEEP_LANE_SPACING_MANUAL_OVERRIDE",
        "SWEEP_COVERAGE_RADIUS_M",
        "SWEEP_COVERAGE_THRESHOLD",
        "SWEEP_GOAL_TIMEOUT_S",
        "SWEEP_FAIL_LIMIT",
        "SWEEP_LANE_TOLERANCE_M",
        "SWEEP_HEADING_TOLERANCE_DEG",
        "SWEEP_ALLOW_PURE_TURN",
        "SWEEP_STALL_ACTION",
        "MARKER_CAMERA_FOV_DEG",
        "MARKER_RELIABLE_DETECTION_RANGE_M",
        "MARKER_COVERAGE_OVERLAP_RATIO",
        "MARKER_AUTO_LANE_SPACING_ENABLED",
        "MIN_COMPETITION_SPEED_MPS",
        "NAV_RECOVERY_TURN_RAW",
        "NAV_COMPETITION_CLOSED_LOOP_ENABLED",
        "NAV_HEADING_HOLD_ENABLED",
        "NAV_LANE_FOLLOW_ENABLED",
        "NAV_HEADING_HOLD_KP",
        "NAV_HEADING_HOLD_KD",
        "NAV_HEADING_HOLD_DEADBAND_DEG",
        "NAV_HEADING_HOLD_MAX_OMEGA_RPS",
        "NAV_LANE_FOLLOW_KP_HEADING",
        "NAV_LANE_FOLLOW_KP_OMEGA",
        "NAV_LANE_FOLLOW_DEADBAND_M",
        "NAV_LANE_FOLLOW_MAX_HEADING_DEG",
        "NAV_LANE_FOLLOW_MAX_OMEGA_RPS",
        "NAV_ROW_FOLLOWER_SPEED_SCHEDULE_ENABLED",
        "NAV_ROW_FOLLOWER_LOW_SPEED_MPS",
        "NAV_ROW_FOLLOWER_HIGH_SPEED_MPS",
        "NAV_ROW_FOLLOWER_LOW_SPEED_LANE_KP",
        "NAV_ROW_FOLLOWER_HIGH_SPEED_LANE_KP",
        "NAV_ROW_FOLLOWER_LOW_SPEED_HEADING_KP",
        "NAV_ROW_FOLLOWER_HIGH_SPEED_HEADING_KP",
        "NAV_ROW_FOLLOWER_OMEGA_LOW_PASS_ALPHA",
        "NAV_ROW_FOLLOWER_OMEGA_RATE_LIMIT_RPS2",
        "NAV_ROW_FOLLOWER_MIN_CORRECTION_INTERVAL_S",
        "NAV_SWEEP_PLANNER_OMEGA_WEIGHT_ROUND2",
        "NAV_SWEEP_PLANNER_OMEGA_WEIGHT_ROUND3",
        "NAV_SWEEP_METRICS_LOG_ENABLED",
        "NAV_SWEEP_METRICS_LOG_DIR",
        "NAV_FORWARD_ARC_ONLY_ENABLED",
        "NAV_FORWARD_ARC_MARGIN",
        "NAV_MIN_SWEEP_V_MPS",
        "NAV_OMEGA_COMMAND_SIGN",
        "NAV_HEADING_ERROR_SIGN",
        "NAV_LANE_ERROR_SIGN",
        "NAV_LANE_CORRECTION_SIGN",
        "NAV_CLOSED_LOOP_HEALTH_ENABLED",
        "NAV_CLOSED_LOOP_DIVERGENCE_WINDOW",
        "NAV_CLOSED_LOOP_DIVERGENCE_MIN_ERROR_M",
        "NAV_CLOSED_LOOP_DIVERGENCE_MAX_GROWTH_M",
        "NAV_CLOSED_LOOP_DIVERGENCE_ACTION",
    ]:
        assert token in bringup
