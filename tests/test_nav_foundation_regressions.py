import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.closed_loop_controller import (  # noqa: E402
    ClosedLoopHealthSample,
    ClosedLoopHealthState,
    RowFollowerOmegaState,
    apply_forward_arc_only_limit,
    apply_competition_closed_loop_command,
    compute_heading_hold_correction,
    compute_lane_follow_correction,
    scheduled_row_follower_gains,
    smooth_row_follower_omega,
    update_closed_loop_health,
    wheel_targets_from_v_omega,
)
from ugv_nav_dual_mode import (  # noqa: E402
    Costmap2D,
    ControlCommand,
    EncoderPacket,
    FieldMapPacket,
    GoalPacket,
    GridSpec,
    HybridAStarPlanner,
    ImuPacket,
    LidarPacket,
    NavConfig,
    PhysicalStallState,
    Pose2D,
    RobotConfig,
    Ros2Bridge,
    SectorSnapshot,
    SensorConfig,
    SensorFrame,
    SweepMetricsLogger,
    UGVNavigator,
    VelocityLocalPlanner,
    ZedPacket,
    build_nav_status,
    field_cell_to_world,
    laser_scan_to_lidar_observations,
    update_physical_stall_state,
)


def test_dynamic_sensor_hits_do_not_persist_in_global_planning_map():
    robot_cfg = RobotConfig()
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.local_costmap_enabled = True
    start = Pose2D(1.0, 1.0, 0.0)
    goal = Pose2D(3.0, 1.0, 0.0)
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, 5.0, 5.0, start, goal)
    before = int(np.count_nonzero(navigator.known_costmap.data))

    frame = SensorFrame(
        encoder=EncoderPacket(0, 0, 0.1),
        lidar=LidarPacket(hit_points_local=[(1.0, 0.0)], ranges_m=[1.0], angles_rad=[0.0], timestamp=0.1),
        zed=ZedPacket(hit_points_local=[(1.2, 0.1)], timestamp=0.1),
        goal=GoalPacket(goal.x, goal.y, 0.1),
        imu=ImuPacket((0.0, 0.0, 0.0), (0.0, 0.0, 9.81), 0.1),
    )

    navigator.step(frame)

    assert int(np.count_nonzero(navigator.known_costmap.data)) == before
    assert navigator.local_costmap.dynamic_cells


def test_laser_scan_conversion_preserves_no_return_beams_for_clearing():
    range_max = 5.0
    hits, ranges_m, angles_rad = laser_scan_to_lidar_observations(
        ranges=[float("inf"), range_max, float("nan"), 1.25],
        angle_min=-0.3,
        angle_increment=0.1,
        range_min=0.12,
        range_max=range_max,
    )

    assert len(ranges_m) == 4
    assert len(angles_rad) == 4
    assert ranges_m[:3] == [range_max, range_max, range_max]
    assert ranges_m[3] == 1.25
    assert len(hits) == 1
    assert np.isclose(hits[0][0], 1.25 * np.cos(0.0))
    assert np.isclose(hits[0][1], 0.0)


def test_ros_synced_callback_preserves_clearing_beams_but_marks_only_real_hits():
    bridge = object.__new__(Ros2Bridge)
    bridge._latest_synced_seq = 0
    scan = SimpleNamespace(
        ranges=[float("inf"), 4.0, 1.0],
        angle_min=0.0,
        angle_increment=0.25,
        range_min=0.10,
        range_max=4.0,
    )
    msg = SimpleNamespace(
        encoder_available=True,
        scan=scan,
        zed_obstacle_points=SimpleNamespace(poses=[]),
        near_obstacle=False,
        front_clearance_m=float("inf"),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=500_000_000)),
        left_encoder_ticks=11,
        right_encoder_ticks=22,
    )

    Ros2Bridge._synced_cb(bridge, msg)
    left, right, hits, ranges_m, angles_rad, zed_hits, imu_packet, ts_s = bridge._latest_synced

    assert (left, right) == (11, 22)
    assert ts_s == 10.5
    assert zed_hits == []
    assert len(ranges_m) == 3
    assert len(angles_rad) == 3
    assert ranges_m[0] == 4.0
    assert ranges_m[1] == 4.0
    assert ranges_m[2] == 1.0
    assert len(hits) == 1
    assert np.isclose(hits[0][0], 1.0 * np.cos(0.5))
    assert np.isclose(hits[0][1], 1.0 * np.sin(0.5))
    assert imu_packet.timestamp == 10.5


def test_field_map_update_rebuilds_known_costmap_without_stale_static_obstacles():
    robot_cfg = RobotConfig()
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    navigator = UGVNavigator(
        robot_cfg,
        sensor_cfg,
        nav_cfg,
        4.0,
        4.0,
        Pose2D(0.25, 0.25, 0.0),
        Pose2D(1.75, 1.75, 0.0),
    )
    obstacle_cell = (1, 1)
    obstacle_x, obstacle_y = field_cell_to_world(*obstacle_cell, size=4, cell_size_m=0.5)
    v1 = FieldMapPacket(
        size=4,
        cell_size_m=0.5,
        obstacle_cells=[obstacle_cell],
        start_xy=(0.25, 0.25),
        goal_xy=(1.75, 1.75),
        timestamp=1.0,
        version=1,
    )
    v2 = FieldMapPacket(
        size=4,
        cell_size_m=0.5,
        obstacle_cells=[],
        start_xy=(0.25, 0.25),
        goal_xy=(1.75, 1.75),
        timestamp=2.0,
        version=2,
    )

    navigator.apply_field_map(v1)
    assert navigator.known_costmap.is_occupied_world(obstacle_x, obstacle_y)

    navigator.apply_field_map(v2)
    assert not navigator.known_costmap.is_occupied_world(obstacle_x, obstacle_y)
    assert not navigator.static_costmap.is_occupied_world(obstacle_x, obstacle_y)

    navigator.blocked_memory.add_patch(obstacle_x, obstacle_y, 0.15, 10)
    assert navigator._planning_costmap().is_occupied_world(obstacle_x, obstacle_y)


def test_velocity_planner_hard_rejects_local_costmap_collision_by_default():
    robot_cfg = RobotConfig(length_m=0.30, width_m=0.30, track_width_m=0.60)
    nav_cfg = NavConfig()
    nav_cfg.continuous_allow_costmap_soft_penalty = False
    nav_cfg.continuous_horizon_s = 1.0
    nav_cfg.continuous_dt_s = 0.1
    nav_cfg.continuous_min_speed_mps = 0.20
    nav_cfg.continuous_max_speed_mps = 0.40
    nav_cfg.continuous_v_samples = 2
    nav_cfg.continuous_omega_samples = 3
    nav_cfg.continuous_gap_buffer_m = 0.02
    planner = VelocityLocalPlanner(robot_cfg, nav_cfg, HybridAStarPlanner(robot_cfg))

    spec = GridSpec(resolution=0.05, origin_x=-1.0, origin_y=-1.0, width=80, height=80)
    costmap = Costmap2D(spec, np.zeros((spec.height, spec.width), dtype=np.uint8))
    costmap.mark_disk_world(0.45, 0.0, 0.18)

    cmd = planner.choose_command(
        pose=Pose2D(0.0, 0.0, 0.0),
        target=Pose2D(2.0, 0.0, 0.0),
        goal=Pose2D(2.0, 0.0, 0.0),
        costmap=costmap,
        sectors=SectorSnapshot(),
        prev_cmd=type("PrevCmd", (), {"omega_radps": 0.0})(),
        hits_local=[(2.0, 2.0)],
        timestamp=1.0,
    )

    assert planner.last_debug["costmap_collision_policy"] == "hard_reject"
    assert planner.last_debug["rejected_collision"] > 0
    assert planner.last_debug["costmap_soft_penalty"] == 0.0
    assert cmd.v_mps <= 1e-6


def test_competition_sweep_velocity_planner_prefers_forward_arc_over_pure_turn():
    robot_cfg = RobotConfig(length_m=0.76, width_m=0.76, track_width_m=0.61)
    nav_cfg = NavConfig()
    nav_cfg.competition_sweep_active = True
    nav_cfg.sweep_allow_pure_turn = False
    nav_cfg.sweep_heading_tolerance_deg = 25.0
    nav_cfg.continuous_min_speed_mps = 0.0894
    nav_cfg.continuous_max_speed_mps = 0.36
    nav_cfg.continuous_horizon_s = 1.0
    nav_cfg.continuous_dt_s = 0.1
    nav_cfg.continuous_v_samples = 4
    nav_cfg.continuous_omega_samples = 7
    planner = VelocityLocalPlanner(robot_cfg, nav_cfg, HybridAStarPlanner(robot_cfg))

    spec = GridSpec(resolution=0.10, origin_x=-1.0, origin_y=-1.0, width=80, height=80)
    costmap = Costmap2D(spec, np.zeros((spec.height, spec.width), dtype=np.uint8))
    pose = Pose2D(2.38, 0.637, np.deg2rad(-26.0))
    target = Pose2D(3.45, 0.45, 0.0)

    cmd = planner.choose_command(
        pose=pose,
        target=target,
        goal=target,
        costmap=costmap,
        sectors=SectorSnapshot(),
        prev_cmd=ControlCommand("FORWARD", controller="velocity", v_mps=0.08),
        hits_local=[],
        timestamp=1.0,
    )

    assert cmd.mode == "FORWARD"
    assert cmd.v_mps > 0.0
    assert planner.last_debug["competition_sweep_active"]
    assert planner.last_debug["rejected_sweep_pure_turn"] > 0


def test_physical_stall_triggers_after_active_command_and_zero_odom():
    state = PhysicalStallState()
    cmd = ControlCommand(
        "TURN_RIGHT",
        raw_left=0.28,
        raw_right=-0.28,
        v_mps=0.0,
        omega_radps=-0.4,
        controller="velocity",
        command_type="velocity",
    )

    for _ in range(16):
        update_physical_stall_state(
            state,
            cmd,
            {"dt_s": 0.1, "ds_used_m": 0.0, "dtheta_deg": 0.0},
            timeout_s=1.5,
        )

    assert state.detected
    assert state.steps == 16
    assert "active_command_zero_odom" in state.reason


def test_heading_hold_correction_sign_and_deadband():
    error, omega = compute_heading_hold_correction(
        math.radians(10.0),
        0.0,
        yaw_rate_radps=0.0,
        kp=1.0,
        kd=0.0,
        deadband_deg=3.0,
        max_omega_radps=0.6,
    )
    assert error > 0.0
    assert omega > 0.0

    small_error, small_omega = compute_heading_hold_correction(
        math.radians(2.0),
        0.0,
        yaw_rate_radps=0.0,
        kp=1.0,
        kd=0.0,
        deadband_deg=3.0,
        max_omega_radps=0.6,
    )
    assert small_error > 0.0
    assert small_omega == 0.0


def test_lane_cross_track_correction_sign_uses_sweep_direction():
    cte_even, heading_even, omega_even = compute_lane_follow_correction(
        lane_y_m=0.45,
        estimated_y_m=0.20,
        row_direction=1.0,
        kp_heading=1.0,
        kp_omega=1.0,
        deadband_m=0.01,
        max_heading_deg=18.0,
        max_omega_radps=0.35,
    )
    cte_odd, heading_odd, omega_odd = compute_lane_follow_correction(
        lane_y_m=0.45,
        estimated_y_m=0.20,
        row_direction=-1.0,
        kp_heading=1.0,
        kp_omega=1.0,
        deadband_m=0.01,
        max_heading_deg=18.0,
        max_omega_radps=0.35,
    )
    assert cte_even == cte_odd
    assert heading_even > 0.0
    assert omega_even > 0.0
    assert heading_odd < 0.0
    assert omega_odd < 0.0


def test_closed_loop_sign_multipliers_affect_correction_direction():
    _cte, _heading, omega_default = compute_lane_follow_correction(
        lane_y_m=0.45,
        estimated_y_m=0.20,
        row_direction=1.0,
        kp_heading=1.0,
        kp_omega=1.0,
        deadband_m=0.01,
        max_heading_deg=18.0,
        max_omega_radps=0.35,
    )
    _cte_flip, _heading_flip, omega_flip = compute_lane_follow_correction(
        lane_y_m=0.45,
        estimated_y_m=0.20,
        row_direction=1.0,
        kp_heading=1.0,
        kp_omega=1.0,
        deadband_m=0.01,
        max_heading_deg=18.0,
        max_omega_radps=0.35,
        lane_correction_sign=-1.0,
    )
    heading_default, omega_heading_default = compute_heading_hold_correction(
        math.radians(10.0),
        0.0,
        kp=1.0,
        kd=0.0,
        deadband_deg=0.0,
        max_omega_radps=0.6,
    )
    heading_flip, omega_heading_flip = compute_heading_hold_correction(
        math.radians(10.0),
        0.0,
        kp=1.0,
        kd=0.0,
        deadband_deg=0.0,
        max_omega_radps=0.6,
        heading_error_sign=-1.0,
    )

    assert omega_default > 0.0
    assert omega_flip < 0.0
    assert heading_default > 0.0
    assert omega_heading_default > 0.0
    assert heading_flip < 0.0
    assert omega_heading_flip < 0.0


def _health_sample(cte: float, heading: float = 1.0, x: float = 0.0) -> ClosedLoopHealthSample:
    return ClosedLoopHealthSample(
        timestamp_s=x,
        cross_track_error_m=cte,
        heading_error_deg=heading,
        final_omega_radps=0.1,
        estimated_yaw_deg=0.0,
        pose_x_m=x,
        pose_y_m=0.0,
        odom_ds_m=0.02,
        odom_dtheta_deg=1.0,
        command_v_mps=0.1,
        command_omega_radps=0.1,
    )


def test_closed_loop_health_allows_decreasing_cross_track_error():
    state = ClosedLoopHealthState()
    result = {}
    for i, error in enumerate([0.20, 0.17, 0.14, 0.11, 0.09]):
        result = update_closed_loop_health(
            state,
            enabled=True,
            sample=_health_sample(error, x=float(i)),
            lane_active=True,
            heading_active=False,
            correction_sign_ok=True,
            window=5,
            min_error_m=0.08,
            max_growth_m=0.05,
        )

    assert not result["closed_loop_diverging"]
    assert result["cross_track_error_trend"] < 0.0


def test_closed_loop_health_flags_growing_cross_track_error_under_correction():
    state = ClosedLoopHealthState()
    result = {}
    for i, error in enumerate([0.10, 0.13, 0.16, 0.19, 0.22]):
        result = update_closed_loop_health(
            state,
            enabled=True,
            sample=_health_sample(error, x=float(i)),
            lane_active=True,
            heading_active=False,
            correction_sign_ok=True,
            window=5,
            min_error_m=0.08,
            max_growth_m=0.05,
        )

    assert result["closed_loop_diverging"]
    assert "cross_track_error_growing" in result["divergence_reason"]


def test_competition_closed_loop_sweep_keeps_forward_velocity_and_avoids_pure_turn():
    robot_cfg = RobotConfig(length_m=0.76, width_m=0.76, track_width_m=0.6096)
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.competition_closed_loop_enabled = True
    nav_cfg.heading_hold_enabled = True
    nav_cfg.lane_follow_enabled = True
    nav_cfg.continuous_min_speed_mps = 0.0894
    start = Pose2D(2.38, 0.20, 0.0)
    goal = Pose2D(4.0, 0.45, 0.0)
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, 5.0, 2.0, start, goal)
    navigator.velocity_debug = {"safety_state": "clear"}

    frame = SensorFrame(
        encoder=EncoderPacket(0, 0, 1.0),
        lidar=LidarPacket(hit_points_local=[], ranges_m=[], angles_rad=[], timestamp=1.0),
        zed=ZedPacket(hit_points_local=[], timestamp=1.0),
        goal=GoalPacket(goal.x, goal.y, 1.0),
        imu=ImuPacket((0.0, 0.0, 0.0), (0.0, 0.0, 9.81), 1.0),
    )
    turn_cmd = ControlCommand(
        "TURN_RIGHT",
        raw_left=0.28,
        raw_right=-0.28,
        v_mps=0.0,
        omega_radps=-0.4,
        controller="velocity",
        command_type="velocity",
        reason="planner tiny turn",
    )
    mission_status = {
        "competition_v2": {
            "enabled": True,
            "phase": "sweep_search",
            "active_lane_y_m": 0.45,
            "active_cell_row": 0,
            "minimum_speed_mps": 0.0894,
            "sweep_target_yaw_deg": 0.0,
        }
    }

    adjusted = apply_competition_closed_loop_command(navigator, turn_cmd, frame, mission_status)

    assert adjusted.controller == "velocity"
    assert adjusted.command_type == "velocity"
    assert adjusted.mode == "FORWARD"
    assert adjusted.v_mps >= 0.0894
    assert adjusted.raw_left >= 0.0
    assert adjusted.raw_right >= 0.0
    assert navigator.closed_loop_debug["closed_loop_active"]
    assert navigator.closed_loop_debug["cross_track_error_m"] > 0.0
    assert navigator.closed_loop_debug["omega_lane_radps"] > 0.0
    assert navigator.closed_loop_debug["final_v_mps"] > 0.0
    assert navigator.closed_loop_debug["final_left_target_mps"] >= 0.0
    assert navigator.closed_loop_debug["final_right_target_mps"] >= 0.0


def test_closed_loop_inactive_during_sweep_has_explicit_reason():
    robot_cfg = RobotConfig(length_m=0.76, width_m=0.76, track_width_m=0.6096)
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.competition_closed_loop_enabled = False
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, 5.0, 2.0, Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
    frame = SensorFrame(
        encoder=EncoderPacket(0, 0, 1.0),
        lidar=LidarPacket(hit_points_local=[], ranges_m=[], angles_rad=[], timestamp=1.0),
        zed=ZedPacket(hit_points_local=[], timestamp=1.0),
        goal=GoalPacket(1.0, 0.0, 1.0),
    )
    mission_status = {
        "competition_v2": {
            "enabled": True,
            "phase": "sweep_search",
            "active_lane_y_m": 0.45,
            "active_cell_row": 0,
            "minimum_speed_mps": 0.0894,
        }
    }

    adjusted = apply_competition_closed_loop_command(
        navigator,
        ControlCommand("FORWARD", v_mps=0.1, controller="velocity", command_type="velocity"),
        frame,
        mission_status,
    )

    assert adjusted.mode == "FORWARD"
    assert not navigator.closed_loop_debug["closed_loop_active"]
    assert navigator.closed_loop_debug["reason"] == "disabled"


def test_closed_loop_divergence_action_slow_then_stop():
    robot_cfg = RobotConfig(length_m=0.76, width_m=0.76, track_width_m=0.6096)
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.competition_closed_loop_enabled = True
    nav_cfg.closed_loop_divergence_window = 2
    nav_cfg.closed_loop_divergence_min_error_m = 0.08
    nav_cfg.closed_loop_divergence_max_growth_m = 0.01
    nav_cfg.closed_loop_divergence_action = "slow_then_stop"
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, 5.0, 2.0, Pose2D(0.0, -0.10, 0.0), Pose2D(1.0, 0.0, 0.0))
    navigator.velocity_debug = {"safety_state": "clear"}
    mission_status = {
        "competition_v2": {
            "enabled": True,
            "phase": "sweep_search",
            "active_lane_y_m": 0.0,
            "active_cell_row": 0,
            "minimum_speed_mps": 0.0894,
            "sweep_target_yaw_deg": 0.0,
        }
    }
    frame = SensorFrame(
        encoder=EncoderPacket(0, 0, 1.0),
        lidar=LidarPacket(hit_points_local=[], ranges_m=[], angles_rad=[], timestamp=1.0),
        zed=ZedPacket(hit_points_local=[], timestamp=1.0),
        goal=GoalPacket(1.0, 0.0, 1.0),
    )
    cmd = ControlCommand("FORWARD", v_mps=0.1, controller="velocity", command_type="velocity")

    first = apply_competition_closed_loop_command(navigator, cmd, frame, mission_status)
    navigator.state.estimated_pose = Pose2D(0.05, -0.16, 0.0)
    navigator.last_odom_delta["ds_m"] = 0.02
    second = apply_competition_closed_loop_command(navigator, cmd, frame, mission_status)
    second_v = navigator.closed_loop_debug["final_v_mps"]
    navigator.state.estimated_pose = Pose2D(0.10, -0.24, 0.0)
    navigator.last_odom_delta["ds_m"] = 0.02
    third = apply_competition_closed_loop_command(navigator, cmd, frame, mission_status)

    assert first.mode == "FORWARD"
    assert second.mode != "STOP"
    assert second_v < 0.1
    assert navigator.closed_loop_health.divergence_persist_count >= 2
    assert third.mode == "STOP"
    assert "closed-loop divergence" in third.reason


def test_row_follower_speed_schedule_interpolates_gains():
    nav_cfg = NavConfig()
    nav_cfg.row_follower_speed_schedule_enabled = True
    nav_cfg.row_follower_low_speed_mps = 0.10
    nav_cfg.row_follower_high_speed_mps = 0.20
    nav_cfg.row_follower_low_speed_lane_kp = 0.80
    nav_cfg.row_follower_high_speed_lane_kp = 1.20
    nav_cfg.row_follower_low_speed_heading_kp = 0.90
    nav_cfg.row_follower_high_speed_heading_kp = 1.10

    lane_low, heading_low, enabled_low = scheduled_row_follower_gains(nav_cfg, 0.10)
    lane_mid, heading_mid, enabled_mid = scheduled_row_follower_gains(nav_cfg, 0.15)
    lane_high, heading_high, enabled_high = scheduled_row_follower_gains(nav_cfg, 0.20)

    assert enabled_low and enabled_mid and enabled_high
    assert lane_low == 0.80
    assert heading_low == 0.90
    assert math.isclose(lane_mid, 1.00)
    assert math.isclose(heading_mid, 1.00)
    assert lane_high == 1.20
    assert heading_high == 1.10


def test_row_follower_omega_low_pass_and_rate_limiter_work():
    state = RowFollowerOmegaState()
    first, first_limited, _ = smooth_row_follower_omega(
        0.0,
        1.0,
        state,
        low_pass_alpha=0.5,
        rate_limit_rps2=0.6,
    )
    second, second_limited, second_dt = smooth_row_follower_omega(
        1.0,
        1.1,
        state,
        low_pass_alpha=1.0,
        rate_limit_rps2=0.6,
    )

    assert first == 0.0
    assert not first_limited
    assert second_limited
    assert math.isclose(second_dt, 0.1)
    assert math.isclose(second, 0.06, abs_tol=1e-6)


def test_round2_sweep_planner_omega_weight_defaults_to_zero():
    robot_cfg = RobotConfig(length_m=0.76, width_m=0.76, track_width_m=0.6096)
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.competition_closed_loop_enabled = True
    nav_cfg.heading_hold_enabled = True
    nav_cfg.lane_follow_enabled = True
    nav_cfg.sweep_planner_omega_weight_round2 = 0.0
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, 5.0, 2.0, Pose2D(0.0, 0.45, 0.0), Pose2D(1.0, 0.45, 0.0))
    navigator.velocity_debug = {"safety_state": "clear"}
    frame = SensorFrame(
        encoder=EncoderPacket(0, 0, 1.0),
        lidar=LidarPacket(hit_points_local=[], ranges_m=[], angles_rad=[], timestamp=1.0),
        zed=ZedPacket(hit_points_local=[], timestamp=1.0),
        goal=GoalPacket(1.0, 0.45, 1.0),
    )
    mission_status = {
        "competition_v2": {
            "enabled": True,
            "round": "round2",
            "phase": "sweep_search",
            "active_lane_y_m": 0.45,
            "active_cell_row": 0,
            "minimum_speed_mps": 0.0894,
            "sweep_target_yaw_deg": 0.0,
        }
    }

    adjusted = apply_competition_closed_loop_command(
        navigator,
        ControlCommand("FORWARD", v_mps=0.1, omega_radps=0.5, controller="velocity", command_type="velocity"),
        frame,
        mission_status,
    )

    assert adjusted.mode == "FORWARD"
    assert navigator.closed_loop_debug["sweep_planner_omega_weight"] == 0.0
    assert navigator.closed_loop_debug["planner_omega_contribution_radps"] == 0.0
    assert abs(navigator.closed_loop_debug["final_omega_radps"]) <= 1e-6


def test_round3_sweep_planner_omega_weight_can_blend_planner_omega():
    robot_cfg = RobotConfig(length_m=0.76, width_m=0.76, track_width_m=0.6096)
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.competition_closed_loop_enabled = True
    nav_cfg.heading_hold_enabled = True
    nav_cfg.lane_follow_enabled = True
    nav_cfg.sweep_planner_omega_weight_round3 = 0.2
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, 5.0, 2.0, Pose2D(0.0, 0.45, 0.0), Pose2D(1.0, 0.45, 0.0))
    navigator.velocity_debug = {"safety_state": "clear"}
    frame = SensorFrame(
        encoder=EncoderPacket(0, 0, 1.0),
        lidar=LidarPacket(hit_points_local=[], ranges_m=[], angles_rad=[], timestamp=1.0),
        zed=ZedPacket(hit_points_local=[], timestamp=1.0),
        goal=GoalPacket(1.0, 0.45, 1.0),
    )
    mission_status = {
        "competition_v2": {
            "enabled": True,
            "round": "round3",
            "phase": "sweep_search",
            "active_lane_y_m": 0.45,
            "active_cell_row": 0,
            "minimum_speed_mps": 0.0894,
            "sweep_target_yaw_deg": 0.0,
        }
    }

    adjusted = apply_competition_closed_loop_command(
        navigator,
        ControlCommand("FORWARD", v_mps=0.1, omega_radps=0.5, controller="velocity", command_type="velocity"),
        frame,
        mission_status,
    )

    assert adjusted.mode == "FORWARD"
    assert navigator.closed_loop_debug["sweep_planner_omega_weight"] == 0.2
    assert math.isclose(navigator.closed_loop_debug["planner_omega_contribution_radps"], 0.1)
    assert navigator.closed_loop_debug["final_omega_radps"] > 0.0


def test_forward_arc_only_clamps_small_reverse_inner_wheel_targets():
    v_mps, omega, limit, clamped, left, right = apply_forward_arc_only_limit(
        0.10,
        0.50,
        0.6096,
        enabled=True,
        margin=0.75,
        min_v_mps=0.08,
    )

    assert v_mps == 0.10
    assert clamped
    assert limit is not None
    assert abs(omega) <= limit
    assert left >= 0.0
    assert right >= 0.0
    assert right > left


def test_forward_arc_only_disabled_preserves_requested_omega():
    v_mps, omega, limit, clamped, left, right = apply_forward_arc_only_limit(
        0.10,
        0.50,
        0.6096,
        enabled=False,
        margin=0.75,
        min_v_mps=0.08,
    )

    assert v_mps == 0.10
    assert omega == 0.50
    assert limit is None
    assert not clamped
    assert left < 0.0
    assert right > 0.0


def test_forward_arc_only_raises_tiny_sweep_velocity_before_clamping():
    v_mps, omega, _limit, clamped, left, right = apply_forward_arc_only_limit(
        0.0,
        0.50,
        0.6096,
        enabled=True,
        margin=0.75,
        min_v_mps=0.08,
    )

    assert v_mps == 0.08
    assert clamped
    assert left >= 0.0
    assert right >= 0.0
    assert omega > 0.0


def test_forward_arc_left_and_right_target_signs_are_correct():
    left_turn_left, left_turn_right = wheel_targets_from_v_omega(0.10, 0.20, 0.6096)
    right_turn_left, right_turn_right = wheel_targets_from_v_omega(0.10, -0.20, 0.6096)

    assert left_turn_right > left_turn_left
    assert right_turn_left > right_turn_right
    assert left_turn_left >= 0.0
    assert left_turn_right >= 0.0
    assert right_turn_left >= 0.0
    assert right_turn_right >= 0.0


def test_physical_stall_recovery_path_is_not_forward_arc_clamped():
    robot_cfg = RobotConfig(length_m=0.76, width_m=0.76, track_width_m=0.6096)
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.competition_closed_loop_enabled = True
    nav_cfg.forward_arc_only_enabled = True
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, 5.0, 2.0, Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
    navigator.velocity_debug = {"safety_state": "clear"}
    navigator.physical_stall.detected = True
    navigator.physical_stall.reason = "active_command_zero_odom"

    frame = SensorFrame(
        encoder=EncoderPacket(0, 0, 1.0),
        lidar=LidarPacket(hit_points_local=[], ranges_m=[], angles_rad=[], timestamp=1.0),
        zed=ZedPacket(hit_points_local=[], timestamp=1.0),
        goal=GoalPacket(1.0, 0.0, 1.0),
    )
    recovery_cmd = ControlCommand(
        "TURN_LEFT",
        v_mps=0.10,
        omega_radps=0.50,
        controller="velocity",
        command_type="velocity",
        reason="recovery turn",
    )
    mission_status = {
        "competition_v2": {
            "enabled": True,
            "phase": "sweep_search",
            "active_lane_y_m": 0.45,
            "active_cell_row": 0,
            "minimum_speed_mps": 0.0894,
            "sweep_target_yaw_deg": 0.0,
        }
    }

    adjusted = apply_competition_closed_loop_command(navigator, recovery_cmd, frame, mission_status)

    assert adjusted is recovery_cmd
    assert adjusted.omega_radps == 0.50
    assert navigator.closed_loop_debug["reason"] == "physical_stall_recovery"
    assert not navigator.closed_loop_debug["forward_arc_clamped"]


def test_nav_status_exposes_competition_closed_loop_fields():
    robot_cfg = RobotConfig()
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, 5.0, 2.0, Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
    navigator.closed_loop_debug.update(
        {
            "closed_loop_enabled": True,
            "heading_hold_enabled": True,
            "lane_follow_enabled": True,
            "heading_hold_kp": 1.23,
            "heading_hold_kd": 0.07,
            "heading_hold_deadband_deg": 4.0,
            "heading_hold_max_omega_rps": 0.44,
            "lane_follow_kp_heading": 1.45,
            "lane_follow_kp_omega": 0.90,
            "lane_follow_deadband_m": 0.08,
            "lane_follow_max_heading_deg": 15.0,
            "lane_follow_max_omega_rps": 0.25,
            "row_follower_speed_schedule_enabled": True,
            "row_follower_scheduled_lane_kp": 0.85,
            "row_follower_scheduled_heading_kp": 0.95,
            "row_follower_raw_omega_radps": 0.06,
            "row_follower_smoothed_omega_radps": 0.05,
            "row_follower_rate_limited": False,
            "sweep_planner_omega_weight": 0.0,
            "planner_omega_radps": 0.2,
            "planner_omega_contribution_radps": 0.0,
            "forward_arc_only_enabled": True,
            "forward_arc_margin": 0.75,
            "min_sweep_v_mps": 0.08,
            "forward_arc_omega_limit_radps": 0.20,
            "forward_arc_clamped": True,
            "final_left_target_mps": 0.04,
            "final_right_target_mps": 0.16,
            "closed_loop_health_enabled": True,
            "closed_loop_diverging": False,
            "divergence_reason": "",
            "cross_track_error_trend": -0.01,
            "heading_error_trend": 0.0,
            "correction_sign_ok": True,
            "omega_command_sign": 1.0,
            "heading_error_sign": 1.0,
            "lane_error_sign": 1.0,
            "lane_correction_sign": 1.0,
            "last_error_abs": 0.13,
            "current_error_abs": 0.12,
            "divergence_action": "slow_then_stop",
            "target_yaw_deg": 0.0,
            "estimated_yaw_deg": 1.0,
            "heading_error_deg": -1.0,
            "cross_track_error_m": 0.12,
            "omega_heading_radps": -0.02,
            "omega_lane_radps": 0.08,
            "final_v_mps": 0.10,
            "final_omega_radps": 0.06,
            "heading_source": "odom_yaw",
        }
    )
    frame = SensorFrame(
        encoder=EncoderPacket(0, 0, 1.0),
        lidar=LidarPacket(hit_points_local=[], ranges_m=[], angles_rad=[], timestamp=1.0),
        zed=ZedPacket(hit_points_local=[], timestamp=1.0),
        goal=GoalPacket(1.0, 0.0, 1.0),
    )

    status = build_nav_status(navigator, frame, ControlCommand("FORWARD"), {})

    for key in [
        "closed_loop_enabled",
        "heading_hold_enabled",
        "lane_follow_enabled",
        "target_yaw_deg",
        "estimated_yaw_deg",
        "heading_error_deg",
        "cross_track_error_m",
        "omega_heading_radps",
        "omega_lane_radps",
        "final_v_mps",
        "final_omega_radps",
        "heading_source",
    ]:
        assert key in status
    for key in [
        "heading_hold_kp",
        "heading_hold_kd",
        "heading_hold_deadband_deg",
        "heading_hold_max_omega_rps",
        "lane_follow_kp_heading",
        "lane_follow_kp_omega",
        "lane_follow_deadband_m",
        "lane_follow_max_heading_deg",
        "lane_follow_max_omega_rps",
        "row_follower_speed_schedule_enabled",
        "row_follower_scheduled_lane_kp",
        "row_follower_scheduled_heading_kp",
        "row_follower_raw_omega_radps",
        "row_follower_smoothed_omega_radps",
        "row_follower_rate_limited",
        "sweep_planner_omega_weight",
        "planner_omega_radps",
        "planner_omega_contribution_radps",
        "forward_arc_only_enabled",
        "forward_arc_margin",
        "min_sweep_v_mps",
        "forward_arc_omega_limit_radps",
        "forward_arc_clamped",
        "final_left_target_mps",
        "final_right_target_mps",
        "closed_loop_health_enabled",
        "closed_loop_diverging",
        "divergence_reason",
        "cross_track_error_trend",
        "heading_error_trend",
        "correction_sign_ok",
        "omega_command_sign",
        "heading_error_sign",
        "lane_error_sign",
        "lane_correction_sign",
        "last_error_abs",
        "current_error_abs",
        "divergence_action",
    ]:
        assert key in status["competition_closed_loop"]


def test_sweep_metrics_logger_writes_expected_fields(tmp_path):
    logger = SweepMetricsLogger(str(tmp_path))
    status = {
        "stamp": 12.3,
        "pose_m": [1.0, 0.47, 2.0],
        "physical_stall_detected": False,
        "competition_v2": {
            "phase": "sweep_search",
            "active_cell": 4,
        },
        "competition_closed_loop": {
            "cross_track_error_m": 0.02,
            "heading_error_deg": -1.5,
            "final_v_mps": 0.12,
            "final_omega_radps": 0.03,
            "closed_loop_diverging": False,
        },
    }
    motor_status = {
        "target_left_mps": 0.11,
        "target_right_mps": 0.13,
        "measured_left_mps": 0.10,
        "measured_right_mps": 0.12,
    }

    logger.record(status, motor_status)
    summary = logger.summary()
    logger.close()
    content = pathlib.Path(summary["log_file"]).read_text()

    for field in SweepMetricsLogger.FIELDNAMES:
        assert field in content.splitlines()[0]
    assert "sweep_search" in content
    assert summary["samples"] == 1
    assert summary["max_abs_cross_track_error_m"] == 0.02


def test_jetson_round2_clear_tuned_profile_resolves_expected_defaults():
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()

    for token in [
        'UGV_PROFILE="${UGV_PROFILE:-manual}"',
        "round2_clear_tuned)",
        'profile_default ROUND_MODE "round2"',
        'profile_default START_ZED "false"',
        'profile_default START_YOLO_OBSTACLES "false"',
        'profile_default START_MARKER_VISION "false"',
        'profile_default START_DEBUG_DASHBOARD "true"',
        'profile_default NAV_ACTIVE_SCAN_ENABLED "false"',
        'profile_default SWEEP_CELL_SIZE_M "0.75"',
        'profile_default SWEEP_LANE_SPACING_M "0.75"',
        'profile_default SWEEP_COVERAGE_RADIUS_M "0.55"',
        'profile_default SWEEP_COVERAGE_THRESHOLD "0.20"',
        'profile_default SWEEP_GOAL_TIMEOUT_S "12.0"',
        'profile_default SWEEP_FAIL_LIMIT "3"',
        'profile_default ROBOT_TICKS_PER_REV "2151"',
        'profile_default DRIVE_SPEED_LEVEL "2"',
        'profile_default MOTOR_VELOCITY_CONTROL_ENABLED "true"',
        'profile_default MOTOR_VELOCITY_KP "0.45"',
        'profile_default MOTOR_VELOCITY_FEEDFORWARD_RAW_PER_MPS "1.60"',
        'profile_default NAV_FORWARD_ARC_MARGIN "0.60"',
        'profile_default NAV_MIN_SWEEP_V_MPS "0.12"',
        'profile_default NAV_HEADING_HOLD_KP "0.95"',
        'profile_default NAV_HEADING_HOLD_KD "0.18"',
        'profile_default NAV_LANE_FOLLOW_KP_HEADING "0.85"',
        'profile_default NAV_LANE_FOLLOW_KP_OMEGA "0.82"',
        'profile_default NAV_CLOSED_LOOP_DIVERGENCE_ACTION "warn"',
        'profile_default NAV_SWEEP_METRICS_LOG_ENABLED "true"',
        "round2_competition_tuned)",
        'UGV profile resolved: START_ZED=${START_ZED}, START_MARKER_VISION=${START_MARKER_VISION}, NAV_ACTIVE_SCAN_ENABLED=${NAV_ACTIVE_SCAN_ENABLED}, SWEEP_COVERAGE_THRESHOLD=${SWEEP_COVERAGE_THRESHOLD}, SWEEP_GOAL_TIMEOUT_S=${SWEEP_GOAL_TIMEOUT_S}, MOTOR_PORT=${MOTOR_PORT}, LIDAR_PORT=${LIDAR_PORT}',
    ]:
        assert token in bringup


def test_jetson_profile_defaults_preserve_manual_env_overrides():
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()

    assert "profile_default()" in bringup
    assert 'if [[ -z "${!name+x}" ]]; then' in bringup
    assert 'export "${name}=${value}"' in bringup
    assert 'UGV profile: ${UGV_PROFILE}' in bringup
    assert 'START_ZED="${START_ZED:-true}"' in bringup
    assert 'SWEEP_COVERAGE_THRESHOLD="${SWEEP_COVERAGE_THRESHOLD:-0.85}"' in bringup


def test_tuned_profile_run_scripts_exist_and_select_profiles():
    scripts = {
        "run_round2_clear_tuned.sh": "round2_clear_tuned",
        "run_round3_obstacle_tuned.sh": "round3_obstacle_tuned",
        "run_round1_straight_tuned.sh": "round1_straight_tuned",
    }
    for script_name, profile in scripts.items():
        script = (ROOT / "ros2_ws" / "scripts" / script_name).read_text()
        assert f'export UGV_PROFILE="${{UGV_PROFILE:-{profile}}}"' in script
        assert "first_existing_port" in script
        assert "MOTOR_PORT" in script
        assert "LIDAR_PORT" in script
        assert 'exec bash "${WORKSPACE_DIR}/jetson_bringup.sh"' in script


def test_round2_clear_tuned_script_prefers_by_id_ports_and_extra_workspace():
    script = (ROOT / "ros2_ws" / "scripts" / "run_round2_clear_tuned.sh").read_text()

    for token in [
        "first_matching_port",
        "/dev/serial/by-id/*Teensyduino*",
        "/dev/serial/by-id/*Teensy*",
        "/dev/serial/by-id/*Silicon_Labs*CP210*",
        "/dev/serial/by-id/*CP2102*",
        "first_existing_port /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB1 /dev/ttyUSB0",
        "first_existing_port /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1",
        'if [[ -n "${EXTRA_SETUP_BASH:-}" ]]; then',
        '${HOME}/ugv_ws_albert/install/setup.bash',
        '${WORKSPACE_DIR}/install/setup.bash',
    ]:
        assert token in script


def test_local_costmap_cli_launch_and_env_parameters_are_wired():
    nav_script = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").read_text()
    launch_file = (
        ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py"
    ).read_text()
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()

    expected_cli = [
        "--local-costmap-width-m",
        "--local-costmap-height-m",
        "--local-costmap-resolution-m",
        "--local-costmap-dynamic-decay-s",
        "--local-costmap-obstacle-radius-m",
        "--local-costmap-inflation-m",
        "--local-costmap-lidar-clear-radius-m",
        "--continuous-allow-costmap-soft-penalty",
    ]
    for token in expected_cli:
        assert token in nav_script
        assert token in launch_file

    expected_launch = [
        "local_costmap_width_m",
        "local_costmap_height_m",
        "local_costmap_resolution_m",
        "local_costmap_dynamic_decay_s",
        "local_costmap_obstacle_radius_m",
        "local_costmap_inflation_m",
        "local_costmap_lidar_clear_radius_m",
        "continuous_allow_costmap_soft_penalty",
    ]
    for token in expected_launch:
        assert token in launch_file

    expected_env = [
        "NAV_LOCAL_COSTMAP_WIDTH_M",
        "NAV_LOCAL_COSTMAP_HEIGHT_M",
        "NAV_LOCAL_COSTMAP_RESOLUTION_M",
        "NAV_LOCAL_COSTMAP_DYNAMIC_DECAY_S",
        "NAV_LOCAL_COSTMAP_OBSTACLE_RADIUS_M",
        "NAV_LOCAL_COSTMAP_INFLATION_M",
        "NAV_LOCAL_COSTMAP_LIDAR_CLEAR_RADIUS_M",
        "NAV_CONTINUOUS_ALLOW_COSTMAP_SOFT_PENALTY",
    ]
    for token in expected_env:
        assert token in bringup


def test_real_nav_odometry_calibration_parameters_are_wired():
    nav_script = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").read_text()
    launch_file = (
        ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py"
    ).read_text()
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()

    for token in [
        "--robot-wheel-radius-m",
        "--robot-ticks-per-rev",
        "robot_wheel_radius_m: float = 0.06",
        "robot_ticks_per_rev: int = 1000",
        "robot_cfg.wheel_radius_m = max(0.005, float(robot_wheel_radius_m))",
        "robot_cfg.ticks_per_rev = max(1, int(robot_ticks_per_rev))",
    ]:
        assert token in nav_script

    for token in [
        "robot_wheel_radius_m = LaunchConfiguration('robot_wheel_radius_m')",
        "robot_ticks_per_rev = LaunchConfiguration('robot_ticks_per_rev')",
        "DeclareLaunchArgument('robot_wheel_radius_m', default_value='0.06')",
        "DeclareLaunchArgument('robot_ticks_per_rev', default_value='1000')",
        "--robot-wheel-radius-m",
        "--robot-ticks-per-rev",
    ]:
        assert token in launch_file

    for token in [
        'ROBOT_WHEEL_RADIUS_M="${ROBOT_WHEEL_RADIUS_M:-0.06}"',
        'ROBOT_TICKS_PER_REV="${ROBOT_TICKS_PER_REV:-1000}"',
        'MOTOR_WHEEL_RADIUS_M="${MOTOR_WHEEL_RADIUS_M:-${ROBOT_WHEEL_RADIUS_M}}"',
        'MOTOR_TICKS_PER_REV="${MOTOR_TICKS_PER_REV:-${ROBOT_TICKS_PER_REV}}"',
        'UGV robot odometry: ROBOT_WHEEL_RADIUS_M=',
        'UGV motor velocity odometry: MOTOR_WHEEL_RADIUS_M=',
        'MOTOR_TICKS_DELTA_PCT',
        'WARNING: MOTOR_VELOCITY_CONTROL_ENABLED=true but MOTOR_TICKS_PER_REV=',
        'robot_wheel_radius_m:="${ROBOT_WHEEL_RADIUS_M}"',
        'robot_ticks_per_rev:="${ROBOT_TICKS_PER_REV}"',
        'motor_wheel_radius_m:="${MOTOR_WHEEL_RADIUS_M}"',
        'motor_ticks_per_rev:="${MOTOR_TICKS_PER_REV}"',
    ]:
        assert token in bringup
