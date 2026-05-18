import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.closed_loop_controller import (  # noqa: E402
    apply_competition_closed_loop_command,
    compute_heading_hold_correction,
    compute_lane_follow_correction,
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
    ]:
        assert key in status["competition_closed_loop"]


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
