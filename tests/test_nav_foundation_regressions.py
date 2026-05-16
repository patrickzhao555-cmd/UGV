import pathlib
import sys
from types import SimpleNamespace

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_dual_mode import (  # noqa: E402
    Costmap2D,
    EncoderPacket,
    FieldMapPacket,
    GoalPacket,
    GridSpec,
    HybridAStarPlanner,
    ImuPacket,
    LidarPacket,
    NavConfig,
    Pose2D,
    RobotConfig,
    Ros2Bridge,
    SectorSnapshot,
    SensorConfig,
    SensorFrame,
    UGVNavigator,
    VelocityLocalPlanner,
    ZedPacket,
    field_cell_to_world,
    laser_scan_to_lidar_observations,
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
