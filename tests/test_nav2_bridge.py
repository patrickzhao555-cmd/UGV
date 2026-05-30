import json
import math
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.nav2_bridge import (  # noqa: E402
    FieldBounds,
    OccupancyGridSpec,
    Transform2D,
    build_stop_command,
    compose_transform_2d,
    find_nearest_free_target,
    finite_xyz_points,
    evaluate_gyro_bias_samples,
    angle_within_centered_fov,
    field_boundary_decision,
    integrate_planar_odometry,
    inverse_transform_2d,
    map_to_odom_from_pose,
    nav_command_from_twist,
    target_units_scale,
    select_imu_timing_step,
    validate_field_target,
)
from ugv_nav_core.safety_status import motor_fault_reason, normalize_fault_value  # noqa: E402


def test_nav2_cmd_vel_to_ugv_nav_cmd_preserves_velocity_contract_without_raw_pwm():
    command = nav_command_from_twist(linear_x_mps=0.15, angular_z_radps=0.1)
    payload = command.to_payload()
    assert payload["command_type"] == "velocity"
    assert payload["v_mps"] == pytest.approx(0.15)
    assert payload["omega_radps"] == pytest.approx(0.1)
    assert command.motion_rule_ok
    assert "raw_left" not in payload
    assert "raw_right" not in payload
    assert "pwm" not in json.dumps(payload).lower()


def test_nav2_cmd_vel_clamps_sub_min_translation_but_allows_pivot_in_place():
    slow = nav_command_from_twist(linear_x_mps=0.01, angular_z_radps=0.0)
    assert slow.command_type == "velocity"
    assert slow.v_mps == pytest.approx(0.089408)
    assert slow.reason == "nav2_cmd_vel_speed_clamped"
    assert slow.motion_rule_ok

    reverse = nav_command_from_twist(linear_x_mps=-0.01, angular_z_radps=0.0)
    assert reverse.command_type == "stop"
    assert reverse.reason == "reverse_cmd_blocked"

    debug_reverse = nav_command_from_twist(linear_x_mps=-0.01, angular_z_radps=0.0, allow_reverse=True)
    assert debug_reverse.v_mps == pytest.approx(-0.089408)

    pivot = nav_command_from_twist(linear_x_mps=0.0, angular_z_radps=0.2)
    assert pivot.command_type == "velocity"
    assert pivot.v_mps == pytest.approx(0.0)
    assert pivot.omega_radps == pytest.approx(0.2)
    assert pivot.motion_rule_ok


def test_nav2_cmd_vel_rejects_unsupported_lateral_motion_and_allows_stop():
    lateral = nav_command_from_twist(linear_x_mps=0.1, linear_y_mps=0.03, angular_z_radps=0.0)
    assert lateral.command_type == "stop"
    assert lateral.reason == "unsupported_lateral_cmd"

    stop = nav_command_from_twist(linear_x_mps=0.0, angular_z_radps=0.0)
    assert stop == build_stop_command("cmd_vel_stop")


def test_nav2_cmd_vel_rejects_non_finite_values_before_json_output():
    for kwargs in (
        {"linear_x_mps": math.nan, "angular_z_radps": 0.0},
        {"linear_x_mps": 0.0, "angular_z_radps": math.inf},
        {"linear_x_mps": 0.0, "linear_y_mps": math.nan, "angular_z_radps": 0.0},
        {"linear_x_mps": 0.1, "angular_z_radps": 0.0, "competition_min_speed_mps": math.inf},
    ):
        command = nav_command_from_twist(**kwargs)
        payload = command.to_payload()
        assert command.command_type == "stop"
        assert command.reason == "invalid_cmd_vel"
        assert math.isfinite(payload["v_mps"])
        assert math.isfinite(payload["omega_radps"])
        assert "NaN" not in command.to_json()


def test_motor_fault_normalization_matches_motor_bridge_no_fault_sentinels():
    for value in (None, "", "none", "NONE", "0", "false", "ok"):
        assert normalize_fault_value(value) is None
    assert normalize_fault_value("stall") == "stall"
    assert motor_fault_reason({"fault_reason": None, "fault": "none"}) is None
    assert motor_fault_reason({"fault_reason": "stall", "fault": "none"}) == "stall"
    assert motor_fault_reason({"fault_reason": None, "fault": "encoder_jump"}) == "encoder_jump"


def test_transform_math_anchors_manual_field_pose_to_odom_pose():
    desired_map_base = Transform2D(2.0, 3.0, math.radians(90.0))
    current_odom_base = Transform2D(0.5, 0.0, math.radians(15.0))
    map_to_odom = map_to_odom_from_pose(
        desired_map_base=desired_map_base,
        current_odom_base=current_odom_base,
    )
    reconstructed = compose_transform_2d(map_to_odom, current_odom_base)
    assert reconstructed.x == pytest.approx(desired_map_base.x)
    assert reconstructed.y == pytest.approx(desired_map_base.y)
    assert reconstructed.yaw == pytest.approx(desired_map_base.yaw)

    identity = compose_transform_2d(map_to_odom, inverse_transform_2d(map_to_odom))
    assert identity.x == pytest.approx(0.0)
    assert identity.y == pytest.approx(0.0)
    assert identity.yaw == pytest.approx(0.0)


def test_gyro_bias_evaluation_accepts_stable_samples_and_rejects_unstable_or_motion():
    stable = evaluate_gyro_bias_samples(
        [0.010, 0.011, 0.009, 0.010],
        max_stddev_radps=0.03,
        encoder_start_ticks=(100, 200),
        encoder_current_ticks=(101, 201),
        max_encoder_delta_ticks=2,
    )
    assert stable.accepted
    assert stable.reason == "gyro_bias_ready"
    assert stable.mean_radps == pytest.approx(0.010)

    unstable = evaluate_gyro_bias_samples([0.0, 0.08, -0.08], max_stddev_radps=0.03)
    assert not unstable.accepted
    assert unstable.reason == "gyro_bias_unstable"

    moved = evaluate_gyro_bias_samples(
        [0.010, 0.011, 0.009],
        encoder_start_ticks=(100, 200),
        encoder_current_ticks=(104, 200),
        max_encoder_delta_ticks=2,
    )
    assert not moved.accepted
    assert moved.reason == "gyro_bias_encoder_motion"


def test_imu_timing_prefers_header_stamp_and_falls_back_safely():
    first = select_imu_timing_step(
        stamp_s=10.0,
        fallback_now_s=100.0,
        last_stamp_s=None,
        last_fallback_s=None,
        max_dt_s=0.05,
    )
    assert first.skipped
    assert first.reason == "first_imu_sample"

    good = select_imu_timing_step(
        stamp_s=10.02,
        fallback_now_s=100.02,
        last_stamp_s=10.0,
        last_fallback_s=100.0,
        max_dt_s=0.05,
    )
    assert good.dt_s == pytest.approx(0.02)
    assert good.time_source == "header_stamp"
    assert not good.skipped

    fallback = select_imu_timing_step(
        stamp_s=9.0,
        fallback_now_s=100.04,
        last_stamp_s=10.02,
        last_fallback_s=100.02,
        max_dt_s=0.05,
    )
    assert fallback.dt_s == pytest.approx(0.02)
    assert fallback.time_source == "monotonic_fallback"
    assert fallback.reason == "stamp_invalid_or_nonmonotonic"

    jump = select_imu_timing_step(
        stamp_s=11.0,
        fallback_now_s=101.0,
        last_stamp_s=10.0,
        last_fallback_s=100.0,
        max_dt_s=0.05,
    )
    assert jump.skipped
    assert jump.reason == "imu_dt_out_of_range"


def test_integrate_planar_odometry_uses_midpoint_heading():
    pose = Transform2D(0.0, 0.0, 0.0)
    updated = integrate_planar_odometry(pose, distance_m=1.0, delta_yaw_rad=math.pi / 2.0)
    assert updated.x == pytest.approx(math.cos(math.pi / 4.0))
    assert updated.y == pytest.approx(math.sin(math.pi / 4.0))
    assert updated.yaw == pytest.approx(math.pi / 2.0)


def test_field_target_validation_projects_bounds_and_rejects_bad_values():
    bounds = FieldBounds(width_m=10.0, height_m=8.0, margin_m=0.5)
    accepted = validate_field_target(x_m=2.0, y_m=3.0, bounds=bounds)
    assert accepted.accepted
    assert not accepted.adjusted

    projected = validate_field_target(x_m=-2.0, y_m=99.0, bounds=bounds, allow_projection=True)
    assert projected.accepted
    assert projected.adjusted
    assert projected.x_m == pytest.approx(0.5)
    assert projected.y_m == pytest.approx(7.5)

    rejected = validate_field_target(x_m=-2.0, y_m=99.0, bounds=bounds, allow_projection=False)
    assert not rejected.accepted
    assert rejected.reason == "target_out_of_field"

    invalid = validate_field_target(x_m=float("nan"), y_m=1.0, bounds=bounds)
    assert not invalid.accepted
    assert invalid.reason == "target_not_finite"


def test_field_target_validation_uses_occupancy_and_nearest_free_search():
    # 3x3 grid centered at map origin cells. Center cell is occupied.
    grid = OccupancyGridSpec(
        width=3,
        height=3,
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        data=[
            0,
            0,
            0,
            0,
            100,
            0,
            0,
            0,
            0,
        ],
    )
    bounds = FieldBounds(width_m=3.0, height_m=3.0, margin_m=0.0)
    occupied = validate_field_target(x_m=1.5, y_m=1.5, bounds=bounds, occupancy_grid=grid)
    assert not occupied.accepted
    assert occupied.reason == "target_occupied"

    nearest = find_nearest_free_target(x_m=1.5, y_m=1.5, grid=grid, max_search_radius_m=1.5)
    assert nearest is not None
    assert nearest != pytest.approx((1.5, 1.5))
    recovered = validate_field_target(x_m=nearest[0], y_m=nearest[1], bounds=bounds, occupancy_grid=grid)
    assert recovered.accepted


def test_unknown_occupancy_policy_is_configurable():
    bounds = FieldBounds(width_m=2.0, height_m=2.0, margin_m=0.0)
    unknown_free = OccupancyGridSpec(
        width=1,
        height=1,
        resolution_m=2.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        data=[-1],
        unknown_is_blocked=False,
    )
    unknown_blocked = OccupancyGridSpec(
        width=1,
        height=1,
        resolution_m=2.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        data=[-1],
        unknown_is_blocked=True,
    )
    assert validate_field_target(x_m=1.0, y_m=1.0, bounds=bounds, occupancy_grid=unknown_free).accepted
    blocked = validate_field_target(x_m=1.0, y_m=1.0, bounds=bounds, occupancy_grid=unknown_blocked)
    assert not blocked.accepted
    assert blocked.reason == "target_unknown"


def test_target_units_scale_accepts_meters_and_yards_only():
    assert target_units_scale("meters") == pytest.approx(1.0)
    assert target_units_scale("m") == pytest.approx(1.0)
    assert target_units_scale("yards") == pytest.approx(0.9144)
    assert target_units_scale("yd") == pytest.approx(0.9144)
    assert target_units_scale("feet") is None


def test_finite_xyz_points_filters_invalid_points_for_obstacle_cloud():
    points = finite_xyz_points([(1.0, 2.0, 0.0), (math.inf, 0.0, 0.0), (0.0, math.nan, 0.0)])
    assert points == [(1.0, 2.0, 0.0)]


def test_lidar_forward_sector_keeps_front_250_degrees_and_rejects_rear_blockage():
    assert angle_within_centered_fov(0.0, 250.0)
    assert angle_within_centered_fov(math.radians(120.0), 250.0)
    assert angle_within_centered_fov(math.radians(-120.0), 250.0)
    assert not angle_within_centered_fov(math.pi, 250.0)
    assert not angle_within_centered_fov(math.radians(-170.0), 250.0)


def test_field_boundary_gate_enforces_inner_field_for_translation_but_allows_pivot():
    bounds = FieldBounds(width_m=13.716, height_m=13.716, margin_m=0.0)
    near_edge = field_boundary_decision(
        x_m=13.40,
        y_m=5.0,
        yaw_rad=0.0,
        v_mps=0.15,
        bounds=bounds,
        safety_margin_m=0.45,
    )
    assert not near_edge.safe
    assert near_edge.reason == "field_boundary_x_max"

    inside = field_boundary_decision(
        x_m=13.40,
        y_m=5.0,
        yaw_rad=math.pi,
        v_mps=0.15,
        bounds=bounds,
        safety_margin_m=0.45,
    )
    assert not inside.safe
    assert inside.reason == "field_boundary_x_max"

    pivot = field_boundary_decision(
        x_m=13.40,
        y_m=5.0,
        yaw_rad=0.0,
        v_mps=0.0,
        bounds=bounds,
        safety_margin_m=0.45,
    )
    assert pivot.safe


def test_field_boundary_gate_predicts_near_future_exit():
    bounds = FieldBounds(width_m=13.716, height_m=13.716, margin_m=0.0)
    decision = field_boundary_decision(
        x_m=13.16,
        y_m=5.0,
        yaw_rad=0.0,
        v_mps=0.15,
        omega_radps=0.0,
        bounds=bounds,
        safety_margin_m=0.45,
        prediction_time_s=0.75,
    )
    assert not decision.safe
    assert decision.reason == "field_boundary_predicted_exit"


def test_nav2_launch_wires_collision_monitor_between_raw_cmd_vel_and_adapter():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()
    collision_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "config" / "collision_monitor_params.yaml").read_text()

    assert 'SetRemap(src="cmd_vel", dst="cmd_vel_raw")' in launch_text
    assert "start_collision_monitor" in launch_text
    assert 'DeclareLaunchArgument("start_collision_monitor", default_value="true")' in launch_text
    assert "lifecycle_manager_collision_monitor" in launch_text
    assert "cmd_vel_in_topic: /cmd_vel_raw" in collision_text
    assert "cmd_vel_out_topic: /cmd_vel" in collision_text
    assert "SlowPolygon" not in collision_text
    assert "slowdown_ratio" not in collision_text


def test_nav2_launch_rewrites_global_costmap_field_dimensions_from_launch_args():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()

    assert "RewrittenYaml" in launch_text
    assert '"global_costmap.global_costmap.ros__parameters.width": field_width_m' in launch_text
    assert '"global_costmap.global_costmap.ros__parameters.height": field_height_m' in launch_text
    assert '"global_costmap.global_costmap.ros__parameters.resolution": field_map_resolution_m' in launch_text


def test_nav2_launch_publishes_explicit_field_free_space_and_keepout_map():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()
    nav2_params = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "config" / "nav2_field_params.yaml").read_text()
    field_map_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_field_map_node.py").read_text()

    assert "ugv_field_map_node.py" in launch_text
    assert 'DeclareLaunchArgument("start_field_map", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("field_map_resolution_m", default_value="0.05")' in launch_text
    assert "plugins: [static_layer, obstacle_layer, inflation_layer]" in nav2_params
    assert "plugin: nav2_costmap_2d::StaticLayer" in nav2_params
    assert "map_subscribe_transient_local: true" in nav2_params
    assert "inside_free" in field_map_text
    assert "data.append(0 if inside_free else 100)" in field_map_text


def test_nav2_and_fusion_use_filtered_scan_and_15yd_field_defaults():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()
    sensor_launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "sensor_sync_launch.py").read_text()
    nav2_params = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "config" / "nav2_field_params.yaml").read_text()
    collision_params = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "config" / "collision_monitor_params.yaml").read_text()

    assert 'DeclareLaunchArgument("field_width_m", default_value="13.716")' in launch_text
    assert 'DeclareLaunchArgument("field_height_m", default_value="13.716")' in launch_text
    assert "lidar_scan_filter_node" in sensor_launch_text
    assert "lidar_filter_forward_fov_deg" in sensor_launch_text
    assert "ros_param_arg('scan_topic', lidar_filtered_topic)" in sensor_launch_text
    assert "topic: /scan/filtered" in nav2_params
    assert "topic: /scan/filtered" in collision_params
    assert "nav2_back_up_action_bt_node" not in nav2_params
    assert "nav2_behaviors/BackUp" not in nav2_params
    assert "min_velocity: [0.0, 0.0, -0.45]" in nav2_params


def test_nav2_launch_passes_strict_uav_goal_and_adapter_safety_parameters():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()
    goal_bridge_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_uav_goal_bridge.py").read_text()
    adapter_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav2_adapter.py").read_text()

    assert 'DeclareLaunchArgument("allow_reverse", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("field_boundary_prediction_time_s", default_value="0.75")' in launch_text
    assert 'DeclareLaunchArgument("uav_target_units", default_value="meters")' in launch_text
    assert 'DeclareLaunchArgument("uav_require_costmap_for_goal", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("uav_unknown_cost_is_blocked", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("uav_allow_boundary_projection", default_value="false")' in launch_text
    assert 'self.declare_parameter("allow_reverse", False)' in adapter_text
    assert 'self.declare_parameter("require_costmap_for_goal", True)' in goal_bridge_text
    assert 'self.declare_parameter("unknown_cost_is_blocked", True)' in goal_bridge_text
    assert 'self.declare_parameter("require_target_frame", True)' in goal_bridge_text
    assert 'self.declare_parameter("target_units", "meters")' in goal_bridge_text


def test_nav2_launch_parameterizes_sensor_static_transforms():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()

    for name in (
        "lidar_x_m",
        "lidar_y_m",
        "lidar_yaw_rad",
        "zed_x_m",
        "zed_y_m",
        "zed_yaw_rad",
    ):
        assert f'DeclareLaunchArgument("{name}"' in launch_text
    assert "--frame-id" in launch_text
    assert "--child-frame-id" in launch_text
    assert "sensor_extrinsics" in launch_text or "extrinsics" in launch_text


def test_nav2_runtime_dependencies_are_declared_for_launch_and_plugins():
    package_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "package.xml").read_text()

    for dep in (
        "nav2_common",
        "nav2_collision_monitor",
        "nav2_controller",
        "nav2_smac_planner",
        "nav2_regulated_pure_pursuit_controller",
        "nav2_smoother",
        "nav2_velocity_smoother",
        "nav2_behaviors",
        "nav2_bt_navigator",
        "nav2_costmap_2d",
        "nav2_lifecycle_manager",
    ):
        assert f"<exec_depend>{dep}</exec_depend>" in package_text
