import pathlib
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))


def test_extracted_nav_core_modules_import():
    import ugv_nav_dual_mode as nav  # noqa: F401
    from ugv_nav_core import nav_config, nav_status, real_mode_runner, sweep_metrics, velocity_planner

    assert nav.RobotConfig is nav_config.RobotConfig
    assert nav.NavConfig is nav_config.NavConfig
    assert nav.SensorConfig is nav_config.SensorConfig
    assert nav.SimConfig is nav_config.SimConfig
    assert nav.build_nav_status is nav_status.build_nav_status
    assert nav.SweepMetricsLogger is sweep_metrics.SweepMetricsLogger
    assert nav.VelocityLocalPlanner is velocity_planner.VelocityLocalPlanner
    assert nav.competition_v2_status_for_update is real_mode_runner.competition_v2_status_for_update


def test_ugv_nav_dual_mode_help_imports_extracted_modules_and_parses_args():
    nav_script = ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py"
    result = subprocess.run(
        [sys.executable, str(nav_script), "--help"],
        cwd=str(ROOT),
        check=True,
        capture_output=True,
        text=True,
    )

    assert "--row-follower-speed-schedule-enabled" in result.stdout
    assert "--sweep-metrics-log-enabled" in result.stdout
    assert "--competition-mode" in result.stdout


def test_round2_clear_tuned_profile_wires_refactored_nav_config_fields():
    nav_script = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").read_text()
    nav_config = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_core" / "nav_config.py").read_text()
    launch_file = (
        ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py"
    ).read_text()
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()

    for field in [
        "row_follower_speed_schedule_enabled",
        "row_follower_low_speed_lane_kp",
        "row_follower_high_speed_heading_kp",
        "sweep_planner_omega_weight_round2",
        "sweep_metrics_log_enabled",
    ]:
        assert field in nav_config
        assert f"nav_cfg.{field}" in nav_script
        assert field in launch_file

    for token in [
        'profile_default NAV_ROW_FOLLOWER_LOW_SPEED_LANE_KP "0.85"',
        'profile_default NAV_ROW_FOLLOWER_HIGH_SPEED_HEADING_KP "1.10"',
        'profile_default NAV_SWEEP_PLANNER_OMEGA_WEIGHT_ROUND2 "0.0"',
        'profile_default NAV_SWEEP_METRICS_LOG_ENABLED "true"',
    ]:
        assert token in bringup
