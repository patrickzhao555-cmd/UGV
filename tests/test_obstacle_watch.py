import importlib.util
import math
import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
WATCH_PATH = ROOT / "tools" / "ugv_obstacle_watch.py"
SPEC = importlib.util.spec_from_file_location("ugv_obstacle_watch", WATCH_PATH)
ugv_obstacle_watch = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ugv_obstacle_watch)


def test_obstacle_watch_reports_front_obstacle_without_stop():
    line = ugv_obstacle_watch.summarize_obstacle_status(
        {
            "front_clearance_m": 1.5,
            "front_clearance_source": "lidar+zed",
            "front_lidar_range_m": 1.55,
            "lidar_any_min_range_m": 1.2,
            "min_depth_range_m": 1.5,
            "valid_depth_samples": 1200,
            "depth_obstacle_points_filtered": 42,
            "depth_blind_hazard_active": False,
            "front_sensor_health": "lidar+zed_ok",
        },
        {
            "safety_level": "ok",
            "safety_reason": "ok",
            "last_command": {"command_type": "velocity", "v_mps": 0.18},
            "v_mps": 0.18,
        },
        {"valid_depth_samples": 1200},
        {"connected": True, "teensy_pid_params_synced": True, "fault": "none"},
        front_threshold_m=2.0,
        stop_clearance_m=1.0,
        fusion_age_s=0.1,
    )

    assert "FRONT<=2.0m: TRUE" in line
    assert "nearest=1.50m source=lidar+zed" in line
    assert "STOP@1.0m: NO" in line
    assert "zed_front=1.50m valid=1200 pts=42 blind=false" in line
    assert "lidar_front=1.55m lidar_any=1.20m any<=2.0m: TRUE" in line
    assert "nav=ok:ok cmd=velocity v=0.18" in line


def test_obstacle_watch_reports_stop_required():
    snap = ugv_obstacle_watch.build_obstacle_snapshot(
        {
            "front_clearance_m": 0.92,
            "front_clearance_source": "zed",
            "min_depth_range_m": 0.92,
            "front_sensor_health": "zed_only_no_lidar_front",
        },
        front_threshold_m=2.0,
        stop_clearance_m=1.0,
        fusion_age_s=0.0,
    )

    assert snap["front_obstacle_within_threshold"] is True
    assert snap["front_stop_required"] is True
    assert snap["front_label"] == "TRUE"
    assert snap["stop_label"] == "YES"


def test_obstacle_watch_keeps_side_only_lidar_out_of_front_obstacle():
    line = ugv_obstacle_watch.summarize_obstacle_status(
        {
            "front_clearance_m": None,
            "front_clearance_source": "none",
            "front_lidar_range_m": None,
            "lidar_any_min_range_m": 0.55,
            "min_depth_range_m": None,
            "front_sensor_health": "no_front_obstacle_points",
        },
        front_threshold_m=2.0,
        stop_clearance_m=1.0,
        fusion_age_s=0.1,
    )

    assert "FRONT<=2.0m: FALSE" in line
    assert "nearest=CLEAR source=none" in line
    assert "STOP@1.0m: NO" in line
    assert "lidar_front=- lidar_any=0.55m any<=2.0m: TRUE" in line


def test_obstacle_watch_marks_stale_and_missing_data_clearly():
    stale = ugv_obstacle_watch.summarize_obstacle_status(
        {"front_clearance_m": 1.0, "front_clearance_source": "lidar"},
        fusion_age_s=1.5,
        stale_timeout_s=0.75,
    )
    missing = ugv_obstacle_watch.summarize_obstacle_status(None)

    assert "FRONT<=2.0m: STALE" in stale
    assert "STOP@1.0m: STALE" in stale
    assert "FRONT<=2.0m: NO_DATA" in missing
    assert "nearest=NO_DATA source=none" in missing


def test_obstacle_watch_treats_infinite_front_clearance_as_clear():
    line = ugv_obstacle_watch.summarize_obstacle_status(
        {
            "front_clearance_m": math.inf,
            "front_clearance_source": "none",
            "front_sensor_health": "no_front_obstacle_points",
        },
        fusion_age_s=0.1,
    )

    assert "FRONT<=2.0m: FALSE" in line
    assert "nearest=CLEAR source=none" in line
