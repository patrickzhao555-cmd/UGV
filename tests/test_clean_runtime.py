import json
import pathlib
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]


def test_clean_nav_placeholder_outputs_stop_in_sim_help_path():
    nav_script = ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py"
    result = subprocess.run(
        [sys.executable, str(nav_script), "--mode", "sim"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "Clean navigation placeholder" in result.stdout


def test_clean_nav_command_contract_is_stop_safe():
    sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))
    from ugv_nav_dual_mode import build_stop_command  # noqa: E402

    payload = json.loads(build_stop_command().to_json())
    assert payload["mode"] == "STOP"
    assert payload["command_type"] == "stop"
    assert payload["v_mps"] == 0.0
    assert payload["omega_radps"] == 0.0
    assert "raw_left" not in payload
    assert "raw_right" not in payload


def test_clean_runtime_docs_name_active_hardware_truth():
    readme = (ROOT / "README.md").read_text()
    motor_readme = (ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "README.md").read_text()
    nav_arch = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_core" / "ARCHITECTURE.md").read_text()

    assert "Two goBILDA speed controllers" in readme
    assert "two-side PID" in readme
    assert "The bridge is intentionally thin" in motor_readme
    assert "No raw PWM from navigation" in nav_arch
    assert "No motor PID on Jetson" in nav_arch


def test_clean_bringup_is_not_legacy_round_profile_launcher():
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()
    launch = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py").read_text()

    assert "round2_clear_tuned" not in bringup
    assert "MOTOR_VELOCITY_CONTROL_ENABLED" not in bringup
    assert "motor_velocity_control_enabled" not in launch
    assert "start_motor_controller" in launch
    assert "ugv_nav_dual_mode.py" in launch


def test_obsolete_runtime_packages_are_removed_from_source_tree():
    removed_paths = [
        "ros2_ws/start_nav_test.sh",
        "ros2_ws/src/ugv_lidar/package.xml",
        "ros2_ws/src/ugv_serial_odom/package.xml",
        "ros2_ws/src/ugv_sim/package.xml",
        "ros2_ws/src/ugv_sensor_sync/ugv_sensor_sync_nodes/bench_goal_node.py",
        "ros2_ws/src/ugv_sensor_sync/ugv_sensor_sync_nodes/mock_field_map_node.py",
        "ros2_ws/src/ugv_perception/ugv_perception/obstacle_warning.py",
        "ros2_ws/src/ugv_perception/ugv_perception/zed_obj_distance.py",
    ]

    for rel_path in removed_paths:
        assert not (ROOT / rel_path).exists(), rel_path


def test_active_sources_do_not_reference_removed_entrypoints():
    forbidden = [
        "motor_direct_test",
        "velocity_control_enabled",
        "MOTOR_VELOCITY_CONTROL",
        "closed_loop_controller",
        "ugv_serial_odom",
        "ugv_lidar",
        "bench_goal",
        "mock_field_map",
        "/ugv_goal",
        "obstacle_warning",
        "zed_obj_distance",
    ]
    active_paths = [
        ROOT / "README.md",
        ROOT / "ros2_ws" / "STACK_ARCHITECTURE.md",
        ROOT / "ros2_ws" / "JETSON_BRINGUP_CHECKLIST.md",
        ROOT / "ros2_ws" / "JETSON_BRINGUP_CHECKLIST_ZH.md",
        ROOT / "ros2_ws" / "jetson_bringup.sh",
        ROOT / "ros2_ws" / "src" / "ugv_sensor_sync",
        ROOT / "ros2_ws" / "src" / "ugv_perception",
        ROOT / "ros2_ws" / "src" / "ugv_motor_controller",
        ROOT / "ros2_ws" / "src" / "ugv_nav",
    ]

    for path in active_paths:
        files = [path] if path.is_file() else [p for p in path.rglob("*") if p.is_file()]
        for file_path in files:
            if "__pycache__" in file_path.parts:
                continue
            text = file_path.read_text(errors="ignore")
            for token in forbidden:
                assert token not in text, f"{token!r} found in {file_path}"
