import json
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]


def test_clean_nav_chassis_controller_outputs_safe_sim_help_path():
    nav_script = ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py"
    result = subprocess.run(
        [sys.executable, str(nav_script), "--mode", "sim"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "Clean navigation chassis controller" in result.stdout


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

    assert "Two goBILDA 1x15A R/C PWM speed controllers" in readme
    assert "two-controller/four-encoder side" in readme
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


def test_competition_bringup_wires_imu_health_and_debug_status():
    launch = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py").read_text()

    assert 'DeclareLaunchArgument("start_debug_status", default_value="true")' in launch
    assert 'DeclareLaunchArgument("motor_pwm_min_us", default_value="1000")' in launch
    assert 'DeclareLaunchArgument("motor_pwm_max_us", default_value="2000")' in launch
    assert 'DeclareLaunchArgument("motor_teensy_pid_kp", default_value="0.04")' in launch
    assert 'DeclareLaunchArgument("motor_teensy_pid_feedforward_us_per_tps", default_value="0.08")' in launch
    assert 'DeclareLaunchArgument("motor_enable_teensy_side_specific_pid_params", default_value="false")' in launch
    assert 'DeclareLaunchArgument("motor_teensy_pid_static_ff_us", default_value="410.0")' in launch
    assert 'DeclareLaunchArgument("motor_teensy_pid_output_limit_us", default_value="500.0")' in launch
    assert 'DeclareLaunchArgument("nav_imu_timeout_s", default_value="0.30")' in launch
    assert 'DeclareLaunchArgument("nav_imu_min_rate_hz", default_value="20.0")' in launch
    assert 'DeclareLaunchArgument("nav_zed_status_topic", default_value="/zed/status")' in launch
    assert 'DeclareLaunchArgument("nav_encoder_stamped_topic", default_value="/encoder_ticks_stamped")' in launch
    assert 'DeclareLaunchArgument("nav_allow_encoder_heading_fallback", default_value="false")' in launch
    assert 'DeclareLaunchArgument("nav_controller_mode", default_value="competition_tracker")' in launch
    assert 'DeclareLaunchArgument("nav_allow_legacy_controller", default_value="false")' in launch
    assert 'DeclareLaunchArgument("zed_publish_rate_hz", default_value="10.0")' in launch
    assert 'DeclareLaunchArgument("zed_depth_downsample_factor", default_value="2")' in launch
    assert '"--imu-min-rate-hz"' in launch
    assert '"--allow-legacy-controller"' in launch
    assert '"--zed-status-topic"' in launch
    assert '"--encoder-stamped-topic"' in launch
    assert '"--allow-encoder-heading-fallback"' in launch
    assert '"start_debug_status": start_debug_status' in launch
    assert 'DeclareLaunchArgument("nav_tracking_enabled", default_value="true")' in launch
    assert 'DeclareLaunchArgument("nav_target_topic", default_value="/ugv/uav_target")' in launch
    assert 'DeclareLaunchArgument("nav_tracking_max_omega_radps", default_value="0.85")' in launch
    assert '"--tracking-enabled"' in launch
    assert '"--manual-target-x-m"' in launch
    assert '"--obstacle-warn-m"' in launch
    assert '"--bypass-offset-m"' in launch


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
        "ros2_ws/src/third_party/zed-ros2-wrapper",
        "ros2_ws/src/ugv_esp/communication.ino",
        "ros2_ws/src/ugv_sensor_sync/msg/UwbRange.msg",
        "ros2_ws/src/ugv_sensor_sync/include/ugv_sensor_sync/clock_mapper.hpp",
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
        "zed_msgs",
        "UwbRange",
        "clock_mapper",
        "third_party",
        "zed-ros2-wrapper",
        "mission_flag",
        "/ugv/uav_flag",
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
            if file_path.suffix.lower() in {".jpg", ".jpeg", ".png", ".npz"}:
                continue
            text = file_path.read_text(errors="ignore")
            for token in forbidden:
                assert token not in text, f"{token!r} found in {file_path}"


def _package_dependency_names(package_xml: pathlib.Path) -> set[str]:
    root = ET.parse(package_xml).getroot()
    deps = set()
    for element in root:
        if element.tag.endswith("depend") and element.text:
            deps.add(element.text.strip())
    return deps


def test_package_metadata_declares_runtime_dependencies_for_active_launches_and_nodes():
    sensor_deps = _package_dependency_names(ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "package.xml")
    motor_deps = _package_dependency_names(ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "package.xml")

    assert {"cv_bridge", "launch", "launch_ros"}.issubset(sensor_deps)
    assert {"launch", "launch_ros", "python3-serial"}.issubset(motor_deps)
