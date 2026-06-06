import pathlib
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "run_challenge3_prompt.py"


def test_run_challenge3_prompt_starts_standalone_c3_and_keeps_manual_target_by_default():
    result = subprocess.run(
        [
            sys.executable,
            str(TOOL),
            "--start-x-m",
            "0",
            "--start-y-m",
            "0",
            "--start-yaw-deg",
            "0",
            "--dry-run",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "start_nav:=false" in result.stdout
    assert "start_challenge3_corridor:=true" in result.stdout
    assert "challenge3_start_pose_set:=true" in result.stdout
    assert "lidar_filter_forward_fov_deg:=230.000000" in result.stdout
    assert "challenge3_hard_turn_speed_mps:=1.700000" in result.stdout
    assert "challenge3_hard_turn_max_omega_radps:=7.800000" in result.stdout
    assert "challenge3_lidar_min_cluster_points:=3" in result.stdout
    assert "challenge3_lidar_cluster_max_gap_m:=0.350000" in result.stdout
    assert "start_uav_target_receiver:=true" not in result.stdout
    assert "tools/send_uav_target.py" in result.stdout


def test_run_challenge3_prompt_can_enable_esp_target_receiver_in_dry_run():
    result = subprocess.run(
        [
            sys.executable,
            str(TOOL),
            "--start-x-m",
            "1.5",
            "--start-y-m",
            "2.0",
            "--start-yaw-deg",
            "45",
            "--esp-target",
            "--uav-esp-port",
            "/dev/ttyUSB9",
            "--uav-esp-require-checksum",
            "--dry-run",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "start_uav_target_receiver:=true" in result.stdout
    assert "uav_target_input_mode:=serial" in result.stdout
    assert "uav_esp_serial_port:=/dev/ttyUSB9" in result.stdout
    assert "uav_esp_serial_baud:=115200" in result.stdout
    assert "uav_esp_serial_protocol:=binary14" in result.stdout
    assert "uav_esp_require_checksum:=true" in result.stdout
    assert "challenge3_start_x_m:=1.500000" in result.stdout
    assert "challenge3_start_yaw_deg:=45.000000" in result.stdout


def test_run_challenge3_prompt_forwards_extra_launch_args():
    result = subprocess.run(
        [
            sys.executable,
            str(TOOL),
            "--start-x-m",
            "0",
            "--start-y-m",
            "0",
            "--start-yaw-deg",
            "0",
            "--dry-run",
            "--",
            "challenge3_obstacle_lookahead_m:=4.0",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "challenge3_obstacle_lookahead_m:=4.0" in result.stdout
