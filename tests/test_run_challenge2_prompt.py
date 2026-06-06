import pathlib
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "run_challenge2_prompt.py"


def test_run_challenge2_prompt_keeps_manual_target_path_by_default():
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

    assert "nav_controller_mode:=challenge2_align_straight" in result.stdout
    assert "start_uav_target_receiver:=true" not in result.stdout
    assert "uav_target_input_mode:=serial" not in result.stdout


def test_run_challenge2_prompt_can_enable_esp_target_receiver_in_dry_run():
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
    assert "uav_esp_require_checksum:=true" in result.stdout
