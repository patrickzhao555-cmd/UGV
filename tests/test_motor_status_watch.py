import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ugv_motor_status_watch import summarize_status  # noqa: E402


def test_summarize_status_includes_side_targets_measurements_and_pwm():
    line = summarize_status(
        {
            "commanded_v_mps": 0.3,
            "commanded_omega_radps": 1.0577,
            "target_left_mps": 0.08,
            "target_right_mps": 0.52,
            "measured_left_mps": 0.07,
            "measured_right_mps": 0.20,
            "left_pwm": 1580,
            "right_pwm": 2000,
            "fault": "none",
        }
    )
    assert "target L/R=" in line
    assert "meas L/R=" in line
    assert "pwm L/R=1580/2000" in line
    assert "fault=none" in line


def test_summarize_status_can_include_nav_heading_telemetry():
    line = summarize_status(
        {
            "commanded_v_mps": 0.16,
            "commanded_omega_radps": -0.2,
            "target_left_mps": 0.20,
            "target_right_mps": 0.12,
            "measured_left_mps": 0.19,
            "measured_right_mps": 0.13,
            "left_pwm": 1600,
            "right_pwm": 1550,
        },
        {
            "heading_error_rad": -0.05,
            "yaw_rate_radps": -0.1,
            "omega_radps": -0.25,
        },
    )
    assert "nav err=" in line
    assert "yaw_rate=" in line
    assert "nav_w=" in line
    assert "target L/R=" in line
