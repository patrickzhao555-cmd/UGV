"""ROS-free helpers for the Teensy two-side PID motor controller."""

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from ugv_motor_controller.velocity_control import (
    command_is_timed_out,
    extract_raw_drive,
    extract_velocity_command,
    finite_float,
    velocity_to_wheel_speeds,
)


TEENSY_PID_LOCATION = "teensy_pid"
DEFAULT_MOTOR_CONTROL_LOCATION = "ros"


def normalize_motor_control_location(value: Any) -> str:
    location = str(value or DEFAULT_MOTOR_CONTROL_LOCATION).strip().lower()
    aliases = {
        "python": "ros",
        "python_pid": "ros",
        "ros_pid": "ros",
        "jetson_pid": "ros",
        "teensy": TEENSY_PID_LOCATION,
        "teensy_side_pid": TEENSY_PID_LOCATION,
    }
    return aliases.get(location, location or DEFAULT_MOTOR_CONTROL_LOCATION)


def uses_teensy_side_pid(value: Any) -> bool:
    return normalize_motor_control_location(value) == TEENSY_PID_LOCATION


def python_velocity_pid_enabled(motor_control_location: Any, velocity_control_enabled: bool) -> bool:
    return bool(velocity_control_enabled) and not uses_teensy_side_pid(motor_control_location)


def mps_to_ticks_per_sec(mps: float, *, wheel_radius_m: float, ticks_per_rev: int) -> float:
    radius = max(1e-6, float(wheel_radius_m))
    ticks = max(1, int(ticks_per_rev))
    return float(mps) * float(ticks) / (2.0 * math.pi * radius)


def ticks_per_sec_to_mps(ticks_per_sec: float, *, wheel_radius_m: float, ticks_per_rev: int) -> float:
    radius = max(1e-6, float(wheel_radius_m))
    ticks = max(1, int(ticks_per_rev))
    return float(ticks_per_sec) * (2.0 * math.pi * radius) / float(ticks)


def velocity_to_side_pid_targets(
    v_mps: float,
    omega_radps: float,
    *,
    track_width_m: float,
    wheel_radius_m: float,
    ticks_per_rev: int,
) -> Tuple[float, float, float, float]:
    left_mps, right_mps = velocity_to_wheel_speeds(v_mps, omega_radps, track_width_m)
    return (
        left_mps,
        right_mps,
        mps_to_ticks_per_sec(left_mps, wheel_radius_m=wheel_radius_m, ticks_per_rev=ticks_per_rev),
        mps_to_ticks_per_sec(right_mps, wheel_radius_m=wheel_radius_m, ticks_per_rev=ticks_per_rev),
    )


def build_teensy_velocity_command(v_mps: float, omega_radps: float) -> str:
    v = finite_float(v_mps, name="v_mps")
    omega = finite_float(omega_radps, name="omega_radps")
    return f"CMD V {v:.6f} {omega:.6f}\n"


def build_teensy_stop_command() -> str:
    return "CMD STOP\n"


def build_teensy_raw2_command(left_us: int, right_us: int) -> str:
    return f"CMD RAW2 {int(left_us)} {int(right_us)}\n"


def build_teensy_param_command(name: str, value: Any) -> str:
    param_name = str(name).strip()
    if not param_name:
        raise ValueError("Teensy parameter name must not be empty")
    if isinstance(value, bool):
        param_value = "1" if value else "0"
    elif isinstance(value, int):
        param_value = str(value)
    else:
        numeric = finite_float(value, name=param_name)
        param_value = f"{numeric:.6f}".rstrip("0").rstrip(".")
        if param_value == "-0":
            param_value = "0"
    return f"CMD PARAM {param_name} {param_value}\n"


def build_teensy_param_init_commands(params: Dict[str, Any]) -> Tuple[str, ...]:
    return tuple(build_teensy_param_command(name, value) for name, value in params.items())


def teensy_timeout_stop_command_due(
    last_command_time_s: float,
    now_s: float,
    timeout_s: float,
) -> Optional[str]:
    if command_is_timed_out(last_command_time_s, now_s, timeout_s):
        return build_teensy_stop_command()
    return None


def select_teensy_side_pid_command(
    obj: Dict[str, Any],
    *,
    prefer_velocity_fields: bool = True,
) -> Tuple[str, Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
    velocity_cmd = extract_velocity_command(
        obj,
        prefer_velocity_fields=prefer_velocity_fields,
    )
    if velocity_cmd is not None:
        return "velocity", velocity_cmd, None
    return "raw", None, extract_raw_drive(obj)


@dataclass(frozen=True)
class TeensySidePidStatus:
    controller_millis: int
    fl_ticks: int
    fr_ticks: int
    rl_ticks: int
    rr_ticks: int
    fl_tps: float
    fr_tps: float
    rl_tps: float
    rr_tps: float
    left_target_tps: float
    right_target_tps: float
    left_measured_tps: float
    right_measured_tps: float
    left_pwm: int
    right_pwm: int
    left_error_tps: float
    right_error_tps: float
    left_p: float
    left_i: float
    left_d: float
    right_p: float
    right_i: float
    right_d: float
    fault: str

    @property
    def side_ticks(self) -> Tuple[int, int]:
        return (
            int(round((self.fl_ticks + self.rl_ticks) / 2.0)),
            int(round((self.fr_ticks + self.rr_ticks) / 2.0)),
        )

    @property
    def wheel_ticks(self) -> Tuple[int, int, int, int]:
        return (self.fl_ticks, self.fr_ticks, self.rl_ticks, self.rr_ticks)

    @property
    def wheel_tps(self) -> Tuple[float, float, float, float]:
        return (self.fl_tps, self.fr_tps, self.rl_tps, self.rr_tps)

    def as_bridge_status_dict(self) -> Dict[str, Any]:
        return {
            "teensy_side_pid": True,
            "teensy_ms": self.controller_millis,
            "raw_ticks": list(self.wheel_ticks),
            "fl_tps": round(self.fl_tps, 3),
            "fr_tps": round(self.fr_tps, 3),
            "rl_tps": round(self.rl_tps, 3),
            "rr_tps": round(self.rr_tps, 3),
            "left_target_tps": round(self.left_target_tps, 3),
            "right_target_tps": round(self.right_target_tps, 3),
            "left_measured_tps": round(self.left_measured_tps, 3),
            "right_measured_tps": round(self.right_measured_tps, 3),
            "left_pwm": self.left_pwm,
            "right_pwm": self.right_pwm,
            "left_error_tps": round(self.left_error_tps, 3),
            "right_error_tps": round(self.right_error_tps, 3),
            "left_front_vs_rear_mismatch": round(abs(self.fl_tps - self.rl_tps), 3),
            "right_front_vs_rear_mismatch": round(abs(self.fr_tps - self.rr_tps), 3),
            "pid_left": {
                "p": round(self.left_p, 4),
                "i": round(self.left_i, 4),
                "d": round(self.left_d, 4),
            },
            "pid_right": {
                "p": round(self.right_p, 4),
                "i": round(self.right_i, 4),
                "d": round(self.right_d, 4),
            },
            "fault": self.fault,
            "fault_reason": None if self.fault in {"", "none", "0"} else self.fault,
            "last_pwm": [self.left_pwm, self.right_pwm],
            "target_pwm": [self.left_pwm, self.right_pwm],
        }


def parse_teensy_side_pid_status_line(line: str) -> TeensySidePidStatus:
    text = str(line).strip()
    if not text.startswith("S,"):
        raise ValueError(f"not a Teensy side PID status line: {line!r}")

    parts = text.split(",")
    if len(parts) < 25:
        raise ValueError(f"Teensy side PID status line has {len(parts)} fields; expected at least 25")

    try:
        return TeensySidePidStatus(
            controller_millis=int(parts[1]),
            fl_ticks=int(parts[2]),
            fr_ticks=int(parts[3]),
            rl_ticks=int(parts[4]),
            rr_ticks=int(parts[5]),
            fl_tps=float(parts[6]),
            fr_tps=float(parts[7]),
            rl_tps=float(parts[8]),
            rr_tps=float(parts[9]),
            left_target_tps=float(parts[10]),
            right_target_tps=float(parts[11]),
            left_measured_tps=float(parts[12]),
            right_measured_tps=float(parts[13]),
            left_pwm=int(round(float(parts[14]))),
            right_pwm=int(round(float(parts[15]))),
            left_error_tps=float(parts[16]),
            right_error_tps=float(parts[17]),
            left_p=float(parts[18]),
            left_i=float(parts[19]),
            left_d=float(parts[20]),
            right_p=float(parts[21]),
            right_i=float(parts[22]),
            right_d=float(parts[23]),
            fault=",".join(parts[24:]).strip() or "none",
        )
    except ValueError as exc:
        raise ValueError(f"failed to parse Teensy side PID status line: {line!r}") from exc
