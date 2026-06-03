"""Protocol helpers for the active Teensy two-controller/four-encoder motor controller.

Hardware truth for this robot:

* Four Pololu 50:1 37D motors with quadrature encoders.
* Four encoder channels, one per motor.
* Two goBILDA 1x15A R/C PWM speed controllers: one left side and one right side.

That means the active controller is left/right side velocity PID, not four
independent motor PID. Four encoder streams are still useful for averaged side
feedback and diagnostics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple


def finite_float(value: Any, *, name: str) -> float:
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite")
    return out


def is_stop_command(obj: Dict[str, Any]) -> bool:
    mode = str(obj.get("mode", "")).upper()
    command_type = str(obj.get("command_type", "")).lower()
    return mode == "STOP" or command_type == "stop"


def extract_velocity_command(obj: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    command_type = str(obj.get("command_type", "")).lower()
    controller = str(obj.get("controller", "")).lower()
    mode = str(obj.get("mode", "")).upper()
    has_velocity_fields = "v_mps" in obj and "omega_radps" in obj
    explicit_velocity = command_type == "velocity" or controller == "velocity" or mode == "VELOCITY"
    if not has_velocity_fields or not explicit_velocity:
        return None
    return (
        finite_float(obj["v_mps"], name="v_mps"),
        finite_float(obj["omega_radps"], name="omega_radps"),
    )


def velocity_to_side_speeds(v_mps: float, omega_radps: float, track_width_m: float) -> Tuple[float, float]:
    half_track = 0.5 * max(1e-6, float(track_width_m))
    return (
        float(v_mps) - float(omega_radps) * half_track,
        float(v_mps) + float(omega_radps) * half_track,
    )


def side_speeds_to_velocity(left_mps: float, right_mps: float, track_width_m: float) -> Tuple[float, float]:
    track = max(1e-6, float(track_width_m))
    left = float(left_mps)
    right = float(right_mps)
    return (
        0.5 * (left + right),
        (right - left) / track,
    )


def _scaled_side_speed(side_mps: float, *, forward_scale: float, reverse_scale: float) -> float:
    speed = float(side_mps)
    if speed > 0.0:
        return speed * max(0.0, float(forward_scale))
    if speed < 0.0:
        return speed * max(0.0, float(reverse_scale))
    return 0.0


def apply_side_speed_scales(
    left_mps: float,
    right_mps: float,
    *,
    left_forward_scale: float = 1.0,
    right_forward_scale: float = 1.0,
    left_reverse_scale: float = 1.0,
    right_reverse_scale: float = 1.0,
) -> Tuple[float, float]:
    return (
        _scaled_side_speed(
            left_mps,
            forward_scale=left_forward_scale,
            reverse_scale=left_reverse_scale,
        ),
        _scaled_side_speed(
            right_mps,
            forward_scale=right_forward_scale,
            reverse_scale=right_reverse_scale,
        ),
    )


def mps_to_ticks_per_sec(mps: float, *, wheel_radius_m: float, ticks_per_rev: int) -> float:
    radius = max(1e-6, float(wheel_radius_m))
    ticks = max(1, int(ticks_per_rev))
    return float(mps) * float(ticks) / (2.0 * math.pi * radius)


def ticks_per_sec_to_mps(ticks_per_sec: float, *, wheel_radius_m: float, ticks_per_rev: int) -> float:
    radius = max(1e-6, float(wheel_radius_m))
    ticks = max(1, int(ticks_per_rev))
    return float(ticks_per_sec) * (2.0 * math.pi * radius) / float(ticks)


def side_average(front_value: float, rear_value: float) -> float:
    return 0.5 * (float(front_value) + float(rear_value))


def side_ticks_from_wheels(front_ticks: int, rear_ticks: int) -> int:
    return int(round(side_average(int(front_ticks), int(rear_ticks))))


def side_encoder_mismatch(front_tps: float, rear_tps: float) -> float:
    return abs(float(front_tps) - float(rear_tps))


def side_mismatch_flags(
    front_tps: float,
    rear_tps: float,
    *,
    warn_tps: float,
    fault_tps: float,
) -> Tuple[bool, bool]:
    mismatch = side_encoder_mismatch(front_tps, rear_tps)
    warn = mismatch >= max(0.0, float(warn_tps))
    fault = mismatch >= max(0.0, float(fault_tps))
    return warn, fault


def velocity_to_side_pid_targets(
    v_mps: float,
    omega_radps: float,
    *,
    track_width_m: float,
    wheel_radius_m: float,
    ticks_per_rev: int,
) -> Tuple[float, float, float, float]:
    left_mps, right_mps = velocity_to_side_speeds(v_mps, omega_radps, track_width_m)
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


@dataclass(frozen=True)
class TeensyParamAck:
    name: str
    status: str

    @property
    def ok(self) -> bool:
        return self.status == "ok"


def parse_teensy_param_ack_line(line: str) -> TeensyParamAck:
    text = str(line).strip()
    parts = text.split(",")
    if len(parts) != 3 or parts[0] != "PARAM":
        raise ValueError(f"not a Teensy PARAM ack line: {line!r}")
    name = parts[1].strip()
    status = parts[2].strip().lower()
    if not name:
        raise ValueError(f"Teensy PARAM ack missing parameter name: {line!r}")
    if status not in {"ok", "unknown"}:
        raise ValueError(f"Teensy PARAM ack has unsupported status: {line!r}")
    return TeensyParamAck(name=name, status=status)


@dataclass
class TeensyParamSyncTracker:
    expected_names: Tuple[str, ...] = ()
    pending_names: Tuple[str, ...] = ()
    failed: bool = False
    reason: str = "not_started"
    started_s: Optional[float] = None
    completed_s: Optional[float] = None
    acked_count: int = 0

    @property
    def synced(self) -> bool:
        return self.started_s is not None and not self.failed and not self.pending_names

    def start(self, names: Iterable[str], now_s: float) -> None:
        unique_names: List[str] = []
        seen = set()
        for raw_name in names:
            name = str(raw_name).strip()
            if name and name not in seen:
                unique_names.append(name)
                seen.add(name)
        self.expected_names = tuple(unique_names)
        self.pending_names = tuple(unique_names)
        self.failed = False
        self.reason = "waiting_for_ack" if unique_names else "ok"
        self.started_s = float(now_s)
        self.completed_s = float(now_s) if not unique_names else None
        self.acked_count = 0

    def handle_ack(self, ack: TeensyParamAck, now_s: float) -> bool:
        if self.started_s is None:
            self.failed = True
            self.reason = f"ack_without_sync:{ack.name}"
            return False
        if self.failed:
            return False
        if ack.name not in self.expected_names:
            self.failed = True
            self.reason = f"unexpected_param_ack:{ack.name}"
            return False
        if not ack.ok:
            self.failed = True
            self.reason = f"param_unknown:{ack.name}"
            return False
        if ack.name in self.pending_names:
            self.pending_names = tuple(name for name in self.pending_names if name != ack.name)
            self.acked_count += 1
        if not self.pending_names:
            self.reason = "ok"
            self.completed_s = float(now_s)
            return True
        self.reason = "waiting_for_ack"
        return False

    def mark_write_failed(self, name: str) -> None:
        self.failed = True
        self.reason = f"write_failed:{str(name).strip() or 'unknown'}"

    def mark_disconnected(self) -> None:
        self.failed = True
        self.reason = "serial_disconnected"
        self.pending_names = ()

    def check_timeout(self, now_s: float, timeout_s: float) -> bool:
        if self.started_s is None or self.failed or not self.pending_names:
            return False
        if float(now_s) - self.started_s < float(timeout_s):
            return False
        self.failed = True
        self.reason = f"ack_timeout:{','.join(self.pending_names)}"
        return True


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
    def wheel_ticks(self) -> Tuple[int, int, int, int]:
        return (self.fl_ticks, self.fr_ticks, self.rl_ticks, self.rr_ticks)

    @property
    def side_ticks(self) -> Tuple[int, int]:
        return (
            side_ticks_from_wheels(self.fl_ticks, self.rl_ticks),
            side_ticks_from_wheels(self.fr_ticks, self.rr_ticks),
        )

    def as_status_dict(self) -> Dict[str, Any]:
        return {
            "teensy_side_pid": True,
            "teensy_ms": self.controller_millis,
            "raw_ticks": list(self.wheel_ticks),
            "left_ticks": self.side_ticks[0],
            "right_ticks": self.side_ticks[1],
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
            "left_front_vs_rear_mismatch": round(side_encoder_mismatch(self.fl_tps, self.rl_tps), 3),
            "right_front_vs_rear_mismatch": round(side_encoder_mismatch(self.fr_tps, self.rr_tps), 3),
            "pid_left": {"p": round(self.left_p, 4), "i": round(self.left_i, 4), "d": round(self.left_d, 4)},
            "pid_right": {"p": round(self.right_p, 4), "i": round(self.right_i, 4), "d": round(self.right_d, 4)},
            "fault": self.fault,
            "fault_reason": None if self.fault in {"", "none", "0"} else self.fault,
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
