"""Mission-level helpers for competition-safe chassis motion."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Sequence

from .chassis_controller import (
    ChassisControllerConfig,
    clamp,
    encoder_ticks_to_distance_m,
    motor_status_ready,
    slew_limit,
)


COMPETITION_MIN_SPEED_MPS = 0.2 * 0.44704
MOTION_EPSILON = 1e-6


@dataclass(frozen=True)
class MissionSegment:
    segment_type: str
    distance_m: float = 0.0
    angle_deg: float = 0.0
    speed_mps: Optional[float] = None
    max_omega_radps: Optional[float] = None
    timeout_s: Optional[float] = None
    wait_s: float = 0.0
    raw: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class MissionPlan:
    mission_id: str
    segments: tuple[MissionSegment, ...]
    source: str = ""


@dataclass(frozen=True)
class MissionSafetyDecision:
    level: str
    reason: str

    @property
    def ok(self) -> bool:
        return self.level == "ok"

    @property
    def critical(self) -> bool:
        return self.level == "critical"


def apply_competition_speed_rule(
    v_mps: float,
    *,
    allow_stop: bool = True,
    min_speed_mps: float = COMPETITION_MIN_SPEED_MPS,
) -> float:
    value = float(v_mps)
    if allow_stop and abs(value) < MOTION_EPSILON:
        return 0.0
    sign = 1.0 if value >= 0.0 else -1.0
    return sign * max(abs(value), max(0.0, float(min_speed_mps)))


def motion_rule_ok(v_mps: float, *, min_speed_mps: float = COMPETITION_MIN_SPEED_MPS) -> bool:
    value = abs(float(v_mps))
    return value < MOTION_EPSILON or value + 1e-9 >= max(0.0, float(min_speed_mps))


def _finite_float(value: Any, *, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _optional_finite_float(value: Any, *, name: str) -> Optional[float]:
    if value is None:
        return None
    return _finite_float(value, name=name)


def parse_mission_plan(data: Dict[str, Any], *, source: str = "") -> MissionPlan:
    if not isinstance(data, dict):
        raise ValueError("mission file root must be an object")
    mission_id = str(data.get("mission_id") or "mission").strip() or "mission"
    raw_segments = data.get("segments")
    if not isinstance(raw_segments, list) or not raw_segments:
        raise ValueError("mission must contain a non-empty segments list")

    segments: list[MissionSegment] = []
    for index, item in enumerate(raw_segments):
        if not isinstance(item, dict):
            raise ValueError(f"segment {index} must be an object")
        segment_type = str(item.get("type", "")).strip().lower()
        if segment_type not in {"straight", "pivot", "wait"}:
            raise ValueError(f"segment {index} has unsupported type {segment_type!r}")
        if segment_type == "straight":
            distance_m = _finite_float(item.get("distance_m"), name=f"segment {index} distance_m")
            if distance_m <= 0.0:
                raise ValueError(f"segment {index} distance_m must be positive")
            speed_mps = _optional_finite_float(item.get("speed_mps"), name=f"segment {index} speed_mps")
            if speed_mps is not None and speed_mps <= 0.0:
                raise ValueError(f"segment {index} speed_mps must be positive")
            timeout_s = _optional_finite_float(item.get("timeout_s"), name=f"segment {index} timeout_s")
            if timeout_s is not None and timeout_s <= 0.0:
                raise ValueError(f"segment {index} timeout_s must be positive")
            segments.append(
                MissionSegment(
                    segment_type="straight",
                    distance_m=distance_m,
                    speed_mps=speed_mps,
                    timeout_s=timeout_s,
                    raw=dict(item),
                )
            )
        elif segment_type == "pivot":
            angle_deg = _finite_float(item.get("angle_deg"), name=f"segment {index} angle_deg")
            if abs(angle_deg) < 1e-6:
                raise ValueError(f"segment {index} angle_deg must be non-zero")
            max_omega_radps = _optional_finite_float(
                item.get("max_omega_radps"),
                name=f"segment {index} max_omega_radps",
            )
            if max_omega_radps is not None and max_omega_radps <= 0.0:
                raise ValueError(f"segment {index} max_omega_radps must be positive")
            timeout_s = _optional_finite_float(item.get("timeout_s"), name=f"segment {index} timeout_s")
            if timeout_s is not None and timeout_s <= 0.0:
                raise ValueError(f"segment {index} timeout_s must be positive")
            segments.append(
                MissionSegment(
                    segment_type="pivot",
                    angle_deg=angle_deg,
                    max_omega_radps=max_omega_radps,
                    timeout_s=timeout_s,
                    raw=dict(item),
                )
            )
        else:
            wait_s = _finite_float(item.get("wait_s", 0.0), name=f"segment {index} wait_s")
            if wait_s < 0.0:
                raise ValueError(f"segment {index} wait_s must be non-negative")
            segments.append(MissionSegment(segment_type="wait", wait_s=wait_s, raw=dict(item)))

    return MissionPlan(mission_id=mission_id, segments=tuple(segments), source=source)


def load_mission_plan(path: str) -> MissionPlan:
    mission_path = Path(path).expanduser()
    text = mission_path.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml  # type: ignore
        except ImportError as exc:
            raise ValueError("mission file is not JSON and PyYAML is not installed") from exc
        data = yaml.safe_load(text)
    return parse_mission_plan(data, source=str(mission_path))


def segment_speed_mps(segment: MissionSegment, config: ChassisControllerConfig) -> float:
    requested = config.mission_default_speed_mps if segment.speed_mps is None else float(segment.speed_mps)
    return apply_competition_speed_rule(
        requested,
        allow_stop=False,
        min_speed_mps=float(config.competition_min_speed_mps),
    )


def segment_timeout_s(segment: MissionSegment, config: ChassisControllerConfig) -> Optional[float]:
    if segment.timeout_s is not None:
        return float(segment.timeout_s)
    if segment.segment_type == "straight":
        speed = max(abs(segment_speed_mps(segment, config)), MOTION_EPSILON)
        return max(3.0, float(segment.distance_m) / speed * 3.0 + 1.0)
    if segment.segment_type == "pivot":
        return max(0.0, float(config.pivot_timeout_s))
    return None


def encoder_average_distance_m(
    *,
    start_left_ticks: Optional[int],
    start_right_ticks: Optional[int],
    current_left_ticks: Optional[int],
    current_right_ticks: Optional[int],
    config: ChassisControllerConfig,
) -> float:
    if (
        start_left_ticks is None
        or start_right_ticks is None
        or current_left_ticks is None
        or current_right_ticks is None
    ):
        return 0.0
    left_m = encoder_ticks_to_distance_m(
        int(current_left_ticks) - int(start_left_ticks),
        wheel_radius_m=config.wheel_radius_m,
        ticks_per_rev=config.ticks_per_rev,
    )
    right_m = encoder_ticks_to_distance_m(
        int(current_right_ticks) - int(start_right_ticks),
        wheel_radius_m=config.wheel_radius_m,
        ticks_per_rev=config.ticks_per_rev,
    )
    return 0.5 * (left_m + right_m)


def classify_mission_safety(
    *,
    now_s: float,
    last_sensor_s: Optional[float],
    last_imu_s: Optional[float],
    last_motor_status_s: Optional[float],
    motor_status: Optional[Dict[str, Any]],
    near_obstacle: bool,
    front_clearance_m: Optional[float],
    config: ChassisControllerConfig,
    require_imu: bool = True,
) -> MissionSafetyDecision:
    if last_motor_status_s is None or now_s - last_motor_status_s > config.motor_status_timeout_s:
        return MissionSafetyDecision("critical", "motor_status_stale")
    motor = motor_status_ready(motor_status)
    if not motor.safe:
        return MissionSafetyDecision("critical", motor.reason)
    if require_imu and (last_imu_s is None or now_s - last_imu_s > config.imu_timeout_s):
        return MissionSafetyDecision("critical", "imu_stale")

    if last_sensor_s is None:
        return MissionSafetyDecision("degraded", "sensor_missing")
    sensor_age_s = now_s - last_sensor_s
    if sensor_age_s > config.mission_critical_sensor_timeout_s:
        return MissionSafetyDecision("critical", "sensor_critical_stale")
    if sensor_age_s > config.sensor_timeout_s:
        return MissionSafetyDecision("degraded", "sensor_stale")

    if front_clearance_m is None or math.isnan(front_clearance_m):
        return MissionSafetyDecision("degraded", "front_clearance_invalid")
    if math.isfinite(front_clearance_m) and front_clearance_m < config.mission_emergency_stop_clearance_m:
        return MissionSafetyDecision("critical", "front_clearance_emergency")
    if near_obstacle:
        return MissionSafetyDecision("degraded", "near_obstacle")
    if math.isfinite(front_clearance_m) and front_clearance_m < config.stop_clearance_m:
        return MissionSafetyDecision("degraded", "front_clearance_low")
    return MissionSafetyDecision("ok", "ok")


def straight_omega_with_slew(
    *,
    heading_error_rad: float,
    yaw_rate_radps: float,
    previous_omega_radps: float,
    dt_s: float,
    config: ChassisControllerConfig,
) -> float:
    control_error = 0.0 if abs(heading_error_rad) < config.heading_deadband_rad else heading_error_rad
    omega_target = config.heading_kp * control_error - config.heading_kd * yaw_rate_radps
    straight_max = max(0.0, min(float(config.max_omega_radps), float(config.mission_straight_max_omega_radps)))
    omega_target = clamp(omega_target, -straight_max, straight_max)
    max_delta = max(0.0, float(config.mission_straight_omega_slew_radps2)) * max(0.0, float(dt_s))
    return slew_limit(previous_omega_radps, omega_target, max_delta)


def heading_error_rms(samples: Sequence[float]) -> float:
    if not samples:
        return 0.0
    return math.sqrt(sum(float(sample) ** 2 for sample in samples) / float(len(samples)))


class MissionTelemetryRecorder:
    def __init__(self, *, enabled: bool, telemetry_dir: str, mission_id: str) -> None:
        self.enabled = bool(enabled)
        self.path: Optional[Path] = None
        self._file = None
        if not self.enabled:
            return
        safe_id = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in mission_id) or "mission"
        log_dir = Path(telemetry_dir).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"{stamp}_{safe_id}.jsonl"
        self._file = self.path.open("a", encoding="utf-8")

    def write(self, record: Dict[str, Any]) -> None:
        if self._file is None:
            return
        self._file.write(json.dumps(record, sort_keys=True) + "\n")
        self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None


def summarize_mission_records(records: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    by_segment: Dict[int, Dict[str, Any]] = {}
    sub_min_count = 0
    critical_count = 0
    sensor_stale_count = 0
    for record in records:
        if not bool(record.get("motion_rule_ok", True)):
            sub_min_count += 1
        if str(record.get("safety_level", "")) == "critical":
            critical_count += 1
        if "stale" in str(record.get("safety_reason", "")):
            sensor_stale_count += 1
        index = record.get("segment_index")
        if index is None:
            continue
        try:
            segment_index = int(index)
        except (TypeError, ValueError):
            continue
        summary = by_segment.setdefault(
            segment_index,
            {
                "segment_type": record.get("segment_type"),
                "samples": 0,
                "last_distance_m": 0.0,
                "target_distance_m": None,
                "distance_error_m": None,
                "last_heading_error_rad": 0.0,
                "final_heading_error_rad": 0.0,
                "max_abs_heading_error_rad": 0.0,
                "heading_error_rms_rad": 0.0,
                "pivot_overshoot_rad": 0.0,
                "_heading_error_sq_sum": 0.0,
                "_first_heading_error_rad": None,
            },
        )
        summary["samples"] += 1
        summary["segment_type"] = record.get("segment_type", summary.get("segment_type"))
        summary["last_distance_m"] = float(record.get("segment_distance_m") or summary["last_distance_m"])
        target_distance = record.get("target_distance_m")
        if target_distance is not None:
            summary["target_distance_m"] = float(target_distance)
        heading_error = float(record.get("heading_error_rad") or 0.0)
        summary["last_heading_error_rad"] = heading_error
        summary["final_heading_error_rad"] = heading_error
        summary["max_abs_heading_error_rad"] = max(summary["max_abs_heading_error_rad"], abs(heading_error))
        summary["_heading_error_sq_sum"] += heading_error ** 2
        first_error = summary["_first_heading_error_rad"]
        if first_error is None and abs(heading_error) > 1e-9:
            summary["_first_heading_error_rad"] = heading_error
            first_error = heading_error
        if first_error is not None and heading_error * float(first_error) < 0.0:
            summary["pivot_overshoot_rad"] = max(summary["pivot_overshoot_rad"], abs(heading_error))

    for summary in by_segment.values():
        samples = max(1, int(summary["samples"]))
        summary["heading_error_rms_rad"] = math.sqrt(float(summary["_heading_error_sq_sum"]) / float(samples))
        if summary["target_distance_m"] is not None:
            summary["distance_error_m"] = float(summary["last_distance_m"]) - float(summary["target_distance_m"])
        summary.pop("_heading_error_sq_sum", None)
        summary.pop("_first_heading_error_rad", None)

    return {
        "segments": by_segment,
        "sub_min_speed_command_count": sub_min_count,
        "critical_stop_count": critical_count,
        "sensor_stale_count": sensor_stale_count,
    }
