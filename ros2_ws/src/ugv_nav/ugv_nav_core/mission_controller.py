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


MIN_RULE_SPEED_MPH = 0.2
MIN_RULE_SPEED_MPS = 0.0894
MOVING_TARGET_SPEED_MPS = 0.12
COMPETITION_MIN_SPEED_MPS = MIN_RULE_SPEED_MPS
MOTION_EPSILON = 1e-6

WAITING_TO_START_PHASES = frozenset({"init", "idle", "waiting_to_start"})
ACTIVE_MOVEMENT_PHASES = frozenset(
    {
        "active_movement",
        "path_following",
        "replanning",
        "marker_search",
        "terminal_approach",
        "uav_landing_support",
    }
)
STOP_ALLOWED_PHASES = WAITING_TO_START_PHASES | frozenset(
    {"arrived", "destination_reached", "kill_switch", "e_stop", "fault", "safety_stop"}
)


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


@dataclass
class StuckMonitorState:
    command_kind: str = "stop"
    segment_key: Optional[str] = None
    command_start_s: Optional[float] = None
    last_response_s: Optional[float] = None
    recovery_count: int = 0
    stuck_detected: bool = False
    reason: str = "ok"


@dataclass(frozen=True)
class StuckMonitorResult:
    stuck: bool
    reason: str
    recovery_count: int


@dataclass(frozen=True)
class ContinuousMotionDecision:
    v_mps: float
    phase: str
    changed: bool = False
    zero_replaced: bool = False
    sub_min_clamped: bool = False
    active_violation: bool = False
    reason: str = "ok"


def normalized_competition_motion_phase(value: str) -> str:
    phase = str(value or "").strip().lower()
    if not phase:
        return "waiting_to_start"
    aliases = {
        "wait": "waiting_to_start",
        "waiting": "waiting_to_start",
        "preflight": "waiting_to_start",
        "started": "active_movement",
        "moving": "active_movement",
        "nav2_to_staging": "path_following",
        "destination_stop": "destination_reached",
        "mission_complete": "destination_reached",
        "abort": "fault",
        "estop": "e_stop",
        "emergency_stop": "e_stop",
    }
    return aliases.get(phase, phase)


def phase_requires_continuous_motion(phase: str) -> bool:
    return normalized_competition_motion_phase(phase) in ACTIVE_MOVEMENT_PHASES


def phase_allows_zero_speed(phase: str) -> bool:
    normalized = normalized_competition_motion_phase(phase)
    return normalized in STOP_ALLOWED_PHASES or normalized not in ACTIVE_MOVEMENT_PHASES


def apply_competition_speed_rule(
    v_mps: float,
    *,
    allow_stop: bool = True,
    min_speed_mps: float = COMPETITION_MIN_SPEED_MPS,
    moving_target_speed_mps: float = MOVING_TARGET_SPEED_MPS,
) -> float:
    value = float(v_mps)
    raw_min_speed = float(min_speed_mps)
    raw_moving_target = float(moving_target_speed_mps)
    if not math.isfinite(value) or not math.isfinite(raw_min_speed):
        return 0.0
    min_speed = max(0.0, raw_min_speed)
    moving_target = max(0.0, raw_moving_target) if math.isfinite(raw_moving_target) else min_speed
    if allow_stop and abs(value) < MOTION_EPSILON:
        return 0.0
    sign = 1.0 if value >= 0.0 else -1.0
    target = max(min_speed, moving_target)
    return sign * max(abs(value), target)


def enforce_continuous_movement_speed(
    v_mps: float,
    *,
    phase: str,
    min_speed_mps: float = MIN_RULE_SPEED_MPS,
    moving_target_speed_mps: float = MOVING_TARGET_SPEED_MPS,
) -> ContinuousMotionDecision:
    """Enforce the competition active-movement speed rule for autonomous modes.

    Once the UGV is in an active travel phase, normal zero-speed or sub-minimum
    translational commands are replaced by a crawl above the 0.2 mph minimum.
    Safety/fault/arrival phases are intentionally allowed to command STOP.
    """

    normalized = normalized_competition_motion_phase(phase)
    value = float(v_mps)
    raw_min_speed = float(min_speed_mps)
    raw_moving_target = float(moving_target_speed_mps)
    if not math.isfinite(value) or not math.isfinite(raw_min_speed):
        return ContinuousMotionDecision(0.0, normalized, changed=True, reason="invalid_speed_rule_input")
    min_speed = max(0.0, raw_min_speed)
    moving_target = max(0.0, raw_moving_target) if math.isfinite(raw_moving_target) else min_speed
    target = max(min_speed, moving_target)
    if not phase_requires_continuous_motion(normalized):
        return ContinuousMotionDecision(value, normalized, reason="stop_allowed" if abs(value) < MOTION_EPSILON else "ok")
    if abs(value) < MOTION_EPSILON:
        return ContinuousMotionDecision(
            target,
            normalized,
            changed=True,
            zero_replaced=True,
            active_violation=True,
            reason="active_zero_speed_replaced",
        )
    if abs(value) + 1e-9 < min_speed:
        sign = 1.0 if value >= 0.0 else -1.0
        return ContinuousMotionDecision(
            sign * target,
            normalized,
            changed=True,
            sub_min_clamped=True,
            active_violation=True,
            reason="active_sub_min_speed_clamped",
        )
    return ContinuousMotionDecision(value, normalized, reason="ok")


def motion_rule_ok(v_mps: float, *, min_speed_mps: float = COMPETITION_MIN_SPEED_MPS) -> bool:
    value = abs(float(v_mps))
    return value < MOTION_EPSILON or value + 1e-9 >= max(0.0, float(min_speed_mps))


def normalized_imu_qos(value: str) -> str:
    text = str(value).strip().lower().replace("-", "_")
    if text in {"sensor", "sensor_data", "best_effort", "besteffort"}:
        return "sensor_data"
    if text in {"default", "reliable"}:
        return "default"
    raise ValueError(f"unsupported IMU QoS {value!r}")


def telemetry_force_flush_key(
    *,
    mission_state: str,
    safety_level: str,
    safety_reason: str,
    segment_index: Optional[int],
) -> Optional[str]:
    state = str(mission_state)
    level = str(safety_level)
    if state not in {"mission_complete", "abort"} and level != "critical":
        return None
    return f"{state}:{level}:{safety_reason}:{segment_index}"


def mission_segment_start_hold_reason(
    segment: MissionSegment,
    *,
    safety_level: str,
    safety_reason: str,
    encoder_available: bool,
    left_ticks: Optional[int],
    right_ticks: Optional[int],
    pivot_clearance_reason: Optional[str],
    config: ChassisControllerConfig,
) -> Optional[str]:
    if str(safety_level) == "critical":
        return str(safety_reason)

    reason = str(safety_reason)
    if str(safety_level) == "degraded":
        if reason in {"sensor_missing", "sensor_stale", "front_clearance_invalid"}:
            return reason
        if (
            segment.segment_type == "straight"
            and bool(config.mission_stop_on_degraded_obstacle)
            and reason in {"near_obstacle", "front_clearance_low"}
        ):
            return reason

    if segment.segment_type == "straight":
        if not encoder_available or left_ticks is None or right_ticks is None:
            return "encoder_unavailable"

    if segment.segment_type == "pivot" and pivot_clearance_reason:
        return str(pivot_clearance_reason)

    return None


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
            if abs(angle_deg) > 180.0:
                raise ValueError(f"segment {index} angle_deg must be between -180 and 180 degrees")
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
        moving_target_speed_mps=float(config.competition_moving_target_speed_mps),
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
    imu_rate_hz: Optional[float] = None,
) -> MissionSafetyDecision:
    if last_motor_status_s is None or now_s - last_motor_status_s > config.motor_status_timeout_s:
        return MissionSafetyDecision("critical", "motor_status_stale")
    motor = motor_status_ready(motor_status)
    if not motor.safe:
        return MissionSafetyDecision("critical", motor.reason)
    if require_imu and (last_imu_s is None or now_s - last_imu_s > config.imu_timeout_s):
        return MissionSafetyDecision("critical", "imu_stale")
    if (
        require_imu
        and imu_rate_hz is not None
        and float(config.imu_min_rate_hz) > 0.0
        and float(imu_rate_hz) < float(config.imu_min_rate_hz)
    ):
        return MissionSafetyDecision("critical", "imu_rate_low")

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


def average_abs_measured_speed_mps(motor_status: Optional[Dict[str, Any]]) -> float:
    if not isinstance(motor_status, dict):
        return 0.0
    try:
        left = abs(float(motor_status.get("measured_left_mps", 0.0) or 0.0))
        right = abs(float(motor_status.get("measured_right_mps", 0.0) or 0.0))
    except (TypeError, ValueError):
        return 0.0
    return 0.5 * (left + right)


def reset_stuck_monitor(state: StuckMonitorState, *, keep_recovery_count: bool = False) -> None:
    recovery_count = state.recovery_count if keep_recovery_count else 0
    state.command_kind = "stop"
    state.segment_key = None
    state.command_start_s = None
    state.last_response_s = None
    state.recovery_count = recovery_count
    state.stuck_detected = False
    state.reason = "ok"


def update_stuck_monitor(
    state: StuckMonitorState,
    *,
    now_s: float,
    segment_key: str,
    command_kind: str,
    v_cmd_mps: float,
    omega_cmd_radps: float,
    yaw_rate_radps: float,
    motor_status: Optional[Dict[str, Any]],
    config: ChassisControllerConfig,
) -> StuckMonitorResult:
    if not config.stuck_detection_enabled:
        reset_stuck_monitor(state)
        return StuckMonitorResult(False, "disabled", state.recovery_count)

    dry_run_value = motor_status.get("dry_run", False) if isinstance(motor_status, dict) else False
    dry_run = dry_run_value if isinstance(dry_run_value, bool) else str(dry_run_value).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    if dry_run:
        reset_stuck_monitor(state)
        state.reason = "dry_run_bypass"
        return StuckMonitorResult(False, "dry_run_bypass", state.recovery_count)

    if command_kind == "pivot":
        moving = abs(float(omega_cmd_radps)) >= max(0.0, float(config.pivot_stuck_min_yaw_rate_radps))
    elif command_kind == "straight":
        moving = abs(float(v_cmd_mps)) >= max(MOTION_EPSILON, float(config.straight_stuck_min_measured_mps))
    else:
        moving = abs(float(v_cmd_mps)) > MOTION_EPSILON or abs(float(omega_cmd_radps)) > MOTION_EPSILON
    if command_kind == "stop" or not moving:
        reset_stuck_monitor(state)
        return StuckMonitorResult(False, "ok", state.recovery_count)

    if state.segment_key != segment_key or state.command_kind != command_kind:
        state.command_kind = command_kind
        state.segment_key = segment_key
        state.command_start_s = float(now_s)
        state.last_response_s = float(now_s)
        state.stuck_detected = False
        state.reason = "ok"

    if command_kind == "straight":
        responding = average_abs_measured_speed_mps(motor_status) >= max(
            0.0,
            float(config.straight_stuck_min_measured_mps),
        )
        timeout_s = max(0.0, float(config.straight_stuck_timeout_s))
        reason = "straight_stuck"
    elif command_kind == "pivot":
        responding = abs(float(yaw_rate_radps)) >= max(0.0, float(config.pivot_stuck_min_yaw_rate_radps))
        timeout_s = max(0.0, float(config.pivot_stuck_timeout_s))
        reason = "pivot_stuck"
    else:
        reset_stuck_monitor(state)
        return StuckMonitorResult(False, "ok", state.recovery_count)

    if responding:
        state.last_response_s = float(now_s)
        state.stuck_detected = False
        state.reason = "ok"
        return StuckMonitorResult(False, "ok", state.recovery_count)

    reference_s = state.last_response_s if state.last_response_s is not None else state.command_start_s
    reference_s = float(reference_s) if reference_s is not None else float(now_s)
    if now_s - reference_s >= timeout_s:
        state.stuck_detected = True
        state.reason = reason
        return StuckMonitorResult(True, reason, state.recovery_count)

    state.stuck_detected = False
    state.reason = "ok"
    return StuckMonitorResult(False, "ok", state.recovery_count)


class MissionTelemetryRecorder:
    def __init__(
        self,
        *,
        enabled: bool,
        telemetry_dir: str,
        mission_id: str,
        flush_period_s: float = 0.50,
        flush_max_records: int = 25,
    ) -> None:
        self.enabled = bool(enabled)
        self.path: Optional[Path] = None
        self._file = None
        self.error: Optional[str] = None
        self.failed = False
        self.flush_period_s = max(0.0, float(flush_period_s))
        self.flush_max_records = max(1, int(flush_max_records))
        self._pending_records = 0
        self._last_flush_s: Optional[float] = None
        if not self.enabled:
            return
        safe_id = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in mission_id) or "mission"
        log_dir = Path(telemetry_dir).expanduser()
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.path = log_dir / f"{stamp}_{safe_id}.jsonl"
            self._file = self.path.open("a", encoding="utf-8")
        except OSError as exc:
            self._mark_failed(exc)

    def _mark_failed(self, exc: OSError) -> None:
        self.error = str(exc)
        self.failed = True
        self.enabled = False
        file_obj = self._file
        self._file = None
        if file_obj is not None:
            try:
                file_obj.close()
            except OSError:
                pass

    def write(
        self,
        record: Dict[str, Any],
        *,
        now_s: Optional[float] = None,
        force_flush: bool = False,
    ) -> None:
        if self._file is None or self.failed:
            return
        now = time.monotonic() if now_s is None else float(now_s)
        if self._last_flush_s is None:
            self._last_flush_s = now
        try:
            self._file.write(json.dumps(record, sort_keys=True) + "\n")
        except OSError as exc:
            self._mark_failed(exc)
            return
        self._pending_records += 1
        period_elapsed = self.flush_period_s > 0.0 and (now - self._last_flush_s) >= self.flush_period_s
        max_records_reached = self._pending_records >= self.flush_max_records
        if force_flush or period_elapsed or max_records_reached:
            self.flush(now_s=now)

    def flush(self, *, now_s: Optional[float] = None) -> None:
        if self._file is None or self.failed:
            return
        try:
            self._file.flush()
        except OSError as exc:
            self._mark_failed(exc)
            return
        self._pending_records = 0
        self._last_flush_s = time.monotonic() if now_s is None else float(now_s)

    def close(self) -> None:
        if self._file is not None:
            self.flush()
        if self._file is not None:
            try:
                self._file.close()
            except OSError as exc:
                self._mark_failed(exc)
                return
            self._file = None


def summarize_mission_records(records: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    by_segment: Dict[int, Dict[str, Any]] = {}
    sub_min_count = 0
    critical_count = 0
    sensor_stale_count = 0
    saturation_count = 0
    stop_reasons: Dict[str, int] = {}
    for record in records:
        if not bool(record.get("motion_rule_ok", True)):
            sub_min_count += 1
        if str(record.get("safety_level", "")) == "critical":
            critical_count += 1
        if "stale" in str(record.get("safety_reason", "")):
            sensor_stale_count += 1
        if bool(record.get("omega_saturated", False)):
            saturation_count += 1
        reason = record.get("last_command", {}).get("reason") if isinstance(record.get("last_command"), dict) else None
        if reason is None:
            reason = record.get("safety_reason")
        if reason:
            reason_text = str(reason)
            stop_reasons[reason_text] = stop_reasons.get(reason_text, 0) + 1
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
        "omega_saturation_count": saturation_count,
        "stop_reasons": stop_reasons,
    }
