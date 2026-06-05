#!/usr/bin/env python3
"""Run repeatable UGV motion primitive field tests.

The runner intentionally does not add motion-control logic. It launches the
existing ROS 2 bringup with test or mission controller modes, records lightweight
status/command JSONL samples, prompts for manual measurements, and writes a
small report bundle for later tuning.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence


STATUS_TOPIC = "/ugv_nav_status"
CMD_TOPIC = "/ugv_nav_cmd"
BAG_TOPICS = (
    "/ugv_nav_status",
    "/ugv_nav_cmd",
    "/motor_controller/status",
    "/sensors/nav_frame",
    "/zed/imu",
    "/encoder_ticks",
)
LAUNCH_PACKAGE = "ugv_sensor_sync"
LAUNCH_FILE = "competition_bringup.launch.py"
DEFAULT_COMPETITION_MIN_SPEED_MPS = 0.0894
DEFAULT_ARC_MIN_TURN_RADIUS_M = 0.75
DEFAULT_ARC_MAX_OMEGA_RADPS = 0.45
DEFAULT_TRACK_WIDTH_M = 0.416


@dataclass(frozen=True)
class MotionTestCase:
    id: str
    mode: str
    description: str = ""
    expected_distance_m: Optional[float] = None
    expected_angle_deg: Optional[float] = None
    speed_mps: Optional[float] = None
    duration_s: Optional[float] = None
    max_case_duration_s: Optional[float] = None
    repeat: int = 1
    launch_args: dict[str, str] = field(default_factory=dict)
    mission: Optional[dict[str, Any]] = None


@dataclass(frozen=True)
class MotionTestSuite:
    suite_id: str
    cases: tuple[MotionTestCase, ...]


@dataclass
class TopicSample:
    received_time_s: float
    topic: str
    data: dict[str, Any]


def _fmt_float(value: float) -> str:
    return f"{float(value):.6g}"


def estimate_curve_watchdog_s(
    *,
    angle_deg: float,
    radius_m: float,
    speed_mps: float,
    min_turn_radius_m: float = DEFAULT_ARC_MIN_TURN_RADIUS_M,
    max_omega_radps: float = DEFAULT_ARC_MAX_OMEGA_RADPS,
    track_width_m: float = DEFAULT_TRACK_WIDTH_M,
    competition_min_speed_mps: float = DEFAULT_COMPETITION_MIN_SPEED_MPS,
) -> float:
    angle_rad = abs(math.radians(float(angle_deg)))
    if angle_rad <= 1e-9:
        return 3.0
    speed = max(abs(float(speed_mps)), max(0.0, float(competition_min_speed_mps)))
    radius = max(abs(float(radius_m)), max(1e-6, float(min_turn_radius_m)))
    omega_abs = min(
        speed / radius,
        max(0.0, float(max_omega_radps)),
        2.0 * speed / max(1e-6, float(track_width_m)),
    )
    if omega_abs <= 1e-9:
        return 3.0
    nominal_s = angle_rad / omega_abs
    return max(4.0, nominal_s * 1.6 + 1.5)


def parse_optional_float(text: str) -> Optional[float]:
    value = str(text).strip()
    if not value or value.lower() in {"skip", "n/a", "na", "none"}:
        return None
    return float(value)


def parse_launch_arg(text: str) -> tuple[str, str]:
    if ":=" not in text:
        raise argparse.ArgumentTypeError(f"launch argument must look like name:=value, got {text!r}")
    key, value = text.split(":=", 1)
    key = key.strip()
    if not key:
        raise argparse.ArgumentTypeError(f"launch argument has an empty name: {text!r}")
    return key, value


def parse_launch_arg_list(values: Sequence[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in values:
        key, value = parse_launch_arg(item)
        result[key] = value
    return result


def validate_case(case: MotionTestCase) -> None:
    if not case.id or any(ch.isspace() for ch in case.id):
        raise ValueError(f"invalid case id {case.id!r}")
    if case.mode not in {"idle", "straight_test", "pivot_test", "curve_test", "mission_sequence"}:
        raise ValueError(f"case {case.id}: unsupported mode {case.mode!r}")
    if case.repeat < 1:
        raise ValueError(f"case {case.id}: repeat must be >= 1")
    if case.mode == "straight_test":
        if case.speed_mps is None or case.speed_mps <= 0.0:
            raise ValueError(f"case {case.id}: straight_test requires positive speed_mps")
        if case.duration_s is None or case.duration_s <= 0.0:
            raise ValueError(f"case {case.id}: straight_test requires positive duration_s")
    if case.mode == "pivot_test":
        if case.expected_angle_deg is None or abs(case.expected_angle_deg) < 1e-9:
            raise ValueError(f"case {case.id}: pivot_test requires non-zero expected_angle_deg")
    if case.mode == "curve_test":
        if case.expected_angle_deg is None or abs(case.expected_angle_deg) < 1e-9:
            raise ValueError(f"case {case.id}: curve_test requires non-zero expected_angle_deg")
        radius = float(case.launch_args.get("nav_curve_radius_m", 0.0) or 0.0)
        if radius <= 0.0:
            raise ValueError(f"case {case.id}: curve_test requires positive nav_curve_radius_m")
    if case.mode == "mission_sequence":
        if not isinstance(case.mission, dict):
            raise ValueError(f"case {case.id}: mission_sequence requires mission")
        if not isinstance(case.mission.get("segments"), list) or not case.mission["segments"]:
            raise ValueError(f"case {case.id}: mission must contain non-empty segments")


def _straight_test_case(
    case_id: str,
    *,
    speed_mps: float,
    duration_s: float,
    heading_kp: float,
    heading_kd: float,
    max_omega_radps: float,
    description: str,
) -> MotionTestCase:
    launch_args = {
        "nav_straight_speed_mps": _fmt_float(speed_mps),
        "nav_straight_duration_s": _fmt_float(duration_s),
        "nav_heading_kp": _fmt_float(heading_kp),
        "nav_heading_kd": _fmt_float(heading_kd),
        "nav_max_omega_radps": _fmt_float(max_omega_radps),
        "nav_max_test_duration_s": _fmt_float(duration_s + 1.0),
    }
    return MotionTestCase(
        id=case_id,
        mode="straight_test",
        description=description,
        expected_distance_m=speed_mps * duration_s,
        speed_mps=speed_mps,
        duration_s=duration_s,
        max_case_duration_s=duration_s + 2.0,
        launch_args=launch_args,
    )


def _pivot_test_case(angle_deg: float) -> MotionTestCase:
    side = "left" if angle_deg > 0.0 else "right"
    abs_angle = int(abs(angle_deg))
    case_id = f"pivot_{side}_{abs_angle}"
    return MotionTestCase(
        id=case_id,
        mode="pivot_test",
        description=f"Pivot {side} {abs_angle} deg; CCW is positive per REP 103.",
        expected_angle_deg=float(angle_deg),
        max_case_duration_s=6.5,
        launch_args={
            "nav_pivot_angle_deg": _fmt_float(angle_deg),
            "nav_max_test_duration_s": "6.0",
            "nav_pivot_timeout_s": "4.0",
        },
    )


def _curve_test_case(*, angle_deg: float, radius_m: float, speed_mps: float = 0.15) -> MotionTestCase:
    side = "left" if angle_deg > 0.0 else "right"
    abs_angle = int(abs(angle_deg))
    radius_token = _sanitize_case_token(radius_m, "")
    case_id = f"curve_{side}_R{radius_token}_{abs_angle}"
    arc_length_m = abs(float(radius_m) * math.radians(float(angle_deg)))
    watchdog_s = estimate_curve_watchdog_s(angle_deg=angle_deg, radius_m=radius_m, speed_mps=speed_mps)
    return MotionTestCase(
        id=case_id,
        mode="curve_test",
        description=(
            f"Rolling arc {side} {abs_angle} deg at R={radius_m:.2f} m; "
            "preferred heavy-load turn calibration."
        ),
        expected_distance_m=arc_length_m,
        expected_angle_deg=float(angle_deg),
        speed_mps=float(speed_mps),
        max_case_duration_s=watchdog_s + 1.0,
        launch_args={
            "nav_curve_angle_deg": _fmt_float(abs(angle_deg)),
            "nav_curve_direction": side,
            "nav_curve_radius_m": _fmt_float(radius_m),
            "nav_curve_speed_mps": _fmt_float(speed_mps),
            "nav_curve_timeout_s": _fmt_float(watchdog_s),
            "nav_max_test_duration_s": _fmt_float(watchdog_s),
        },
    )


def _mission_case(case_id: str, *, segments: list[dict[str, Any]], description: str) -> MotionTestCase:
    mission = {"mission_id": case_id, "segments": segments}
    return MotionTestCase(
        id=case_id,
        mode="mission_sequence",
        description=description,
        expected_distance_m=sum(float(s.get("distance_m", 0.0)) for s in segments if s.get("type") == "straight"),
        mission=mission,
        max_case_duration_s=sum(float(s.get("timeout_s", 0.0) or 0.0) for s in segments) or None,
        launch_args={},
    )


def _sanitize_case_token(value: float, suffix: str) -> str:
    text = f"{float(value):.3f}".rstrip("0").rstrip(".").replace("-", "neg").replace(".", "p")
    return f"{text}{suffix}"


def custom_straight_case(
    *,
    speed_mps: float,
    distance_m: Optional[float],
    duration_s: Optional[float],
    heading_kp: float,
    heading_kd: float,
    max_omega_radps: float,
    open_loop: bool = False,
    case_id: Optional[str] = None,
    repeat: int = 1,
) -> MotionTestCase:
    if speed_mps <= 0.0 or not math.isfinite(float(speed_mps)):
        raise ValueError("custom straight requires --straight-speed-mps > 0")
    if distance_m is None and duration_s is None:
        raise ValueError("custom straight requires --straight-distance-m or --straight-duration-s")
    if distance_m is not None and duration_s is not None:
        raise ValueError("custom straight accepts distance or duration, not both")
    if repeat < 1:
        raise ValueError("--custom-repeat must be >= 1")

    kp = 0.0 if open_loop else float(heading_kp)
    kd = 0.0 if open_loop else float(heading_kd)
    max_omega = 0.0 if open_loop else float(max_omega_radps)

    if distance_m is not None:
        distance = float(distance_m)
        if distance <= 0.0 or not math.isfinite(distance):
            raise ValueError("--straight-distance-m must be > 0")
        case_name = case_id or (
            "custom_straight_"
            + _sanitize_case_token(distance, "m")
            + "_"
            + _sanitize_case_token(speed_mps, "mps")
        )
        segment = {
            "type": "straight",
            "distance_m": distance,
            "speed_mps": float(speed_mps),
        }
        launch_args = {
            "nav_heading_kp": _fmt_float(kp),
            "nav_heading_kd": _fmt_float(kd),
            "nav_mission_straight_max_omega_radps": _fmt_float(max_omega),
        }
        return MotionTestCase(
            id=case_name,
            mode="mission_sequence",
            description=(
                f"Custom distance-based straight: {distance:.3f} m at {speed_mps:.3f} m/s "
                f"({'open loop' if open_loop else 'heading hold'})."
            ),
            expected_distance_m=distance,
            speed_mps=float(speed_mps),
            repeat=int(repeat),
            mission={"mission_id": case_name, "segments": [segment]},
            max_case_duration_s=estimate_mission_duration_s({"segments": [segment]}),
            launch_args=launch_args,
        )

    duration = float(duration_s)
    if duration <= 0.0 or not math.isfinite(duration):
        raise ValueError("--straight-duration-s must be > 0")
    case_name = case_id or (
        "custom_straight_"
        + _sanitize_case_token(duration, "s")
        + "_"
        + _sanitize_case_token(speed_mps, "mps")
    )
    case = _straight_test_case(
        case_name,
        speed_mps=float(speed_mps),
        duration_s=duration,
        heading_kp=kp,
        heading_kd=kd,
        max_omega_radps=max_omega,
        description=(
            f"Custom time-based straight: {duration:.3f} s at {speed_mps:.3f} m/s "
            f"({'open loop' if open_loop else 'heading hold'})."
        ),
    )
    return replace(case, repeat=int(repeat))


def custom_pivot_case(
    *,
    angle_deg: float,
    case_id: Optional[str] = None,
    repeat: int = 1,
    max_test_duration_s: float = 6.0,
    pivot_timeout_s: float = 4.0,
) -> MotionTestCase:
    angle = float(angle_deg)
    if not math.isfinite(angle) or abs(angle) < 1e-9:
        raise ValueError("--pivot-angle-deg must be a non-zero finite angle")
    if abs(angle) > 180.0:
        raise ValueError("--pivot-angle-deg supports -180..180; split larger rotations into multiple tests")
    if repeat < 1:
        raise ValueError("--custom-repeat must be >= 1")
    base = _pivot_test_case(angle)
    if case_id is None:
        side = "left" if angle > 0.0 else "right"
        case_id = "custom_pivot_" + side + "_" + _sanitize_case_token(abs(angle), "deg")
    launch_args = dict(base.launch_args)
    launch_args["nav_max_test_duration_s"] = _fmt_float(max_test_duration_s)
    launch_args["nav_pivot_timeout_s"] = _fmt_float(pivot_timeout_s)
    return replace(
        base,
        id=case_id,
        repeat=int(repeat),
        max_case_duration_s=float(max_test_duration_s) + 0.5,
        launch_args=launch_args,
        description=f"Custom pivot: {angle:.3f} deg; CCW is positive per REP 103.",
    )


def custom_curve_case(
    *,
    angle_deg: float,
    radius_m: float,
    speed_mps: float,
    case_id: Optional[str] = None,
    repeat: int = 1,
    max_test_duration_s: Optional[float] = None,
) -> MotionTestCase:
    angle = float(angle_deg)
    radius = float(radius_m)
    speed = float(speed_mps)
    if not math.isfinite(angle) or abs(angle) < 1e-9:
        raise ValueError("--curve-angle-deg must be a non-zero finite angle")
    if abs(angle) > 180.0:
        raise ValueError("--curve-angle-deg supports -180..180; split larger curves into multiple tests")
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("--curve-radius-m must be > 0")
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("--curve-speed-mps must be > 0")
    if max_test_duration_s is not None and (
        not math.isfinite(float(max_test_duration_s)) or float(max_test_duration_s) <= 0.0
    ):
        raise ValueError("--custom-curve-max-test-duration-s must be > 0 when provided")
    if repeat < 1:
        raise ValueError("--custom-repeat must be >= 1")
    base = _curve_test_case(angle_deg=angle, radius_m=radius, speed_mps=speed)
    if case_id is None:
        side = "left" if angle > 0.0 else "right"
        case_id = "custom_curve_" + side + "_R" + _sanitize_case_token(radius, "") + "_" + _sanitize_case_token(abs(angle), "deg")
    launch_args = dict(base.launch_args)
    watchdog_s = (
        float(max_test_duration_s)
        if max_test_duration_s is not None
        else estimate_curve_watchdog_s(angle_deg=angle, radius_m=radius, speed_mps=speed)
    )
    launch_args["nav_curve_timeout_s"] = _fmt_float(watchdog_s)
    launch_args["nav_max_test_duration_s"] = _fmt_float(watchdog_s)
    return replace(
        base,
        id=case_id,
        repeat=int(repeat),
        max_case_duration_s=watchdog_s + 1.0,
        launch_args=launch_args,
        description=f"Custom rolling arc: {angle:.3f} deg at R={radius:.3f} m and {speed:.3f} m/s.",
    )


def builtin_suite(name: str) -> MotionTestSuite:
    suite_id = str(name).strip().lower()
    if suite_id == "basic":
        cases: list[MotionTestCase] = [
            MotionTestCase(
                id="idle_health_10s",
                mode="idle",
                description="Idle health check: STOP only, fresh sensors, no motor fault.",
                duration_s=10.0,
                max_case_duration_s=10.0,
            ),
            _straight_test_case(
                "straight_open_015_2s",
                speed_mps=0.15,
                duration_s=2.0,
                heading_kp=0.0,
                heading_kd=0.0,
                max_omega_radps=0.0,
                description="Open-loop straight baseline at 0.15 m/s for 2 s.",
            ),
            _straight_test_case(
                "straight_hold_015_2s",
                speed_mps=0.15,
                duration_s=2.0,
                heading_kp=0.3,
                heading_kd=0.05,
                max_omega_radps=0.15,
                description="Heading-hold straight at 0.15 m/s for 2 s.",
            ),
            _straight_test_case(
                "straight_hold_020_2s",
                speed_mps=0.20,
                duration_s=2.0,
                heading_kp=0.3,
                heading_kd=0.05,
                max_omega_radps=0.15,
                description="Heading-hold straight at 0.20 m/s for 2 s.",
            ),
        ]
        for angle in (15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 90.0, -90.0):
            cases.append(_pivot_test_case(angle))
        return MotionTestSuite(suite_id="basic", cases=tuple(cases))

    if suite_id == "pivot_calibration":
        cases = [_pivot_test_case(angle) for angle in (15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 90.0, -90.0)]
        cases.append(replace(_pivot_test_case(45.0), id="pivot_left_45_repeat_3", repeat=3))
        cases.append(replace(_pivot_test_case(-45.0), id="pivot_right_45_repeat_3", repeat=3))
        return MotionTestSuite(suite_id="pivot_calibration", cases=tuple(cases))

    if suite_id == "curve_calibration":
        cases = [
            _curve_test_case(angle_deg=45.0, radius_m=1.0),
            _curve_test_case(angle_deg=-45.0, radius_m=1.0),
            _curve_test_case(angle_deg=90.0, radius_m=1.0),
            _curve_test_case(angle_deg=-90.0, radius_m=1.0),
            _curve_test_case(angle_deg=90.0, radius_m=0.75),
            _curve_test_case(angle_deg=-90.0, radius_m=0.75),
        ]
        return MotionTestSuite(suite_id="curve_calibration", cases=tuple(cases))

    if suite_id == "turn_debug":
        cases = [
            _pivot_test_case(-30.0),
            _pivot_test_case(30.0),
            _pivot_test_case(-45.0),
            _curve_test_case(angle_deg=-45.0, radius_m=0.75, speed_mps=0.14),
            _curve_test_case(angle_deg=-45.0, radius_m=0.60, speed_mps=0.14),
            _curve_test_case(angle_deg=-45.0, radius_m=0.45, speed_mps=0.12),
            _curve_test_case(angle_deg=45.0, radius_m=0.75, speed_mps=0.14),
        ]
        cases = [
            replace(
                case,
                description=(
                    case.description
                    + " Mark robot center before/after; enter center chord as actual distance."
                    if case.mode == "curve_test"
                    else case.description
                ),
            )
            for case in cases
        ]
        return MotionTestSuite(suite_id="turn_debug", cases=tuple(cases))

    if suite_id == "mission_smoke":
        cases = [
            _mission_case(
                "mission_straight_05_pivot_45_straight_05",
                segments=[
                    {"type": "straight", "distance_m": 0.5, "speed_mps": 0.15},
                    {"type": "pivot", "angle_deg": 45.0},
                    {"type": "straight", "distance_m": 0.5, "speed_mps": 0.15},
                ],
                description="Mission smoke: straight 0.5 m, pivot left 45 deg, straight 0.5 m.",
            ),
            _mission_case(
                "mission_straight_05_pivot_-45_straight_05",
                segments=[
                    {"type": "straight", "distance_m": 0.5, "speed_mps": 0.15},
                    {"type": "pivot", "angle_deg": -45.0},
                    {"type": "straight", "distance_m": 0.5, "speed_mps": 0.15},
                ],
                description="Mission smoke: straight 0.5 m, pivot right 45 deg, straight 0.5 m.",
            ),
            _mission_case(
                "mission_straight_05_pivot_90_straight_05",
                segments=[
                    {"type": "straight", "distance_m": 0.5, "speed_mps": 0.15},
                    {"type": "pivot", "angle_deg": 90.0},
                    {"type": "straight", "distance_m": 0.5, "speed_mps": 0.15},
                ],
                description="Mission smoke: straight 0.5 m, pivot left 90 deg, straight 0.5 m.",
            ),
        ]
        cases.append(
            _mission_case(
                "straight_hold_015_1m_repeat_3",
                segments=[{"type": "straight", "distance_m": 1.0, "speed_mps": 0.15}],
                description="Distance-based straight 1 m repeatability mission.",
            )
        )
        cases[-1] = replace(cases[-1], repeat=3)
        return MotionTestSuite(suite_id="mission_smoke", cases=tuple(cases))

    raise ValueError(f"unknown built-in suite {name!r}")


def load_suite_file(path: Path) -> MotionTestSuite:
    data = json.loads(path.expanduser().read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("suite file root must be an object")
    suite_id = str(data.get("suite_id") or path.stem).strip() or path.stem
    raw_cases = data.get("cases")
    if not isinstance(raw_cases, list) or not raw_cases:
        raise ValueError("suite file must contain a non-empty cases list")
    cases: list[MotionTestCase] = []
    for index, item in enumerate(raw_cases):
        if not isinstance(item, dict):
            raise ValueError(f"case {index} must be an object")
        case_id = str(item.get("id") or "").strip()
        mode = str(item.get("mode") or "").strip()
        if not case_id or not mode:
            raise ValueError(f"case {index} requires id and mode")
        launch_args = {str(k): str(v) for k, v in dict(item.get("launch_args") or {}).items()}
        if mode == "curve_test":
            angle = _maybe_float(item.get("expected_angle_deg", item.get("angle_deg")))
            radius = _maybe_float(item.get("radius_m"))
            speed = _maybe_float(item.get("speed_mps"))
            if angle is not None:
                launch_args.setdefault("nav_curve_angle_deg", _fmt_float(abs(angle)))
                launch_args.setdefault("nav_curve_direction", "left" if angle >= 0.0 else "right")
            if radius is not None:
                launch_args.setdefault("nav_curve_radius_m", _fmt_float(radius))
            if speed is not None:
                launch_args.setdefault("nav_curve_speed_mps", _fmt_float(speed))
        case = MotionTestCase(
            id=case_id,
            mode=mode,
            description=str(item.get("description") or ""),
            expected_distance_m=_maybe_float(item.get("expected_distance_m")),
            expected_angle_deg=_maybe_float(item.get("expected_angle_deg", item.get("angle_deg"))),
            speed_mps=_maybe_float(item.get("speed_mps")),
            duration_s=_maybe_float(item.get("duration_s")),
            max_case_duration_s=_maybe_float(item.get("max_case_duration_s")),
            repeat=int(item.get("repeat", 1)),
            launch_args=launch_args,
            mission=item.get("mission") if isinstance(item.get("mission"), dict) else None,
        )
        if case.mode == "mission_sequence" and case.mission is None and isinstance(item.get("segments"), list):
            case = replace(case, mission={"mission_id": case.id, "segments": item["segments"]})
        validate_case(case)
        cases.append(case)
    return MotionTestSuite(suite_id=suite_id, cases=tuple(cases))


def _maybe_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"expected finite number, got {value!r}")
    return number


def select_cases(suite: MotionTestSuite, case_ids: Sequence[str]) -> tuple[MotionTestCase, ...]:
    if not case_ids:
        return suite.cases
    requested = set(case_ids)
    selected = tuple(case for case in suite.cases if case.id in requested)
    missing = sorted(requested - {case.id for case in selected})
    if missing:
        raise ValueError(f"case id(s) not found in suite {suite.suite_id}: {', '.join(missing)}")
    return selected


def expand_repeats(cases: Iterable[MotionTestCase]) -> list[MotionTestCase]:
    expanded: list[MotionTestCase] = []
    for case in cases:
        validate_case(case)
        if case.repeat == 1:
            expanded.append(case)
            continue
        for index in range(1, case.repeat + 1):
            expanded.append(replace(case, id=f"{case.id}_run{index}", repeat=1))
    return expanded


def expected_distance_for_case(case: MotionTestCase) -> Optional[float]:
    if case.expected_distance_m is not None:
        return float(case.expected_distance_m)
    if case.mode == "straight_test" and case.speed_mps is not None and case.duration_s is not None:
        return float(case.speed_mps) * float(case.duration_s)
    if case.mode == "mission_sequence" and case.mission:
        return sum(
            float(segment.get("distance_m", 0.0))
            for segment in case.mission.get("segments", [])
            if isinstance(segment, dict) and str(segment.get("type")) == "straight"
        )
    return None


def case_launch_args(case: MotionTestCase, *, case_dir: Optional[Path] = None) -> dict[str, str]:
    args = {"nav_controller_mode": case.mode}
    args.update({str(k): str(v) for k, v in case.launch_args.items()})
    if case.mode == "idle":
        args.setdefault("nav_max_test_duration_s", _fmt_float(case.max_case_duration_s or case.duration_s or 10.0))
    elif case.mode == "straight_test":
        args.setdefault("nav_straight_speed_mps", _fmt_float(case.speed_mps or 0.15))
        args.setdefault("nav_straight_duration_s", _fmt_float(case.duration_s or 2.0))
        args.setdefault("nav_max_test_duration_s", _fmt_float((case.duration_s or 2.0) + 1.0))
    elif case.mode == "pivot_test":
        args.setdefault("nav_pivot_angle_deg", _fmt_float(case.expected_angle_deg or 90.0))
        args.setdefault("nav_max_test_duration_s", "6.0")
        args.setdefault("nav_pivot_timeout_s", "4.0")
    elif case.mode == "curve_test":
        angle = float(case.expected_angle_deg or 90.0)
        args.setdefault("nav_curve_angle_deg", _fmt_float(abs(angle)))
        args.setdefault("nav_curve_direction", "left" if angle >= 0.0 else "right")
        args.setdefault("nav_curve_radius_m", "1.0")
        args.setdefault("nav_curve_speed_mps", _fmt_float(case.speed_mps or 0.15))
        radius = float(args.get("nav_curve_radius_m", 1.0))
        speed = float(args.get("nav_curve_speed_mps", case.speed_mps or 0.15))
        watchdog_s = estimate_curve_watchdog_s(angle_deg=angle, radius_m=radius, speed_mps=speed)
        args.setdefault("nav_curve_timeout_s", _fmt_float(watchdog_s))
        args.setdefault("nav_max_test_duration_s", _fmt_float(watchdog_s))
    elif case.mode == "mission_sequence":
        # The runner is a calibration/debug tool.  It may intentionally test
        # stop/pivot/wait primitives that are invalid during formal continuous
        # competition travel, so keep that rule disabled unless overridden.
        args.setdefault("nav_competition_continuous_motion_enabled", "false")
        if case_dir is not None:
            mission_path = write_case_mission_file(case, case_dir)
            args["nav_mission_file"] = str(mission_path)
    return args


def estimated_curve_case_duration_s(case: MotionTestCase) -> float:
    if case.mode != "curve_test":
        return 8.0
    args = case_launch_args(case)
    angle = float(case.expected_angle_deg or args.get("nav_curve_angle_deg", 90.0))
    radius = float(args.get("nav_curve_radius_m", 1.0))
    speed = float(args.get("nav_curve_speed_mps", case.speed_mps or 0.15))
    raw_timeout = float(args.get("nav_curve_timeout_s", 0.0) or 0.0)
    watchdog_s = (
        raw_timeout
        if raw_timeout > 0.0
        else estimate_curve_watchdog_s(angle_deg=angle, radius_m=radius, speed_mps=speed)
    )
    return watchdog_s + 1.0


def write_case_mission_file(case: MotionTestCase, case_dir: Path) -> Path:
    if case.mode != "mission_sequence" or not case.mission:
        raise ValueError(f"case {case.id} is not a mission case")
    case_dir.mkdir(parents=True, exist_ok=True)
    mission_path = case_dir / "mission.json"
    mission_path.write_text(json.dumps(case.mission, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return mission_path


def merge_launch_args(base: Mapping[str, str], overrides: Mapping[str, str]) -> dict[str, str]:
    merged = dict(base)
    merged.update({str(k): str(v) for k, v in overrides.items()})
    return merged


def build_launch_command(launch_args: Mapping[str, str]) -> list[str]:
    command = ["ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE]
    for key in sorted(launch_args):
        command.append(f"{key}:={launch_args[key]}")
    return command


def build_bag_command(bag_dir: Path, topics: Sequence[str] = BAG_TOPICS) -> list[str]:
    return ["ros2", "bag", "record", "-o", str(bag_dir), *topics]


def case_run_duration_s(case: MotionTestCase, *, startup_wait_s: float, post_stop_wait_s: float) -> float:
    active_s = case.max_case_duration_s
    if active_s is None:
        if case.mode == "idle":
            active_s = case.duration_s or 10.0
        elif case.mode == "straight_test":
            active_s = (case.duration_s or 2.0) + 2.0
        elif case.mode == "pivot_test":
            active_s = 6.5
        elif case.mode == "curve_test":
            active_s = estimated_curve_case_duration_s(case)
        elif case.mode == "mission_sequence" and case.mission:
            active_s = estimate_mission_duration_s(case.mission)
        else:
            active_s = 5.0
    return max(0.0, float(startup_wait_s)) + max(0.1, float(active_s)) + max(0.0, float(post_stop_wait_s))


def estimate_mission_duration_s(mission: Mapping[str, Any]) -> float:
    total = 0.0
    for segment in mission.get("segments", []):
        if not isinstance(segment, dict):
            continue
        segment_type = str(segment.get("type", ""))
        if segment_type == "straight":
            speed = max(DEFAULT_COMPETITION_MIN_SPEED_MPS, abs(float(segment.get("speed_mps", 0.15) or 0.15)))
            total += float(segment.get("distance_m", 0.0) or 0.0) / speed + 2.0
        elif segment_type == "pivot":
            total += float(segment.get("timeout_s", 4.0) or 4.0) + 2.0
        elif segment_type == "wait":
            total += float(segment.get("wait_s", 0.0) or 0.0) + 0.5
    return max(5.0, total + 2.0)


def parse_ros_string_payload(text: str) -> dict[str, Any]:
    if not text:
        return {}
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {"raw": text}
    return data if isinstance(data, dict) else {"value": data}


def _safe_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _last_float(records: Sequence[dict[str, Any]], key: str) -> Optional[float]:
    for record in reversed(records):
        value = _safe_float(record.get(key))
        if value is not None:
            return value
    return None


def _max_abs(records: Sequence[dict[str, Any]], key: str) -> float:
    values = [_safe_float(record.get(key)) for record in records]
    finite = [abs(v) for v in values if v is not None]
    return max(finite) if finite else 0.0


def _count_stop_reasons(cmd_records: Sequence[dict[str, Any]], status_records: Sequence[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for record in cmd_records:
        if str(record.get("command_type", "")).lower() == "stop":
            reason = str(record.get("reason") or "unknown")
            counts[reason] = counts.get(reason, 0) + 1
    if not counts:
        for record in status_records:
            reason = record.get("safety_reason")
            if reason:
                text = str(reason)
                counts[text] = counts.get(text, 0) + 1
    return counts


def compute_case_metrics(
    case: MotionTestCase,
    status_records: Sequence[dict[str, Any]],
    cmd_records: Sequence[dict[str, Any]],
    manual: Mapping[str, Any],
) -> dict[str, Any]:
    expected_distance_m = expected_distance_for_case(case)
    expected_angle_deg = case.expected_angle_deg
    actual_distance_m = _safe_float(manual.get("actual_distance_m"))
    actual_angle_deg = _safe_float(manual.get("actual_angle_deg"))
    lateral_drift_m = _safe_float(manual.get("lateral_drift_m"))
    actual_turn_radius_m = None
    if case.mode == "curve_test" and actual_distance_m is not None:
        angle_for_radius_deg = actual_angle_deg if actual_angle_deg is not None else expected_angle_deg
        if angle_for_radius_deg is not None:
            half_angle_rad = 0.5 * abs(math.radians(float(angle_for_radius_deg)))
            denominator = 2.0 * math.sin(half_angle_rad)
            if denominator > 1e-9:
                actual_turn_radius_m = actual_distance_m / denominator
    encoder_distance_m = _last_float(status_records, "segment_distance_m")
    heading_rms = _last_float(status_records, "straight_heading_error_rms")
    if heading_rms is None:
        heading_values = [_safe_float(record.get("heading_error_rad")) for record in status_records]
        finite_heading = [value for value in heading_values if value is not None]
        heading_rms = math.sqrt(sum(value * value for value in finite_heading) / len(finite_heading)) if finite_heading else None
    final_heading_error_rad = _last_float(status_records, "heading_error_rad")
    pivot_final_error_rad = _last_float(status_records, "pivot_final_error_rad")
    pivot_overshoot_rad = _last_float(status_records, "pivot_overshoot_rad")
    pivot_retry_count = _last_float(status_records, "pivot_retry_count")
    curve_heading_error_rad = _last_float(status_records, "curve_heading_error_rad")
    curve_radius_m = _last_float(status_records, "curve_radius_m")
    curve_arc_length_m = _last_float(status_records, "curve_arc_length_m")
    omega_limit = _last_float(status_records, "max_omega_radps")
    omega_max_abs = max(_max_abs(status_records, "omega_cmd_radps"), _max_abs(cmd_records, "omega_radps"))
    omega_saturated_samples = sum(1 for record in status_records if bool(record.get("omega_saturated", False)))
    imu_rates = [_safe_float(record.get("imu_rate_hz")) for record in status_records]
    finite_imu_rates = [value for value in imu_rates if value is not None and value > 0.0]
    imu_skipped_final = _last_float(status_records, "imu_skipped_integrations")
    estimated_angle_deg = None
    if expected_angle_deg is not None and pivot_final_error_rad is not None:
        estimated_angle_deg = float(expected_angle_deg) - math.degrees(float(pivot_final_error_rad))
    elif expected_angle_deg is not None and curve_heading_error_rad is not None:
        estimated_angle_deg = float(expected_angle_deg) - math.degrees(float(curve_heading_error_rad))
    metrics: dict[str, Any] = {
        "expected_distance_m": expected_distance_m,
        "encoder_distance_m": encoder_distance_m,
        "actual_distance_m": actual_distance_m,
        "distance_error_m": None if actual_distance_m is None or expected_distance_m is None else actual_distance_m - expected_distance_m,
        "encoder_distance_error_m": None
        if encoder_distance_m is None or expected_distance_m is None
        else encoder_distance_m - expected_distance_m,
        "lateral_drift_m": lateral_drift_m,
        "heading_rms_rad": heading_rms,
        "heading_rms_deg": None if heading_rms is None else math.degrees(float(heading_rms)),
        "final_heading_error_rad": final_heading_error_rad,
        "final_heading_error_deg": None
        if final_heading_error_rad is None
        else math.degrees(float(final_heading_error_rad)),
        "target_angle_deg": expected_angle_deg,
        "estimated_angle_deg": estimated_angle_deg,
        "actual_angle_deg": actual_angle_deg,
        "actual_angle_error_deg": None
        if actual_angle_deg is None or expected_angle_deg is None
        else actual_angle_deg - expected_angle_deg,
        "gyro_manual_disagreement_deg": None
        if actual_angle_deg is None or estimated_angle_deg is None
        else actual_angle_deg - estimated_angle_deg,
        "pivot_final_error_rad": pivot_final_error_rad,
        "pivot_final_error_deg": None
        if pivot_final_error_rad is None
        else math.degrees(float(pivot_final_error_rad)),
        "pivot_overshoot_rad": pivot_overshoot_rad,
        "pivot_overshoot_deg": None
        if pivot_overshoot_rad is None
        else math.degrees(float(pivot_overshoot_rad)),
        "pivot_retry_count": pivot_retry_count,
        "curve_heading_error_rad": curve_heading_error_rad,
        "curve_heading_error_deg": None
        if curve_heading_error_rad is None
        else math.degrees(float(curve_heading_error_rad)),
        "curve_radius_m": curve_radius_m,
        "curve_arc_length_m": curve_arc_length_m,
        "actual_turn_radius_m": actual_turn_radius_m,
        "actual_turn_radius_error_m": None
        if actual_turn_radius_m is None or curve_radius_m is None
        else actual_turn_radius_m - curve_radius_m,
        "omega_max_abs_radps": omega_max_abs,
        "omega_saturation_percent": 0.0
        if not status_records
        else 100.0 * float(omega_saturated_samples) / float(len(status_records)),
        "omega_limit_radps": omega_limit,
        "imu_rate_avg_hz": None if not finite_imu_rates else sum(finite_imu_rates) / len(finite_imu_rates),
        "imu_rate_min_hz": None if not finite_imu_rates else min(finite_imu_rates),
        "imu_rate_max_hz": None if not finite_imu_rates else max(finite_imu_rates),
        "imu_skipped_integrations_final": imu_skipped_final,
        "nav_frame_age_s": _last_float(status_records, "nav_frame_age_s"),
        "motor_status_age_s": _last_float(status_records, "motor_status_age_s"),
        "stuck_recovery_count": _last_float(status_records, "stuck_recovery_count"),
        "status_sample_count": len(status_records),
        "cmd_sample_count": len(cmd_records),
        "stop_reasons": _count_stop_reasons(cmd_records, status_records),
        "motion_rule_violation_count": sum(1 for record in status_records if not bool(record.get("motion_rule_ok", True))),
        "critical_status_count": sum(1 for record in status_records if str(record.get("safety_level", "")) == "critical"),
    }
    return metrics


def compensation_table_draft(case_results: Sequence[dict[str, Any]]) -> dict[str, dict[str, dict[str, float]]]:
    grouped: dict[tuple[str, str], list[float]] = {}
    for result in case_results:
        metrics = result.get("metrics") if isinstance(result.get("metrics"), dict) else {}
        target = _safe_float(metrics.get("target_angle_deg"))
        error = _safe_float(metrics.get("actual_angle_error_deg"))
        actual = _safe_float(metrics.get("actual_angle_deg"))
        if target is None or error is None or actual is None or abs(target) < 1e-9:
            continue
        side = "left" if target > 0.0 else "right"
        key = (side, str(int(round(abs(target)))))
        grouped.setdefault(key, []).append(error)
    table: dict[str, dict[str, dict[str, float]]] = {"left": {}, "right": {}}
    for (side, angle_text), errors in sorted(grouped.items()):
        mean_error = sum(errors) / len(errors)
        target = float(angle_text)
        table[side][angle_text] = {
            "bias_deg": -mean_error,
            "scale": 1.0 if abs(target + mean_error) < 1e-9 else target / (target + mean_error),
            "sample_count": float(len(errors)),
        }
    return table


def write_jsonl(path: Path, samples: Sequence[TopicSample]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for sample in samples:
            handle.write(json.dumps({"received_time_s": sample.received_time_s, **sample.data}, sort_keys=True) + "\n")


def write_summary_files(suite_dir: Path, suite_id: str, case_results: Sequence[dict[str, Any]]) -> None:
    suite_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "suite_id": suite_id,
        "generated_time_s": time.time(),
        "case_count": len(case_results),
        "cases": list(case_results),
        "pivot_compensation_draft": compensation_table_draft(case_results),
    }
    (suite_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    fieldnames = [
        "case_id",
        "mode",
        "pass_fail",
        "expected_distance_m",
        "actual_distance_m",
        "distance_error_m",
        "encoder_distance_m",
        "heading_rms_deg",
        "target_angle_deg",
        "actual_angle_deg",
        "actual_angle_error_deg",
        "estimated_angle_deg",
        "pivot_final_error_deg",
        "pivot_overshoot_deg",
        "curve_radius_m",
        "actual_turn_radius_m",
        "actual_turn_radius_error_m",
        "curve_arc_length_m",
        "curve_heading_error_deg",
        "omega_saturation_percent",
        "motion_rule_violation_count",
        "critical_status_count",
    ]
    with (suite_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for result in case_results:
            metrics = result.get("metrics", {})
            manual = result.get("manual", {})
            row = {
                "case_id": result.get("case_id"),
                "mode": result.get("mode"),
                "pass_fail": manual.get("pass_fail"),
            }
            for key in fieldnames:
                if key not in row:
                    row[key] = metrics.get(key)
            writer.writerow(row)

    lines = [f"# UGV Motion Test Summary: {suite_id}", "", f"Cases: {len(case_results)}", ""]
    lines.append("| Case | Mode | Manual | Distance Error | Angle Error | Radius Error | Heading RMS | Stops |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for result in case_results:
        metrics = result.get("metrics", {})
        manual = result.get("manual", {})
        stops = metrics.get("stop_reasons") or {}
        stop_text = ", ".join(f"{k}={v}" for k, v in sorted(stops.items())) if isinstance(stops, dict) else ""
        lines.append(
            "| {case} | {mode} | {pf} | {dist} | {angle} | {radius} | {rms} | {stops} |".format(
                case=result.get("case_id"),
                mode=result.get("mode"),
                pf=manual.get("pass_fail") or "",
                dist=_md_value(metrics.get("distance_error_m"), "m"),
                angle=_md_value(metrics.get("actual_angle_error_deg"), "deg"),
                radius=_md_value(metrics.get("actual_turn_radius_error_m"), "m"),
                rms=_md_value(metrics.get("heading_rms_deg"), "deg"),
                stops=stop_text,
            )
        )
    compensation = summary["pivot_compensation_draft"]
    if compensation:
        lines.extend(["", "## Pivot Compensation Draft", "", "```json", json.dumps(compensation, indent=2, sort_keys=True), "```"])
    (suite_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _md_value(value: Any, unit: str) -> str:
    number = _safe_float(value)
    if number is None:
        return ""
    return f"{number:.4g} {unit}"


class RosTopicCollector:
    def __init__(self, topics: Sequence[str] = (STATUS_TOPIC, CMD_TOPIC)) -> None:
        self.samples: list[TopicSample] = []
        self.error: Optional[str] = None
        self._topics = tuple(topics)

    def collect_for(self, duration_s: float) -> list[TopicSample]:
        try:
            import rclpy  # type: ignore
            from std_msgs.msg import String  # type: ignore
        except Exception as exc:  # pragma: no cover - only used on robot when ROS is installed
            self.error = f"ROS topic collection unavailable: {exc}"
            time.sleep(max(0.0, duration_s))
            return []

        rclpy.init(args=None)
        node = rclpy.create_node("ugv_motion_test_runner_collector")

        def make_callback(topic: str):
            def callback(msg) -> None:
                self.samples.append(
                    TopicSample(
                        received_time_s=time.time(),
                        topic=topic,
                        data=parse_ros_string_payload(str(msg.data)),
                    )
                )

            return callback

        try:
            for topic in self._topics:
                node.create_subscription(String, topic, make_callback(topic), 10)
            deadline_s = time.monotonic() + max(0.0, duration_s)
            while time.monotonic() < deadline_s:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:  # pragma: no cover - ROS runtime cleanup
            try:
                node.destroy_node()
            finally:
                rclpy.shutdown()
        return list(self.samples)


def terminate_process(proc: Optional[subprocess.Popen], *, timeout_s: float = 5.0) -> Optional[int]:
    if proc is None:
        return None
    if proc.poll() is not None:
        return proc.returncode
    try:
        proc.send_signal(signal.SIGINT)
        return proc.wait(timeout=timeout_s)
    except Exception:
        try:
            proc.terminate()
            return proc.wait(timeout=timeout_s)
        except Exception:
            proc.kill()
            return proc.wait(timeout=timeout_s)


def prompt_before_case(case: MotionTestCase) -> None:
    print()
    print(f"Next case: {case.id}")
    if case.description:
        print(case.description)
    print("Checklist: vehicle placed at start mark, wheels/path clear, emergency stop reachable.")
    input("Press Enter to launch this case, or Ctrl-C to stop the suite. ")


def prompt_manual_measurements(case: MotionTestCase, *, yes: bool) -> dict[str, Any]:
    if yes:
        return {
            "actual_distance_m": None,
            "actual_angle_deg": None,
            "lateral_drift_m": None,
            "pass_fail": "unmeasured",
            "notes": "",
        }
    print()
    print(f"Manual measurements for {case.id}. Blank or 'skip' leaves a field empty.")
    if case.mode == "curve_test":
        distance_prompt = "Actual center chord in meters, start center to stop center: "
    else:
        distance_prompt = "Actual travel distance in meters: "
    actual_distance = parse_optional_float(input(distance_prompt))
    actual_angle = parse_optional_float(input("Actual final turn angle in degrees, CCW positive: "))
    lateral_drift = parse_optional_float(input("Lateral drift in meters: "))
    pass_fail = input("Pass/fail label [pass/fail/needs_review]: ").strip() or "needs_review"
    notes = input("Notes: ").strip()
    return {
        "actual_distance_m": actual_distance,
        "actual_angle_deg": actual_angle,
        "lateral_drift_m": lateral_drift,
        "pass_fail": pass_fail,
        "notes": notes,
    }


def run_case(
    case: MotionTestCase,
    *,
    suite_dir: Path,
    global_launch_args: Mapping[str, str],
    startup_wait_s: float,
    post_stop_wait_s: float,
    record_bag: bool,
    yes: bool,
    no_launch: bool = False,
) -> dict[str, Any]:
    case_dir = suite_dir / case.id
    case_dir.mkdir(parents=True, exist_ok=True)
    launch_args = merge_launch_args(case_launch_args(case, case_dir=case_dir), global_launch_args)
    launch_args.setdefault("nav_telemetry_dir", str(case_dir / "mission_telemetry"))
    launch_command = build_launch_command(launch_args)
    bag_command: Optional[list[str]] = None
    launch_returncode: Optional[int] = None
    bag_returncode: Optional[int] = None

    (case_dir / "case_spec.json").write_text(json.dumps(asdict(case), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (case_dir / "launch_command.json").write_text(json.dumps(launch_command, indent=2) + "\n", encoding="utf-8")

    if not yes:
        prompt_before_case(case)

    run_duration = case_run_duration_s(case, startup_wait_s=startup_wait_s, post_stop_wait_s=post_stop_wait_s)
    launch_proc: Optional[subprocess.Popen] = None
    bag_proc: Optional[subprocess.Popen] = None
    collector = RosTopicCollector()
    collector_samples: list[TopicSample] = []
    collector_thread: Optional[threading.Thread] = None

    try:
        if no_launch:
            time.sleep(0.01)
        else:
            launch_proc = subprocess.Popen(launch_command)
            if record_bag:
                bag_dir = case_dir / "rosbag"
                bag_command = build_bag_command(bag_dir)
                (case_dir / "bag_command.json").write_text(json.dumps(bag_command, indent=2) + "\n", encoding="utf-8")
                bag_proc = subprocess.Popen(bag_command)

            def collect() -> None:
                nonlocal collector_samples
                collector_samples = collector.collect_for(run_duration)

            collector_thread = threading.Thread(target=collect, daemon=True)
            collector_thread.start()
            collector_thread.join(timeout=run_duration + 5.0)
    finally:
        bag_returncode = terminate_process(bag_proc)
        launch_returncode = terminate_process(launch_proc)

    samples = collector_samples or collector.samples
    status_samples = [sample for sample in samples if sample.topic == STATUS_TOPIC]
    cmd_samples = [sample for sample in samples if sample.topic == CMD_TOPIC]
    write_jsonl(case_dir / "status_samples.jsonl", status_samples)
    write_jsonl(case_dir / "cmd_samples.jsonl", cmd_samples)
    manual = prompt_manual_measurements(case, yes=yes)
    metrics = compute_case_metrics(
        case,
        [sample.data for sample in status_samples],
        [sample.data for sample in cmd_samples],
        manual,
    )
    mission_telemetry_files = sorted(str(path) for path in (case_dir / "mission_telemetry").glob("*.jsonl"))
    result = {
        "case_id": case.id,
        "mode": case.mode,
        "description": case.description,
        "manual": manual,
        "metrics": metrics,
        "launch_args": launch_args,
        "launch_command": launch_command,
        "bag_command": bag_command,
        "launch_returncode": launch_returncode,
        "bag_returncode": bag_returncode,
        "collector_error": collector.error,
        "mission_telemetry_files": mission_telemetry_files,
        "case_dir": str(case_dir),
    }
    (case_dir / "case_result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument(
        "--suite",
        default="basic",
        help="Built-in suite: basic, turn_debug, pivot_calibration, curve_calibration, mission_smoke",
    )
    parser.add_argument("--suite-file", help="Path to a custom JSON suite")
    parser.add_argument("--case", action="append", default=[], help="Run only this case id; can be repeated")
    parser.add_argument("--straight-speed-mps", type=float, help="Create a one-off custom straight test at this speed")
    parser.add_argument("--straight-distance-m", type=float, help="Create a distance-based straight test using mission_sequence")
    parser.add_argument("--straight-duration-s", type=float, help="Create a time-based straight_test")
    parser.add_argument("--straight-heading-kp", type=float, default=0.3)
    parser.add_argument("--straight-heading-kd", type=float, default=0.05)
    parser.add_argument("--straight-max-omega-radps", type=float, default=0.15)
    parser.add_argument("--straight-open-loop", action="store_true", help="Disable heading correction for a custom straight test")
    parser.add_argument("--pivot-angle-deg", type=float, help="Create a one-off custom pivot_test; CCW/left is positive")
    parser.add_argument("--curve-angle-deg", type=float, help="Create a one-off custom curve_test; CCW/left is positive")
    parser.add_argument("--curve-radius-m", type=float, default=1.0)
    parser.add_argument("--curve-speed-mps", type=float, default=0.15)
    parser.add_argument("--custom-case-id", help="Optional id for a custom straight/pivot/curve case")
    parser.add_argument("--custom-repeat", type=int, default=1, help="Repeat count for a custom straight/pivot/curve case")
    parser.add_argument("--custom-pivot-max-test-duration-s", type=float, default=6.0)
    parser.add_argument("--custom-pivot-timeout-s", type=float, default=4.0)
    parser.add_argument(
        "--custom-curve-max-test-duration-s",
        type=float,
        default=None,
        help="Optional curve watchdog override; default is computed from angle/radius/speed.",
    )
    parser.add_argument("--yes", action="store_true", help="Run without interactive pre-case prompts or manual measurements")
    parser.add_argument("--record-bag", action="store_true", help="Also run ros2 bag record for key topics")
    parser.add_argument("--output-dir", default="~/.ros/ugv_motion_tests")
    parser.add_argument("--launch-arg", action="append", default=[], help="Extra launch argument, e.g. motor_port:=/dev/ttyACM0")
    parser.add_argument("--startup-wait-s", type=float, default=6.0)
    parser.add_argument("--post-stop-wait-s", type=float, default=1.0)
    parser.add_argument("--no-launch", action="store_true", help="Generate reports without launching ROS; for script dry checks")
    return parser.parse_args(argv)


def _has_custom_straight_args(args: argparse.Namespace) -> bool:
    return (
        args.straight_speed_mps is not None
        or args.straight_distance_m is not None
        or args.straight_duration_s is not None
        or bool(args.straight_open_loop)
    )


def load_requested_suite(args: argparse.Namespace) -> MotionTestSuite:
    has_custom_straight = _has_custom_straight_args(args)
    has_custom_pivot = args.pivot_angle_deg is not None
    has_custom_curve = args.curve_angle_deg is not None
    if has_custom_straight or has_custom_pivot or has_custom_curve:
        custom_count = sum(1 for enabled in (has_custom_straight, has_custom_pivot, has_custom_curve) if enabled)
        if custom_count > 1:
            raise ValueError("custom straight, pivot, and curve options cannot be combined")
        if args.suite_file:
            raise ValueError("--suite-file cannot be combined with custom straight/pivot/curve options")
        if args.case:
            raise ValueError("--case cannot be combined with custom straight/pivot/curve options")
        if has_custom_straight:
            case = custom_straight_case(
                speed_mps=float(args.straight_speed_mps) if args.straight_speed_mps is not None else 0.0,
                distance_m=args.straight_distance_m,
                duration_s=args.straight_duration_s,
                heading_kp=float(args.straight_heading_kp),
                heading_kd=float(args.straight_heading_kd),
                max_omega_radps=float(args.straight_max_omega_radps),
                open_loop=bool(args.straight_open_loop),
                case_id=args.custom_case_id,
                repeat=int(args.custom_repeat),
            )
            return MotionTestSuite(suite_id="custom_straight", cases=(case,))
        if has_custom_pivot:
            case = custom_pivot_case(
                angle_deg=float(args.pivot_angle_deg),
                case_id=args.custom_case_id,
                repeat=int(args.custom_repeat),
                max_test_duration_s=float(args.custom_pivot_max_test_duration_s),
                pivot_timeout_s=float(args.custom_pivot_timeout_s),
            )
            return MotionTestSuite(suite_id="custom_pivot", cases=(case,))
        case = custom_curve_case(
            angle_deg=float(args.curve_angle_deg),
            radius_m=float(args.curve_radius_m),
            speed_mps=float(args.curve_speed_mps),
            case_id=args.custom_case_id,
            repeat=int(args.custom_repeat),
            max_test_duration_s=(
                None
                if args.custom_curve_max_test_duration_s is None
                else float(args.custom_curve_max_test_duration_s)
            ),
        )
        return MotionTestSuite(suite_id="custom_curve", cases=(case,))
    if args.suite_file:
        return load_suite_file(Path(args.suite_file))
    return builtin_suite(args.suite)


def make_suite_output_dir(base_dir: Path, suite_id: str) -> Path:
    safe_suite = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in suite_id) or "suite"
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return base_dir.expanduser() / f"{stamp}_{safe_suite}"


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        suite = load_requested_suite(args)
        cases = expand_repeats(select_cases(suite, args.case))
        global_launch_args = parse_launch_arg_list(args.launch_arg)
    except (OSError, ValueError, argparse.ArgumentTypeError, json.JSONDecodeError) as exc:
        print(f"ugv_motion_test_runner: {exc}", file=sys.stderr)
        return 2

    suite_dir = make_suite_output_dir(Path(args.output_dir), suite.suite_id)
    suite_dir.mkdir(parents=True, exist_ok=True)
    print(f"Writing motion test results to {suite_dir}")
    print(f"Suite {suite.suite_id}: {len(cases)} case(s)")

    case_results: list[dict[str, Any]] = []
    for index, case in enumerate(cases, start=1):
        print(f"[{index}/{len(cases)}] {case.id}")
        try:
            result = run_case(
                case,
                suite_dir=suite_dir,
                global_launch_args=global_launch_args,
                startup_wait_s=float(args.startup_wait_s),
                post_stop_wait_s=float(args.post_stop_wait_s),
                record_bag=bool(args.record_bag),
                yes=bool(args.yes),
                no_launch=bool(args.no_launch),
            )
        except KeyboardInterrupt:
            print("Interrupted; writing summary for completed cases.", file=sys.stderr)
            break
        case_results.append(result)
        write_summary_files(suite_dir, suite.suite_id, case_results)

    write_summary_files(suite_dir, suite.suite_id, case_results)
    print(f"Summary: {suite_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
