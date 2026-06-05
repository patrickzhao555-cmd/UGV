#!/usr/bin/env python3
"""Bench PASS/FAIL checks for the Teensy side velocity controller.

Run with the wheels off the ground first. The tool verifies side mapping,
straight speed tracking, and differential tracking from Teensy status lines.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from teensy_serial_test_utils import (
    add_common_serial_args,
    open_teensy_serial,
    prepare_direct_bench_session,
    quiet_status_stream,
    run_with_serial_errors,
    safe_stop_and_restore_stream,
    send_param,
    sync_standard_params,
    write_line,
)


WARNING = "WARNING: first run must be with wheels off the ground."


@dataclass(frozen=True)
class StatusSample:
    left_target_tps: float
    right_target_tps: float
    left_measured_tps: float
    right_measured_tps: float
    left_pwm: int
    right_pwm: int
    fault: str


@dataclass(frozen=True)
class CaseResult:
    name: str
    target_left_mps: float
    target_right_mps: float
    measured_left_mps: Optional[float]
    measured_right_mps: Optional[float]
    pwm_left: Optional[int]
    pwm_right: Optional[int]
    passed: bool
    reason: str


def _parse_float_list(text: str) -> List[float]:
    values: List[float] = []
    for item in str(text).split(","):
        item = item.strip()
        if item:
            values.append(float(item))
    return values


def _parse_pair_list(text: str) -> List[Tuple[float, float]]:
    pairs: List[Tuple[float, float]] = []
    for item in str(text).split(","):
        item = item.strip()
        if not item:
            continue
        left_text, right_text = item.split(":", 1)
        pairs.append((float(left_text), float(right_text)))
    return pairs


def _status_from_line(line: str) -> Optional[StatusSample]:
    text = str(line).strip()
    if not text.startswith("S,"):
        return None
    parts = text.split(",")
    if len(parts) < 25:
        return None
    try:
        return StatusSample(
            left_target_tps=float(parts[10]),
            right_target_tps=float(parts[11]),
            left_measured_tps=float(parts[12]),
            right_measured_tps=float(parts[13]),
            left_pwm=int(round(float(parts[14]))),
            right_pwm=int(round(float(parts[15]))),
            fault=",".join(parts[24:]).strip() or "none",
        )
    except ValueError:
        return None


def _readline_text(dev) -> str:
    return dev.readline().decode("utf-8", errors="replace").strip()


def _tps_to_mps(tps: float, *, wheel_radius_m: float, ticks_per_rev: int) -> float:
    circumference_m = 2.0 * 3.141592653589793 * max(1e-6, float(wheel_radius_m))
    return float(tps) * circumference_m / float(max(1, int(ticks_per_rev)))


def _median(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return float(statistics.median(values))


def _median_int(values: Sequence[int]) -> Optional[int]:
    if not values:
        return None
    return int(round(statistics.median(values)))


def _command_from_sides(left_mps: float, right_mps: float, track_width_m: float) -> str:
    track = max(1e-6, float(track_width_m))
    v_mps = 0.5 * (float(left_mps) + float(right_mps))
    omega_radps = (float(right_mps) - float(left_mps)) / track
    return f"CMD V {v_mps:.6f} {omega_radps:.6f}\n"


def _collect_case(dev, args: argparse.Namespace, name: str, left_mps: float, right_mps: float) -> CaseResult:
    command = _command_from_sides(left_mps, right_mps, args.track_width_m)
    write_line(dev, "CMD STOP\n")
    time.sleep(max(0.05, float(args.between_case_stop_s)))
    try:
        dev.reset_input_buffer()
    except Exception:
        pass

    start_s = time.monotonic()
    deadline_s = start_s + max(0.1, float(args.duration_s))
    sample_start_s = start_s + max(0.0, float(args.discard_s))
    next_refresh_s = start_s
    samples: List[StatusSample] = []
    fault: Optional[str] = None
    while time.monotonic() < deadline_s:
        now_s = time.monotonic()
        if now_s >= next_refresh_s:
            write_line(dev, command)
            next_refresh_s = now_s + max(0.02, float(args.refresh_period_s))
        sample = _status_from_line(_readline_text(dev))
        sample_s = time.monotonic()
        if sample is None:
            continue
        if sample.fault not in {"", "none", "0"}:
            fault = sample.fault
        if sample_s >= sample_start_s:
            samples.append(sample)

    write_line(dev, "CMD STOP\n")
    left_meas = _median(
        [_tps_to_mps(s.left_measured_tps, wheel_radius_m=args.wheel_radius_m, ticks_per_rev=args.ticks_per_rev) for s in samples]
    )
    right_meas = _median(
        [_tps_to_mps(s.right_measured_tps, wheel_radius_m=args.wheel_radius_m, ticks_per_rev=args.ticks_per_rev) for s in samples]
    )
    left_pwm = _median_int([s.left_pwm for s in samples])
    right_pwm = _median_int([s.right_pwm for s in samples])

    if not samples:
        return CaseResult(name, left_mps, right_mps, left_meas, right_meas, left_pwm, right_pwm, False, "no_status_samples")
    if fault:
        return CaseResult(name, left_mps, right_mps, left_meas, right_meas, left_pwm, right_pwm, False, f"fault:{fault}")
    assert left_meas is not None and right_meas is not None

    left_err = left_mps - left_meas
    right_err = right_mps - right_meas
    if name.startswith("mapping_left"):
        passed = (
            left_meas > args.mapping_active_min_mps
            and abs(right_meas) <= args.inactive_side_max_mps
            and left_meas > right_meas + args.ordering_margin_mps
        )
        reason = "ok" if passed else "left_mapping_failed"
    elif name.startswith("mapping_right"):
        passed = (
            right_meas > args.mapping_active_min_mps
            and abs(left_meas) <= args.inactive_side_max_mps
            and right_meas > left_meas + args.ordering_margin_mps
        )
        reason = "ok" if passed else "right_mapping_failed"
    elif name.startswith("straight"):
        passed = (
            abs(left_err) <= args.speed_error_pass_mps
            and abs(right_err) <= args.speed_error_pass_mps
            and abs(left_meas - right_meas) <= args.side_diff_pass_mps
        )
        reason = "ok" if passed else "straight_tracking_failed"
    else:
        target_order = left_mps - right_mps
        measured_order = left_meas - right_meas
        passed = (
            target_order * measured_order > 0.0
            and abs(measured_order) >= args.ordering_margin_mps
            and abs(left_err) <= args.diff_error_pass_mps
            and abs(right_err) <= args.diff_error_pass_mps
        )
        reason = "ok" if passed else "differential_tracking_failed"
    return CaseResult(name, left_mps, right_mps, left_meas, right_meas, left_pwm, right_pwm, passed, reason)


def _print_result(result: CaseResult) -> None:
    ml = "-" if result.measured_left_mps is None else f"{result.measured_left_mps:.3f}"
    mr = "-" if result.measured_right_mps is None else f"{result.measured_right_mps:.3f}"
    pl = "-" if result.pwm_left is None else str(result.pwm_left)
    pr = "-" if result.pwm_right is None else str(result.pwm_right)
    verdict = "PASS" if result.passed else "FAIL"
    print(
        f"{verdict:4s} {result.name:18s} "
        f"target L/R={result.target_left_mps:.3f}/{result.target_right_mps:.3f} m/s "
        f"measured L/R={ml}/{mr} m/s pwm L/R={pl}/{pr} reason={result.reason}"
    )


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--kp", type=float, default=0.03)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--ff-us-per-tps", type=float, default=0.02)
    parser.add_argument("--pid-output-limit-us", type=float, default=180.0)
    parser.add_argument("--speeds-mps", default="0.08,0.12,0.16,0.22")
    parser.add_argument("--diff-pairs-mps", default="0.22:0.10,0.10:0.22")
    parser.add_argument("--mapping-speed-mps", type=float, default=0.12)
    parser.add_argument("--duration-s", type=float, default=2.0)
    parser.add_argument("--discard-s", type=float, default=0.5)
    parser.add_argument("--between-case-stop-s", type=float, default=0.25)
    parser.add_argument("--speed-error-pass-mps", type=float, default=0.04)
    parser.add_argument("--diff-error-pass-mps", type=float, default=0.07)
    parser.add_argument("--side-diff-pass-mps", type=float, default=0.05)
    parser.add_argument("--inactive-side-max-mps", type=float, default=0.04)
    parser.add_argument("--mapping-active-min-mps", type=float, default=0.04)
    parser.add_argument("--ordering-margin-mps", type=float, default=0.03)
    parser.add_argument("--skip-mapping", action="store_true")
    parser.add_argument("--skip-straight", action="store_true")
    parser.add_argument("--skip-differential", action="store_true")
    parser.add_argument("--track-width-m", type=float, default=0.416)
    parser.add_argument("--wheel-radius-m", type=float, default=0.0825)
    parser.add_argument("--ticks-per-rev", type=int, default=3200)
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--left-motor-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--right-motor-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--fl-encoder-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--fr-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--rl-encoder-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--rr-encoder-sign", type=int, choices=(-1, 1), default=1)
    add_common_serial_args(parser)
    parser.add_argument("--yes", action="store_true", help="confirm wheels are off ground and run")
    return parser.parse_args(list(argv) if argv is not None else None)


def _sync_closed_loop_params(dev, args: argparse.Namespace) -> None:
    sync_standard_params(dev, args)
    params = [
        ("kp", args.kp),
        ("ki", args.ki),
        ("kd", args.kd),
        ("ff_us_per_tps", args.ff_us_per_tps),
        ("pid_output_limit_us", args.pid_output_limit_us),
    ]
    for name, value in params:
        send_param(dev, name, value, args.param_timeout_s, args.param_retries)


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)
    print(WARNING, file=sys.stderr)
    print("VISUAL CHECK: confirm the physical side named by each mapping case is the side that moves.", file=sys.stderr)
    if not args.yes:
        print("Refusing to run without --yes.", file=sys.stderr)
        return 2

    cases: List[Tuple[str, float, float]] = []
    mapping_speed = max(0.0, float(args.mapping_speed_mps))
    if not args.skip_mapping:
        cases.append(("mapping_left_only", mapping_speed, 0.0))
        cases.append(("mapping_right_only", 0.0, mapping_speed))
    if not args.skip_straight:
        for speed in _parse_float_list(args.speeds_mps):
            cases.append((f"straight_{speed:.2f}", speed, speed))
    if not args.skip_differential:
        for left_mps, right_mps in _parse_pair_list(args.diff_pairs_mps):
            cases.append((f"diff_{left_mps:.2f}_{right_mps:.2f}", left_mps, right_mps))

    results: List[CaseResult] = []
    with open_teensy_serial(args.port, args.baud) as dev:
        prepare_direct_bench_session(dev, args)
        try:
            _sync_closed_loop_params(dev, args)
            quiet_status_stream(dev, True)
            for name, left_mps, right_mps in cases:
                result = _collect_case(dev, args, name, left_mps, right_mps)
                results.append(result)
                _print_result(result)
        finally:
            safe_stop_and_restore_stream(dev)

    if not results:
        print("FAIL no cases selected")
        return 1
    passed = all(result.passed for result in results)
    print("SUMMARY " + ("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(run_with_serial_errors(main))
