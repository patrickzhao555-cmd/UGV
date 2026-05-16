#!/usr/bin/env python3
"""Summarize navigation replay, command, and status JSONL logs."""

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


PREFIXES = ("REAL CMD ", "SIM CMD ", "NAV STATUS ", "UAV FLAG ")


def _load_json_line(line: str) -> Optional[Dict[str, Any]]:
    line = line.strip().lstrip("\ufeff")
    if not line:
        return None
    for prefix in PREFIXES:
        if line.startswith(prefix):
            line = line[len(prefix):]
            if prefix == "SIM CMD " and ": " in line:
                line = line.split(": ", 1)[1]
            break
    if line.startswith("{") and line.endswith("}"):
        try:
            obj = json.loads(line)
            return obj if isinstance(obj, dict) else None
        except json.JSONDecodeError:
            return None
    return None


def _finite_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _iter_objects(path: Path) -> Iterable[Dict[str, Any]]:
    with path.open("r", encoding="utf-8-sig") as fp:
        for line in fp:
            obj = _load_json_line(line)
            if obj is not None:
                yield obj


def _command_from_object(obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if isinstance(obj.get("cmd"), dict):
        return obj["cmd"]
    if "mode" in obj or "raw_left" in obj or "v_mps" in obj:
        return obj
    return None


def summarize(path: Path) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {
        "total_frames": 0,
        "total_commands": 0,
        "stop_count": 0,
        "no_safe_trajectory_count": 0,
        "active_scan_count": 0,
        "replan_count": 0,
        "average_abs_v_mps": None,
        "average_abs_omega_radps": None,
        "turn_switch_count": 0,
        "forward_command_count": 0,
        "velocity_command_count": 0,
        "raw_command_count": 0,
        "encoder_warning_count": 0,
        "final_distance_to_goal_m": None,
        "min_reported_clearance_m": None,
        "planner_failed_count": 0,
        "stuck_or_blocked_count": 0,
        "missing_fields": [],
    }
    missing = set()
    v_abs: List[float] = []
    omega_abs: List[float] = []
    prev_turn_sign = 0
    last_replans = 0

    for obj in _iter_objects(path):
        is_replay_frame = "left_ticks" in obj and "right_ticks" in obj and "lidar_hits_local" in obj
        if is_replay_frame:
            metrics["total_frames"] += 1

        cmd = _command_from_object(obj)
        if cmd is not None:
            metrics["total_commands"] += 1
            mode = str(cmd.get("mode", "")).upper()
            command_type = str(cmd.get("command_type", "")).lower()
            controller = str(cmd.get("controller", "")).lower()
            reason = str(cmd.get("reason", "")).lower()
            if mode == "STOP" or command_type == "stop":
                metrics["stop_count"] += 1
            if mode == "FORWARD":
                metrics["forward_command_count"] += 1
            if command_type == "velocity":
                metrics["velocity_command_count"] += 1
            elif command_type == "raw":
                metrics["raw_command_count"] += 1
            elif mode != "STOP" and controller == "velocity":
                metrics["velocity_command_count"] += 1
            elif mode != "STOP":
                metrics["raw_command_count"] += 1
            if "no safe" in reason or "no_safe_trajectory" in reason:
                metrics["no_safe_trajectory_count"] += 1
            v = _finite_float(cmd.get("v_mps"))
            omega = _finite_float(cmd.get("omega_radps"))
            if v is not None:
                v_abs.append(abs(v))
            if omega is not None:
                omega_abs.append(abs(omega))
                sign = 1 if omega > 0.05 else -1 if omega < -0.05 else 0
                if sign and prev_turn_sign and sign != prev_turn_sign:
                    metrics["turn_switch_count"] += 1
                if sign:
                    prev_turn_sign = sign

        if "distance_to_goal_m" in obj:
            metrics["final_distance_to_goal_m"] = _finite_float(obj.get("distance_to_goal_m"))
        elif not is_replay_frame:
            missing.add("distance_to_goal_m")

        velocity_debug = obj.get("velocity_control") if isinstance(obj.get("velocity_control"), dict) else {}
        safety_state = str(velocity_debug.get("safety_state", "")).lower()
        if safety_state == "no_safe_trajectory":
            metrics["no_safe_trajectory_count"] += 1
        clearance = _finite_float(velocity_debug.get("min_clearance_m"))
        if clearance is not None:
            prev = metrics["min_reported_clearance_m"]
            metrics["min_reported_clearance_m"] = clearance if prev is None else min(prev, clearance)

        active_scan = obj.get("active_scan") if isinstance(obj.get("active_scan"), dict) else {}
        if active_scan.get("remaining", 0) or active_scan.get("active"):
            metrics["active_scan_count"] += 1

        replans = obj.get("replans")
        if isinstance(replans, int):
            metrics["replan_count"] += max(0, replans - last_replans)
            last_replans = replans
        elif not is_replay_frame:
            missing.add("replans")

        planner = str(obj.get("planner", "")).lower()
        if planner == "failed":
            metrics["planner_failed_count"] += 1
        odom_delta = obj.get("odom_delta") if isinstance(obj.get("odom_delta"), dict) else {}
        if odom_delta.get("warning"):
            metrics["encoder_warning_count"] += 1
        finish_reason = str(obj.get("finish_reason", "")).lower()
        if "stuck" in finish_reason or "blocked" in finish_reason:
            metrics["stuck_or_blocked_count"] += 1

    if v_abs:
        metrics["average_abs_v_mps"] = round(sum(v_abs) / len(v_abs), 4)
    else:
        missing.add("cmd.v_mps")
    if omega_abs:
        metrics["average_abs_omega_radps"] = round(sum(omega_abs) / len(omega_abs), 4)
    else:
        missing.add("cmd.omega_radps")
    metrics["missing_fields"] = sorted(missing)
    return metrics


def print_human(metrics: Dict[str, Any]) -> None:
    print("Replay metrics")
    for key in [
        "total_frames",
        "total_commands",
        "stop_count",
        "no_safe_trajectory_count",
        "active_scan_count",
        "replan_count",
        "average_abs_v_mps",
        "average_abs_omega_radps",
        "turn_switch_count",
        "forward_command_count",
        "velocity_command_count",
        "raw_command_count",
        "encoder_warning_count",
        "final_distance_to_goal_m",
        "min_reported_clearance_m",
        "planner_failed_count",
        "stuck_or_blocked_count",
    ]:
        print(f"  {key}: {metrics.get(key)}")
    if metrics.get("missing_fields"):
        print(f"  missing_fields: {', '.join(metrics['missing_fields'])}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize UGV nav replay/status JSONL metrics.")
    parser.add_argument("path", type=Path)
    parser.add_argument("--json-only", action="store_true")
    args = parser.parse_args()

    metrics = summarize(args.path)
    if not args.json_only:
        print_human(metrics)
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
