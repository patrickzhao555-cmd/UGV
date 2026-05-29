#!/usr/bin/env python3
"""Summarize UGV mission telemetry JSONL logs."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.mission_controller import summarize_mission_records  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze /ugv_nav_status mission telemetry JSONL.")
    parser.add_argument("log", help="Path to a mission telemetry .jsonl file")
    return parser.parse_args()


def load_records(path: pathlib.Path) -> list[dict]:
    records: list[dict] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise SystemExit(f"{path}:{line_number}: invalid JSON: {exc}") from exc
        if isinstance(record, dict):
            records.append(record)
    return records


def main() -> None:
    args = parse_args()
    path = pathlib.Path(args.log).expanduser()
    records = load_records(path)
    summary = summarize_mission_records(records)

    print(f"Mission log: {path}")
    print(f"Samples: {len(records)}")
    print(f"Sub-min speed command count: {summary['sub_min_speed_command_count']}")
    print(f"Critical stop count: {summary['critical_stop_count']}")
    print(f"Sensor stale count: {summary['sensor_stale_count']}")
    print("Segments:")
    for index in sorted(summary["segments"]):
        segment = summary["segments"][index]
        distance_error = segment.get("distance_error_m")
        distance_text = "n/a" if distance_error is None else f"{float(distance_error):.3f}"
        print(
            f"  {index}: type={segment.get('segment_type')} samples={segment.get('samples')} "
            f"last_distance_m={segment.get('last_distance_m'):.3f} "
            f"distance_error_m={distance_text} "
            f"heading_rms_deg={math.degrees(float(segment.get('heading_error_rms_rad') or 0.0)):.2f} "
            f"last_heading_error_rad={segment.get('last_heading_error_rad'):.4f} "
            f"final_error_deg={math.degrees(float(segment.get('final_heading_error_rad') or 0.0)):.2f} "
            f"overshoot_deg={math.degrees(float(segment.get('pivot_overshoot_rad') or 0.0)):.2f} "
            f"max_abs_heading_error_rad={segment.get('max_abs_heading_error_rad'):.4f}"
        )


if __name__ == "__main__":
    main()
