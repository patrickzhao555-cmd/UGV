#!/usr/bin/env python3
"""Measure UGV planner latency on the machine that will run the robot.

This script does not need ROS. Run it from the workspace root on the Nano:

    python3 src/ugv_nav/tools/benchmark_replan_latency.py --repeats 50
"""

from __future__ import annotations

import argparse
import math
import random
import statistics
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import ugv_nav_dual_mode as nav  # noqa: E402


FIELD_M = nav.yd(15.0)
START = nav.Pose2D(0.46, 0.46, math.radians(45.0))
GOAL = nav.Pose2D(6.86, 6.86, 0.0)


def _percentile(sorted_values: List[float], pct: float) -> float:
    if not sorted_values:
        return float("nan")
    idx = min(len(sorted_values) - 1, max(0, int(math.ceil(pct * len(sorted_values))) - 1))
    return sorted_values[idx]


def _summary(values: List[float]) -> str:
    values = sorted(values)
    if not values:
        return "no data"
    return (
        f"min={values[0]:7.2f} ms "
        f"median={statistics.median(values):7.2f} ms "
        f"p95={_percentile(values, 0.95):7.2f} ms "
        f"max={values[-1]:7.2f} ms"
    )


def make_navigator(args: argparse.Namespace) -> nav.UGVNavigator:
    robot_cfg = nav.RobotConfig()
    sensor_cfg = nav.SensorConfig()
    nav_cfg = nav.NavConfig()
    nav_cfg.nonstop_when_blocked = True
    nav_cfg.replan_cooldown_s = 0.0
    nav_cfg.front_safety_margin_m = args.front_margin_m
    nav_cfg.rear_safety_margin_m = args.rear_margin_m
    nav_cfg.local_plan_inflation_m = args.local_inflation_m
    nav_cfg.global_plan_inflation_m = args.global_inflation_m
    return nav.UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, FIELD_M, FIELD_M, START, GOAL)


def mark_disc(navigator: nav.UGVNavigator, x_m: float, y_m: float, radius_m: float) -> None:
    navigator.known_costmap.mark_disk_world(x_m, y_m, radius_m)
    navigator._touch_map()


def scenario_empty(navigator: nav.UGVNavigator) -> None:
    del navigator


def scenario_wall_gap(navigator: nav.UGVNavigator) -> None:
    """Obstacle wall with one navigable opening near the diagonal route."""
    x_m = 3.35
    y_m = 0.9
    while y_m <= 8.8:
        if not (4.2 <= y_m <= 5.8):
            mark_disc(navigator, x_m, y_m, 0.16)
        y_m += 0.16


def scenario_dense(navigator: nav.UGVNavigator) -> None:
    """Deterministic clutter while leaving a loose diagonal corridor open."""
    rnd = random.Random(42)
    for _ in range(115):
        x_m = rnd.uniform(1.0, 8.4)
        y_m = rnd.uniform(0.9, 8.8)
        if abs(y_m - x_m) < 0.55:
            continue
        if math.hypot(x_m - START.x, y_m - START.y) < 1.1:
            continue
        if math.hypot(x_m - GOAL.x, y_m - GOAL.y) < 1.1:
            continue
        mark_disc(navigator, x_m, y_m, 0.12)


def benchmark_replans(
    args: argparse.Namespace,
    name: str,
    setup: Callable[[nav.UGVNavigator], None],
) -> None:
    times_ms: List[float] = []
    planners: Dict[str, int] = {}
    path_lengths: List[int] = []
    for i in range(args.repeats):
        navigator = make_navigator(args)
        setup(navigator)
        navigator._plan_costmap_dirty = True
        t0 = time.perf_counter()
        navigator._maybe_replan(1000.0 + i, force=True)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        times_ms.append(elapsed_ms)
        planners[navigator.state.planner_name] = planners.get(navigator.state.planner_name, 0) + 1
        path_lengths.append(len(navigator.state.path))
    print(
        f"replan {name:12s}: {_summary(times_ms)} "
        f"planner={planners} path_len_median={statistics.median(path_lengths):.0f}"
    )


def fake_frame(
    timestamp_s: float,
    hits: List[Tuple[float, float]] | None = None,
    left_ticks: int = 0,
    right_ticks: int = 0,
) -> nav.SensorFrame:
    return nav.SensorFrame(
        encoder=nav.EncoderPacket(left_ticks, right_ticks, timestamp_s),
        lidar=nav.LidarPacket(hits or [], [], [], timestamp_s),
        zed=nav.ZedPacket([], timestamp_s),
        goal=nav.GoalPacket(GOAL.x, GOAL.y, timestamp_s),
    )


def benchmark_step_with_close_obstacle(args: argparse.Namespace) -> None:
    navigator = make_navigator(args)
    navigator.step(fake_frame(0.0))
    close_hits = [(0.33, 0.0), (0.42, 0.10), (0.42, -0.10), (0.55, 0.0)]
    times_ms: List[float] = []
    modes: Dict[str, int] = {}
    for i in range(args.step_repeats):
        frame = fake_frame(0.10 * (i + 1), close_hits, left_ticks=i * 3, right_ticks=i * 3)
        t0 = time.perf_counter()
        command = navigator.step(frame)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        times_ms.append(elapsed_ms)
        modes[command.mode] = modes.get(command.mode, 0) + 1
    print(
        f"step close_obst : {_summary(times_ms)} "
        f"modes={modes} last_plan={navigator.state.plan_time_ms:.2f} ms "
        f"replans={navigator.state.replans}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=50)
    parser.add_argument("--step-repeats", type=int, default=150)
    parser.add_argument("--front-margin-m", type=float, default=0.10)
    parser.add_argument("--rear-margin-m", type=float, default=0.08)
    parser.add_argument("--local-inflation-m", type=float, default=0.08)
    parser.add_argument("--global-inflation-m", type=float, default=0.18)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    for name, setup in (
        ("empty", scenario_empty),
        ("wall_gap", scenario_wall_gap),
        ("dense", scenario_dense),
    ):
        benchmark_replans(args, name, setup)
    benchmark_step_with_close_obstacle(args)


if __name__ == "__main__":
    main()
