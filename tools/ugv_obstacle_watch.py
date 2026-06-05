#!/usr/bin/env python3
"""Human-readable obstacle clearance watcher for field testing."""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any, Dict, Iterable, Optional, Tuple


DEFAULT_SUMMARY_TOPIC = "/sensors/synced_summary"
DEFAULT_NAV_TOPIC = "/ugv_nav_status"
DEFAULT_ZED_TOPIC = "/zed/status"
DEFAULT_MOTOR_TOPIC = "/motor_controller/status"


def _number(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number):
        return None
    return number


def _finite_number(value: Any) -> Optional[float]:
    number = _number(value)
    if number is None or not math.isfinite(number):
        return None
    return number


def _bool_text(value: Optional[bool]) -> str:
    if value is None:
        return "NO_DATA"
    return "TRUE" if value else "FALSE"


def _bool_lower(value: Any) -> str:
    return "true" if bool(value) else "false"


def _fmt_m(value: Optional[float], *, missing: str = "NO_DATA") -> str:
    if value is None:
        return missing
    if not math.isfinite(value):
        return "CLEAR" if value > 0.0 else "NO_DATA"
    return f"{value:.2f}m"


def _fmt_number(value: Optional[float], precision: int = 2, *, missing: str = "NO_DATA") -> str:
    if value is None:
        return missing
    if not math.isfinite(value):
        return missing
    return f"{value:.{precision}f}"


def _is_stale(age_s: Optional[float], stale_timeout_s: float) -> bool:
    return age_s is not None and age_s > max(0.0, float(stale_timeout_s))


def _front_reading_state(fusion: Optional[Dict[str, Any]], *, stale: bool) -> str:
    if fusion is None:
        return "no_data"
    if stale:
        return "stale"
    nearest = _finite_number(fusion.get("front_clearance_m"))
    if nearest is not None:
        return "finite"
    health = str(fusion.get("front_sensor_health") or "")
    source = str(fusion.get("front_clearance_source") or "none")
    if health.startswith("no_front_sensor"):
        return "no_data"
    if source == "none":
        return "clear"
    return "no_data"


def _nav_summary(nav: Optional[Dict[str, Any]]) -> Tuple[str, str, Optional[float]]:
    if not isinstance(nav, dict):
        return "NO_DATA", "NO_DATA", None
    level = nav.get("safety_level")
    reason = nav.get("safety_reason")
    safety = f"{level or 'NO_DATA'}:{reason or 'NO_DATA'}"
    cmd = nav.get("last_command", {})
    if not isinstance(cmd, dict):
        cmd = nav.get("cmd", {})
    if not isinstance(cmd, dict):
        cmd = {}
    command = cmd.get("command_type") or cmd.get("mode") or nav.get("mission_state") or "NO_DATA"
    v_mps = _finite_number(nav.get("v_mps"))
    if v_mps is None:
        v_mps = _finite_number(cmd.get("v_mps"))
    return safety, str(command), v_mps


def _motor_summary(motor: Optional[Dict[str, Any]]) -> str:
    if not isinstance(motor, dict):
        return "NO_DATA"
    connected = motor.get("connected")
    synced = motor.get("teensy_pid_params_synced")
    fault = motor.get("fault_reason") or motor.get("fault") or "none"
    return f"connected={_bool_lower(connected)} synced={_bool_lower(synced)} fault={fault}"


def build_obstacle_snapshot(
    fusion: Optional[Dict[str, Any]],
    nav: Optional[Dict[str, Any]] = None,
    zed: Optional[Dict[str, Any]] = None,
    motor: Optional[Dict[str, Any]] = None,
    *,
    front_threshold_m: float = 2.0,
    stop_clearance_m: float = 1.0,
    fusion_age_s: Optional[float] = None,
    stale_timeout_s: float = 0.75,
) -> Dict[str, Any]:
    stale = _is_stale(fusion_age_s, stale_timeout_s)
    state = _front_reading_state(fusion, stale=stale)
    fusion = fusion if isinstance(fusion, dict) else {}
    zed = zed if isinstance(zed, dict) else {}

    nearest = _finite_number(fusion.get("front_clearance_m"))
    lidar_front = _finite_number(fusion.get("front_lidar_range_m"))
    lidar_any = _finite_number(fusion.get("lidar_any_min_range_m"))
    if lidar_any is None:
        lidar_any = _finite_number(fusion.get("min_lidar_range_m"))
    zed_front = _finite_number(fusion.get("min_depth_range_m"))

    if state == "stale":
        front_obstacle = None
        stop_required = None
        front_label = "STALE"
        stop_label = "STALE"
        nearest_text = "STALE"
    elif state == "no_data":
        front_obstacle = None
        stop_required = None
        front_label = "NO_DATA"
        stop_label = "NO_DATA"
        nearest_text = "NO_DATA"
    elif state == "clear":
        front_obstacle = False
        stop_required = False
        front_label = "FALSE"
        stop_label = "NO"
        nearest_text = "CLEAR"
    else:
        front_obstacle = bool(nearest is not None and nearest <= float(front_threshold_m))
        stop_required = bool(nearest is not None and nearest < float(stop_clearance_m))
        front_label = _bool_text(front_obstacle)
        stop_label = "YES" if stop_required else "NO"
        nearest_text = _fmt_m(nearest)

    any_lidar_obstacle = None
    if lidar_any is not None:
        any_lidar_obstacle = lidar_any <= float(front_threshold_m)

    valid_depth_samples = fusion.get("valid_depth_samples")
    if valid_depth_samples is None:
        valid_depth_samples = zed.get("valid_depth_samples")
    depth_points = fusion.get("depth_obstacle_points_filtered")
    if depth_points is None:
        depth_points = fusion.get("depth_obstacle_points")
    depth_blind = bool(
        fusion.get("depth_blind_hazard_active")
        or fusion.get("depth_blind_hazard")
        or fusion.get("depth_warning")
    )
    safety, command, v_mps = _nav_summary(nav)

    return {
        "front_label": front_label,
        "front_obstacle_within_threshold": front_obstacle,
        "front_threshold_m": float(front_threshold_m),
        "nearest_front_m": nearest,
        "nearest_text": nearest_text,
        "source": str(fusion.get("front_clearance_source") or "none"),
        "stop_clearance_m": float(stop_clearance_m),
        "stop_label": stop_label,
        "front_stop_required": stop_required,
        "zed_front_m": zed_front,
        "valid_depth_samples": valid_depth_samples,
        "depth_obstacle_points": depth_points,
        "depth_blind": depth_blind,
        "lidar_front_m": lidar_front,
        "lidar_any_min_m": lidar_any,
        "any_lidar_within_threshold": any_lidar_obstacle,
        "front_sensor_health": str(fusion.get("front_sensor_health") or state),
        "fusion_age_s": fusion_age_s,
        "nav_safety": safety,
        "nav_command": command,
        "nav_v_mps": v_mps,
        "motor": _motor_summary(motor),
    }


def summarize_obstacle_status(
    fusion: Optional[Dict[str, Any]],
    nav: Optional[Dict[str, Any]] = None,
    zed: Optional[Dict[str, Any]] = None,
    motor: Optional[Dict[str, Any]] = None,
    *,
    front_threshold_m: float = 2.0,
    stop_clearance_m: float = 1.0,
    fusion_age_s: Optional[float] = None,
    stale_timeout_s: float = 0.75,
) -> str:
    snap = build_obstacle_snapshot(
        fusion,
        nav,
        zed,
        motor,
        front_threshold_m=front_threshold_m,
        stop_clearance_m=stop_clearance_m,
        fusion_age_s=fusion_age_s,
        stale_timeout_s=stale_timeout_s,
    )
    return (
        f"FRONT<={snap['front_threshold_m']:.1f}m: {snap['front_label']} | "
        f"nearest={snap['nearest_text']} source={snap['source']} | "
        f"STOP@{snap['stop_clearance_m']:.1f}m: {snap['stop_label']} | "
        f"zed_front={_fmt_m(snap['zed_front_m'], missing='-')} "
        f"valid={snap['valid_depth_samples'] if snap['valid_depth_samples'] is not None else '-'} "
        f"pts={snap['depth_obstacle_points'] if snap['depth_obstacle_points'] is not None else '-'} "
        f"blind={_bool_lower(snap['depth_blind'])} | "
        f"lidar_front={_fmt_m(snap['lidar_front_m'], missing='-')} "
        f"lidar_any={_fmt_m(snap['lidar_any_min_m'], missing='-')} "
        f"any<={snap['front_threshold_m']:.1f}m: {_bool_text(snap['any_lidar_within_threshold'])} | "
        f"health={snap['front_sensor_health']} | "
        f"nav={snap['nav_safety']} cmd={snap['nav_command']} v={_fmt_number(snap['nav_v_mps'], 2, missing='-')} | "
        f"motor={snap['motor']}"
    )


def _parse_json(raw: str) -> Optional[Dict[str, Any]]:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print readable front-obstacle clearance telemetry.")
    parser.add_argument("--summary-topic", default=DEFAULT_SUMMARY_TOPIC)
    parser.add_argument("--nav-topic", default=DEFAULT_NAV_TOPIC)
    parser.add_argument("--zed-topic", default=DEFAULT_ZED_TOPIC)
    parser.add_argument("--motor-topic", default=DEFAULT_MOTOR_TOPIC)
    parser.add_argument("--no-nav", action="store_true")
    parser.add_argument("--no-zed", action="store_true")
    parser.add_argument("--no-motor", action="store_true")
    parser.add_argument("--front-threshold-m", type=float, default=2.0)
    parser.add_argument("--stop-clearance-m", type=float, default=1.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--stale-timeout-s", type=float, default=0.75)
    parser.add_argument("--json-lines", action="store_true")
    return parser.parse_args(list(argv) if argv is not None else None)


def main(argv: Optional[Iterable[str]] = None) -> None:
    args = parse_args(argv)

    import rclpy  # type: ignore
    from std_msgs.msg import String  # type: ignore

    period_s = 1.0 / max(0.2, float(args.hz))
    latest: Dict[str, Tuple[Dict[str, Any], float]] = {}

    rclpy.init()
    node = rclpy.create_node("ugv_obstacle_watch")

    def subscribe_json(key: str, topic: str) -> None:
        def callback(msg: String) -> None:
            data = _parse_json(msg.data)
            if data is not None:
                latest[key] = (data, time.monotonic())

        node.create_subscription(String, topic, callback, 10)

    subscribe_json("fusion", str(args.summary_topic))
    if not args.no_nav:
        subscribe_json("nav", str(args.nav_topic))
    if not args.no_zed:
        subscribe_json("zed", str(args.zed_topic))
    if not args.no_motor:
        subscribe_json("motor", str(args.motor_topic))

    def print_status() -> None:
        now_s = time.monotonic()
        fusion_item = latest.get("fusion")
        fusion = fusion_item[0] if fusion_item else None
        fusion_age_s = None if fusion_item is None else now_s - fusion_item[1]
        nav = latest.get("nav", ({}, 0.0))[0]
        zed = latest.get("zed", ({}, 0.0))[0]
        motor = latest.get("motor", ({}, 0.0))[0]
        if args.json_lines:
            snap = build_obstacle_snapshot(
                fusion,
                nav,
                zed,
                motor,
                front_threshold_m=args.front_threshold_m,
                stop_clearance_m=args.stop_clearance_m,
                fusion_age_s=fusion_age_s,
                stale_timeout_s=args.stale_timeout_s,
            )
            print(json.dumps(snap, sort_keys=True), flush=True)
            return
        print(
            summarize_obstacle_status(
                fusion,
                nav,
                zed,
                motor,
                front_threshold_m=args.front_threshold_m,
                stop_clearance_m=args.stop_clearance_m,
                fusion_age_s=fusion_age_s,
                stale_timeout_s=args.stale_timeout_s,
            ),
            flush=True,
        )

    node.create_timer(period_s, print_status)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
