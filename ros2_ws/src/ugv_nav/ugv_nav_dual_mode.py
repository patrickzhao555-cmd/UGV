from __future__ import annotations

import argparse
import json
import math
import heapq
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

import numpy as np
try:
    import cv2
except Exception:
    cv2 = None

from ugv_nav_core.local_costmap import LocalCostmapConfig, RollingLocalCostmap
from ugv_nav_core.nav_config import (
    CM_TO_M,
    FIELD_CELLS_DEFAULT,
    FT_TO_M,
    M_TO_FT,
    YARD_TO_M,
    NavConfig,
    RobotConfig,
    SensorConfig,
    SimConfig,
    drive_speed_factor,
    drive_speed_level_arg,
    ft,
    normalize_drive_speed_level,
    yd,
)
from ugv_nav_core.nav_status import build_nav_status
from ugv_nav_core.sweep_metrics import SweepMetricsLogger
from ugv_nav_core.velocity_planner import VelocityLocalPlanner
from ugv_nav_core.real_mode_runner import (
    apply_competition_v2_motion_policy,
    competition_v2_status_for_update,
    navigation_feedback_for_competition_v2,
)
from ugv_nav_core.competition_mission import (
    CompetitionMissionConfig,
    CompetitionMissionV2,
    normalize_mission_flag_state,
)
from ugv_nav_core.closed_loop_controller import (
    ClosedLoopHealthState,
    RowFollowerOmegaState,
    apply_competition_closed_loop_command,
    compute_heading_hold_correction,
    compute_lane_follow_correction,
    default_closed_loop_debug,
)
from ugv_nav_core.recovery import RecoveryContext, RecoveryState, classify_recovery_state

plt = None
FuncAnimation = None
Line2D = None
Circle = None
Polygon = None
Rectangle = None


def ensure_matplotlib() -> None:
    """Import matplotlib only for simulation visualization.

    Real robot mode does not need matplotlib. Keeping the import lazy prevents
    ROS Humble deployments from dying when an optional Python package upgrades
    NumPy beyond the ABI used by Ubuntu's matplotlib build.
    """
    global plt, FuncAnimation, Line2D, Circle, Polygon, Rectangle
    if plt is not None:
        return

    import matplotlib
    try:
        if os.name == "nt" or os.environ.get("DISPLAY"):
            matplotlib.use("TkAgg")
        else:
            matplotlib.use("Agg")
    except Exception:
        matplotlib.use("Agg")

    import matplotlib.pyplot as _plt
    from matplotlib.animation import FuncAnimation as _FuncAnimation
    from matplotlib.lines import Line2D as _Line2D
    from matplotlib.patches import Circle as _Circle, Polygon as _Polygon, Rectangle as _Rectangle

    plt = _plt
    FuncAnimation = _FuncAnimation
    Line2D = _Line2D
    Circle = _Circle
    Polygon = _Polygon
    Rectangle = _Rectangle

# =========================================================
# Core data types
# =========================================================

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int


@dataclass(frozen=True)
class DiscreteState:
    ix: int
    iy: int
    iyaw: int


@dataclass
class Node:
    state: DiscreteState
    pose: Pose2D
    g: float
    f: float
    parent: Optional[DiscreteState]


@dataclass
class Obstacle:
    kind: str
    x: float
    y: float
    w: float
    h: float


@dataclass
class EncoderPacket:
    left_total: int
    right_total: int
    timestamp: float


@dataclass
class LidarPacket:
    hit_points_local: List[Tuple[float, float]]
    ranges_m: List[float]
    angles_rad: List[float]
    timestamp: float


def laser_scan_to_lidar_observations(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> Tuple[List[Tuple[float, float]], List[float], List[float]]:
    """Convert a LaserScan-like beam list into obstacle hits plus raytrace beams.

    The hit list is intentionally stricter than the clearing beam list:
    max-range, inf, and nan beams are kept for freespace ray clearing but are not
    treated as obstacle returns.
    """
    hits: List[Tuple[float, float]] = []
    ranges_m: List[float] = []
    angles_rad: List[float] = []
    min_r = max(0.0, float(range_min))
    max_r = max(min_r, float(range_max))
    hit_max_r = max_r * 0.995
    angle = float(angle_min)
    inc = float(angle_increment)

    for raw_r in ranges:
        try:
            r = float(raw_r)
        except (TypeError, ValueError):
            r = float("nan")

        real_hit = math.isfinite(r) and min_r <= r < hit_max_r
        if real_hit:
            hits.append((r * math.cos(angle), r * math.sin(angle)))

        if math.isfinite(r):
            clear_r = clamp(r, min_r, max_r)
        else:
            clear_r = max_r
        ranges_m.append(float(clear_r))
        angles_rad.append(float(angle))
        angle += inc

    return hits, ranges_m, angles_rad


@dataclass
class ZedPacket:
    hit_points_local: List[Tuple[float, float]]
    timestamp: float


@dataclass
class ImuPacket:
    angular_velocity_rps: Tuple[float, float, float]
    linear_accel_mps2: Tuple[float, float, float]
    timestamp: float

    def yaw_rate(self, axis: str = "z", sign: float = 1.0) -> float:
        axes = {"x": 0, "y": 1, "z": 2}
        idx = axes.get(str(axis).lower(), 2)
        return float(sign) * float(self.angular_velocity_rps[idx])


@dataclass
class GoalPacket:
    x: float
    y: float
    timestamp: float
    distance_m: Optional[float] = None


@dataclass
class FieldMapPacket:
    size: int
    cell_size_m: float
    obstacle_cells: List[Tuple[int, int]]
    start_xy: Optional[Tuple[float, float]]
    goal_xy: Optional[Tuple[float, float]]
    timestamp: float
    version: int
    source: str = "field_map"


@dataclass
class MissionFlagPacket:
    state: str
    source: str
    timestamp: float
    raw: Dict[str, Any] = field(default_factory=dict)


FieldMapKey = Tuple[
    int,
    float,
    Tuple[Tuple[int, int], ...],
    Optional[Tuple[float, float]],
    Optional[Tuple[float, float]],
    str,
]


def _field_map_xy_key(xy: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    if xy is None:
        return None
    return round(float(xy[0]), 4), round(float(xy[1]), 4)


def field_map_content_key(packet: FieldMapPacket) -> FieldMapKey:
    return (
        int(packet.size),
        round(float(packet.cell_size_m), 6),
        tuple(packet.obstacle_cells),
        _field_map_xy_key(packet.start_xy),
        _field_map_xy_key(packet.goal_xy),
        str(packet.source),
    )


def parse_mission_flag(raw: str, timestamp: float) -> MissionFlagPacket:
    source = "manual"
    payload: Dict[str, Any] = {"raw": str(raw)}
    state_value = str(raw)
    try:
        parsed = json.loads(raw)
        if isinstance(parsed, dict):
            payload = parsed
            source = str(parsed.get("source", source))
            for key in ("state", "uav_state", "flag", "event", "mode"):
                if key in parsed:
                    state_value = str(parsed[key])
                    break
        else:
            state_value = str(parsed)
            payload = {"raw": parsed}
    except json.JSONDecodeError:
        pass

    state = state_value.strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "land": "landing",
        "uav_land": "landing",
        "uav_landing": "landing",
        "descent": "landing",
        "descending": "landing",
        "takeoff": "leaving",
        "take_off": "leaving",
        "uav_leaving": "leaving",
        "departing": "leaving",
        "scan": "scanning",
        "search": "scanning",
        "touchdown": "landed",
        "uav_landed": "landed",
        "landing_done": "landing_complete",
        "landing_finished": "landing_complete",
        "land_complete": "landing_complete",
        "uav_landing_complete": "landing_complete",
        "mission_done": "mission_complete",
        "mission_finished": "mission_complete",
        "finished": "complete",
        "finish": "complete",
        "round_finished": "round_complete",
        "stopping": "stop",
        "e_stop": "emergency_stop",
        "estop": "emergency_stop",
    }
    state = normalize_mission_flag_state(aliases.get(state, state or "unknown"))
    return MissionFlagPacket(state=state, source=source, timestamp=timestamp, raw=payload)


@dataclass
class SensorFrame:
    encoder: EncoderPacket
    lidar: LidarPacket
    zed: ZedPacket
    goal: GoalPacket
    imu: Optional[ImuPacket] = None
    field_map: Optional[FieldMapPacket] = None
    marker_goal: Optional[GoalPacket] = None
    mission_flag: Optional[MissionFlagPacket] = None


@dataclass
class ControlCommand:
    mode: str
    turn_deg: float = 0.0
    move_m: float = 0.0
    raw_left: float = 0.0
    raw_right: float = 0.0
    reason: str = ""
    v_mps: float = 0.0
    omega_radps: float = 0.0
    controller: str = "step"
    command_type: str = ""

    def short_text(self) -> str:
        if self.mode in {"TURN_LEFT", "TURN_RIGHT"}:
            return f"{self.mode} {self.turn_deg:.1f} deg"
        if self.mode in {"FORWARD", "BACKWARD"}:
            return f"{self.mode} {self.move_m:.2f} m"
        return self.mode

    def as_dict(self) -> dict:
        command_type = self.command_type
        if not command_type:
            if self.mode == "STOP":
                command_type = "stop"
            elif self.controller == "velocity":
                command_type = "velocity"
            else:
                command_type = "raw"
        return {
            "mode": self.mode,
            "command_type": command_type,
            "turn_deg": round(self.turn_deg, 3),
            "move_m": round(self.move_m, 4),
            "raw_left": round(self.raw_left, 3),
            "raw_right": round(self.raw_right, 3),
            "reason": self.reason,
            "v_mps": round(self.v_mps, 4),
            "omega_radps": round(self.omega_radps, 4),
            "controller": self.controller,
        }


def scale_control_command(cmd: ControlCommand, scale: float, reason: str, min_motion_raw: float = 0.0) -> ControlCommand:
    scale = clamp(float(scale), 0.0, 1.0)
    if cmd.mode == "STOP":
        return cmd
    if cmd.controller == "active_scan":
        scale = max(scale, 0.75)
    suffix = ""
    if abs(scale - 1.0) >= 1e-3:
        suffix = f"; speed_scale={scale:.2f}"
    if reason and suffix:
        suffix += f" {reason}"
    elif reason:
        suffix = f"; {reason}"
    raw_left = cmd.raw_left * scale
    raw_right = cmd.raw_right * scale
    v_mps = cmd.v_mps * scale
    omega_radps = cmd.omega_radps * scale
    min_raw = max(0.0, float(min_motion_raw))
    if min_raw > 0.0 and cmd.mode in {"FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT"}:
        before = (raw_left, raw_right)

        if cmd.controller == "velocity":
            max_abs = max(abs(raw_left), abs(raw_right))
            if 1e-9 < max_abs < min_raw:
                factor = min_raw / max_abs
                raw_left = clamp(raw_left * factor, -1.0, 1.0)
                raw_right = clamp(raw_right * factor, -1.0, 1.0)
        else:
            def floor_raw(value: float) -> float:
                if abs(value) < 1e-9:
                    return 0.0
                return math.copysign(min(1.0, max(abs(value), min_raw)), value)

            raw_left = floor_raw(raw_left)
            raw_right = floor_raw(raw_right)
        if (raw_left, raw_right) != before:
            suffix += f"; min_motion_raw={min_raw:.2f}"
    return ControlCommand(
        mode=cmd.mode,
        turn_deg=cmd.turn_deg,
        move_m=cmd.move_m,
        raw_left=raw_left,
        raw_right=raw_right,
        reason=f"{cmd.reason}{suffix}",
        v_mps=v_mps,
        omega_radps=omega_radps,
        controller=cmd.controller,
        command_type=cmd.command_type,
    )


# =========================================================
# Math helpers
# =========================================================


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class PhysicalStallState:
    detected: bool = False
    steps: int = 0
    duration_s: float = 0.0
    reason: str = ""


def command_meaningful_for_physical_stall(
    cmd: ControlCommand,
    *,
    min_raw: float = 0.12,
    min_v_mps: float = 0.04,
) -> bool:
    if cmd.mode == "STOP":
        return False
    raw_meaningful = max(abs(float(cmd.raw_left)), abs(float(cmd.raw_right))) >= max(0.0, float(min_raw))
    velocity_meaningful = abs(float(cmd.v_mps)) >= max(0.0, float(min_v_mps)) or abs(float(cmd.omega_radps)) >= 0.12
    return bool(raw_meaningful or velocity_meaningful)


def update_physical_stall_state(
    state: PhysicalStallState,
    cmd: ControlCommand,
    odom_delta: Dict[str, Any],
    *,
    timeout_s: float = 1.5,
    min_raw: float = 0.12,
    min_v_mps: float = 0.04,
    linear_epsilon_m: float = 0.006,
    angular_epsilon_deg: float = 1.0,
) -> PhysicalStallState:
    dt_s = max(0.0, float(odom_delta.get("dt_s") or 0.0))
    ds_m = abs(float(odom_delta.get("ds_used_m", odom_delta.get("ds_m", 0.0)) or 0.0))
    dtheta_deg = abs(float(odom_delta.get("dtheta_deg") or 0.0))
    meaningful = command_meaningful_for_physical_stall(cmd, min_raw=min_raw, min_v_mps=min_v_mps)
    no_motion = ds_m <= max(0.0, float(linear_epsilon_m)) and dtheta_deg <= max(0.0, float(angular_epsilon_deg))
    if meaningful and no_motion and dt_s > 0.0:
        state.steps += 1
        state.duration_s += dt_s
        state.detected = state.duration_s >= max(0.0, float(timeout_s))
        state.reason = (
            f"active_command_zero_odom_{state.duration_s:.2f}s"
            if state.detected
            else f"active_command_zero_odom_{state.duration_s:.2f}s_pending"
        )
    else:
        state.detected = False
        state.steps = 0
        state.duration_s = 0.0
        state.reason = ""
    return state


def normalize_corner_name(name: str) -> str:
    value = str(name or "lower_left").strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "ll": "lower_left",
        "bottom_left": "lower_left",
        "sw": "lower_left",
        "left_bottom": "lower_left",
        "ul": "upper_left",
        "top_left": "upper_left",
        "nw": "upper_left",
        "left_top": "upper_left",
        "lr": "lower_right",
        "bottom_right": "lower_right",
        "se": "lower_right",
        "right_bottom": "lower_right",
        "ur": "upper_right",
        "top_right": "upper_right",
        "ne": "upper_right",
        "right_top": "upper_right",
    }
    value = aliases.get(value, value)
    if value not in {"lower_left", "upper_left", "lower_right", "upper_right"}:
        raise ValueError(f"Unsupported start corner {name!r}")
    return value


def normalize_mission_mode(mode: str, competition_mode: bool = False) -> str:
    value = str(mode or "manual").strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "r1": "round1",
        "round_1": "round1",
        "straight": "round1",
        "straight_line": "round1",
        "r2": "round2",
        "round_2": "round2",
        "uav_landing": "round2",
        "marker_landing": "round2",
        "r3": "round3",
        "round_3": "round3",
        "competition": "round3",
        "room": "indoor",
        "classroom": "indoor",
        "roomba": "indoor",
        "wander": "indoor",
        "indoor_search": "indoor",
        "manual_goal": "manual",
        "normal": "manual",
    }
    value = aliases.get(value, value)
    if competition_mode and value == "manual":
        value = "round3"
    if value not in {"manual", "round1", "round2", "round3", "indoor"}:
        raise ValueError(f"Unsupported mission mode {mode!r}")
    return value


def field_cell_to_world(row: int, col: int, size: int = FIELD_CELLS_DEFAULT, cell_size_m: float = YARD_TO_M) -> Tuple[float, float]:
    x = (float(col) + 0.5) * cell_size_m
    y = (float(size - 1 - row) + 0.5) * cell_size_m
    return x, y


def start_pose_for_corner(
    corner: str,
    field_w_m: float = yd(15.0),
    field_h_m: float = yd(15.0),
    inset_m: float = 0.5 * YARD_TO_M,
) -> Pose2D:
    corner = normalize_corner_name(corner)
    x = inset_m if "left" in corner else field_w_m - inset_m
    y = inset_m if "lower" in corner else field_h_m - inset_m
    yaw = math.atan2(0.5 * field_h_m - y, 0.5 * field_w_m - x)
    return Pose2D(x, y, yaw)


def start_pose_from_xy(
    x_m: float,
    y_m: float,
    field_w_m: float = yd(15.0),
    field_h_m: float = yd(15.0),
    yaw_deg: Optional[float] = None,
    inset_m: float = 0.05,
) -> Pose2D:
    x = clamp(float(x_m), inset_m, max(inset_m, field_w_m - inset_m))
    y = clamp(float(y_m), inset_m, max(inset_m, field_h_m - inset_m))
    if yaw_deg is not None and math.isfinite(float(yaw_deg)):
        yaw = math.radians(float(yaw_deg))
    else:
        yaw = math.atan2(0.5 * field_h_m - y, 0.5 * field_w_m - x)
    return Pose2D(x, y, yaw)


def finite_optional(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _cell_value_role(value: Any) -> Optional[str]:
    if isinstance(value, str):
        token = value.strip().lower()
        if token in {"1", "o", "obs", "obstacle", "blocked", "block", "x", "#"}:
            return "obstacle"
        if token in {"2", "s", "start", "ugv", "robot", "current"}:
            return "start"
        if token in {"3", "g", "goal", "m", "marker", "destination", "dest", "target"}:
            return "goal"
        return None
    if isinstance(value, (int, float)):
        code = int(value)
        if code == 1:
            return "obstacle"
        if code == 2:
            return "start"
        if code == 3:
            return "goal"
    return None


def _coerce_cell(value: Any) -> Optional[Tuple[int, int]]:
    if isinstance(value, dict):
        if "row" in value and "col" in value:
            return int(value["row"]), int(value["col"])
        if "r" in value and "c" in value:
            return int(value["r"]), int(value["c"])
        if "y" in value and "x" in value:
            return int(value["y"]), int(value["x"])
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return int(value[0]), int(value[1])
    return None


def _coerce_xy_m(value: Any) -> Optional[Tuple[float, float]]:
    if isinstance(value, dict):
        key_pairs = (
            ("x", "y"),
            ("x_m", "y_m"),
            ("target_x", "target_y"),
            ("target_x_m", "target_y_m"),
            ("marker_x", "marker_y"),
            ("marker_x_m", "marker_y_m"),
            ("goal_x", "goal_y"),
            ("goal_x_m", "goal_y_m"),
        )
        for x_key, y_key in key_pairs:
            if x_key in value and y_key in value:
                return float(value[x_key]), float(value[y_key])
        if "coordinates" in value:
            return _coerce_xy_m(value["coordinates"])
        if "coord" in value:
            return _coerce_xy_m(value["coord"])
        if "point" in value:
            return _coerce_xy_m(value["point"])
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        if isinstance(value[0], (int, float)) and isinstance(value[1], (int, float)):
            return float(value[0]), float(value[1])
    if isinstance(value, str):
        text = value.strip()
        if "," in text:
            parts = [p.strip() for p in text.split(",")]
            if len(parts) >= 2:
                return float(parts[0]), float(parts[1])
    return None


def _find_named_xy_m(obj: Dict[str, Any], names: Sequence[str]) -> Optional[Tuple[float, float]]:
    for name in names:
        if name in obj:
            xy = _coerce_xy_m(obj[name])
            if xy is not None:
                return xy
    return None


def _find_target_xy_m(obj: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    xy = _find_named_xy_m(obj, ("target", "marker", "goal", "destination", "target_location", "marker_location"))
    if xy is not None:
        return xy
    return _coerce_xy_m(obj)


def _find_start_xy_m(obj: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    xy = _find_named_xy_m(obj, ("start", "ugv", "robot", "current", "start_location", "ugv_start"))
    if xy is not None:
        return xy
    for x_key, y_key in (
        ("start_x", "start_y"),
        ("start_x_m", "start_y_m"),
        ("ugv_x", "ugv_y"),
        ("ugv_x_m", "ugv_y_m"),
        ("robot_x", "robot_y"),
        ("robot_x_m", "robot_y_m"),
    ):
        if x_key in obj and y_key in obj:
            return float(obj[x_key]), float(obj[y_key])
    return None


def _clamp_xy_to_field(xy: Tuple[float, float], size: int, cell_size_m: float, margin_m: float = 0.05) -> Tuple[float, float]:
    field_extent_m = max(margin_m, float(size) * float(cell_size_m))
    return (
        clamp(float(xy[0]), margin_m, max(margin_m, field_extent_m - margin_m)),
        clamp(float(xy[1]), margin_m, max(margin_m, field_extent_m - margin_m)),
    )


def _find_named_cell(obj: Dict[str, Any], names: Sequence[str]) -> Optional[Tuple[int, int]]:
    for name in names:
        if name in obj:
            cell = _coerce_cell(obj[name])
            if cell is not None:
                return cell
    return None


def field_map_from_json(raw: str, timestamp: float, version: int) -> FieldMapPacket:
    try:
        obj = json.loads(raw)
    except json.JSONDecodeError:
        target_xy = _coerce_xy_m(raw)
        if target_xy is None:
            raise
        target_xy = _clamp_xy_to_field(target_xy, FIELD_CELLS_DEFAULT, YARD_TO_M)
        return FieldMapPacket(
            size=FIELD_CELLS_DEFAULT,
            cell_size_m=YARD_TO_M,
            obstacle_cells=[],
            start_xy=None,
            goal_xy=target_xy,
            timestamp=timestamp,
            version=version,
            source="uav_target_xy",
        )

    if isinstance(obj, list):
        target_xy = _coerce_xy_m(obj)
        if target_xy is not None and not any(isinstance(item, list) for item in obj[:2]):
            target_xy = _clamp_xy_to_field(target_xy, FIELD_CELLS_DEFAULT, YARD_TO_M)
            return FieldMapPacket(
                size=FIELD_CELLS_DEFAULT,
                cell_size_m=YARD_TO_M,
                obstacle_cells=[],
                start_xy=None,
                goal_xy=target_xy,
                timestamp=timestamp,
                version=version,
                source="uav_target_xy",
            )
        obj = {"matrix": obj}
    if isinstance(obj, str):
        target_xy = _coerce_xy_m(obj)
        if target_xy is not None:
            target_xy = _clamp_xy_to_field(target_xy, FIELD_CELLS_DEFAULT, YARD_TO_M)
            return FieldMapPacket(
                size=FIELD_CELLS_DEFAULT,
                cell_size_m=YARD_TO_M,
                obstacle_cells=[],
                start_xy=None,
                goal_xy=target_xy,
                timestamp=timestamp,
                version=version,
                source="uav_target_xy",
            )
    if not isinstance(obj, dict):
        raise ValueError("target/map JSON must be an object, an [x,y] target, or a 2D array")

    matrix = obj.get("matrix", obj.get("grid", obj.get("map")))
    if matrix is None:
        target_xy = _find_target_xy_m(obj)
        if target_xy is None:
            raise ValueError("target JSON needs x/y meter fields, or field map JSON needs a matrix/grid/map field")
        size = int(obj.get("size", FIELD_CELLS_DEFAULT))
        cell_size_m = float(obj.get("cell_size_m", YARD_TO_M))
        target_xy = _clamp_xy_to_field(target_xy, size, cell_size_m)
        start_xy = _find_start_xy_m(obj)
        if start_xy is not None:
            start_xy = _clamp_xy_to_field(start_xy, size, cell_size_m)
        return FieldMapPacket(
            size=size,
            cell_size_m=cell_size_m,
            obstacle_cells=[],
            start_xy=start_xy,
            goal_xy=target_xy,
            timestamp=timestamp,
            version=version,
            source=str(obj.get("source", "uav_target_xy")),
        )
    if not isinstance(matrix, list) or not matrix:
        raise ValueError("field map matrix must be a non-empty 2D list")

    size = int(obj.get("size", len(matrix)))
    cell_size_yard = float(obj.get("cell_size_yard", obj.get("cell_yard", 1.0)))
    cell_size_m = float(obj.get("cell_size_m", cell_size_yard * YARD_TO_M))
    if size <= 0 or cell_size_m <= 0.0:
        raise ValueError("field map size and cell size must be positive")

    obstacle_cells: Set[Tuple[int, int]] = set()
    start_cell: Optional[Tuple[int, int]] = _find_named_cell(
        obj, ("ugv", "robot", "start", "start_cell", "ugv_cell", "current")
    )
    goal_cell: Optional[Tuple[int, int]] = _find_named_cell(
        obj, ("marker", "goal", "destination", "target", "marker_cell", "goal_cell")
    )

    for row, row_values in enumerate(matrix):
        if not isinstance(row_values, list):
            continue
        for col, value in enumerate(row_values):
            role = _cell_value_role(value)
            if role == "obstacle":
                obstacle_cells.add((row, col))
            elif role == "start" and start_cell is None:
                start_cell = (row, col)
            elif role == "goal" and goal_cell is None:
                goal_cell = (row, col)

    for cell_obj in obj.get("obstacles", []):
        cell = _coerce_cell(cell_obj)
        if cell is not None:
            obstacle_cells.add(cell)

    def valid_cell(cell: Optional[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
        if cell is None:
            return None
        row, col = cell
        if 0 <= row < size and 0 <= col < size:
            return row, col
        return None

    start_cell = valid_cell(start_cell)
    goal_cell = valid_cell(goal_cell)
    obstacle_cells = {
        (row, col)
        for row, col in obstacle_cells
        if 0 <= row < size and 0 <= col < size and (row, col) not in {start_cell, goal_cell}
    }

    return FieldMapPacket(
        size=size,
        cell_size_m=cell_size_m,
        obstacle_cells=sorted(obstacle_cells),
        start_xy=field_cell_to_world(*start_cell, size=size, cell_size_m=cell_size_m) if start_cell else None,
        goal_xy=field_cell_to_world(*goal_cell, size=size, cell_size_m=cell_size_m) if goal_cell else None,
        timestamp=timestamp,
        version=version,
        source=str(obj.get("source", "field_map")),
    )


# =========================================================
# Costmap
# =========================================================

class Costmap2D:
    def __init__(self, spec: GridSpec, data: np.ndarray):
        self.spec = spec
        self.data = data.astype(np.uint8)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.spec.width and 0 <= gy < self.spec.height

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.spec.origin_x) / self.spec.resolution)
        gy = int((y - self.spec.origin_y) / self.spec.resolution)
        return gx, gy

    def grid_to_world_center(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.spec.origin_x + (gx + 0.5) * self.spec.resolution
        y = self.spec.origin_y + (gy + 0.5) * self.spec.resolution
        return x, y

    def is_occupied(self, gx: int, gy: int) -> bool:
        if not self.in_bounds(gx, gy):
            return True
        return self.data[gy, gx] >= 1

    def is_occupied_world(self, x: float, y: float) -> bool:
        gx, gy = self.world_to_grid(x, y)
        return self.is_occupied(gx, gy)

    def mark_disk_world(self, x: float, y: float, radius_m: float) -> None:
        gx0, gy0 = self.world_to_grid(x - radius_m, y - radius_m)
        gx1, gy1 = self.world_to_grid(x + radius_m, y + radius_m)
        gx0 = max(0, gx0 - 1)
        gy0 = max(0, gy0 - 1)
        gx1 = min(self.spec.width - 1, gx1 + 1)
        gy1 = min(self.spec.height - 1, gy1 + 1)

        r2 = radius_m * radius_m
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                cx, cy = self.grid_to_world_center(gx, gy)
                if (cx - x) ** 2 + (cy - y) ** 2 <= r2:
                    self.data[gy, gx] = 1

    def clear_disk_world(self, x: float, y: float, radius_m: float) -> None:
        gx0, gy0 = self.world_to_grid(x - radius_m, y - radius_m)
        gx1, gy1 = self.world_to_grid(x + radius_m, y + radius_m)
        gx0 = max(0, gx0 - 1)
        gy0 = max(0, gy0 - 1)
        gx1 = min(self.spec.width - 1, gx1 + 1)
        gy1 = min(self.spec.height - 1, gy1 + 1)

        r2 = radius_m * radius_m
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                cx, cy = self.grid_to_world_center(gx, gy)
                if (cx - x) ** 2 + (cy - y) ** 2 <= r2:
                    self.data[gy, gx] = 0

    def clear_segment_world(self, ax: float, ay: float, bx: float, by: float, radius_m: float = 0.04) -> None:
        dist = math.hypot(bx - ax, by - ay)
        n = max(1, int(dist / max(self.spec.resolution * 0.75, 0.04)))
        for i in range(n + 1):
            t = i / n
            x = ax + t * (bx - ax)
            y = ay + t * (by - ay)
            self.clear_disk_world(x, y, radius_m)


# =========================================================
# Hybrid A*
# =========================================================

class HybridAStarPlanner:
    def __init__(
        self,
        robot_cfg: RobotConfig,
        yaw_bins: int = 48,
        step_size: float = 0.22,
        max_steer_deg: float = 38.0,
        steer_samples: int = 7,
        wheelbase: float = 0.42,
        reverse_allowed: bool = True,
        allow_turn_in_place: bool = True,
        spin_step_deg: float = 18.0,
        spin_cost_per_rad: float = 0.28,
        goal_tolerance_xy: float = 0.16,
        goal_tolerance_yaw_deg: float = 28.0,
        obstacle_check_step: float = 0.06,
        heuristic_weight: float = 1.05,
        reverse_penalty: float = 1.25,
        steer_penalty: float = 0.16,
        steer_change_penalty: float = 0.08,
        max_nodes: int = 80000,
    ):
        self.robot_cfg = robot_cfg
        self.yaw_bins = yaw_bins
        self.step_size = step_size
        self.max_steer = math.radians(max_steer_deg)
        self.steer_samples = steer_samples
        self.wheelbase = wheelbase
        self.reverse_allowed = reverse_allowed
        self.allow_turn_in_place = allow_turn_in_place
        self.spin_step = math.radians(spin_step_deg)
        self.spin_cost_per_rad = spin_cost_per_rad
        self.goal_tolerance_xy = goal_tolerance_xy
        self.goal_tolerance_yaw = math.radians(goal_tolerance_yaw_deg)
        self.obstacle_check_step = obstacle_check_step
        self.heuristic_weight = heuristic_weight
        self.reverse_penalty = reverse_penalty
        self.steer_penalty = steer_penalty
        self.steer_change_penalty = steer_change_penalty
        self.max_nodes = max_nodes

    def plan(self, costmap: Costmap2D, start: Pose2D, goal: Pose2D) -> List[Pose2D]:
        if self._pose_in_collision(costmap, start) or self._pose_in_collision(costmap, goal):
            return []

        start_key = self._disc(costmap, start)
        open_heap: List[Tuple[float, int, DiscreteState]] = []
        open_dict: Dict[DiscreteState, Node] = {}
        closed: Dict[DiscreteState, Node] = {}
        last_steer: Dict[DiscreteState, float] = {start_key: 0.0}

        push_id = 0
        h0 = self._heuristic(start, goal)
        open_dict[start_key] = Node(start_key, start, 0.0, h0, None)
        heapq.heappush(open_heap, (h0, push_id, start_key))
        expanded = 0

        while open_heap:
            _, _, cur_key = heapq.heappop(open_heap)
            if cur_key not in open_dict:
                continue
            cur = open_dict.pop(cur_key)
            closed[cur_key] = cur
            expanded += 1
            if expanded > self.max_nodes:
                return []

            if self._is_goal(cur.pose, goal):
                return self._reconstruct(closed, cur_key)

            prev_steer = last_steer.get(cur_key, 0.0)

            for yaw_delta in self._spin_primitives():
                nxt_pose = Pose2D(cur.pose.x, cur.pose.y, wrap_to_pi(cur.pose.yaw + yaw_delta))
                if self._spin_in_collision(costmap, cur.pose, nxt_pose):
                    continue
                nxt_key = self._disc(costmap, nxt_pose)
                if nxt_key in closed:
                    continue

                g2 = cur.g + abs(yaw_delta) * self.spin_cost_per_rad
                h2 = self._heuristic(nxt_pose, goal)
                f2 = g2 + self.heuristic_weight * h2
                ex = open_dict.get(nxt_key)
                if ex is None or g2 < ex.g:
                    open_dict[nxt_key] = Node(nxt_key, nxt_pose, g2, f2, cur_key)
                    last_steer[nxt_key] = 0.0
                    push_id += 1
                    heapq.heappush(open_heap, (f2, push_id, nxt_key))

            for steer, direction in self._drive_primitives():
                nxt_pose = self._simulate_drive(cur.pose, steer, direction)
                if nxt_pose is None:
                    continue
                if self._segment_in_collision(costmap, cur.pose, nxt_pose):
                    continue
                nxt_key = self._disc(costmap, nxt_pose)
                if nxt_key in closed:
                    continue

                step_cost = self.step_size
                if direction < 0:
                    step_cost *= self.reverse_penalty
                step_cost *= 1.0 + self.steer_penalty * abs(steer)
                step_cost *= 1.0 + self.steer_change_penalty * abs(steer - prev_steer)
                g2 = cur.g + step_cost
                h2 = self._heuristic(nxt_pose, goal)
                f2 = g2 + self.heuristic_weight * h2
                ex = open_dict.get(nxt_key)
                if ex is None or g2 < ex.g:
                    open_dict[nxt_key] = Node(nxt_key, nxt_pose, g2, f2, cur_key)
                    last_steer[nxt_key] = steer
                    push_id += 1
                    heapq.heappush(open_heap, (f2, push_id, nxt_key))

        return []

    def _disc(self, costmap: Costmap2D, pose: Pose2D) -> DiscreteState:
        gx, gy = costmap.world_to_grid(pose.x, pose.y)
        yaw = pose.yaw % (2.0 * math.pi)
        iyaw = int(yaw / (2.0 * math.pi) * self.yaw_bins) % self.yaw_bins
        return DiscreteState(gx, gy, iyaw)

    def _heuristic(self, a: Pose2D, b: Pose2D) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _is_goal(self, pose: Pose2D, goal: Pose2D) -> bool:
        return (
            math.hypot(pose.x - goal.x, pose.y - goal.y) <= self.goal_tolerance_xy
            and abs(wrap_to_pi(pose.yaw - goal.yaw)) <= self.goal_tolerance_yaw
        )

    def _drive_primitives(self) -> List[Tuple[float, int]]:
        steers = np.linspace(-self.max_steer, self.max_steer, self.steer_samples).tolist()
        out: List[Tuple[float, int]] = []
        for s in steers:
            out.append((s, +1))
            if self.reverse_allowed:
                out.append((s, -1))
        return out

    def _spin_primitives(self) -> List[float]:
        if not self.allow_turn_in_place:
            return []
        return [-self.spin_step, self.spin_step]

    def _simulate_drive(self, pose: Pose2D, steer: float, direction: int) -> Optional[Pose2D]:
        ds = self.step_size * float(direction)
        if abs(steer) < 1e-6:
            return Pose2D(pose.x + ds * math.cos(pose.yaw), pose.y + ds * math.sin(pose.yaw), pose.yaw)
        beta = ds / self.wheelbase * math.tan(steer)
        return Pose2D(
            pose.x + ds * math.cos(pose.yaw),
            pose.y + ds * math.sin(pose.yaw),
            wrap_to_pi(pose.yaw + beta),
        )

    def _spin_in_collision(self, costmap: Costmap2D, a: Pose2D, b: Pose2D) -> bool:
        dtheta = wrap_to_pi(b.yaw - a.yaw)
        n = max(2, int(abs(dtheta) / math.radians(5.0)))
        for i in range(n + 1):
            t = i / n
            if self._pose_in_collision(costmap, Pose2D(a.x, a.y, wrap_to_pi(a.yaw + t * dtheta))):
                return True
        return False

    def _segment_in_collision(self, costmap: Costmap2D, a: Pose2D, b: Pose2D) -> bool:
        dist = math.hypot(b.x - a.x, b.y - a.y)
        n = max(2, int(dist / self.obstacle_check_step))
        dyaw = wrap_to_pi(b.yaw - a.yaw)
        for i in range(n + 1):
            t = i / n
            p = Pose2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), wrap_to_pi(a.yaw + t * dyaw))
            if self._pose_in_collision(costmap, p):
                return True
        return False

    def _pose_in_collision(self, costmap: Costmap2D, pose: Pose2D) -> bool:
        half_l = self.robot_cfg.length_m / 2.0 + self.robot_cfg.obstacle_buffer_m
        half_w = self.robot_cfg.width_m / 2.0 + self.robot_cfg.obstacle_buffer_m
        sample_step = max(costmap.spec.resolution * 0.95, 0.06)
        nx = max(3, int((2.0 * half_l) / sample_step) + 1)
        ny = max(3, int((2.0 * half_w) / sample_step) + 1)
        xs = np.linspace(-half_l, half_l, nx)
        ys = np.linspace(-half_w, half_w, ny)
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        for lx in xs:
            for ly in ys:
                wx = pose.x + lx * c - ly * s
                wy = pose.y + lx * s + ly * c
                if costmap.is_occupied_world(wx, wy):
                    return True
        return False

    def _reconstruct(self, closed: Dict[DiscreteState, Node], last: DiscreteState) -> List[Pose2D]:
        out: List[Pose2D] = []
        cur = last
        while True:
            node = closed[cur]
            out.append(node.pose)
            if node.parent is None:
                break
            cur = node.parent
        out.reverse()
        return out


@dataclass
class PlanResult:
    path: List[Pose2D]
    planner_name: str
    elapsed_ms: float


@dataclass
class TemporaryBlockedPatch:
    x: float
    y: float
    radius_m: float
    ttl_steps: int


class BlockedPatchMemory:
    def __init__(self):
        self.patches: List[TemporaryBlockedPatch] = []
        self.version: int = 0

    def step(self) -> None:
        changed = False
        keep: List[TemporaryBlockedPatch] = []
        for p in self.patches:
            p.ttl_steps -= 1
            if p.ttl_steps > 0:
                keep.append(p)
            else:
                changed = True
        if changed or len(keep) != len(self.patches):
            self.version += 1
        self.patches = keep

    def add_patch(self, x: float, y: float, radius_m: float, ttl_steps: int) -> None:
        for p in self.patches:
            if math.hypot(p.x - x, p.y - y) <= max(p.radius_m, radius_m) * 0.8:
                p.x = 0.5 * (p.x + x)
                p.y = 0.5 * (p.y + y)
                p.radius_m = max(p.radius_m, radius_m)
                p.ttl_steps = max(p.ttl_steps, ttl_steps)
                self.version += 1
                return
        self.patches.append(TemporaryBlockedPatch(x, y, radius_m, ttl_steps))
        self.version += 1

    def rasterize(self, costmap: Costmap2D) -> None:
        for p in self.patches:
            costmap.mark_disk_world(p.x, p.y, p.radius_m)


@dataclass
class SectorSnapshot:
    front_m: float = float("inf")
    front_left_m: float = float("inf")
    front_right_m: float = float("inf")
    left_m: float = float("inf")
    right_m: float = float("inf")
    rear_m: float = float("inf")


class FastGridPlanner:
    def __init__(self, robot_cfg: RobotConfig, connectivity: int = 8):
        self.robot_cfg = robot_cfg
        self.connectivity = connectivity

    def plan(self, costmap: Costmap2D, start: Pose2D, goal: Pose2D) -> List[Pose2D]:
        start_g = costmap.world_to_grid(start.x, start.y)
        goal_g = costmap.world_to_grid(goal.x, goal.y)
        if costmap.is_occupied(*start_g) or costmap.is_occupied(*goal_g):
            return []

        if self.connectivity == 8:
            nbrs = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
        else:
            nbrs = [(0, -1), (-1, 0), (1, 0), (0, 1)]

        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_heap = []
        heapq.heappush(open_heap, (0.0, 0, start_g))
        came = {start_g: None}
        gscore = {start_g: 0.0}
        push_id = 0

        while open_heap:
            _, _, cur = heapq.heappop(open_heap)
            if cur == goal_g:
                raw = []
                k = cur
                while k is not None:
                    raw.append(k)
                    k = came[k]
                raw.reverse()
                return self._grid_path_to_pose_path(costmap, raw, start, goal)
            cx, cy = cur
            cg = gscore[cur]
            for dx, dy in nbrs:
                nx = cx + dx
                ny = cy + dy
                if not costmap.in_bounds(nx, ny) or costmap.is_occupied(nx, ny):
                    continue
                step = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                ng = cg + step
                nk = (nx, ny)
                if ng + 1e-9 < gscore.get(nk, float('inf')):
                    gscore[nk] = ng
                    came[nk] = cur
                    push_id += 1
                    heapq.heappush(open_heap, (ng + h(nk, goal_g), push_id, nk))
        return []

    def _grid_path_to_pose_path(self, costmap: Costmap2D, raw, start: Pose2D, goal: Pose2D) -> List[Pose2D]:
        if not raw:
            return []
        pts = [costmap.grid_to_world_center(gx, gy) for gx, gy in raw]
        stride_cells = max(1, int(round(0.16 / costmap.spec.resolution)))
        reduced = [pts[0]]
        if len(pts) >= 2:
            prev_dir = (int(math.copysign(1, raw[1][0] - raw[0][0])) if raw[1][0] != raw[0][0] else 0,
                        int(math.copysign(1, raw[1][1] - raw[0][1])) if raw[1][1] != raw[0][1] else 0)
        else:
            prev_dir = (0, 0)
        last_keep = raw[0]
        for i in range(1, len(raw) - 1):
            cur = raw[i]
            nxt = raw[i + 1]
            cur_dir = (
                int(math.copysign(1, nxt[0] - cur[0])) if nxt[0] != cur[0] else 0,
                int(math.copysign(1, nxt[1] - cur[1])) if nxt[1] != cur[1] else 0,
            )
            cheb = max(abs(cur[0] - last_keep[0]), abs(cur[1] - last_keep[1]))
            if cur_dir != prev_dir or cheb >= stride_cells:
                reduced.append(pts[i])
                last_keep = cur
                prev_dir = cur_dir
        if len(pts) > 1:
            reduced.append(pts[-1])

        out = [start]
        prev_x, prev_y = start.x, start.y
        prev_yaw = start.yaw
        for x, y in reduced[1:]:
            if math.hypot(x - prev_x, y - prev_y) > 1e-9:
                prev_yaw = math.atan2(y - prev_y, x - prev_x)
            out.append(Pose2D(x, y, prev_yaw))
            prev_x, prev_y = x, y
        if out:
            goal_yaw = out[-1].yaw if len(out) == 1 else math.atan2(goal.y - out[-1].y, goal.x - out[-1].x) if math.hypot(goal.x - out[-1].x, goal.y - out[-1].y) > 1e-9 else out[-1].yaw
            out.append(Pose2D(goal.x, goal.y, goal_yaw))
        return out


class LocalPlanner:
    def __init__(self, robot_cfg: RobotConfig, nav_cfg: NavConfig, collision_checker: HybridAStarPlanner):
        self.robot_cfg = robot_cfg
        self.nav_cfg = nav_cfg
        self.checker = collision_checker

    @staticmethod
    def build_sector_snapshot(hits_local: Sequence[Tuple[float, float]]) -> SectorSnapshot:
        snap = SectorSnapshot()
        for lx, ly in hits_local:
            d = math.hypot(lx, ly)
            if d < 1e-6:
                continue
            ang = math.degrees(math.atan2(ly, lx))
            if -25.0 <= ang <= 25.0:
                snap.front_m = min(snap.front_m, d)
            if 25.0 < ang <= 80.0:
                snap.front_left_m = min(snap.front_left_m, d)
            if -80.0 <= ang < -25.0:
                snap.front_right_m = min(snap.front_right_m, d)
            if 80.0 < ang <= 150.0:
                snap.left_m = min(snap.left_m, d)
            if -150.0 <= ang < -80.0:
                snap.right_m = min(snap.right_m, d)
            if ang > 150.0 or ang < -150.0:
                snap.rear_m = min(snap.rear_m, d)
        return snap

    def _simulate_cmd_end_pose(self, pose: Pose2D, cmd: ControlCommand) -> Pose2D:
        if cmd.mode == 'TURN_LEFT':
            return Pose2D(pose.x, pose.y, wrap_to_pi(pose.yaw + math.radians(abs(cmd.turn_deg))))
        if cmd.mode == 'TURN_RIGHT':
            return Pose2D(pose.x, pose.y, wrap_to_pi(pose.yaw - math.radians(abs(cmd.turn_deg))))
        if cmd.mode in {'FORWARD', 'BACKWARD'}:
            sign = 1.0 if cmd.mode == 'FORWARD' else -1.0
            ds = sign * abs(cmd.move_m)
            return Pose2D(pose.x + ds * math.cos(pose.yaw), pose.y + ds * math.sin(pose.yaw), pose.yaw)
        return pose

    def _safe_on_costmap(self, costmap: Costmap2D, pose: Pose2D, cmd: ControlCommand) -> bool:
        q = self._simulate_cmd_end_pose(pose, cmd)
        if cmd.mode in {'FORWARD', 'BACKWARD'}:
            return not self.checker._segment_in_collision(costmap, pose, q)
        if cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'}:
            return not self.checker._spin_in_collision(costmap, pose, q)
        return True

    def _safe_on_sectors(self, cmd: ControlCommand, sectors: SectorSnapshot) -> bool:
        if cmd.mode == 'BACKWARD' and not self.nav_cfg.allow_reverse:
            return False
        front_need = self.robot_cfg.length_m * 0.50 + self.nav_cfg.front_safety_margin_m + abs(cmd.move_m)
        rear_need = self.robot_cfg.length_m * 0.42 + self.nav_cfg.rear_safety_margin_m + abs(cmd.move_m)
        if cmd.mode == 'FORWARD' and sectors.front_m < front_need:
            return False
        if cmd.mode == 'BACKWARD' and sectors.rear_m < rear_need:
            return False
        return True

    def _corridor_forward_clear(self, sectors: SectorSnapshot) -> bool:
        front_clear = sectors.front_m > max(
            self.nav_cfg.local_corridor_front_clear_m,
            self.robot_cfg.length_m * 0.50 + self.nav_cfg.front_safety_margin_m + min(self.nav_cfg.forward_step_choices_m),
        )
        side_clear = min(sectors.front_left_m, sectors.front_right_m) > self.nav_cfg.local_corridor_side_clear_m
        return bool(front_clear and side_clear)

    def _score(self, pose: Pose2D, target: Pose2D, goal: Pose2D, cmd: ControlCommand, sectors: SectorSnapshot, prev_cmd: ControlCommand) -> float:
        end = self._simulate_cmd_end_pose(pose, cmd)
        target_before = math.hypot(target.x - pose.x, target.y - pose.y)
        target_after = math.hypot(target.x - end.x, target.y - end.y)
        goal_after = math.hypot(goal.x - end.x, goal.y - end.y)
        desired = math.atan2(target.y - end.y, target.x - end.x)
        yaw_ref = wrap_to_pi(end.yaw + math.pi) if cmd.mode == 'BACKWARD' else end.yaw
        heading_err = abs(wrap_to_pi(desired - yaw_ref))
        score = self.nav_cfg.local_goal_progress_weight * (target_before - target_after)
        score -= self.nav_cfg.local_heading_weight * heading_err
        score -= 0.04 * goal_after
        if cmd.mode == 'FORWARD':
            score += 0.22 + 0.12 * min(max(sectors.front_m - 0.30, 0.0), 1.0)
        elif cmd.mode == 'BACKWARD':
            score -= self.nav_cfg.local_reverse_penalty
            score += 0.05 * min(max(sectors.rear_m - 0.20, 0.0), 1.0)
        else:
            score -= self.nav_cfg.local_turn_penalty * (abs(cmd.turn_deg) / max(1.0, self.nav_cfg.max_turn_cmd_deg))
            if cmd.mode == 'TURN_LEFT':
                score += 0.10 * min(sectors.left_m, 1.5) + 0.07 * min(sectors.front_left_m, 1.5)
            else:
                score += 0.10 * min(sectors.right_m, 1.5) + 0.07 * min(sectors.front_right_m, 1.5)
        if self._corridor_forward_clear(sectors):
            if cmd.mode == 'FORWARD':
                score += self.nav_cfg.local_corridor_forward_bonus
            elif cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'}:
                score -= self.nav_cfg.local_corridor_turn_penalty
        if prev_cmd.mode and prev_cmd.mode != 'STOP':
            if prev_cmd.mode != cmd.mode:
                score -= 0.05
            if prev_cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'} and cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'} and prev_cmd.mode != cmd.mode:
                score -= self.nav_cfg.local_turn_switch_penalty
        return score

    def _forward_candidates(self, goal_dist: float, sectors: SectorSnapshot) -> List[ControlCommand]:
        steps = list(self.nav_cfg.forward_step_choices_m)
        if not steps:
            return []

        profiles: List[Tuple[float, float, str]] = []
        profiles.append((steps[0], 0.30, 'local forward slow'))

        medium_clearance = self.robot_cfg.length_m * 0.55 + 0.30
        fast_clearance = self.robot_cfg.length_m * 0.55 + 0.70
        if len(steps) >= 2 and sectors.front_m > medium_clearance and goal_dist > 0.30:
            profiles.append((steps[1], 0.38, 'local forward medium'))
        if len(steps) >= 3 and sectors.front_m > fast_clearance and goal_dist > 0.60:
            profiles.append((steps[2], 0.48, 'local forward fast'))

        return [
            ControlCommand('FORWARD', move_m=move_m, raw_left=raw, raw_right=raw, reason=reason)
            for move_m, raw, reason in profiles
        ]

    def choose_command(
        self,
        pose: Pose2D,
        target: Pose2D,
        goal: Pose2D,
        costmap: Costmap2D,
        sectors: SectorSnapshot,
        prev_cmd: ControlCommand,
    ) -> ControlCommand:
        goal_dist = math.hypot(goal.x - pose.x, goal.y - pose.y)
        if goal_dist <= self.nav_cfg.goal_tol_m:
            return ControlCommand('STOP', reason='goal reached')

        desired = math.atan2(target.y - pose.y, target.x - pose.x)
        yaw_err = wrap_to_pi(desired - pose.yaw)
        reverse_yaw_err = wrap_to_pi(desired - wrap_to_pi(pose.yaw + math.pi))
        corridor_clear = self._corridor_forward_clear(sectors)

        candidates: List[ControlCommand] = []
        for deg in self.nav_cfg.turn_step_choices_deg:
            candidates.append(ControlCommand('TURN_LEFT', turn_deg=deg, raw_left=-0.30, raw_right=0.30, reason='local turn left'))
            candidates.append(ControlCommand('TURN_RIGHT', turn_deg=deg, raw_left=0.30, raw_right=-0.30, reason='local turn right'))
        candidates.extend(self._forward_candidates(goal_dist, sectors))

        best_score = -1e18
        best_cmd: Optional[ControlCommand] = None
        for cmd in candidates:
            forward_yaw_limit_deg = 68.0 if corridor_clear else 32.0
            if cmd.mode == 'FORWARD' and abs(yaw_err) > math.radians(forward_yaw_limit_deg):
                continue
            if cmd.mode in {'FORWARD', 'BACKWARD'} and not self._safe_on_sectors(cmd, sectors):
                continue
            if not self._safe_on_costmap(costmap, pose, cmd):
                continue
            score = self._score(pose, target, goal, cmd, sectors, prev_cmd)
            if cmd.mode == 'FORWARD' and abs(yaw_err) <= math.radians(self.nav_cfg.turn_threshold_deg):
                score += 0.25
            if cmd.mode == 'FORWARD' and abs(yaw_err) > math.radians(20.0):
                score -= 0.55 * min(1.0, abs(yaw_err) / math.radians(90.0))
            if cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'} and abs(yaw_err) > math.radians(20.0):
                score += 0.18
            if cmd.mode == 'BACKWARD' and abs(reverse_yaw_err) <= math.radians(self.nav_cfg.turn_threshold_deg + 4.0):
                score += 0.12
            if cmd.mode == 'BACKWARD' and abs(yaw_err) < math.radians(85.0):
                score -= 0.45
            if cmd.mode == 'TURN_LEFT' and yaw_err > 0.0:
                score += 0.16
            if cmd.mode == 'TURN_RIGHT' and yaw_err < 0.0:
                score += 0.16
            if cmd.mode == 'FORWARD' and sectors.front_m < self.robot_cfg.length_m * 0.48 + 0.10:
                score -= 0.55
            if cmd.mode == 'BACKWARD' and sectors.rear_m < self.robot_cfg.length_m * 0.40 + 0.10:
                score -= 0.45
            if score > best_score:
                best_score = score
                best_cmd = cmd

        if best_cmd is not None:
            if best_cmd.mode == 'FORWARD':
                steer_ratio = clamp(yaw_err / math.radians(28.0), -1.0, 1.0)
                if abs(steer_ratio) >= 0.18:
                    base = max(abs(best_cmd.raw_left), abs(best_cmd.raw_right))
                    inner = max(0.22, base * (1.0 - 0.55 * abs(steer_ratio)))
                    outer = min(0.55, base * (1.0 + 0.18 * abs(steer_ratio)))
                    if steer_ratio > 0.0:
                        best_cmd.raw_left = inner
                        best_cmd.raw_right = outer
                        best_cmd.reason = 'local forward arc left'
                    else:
                        best_cmd.raw_left = outer
                        best_cmd.raw_right = inner
                        best_cmd.reason = 'local forward arc right'
            return best_cmd

        recovery_turn_raw = clamp(self.nav_cfg.recovery_turn_raw, 0.0, 1.0)
        if sectors.left_m + sectors.front_left_m >= sectors.right_m + sectors.front_right_m:
            cmd = ControlCommand('TURN_LEFT', turn_deg=28.0, raw_left=-recovery_turn_raw, raw_right=recovery_turn_raw, reason='local escape turn')
            if self._safe_on_costmap(costmap, pose, cmd):
                return cmd
        else:
            cmd = ControlCommand('TURN_RIGHT', turn_deg=28.0, raw_left=recovery_turn_raw, raw_right=-recovery_turn_raw, reason='local escape turn')
            if self._safe_on_costmap(costmap, pose, cmd):
                return cmd
        if self.nav_cfg.allow_reverse and sectors.rear_m > self.robot_cfg.length_m * 0.45 + 0.08:
            return ControlCommand('BACKWARD', move_m=min(self.nav_cfg.backward_step_choices_m), raw_left=-0.34, raw_right=-0.34, reason='local emergency reverse')
        if self.nav_cfg.nonstop_when_blocked:
            return self._nonstop_blocked_escape(sectors, prev_cmd)
        return ControlCommand('STOP', reason='local no safe motion')

    def _nonstop_blocked_escape(self, sectors: SectorSnapshot, prev_cmd: ControlCommand) -> ControlCommand:
        recovery_turn_raw = clamp(self.nav_cfg.recovery_turn_raw, 0.0, 1.0)
        if self.nav_cfg.allow_reverse and sectors.rear_m > self.robot_cfg.length_m * 0.25:
            return ControlCommand(
                'BACKWARD',
                move_m=min(self.nav_cfg.backward_step_choices_m),
                raw_left=-0.30,
                raw_right=-0.30,
                reason='local no-stop reverse; no fully safe motion',
            )
        left_score = min(sectors.left_m, 2.0) + min(sectors.front_left_m, 2.0)
        right_score = min(sectors.right_m, 2.0) + min(sectors.front_right_m, 2.0)
        if prev_cmd.mode == 'TURN_LEFT':
            left_score += 0.55
        elif prev_cmd.mode == 'TURN_RIGHT':
            right_score += 0.55

        if left_score >= right_score:
            return ControlCommand(
                'TURN_LEFT',
                turn_deg=28.0,
                raw_left=-recovery_turn_raw,
                raw_right=recovery_turn_raw,
                reason='local no-stop turn left; no fully safe motion',
            )
        return ControlCommand(
            'TURN_RIGHT',
            turn_deg=28.0,
            raw_left=recovery_turn_raw,
            raw_right=-recovery_turn_raw,
            reason='local no-stop turn right; no fully safe motion',
        )


# =========================================================
# Virtual world
# =========================================================

class VirtualWorld:
    def __init__(self, sim_cfg: SimConfig, nav_cfg: NavConfig):
        self.sim_cfg = sim_cfg
        self.nav_cfg = nav_cfg
        self.static_obstacles = self._make_obstacles()
        spec = GridSpec(
            resolution=nav_cfg.map_resolution_m,
            origin_x=0.0,
            origin_y=0.0,
            width=int(math.ceil(sim_cfg.field_w_m / nav_cfg.map_resolution_m)),
            height=int(math.ceil(sim_cfg.field_h_m / nav_cfg.map_resolution_m)),
        )
        data = np.zeros((spec.height, spec.width), dtype=np.uint8)
        self.true_costmap = Costmap2D(spec, data)
        for obs in self.static_obstacles:
            self.rasterize_obstacle(self.true_costmap, obs, 0.0)

    def _make_obstacles(self) -> List[Obstacle]:
        return [
            Obstacle("box",   yd(3.5),  yd(3.4),  ft(1.9),  ft(1.3)),
            Obstacle("box",   yd(5.0),  yd(11.1), ft(2.0),  ft(1.3)),
            Obstacle("box",   yd(11.8), yd(10.8), ft(2.0),  ft(1.3)),
            Obstacle("chair", yd(7.4),  yd(7.0),  ft(1.45), ft(1.45)),
            Obstacle("chair", yd(10.6), yd(3.7),  ft(1.4),  ft(1.4)),
            Obstacle("cone",  yd(2.6),  yd(8.2),  ft(0.85), ft(0.85)),
            Obstacle("cone",  yd(4.4),  yd(9.1),  ft(0.85), ft(0.85)),
            Obstacle("cone",  yd(8.7),  yd(9.8),  ft(0.85), ft(0.85)),
            Obstacle("cone",  yd(11.0), yd(8.8),  ft(0.85), ft(0.85)),
        ]

    @staticmethod
    def rasterize_obstacle(costmap: Costmap2D, obs: Obstacle, inflation_m: float) -> None:
        if obs.kind == "cone":
            radius = obs.w / 2.0 + inflation_m
            x_min, x_max = obs.x - radius, obs.x + radius
            y_min, y_max = obs.y - radius, obs.y + radius
        else:
            x_min = obs.x - obs.w / 2.0 - inflation_m
            x_max = obs.x + obs.w / 2.0 + inflation_m
            y_min = obs.y - obs.h / 2.0 - inflation_m
            y_max = obs.y + obs.h / 2.0 + inflation_m

        gx0, gy0 = costmap.world_to_grid(x_min, y_min)
        gx1, gy1 = costmap.world_to_grid(x_max, y_max)
        gx0 = max(0, gx0 - 1)
        gy0 = max(0, gy0 - 1)
        gx1 = min(costmap.spec.width - 1, gx1 + 1)
        gy1 = min(costmap.spec.height - 1, gy1 + 1)

        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                cx, cy = costmap.grid_to_world_center(gx, gy)
                if obs.kind == "cone":
                    if (cx - obs.x) ** 2 + (cy - obs.y) ** 2 <= radius ** 2:
                        costmap.data[gy, gx] = 1
                else:
                    if abs(cx - obs.x) <= obs.w / 2.0 + inflation_m and abs(cy - obs.y) <= obs.h / 2.0 + inflation_m:
                        costmap.data[gy, gx] = 1

    def ray_cast(self, pose: Pose2D, world_angle: float, max_range_m: float, step_m: float = 0.03) -> float:
        d = 0.0
        while d <= max_range_m:
            x = pose.x + d * math.cos(world_angle)
            y = pose.y + d * math.sin(world_angle)
            if x < 0.0 or x > self.sim_cfg.field_w_m or y < 0.0 or y > self.sim_cfg.field_h_m:
                return d
            if self.true_costmap.is_occupied_world(x, y):
                return d
            d += step_m
        return max_range_m

    def segment_collides(self, a: Pose2D, b: Pose2D, planner: HybridAStarPlanner) -> bool:
        return planner._segment_in_collision(self.true_costmap, a, b)


# =========================================================
# Simulated sensors and robot
# =========================================================

@dataclass
class SimRobotState:
    true_pose: Pose2D
    left_ticks_total: int = 0
    right_ticks_total: int = 0


class SimulatedRobot:
    def __init__(self, robot_cfg: RobotConfig, init_pose: Pose2D):
        self.robot_cfg = robot_cfg
        self.state = SimRobotState(init_pose)

    def _meters_to_ticks(self, meters: float) -> int:
        revs = meters / (2.0 * math.pi * self.robot_cfg.wheel_radius_m)
        return int(round(revs * self.robot_cfg.ticks_per_rev))

    def apply_command(self, cmd: ControlCommand, world: VirtualWorld, planner: HybridAStarPlanner, duration_s: float = 0.10) -> None:
        pose = self.state.true_pose
        new_pose = pose
        left_move = 0.0
        right_move = 0.0

        if cmd.controller == "velocity":
            v = float(cmd.v_mps)
            omega = float(cmd.omega_radps)
            dt = max(0.0, float(duration_s))
            if abs(v) > 1e-6 or abs(omega) > 1e-6:
                mid = pose.yaw + 0.5 * omega * dt
                new_pose = Pose2D(
                    pose.x + v * math.cos(mid) * dt,
                    pose.y + v * math.sin(mid) * dt,
                    wrap_to_pi(pose.yaw + omega * dt),
                )
                if world.segment_collides(pose, new_pose, planner):
                    new_pose = pose
                else:
                    left_move = (v - 0.5 * omega * self.robot_cfg.track_width_m) * dt
                    right_move = (v + 0.5 * omega * self.robot_cfg.track_width_m) * dt
        elif cmd.mode == "TURN_LEFT":
            ang = math.radians(abs(cmd.turn_deg))
            new_pose = Pose2D(pose.x, pose.y, wrap_to_pi(pose.yaw + ang))
            arc = ang * self.robot_cfg.track_width_m / 2.0
            left_move = -arc
            right_move = arc
        elif cmd.mode == "TURN_RIGHT":
            ang = math.radians(abs(cmd.turn_deg))
            new_pose = Pose2D(pose.x, pose.y, wrap_to_pi(pose.yaw - ang))
            arc = ang * self.robot_cfg.track_width_m / 2.0
            left_move = arc
            right_move = -arc
        elif cmd.mode in {"FORWARD", "BACKWARD"}:
            sign = 1.0 if cmd.mode == "FORWARD" else -1.0
            ds = sign * abs(cmd.move_m)
            new_pose = Pose2D(
                pose.x + ds * math.cos(pose.yaw),
                pose.y + ds * math.sin(pose.yaw),
                pose.yaw,
            )
            if world.segment_collides(pose, new_pose, planner):
                new_pose = pose
                left_move = 0.0
                right_move = 0.0
            else:
                left_move = ds
                right_move = ds

        self.state.true_pose = new_pose
        self.state.left_ticks_total += self._meters_to_ticks(left_move)
        self.state.right_ticks_total += self._meters_to_ticks(right_move)


class SimulatedSensors:
    def __init__(self, robot_cfg: RobotConfig, sensor_cfg: SensorConfig, world: VirtualWorld):
        self.robot_cfg = robot_cfg
        self.sensor_cfg = sensor_cfg
        self.world = world

    def _obstacle_sample_points(self, obs: Obstacle) -> List[Tuple[float, float]]:
        if obs.kind == "cone":
            r = obs.w / 2.0
            return [
                (obs.x, obs.y),
                (obs.x + 0.7 * r, obs.y),
                (obs.x - 0.7 * r, obs.y),
                (obs.x, obs.y + 0.7 * r),
                (obs.x, obs.y - 0.7 * r),
            ]
        hw = obs.w / 2.0
        hh = obs.h / 2.0
        return [
            (obs.x, obs.y),
            (obs.x - hw, obs.y),
            (obs.x + hw, obs.y),
            (obs.x, obs.y - hh),
            (obs.x, obs.y + hh),
            (obs.x - hw, obs.y - hh),
            (obs.x - hw, obs.y + hh),
            (obs.x + hw, obs.y - hh),
            (obs.x + hw, obs.y + hh),
        ]

    def _line_of_sight(self, pose: Pose2D, tx: float, ty: float) -> bool:
        dist = math.hypot(tx - pose.x, ty - pose.y)
        n = max(2, int(dist / self.sensor_cfg.zed_los_step_m))
        tail_skip = max(2, int(0.20 / self.sensor_cfg.zed_los_step_m))
        for i in range(1, max(1, n - tail_skip)):
            t = i / n
            x = pose.x + t * (tx - pose.x)
            y = pose.y + t * (ty - pose.y)
            if self.world.true_costmap.is_occupied_world(x, y):
                return False
        return True

    def read(self, robot_state: SimRobotState, goal_xy: Tuple[float, float], timestamp: float) -> SensorFrame:
        pose = robot_state.true_pose

        angles = np.linspace(-math.pi, math.pi, self.sensor_cfg.lidar_num_beams)
        lidar_ranges: List[float] = []
        lidar_hits_local: List[Tuple[float, float]] = []
        for rel in angles:
            world_angle = pose.yaw + rel
            r = self.world.ray_cast(pose, world_angle, self.sensor_cfg.lidar_range_m)
            lidar_ranges.append(r)
            if r < self.sensor_cfg.lidar_range_m * 0.995:
                lx = r * math.cos(rel)
                ly = r * math.sin(rel)
                lidar_hits_local.append((lx, ly))

        zed_hits_local: List[Tuple[float, float]] = []
        half = math.radians(self.sensor_cfg.zed_fov_deg * 0.5)
        for obs in self.world.static_obstacles:
            for sx, sy in self._obstacle_sample_points(obs):
                dx = sx - pose.x
                dy = sy - pose.y
                dist = math.hypot(dx, dy)
                if dist > self.sensor_cfg.zed_range_m:
                    continue
                rel = wrap_to_pi(math.atan2(dy, dx) - pose.yaw)
                if abs(rel) > half:
                    continue
                if self._line_of_sight(pose, sx, sy):
                    lx = dx * math.cos(-pose.yaw) - dy * math.sin(-pose.yaw)
                    ly = dx * math.sin(-pose.yaw) + dy * math.cos(-pose.yaw)
                    zed_hits_local.append((lx, ly))

        return SensorFrame(
            encoder=EncoderPacket(robot_state.left_ticks_total, robot_state.right_ticks_total, timestamp),
            lidar=LidarPacket(lidar_hits_local, lidar_ranges, angles.tolist(), timestamp),
            zed=ZedPacket(zed_hits_local, timestamp),
            goal=GoalPacket(goal_xy[0], goal_xy[1], timestamp),
            imu=ImuPacket((0.0, 0.0, 0.0), (0.0, 0.0, 9.81), timestamp),
        )


# =========================================================
# Real robot bridge
# =========================================================

class RealRobotBridgeBase:
    def read_frame(self) -> Optional[SensorFrame]:
        raise NotImplementedError

    def send_control(self, cmd: ControlCommand) -> None:
        raise NotImplementedError

    def publish_nav_status(self, status: dict) -> None:
        del status

    def send_uav_flag(self, flag: dict) -> None:
        del flag


class JsonReplayBridge(RealRobotBridgeBase):
    """
    Handy fallback when ROS2 is not available.
    Expects one JSON object per line with keys:
    left_ticks, right_ticks, lidar_hits_local, zed_hits_local, goal_x, goal_y, timestamp
    """
    def __init__(self, path: str):
        self.fp = open(path, "r", encoding="utf-8-sig")

    def read_frame(self) -> Optional[SensorFrame]:
        line = self.fp.readline()
        if not line:
            return None
        obj = json.loads(line.lstrip("\ufeff"))
        ts = float(obj.get("timestamp", time.time()))
        lidar_hits = [tuple(map(float, p)) for p in obj.get("lidar_hits_local", [])]
        lidar_ranges = [float(v) for v in obj.get("lidar_ranges_m", obj.get("ranges_m", []))]
        lidar_angles = [float(v) for v in obj.get("lidar_angles_rad", obj.get("angles_rad", []))]
        if len(lidar_ranges) != len(lidar_angles):
            lidar_ranges = []
            lidar_angles = []
        if not lidar_ranges:
            lidar_ranges = [math.hypot(p[0], p[1]) for p in lidar_hits]
            lidar_angles = [math.atan2(p[1], p[0]) for p in lidar_hits]
        zed_hits = [tuple(map(float, p)) for p in obj.get("zed_hits_local", [])]
        imu = ImuPacket(
            tuple(map(float, obj.get("imu_angular_velocity_rps", [0.0, 0.0, 0.0]))),
            tuple(map(float, obj.get("imu_linear_accel_mps2", [0.0, 0.0, 9.81]))),
            ts,
        )
        field_map = None
        if "field_map" in obj:
            field_map = field_map_from_json(json.dumps(obj["field_map"]), ts, int(obj.get("field_map_version", 1)))
        mission_flag = parse_mission_flag(json.dumps(obj["mission_flag"]), ts) if "mission_flag" in obj else None
        return SensorFrame(
            encoder=EncoderPacket(int(obj["left_ticks"]), int(obj["right_ticks"]), ts),
            lidar=LidarPacket(lidar_hits, lidar_ranges, lidar_angles, ts),
            zed=ZedPacket(zed_hits, ts),
            goal=GoalPacket(float(obj.get("goal_x", float("nan"))), float(obj.get("goal_y", float("nan"))), ts),
            imu=imu,
            field_map=field_map,
            mission_flag=mission_flag,
        )

    def send_control(self, cmd: ControlCommand) -> None:
        print("REAL CMD", json.dumps(cmd.as_dict()))

    def publish_nav_status(self, status: dict) -> None:
        print("NAV STATUS", json.dumps(status))

    def send_uav_flag(self, flag: dict) -> None:
        print("UAV FLAG", json.dumps(flag))


class Ros2Bridge(RealRobotBridgeBase):
    def __init__(
        self,
        allow_missing_goal: bool = False,
        field_map_topic: str = "/ugv/field_map",
        target_topic: str = "/ugv/target",
        marker_topic: str = "/ugv/marker_detection",
        mission_flag_topic: str = "/ugv/mission_flag",
        motor_status_topic: str = "/motor_controller/status",
        nav_status_topic: str = "/ugv_nav_status",
        uav_flag_topic: str = "/ugv/uav_flag",
    ):
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import PointStamped
            from std_msgs.msg import String
            from ugv_sensor_sync.msg import NavSensorFrame
        except Exception as e:
            raise RuntimeError(
                "ROS2 bridge needs rclpy, geometry_msgs, std_msgs, and the built ugv_sensor_sync "
                "message package available in the sourced workspace. "
                "If you are not on the Jetson yet, use --replay-json instead."
            ) from e

        self._rclpy = rclpy
        self._String = String
        self._NavSensorFrame = NavSensorFrame
        self.allow_missing_goal = bool(allow_missing_goal)
        self.field_map_topic = field_map_topic
        self.target_topic = target_topic
        self.marker_topic = marker_topic
        self.mission_flag_topic = mission_flag_topic
        self.motor_status_topic = motor_status_topic
        self.nav_status_topic = nav_status_topic
        self.uav_flag_topic = uav_flag_topic
        self._latest_synced: Optional[
            Tuple[
                int,
                int,
                List[Tuple[float, float]],
                List[float],
                List[float],
                List[Tuple[float, float]],
                ImuPacket,
                float,
            ]
        ] = None
        self._latest_goal: Optional[Tuple[float, float, float]] = None
        self._latest_field_map: Optional[FieldMapPacket] = None
        self._latest_marker_goal: Optional[GoalPacket] = None
        self._latest_mission_flag: Optional[MissionFlagPacket] = None
        self.latest_motor_status: Dict[str, Any] = {}
        self._latest_synced_seq = 0
        self._goal_version = 0
        self._field_map_version = 0
        self._field_map_key = None
        self._marker_version = 0
        self._mission_flag_version = 0
        self._last_returned_signature: Tuple[int, int, int, int, int] = (-1, -1, -1, -1, -1)
        self.nav_frame_timeout_s = 0.75
        self._last_printed_cmd = ""
        self._last_printed_cmd_s = 0.0

        class BridgeNode(Node):
            pass

        rclpy.init(args=None)
        self.node = BridgeNode("ugv_nav_bridge")
        self.node.create_subscription(NavSensorFrame, "/sensors/nav_frame", self._synced_cb, 10)
        self.node.create_subscription(PointStamped, "/ugv_goal", self._goal_cb, 10)
        self.node.create_subscription(String, self.field_map_topic, self._field_map_cb, 10)
        if self.target_topic != self.field_map_topic:
            self.node.create_subscription(String, self.target_topic, self._field_map_cb, 10)
        self.node.create_subscription(PointStamped, self.marker_topic, self._marker_cb, 10)
        self.node.create_subscription(String, self.mission_flag_topic, self._mission_flag_cb, 10)
        self.node.create_subscription(String, self.motor_status_topic, self._motor_status_cb, 10)
        self.pub = self.node.create_publisher(String, "/ugv_nav_cmd", 10)
        self.status_pub = self.node.create_publisher(String, self.nav_status_topic, 10)
        self.uav_flag_pub = self.node.create_publisher(String, self.uav_flag_topic, 10)
        self.node.get_logger().info(
            "UGV nav bridge ready "
            f"(allow_missing_goal={self.allow_missing_goal}, field_map={self.field_map_topic}, "
            f"target={self.target_topic}, marker={self.marker_topic}, mission_flag={self.mission_flag_topic}, "
            f"motor_status={self.motor_status_topic}, nav_status={self.nav_status_topic}, "
            f"uav_flag={self.uav_flag_topic})"
        )

    def _synced_cb(self, msg):
        if not msg.encoder_available:
            return
        hits, ranges_m, angles_rad = laser_scan_to_lidar_observations(
            msg.scan.ranges,
            float(msg.scan.angle_min),
            float(msg.scan.angle_increment),
            float(msg.scan.range_min),
            float(msg.scan.range_max),
        )

        zed_hits = []
        for p in msg.zed_obstacle_points.poses:
            zed_hits.append((float(p.position.x), float(p.position.y)))
        if bool(msg.near_obstacle):
            protective_x = float(msg.front_clearance_m) if math.isfinite(float(msg.front_clearance_m)) else 0.12
            protective_x = clamp(protective_x, 0.06, 0.30)
            for offset_y in (-0.16, 0.0, 0.16):
                zed_hits.append((protective_x, offset_y))

        ts = self._stamp_to_seconds(msg.header.stamp)
        imu_msg = getattr(msg, "imu", None)
        if imu_msg is None:
            imu_packet = ImuPacket((0.0, 0.0, 0.0), (0.0, 0.0, 9.81), ts)
        else:
            imu_packet = ImuPacket(
                (
                    float(imu_msg.angular_velocity.x),
                    float(imu_msg.angular_velocity.y),
                    float(imu_msg.angular_velocity.z),
                ),
                (
                    float(imu_msg.linear_acceleration.x),
                    float(imu_msg.linear_acceleration.y),
                    float(imu_msg.linear_acceleration.z),
                ),
                ts,
            )
        self._latest_synced = (
            int(msg.left_encoder_ticks),
            int(msg.right_encoder_ticks),
            hits,
            ranges_m,
            angles_rad,
            zed_hits,
            imu_packet,
            ts,
        )
        self._latest_synced_seq += 1

    def _goal_cb(self, msg):
        stamp_now = self.node.get_clock().now().to_msg()
        self._latest_goal = (
            float(msg.point.x),
            float(msg.point.y),
            self._stamp_to_seconds(stamp_now),
        )
        self._goal_version += 1

    def _field_map_cb(self, msg):
        now_s = self._stamp_to_seconds(self.node.get_clock().now().to_msg())
        next_version = self._field_map_version + 1
        try:
            packet = field_map_from_json(msg.data, now_s, next_version)
        except Exception as exc:
            self.node.get_logger().warn(f"Rejected target/map JSON: {exc}")
            return
        packet_key = field_map_content_key(packet)
        if packet_key == self._field_map_key:
            return
        self._latest_field_map = packet
        self._field_map_version = next_version
        self._field_map_key = packet_key
        self.node.get_logger().info(
            "Received target/map "
            f"v{packet.version}: size={packet.size}, obstacles={len(packet.obstacle_cells)}, "
            f"start={packet.start_xy}, goal={packet.goal_xy}, source={packet.source}"
        )
        if packet.goal_xy is not None:
            self.send_uav_flag({
                "target_found": True,
                "source": packet.source,
                "target_m": [round(packet.goal_xy[0], 3), round(packet.goal_xy[1], 3)],
                "stamp": round(now_s, 3),
            })

    def _marker_cb(self, msg):
        stamp_now = self.node.get_clock().now().to_msg()
        ts = self._stamp_to_seconds(stamp_now)
        distance_m = float(msg.point.z) if math.isfinite(float(msg.point.z)) and float(msg.point.z) > 0.0 else None
        self._latest_marker_goal = GoalPacket(float(msg.point.x), float(msg.point.y), ts, distance_m)
        self._marker_version += 1
        flag = {
            "target_found": True,
            "source": "zed_marker_detection",
            "target_m": [round(float(msg.point.x), 3), round(float(msg.point.y), 3)],
            "stamp": round(ts, 3),
        }
        if distance_m is not None:
            flag["distance_m"] = round(distance_m, 3)
        self.send_uav_flag(flag)

    def _mission_flag_cb(self, msg):
        now_s = self._stamp_to_seconds(self.node.get_clock().now().to_msg())
        self._latest_mission_flag = parse_mission_flag(msg.data, now_s)
        self._mission_flag_version += 1
        self.node.get_logger().info(
            f"Received mission flag {self._latest_mission_flag.state} "
            f"from {self._latest_mission_flag.source}"
        )

    def _motor_status_cb(self, msg):
        try:
            parsed = json.loads(msg.data)
        except Exception:
            return
        if isinstance(parsed, dict):
            self.latest_motor_status = parsed

    def read_frame(self) -> Optional[SensorFrame]:
        self._rclpy.spin_once(self.node, timeout_sec=0.02)
        if self._latest_synced is None:
            return None
        if (
            not self.allow_missing_goal
            and self._latest_goal is None
            and (self._latest_field_map is None or self._latest_field_map.goal_xy is None)
            and self._latest_marker_goal is None
        ):
            return None
        signature = (
            self._latest_synced_seq,
            self._goal_version,
            self._field_map_version,
            self._marker_version,
            self._mission_flag_version,
        )
        if signature == self._last_returned_signature:
            return None
        left, right, scan_hits, ranges_m, angles_rad, zed_hits, imu_packet, ts_s = self._latest_synced
        now_s = self._stamp_to_seconds(self.node.get_clock().now().to_msg())
        if now_s - ts_s > self.nav_frame_timeout_s:
            return None

        goal_packet = GoalPacket(float("nan"), float("nan"), ts_s)
        if self._latest_marker_goal is not None:
            goal_packet = self._latest_marker_goal
        elif self._latest_field_map is not None and self._latest_field_map.goal_xy is not None:
            gx, gy = self._latest_field_map.goal_xy
            goal_packet = GoalPacket(gx, gy, self._latest_field_map.timestamp)
        elif self._latest_goal is not None:
            gx, gy, ts_g = self._latest_goal
            goal_packet = GoalPacket(gx, gy, ts_g)

        ts = max(ts_s, goal_packet.timestamp)
        self._last_returned_signature = signature
        return SensorFrame(
            encoder=EncoderPacket(left, right, ts),
            lidar=LidarPacket(scan_hits, ranges_m, angles_rad, ts),
            zed=ZedPacket(zed_hits, ts),
            goal=GoalPacket(goal_packet.x, goal_packet.y, ts, goal_packet.distance_m),
            imu=imu_packet,
            field_map=self._latest_field_map,
            marker_goal=self._latest_marker_goal,
            mission_flag=self._latest_mission_flag,
        )

    def send_control(self, cmd: ControlCommand) -> None:
        msg = self._String()
        msg.data = json.dumps(cmd.as_dict())
        self.pub.publish(msg)
        now_s = self._stamp_to_seconds(self.node.get_clock().now().to_msg())
        if msg.data != self._last_printed_cmd or now_s - self._last_printed_cmd_s >= 1.0:
            print("REAL CMD", msg.data)
            self._last_printed_cmd = msg.data
            self._last_printed_cmd_s = now_s

    def publish_nav_status(self, status: dict) -> None:
        msg = self._String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def send_uav_flag(self, flag: dict) -> None:
        msg = self._String()
        msg.data = json.dumps(flag)
        self.uav_flag_pub.publish(msg)
        print("UAV FLAG", msg.data)

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9


# =========================================================
# Navigator core
# =========================================================

@dataclass
class NavigatorState:
    estimated_pose: Pose2D
    goal_pose: Pose2D
    path: List[Pose2D] = field(default_factory=list)
    path_idx: int = 0
    replans: int = 0
    discovered_points: int = 0
    latest_cmd: ControlCommand = field(default_factory=lambda: ControlCommand("STOP"))
    planner_name: str = "none"
    plan_time_ms: float = 0.0
    finish_reason: str = "running"
    sectors: SectorSnapshot = field(default_factory=SectorSnapshot)


class UGVNavigator:
    def __init__(
        self,
        robot_cfg: RobotConfig,
        sensor_cfg: SensorConfig,
        nav_cfg: NavConfig,
        field_w_m: float,
        field_h_m: float,
        init_pose: Pose2D,
        init_goal: Pose2D,
    ):
        self.robot_cfg = robot_cfg
        self.sensor_cfg = sensor_cfg
        self.nav_cfg = nav_cfg
        spec = GridSpec(
            resolution=nav_cfg.map_resolution_m,
            origin_x=0.0,
            origin_y=0.0,
            width=int(math.ceil(field_w_m / nav_cfg.map_resolution_m)),
            height=int(math.ceil(field_h_m / nav_cfg.map_resolution_m)),
        )
        self.known_costmap = Costmap2D(spec, np.zeros((spec.height, spec.width), dtype=np.uint8))
        self.static_costmap = Costmap2D(spec, np.zeros((spec.height, spec.width), dtype=np.uint8))
        local_cfg = LocalCostmapConfig(
            resolution_m=max(0.02, float(nav_cfg.local_costmap_resolution_m)),
            width_m=max(1.0, float(nav_cfg.local_costmap_width_m)),
            height_m=max(1.0, float(nav_cfg.local_costmap_height_m)),
            dynamic_decay_s=max(0.0, float(nav_cfg.local_costmap_dynamic_decay_s)),
            obstacle_radius_m=max(0.01, float(nav_cfg.local_costmap_obstacle_radius_m)),
            inflation_radius_m=max(0.0, float(nav_cfg.local_costmap_inflation_m)),
            lidar_clear_radius_m=max(0.0, float(nav_cfg.local_costmap_lidar_clear_radius_m)),
            robot_radius_m=max(
                0.65,
                0.5 * math.hypot(robot_cfg.length_m, robot_cfg.width_m)
                + robot_cfg.obstacle_buffer_m
                + 0.12,
            ),
            max_obstacle_range_m=max(0.2, float(nav_cfg.local_costmap_max_raytrace_m)),
            max_raytrace_range_m=max(0.2, float(nav_cfg.local_costmap_max_raytrace_m)),
        )
        self.local_costmap = RollingLocalCostmap(local_cfg)
        self.local_costmap_debug: Dict[str, Any] = self.local_costmap.stats.as_dict()
        self.fast_planner = FastGridPlanner(robot_cfg)
        self.hybrid_fallback = HybridAStarPlanner(
            robot_cfg,
            yaw_bins=48,
            step_size=0.20,
            steer_samples=7,
            max_nodes=50000,
        )
        self.local_planner = LocalPlanner(robot_cfg, nav_cfg, self.hybrid_fallback)
        self.velocity_planner = VelocityLocalPlanner(
            robot_cfg,
            nav_cfg,
            self.hybrid_fallback,
            command_cls=ControlCommand,
            pose_cls=Pose2D,
        )
        self.velocity_debug: Dict[str, Any] = {"enabled": bool(nav_cfg.continuous_control_enabled)}
        self.blocked_memory = BlockedPatchMemory()
        self.state = NavigatorState(init_pose, init_goal)
        self._last_left: Optional[int] = None
        self._last_right: Optional[int] = None
        self._last_odom_timestamp: Optional[float] = None
        self._last_replan_time = -1e9
        self._mark_radius = max(robot_cfg.obstacle_buffer_m, 0.025)
        self._stuck_counter = 0
        self._escape_queue: List[ControlCommand] = []
        self._last_pose_before_update = init_pose
        self._plan_costmap_dirty = True
        self._plan_costmap_cache: Optional[Costmap2D] = None
        self._plan_costmap_cache_key: Tuple[int, int] = (-1, -1)
        self._map_version = 0
        self._last_path_map_version = -1
        self._last_path_block_version = -1
        self._last_escape_side_left = True
        self._turn_loop_counter = 0
        self._active_scan_remaining = 0
        self._active_scan_direction = 'TURN_RIGHT'
        self._active_scan_reason = ''
        self._active_scan_cooldown_steps = 0
        self._active_scan_probe_steps = 0
        self._active_scan_last_direction: Optional[str] = None
        self._active_scan_needs_opposite_side = False
        self._last_front_depth_corridor_points = 0
        self._last_front_depth_corridor_min_m = float('inf')
        self._last_front_corridor_passable = True
        self._last_front_corridor_front_depth_m = float('inf')
        self._last_front_corridor_gap_heading_deg = 0.0
        self._last_front_corridor_gap_depth_m = float('inf')
        self._last_front_corridor_safe_headings = 0
        self._last_front_corridor_frontish_headings = 0
        self._last_front_corridor_straight_headings = 0
        self._front_blocked_evidence_counter = 0
        self._front_uncertain_evidence_counter = 0
        self._plan_failed_counter = 0
        self.recovery_state = RecoveryState.NORMAL
        self._odom_warning_counter = 0
        self.last_sent_cmd = ControlCommand("STOP")
        self.physical_stall = PhysicalStallState()
        self.closed_loop_health = ClosedLoopHealthState()
        self.row_follower_omega_state = RowFollowerOmegaState()
        self.closed_loop_debug = default_closed_loop_debug(nav_cfg)
        self._last_field_map_version = -1
        self._last_field_map_key = None
        self.last_odom_delta = {
            "left_ticks": 0,
            "right_ticks": 0,
            "ds_m": 0.0,
            "dtheta_deg": 0.0,
            "dt_s": 0.0,
            "warning": None,
        }

    def note_sent_command(self, cmd: ControlCommand) -> None:
        self.last_sent_cmd = cmd

    def physical_stall_status(self) -> Dict[str, Any]:
        return {
            "physical_stall_detected": bool(self.physical_stall.detected),
            "physical_stall_steps": int(self.physical_stall.steps),
            "physical_stall_duration_s": round(float(self.physical_stall.duration_s), 3),
            "physical_stall_reason": self.physical_stall.reason,
        }

    def _touch_map(self) -> None:
        self._map_version += 1
        self._plan_costmap_dirty = True

    def _ticks_to_meters(self, ticks: int) -> float:
        revs = ticks / float(self.robot_cfg.ticks_per_rev)
        return revs * 2.0 * math.pi * self.robot_cfg.wheel_radius_m

    def _update_pose_from_encoders(self, packet: EncoderPacket, imu: Optional[ImuPacket] = None) -> float:
        prev_pose = self.state.estimated_pose
        if self._last_left is None or self._last_right is None:
            self._last_left = packet.left_total
            self._last_right = packet.right_total
            self._last_odom_timestamp = packet.timestamp
            self._last_pose_before_update = prev_pose
            self.last_odom_delta = {
                "left_ticks": 0,
                "right_ticks": 0,
                "ds_m": 0.0,
                "dtheta_deg": 0.0,
                "dt_s": 0.0,
                "warning": None,
            }
            return 0.0

        dleft_ticks = packet.left_total - self._last_left
        dright_ticks = packet.right_total - self._last_right
        self._last_left = packet.left_total
        self._last_right = packet.right_total

        dl = self._ticks_to_meters(dleft_ticks)
        dr = self._ticks_to_meters(dright_ticks)
        ds = 0.5 * (dl + dr)
        dtheta = (dr - dl) / max(1e-6, self.robot_cfg.track_width_m)
        dt = 0.0 if self._last_odom_timestamp is None else max(0.0, packet.timestamp - self._last_odom_timestamp)
        self._last_odom_timestamp = packet.timestamp
        warning = None
        ds_for_pose = ds
        prev_cmd = self.state.latest_cmd
        prev_cmd_mode = prev_cmd.mode
        prev_velocity = prev_cmd.controller == "velocity"
        if (
            prev_cmd_mode in {'TURN_LEFT', 'TURN_RIGHT'}
            and abs(ds) > 0.035
            and not (prev_velocity and abs(prev_cmd.v_mps) > 0.025)
        ):
            warning = 'turn_command_has_linear_odom_check_encoder_inversion'
            ds_for_pose = 0.0
        elif (
            prev_cmd_mode in {'FORWARD', 'BACKWARD'}
            and dleft_ticks * dright_ticks < 0
            and abs(dleft_ticks) > 3
            and abs(dright_ticks) > 3
        ):
            max_cmd_raw = max(abs(prev_cmd.raw_left), abs(prev_cmd.raw_right))
            min_cmd_raw = min(abs(prev_cmd.raw_left), abs(prev_cmd.raw_right))
            straightish_velocity_cmd = (
                prev_cmd.raw_left * prev_cmd.raw_right > 0.0
                and min_cmd_raw >= max(0.08, 0.45 * max_cmd_raw)
                and abs(prev_cmd.raw_left - prev_cmd.raw_right) <= max(0.08, 0.35 * max_cmd_raw)
            )
            if not prev_velocity or straightish_velocity_cmd:
                warning = 'forward_command_has_opposite_encoder_signs'
                ds_for_pose = 0.0
                dtheta = 0.0
        if self.nav_cfg.use_imu_yaw and imu is not None and 0.0 < dt <= 0.5:
            yaw_rate = imu.yaw_rate(self.nav_cfg.imu_yaw_axis, self.nav_cfg.imu_yaw_sign)
            tick_motion = abs(dleft_ticks) + abs(dright_ticks)
            if math.isfinite(yaw_rate) and abs(yaw_rate) <= self.nav_cfg.imu_yaw_max_rate_rps and tick_motion > 0:
                blend = clamp(self.nav_cfg.imu_yaw_blend, 0.0, 1.0)
                dtheta_imu = yaw_rate * dt
                dtheta = (1.0 - blend) * dtheta + blend * dtheta_imu
        self.last_odom_delta = {
            "left_ticks": int(dleft_ticks),
            "right_ticks": int(dright_ticks),
            "ds_m": round(float(ds), 4),
            "ds_used_m": round(float(ds_for_pose), 4),
            "dtheta_deg": round(math.degrees(dtheta), 2),
            "dt_s": round(float(dt), 3),
            "warning": warning,
        }

        p = self.state.estimated_pose
        mid = p.yaw + 0.5 * dtheta
        new_pose = Pose2D(
            p.x + ds_for_pose * math.cos(mid),
            p.y + ds_for_pose * math.sin(mid),
            wrap_to_pi(p.yaw + dtheta),
        )
        self._last_pose_before_update = prev_pose
        self.state.estimated_pose = new_pose
        return math.hypot(new_pose.x - prev_pose.x, new_pose.y - prev_pose.y)

    def _local_to_world(self, pose: Pose2D, pt: Tuple[float, float]) -> Tuple[float, float]:
        lx, ly = pt
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        return pose.x + lx * c - ly * s, pose.y + lx * s + ly * c

    def _lidar_hits_in_base_frame(self, hits_local: Sequence[Tuple[float, float]], stride: int = 1) -> List[Tuple[float, float]]:
        half_fov_rad = 0.5 * math.radians(clamp(float(self.nav_cfg.lidar_used_fov_deg), 1.0, 360.0))
        out: List[Tuple[float, float]] = []
        ox = float(self.robot_cfg.lidar_offset_x_m)
        oy = float(self.robot_cfg.lidar_offset_y_m)
        stride = max(1, int(stride))
        accepted = 0
        for lx, ly in hits_local:
            if abs(wrap_to_pi(math.atan2(ly, lx))) > half_fov_rad:
                continue
            if accepted % stride != 0:
                accepted += 1
                continue
            accepted += 1
            out.append((float(lx) + ox, float(ly) + oy))
        return out

    def _lidar_ray_endpoints_in_base_frame(self, packet: LidarPacket, stride: int = 1) -> List[Tuple[float, float]]:
        half_fov_rad = 0.5 * math.radians(clamp(float(self.nav_cfg.lidar_used_fov_deg), 1.0, 360.0))
        max_ray_m = max(0.05, float(self.nav_cfg.local_costmap_max_raytrace_m))
        ox = float(self.robot_cfg.lidar_offset_x_m)
        oy = float(self.robot_cfg.lidar_offset_y_m)
        out: List[Tuple[float, float]] = []
        stride = max(1, int(stride))
        for idx, (rng, angle) in enumerate(zip(packet.ranges_m, packet.angles_rad)):
            if idx % stride != 0:
                continue
            angle = float(angle)
            if abs(wrap_to_pi(angle)) > half_fov_rad:
                continue
            try:
                r = float(rng)
            except (TypeError, ValueError):
                r = max_ray_m
            if not math.isfinite(r):
                r = max_ray_m
            if r <= 0.0:
                continue
            clear_r = min(r, max_ray_m)
            out.append((ox + clear_r * math.cos(angle), oy + clear_r * math.sin(angle)))
        return out

    def _clear_robot_footprint(self) -> None:
        pose = self.state.estimated_pose
        radius = max(self.robot_cfg.length_m, self.robot_cfg.width_m) * 0.42
        self.known_costmap.clear_disk_world(pose.x, pose.y, radius)

    def _integrate_lidar_freespace(self, packet: LidarPacket) -> None:
        # Keep persistent map obstacle-only. Free-space carving is applied only in the
        # local safety map so we do not erase and then re-add the same obstacle cells
        # every frame, which would force pointless global replans.
        return

    def _mark_world_hit(self, wx: float, wy: float, radius_m: Optional[float] = None) -> int:
        if wx < 0.0 or wy < 0.0:
            return 0
        if wx >= self.known_costmap.spec.width * self.known_costmap.spec.resolution:
            return 0
        if wy >= self.known_costmap.spec.height * self.known_costmap.spec.resolution:
            return 0
        gx, gy = self.known_costmap.world_to_grid(wx, wy)
        patch_before = self.known_costmap.data[max(0, gy-1):gy+2, max(0, gx-1):gx+2].copy()
        self.known_costmap.mark_disk_world(wx, wy, radius_m if radius_m is not None else self._mark_radius)
        patch_after = self.known_costmap.data[max(0, gy-1):gy+2, max(0, gx-1):gx+2]
        changed = int(np.any(patch_after > patch_before))
        if changed:
            self._touch_map()
        return changed

    def _mark_rect_world(self, cx: float, cy: float, w: float, h: float) -> int:
        gx0, gy0 = self.known_costmap.world_to_grid(cx - 0.5 * w, cy - 0.5 * h)
        gx1, gy1 = self.known_costmap.world_to_grid(cx + 0.5 * w, cy + 0.5 * h)
        gx0 = max(0, gx0)
        gy0 = max(0, gy0)
        gx1 = min(self.known_costmap.spec.width - 1, gx1)
        gy1 = min(self.known_costmap.spec.height - 1, gy1)
        if gx1 < gx0 or gy1 < gy0:
            return 0
        patch_before = self.known_costmap.data[gy0:gy1+1, gx0:gx1+1].copy()
        self.known_costmap.data[gy0:gy1+1, gx0:gx1+1] = 1
        patch_after = self.known_costmap.data[gy0:gy1+1, gx0:gx1+1]
        changed = int(np.any(patch_after > patch_before))
        if changed:
            self._touch_map()
        return changed

    def apply_field_map(self, packet: Optional[FieldMapPacket]) -> int:
        if packet is None:
            return 0
        if packet.version == self._last_field_map_version:
            return 0
        packet_key = field_map_content_key(packet)
        if packet_key == self._last_field_map_key:
            self._last_field_map_version = packet.version
            return 0

        added = 0
        self.known_costmap.data[:, :] = 0
        self._touch_map()
        self.static_costmap.data[:, :] = 0
        self.local_costmap.clear_static()
        for row, col in packet.obstacle_cells:
            cx, cy = field_cell_to_world(row, col, packet.size, packet.cell_size_m)
            added += self._mark_rect_world(cx, cy, packet.cell_size_m, packet.cell_size_m)
            self.static_costmap.data[
                max(0, self.static_costmap.world_to_grid(cx - 0.5 * packet.cell_size_m, cy - 0.5 * packet.cell_size_m)[1]):
                min(
                    self.static_costmap.spec.height,
                    self.static_costmap.world_to_grid(cx + 0.5 * packet.cell_size_m, cy + 0.5 * packet.cell_size_m)[1] + 1,
                ),
                max(0, self.static_costmap.world_to_grid(cx - 0.5 * packet.cell_size_m, cy - 0.5 * packet.cell_size_m)[0]):
                min(
                    self.static_costmap.spec.width,
                    self.static_costmap.world_to_grid(cx + 0.5 * packet.cell_size_m, cy + 0.5 * packet.cell_size_m)[0] + 1,
                ),
            ] = 1
            self.local_costmap.mark_static_world(
                cx,
                cy,
                radius_m=max(0.5 * packet.cell_size_m, self.nav_cfg.local_costmap_obstacle_radius_m),
                label="field_map",
            )

        if packet.start_xy is not None:
            self.known_costmap.clear_disk_world(packet.start_xy[0], packet.start_xy[1], packet.cell_size_m * 0.45)
            self.static_costmap.clear_disk_world(packet.start_xy[0], packet.start_xy[1], packet.cell_size_m * 0.45)
        if packet.goal_xy is not None:
            self.known_costmap.clear_disk_world(packet.goal_xy[0], packet.goal_xy[1], packet.cell_size_m * 0.45)
            self.static_costmap.clear_disk_world(packet.goal_xy[0], packet.goal_xy[1], packet.cell_size_m * 0.45)

        self._last_field_map_version = packet.version
        self._last_field_map_key = packet_key
        self._plan_costmap_dirty = True
        return added

    def _solidify_world_clusters(self, world_pts: Sequence[Tuple[float, float]]) -> int:
        if len(world_pts) < 4:
            return 0
        pts = list(world_pts)
        used = [False] * len(pts)
        added = 0
        cluster_thresh = 0.26
        pad = max(self.known_costmap.spec.resolution * 0.8, 0.06)
        for i in range(len(pts)):
            if used[i]:
                continue
            q = [i]
            used[i] = True
            cluster = [pts[i]]
            while q:
                k = q.pop()
                ax, ay = pts[k]
                for j in range(len(pts)):
                    if used[j]:
                        continue
                    bx, by = pts[j]
                    if math.hypot(bx - ax, by - ay) <= cluster_thresh:
                        used[j] = True
                        q.append(j)
                        cluster.append(pts[j])
            if len(cluster) < 4:
                continue
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            xmin, xmax = min(xs) - pad, max(xs) + pad
            ymin, ymax = min(ys) - pad, max(ys) + pad
            span_x = xmax - xmin
            span_y = ymax - ymin
            gx0, gy0 = self.known_costmap.world_to_grid(xmin, ymin)
            gx1, gy1 = self.known_costmap.world_to_grid(xmax, ymax)
            gx0 = max(0, gx0)
            gy0 = max(0, gy0)
            gx1 = min(self.known_costmap.spec.width - 1, gx1)
            gy1 = min(self.known_costmap.spec.height - 1, gy1)
            patch_before = self.known_costmap.data[gy0:gy1+1, gx0:gx1+1].copy()
            if span_x >= 0.12 and span_y >= 0.12:
                self.known_costmap.data[gy0:gy1+1, gx0:gx1+1] = 1
            else:
                cx = 0.5 * (xmin + xmax)
                cy = 0.5 * (ymin + ymax)
                self.known_costmap.mark_disk_world(cx, cy, max(0.10, 0.5 * max(span_x, span_y)))
            patch_after = self.known_costmap.data[gy0:gy1+1, gx0:gx1+1]
            if np.any(patch_after > patch_before):
                added += 1
                self._touch_map()
        return added

    def _integrate_hits_into_map(self, hits_local: Sequence[Tuple[float, float]], connect_adjacent: bool = False, solidify_clusters: bool = False) -> int:
        pose = self.state.estimated_pose
        added = 0
        world_pts = []
        for pt in hits_local:
            wx, wy = self._local_to_world(pose, pt)
            world_pts.append((wx, wy))
            added += self._mark_world_hit(wx, wy)
        if connect_adjacent and len(world_pts) >= 2:
            max_gap = max(self.known_costmap.spec.resolution * 2.5, 0.22)
            for (ax, ay), (bx, by) in zip(world_pts[:-1], world_pts[1:]):
                d = math.hypot(bx - ax, by - ay)
                if d <= max_gap:
                    n = max(2, int(d / max(self.known_costmap.spec.resolution * 0.8, 0.05)))
                    for i in range(1, n):
                        t = i / n
                        added += self._mark_world_hit(ax + t * (bx - ax), ay + t * (by - ay))
        if solidify_clusters:
            added += self._solidify_world_clusters(world_pts)
        return added

    def _inflate_costmap(self) -> Costmap2D:
        key = (self._map_version, self.blocked_memory.version)
        if (not self._plan_costmap_dirty) and (self._plan_costmap_cache is not None) and key == self._plan_costmap_cache_key:
            return self._plan_costmap_cache

        src = self.known_costmap
        inflated = src.data.copy()
        work = Costmap2D(src.spec, inflated)
        self.blocked_memory.rasterize(work)
        inflated = work.data.copy()

        inflate_radius = max(self.nav_cfg.global_plan_inflation_m, self.robot_cfg.width_m * 0.52 + self.robot_cfg.obstacle_buffer_m)
        cells = max(1, int(math.ceil(inflate_radius / src.spec.resolution)))
        inflated = self._inflate_binary_grid(work.data, cells)

        out_map = Costmap2D(src.spec, inflated)
        clear_r = max(0.5 * math.hypot(self.robot_cfg.length_m, self.robot_cfg.width_m) * 0.7, 0.18)
        out_map.clear_disk_world(self.state.estimated_pose.x, self.state.estimated_pose.y, clear_r)
        out_map.clear_disk_world(self.state.goal_pose.x, self.state.goal_pose.y, clear_r)
        self._plan_costmap_cache = out_map
        self._plan_costmap_cache_key = key
        self._plan_costmap_dirty = False
        return self._plan_costmap_cache

    def _planning_costmap(self) -> Costmap2D:
        return self._inflate_costmap()

    def _update_rolling_local_costmap(
        self,
        lidar_packet: LidarPacket,
        lidar_hits_base: Sequence[Tuple[float, float]],
        zed_hits_base: Sequence[Tuple[float, float]],
        timestamp: float,
    ) -> None:
        if not self.nav_cfg.local_costmap_enabled:
            self.local_costmap_debug = {"enabled": False}
            return
        pose = self.state.estimated_pose
        lidar_clear_points_base = self._lidar_ray_endpoints_in_base_frame(lidar_packet)
        stats = self.local_costmap.update(
            pose=(pose.x, pose.y, pose.yaw),
            lidar_points_base=lidar_hits_base,
            lidar_clear_points_base=lidar_clear_points_base,
            depth_points_base=zed_hits_base,
            ray_origin_base=(self.robot_cfg.lidar_offset_x_m, self.robot_cfg.lidar_offset_y_m),
            timestamp=timestamp,
        )
        self.local_costmap_debug = {"enabled": True, **stats.as_dict()}

    @staticmethod
    def _inflate_binary_grid(data: np.ndarray, cells: int) -> np.ndarray:
        cells = max(1, int(cells))
        src = (data > 0).astype(np.uint8)
        if cv2 is not None:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * cells + 1, 2 * cells + 1))
            return cv2.dilate(src, kernel, iterations=1).astype(np.uint8)

        inflated = src.copy()
        height, width = src.shape[:2]
        occ = np.argwhere(src > 0)
        for gy, gx in occ:
            for ddy in range(-cells, cells + 1):
                ny = gy + ddy
                if ny < 0 or ny >= height:
                    continue
                for ddx in range(-cells, cells + 1):
                    nx = gx + ddx
                    if nx < 0 or nx >= width:
                        continue
                    if ddx * ddx + ddy * ddy <= cells * cells:
                        inflated[ny, nx] = 1
        return inflated

    def _local_safety_costmap(self) -> Costmap2D:
        if self.nav_cfg.local_costmap_enabled:
            spec_like, data, stats = self.local_costmap.make_grid()
            spec = GridSpec(
                resolution=float(spec_like.resolution),
                origin_x=float(spec_like.origin_x),
                origin_y=float(spec_like.origin_y),
                width=int(spec_like.width),
                height=int(spec_like.height),
            )
            work = Costmap2D(spec, data.copy())
            self.blocked_memory.rasterize(work)
            self.local_costmap_debug = {"enabled": True, **stats.as_dict()}
            return work

        src = self.known_costmap
        work = Costmap2D(src.spec, src.data.copy())
        self.blocked_memory.rasterize(work)
        if self.nav_cfg.local_plan_inflation_m > 1e-6:
            cells = max(1, int(math.ceil(self.nav_cfg.local_plan_inflation_m / src.spec.resolution)))
            inflated = self._inflate_binary_grid(work.data, cells)
            work = Costmap2D(src.spec, inflated)
        clear_r = max(
            0.65,
            0.5 * math.hypot(self.robot_cfg.length_m, self.robot_cfg.width_m)
            + self.nav_cfg.local_plan_inflation_m
            + self.robot_cfg.obstacle_buffer_m
            + 0.12,
        )
        work.clear_disk_world(self.state.estimated_pose.x, self.state.estimated_pose.y, clear_r)
        return work

    def _plan_with_fallback(self, start: Pose2D, goal: Pose2D) -> PlanResult:
        t0 = time.perf_counter()
        plan_map = self._planning_costmap()
        path = self.fast_planner.plan(plan_map, start, goal)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        if path:
            return PlanResult(path, 'grid_astar', elapsed_ms)

        t1 = time.perf_counter()
        path = self.hybrid_fallback.plan(plan_map, start, goal)
        elapsed2_ms = (time.perf_counter() - t1) * 1000.0
        return PlanResult(path, 'hybrid_astar' if path else 'failed', elapsed_ms + elapsed2_ms)

    def _path_conflicts(self) -> bool:
        path = self.state.path
        idx = self.state.path_idx
        if not path:
            return True
        checker = self.hybrid_fallback
        plan_map = self._planning_costmap()
        end = min(len(path), idx + self.nav_cfg.replan_lookahead_poses)
        start_check = idx
        while start_check < end and math.hypot(path[start_check].x - self.state.estimated_pose.x, path[start_check].y - self.state.estimated_pose.y) < max(self.nav_cfg.path_reach_tol_m, 0.18):
            start_check += 1
        for i in range(start_check, end):
            if checker._pose_in_collision(plan_map, path[i]):
                return True
        for i in range(max(1, start_check), end):
            if checker._segment_in_collision(plan_map, path[i - 1], path[i]):
                return True
        return False

    def _maybe_replan(self, now: float, force: bool = False) -> None:
        if not force and now - self._last_replan_time < self.nav_cfg.replan_cooldown_s:
            return
        result = self._plan_with_fallback(self.state.estimated_pose, self.state.goal_pose)
        self._last_replan_time = now
        self.state.planner_name = result.planner_name
        self.state.plan_time_ms = result.elapsed_ms
        if result.path:
            self.state.path = result.path
            self.state.path_idx = 0
            self.state.replans += 1
            self._last_path_map_version = self._map_version
            self._last_path_block_version = self.blocked_memory.version

    def _choose_target(self) -> Optional[Pose2D]:
        path = self.state.path
        if not path:
            return None
        p = self.state.estimated_pose
        while self.state.path_idx < len(path):
            cur = path[self.state.path_idx]
            d = math.hypot(cur.x - p.x, cur.y - p.y)
            if d < self.nav_cfg.path_reach_tol_m:
                self.state.path_idx += 1
            else:
                break
        if self.state.path_idx >= len(path):
            return None

        accum = 0.0
        idx = self.state.path_idx
        prev = p
        chosen = path[-1]
        while idx < len(path):
            nxt = path[idx]
            accum += math.hypot(nxt.x - prev.x, nxt.y - prev.y)
            prev = nxt
            chosen = nxt
            if accum >= self.nav_cfg.local_lookahead_m:
                break
            idx += 1
        # avoid choosing a target that is strongly behind the robot when a farther path point is available
        j = idx
        while j < len(path):
            cand = path[j]
            ang = wrap_to_pi(math.atan2(cand.y - p.y, cand.x - p.x) - p.yaw)
            if abs(math.degrees(ang)) <= 110.0:
                return cand
            j += 1
        return chosen

    def _add_blocked_patch_ahead(self, reverse: bool = False) -> None:
        pose = self.state.estimated_pose
        sign = -1.0 if reverse else 1.0
        d = self.nav_cfg.blocked_patch_distance_m
        radius = max(self.nav_cfg.blocked_patch_radius_m, self.robot_cfg.width_m * 0.38)
        wx = pose.x + sign * d * math.cos(pose.yaw)
        wy = pose.y + sign * d * math.sin(pose.yaw)
        self.blocked_memory.add_patch(wx, wy, radius, self.nav_cfg.blocked_patch_ttl_steps)
        self._plan_costmap_dirty = True

    def _assess_front_corridor_gap(self, hits_local: Sequence[Tuple[float, float]]) -> bool:
        best_heading_deg = 0.0
        best_depth = 0.0
        best_score = -1e18
        safe_headings = 0
        frontish_safe_headings = 0
        straight_safe_headings = 0
        front_depth = 0.0
        frontish_best_depth = 0.0
        required_depth = self.nav_cfg.active_scan_front_clear_m
        for deg in range(-70, 71, 5):
            heading = math.radians(float(deg))
            depth = self.velocity_planner._gap_depth_for_heading(hits_local, heading)
            if deg == 0:
                front_depth = depth
            if abs(float(deg)) <= 35.0:
                frontish_best_depth = max(frontish_best_depth, depth)
            if depth >= required_depth:
                safe_headings += 1
                if abs(float(deg)) <= 35.0:
                    frontish_safe_headings += 1
                if abs(float(deg)) <= 20.0:
                    straight_safe_headings += 1
            # Prefer a wide/deep passage that is still a forward arc. A nearly
            # sideways opening is useful for scan direction, not proof that the
            # present forward corridor is passable.
            score = depth - 0.004 * abs(float(deg)) + 0.08 * math.cos(heading)
            if score > best_score:
                best_score = score
                best_heading_deg = float(deg)
                best_depth = depth
        passable = (
            (
                front_depth >= required_depth
                and frontish_safe_headings >= 2
                and abs(best_heading_deg) <= 45.0
            )
            or (
                straight_safe_headings >= 3
                and frontish_best_depth >= required_depth
                and abs(best_heading_deg) <= 35.0
            )
        )
        self._last_front_corridor_passable = bool(passable)
        self._last_front_corridor_front_depth_m = front_depth
        self._last_front_corridor_gap_heading_deg = best_heading_deg
        self._last_front_corridor_gap_depth_m = best_depth
        self._last_front_corridor_safe_headings = safe_headings
        self._last_front_corridor_frontish_headings = frontish_safe_headings
        self._last_front_corridor_straight_headings = straight_safe_headings
        return bool(passable)

    def _front_depth_corridor_blocked(
        self,
        zed_hits_local: Sequence[Tuple[float, float]],
        all_hits_local: Optional[Sequence[Tuple[float, float]]] = None,
    ) -> bool:
        front_limit = max(
            self.nav_cfg.active_scan_front_clear_m,
            self.robot_cfg.length_m * 0.5 + self.nav_cfg.front_safety_margin_m + 0.20,
        )
        half_width = (
            self.robot_cfg.width_m * 0.5
            + self.nav_cfg.local_plan_inflation_m
            + self.nav_cfg.active_scan_corridor_extra_width_m
        )
        min_x = float('inf')
        count = 0
        for lx, ly in zed_hits_local:
            x = float(lx)
            y = float(ly)
            if x < 0.15 or x > front_limit or abs(y) > half_width:
                continue
            count += 1
            min_x = min(min_x, x)
        self._last_front_depth_corridor_points = count
        self._last_front_depth_corridor_min_m = min_x
        gap_passable = self._assess_front_corridor_gap(all_hits_local if all_hits_local is not None else zed_hits_local)
        if count >= self.nav_cfg.active_scan_depth_points_threshold:
            return not gap_passable
        emergency_need = self.robot_cfg.length_m * 0.5 + self.nav_cfg.front_safety_margin_m
        return count >= 4 and min_x < emergency_need and not gap_passable

    @staticmethod
    def _clearance_score(value: float, cap: float = 3.0) -> float:
        if not math.isfinite(value):
            return cap
        return min(max(float(value), 0.0), cap)

    def _scan_direction_from_sectors(self, sectors: SectorSnapshot) -> str:
        left_score = (
            0.65 * self._clearance_score(sectors.front_left_m)
            + 0.35 * self._clearance_score(sectors.left_m)
        )
        right_score = (
            0.65 * self._clearance_score(sectors.front_right_m)
            + 0.35 * self._clearance_score(sectors.right_m)
        )
        if math.isfinite(self._last_front_corridor_gap_depth_m):
            gap_depth_bonus = clamp(self._last_front_corridor_gap_depth_m / 3.0, 0.0, 0.35)
            if self._last_front_corridor_gap_heading_deg > 18.0:
                left_score += gap_depth_bonus
            elif self._last_front_corridor_gap_heading_deg < -18.0:
                right_score += gap_depth_bonus
        if self.state.latest_cmd.mode == 'TURN_LEFT':
            left_score += 0.18
        elif self.state.latest_cmd.mode == 'TURN_RIGHT':
            right_score += 0.18
        if self._active_scan_needs_opposite_side and self._active_scan_last_direction:
            return 'TURN_RIGHT' if self._active_scan_last_direction == 'TURN_LEFT' else 'TURN_LEFT'
        if right_score > left_score + 0.05:
            return 'TURN_RIGHT'
        if left_score > right_score + 0.05:
            return 'TURN_LEFT'
        return 'TURN_LEFT' if self._last_escape_side_left else 'TURN_RIGHT'

    def _forward_view_confident(self, sectors: SectorSnapshot, front_depth_blocked: bool) -> bool:
        if front_depth_blocked:
            return False
        front_clear = sectors.front_m >= self.nav_cfg.active_scan_release_clear_m
        side_clear = min(sectors.front_left_m, sectors.front_right_m) >= self.nav_cfg.local_corridor_side_clear_m
        return bool(front_clear and side_clear)

    def _can_start_active_scan(self) -> bool:
        return (
            self.nav_cfg.active_scan_enabled
            and self._active_scan_remaining <= 0
            and self._active_scan_cooldown_steps <= 0
            and self._active_scan_probe_steps <= 0
        )

    def _update_recovery_state(self, front_clearance_m: Optional[float], forward_view_confident: bool) -> None:
        decision = classify_recovery_state(
            RecoveryContext(
                state=self.recovery_state,
                front_clearance_m=front_clearance_m,
                stuck_steps=self._stuck_counter,
                plan_failed_steps=self._plan_failed_counter,
                active_scan_remaining=self._active_scan_remaining,
                active_scan_cooldown_steps=self._active_scan_cooldown_steps + self._active_scan_probe_steps,
                forward_view_confident=forward_view_confident,
            )
        )
        self.recovery_state = decision.state

    def _start_active_scan(
        self,
        sectors: SectorSnapshot,
        reason: str,
        force_direction: Optional[str] = None,
        steps: Optional[int] = None,
    ) -> None:
        if not self.nav_cfg.active_scan_enabled:
            return
        forced_direction = force_direction in {'TURN_LEFT', 'TURN_RIGHT'}
        if forced_direction:
            self._active_scan_direction = force_direction
        else:
            self._active_scan_direction = self._scan_direction_from_sectors(sectors)
        self._last_escape_side_left = self._active_scan_direction == 'TURN_LEFT'
        if not forced_direction and self._active_scan_needs_opposite_side and self._active_scan_last_direction:
            self._active_scan_direction = 'TURN_RIGHT' if self._active_scan_last_direction == 'TURN_LEFT' else 'TURN_LEFT'
            self._last_escape_side_left = self._active_scan_direction == 'TURN_LEFT'
        self._active_scan_remaining = max(1, int(steps if steps is not None else self.nav_cfg.active_scan_steps))
        self._active_scan_reason = reason
        if not forced_direction and self._active_scan_needs_opposite_side and self._active_scan_last_direction:
            self._active_scan_reason = f"{reason}; checking opposite side after prior scan"
        self._active_scan_needs_opposite_side = False
        self._active_scan_cooldown_steps = 0
        self._active_scan_probe_steps = 0
        self._front_blocked_evidence_counter = 0
        self._front_uncertain_evidence_counter = 0
        self._plan_failed_counter = 0
        self._escape_queue = []
        self.state.path = []
        self.state.path_idx = 0
        self.velocity_planner.reset()

    def _finish_active_scan(self, *, view_cleared: bool = False) -> None:
        previous_direction = self._active_scan_direction
        self._active_scan_remaining = 0
        self._active_scan_reason = ''
        self._front_blocked_evidence_counter = 0
        self._front_uncertain_evidence_counter = 0
        self._plan_failed_counter = 0
        self._active_scan_last_direction = previous_direction
        self._active_scan_needs_opposite_side = not view_cleared
        if view_cleared:
            self._active_scan_cooldown_steps = 0
            self._active_scan_probe_steps = 0
        else:
            self._active_scan_cooldown_steps = max(0, int(self.nav_cfg.active_scan_cooldown_steps))
            self._active_scan_probe_steps = max(0, int(self.nav_cfg.active_scan_probe_steps))
        self.state.path = []
        self.state.path_idx = 0
        self.velocity_planner.reset()

    def _active_scan_command(self, local_map: Costmap2D) -> ControlCommand:
        turn_deg = max(8.0, float(self.nav_cfg.active_scan_turn_deg))
        direction = self._active_scan_direction
        scan_raw = 0.60
        raw_left = -scan_raw if direction == 'TURN_LEFT' else scan_raw
        raw_right = scan_raw if direction == 'TURN_LEFT' else -scan_raw
        cmd = ControlCommand(
            direction,
            turn_deg=turn_deg,
            raw_left=raw_left,
            raw_right=raw_right,
            reason=f"active scan; {self._active_scan_reason}; remaining={self._active_scan_remaining}",
            controller="active_scan",
        )
        if not self.local_planner._safe_on_costmap(local_map, self.state.estimated_pose, cmd):
            other = 'TURN_RIGHT' if direction == 'TURN_LEFT' else 'TURN_LEFT'
            cmd = ControlCommand(
                other,
                turn_deg=turn_deg,
                raw_left=scan_raw if other == 'TURN_RIGHT' else -scan_raw,
                raw_right=-scan_raw if other == 'TURN_RIGHT' else scan_raw,
                reason=f"active scan alternate; {self._active_scan_reason}; remaining={self._active_scan_remaining}",
                controller="active_scan",
            )
            self._active_scan_direction = other
        self._active_scan_remaining = max(0, self._active_scan_remaining - 1)
        if self._active_scan_remaining <= 0:
            self._finish_active_scan(view_cleared=False)
        return cmd

    def active_scan_status(self) -> dict:
        return {
            "enabled": bool(self.nav_cfg.active_scan_enabled),
            "remaining": int(self._active_scan_remaining),
            "direction": self._active_scan_direction,
            "reason": self._active_scan_reason,
            "cooldown_steps": int(self._active_scan_cooldown_steps),
            "probe_steps": int(self._active_scan_probe_steps),
            "needs_opposite_side": bool(self._active_scan_needs_opposite_side),
            "last_direction": self._active_scan_last_direction,
            "front_blocked_evidence": int(self._front_blocked_evidence_counter),
            "front_uncertain_evidence": int(self._front_uncertain_evidence_counter),
            "plan_failed_evidence": int(self._plan_failed_counter),
            "front_depth_corridor_points": int(self._last_front_depth_corridor_points),
            "front_depth_corridor_min_m": (
                None if math.isinf(self._last_front_depth_corridor_min_m)
                else round(self._last_front_depth_corridor_min_m, 3)
            ),
            "front_corridor_passable": bool(self._last_front_corridor_passable),
            "front_corridor_front_depth_m": (
                None if math.isinf(self._last_front_corridor_front_depth_m)
                else round(self._last_front_corridor_front_depth_m, 3)
            ),
            "front_corridor_gap_heading_deg": round(self._last_front_corridor_gap_heading_deg, 1),
            "front_corridor_gap_depth_m": (
                None if math.isinf(self._last_front_corridor_gap_depth_m)
                else round(self._last_front_corridor_gap_depth_m, 3)
            ),
            "front_corridor_safe_headings": int(self._last_front_corridor_safe_headings),
            "front_corridor_frontish_headings": int(self._last_front_corridor_frontish_headings),
            "front_corridor_straight_headings": int(self._last_front_corridor_straight_headings),
        }

    def _continuous_active_scan_reason(
        self,
        cmd: ControlCommand,
        front_depth_blocked: bool,
        confirmed_front_blocked: bool,
        confirmed_plan_failed: bool,
    ) -> str:
        if not self.nav_cfg.active_scan_enabled:
            return ""
        if self._active_scan_cooldown_steps > 0 or self._active_scan_probe_steps > 0:
            return ""
        if confirmed_front_blocked and not self._last_front_corridor_passable:
            return "panoramic confirmed front blocked during velocity control"
        if confirmed_plan_failed and not self._last_front_corridor_passable:
            return "panoramic path failed while front constrained during velocity control"

        debug = self.velocity_debug or {}
        samples = int(debug.get("samples") or 0)
        safe_samples = int(debug.get("safe_samples") or 0)
        safe_ratio = (safe_samples / samples) if samples > 0 else 1.0
        try:
            gap_heading_deg = abs(float(debug.get("best_gap_heading_deg") or 0.0))
        except (TypeError, ValueError):
            gap_heading_deg = 0.0
        try:
            gap_depth_m = float(debug.get("best_gap_depth_m") or 0.0)
        except (TypeError, ValueError):
            gap_depth_m = 0.0
        try:
            selected_v = abs(float(debug.get("selected_v_mps", cmd.v_mps) or 0.0))
        except (TypeError, ValueError):
            selected_v = abs(float(cmd.v_mps or 0.0))

        front_values = [self.state.sectors.front_m]
        try:
            dbg_front = float(debug.get("front_clearance_m"))
            if math.isfinite(dbg_front):
                front_values.append(dbg_front)
        except (TypeError, ValueError):
            pass
        if math.isfinite(self._last_front_depth_corridor_min_m) and not self._last_front_corridor_passable:
            front_values.append(self._last_front_depth_corridor_min_m)
        front_constraint_m = min(front_values) if front_values else float('inf')
        depth_corridor_close = (
            front_depth_blocked
            and math.isfinite(self._last_front_depth_corridor_min_m)
            and self._last_front_depth_corridor_min_m < self.nav_cfg.active_scan_front_clear_m
            and not self._last_front_corridor_passable
        )
        front_constrained = (
            (
                self.state.sectors.front_m < self.nav_cfg.active_scan_allow_probe_clear_m
                and not self._last_front_corridor_passable
            )
            or depth_corridor_close
            or (
                front_constraint_m < self.nav_cfg.active_scan_front_clear_m
                and not self._last_front_corridor_passable
            )
        )
        evidence_ready = self._front_blocked_evidence_counter >= max(
            2,
            int(self.nav_cfg.active_scan_confirm_steps) - 1,
        )
        lateral_escape = (
            gap_heading_deg >= 45.0
            and math.isfinite(gap_depth_m)
            and gap_depth_m >= self.nav_cfg.active_scan_release_clear_m
            and not self._last_front_corridor_passable
        )
        few_safe_trajectories = samples >= 20 and safe_ratio <= 0.22
        crawling_or_turning = (
            selected_v < max(0.035, 0.70 * self.nav_cfg.continuous_min_speed_mps)
            and cmd.mode in {'STOP', 'TURN_LEFT', 'TURN_RIGHT'}
        )

        if evidence_ready and front_constrained and (lateral_escape or few_safe_trajectories or crawling_or_turning):
            details = [
                f"front={front_constraint_m:.2f}m",
                f"gap={gap_heading_deg:.0f}deg/{gap_depth_m:.2f}m",
                f"safe={safe_samples}/{samples}",
            ]
            if depth_corridor_close:
                details.append(f"depth_pts={self._last_front_depth_corridor_points}")
            if lateral_escape:
                return "panoramic front clutter side-gap only; " + " ".join(details)
            return "proactive scan before close obstacle; " + " ".join(details)
        return ""

    def _build_escape_queue(self, sectors: SectorSnapshot) -> None:
        self._escape_queue = []
        rear_need = self.robot_cfg.length_m * 0.40 + self.nav_cfg.rear_safety_margin_m
        if self.nav_cfg.allow_reverse and sectors.rear_m > rear_need:
            self._escape_queue.append(ControlCommand('BACKWARD', move_m=min(self.nav_cfg.backward_step_choices_m), raw_left=-0.34, raw_right=-0.34, reason='escape reverse'))
        go_left = (sectors.left_m + sectors.front_left_m) >= (sectors.right_m + sectors.front_right_m)
        if math.isinf(sectors.left_m) and math.isinf(sectors.right_m):
            go_left = self._last_escape_side_left
        self._last_escape_side_left = go_left
        recovery_turn_raw = clamp(self.nav_cfg.recovery_turn_raw, 0.0, 1.0)
        if go_left:
            self._escape_queue.append(ControlCommand('TURN_LEFT', turn_deg=28.0, raw_left=-recovery_turn_raw, raw_right=recovery_turn_raw, reason='escape turn'))
        else:
            self._escape_queue.append(ControlCommand('TURN_RIGHT', turn_deg=28.0, raw_left=recovery_turn_raw, raw_right=-recovery_turn_raw, reason='escape turn'))

    def _consume_escape_queue(self, plan_map: Costmap2D) -> Optional[ControlCommand]:
        while self._escape_queue:
            cmd = self._escape_queue.pop(0)
            if self.local_planner._safe_on_costmap(plan_map, self.state.estimated_pose, cmd):
                return cmd
        return None

    def step(self, frame: SensorFrame) -> ControlCommand:
        self.state.finish_reason = 'running'
        map_added = self.apply_field_map(frame.field_map)
        if map_added:
            self.state.discovered_points += map_added
        if math.isfinite(frame.goal.x) and math.isfinite(frame.goal.y):
            self.state.goal_pose = Pose2D(frame.goal.x, frame.goal.y, 0.0)
        moved_m = self._update_pose_from_encoders(frame.encoder, frame.imu)
        update_physical_stall_state(
            self.physical_stall,
            self.last_sent_cmd,
            self.last_odom_delta,
            timeout_s=self.nav_cfg.physical_stall_timeout_s,
            min_raw=self.nav_cfg.physical_stall_min_raw,
            min_v_mps=self.nav_cfg.physical_stall_min_v_mps,
        )

        self.blocked_memory.step()
        if self.blocked_memory.version != self._plan_costmap_cache_key[1]:
            self._plan_costmap_dirty = True
        if self._active_scan_cooldown_steps > 0:
            self._active_scan_cooldown_steps -= 1
        if self._active_scan_probe_steps > 0:
            self._active_scan_probe_steps -= 1

        odom_warning = self.last_odom_delta.get("warning")
        if odom_warning in {
            'forward_command_has_opposite_encoder_signs',
            'turn_command_has_linear_odom_check_encoder_inversion',
        }:
            self._odom_warning_counter += 1
        else:
            self._odom_warning_counter = 0
        if self._odom_warning_counter >= 5:
            cmd = ControlCommand('STOP', reason=f'encoder sign calibration fault: {odom_warning}')
            self.state.finish_reason = 'encoder calibration fault'
            self.state.latest_cmd = cmd
            return cmd

        last_cmd = self.state.latest_cmd
        last_cmd_mode = last_cmd.mode
        stuck_eps = self.nav_cfg.stuck_pose_epsilon_m
        commanded_meaningful_forward = (
            last_cmd_mode in {'FORWARD', 'BACKWARD'}
            and not (last_cmd.controller == "velocity" and abs(last_cmd.v_mps) < 0.07)
        )
        if last_cmd.controller == "velocity":
            odom_dt = float(self.last_odom_delta.get("dt_s") or 0.10)
            expected_move = abs(last_cmd.v_mps) * clamp(odom_dt, 0.05, 0.25)
            commanded_meaningful_forward = last_cmd_mode in {'FORWARD', 'BACKWARD'} and expected_move >= 0.010
            stuck_eps = max(0.004, min(stuck_eps, expected_move * 0.35))
        if commanded_meaningful_forward and moved_m < stuck_eps:
            self._stuck_counter += 1
        elif moved_m >= stuck_eps:
            self._stuck_counter = 0

        self._integrate_lidar_freespace(frame.lidar)
        lidar_hits_local = self._lidar_hits_in_base_frame(frame.lidar.hit_points_local)
        # Ordinary sensor hits stay in the rolling local costmap. Do not poison
        # the persistent global planning map with false-positive dynamic cells.
        added = 0
        self.state.discovered_points += added
        local_obstacle_hits = lidar_hits_local + list(frame.zed.hit_points_local)
        self._update_rolling_local_costmap(frame.lidar, lidar_hits_local, frame.zed.hit_points_local, frame.encoder.timestamp)
        self.state.sectors = self.local_planner.build_sector_snapshot(local_obstacle_hits)
        front_depth_blocked = self._front_depth_corridor_blocked(frame.zed.hit_points_local, local_obstacle_hits)
        front_depth_corridor_close = (
            front_depth_blocked
            and math.isfinite(self._last_front_depth_corridor_min_m)
            and self._last_front_depth_corridor_min_m < self.nav_cfg.active_scan_front_clear_m
            and not self._last_front_corridor_passable
        )
        front_lidar_center_close = (
            self.state.sectors.front_m < self.nav_cfg.active_scan_allow_probe_clear_m
            and not self._last_front_corridor_passable
        )
        front_evidence = front_lidar_center_close or front_depth_corridor_close
        active_scan_suppressed = self._active_scan_cooldown_steps > 0 or self._active_scan_probe_steps > 0
        forward_view_confident = self._forward_view_confident(self.state.sectors, front_depth_blocked)
        front_clearance_for_recovery = None if math.isinf(self.state.sectors.front_m) else self.state.sectors.front_m
        self._update_recovery_state(front_clearance_for_recovery, forward_view_confident)
        if front_evidence and not active_scan_suppressed:
            self._front_blocked_evidence_counter += 1
        elif active_scan_suppressed or forward_view_confident:
            self._front_blocked_evidence_counter = 0
        else:
            self._front_blocked_evidence_counter = max(0, self._front_blocked_evidence_counter - 1)
        side_gap_only = (
            not self._last_front_corridor_passable
            and abs(self._last_front_corridor_gap_heading_deg) >= 45.0
            and math.isfinite(self._last_front_corridor_gap_depth_m)
            and self._last_front_corridor_gap_depth_m >= self.nav_cfg.active_scan_release_clear_m
            and (
                front_depth_corridor_close
                or self.state.sectors.front_m < self.nav_cfg.active_scan_front_clear_m
            )
        )
        if side_gap_only and not active_scan_suppressed:
            self._front_uncertain_evidence_counter += 1
        elif active_scan_suppressed or forward_view_confident:
            self._front_uncertain_evidence_counter = 0
        else:
            self._front_uncertain_evidence_counter = max(0, self._front_uncertain_evidence_counter - 1)
        if forward_view_confident:
            self._active_scan_needs_opposite_side = False
            self._active_scan_last_direction = None
        if last_cmd_mode == 'FORWARD' and moved_m >= max(self.nav_cfg.stuck_pose_epsilon_m, 0.020):
            self._active_scan_needs_opposite_side = False

        stuck_trigger_steps = self.nav_cfg.stuck_trigger_steps
        if last_cmd.controller == "velocity":
            # Continuous velocity control moves in short integration slices; give
            # encoder quantization and low-speed arcs time before declaring stuck.
            stuck_trigger_steps = max(stuck_trigger_steps, 12)
        if self._stuck_counter >= stuck_trigger_steps:
            self._add_blocked_patch_ahead(reverse=(last_cmd_mode == 'BACKWARD'))
            if self._can_start_active_scan():
                panoramic = not self._last_front_corridor_passable
                self._start_active_scan(
                    self.state.sectors,
                    'panoramic stuck after forward probe' if panoramic else 'stuck after forward probe',
                    force_direction='TURN_RIGHT' if panoramic else None,
                    steps=self.nav_cfg.active_scan_panoramic_steps if panoramic else None,
                )
            elif not self.nav_cfg.active_scan_enabled:
                self._build_escape_queue(self.state.sectors)
            self.state.path = []
            self.state.path_idx = 0
            self._stuck_counter = 0

        need_replan = False
        map_changed_since_plan = (
            self._map_version != self._last_path_map_version
            or self.blocked_memory.version != self._last_path_block_version
        )
        if not self.state.path:
            need_replan = True
        elif self._escape_queue:
            need_replan = True
        elif map_changed_since_plan and self._path_conflicts():
            need_replan = True

        if need_replan:
            self._maybe_replan(frame.encoder.timestamp, force=True)
        if self.state.planner_name == 'failed' and not self.state.path:
            self._plan_failed_counter += 1
        else:
            self._plan_failed_counter = 0

        p = self.state.estimated_pose
        g = self.state.goal_pose
        if math.hypot(g.x - p.x, g.y - p.y) <= self.nav_cfg.goal_tol_m:
            if self.nav_cfg.allow_stop_at_goal:
                cmd = ControlCommand('STOP', reason='goal reached')
            elif self.nav_cfg.competition_sweep_active and not self.nav_cfg.sweep_allow_pure_turn:
                raw = max(0.30, self.nav_cfg.min_motion_raw)
                cmd = ControlCommand('FORWARD', move_m=min(self.nav_cfg.forward_step_choices_m), raw_left=raw, raw_right=raw, reason='competition sweep forward probe at transient goal')
            else:
                cmd = ControlCommand('TURN_LEFT', turn_deg=10.0, raw_left=-0.24, raw_right=0.24, reason='goal reached loiter scan')
            self.state.finish_reason = 'goal reached'
            self.state.latest_cmd = cmd
            return cmd

        plan_map = self._planning_costmap()
        local_map = self._local_safety_costmap()
        if self._active_scan_remaining > 0:
            if self._forward_view_confident(self.state.sectors, front_depth_blocked):
                self._finish_active_scan(view_cleared=True)
            else:
                scan_cmd = self._active_scan_command(local_map)
                self.state.latest_cmd = scan_cmd
                return scan_cmd

        confirmed_front_uncertain = self._front_uncertain_evidence_counter >= max(
            1,
            int(self.nav_cfg.active_scan_confirm_steps) - 2,
        )
        if self._can_start_active_scan() and confirmed_front_uncertain:
            front_for_reason = min(
                self.state.sectors.front_m,
                self._last_front_depth_corridor_min_m
                if math.isfinite(self._last_front_depth_corridor_min_m)
                else float('inf'),
            )
            self._start_active_scan(
                self.state.sectors,
                (
                    "panoramic front clutter side-gap only; "
                    f"front={front_for_reason:.2f}m "
                    f"gap={self._last_front_corridor_gap_heading_deg:.0f}deg/"
                    f"{self._last_front_corridor_gap_depth_m:.2f}m "
                    f"evidence={self._front_uncertain_evidence_counter}"
                ),
                force_direction='TURN_RIGHT',
                steps=self.nav_cfg.active_scan_panoramic_steps,
            )
            scan_cmd = self._active_scan_command(local_map)
            self.state.latest_cmd = scan_cmd
            return scan_cmd

        making_forward_progress = last_cmd_mode == 'FORWARD' and moved_m >= self.nav_cfg.stuck_pose_epsilon_m
        front_constrained_now = front_lidar_center_close or front_depth_corridor_close
        confirmed_front_blocked = (
            self._front_blocked_evidence_counter >= self.nav_cfg.active_scan_confirm_steps
            and (
                not making_forward_progress
                or front_constrained_now
            )
        )
        confirmed_plan_failed = (
            self._plan_failed_counter >= self.nav_cfg.active_scan_plan_fail_confirm_steps
            and self._front_blocked_evidence_counter >= 2
            and not self._forward_view_confident(self.state.sectors, front_depth_blocked)
            and (
                not making_forward_progress
                or front_constrained_now
            )
        )
        if (
            self._can_start_active_scan()
            and not self.nav_cfg.continuous_control_enabled
            and (confirmed_front_blocked or confirmed_plan_failed)
        ):
            reason_parts = []
            if confirmed_front_blocked:
                reason_parts.append('confirmed front blocked')
            if confirmed_plan_failed:
                reason_parts.append('path failed while front constrained')
            reason = ', '.join(reason_parts)
            panoramic = not self._last_front_corridor_passable
            self._start_active_scan(
                self.state.sectors,
                f"panoramic {reason}" if panoramic else reason,
                force_direction='TURN_RIGHT' if panoramic else None,
                steps=self.nav_cfg.active_scan_panoramic_steps if panoramic else None,
            )
            scan_cmd = self._active_scan_command(local_map)
            self.state.latest_cmd = scan_cmd
            return scan_cmd

        escape_cmd = self._consume_escape_queue(local_map)
        if escape_cmd is not None:
            self.state.latest_cmd = escape_cmd
            return escape_cmd

        target = self._choose_target()
        if target is None:
            target = self.state.goal_pose
            self.state.path = []
            self.state.path_idx = 0

        if self.nav_cfg.continuous_control_enabled:
            cmd = self.velocity_planner.choose_command(
                self.state.estimated_pose,
                target,
                self.state.goal_pose,
                local_map,
                self.state.sectors,
                self.state.latest_cmd,
                local_obstacle_hits,
                frame.encoder.timestamp,
            )
            self.velocity_debug = dict(self.velocity_planner.last_debug)
            no_safe = self.velocity_debug.get("safety_state") == "no_safe_trajectory"
            if no_safe and self._can_start_active_scan():
                self._start_active_scan(self.state.sectors, "continuous controller found no safe trajectory")
                scan_cmd = self._active_scan_command(local_map)
                self.state.latest_cmd = scan_cmd
                self.velocity_debug["active_scan_requested"] = True
                return scan_cmd
            scan_reason = self._continuous_active_scan_reason(
                cmd,
                front_depth_blocked,
                confirmed_front_blocked,
                confirmed_plan_failed,
            )
            if scan_reason and self._can_start_active_scan():
                panoramic = scan_reason.startswith("panoramic")
                self._start_active_scan(
                    self.state.sectors,
                    scan_reason,
                    force_direction='TURN_RIGHT' if panoramic else None,
                    steps=self.nav_cfg.active_scan_panoramic_steps if panoramic else None,
                )
                scan_cmd = self._active_scan_command(local_map)
                self.state.latest_cmd = scan_cmd
                self.velocity_debug["active_scan_requested"] = scan_reason
                return scan_cmd
            if cmd.mode in {'FORWARD', 'BACKWARD'} and not self.local_planner._safe_on_costmap(local_map, self.state.estimated_pose, cmd):
                self._add_blocked_patch_ahead(reverse=(cmd.mode == 'BACKWARD'))
                self.state.path = []
                self.state.path_idx = 0
                self._maybe_replan(frame.encoder.timestamp, force=True)
                cmd = self.velocity_planner.choose_command(
                    self.state.estimated_pose,
                    target,
                    self.state.goal_pose,
                    self._local_safety_costmap(),
                    self.state.sectors,
                    self.state.latest_cmd,
                    local_obstacle_hits,
                    frame.encoder.timestamp,
                )
                self.velocity_debug = dict(self.velocity_planner.last_debug)
                scan_reason = self._continuous_active_scan_reason(
                    cmd,
                    front_depth_blocked,
                    confirmed_front_blocked,
                    confirmed_plan_failed,
                )
                if scan_reason and self._can_start_active_scan():
                    panoramic = scan_reason.startswith("panoramic")
                    self._start_active_scan(
                        self.state.sectors,
                        scan_reason,
                        force_direction='TURN_RIGHT' if panoramic else None,
                        steps=self.nav_cfg.active_scan_panoramic_steps if panoramic else None,
                    )
                    scan_cmd = self._active_scan_command(self._local_safety_costmap())
                    self.state.latest_cmd = scan_cmd
                    self.velocity_debug["active_scan_requested"] = scan_reason
                    return scan_cmd
            self.state.latest_cmd = cmd
            return cmd

        cmd = self.local_planner.choose_command(
            self.state.estimated_pose,
            target,
            self.state.goal_pose,
            local_map,
            self.state.sectors,
            self.state.latest_cmd,
        )

        if (
            self.nav_cfg.competition_sweep_active
            and not self.nav_cfg.sweep_allow_pure_turn
            and cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'}
        ):
            yaw_err = wrap_to_pi(math.atan2(target.y - self.state.estimated_pose.y, target.x - self.state.estimated_pose.x) - self.state.estimated_pose.yaw)
            large_heading = math.radians(max(70.0, 2.5 * float(self.nav_cfg.sweep_heading_tolerance_deg)))
            if abs(yaw_err) <= large_heading and self.local_planner._corridor_forward_clear(self.state.sectors):
                base = max(0.30, self.nav_cfg.min_motion_raw)
                steer_ratio = clamp(yaw_err / math.radians(35.0), -1.0, 1.0)
                inner = max(self.nav_cfg.min_motion_raw, base * (1.0 - 0.45 * abs(steer_ratio)))
                outer = min(0.55, base * (1.0 + 0.16 * abs(steer_ratio)))
                if steer_ratio >= 0.0:
                    probe = ControlCommand('FORWARD', move_m=min(self.nav_cfg.forward_step_choices_m), raw_left=inner, raw_right=outer, reason='competition sweep forward arc left')
                else:
                    probe = ControlCommand('FORWARD', move_m=min(self.nav_cfg.forward_step_choices_m), raw_left=outer, raw_right=inner, reason='competition sweep forward arc right')
                if self.local_planner._safe_on_costmap(local_map, self.state.estimated_pose, probe) and self.local_planner._safe_on_sectors(probe, self.state.sectors):
                    cmd = probe

        if cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'}:
            if self.state.latest_cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'} and cmd.mode != self.state.latest_cmd.mode:
                prev_turn = ControlCommand(
                    self.state.latest_cmd.mode,
                    turn_deg=max(self.state.latest_cmd.turn_deg, cmd.turn_deg),
                    raw_left=self.state.latest_cmd.raw_left if self.state.latest_cmd.raw_left != 0.0 else (-0.30 if self.state.latest_cmd.mode == 'TURN_LEFT' else 0.30),
                    raw_right=self.state.latest_cmd.raw_right if self.state.latest_cmd.raw_right != 0.0 else (0.30 if self.state.latest_cmd.mode == 'TURN_LEFT' else -0.30),
                    reason='turn commitment',
                )
                if self.local_planner._safe_on_costmap(local_map, self.state.estimated_pose, prev_turn):
                    cmd = prev_turn
            self._turn_loop_counter += 1
            if self._turn_loop_counter >= 3:
                probe = ControlCommand('FORWARD', move_m=min(self.nav_cfg.forward_step_choices_m), raw_left=0.34, raw_right=0.34, reason='forward probe after turn loop')
                if self.local_planner._safe_on_costmap(local_map, self.state.estimated_pose, probe) and self.local_planner._safe_on_sectors(probe, self.state.sectors):
                    cmd = probe
                    self._turn_loop_counter = 0
        else:
            self._turn_loop_counter = 0

        if cmd.mode in {'FORWARD', 'BACKWARD'} and not self.local_planner._safe_on_costmap(local_map, self.state.estimated_pose, cmd):
            self._add_blocked_patch_ahead(reverse=(cmd.mode == 'BACKWARD'))
            self._build_escape_queue(self.state.sectors)
            self.state.path = []
            self.state.path_idx = 0
            self._maybe_replan(frame.encoder.timestamp, force=True)
            escape_cmd = self._consume_escape_queue(self._local_safety_costmap())
            if escape_cmd is not None:
                cmd = escape_cmd

        self.state.latest_cmd = cmd
        return cmd


# =========================================================
# Visualization
# =========================================================


def robot_corners(pose: Pose2D, length_m: float, width_m: float) -> np.ndarray:
    hl = length_m / 2.0
    hw = width_m / 2.0
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    rot = np.array([[c, -s], [s, c]])
    world = local @ rot.T
    world[:, 0] += pose.x
    world[:, 1] += pose.y
    return world


def fov_polygon_points(pose: Pose2D, range_m: float, fov_deg: float, arc_points: int = 30) -> np.ndarray:
    half = math.radians(fov_deg * 0.5)
    angs = np.linspace(pose.yaw - half, pose.yaw + half, arc_points)
    pts = [(pose.x, pose.y)]
    for a in angs:
        pts.append((pose.x + range_m * math.cos(a), pose.y + range_m * math.sin(a)))
    return np.array(pts)


class SimVisualizer:
    def __init__(self, world: VirtualWorld, navigator: UGVNavigator, robot: SimulatedRobot, sensor_cfg: SensorConfig):
        ensure_matplotlib()
        self.world = world
        self.navigator = navigator
        self.robot = robot
        self.sensor_cfg = sensor_cfg

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title("UGV dual-mode sim, virtual sensors + Hybrid A*")
        self.ax.set_xlim(0.0, world.sim_cfg.field_w_m)
        self.ax.set_ylim(0.0, world.sim_cfg.field_h_m)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.grid(True, alpha=0.25)
        self.ax.set_facecolor("#fafafa")

        for obs in world.static_obstacles:
            if obs.kind == "cone":
                p = Circle((obs.x, obs.y), obs.w / 2.0, facecolor=(1.0, 0.55, 0.1, 0.35), edgecolor=(0.85, 0.35, 0.0, 1.0), linewidth=2)
                self.ax.add_patch(p)
            else:
                p = Rectangle((obs.x - obs.w / 2.0, obs.y - obs.h / 2.0), obs.w, obs.h, facecolor=(0.5, 0.3, 0.15, 0.25) if obs.kind == "box" else (0.48, 0.35, 0.78, 0.22), edgecolor=(0.35, 0.2, 0.05, 1.0) if obs.kind == "box" else (0.32, 0.15, 0.60, 1.0), linewidth=2, linestyle="--")
                self.ax.add_patch(p)

        self.goal_dot, = self.ax.plot([], [], "o", color="red", markersize=10)
        self.path_line, = self.ax.plot([], [], color="#0b57ff", linewidth=2.5)
        self.trail_line, = self.ax.plot([], [], color="#00a7a7", linewidth=1.5)
        self.est_trail_line, = self.ax.plot([], [], color="#7f3fbf", linewidth=1.4, alpha=0.85)
        self.true_poly = Polygon(robot_corners(robot.state.true_pose, navigator.robot_cfg.length_m, navigator.robot_cfg.width_m), closed=True, facecolor=(0.0, 0.75, 0.78, 0.45), edgecolor=(0.0, 0.45, 0.50, 1.0), linewidth=2.0)
        self.est_poly = Polygon(robot_corners(navigator.state.estimated_pose, navigator.robot_cfg.length_m, navigator.robot_cfg.width_m), closed=True, fill=False, edgecolor=(0.65, 0.0, 0.85, 1.0), linewidth=2.0)
        self.ax.add_patch(self.true_poly)
        self.ax.add_patch(self.est_poly)
        self.fov_poly = Polygon(fov_polygon_points(robot.state.true_pose, sensor_cfg.zed_range_m, sensor_cfg.zed_fov_deg), closed=True, facecolor=(0.10, 0.45, 1.00, 0.08), edgecolor=(0.10, 0.45, 1.00, 0.35), linewidth=1.0)
        self.ax.add_patch(self.fov_poly)
        self.hit_scatter = self.ax.scatter([], [], s=10, c="#aa22ff", alpha=0.45)
        self.status_text = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes, va="top", fontsize=10, bbox=dict(facecolor="white", alpha=0.90, edgecolor="lightgray"))

        self.legend = self.ax.legend(handles=[
            Line2D([0], [0], color="#0b57ff", lw=2.5, label="planned path"),
            Line2D([0], [0], color="#00a7a7", lw=1.5, label="true trail"),
            Line2D([0], [0], color="#7f3fbf", lw=1.4, label="odom trail"),
        ], loc="upper right")

    def update(self, true_trail: List[Tuple[float, float]], est_trail: List[Tuple[float, float]], latest_hits_world: List[Tuple[float, float]], step_i: int) -> None:
        self.goal_dot.set_data([self.navigator.state.goal_pose.x], [self.navigator.state.goal_pose.y])
        path = self.navigator.state.path
        if path:
            self.path_line.set_data([p.x for p in path], [p.y for p in path])
        else:
            self.path_line.set_data([], [])

        if true_trail:
            self.trail_line.set_data([p[0] for p in true_trail], [p[1] for p in true_trail])
        if est_trail:
            self.est_trail_line.set_data([p[0] for p in est_trail], [p[1] for p in est_trail])

        self.true_poly.set_xy(robot_corners(self.robot.state.true_pose, self.navigator.robot_cfg.length_m, self.navigator.robot_cfg.width_m))
        self.est_poly.set_xy(robot_corners(self.navigator.state.estimated_pose, self.navigator.robot_cfg.length_m, self.navigator.robot_cfg.width_m))
        self.fov_poly.set_xy(fov_polygon_points(self.robot.state.true_pose, self.sensor_cfg.zed_range_m, self.sensor_cfg.zed_fov_deg))

        if latest_hits_world:
            arr = np.array(latest_hits_world)
            self.hit_scatter.set_offsets(arr)
        else:
            self.hit_scatter.set_offsets(np.empty((0, 2)))

        pose_t = self.robot.state.true_pose
        pose_e = self.navigator.state.estimated_pose
        cmd = self.navigator.state.latest_cmd
        msg = (
            f"step: {step_i}\n"
            f"goal: ({self.navigator.state.goal_pose.x:.2f}, {self.navigator.state.goal_pose.y:.2f}) m\n"
            f"replans: {self.navigator.state.replans}\n"
            f"known hits: {self.navigator.state.discovered_points}\n"
            f"planner: {self.navigator.state.planner_name}\n"
            f"plan time: {self.navigator.state.plan_time_ms:.1f} ms\n"
            f"cmd: {cmd.short_text()}\n"
            f"cmd why: {cmd.reason}\n"
            f"true pose: ({pose_t.x:.2f}, {pose_t.y:.2f}, {math.degrees(pose_t.yaw):.1f} deg)\n"
            f"odom pose: ({pose_e.x:.2f}, {pose_e.y:.2f}, {math.degrees(pose_e.yaw):.1f} deg)\n"
            f"sectors f/l/r: {self.navigator.state.sectors.front_m:.2f}, {self.navigator.state.sectors.left_m:.2f}, {self.navigator.state.sectors.right_m:.2f} m\n"
            f"status: {self.navigator.state.finish_reason}"
        )
        self.status_text.set_text(msg)


# =========================================================
# Competition mission manager
# =========================================================

class CompetitionMission:
    def __init__(
        self,
        enabled: bool,
        mission_mode: str,
        start_corner: str,
        field_w_m: float,
        field_h_m: float,
        start_pose: Optional[Pose2D] = None,
        center_loiter_radius_m: float = 0.75,
        waypoint_switch_radius_m: float = 0.35,
        target_accept_radius_m: float = YARD_TO_M,
        min_speed_mps: float = 0.178816,
        search_radius_step_m: float = 0.75,
        straight_distance_m: float = yd(13.0),
        lidar_used_fov_deg: float = 180.0,
        lidar_offset_x_m: float = 0.30,
        lidar_offset_y_m: float = 0.0,
    ):
        self.enabled = bool(enabled)
        self.mission_mode = normalize_mission_mode(mission_mode)
        self.start_corner = normalize_corner_name(start_corner)
        self.field_w_m = field_w_m
        self.field_h_m = field_h_m
        self.center = (0.5 * field_w_m, 0.5 * field_h_m)
        self.start_xy_m = None if start_pose is None else (start_pose.x, start_pose.y)
        straight_start_x = yd(0.5) if start_pose is None else start_pose.x
        straight_start_y = yd(0.5) if start_pose is None else start_pose.y
        self.straight_goal = (
            clamp(straight_start_x + max(0.5, float(straight_distance_m)), 0.45, field_w_m - 0.45),
            clamp(straight_start_y, 0.45, field_h_m - 0.45),
        )
        self.center_loiter_radius_m = center_loiter_radius_m
        self.waypoint_switch_radius_m = waypoint_switch_radius_m
        self.target_accept_radius_m = max(0.35, float(target_accept_radius_m))
        self.min_speed_mps = max(0.0, float(min_speed_mps))
        self.search_radius_step_m = max(0.25, search_radius_step_m)
        self.search_max_radius_m = max(0.5, min(field_w_m, field_h_m) * 0.5 - 0.65)
        self.lidar_used_fov_deg = clamp(float(lidar_used_fov_deg), 1.0, 360.0)
        self.lidar_offset_x_m = float(lidar_offset_x_m)
        self.lidar_offset_y_m = float(lidar_offset_y_m)
        self.final_goal: Optional[Tuple[float, float]] = None
        self.final_goal_source = "none"
        self.latest_marker_distance_m: Optional[float] = None
        self.phase = "manual"
        self.stop_requested = False
        self.uav_state = "unknown"
        self.uav_state_source = "none"
        self._loiter_index = 0
        self._active_loiter_center = self.center
        self._search_index = 0
        self._search_radius_index = 0
        self._current_search_radius_m = self.center_loiter_radius_m
        self._indoor_goal_xy: Optional[Tuple[float, float]] = None
        self._indoor_goal_started_s = 0.0
        self._indoor_visited_cells: Set[Tuple[int, int]] = set()

    def update_frame(self, frame: SensorFrame, pose: Pose2D) -> dict:
        if frame.mission_flag is not None:
            self.uav_state = frame.mission_flag.state
            self.uav_state_source = frame.mission_flag.source

        if frame.marker_goal is not None and math.isfinite(frame.marker_goal.x) and math.isfinite(frame.marker_goal.y):
            self.final_goal = (frame.marker_goal.x, frame.marker_goal.y)
            self.final_goal_source = "camera_marker"
            self.latest_marker_distance_m = frame.marker_goal.distance_m
        elif self.mission_mode == "indoor":
            if self.final_goal_source != "camera_marker":
                self.final_goal = None
                self.final_goal_source = "none"
                self.latest_marker_distance_m = None
        elif frame.field_map is not None and frame.field_map.goal_xy is not None:
            self.final_goal = frame.field_map.goal_xy
            self.final_goal_source = frame.field_map.source
        elif math.isfinite(frame.goal.x) and math.isfinite(frame.goal.y):
            self.final_goal = (frame.goal.x, frame.goal.y)
            self.final_goal_source = "manual_point"

        if not self.enabled:
            self.phase = "manual"
            self.stop_requested = False
            return self._status_dict(frame.goal)

        self.stop_requested = False
        if self.mission_mode == "indoor":
            goal = self._update_indoor(frame, pose)
        elif self.mission_mode in {"round1", "round2"}:
            goal = self._update_straight_round(frame, pose)
        else:
            goal = self._update_round3(frame, pose)

        frame.goal = goal
        return self._status_dict(goal)

    def _target_reached(self, pose: Pose2D) -> bool:
        map_reached = False
        if self.final_goal is not None:
            map_reached = math.hypot(self.final_goal[0] - pose.x, self.final_goal[1] - pose.y) <= self.target_accept_radius_m
        depth_reached = (
            self.latest_marker_distance_m is not None
            and math.isfinite(self.latest_marker_distance_m)
            and self.latest_marker_distance_m <= self.target_accept_radius_m
        )
        return bool(map_reached or depth_reached)

    def _update_straight_round(self, frame: SensorFrame, pose: Pose2D) -> GoalPacket:
        if self.final_goal is not None:
            if self._target_reached(pose):
                self.phase = "target_stop"
                self.stop_requested = True
                return GoalPacket(pose.x, pose.y, frame.encoder.timestamp, self.latest_marker_distance_m)
            self.phase = "target_nav"
            return GoalPacket(self.final_goal[0], self.final_goal[1], frame.encoder.timestamp, self.latest_marker_distance_m)

        self.phase = "round1_straight" if self.mission_mode == "round1" else "round2_search_straight"
        return GoalPacket(self.straight_goal[0], self.straight_goal[1], frame.encoder.timestamp)

    def _update_round3(self, frame: SensorFrame, pose: Pose2D) -> GoalPacket:
        if self.final_goal is not None:
            if self._target_reached(pose):
                self.phase = "target_loiter"
                return self._loiter_goal(self.final_goal, pose)
            self.phase = "target_nav"
            return GoalPacket(self.final_goal[0], self.final_goal[1], frame.encoder.timestamp, self.latest_marker_distance_m)

        dist_to_center = math.hypot(self.center[0] - pose.x, self.center[1] - pose.y)
        if dist_to_center <= self.waypoint_switch_radius_m:
            self.phase = "search_expand"
            return self._search_goal(pose)
        self.phase = "startup_to_center"
        return GoalPacket(self.center[0], self.center[1], frame.encoder.timestamp)

    def _update_indoor(self, frame: SensorFrame, pose: Pose2D) -> GoalPacket:
        self._mark_indoor_visited(pose)
        if self.final_goal is not None:
            if self._target_reached(pose):
                self.phase = "target_loiter"
                return self._loiter_goal(self.final_goal, pose)
            self.phase = "target_nav"
            return GoalPacket(self.final_goal[0], self.final_goal[1], frame.encoder.timestamp, self.latest_marker_distance_m)

        self.phase = "indoor_search"
        return self._indoor_search_goal(frame, pose)

    def _mark_indoor_visited(self, pose: Pose2D) -> None:
        cell_m = 0.75
        self._indoor_visited_cells.add((int(math.floor(pose.x / cell_m)), int(math.floor(pose.y / cell_m))))

    def _indoor_search_goal(self, frame: SensorFrame, pose: Pose2D) -> GoalPacket:
        now_s = frame.encoder.timestamp
        lidar_hits = self._lidar_hits_for_indoor_sectors(frame.lidar.hit_points_local)
        sectors = LocalPlanner.build_sector_snapshot(lidar_hits + frame.zed.hit_points_local)
        front_blocked = sectors.front_m < 0.78
        front_constrained = (
            sectors.front_m < 1.15
            and min(sectors.front_left_m, sectors.front_right_m) < 1.10
        )
        goal_age_s = now_s - self._indoor_goal_started_s
        need_new = (
            self._indoor_goal_xy is None
            or goal_age_s > 4.5
            or (front_blocked and goal_age_s > 0.8)
            or (front_constrained and goal_age_s > 2.0)
        )
        if self._indoor_goal_xy is not None:
            if math.hypot(self._indoor_goal_xy[0] - pose.x, self._indoor_goal_xy[1] - pose.y) < 0.55:
                need_new = True

        if need_new:
            self._indoor_goal_xy = self._choose_indoor_goal(frame, pose, sectors)
            self._indoor_goal_started_s = now_s

        gx, gy = self._indoor_goal_xy
        return GoalPacket(gx, gy, now_s)

    def _lidar_hits_for_indoor_sectors(self, hits_local: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        half_fov_rad = 0.5 * math.radians(self.lidar_used_fov_deg)
        out: List[Tuple[float, float]] = []
        for lx, ly in hits_local:
            if abs(wrap_to_pi(math.atan2(ly, lx))) > half_fov_rad:
                continue
            out.append((float(lx) + self.lidar_offset_x_m, float(ly) + self.lidar_offset_y_m))
        return out

    def _choose_indoor_goal(self, frame: SensorFrame, pose: Pose2D, sectors: SectorSnapshot) -> Tuple[float, float]:
        del frame
        lookahead_m = 2.0
        cell_m = 0.75
        candidates_deg = (0.0, 25.0, -25.0, 55.0, -55.0, 90.0, -90.0)

        def sector_clearance(deg: float) -> float:
            if -25.0 <= deg <= 25.0:
                return sectors.front_m
            if 25.0 < deg <= 80.0:
                return min(sectors.front_left_m, sectors.left_m)
            if -80.0 <= deg < -25.0:
                return min(sectors.front_right_m, sectors.right_m)
            if 80.0 < deg <= 150.0:
                return sectors.left_m
            if -150.0 <= deg < -80.0:
                return sectors.right_m
            return 1.25

        best_score = -1e18
        best_xy = (
            clamp(pose.x + lookahead_m * math.cos(pose.yaw), 0.65, self.field_w_m - 0.65),
            clamp(pose.y + lookahead_m * math.sin(pose.yaw), 0.65, self.field_h_m - 0.65),
        )
        for deg in candidates_deg:
            heading = wrap_to_pi(pose.yaw + math.radians(deg))
            gx = clamp(pose.x + lookahead_m * math.cos(heading), 0.65, self.field_w_m - 0.65)
            gy = clamp(pose.y + lookahead_m * math.sin(heading), 0.65, self.field_h_m - 0.65)
            cell = (int(math.floor(gx / cell_m)), int(math.floor(gy / cell_m)))
            clearance = sector_clearance(deg)
            if not math.isfinite(clearance):
                clearance = 3.0
            score = min(clearance, 3.0)
            score += 1.2 if cell not in self._indoor_visited_cells else -0.35
            if abs(deg) <= 1e-6 and clearance > 0.65:
                score += 0.95
            if abs(deg) >= 80.0:
                score -= 0.35
            score -= 0.007 * abs(deg)
            if score > best_score:
                best_score = score
                best_xy = (gx, gy)
        return best_xy

    def _loiter_goal(self, center: Tuple[float, float], pose: Pose2D) -> GoalPacket:
        if center != self._active_loiter_center:
            self._active_loiter_center = center
            self._loiter_index = 0
        points = [
            (center[0] + self.center_loiter_radius_m, center[1]),
            (center[0], center[1] + self.center_loiter_radius_m),
            (center[0] - self.center_loiter_radius_m, center[1]),
            (center[0], center[1] - self.center_loiter_radius_m),
        ]
        cur = points[self._loiter_index % len(points)]
        if math.hypot(cur[0] - pose.x, cur[1] - pose.y) <= self.waypoint_switch_radius_m:
            self._loiter_index = (self._loiter_index + 1) % len(points)
            cur = points[self._loiter_index]
        return GoalPacket(cur[0], cur[1], time.time())

    def _search_goal(self, pose: Pose2D) -> GoalPacket:
        radius = min(
            self.search_max_radius_m,
            self.center_loiter_radius_m + self._search_radius_index * self.search_radius_step_m,
        )
        self._current_search_radius_m = radius
        cx, cy = self.center
        points = [
            (cx + radius, cy),
            (cx + radius, cy + radius),
            (cx, cy + radius),
            (cx - radius, cy + radius),
            (cx - radius, cy),
            (cx - radius, cy - radius),
            (cx, cy - radius),
            (cx + radius, cy - radius),
        ]
        points = [
            (
                clamp(x, 0.45, self.field_w_m - 0.45),
                clamp(y, 0.45, self.field_h_m - 0.45),
            )
            for x, y in points
        ]
        cur = points[self._search_index % len(points)]
        if math.hypot(cur[0] - pose.x, cur[1] - pose.y) <= self.waypoint_switch_radius_m:
            self._search_index = (self._search_index + 1) % len(points)
            if self._search_index == 0 and radius < self.search_max_radius_m:
                self._search_radius_index += 1
            cur = points[self._search_index]
        return GoalPacket(cur[0], cur[1], time.time())

    def command_speed_scale(self) -> Tuple[float, str]:
        if self.phase == "target_stop":
            return 0.0, "target_reached"
        if self.uav_state in {"landing", "landed"}:
            return 0.35, "uav_landing"
        if self.phase == "round1_straight":
            return 0.85, "round1_straight"
        if self.phase == "round2_search_straight":
            return 0.65, "round2_waiting_for_marker_or_uav"
        if self.phase == "startup_to_center":
            return 0.55, "waiting_for_uav_scan"
        if self.phase == "search_expand":
            return 0.70, "searching_from_center"
        if self.phase == "target_loiter":
            return 0.45, "target_loiter"
        if self.phase == "target_nav":
            return 1.0, "target_known"
        return 1.0, ""

    def _status_dict(self, active_goal: GoalPacket) -> dict:
        speed_scale, speed_reason = self.command_speed_scale()
        return {
            "enabled": self.enabled,
            "mode": self.mission_mode,
            "phase": self.phase,
            "stop_requested": self.stop_requested,
            "start_corner": self.start_corner,
            "start_m": [
                round(self.start_xy_m[0], 3),
                round(self.start_xy_m[1], 3),
            ] if self.start_xy_m is not None else None,
            "uav_state": self.uav_state,
            "uav_state_source": self.uav_state_source,
            "speed_scale": round(speed_scale, 3),
            "speed_reason": speed_reason,
            "min_speed_mps": round(self.min_speed_mps, 6),
            "search_radius_m": round(self._current_search_radius_m, 3),
            "target_accept_radius_m": round(self.target_accept_radius_m, 3),
            "marker_distance_m": None if self.latest_marker_distance_m is None else round(self.latest_marker_distance_m, 3),
            "active_goal_m": [
                round(active_goal.x, 3) if math.isfinite(active_goal.x) else None,
                round(active_goal.y, 3) if math.isfinite(active_goal.y) else None,
            ],
            "final_goal_m": [
                round(self.final_goal[0], 3),
                round(self.final_goal[1], 3),
            ] if self.final_goal is not None else None,
            "final_goal_source": self.final_goal_source,
        }


# =========================================================
# Running modes
# =========================================================


def run_simulation(
    show_gui: bool = True,
    max_steps: int = 900,
    robot_obstacle_buffer_m: float = 0.025,
    local_plan_inflation_m: float = 0.0,
    continuous_control_enabled: bool = True,
    continuous_max_speed_mps: float = 0.36,
    continuous_max_omega_rps: float = 1.15,
    continuous_horizon_s: float = 1.35,
    continuous_accel_limit_mps2: float = 0.35,
    continuous_omega_accel_limit_rps2: float = 1.80,
    continuous_lowpass_alpha: float = 0.55,
    continuous_raw_per_mps: float = 1.35,
    continuous_slowdown_clearance_m: float = 1.20,
    continuous_stop_clearance_m: float = 0.48,
    continuous_gap_buffer_m: float = 0.025,
    continuous_latency_buffer_s: float = 0.25,
    continuous_allow_costmap_soft_penalty: bool = False,
    emit_velocity_commands: bool = True,
    competition_closed_loop_enabled: bool = True,
    heading_hold_enabled: bool = True,
    lane_follow_enabled: bool = True,
    heading_hold_kp: float = 1.10,
    heading_hold_kd: float = 0.05,
    heading_hold_deadband_deg: float = 3.0,
    heading_hold_max_omega_rps: float = 0.55,
    lane_follow_kp_heading: float = 1.10,
    lane_follow_kp_omega: float = 1.00,
    lane_follow_deadband_m: float = 0.03,
    lane_follow_max_heading_deg: float = 18.0,
    lane_follow_max_omega_rps: float = 0.35,
    row_follower_speed_schedule_enabled: bool = True,
    row_follower_low_speed_mps: float = 0.09,
    row_follower_high_speed_mps: float = 0.22,
    row_follower_low_speed_lane_kp: float = 0.85,
    row_follower_high_speed_lane_kp: float = 1.00,
    row_follower_low_speed_heading_kp: float = 0.95,
    row_follower_high_speed_heading_kp: float = 1.10,
    row_follower_omega_low_pass_alpha: float = 0.35,
    row_follower_omega_rate_limit_rps2: float = 0.60,
    row_follower_min_correction_interval_s: float = 0.0,
    sweep_planner_omega_weight_round2: float = 0.0,
    sweep_planner_omega_weight_round3: float = 0.2,
    sweep_metrics_log_enabled: bool = False,
    sweep_metrics_log_dir: str = "~/.ros/ugv_sweep_metrics",
    forward_arc_only_enabled: bool = True,
    forward_arc_margin: float = 0.75,
    min_sweep_v_mps: float = 0.08,
    omega_command_sign: float = 1.0,
    heading_error_sign: float = 1.0,
    lane_error_sign: float = 1.0,
    lane_correction_sign: float = 1.0,
    closed_loop_health_enabled: bool = True,
    closed_loop_divergence_window: int = 5,
    closed_loop_divergence_min_error_m: float = 0.08,
    closed_loop_divergence_max_growth_m: float = 0.05,
    closed_loop_divergence_action: str = "slow_then_stop",
    local_costmap_enabled: bool = True,
    local_costmap_width_m: float = 4.0,
    local_costmap_height_m: float = 4.0,
    local_costmap_resolution_m: float = 0.06,
    local_costmap_dynamic_decay_s: float = 1.00,
    local_costmap_obstacle_radius_m: float = 0.06,
    local_costmap_inflation_m: float = 0.08,
    local_costmap_lidar_clear_radius_m: float = 0.05,
    local_costmap_max_raytrace_m: float = 4.0,
    recovery_turn_raw: float = 0.32,
    mission_mode: str = "manual",
    competition_mission_v2_enabled: bool = True,
    straight_distance_m: float = yd(13.0),
    sweep_field_width_m: float = yd(15.0),
    sweep_field_height_m: float = yd(15.0),
    sweep_row_length_m: float = 0.0,
    sweep_headland_margin_m: float = 0.75,
    sweep_boundary_margin_m: float = 0.45,
    sweep_turn_radius_m: float = 1.00,
    sweep_turn_style: str = "segmented_90",
    sweep_row_transition_enabled: bool = False,
    sweep_max_rows: int = 0,
    sweep_row_end_tolerance_m: float = 0.35,
    sweep_turn_90_yaw_tolerance_deg: float = 7.0,
    sweep_turn_strong_error_deg: float = 30.0,
    sweep_turn_capture_error_deg: float = 12.0,
    sweep_turn_settle_error_deg: float = 7.0,
    sweep_turn_settle_frames: int = 3,
    sweep_pivot_min_emitted_omega_rps: float = 0.25,
    sweep_pivot_max_emitted_omega_rps: float = 0.45,
    sweep_pivot_wheel_min_mps: float = 0.08,
    sweep_pivot_settle_frames: int = 3,
    sweep_cross_lane_v_mps: float = 0.12,
    sweep_cross_lane_heading_kp: float = 1.10,
    sweep_lane_capture_tolerance_m: float = 0.20,
    sweep_yaw_capture_tolerance_deg: float = 8.0,
    sweep_transition_min_emitted_v_mps: float = 0.12,
    sweep_transition_min_emitted_omega_rps: float = 0.20,
    sweep_transition_max_emitted_omega_rps: float = 0.40,
    sweep_transition_inner_wheel_min_mps: float = 0.04,
    sweep_transition_timeout_s: float = 20.0,
    sweep_transition_divergence_action: str = "stop",
    sweep_cell_size_m: float = 0.75,
    sweep_lane_spacing_m: float = 0.75,
    sweep_lane_spacing_manual_override: bool = False,
    sweep_coverage_radius_m: float = 0.55,
    sweep_coverage_threshold: float = 0.85,
    sweep_goal_timeout_s: float = 8.0,
    sweep_fail_limit: int = 3,
    sweep_lane_tolerance_m: float = 0.30,
    sweep_heading_tolerance_deg: float = 25.0,
    sweep_allow_pure_turn: bool = False,
    sweep_stall_action: str = "skip",
    marker_camera_fov_deg: float = 120.0,
    marker_reliable_detection_range_m: float = float("nan"),
    marker_coverage_overlap_ratio: float = 0.5,
    marker_auto_lane_spacing_enabled: bool = False,
    min_competition_speed_mps: float = 0.0894,
) -> dict:
    mission_mode = normalize_mission_mode(mission_mode)
    competition_v2_active = bool(competition_mission_v2_enabled) and mission_mode in {"round1", "round2", "round3"}
    robot_cfg = RobotConfig()
    robot_cfg.obstacle_buffer_m = clamp(float(robot_obstacle_buffer_m), 0.0, 0.20)
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.local_plan_inflation_m = clamp(float(local_plan_inflation_m), 0.0, 1.0)
    nav_cfg.continuous_control_enabled = bool(continuous_control_enabled)
    nav_cfg.continuous_max_speed_mps = clamp(float(continuous_max_speed_mps), 0.05, 1.0)
    nav_cfg.continuous_max_omega_rps = clamp(float(continuous_max_omega_rps), 0.20, 3.0)
    nav_cfg.continuous_horizon_s = clamp(float(continuous_horizon_s), 0.40, 2.0)
    nav_cfg.continuous_accel_limit_mps2 = clamp(float(continuous_accel_limit_mps2), 0.05, 2.0)
    nav_cfg.continuous_omega_accel_limit_rps2 = clamp(float(continuous_omega_accel_limit_rps2), 0.20, 5.0)
    nav_cfg.continuous_lowpass_alpha = clamp(float(continuous_lowpass_alpha), 0.05, 1.0)
    nav_cfg.continuous_raw_per_mps = clamp(float(continuous_raw_per_mps), 0.20, 4.0)
    nav_cfg.continuous_slowdown_clearance_m = clamp(float(continuous_slowdown_clearance_m), 0.50, 3.0)
    nav_cfg.continuous_stop_clearance_m = clamp(float(continuous_stop_clearance_m), 0.30, 1.50)
    nav_cfg.continuous_gap_buffer_m = clamp(float(continuous_gap_buffer_m), 0.0, 0.30)
    nav_cfg.continuous_latency_buffer_s = clamp(float(continuous_latency_buffer_s), 0.0, 1.0)
    nav_cfg.continuous_allow_costmap_soft_penalty = bool(continuous_allow_costmap_soft_penalty)
    nav_cfg.emit_velocity_commands = bool(emit_velocity_commands)
    nav_cfg.competition_closed_loop_enabled = bool(competition_closed_loop_enabled)
    nav_cfg.heading_hold_enabled = bool(heading_hold_enabled)
    nav_cfg.lane_follow_enabled = bool(lane_follow_enabled)
    nav_cfg.heading_hold_kp = clamp(float(heading_hold_kp), 0.0, 8.0)
    nav_cfg.heading_hold_kd = clamp(float(heading_hold_kd), 0.0, 4.0)
    nav_cfg.heading_hold_deadband_deg = clamp(float(heading_hold_deadband_deg), 0.0, 45.0)
    nav_cfg.heading_hold_max_omega_rps = clamp(float(heading_hold_max_omega_rps), 0.0, 3.0)
    nav_cfg.lane_follow_kp_heading = clamp(float(lane_follow_kp_heading), 0.0, 8.0)
    nav_cfg.lane_follow_kp_omega = clamp(float(lane_follow_kp_omega), 0.0, 8.0)
    nav_cfg.lane_follow_deadband_m = clamp(float(lane_follow_deadband_m), 0.0, 1.0)
    nav_cfg.lane_follow_max_heading_deg = clamp(float(lane_follow_max_heading_deg), 0.0, 60.0)
    nav_cfg.lane_follow_max_omega_rps = clamp(float(lane_follow_max_omega_rps), 0.0, 3.0)
    nav_cfg.row_follower_speed_schedule_enabled = bool(row_follower_speed_schedule_enabled)
    nav_cfg.row_follower_low_speed_mps = clamp(float(row_follower_low_speed_mps), 0.0, 2.0)
    nav_cfg.row_follower_high_speed_mps = clamp(float(row_follower_high_speed_mps), 0.0, 2.0)
    nav_cfg.row_follower_low_speed_lane_kp = clamp(float(row_follower_low_speed_lane_kp), 0.0, 8.0)
    nav_cfg.row_follower_high_speed_lane_kp = clamp(float(row_follower_high_speed_lane_kp), 0.0, 8.0)
    nav_cfg.row_follower_low_speed_heading_kp = clamp(float(row_follower_low_speed_heading_kp), 0.0, 8.0)
    nav_cfg.row_follower_high_speed_heading_kp = clamp(float(row_follower_high_speed_heading_kp), 0.0, 8.0)
    nav_cfg.row_follower_omega_low_pass_alpha = clamp(float(row_follower_omega_low_pass_alpha), 0.0, 1.0)
    nav_cfg.row_follower_omega_rate_limit_rps2 = clamp(float(row_follower_omega_rate_limit_rps2), 0.0, 10.0)
    nav_cfg.row_follower_min_correction_interval_s = clamp(float(row_follower_min_correction_interval_s), 0.0, 1.0)
    nav_cfg.sweep_planner_omega_weight_round2 = clamp(float(sweep_planner_omega_weight_round2), 0.0, 1.0)
    nav_cfg.sweep_planner_omega_weight_round3 = clamp(float(sweep_planner_omega_weight_round3), 0.0, 1.0)
    nav_cfg.sweep_metrics_log_enabled = bool(sweep_metrics_log_enabled)
    nav_cfg.sweep_metrics_log_dir = str(sweep_metrics_log_dir)
    nav_cfg.forward_arc_only_enabled = bool(forward_arc_only_enabled)
    nav_cfg.forward_arc_margin = clamp(float(forward_arc_margin), 0.0, 1.0)
    nav_cfg.min_sweep_v_mps = clamp(float(min_sweep_v_mps), 0.0, 1.0)
    nav_cfg.omega_command_sign = 1.0 if float(omega_command_sign) >= 0.0 else -1.0
    nav_cfg.heading_error_sign = 1.0 if float(heading_error_sign) >= 0.0 else -1.0
    nav_cfg.lane_error_sign = 1.0 if float(lane_error_sign) >= 0.0 else -1.0
    nav_cfg.lane_correction_sign = 1.0 if float(lane_correction_sign) >= 0.0 else -1.0
    nav_cfg.closed_loop_health_enabled = bool(closed_loop_health_enabled)
    nav_cfg.closed_loop_divergence_window = max(2, int(closed_loop_divergence_window))
    nav_cfg.closed_loop_divergence_min_error_m = clamp(float(closed_loop_divergence_min_error_m), 0.0, 2.0)
    nav_cfg.closed_loop_divergence_max_growth_m = clamp(float(closed_loop_divergence_max_growth_m), 0.0, 2.0)
    nav_cfg.closed_loop_divergence_action = str(closed_loop_divergence_action).strip().lower()
    if nav_cfg.closed_loop_divergence_action not in {"warn", "slow", "slow_then_stop", "stop"}:
        nav_cfg.closed_loop_divergence_action = "slow_then_stop"
    nav_cfg.local_costmap_enabled = bool(local_costmap_enabled)
    nav_cfg.local_costmap_width_m = clamp(float(local_costmap_width_m), 1.0, 10.0)
    nav_cfg.local_costmap_height_m = clamp(float(local_costmap_height_m), 1.0, 10.0)
    nav_cfg.local_costmap_resolution_m = clamp(float(local_costmap_resolution_m), 0.02, 0.25)
    nav_cfg.local_costmap_dynamic_decay_s = clamp(float(local_costmap_dynamic_decay_s), 0.0, 10.0)
    nav_cfg.local_costmap_obstacle_radius_m = clamp(float(local_costmap_obstacle_radius_m), 0.01, 0.40)
    nav_cfg.local_costmap_inflation_m = clamp(float(local_costmap_inflation_m), 0.0, 0.80)
    nav_cfg.local_costmap_lidar_clear_radius_m = clamp(float(local_costmap_lidar_clear_radius_m), 0.0, 0.40)
    nav_cfg.local_costmap_max_raytrace_m = clamp(float(local_costmap_max_raytrace_m), 0.20, 12.0)
    nav_cfg.recovery_turn_raw = clamp(float(recovery_turn_raw), 0.0, 1.0)
    nav_cfg.allow_stop_at_goal = not competition_v2_active
    if competition_v2_active:
        nav_cfg.continuous_min_speed_mps = clamp(
            max(nav_cfg.continuous_min_speed_mps, float(min_competition_speed_mps)),
            0.02,
            nav_cfg.continuous_max_speed_mps,
        )
    sim_cfg = SimConfig(
        field_w_m=max(0.5, float(sweep_field_width_m)),
        field_h_m=max(0.5, float(sweep_field_height_m)),
        show_gui=show_gui,
        max_steps=max_steps,
    )

    start = Pose2D(yd(0.5), yd(0.5), 0.0)
    goal = Pose2D(yd(13.4), yd(13.0), 0.0)

    world = VirtualWorld(sim_cfg, nav_cfg)
    robot = SimulatedRobot(robot_cfg, start)
    sensors = SimulatedSensors(robot_cfg, sensor_cfg, world)
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, sim_cfg.field_w_m, sim_cfg.field_h_m, start, goal)
    mission_v2 = None
    if competition_v2_active:
        mission_v2 = CompetitionMissionV2(
            mission_mode,
            CompetitionMissionConfig(
                field_width_m=sim_cfg.field_w_m,
                field_height_m=sim_cfg.field_h_m,
                start_x_m=start.x,
                start_y_m=start.y,
                start_yaw_deg=math.degrees(start.yaw),
                min_competition_speed_mps=min_competition_speed_mps,
                straight_distance_m=straight_distance_m,
                sweep_cell_size_m=sweep_cell_size_m,
                sweep_lane_spacing_m=sweep_lane_spacing_m,
                sweep_lane_spacing_manual_override=sweep_lane_spacing_manual_override,
                sweep_coverage_radius_m=sweep_coverage_radius_m,
                sweep_coverage_threshold=sweep_coverage_threshold,
                sweep_fail_limit=sweep_fail_limit,
                sweep_goal_timeout_s=sweep_goal_timeout_s,
                sweep_lane_tolerance_m=sweep_lane_tolerance_m,
                sweep_heading_tolerance_deg=sweep_heading_tolerance_deg,
                sweep_allow_pure_turn=sweep_allow_pure_turn,
                sweep_stall_action=sweep_stall_action,
                sweep_row_length_m=None if sweep_row_length_m <= 0.0 else sweep_row_length_m,
                sweep_headland_margin_m=sweep_headland_margin_m,
                sweep_boundary_margin_m=sweep_boundary_margin_m,
                sweep_turn_radius_m=sweep_turn_radius_m,
                sweep_turn_style=sweep_turn_style,
                sweep_row_transition_enabled=sweep_row_transition_enabled,
                sweep_max_rows=sweep_max_rows,
                sweep_row_end_tolerance_m=sweep_row_end_tolerance_m,
                sweep_turn_90_yaw_tolerance_deg=sweep_turn_90_yaw_tolerance_deg,
                sweep_turn_strong_error_deg=sweep_turn_strong_error_deg,
                sweep_turn_capture_error_deg=sweep_turn_capture_error_deg,
                sweep_turn_settle_error_deg=sweep_turn_settle_error_deg,
                sweep_turn_settle_frames=sweep_turn_settle_frames,
                sweep_pivot_min_emitted_omega_rps=sweep_pivot_min_emitted_omega_rps,
                sweep_pivot_max_emitted_omega_rps=sweep_pivot_max_emitted_omega_rps,
                sweep_pivot_wheel_min_mps=sweep_pivot_wheel_min_mps,
                sweep_pivot_settle_frames=sweep_pivot_settle_frames,
                sweep_cross_lane_v_mps=sweep_cross_lane_v_mps,
                sweep_cross_lane_heading_kp=sweep_cross_lane_heading_kp,
                sweep_lane_capture_tolerance_m=sweep_lane_capture_tolerance_m,
                sweep_yaw_capture_tolerance_deg=sweep_yaw_capture_tolerance_deg,
                sweep_transition_min_emitted_v_mps=sweep_transition_min_emitted_v_mps,
                sweep_transition_min_emitted_omega_rps=sweep_transition_min_emitted_omega_rps,
                sweep_transition_max_emitted_omega_rps=sweep_transition_max_emitted_omega_rps,
                sweep_transition_inner_wheel_min_mps=sweep_transition_inner_wheel_min_mps,
                sweep_transition_timeout_s=sweep_transition_timeout_s,
                sweep_transition_divergence_action=sweep_transition_divergence_action,
                marker_camera_fov_deg=marker_camera_fov_deg,
                marker_reliable_detection_range_m=finite_optional(marker_reliable_detection_range_m),
                marker_coverage_overlap_ratio=marker_coverage_overlap_ratio,
                marker_auto_lane_spacing_enabled=marker_auto_lane_spacing_enabled,
                target_accept_radius_m=YARD_TO_M,
                obstacle_aware=mission_mode == "round3",
            ),
            enabled=True,
        )

    control_planner = navigator.hybrid_fallback
    vis = SimVisualizer(world, navigator, robot, sensor_cfg) if show_gui else None

    true_trail = [(robot.state.true_pose.x, robot.state.true_pose.y)]
    est_trail = [(navigator.state.estimated_pose.x, navigator.state.estimated_pose.y)]
    latest_hits_world: List[Tuple[float, float]] = []
    step_i = 0
    done = False

    def sim_step(step_i_local: int):
        nonlocal latest_hits_world, done
        ts = step_i_local * sim_cfg.dt_s
        active_goal_xy = (goal.x, goal.y)
        mission_update = None
        if mission_v2 is not None:
            mission_update = mission_v2.update(
                (
                    navigator.state.estimated_pose.x,
                    navigator.state.estimated_pose.y,
                    navigator.state.estimated_pose.yaw,
                ),
                ts,
            )
            active_goal_xy = mission_update.active_goal_m
        frame = sensors.read(robot.state, active_goal_xy, ts)
        if mission_update is not None:
            frame.goal = GoalPacket(active_goal_xy[0], active_goal_xy[1], ts)
            apply_competition_v2_motion_policy(navigator, competition_v2_status_for_update(mission_update))
        cmd = navigator.step(frame)
        if mission_v2 is not None:
            cmd = apply_competition_closed_loop_command(
                navigator,
                cmd,
                frame,
                competition_v2_status_for_update(mission_update),
            )
            mission_v2.observe_navigation_feedback(navigation_feedback_for_competition_v2(navigator, cmd, frame))
            if mission_update is not None and mission_update.stop_requested:
                cmd = ControlCommand("STOP", reason=f"{mission_update.phase} stop requested: {mission_update.reason}")
        print(f"SIM CMD step={step_i_local:03d}: {json.dumps(cmd.as_dict())}")
        navigator.note_sent_command(cmd)
        robot.apply_command(cmd, world, control_planner, sim_cfg.dt_s)
        true_trail.append((robot.state.true_pose.x, robot.state.true_pose.y))
        est = navigator.state.estimated_pose
        est_trail.append((est.x, est.y))

        latest_hits_world = []
        for pt in frame.lidar.hit_points_local[:90]:
            latest_hits_world.append(navigator._local_to_world(est, pt))
        for pt in frame.zed.hit_points_local[:40]:
            latest_hits_world.append(navigator._local_to_world(est, pt))
        if vis is not None:
            vis.update(true_trail, est_trail, latest_hits_world, step_i_local)
        if cmd.mode == 'STOP' and cmd.reason == 'goal reached':
            done = True

    if show_gui and vis is not None:
        def animate(_):
            nonlocal step_i, done
            if done or step_i >= sim_cfg.max_steps:
                navigator.state.finish_reason = 'goal reached' if done else 'max steps reached'
                print(f"SIM finished: {navigator.state.finish_reason} at step {step_i}.")
                vis.update(true_trail, est_trail, latest_hits_world, step_i)
                try:
                    vis.fig.canvas.manager.set_window_title(f"UGV sim, {navigator.state.finish_reason}")
                except Exception:
                    pass
                if hasattr(animate, 'ani') and animate.ani is not None:
                    animate.ani.event_source.stop()
                return []
            sim_step(step_i)
            step_i += 1
            return []

        ani = FuncAnimation(vis.fig, animate, interval=int(sim_cfg.dt_s * 1000.0), cache_frame_data=False)
        animate.ani = ani
        plt.show()
        del ani
    else:
        for i in range(sim_cfg.max_steps):
            step_i = i
            sim_step(i)
            if done:
                break
        if not done:
            navigator.state.finish_reason = 'max steps reached'
        print(f"SIM finished: {navigator.state.finish_reason} at step {step_i}.")

    return {
        "steps": step_i,
        "replans": navigator.state.replans,
        "planner": navigator.state.planner_name,
        "plan_time_ms": navigator.state.plan_time_ms,
        "finish_reason": navigator.state.finish_reason,
        "final_true_pose": robot.state.true_pose,
        "final_estimated_pose": navigator.state.estimated_pose,
        "latest_cmd": navigator.state.latest_cmd.as_dict(),
    }


def run_real_mode(
    replay_json: Optional[str] = None,
    competition_mode: bool = False,
    mission_mode: str = "manual",
    start_corner: str = "lower_left",
    start_x_m: Optional[float] = None,
    start_y_m: Optional[float] = None,
    start_yaw_deg: Optional[float] = None,
    center_loiter_radius_m: float = 0.75,
    target_accept_radius_m: float = YARD_TO_M,
    straight_distance_m: float = yd(13.0),
    competition_mission_v2_enabled: bool = True,
    sweep_field_width_m: float = yd(15.0),
    sweep_field_height_m: float = yd(15.0),
    sweep_row_length_m: float = 0.0,
    sweep_headland_margin_m: float = 0.75,
    sweep_boundary_margin_m: float = 0.45,
    sweep_turn_radius_m: float = 1.00,
    sweep_turn_style: str = "segmented_90",
    sweep_row_transition_enabled: bool = False,
    sweep_max_rows: int = 0,
    sweep_row_end_tolerance_m: float = 0.35,
    sweep_turn_90_yaw_tolerance_deg: float = 7.0,
    sweep_turn_strong_error_deg: float = 30.0,
    sweep_turn_capture_error_deg: float = 12.0,
    sweep_turn_settle_error_deg: float = 7.0,
    sweep_turn_settle_frames: int = 3,
    sweep_pivot_min_emitted_omega_rps: float = 0.25,
    sweep_pivot_max_emitted_omega_rps: float = 0.45,
    sweep_pivot_wheel_min_mps: float = 0.08,
    sweep_pivot_settle_frames: int = 3,
    sweep_cross_lane_v_mps: float = 0.12,
    sweep_cross_lane_heading_kp: float = 1.10,
    sweep_lane_capture_tolerance_m: float = 0.20,
    sweep_yaw_capture_tolerance_deg: float = 8.0,
    sweep_transition_min_emitted_v_mps: float = 0.12,
    sweep_transition_min_emitted_omega_rps: float = 0.20,
    sweep_transition_max_emitted_omega_rps: float = 0.40,
    sweep_transition_inner_wheel_min_mps: float = 0.04,
    sweep_transition_timeout_s: float = 20.0,
    sweep_transition_divergence_action: str = "stop",
    sweep_cell_size_m: float = 0.75,
    sweep_lane_spacing_m: float = 0.75,
    sweep_lane_spacing_manual_override: bool = False,
    sweep_coverage_radius_m: float = 0.55,
    sweep_coverage_threshold: float = 0.85,
    sweep_goal_timeout_s: float = 8.0,
    sweep_fail_limit: int = 3,
    sweep_lane_tolerance_m: float = 0.30,
    sweep_heading_tolerance_deg: float = 25.0,
    sweep_allow_pure_turn: bool = False,
    sweep_stall_action: str = "skip",
    marker_camera_fov_deg: float = 120.0,
    marker_reliable_detection_range_m: float = float("nan"),
    marker_coverage_overlap_ratio: float = 0.5,
    marker_auto_lane_spacing_enabled: bool = False,
    min_competition_speed_mps: float = 0.0894,
    field_map_topic: str = "/ugv/field_map",
    target_topic: str = "/ugv/target",
    marker_topic: str = "/ugv/marker_detection",
    mission_flag_topic: str = "/ugv/mission_flag",
    nav_status_period_s: float = 1.0,
    use_imu_yaw: bool = False,
    imu_yaw_blend: float = 0.25,
    imu_yaw_axis: str = "z",
    imu_yaw_sign: float = 1.0,
    robot_length_m: float = ft(30.0 / 12.0),
    robot_width_m: float = ft(30.0 / 12.0),
    robot_track_width_m: float = ft(2.0),
    robot_wheel_radius_m: float = 0.06,
    robot_ticks_per_rev: int = 1000,
    robot_obstacle_buffer_m: float = 0.025,
    lidar_offset_x_m: float = 0.30,
    lidar_offset_y_m: float = 0.0,
    lidar_used_fov_deg: float = 180.0,
    allow_reverse: bool = False,
    min_motion_raw: float = 0.22,
    min_speed_mps: float = 0.178816,
    drive_speed_level: int = 4,
    front_safety_margin_m: float = 0.08,
    rear_safety_margin_m: float = 0.08,
    local_plan_inflation_m: float = 0.0,
    active_scan_enabled: bool = True,
    active_scan_confirm_steps: int = 3,
    active_scan_steps: int = 7,
    active_scan_panoramic_steps: int = 20,
    active_scan_cooldown_steps: int = 4,
    active_scan_probe_steps: int = 5,
    active_scan_front_clear_m: float = 1.25,
    active_scan_corridor_extra_width_m: float = 0.03,
    continuous_control_enabled: bool = True,
    continuous_max_speed_mps: float = 0.36,
    continuous_max_omega_rps: float = 1.15,
    continuous_horizon_s: float = 1.35,
    continuous_accel_limit_mps2: float = 0.35,
    continuous_omega_accel_limit_rps2: float = 1.80,
    continuous_lowpass_alpha: float = 0.55,
    continuous_raw_per_mps: float = 1.35,
    continuous_slowdown_clearance_m: float = 1.20,
    continuous_stop_clearance_m: float = 0.48,
    continuous_gap_buffer_m: float = 0.025,
    continuous_latency_buffer_s: float = 0.25,
    continuous_allow_costmap_soft_penalty: bool = False,
    emit_velocity_commands: bool = True,
    competition_closed_loop_enabled: bool = True,
    heading_hold_enabled: bool = True,
    lane_follow_enabled: bool = True,
    heading_hold_kp: float = 1.10,
    heading_hold_kd: float = 0.05,
    heading_hold_deadband_deg: float = 3.0,
    heading_hold_max_omega_rps: float = 0.55,
    lane_follow_kp_heading: float = 1.10,
    lane_follow_kp_omega: float = 1.00,
    lane_follow_deadband_m: float = 0.03,
    lane_follow_max_heading_deg: float = 18.0,
    lane_follow_max_omega_rps: float = 0.35,
    row_follower_speed_schedule_enabled: bool = True,
    row_follower_low_speed_mps: float = 0.09,
    row_follower_high_speed_mps: float = 0.22,
    row_follower_low_speed_lane_kp: float = 0.85,
    row_follower_high_speed_lane_kp: float = 1.00,
    row_follower_low_speed_heading_kp: float = 0.95,
    row_follower_high_speed_heading_kp: float = 1.10,
    row_follower_omega_low_pass_alpha: float = 0.35,
    row_follower_omega_rate_limit_rps2: float = 0.60,
    row_follower_min_correction_interval_s: float = 0.0,
    sweep_planner_omega_weight_round2: float = 0.0,
    sweep_planner_omega_weight_round3: float = 0.2,
    sweep_metrics_log_enabled: bool = False,
    sweep_metrics_log_dir: str = "~/.ros/ugv_sweep_metrics",
    forward_arc_only_enabled: bool = True,
    forward_arc_margin: float = 0.75,
    min_sweep_v_mps: float = 0.08,
    omega_command_sign: float = 1.0,
    heading_error_sign: float = 1.0,
    lane_error_sign: float = 1.0,
    lane_correction_sign: float = 1.0,
    closed_loop_health_enabled: bool = True,
    closed_loop_divergence_window: int = 5,
    closed_loop_divergence_min_error_m: float = 0.08,
    closed_loop_divergence_max_growth_m: float = 0.05,
    closed_loop_divergence_action: str = "slow_then_stop",
    local_costmap_enabled: bool = True,
    local_costmap_width_m: float = 4.0,
    local_costmap_height_m: float = 4.0,
    local_costmap_resolution_m: float = 0.06,
    local_costmap_dynamic_decay_s: float = 1.00,
    local_costmap_obstacle_radius_m: float = 0.06,
    local_costmap_inflation_m: float = 0.08,
    local_costmap_lidar_clear_radius_m: float = 0.05,
    local_costmap_max_raytrace_m: float = 4.0,
    recovery_turn_raw: float = 0.32,
    max_steps: int = 0,
) -> None:
    mission_mode = normalize_mission_mode(mission_mode, competition_mode)
    competition_mode = mission_mode == "round3"
    competition_v2_active = bool(competition_mission_v2_enabled) and mission_mode in {"round1", "round2", "round3"}
    robot_cfg = RobotConfig()
    robot_cfg.length_m = max(0.20, float(robot_length_m))
    robot_cfg.width_m = max(0.20, float(robot_width_m))
    robot_cfg.track_width_m = max(0.20, float(robot_track_width_m))
    robot_cfg.wheel_radius_m = max(0.005, float(robot_wheel_radius_m))
    robot_cfg.ticks_per_rev = max(1, int(robot_ticks_per_rev))
    robot_cfg.obstacle_buffer_m = clamp(float(robot_obstacle_buffer_m), 0.0, 0.20)
    robot_cfg.lidar_offset_x_m = float(lidar_offset_x_m)
    robot_cfg.lidar_offset_y_m = float(lidar_offset_y_m)
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    nav_cfg.allow_stop_at_goal = (not competition_v2_active) and mission_mode not in {"round3", "indoor"}
    nav_cfg.nonstop_when_blocked = mission_mode in {"round3", "indoor"}
    nav_cfg.allow_reverse = bool(allow_reverse)
    nav_cfg.lidar_used_fov_deg = clamp(float(lidar_used_fov_deg), 1.0, 360.0)
    nav_cfg.use_imu_yaw = bool(use_imu_yaw)
    nav_cfg.imu_yaw_blend = clamp(float(imu_yaw_blend), 0.0, 1.0)
    nav_cfg.imu_yaw_axis = str(imu_yaw_axis).lower()
    nav_cfg.imu_yaw_sign = 1.0 if float(imu_yaw_sign) >= 0.0 else -1.0
    nav_cfg.min_motion_raw = clamp(float(min_motion_raw), 0.0, 1.0)
    nav_cfg.front_safety_margin_m = clamp(float(front_safety_margin_m), 0.0, 1.0)
    nav_cfg.rear_safety_margin_m = clamp(float(rear_safety_margin_m), 0.0, 1.0)
    nav_cfg.local_plan_inflation_m = clamp(float(local_plan_inflation_m), 0.0, 1.0)
    nav_cfg.active_scan_enabled = bool(active_scan_enabled)
    nav_cfg.active_scan_confirm_steps = max(1, int(active_scan_confirm_steps))
    nav_cfg.active_scan_steps = max(1, int(active_scan_steps))
    nav_cfg.active_scan_panoramic_steps = max(1, int(active_scan_panoramic_steps))
    nav_cfg.active_scan_cooldown_steps = max(0, int(active_scan_cooldown_steps))
    nav_cfg.active_scan_probe_steps = max(0, int(active_scan_probe_steps))
    nav_cfg.active_scan_front_clear_m = clamp(float(active_scan_front_clear_m), 0.50, 3.0)
    nav_cfg.active_scan_corridor_extra_width_m = clamp(float(active_scan_corridor_extra_width_m), 0.0, 0.50)
    nav_cfg.continuous_control_enabled = bool(continuous_control_enabled)
    nav_cfg.continuous_max_speed_mps = clamp(float(continuous_max_speed_mps), 0.05, 1.0)
    nav_cfg.continuous_max_omega_rps = clamp(float(continuous_max_omega_rps), 0.20, 3.0)
    nav_cfg.continuous_horizon_s = clamp(float(continuous_horizon_s), 0.40, 2.0)
    nav_cfg.continuous_accel_limit_mps2 = clamp(float(continuous_accel_limit_mps2), 0.05, 2.0)
    nav_cfg.continuous_omega_accel_limit_rps2 = clamp(float(continuous_omega_accel_limit_rps2), 0.20, 5.0)
    nav_cfg.continuous_lowpass_alpha = clamp(float(continuous_lowpass_alpha), 0.05, 1.0)
    nav_cfg.continuous_raw_per_mps = clamp(float(continuous_raw_per_mps), 0.20, 4.0)
    nav_cfg.continuous_slowdown_clearance_m = clamp(float(continuous_slowdown_clearance_m), 0.50, 3.0)
    nav_cfg.continuous_stop_clearance_m = clamp(float(continuous_stop_clearance_m), 0.30, 1.50)
    nav_cfg.continuous_gap_buffer_m = clamp(float(continuous_gap_buffer_m), 0.0, 0.30)
    nav_cfg.continuous_latency_buffer_s = clamp(float(continuous_latency_buffer_s), 0.0, 1.0)
    nav_cfg.continuous_allow_costmap_soft_penalty = bool(continuous_allow_costmap_soft_penalty)
    nav_cfg.emit_velocity_commands = bool(emit_velocity_commands)
    nav_cfg.competition_closed_loop_enabled = bool(competition_closed_loop_enabled)
    nav_cfg.heading_hold_enabled = bool(heading_hold_enabled)
    nav_cfg.lane_follow_enabled = bool(lane_follow_enabled)
    nav_cfg.heading_hold_kp = clamp(float(heading_hold_kp), 0.0, 8.0)
    nav_cfg.heading_hold_kd = clamp(float(heading_hold_kd), 0.0, 4.0)
    nav_cfg.heading_hold_deadband_deg = clamp(float(heading_hold_deadband_deg), 0.0, 45.0)
    nav_cfg.heading_hold_max_omega_rps = clamp(float(heading_hold_max_omega_rps), 0.0, 3.0)
    nav_cfg.lane_follow_kp_heading = clamp(float(lane_follow_kp_heading), 0.0, 8.0)
    nav_cfg.lane_follow_kp_omega = clamp(float(lane_follow_kp_omega), 0.0, 8.0)
    nav_cfg.lane_follow_deadband_m = clamp(float(lane_follow_deadband_m), 0.0, 1.0)
    nav_cfg.lane_follow_max_heading_deg = clamp(float(lane_follow_max_heading_deg), 0.0, 60.0)
    nav_cfg.lane_follow_max_omega_rps = clamp(float(lane_follow_max_omega_rps), 0.0, 3.0)
    nav_cfg.row_follower_speed_schedule_enabled = bool(row_follower_speed_schedule_enabled)
    nav_cfg.row_follower_low_speed_mps = clamp(float(row_follower_low_speed_mps), 0.0, 2.0)
    nav_cfg.row_follower_high_speed_mps = clamp(float(row_follower_high_speed_mps), 0.0, 2.0)
    nav_cfg.row_follower_low_speed_lane_kp = clamp(float(row_follower_low_speed_lane_kp), 0.0, 8.0)
    nav_cfg.row_follower_high_speed_lane_kp = clamp(float(row_follower_high_speed_lane_kp), 0.0, 8.0)
    nav_cfg.row_follower_low_speed_heading_kp = clamp(float(row_follower_low_speed_heading_kp), 0.0, 8.0)
    nav_cfg.row_follower_high_speed_heading_kp = clamp(float(row_follower_high_speed_heading_kp), 0.0, 8.0)
    nav_cfg.row_follower_omega_low_pass_alpha = clamp(float(row_follower_omega_low_pass_alpha), 0.0, 1.0)
    nav_cfg.row_follower_omega_rate_limit_rps2 = clamp(float(row_follower_omega_rate_limit_rps2), 0.0, 10.0)
    nav_cfg.row_follower_min_correction_interval_s = clamp(float(row_follower_min_correction_interval_s), 0.0, 1.0)
    nav_cfg.sweep_planner_omega_weight_round2 = clamp(float(sweep_planner_omega_weight_round2), 0.0, 1.0)
    nav_cfg.sweep_planner_omega_weight_round3 = clamp(float(sweep_planner_omega_weight_round3), 0.0, 1.0)
    nav_cfg.sweep_metrics_log_enabled = bool(sweep_metrics_log_enabled)
    nav_cfg.sweep_metrics_log_dir = str(sweep_metrics_log_dir)
    nav_cfg.forward_arc_only_enabled = bool(forward_arc_only_enabled)
    nav_cfg.forward_arc_margin = clamp(float(forward_arc_margin), 0.0, 1.0)
    nav_cfg.min_sweep_v_mps = clamp(float(min_sweep_v_mps), 0.0, 1.0)
    nav_cfg.omega_command_sign = 1.0 if float(omega_command_sign) >= 0.0 else -1.0
    nav_cfg.heading_error_sign = 1.0 if float(heading_error_sign) >= 0.0 else -1.0
    nav_cfg.lane_error_sign = 1.0 if float(lane_error_sign) >= 0.0 else -1.0
    nav_cfg.lane_correction_sign = 1.0 if float(lane_correction_sign) >= 0.0 else -1.0
    nav_cfg.closed_loop_health_enabled = bool(closed_loop_health_enabled)
    nav_cfg.closed_loop_divergence_window = max(2, int(closed_loop_divergence_window))
    nav_cfg.closed_loop_divergence_min_error_m = clamp(float(closed_loop_divergence_min_error_m), 0.0, 2.0)
    nav_cfg.closed_loop_divergence_max_growth_m = clamp(float(closed_loop_divergence_max_growth_m), 0.0, 2.0)
    nav_cfg.closed_loop_divergence_action = str(closed_loop_divergence_action).strip().lower()
    if nav_cfg.closed_loop_divergence_action not in {"warn", "slow", "slow_then_stop", "stop"}:
        nav_cfg.closed_loop_divergence_action = "slow_then_stop"
    nav_cfg.local_costmap_enabled = bool(local_costmap_enabled)
    nav_cfg.local_costmap_width_m = clamp(float(local_costmap_width_m), 1.0, 10.0)
    nav_cfg.local_costmap_height_m = clamp(float(local_costmap_height_m), 1.0, 10.0)
    nav_cfg.local_costmap_resolution_m = clamp(float(local_costmap_resolution_m), 0.02, 0.25)
    nav_cfg.local_costmap_dynamic_decay_s = clamp(float(local_costmap_dynamic_decay_s), 0.0, 10.0)
    nav_cfg.local_costmap_obstacle_radius_m = clamp(float(local_costmap_obstacle_radius_m), 0.01, 0.40)
    nav_cfg.local_costmap_inflation_m = clamp(float(local_costmap_inflation_m), 0.0, 0.80)
    nav_cfg.local_costmap_lidar_clear_radius_m = clamp(float(local_costmap_lidar_clear_radius_m), 0.0, 0.40)
    nav_cfg.local_costmap_max_raytrace_m = clamp(float(local_costmap_max_raytrace_m), 0.20, 12.0)
    nav_cfg.recovery_turn_raw = clamp(float(recovery_turn_raw), 0.0, 1.0)
    drive_speed_level = normalize_drive_speed_level(drive_speed_level)
    drive_factor = drive_speed_factor(drive_speed_level)
    if competition_v2_active:
        nav_cfg.continuous_min_speed_mps = clamp(
            max(
                nav_cfg.continuous_min_speed_mps,
                float(min_competition_speed_mps) / max(drive_factor, 1e-6),
            ),
            0.02,
            nav_cfg.continuous_max_speed_mps,
        )
    field_w_m = max(0.5, float(sweep_field_width_m))
    field_h_m = max(0.5, float(sweep_field_height_m))
    custom_start_x = finite_optional(start_x_m)
    custom_start_y = finite_optional(start_y_m)
    custom_start_yaw = finite_optional(start_yaw_deg)
    if (custom_start_x is None) != (custom_start_y is None):
        raise ValueError("Both --start-x-m and --start-y-m are required when overriding the UGV start pose.")
    if custom_start_x is not None and custom_start_y is not None:
        init_pose = start_pose_from_xy(custom_start_x, custom_start_y, field_w_m, field_h_m, custom_start_yaw)
    elif mission_mode == "round3":
        init_pose = start_pose_for_corner(start_corner, field_w_m, field_h_m)
    if mission_mode == "round3":
        init_goal = Pose2D(0.5 * field_w_m, 0.5 * field_h_m, 0.0)
    elif mission_mode == "indoor":
        if custom_start_x is None or custom_start_y is None:
            init_pose = Pose2D(0.5 * field_w_m, 0.5 * field_h_m, 0.0)
        init_goal = Pose2D(
            clamp(init_pose.x + 2.0 * math.cos(init_pose.yaw), 0.65, field_w_m - 0.65),
            clamp(init_pose.y + 2.0 * math.sin(init_pose.yaw), 0.65, field_h_m - 0.65),
            init_pose.yaw,
        )
    elif mission_mode in {"round1", "round2"}:
        if custom_start_x is None or custom_start_y is None:
            init_pose = Pose2D(yd(0.5), yd(0.5), 0.0)
        init_goal = Pose2D(
            clamp(init_pose.x + max(0.5, float(straight_distance_m)), 0.45, field_w_m - 0.45),
            clamp(init_pose.y, 0.45, field_h_m - 0.45),
            0.0,
        )
    else:
        if custom_start_x is None or custom_start_y is None:
            init_pose = Pose2D(yd(0.5), yd(0.5), 0.0)
        init_goal = Pose2D(yd(13.4), yd(13.0), 0.0)
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, field_w_m, field_h_m, init_pose, init_goal)
    if competition_v2_active:
        mission = CompetitionMissionV2(
            mission_mode,
            CompetitionMissionConfig(
                field_width_m=field_w_m,
                field_height_m=field_h_m,
                start_x_m=init_pose.x,
                start_y_m=init_pose.y,
                start_yaw_deg=math.degrees(init_pose.yaw),
                min_competition_speed_mps=min_competition_speed_mps,
                straight_distance_m=straight_distance_m,
                sweep_cell_size_m=sweep_cell_size_m,
                sweep_lane_spacing_m=sweep_lane_spacing_m,
                sweep_lane_spacing_manual_override=sweep_lane_spacing_manual_override,
                sweep_coverage_radius_m=sweep_coverage_radius_m,
                sweep_coverage_threshold=sweep_coverage_threshold,
                sweep_fail_limit=sweep_fail_limit,
                sweep_goal_timeout_s=sweep_goal_timeout_s,
                sweep_lane_tolerance_m=sweep_lane_tolerance_m,
                sweep_heading_tolerance_deg=sweep_heading_tolerance_deg,
                sweep_allow_pure_turn=sweep_allow_pure_turn,
                sweep_stall_action=sweep_stall_action,
                sweep_row_length_m=None if sweep_row_length_m <= 0.0 else sweep_row_length_m,
                sweep_headland_margin_m=sweep_headland_margin_m,
                sweep_boundary_margin_m=sweep_boundary_margin_m,
                sweep_turn_radius_m=sweep_turn_radius_m,
                sweep_turn_style=sweep_turn_style,
                sweep_row_transition_enabled=sweep_row_transition_enabled,
                sweep_max_rows=sweep_max_rows,
                sweep_row_end_tolerance_m=sweep_row_end_tolerance_m,
                sweep_turn_90_yaw_tolerance_deg=sweep_turn_90_yaw_tolerance_deg,
                sweep_turn_strong_error_deg=sweep_turn_strong_error_deg,
                sweep_turn_capture_error_deg=sweep_turn_capture_error_deg,
                sweep_turn_settle_error_deg=sweep_turn_settle_error_deg,
                sweep_turn_settle_frames=sweep_turn_settle_frames,
                sweep_pivot_min_emitted_omega_rps=sweep_pivot_min_emitted_omega_rps,
                sweep_pivot_max_emitted_omega_rps=sweep_pivot_max_emitted_omega_rps,
                sweep_pivot_wheel_min_mps=sweep_pivot_wheel_min_mps,
                sweep_pivot_settle_frames=sweep_pivot_settle_frames,
                sweep_cross_lane_v_mps=sweep_cross_lane_v_mps,
                sweep_cross_lane_heading_kp=sweep_cross_lane_heading_kp,
                sweep_lane_capture_tolerance_m=sweep_lane_capture_tolerance_m,
                sweep_yaw_capture_tolerance_deg=sweep_yaw_capture_tolerance_deg,
                sweep_transition_min_emitted_v_mps=sweep_transition_min_emitted_v_mps,
                sweep_transition_min_emitted_omega_rps=sweep_transition_min_emitted_omega_rps,
                sweep_transition_max_emitted_omega_rps=sweep_transition_max_emitted_omega_rps,
                sweep_transition_inner_wheel_min_mps=sweep_transition_inner_wheel_min_mps,
                sweep_transition_timeout_s=sweep_transition_timeout_s,
                sweep_transition_divergence_action=sweep_transition_divergence_action,
                marker_camera_fov_deg=marker_camera_fov_deg,
                marker_reliable_detection_range_m=finite_optional(marker_reliable_detection_range_m),
                marker_coverage_overlap_ratio=marker_coverage_overlap_ratio,
                marker_auto_lane_spacing_enabled=marker_auto_lane_spacing_enabled,
                target_accept_radius_m=target_accept_radius_m,
                obstacle_aware=mission_mode == "round3",
                use_legacy_center_expand=False,
            ),
            enabled=True,
        )
    else:
        mission = CompetitionMission(
            enabled=mission_mode != "manual",
            mission_mode=mission_mode,
            start_corner=start_corner,
            field_w_m=field_w_m,
            field_h_m=field_h_m,
            start_pose=init_pose,
            center_loiter_radius_m=center_loiter_radius_m,
            target_accept_radius_m=target_accept_radius_m,
            min_speed_mps=min_speed_mps,
            straight_distance_m=straight_distance_m,
            lidar_used_fov_deg=nav_cfg.lidar_used_fov_deg,
            lidar_offset_x_m=robot_cfg.lidar_offset_x_m,
            lidar_offset_y_m=robot_cfg.lidar_offset_y_m,
        )

    if replay_json:
        bridge: RealRobotBridgeBase = JsonReplayBridge(replay_json)
    else:
        bridge = Ros2Bridge(
            allow_missing_goal=mission_mode != "manual",
            field_map_topic=field_map_topic,
            target_topic=target_topic,
            marker_topic=marker_topic,
            mission_flag_topic=mission_flag_topic,
        )

    print("Running REAL mode. No GUI. Printing and sending control commands.")
    print(
        f"Mission mode: {mission_mode}, competition_mode={competition_mode}, "
        f"competition_mission_v2={competition_v2_active}, "
        f"start_corner={normalize_corner_name(start_corner)}, "
        f"start_pose=({init_pose.x:.2f}, {init_pose.y:.2f}, {math.degrees(init_pose.yaw):.1f}deg), "
        f"field=({field_w_m:.2f}m x {field_h_m:.2f}m), "
        f"straight_distance_m={straight_distance_m:.2f}, "
        f"sweep_cell={sweep_cell_size_m:.2f}m lane={sweep_lane_spacing_m:.2f}m "
        f"row_length={sweep_row_length_m:.2f}m headland={sweep_headland_margin_m:.2f}m "
        f"boundary={sweep_boundary_margin_m:.2f}m turn_style={sweep_turn_style} "
        f"turn_radius={sweep_turn_radius_m:.2f}m "
        f"row_transition={sweep_row_transition_enabled} max_rows={sweep_max_rows}, "
        f"coverage_radius={sweep_coverage_radius_m:.2f}m threshold={sweep_coverage_threshold:.2f}, "
        f"lane_tol={sweep_lane_tolerance_m:.2f}m heading_tol={sweep_heading_tolerance_deg:.1f}deg "
        f"pure_turn={sweep_allow_pure_turn}, "
        f"min_competition_speed={min_competition_speed_mps:.3f}m/s, "
        f"drive_speed_level={drive_speed_level}/4 ({drive_factor:.2f}x), "
        f"robot=({robot_cfg.length_m:.2f}m x {robot_cfg.width_m:.2f}m track={robot_cfg.track_width_m:.4f}m), "
        f"lidar_offset=({robot_cfg.lidar_offset_x_m:.2f}, {robot_cfg.lidar_offset_y_m:.2f})m, "
        f"lidar_fov={nav_cfg.lidar_used_fov_deg:.0f}deg, allow_reverse={nav_cfg.allow_reverse}, "
        f"active_scan={nav_cfg.active_scan_enabled} "
        f"(steps={nav_cfg.active_scan_steps}, panoramic={nav_cfg.active_scan_panoramic_steps}, "
        f"cooldown={nav_cfg.active_scan_cooldown_steps}, "
        f"probe={nav_cfg.active_scan_probe_steps}), "
        f"continuous_control={nav_cfg.continuous_control_enabled} "
        f"(vmax={nav_cfg.continuous_max_speed_mps:.2f}m/s, "
        f"omax={nav_cfg.continuous_max_omega_rps:.2f}rad/s, "
        f"horizon={nav_cfg.continuous_horizon_s:.2f}s, "
        f"emit_velocity_commands={nav_cfg.emit_velocity_commands}, "
        f"competition_closed_loop={nav_cfg.competition_closed_loop_enabled}, "
        f"heading_hold={nav_cfg.heading_hold_enabled}, "
        f"heading_kp={nav_cfg.heading_hold_kp:.2f}, heading_kd={nav_cfg.heading_hold_kd:.2f}, "
        f"heading_deadband={nav_cfg.heading_hold_deadband_deg:.1f}deg, "
        f"heading_omax={nav_cfg.heading_hold_max_omega_rps:.2f}rad/s, "
        f"lane_follow={nav_cfg.lane_follow_enabled}, "
        f"lane_kp_heading={nav_cfg.lane_follow_kp_heading:.2f}, "
        f"lane_kp_omega={nav_cfg.lane_follow_kp_omega:.2f}, "
        f"lane_deadband={nav_cfg.lane_follow_deadband_m:.2f}m, "
        f"lane_heading_max={nav_cfg.lane_follow_max_heading_deg:.1f}deg, "
        f"lane_omax={nav_cfg.lane_follow_max_omega_rps:.2f}rad/s, "
        f"row_schedule={nav_cfg.row_follower_speed_schedule_enabled} "
        f"(low={nav_cfg.row_follower_low_speed_mps:.2f}m/s, "
        f"high={nav_cfg.row_follower_high_speed_mps:.2f}m/s, "
        f"lane_kp={nav_cfg.row_follower_low_speed_lane_kp:.2f}->{nav_cfg.row_follower_high_speed_lane_kp:.2f}, "
        f"heading_kp={nav_cfg.row_follower_low_speed_heading_kp:.2f}->{nav_cfg.row_follower_high_speed_heading_kp:.2f}, "
        f"omega_alpha={nav_cfg.row_follower_omega_low_pass_alpha:.2f}, "
        f"omega_rate={nav_cfg.row_follower_omega_rate_limit_rps2:.2f}rad/s^2), "
        f"planner_omega_weight_round2={nav_cfg.sweep_planner_omega_weight_round2:.2f}, "
        f"planner_omega_weight_round3={nav_cfg.sweep_planner_omega_weight_round3:.2f}, "
        f"forward_arc_only={nav_cfg.forward_arc_only_enabled}, "
        f"forward_arc_margin={nav_cfg.forward_arc_margin:.2f}, "
        f"min_sweep_v={nav_cfg.min_sweep_v_mps:.2f}m/s, "
        f"omega_sign={nav_cfg.omega_command_sign:.0f}, "
        f"heading_sign={nav_cfg.heading_error_sign:.0f}, "
        f"lane_error_sign={nav_cfg.lane_error_sign:.0f}, "
        f"lane_correction_sign={nav_cfg.lane_correction_sign:.0f}, "
        f"health={nav_cfg.closed_loop_health_enabled} "
        f"(window={nav_cfg.closed_loop_divergence_window}, "
        f"min_error={nav_cfg.closed_loop_divergence_min_error_m:.2f}m, "
        f"max_growth={nav_cfg.closed_loop_divergence_max_growth_m:.2f}m, "
        f"action={nav_cfg.closed_loop_divergence_action}), "
        f"local_costmap={nav_cfg.local_costmap_enabled})"
    )
    print("Expected ROS2 topics if using Ros2Bridge:")
    print("  /sensors/nav_frame    ugv_sensor_sync/msg/NavSensorFrame")
    print("                        includes scan, zed obstacle points, latest encoder ticks, and clearance summaries")
    print("  /ugv_goal             geometry_msgs/PointStamped in map frame")
    print(f"  {target_topic}       std_msgs/String JSON target x/y in meters from bottom-left field origin")
    print(f"  {field_map_topic}       optional legacy std_msgs/String JSON 15x15 matrix")
    print(f"  {marker_topic}  geometry_msgs/PointStamped marker found by camera/CV")
    print(f"  {mission_flag_topic}     std_msgs/String JSON/plain UAV state, e.g. landing/leaving/scanning")
    print("Publishing:")
    print("  /ugv_nav_cmd          std_msgs/String with JSON control command")
    print("  /ugv_nav_status       std_msgs/String live nav/debug status")
    print("  /ugv/uav_flag         std_msgs/String target-found flag for ESP/UAV handoff")

    metrics_logger = None
    if nav_cfg.sweep_metrics_log_enabled:
        metrics_logger = SweepMetricsLogger(nav_cfg.sweep_metrics_log_dir)
        print(f"Sweep metrics logging enabled: {metrics_logger.path}")

    last_status_s = 0.0
    last_metrics_summary_s = 0.0
    steps = 0
    try:
        while True:
            frame = bridge.read_frame()
            if frame is None:
                if replay_json:
                    print("REAL replay finished: end of replay file.")
                    break
                time.sleep(0.02)
                continue
            if competition_v2_active:
                marker_goal = None
                marker_distance_m = None
                if frame.marker_goal is not None:
                    marker_goal = (frame.marker_goal.x, frame.marker_goal.y)
                    marker_distance_m = frame.marker_goal.distance_m
                field_map_goal = frame.field_map.goal_xy if frame.field_map is not None else None
                field_map_source = frame.field_map.source if frame.field_map is not None else None
                mission_flag_state = frame.mission_flag.state if frame.mission_flag is not None else None
                mission_flag_source = frame.mission_flag.source if frame.mission_flag is not None else None
                update = mission.update(
                    (
                        navigator.state.estimated_pose.x,
                        navigator.state.estimated_pose.y,
                        navigator.state.estimated_pose.yaw,
                    ),
                    frame.encoder.timestamp,
                    marker_goal=marker_goal,
                    marker_distance_m=marker_distance_m,
                    field_map_goal=field_map_goal,
                    field_map_source=field_map_source,
                    mission_flag_state=mission_flag_state,
                    mission_flag_source=mission_flag_source,
                )
                frame.goal = GoalPacket(update.active_goal_m[0], update.active_goal_m[1], frame.encoder.timestamp, marker_distance_m)
                mission_status = competition_v2_status_for_update(update)
                apply_competition_v2_motion_policy(navigator, mission_status)
            else:
                mission_status = dict(mission.update_frame(frame, navigator.state.estimated_pose))
                apply_competition_v2_motion_policy(navigator, {})
            speed_scale, speed_reason = mission.command_speed_scale()
            effective_speed_scale = speed_scale * drive_factor
            mission_status["drive_speed_level"] = drive_speed_level
            mission_status["drive_speed_factor"] = round(drive_factor, 3)
            mission_status["effective_speed_scale"] = round(effective_speed_scale, 3)
            mission_status["min_motion_raw"] = round(nav_cfg.min_motion_raw, 3)
            cmd = navigator.step(frame)
            if competition_v2_active:
                cmd = apply_competition_closed_loop_command(navigator, cmd, frame, mission_status)
                feedback_changed = mission.observe_navigation_feedback(
                    navigation_feedback_for_competition_v2(navigator, cmd, frame)
                )
                if feedback_changed:
                    mission_status = competition_v2_status_for_update(mission.current_update())
                    mission_status["drive_speed_level"] = drive_speed_level
                    mission_status["drive_speed_factor"] = round(drive_factor, 3)
                    mission_status["effective_speed_scale"] = round(effective_speed_scale, 3)
                    mission_status["min_motion_raw"] = round(nav_cfg.min_motion_raw, 3)
            speed_scale, speed_reason = mission.command_speed_scale()
            effective_speed_scale = speed_scale * drive_factor
            mission_status["drive_speed_level"] = drive_speed_level
            mission_status["drive_speed_factor"] = round(drive_factor, 3)
            mission_status["effective_speed_scale"] = round(effective_speed_scale, 3)
            mission_status["min_motion_raw"] = round(nav_cfg.min_motion_raw, 3)
            if mission_status.get("stop_requested"):
                cmd = ControlCommand(
                    "STOP",
                    reason=f"{mission_status.get('phase', 'target')} stop requested: {mission_status.get('speed_reason', mission_status.get('final_goal_source', 'mission'))}",
                )
            combined_reason = speed_reason
            if drive_speed_level != 4:
                combined_reason = f"{combined_reason}; " if combined_reason else ""
                combined_reason += f"drive_level_{drive_speed_level}"
            cmd = scale_control_command(
                cmd,
                effective_speed_scale,
                combined_reason,
                nav_cfg.min_motion_raw,
            )
            navigator.note_sent_command(cmd)
            bridge.send_control(cmd)
            now_s = time.monotonic()
            should_publish_status = now_s - last_status_s >= max(nav_status_period_s, 0.2)
            if metrics_logger is not None or should_publish_status:
                status = build_nav_status(navigator, frame, cmd, mission_status)
                if metrics_logger is not None:
                    metrics_logger.record(status, getattr(bridge, "latest_motor_status", {}))
                    if now_s - last_metrics_summary_s >= 10.0:
                        print("SWEEP METRICS", json.dumps(metrics_logger.summary()))
                        last_metrics_summary_s = now_s
                if should_publish_status:
                    bridge.publish_nav_status(status)
                    last_status_s = now_s
            steps += 1
            if replay_json and max_steps > 0 and steps >= max_steps:
                print(f"REAL replay finished: max steps reached at step {steps - 1}.")
                break
    finally:
        if metrics_logger is not None:
            print("SWEEP METRICS FINAL", json.dumps(metrics_logger.summary()))
            metrics_logger.close()


# =========================================================
# CLI
# =========================================================

def str_to_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y", "on"}


def competition_mode_cli_value(value: str) -> Tuple[bool, Optional[str]]:
    text = str(value or "false").strip().lower().replace("-", "_").replace(" ", "_")
    if text in {"1", "true", "yes", "y", "on"}:
        return True, None
    if text in {"0", "false", "no", "n", "off", ""}:
        return False, None
    try:
        return True, normalize_mission_mode(text)
    except ValueError:
        return str_to_bool(text), None


def main() -> None:
    parser = argparse.ArgumentParser(description="UGV dual-mode navigation bridge, sim mode + real mode")
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument("--headless", action="store_true", help="sim mode without matplotlib GUI")
    parser.add_argument("--max-steps", type=int, default=900)
    parser.add_argument("--replay-json", type=str, default=None, help="use JSONL replay file for real mode testing")
    parser.add_argument("--competition-mode", type=str, default="false", help="enable competition mode; accepts true/false or round1/round2/round3")
    parser.add_argument("--mission-mode", choices=["manual", "round1", "round2", "round3", "indoor"], default="manual")
    parser.add_argument("--start-corner", type=str, default="lower_left", help="one of lower_left, lower_right, upper_left, upper_right")
    parser.add_argument("--start-x-m", type=float, default=float("nan"), help="optional UGV start x in meters from the lower-left field origin")
    parser.add_argument("--start-y-m", type=float, default=float("nan"), help="optional UGV start y in meters from the lower-left field origin")
    parser.add_argument("--start-yaw-deg", type=float, default=float("nan"), help="optional UGV initial yaw in degrees; defaults to facing field center")
    parser.add_argument("--center-loiter-radius-m", type=float, default=0.75)
    parser.add_argument("--target-accept-radius-m", type=float, default=YARD_TO_M, help="competition-mode radius that counts as reaching the marker")
    parser.add_argument("--straight-distance-m", type=float, default=yd(13.0), help="round1/round2 straight-ahead runout goal distance")
    parser.add_argument("--competition-mission-v2-enabled", type=str_to_bool, default=True, help="use formal competition mission V2 for round1/round2/round3")
    parser.add_argument("--sweep-field-width-m", type=float, default=yd(15.0))
    parser.add_argument("--sweep-field-height-m", type=float, default=yd(15.0))
    parser.add_argument("--sweep-row-length-m", type=float, default=0.0, help="explicit row length for classroom sweep tests; <=0 uses field width minus margins")
    parser.add_argument("--sweep-headland-margin-m", type=float, default=0.75)
    parser.add_argument("--sweep-boundary-margin-m", type=float, default=0.45)
    parser.add_argument("--sweep-turn-radius-m", type=float, default=1.00)
    parser.add_argument("--sweep-turn-style", choices=["segmented_90", "segmented_pivot_90"], default="segmented_90")
    parser.add_argument("--sweep-row-transition-enabled", type=str_to_bool, default=False)
    parser.add_argument("--sweep-max-rows", type=int, default=0, help="0 means all rows that fit in the configured field")
    parser.add_argument("--sweep-row-end-tolerance-m", type=float, default=0.35)
    parser.add_argument("--sweep-turn-90-yaw-tolerance-deg", type=float, default=7.0)
    parser.add_argument("--sweep-turn-strong-error-deg", type=float, default=30.0)
    parser.add_argument("--sweep-turn-capture-error-deg", type=float, default=12.0)
    parser.add_argument("--sweep-turn-settle-error-deg", type=float, default=7.0)
    parser.add_argument("--sweep-turn-settle-frames", type=int, default=3)
    parser.add_argument("--sweep-pivot-min-emitted-omega-rps", type=float, default=0.25)
    parser.add_argument("--sweep-pivot-max-emitted-omega-rps", type=float, default=0.45)
    parser.add_argument("--sweep-pivot-wheel-min-mps", type=float, default=0.08)
    parser.add_argument("--sweep-pivot-settle-frames", type=int, default=3)
    parser.add_argument("--sweep-cross-lane-v-mps", type=float, default=0.12)
    parser.add_argument("--sweep-cross-lane-heading-kp", type=float, default=1.10)
    parser.add_argument("--sweep-lane-capture-tolerance-m", type=float, default=0.20)
    parser.add_argument("--sweep-yaw-capture-tolerance-deg", type=float, default=8.0)
    parser.add_argument("--sweep-transition-min-emitted-v-mps", type=float, default=0.12)
    parser.add_argument("--sweep-transition-min-emitted-omega-rps", type=float, default=0.20)
    parser.add_argument("--sweep-transition-max-emitted-omega-rps", type=float, default=0.40)
    parser.add_argument("--sweep-transition-inner-wheel-min-mps", type=float, default=0.04)
    parser.add_argument("--sweep-transition-timeout-s", type=float, default=20.0)
    parser.add_argument("--sweep-transition-divergence-action", choices=["stop", "warn"], default="stop")
    parser.add_argument("--sweep-cell-size-m", type=float, default=0.75)
    parser.add_argument("--sweep-lane-spacing-m", type=float, default=0.75)
    parser.add_argument("--sweep-lane-spacing-manual-override", type=str_to_bool, default=False)
    parser.add_argument("--sweep-coverage-radius-m", type=float, default=0.55)
    parser.add_argument("--sweep-coverage-threshold", type=float, default=0.85)
    parser.add_argument("--sweep-goal-timeout-s", type=float, default=8.0)
    parser.add_argument("--sweep-fail-limit", type=int, default=3)
    parser.add_argument("--sweep-lane-tolerance-m", type=float, default=0.30)
    parser.add_argument("--sweep-heading-tolerance-deg", type=float, default=25.0)
    parser.add_argument("--sweep-allow-pure-turn", type=str_to_bool, default=False)
    parser.add_argument("--sweep-stall-action", choices=["skip", "stop"], default="skip")
    parser.add_argument("--marker-camera-fov-deg", type=float, default=120.0)
    parser.add_argument("--marker-reliable-detection-range-m", type=float, default=float("nan"))
    parser.add_argument("--marker-coverage-overlap-ratio", type=float, default=0.5)
    parser.add_argument("--marker-auto-lane-spacing-enabled", type=str_to_bool, default=False)
    parser.add_argument("--min-competition-speed-mps", type=float, default=0.0894, help="minimum moving speed required by competition rules, 0.2 mph in m/s")
    parser.add_argument("--field-map-topic", type=str, default="/ugv/field_map")
    parser.add_argument("--target-topic", type=str, default="/ugv/target")
    parser.add_argument("--marker-topic", type=str, default="/ugv/marker_detection")
    parser.add_argument("--mission-flag-topic", type=str, default="/ugv/mission_flag")
    parser.add_argument("--nav-status-period-s", type=float, default=1.0)
    parser.add_argument("--use-imu-yaw", type=str_to_bool, default=False)
    parser.add_argument("--imu-yaw-blend", type=float, default=0.25)
    parser.add_argument("--imu-yaw-axis", type=str, default="z")
    parser.add_argument("--imu-yaw-sign", type=float, default=1.0)
    parser.add_argument("--robot-length-m", type=float, default=ft(30.0 / 12.0), help="robot footprint length used for collision checks")
    parser.add_argument("--robot-width-m", type=float, default=ft(30.0 / 12.0), help="robot footprint width used for gap checks")
    parser.add_argument("--robot-track-width-m", type=float, default=ft(2.0), help="wheel track width for encoder odometry")
    parser.add_argument("--robot-wheel-radius-m", type=float, default=0.06, help="wheel radius used by real-mode navigation encoder odometry")
    parser.add_argument("--robot-ticks-per-rev", type=int, default=1000, help="effective encoder ticks per wheel revolution used by real-mode navigation odometry")
    parser.add_argument("--robot-obstacle-buffer-m", type=float, default=0.025, help="extra footprint padding used by local trajectory collision checks")
    parser.add_argument("--lidar-offset-x-m", type=float, default=0.30, help="LiDAR x offset from robot center; positive is forward")
    parser.add_argument("--lidar-offset-y-m", type=float, default=0.0, help="LiDAR y offset from robot center; positive is left")
    parser.add_argument("--lidar-used-fov-deg", type=float, default=180.0, help="front-centered LiDAR field of view used by nav; rear hits are ignored")
    parser.add_argument("--allow-reverse", type=str_to_bool, default=False, help="allow planner to command BACKWARD; default false because rear LiDAR is obstructed")
    parser.add_argument("--min-motion-raw", type=float, default=0.22, help="floor applied to non-stop raw wheel commands after mission speed scaling")
    parser.add_argument("--recovery-turn-raw", type=float, default=0.32, help="strong in-place turn raw used only for blocked/stuck recovery")
    parser.add_argument("--min-speed-mps", type=float, default=0.178816, help="mission minimum moving-speed requirement, 0.4 mph expressed in m/s")
    parser.add_argument("--drive-speed-level", type=drive_speed_level_arg, default=4, help="overall real-mode drive speed cap: 1=25%%, 2=50%%, 3=75%%, 4=100%%")
    parser.add_argument("--front-safety-margin-m", type=float, default=0.08, help="extra front clearance required before forward commands")
    parser.add_argument("--rear-safety-margin-m", type=float, default=0.08, help="extra rear clearance required before reverse commands")
    parser.add_argument("--local-plan-inflation-m", type=float, default=0.0, help="extra local costmap inflation beyond the robot footprint")
    parser.add_argument("--active-scan-enabled", type=str_to_bool, default=True, help="turn in place to gather a wider view after repeated front blockage evidence")
    parser.add_argument("--active-scan-confirm-steps", type=int, default=3, help="consecutive constrained frames before active scan can interrupt forward probing")
    parser.add_argument("--active-scan-steps", type=int, default=7, help="number of turn commands used for each active scan burst")
    parser.add_argument("--active-scan-panoramic-steps", type=int, default=20, help="number of turn commands for a forced clockwise scan when the front view is cluttered")
    parser.add_argument("--active-scan-cooldown-steps", type=int, default=4, help="control cycles after a scan burst before another scan can start")
    parser.add_argument("--active-scan-probe-steps", type=int, default=5, help="control cycles after a scan burst where velocity control probes the newly scanned view")
    parser.add_argument("--active-scan-front-clear-m", type=float, default=1.25, help="front clearance below which repeated depth/costmap evidence can trigger active scan")
    parser.add_argument("--active-scan-corridor-extra-width-m", type=float, default=0.03, help="extra half-width used when counting ZED depth points for active scan evidence")
    parser.add_argument("--continuous-control-enabled", type=str_to_bool, default=True, help="use the Nav2-inspired DWA/RPP velocity controller instead of the legacy action-block local planner")
    parser.add_argument("--continuous-max-speed-mps", type=float, default=0.36, help="max forward speed used by the continuous local controller before drive-speed-level scaling")
    parser.add_argument("--continuous-max-omega-rps", type=float, default=1.15, help="max yaw rate used by the continuous local controller")
    parser.add_argument("--continuous-horizon-s", type=float, default=1.35, help="DWA rollout horizon in seconds")
    parser.add_argument("--continuous-accel-limit-mps2", type=float, default=0.35, help="linear acceleration limit in the velocity layer")
    parser.add_argument("--continuous-omega-accel-limit-rps2", type=float, default=1.80, help="angular acceleration limit in the velocity layer")
    parser.add_argument("--continuous-lowpass-alpha", type=float, default=0.55, help="velocity low-pass blend after acceleration limiting")
    parser.add_argument("--continuous-raw-per-mps", type=float, default=1.35, help="tank wheel m/s to raw command conversion gain")
    parser.add_argument("--continuous-slowdown-clearance-m", type=float, default=1.20, help="front clearance where Collision Monitor starts capping speed")
    parser.add_argument("--continuous-stop-clearance-m", type=float, default=0.48, help="front clearance where Collision Monitor commands zero linear speed")
    parser.add_argument("--continuous-gap-buffer-m", type=float, default=0.025, help="extra half-width used by polar gap checks beyond the robot width")
    parser.add_argument("--continuous-latency-buffer-s", type=float, default=0.25, help="extra obstacle padding equal to speed times this latency allowance")
    parser.add_argument("--continuous-allow-costmap-soft-penalty", type=str_to_bool, default=False, help="allow occupied local-costmap trajectories as a soft penalty; default false hard-rejects them")
    parser.add_argument("--emit-velocity-commands", type=str_to_bool, default=True, help="emit command_type=velocity for continuous local planner commands; false keeps raw fallback JSON preferred")
    parser.add_argument("--competition-closed-loop-enabled", type=str_to_bool, default=True, help="apply competition V2 sweep heading/lane closed-loop velocity adapter")
    parser.add_argument("--heading-hold-enabled", type=str_to_bool, default=True, help="hold sweep row yaw with odom/IMU yaw-rate feedback")
    parser.add_argument("--lane-follow-enabled", type=str_to_bool, default=True, help="apply sweep lane cross-track correction during competition V2 sweep")
    parser.add_argument("--heading-hold-kp", type=float, default=1.10, help="proportional gain for competition sweep heading hold")
    parser.add_argument("--heading-hold-kd", type=float, default=0.05, help="yaw-rate damping gain for competition sweep heading hold")
    parser.add_argument("--heading-hold-deadband-deg", type=float, default=3.0, help="heading error deadband before sweep heading hold commands omega")
    parser.add_argument("--heading-hold-max-omega-rps", type=float, default=0.55, help="max omega contribution from sweep heading hold")
    parser.add_argument("--lane-follow-kp-heading", type=float, default=1.10, help="cross-track to desired-heading gain for sweep lane following")
    parser.add_argument("--lane-follow-kp-omega", type=float, default=1.00, help="desired-heading to omega gain for sweep lane following")
    parser.add_argument("--lane-follow-deadband-m", type=float, default=0.03, help="cross-track deadband for sweep lane following")
    parser.add_argument("--lane-follow-max-heading-deg", type=float, default=18.0, help="max desired heading offset from sweep lane following")
    parser.add_argument("--lane-follow-max-omega-rps", type=float, default=0.35, help="max omega contribution from sweep lane following")
    parser.add_argument("--row-follower-speed-schedule-enabled", type=str_to_bool, default=True, help="schedule sweep row-follower gains by target forward speed")
    parser.add_argument("--row-follower-low-speed-mps", type=float, default=0.09, help="low-speed endpoint for sweep row-follower gain scheduling")
    parser.add_argument("--row-follower-high-speed-mps", type=float, default=0.22, help="high-speed endpoint for sweep row-follower gain scheduling")
    parser.add_argument("--row-follower-low-speed-lane-kp", type=float, default=0.85, help="lane gain at the low-speed scheduling endpoint")
    parser.add_argument("--row-follower-high-speed-lane-kp", type=float, default=1.00, help="lane gain at the high-speed scheduling endpoint")
    parser.add_argument("--row-follower-low-speed-heading-kp", type=float, default=0.95, help="heading gain at the low-speed scheduling endpoint")
    parser.add_argument("--row-follower-high-speed-heading-kp", type=float, default=1.10, help="heading gain at the high-speed scheduling endpoint")
    parser.add_argument("--row-follower-omega-low-pass-alpha", type=float, default=0.35, help="low-pass alpha applied to sweep row omega corrections")
    parser.add_argument("--row-follower-omega-rate-limit-rps2", type=float, default=0.60, help="rate limit for sweep row omega corrections")
    parser.add_argument("--row-follower-min-correction-interval-s", type=float, default=0.0, help="minimum interval between row omega correction updates")
    parser.add_argument("--sweep-planner-omega-weight-round2", type=float, default=0.0, help="round2 sweep blend weight for local planner omega; row follower owns steering by default")
    parser.add_argument("--sweep-planner-omega-weight-round3", type=float, default=0.2, help="round3 sweep blend weight for local planner omega when obstacle evidence exists")
    parser.add_argument("--sweep-metrics-log-enabled", type=str_to_bool, default=False, help="write competition sweep closed-loop metrics CSV")
    parser.add_argument("--sweep-metrics-log-dir", type=str, default="~/.ros/ugv_sweep_metrics", help="directory for sweep metrics CSV logs")
    parser.add_argument("--forward-arc-only-enabled", type=str_to_bool, default=True, help="keep normal competition sweep arcs forward-only so neither wheel target reverses")
    parser.add_argument("--forward-arc-margin", type=float, default=0.75, help="fraction of the no-reverse arc omega limit used during normal sweep")
    parser.add_argument("--min-sweep-v-mps", type=float, default=0.08, help="minimum forward velocity used by normal competition sweep closed-loop control")
    parser.add_argument("--omega-command-sign", type=float, default=1.0, help="sign multiplier applied to final competition closed-loop omega command")
    parser.add_argument("--heading-error-sign", type=float, default=1.0, help="sign multiplier applied to sweep heading error")
    parser.add_argument("--lane-error-sign", type=float, default=1.0, help="sign multiplier applied to sweep lane cross-track error")
    parser.add_argument("--lane-correction-sign", type=float, default=1.0, help="sign multiplier applied to sweep lane correction direction")
    parser.add_argument("--closed-loop-health-enabled", type=str_to_bool, default=True, help="enable competition sweep closed-loop divergence supervision")
    parser.add_argument("--closed-loop-divergence-window", type=int, default=5, help="rolling frame window for competition closed-loop divergence detection")
    parser.add_argument("--closed-loop-divergence-min-error-m", type=float, default=0.08, help="minimum lane error considered by closed-loop divergence detection")
    parser.add_argument("--closed-loop-divergence-max-growth-m", type=float, default=0.05, help="max allowed lane-error growth over the divergence window")
    parser.add_argument("--closed-loop-divergence-action", choices=["warn", "slow", "slow_then_stop", "stop"], default="slow_then_stop", help="action when competition closed-loop divergence persists")
    parser.add_argument("--local-costmap-enabled", type=str_to_bool, default=True, help="use rolling local costmap for local safety/collision checks")
    parser.add_argument("--local-costmap-width-m", type=float, default=4.0, help="rolling local costmap width in meters")
    parser.add_argument("--local-costmap-height-m", type=float, default=4.0, help="rolling local costmap height in meters")
    parser.add_argument("--local-costmap-resolution-m", type=float, default=0.06, help="rolling local costmap resolution in meters/cell")
    parser.add_argument("--local-costmap-dynamic-decay-s", type=float, default=1.0, help="seconds before dynamic local costmap cells decay")
    parser.add_argument("--local-costmap-obstacle-radius-m", type=float, default=0.06, help="radius used when marking local dynamic obstacle hits")
    parser.add_argument("--local-costmap-inflation-m", type=float, default=0.08, help="inflation radius applied to local costmap obstacles")
    parser.add_argument("--local-costmap-lidar-clear-radius-m", type=float, default=0.05, help="radius used when ray-clearing dynamic local costmap cells")
    parser.add_argument("--local-costmap-max-raytrace-m", type=float, default=4.0, help="max LiDAR range used for local costmap no-hit ray clearing")
    args = parser.parse_args()
    competition_mode_enabled, competition_mode_override = competition_mode_cli_value(args.competition_mode)
    if competition_mode_override is not None and args.mission_mode == "manual":
        args.mission_mode = competition_mode_override

    if args.mode == "sim":
        run_simulation(
            show_gui=not args.headless,
            max_steps=args.max_steps,
            robot_obstacle_buffer_m=args.robot_obstacle_buffer_m,
            local_plan_inflation_m=args.local_plan_inflation_m,
            continuous_control_enabled=args.continuous_control_enabled,
            continuous_max_speed_mps=args.continuous_max_speed_mps,
            continuous_max_omega_rps=args.continuous_max_omega_rps,
            continuous_horizon_s=args.continuous_horizon_s,
            continuous_accel_limit_mps2=args.continuous_accel_limit_mps2,
            continuous_omega_accel_limit_rps2=args.continuous_omega_accel_limit_rps2,
            continuous_lowpass_alpha=args.continuous_lowpass_alpha,
            continuous_raw_per_mps=args.continuous_raw_per_mps,
            continuous_slowdown_clearance_m=args.continuous_slowdown_clearance_m,
            continuous_stop_clearance_m=args.continuous_stop_clearance_m,
            continuous_gap_buffer_m=args.continuous_gap_buffer_m,
            continuous_latency_buffer_s=args.continuous_latency_buffer_s,
            continuous_allow_costmap_soft_penalty=args.continuous_allow_costmap_soft_penalty,
            emit_velocity_commands=args.emit_velocity_commands,
            competition_closed_loop_enabled=args.competition_closed_loop_enabled,
            heading_hold_enabled=args.heading_hold_enabled,
            lane_follow_enabled=args.lane_follow_enabled,
            heading_hold_kp=args.heading_hold_kp,
            heading_hold_kd=args.heading_hold_kd,
            heading_hold_deadband_deg=args.heading_hold_deadband_deg,
            heading_hold_max_omega_rps=args.heading_hold_max_omega_rps,
            lane_follow_kp_heading=args.lane_follow_kp_heading,
            lane_follow_kp_omega=args.lane_follow_kp_omega,
            lane_follow_deadband_m=args.lane_follow_deadband_m,
            lane_follow_max_heading_deg=args.lane_follow_max_heading_deg,
            lane_follow_max_omega_rps=args.lane_follow_max_omega_rps,
            row_follower_speed_schedule_enabled=args.row_follower_speed_schedule_enabled,
            row_follower_low_speed_mps=args.row_follower_low_speed_mps,
            row_follower_high_speed_mps=args.row_follower_high_speed_mps,
            row_follower_low_speed_lane_kp=args.row_follower_low_speed_lane_kp,
            row_follower_high_speed_lane_kp=args.row_follower_high_speed_lane_kp,
            row_follower_low_speed_heading_kp=args.row_follower_low_speed_heading_kp,
            row_follower_high_speed_heading_kp=args.row_follower_high_speed_heading_kp,
            row_follower_omega_low_pass_alpha=args.row_follower_omega_low_pass_alpha,
            row_follower_omega_rate_limit_rps2=args.row_follower_omega_rate_limit_rps2,
            row_follower_min_correction_interval_s=args.row_follower_min_correction_interval_s,
            sweep_planner_omega_weight_round2=args.sweep_planner_omega_weight_round2,
            sweep_planner_omega_weight_round3=args.sweep_planner_omega_weight_round3,
            sweep_metrics_log_enabled=args.sweep_metrics_log_enabled,
            sweep_metrics_log_dir=args.sweep_metrics_log_dir,
            forward_arc_only_enabled=args.forward_arc_only_enabled,
            forward_arc_margin=args.forward_arc_margin,
            min_sweep_v_mps=args.min_sweep_v_mps,
            omega_command_sign=args.omega_command_sign,
            heading_error_sign=args.heading_error_sign,
            lane_error_sign=args.lane_error_sign,
            lane_correction_sign=args.lane_correction_sign,
            closed_loop_health_enabled=args.closed_loop_health_enabled,
            closed_loop_divergence_window=args.closed_loop_divergence_window,
            closed_loop_divergence_min_error_m=args.closed_loop_divergence_min_error_m,
            closed_loop_divergence_max_growth_m=args.closed_loop_divergence_max_growth_m,
            closed_loop_divergence_action=args.closed_loop_divergence_action,
            local_costmap_enabled=args.local_costmap_enabled,
            local_costmap_width_m=args.local_costmap_width_m,
            local_costmap_height_m=args.local_costmap_height_m,
            local_costmap_resolution_m=args.local_costmap_resolution_m,
            local_costmap_dynamic_decay_s=args.local_costmap_dynamic_decay_s,
            local_costmap_obstacle_radius_m=args.local_costmap_obstacle_radius_m,
            local_costmap_inflation_m=args.local_costmap_inflation_m,
            local_costmap_lidar_clear_radius_m=args.local_costmap_lidar_clear_radius_m,
            local_costmap_max_raytrace_m=args.local_costmap_max_raytrace_m,
            recovery_turn_raw=args.recovery_turn_raw,
            mission_mode=args.mission_mode,
            competition_mission_v2_enabled=args.competition_mission_v2_enabled,
            straight_distance_m=args.straight_distance_m,
            sweep_field_width_m=args.sweep_field_width_m,
            sweep_field_height_m=args.sweep_field_height_m,
            sweep_row_length_m=args.sweep_row_length_m,
            sweep_headland_margin_m=args.sweep_headland_margin_m,
            sweep_boundary_margin_m=args.sweep_boundary_margin_m,
            sweep_turn_radius_m=args.sweep_turn_radius_m,
            sweep_turn_style=args.sweep_turn_style,
            sweep_row_transition_enabled=args.sweep_row_transition_enabled,
            sweep_max_rows=args.sweep_max_rows,
            sweep_row_end_tolerance_m=args.sweep_row_end_tolerance_m,
            sweep_turn_90_yaw_tolerance_deg=args.sweep_turn_90_yaw_tolerance_deg,
            sweep_turn_strong_error_deg=args.sweep_turn_strong_error_deg,
            sweep_turn_capture_error_deg=args.sweep_turn_capture_error_deg,
            sweep_turn_settle_error_deg=args.sweep_turn_settle_error_deg,
            sweep_turn_settle_frames=args.sweep_turn_settle_frames,
            sweep_pivot_min_emitted_omega_rps=args.sweep_pivot_min_emitted_omega_rps,
            sweep_pivot_max_emitted_omega_rps=args.sweep_pivot_max_emitted_omega_rps,
            sweep_pivot_wheel_min_mps=args.sweep_pivot_wheel_min_mps,
            sweep_pivot_settle_frames=args.sweep_pivot_settle_frames,
            sweep_cross_lane_v_mps=args.sweep_cross_lane_v_mps,
            sweep_cross_lane_heading_kp=args.sweep_cross_lane_heading_kp,
            sweep_lane_capture_tolerance_m=args.sweep_lane_capture_tolerance_m,
            sweep_yaw_capture_tolerance_deg=args.sweep_yaw_capture_tolerance_deg,
            sweep_transition_min_emitted_v_mps=args.sweep_transition_min_emitted_v_mps,
            sweep_transition_min_emitted_omega_rps=args.sweep_transition_min_emitted_omega_rps,
            sweep_transition_max_emitted_omega_rps=args.sweep_transition_max_emitted_omega_rps,
            sweep_transition_inner_wheel_min_mps=args.sweep_transition_inner_wheel_min_mps,
            sweep_transition_timeout_s=args.sweep_transition_timeout_s,
            sweep_transition_divergence_action=args.sweep_transition_divergence_action,
            sweep_cell_size_m=args.sweep_cell_size_m,
            sweep_lane_spacing_m=args.sweep_lane_spacing_m,
            sweep_lane_spacing_manual_override=args.sweep_lane_spacing_manual_override,
            sweep_coverage_radius_m=args.sweep_coverage_radius_m,
            sweep_coverage_threshold=args.sweep_coverage_threshold,
            sweep_goal_timeout_s=args.sweep_goal_timeout_s,
            sweep_fail_limit=args.sweep_fail_limit,
            sweep_lane_tolerance_m=args.sweep_lane_tolerance_m,
            sweep_heading_tolerance_deg=args.sweep_heading_tolerance_deg,
            sweep_allow_pure_turn=args.sweep_allow_pure_turn,
            sweep_stall_action=args.sweep_stall_action,
            marker_camera_fov_deg=args.marker_camera_fov_deg,
            marker_reliable_detection_range_m=args.marker_reliable_detection_range_m,
            marker_coverage_overlap_ratio=args.marker_coverage_overlap_ratio,
            marker_auto_lane_spacing_enabled=args.marker_auto_lane_spacing_enabled,
            min_competition_speed_mps=args.min_competition_speed_mps,
        )
    else:
        run_real_mode(
            replay_json=args.replay_json,
            competition_mode=competition_mode_enabled,
            mission_mode=args.mission_mode,
            start_corner=args.start_corner,
            start_x_m=args.start_x_m,
            start_y_m=args.start_y_m,
            start_yaw_deg=args.start_yaw_deg,
            center_loiter_radius_m=args.center_loiter_radius_m,
            target_accept_radius_m=args.target_accept_radius_m,
            straight_distance_m=args.straight_distance_m,
            competition_mission_v2_enabled=args.competition_mission_v2_enabled,
            sweep_field_width_m=args.sweep_field_width_m,
            sweep_field_height_m=args.sweep_field_height_m,
            sweep_row_length_m=args.sweep_row_length_m,
            sweep_headland_margin_m=args.sweep_headland_margin_m,
            sweep_boundary_margin_m=args.sweep_boundary_margin_m,
            sweep_turn_radius_m=args.sweep_turn_radius_m,
            sweep_turn_style=args.sweep_turn_style,
            sweep_row_transition_enabled=args.sweep_row_transition_enabled,
            sweep_max_rows=args.sweep_max_rows,
            sweep_row_end_tolerance_m=args.sweep_row_end_tolerance_m,
            sweep_turn_90_yaw_tolerance_deg=args.sweep_turn_90_yaw_tolerance_deg,
            sweep_turn_strong_error_deg=args.sweep_turn_strong_error_deg,
            sweep_turn_capture_error_deg=args.sweep_turn_capture_error_deg,
            sweep_turn_settle_error_deg=args.sweep_turn_settle_error_deg,
            sweep_turn_settle_frames=args.sweep_turn_settle_frames,
            sweep_pivot_min_emitted_omega_rps=args.sweep_pivot_min_emitted_omega_rps,
            sweep_pivot_max_emitted_omega_rps=args.sweep_pivot_max_emitted_omega_rps,
            sweep_pivot_wheel_min_mps=args.sweep_pivot_wheel_min_mps,
            sweep_pivot_settle_frames=args.sweep_pivot_settle_frames,
            sweep_cross_lane_v_mps=args.sweep_cross_lane_v_mps,
            sweep_cross_lane_heading_kp=args.sweep_cross_lane_heading_kp,
            sweep_lane_capture_tolerance_m=args.sweep_lane_capture_tolerance_m,
            sweep_yaw_capture_tolerance_deg=args.sweep_yaw_capture_tolerance_deg,
            sweep_transition_min_emitted_v_mps=args.sweep_transition_min_emitted_v_mps,
            sweep_transition_min_emitted_omega_rps=args.sweep_transition_min_emitted_omega_rps,
            sweep_transition_max_emitted_omega_rps=args.sweep_transition_max_emitted_omega_rps,
            sweep_transition_inner_wheel_min_mps=args.sweep_transition_inner_wheel_min_mps,
            sweep_transition_timeout_s=args.sweep_transition_timeout_s,
            sweep_transition_divergence_action=args.sweep_transition_divergence_action,
            sweep_cell_size_m=args.sweep_cell_size_m,
            sweep_lane_spacing_m=args.sweep_lane_spacing_m,
            sweep_lane_spacing_manual_override=args.sweep_lane_spacing_manual_override,
            sweep_coverage_radius_m=args.sweep_coverage_radius_m,
            sweep_coverage_threshold=args.sweep_coverage_threshold,
            sweep_goal_timeout_s=args.sweep_goal_timeout_s,
            sweep_fail_limit=args.sweep_fail_limit,
            sweep_lane_tolerance_m=args.sweep_lane_tolerance_m,
            sweep_heading_tolerance_deg=args.sweep_heading_tolerance_deg,
            sweep_allow_pure_turn=args.sweep_allow_pure_turn,
            sweep_stall_action=args.sweep_stall_action,
            marker_camera_fov_deg=args.marker_camera_fov_deg,
            marker_reliable_detection_range_m=args.marker_reliable_detection_range_m,
            marker_coverage_overlap_ratio=args.marker_coverage_overlap_ratio,
            marker_auto_lane_spacing_enabled=args.marker_auto_lane_spacing_enabled,
            min_competition_speed_mps=args.min_competition_speed_mps,
            field_map_topic=args.field_map_topic,
            target_topic=args.target_topic,
            marker_topic=args.marker_topic,
            mission_flag_topic=args.mission_flag_topic,
            nav_status_period_s=args.nav_status_period_s,
            use_imu_yaw=args.use_imu_yaw,
            imu_yaw_blend=args.imu_yaw_blend,
            imu_yaw_axis=args.imu_yaw_axis,
            imu_yaw_sign=args.imu_yaw_sign,
            robot_length_m=args.robot_length_m,
            robot_width_m=args.robot_width_m,
            robot_track_width_m=args.robot_track_width_m,
            robot_wheel_radius_m=args.robot_wheel_radius_m,
            robot_ticks_per_rev=args.robot_ticks_per_rev,
            robot_obstacle_buffer_m=args.robot_obstacle_buffer_m,
            lidar_offset_x_m=args.lidar_offset_x_m,
            lidar_offset_y_m=args.lidar_offset_y_m,
            lidar_used_fov_deg=args.lidar_used_fov_deg,
            allow_reverse=args.allow_reverse,
            min_motion_raw=args.min_motion_raw,
            recovery_turn_raw=args.recovery_turn_raw,
            min_speed_mps=args.min_speed_mps,
            drive_speed_level=args.drive_speed_level,
            front_safety_margin_m=args.front_safety_margin_m,
            rear_safety_margin_m=args.rear_safety_margin_m,
            local_plan_inflation_m=args.local_plan_inflation_m,
            active_scan_enabled=args.active_scan_enabled,
            active_scan_confirm_steps=args.active_scan_confirm_steps,
            active_scan_steps=args.active_scan_steps,
            active_scan_panoramic_steps=args.active_scan_panoramic_steps,
            active_scan_cooldown_steps=args.active_scan_cooldown_steps,
            active_scan_probe_steps=args.active_scan_probe_steps,
            active_scan_front_clear_m=args.active_scan_front_clear_m,
            active_scan_corridor_extra_width_m=args.active_scan_corridor_extra_width_m,
            continuous_control_enabled=args.continuous_control_enabled,
            continuous_max_speed_mps=args.continuous_max_speed_mps,
            continuous_max_omega_rps=args.continuous_max_omega_rps,
            continuous_horizon_s=args.continuous_horizon_s,
            continuous_accel_limit_mps2=args.continuous_accel_limit_mps2,
            continuous_omega_accel_limit_rps2=args.continuous_omega_accel_limit_rps2,
            continuous_lowpass_alpha=args.continuous_lowpass_alpha,
            continuous_raw_per_mps=args.continuous_raw_per_mps,
            continuous_slowdown_clearance_m=args.continuous_slowdown_clearance_m,
            continuous_stop_clearance_m=args.continuous_stop_clearance_m,
            continuous_gap_buffer_m=args.continuous_gap_buffer_m,
            continuous_latency_buffer_s=args.continuous_latency_buffer_s,
            continuous_allow_costmap_soft_penalty=args.continuous_allow_costmap_soft_penalty,
            emit_velocity_commands=args.emit_velocity_commands,
            competition_closed_loop_enabled=args.competition_closed_loop_enabled,
            heading_hold_enabled=args.heading_hold_enabled,
            lane_follow_enabled=args.lane_follow_enabled,
            heading_hold_kp=args.heading_hold_kp,
            heading_hold_kd=args.heading_hold_kd,
            heading_hold_deadband_deg=args.heading_hold_deadband_deg,
            heading_hold_max_omega_rps=args.heading_hold_max_omega_rps,
            lane_follow_kp_heading=args.lane_follow_kp_heading,
            lane_follow_kp_omega=args.lane_follow_kp_omega,
            lane_follow_deadband_m=args.lane_follow_deadband_m,
            lane_follow_max_heading_deg=args.lane_follow_max_heading_deg,
            lane_follow_max_omega_rps=args.lane_follow_max_omega_rps,
            row_follower_speed_schedule_enabled=args.row_follower_speed_schedule_enabled,
            row_follower_low_speed_mps=args.row_follower_low_speed_mps,
            row_follower_high_speed_mps=args.row_follower_high_speed_mps,
            row_follower_low_speed_lane_kp=args.row_follower_low_speed_lane_kp,
            row_follower_high_speed_lane_kp=args.row_follower_high_speed_lane_kp,
            row_follower_low_speed_heading_kp=args.row_follower_low_speed_heading_kp,
            row_follower_high_speed_heading_kp=args.row_follower_high_speed_heading_kp,
            row_follower_omega_low_pass_alpha=args.row_follower_omega_low_pass_alpha,
            row_follower_omega_rate_limit_rps2=args.row_follower_omega_rate_limit_rps2,
            row_follower_min_correction_interval_s=args.row_follower_min_correction_interval_s,
            sweep_planner_omega_weight_round2=args.sweep_planner_omega_weight_round2,
            sweep_planner_omega_weight_round3=args.sweep_planner_omega_weight_round3,
            sweep_metrics_log_enabled=args.sweep_metrics_log_enabled,
            sweep_metrics_log_dir=args.sweep_metrics_log_dir,
            forward_arc_only_enabled=args.forward_arc_only_enabled,
            forward_arc_margin=args.forward_arc_margin,
            min_sweep_v_mps=args.min_sweep_v_mps,
            omega_command_sign=args.omega_command_sign,
            heading_error_sign=args.heading_error_sign,
            lane_error_sign=args.lane_error_sign,
            lane_correction_sign=args.lane_correction_sign,
            closed_loop_health_enabled=args.closed_loop_health_enabled,
            closed_loop_divergence_window=args.closed_loop_divergence_window,
            closed_loop_divergence_min_error_m=args.closed_loop_divergence_min_error_m,
            closed_loop_divergence_max_growth_m=args.closed_loop_divergence_max_growth_m,
            closed_loop_divergence_action=args.closed_loop_divergence_action,
            local_costmap_enabled=args.local_costmap_enabled,
            local_costmap_width_m=args.local_costmap_width_m,
            local_costmap_height_m=args.local_costmap_height_m,
            local_costmap_resolution_m=args.local_costmap_resolution_m,
            local_costmap_dynamic_decay_s=args.local_costmap_dynamic_decay_s,
            local_costmap_obstacle_radius_m=args.local_costmap_obstacle_radius_m,
            local_costmap_inflation_m=args.local_costmap_inflation_m,
            local_costmap_lidar_clear_radius_m=args.local_costmap_lidar_clear_radius_m,
            local_costmap_max_raytrace_m=args.local_costmap_max_raytrace_m,
            max_steps=args.max_steps if args.replay_json else 0,
        )


if __name__ == "__main__":
    main()
