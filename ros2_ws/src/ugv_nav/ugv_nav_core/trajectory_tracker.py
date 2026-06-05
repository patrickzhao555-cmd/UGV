"""Closed-loop local trajectory tracking from encoder distance and IMU yaw."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

from .chassis_controller import clamp, encoder_ticks_to_distance_m, wrap_pi


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class TrackerConfig:
    target_stop_radius_m: float = 0.75
    lookahead_min_m: float = 0.45
    lookahead_max_m: float = 1.20
    lookahead_speed_gain: float = 1.8
    nominal_speed_mps: float = 0.25
    max_speed_mps: float = 0.42
    max_omega_radps: float = 0.85
    heading_kp: float = 0.85
    cross_track_kp: float = 0.75
    slowdown_distance_m: float = 1.20
    min_finish_speed_mps: float = 0.10
    obstacle_warn_m: float = 2.0
    obstacle_stop_m: float = 1.0
    bypass_offset_m: float = 1.1
    bypass_forward_m: float = 2.0
    bypass_rejoin_ahead_m: float = 2.0
    side_clearance_min_m: float = 0.70
    track_width_m: float = 0.416
    wheel_radius_m: float = 0.0825
    ticks_per_rev: float = 3200.0


@dataclass
class TrackerPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    distance_m: float = 0.0


@dataclass
class TrackerState:
    state: str = "WAIT_TARGET"
    obstacle_state: str = "CLEAR"
    pose: TrackerPose = field(default_factory=TrackerPose)
    target: Optional[Point2D] = None
    base_path: list[Point2D] = field(default_factory=list)
    active_path: list[Point2D] = field(default_factory=list)
    start_left_ticks: Optional[int] = None
    start_right_ticks: Optional[int] = None
    last_left_ticks: Optional[int] = None
    last_right_ticks: Optional[int] = None
    start_heading_rad: float = 0.0
    last_pose_yaw_rad: float = 0.0
    path_index: int = 0
    bypass_side: str = "none"
    bypass_rejoin_s_m: Optional[float] = None
    fault_reason: Optional[str] = None
    completion_reason: Optional[str] = None
    remaining_m: Optional[float] = None
    cross_track_error_m: float = 0.0
    heading_error_rad: float = 0.0
    lookahead_m: float = 0.0
    radius_error_m: Optional[float] = None


@dataclass(frozen=True)
class TrackerStep:
    command_type: str
    v_mps: float
    omega_radps: float
    reason: str
    state: str


def reset_tracker(state: TrackerState) -> None:
    state.state = "WAIT_TARGET"
    state.obstacle_state = "CLEAR"
    state.pose = TrackerPose()
    state.target = None
    state.base_path = []
    state.active_path = []
    state.start_left_ticks = None
    state.start_right_ticks = None
    state.last_left_ticks = None
    state.last_right_ticks = None
    state.start_heading_rad = 0.0
    state.last_pose_yaw_rad = 0.0
    state.path_index = 0
    state.bypass_side = "none"
    state.bypass_rejoin_s_m = None
    state.fault_reason = None
    state.completion_reason = None
    state.remaining_m = None
    state.cross_track_error_m = 0.0
    state.heading_error_rad = 0.0
    state.lookahead_m = 0.0
    state.radius_error_m = None


def start_tracker_goal(
    state: TrackerState,
    *,
    target_x_m: float,
    target_y_m: float,
    left_ticks: int,
    right_ticks: int,
    heading_rad: float,
) -> None:
    reset_tracker(state)
    target = Point2D(float(target_x_m), float(target_y_m))
    state.target = target
    state.base_path = [Point2D(0.0, 0.0), target]
    state.active_path = list(state.base_path)
    state.start_left_ticks = int(left_ticks)
    state.start_right_ticks = int(right_ticks)
    state.last_left_ticks = int(left_ticks)
    state.last_right_ticks = int(right_ticks)
    state.start_heading_rad = float(heading_rad)
    state.last_pose_yaw_rad = 0.0
    state.pose = TrackerPose()
    state.state = "TRACK_PATH"


def update_tracker_odometry(
    state: TrackerState,
    *,
    left_ticks: int,
    right_ticks: int,
    heading_rad: float,
    config: TrackerConfig,
) -> None:
    if state.last_left_ticks is None or state.last_right_ticks is None:
        state.last_left_ticks = int(left_ticks)
        state.last_right_ticks = int(right_ticks)
        return
    left_delta = int(left_ticks) - int(state.last_left_ticks)
    right_delta = int(right_ticks) - int(state.last_right_ticks)
    state.last_left_ticks = int(left_ticks)
    state.last_right_ticks = int(right_ticks)
    left_m = encoder_ticks_to_distance_m(
        left_delta,
        wheel_radius_m=config.wheel_radius_m,
        ticks_per_rev=config.ticks_per_rev,
    )
    right_m = encoder_ticks_to_distance_m(
        right_delta,
        wheel_radius_m=config.wheel_radius_m,
        ticks_per_rev=config.ticks_per_rev,
    )
    distance_delta = 0.5 * (left_m + right_m)
    yaw = wrap_pi(float(heading_rad) - float(state.start_heading_rad))
    yaw_delta = wrap_pi(yaw - state.last_pose_yaw_rad)
    mid_yaw = wrap_pi(state.last_pose_yaw_rad + 0.5 * yaw_delta)
    state.pose.x += distance_delta * math.cos(mid_yaw)
    state.pose.y += distance_delta * math.sin(mid_yaw)
    state.pose.yaw = yaw
    state.pose.distance_m += abs(distance_delta)
    state.last_pose_yaw_rad = yaw


def path_length(path: Sequence[Point2D]) -> float:
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


def _point_at_s(path: Sequence[Point2D], s_m: float) -> Point2D:
    if not path:
        return Point2D(0.0, 0.0)
    if len(path) == 1:
        return path[0]
    remaining = max(0.0, float(s_m))
    for a, b in zip(path, path[1:]):
        seg_len = math.hypot(b.x - a.x, b.y - a.y)
        if seg_len <= 1e-9:
            continue
        if remaining <= seg_len:
            t = remaining / seg_len
            return Point2D(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
        remaining -= seg_len
    return path[-1]


@dataclass(frozen=True)
class PathProjection:
    s_m: float
    cross_track_m: float
    point: Point2D
    segment_index: int


def project_pose_to_path(path: Sequence[Point2D], pose: TrackerPose) -> PathProjection:
    if len(path) < 2:
        return PathProjection(0.0, 0.0, Point2D(pose.x, pose.y), 0)
    best_dist = math.inf
    best = PathProjection(0.0, 0.0, path[0], 0)
    s_base = 0.0
    for index, (a, b) in enumerate(zip(path, path[1:])):
        dx = b.x - a.x
        dy = b.y - a.y
        seg_len = math.hypot(dx, dy)
        if seg_len <= 1e-9:
            continue
        ux = dx / seg_len
        uy = dy / seg_len
        px = pose.x - a.x
        py = pose.y - a.y
        t = clamp(px * ux + py * uy, 0.0, seg_len)
        proj = Point2D(a.x + ux * t, a.y + uy * t)
        err_x = pose.x - proj.x
        err_y = pose.y - proj.y
        dist = math.hypot(err_x, err_y)
        signed = ux * err_y - uy * err_x
        if dist < best_dist:
            best_dist = dist
            best = PathProjection(s_base + t, signed, proj, index)
        s_base += seg_len
    return best


def _base_path_axes(state: TrackerState) -> tuple[float, float, float, float]:
    if len(state.base_path) < 2:
        return 1.0, 0.0, 0.0, 1.0
    start = state.base_path[0]
    end = state.base_path[-1]
    dx = end.x - start.x
    dy = end.y - start.y
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return 1.0, 0.0, 0.0, 1.0
    ux = dx / length
    uy = dy / length
    return ux, uy, -uy, ux


def _plan_bypass(
    state: TrackerState,
    *,
    side: str,
    config: TrackerConfig,
) -> None:
    if state.target is None:
        return
    sign = 1.0 if side == "left" else -1.0
    ux, uy, nx, ny = _base_path_axes(state)
    projection = project_pose_to_path(state.base_path, state.pose)
    base_total = path_length(state.base_path)
    s0 = clamp(projection.s_m, 0.0, base_total)
    offset = max(0.0, float(config.bypass_offset_m))
    p1_s = min(base_total, s0 + 0.6)
    p2_s = min(base_total, s0 + max(0.6, float(config.bypass_forward_m)))
    rejoin_s = min(base_total, p2_s + max(0.1, float(config.bypass_rejoin_ahead_m)))
    p1_base = _point_at_s(state.base_path, p1_s)
    p2_base = _point_at_s(state.base_path, p2_s)
    rejoin = _point_at_s(state.base_path, rejoin_s)
    p1 = Point2D(p1_base.x + sign * nx * offset, p1_base.y + sign * ny * offset)
    p2 = Point2D(p2_base.x + sign * nx * offset, p2_base.y + sign * ny * offset)
    state.active_path = [Point2D(state.pose.x, state.pose.y), p1, p2, rejoin, state.target]
    state.bypass_side = side
    state.bypass_rejoin_s_m = rejoin_s
    state.obstacle_state = "BYPASS_TRACK"
    state.state = "BYPASS_TRACK"
    state.path_index = 0


def step_tracker(
    state: TrackerState,
    *,
    config: TrackerConfig,
    front_clearance_m: Optional[float] = None,
    left_clearance_m: Optional[float] = None,
    right_clearance_m: Optional[float] = None,
) -> TrackerStep:
    if state.target is None or not state.active_path:
        state.state = "WAIT_TARGET"
        return TrackerStep("stop", 0.0, 0.0, "tracker_wait_target", state.state)

    target_distance = math.hypot(state.target.x - state.pose.x, state.target.y - state.pose.y)
    state.remaining_m = target_distance
    if target_distance <= max(0.0, float(config.target_stop_radius_m)):
        state.state = "GOAL_REACHED"
        state.obstacle_state = "CLEAR"
        state.completion_reason = "goal_reached"
        return TrackerStep("stop", 0.0, 0.0, "goal_reached", state.state)

    if front_clearance_m is not None and math.isfinite(float(front_clearance_m)):
        front = float(front_clearance_m)
        if front <= max(0.0, float(config.obstacle_stop_m)):
            state.obstacle_state = "OBSTACLE_STOP"
            return TrackerStep("stop", 0.0, 0.0, "obstacle_stop", state.state)
        if front <= max(float(config.obstacle_stop_m), float(config.obstacle_warn_m)) and state.obstacle_state not in {
            "BYPASS_TRACK",
            "REJOIN_PATH",
        }:
            left = left_clearance_m if left_clearance_m is not None and math.isfinite(float(left_clearance_m)) else 0.0
            right = right_clearance_m if right_clearance_m is not None and math.isfinite(float(right_clearance_m)) else 0.0
            side_min = max(0.0, float(config.side_clearance_min_m))
            if max(left, right) < side_min:
                state.obstacle_state = "BLOCKED"
                state.fault_reason = "obstacle_no_clear_side"
                return TrackerStep("stop", 0.0, 0.0, "obstacle_no_clear_side", "FAULT")
            _plan_bypass(state, side="left" if left >= right else "right", config=config)
        elif state.obstacle_state == "CLEAR":
            state.obstacle_state = "CLEAR"

    projection = project_pose_to_path(state.active_path, state.pose)
    state.path_index = projection.segment_index
    state.cross_track_error_m = projection.cross_track_m
    total_len = path_length(state.active_path)
    remaining_path_m = max(0.0, total_len - projection.s_m)
    state.remaining_m = min(target_distance, remaining_path_m) if remaining_path_m > 0.0 else target_distance

    lookahead_min = max(0.05, float(config.lookahead_min_m))
    lookahead_max = max(lookahead_min, float(config.lookahead_max_m))
    lookahead = clamp(
        lookahead_min + float(config.nominal_speed_mps) * float(config.lookahead_speed_gain),
        lookahead_min,
        lookahead_max,
    )
    lookahead = min(lookahead, max(lookahead_min, remaining_path_m))
    state.lookahead_m = lookahead
    lookahead_point = _point_at_s(state.active_path, projection.s_m + lookahead)

    dx = lookahead_point.x - state.pose.x
    dy = lookahead_point.y - state.pose.y
    distance_to_lookahead = max(1e-6, math.hypot(dx, dy))
    heading_to_lookahead = math.atan2(dy, dx)
    heading_error = wrap_pi(heading_to_lookahead - state.pose.yaw)
    state.heading_error_rad = heading_error
    robot_y = -math.sin(state.pose.yaw) * dx + math.cos(state.pose.yaw) * dy
    curvature = 2.0 * robot_y / max(1e-6, distance_to_lookahead * distance_to_lookahead)
    v = min(max(0.0, float(config.nominal_speed_mps)), max(0.0, float(config.max_speed_mps)))
    if abs(curvature) > 1e-6:
        v = min(v, max(0.0, float(config.max_omega_radps)) / abs(curvature))
    if state.remaining_m is not None and state.remaining_m < max(1e-6, float(config.slowdown_distance_m)):
        ratio = clamp(state.remaining_m / max(1e-6, float(config.slowdown_distance_m)), 0.0, 1.0)
        v = max(float(config.min_finish_speed_mps), v * ratio)
    if state.obstacle_state == "BYPASS_TRACK":
        v = min(v, max(float(config.min_finish_speed_mps), float(config.nominal_speed_mps) * 0.75))

    omega = v * curvature
    omega += float(config.heading_kp) * heading_error
    omega -= float(config.cross_track_kp) * projection.cross_track_m / max(0.35, lookahead)
    omega = clamp(omega, -float(config.max_omega_radps), float(config.max_omega_radps))

    if state.obstacle_state == "BYPASS_TRACK":
        base_projection = project_pose_to_path(state.base_path, state.pose)
        if state.bypass_rejoin_s_m is not None and base_projection.s_m >= state.bypass_rejoin_s_m - 0.25:
            state.active_path = list(state.base_path)
            state.obstacle_state = "REJOIN_PATH"
            state.state = "REJOIN_PATH"
    elif state.state == "REJOIN_PATH":
        state.obstacle_state = "CLEAR"
        state.state = "TRACK_PATH"
        state.bypass_side = "none"
        state.bypass_rejoin_s_m = None

    return TrackerStep("velocity", v, omega, "tracker_path_follow", state.state)


def tracker_status(state: TrackerState) -> dict[str, object]:
    return {
        "tracker_state": state.state,
        "tracker_pose_m": [round(state.pose.x, 4), round(state.pose.y, 4), round(math.degrees(state.pose.yaw), 3)],
        "tracker_target_m": None if state.target is None else [round(state.target.x, 4), round(state.target.y, 4)],
        "tracker_path_index": state.path_index,
        "tracker_remaining_m": None if state.remaining_m is None else round(float(state.remaining_m), 4),
        "tracker_cross_track_error_m": round(float(state.cross_track_error_m), 4),
        "tracker_heading_error_rad": round(float(state.heading_error_rad), 5),
        "tracker_lookahead_m": round(float(state.lookahead_m), 4),
        "tracker_obstacle_state": state.obstacle_state,
        "tracker_bypass_side": state.bypass_side,
        "tracker_fault_reason": state.fault_reason,
        "tracker_completion_reason": state.completion_reason,
    }
