from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def wheel_targets_from_v_omega(v_mps: float, omega_radps: float, track_width_m: float) -> Tuple[float, float]:
    half_track = 0.5 * max(1e-6, float(track_width_m))
    return (
        float(v_mps) - float(omega_radps) * half_track,
        float(v_mps) + float(omega_radps) * half_track,
    )


@dataclass
class RowTransitionRequest:
    pose_x_m: float
    pose_y_m: float
    pose_yaw_rad: float
    row_direction: float
    current_lane_y_m: float
    next_lane_y_m: float
    row_end_x_m: float
    turn_radius_m: float
    track_width_m: float
    min_v_mps: float = 0.08
    max_v_mps: float = 0.36
    max_omega_radps: float = 1.15
    lookahead_m: float = 0.85
    forward_arc_only_enabled: bool = True
    forward_arc_margin: float = 0.75
    lane_capture_tolerance_m: float = 0.25
    yaw_capture_tolerance_rad: float = math.radians(12.0)
    row_end_tolerance_m: float = 0.40
    forward_corridor_safe: bool = True
    turn_side: Optional[str] = None


@dataclass
class RowTransitionCommand:
    desired_v_mps: float
    desired_omega_radps: float
    active_target_m: Tuple[float, float]
    transition_done: bool
    debug: Dict[str, Any]


class RowTransitionController:
    """Closed-loop pure-pursuit row transition for wide forward U-turns."""

    style = "wide_forward_arc_pure_pursuit"

    @staticmethod
    def _row_direction(value: float) -> float:
        return 1.0 if float(value) >= 0.0 else -1.0

    @staticmethod
    def _target_yaw_for_direction(row_direction: float) -> float:
        return 0.0 if row_direction >= 0.0 else math.pi

    @staticmethod
    def _turn_side(row_direction: float, lane_delta_m: float) -> Tuple[str, float]:
        side_sign = 1.0 if float(lane_delta_m) * float(row_direction) >= 0.0 else -1.0
        return ("left" if side_sign >= 0.0 else "right"), side_sign

    @staticmethod
    def _project_pose_to_polyline(
        pose_xy: Tuple[float, float],
        points: List[Tuple[float, float]],
    ) -> Tuple[float, List[float]]:
        cumulative = [0.0]
        for idx in range(1, len(points)):
            cumulative.append(
                cumulative[-1]
                + math.hypot(points[idx][0] - points[idx - 1][0], points[idx][1] - points[idx - 1][1])
            )
        if len(points) < 2:
            return 0.0, cumulative
        best_s = 0.0
        best_d2 = float("inf")
        px, py = pose_xy
        for idx in range(len(points) - 1):
            ax, ay = points[idx]
            bx, by = points[idx + 1]
            vx = bx - ax
            vy = by - ay
            seg_len2 = vx * vx + vy * vy
            if seg_len2 <= 1e-12:
                continue
            t = clamp(((px - ax) * vx + (py - ay) * vy) / seg_len2, 0.0, 1.0)
            qx = ax + t * vx
            qy = ay + t * vy
            d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy)
            if d2 < best_d2:
                best_d2 = d2
                best_s = cumulative[idx] + math.sqrt(seg_len2) * t
        return best_s, cumulative

    @staticmethod
    def _point_at_s(points: List[Tuple[float, float]], cumulative: List[float], s_target: float) -> Tuple[float, float]:
        if not points:
            return 0.0, 0.0
        if len(points) == 1 or s_target <= 0.0:
            return points[0]
        if s_target >= cumulative[-1]:
            return points[-1]
        for idx in range(len(points) - 1):
            if cumulative[idx + 1] < s_target:
                continue
            seg_len = max(1e-9, cumulative[idx + 1] - cumulative[idx])
            t = clamp((s_target - cumulative[idx]) / seg_len, 0.0, 1.0)
            ax, ay = points[idx]
            bx, by = points[idx + 1]
            return (ax + t * (bx - ax), ay + t * (by - ay))
        return points[-1]

    def build_path(self, request: RowTransitionRequest) -> List[Tuple[float, float]]:
        direction = self._row_direction(request.row_direction)
        lane_delta = float(request.next_lane_y_m) - float(request.current_lane_y_m)
        radius = max(
            float(request.turn_radius_m),
            0.55,
            0.50 * abs(lane_delta) + 0.50 * max(1e-6, float(request.track_width_m)),
        )
        capture_depth = max(radius, abs(lane_delta), float(request.row_end_tolerance_m) + 0.25)
        row_end_x = float(request.row_end_x_m)
        capture_x = row_end_x - direction * capture_depth
        final_x = capture_x - direction * max(0.65, 0.75 * radius)
        mid_x = row_end_x - direction * min(0.45 * capture_depth, max(0.35, 0.70 * radius))
        current_y = float(request.current_lane_y_m)
        next_y = float(request.next_lane_y_m)
        mid_y = current_y + 0.55 * lane_delta
        return [
            (row_end_x, current_y),
            (row_end_x, current_y + 0.25 * lane_delta),
            (mid_x, mid_y),
            (capture_x, next_y),
            (final_x, next_y),
        ]

    def compute(self, request: RowTransitionRequest) -> RowTransitionCommand:
        direction = self._row_direction(request.row_direction)
        next_direction = -direction
        next_row_yaw = self._target_yaw_for_direction(next_direction)
        lane_delta = float(request.next_lane_y_m) - float(request.current_lane_y_m)
        turn_side, turn_side_sign = self._turn_side(direction, lane_delta)
        if request.turn_side in {"left", "right"}:
            turn_side = str(request.turn_side)
            turn_side_sign = 1.0 if turn_side == "left" else -1.0

        path = self.build_path(request)
        lookahead = clamp(
            float(request.lookahead_m),
            0.35,
            max(0.36, 1.50 * max(float(request.turn_radius_m), abs(lane_delta), 0.5)),
        )
        pose_xy = (float(request.pose_x_m), float(request.pose_y_m))
        progress_s, cumulative = self._project_pose_to_polyline(pose_xy, path)
        target = self._point_at_s(path, cumulative, progress_s + lookahead)
        dx = target[0] - pose_xy[0]
        dy = target[1] - pose_xy[1]
        c = math.cos(float(request.pose_yaw_rad))
        s = math.sin(float(request.pose_yaw_rad))
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        lookahead_dist2 = max(0.12 * 0.12, dx * dx + dy * dy)
        curvature = 2.0 * local_y / lookahead_dist2
        min_v = max(0.0, float(request.min_v_mps))
        max_v = max(min_v, float(request.max_v_mps))
        yaw_error = wrap_to_pi(next_row_yaw - float(request.pose_yaw_rad))
        lane_error = float(request.pose_y_m) - float(request.next_lane_y_m)
        speed_scale = 1.0 - 0.35 * clamp(abs(yaw_error) / math.pi, 0.0, 1.0)
        v = clamp(max(min_v, 0.16 * speed_scale), min_v, max_v)
        if local_x < 0.05:
            curvature = turn_side_sign / max(0.35, float(request.turn_radius_m))
            v = min(v, max(min_v, 0.13))
        omega = clamp(v * curvature, -abs(float(request.max_omega_radps)), abs(float(request.max_omega_radps)))

        forward_arc_limit = None
        forward_arc_clamped = False
        if bool(request.forward_arc_only_enabled):
            track_width = max(1e-6, float(request.track_width_m))
            v = max(v, min_v)
            forward_arc_limit = 2.0 * v / track_width * clamp(float(request.forward_arc_margin), 0.0, 1.0)
            limited_omega = clamp(omega, -forward_arc_limit, forward_arc_limit)
            forward_arc_clamped = abs(limited_omega - omega) > 1e-9
            omega = limited_omega
        left_target, right_target = wheel_targets_from_v_omega(v, omega, request.track_width_m)

        inside_offset = max(0.20, float(request.row_end_tolerance_m))
        safely_inside = (
            pose_xy[0] <= float(request.row_end_x_m) - inside_offset
            if direction > 0.0
            else pose_xy[0] >= float(request.row_end_x_m) + inside_offset
        )
        lane_captured = abs(lane_error) < max(0.0, float(request.lane_capture_tolerance_m))
        yaw_captured = abs(yaw_error) < max(0.0, float(request.yaw_capture_tolerance_rad))
        transition_done = bool(lane_captured and yaw_captured and safely_inside and request.forward_corridor_safe)
        lane_progress = 1.0 if abs(lane_delta) < 1e-6 else clamp(
            abs((pose_xy[1] - float(request.current_lane_y_m)) / lane_delta),
            0.0,
            1.0,
        )
        yaw_progress = clamp(1.0 - abs(yaw_error) / math.pi, 0.0, 1.0)
        path_progress = 1.0 if cumulative[-1] <= 1e-9 else clamp(progress_s / cumulative[-1], 0.0, 1.0)
        progress = clamp(0.45 * lane_progress + 0.35 * yaw_progress + 0.20 * path_progress, 0.0, 1.0)

        reasons = []
        if not lane_captured:
            reasons.append("lane_not_captured")
        if not yaw_captured:
            reasons.append("yaw_not_captured")
        if not safely_inside:
            reasons.append("not_inside_next_row")
        if not request.forward_corridor_safe:
            reasons.append("forward_corridor_unsafe")
        reason = "transition_complete" if transition_done else "_".join(reasons or ["transition_tracking"])
        debug = {
            "row_transition_style": self.style,
            "row_transition_progress": round(progress, 4),
            "row_transition_done": bool(transition_done),
            "row_transition_reason": reason,
            "row_transition_target_m": [round(float(target[0]), 3), round(float(target[1]), 3)],
            "target_row_yaw_deg": round(math.degrees(next_row_yaw), 2),
            "yaw_capture_error_deg": round(math.degrees(yaw_error), 3),
            "lane_capture_error_m": round(lane_error, 4),
            "turn_radius_m": round(float(request.turn_radius_m), 3),
            "turn_side": turn_side,
            "desired_v_mps": round(v, 4),
            "desired_omega_radps": round(omega, 4),
            "desired_left_target_mps": round(left_target, 4),
            "desired_right_target_mps": round(right_target, 4),
            "forward_arc_omega_limit_radps": None if forward_arc_limit is None else round(forward_arc_limit, 4),
            "forward_arc_clamped": bool(forward_arc_clamped),
            "forward_corridor_safe": bool(request.forward_corridor_safe),
            "safely_inside_next_row": bool(safely_inside),
        }
        return RowTransitionCommand(
            desired_v_mps=v,
            desired_omega_radps=omega,
            active_target_m=target,
            transition_done=transition_done,
            debug=debug,
        )


__all__ = [
    "RowTransitionCommand",
    "RowTransitionController",
    "RowTransitionRequest",
    "clamp",
    "wheel_targets_from_v_omega",
    "wrap_to_pi",
]
