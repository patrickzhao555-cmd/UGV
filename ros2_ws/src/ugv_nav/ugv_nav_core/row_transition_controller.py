from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


SEGMENTED_TURN_STYLE = "segmented_90"
TRANSITION_SEGMENTS = {"turn_out_90", "cross_lane", "turn_in_90", "acquire_next_row"}


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


def yaw_deg(yaw_rad: float) -> float:
    return math.degrees(wrap_to_pi(float(yaw_rad)))


@dataclass
class RowTransitionRequest:
    segment: str
    pose_x_m: float
    pose_y_m: float
    pose_yaw_rad: float
    row_direction: float
    current_lane_y_m: float
    next_lane_y_m: float
    row_end_x_m: float
    track_width_m: float
    drive_speed_factor: float = 1.0
    yaw_rate_radps: Optional[float] = None
    min_emitted_v_mps: float = 0.12
    min_emitted_omega_radps: float = 0.20
    max_emitted_omega_radps: float = 0.40
    inner_wheel_min_mps: float = 0.04
    max_v_mps: float = 0.36
    turn_90_yaw_tolerance_rad: float = math.radians(7.0)
    lane_capture_tolerance_m: float = 0.20
    yaw_capture_tolerance_rad: float = math.radians(8.0)
    forward_corridor_safe: bool = True
    kp_yaw: float = 1.25
    kd_yaw: float = 0.10


@dataclass
class RowTransitionCommand:
    desired_v_mps: float
    desired_omega_radps: float
    active_target_m: Tuple[float, float]
    segment_done: bool
    transition_done: bool
    debug: Dict[str, Any]


class RowTransitionController:
    """Closed-loop segmented 90/cross/90 row transition controller."""

    style = SEGMENTED_TURN_STYLE

    @staticmethod
    def row_direction(value: float) -> float:
        return 1.0 if float(value) >= 0.0 else -1.0

    @staticmethod
    def next_row_yaw(row_direction: float) -> float:
        return 0.0 if row_direction >= 0.0 else math.pi

    @staticmethod
    def side_yaw(current_lane_y_m: float, next_lane_y_m: float) -> float:
        return math.pi * 0.5 if float(next_lane_y_m) >= float(current_lane_y_m) else -math.pi * 0.5

    @staticmethod
    def turn_side(row_direction: float, current_lane_y_m: float, next_lane_y_m: float) -> str:
        side = RowTransitionController.side_yaw(current_lane_y_m, next_lane_y_m)
        current = RowTransitionController.next_row_yaw(row_direction)
        return "left" if wrap_to_pi(side - current) >= 0.0 else "right"

    @staticmethod
    def _target_for_segment(request: RowTransitionRequest, target_yaw: float) -> Tuple[float, float]:
        lookahead = 0.80
        if request.segment == "cross_lane":
            lookahead = max(0.35, abs(float(request.pose_y_m) - float(request.next_lane_y_m)))
        return (
            float(request.pose_x_m) + lookahead * math.cos(target_yaw),
            float(request.pose_y_m) + lookahead * math.sin(target_yaw),
        )

    @staticmethod
    def _apply_transition_wheel_floor(
        v_mps: float,
        omega_radps: float,
        *,
        track_width_m: float,
        max_v_mps: float,
        inner_wheel_min_mps: float,
    ) -> Tuple[float, float, float, float, bool, bool]:
        track = max(1e-6, float(track_width_m))
        v = max(0.0, float(v_mps))
        omega = float(omega_radps)
        inner_min = max(0.0, float(inner_wheel_min_mps))
        max_v = max(0.0, float(max_v_mps))
        raised_v = False
        reduced_omega = False

        needed_v = inner_min + 0.5 * track * abs(omega)
        if v < needed_v:
            v = min(max_v, needed_v)
            raised_v = True
        if max_v > 0.0 and v > max_v:
            v = max_v
        max_omega_for_inner = max(0.0, 2.0 * (v - inner_min) / track)
        if abs(omega) > max_omega_for_inner:
            omega = math.copysign(max_omega_for_inner, omega)
            reduced_omega = True

        left, right = wheel_targets_from_v_omega(v, omega, track)
        if left < inner_min - 1e-9 or right < inner_min - 1e-9:
            # One last numerical guard: do not quietly emit a reverse/pivot target
            # from normal row transition control.
            v = max(v, inner_min + 0.5 * track * abs(omega))
            v = min(max_v, v) if max_v > 0.0 else v
            left, right = wheel_targets_from_v_omega(v, omega, track)
        return v, omega, left, right, raised_v, reduced_omega

    def _yaw_command(self, yaw_error: float, request: RowTransitionRequest) -> float:
        yaw_rate = 0.0
        if request.yaw_rate_radps is not None and math.isfinite(float(request.yaw_rate_radps)):
            yaw_rate = float(request.yaw_rate_radps)
        omega = float(request.kp_yaw) * float(yaw_error) - float(request.kd_yaw) * yaw_rate
        drive_scale = max(1e-6, float(request.drive_speed_factor))
        min_omega = max(0.0, float(request.min_emitted_omega_radps)) / drive_scale
        max_omega = max(min_omega, float(request.max_emitted_omega_radps) / drive_scale)
        if abs(yaw_error) > 1e-6:
            omega = math.copysign(max(min_omega, abs(omega)), yaw_error)
        return clamp(omega, -max_omega, max_omega)

    def compute(self, request: RowTransitionRequest) -> RowTransitionCommand:
        segment = str(request.segment or "").strip().lower()
        direction = self.row_direction(request.row_direction)
        next_direction = -direction
        side_yaw = self.side_yaw(request.current_lane_y_m, request.next_lane_y_m)
        next_yaw = self.next_row_yaw(next_direction)
        lane_error = float(request.pose_y_m) - float(request.next_lane_y_m)
        target_yaw = side_yaw if segment in {"turn_out_90", "cross_lane"} else next_yaw
        yaw_error = wrap_to_pi(target_yaw - float(request.pose_yaw_rad))
        side_yaw_error = wrap_to_pi(side_yaw - float(request.pose_yaw_rad))
        next_yaw_error = wrap_to_pi(next_yaw - float(request.pose_yaw_rad))

        turn_90_done = abs(yaw_error) < max(0.0, float(request.turn_90_yaw_tolerance_rad))
        lane_done = abs(lane_error) < max(0.0, float(request.lane_capture_tolerance_m))
        next_yaw_done = abs(next_yaw_error) < max(0.0, float(request.yaw_capture_tolerance_rad))

        if segment == "turn_out_90":
            segment_done = turn_90_done
            transition_done = False
            progress = 1.0 - min(1.0, abs(yaw_error) / max(math.radians(90.0), 1e-6))
        elif segment == "cross_lane":
            segment_done = lane_done
            transition_done = False
            lane_span = max(1e-6, abs(float(request.next_lane_y_m) - float(request.current_lane_y_m)))
            progress = 1.0 - min(1.0, abs(lane_error) / lane_span)
        elif segment == "turn_in_90":
            segment_done = next_yaw_done
            transition_done = False
            progress = 1.0 - min(1.0, abs(next_yaw_error) / max(math.radians(90.0), 1e-6))
        elif segment == "acquire_next_row":
            segment_done = bool(lane_done and next_yaw_done and request.forward_corridor_safe)
            transition_done = segment_done
            progress = 1.0 if transition_done else 0.5 * float(lane_done) + 0.5 * float(next_yaw_done)
        else:
            segment_done = False
            transition_done = False
            progress = 0.0

        drive_scale = max(1e-6, float(request.drive_speed_factor))
        min_v = max(0.0, float(request.min_emitted_v_mps)) / drive_scale
        inner_min = max(0.0, float(request.inner_wheel_min_mps)) / drive_scale
        max_v = max(min_v, float(request.max_v_mps))

        if segment == "cross_lane":
            omega = self._yaw_command(yaw_error, request)
            v = min(max_v, max(min_v, inner_min + 0.5 * request.track_width_m * abs(omega)))
        elif segment in {"turn_out_90", "turn_in_90", "acquire_next_row"}:
            omega = self._yaw_command(yaw_error, request)
            v = min(max_v, max(min_v, inner_min + 0.5 * request.track_width_m * abs(omega)))
        else:
            omega = 0.0
            v = min_v

        if segment_done:
            omega = 0.0 if segment == "acquire_next_row" else omega
            v = min_v

        v, omega, left, right, raised_v, reduced_omega = self._apply_transition_wheel_floor(
            v,
            omega,
            track_width_m=request.track_width_m,
            max_v_mps=max_v,
            inner_wheel_min_mps=inner_min,
        )
        target = self._target_for_segment(request, target_yaw)
        emitted_v = v * drive_scale
        emitted_omega = omega * drive_scale
        emitted_left = left * drive_scale
        emitted_right = right * drive_scale

        reasons = []
        if segment in {"turn_out_90", "cross_lane"} and not turn_90_done:
            reasons.append("side_yaw_not_captured")
        if segment in {"cross_lane", "acquire_next_row"} and not lane_done:
            reasons.append("lane_not_captured")
        if segment in {"turn_in_90", "acquire_next_row"} and not next_yaw_done:
            reasons.append("next_row_yaw_not_captured")
        if segment == "acquire_next_row" and not request.forward_corridor_safe:
            reasons.append("forward_corridor_unsafe")
        reason = "segment_complete" if segment_done else "_".join(reasons or ["segment_tracking"])
        debug = {
            "row_transition_style": self.style,
            "row_transition_segment": segment,
            "row_transition_progress": round(clamp(progress, 0.0, 1.0), 3),
            "row_transition_done": bool(transition_done),
            "row_transition_reason": reason,
            "row_transition_target_m": [round(float(target[0]), 3), round(float(target[1]), 3)],
            "turn_side": self.turn_side(direction, request.current_lane_y_m, request.next_lane_y_m),
            "target_side_yaw_deg": round(yaw_deg(side_yaw), 2),
            "target_next_row_yaw_deg": round(yaw_deg(next_yaw), 2),
            "target_row_yaw_deg": round(yaw_deg(next_yaw), 2),
            "segment_yaw_error_deg": round(math.degrees(yaw_error), 3),
            "segment_lane_error_m": round(lane_error, 4),
            "yaw_capture_error_deg": round(math.degrees(next_yaw_error), 3),
            "lane_capture_error_m": round(lane_error, 4),
            "transition_emitted_v_mps": round(emitted_v, 4),
            "transition_emitted_omega_radps": round(emitted_omega, 4),
            "transition_left_target_mps": round(emitted_left, 4),
            "transition_right_target_mps": round(emitted_right, 4),
            "desired_v_mps": round(v, 4),
            "desired_omega_radps": round(omega, 4),
            "desired_left_target_mps": round(left, 4),
            "desired_right_target_mps": round(right, 4),
            "transition_inner_wheel_min_mps": round(float(request.inner_wheel_min_mps), 4),
            "transition_min_emitted_v_mps": round(float(request.min_emitted_v_mps), 4),
            "transition_min_emitted_omega_radps": round(float(request.min_emitted_omega_radps), 4),
            "transition_max_emitted_omega_radps": round(float(request.max_emitted_omega_radps), 4),
            "transition_velocity_raised_for_inner_wheel": bool(raised_v),
            "transition_omega_reduced_for_inner_wheel": bool(reduced_omega),
            "forward_corridor_safe": bool(request.forward_corridor_safe),
            "drive_speed_factor": round(drive_scale, 4),
        }
        return RowTransitionCommand(
            desired_v_mps=v,
            desired_omega_radps=omega,
            active_target_m=target,
            segment_done=bool(segment_done),
            transition_done=bool(transition_done),
            debug=debug,
        )


__all__ = [
    "SEGMENTED_TURN_STYLE",
    "TRANSITION_SEGMENTS",
    "RowTransitionCommand",
    "RowTransitionController",
    "RowTransitionRequest",
    "clamp",
    "wheel_targets_from_v_omega",
    "wrap_to_pi",
    "yaw_deg",
]
