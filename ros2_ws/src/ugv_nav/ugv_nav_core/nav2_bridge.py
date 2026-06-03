"""Pure helpers for Nav2 field navigation integration.

These helpers intentionally avoid ROS imports so command contracts, field
target validation, and 2D transform math can be unit-tested off-robot.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Iterable, Optional, Sequence, Tuple

from .mission_controller import (
    COMPETITION_MIN_SPEED_MPS,
    MOTION_EPSILON,
    apply_competition_speed_rule,
    motion_rule_ok,
)


@dataclass(frozen=True)
class VelocityCommand:
    command_type: str
    v_mps: float
    omega_radps: float
    reason: str
    controller: str = "ugv_nav2_adapter"

    @property
    def motion_rule_ok(self) -> bool:
        return motion_rule_ok(self.v_mps)

    def to_payload(self) -> dict[str, Any]:
        mode = "STOP" if self.command_type == "stop" else "VELOCITY"
        return {
            "mode": mode,
            "command_type": self.command_type,
            "controller": self.controller,
            "v_mps": float(self.v_mps),
            "omega_radps": float(self.omega_radps),
            "reason": self.reason,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_payload(), sort_keys=True)


@dataclass(frozen=True)
class Transform2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class FieldBounds:
    width_m: float
    height_m: float
    margin_m: float = 0.25


@dataclass(frozen=True)
class OccupancyGridSpec:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    data: Sequence[int]
    lethal_threshold: int = 100
    unknown_is_blocked: bool = False


@dataclass(frozen=True)
class FieldTargetValidation:
    accepted: bool
    x_m: float
    y_m: float
    reason: str = "ok"
    adjusted: bool = False


@dataclass(frozen=True)
class GyroBiasEvaluation:
    accepted: bool
    mean_radps: float
    stddev_radps: float
    sample_count: int
    reason: str


@dataclass(frozen=True)
class ImuTimingStep:
    dt_s: Optional[float]
    stamp_s: Optional[float]
    time_source: str
    skipped: bool
    reason: str


@dataclass(frozen=True)
class FieldBoundaryDecision:
    safe: bool
    reason: str = "ok"


@dataclass(frozen=True)
class RollingArcLimitResult:
    command: VelocityCommand
    side_reverse_blocked: bool = False
    omega_clamped: bool = False
    left_mps: float = 0.0
    right_mps: float = 0.0
    omega_limit_radps: float = 0.0
    reason: str = "ok"


def build_stop_command(reason: str, *, controller: str = "ugv_nav2_adapter") -> VelocityCommand:
    return VelocityCommand(
        command_type="stop",
        v_mps=0.0,
        omega_radps=0.0,
        reason=str(reason),
        controller=controller,
    )


def build_velocity_command(
    v_mps: float,
    omega_radps: float,
    reason: str,
    *,
    controller: str = "ugv_nav2_adapter",
) -> VelocityCommand:
    return VelocityCommand(
        command_type="velocity",
        v_mps=float(v_mps),
        omega_radps=float(omega_radps),
        reason=str(reason),
        controller=controller,
    )


def nav_command_from_twist(
    *,
    linear_x_mps: float,
    angular_z_radps: float,
    linear_y_mps: float = 0.0,
    competition_min_speed_mps: float = COMPETITION_MIN_SPEED_MPS,
    allow_reverse: bool = False,
    lateral_epsilon_mps: float = 1e-4,
    angular_epsilon_radps: float = 1e-5,
    controller: str = "ugv_nav2_adapter",
) -> VelocityCommand:
    """Convert a Nav2 Twist command into the project's velocity JSON contract."""
    raw_v = float(linear_x_mps)
    raw_y = float(linear_y_mps)
    raw_omega = float(angular_z_radps)
    min_speed = float(competition_min_speed_mps)
    lateral_epsilon = float(lateral_epsilon_mps)
    angular_epsilon = float(angular_epsilon_radps)
    if not all(
        math.isfinite(value)
        for value in (raw_v, raw_y, raw_omega, min_speed, lateral_epsilon, angular_epsilon)
    ):
        return build_stop_command("invalid_cmd_vel", controller=controller)

    if min_speed < 0.0 or lateral_epsilon < 0.0 or angular_epsilon < 0.0:
        return build_stop_command("invalid_cmd_vel", controller=controller)

    if abs(raw_y) > lateral_epsilon:
        return build_stop_command("unsupported_lateral_cmd", controller=controller)

    if raw_v < -MOTION_EPSILON and not bool(allow_reverse):
        return build_stop_command("reverse_cmd_blocked", controller=controller)

    if abs(raw_v) < MOTION_EPSILON and abs(raw_omega) < angular_epsilon:
        return build_stop_command("cmd_vel_stop", controller=controller)

    if abs(raw_v) < MOTION_EPSILON:
        v_cmd = 0.0
    else:
        v_cmd = apply_competition_speed_rule(
            raw_v,
            allow_stop=True,
            min_speed_mps=min_speed,
        )
    reason = "nav2_cmd_vel" if abs(v_cmd - raw_v) < 1e-9 else "nav2_cmd_vel_speed_clamped"
    return build_velocity_command(v_cmd, raw_omega, reason, controller=controller)


def side_speeds_for_chassis(v_mps: float, omega_radps: float, *, track_width_m: float) -> tuple[float, float]:
    """Return left/right side speeds for the project's differential velocity convention."""
    half_track = 0.5 * max(1e-9, float(track_width_m))
    v = float(v_mps)
    omega = float(omega_radps)
    return v - omega * half_track, v + omega * half_track


def limit_rolling_arc_command(
    command: VelocityCommand,
    *,
    track_width_m: float,
    min_turn_radius_m: float,
    max_omega_radps: Optional[float] = None,
    allow_side_reverse: bool = False,
    controller: str = "ugv_nav2_adapter",
) -> RollingArcLimitResult:
    """Constrain velocity commands to rolling arcs before autonomous motor output.

    With ``allow_side_reverse=False`` the command must keep both sides rolling in
    the same translational direction.  Pure pivot commands are therefore blocked
    in autonomous Nav2, while normal forward arcs are allowed or gently angular-
    rate limited to the configured minimum turn radius.
    """
    if command.command_type != "velocity":
        left, right = side_speeds_for_chassis(command.v_mps, command.omega_radps, track_width_m=max(1e-9, float(track_width_m)))
        return RollingArcLimitResult(command=command, left_mps=left, right_mps=right)

    v = float(command.v_mps)
    omega = float(command.omega_radps)
    track = float(track_width_m)
    radius = float(min_turn_radius_m)
    max_omega = None if max_omega_radps is None else float(max_omega_radps)
    finite_values = [v, omega, track, radius]
    if max_omega is not None:
        finite_values.append(max_omega)
    if not all(math.isfinite(value) for value in finite_values) or track <= 0.0 or radius <= 0.0:
        stop = build_stop_command("invalid_arc_params", controller=controller)
        return RollingArcLimitResult(command=stop, side_reverse_blocked=True, reason="invalid_arc_params")

    left, right = side_speeds_for_chassis(v, omega, track_width_m=track)
    if abs(omega) < MOTION_EPSILON:
        return RollingArcLimitResult(command=command, left_mps=left, right_mps=right, reason="ok")

    if allow_side_reverse:
        omega_limit = abs(max_omega) if max_omega is not None else abs(omega)
    else:
        if abs(v) < MOTION_EPSILON:
            stop = build_stop_command("side_reverse_blocked", controller=controller)
            return RollingArcLimitResult(
                command=stop,
                side_reverse_blocked=True,
                left_mps=left,
                right_mps=right,
                reason="side_reverse_blocked",
            )
        omega_limit = min(abs(v) / radius, 2.0 * abs(v) / track)
        if max_omega is not None:
            omega_limit = min(omega_limit, abs(max_omega))

    if omega_limit < MOTION_EPSILON:
        stop = build_stop_command("side_reverse_blocked", controller=controller)
        return RollingArcLimitResult(
            command=stop,
            side_reverse_blocked=True,
            left_mps=left,
            right_mps=right,
            omega_limit_radps=omega_limit,
            reason="side_reverse_blocked",
        )

    clamped_omega = max(-omega_limit, min(omega_limit, omega))
    clamped = abs(clamped_omega - omega) > 1e-9
    out = command
    if clamped:
        out = build_velocity_command(v, clamped_omega, "rolling_arc_omega_clamped", controller=command.controller)
        left, right = side_speeds_for_chassis(v, clamped_omega, track_width_m=track)

    if not allow_side_reverse and left * right < -1e-9:
        stop = build_stop_command("side_reverse_blocked", controller=controller)
        return RollingArcLimitResult(
            command=stop,
            side_reverse_blocked=True,
            left_mps=left,
            right_mps=right,
            omega_limit_radps=omega_limit,
            reason="side_reverse_blocked",
        )

    return RollingArcLimitResult(
        command=out,
        omega_clamped=clamped,
        left_mps=left,
        right_mps=right,
        omega_limit_radps=omega_limit,
        reason="rolling_arc_omega_clamped" if clamped else "ok",
    )


def evaluate_gyro_bias_samples(
    samples: Sequence[float],
    *,
    max_stddev_radps: float = 0.03,
    encoder_start_ticks: Optional[tuple[int, int]] = None,
    encoder_current_ticks: Optional[tuple[int, int]] = None,
    max_encoder_delta_ticks: int = 2,
) -> GyroBiasEvaluation:
    sample_count = len(samples)
    if sample_count <= 0:
        return GyroBiasEvaluation(False, 0.0, 0.0, 0, "gyro_bias_no_samples")
    values = [float(sample) for sample in samples]
    if not all(math.isfinite(value) for value in values):
        return GyroBiasEvaluation(False, 0.0, 0.0, sample_count, "gyro_bias_non_finite")
    mean = sum(values) / float(sample_count)
    variance = sum((value - mean) ** 2 for value in values) / float(sample_count)
    stddev = math.sqrt(max(0.0, variance))
    if stddev > max(0.0, float(max_stddev_radps)):
        return GyroBiasEvaluation(False, mean, stddev, sample_count, "gyro_bias_unstable")
    if encoder_start_ticks is not None and encoder_current_ticks is not None:
        left_delta = abs(int(encoder_current_ticks[0]) - int(encoder_start_ticks[0]))
        right_delta = abs(int(encoder_current_ticks[1]) - int(encoder_start_ticks[1]))
        if max(left_delta, right_delta) > max(0, int(max_encoder_delta_ticks)):
            return GyroBiasEvaluation(False, mean, stddev, sample_count, "gyro_bias_encoder_motion")
    return GyroBiasEvaluation(True, mean, stddev, sample_count, "gyro_bias_ready")


def select_imu_timing_step(
    *,
    stamp_s: Optional[float],
    fallback_now_s: float,
    last_stamp_s: Optional[float],
    last_fallback_s: Optional[float],
    min_dt_s: float = 0.001,
    max_dt_s: float = 0.05,
) -> ImuTimingStep:
    min_dt = max(0.0, float(min_dt_s))
    max_dt = max(min_dt, float(max_dt_s))
    fallback_now = float(fallback_now_s)
    stamp_valid = stamp_s is not None and math.isfinite(float(stamp_s)) and float(stamp_s) > 0.0
    if stamp_valid and (last_stamp_s is None or float(stamp_s) > float(last_stamp_s)):
        if last_stamp_s is None:
            return ImuTimingStep(None, float(stamp_s), "header_stamp", True, "first_imu_sample")
        dt_s = float(stamp_s) - float(last_stamp_s)
        if min_dt <= dt_s <= max_dt:
            return ImuTimingStep(dt_s, float(stamp_s), "header_stamp", False, "ok")
        return ImuTimingStep(None, float(stamp_s), "header_stamp", True, "imu_dt_out_of_range")

    if last_fallback_s is None:
        return ImuTimingStep(None, None, "monotonic", True, "first_imu_sample")
    dt_s = fallback_now - float(last_fallback_s)
    if min_dt <= dt_s <= max_dt:
        return ImuTimingStep(dt_s, None, "monotonic_fallback", False, "stamp_invalid_or_nonmonotonic")
    return ImuTimingStep(None, None, "monotonic_fallback", True, "imu_dt_out_of_range")


def angle_within_centered_fov(angle_rad: float, fov_deg: float, *, center_rad: float = 0.0) -> bool:
    half_fov = 0.5 * math.radians(max(0.0, min(360.0, float(fov_deg))))
    return abs(wrap_pi(float(angle_rad) - float(center_rad))) <= half_fov


def field_boundary_decision(
    *,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    v_mps: float,
    omega_radps: float = 0.0,
    bounds: FieldBounds,
    safety_margin_m: float,
    prediction_time_s: float = 0.75,
    epsilon_mps: float = MOTION_EPSILON,
) -> FieldBoundaryDecision:
    x = float(x_m)
    y = float(y_m)
    yaw = float(yaw_rad)
    v = float(v_mps)
    omega = float(omega_radps)
    margin = max(0.0, float(safety_margin_m))
    horizon = max(0.0, float(prediction_time_s))
    width = float(bounds.width_m)
    height = float(bounds.height_m)
    if not all(math.isfinite(value) for value in (x, y, yaw, v, omega, margin, horizon, width, height)):
        return FieldBoundaryDecision(False, "field_pose_invalid")
    if width <= 0.0 or height <= 0.0 or margin * 2.0 >= min(width, height):
        return FieldBoundaryDecision(False, "field_bounds_invalid")
    min_x = margin
    max_x = width - margin
    min_y = margin
    max_y = height - margin
    if abs(v) < max(0.0, float(epsilon_mps)):
        if x < 0.0 or x > width or y < 0.0 or y > height:
            return FieldBoundaryDecision(False, "field_boundary_outside")
        return FieldBoundaryDecision(True, "ok")

    if x < min_x:
        return FieldBoundaryDecision(False, "field_boundary_x_min")
    if x > max_x:
        return FieldBoundaryDecision(False, "field_boundary_x_max")
    if y < min_y:
        return FieldBoundaryDecision(False, "field_boundary_y_min")
    if y > max_y:
        return FieldBoundaryDecision(False, "field_boundary_y_max")

    if horizon > 0.0:
        steps = max(1, min(30, int(math.ceil(horizon / 0.05))))
        dt_s = horizon / float(steps)
        predicted_x = x
        predicted_y = y
        predicted_yaw = yaw
        for _ in range(steps):
            mid_yaw = wrap_pi(predicted_yaw + 0.5 * omega * dt_s)
            predicted_x += v * dt_s * math.cos(mid_yaw)
            predicted_y += v * dt_s * math.sin(mid_yaw)
            predicted_yaw = wrap_pi(predicted_yaw + omega * dt_s)
            if predicted_x < min_x or predicted_x > max_x or predicted_y < min_y or predicted_y > max_y:
                return FieldBoundaryDecision(False, "field_boundary_predicted_exit")
    return FieldBoundaryDecision(True, "ok")


def wrap_pi(angle_rad: float) -> float:
    wrapped = (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped == -math.pi and angle_rad > 0.0:
        return math.pi
    return wrapped


def compose_transform_2d(a: Transform2D, b: Transform2D) -> Transform2D:
    cos_yaw = math.cos(a.yaw)
    sin_yaw = math.sin(a.yaw)
    return Transform2D(
        x=a.x + cos_yaw * b.x - sin_yaw * b.y,
        y=a.y + sin_yaw * b.x + cos_yaw * b.y,
        yaw=wrap_pi(a.yaw + b.yaw),
    )


def inverse_transform_2d(t: Transform2D) -> Transform2D:
    cos_yaw = math.cos(t.yaw)
    sin_yaw = math.sin(t.yaw)
    return Transform2D(
        x=-(cos_yaw * t.x + sin_yaw * t.y),
        y=sin_yaw * t.x - cos_yaw * t.y,
        yaw=wrap_pi(-t.yaw),
    )


def map_to_odom_from_pose(
    *,
    desired_map_base: Transform2D,
    current_odom_base: Transform2D,
) -> Transform2D:
    return compose_transform_2d(desired_map_base, inverse_transform_2d(current_odom_base))


def integrate_planar_odometry(
    pose: Transform2D,
    *,
    distance_m: float,
    delta_yaw_rad: float,
) -> Transform2D:
    mid_yaw = wrap_pi(pose.yaw + 0.5 * float(delta_yaw_rad))
    return Transform2D(
        x=pose.x + float(distance_m) * math.cos(mid_yaw),
        y=pose.y + float(distance_m) * math.sin(mid_yaw),
        yaw=wrap_pi(pose.yaw + float(delta_yaw_rad)),
    )


def project_target_to_field(x_m: float, y_m: float, bounds: FieldBounds) -> Tuple[float, float, bool]:
    margin = max(0.0, float(bounds.margin_m))
    min_x = margin
    min_y = margin
    max_x = max(min_x, float(bounds.width_m) - margin)
    max_y = max(min_y, float(bounds.height_m) - margin)
    projected_x = min(max(float(x_m), min_x), max_x)
    projected_y = min(max(float(y_m), min_y), max_y)
    return projected_x, projected_y, (projected_x != float(x_m) or projected_y != float(y_m))


def _grid_index_for_xy(x_m: float, y_m: float, grid: OccupancyGridSpec) -> Optional[int]:
    if grid.resolution_m <= 0.0 or grid.width <= 0 or grid.height <= 0:
        return None
    mx = int(math.floor((float(x_m) - float(grid.origin_x_m)) / float(grid.resolution_m)))
    my = int(math.floor((float(y_m) - float(grid.origin_y_m)) / float(grid.resolution_m)))
    if mx < 0 or my < 0 or mx >= int(grid.width) or my >= int(grid.height):
        return None
    return my * int(grid.width) + mx


def occupancy_cost_at(x_m: float, y_m: float, grid: OccupancyGridSpec) -> Optional[int]:
    index = _grid_index_for_xy(x_m, y_m, grid)
    if index is None or index >= len(grid.data):
        return None
    return int(grid.data[index])


def occupancy_allows_target(x_m: float, y_m: float, grid: Optional[OccupancyGridSpec]) -> bool:
    if grid is None:
        return True
    cost = occupancy_cost_at(x_m, y_m, grid)
    if cost is None:
        return False
    if cost < 0:
        return not bool(grid.unknown_is_blocked)
    return cost < int(grid.lethal_threshold)


def occupancy_target_rejection_reason(x_m: float, y_m: float, grid: Optional[OccupancyGridSpec]) -> Optional[str]:
    if grid is None:
        return None
    cost = occupancy_cost_at(x_m, y_m, grid)
    if cost is None:
        return "target_unknown"
    if cost < 0:
        return "target_unknown" if bool(grid.unknown_is_blocked) else None
    if cost >= int(grid.lethal_threshold):
        return "target_occupied"
    return None


def target_units_scale(units: str) -> Optional[float]:
    normalized = str(units).strip().lower()
    if normalized in {"m", "meter", "meters", "metre", "metres"}:
        return 1.0
    if normalized in {"yd", "yard", "yards"}:
        return 0.9144
    return None


def find_nearest_free_target(
    *,
    x_m: float,
    y_m: float,
    grid: OccupancyGridSpec,
    max_search_radius_m: float = 1.0,
) -> Optional[tuple[float, float]]:
    if grid.resolution_m <= 0.0:
        return None
    start_index = _grid_index_for_xy(x_m, y_m, grid)
    if start_index is None:
        return None
    start_mx = start_index % int(grid.width)
    start_my = start_index // int(grid.width)
    max_cells = max(0, int(math.ceil(float(max_search_radius_m) / float(grid.resolution_m))))
    best: Optional[tuple[float, float, float]] = None
    for dy in range(-max_cells, max_cells + 1):
        for dx in range(-max_cells, max_cells + 1):
            mx = start_mx + dx
            my = start_my + dy
            if mx < 0 or my < 0 or mx >= int(grid.width) or my >= int(grid.height):
                continue
            candidate_x = float(grid.origin_x_m) + (mx + 0.5) * float(grid.resolution_m)
            candidate_y = float(grid.origin_y_m) + (my + 0.5) * float(grid.resolution_m)
            distance = math.hypot(candidate_x - float(x_m), candidate_y - float(y_m))
            if distance > float(max_search_radius_m):
                continue
            if not occupancy_allows_target(candidate_x, candidate_y, grid):
                continue
            if best is None or distance < best[0]:
                best = (distance, candidate_x, candidate_y)
    if best is None:
        return None
    return best[1], best[2]


def validate_field_target(
    *,
    x_m: float,
    y_m: float,
    bounds: FieldBounds,
    occupancy_grid: Optional[OccupancyGridSpec] = None,
    allow_projection: bool = False,
) -> FieldTargetValidation:
    if not math.isfinite(float(x_m)) or not math.isfinite(float(y_m)):
        return FieldTargetValidation(False, float(x_m), float(y_m), "target_not_finite")

    projected_x, projected_y, adjusted = project_target_to_field(float(x_m), float(y_m), bounds)
    if adjusted and not allow_projection:
        return FieldTargetValidation(False, float(x_m), float(y_m), "target_out_of_field")

    rejection_reason = occupancy_target_rejection_reason(projected_x, projected_y, occupancy_grid)
    if rejection_reason is not None:
        return FieldTargetValidation(False, projected_x, projected_y, rejection_reason, adjusted=adjusted)

    return FieldTargetValidation(True, projected_x, projected_y, "ok", adjusted=adjusted)


def finite_xyz_points(points: Iterable[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    result: list[tuple[float, float, float]] = []
    for x, y, z in points:
        xf = float(x)
        yf = float(y)
        zf = float(z)
        if math.isfinite(xf) and math.isfinite(yf) and math.isfinite(zf):
            result.append((xf, yf, zf))
    return result
