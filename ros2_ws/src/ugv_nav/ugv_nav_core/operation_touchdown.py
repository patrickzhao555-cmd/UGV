"""Pure helpers for Operation Touchdown mission logic.

The functions in this module intentionally avoid ROS imports.  They encode the
competition-specific rules that must stay consistent across the Nav2 mission
supervisor, ArUco terminal approach, and unit tests.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from .mission_controller import COMPETITION_MIN_SPEED_MPS, MOTION_EPSILON, apply_competition_speed_rule
from .nav2_bridge import (
    FieldBounds,
    OccupancyGridSpec,
    Transform2D,
    VelocityCommand,
    build_stop_command,
    build_velocity_command,
    occupancy_allows_target,
    validate_field_target,
    wrap_pi,
)


ARUCO_MARKER_SIDE_M = 0.3048
DESTINATION_RADIUS_M = 1.524
DEFAULT_ALLOWED_ARUCO_IDS = (0, 1, 2, 3, 4)


@dataclass(frozen=True)
class StagingPose:
    x_m: float
    y_m: float
    yaw_rad: float
    distance_to_marker_m: float
    reason: str = "ok"


@dataclass(frozen=True)
class StagingDecision:
    accepted: bool
    pose: Optional[StagingPose] = None
    reason: str = "ok"


@dataclass(frozen=True)
class TerminalCommandDecision:
    command: VelocityCommand
    destination_reached: bool
    distance_to_marker_m: Optional[float]
    heading_error_rad: Optional[float]
    reason: str


@dataclass(frozen=True)
class CoordinateArrivalDecision:
    destination_reached: bool
    distance_to_target_m: Optional[float]
    reason: str


def parse_allowed_marker_ids(raw: object, *, default: Sequence[int] = DEFAULT_ALLOWED_ARUCO_IDS) -> tuple[int, ...]:
    """Parse a comma/space separated marker ID list.

    Invalid entries are ignored, duplicates are removed, and the final order is
    sorted to keep telemetry stable.
    """
    if raw is None:
        return tuple(default)
    if isinstance(raw, (list, tuple, set)):
        values = raw
    else:
        text = str(raw).strip()
        if not text:
            return tuple(default)
        values = text.replace(";", ",").replace(" ", ",").split(",")
    parsed: set[int] = set()
    for value in values:
        try:
            parsed.add(int(str(value).strip()))
        except (TypeError, ValueError):
            continue
    return tuple(sorted(parsed)) if parsed else tuple(default)


def destination_reached_by_coordinate(
    *,
    robot_x_m: float,
    robot_y_m: float,
    target_x_m: float,
    target_y_m: float,
    destination_radius_m: float = DESTINATION_RADIUS_M,
    buffer_m: float = 0.0,
) -> bool:
    if not all(math.isfinite(float(v)) for v in (robot_x_m, robot_y_m, target_x_m, target_y_m)):
        return False
    allowed = max(0.0, float(destination_radius_m) - max(0.0, float(buffer_m)))
    return math.hypot(float(robot_x_m) - float(target_x_m), float(robot_y_m) - float(target_y_m)) <= allowed


def coordinate_arrival_decision(
    *,
    robot_pose: Transform2D,
    target_x_m: float,
    target_y_m: float,
    marker_confirmed: bool = False,
    destination_radius_m: float = DESTINATION_RADIUS_M,
    buffer_m: float = 0.0,
) -> CoordinateArrivalDecision:
    """Evaluate the rulebook destination circle using the UAV coordinate.

    Reaching the coordinate destination circle is a hard terminal condition for
    Operation Touchdown.  ArUco may confirm the stop, but it must not command
    further motion once this check is satisfied.
    """

    values = (
        robot_pose.x,
        robot_pose.y,
        target_x_m,
        target_y_m,
        destination_radius_m,
        buffer_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return CoordinateArrivalDecision(False, None, "coordinate_pose_or_target_invalid")

    distance = math.hypot(float(robot_pose.x) - float(target_x_m), float(robot_pose.y) - float(target_y_m))
    allowed = max(0.0, float(destination_radius_m) - max(0.0, float(buffer_m)))
    if distance <= allowed:
        reason = (
            "destination_reached_coordinate_marker_confirmed"
            if bool(marker_confirmed)
            else "destination_reached_by_coordinate_no_marker_visual"
        )
        return CoordinateArrivalDecision(True, distance, reason)
    return CoordinateArrivalDecision(False, distance, "destination_not_reached_by_coordinate")


def choose_marker_staging_pose(
    *,
    robot_pose: Transform2D,
    marker_x_m: float,
    marker_y_m: float,
    bounds: FieldBounds,
    occupancy_grid: Optional[OccupancyGridSpec] = None,
    destination_radius_m: float = DESTINATION_RADIUS_M,
    preferred_distances_m: Sequence[float] = (1.2, 1.0, 0.9),
    angular_samples: int = 24,
    footprint_radius_m: float = 0.45,
) -> StagingDecision:
    """Choose a safe pose inside the destination circle that faces the marker."""
    if not all(
        math.isfinite(float(value))
        for value in (
            robot_pose.x,
            robot_pose.y,
            robot_pose.yaw,
            marker_x_m,
            marker_y_m,
            bounds.width_m,
            bounds.height_m,
            bounds.margin_m,
            destination_radius_m,
        )
    ):
        return StagingDecision(False, reason="target_or_pose_not_finite")
    if destination_radius_m <= 0.0:
        return StagingDecision(False, reason="destination_radius_invalid")

    target_validation = validate_field_target(
        x_m=float(marker_x_m),
        y_m=float(marker_y_m),
        bounds=bounds,
        occupancy_grid=None,
        allow_projection=False,
    )
    if not target_validation.accepted:
        return StagingDecision(False, reason=target_validation.reason)

    robot_angle_from_marker = math.atan2(robot_pose.y - float(marker_y_m), robot_pose.x - float(marker_x_m))
    candidate_angles = [robot_angle_from_marker]
    samples = max(4, int(angular_samples))
    for step in range(samples):
        candidate_angles.append(-math.pi + (2.0 * math.pi * step) / float(samples))

    best: Optional[tuple[float, StagingPose]] = None
    for distance in preferred_distances_m:
        distance_m = max(0.05, min(float(distance), float(destination_radius_m)))
        for angle in candidate_angles:
            x_m = float(marker_x_m) + distance_m * math.cos(angle)
            y_m = float(marker_y_m) + distance_m * math.sin(angle)
            validation = validate_field_target(
                x_m=x_m,
                y_m=y_m,
                bounds=bounds,
                occupancy_grid=occupancy_grid,
                allow_projection=False,
            )
            if not validation.accepted:
                continue
            if not footprint_allows_pose(x_m, y_m, occupancy_grid, radius_m=footprint_radius_m):
                continue
            yaw_rad = math.atan2(float(marker_y_m) - y_m, float(marker_x_m) - x_m)
            travel_distance = math.hypot(robot_pose.x - x_m, robot_pose.y - y_m)
            heading_change = abs(wrap_pi(yaw_rad - robot_pose.yaw))
            score = travel_distance + 0.20 * heading_change + 0.05 * abs(distance_m - preferred_distances_m[0])
            pose = StagingPose(
                x_m=x_m,
                y_m=y_m,
                yaw_rad=wrap_pi(yaw_rad),
                distance_to_marker_m=distance_m,
            )
            if best is None or score < best[0]:
                best = (score, pose)
    if best is None:
        return StagingDecision(False, reason="no_safe_marker_staging_pose")
    return StagingDecision(True, pose=best[1])


def footprint_allows_pose(
    x_m: float,
    y_m: float,
    occupancy_grid: Optional[OccupancyGridSpec],
    *,
    radius_m: float = 0.45,
    samples: int = 8,
) -> bool:
    if occupancy_grid is None:
        return True
    if not occupancy_allows_target(float(x_m), float(y_m), occupancy_grid):
        return False
    radius = max(0.0, float(radius_m))
    if radius <= 0.0:
        return True
    sample_count = max(4, int(samples))
    for idx in range(sample_count):
        angle = (2.0 * math.pi * idx) / float(sample_count)
        sx = float(x_m) + radius * math.cos(angle)
        sy = float(y_m) + radius * math.sin(angle)
        if not occupancy_allows_target(sx, sy, occupancy_grid):
            return False
    return True


def terminal_approach_command(
    *,
    robot_pose: Transform2D,
    marker_x_m: float,
    marker_y_m: float,
    marker_fresh: bool,
    destination_radius_m: float = DESTINATION_RADIUS_M,
    terminal_stop_distance_m: float = 1.0,
    terminal_arrival_buffer_m: float = 0.20,
    forward_speed_mps: float = 0.10,
    heading_kp: float = 0.8,
    max_omega_radps: float = 0.20,
    pivot_heading_error_rad: float = 0.35,
    min_speed_mps: float = COMPETITION_MIN_SPEED_MPS,
) -> TerminalCommandDecision:
    """Return a legal command for final marker approach.

    The controller never emits a sub-minimum nonzero translational speed.  If
    the marker is not fresh, it either stops because coordinate-based arrival is
    already satisfied or holds STOP with marker_lost.
    """
    if not all(
        math.isfinite(float(value))
        for value in (
            robot_pose.x,
            robot_pose.y,
            robot_pose.yaw,
            marker_x_m,
            marker_y_m,
            destination_radius_m,
            terminal_stop_distance_m,
            terminal_arrival_buffer_m,
            forward_speed_mps,
            heading_kp,
            max_omega_radps,
            pivot_heading_error_rad,
            min_speed_mps,
        )
    ):
        return TerminalCommandDecision(
            build_stop_command("marker_pose_invalid"),
            False,
            None,
            None,
            "marker_pose_invalid",
        )

    distance = math.hypot(float(marker_x_m) - robot_pose.x, float(marker_y_m) - robot_pose.y)
    bearing = math.atan2(float(marker_y_m) - robot_pose.y, float(marker_x_m) - robot_pose.x)
    heading_error = wrap_pi(bearing - robot_pose.yaw)
    if distance <= max(0.0, float(destination_radius_m) - max(0.0, float(terminal_arrival_buffer_m))):
        reason = "destination_reached"
        return TerminalCommandDecision(build_stop_command(reason), True, distance, heading_error, reason)
    if not bool(marker_fresh):
        reason = "marker_lost"
        return TerminalCommandDecision(build_stop_command(reason), False, distance, heading_error, reason)
    if distance <= max(0.0, float(terminal_stop_distance_m)):
        reason = "destination_reached"
        return TerminalCommandDecision(build_stop_command(reason), True, distance, heading_error, reason)

    omega = max(-float(max_omega_radps), min(float(max_omega_radps), float(heading_kp) * heading_error))
    if abs(heading_error) > max(0.0, float(pivot_heading_error_rad)):
        reason = "terminal_align_to_marker"
        return TerminalCommandDecision(
            build_velocity_command(0.0, omega, reason),
            False,
            distance,
            heading_error,
            reason,
        )

    v_cmd = apply_competition_speed_rule(float(forward_speed_mps), min_speed_mps=float(min_speed_mps))
    if abs(v_cmd) < MOTION_EPSILON:
        reason = "terminal_forward_speed_invalid"
        return TerminalCommandDecision(build_stop_command(reason), False, distance, heading_error, reason)
    reason = "terminal_approach_marker"
    return TerminalCommandDecision(
        build_velocity_command(v_cmd, omega, reason),
        False,
        distance,
        heading_error,
        reason,
    )
