"""Pure helpers for Challenge 3 large-radius corridor bypass control.

Challenge 3 is intentionally different from the small-field Challenge 2
controller: the UGV has a large practical turn radius, so obstacle avoidance is
modeled as early lane offset selection around the start-to-target route instead
of last-moment waypoint dodging.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence

from .chassis_controller import clamp, encoder_ticks_to_distance_m, wrap_pi
from .nav2_bridge import VelocityCommand, build_stop_command, build_velocity_command


CHALLENGE3_DESTINATION_RADIUS_M = 1.524
CHALLENGE3_ARRIVAL_BUFFER_M = 0.20
CHALLENGE3_DEFAULT_LANE_OFFSETS_M = (0.0, 1.6, -1.6, 2.2, -2.2)
CHALLENGE3_TURN_ENVELOPE_SPEED_MPS = 1.70
CHALLENGE3_TURN_ENVELOPE_OMEGA_RADPS = 7.80
CHALLENGE3_TURN_ENVELOPE_MIN_RADIUS_M = (
    CHALLENGE3_TURN_ENVELOPE_SPEED_MPS / CHALLENGE3_TURN_ENVELOPE_OMEGA_RADPS
)


@dataclass(frozen=True)
class FieldPose:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class RouteFrame:
    start_x_m: float
    start_y_m: float
    target_x_m: float
    target_y_m: float
    ux: float
    uy: float
    nx: float
    ny: float
    length_m: float


@dataclass(frozen=True)
class RoutePoint:
    u_m: float
    v_m: float


@dataclass(frozen=True)
class RouteObservation:
    u_m: float
    v_m: float
    field_x_m: float
    field_y_m: float
    robot_forward_m: float
    robot_left_m: float
    range_m: float


@dataclass
class ObstacleMemoryPoint:
    u_m: float
    v_m: float
    field_x_m: float
    field_y_m: float
    first_seen_s: float
    last_seen_s: float


@dataclass(frozen=True)
class LaneEvaluation:
    offset_m: float
    safe: bool
    blocked: bool
    in_bounds: bool
    min_clearance_m: float
    boundary_margin_m: float
    reason: str


@dataclass(frozen=True)
class LaneSelection:
    offset_m: float
    changed: bool
    reason: str
    evaluations: tuple[LaneEvaluation, ...]


@dataclass(frozen=True)
class Challenge3CorridorConfig:
    field_width_m: float = 13.716
    field_height_m: float = 13.716
    field_margin_m: float = 0.45
    destination_radius_m: float = CHALLENGE3_DESTINATION_RADIUS_M
    arrival_buffer_m: float = CHALLENGE3_ARRIVAL_BUFFER_M
    lane_offsets_m: tuple[float, ...] = CHALLENGE3_DEFAULT_LANE_OFFSETS_M
    obstacle_lookahead_m: float = 5.5
    route_corridor_half_width_m: float = 0.70
    emergency_stop_m: float = 0.65
    obstacle_memory_ttl_s: float = 8.0
    obstacle_merge_distance_m: float = 0.55
    obstacle_passed_behind_m: float = 1.0
    lidar_min_cluster_points: int = 3
    lidar_cluster_max_gap_m: float = 0.35
    lane_change_distance_m: float = 5.0
    rejoin_distance_m: float = 6.0
    lookahead_m: float = 2.4
    cruise_speed_mps: float = 0.24
    hard_turn_speed_mps: float = CHALLENGE3_TURN_ENVELOPE_SPEED_MPS
    hard_turn_error_rad: float = 0.35
    cruise_heading_kp: float = 0.85
    hard_turn_heading_kp: float = (
        CHALLENGE3_TURN_ENVELOPE_OMEGA_RADPS / math.radians(20.0)
    )
    cruise_max_omega_radps: float = 0.85
    hard_turn_max_omega_radps: float = CHALLENGE3_TURN_ENVELOPE_OMEGA_RADPS
    min_turn_radius_m: float = CHALLENGE3_TURN_ENVELOPE_MIN_RADIUS_M
    track_width_m: float = 0.416
    wheel_radius_m: float = 0.0825
    ticks_per_rev: float = 3200.0


@dataclass
class Challenge3CorridorState:
    mode: str = "WAIT_TARGET"
    desired_lane_offset_m: float = 0.0
    lane_change_start_u_m: float = 0.0
    lane_change_start_offset_m: float = 0.0
    lane_change_distance_m: float = 5.0
    obstacles: list[ObstacleMemoryPoint] = field(default_factory=list)
    last_lane_selection: Optional[LaneSelection] = None
    last_route_point: Optional[RoutePoint] = None
    last_command: Optional[VelocityCommand] = None
    last_heading_error_rad: float = 0.0
    last_lane_offset_m: float = 0.0
    completion_reason: Optional[str] = None
    fault_reason: Optional[str] = None


@dataclass(frozen=True)
class Challenge3Step:
    command: VelocityCommand
    mode: str
    reason: str
    route_point: Optional[RoutePoint]
    lane_offset_m: float
    heading_error_rad: float
    target_distance_m: Optional[float]
    left_mps: float = 0.0
    right_mps: float = 0.0


def make_route_frame(
    *,
    start_x_m: float,
    start_y_m: float,
    target_x_m: float,
    target_y_m: float,
) -> RouteFrame:
    values = (start_x_m, start_y_m, target_x_m, target_y_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("route coordinates must be finite")
    dx = float(target_x_m) - float(start_x_m)
    dy = float(target_y_m) - float(start_y_m)
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        raise ValueError("route target must be separated from start")
    ux = dx / length
    uy = dy / length
    return RouteFrame(
        start_x_m=float(start_x_m),
        start_y_m=float(start_y_m),
        target_x_m=float(target_x_m),
        target_y_m=float(target_y_m),
        ux=ux,
        uy=uy,
        nx=-uy,
        ny=ux,
        length_m=length,
    )


def project_field_to_route(route: RouteFrame, *, x_m: float, y_m: float) -> RoutePoint:
    dx = float(x_m) - route.start_x_m
    dy = float(y_m) - route.start_y_m
    return RoutePoint(
        u_m=dx * route.ux + dy * route.uy,
        v_m=dx * route.nx + dy * route.ny,
    )


def route_to_field(route: RouteFrame, *, u_m: float, v_m: float) -> tuple[float, float]:
    return (
        route.start_x_m + route.ux * float(u_m) + route.nx * float(v_m),
        route.start_y_m + route.uy * float(u_m) + route.ny * float(v_m),
    )


def robot_point_to_field(
    pose: FieldPose,
    *,
    forward_m: float,
    left_m: float,
) -> tuple[float, float]:
    c = math.cos(pose.yaw_rad)
    s = math.sin(pose.yaw_rad)
    return (
        pose.x_m + float(forward_m) * c - float(left_m) * s,
        pose.y_m + float(forward_m) * s + float(left_m) * c,
    )


def robot_point_to_route(
    pose: FieldPose,
    route: RouteFrame,
    *,
    forward_m: float,
    left_m: float,
    range_m: Optional[float] = None,
) -> RouteObservation:
    field_x, field_y = robot_point_to_field(pose, forward_m=forward_m, left_m=left_m)
    route_point = project_field_to_route(route, x_m=field_x, y_m=field_y)
    if range_m is None:
        range_m = math.hypot(float(forward_m), float(left_m))
    return RouteObservation(
        u_m=route_point.u_m,
        v_m=route_point.v_m,
        field_x_m=field_x,
        field_y_m=field_y,
        robot_forward_m=float(forward_m),
        robot_left_m=float(left_m),
        range_m=float(range_m),
    )


def scan_ranges_to_route_observations(
    *,
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
    pose: FieldPose,
    route: RouteFrame,
    max_range_m: float,
    min_cluster_points: int = 3,
    cluster_max_gap_m: float = 0.35,
) -> list[RouteObservation]:
    observations: list[RouteObservation] = []
    min_range = max(0.0, float(range_min_m))
    max_range = min(float(range_max_m), max(0.0, float(max_range_m)))
    if not math.isfinite(max_range) or max_range <= 0.0:
        return observations
    min_points = max(1, int(min_cluster_points))
    max_gap = max(0.0, float(cluster_max_gap_m))
    cluster: list[tuple[float, float, float]] = []

    def flush_cluster() -> None:
        nonlocal cluster
        if len(cluster) >= min_points:
            for forward, left, range_m in cluster:
                observations.append(
                    robot_point_to_route(
                        pose,
                        route,
                        forward_m=forward,
                        left_m=left,
                        range_m=range_m,
                    )
                )
        cluster = []

    for idx, raw_range in enumerate(ranges):
        try:
            r = float(raw_range)
        except (TypeError, ValueError):
            flush_cluster()
            continue
        if not math.isfinite(r) or r < min_range or r > max_range:
            flush_cluster()
            continue
        angle = float(angle_min_rad) + float(idx) * float(angle_increment_rad)
        forward = r * math.cos(angle)
        left = r * math.sin(angle)
        if forward <= 0.03:
            flush_cluster()
            continue
        if cluster and max_gap > 0.0:
            prev_forward, prev_left, _ = cluster[-1]
            if math.hypot(forward - prev_forward, left - prev_left) > max_gap:
                flush_cluster()
        cluster.append((forward, left, r))
    flush_cluster()
    return observations


def update_obstacle_memory(
    state: Challenge3CorridorState,
    *,
    observations: Iterable[RouteObservation],
    current_u_m: float,
    now_s: float,
    config: Challenge3CorridorConfig,
) -> None:
    merge_distance = max(0.05, float(config.obstacle_merge_distance_m))
    for obs in observations:
        if not all(math.isfinite(v) for v in (obs.u_m, obs.v_m, obs.field_x_m, obs.field_y_m)):
            continue
        merged = False
        for memory in state.obstacles:
            if math.hypot(memory.u_m - obs.u_m, memory.v_m - obs.v_m) <= merge_distance:
                memory.u_m = 0.65 * memory.u_m + 0.35 * obs.u_m
                memory.v_m = 0.65 * memory.v_m + 0.35 * obs.v_m
                memory.field_x_m = 0.65 * memory.field_x_m + 0.35 * obs.field_x_m
                memory.field_y_m = 0.65 * memory.field_y_m + 0.35 * obs.field_y_m
                memory.last_seen_s = float(now_s)
                merged = True
                break
        if not merged:
            state.obstacles.append(
                ObstacleMemoryPoint(
                    u_m=float(obs.u_m),
                    v_m=float(obs.v_m),
                    field_x_m=float(obs.field_x_m),
                    field_y_m=float(obs.field_y_m),
                    first_seen_s=float(now_s),
                    last_seen_s=float(now_s),
                )
            )

    passed_u = float(current_u_m) - max(0.0, float(config.obstacle_passed_behind_m))
    ttl = max(0.1, float(config.obstacle_memory_ttl_s))
    retained: list[ObstacleMemoryPoint] = []
    for memory in state.obstacles:
        age = float(now_s) - float(memory.last_seen_s)
        if memory.u_m < passed_u:
            continue
        if age > ttl and memory.u_m < float(current_u_m) + 0.5:
            continue
        retained.append(memory)
    state.obstacles = retained


def _point_field_margin(config: Challenge3CorridorConfig, x_m: float, y_m: float) -> float:
    return min(
        float(x_m) - float(config.field_margin_m),
        float(y_m) - float(config.field_margin_m),
        float(config.field_width_m) - float(config.field_margin_m) - float(x_m),
        float(config.field_height_m) - float(config.field_margin_m) - float(y_m),
    )


def _lane_boundary_margin(
    route: RouteFrame,
    *,
    current_u_m: float,
    offset_m: float,
    config: Challenge3CorridorConfig,
) -> float:
    samples = (
        max(0.0, float(current_u_m)),
        min(route.length_m, float(current_u_m) + 0.5 * float(config.obstacle_lookahead_m)),
        min(route.length_m, float(current_u_m) + float(config.obstacle_lookahead_m)),
    )
    margins = []
    for sample_u in samples:
        x_m, y_m = route_to_field(route, u_m=sample_u, v_m=offset_m)
        margins.append(_point_field_margin(config, x_m, y_m))
    return min(margins) if margins else -math.inf


def evaluate_lane(
    route: RouteFrame,
    *,
    current_u_m: float,
    offset_m: float,
    obstacles: Sequence[ObstacleMemoryPoint],
    config: Challenge3CorridorConfig,
) -> LaneEvaluation:
    boundary_margin = _lane_boundary_margin(route, current_u_m=current_u_m, offset_m=offset_m, config=config)
    in_bounds = boundary_margin >= 0.0
    lookahead = max(0.0, float(config.obstacle_lookahead_m))
    corridor_half = max(0.0, float(config.route_corridor_half_width_m))
    min_clearance = math.inf
    blocked = False
    for obstacle in obstacles:
        du = obstacle.u_m - float(current_u_m)
        if du < -0.25 or du > lookahead:
            continue
        clearance = abs(obstacle.v_m - float(offset_m))
        min_clearance = min(min_clearance, clearance)
        if clearance <= corridor_half:
            blocked = True
    if math.isinf(min_clearance):
        min_clearance = 99.0
    safe = bool(in_bounds and not blocked)
    if not in_bounds:
        reason = "lane_out_of_bounds"
    elif blocked:
        reason = "lane_blocked"
    else:
        reason = "lane_clear"
    return LaneEvaluation(
        offset_m=float(offset_m),
        safe=safe,
        blocked=blocked,
        in_bounds=in_bounds,
        min_clearance_m=float(min_clearance),
        boundary_margin_m=float(boundary_margin),
        reason=reason,
    )


def _candidate_lane_offsets(config: Challenge3CorridorConfig, current_offset_m: float) -> tuple[float, ...]:
    offsets = [float(current_offset_m)]
    for offset in config.lane_offsets_m:
        if not any(abs(float(offset) - existing) < 1e-6 for existing in offsets):
            offsets.append(float(offset))
    return tuple(offsets)


def select_lane_offset(
    route: RouteFrame,
    *,
    current_u_m: float,
    current_offset_m: float,
    obstacles: Sequence[ObstacleMemoryPoint],
    config: Challenge3CorridorConfig,
) -> LaneSelection:
    evaluations = tuple(
        evaluate_lane(
            route,
            current_u_m=current_u_m,
            offset_m=offset,
            obstacles=obstacles,
            config=config,
        )
        for offset in _candidate_lane_offsets(config, current_offset_m)
    )
    current_eval = evaluations[0]
    if current_eval.safe:
        return LaneSelection(
            offset_m=float(current_offset_m),
            changed=False,
            reason="current_lane_clear",
            evaluations=evaluations,
        )
    safe_candidates = [ev for ev in evaluations[1:] if ev.safe]
    if not safe_candidates:
        return LaneSelection(
            offset_m=float(current_offset_m),
            changed=False,
            reason="no_safe_lane",
            evaluations=evaluations,
        )
    safe_candidates.sort(
        key=lambda ev: (
            abs(ev.offset_m),
            -min(ev.min_clearance_m, 4.0),
            -min(ev.boundary_margin_m, 3.0),
        )
    )
    chosen = safe_candidates[0]
    return LaneSelection(
        offset_m=chosen.offset_m,
        changed=abs(chosen.offset_m - float(current_offset_m)) > 1e-6,
        reason="lane_change_selected",
        evaluations=evaluations,
    )


def _smoothstep(value: float) -> float:
    t = clamp(float(value), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def lane_offset_at_u(state: Challenge3CorridorState, u_m: float) -> float:
    distance = max(0.05, float(state.lane_change_distance_m))
    progress = (float(u_m) - float(state.lane_change_start_u_m)) / distance
    return float(state.lane_change_start_offset_m) + (
        float(state.desired_lane_offset_m) - float(state.lane_change_start_offset_m)
    ) * _smoothstep(progress)


def _set_desired_lane(
    state: Challenge3CorridorState,
    *,
    current_u_m: float,
    current_offset_m: float,
    desired_offset_m: float,
    distance_m: float,
) -> None:
    state.lane_change_start_u_m = float(current_u_m)
    state.lane_change_start_offset_m = float(current_offset_m)
    state.desired_lane_offset_m = float(desired_offset_m)
    state.lane_change_distance_m = max(0.5, float(distance_m))


def centerline_obstacles_passed(
    *,
    current_u_m: float,
    obstacles: Sequence[ObstacleMemoryPoint],
    config: Challenge3CorridorConfig,
) -> bool:
    behind = max(0.0, float(config.obstacle_passed_behind_m))
    corridor_half = max(0.0, float(config.route_corridor_half_width_m))
    for obstacle in obstacles:
        if obstacle.u_m >= float(current_u_m) - behind and abs(obstacle.v_m) <= corridor_half:
            return False
    return True


def _emergency_obstacle_ahead(
    *,
    observations: Sequence[RouteObservation],
    pose_route: RoutePoint,
    config: Challenge3CorridorConfig,
) -> bool:
    stop_m = max(0.0, float(config.emergency_stop_m))
    half = max(0.0, float(config.route_corridor_half_width_m))
    for obs in observations:
        if (
            obs.range_m <= stop_m
            and obs.robot_forward_m >= 0.0
            and abs(obs.robot_left_m) <= half
        ):
            return True
        du = obs.u_m - pose_route.u_m
        if 0.0 <= du <= stop_m and abs(obs.v_m - pose_route.v_m) <= half:
            return True
    return False


def forward_arc_command(
    *,
    heading_error_rad: float,
    config: Challenge3CorridorConfig,
) -> tuple[VelocityCommand, float, float]:
    abs_error = abs(float(heading_error_rad))
    hard_turn = abs_error >= max(0.0, float(config.hard_turn_error_rad))
    if hard_turn:
        v_mps = max(0.0, float(config.hard_turn_speed_mps))
        raw_omega = float(config.hard_turn_heading_kp) * float(heading_error_rad)
        max_omega = max(0.0, float(config.hard_turn_max_omega_radps))
        reason = "challenge3_hard_forward_arc"
    else:
        v_mps = max(0.0, float(config.cruise_speed_mps))
        raw_omega = float(config.cruise_heading_kp) * float(heading_error_rad)
        max_omega = max(0.0, float(config.cruise_max_omega_radps))
        reason = "challenge3_corridor_track"

    if v_mps <= 1e-9:
        command = build_stop_command("challenge3_speed_zero", controller="challenge3_corridor")
        return command, 0.0, 0.0

    radius_limit = v_mps / max(1e-6, float(config.min_turn_radius_m))
    side_limit = (2.0 * v_mps) / max(1e-6, float(config.track_width_m))
    omega_limit = min(max_omega, radius_limit, side_limit)
    omega = clamp(raw_omega, -omega_limit, omega_limit)
    half_track = 0.5 * max(1e-6, float(config.track_width_m))
    left = v_mps - omega * half_track
    right = v_mps + omega * half_track
    if left < -1e-6 or right < -1e-6:
        omega = clamp(omega, -side_limit, side_limit)
        left = max(0.0, v_mps - omega * half_track)
        right = max(0.0, v_mps + omega * half_track)
    command = build_velocity_command(v_mps, omega, reason, controller="challenge3_corridor")
    return command, left, right


def step_corridor_controller(
    state: Challenge3CorridorState,
    *,
    route: RouteFrame,
    pose: FieldPose,
    now_s: float,
    observations: Sequence[RouteObservation],
    config: Challenge3CorridorConfig,
) -> Challenge3Step:
    target_distance = math.hypot(route.target_x_m - pose.x_m, route.target_y_m - pose.y_m)
    pose_route = project_field_to_route(route, x_m=pose.x_m, y_m=pose.y_m)
    state.last_route_point = pose_route

    stop_radius = max(0.0, float(config.destination_radius_m) - max(0.0, float(config.arrival_buffer_m)))
    if target_distance <= stop_radius:
        state.mode = "DESTINATION_STOP"
        state.completion_reason = "challenge3_destination_reached"
        command = build_stop_command(state.completion_reason, controller="challenge3_corridor")
        state.last_command = command
        return Challenge3Step(command, state.mode, command.reason, pose_route, lane_offset_at_u(state, pose_route.u_m), 0.0, target_distance)

    update_obstacle_memory(
        state,
        observations=observations,
        current_u_m=pose_route.u_m,
        now_s=now_s,
        config=config,
    )

    if _emergency_obstacle_ahead(observations=observations, pose_route=pose_route, config=config):
        state.mode = "FAULT"
        state.fault_reason = "challenge3_obstacle_emergency_stop"
        command = build_stop_command(state.fault_reason, controller="challenge3_corridor")
        state.last_command = command
        return Challenge3Step(command, state.mode, command.reason, pose_route, lane_offset_at_u(state, pose_route.u_m), 0.0, target_distance)

    current_lane = lane_offset_at_u(state, pose_route.u_m)
    if abs(state.desired_lane_offset_m) > 1e-6 and centerline_obstacles_passed(
        current_u_m=pose_route.u_m,
        obstacles=state.obstacles,
        config=config,
    ):
        _set_desired_lane(
            state,
            current_u_m=pose_route.u_m,
            current_offset_m=current_lane,
            desired_offset_m=0.0,
            distance_m=max(float(config.rejoin_distance_m), float(config.lane_change_distance_m)),
        )
        current_lane = lane_offset_at_u(state, pose_route.u_m)
        state.mode = "REJOIN_BASELINE"
    else:
        selection = select_lane_offset(
            route,
            current_u_m=pose_route.u_m,
            current_offset_m=state.desired_lane_offset_m,
            obstacles=state.obstacles,
            config=config,
        )
        state.last_lane_selection = selection
        if selection.reason == "no_safe_lane":
            state.mode = "FAULT"
            state.fault_reason = "challenge3_no_safe_lane"
            command = build_stop_command(state.fault_reason, controller="challenge3_corridor")
            state.last_command = command
            return Challenge3Step(command, state.mode, command.reason, pose_route, current_lane, 0.0, target_distance)
        if selection.changed:
            _set_desired_lane(
                state,
                current_u_m=pose_route.u_m,
                current_offset_m=current_lane,
                desired_offset_m=selection.offset_m,
                distance_m=float(config.lane_change_distance_m),
            )
            current_lane = lane_offset_at_u(state, pose_route.u_m)
            state.mode = "ENTER_LANE" if abs(selection.offset_m) > 1e-6 else "REJOIN_BASELINE"
        elif abs(state.desired_lane_offset_m) > 1e-6:
            state.mode = "HOLD_LANE"
        else:
            state.mode = "TRACK_BASELINE"

    lookahead_u = min(route.length_m, pose_route.u_m + max(0.2, float(config.lookahead_m)))
    lookahead_lane = lane_offset_at_u(state, lookahead_u)
    goal_x, goal_y = route_to_field(route, u_m=lookahead_u, v_m=lookahead_lane)
    heading_to_goal = math.atan2(goal_y - pose.y_m, goal_x - pose.x_m)
    heading_error = wrap_pi(heading_to_goal - pose.yaw_rad)
    command, left_mps, right_mps = forward_arc_command(heading_error_rad=heading_error, config=config)
    state.last_heading_error_rad = heading_error
    state.last_lane_offset_m = current_lane
    state.last_command = command
    return Challenge3Step(
        command=command,
        mode=state.mode,
        reason=command.reason,
        route_point=pose_route,
        lane_offset_m=current_lane,
        heading_error_rad=heading_error,
        target_distance_m=target_distance,
        left_mps=left_mps,
        right_mps=right_mps,
    )


def integrate_encoder_imu_pose(
    pose: FieldPose,
    *,
    left_delta_ticks: int,
    right_delta_ticks: int,
    yaw_rad: float,
    wheel_radius_m: float,
    ticks_per_rev: float,
    previous_yaw_rad: Optional[float] = None,
) -> FieldPose:
    left_m = encoder_ticks_to_distance_m(
        int(left_delta_ticks),
        wheel_radius_m=float(wheel_radius_m),
        ticks_per_rev=float(ticks_per_rev),
    )
    right_m = encoder_ticks_to_distance_m(
        int(right_delta_ticks),
        wheel_radius_m=float(wheel_radius_m),
        ticks_per_rev=float(ticks_per_rev),
    )
    distance_delta = 0.5 * (left_m + right_m)
    if previous_yaw_rad is None:
        mid_yaw = float(yaw_rad)
    else:
        yaw_delta = wrap_pi(float(yaw_rad) - float(previous_yaw_rad))
        mid_yaw = wrap_pi(float(previous_yaw_rad) + 0.5 * yaw_delta)
    return FieldPose(
        x_m=pose.x_m + distance_delta * math.cos(mid_yaw),
        y_m=pose.y_m + distance_delta * math.sin(mid_yaw),
        yaw_rad=wrap_pi(float(yaw_rad)),
    )
