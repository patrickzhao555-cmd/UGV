"""Competition mission V2 for formal round1/round2/round3 behavior.

This module is intentionally independent of ROS message classes and the large
dual-mode navigator.  The adapter in ``ugv_nav_dual_mode.py`` supplies plain
poses, targets, mission flags, and planner feedback.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


YARD_TO_M = 0.9144
DEFAULT_FIELD_M = 15.0 * YARD_TO_M

CELL_UNVISITED = "unvisited"
CELL_ACTIVE = "active"
CELL_VISITED = "visited"
CELL_BLOCKED = "blocked"
CELL_SKIPPED = "skipped"

STOP_FLAG_STATES = {
    "abort",
    "aborted",
    "complete",
    "done",
    "emergency_stop",
    "halt",
    "landed",
    "landing_complete",
    "mission_complete",
    "round_complete",
    "round_done",
    "stop",
    "stopped",
    "success",
}

LANDING_BUT_KEEP_MOVING_STATES = {
    "descent",
    "descending",
    "land",
    "landing",
    "uav_land",
    "uav_landing",
}


def _finite_float(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _finite_xy(value: Optional[Sequence[float]]) -> Optional[Tuple[float, float]]:
    if value is None or len(value) < 2:
        return None
    x = _finite_float(value[0])
    y = _finite_float(value[1])
    if x is None or y is None:
        return None
    return x, y


def _norm_text(value: Optional[str]) -> str:
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_")


def normalize_competition_round(value: str) -> str:
    text = _norm_text(value)
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
    }
    text = aliases.get(text, text)
    if text not in {"round1", "round2", "round3"}:
        raise ValueError(f"Unsupported competition round {value!r}")
    return text


def normalize_mission_flag_state(value: Optional[str]) -> str:
    text = _norm_text(value)
    aliases = {
        "land": "landing",
        "uav_land": "landing",
        "uav_landing": "landing",
        "descent": "landing",
        "descending": "landing",
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
        "halt": "stop",
        "stopping": "stop",
        "e_stop": "emergency_stop",
        "estop": "emergency_stop",
    }
    return aliases.get(text, text or "unknown")


def mission_flag_requests_stop(value: Optional[str]) -> bool:
    return normalize_mission_flag_state(value) in STOP_FLAG_STATES


@dataclass
class CompetitionMissionConfig:
    field_width_m: float = DEFAULT_FIELD_M
    field_height_m: float = DEFAULT_FIELD_M
    start_x_m: Optional[float] = None
    start_y_m: Optional[float] = None
    start_yaw_deg: Optional[float] = None
    min_competition_speed_mps: float = 0.0894
    straight_distance_m: float = 13.0 * YARD_TO_M
    sweep_cell_size_m: float = 0.75
    sweep_lane_spacing_m: float = 0.75
    sweep_coverage_radius_m: float = 0.55
    sweep_coverage_threshold: float = 0.85
    sweep_fail_limit: int = 3
    sweep_goal_timeout_s: float = 8.0
    target_accept_radius_m: float = YARD_TO_M
    stop_on_uav_landed: bool = True
    stop_on_marker_reached: bool = True
    obstacle_aware: Optional[bool] = None
    use_legacy_center_expand: bool = False
    sweep_boundary_margin_m: float = 0.45

    def normalized_for_round(self, round_name: str) -> "CompetitionMissionConfig":
        round_name = normalize_competition_round(round_name)
        return CompetitionMissionConfig(
            field_width_m=max(0.1, float(self.field_width_m)),
            field_height_m=max(0.1, float(self.field_height_m)),
            start_x_m=_finite_float(self.start_x_m),
            start_y_m=_finite_float(self.start_y_m),
            start_yaw_deg=_finite_float(self.start_yaw_deg),
            min_competition_speed_mps=max(0.0, float(self.min_competition_speed_mps)),
            straight_distance_m=max(0.5, float(self.straight_distance_m)),
            sweep_cell_size_m=max(0.05, float(self.sweep_cell_size_m)),
            sweep_lane_spacing_m=max(0.05, float(self.sweep_lane_spacing_m)),
            sweep_coverage_radius_m=max(0.05, float(self.sweep_coverage_radius_m)),
            sweep_coverage_threshold=max(0.0, min(1.0, float(self.sweep_coverage_threshold))),
            sweep_fail_limit=max(1, int(self.sweep_fail_limit)),
            sweep_goal_timeout_s=max(0.1, float(self.sweep_goal_timeout_s)),
            target_accept_radius_m=max(0.05, float(self.target_accept_radius_m)),
            stop_on_uav_landed=bool(self.stop_on_uav_landed),
            stop_on_marker_reached=bool(self.stop_on_marker_reached),
            obstacle_aware=(round_name == "round3") if self.obstacle_aware is None else bool(self.obstacle_aware),
            use_legacy_center_expand=bool(self.use_legacy_center_expand),
            sweep_boundary_margin_m=max(0.0, float(self.sweep_boundary_margin_m)),
        )


@dataclass
class SweepCell:
    index: int
    row: int
    col: int
    x: float
    y: float
    state: str = CELL_UNVISITED
    visit_count: int = 0
    failed_attempts: int = 0
    last_attempt_time: Optional[float] = None

    @property
    def xy(self) -> Tuple[float, float]:
        return self.x, self.y


@dataclass
class SweepGrid:
    field_width_m: float = DEFAULT_FIELD_M
    field_height_m: float = DEFAULT_FIELD_M
    cell_size_m: float = 0.75
    lane_spacing_m: float = 0.75
    boundary_margin_m: float = 0.45
    cells: List[SweepCell] = field(init=False)
    active_index: Optional[int] = field(default=None, init=False)
    _cursor: int = field(default=-1, init=False)

    def __post_init__(self) -> None:
        self.field_width_m = max(0.1, float(self.field_width_m))
        self.field_height_m = max(0.1, float(self.field_height_m))
        self.cell_size_m = max(0.05, float(self.cell_size_m))
        self.lane_spacing_m = max(0.05, float(self.lane_spacing_m))
        self.boundary_margin_m = max(0.0, min(float(self.boundary_margin_m), 0.49 * min(self.field_width_m, self.field_height_m)))
        self.cells = self._build_cells()

    def _axis_values(self, extent_m: float, step_m: float) -> List[float]:
        lo = self.boundary_margin_m
        hi = max(lo, extent_m - self.boundary_margin_m)
        values: List[float] = []
        cur = lo
        while cur <= hi + 1e-9:
            values.append(round(cur, 6))
            cur += step_m
        if not values:
            values.append(round(0.5 * extent_m, 6))
        return values

    def _build_cells(self) -> List[SweepCell]:
        xs = self._axis_values(self.field_width_m, self.cell_size_m)
        ys = self._axis_values(self.field_height_m, self.lane_spacing_m)
        cells: List[SweepCell] = []
        for row, y in enumerate(ys):
            row_xs = xs if row % 2 == 0 else list(reversed(xs))
            for col_in_order, x in enumerate(row_xs):
                physical_col = col_in_order if row % 2 == 0 else len(xs) - 1 - col_in_order
                cells.append(SweepCell(len(cells), row, physical_col, x, y))
        return cells

    @property
    def total_cells(self) -> int:
        return len(self.cells)

    @property
    def visited_count(self) -> int:
        return sum(1 for cell in self.cells if cell.state == CELL_VISITED or cell.visit_count > 0)

    @property
    def blocked_count(self) -> int:
        return sum(1 for cell in self.cells if cell.state == CELL_BLOCKED)

    @property
    def skipped_count(self) -> int:
        return sum(1 for cell in self.cells if cell.state == CELL_SKIPPED)

    @property
    def coverage_fraction(self) -> float:
        # Coverage is visited / currently reachable cells.  Blocked and skipped
        # cells are excluded so obstacle-heavy round3 fields do not make
        # completion impossible.
        reachable = self.total_cells - self.blocked_count - self.skipped_count
        if reachable <= 0:
            return 1.0 if self.total_cells > 0 else 0.0
        return max(0.0, min(1.0, self.visited_count / float(reachable)))

    def active_cell(self) -> Optional[SweepCell]:
        if self.active_index is None:
            return None
        if 0 <= self.active_index < len(self.cells):
            return self.cells[self.active_index]
        return None

    def _set_active(self, cell: SweepCell, now_s: float) -> SweepCell:
        old = self.active_cell()
        if old is not None and old.index != cell.index and old.state == CELL_ACTIVE:
            old.state = CELL_VISITED if old.visit_count > 0 else CELL_UNVISITED
        if cell.state not in {CELL_BLOCKED, CELL_SKIPPED}:
            cell.state = CELL_ACTIVE
        cell.last_attempt_time = now_s
        self.active_index = cell.index
        self._cursor = cell.index
        return cell

    def ensure_active(self, now_s: float) -> Optional[SweepCell]:
        active = self.active_cell()
        if active is not None and active.state == CELL_ACTIVE:
            return active
        return self.select_next(now_s)

    def select_next(self, now_s: float, allow_revisit: bool = True) -> Optional[SweepCell]:
        if not self.cells:
            self.active_index = None
            return None
        start = (self._cursor + 1) % len(self.cells)
        for offset in range(len(self.cells)):
            cell = self.cells[(start + offset) % len(self.cells)]
            if cell.state == CELL_UNVISITED:
                return self._set_active(cell, now_s)
        if allow_revisit:
            candidates = [
                cell for cell in self.cells
                if cell.state not in {CELL_BLOCKED, CELL_SKIPPED}
            ]
            if candidates:
                min_visits = min(cell.visit_count for cell in candidates)
                for offset in range(len(self.cells)):
                    cell = self.cells[(start + offset) % len(self.cells)]
                    if cell in candidates and cell.visit_count == min_visits:
                        return self._set_active(cell, now_s)
        self.active_index = None
        return None

    def mark_active_visited(self, now_s: float) -> Optional[SweepCell]:
        cell = self.active_cell()
        if cell is None:
            return None
        cell.visit_count += 1
        cell.state = CELL_VISITED
        cell.failed_attempts = 0
        cell.last_attempt_time = now_s
        return cell

    def mark_active_blocked(self, reason: str, now_s: float) -> Optional[SweepCell]:
        del reason
        cell = self.active_cell()
        if cell is None:
            return None
        cell.state = CELL_BLOCKED
        cell.failed_attempts += 1
        cell.last_attempt_time = now_s
        return cell

    def mark_active_skipped(self, reason: str, now_s: float) -> Optional[SweepCell]:
        del reason
        cell = self.active_cell()
        if cell is None:
            return None
        cell.state = CELL_SKIPPED
        cell.failed_attempts += 1
        cell.last_attempt_time = now_s
        return cell

    def advance_if_reached(self, pose_xy: Tuple[float, float], radius_m: float, now_s: float) -> bool:
        changed = False
        guard = 0
        while guard < max(1, len(self.cells)):
            guard += 1
            cell = self.ensure_active(now_s)
            if cell is None:
                return changed
            if math.hypot(cell.x - pose_xy[0], cell.y - pose_xy[1]) > radius_m:
                return changed
            self.mark_active_visited(now_s)
            changed = True
            self.select_next(now_s)
        return changed


@dataclass
class NavigationFeedback:
    now_s: float
    no_safe_trajectory: bool = False
    local_planner_failed: bool = False
    stuck: bool = False
    stuck_steps: int = 0
    finish_reason: str = ""
    progress_m: float = 0.0


@dataclass
class MissionUpdate:
    enabled: bool
    round: str
    phase: str
    active_goal_m: Tuple[float, float]
    stop_requested: bool
    target_known: bool
    target_source: str
    reason: str
    speed_scale: float
    minimum_speed_mps: float
    status: Dict[str, Any]


class CompetitionMissionV2:
    def __init__(self, round_name: str, config: Optional[CompetitionMissionConfig] = None, enabled: bool = True):
        self.round = normalize_competition_round(round_name)
        self.config = (config or CompetitionMissionConfig()).normalized_for_round(self.round)
        self.enabled = bool(enabled)
        self.grid: Optional[SweepGrid] = None
        if self.round in {"round2", "round3"}:
            self.grid = SweepGrid(
                field_width_m=self.config.field_width_m,
                field_height_m=self.config.field_height_m,
                cell_size_m=self.config.sweep_cell_size_m,
                lane_spacing_m=self.config.sweep_lane_spacing_m,
                boundary_margin_m=self.config.sweep_boundary_margin_m,
            )
        self.phase = "manual"
        self.reason = "initialized"
        self.target_goal: Optional[Tuple[float, float]] = None
        self.target_source = "none"
        self.marker_distance_m: Optional[float] = None
        self.uav_state = "unknown"
        self.uav_state_source = "none"
        self._start_pose: Optional[Tuple[float, float, float]] = None
        self._straight_runout_m = self.config.straight_distance_m
        self._straight_goal_m: Optional[Tuple[float, float]] = None
        self._active_cell_start_time_s: Optional[float] = None
        self._active_cell_start_distance_m: Optional[float] = None
        self._active_cell_best_distance_m: Optional[float] = None
        self._last_update = self._make_update((0.0, 0.0), False)

    def _ensure_start_pose(self, pose: Tuple[float, float, float]) -> Tuple[float, float, float]:
        if self._start_pose is not None:
            return self._start_pose
        x = self.config.start_x_m if self.config.start_x_m is not None else pose[0]
        y = self.config.start_y_m if self.config.start_y_m is not None else pose[1]
        yaw = math.radians(self.config.start_yaw_deg) if self.config.start_yaw_deg is not None else pose[2]
        self._start_pose = (float(x), float(y), float(yaw))
        self._straight_goal_m = self._straight_goal_from_runout()
        return self._start_pose

    def _straight_goal_from_runout(self) -> Tuple[float, float]:
        if self._start_pose is None:
            return 0.0, 0.0
        x, y, yaw = self._start_pose
        return (
            x + self._straight_runout_m * math.cos(yaw),
            y + self._straight_runout_m * math.sin(yaw),
        )

    def _ingest_target(
        self,
        marker_goal: Optional[Sequence[float]],
        marker_distance_m: Optional[float],
        field_map_goal: Optional[Sequence[float]],
        field_map_source: Optional[str],
    ) -> None:
        marker_xy = _finite_xy(marker_goal)
        field_xy = _finite_xy(field_map_goal)
        if marker_xy is not None:
            self.target_goal = marker_xy
            self.target_source = "camera_marker"
            self.marker_distance_m = _finite_float(marker_distance_m)
        elif field_xy is not None and self.target_source != "camera_marker":
            self.target_goal = field_xy
            self.target_source = str(field_map_source or "uav_target")
            self.marker_distance_m = None

    def _target_reached(self, pose_xy: Tuple[float, float]) -> bool:
        map_reached = (
            self.target_goal is not None
            and math.hypot(self.target_goal[0] - pose_xy[0], self.target_goal[1] - pose_xy[1])
            <= self.config.target_accept_radius_m
        )
        depth_reached = (
            self.marker_distance_m is not None
            and math.isfinite(self.marker_distance_m)
            and self.marker_distance_m <= self.config.target_accept_radius_m
        )
        return bool(map_reached or depth_reached)

    def _stop_flag_active(self) -> bool:
        state = normalize_mission_flag_state(self.uav_state)
        if state in {"landed", "landing_complete"}:
            return self.config.stop_on_uav_landed
        return state in STOP_FLAG_STATES

    def update(
        self,
        pose: Tuple[float, float, float],
        now_s: float,
        marker_goal: Optional[Sequence[float]] = None,
        marker_distance_m: Optional[float] = None,
        field_map_goal: Optional[Sequence[float]] = None,
        field_map_source: Optional[str] = None,
        mission_flag_state: Optional[str] = None,
        mission_flag_source: Optional[str] = None,
    ) -> MissionUpdate:
        now_s = float(now_s)
        pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        pose_xy = (pose[0], pose[1])
        self._ensure_start_pose(pose)
        self._ingest_target(marker_goal, marker_distance_m, field_map_goal, field_map_source)
        if mission_flag_state is not None:
            self.uav_state = normalize_mission_flag_state(mission_flag_state)
            self.uav_state_source = str(mission_flag_source or "mission_flag")

        if not self.enabled:
            self.phase = "manual"
            self.reason = "competition_v2_disabled"
            return self._store_update(self._make_update(pose_xy, False))

        if self._stop_flag_active():
            self.phase = "complete"
            self.reason = f"mission_flag_{self.uav_state}"
            return self._store_update(self._make_update(pose_xy, True))

        if self.round == "round1":
            return self._store_update(self._update_round1(pose_xy))
        return self._store_update(self._update_search_round(pose_xy, now_s))

    def _update_round1(self, pose_xy: Tuple[float, float]) -> MissionUpdate:
        self.phase = "round1_straight"
        if self._straight_goal_m is None:
            self._straight_goal_m = self._straight_goal_from_runout()
        trigger_m = max(0.75, self.config.sweep_coverage_radius_m, 2.0 * self.config.target_accept_radius_m)
        guard = 0
        while math.hypot(self._straight_goal_m[0] - pose_xy[0], self._straight_goal_m[1] - pose_xy[1]) <= trigger_m and guard < 20:
            self._straight_runout_m += max(1.0, self.config.straight_distance_m)
            self._straight_goal_m = self._straight_goal_from_runout()
            guard += 1
        self.reason = "round1_keep_moving_until_landed_or_complete"
        return self._make_update(self._straight_goal_m, False)

    def _update_search_round(self, pose_xy: Tuple[float, float], now_s: float) -> MissionUpdate:
        if self.target_goal is not None:
            if self.config.stop_on_marker_reached and self._target_reached(pose_xy):
                self.phase = "complete"
                self.reason = "target_reached"
                return self._make_update(pose_xy, True)
            self.phase = "target_nav"
            self.reason = f"target_known_{self.target_source}"
            return self._make_update(self.target_goal, False)

        self.phase = "sweep_search"
        if self.grid is None:
            self.reason = "no_sweep_grid"
            return self._make_update(pose_xy, False)

        self._maybe_timeout_active_cell(pose_xy, now_s)
        reached = self.grid.advance_if_reached(pose_xy, self.config.sweep_coverage_radius_m, now_s)
        active = self.grid.ensure_active(now_s)
        self._refresh_active_cell_progress(active, pose_xy, now_s)
        if active is None:
            self.reason = "sweep_no_reachable_cells"
            return self._make_update(pose_xy, False)
        if reached:
            self.reason = "sweep_cell_visited_advanced"
        elif self.grid.coverage_fraction >= self.config.sweep_coverage_threshold:
            self.reason = "coverage_threshold_reached_continuing_patrol"
        else:
            self.reason = "sweep_searching"
        return self._make_update(active.xy, False)

    def _refresh_active_cell_progress(
        self,
        active: Optional[SweepCell],
        pose_xy: Tuple[float, float],
        now_s: float,
    ) -> None:
        if active is None:
            self._active_cell_start_time_s = None
            self._active_cell_start_distance_m = None
            self._active_cell_best_distance_m = None
            return
        dist = math.hypot(active.x - pose_xy[0], active.y - pose_xy[1])
        if (
            self._active_cell_start_time_s is None
            or self.grid is None
            or self.grid.active_index != active.index
            or active.last_attempt_time == now_s
        ):
            self._active_cell_start_time_s = now_s
            self._active_cell_start_distance_m = dist
            self._active_cell_best_distance_m = dist
            return
        if self._active_cell_best_distance_m is None:
            self._active_cell_best_distance_m = dist
        else:
            self._active_cell_best_distance_m = min(self._active_cell_best_distance_m, dist)

    def _maybe_timeout_active_cell(self, pose_xy: Tuple[float, float], now_s: float) -> None:
        if self.grid is None or not self.config.obstacle_aware:
            return
        active = self.grid.ensure_active(now_s)
        if active is None:
            return
        dist = math.hypot(active.x - pose_xy[0], active.y - pose_xy[1])
        if self._active_cell_start_time_s is None or self._active_cell_start_distance_m is None:
            self._active_cell_start_time_s = now_s
            self._active_cell_start_distance_m = dist
            self._active_cell_best_distance_m = dist
            return
        elapsed_s = now_s - self._active_cell_start_time_s
        best = self._active_cell_best_distance_m if self._active_cell_best_distance_m is not None else dist
        progress_m = max(0.0, self._active_cell_start_distance_m - best)
        little_progress = progress_m < max(0.10, 0.25 * self.config.sweep_coverage_radius_m)
        if elapsed_s >= self.config.sweep_goal_timeout_s and little_progress:
            self.grid.mark_active_skipped("timeout_little_progress", now_s)
            self.reason = "active_cell_skipped_timeout_little_progress"
            self.grid.select_next(now_s)
            self._active_cell_start_time_s = None
            self._active_cell_start_distance_m = None
            self._active_cell_best_distance_m = None

    def observe_navigation_feedback(self, feedback: NavigationFeedback) -> bool:
        if (
            not self.enabled
            or self.round != "round3"
            or not self.config.obstacle_aware
            or self.phase != "sweep_search"
            or self.grid is None
        ):
            return False
        active = self.grid.active_cell()
        if active is None:
            return False
        failed = bool(feedback.no_safe_trajectory or feedback.local_planner_failed or feedback.stuck)
        if not failed:
            return False
        active.failed_attempts += 1
        active.last_attempt_time = feedback.now_s
        reasons = []
        if feedback.no_safe_trajectory:
            reasons.append("no_safe_trajectory")
        if feedback.local_planner_failed:
            reasons.append("local_planner_failed")
        if feedback.stuck:
            reasons.append("stuck")
        self.reason = "active_cell_failure_" + "_".join(reasons or ["unknown"])
        if active.failed_attempts >= self.config.sweep_fail_limit:
            self.grid.mark_active_blocked(self.reason, feedback.now_s)
            self.grid.select_next(feedback.now_s)
            self._active_cell_start_time_s = None
            self._active_cell_start_distance_m = None
            self._active_cell_best_distance_m = None
            self.reason = "active_cell_blocked_after_repeated_failure"
            self._last_update = self._make_update_from_active_or_previous()
            return True
        self._last_update = self._make_update_from_active_or_previous()
        return False

    def _make_update_from_active_or_previous(self) -> MissionUpdate:
        if self.grid is not None and self.grid.active_cell() is not None:
            goal = self.grid.active_cell().xy
        else:
            goal = self._last_update.active_goal_m
        return self._make_update(goal, self._last_update.stop_requested)

    def command_speed_scale(self) -> Tuple[float, str]:
        if self._last_update.stop_requested:
            return 0.0, self._last_update.reason
        return 1.0, self._last_update.reason

    def status_dict(self) -> Dict[str, Any]:
        return dict(self._last_update.status)

    def current_update(self) -> MissionUpdate:
        return self._last_update

    def _store_update(self, update: MissionUpdate) -> MissionUpdate:
        self._last_update = update
        return update

    def _make_update(self, active_goal_m: Tuple[float, float], stop_requested: bool) -> MissionUpdate:
        status = self._status(active_goal_m, stop_requested)
        return MissionUpdate(
            enabled=self.enabled,
            round=self.round,
            phase=self.phase,
            active_goal_m=active_goal_m,
            stop_requested=stop_requested,
            target_known=self.target_goal is not None,
            target_source=self.target_source,
            reason=self.reason,
            speed_scale=0.0 if stop_requested else 1.0,
            minimum_speed_mps=self.config.min_competition_speed_mps,
            status=status,
        )

    def _status(self, active_goal_m: Tuple[float, float], stop_requested: bool) -> Dict[str, Any]:
        active_cell = None
        visited = blocked = skipped = total = 0
        coverage = 0.0
        if self.grid is not None:
            cell = self.grid.active_cell()
            active_cell = None if cell is None else cell.index
            visited = self.grid.visited_count
            blocked = self.grid.blocked_count
            skipped = self.grid.skipped_count
            total = self.grid.total_cells
            coverage = self.grid.coverage_fraction
        return {
            "enabled": bool(self.enabled),
            "round": self.round,
            "phase": self.phase,
            "active_goal_m": [round(active_goal_m[0], 3), round(active_goal_m[1], 3)],
            "target_known": self.target_goal is not None,
            "target_goal_m": [
                round(self.target_goal[0], 3),
                round(self.target_goal[1], 3),
            ] if self.target_goal is not None else None,
            "target_source": self.target_source,
            "active_cell": active_cell,
            "visited_count": int(visited),
            "blocked_count": int(blocked),
            "skipped_count": int(skipped),
            "total_cells": int(total),
            "coverage_fraction": round(float(coverage), 4),
            "coverage_threshold": round(float(self.config.sweep_coverage_threshold), 4),
            "minimum_speed_mps": round(float(self.config.min_competition_speed_mps), 4),
            "reason": self.reason,
            "stop_requested": bool(stop_requested),
            "uav_state": self.uav_state,
            "uav_state_source": self.uav_state_source,
        }


__all__ = [
    "CELL_ACTIVE",
    "CELL_BLOCKED",
    "CELL_SKIPPED",
    "CELL_UNVISITED",
    "CELL_VISITED",
    "CompetitionMissionConfig",
    "CompetitionMissionV2",
    "MissionUpdate",
    "NavigationFeedback",
    "SweepCell",
    "SweepGrid",
    "mission_flag_requests_stop",
    "normalize_competition_round",
    "normalize_mission_flag_state",
]
