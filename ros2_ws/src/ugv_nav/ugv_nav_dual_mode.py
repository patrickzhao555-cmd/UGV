from __future__ import annotations

import argparse
import json
import math
import heapq
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np

import matplotlib
try:
    if os.name == "nt" or os.environ.get("DISPLAY"):
        matplotlib.use("TkAgg")
    else:
        matplotlib.use("Agg")
except Exception:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon, Rectangle

# =========================================================
# Unit helpers
# =========================================================

FT_TO_M = 0.3048
M_TO_FT = 1.0 / FT_TO_M
CM_TO_M = 0.01


def ft(x: float) -> float:
    return x * FT_TO_M


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


@dataclass
class ZedPacket:
    hit_points_local: List[Tuple[float, float]]
    timestamp: float


@dataclass
class GoalPacket:
    x: float
    y: float
    timestamp: float


@dataclass
class SensorFrame:
    encoder: EncoderPacket
    lidar: LidarPacket
    zed: ZedPacket
    goal: GoalPacket


@dataclass
class ControlCommand:
    mode: str
    turn_deg: float = 0.0
    move_m: float = 0.0
    raw_left: float = 0.0
    raw_right: float = 0.0
    reason: str = ""

    def short_text(self) -> str:
        if self.mode in {"TURN_LEFT", "TURN_RIGHT"}:
            return f"{self.mode} {self.turn_deg:.1f} deg"
        if self.mode in {"FORWARD", "BACKWARD"}:
            return f"{self.mode} {self.move_m:.2f} m"
        return self.mode

    def as_dict(self) -> dict:
        return {
            "mode": self.mode,
            "turn_deg": round(self.turn_deg, 3),
            "move_m": round(self.move_m, 4),
            "raw_left": round(self.raw_left, 3),
            "raw_right": round(self.raw_right, 3),
            "reason": self.reason,
        }


@dataclass
class RobotConfig:
    length_m: float = ft(25.0 / 12.0)
    width_m: float = ft(15.0 / 12.0)
    track_width_m: float = ft(1.35)
    wheel_radius_m: float = 0.06
    ticks_per_rev: int = 1000
    obstacle_buffer_m: float = 0.05


@dataclass
class SensorConfig:
    lidar_range_m: float = 3.8
    lidar_num_beams: int = 181
    zed_fov_deg: float = 110.0
    zed_range_m: float = 3.2
    zed_los_step_m: float = 0.04


@dataclass
class NavConfig:
    map_resolution_m: float = 0.08
    planner_step_m: float = 0.22
    goal_tol_m: float = 0.18
    path_reach_tol_m: float = 0.12
    local_lookahead_m: float = 0.55
    turn_threshold_deg: float = 16.0
    max_turn_cmd_deg: float = 28.0
    turn_step_choices_deg: Tuple[float, ...] = (10.0, 18.0, 28.0, 38.0)
    forward_step_choices_m: Tuple[float, ...] = (0.08, 0.12, 0.16)
    backward_step_choices_m: Tuple[float, ...] = (0.08, 0.12)
    replan_cooldown_s: float = 0.22
    replan_lookahead_poses: int = 28
    stuck_pose_epsilon_m: float = 0.018
    stuck_trigger_steps: int = 3
    blocked_patch_radius_m: float = 0.22
    blocked_patch_distance_m: float = 0.30
    blocked_patch_ttl_steps: int = 28
    front_safety_margin_m: float = 0.10
    rear_safety_margin_m: float = 0.08
    global_plan_inflation_m: float = 0.18
    local_plan_inflation_m: float = 0.08
    local_turn_switch_penalty: float = 0.18
    local_reverse_penalty: float = 0.28
    local_turn_penalty: float = 0.10
    local_goal_progress_weight: float = 4.5
    local_heading_weight: float = 0.95


@dataclass
class SimConfig:
    field_w_m: float = ft(15.0)
    field_h_m: float = ft(15.0)
    dt_s: float = 0.10
    max_steps: int = 900
    show_gui: bool = True


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
        front_need = self.robot_cfg.length_m * 0.50 + self.nav_cfg.front_safety_margin_m + abs(cmd.move_m)
        rear_need = self.robot_cfg.length_m * 0.42 + self.nav_cfg.rear_safety_margin_m + abs(cmd.move_m)
        if cmd.mode == 'FORWARD' and sectors.front_m < front_need:
            return False
        if cmd.mode == 'BACKWARD' and sectors.rear_m < rear_need:
            return False
        return True

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
        if prev_cmd.mode and prev_cmd.mode != 'STOP':
            if prev_cmd.mode != cmd.mode:
                score -= 0.05
            if prev_cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'} and cmd.mode in {'TURN_LEFT', 'TURN_RIGHT'} and prev_cmd.mode != cmd.mode:
                score -= self.nav_cfg.local_turn_switch_penalty
        return score

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

        candidates: List[ControlCommand] = []
        for deg in self.nav_cfg.turn_step_choices_deg:
            candidates.append(ControlCommand('TURN_LEFT', turn_deg=deg, raw_left=-0.30, raw_right=0.30, reason='local turn left'))
            candidates.append(ControlCommand('TURN_RIGHT', turn_deg=deg, raw_left=0.30, raw_right=-0.30, reason='local turn right'))
        for mv in self.nav_cfg.forward_step_choices_m:
            candidates.append(ControlCommand('FORWARD', move_m=mv, raw_left=0.42, raw_right=0.42, reason='local forward'))

        best_score = -1e18
        best_cmd: Optional[ControlCommand] = None
        for cmd in candidates:
            if cmd.mode == 'FORWARD' and abs(yaw_err) > math.radians(32.0):
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
            return best_cmd

        if sectors.left_m + sectors.front_left_m >= sectors.right_m + sectors.front_right_m:
            cmd = ControlCommand('TURN_LEFT', turn_deg=28.0, raw_left=-0.32, raw_right=0.32, reason='local escape turn')
            if self._safe_on_costmap(costmap, pose, cmd):
                return cmd
        else:
            cmd = ControlCommand('TURN_RIGHT', turn_deg=28.0, raw_left=0.32, raw_right=-0.32, reason='local escape turn')
            if self._safe_on_costmap(costmap, pose, cmd):
                return cmd
        if sectors.rear_m > self.robot_cfg.length_m * 0.45 + 0.08:
            return ControlCommand('BACKWARD', move_m=min(self.nav_cfg.backward_step_choices_m), raw_left=-0.34, raw_right=-0.34, reason='local emergency reverse')
        return ControlCommand('STOP', reason='local no safe motion')


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
            Obstacle("box",   ft(3.5),  ft(3.4),  ft(1.9),  ft(1.3)),
            Obstacle("box",   ft(5.0),  ft(11.1), ft(2.0),  ft(1.3)),
            Obstacle("box",   ft(11.8), ft(10.8), ft(2.0),  ft(1.3)),
            Obstacle("chair", ft(7.4),  ft(7.0),  ft(1.45), ft(1.45)),
            Obstacle("chair", ft(10.6), ft(3.7),  ft(1.4),  ft(1.4)),
            Obstacle("cone",  ft(2.6),  ft(8.2),  ft(0.85), ft(0.85)),
            Obstacle("cone",  ft(4.4),  ft(9.1),  ft(0.85), ft(0.85)),
            Obstacle("cone",  ft(8.7),  ft(9.8),  ft(0.85), ft(0.85)),
            Obstacle("cone",  ft(11.0), ft(8.8),  ft(0.85), ft(0.85)),
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

    def apply_command(self, cmd: ControlCommand, world: VirtualWorld, planner: HybridAStarPlanner) -> None:
        pose = self.state.true_pose
        new_pose = pose
        left_move = 0.0
        right_move = 0.0

        if cmd.mode == "TURN_LEFT":
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
        )


# =========================================================
# Real robot bridge
# =========================================================

class RealRobotBridgeBase:
    def read_frame(self) -> Optional[SensorFrame]:
        raise NotImplementedError

    def send_control(self, cmd: ControlCommand) -> None:
        raise NotImplementedError


class JsonReplayBridge(RealRobotBridgeBase):
    """
    Handy fallback when ROS2 is not available.
    Expects one JSON object per line with keys:
    left_ticks, right_ticks, lidar_hits_local, zed_hits_local, goal_x, goal_y, timestamp
    """
    def __init__(self, path: str):
        self.fp = open(path, "r", encoding="utf-8")

    def read_frame(self) -> Optional[SensorFrame]:
        line = self.fp.readline()
        if not line:
            return None
        obj = json.loads(line)
        ts = float(obj.get("timestamp", time.time()))
        lidar_hits = [tuple(map(float, p)) for p in obj.get("lidar_hits_local", [])]
        zed_hits = [tuple(map(float, p)) for p in obj.get("zed_hits_local", [])]
        return SensorFrame(
            encoder=EncoderPacket(int(obj["left_ticks"]), int(obj["right_ticks"]), ts),
            lidar=LidarPacket(lidar_hits, [0.0] * len(lidar_hits), [0.0] * len(lidar_hits), ts),
            zed=ZedPacket(zed_hits, ts),
            goal=GoalPacket(float(obj["goal_x"]), float(obj["goal_y"]), ts),
        )

    def send_control(self, cmd: ControlCommand) -> None:
        print("REAL CMD", json.dumps(cmd.as_dict()))


class Ros2Bridge(RealRobotBridgeBase):
    def __init__(self):
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Int32MultiArray, String
            from sensor_msgs.msg import LaserScan
            from geometry_msgs.msg import PoseArray, PointStamped
        except Exception as e:
            raise RuntimeError(
                "ROS2 bridge needs rclpy, std_msgs, sensor_msgs, geometry_msgs installed. "
                "If you are not on the Jetson yet, use --replay-json instead."
            ) from e

        self._rclpy = rclpy
        self._String = String
        self._latest_ticks: Optional[Tuple[int, int, float]] = None
        self._latest_scan: Optional[Tuple[List[Tuple[float, float]], float]] = None
        self._latest_zed: Optional[Tuple[List[Tuple[float, float]], float]] = None
        self._latest_goal: Optional[Tuple[float, float, float]] = None

        class BridgeNode(Node):
            pass

        rclpy.init(args=None)
        self.node = BridgeNode("ugv_nav_bridge")
        self.node.create_subscription(Int32MultiArray, "/encoder_ticks", self._ticks_cb, 10)
        self.node.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.node.create_subscription(PoseArray, "/zed_obstacle_points", self._zed_cb, 10)
        self.node.create_subscription(PointStamped, "/ugv_goal", self._goal_cb, 10)
        self.pub = self.node.create_publisher(String, "/ugv_nav_cmd", 10)

    def _ticks_cb(self, msg):
        ts = time.time()
        if len(msg.data) >= 2:
            self._latest_ticks = (int(msg.data[0]), int(msg.data[1]), ts)

    def _scan_cb(self, msg):
        hits: List[Tuple[float, float]] = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                hits.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment
        self._latest_scan = (hits, time.time())

    def _zed_cb(self, msg):
        hits = []
        for p in msg.poses:
            hits.append((float(p.position.x), float(p.position.y)))
        self._latest_zed = (hits, time.time())

    def _goal_cb(self, msg):
        self._latest_goal = (float(msg.point.x), float(msg.point.y), time.time())

    def read_frame(self) -> Optional[SensorFrame]:
        self._rclpy.spin_once(self.node, timeout_sec=0.02)
        if self._latest_ticks is None or self._latest_scan is None or self._latest_goal is None:
            return None
        left, right, ts_e = self._latest_ticks
        scan_hits, ts_l = self._latest_scan
        if self._latest_zed is None:
            zed_hits, ts_z = [], max(ts_e, ts_l)
        else:
            zed_hits, ts_z = self._latest_zed
        gx, gy, ts_g = self._latest_goal
        ts = max(ts_e, ts_l, ts_z, ts_g)
        return SensorFrame(
            encoder=EncoderPacket(left, right, ts),
            lidar=LidarPacket(scan_hits, [0.0] * len(scan_hits), [0.0] * len(scan_hits), ts),
            zed=ZedPacket(zed_hits, ts),
            goal=GoalPacket(gx, gy, ts),
        )

    def send_control(self, cmd: ControlCommand) -> None:
        msg = self._String()
        msg.data = json.dumps(cmd.as_dict())
        self.pub.publish(msg)
        print("REAL CMD", msg.data)


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
        self.fast_planner = FastGridPlanner(robot_cfg)
        self.hybrid_fallback = HybridAStarPlanner(
            robot_cfg,
            yaw_bins=48,
            step_size=0.20,
            steer_samples=7,
            max_nodes=50000,
        )
        self.local_planner = LocalPlanner(robot_cfg, nav_cfg, self.hybrid_fallback)
        self.blocked_memory = BlockedPatchMemory()
        self.state = NavigatorState(init_pose, init_goal)
        self._last_left: Optional[int] = None
        self._last_right: Optional[int] = None
        self._last_replan_time = -1e9
        self._mark_radius = max(robot_cfg.obstacle_buffer_m, 0.05)
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

    def _touch_map(self) -> None:
        self._map_version += 1
        self._plan_costmap_dirty = True

    def _ticks_to_meters(self, ticks: int) -> float:
        revs = ticks / float(self.robot_cfg.ticks_per_rev)
        return revs * 2.0 * math.pi * self.robot_cfg.wheel_radius_m

    def _update_pose_from_encoders(self, packet: EncoderPacket) -> float:
        prev_pose = self.state.estimated_pose
        if self._last_left is None or self._last_right is None:
            self._last_left = packet.left_total
            self._last_right = packet.right_total
            self._last_pose_before_update = prev_pose
            return 0.0

        dleft_ticks = packet.left_total - self._last_left
        dright_ticks = packet.right_total - self._last_right
        self._last_left = packet.left_total
        self._last_right = packet.right_total

        dl = self._ticks_to_meters(dleft_ticks)
        dr = self._ticks_to_meters(dright_ticks)
        ds = 0.5 * (dl + dr)
        dtheta = (dr - dl) / max(1e-6, self.robot_cfg.track_width_m)

        p = self.state.estimated_pose
        mid = p.yaw + 0.5 * dtheta
        new_pose = Pose2D(
            p.x + ds * math.cos(mid),
            p.y + ds * math.sin(mid),
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
        occ = np.argwhere(work.data > 0)
        for gy, gx in occ:
            for ddy in range(-cells, cells + 1):
                ny = gy + ddy
                if ny < 0 or ny >= src.spec.height:
                    continue
                for ddx in range(-cells, cells + 1):
                    nx = gx + ddx
                    if nx < 0 or nx >= src.spec.width:
                        continue
                    if ddx * ddx + ddy * ddy <= cells * cells:
                        inflated[ny, nx] = 1

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

    def _local_safety_costmap(self) -> Costmap2D:
        src = self.known_costmap
        work = Costmap2D(src.spec, src.data.copy())
        self.blocked_memory.rasterize(work)
        if self.nav_cfg.local_plan_inflation_m > 1e-6:
            inflated = work.data.copy()
            cells = max(1, int(math.ceil(self.nav_cfg.local_plan_inflation_m / src.spec.resolution)))
            occ = np.argwhere(work.data > 0)
            for gy, gx in occ:
                for ddy in range(-cells, cells + 1):
                    ny = gy + ddy
                    if ny < 0 or ny >= src.spec.height:
                        continue
                    for ddx in range(-cells, cells + 1):
                        nx = gx + ddx
                        if nx < 0 or nx >= src.spec.width:
                            continue
                        if ddx * ddx + ddy * ddy <= cells * cells:
                            inflated[ny, nx] = 1
            work = Costmap2D(src.spec, inflated)
        clear_r = max(0.50, 0.5 * math.hypot(self.robot_cfg.length_m, self.robot_cfg.width_m) + 0.08)
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

    def _build_escape_queue(self, sectors: SectorSnapshot) -> None:
        self._escape_queue = []
        rear_need = self.robot_cfg.length_m * 0.40 + self.nav_cfg.rear_safety_margin_m
        if sectors.rear_m > rear_need:
            self._escape_queue.append(ControlCommand('BACKWARD', move_m=min(self.nav_cfg.backward_step_choices_m), raw_left=-0.34, raw_right=-0.34, reason='escape reverse'))
        go_left = (sectors.left_m + sectors.front_left_m) >= (sectors.right_m + sectors.front_right_m)
        if math.isinf(sectors.left_m) and math.isinf(sectors.right_m):
            go_left = self._last_escape_side_left
        self._last_escape_side_left = go_left
        if go_left:
            self._escape_queue.append(ControlCommand('TURN_LEFT', turn_deg=28.0, raw_left=-0.32, raw_right=0.32, reason='escape turn'))
        else:
            self._escape_queue.append(ControlCommand('TURN_RIGHT', turn_deg=28.0, raw_left=0.32, raw_right=-0.32, reason='escape turn'))

    def _consume_escape_queue(self, plan_map: Costmap2D) -> Optional[ControlCommand]:
        while self._escape_queue:
            cmd = self._escape_queue.pop(0)
            if self.local_planner._safe_on_costmap(plan_map, self.state.estimated_pose, cmd):
                return cmd
        return None

    def step(self, frame: SensorFrame) -> ControlCommand:
        self.state.finish_reason = 'running'
        self.state.goal_pose = Pose2D(frame.goal.x, frame.goal.y, 0.0)
        moved_m = self._update_pose_from_encoders(frame.encoder)

        self.blocked_memory.step()
        if self.blocked_memory.version != self._plan_costmap_cache_key[1]:
            self._plan_costmap_dirty = True

        last_cmd_mode = self.state.latest_cmd.mode
        if last_cmd_mode in {'FORWARD', 'BACKWARD'} and moved_m < self.nav_cfg.stuck_pose_epsilon_m:
            self._stuck_counter += 1
        elif moved_m >= self.nav_cfg.stuck_pose_epsilon_m:
            self._stuck_counter = 0

        self._integrate_lidar_freespace(frame.lidar)
        added = 0
        added += self._integrate_hits_into_map(frame.lidar.hit_points_local, connect_adjacent=True, solidify_clusters=False)
        added += self._integrate_hits_into_map(frame.zed.hit_points_local, connect_adjacent=False, solidify_clusters=True)
        self.state.discovered_points += added
        self.state.sectors = self.local_planner.build_sector_snapshot(frame.lidar.hit_points_local + frame.zed.hit_points_local)

        if self._stuck_counter >= self.nav_cfg.stuck_trigger_steps:
            self._add_blocked_patch_ahead(reverse=(last_cmd_mode == 'BACKWARD'))
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

        p = self.state.estimated_pose
        g = self.state.goal_pose
        if math.hypot(g.x - p.x, g.y - p.y) <= self.nav_cfg.goal_tol_m:
            cmd = ControlCommand('STOP', reason='goal reached')
            self.state.finish_reason = 'goal reached'
            self.state.latest_cmd = cmd
            return cmd

        plan_map = self._planning_costmap()
        local_map = self._local_safety_costmap()
        escape_cmd = self._consume_escape_queue(local_map)
        if escape_cmd is not None:
            self.state.latest_cmd = escape_cmd
            return escape_cmd

        target = self._choose_target()
        if target is None:
            target = self.state.goal_pose
            self.state.path = []
            self.state.path_idx = 0

        cmd = self.local_planner.choose_command(
            self.state.estimated_pose,
            target,
            self.state.goal_pose,
            local_map,
            self.state.sectors,
            self.state.latest_cmd,
        )

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
            if self._turn_loop_counter >= 6:
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
# Running modes
# =========================================================


def run_simulation(show_gui: bool = True, max_steps: int = 900) -> dict:
    robot_cfg = RobotConfig()
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    sim_cfg = SimConfig(show_gui=show_gui, max_steps=max_steps)

    start = Pose2D(ft(1.2), ft(1.2), 0.0)
    goal = Pose2D(ft(13.4), ft(13.0), 0.0)

    world = VirtualWorld(sim_cfg, nav_cfg)
    robot = SimulatedRobot(robot_cfg, start)
    sensors = SimulatedSensors(robot_cfg, sensor_cfg, world)
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, sim_cfg.field_w_m, sim_cfg.field_h_m, start, goal)

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
        frame = sensors.read(robot.state, (goal.x, goal.y), ts)
        cmd = navigator.step(frame)
        print(f"SIM CMD step={step_i_local:03d}: {json.dumps(cmd.as_dict())}")
        robot.apply_command(cmd, world, control_planner)
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


def run_real_mode(replay_json: Optional[str] = None) -> None:
    robot_cfg = RobotConfig()
    sensor_cfg = SensorConfig()
    nav_cfg = NavConfig()
    field_w_m = ft(15.0)
    field_h_m = ft(15.0)
    init_pose = Pose2D(ft(1.2), ft(1.2), 0.0)
    init_goal = Pose2D(ft(13.4), ft(13.0), 0.0)
    navigator = UGVNavigator(robot_cfg, sensor_cfg, nav_cfg, field_w_m, field_h_m, init_pose, init_goal)

    if replay_json:
        bridge: RealRobotBridgeBase = JsonReplayBridge(replay_json)
    else:
        bridge = Ros2Bridge()

    print("Running REAL mode. No GUI. Printing and sending control commands.")
    print("Expected ROS2 topics if using Ros2Bridge:")
    print("  /encoder_ticks        std_msgs/Int32MultiArray [left_ticks, right_ticks]")
    print("  /scan                 sensor_msgs/LaserScan")
    print("  /zed_obstacle_points  geometry_msgs/PoseArray in base_link frame")
    print("  /ugv_goal             geometry_msgs/PointStamped in map frame")
    print("Publishing:")
    print("  /ugv_nav_cmd          std_msgs/String with JSON control command")

    while True:
        frame = bridge.read_frame()
        if frame is None:
            time.sleep(0.02)
            continue
        cmd = navigator.step(frame)
        bridge.send_control(cmd)


# =========================================================
# CLI
# =========================================================


def main() -> None:
    parser = argparse.ArgumentParser(description="UGV dual-mode navigation bridge, sim mode + real mode")
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument("--headless", action="store_true", help="sim mode without matplotlib GUI")
    parser.add_argument("--max-steps", type=int, default=900)
    parser.add_argument("--replay-json", type=str, default=None, help="use JSONL replay file for real mode testing")
    args = parser.parse_args()

    if args.mode == "sim":
        run_simulation(show_gui=not args.headless, max_steps=args.max_steps)
    else:
        run_real_mode(replay_json=args.replay_json)


if __name__ == "__main__":
    main()
