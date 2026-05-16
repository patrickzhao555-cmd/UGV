#!/usr/bin/env python3
"""Rolling local costmap with dynamic marking, ray clearing, decay, and inflation."""

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class LocalCostmapConfig:
    resolution_m: float = 0.06
    width_m: float = 4.0
    height_m: float = 4.0
    dynamic_decay_s: float = 1.0
    obstacle_radius_m: float = 0.06
    semantic_radius_m: float = 0.08
    inflation_radius_m: float = 0.08
    lidar_clear_radius_m: float = 0.05
    robot_radius_m: float = 0.46
    max_obstacle_range_m: float = 4.0
    max_raytrace_range_m: float = 4.0


@dataclass
class LocalCostmapStats:
    obstacle_count: int = 0
    static_obstacle_count: int = 0
    dynamic_obstacle_count: int = 0
    lidar_marked_count: int = 0
    depth_marked_count: int = 0
    semantic_marked_count: int = 0
    ray_traced_count: int = 0
    ray_cleared_count: int = 0
    decayed_count: int = 0
    inflated_count: int = 0
    robot_cleared_count: int = 0
    last_update_time: Optional[float] = None

    def as_dict(self) -> dict:
        return {
            "obstacle_count": self.obstacle_count,
            "static_obstacle_count": self.static_obstacle_count,
            "dynamic_obstacle_count": self.dynamic_obstacle_count,
            "lidar_marked_count": self.lidar_marked_count,
            "depth_marked_count": self.depth_marked_count,
            "semantic_marked_count": self.semantic_marked_count,
            "ray_traced_count": self.ray_traced_count,
            "ray_cleared_count": self.ray_cleared_count,
            "decayed_count": self.decayed_count,
            "inflated_count": self.inflated_count,
            "robot_cleared_count": self.robot_cleared_count,
            "last_update_time": None if self.last_update_time is None else round(self.last_update_time, 3),
        }


class RollingLocalCostmap:
    def __init__(self, config: LocalCostmapConfig):
        self.config = config
        self.dynamic_cells: Dict[Tuple[int, int], Tuple[float, str]] = {}
        self.static_cells: Dict[Tuple[int, int], str] = {}
        self.center_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.stats = LocalCostmapStats()

    @property
    def resolution(self) -> float:
        return max(1e-6, float(self.config.resolution_m))

    def clear(self) -> None:
        self.dynamic_cells.clear()
        self.static_cells.clear()
        self.stats = LocalCostmapStats()

    def clear_static(self) -> None:
        self.static_cells.clear()

    def mark_static_world(self, x: float, y: float, radius_m: Optional[float] = None, label: str = "static") -> int:
        return self._mark_cells(self.static_cells, x, y, radius_m or self.config.obstacle_radius_m, label)

    def update(
        self,
        *,
        pose: Tuple[float, float, float],
        lidar_points_base: Sequence[Tuple[float, float]] = (),
        lidar_clear_points_base: Sequence[Tuple[float, float]] = (),
        depth_points_base: Sequence[Tuple[float, float]] = (),
        semantic_points_base: Sequence[Tuple[float, float]] = (),
        ray_origin_base: Tuple[float, float] = (0.0, 0.0),
        timestamp: float,
    ) -> LocalCostmapStats:
        self.center_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        stats = LocalCostmapStats(last_update_time=float(timestamp))
        stats.decayed_count = self.decay(timestamp)

        ray_origin_world = self.base_to_world(ray_origin_base)
        for point in lidar_clear_points_base:
            if not self._point_in_raytrace_range(point, ray_origin_base):
                continue
            clear_world = self.base_to_world(point)
            stats.ray_traced_count += 1
            stats.ray_cleared_count += self.clear_dynamic_ray(
                ray_origin_world,
                clear_world,
                radius_m=self.config.lidar_clear_radius_m,
            )

        for point in lidar_points_base:
            if not self._point_in_range(point):
                continue
            hit_world = self.base_to_world(point)
            stats.ray_traced_count += 1
            stats.ray_cleared_count += self.clear_dynamic_ray(
                ray_origin_world,
                hit_world,
                radius_m=self.config.lidar_clear_radius_m,
            )
            stats.lidar_marked_count += self.mark_dynamic_world(
                hit_world[0],
                hit_world[1],
                timestamp,
                radius_m=self.config.obstacle_radius_m,
                source="lidar",
            )

        for point in depth_points_base:
            if not self._point_in_range(point):
                continue
            wx, wy = self.base_to_world(point)
            stats.depth_marked_count += self.mark_dynamic_world(
                wx,
                wy,
                timestamp,
                radius_m=self.config.obstacle_radius_m,
                source="depth",
            )

        for point in semantic_points_base:
            if not self._point_in_range(point):
                continue
            wx, wy = self.base_to_world(point)
            stats.semantic_marked_count += self.mark_dynamic_world(
                wx,
                wy,
                timestamp,
                radius_m=self.config.semantic_radius_m,
                source="semantic",
            )

        self.stats = stats
        return stats

    def decay(self, timestamp: float) -> int:
        ttl = max(0.0, float(self.config.dynamic_decay_s))
        if ttl <= 0.0:
            count = len(self.dynamic_cells)
            self.dynamic_cells.clear()
            return count
        stale = [
            key
            for key, (seen_s, _source) in self.dynamic_cells.items()
            if float(timestamp) - seen_s > ttl
        ]
        for key in stale:
            self.dynamic_cells.pop(key, None)
        return len(stale)

    def mark_dynamic_world(
        self,
        x: float,
        y: float,
        timestamp: float,
        *,
        radius_m: Optional[float] = None,
        source: str = "dynamic",
    ) -> int:
        radius = self.config.obstacle_radius_m if radius_m is None else radius_m
        return self._mark_dynamic_cells(x, y, radius, float(timestamp), source)

    def clear_dynamic_ray(
        self,
        start_world: Tuple[float, float],
        end_world: Tuple[float, float],
        *,
        radius_m: Optional[float] = None,
    ) -> int:
        sx, sy = start_world
        ex, ey = end_world
        dx = ex - sx
        dy = ey - sy
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            return 0
        clear_len = max(0.0, length - max(self.config.obstacle_radius_m, self.resolution))
        if clear_len <= 0.0:
            return 0
        steps = max(1, int(math.ceil(clear_len / max(self.resolution * 0.75, 0.01))))
        clear_radius = self.config.lidar_clear_radius_m if radius_m is None else radius_m
        removed = 0
        for i in range(steps + 1):
            t = (i / steps) * (clear_len / length)
            removed += self._clear_dynamic_disk(sx + dx * t, sy + dy * t, clear_radius)
        return removed

    def base_to_world(self, point: Tuple[float, float]) -> Tuple[float, float]:
        px, py, yaw = self.center_pose
        lx, ly = point
        c = math.cos(yaw)
        s = math.sin(yaw)
        return px + lx * c - ly * s, py + lx * s + ly * c

    def make_grid(self) -> Tuple[object, np.ndarray, LocalCostmapStats]:
        res = self.resolution
        width = max(1, int(math.ceil(float(self.config.width_m) / res)))
        height = max(1, int(math.ceil(float(self.config.height_m) / res)))
        cx, cy, _yaw = self.center_pose
        origin_x = cx - 0.5 * width * res
        origin_y = cy - 0.5 * height * res
        data = np.zeros((height, width), dtype=np.uint8)

        static_count = self._paint_cells(data, origin_x, origin_y, self.static_cells.keys())
        dynamic_count = self._paint_cells(data, origin_x, origin_y, self.dynamic_cells.keys())
        pre_inflation = int(np.count_nonzero(data))
        if self.config.inflation_radius_m > 1e-6:
            data = self._inflate(data, self.config.inflation_radius_m)
        inflated_count = max(0, int(np.count_nonzero(data)) - pre_inflation)
        robot_cleared = self._clear_robot_footprint(data, origin_x, origin_y)

        stats = LocalCostmapStats(
            obstacle_count=int(np.count_nonzero(data)),
            static_obstacle_count=static_count,
            dynamic_obstacle_count=dynamic_count,
            lidar_marked_count=self.stats.lidar_marked_count,
            depth_marked_count=self.stats.depth_marked_count,
            semantic_marked_count=self.stats.semantic_marked_count,
            ray_traced_count=self.stats.ray_traced_count,
            ray_cleared_count=self.stats.ray_cleared_count,
            decayed_count=self.stats.decayed_count,
            inflated_count=inflated_count,
            robot_cleared_count=robot_cleared,
            last_update_time=self.stats.last_update_time,
        )
        self.stats = stats

        spec = type(
            "LocalGridSpec",
            (),
            {
                "resolution": res,
                "origin_x": origin_x,
                "origin_y": origin_y,
                "width": width,
                "height": height,
            },
        )()
        return spec, data, stats

    def _point_in_range(self, point: Tuple[float, float]) -> bool:
        return math.hypot(float(point[0]), float(point[1])) <= max(0.0, float(self.config.max_obstacle_range_m))

    def _point_in_raytrace_range(
        self,
        point: Tuple[float, float],
        origin: Tuple[float, float] = (0.0, 0.0),
    ) -> bool:
        return math.hypot(
            float(point[0]) - float(origin[0]),
            float(point[1]) - float(origin[1]),
        ) <= max(0.0, float(self.config.max_raytrace_range_m))

    def _world_key(self, x: float, y: float) -> Tuple[int, int]:
        res = self.resolution
        return int(math.floor(float(x) / res)), int(math.floor(float(y) / res))

    def _cell_center(self, key: Tuple[int, int]) -> Tuple[float, float]:
        res = self.resolution
        return (key[0] + 0.5) * res, (key[1] + 0.5) * res

    def _disk_keys(self, x: float, y: float, radius_m: float) -> Iterable[Tuple[int, int]]:
        res = self.resolution
        radius = max(0.0, float(radius_m))
        cells = max(0, int(math.ceil(radius / res)))
        cx, cy = self._world_key(x, y)
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                key = (cx + dx, cy + dy)
                wx, wy = self._cell_center(key)
                if math.hypot(wx - x, wy - y) <= radius + 0.5 * res:
                    yield key

    def _mark_cells(self, cells: Dict[Tuple[int, int], str], x: float, y: float, radius_m: float, label: str) -> int:
        before = len(cells)
        for key in self._disk_keys(x, y, radius_m):
            cells[key] = label
        return max(0, len(cells) - before)

    def _mark_dynamic_cells(self, x: float, y: float, radius_m: float, timestamp: float, source: str) -> int:
        before = len(self.dynamic_cells)
        for key in self._disk_keys(x, y, radius_m):
            self.dynamic_cells[key] = (timestamp, source)
        return max(0, len(self.dynamic_cells) - before)

    def _clear_dynamic_disk(self, x: float, y: float, radius_m: float) -> int:
        removed = 0
        for key in list(self._disk_keys(x, y, radius_m)):
            if key in self.dynamic_cells:
                self.dynamic_cells.pop(key, None)
                removed += 1
        return removed

    def _paint_cells(
        self,
        data: np.ndarray,
        origin_x: float,
        origin_y: float,
        cells: Iterable[Tuple[int, int]],
    ) -> int:
        res = self.resolution
        height, width = data.shape
        painted = 0
        for key in cells:
            wx, wy = self._cell_center(key)
            gx = int(math.floor((wx - origin_x) / res))
            gy = int(math.floor((wy - origin_y) / res))
            if 0 <= gx < width and 0 <= gy < height:
                if data[gy, gx] == 0:
                    painted += 1
                data[gy, gx] = 1
        return painted

    def _inflate(self, data: np.ndarray, radius_m: float) -> np.ndarray:
        cells = max(1, int(math.ceil(float(radius_m) / self.resolution)))
        inflated = data.copy()
        height, width = data.shape
        occupied = np.argwhere(data > 0)
        for gy, gx in occupied:
            for dy in range(-cells, cells + 1):
                ny = int(gy + dy)
                if ny < 0 or ny >= height:
                    continue
                for dx in range(-cells, cells + 1):
                    nx = int(gx + dx)
                    if nx < 0 or nx >= width:
                        continue
                    if dx * dx + dy * dy <= cells * cells:
                        inflated[ny, nx] = 1
        return inflated

    def _clear_robot_footprint(self, data: np.ndarray, origin_x: float, origin_y: float) -> int:
        cx, cy, _yaw = self.center_pose
        res = self.resolution
        radius = max(0.0, float(self.config.robot_radius_m))
        cells = max(0, int(math.ceil(radius / res)))
        gx0 = int(math.floor((cx - origin_x) / res))
        gy0 = int(math.floor((cy - origin_y) / res))
        height, width = data.shape
        cleared = 0
        for dy in range(-cells, cells + 1):
            gy = gy0 + dy
            if gy < 0 or gy >= height:
                continue
            for dx in range(-cells, cells + 1):
                gx = gx0 + dx
                if gx < 0 or gx >= width:
                    continue
                wx = origin_x + (gx + 0.5) * res
                wy = origin_y + (gy + 0.5) * res
                if math.hypot(wx - cx, wy - cy) <= radius and data[gy, gx] > 0:
                    data[gy, gx] = 0
                    cleared += 1
        return cleared
