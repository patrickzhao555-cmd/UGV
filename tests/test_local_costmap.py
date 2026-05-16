import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.local_costmap import LocalCostmapConfig, RollingLocalCostmap  # noqa: E402


def make_costmap(**overrides):
    values = {
        "resolution_m": 0.10,
        "width_m": 3.0,
        "height_m": 3.0,
        "dynamic_decay_s": 0.5,
        "obstacle_radius_m": 0.05,
        "inflation_radius_m": 0.10,
        "lidar_clear_radius_m": 0.08,
        "robot_radius_m": 0.15,
    }
    values.update(overrides)
    cfg = LocalCostmapConfig(**values)
    return RollingLocalCostmap(cfg)


def test_marking_creates_occupied_cells():
    cm = make_costmap()
    cm.update(pose=(0.0, 0.0, 0.0), lidar_points_base=[(1.0, 0.0)], timestamp=1.0)
    _spec, data, stats = cm.make_grid()
    assert data.sum() > 0
    assert stats.lidar_marked_count > 0


def test_ray_clearing_clears_dynamic_cells_before_hit():
    cm = make_costmap(inflation_radius_m=0.0)
    cm.mark_dynamic_world(0.5, 0.0, 1.0, radius_m=0.05, source="old")
    before = len(cm.dynamic_cells)
    cm.update(
        pose=(0.0, 0.0, 0.0),
        lidar_points_base=[(1.0, 0.0)],
        ray_origin_base=(0.0, 0.0),
        timestamp=1.1,
    )
    assert cm.stats.ray_cleared_count > 0
    assert len(cm.dynamic_cells) <= before


def test_decay_removes_stale_dynamic_obstacles():
    cm = make_costmap()
    cm.mark_dynamic_world(1.0, 0.0, 1.0, radius_m=0.05, source="old")
    assert cm.decay(1.4) == 0
    assert cm.decay(1.6) > 0
    assert not cm.dynamic_cells


def test_inflation_expands_occupied_cells():
    base = make_costmap(inflation_radius_m=0.0)
    inflated = make_costmap(inflation_radius_m=0.20)
    for cm in (base, inflated):
        cm.update(pose=(0.0, 0.0, 0.0), lidar_points_base=[(1.0, 0.0)], timestamp=1.0)
    _spec, base_data, _stats = base.make_grid()
    _spec, inflated_data, stats = inflated.make_grid()
    assert inflated_data.sum() > base_data.sum()
    assert stats.inflated_count > 0


def test_static_obstacles_are_not_cleared_by_dynamic_ray_clearing():
    cm = make_costmap(inflation_radius_m=0.0)
    cm.mark_static_world(0.5, 0.0, radius_m=0.05)
    cm.update(
        pose=(0.0, 0.0, 0.0),
        lidar_points_base=[(1.0, 0.0)],
        ray_origin_base=(0.0, 0.0),
        timestamp=1.0,
    )
    assert cm.static_cells


def test_robot_footprint_clearing_removes_output_cells_near_robot():
    cm = make_costmap(inflation_radius_m=0.0, robot_radius_m=0.35)
    cm.mark_dynamic_world(0.05, 0.0, 1.0, radius_m=0.05, source="self")
    _spec, data, stats = cm.make_grid()
    assert data.sum() == 0
    assert stats.robot_cleared_count > 0
