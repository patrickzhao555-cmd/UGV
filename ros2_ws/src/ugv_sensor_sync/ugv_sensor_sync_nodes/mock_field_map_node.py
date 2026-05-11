#!/usr/bin/env python3
import json
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_OBSTACLES: List[Tuple[int, int]] = [
    (3, 5),
    (4, 9),
    (6, 6),
    (7, 10),
    (9, 3),
    (10, 12),
    (12, 7),
]

YARD_TO_M = 0.9144

START_CELLS = {
    'upper_left': (0, 0),
    'upper_right': (0, 14),
    'lower_left': (14, 0),
    'lower_right': (14, 14),
}


def normalize_corner(name: str) -> str:
    value = str(name or 'lower_left').strip().lower().replace('-', '_').replace(' ', '_')
    aliases = {
        'top_left': 'upper_left',
        'top_right': 'upper_right',
        'bottom_left': 'lower_left',
        'bottom_right': 'lower_right',
        'ul': 'upper_left',
        'ur': 'upper_right',
        'll': 'lower_left',
        'lr': 'lower_right',
    }
    value = aliases.get(value, value)
    if value not in START_CELLS:
        return 'lower_left'
    return value


def parse_cell(text: str, default: Tuple[int, int]) -> Tuple[int, int]:
    try:
        parts = [int(p.strip()) for p in str(text).split(',')]
        if len(parts) >= 2:
            return parts[0], parts[1]
    except ValueError:
        pass
    return default


class MockFieldMapNode(Node):
    def __init__(self):
        super().__init__('mock_field_map_node')

        self.declare_parameter('topic', '/ugv/target')
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('field_size', 15)
        self.declare_parameter('cell_size_m', YARD_TO_M)
        self.declare_parameter('start_corner', 'lower_left')
        self.declare_parameter('marker_cell', '7,7')
        self.declare_parameter('target_x_m', float('nan'))
        self.declare_parameter('target_y_m', float('nan'))
        self.declare_parameter('publish_format', 'target_xy')
        self.declare_parameter('use_default_obstacles', True)
        self.declare_parameter('obstacles_json', '')

        topic = self.get_parameter('topic').value
        self.publish_period_s = max(0.2, float(self.get_parameter('publish_period_s').value))
        self.field_size = int(self.get_parameter('field_size').value)
        self.cell_size_m = float(self.get_parameter('cell_size_m').value)
        self.start_corner = normalize_corner(self.get_parameter('start_corner').value)
        self.marker_cell = parse_cell(self.get_parameter('marker_cell').value, (7, 7))
        self.target_x_m = float(self.get_parameter('target_x_m').value)
        self.target_y_m = float(self.get_parameter('target_y_m').value)
        self.publish_format = str(self.get_parameter('publish_format').value).strip().lower()
        self.use_default_obstacles = bool(self.get_parameter('use_default_obstacles').value)
        self.obstacles_json = self.get_parameter('obstacles_json').value
        self.extra_obstacles = self._parse_obstacles_json(self.obstacles_json)

        self.pub = self.create_publisher(String, topic, 10)
        self.create_timer(self.publish_period_s, self.publish_map)
        self.get_logger().info(
            f'Mock field-map publisher started (topic={topic}, start_corner={self.start_corner}, '
            f'marker_cell={self.marker_cell}, target=({self.target_x_m}, {self.target_y_m}), '
            f'format={self.publish_format}, extra_obstacles={len(self.extra_obstacles)}, '
            f'period={self.publish_period_s}s)'
        )
        self.publish_map()

    def _parse_obstacles_json(self, raw: str) -> List[Tuple[int, int]]:
        if not raw:
            return []
        try:
            parsed = json.loads(str(raw))
        except Exception as exc:
            self.get_logger().warn(f'Invalid obstacles_json; using defaults only: {exc}')
            return []
        if not isinstance(parsed, list):
            self.get_logger().warn(
                'Invalid obstacles_json; expected a list of [row, col] or {"row": r, "col": c} entries'
            )
            return []

        obstacles: List[Tuple[int, int]] = []
        for item in parsed:
            try:
                if isinstance(item, dict):
                    obstacles.append((int(item['row']), int(item['col'])))
                elif isinstance(item, list) and len(item) >= 2:
                    obstacles.append((int(item[0]), int(item[1])))
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warn(f'Skipping invalid mock obstacle entry {item!r}: {exc}')
        return obstacles

    def publish_map(self) -> None:
        size = self.field_size
        marker_xy = self._target_xy_m()
        if self.publish_format in {'target', 'target_xy', 'xy', 'point'}:
            payload = {
                'type': 'ugv_target_v1',
                'source': 'mock_target_node',
                'frame': 'field_bottom_left',
                'unit': 'm',
                'x': marker_xy[0],
                'y': marker_xy[1],
            }
            self.pub.publish(String(data=json.dumps(payload)))
            return

        grid = [[0 for _ in range(size)] for _ in range(size)]
        start = START_CELLS.get(self.start_corner, START_CELLS['lower_left'])
        marker = self.marker_cell

        obstacles = list(DEFAULT_OBSTACLES) if self.use_default_obstacles else []
        obstacles.extend(self.extra_obstacles)

        for row, col in obstacles:
            if 0 <= row < size and 0 <= col < size and (row, col) not in {start, marker}:
                grid[row][col] = 1
        if 0 <= start[0] < size and 0 <= start[1] < size:
            grid[start[0]][start[1]] = 2
        if 0 <= marker[0] < size and 0 <= marker[1] < size:
            grid[marker[0]][marker[1]] = 3

        payload = {
            'type': 'ugv_field_map_v1',
            'source': 'mock_field_map_node',
            'size': size,
            'cell_size_yard': 1.0,
            'matrix_origin': 'top_left',
            'world_origin': 'lower_left',
            'legend': {
                '0': 'unknown_or_free',
                '1': 'obstacle',
                '2': 'ugv_start',
                '3': 'marker_destination',
            },
            'matrix': grid,
        }
        self.pub.publish(String(data=json.dumps(payload)))

    def _target_xy_m(self) -> Tuple[float, float]:
        if math.isfinite(self.target_x_m) and math.isfinite(self.target_y_m):
            return self.target_x_m, self.target_y_m
        row, col = self.marker_cell
        x = (float(col) + 0.5) * self.cell_size_m
        y = (float(self.field_size - 1 - row) + 0.5) * self.cell_size_m
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = MockFieldMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
