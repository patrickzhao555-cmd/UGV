#!/usr/bin/env python3
"""Validate field-frame UAV targets and send them to Nav2 NavigateToPose."""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from ugv_nav_core.nav2_bridge import (
    FieldBounds,
    OccupancyGridSpec,
    find_nearest_free_target,
    target_units_scale,
    validate_field_target,
)


def _quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * float(yaw_rad)
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class UavGoalBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_uav_goal_bridge")

        self.declare_parameter("target_topic", "/ugv/uav_target")
        self.declare_parameter("status_topic", "/ugv/uav_goal_bridge/status")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("navigate_action_name", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("field_width_m", 13.716)
        self.declare_parameter("field_height_m", 13.716)
        self.declare_parameter("field_margin_m", 0.45)
        self.declare_parameter("goal_yaw_deg", 0.0)
        self.declare_parameter("allow_boundary_projection", False)
        self.declare_parameter("require_costmap_for_goal", True)
        self.declare_parameter("unknown_cost_is_blocked", True)
        self.declare_parameter("require_target_frame", True)
        self.declare_parameter("required_target_frame", "map")
        self.declare_parameter("target_units", "meters")
        self.declare_parameter("goal_lethal_threshold", 100)
        self.declare_parameter("occupied_goal_search_radius_m", 1.0)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.costmap_topic = str(self.get_parameter("costmap_topic").value)
        self.action_name = str(self.get_parameter("navigate_action_name").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.bounds = FieldBounds(
            width_m=float(self.get_parameter("field_width_m").value),
            height_m=float(self.get_parameter("field_height_m").value),
            margin_m=float(self.get_parameter("field_margin_m").value),
        )
        self.goal_yaw_rad = math.radians(float(self.get_parameter("goal_yaw_deg").value))
        self.allow_boundary_projection = bool(self.get_parameter("allow_boundary_projection").value)
        self.require_costmap_for_goal = bool(self.get_parameter("require_costmap_for_goal").value)
        self.unknown_cost_is_blocked = bool(self.get_parameter("unknown_cost_is_blocked").value)
        self.require_target_frame = bool(self.get_parameter("require_target_frame").value)
        self.required_target_frame = str(self.get_parameter("required_target_frame").value).strip()
        self.target_units = str(self.get_parameter("target_units").value).strip()
        self.target_unit_scale = target_units_scale(self.target_units)
        self.goal_lethal_threshold = int(self.get_parameter("goal_lethal_threshold").value)
        self.occupied_goal_search_radius_m = max(0.0, float(self.get_parameter("occupied_goal_search_radius_m").value))

        self.latest_grid: Optional[OccupancyGridSpec] = None
        self.last_target_s: Optional[float] = None
        self.last_status: dict = {"state": "idle", "reason": "startup"}

        self.goal_client = ActionClient(self, NavigateToPose, self.action_name)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(PointStamped, self.target_topic, self.target_callback, 10)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_callback, 10)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            f"UAV goal bridge started: {self.target_topic} -> Nav2 {self.action_name} "
            f"(field={self.bounds.width_m:.2f}x{self.bounds.height_m:.2f}m, "
            f"target_frame={self.required_target_frame}, target_units={self.target_units}, "
            f"costmap_required={self.require_costmap_for_goal}, unknown_blocked={self.unknown_cost_is_blocked})"
        )
        if self.target_unit_scale is None:
            self.get_logger().error(f"Unsupported UAV target_units={self.target_units!r}; targets will be rejected.")

    def costmap_callback(self, msg: OccupancyGrid) -> None:
        self.latest_grid = OccupancyGridSpec(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution_m=float(msg.info.resolution),
            origin_x_m=float(msg.info.origin.position.x),
            origin_y_m=float(msg.info.origin.position.y),
            data=list(msg.data),
            lethal_threshold=self.goal_lethal_threshold,
            unknown_is_blocked=self.unknown_cost_is_blocked,
        )

    def target_callback(self, msg: PointStamped) -> None:
        self.last_target_s = time.monotonic()
        raw_x = float(msg.point.x)
        raw_y = float(msg.point.y)
        if self.target_unit_scale is None:
            self._set_status("rejected", "target_units_invalid", raw_x, raw_y)
            return
        frame_id = str(msg.header.frame_id).strip()
        if self.require_target_frame and frame_id != self.required_target_frame:
            self._set_status("rejected", "target_frame_invalid", raw_x, raw_y)
            self.last_status["target_frame"] = frame_id
            self.last_status["required_target_frame"] = self.required_target_frame
            self.publish_status()
            self.get_logger().warn(
                f"Rejected UAV target frame={frame_id!r}; expected {self.required_target_frame!r}."
            )
            return
        target_x_m = raw_x * self.target_unit_scale
        target_y_m = raw_y * self.target_unit_scale
        grid = self.latest_grid
        if self.require_costmap_for_goal and grid is None:
            self._set_status("rejected", "costmap_missing", target_x_m, target_y_m)
            return

        validation = validate_field_target(
            x_m=target_x_m,
            y_m=target_y_m,
            bounds=self.bounds,
            occupancy_grid=grid,
            allow_projection=self.allow_boundary_projection,
        )
        goal_x = validation.x_m
        goal_y = validation.y_m
        adjusted = validation.adjusted
        if not validation.accepted and validation.reason == "target_occupied" and grid is not None:
            nearest = find_nearest_free_target(
                x_m=validation.x_m,
                y_m=validation.y_m,
                grid=grid,
                max_search_radius_m=self.occupied_goal_search_radius_m,
            )
            if nearest is not None:
                goal_x, goal_y = nearest
                adjusted = True
                validation = validate_field_target(
                    x_m=goal_x,
                    y_m=goal_y,
                    bounds=self.bounds,
                    occupancy_grid=grid,
                    allow_projection=True,
                )

        if not validation.accepted:
            self._set_status("rejected", validation.reason, validation.x_m, validation.y_m)
            self.get_logger().warn(
                f"Rejected UAV target: {validation.reason} ({target_x_m:.3f}, {target_y_m:.3f})"
            )
            return
        if not self.goal_client.server_is_ready():
            self._set_status("rejected", "nav2_action_server_not_ready", goal_x, goal_y)
            self.get_logger().warn("Nav2 NavigateToPose action server is not ready; target not sent.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = float(goal_x)
        goal.pose.pose.position.y = float(goal_y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _quaternion_from_yaw(self.goal_yaw_rad)
        send_future = self.goal_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_callback)
        reason = "sent_adjusted_goal" if adjusted else "sent_goal"
        self._set_status("sent", reason, goal_x, goal_y)
        self.get_logger().info(f"Sent Nav2 goal: ({goal_x:.3f}, {goal_y:.3f}) reason={reason}")

    def _goal_response_callback(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.last_status.update({"state": "rejected", "reason": f"send_failed:{exc}"})
            return
        if not handle.accepted:
            self.last_status.update({"state": "rejected", "reason": "goal_rejected_by_nav2"})
            return
        self.last_status.update({"state": "accepted", "reason": "goal_accepted_by_nav2"})
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            self.last_status.update({"state": "result_error", "reason": str(exc)})
            return
        self.last_status.update({"state": "complete", "reason": f"nav2_result_status_{status}"})

    def _set_status(self, state: str, reason: str, x_m: float, y_m: float) -> None:
        self.last_status = {
            "state": state,
            "reason": reason,
            "target_m": [float(x_m), float(y_m)],
            "target_units": self.target_units,
            "target_unit_scale": self.target_unit_scale,
            "required_target_frame": self.required_target_frame,
            "require_target_frame": self.require_target_frame,
            "field_width_m": self.bounds.width_m,
            "field_height_m": self.bounds.height_m,
            "field_margin_m": self.bounds.margin_m,
            "costmap_available": self.latest_grid is not None,
            "require_costmap_for_goal": self.require_costmap_for_goal,
            "unknown_cost_is_blocked": self.unknown_cost_is_blocked,
            "allow_boundary_projection": self.allow_boundary_projection,
        }
        self.publish_status()

    def publish_status(self) -> None:
        payload = dict(self.last_status)
        payload["node"] = "ugv_uav_goal_bridge"
        payload["last_target_age_s"] = None if self.last_target_s is None else time.monotonic() - self.last_target_s
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main() -> None:
    rclpy.init()
    node = UavGoalBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
