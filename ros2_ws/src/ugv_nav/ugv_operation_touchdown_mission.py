#!/usr/bin/env python3
"""Operation Touchdown mission supervisor.

The supervisor turns the UAV-provided field-frame marker coordinate into a
safe Nav2 staging goal, then performs a conservative terminal stop/approach
using confirmed ArUco detections when available.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from ugv_nav_core.nav2_bridge import FieldBounds, OccupancyGridSpec, Transform2D, target_units_scale
from ugv_nav_core.mission_controller import MOVING_TARGET_SPEED_MPS
from ugv_nav_core.operation_touchdown import (
    DESTINATION_RADIUS_M,
    choose_marker_staging_pose,
    coordinate_arrival_decision,
    destination_reached_by_coordinate,
    terminal_approach_command,
)


def _quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * float(yaw_rad)
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class OperationTouchdownMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_operation_touchdown_mission")

        self.declare_parameter("uav_target_topic", "/ugv/uav_target")
        self.declare_parameter("aruco_marker_topic", "/ugv/aruco_detection")
        self.declare_parameter("localization_status_topic", "/ugv_localization/status")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")
        self.declare_parameter("status_topic", "/ugv/operation_touchdown/status")
        self.declare_parameter("kill_switch_topic", "/ugv/kill_switch")
        self.declare_parameter("competition_motion_phase_topic", "/ugv/competition_motion_phase")
        self.declare_parameter("navigate_action_name", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("field_width_m", 13.716)
        self.declare_parameter("field_height_m", 13.716)
        self.declare_parameter("field_margin_m", 0.45)
        self.declare_parameter("target_units", "meters")
        self.declare_parameter("require_target_frame", True)
        self.declare_parameter("required_target_frame", "map")
        self.declare_parameter("require_costmap_for_goal", True)
        self.declare_parameter("unknown_cost_is_blocked", True)
        self.declare_parameter("goal_lethal_threshold", 100)
        self.declare_parameter("destination_radius_m", DESTINATION_RADIUS_M)
        self.declare_parameter("terminal_stop_distance_m", 1.0)
        self.declare_parameter("terminal_arrival_buffer_m", 0.20)
        self.declare_parameter("terminal_forward_speed_mps", 0.10)
        self.declare_parameter("competition_moving_target_speed_mps", MOVING_TARGET_SPEED_MPS)
        self.declare_parameter("terminal_heading_kp", 0.8)
        self.declare_parameter("terminal_max_omega_radps", 0.20)
        self.declare_parameter("marker_target_gate_radius_m", 2.274)
        self.declare_parameter("marker_lost_timeout_s", 0.75)
        self.declare_parameter("marker_search_timeout_s", 6.0)
        self.declare_parameter("status_period_s", 0.25)
        self.declare_parameter("terminal_control_period_s", 0.05)

        self.uav_target_topic = str(self.get_parameter("uav_target_topic").value)
        self.aruco_marker_topic = str(self.get_parameter("aruco_marker_topic").value)
        self.localization_status_topic = str(self.get_parameter("localization_status_topic").value)
        self.costmap_topic = str(self.get_parameter("costmap_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.kill_switch_topic = str(self.get_parameter("kill_switch_topic").value)
        self.competition_motion_phase_topic = str(self.get_parameter("competition_motion_phase_topic").value)
        self.action_name = str(self.get_parameter("navigate_action_name").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.bounds = FieldBounds(
            width_m=float(self.get_parameter("field_width_m").value),
            height_m=float(self.get_parameter("field_height_m").value),
            margin_m=float(self.get_parameter("field_margin_m").value),
        )
        self.target_units = str(self.get_parameter("target_units").value)
        self.target_unit_scale = target_units_scale(self.target_units)
        self.require_target_frame = bool(self.get_parameter("require_target_frame").value)
        self.required_target_frame = str(self.get_parameter("required_target_frame").value).strip()
        self.require_costmap_for_goal = bool(self.get_parameter("require_costmap_for_goal").value)
        self.unknown_cost_is_blocked = bool(self.get_parameter("unknown_cost_is_blocked").value)
        self.goal_lethal_threshold = int(self.get_parameter("goal_lethal_threshold").value)
        self.destination_radius_m = max(0.1, float(self.get_parameter("destination_radius_m").value))
        self.terminal_stop_distance_m = max(0.05, float(self.get_parameter("terminal_stop_distance_m").value))
        self.terminal_arrival_buffer_m = max(0.0, float(self.get_parameter("terminal_arrival_buffer_m").value))
        self.terminal_forward_speed_mps = float(self.get_parameter("terminal_forward_speed_mps").value)
        self.competition_moving_target_speed_mps = max(
            MOVING_TARGET_SPEED_MPS,
            float(self.get_parameter("competition_moving_target_speed_mps").value),
        )
        self.terminal_heading_kp = float(self.get_parameter("terminal_heading_kp").value)
        self.terminal_max_omega_radps = max(0.0, float(self.get_parameter("terminal_max_omega_radps").value))
        self.marker_target_gate_radius_m = max(0.0, float(self.get_parameter("marker_target_gate_radius_m").value))
        self.marker_lost_timeout_s = max(0.0, float(self.get_parameter("marker_lost_timeout_s").value))
        self.marker_search_timeout_s = max(0.0, float(self.get_parameter("marker_search_timeout_s").value))
        self.status_period_s = max(0.05, float(self.get_parameter("status_period_s").value))
        self.terminal_control_period_s = max(0.02, float(self.get_parameter("terminal_control_period_s").value))

        self.state = "wait_for_uav_target"
        self.reason = "startup"
        self.state_start_s = time.monotonic()
        self.latest_pose: Optional[Transform2D] = None
        self.last_pose_s: Optional[float] = None
        self.latest_grid: Optional[OccupancyGridSpec] = None
        self.target_xy_m: Optional[tuple[float, float]] = None
        self.staging_pose = None
        self.latest_marker_xy_m: Optional[tuple[float, float]] = None
        self.latest_marker_distance_m: Optional[float] = None
        self.last_marker_s: Optional[float] = None
        self.last_marker_reject_reason: Optional[str] = None
        self.last_marker_reject_s: Optional[float] = None
        self.marker_target_disagreement_m: Optional[float] = None
        self.visual_marker_used_for_motion = False
        self.nav2_goal_sent = False
        self.goal_handle = None
        self.goal_sequence = 0
        self.nav2_goal_cancel_requested = False
        self.kill_switch_active = False
        self.last_status_publish_s = 0.0
        self.last_terminal_command = {"v_mps": 0.0, "omega_radps": 0.0, "reason": "startup"}
        self.ugv_start_time_s: Optional[float] = None
        self.destination_reached_s: Optional[float] = None
        self.competition_motion_phase = "waiting_to_start"

        self.goal_client = ActionClient(self, NavigateToPose, self.action_name)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.phase_pub = self.create_publisher(String, self.competition_motion_phase_topic, 10)
        self.create_subscription(PointStamped, self.uav_target_topic, self.target_callback, 10)
        self.create_subscription(PointStamped, self.aruco_marker_topic, self.marker_callback, 10)
        self.create_subscription(String, self.localization_status_topic, self.localization_status_callback, 10)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_callback, 10)
        self.create_subscription(Bool, self.kill_switch_topic, self.kill_switch_callback, 10)
        self.create_timer(self.terminal_control_period_s, self.timer_callback)

        self.get_logger().info(
            "Operation Touchdown mission supervisor started "
            f"(target={self.uav_target_topic}, aruco={self.aruco_marker_topic}, "
            f"cmd_vel={self.cmd_vel_topic}, destination_radius={self.destination_radius_m:.3f}m)"
        )
        if self.target_unit_scale is None:
            self.get_logger().error(f"Unsupported target_units={self.target_units!r}; UAV targets will be rejected.")

    def _now_s(self) -> float:
        return time.monotonic()

    def _set_state(self, state: str, reason: str) -> None:
        if state != self.state:
            self.get_logger().info(f"Mission state {self.state} -> {state}: {reason}")
            if state == "abort":
                self._cancel_active_goal(reason)
            self.state = state
            self.state_start_s = self._now_s()
            self._update_motion_phase_for_state(reason)
        self.reason = reason

    def target_callback(self, msg: PointStamped) -> None:
        raw_x = float(msg.point.x)
        raw_y = float(msg.point.y)
        if self.target_unit_scale is None:
            self._set_state("abort", "target_units_invalid")
            return
        frame_id = str(msg.header.frame_id).strip()
        if self.require_target_frame and frame_id != self.required_target_frame:
            self._set_state("abort", "target_frame_invalid")
            return
        target_x = raw_x * self.target_unit_scale
        target_y = raw_y * self.target_unit_scale
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            self._set_state("abort", "invalid_uav_target")
            return
        self._cancel_active_goal("new_uav_target")
        self.target_xy_m = (target_x, target_y)
        self.staging_pose = None
        self.latest_marker_xy_m = None
        self.latest_marker_distance_m = None
        self.last_marker_s = None
        self.last_marker_reject_reason = None
        self.last_marker_reject_s = None
        self.marker_target_disagreement_m = None
        self.visual_marker_used_for_motion = False
        self.nav2_goal_sent = False
        self.goal_handle = None
        self.nav2_goal_cancel_requested = False
        self._set_state("plan_to_staging", "uav_target_received")

    def marker_callback(self, msg: PointStamped) -> None:
        if str(msg.header.frame_id).strip() != self.map_frame:
            self.last_marker_reject_reason = "marker_frame_invalid"
            self.last_marker_reject_s = self._now_s()
            return
        x_m = float(msg.point.x)
        y_m = float(msg.point.y)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            self.last_marker_reject_reason = "marker_pose_invalid"
            self.last_marker_reject_s = self._now_s()
            return
        if self.target_xy_m is None:
            self.last_marker_reject_reason = "marker_without_uav_target"
            self.last_marker_reject_s = self._now_s()
            return
        distance_to_target = math.hypot(x_m - self.target_xy_m[0], y_m - self.target_xy_m[1])
        self.marker_target_disagreement_m = distance_to_target
        if self.marker_target_gate_radius_m > 0.0:
            if distance_to_target > self.marker_target_gate_radius_m:
                self.last_marker_reject_reason = "marker_target_disagreement"
                self.last_marker_reject_s = self._now_s()
                return
        self.latest_marker_xy_m = (x_m, y_m)
        self.latest_marker_distance_m = float(msg.point.z) if math.isfinite(float(msg.point.z)) else None
        self.last_marker_s = self._now_s()
        self.last_marker_reject_reason = None
        self.last_marker_reject_s = None

    def localization_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            pose = payload.get("pose_m")
            if isinstance(pose, list) and len(pose) >= 3 and bool(payload.get("localization_ready", False)):
                self.latest_pose = Transform2D(float(pose[0]), float(pose[1]), math.radians(float(pose[2])))
                self.last_pose_s = self._now_s()
        except (TypeError, ValueError, json.JSONDecodeError):
            return

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

    def kill_switch_callback(self, msg: Bool) -> None:
        self.kill_switch_active = bool(msg.data)
        if self.kill_switch_active:
            self._set_state("abort", "kill_switch")
            self._set_motion_phase("kill_switch")

    def timer_callback(self) -> None:
        now_s = self._now_s()
        if self.kill_switch_active:
            self._publish_stop("kill_switch")
            self._publish_motion_phase()
            self._publish_status(now_s, force=True)
            return

        if self._coordinate_terminal_gate(now_s):
            self._publish_motion_phase()
            self._publish_status(now_s, force=True)
            return

        if self.state == "plan_to_staging":
            self._plan_or_wait_for_staging()
        elif self.state == "marker_search":
            self._marker_search_step(now_s)
        elif self.state == "terminal_approach":
            self._terminal_step(now_s)
        elif self.state in {"destination_stop", "mission_complete", "abort", "wait_for_uav_target"}:
            self._publish_stop(self.reason)

        self._publish_motion_phase()
        self._publish_status(now_s)

    def _coordinate_terminal_gate(self, now_s: float) -> bool:
        """Stop the mission as soon as the rulebook destination circle is reached.

        This gate intentionally runs above the state-specific logic, including
        while Nav2 is still driving to the staging goal.  The competition only
        requires the UGV to stop inside the destination radius, so ArUco visual
        refinement must never keep the robot moving after the coordinate-based
        success condition is satisfied.
        """

        if self.state not in {"plan_to_staging", "nav2_goal_pending", "nav2_to_staging", "marker_search", "terminal_approach"}:
            return False
        if self.target_xy_m is None or self.latest_pose is None:
            return False
        marker_fresh = self._marker_fresh(now_s)
        decision = coordinate_arrival_decision(
            robot_pose=self.latest_pose,
            target_x_m=self.target_xy_m[0],
            target_y_m=self.target_xy_m[1],
            marker_confirmed=marker_fresh,
            destination_radius_m=self.destination_radius_m,
            buffer_m=self.terminal_arrival_buffer_m,
        )
        if not decision.destination_reached:
            return False
        if self.state in {"nav2_goal_pending", "nav2_to_staging"}:
            self._cancel_active_goal(decision.reason)
        self._set_state("destination_stop", decision.reason)
        self._publish_stop(decision.reason)
        return True

    def _plan_or_wait_for_staging(self) -> None:
        if self.target_xy_m is None:
            self._set_state("wait_for_uav_target", "missing_target")
            return
        if self.latest_pose is None:
            if self.ugv_start_time_s is not None:
                self._set_state("abort", "active_replan_localization_missing")
                self._publish_stop(self.reason)
            else:
                self.reason = "waiting_for_localization"
                self._publish_stop(self.reason)
            return
        if self.require_costmap_for_goal and self.latest_grid is None:
            if self.ugv_start_time_s is not None:
                self._set_state("abort", "active_replan_costmap_missing")
                self._publish_stop(self.reason)
            else:
                self.reason = "waiting_for_costmap"
                self._publish_stop(self.reason)
            return
        if not self.goal_client.server_is_ready():
            if self.ugv_start_time_s is not None:
                self._set_state("abort", "active_replan_nav2_action_server_missing")
                self._publish_stop(self.reason)
            else:
                self.reason = "waiting_for_nav2_action_server"
                self._publish_stop(self.reason)
            return

        target_x, target_y = self.target_xy_m
        decision = choose_marker_staging_pose(
            robot_pose=self.latest_pose,
            marker_x_m=target_x,
            marker_y_m=target_y,
            bounds=self.bounds,
            occupancy_grid=self.latest_grid,
            destination_radius_m=self.destination_radius_m,
        )
        if not decision.accepted or decision.pose is None:
            self._set_state("abort", decision.reason)
            self._publish_stop(decision.reason)
            return
        self.staging_pose = decision.pose
        self._send_nav2_goal(decision.pose)

    def _send_nav2_goal(self, staging_pose) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = float(staging_pose.x_m)
        goal.pose.pose.position.y = float(staging_pose.y_m)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _quaternion_from_yaw(staging_pose.yaw_rad)
        self.goal_sequence += 1
        sequence = self.goal_sequence
        self.goal_handle = None
        self.nav2_goal_cancel_requested = False
        future = self.goal_client.send_goal_async(goal)
        future.add_done_callback(lambda result_future, seq=sequence: self._goal_response_callback(result_future, seq))
        self.nav2_goal_sent = True
        self._set_state("nav2_goal_pending", "nav2_goal_sent")

    def _goal_response_callback(self, future, sequence: int) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            if sequence != self.goal_sequence or self.state != "nav2_goal_pending":
                return
            self._set_state("abort", f"nav2_goal_send_failed:{exc}")
            return
        if sequence != self.goal_sequence or self.state != "nav2_goal_pending":
            if getattr(handle, "accepted", False):
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            return
        if not handle.accepted:
            self._set_state("abort", "nav2_goal_rejected")
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda done_future, seq=sequence: self._goal_result_callback(done_future, seq))
        self._mark_run_started()
        self._set_state("nav2_to_staging", "nav2_goal_accepted")

    def _goal_result_callback(self, future, sequence: int) -> None:
        if sequence != self.goal_sequence or self.state != "nav2_to_staging":
            return
        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            self._set_state("abort", f"nav2_result_failed:{exc}")
            return
        self.goal_handle = None
        self.nav2_goal_cancel_requested = False
        if status == 4:
            self._set_state("marker_search", "nav2_staging_reached")
        else:
            self._set_state("abort", f"nav2_result_status_{status}")

    def _cancel_active_goal(self, reason: str) -> None:
        if self.goal_handle is None or self.nav2_goal_cancel_requested:
            return
        self.nav2_goal_cancel_requested = True
        try:
            self.goal_handle.cancel_goal_async()
            self.get_logger().warn(f"Canceling active Nav2 goal: {reason}")
        except Exception as exc:
            self.get_logger().warn(f"Failed to request Nav2 goal cancel ({reason}): {exc}")

    def _marker_search_step(self, now_s: float) -> None:
        if self.target_xy_m is None or self.latest_pose is None:
            self._set_state("abort", "terminal_missing_pose_or_target")
            self._publish_stop(self.reason)
            return
        target_x, target_y = self.target_xy_m
        marker_fresh = self._marker_fresh(now_s)
        if self._recent_marker_disagreement(now_s):
            self.reason = "marker_target_disagreement_coordinate_crawl"
            self._publish_coordinate_crawl(self.reason)
            return
        if marker_fresh:
            self._set_state("terminal_approach", "marker_confirmed")
            return
        if self.marker_search_timeout_s > 0.0 and now_s - self.state_start_s > self.marker_search_timeout_s:
            self._set_state("terminal_approach", "marker_search_timeout_coordinate_fallback")
            self._publish_coordinate_crawl("marker_search_timeout_coordinate_fallback")
            return
        self._publish_coordinate_crawl("marker_search_coordinate_crawl")

    def _terminal_step(self, now_s: float) -> None:
        if self.latest_pose is None or self.target_xy_m is None:
            self._set_state("abort", "terminal_missing_pose_or_target")
            self._publish_stop(self.reason)
            return
        marker_fresh = self._marker_fresh(now_s)
        coordinate_decision = coordinate_arrival_decision(
            robot_pose=self.latest_pose,
            target_x_m=self.target_xy_m[0],
            target_y_m=self.target_xy_m[1],
            marker_confirmed=marker_fresh,
            destination_radius_m=self.destination_radius_m,
            buffer_m=self.terminal_arrival_buffer_m,
        )
        if coordinate_decision.destination_reached:
            self._set_state("destination_stop", coordinate_decision.reason)
            self._publish_stop(coordinate_decision.reason)
            return
        if self._recent_marker_disagreement(now_s):
            self.reason = "marker_target_disagreement_coordinate_crawl"
            self._publish_coordinate_crawl(self.reason)
            return
        marker_xy = self.latest_marker_xy_m if marker_fresh and self.latest_marker_xy_m is not None else self.target_xy_m
        decision = terminal_approach_command(
            robot_pose=self.latest_pose,
            marker_x_m=marker_xy[0],
            marker_y_m=marker_xy[1],
            marker_fresh=marker_fresh,
            destination_radius_m=self.destination_radius_m,
            terminal_stop_distance_m=self.terminal_stop_distance_m,
            terminal_arrival_buffer_m=self.terminal_arrival_buffer_m,
            forward_speed_mps=self.terminal_forward_speed_mps,
            heading_kp=self.terminal_heading_kp,
            max_omega_radps=self.terminal_max_omega_radps,
            moving_target_speed_mps=self.competition_moving_target_speed_mps,
        )
        if decision.destination_reached:
            self._set_state("destination_stop", decision.reason)
            self._publish_stop(decision.reason)
            return
        self.visual_marker_used_for_motion = marker_fresh and decision.command.command_type == "velocity"
        self._publish_velocity(decision.command.v_mps, decision.command.omega_radps, decision.reason)

    def _mark_run_started(self) -> None:
        if self.ugv_start_time_s is None:
            self.ugv_start_time_s = self._now_s()
        self._set_motion_phase("active_movement")

    def _set_motion_phase(self, phase: str) -> None:
        self.competition_motion_phase = str(phase)

    def _update_motion_phase_for_state(self, reason: str) -> None:
        if self.state in {"wait_for_uav_target", "plan_to_staging", "nav2_goal_pending"} and self.ugv_start_time_s is None:
            self._set_motion_phase("waiting_to_start")
        elif self.state == "nav2_to_staging":
            self._set_motion_phase("path_following")
        elif self.state == "marker_search":
            self._set_motion_phase("marker_search")
        elif self.state == "terminal_approach":
            self._set_motion_phase("terminal_approach")
        elif self.state in {"destination_stop", "mission_complete"}:
            self.destination_reached_s = self.destination_reached_s or self._now_s()
            self._set_motion_phase("destination_reached")
        elif self.state == "abort":
            if reason == "kill_switch":
                self._set_motion_phase("kill_switch")
            else:
                self._set_motion_phase("fault")
        elif self.ugv_start_time_s is not None:
            self._set_motion_phase("replanning")

    def _publish_motion_phase(self) -> None:
        self.phase_pub.publish(String(data=str(self.competition_motion_phase)))

    def _publish_coordinate_crawl(self, reason: str) -> None:
        if self.latest_pose is None or self.target_xy_m is None:
            self._set_state("abort", "coordinate_crawl_pose_or_target_missing")
            self._publish_stop(self.reason)
            return
        decision = terminal_approach_command(
            robot_pose=self.latest_pose,
            marker_x_m=self.target_xy_m[0],
            marker_y_m=self.target_xy_m[1],
            marker_fresh=False,
            destination_radius_m=self.destination_radius_m,
            terminal_stop_distance_m=self.terminal_stop_distance_m,
            terminal_arrival_buffer_m=self.terminal_arrival_buffer_m,
            forward_speed_mps=self.competition_moving_target_speed_mps,
            heading_kp=self.terminal_heading_kp,
            max_omega_radps=self.terminal_max_omega_radps,
            moving_target_speed_mps=self.competition_moving_target_speed_mps,
        )
        if decision.destination_reached:
            self._set_state("destination_stop", decision.reason)
            self._publish_stop(decision.reason)
            return
        if decision.command.command_type == "velocity":
            self._publish_velocity(decision.command.v_mps, decision.command.omega_radps, reason)
        else:
            self._publish_velocity(self.competition_moving_target_speed_mps, 0.0, reason)

    def _marker_fresh(self, now_s: float) -> bool:
        return self.last_marker_s is not None and now_s - self.last_marker_s <= self.marker_lost_timeout_s

    def _recent_marker_disagreement(self, now_s: float) -> bool:
        return (
            self.last_marker_reject_reason == "marker_target_disagreement"
            and self.last_marker_reject_s is not None
            and now_s - self.last_marker_reject_s <= self.marker_lost_timeout_s
        )

    def _publish_velocity(self, v_mps: float, omega_radps: float, reason: str) -> None:
        msg = Twist()
        msg.linear.x = float(v_mps)
        msg.angular.z = float(omega_radps)
        self.cmd_pub.publish(msg)
        self.last_terminal_command = {
            "v_mps": float(v_mps),
            "omega_radps": float(omega_radps),
            "reason": str(reason),
        }

    def _publish_stop(self, reason: str) -> None:
        self.visual_marker_used_for_motion = False
        self._publish_velocity(0.0, 0.0, reason)
        if self.state == "destination_stop":
            self._set_state("mission_complete", reason)

    def _publish_status(self, now_s: float, *, force: bool = False) -> None:
        if not force and now_s - self.last_status_publish_s < self.status_period_s:
            return
        self.last_status_publish_s = now_s
        target_distance = None
        coordinate_destination_reached = False
        if self.target_xy_m is not None and self.latest_pose is not None:
            target_distance = math.hypot(self.latest_pose.x - self.target_xy_m[0], self.latest_pose.y - self.target_xy_m[1])
            coordinate_destination_reached = destination_reached_by_coordinate(
                robot_x_m=self.latest_pose.x,
                robot_y_m=self.latest_pose.y,
                target_x_m=self.target_xy_m[0],
                target_y_m=self.target_xy_m[1],
                destination_radius_m=self.destination_radius_m,
                buffer_m=self.terminal_arrival_buffer_m,
            )
        payload = {
            "node": "ugv_operation_touchdown_mission",
            "mission_state": self.state,
            "reason": self.reason,
            "state_elapsed_s": now_s - self.state_start_s,
            "target_m": None if self.target_xy_m is None else [self.target_xy_m[0], self.target_xy_m[1]],
            "staging_pose_m": None
            if self.staging_pose is None
            else [self.staging_pose.x_m, self.staging_pose.y_m, math.degrees(self.staging_pose.yaw_rad)],
            "destination_radius_m": self.destination_radius_m,
            "distance_to_uav_target_m": target_distance,
            "coordinate_distance_to_target_m": target_distance,
            "coordinate_destination_reached": coordinate_destination_reached,
            "latest_marker_m": None if self.latest_marker_xy_m is None else [self.latest_marker_xy_m[0], self.latest_marker_xy_m[1]],
            "marker_age_s": None if self.last_marker_s is None else now_s - self.last_marker_s,
            "marker_distance_m": self.latest_marker_distance_m,
            "marker_target_disagreement_m": self.marker_target_disagreement_m,
            "marker_target_gate_radius_m": self.marker_target_gate_radius_m,
            "last_marker_reject_reason": self.last_marker_reject_reason,
            "visual_marker_used_for_motion": self.visual_marker_used_for_motion,
            "last_terminal_command": self.last_terminal_command,
            "ugv_start_time_s": self.ugv_start_time_s,
            "destination_reached_s": self.destination_reached_s,
            "competition_motion_phase": self.competition_motion_phase,
            "moving_target_speed_mps": self.competition_moving_target_speed_mps,
            "commanded_speed_mps": abs(float(self.last_terminal_command.get("v_mps", 0.0))),
            "kill_switch_active": self.kill_switch_active,
            "costmap_available": self.latest_grid is not None,
            "localization_available": self.latest_pose is not None,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main() -> None:
    rclpy.init()
    node = OperationTouchdownMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
