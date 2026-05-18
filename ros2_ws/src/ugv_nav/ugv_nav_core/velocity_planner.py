from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple, Type

import numpy as np


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class _VelocityCommand:
    mode: str
    turn_deg: float = 0.0
    move_m: float = 0.0
    raw_left: float = 0.0
    raw_right: float = 0.0
    reason: str = ""
    v_mps: float = 0.0
    omega_radps: float = 0.0
    controller: str = "velocity"
    command_type: str = "velocity"

    def as_dict(self) -> dict:
        return {
            "mode": self.mode,
            "command_type": self.command_type,
            "turn_deg": round(self.turn_deg, 3),
            "move_m": round(self.move_m, 4),
            "raw_left": round(self.raw_left, 3),
            "raw_right": round(self.raw_right, 3),
            "reason": self.reason,
            "v_mps": round(self.v_mps, 4),
            "omega_radps": round(self.omega_radps, 4),
            "controller": self.controller,
        }


class VelocityLocalPlanner:
    """
    Nav2-inspired local controller for tank drive.

    The global/search logic still provides a waypoint path. This controller keeps
    the command layer continuous: sample (v, omega), roll each trajectory forward,
    score it, then apply acceleration limits and low-pass filtering before raw PWM.
    """

    def __init__(
        self,
        robot_cfg: Any,
        nav_cfg: Any,
        collision_checker: Any,
        *,
        command_cls: Optional[Type[Any]] = None,
        pose_cls: Optional[Type[Any]] = None,
    ):
        self.robot_cfg = robot_cfg
        self.nav_cfg = nav_cfg
        self.checker = collision_checker
        self.command_cls = command_cls
        self.pose_cls = pose_cls
        self.last_v = 0.0
        self.last_omega = 0.0
        self.last_timestamp: Optional[float] = None
        self.last_debug: Dict[str, Any] = {"enabled": bool(nav_cfg.continuous_control_enabled)}

    def reset(self) -> None:
        self.last_v = 0.0
        self.last_omega = 0.0
        self.last_timestamp = None

    def _make_pose(self, x: float, y: float, yaw: float, like_pose: Optional[Any] = None) -> Any:
        pose_cls = self.pose_cls or (like_pose.__class__ if like_pose is not None else None)
        if pose_cls is None:
            raise RuntimeError("VelocityLocalPlanner needs a pose class before simulating trajectories.")
        return pose_cls(x, y, yaw)

    def _make_command(self, *args, **kwargs) -> Any:
        command_cls = self.command_cls or _VelocityCommand
        return command_cls(*args, **kwargs)

    def _gap_depth_for_heading(self, hits_local: Sequence[Tuple[float, float]], heading: float) -> float:
        half_width = 0.5 * self.robot_cfg.width_m + self.nav_cfg.continuous_gap_buffer_m
        max_depth = max(0.45, self.nav_cfg.continuous_gap_lookahead_m)
        min_forward = 0.5 * self.robot_cfg.length_m + 0.04
        c = math.cos(heading)
        s = math.sin(heading)
        depth = max_depth
        for hx, hy in hits_local:
            fx = hx * c + hy * s
            fy = -hx * s + hy * c
            if fx < min_forward or fx > max_depth:
                continue
            if abs(fy) <= half_width:
                depth = min(depth, fx)
        return depth

    def _polar_gap(self, hits_local: Sequence[Tuple[float, float]], desired_heading: float) -> Dict[str, float]:
        best_heading = 0.0
        best_depth = 0.0
        best_score = -1e18
        headings = [math.radians(v) for v in range(-90, 91, 5)]
        for heading in headings:
            depth = self._gap_depth_for_heading(hits_local, heading)
            alignment = math.cos(wrap_to_pi(heading - desired_heading))
            center_bias = math.cos(heading)
            score = depth + 0.22 * alignment + 0.08 * center_bias
            if score > best_score:
                best_score = score
                best_heading = heading
                best_depth = depth
        return {
            "best_heading_rad": best_heading,
            "best_heading_deg": math.degrees(best_heading),
            "best_depth_m": best_depth,
            "front_depth_m": self._gap_depth_for_heading(hits_local, 0.0),
        }

    def _trajectory_min_clearance(
        self,
        local_states: Sequence[Tuple[float, float, float]],
        hits_local: Sequence[Tuple[float, float]],
        speed_mps: float,
    ) -> float:
        half_l = 0.5 * self.robot_cfg.length_m + self.robot_cfg.obstacle_buffer_m
        half_w = 0.5 * self.robot_cfg.width_m + self.robot_cfg.obstacle_buffer_m
        latency_pad = abs(speed_mps) * max(0.0, self.nav_cfg.continuous_latency_buffer_s)
        half_l += latency_pad
        best = float('inf')
        for sx, sy, syaw in local_states:
            c = math.cos(syaw)
            s = math.sin(syaw)
            for hx, hy in hits_local:
                dx = hx - sx
                dy = hy - sy
                bx = dx * c + dy * s
                by = -dx * s + dy * c
                if bx < -half_l - 0.15:
                    continue
                ox = max(abs(bx) - half_l, 0.0)
                oy = max(abs(by) - half_w, 0.0)
                if ox <= 0.0 and oy <= 0.0:
                    return -0.01
                best = min(best, math.hypot(ox, oy))
        return best

    def _simulate(
        self,
        pose: Any,
        v: float,
        omega: float,
        horizon_s: float,
        dt_s: float,
    ) -> Tuple[Any, List[Tuple[float, float, float]]]:
        x = pose.x
        y = pose.y
        yaw = pose.yaw
        lx = 0.0
        ly = 0.0
        lyaw = 0.0
        local_states: List[Tuple[float, float, float]] = []
        steps = max(1, int(math.ceil(horizon_s / max(0.02, dt_s))))
        step_dt = horizon_s / steps
        for _ in range(steps):
            if abs(omega) < 1e-6:
                x += v * math.cos(yaw) * step_dt
                y += v * math.sin(yaw) * step_dt
                lx += v * math.cos(lyaw) * step_dt
                ly += v * math.sin(lyaw) * step_dt
            else:
                yaw_mid = yaw + 0.5 * omega * step_dt
                lyaw_mid = lyaw + 0.5 * omega * step_dt
                x += v * math.cos(yaw_mid) * step_dt
                y += v * math.sin(yaw_mid) * step_dt
                lx += v * math.cos(lyaw_mid) * step_dt
                ly += v * math.sin(lyaw_mid) * step_dt
            yaw = wrap_to_pi(yaw + omega * step_dt)
            lyaw = wrap_to_pi(lyaw + omega * step_dt)
            local_states.append((lx, ly, lyaw))
        return self._make_pose(x, y, yaw, pose), local_states

    def _trajectory_in_collision(
        self,
        costmap: Any,
        start_pose: Any,
        local_states: Sequence[Tuple[float, float, float]],
    ) -> bool:
        """Check the actual rolled-out arc, not just the chord from start to end."""
        c0 = math.cos(start_pose.yaw)
        s0 = math.sin(start_pose.yaw)
        for lx, ly, lyaw in local_states:
            wx = start_pose.x + lx * c0 - ly * s0
            wy = start_pose.y + lx * s0 + ly * c0
            wyaw = wrap_to_pi(start_pose.yaw + lyaw)
            if self.checker._pose_in_collision(costmap, self._make_pose(wx, wy, wyaw, start_pose)):
                return True
        return False

    def _speed_cap_from_clearance(self, front_clearance_m: float) -> Tuple[float, str]:
        stop = max(
            self.nav_cfg.continuous_stop_clearance_m,
            0.5 * self.robot_cfg.length_m + self.nav_cfg.front_safety_margin_m + 0.05,
        )
        slowdown = max(stop + 0.20, self.nav_cfg.continuous_slowdown_clearance_m)
        if not math.isfinite(front_clearance_m):
            return self.nav_cfg.continuous_max_speed_mps, "clear"
        if front_clearance_m <= stop:
            return 0.0, "stop"
        if front_clearance_m >= slowdown:
            return self.nav_cfg.continuous_max_speed_mps, "clear"
        ratio = (front_clearance_m - stop) / max(1e-6, slowdown - stop)
        return self.nav_cfg.continuous_max_speed_mps * clamp(ratio, 0.0, 1.0), "slow"

    def _sample_velocities(self, speed_cap_mps: float) -> Tuple[List[float], List[float]]:
        max_v = max(0.0, min(self.nav_cfg.continuous_max_speed_mps, speed_cap_mps))
        v_samples = [0.0]
        if max_v >= self.nav_cfg.continuous_min_speed_mps:
            count = max(2, int(self.nav_cfg.continuous_v_samples))
            vals = np.linspace(self.nav_cfg.continuous_min_speed_mps, max_v, count).tolist()
            v_samples.extend(float(v) for v in vals)
        omega_count = max(3, int(self.nav_cfg.continuous_omega_samples))
        max_omega = max(0.15, self.nav_cfg.continuous_max_omega_rps)
        omega_samples = [float(w) for w in np.linspace(-max_omega, max_omega, omega_count)]
        if 0.0 not in omega_samples:
            omega_samples.append(0.0)
        omega_samples = sorted(set(round(w, 6) for w in omega_samples))
        return v_samples, omega_samples

    def _apply_velocity_limits(self, target_v: float, target_omega: float, timestamp: float, immediate_stop: bool = False) -> Tuple[float, float]:
        if immediate_stop:
            self.last_v = 0.0
            self.last_omega = 0.0
            self.last_timestamp = timestamp
            return 0.0, 0.0

        dt = 0.10 if self.last_timestamp is None else clamp(timestamp - self.last_timestamp, 0.02, 0.25)
        self.last_timestamp = timestamp
        dv = clamp(
            target_v - self.last_v,
            -self.nav_cfg.continuous_accel_limit_mps2 * dt,
            self.nav_cfg.continuous_accel_limit_mps2 * dt,
        )
        domega = clamp(
            target_omega - self.last_omega,
            -self.nav_cfg.continuous_omega_accel_limit_rps2 * dt,
            self.nav_cfg.continuous_omega_accel_limit_rps2 * dt,
        )
        limited_v = self.last_v + dv
        limited_omega = self.last_omega + domega
        alpha = clamp(self.nav_cfg.continuous_lowpass_alpha, 0.0, 1.0)
        smooth_v = (1.0 - alpha) * self.last_v + alpha * limited_v
        smooth_omega = (1.0 - alpha) * self.last_omega + alpha * limited_omega
        if target_v <= 1e-6 and abs(target_omega) <= 1e-6:
            smooth_v = 0.0
            smooth_omega = 0.0
        if smooth_v > 0.005:
            # Forward arcs should not quietly become a pivot with one side reversing.
            max_arc_omega = max(0.02, 1.85 * smooth_v / max(1e-6, self.robot_cfg.track_width_m))
            smooth_omega = clamp(smooth_omega, -max_arc_omega, max_arc_omega)
        self.last_v = smooth_v
        self.last_omega = smooth_omega
        return smooth_v, smooth_omega

    def _velocity_to_command(
        self,
        v: float,
        omega: float,
        reason: str,
        horizon_s: float,
    ) -> Any:
        left_mps = v - 0.5 * omega * self.robot_cfg.track_width_m
        right_mps = v + 0.5 * omega * self.robot_cfg.track_width_m
        raw_left = clamp(left_mps * self.nav_cfg.continuous_raw_per_mps, -self.nav_cfg.continuous_max_raw, self.nav_cfg.continuous_max_raw)
        raw_right = clamp(right_mps * self.nav_cfg.continuous_raw_per_mps, -self.nav_cfg.continuous_max_raw, self.nav_cfg.continuous_max_raw)
        if abs(raw_left) < 1e-4:
            raw_left = 0.0
        if abs(raw_right) < 1e-4:
            raw_right = 0.0
        if abs(v) < 0.006 and abs(omega) < 0.02:
            mode = "STOP"
        elif v >= -1e-6 and raw_left >= -1e-5 and raw_right >= -1e-5:
            mode = "FORWARD"
        elif abs(v) < 0.04 or raw_left * raw_right < 0.0:
            mode = "TURN_LEFT" if omega > 0.0 else "TURN_RIGHT"
        elif v >= 0.0:
            mode = "FORWARD"
        else:
            mode = "BACKWARD"
        display_dt = min(0.20, max(0.05, self.nav_cfg.continuous_dt_s))
        return self._make_command(
            mode,
            turn_deg=abs(math.degrees(omega * display_dt)),
            move_m=abs(v * display_dt),
            raw_left=raw_left,
            raw_right=raw_right,
            reason=reason,
            v_mps=v,
            omega_radps=omega,
            controller="velocity",
            command_type="velocity" if self.nav_cfg.emit_velocity_commands else "raw",
        )

    def choose_command(
        self,
        pose: Any,
        target: Any,
        goal: Any,
        costmap: Any,
        sectors: Any,
        prev_cmd: Any,
        hits_local: Sequence[Tuple[float, float]],
        timestamp: float,
    ) -> Any:
        if self.pose_cls is None:
            self.pose_cls = pose.__class__
        if self.command_cls is None and prev_cmd is not None and hasattr(prev_cmd, "as_dict"):
            self.command_cls = prev_cmd.__class__

        goal_dist = math.hypot(goal.x - pose.x, goal.y - pose.y)
        if goal_dist <= self.nav_cfg.goal_tol_m:
            cmd = self._velocity_to_command(0.0, 0.0, "goal reached", self.nav_cfg.continuous_horizon_s)
            self.last_debug = {"enabled": True, "safety_state": "goal", "samples": 0, "safe_samples": 0}
            return cmd

        desired_heading = wrap_to_pi(math.atan2(target.y - pose.y, target.x - pose.x) - pose.yaw)
        gap = self._polar_gap(hits_local, desired_heading)
        straight_front_clearance = min(
            sectors.front_m,
            gap.get("front_depth_m", float('inf')),
        )
        gap_depth = gap.get("best_depth_m", float('inf'))
        path_clearance = straight_front_clearance
        path_clearance_source = "front"
        if math.isfinite(gap_depth):
            # Only an ahead-ish gap may raise the speed cap. A 75-90 degree side
            # opening is useful for choosing scan/turn direction, but it should
            # not make the controller think the current forward corridor is clear.
            gap_heading_abs = abs(gap["best_heading_rad"])
            if gap_heading_abs <= math.radians(35.0):
                path_clearance = max(path_clearance, gap_depth)
                path_clearance_source = "front_gap"
            elif (
                gap_heading_abs <= math.radians(40.0)
                and gap_depth >= self.nav_cfg.active_scan_front_clear_m
                and straight_front_clearance >= self.nav_cfg.active_scan_release_clear_m
            ):
                path_clearance = max(path_clearance, min(gap_depth, self.nav_cfg.continuous_slowdown_clearance_m))
                path_clearance_source = "arc_gap"
        speed_cap, safety_state = self._speed_cap_from_clearance(path_clearance)
        v_samples, omega_samples = self._sample_velocities(speed_cap)
        sweep_no_pure_turn = (
            bool(self.nav_cfg.competition_sweep_active)
            and not bool(self.nav_cfg.sweep_allow_pure_turn)
            and safety_state != "stop"
            and goal_dist > self.nav_cfg.goal_tol_m
        )
        sweep_large_heading_rad = math.radians(max(70.0, 2.5 * float(self.nav_cfg.sweep_heading_tolerance_deg)))

        samples = 0
        safe_samples = 0
        rejections = {
            "rejected_collision": 0,
            "rejected_clearance": 0,
            "rejected_reverse_disabled": 0,
            "rejected_arc_constraint": 0,
            "rejected_sweep_pure_turn": 0,
            "rejected_other": 0,
        }
        best_score = -1e18
        best: Optional[Tuple[float, float, Any, float]] = None
        best_components: Dict[str, float] = {}
        best_costmap_soft_penalty = 0.0
        target_before = math.hypot(target.x - pose.x, target.y - pose.y)
        horizon = clamp(self.nav_cfg.continuous_horizon_s, 0.4, 2.0)
        dt = clamp(self.nav_cfg.continuous_dt_s, 0.04, 0.25)
        max_v = max(1e-6, self.nav_cfg.continuous_max_speed_mps)
        left_room_score = (
            0.65 * min(sectors.front_left_m, 3.0)
            + 0.35 * min(sectors.left_m, 3.0)
        )
        right_room_score = (
            0.65 * min(sectors.front_right_m, 3.0)
            + 0.35 * min(sectors.right_m, 3.0)
        )
        lateral_room_delta = clamp((left_room_score - right_room_score) / 3.0, -1.0, 1.0)

        for v in v_samples:
            for omega in omega_samples:
                samples += 1
                if sweep_no_pure_turn and v <= 1e-6 and abs(desired_heading) <= sweep_large_heading_rad:
                    rejections["rejected_sweep_pure_turn"] += 1
                    continue
                left_mps = v - 0.5 * omega * self.robot_cfg.track_width_m
                right_mps = v + 0.5 * omega * self.robot_cfg.track_width_m
                if v > 0.005 and (left_mps < -1e-6 or right_mps < -1e-6):
                    rejections["rejected_arc_constraint"] += 1
                    continue
                if not self.nav_cfg.allow_reverse and v < -1e-6:
                    rejections["rejected_reverse_disabled"] += 1
                    continue
                end, local_states = self._simulate(pose, v, omega, horizon, dt)
                local_clearance = self._trajectory_min_clearance(local_states, hits_local, v)
                min_metric_clearance = max(0.018, 0.70 * self.nav_cfg.continuous_gap_buffer_m)
                if local_clearance < min_metric_clearance:
                    rejections["rejected_clearance"] += 1
                    continue
                costmap_soft_penalty = 0.0
                if self._trajectory_in_collision(costmap, pose, local_states):
                    # Default to hard collision rejection. The old soft-penalty
                    # path is retained only behind an explicit tuning switch for
                    # replay experiments with coarse maps.
                    if (
                        not self.nav_cfg.continuous_allow_costmap_soft_penalty
                        or not hits_local
                        or not math.isfinite(local_clearance)
                        or local_clearance < min_metric_clearance
                    ):
                        rejections["rejected_collision"] += 1
                        continue
                    costmap_soft_penalty = 0.35
                safe_samples += 1

                target_after = math.hypot(target.x - end.x, target.y - end.y)
                progress = target_before - target_after
                goal_after = math.hypot(goal.x - end.x, goal.y - end.y)
                end_desired = math.atan2(target.y - end.y, target.x - end.x)
                heading_err = abs(wrap_to_pi(end_desired - end.yaw))
                projected_turn = omega * min(horizon, 0.8)
                gap_alignment = math.cos(wrap_to_pi(gap["best_heading_rad"] - projected_turn))
                clearance_score = clamp(local_clearance / 0.80, 0.0, 1.5)
                progress_score = clamp(progress / max(max_v * horizon, 1e-6), -1.0, 1.0)
                path_alignment_score = clamp(math.cos(heading_err), -1.0, 1.0)
                goal_heading_score = clamp(1.0 - heading_err / math.pi, 0.0, 1.0)
                gap_alignment_score = clamp(0.5 * (gap_alignment + 1.0), 0.0, 1.0)
                speed_score = clamp(v / max_v, 0.0, 1.0)
                smoothness_cost = clamp(
                    0.5 * (
                        abs(v - self.last_v) / max_v
                        + abs(omega - self.last_omega) / max(1e-6, self.nav_cfg.continuous_max_omega_rps)
                    ),
                    0.0,
                    2.0,
                )
                oscillation_cost = 0.0
                prev_omega = float(getattr(prev_cmd, "omega_radps", 0.0))
                if abs(prev_omega) > 0.05 and abs(omega) > 0.05 and prev_omega * omega < 0.0:
                    oscillation_cost = 1.0
                if omega > 0.04:
                    side_room_score = clamp(left_room_score / 3.0, 0.0, 1.0)
                    turn_side = 1.0
                elif omega < -0.04:
                    side_room_score = clamp(right_room_score / 3.0, 0.0, 1.0)
                    turn_side = -1.0
                else:
                    side_room_score = clamp(min(left_room_score, right_room_score) / 3.0, 0.0, 1.0)
                    turn_side = 0.0

                score = 4.8 * progress
                score -= 1.15 * heading_err
                score -= 0.035 * goal_after
                score += 0.70 * clearance_score
                score += 0.35 * (v / max_v)
                score += 0.22 * gap_alignment
                score += 0.18 * side_room_score
                if straight_front_clearance < self.nav_cfg.active_scan_front_clear_m and turn_side != 0.0:
                    score += 0.32 * turn_side * lateral_room_delta
                score -= costmap_soft_penalty
                score -= 0.46 * abs(v - self.last_v)
                score -= 0.16 * abs(omega - self.last_omega)
                score -= 0.04 * abs(omega)
                if abs(desired_heading) < math.radians(18.0) and v > 0.0 and abs(omega) < 0.35:
                    score += 0.28
                if safety_state == "stop" and v > 0.0:
                    score -= 3.0
                if v <= 1e-6 and abs(omega) <= 1e-6 and goal_dist > self.nav_cfg.goal_tol_m:
                    score -= 0.75

                if score > best_score:
                    best_score = score
                    best = (v, omega, end, local_clearance)
                    best_components = {
                        "progress_score": progress_score,
                        "path_alignment_score": path_alignment_score,
                        "goal_heading_score": goal_heading_score,
                        "clearance_score": clearance_score,
                        "gap_alignment_score": gap_alignment_score,
                        "speed_score": speed_score,
                        "smoothness_cost": smoothness_cost,
                        "oscillation_cost": oscillation_cost,
                        "final_score": score,
                    }
                    best_costmap_soft_penalty = costmap_soft_penalty

        if best is None:
            v, omega = self._apply_velocity_limits(0.0, 0.0, timestamp, immediate_stop=True)
            self.last_debug = {
                "enabled": True,
                "samples": samples,
                "safe_samples": safe_samples,
                "selected_v_mps": round(v, 4),
                "selected_omega_radps": round(omega, 4),
                "best_score": None,
                "min_clearance_m": None,
                "front_clearance_m": None if math.isinf(straight_front_clearance) else round(straight_front_clearance, 3),
                "path_clearance_m": None if math.isinf(path_clearance) else round(path_clearance, 3),
                "path_clearance_source": path_clearance_source,
                "front_speed_cap_mps": round(speed_cap, 4),
                "costmap_collision_policy": "soft_penalty" if self.nav_cfg.continuous_allow_costmap_soft_penalty else "hard_reject",
                "best_gap_heading_deg": round(gap["best_heading_deg"], 1),
                "best_gap_depth_m": round(gap["best_depth_m"], 3),
                "safety_state": "no_safe_trajectory",
                **rejections,
            }
            return self._velocity_to_command(v, omega, "velocity no safe trajectory", horizon)

        target_v, target_omega, _, min_clearance = best
        sweep_turn_suppressed = False
        if sweep_no_pure_turn and target_v <= 1e-6 and abs(desired_heading) <= sweep_large_heading_rad:
            target_v = min(max(0.0, speed_cap), max(self.nav_cfg.continuous_min_speed_mps, 0.05))
            max_arc_omega = 1.85 * target_v / max(1e-6, self.robot_cfg.track_width_m)
            target_omega = clamp(target_omega, -max_arc_omega, max_arc_omega)
            sweep_turn_suppressed = True
        immediate_stop = safety_state == "stop" and abs(target_omega) < 1e-6 and target_v <= 1e-6
        v, omega = self._apply_velocity_limits(target_v, target_omega, timestamp, immediate_stop=immediate_stop)
        cmd = self._velocity_to_command(
            v,
            omega,
            (
                f"velocity dwa; samples={safe_samples}/{samples} "
                f"gap={gap['best_heading_deg']:.0f}deg clear={min_clearance:.2f}m "
                f"path={path_clearance:.2f}m "
                f"state={safety_state}"
            ),
            horizon,
        )
        if sweep_no_pure_turn and cmd.mode in {"TURN_LEFT", "TURN_RIGHT"} and abs(desired_heading) <= sweep_large_heading_rad:
            forced_v = max(v, min(speed_cap, max(self.nav_cfg.continuous_min_speed_mps, 0.05)))
            forced_omega = clamp(
                omega,
                -1.85 * forced_v / max(1e-6, self.robot_cfg.track_width_m),
                1.85 * forced_v / max(1e-6, self.robot_cfg.track_width_m),
            )
            cmd = self._velocity_to_command(
                forced_v,
                forced_omega,
                cmd.reason + "; competition sweep forward arc",
                horizon,
            )
            v, omega = forced_v, forced_omega
            sweep_turn_suppressed = True
        self.last_debug = {
            "enabled": True,
            "samples": samples,
            "safe_samples": safe_samples,
            "target_v_mps": round(target_v, 4),
            "target_omega_radps": round(target_omega, 4),
            "selected_v_mps": round(v, 4),
            "selected_omega_radps": round(omega, 4),
            "raw_left": round(cmd.raw_left, 3),
            "raw_right": round(cmd.raw_right, 3),
            "best_score": round(best_score, 3),
            "final_score": round(best_components.get("final_score", best_score), 3),
            "progress_score": round(best_components.get("progress_score", 0.0), 3),
            "path_alignment_score": round(best_components.get("path_alignment_score", 0.0), 3),
            "goal_heading_score": round(best_components.get("goal_heading_score", 0.0), 3),
            "clearance_score": round(best_components.get("clearance_score", 0.0), 3),
            "gap_alignment_score": round(best_components.get("gap_alignment_score", 0.0), 3),
            "speed_score": round(best_components.get("speed_score", 0.0), 3),
            "smoothness_cost": round(best_components.get("smoothness_cost", 0.0), 3),
            "oscillation_cost": round(best_components.get("oscillation_cost", 0.0), 3),
            "min_clearance_m": round(min_clearance, 3),
            "costmap_soft_penalty": round(best_costmap_soft_penalty, 3),
            "costmap_collision_policy": "soft_penalty" if self.nav_cfg.continuous_allow_costmap_soft_penalty else "hard_reject",
            "front_clearance_m": None if math.isinf(straight_front_clearance) else round(straight_front_clearance, 3),
            "path_clearance_m": None if math.isinf(path_clearance) else round(path_clearance, 3),
            "path_clearance_source": path_clearance_source,
            "front_speed_cap_mps": round(speed_cap, 4),
            "best_gap_heading_deg": round(gap["best_heading_deg"], 1),
            "best_gap_depth_m": round(gap["best_depth_m"], 3),
            "safety_state": safety_state,
            "competition_sweep_active": bool(self.nav_cfg.competition_sweep_active),
            "sweep_pure_turn_suppressed": bool(sweep_turn_suppressed),
            "prev_v_mps": round(self.last_v, 4),
            "prev_omega_radps": round(self.last_omega, 4),
            **rejections,
        }
        return cmd


__all__ = ["VelocityLocalPlanner"]
