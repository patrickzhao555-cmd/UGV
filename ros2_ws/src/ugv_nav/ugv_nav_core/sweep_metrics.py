from __future__ import annotations

import csv
import math
import os
import time
from typing import Any, Dict, Optional, Tuple


def finite_optional(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


class SweepMetricsLogger:
    FIELDNAMES = [
        "time",
        "pose_x",
        "pose_y",
        "yaw_deg",
        "cross_track_error_m",
        "heading_error_deg",
        "final_v_mps",
        "final_omega_radps",
        "pre_scale_final_v_mps",
        "pre_scale_final_omega_radps",
        "emitted_v_mps",
        "emitted_omega_radps",
        "emitted_left_target_mps",
        "emitted_right_target_mps",
        "motor_target_left",
        "motor_target_right",
        "motor_measured_left",
        "motor_measured_right",
        "closed_loop_diverging",
        "active_cell",
        "phase",
        "sweep_subphase",
        "row_index",
        "row_direction",
        "current_lane_y_m",
        "next_lane_y_m",
        "row_end_x_m",
        "row_transition_active",
        "row_transition_style",
        "row_transition_segment",
        "row_transition_progress",
        "row_transition_done",
        "row_transition_reason",
        "row_transition_health",
        "row_transition_diverging",
        "row_transition_timeout",
        "row_transition_geometry_warning",
        "target_side_yaw_deg",
        "target_next_row_yaw_deg",
        "segment_yaw_error_deg",
        "segment_lane_error_m",
        "target_row_yaw_deg",
        "yaw_capture_error_deg",
        "lane_capture_error_m",
        "turn_radius_m",
        "turn_side",
        "transition_emitted_v_mps",
        "transition_emitted_omega_radps",
        "transition_left_target_mps",
        "transition_right_target_mps",
    ]

    def __init__(self, log_dir: str = "~/.ros/ugv_sweep_metrics"):
        expanded = os.path.expanduser(str(log_dir))
        os.makedirs(expanded, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(expanded, f"ugv_sweep_metrics_{timestamp}.csv")
        self._fp = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._fp, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self.sample_count = 0
        self._prev_pose: Optional[Tuple[float, float]] = None
        self._prev_omega_sign = 0
        self.distance_m = 0.0
        self.max_abs_cross_track_error_m = 0.0
        self.sum_cross_track_error_sq = 0.0
        self.cross_track_error_samples = 0
        self.final_lateral_error_m = 0.0
        self.max_abs_heading_error_deg = 0.0
        self.omega_sign_changes = 0
        self.divergence_count = 0
        self.physical_stall_count = 0

    @staticmethod
    def _float(value: Any, default: float = 0.0) -> float:
        out = finite_optional(value)
        return default if out is None else float(out)

    @staticmethod
    def _omega_sign(value: float) -> int:
        if value > 1e-3:
            return 1
        if value < -1e-3:
            return -1
        return 0

    def record(self, status: dict, motor_status: Optional[dict] = None) -> None:
        motor = motor_status if isinstance(motor_status, dict) else {}
        pose = status.get("pose_m") or [0.0, 0.0, 0.0]
        x = self._float(pose[0] if len(pose) > 0 else 0.0)
        y = self._float(pose[1] if len(pose) > 1 else 0.0)
        yaw_deg = self._float(pose[2] if len(pose) > 2 else 0.0)
        cl = status.get("competition_closed_loop") or {}
        v2 = status.get("competition_v2") or (status.get("mission") or {}).get("competition_v2") or {}
        cte = self._float(cl.get("cross_track_error_m"), 0.0)
        heading_error_deg = self._float(cl.get("heading_error_deg"), 0.0)
        final_v = self._float(cl.get("final_v_mps"), self._float(status.get("final_v_mps"), 0.0))
        final_omega = self._float(cl.get("final_omega_radps"), self._float(status.get("final_omega_radps"), 0.0))
        pre_scale_v = self._float(status.get("pre_scale_final_v_mps", cl.get("pre_scale_final_v_mps")), final_v)
        pre_scale_omega = self._float(status.get("pre_scale_final_omega_radps", cl.get("pre_scale_final_omega_radps")), final_omega)
        cmd = status.get("cmd") if isinstance(status.get("cmd"), dict) else {}
        emitted_v = self._float(status.get("emitted_v_mps", cl.get("emitted_v_mps", cmd.get("v_mps"))), 0.0)
        emitted_omega = self._float(
            status.get("emitted_omega_radps", cl.get("emitted_omega_radps", cmd.get("omega_radps"))),
            0.0,
        )
        row = {
            "time": self._float(status.get("stamp")),
            "pose_x": x,
            "pose_y": y,
            "yaw_deg": yaw_deg,
            "cross_track_error_m": cte,
            "heading_error_deg": heading_error_deg,
            "final_v_mps": final_v,
            "final_omega_radps": final_omega,
            "pre_scale_final_v_mps": pre_scale_v,
            "pre_scale_final_omega_radps": pre_scale_omega,
            "emitted_v_mps": emitted_v,
            "emitted_omega_radps": emitted_omega,
            "emitted_left_target_mps": self._float(status.get("emitted_left_target_mps", cl.get("emitted_left_target_mps")), 0.0),
            "emitted_right_target_mps": self._float(status.get("emitted_right_target_mps", cl.get("emitted_right_target_mps")), 0.0),
            "motor_target_left": self._float(motor.get("target_left_mps"), 0.0),
            "motor_target_right": self._float(motor.get("target_right_mps"), 0.0),
            "motor_measured_left": self._float(motor.get("measured_left_mps"), 0.0),
            "motor_measured_right": self._float(motor.get("measured_right_mps"), 0.0),
            "closed_loop_diverging": bool(cl.get("closed_loop_diverging", False)),
            "active_cell": v2.get("active_cell"),
            "phase": v2.get("phase", ""),
            "sweep_subphase": v2.get("sweep_subphase", ""),
            "row_index": v2.get("row_index"),
            "row_direction": v2.get("row_direction"),
            "current_lane_y_m": v2.get("current_lane_y_m"),
            "next_lane_y_m": v2.get("next_lane_y_m"),
            "row_end_x_m": v2.get("row_end_x_m"),
            "row_transition_active": bool(v2.get("row_transition_active", False)),
            "row_transition_style": v2.get("row_transition_style", ""),
            "row_transition_segment": v2.get("row_transition_segment", ""),
            "row_transition_progress": v2.get("row_transition_progress", 0.0),
            "row_transition_done": bool(v2.get("row_transition_done", False)),
            "row_transition_reason": v2.get("row_transition_reason", ""),
            "row_transition_health": v2.get("row_transition_health", ""),
            "row_transition_diverging": bool(v2.get("row_transition_diverging", False)),
            "row_transition_timeout": bool(v2.get("row_transition_timeout", False)),
            "row_transition_geometry_warning": v2.get("row_transition_geometry_warning", ""),
            "target_side_yaw_deg": v2.get("target_side_yaw_deg"),
            "target_next_row_yaw_deg": v2.get("target_next_row_yaw_deg"),
            "segment_yaw_error_deg": v2.get("segment_yaw_error_deg"),
            "segment_lane_error_m": v2.get("segment_lane_error_m"),
            "target_row_yaw_deg": v2.get("target_row_yaw_deg"),
            "yaw_capture_error_deg": v2.get("yaw_capture_error_deg"),
            "lane_capture_error_m": v2.get("lane_capture_error_m"),
            "turn_radius_m": v2.get("turn_radius_m"),
            "turn_side": v2.get("turn_side", ""),
            "transition_emitted_v_mps": v2.get("transition_emitted_v_mps"),
            "transition_emitted_omega_radps": v2.get("transition_emitted_omega_radps"),
            "transition_left_target_mps": v2.get("transition_left_target_mps"),
            "transition_right_target_mps": v2.get("transition_right_target_mps"),
        }
        self._writer.writerow(row)
        self._fp.flush()
        self.sample_count += 1

        if self._prev_pose is not None:
            self.distance_m += math.hypot(x - self._prev_pose[0], y - self._prev_pose[1])
        self._prev_pose = (x, y)
        self.final_lateral_error_m = cte
        self.max_abs_cross_track_error_m = max(self.max_abs_cross_track_error_m, abs(cte))
        self.sum_cross_track_error_sq += cte * cte
        self.cross_track_error_samples += 1
        self.max_abs_heading_error_deg = max(self.max_abs_heading_error_deg, abs(heading_error_deg))
        omega_sign = self._omega_sign(final_omega)
        if omega_sign and self._prev_omega_sign and omega_sign != self._prev_omega_sign:
            self.omega_sign_changes += 1
        if omega_sign:
            self._prev_omega_sign = omega_sign
        if bool(cl.get("closed_loop_diverging", False)):
            self.divergence_count += 1
        if bool(status.get("physical_stall_detected", False)):
            self.physical_stall_count += 1

    def summary(self) -> Dict[str, Any]:
        rms = 0.0
        if self.cross_track_error_samples > 0:
            rms = math.sqrt(self.sum_cross_track_error_sq / float(self.cross_track_error_samples))
        sign_changes_per_m = self.omega_sign_changes / max(self.distance_m, 1e-6)
        return {
            "log_file": self.path,
            "samples": self.sample_count,
            "distance_traveled_m": round(self.distance_m, 3),
            "final_lateral_error_m": round(self.final_lateral_error_m, 4),
            "max_abs_cross_track_error_m": round(self.max_abs_cross_track_error_m, 4),
            "rms_cross_track_error_m": round(rms, 4),
            "max_abs_heading_error_deg": round(self.max_abs_heading_error_deg, 3),
            "sign_changes_per_meter": round(sign_changes_per_m, 3),
            "divergence_count": int(self.divergence_count),
            "physical_stall_count": int(self.physical_stall_count),
        }

    def close(self) -> None:
        self._fp.close()


__all__ = ["SweepMetricsLogger"]
