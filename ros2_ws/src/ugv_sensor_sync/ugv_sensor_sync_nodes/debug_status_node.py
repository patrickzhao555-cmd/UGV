#!/usr/bin/env python3
import json
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DebugStatusNode(Node):
    def __init__(self):
        super().__init__('ugv_debug_status_node')

        self.declare_parameter('publish_topic', '/ugv/debug_status')
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('zed_status_topic', '/zed/status')
        self.declare_parameter('fusion_summary_topic', '/sensors/synced_summary')
        self.declare_parameter('motor_status_topic', '/motor_controller/status')
        self.declare_parameter('nav_status_topic', '/ugv_nav_status')

        publish_topic = self.get_parameter('publish_topic').value
        self.publish_period_s = max(0.2, float(self.get_parameter('publish_period_s').value))
        self.latest: Dict[str, Tuple[dict, float]] = {}

        self.pub = self.create_publisher(String, publish_topic, 10)
        self._subscribe_json('zed', self.get_parameter('zed_status_topic').value)
        self._subscribe_json('fusion', self.get_parameter('fusion_summary_topic').value)
        self._subscribe_json('motor', self.get_parameter('motor_status_topic').value)
        self._subscribe_json('nav', self.get_parameter('nav_status_topic').value)
        self.create_timer(self.publish_period_s, self.publish_debug)
        self.get_logger().info(f'UGV debug status node publishing {publish_topic}')

    def _subscribe_json(self, key: str, topic: str) -> None:
        def callback(msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                data = {'raw': msg.data}
            self.latest[key] = (data, self._now_s())

        self.create_subscription(String, topic, callback, 10)

    def publish_debug(self) -> None:
        now_s = self._now_s()
        status = {}
        for key, (data, stamp_s) in self.latest.items():
            status[key] = {
                'age_s': round(max(0.0, now_s - stamp_s), 3),
                'data': data,
            }
        self.pub.publish(String(data=json.dumps(status)))
        self._log_one_line(status)

    def _log_one_line(self, status: dict) -> None:
        zed = self._data(status, 'zed')
        fusion = self._data(status, 'fusion')
        motor = self._data(status, 'motor')
        nav = self._data(status, 'nav')
        cmd = nav.get('last_command', {}) if isinstance(nav.get('last_command'), dict) else {}
        if not cmd:
            cmd = nav.get('cmd', {}) if isinstance(nav.get('cmd'), dict) else {}

        self.get_logger().info(
            'debug '
            f"zed_valid={zed.get('valid_depth_samples')} "
            f"zed_imu_rate={zed.get('imu_rate_hz')} "
            f"zed_imu_age={zed.get('imu_age_s')} "
            f"zed_imu_failures={zed.get('imu_publish_failures')} "
            f"zed_error={zed.get('last_imu_error')} "
            f"nav_imu_rate={nav.get('imu_rate_hz')} "
            f"nav_imu_age={nav.get('imu_age_s')} "
            f"depth_p10={zed.get('depth_p10_m')} "
            f"front_lidar={fusion.get('front_lidar_range_m')} "
            f"depth_roi={fusion.get('min_depth_range_m')} "
            f"front_clearance={fusion.get('front_clearance_m')} "
            f"clear_src={fusion.get('front_clearance_source')} "
            f"depth_pts={fusion.get('depth_obstacle_points')} "
            f"depth_filtered={fusion.get('depth_obstacle_points_filtered')} "
            f"depth_comps={fusion.get('depth_obstacle_components')} "
            f"semantic_pts={fusion.get('semantic_obstacle_points')} "
            f"encoder={fusion.get('encoder_available')} "
            f"motor_connected={motor.get('connected')} "
            f"motor_mode={motor.get('control_mode')} "
            f"params_synced={motor.get('teensy_pid_params_synced')} "
            f"fault={motor.get('fault_reason') or motor.get('fault')} "
            f"cmd={cmd.get('command_type') or cmd.get('mode')} "
            f"mission={nav.get('mission_state')} "
            f"seg={nav.get('segment_index')}:{nav.get('segment_type')} "
            f"pivot={nav.get('pivot_state')} "
            f"v={nav.get('v_mps')} "
            f"omega={nav.get('omega_radps')} "
            f"heading_err={nav.get('heading_error_rad')} "
            f"heading_src={nav.get('heading_source')} "
            f"yaw_rate={nav.get('yaw_rate_radps')} "
            f"gyro_bias={nav.get('gyro_bias_radps')} "
            f"safety={nav.get('safety_level')}:{nav.get('safety_reason')} "
            f"stuck={nav.get('stuck_detected')}:{nav.get('stuck_reason')} "
            f"c2_state={nav.get('challenge2_state')} "
            f"c2_pose={nav.get('challenge2_pose_m')} "
            f"c2_field_pose={nav.get('challenge2_field_pose_m')} "
            f"c2_target={nav.get('challenge2_target_m')} "
            f"c2_field_target={nav.get('challenge2_target_field_m')} "
            f"c2_dist={nav.get('challenge2_target_distance_m')} "
            f"c2_bearing={nav.get('challenge2_target_bearing_rad')} "
            f"c2_align_err={nav.get('challenge2_align_error_rad')} "
            f"c2_xtrack={nav.get('challenge2_cross_track_error_m')} "
            f"c2_arc={nav.get('challenge2_arc_align_active')} "
            f"c2_arc_cfg=({nav.get('challenge2_arc_align_speed_mps')},{nav.get('challenge2_arc_align_max_omega_radps')}) "
            f"c2_arc_settle_yaw={nav.get('challenge2_arc_align_settle_yaw_rate_radps')} "
            f"c2_reason={nav.get('challenge2_result_reason')} "
            f"c2_pivot_retry={nav.get('challenge2_pivot_recovery_count')} "
            f"c2_pivot_min_w={nav.get('challenge2_pivot_min_omega_radps')} "
            f"c2_pivot_break_w={nav.get('challenge2_pivot_breakaway_omega_radps')} "
            f"c3_state={nav.get('challenge3_state')} "
            f"c3_pose={nav.get('challenge3_pose_m')} "
            f"c3_target={nav.get('challenge3_target_m')} "
            f"c3_uv={nav.get('challenge3_route_uv_m')} "
            f"c3_dist={nav.get('challenge3_target_distance_m')} "
            f"c3_lane={nav.get('challenge3_lane_offset_m')}->{nav.get('challenge3_desired_lane_offset_m')} "
            f"c3_obs={nav.get('challenge3_scan_observation_count')}/{nav.get('challenge3_obstacle_memory_count')} "
            f"c3_reason={nav.get('challenge3_result_reason')} "
            f"enc_gyro_disagree={nav.get('encoder_gyro_disagreement_rad')} "
            f"target_side_mps=({motor.get('target_left_mps')},{motor.get('target_right_mps')}) "
            f"target_tps=({motor.get('left_target_tps')},{motor.get('right_target_tps')}) "
            f"measured_tps=({motor.get('left_measured_tps')},{motor.get('right_measured_tps')}) "
            f"pwm=({motor.get('left_pwm')},{motor.get('right_pwm')}) "
        )

    @staticmethod
    def _data(status: dict, key: str) -> dict:
        item: Optional[dict] = status.get(key)
        if not item:
            return {}
        data = item.get('data')
        return data if isinstance(data, dict) else {}

    @staticmethod
    def _sector_text(sectors: dict) -> str:
        if not isinstance(sectors, dict) or not sectors:
            return 'None'
        return (
            f"f={sectors.get('front')},"
            f"fl={sectors.get('front_left')},"
            f"fr={sectors.get('front_right')},"
            f"l={sectors.get('left')},"
            f"r={sectors.get('right')},"
            f"rear={sectors.get('rear')}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = DebugStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
