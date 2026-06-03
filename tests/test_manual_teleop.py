import json
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ugv_manual_teleop import (  # noqa: E402
    TeleopConfig,
    TeleopState,
    apply_key_action,
    arc_omega_for_speed,
    build_stop_payload,
    config_from_args,
    normalized_config,
    parse_args,
    payload_for_state,
    payload_json,
    set_motion_key_pressed,
    set_motion_key_released,
    should_publish,
    velocity_for_key,
    velocity_for_pressed_keys,
)


def test_wasd_velocity_mapping_uses_tank_drive_sign_convention():
    config = TeleopConfig(forward_mps=0.15, reverse_mps=0.1, arc_turn_radps=0.35, pivot_turn_radps=0.75)
    assert velocity_for_key("w", config)[:2] == pytest.approx((0.15, 0.0))
    assert velocity_for_key("w", config)[2] == "manual_forward"
    assert velocity_for_key("s", config)[:2] == pytest.approx((-0.1, 0.0))
    assert velocity_for_key("s", config)[2] == "manual_reverse"
    assert velocity_for_pressed_keys(["w", "a"], config)[:2] == pytest.approx((0.15, 0.2))
    assert velocity_for_pressed_keys(["w", "a"], config)[2] == "manual_arc_left"
    assert velocity_for_pressed_keys(["w", "d"], config)[:2] == pytest.approx((0.15, -0.2))
    assert velocity_for_pressed_keys(["w", "d"], config)[2] == "manual_arc_right"
    assert velocity_for_pressed_keys(["s", "a"], config)[:2] == pytest.approx((-0.1, 0.1 / 0.75))
    assert velocity_for_pressed_keys(["s", "a"], config)[2] == "manual_reverse_arc_left"
    assert velocity_for_key("a", config)[:2] == pytest.approx((0.0, 0.75))
    assert velocity_for_key("a", config)[2] == "manual_pivot_left"
    assert velocity_for_key("d", config)[:2] == pytest.approx((0.0, -0.75))
    assert velocity_for_key("d", config)[2] == "manual_pivot_right"
    assert velocity_for_key("z", config) is None


def test_default_turn_rate_is_set_for_loaded_vehicle_pivot():
    config = config_from_args(parse_args([]))
    assert config.forward_mps == pytest.approx(0.15)
    assert config.reverse_mps == pytest.approx(0.10)
    assert config.arc_turn_radps == pytest.approx(0.45)
    assert config.pivot_turn_radps == pytest.approx(0.75)
    assert config.turn_radps == pytest.approx(0.45)
    assert config.max_arc_turn_radps == pytest.approx(0.90)
    assert config.max_pivot_turn_radps == pytest.approx(1.20)
    assert config.turn_step_radps == pytest.approx(0.10)


def test_payload_for_state_keeps_velocity_while_key_is_fresh():
    config = TeleopConfig(forward_mps=0.2, deadman_timeout_s=0.25)
    state = TeleopState(active_key="w", last_motion_time_s=10.0)
    payload = payload_for_state(state, config, 10.1)
    assert payload["command_type"] == "velocity"
    assert payload["v_mps"] == pytest.approx(0.2)
    assert payload["omega_radps"] == pytest.approx(0.0)
    assert payload["reason"] == "manual_forward"


def test_payload_for_state_stops_after_deadman_timeout():
    config = TeleopConfig(deadman_timeout_s=0.25)
    state = TeleopState(active_key="w", last_motion_time_s=10.0)
    payload = payload_for_state(state, config, 10.4)
    assert payload["command_type"] == "stop"
    assert payload["reason"] == "manual_deadman_stop"
    assert state.active_key is None
    assert state.last_motion_time_s is None


def test_release_event_backend_does_not_use_short_terminal_deadman():
    config = TeleopConfig(forward_mps=0.2, deadman_timeout_s=0.25, key_state_stale_timeout_s=2.0)
    state = TeleopState()
    set_motion_key_pressed(state, "w", 10.0, release_events_supported=True)
    payload = payload_for_state(state, config, 10.6)
    assert payload["command_type"] == "velocity"
    assert payload["v_mps"] == pytest.approx(0.2)


def test_release_event_backend_stops_on_key_release():
    state = TeleopState()
    set_motion_key_pressed(state, "w", 10.0, release_events_supported=True)
    set_motion_key_released(state, "w", 10.1)
    payload = payload_for_state(state, TeleopConfig(), 10.1)
    assert payload["command_type"] == "stop"
    assert state.active_key is None


def test_release_event_backend_uses_most_recent_pressed_motion_key():
    state = TeleopState()
    set_motion_key_pressed(state, "w", 10.0, release_events_supported=True)
    set_motion_key_pressed(state, "a", 10.1, release_events_supported=True)
    assert state.active_key == "a"
    set_motion_key_released(state, "a", 10.2)
    assert state.active_key == "w"


def test_payload_for_state_combines_held_forward_and_turn_keys_into_arc():
    config = TeleopConfig(forward_mps=0.2, arc_turn_radps=0.4)
    state = TeleopState()
    set_motion_key_pressed(state, "w", 10.0, release_events_supported=True)
    set_motion_key_pressed(state, "a", 10.1, release_events_supported=True)
    payload = payload_for_state(state, config, 10.2)
    assert payload["command_type"] == "velocity"
    assert payload["v_mps"] == pytest.approx(0.2)
    assert payload["omega_radps"] == pytest.approx(0.2 / 0.75)
    assert payload["reason"] == "manual_arc_left"


def test_manual_arc_omega_respects_min_turn_radius():
    config = normalized_config(TeleopConfig(forward_mps=0.15, arc_turn_radps=4.0, max_arc_turn_radps=4.0))
    assert arc_omega_for_speed(0.15, 4.0, config) == pytest.approx(0.2)
    assert arc_omega_for_speed(0.15, -4.0, config) == pytest.approx(-0.2)


def test_release_event_backend_stale_timeout_is_still_a_safety_stop():
    config = TeleopConfig(key_state_stale_timeout_s=0.5)
    state = TeleopState()
    set_motion_key_pressed(state, "w", 10.0, release_events_supported=True)
    payload = payload_for_state(state, config, 10.8)
    assert payload["command_type"] == "stop"
    assert payload["reason"] == "manual_key_state_stale_stop"


def test_stop_payload_and_json_contract_are_clean_velocity_interface():
    payload = build_stop_payload(reason="unit_stop", sequence=7, config=TeleopConfig())
    text = payload_json(payload)
    decoded = json.loads(text)
    assert decoded["command_type"] == "stop"
    assert decoded["v_mps"] == pytest.approx(0.0)
    assert decoded["omega_radps"] == pytest.approx(0.0)
    assert "raw_pwm" not in decoded
    assert "raw_left" not in decoded
    assert "raw_right" not in decoded


def test_should_publish_can_suppress_unchanged_payloads():
    previous = {"command_type": "stop", "v_mps": 0.0, "omega_radps": 0.0, "reason": "same"}
    current = dict(previous)
    assert should_publish(previous, current, always_publish=False) is False
    assert should_publish(previous, current, always_publish=True) is True
    changed = dict(current, reason="new")
    assert should_publish(previous, changed, always_publish=False) is True


def test_normalized_config_clamps_publish_rate_and_positive_speeds():
    config = normalized_config(
        TeleopConfig(
            forward_mps=-0.2,
            reverse_mps=-0.1,
            arc_turn_radps=-0.3,
            pivot_turn_radps=-0.8,
            max_forward_mps=0.18,
            max_reverse_mps=0.08,
            max_arc_turn_radps=0.25,
            max_pivot_turn_radps=0.6,
            publish_hz=1000.0,
            deadman_timeout_s=0.001,
        )
    )
    assert config.forward_mps == pytest.approx(0.18)
    assert config.reverse_mps == pytest.approx(0.08)
    assert config.arc_turn_radps == pytest.approx(0.25)
    assert config.pivot_turn_radps == pytest.approx(0.6)
    assert config.publish_hz == pytest.approx(50.0)
    assert config.deadman_timeout_s == pytest.approx(0.05)


def test_speed_and_turn_adjustment_keys_update_config():
    config = TeleopConfig(
        forward_mps=0.15,
        reverse_mps=0.1,
        arc_turn_radps=0.35,
        pivot_turn_radps=0.75,
        max_forward_mps=0.17,
        max_reverse_mps=0.11,
        max_arc_turn_radps=0.38,
        max_pivot_turn_radps=0.9,
        speed_step_mps=0.02,
        turn_step_radps=0.05,
    )
    speed_up = apply_key_action("+", config)
    assert speed_up.action_type == "config"
    assert speed_up.config.forward_mps == pytest.approx(0.17)
    assert speed_up.config.reverse_mps == pytest.approx(0.11)
    speed_down = apply_key_action("-", speed_up.config)
    assert speed_down.config.forward_mps == pytest.approx(0.15)
    turn_up = apply_key_action("]", config)
    assert turn_up.config.arc_turn_radps == pytest.approx(0.38)
    turn_down = apply_key_action("[", turn_up.config)
    assert turn_down.config.arc_turn_radps == pytest.approx(0.33)

    pivot_debug = apply_key_action("]", normalized_config(TeleopConfig(adjust_pivot_turn=True, pivot_turn_radps=0.75)))
    assert pivot_debug.config.pivot_turn_radps == pytest.approx(0.85)


def test_parse_args_builds_config_without_importing_ros():
    args = parse_args(
        [
            "--forward-mps",
            "0.22",
            "--reverse-mps",
            "0.12",
            "--turn-radps",
            "0.4",
            "--pivot-turn-radps",
            "0.8",
            "--max-forward-mps",
            "0.25",
            "--max-reverse-mps",
            "0.14",
            "--max-turn-radps",
            "0.5",
            "--max-pivot-turn-radps",
            "1.0",
            "--publish-hz",
            "25",
            "--input-backend",
            "terminal",
        ]
    )
    config = config_from_args(args)
    assert config.forward_mps == pytest.approx(0.22)
    assert config.reverse_mps == pytest.approx(0.12)
    assert config.arc_turn_radps == pytest.approx(0.4)
    assert config.pivot_turn_radps == pytest.approx(0.8)
    assert config.max_forward_mps == pytest.approx(0.25)
    assert config.max_reverse_mps == pytest.approx(0.14)
    assert config.max_arc_turn_radps == pytest.approx(0.5)
    assert config.max_pivot_turn_radps == pytest.approx(1.0)
    assert config.publish_hz == pytest.approx(25.0)
    assert args.input_backend == "terminal"
    assert args.publish_on_change_only is False


def test_publish_on_change_only_flag_is_explicit_debug_mode():
    args = parse_args(["--publish-on-change-only"])
    assert args.publish_on_change_only is True


def test_command_topic_sharing_requires_explicit_debug_flag():
    args = parse_args([])
    assert args.allow_command_topic_sharing is False
    debug_args = parse_args(["--allow-command-topic-sharing"])
    assert debug_args.allow_command_topic_sharing is True
