#!/usr/bin/env python3
"""Keyboard manual control for the UGV velocity-command interface.

This tool publishes the same clean JSON contract used by the autonomous
chassis controller:

    {"command_type": "velocity", "v_mps": <float>, "omega_radps": <float>}

It intentionally does not talk to the Teensy directly and never publishes raw
PWM. The motor bridge remains responsible for converting chassis velocity into
left/right side velocity commands.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import select
import struct
import sys
import time
from dataclasses import dataclass, field, replace
from typing import Any, Dict, Iterable, List, Optional, Tuple


DEFAULT_COMMAND_TOPIC = "/ugv_nav_cmd"
DEFAULT_FORWARD_MPS = 0.15
DEFAULT_REVERSE_MPS = 0.10
DEFAULT_ARC_TURN_RADPS = 0.45
DEFAULT_PIVOT_TURN_RADPS = 0.75
DEFAULT_MAX_FORWARD_MPS = 0.30
DEFAULT_MAX_REVERSE_MPS = 0.15
DEFAULT_MAX_ARC_TURN_RADPS = 0.90
DEFAULT_MAX_PIVOT_TURN_RADPS = 1.20
DEFAULT_PUBLISH_HZ = 20.0
DEFAULT_DEADMAN_TIMEOUT_S = 0.75
DEFAULT_KEY_STATE_STALE_TIMEOUT_S = 2.0
DEFAULT_SPEED_STEP_MPS = 0.02
DEFAULT_TURN_STEP_RADPS = 0.10
DEFAULT_ARC_MIN_TURN_RADIUS_M = 0.75
DEFAULT_ARC_INNER_MIN_MPS = 0.08
DEFAULT_TERMINAL_SINGLE_KEY_ARCS = False
DEFAULT_TRACK_WIDTH_M = 0.416
MIN_PUBLISH_HZ = 1.0
MAX_PUBLISH_HZ = 50.0
MIN_DEADMAN_TIMEOUT_S = 0.05
MIN_KEY_STATE_STALE_TIMEOUT_S = 0.25

MOTION_KEYS = {"w", "a", "s", "d"}
STOP_KEYS = {" ", "x"}
QUIT_KEYS = {"q", "\x03", "\x1b"}  # q, Ctrl-C, Esc
INPUT_BACKENDS = {"auto", "terminal", "evdev"}

EV_KEY = 0x01
EVDEV_KEY_CODES = {
    1: "\x1b",  # KEY_ESC
    12: "-",
    13: "=",
    16: "q",
    17: "w",
    26: "[",
    27: "]",
    30: "a",
    31: "s",
    32: "d",
    45: "x",
    57: " ",
}


@dataclass(frozen=True)
class TeleopConfig:
    command_topic: str = DEFAULT_COMMAND_TOPIC
    forward_mps: float = DEFAULT_FORWARD_MPS
    reverse_mps: float = DEFAULT_REVERSE_MPS
    arc_turn_radps: float = DEFAULT_ARC_TURN_RADPS
    pivot_turn_radps: float = DEFAULT_PIVOT_TURN_RADPS
    max_forward_mps: float = DEFAULT_MAX_FORWARD_MPS
    max_reverse_mps: float = DEFAULT_MAX_REVERSE_MPS
    max_arc_turn_radps: float = DEFAULT_MAX_ARC_TURN_RADPS
    max_pivot_turn_radps: float = DEFAULT_MAX_PIVOT_TURN_RADPS
    publish_hz: float = DEFAULT_PUBLISH_HZ
    deadman_timeout_s: float = DEFAULT_DEADMAN_TIMEOUT_S
    key_state_stale_timeout_s: float = DEFAULT_KEY_STATE_STALE_TIMEOUT_S
    speed_step_mps: float = DEFAULT_SPEED_STEP_MPS
    turn_step_radps: float = DEFAULT_TURN_STEP_RADPS
    arc_min_turn_radius_m: float = DEFAULT_ARC_MIN_TURN_RADIUS_M
    arc_inner_min_mps: float = DEFAULT_ARC_INNER_MIN_MPS
    track_width_m: float = DEFAULT_TRACK_WIDTH_M
    terminal_single_key_arcs: bool = DEFAULT_TERMINAL_SINGLE_KEY_ARCS
    allow_arc_side_reverse: bool = False
    allow_pivot_keys: bool = True
    adjust_pivot_turn: bool = False
    # Backwards-compatible CLI/test aliases.  If provided, they configure arc turns.
    turn_radps: Optional[float] = None
    max_turn_radps: Optional[float] = None
    controller: str = "ugv_manual_teleop"


@dataclass(frozen=True)
class KeyAction:
    action_type: str
    description: str
    config: TeleopConfig


@dataclass
class TeleopState:
    active_key: Optional[str] = None
    last_motion_time_s: Optional[float] = None
    pressed_motion_keys: List[str] = field(default_factory=list)
    release_events_supported: bool = False
    sequence: int = 0
    last_payload: Optional[Dict[str, Any]] = None


@dataclass(frozen=True)
class KeyEvent:
    key: str
    event_type: str = "press"  # press, release, repeat

    @property
    def is_release(self) -> bool:
        return self.event_type == "release"

    @property
    def is_press_like(self) -> bool:
        return self.event_type in {"press", "repeat"}


def finite_float(value: Any, *, name: str) -> float:
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite")
    return out


def clamp(value: float, low: float, high: float) -> float:
    return min(max(float(value), float(low)), float(high))


def normalized_config(config: TeleopConfig) -> TeleopConfig:
    max_forward = abs(finite_float(config.max_forward_mps, name="max_forward_mps"))
    max_reverse = abs(finite_float(config.max_reverse_mps, name="max_reverse_mps"))
    raw_max_arc = config.max_turn_radps if config.max_turn_radps is not None else config.max_arc_turn_radps
    max_arc_turn = abs(finite_float(raw_max_arc, name="max_arc_turn_radps"))
    max_pivot_turn = abs(finite_float(config.max_pivot_turn_radps, name="max_pivot_turn_radps"))
    raw_arc_turn = config.turn_radps if config.turn_radps is not None else config.arc_turn_radps
    forward = min(abs(finite_float(config.forward_mps, name="forward_mps")), max_forward)
    reverse = min(abs(finite_float(config.reverse_mps, name="reverse_mps")), max_reverse)
    arc_turn = min(abs(finite_float(raw_arc_turn, name="arc_turn_radps")), max_arc_turn)
    pivot_turn = min(abs(finite_float(config.pivot_turn_radps, name="pivot_turn_radps")), max_pivot_turn)
    publish_hz = clamp(finite_float(config.publish_hz, name="publish_hz"), MIN_PUBLISH_HZ, MAX_PUBLISH_HZ)
    timeout = max(MIN_DEADMAN_TIMEOUT_S, finite_float(config.deadman_timeout_s, name="deadman_timeout_s"))
    stale_timeout = max(
        MIN_KEY_STATE_STALE_TIMEOUT_S,
        finite_float(config.key_state_stale_timeout_s, name="key_state_stale_timeout_s"),
    )
    speed_step = max(0.0, finite_float(config.speed_step_mps, name="speed_step_mps"))
    turn_step = max(0.0, finite_float(config.turn_step_radps, name="turn_step_radps"))
    arc_radius = max(1e-6, finite_float(config.arc_min_turn_radius_m, name="arc_min_turn_radius_m"))
    arc_inner_min = max(0.0, finite_float(config.arc_inner_min_mps, name="arc_inner_min_mps"))
    track_width = max(1e-6, finite_float(config.track_width_m, name="track_width_m"))
    return replace(
        config,
        forward_mps=forward,
        reverse_mps=reverse,
        arc_turn_radps=arc_turn,
        pivot_turn_radps=pivot_turn,
        max_forward_mps=max_forward,
        max_reverse_mps=max_reverse,
        max_arc_turn_radps=max_arc_turn,
        max_pivot_turn_radps=max_pivot_turn,
        publish_hz=publish_hz,
        deadman_timeout_s=timeout,
        key_state_stale_timeout_s=stale_timeout,
        speed_step_mps=speed_step,
        turn_step_radps=turn_step,
        arc_min_turn_radius_m=arc_radius,
        arc_inner_min_mps=arc_inner_min,
        track_width_m=track_width,
        turn_radps=arc_turn,
        max_turn_radps=max_arc_turn,
    )


def velocity_for_key(key: str, config: TeleopConfig) -> Optional[Tuple[float, float, str]]:
    key = key.lower()
    if key == "w":
        return config.forward_mps, 0.0, "manual_forward"
    if key == "s":
        return -config.reverse_mps, 0.0, "manual_reverse"
    if key == "a":
        if not config.allow_pivot_keys:
            return None
        return 0.0, config.pivot_turn_radps, "manual_pivot_left"
    if key == "d":
        if not config.allow_pivot_keys:
            return None
        return 0.0, -config.pivot_turn_radps, "manual_pivot_right"
    return None


def _last_matching(keys: Iterable[str], candidates: set[str]) -> Optional[str]:
    selected: Optional[str] = None
    for key in keys:
        lower = str(key).lower()
        if lower in candidates:
            selected = lower
    return selected


def arc_omega_for_speed(v_mps: float, requested_omega_radps: float, config: TeleopConfig) -> float:
    speed = abs(float(v_mps))
    if speed <= 1e-9:
        return 0.0
    radius_limit = speed / max(1e-6, float(config.arc_min_turn_radius_m))
    side_reverse_limit = float("inf")
    if not config.allow_arc_side_reverse:
        inner_floor = min(speed, max(0.0, float(config.arc_inner_min_mps)))
        side_reverse_limit = max(0.0, (speed - inner_floor) / (0.5 * max(1e-6, float(config.track_width_m))))
    omega_abs = min(
        abs(float(requested_omega_radps)),
        abs(float(config.max_arc_turn_radps)),
        radius_limit,
        side_reverse_limit,
    )
    return math.copysign(omega_abs, float(requested_omega_radps))


def velocity_for_pressed_keys(
    keys: Iterable[str],
    config: TeleopConfig,
    *,
    terminal_single_key_arcs: bool = False,
) -> Optional[Tuple[float, float, str]]:
    ordered = [str(key).lower() for key in keys if str(key).lower() in MOTION_KEYS]
    if not ordered:
        return None
    linear_key = _last_matching(ordered, {"w", "s"})
    turn_key = _last_matching(ordered, {"a", "d"})
    if linear_key is not None:
        v_mps = config.forward_mps if linear_key == "w" else -config.reverse_mps
        if turn_key is None:
            return (v_mps, 0.0, "manual_forward" if linear_key == "w" else "manual_reverse")
        requested_omega = config.arc_turn_radps if turn_key == "a" else -config.arc_turn_radps
        omega = arc_omega_for_speed(v_mps, requested_omega, config)
        if linear_key == "w":
            reason = "manual_arc_left" if turn_key == "a" else "manual_arc_right"
        else:
            reason = "manual_reverse_arc_left" if turn_key == "a" else "manual_reverse_arc_right"
        return v_mps, omega, reason
    if turn_key is not None and terminal_single_key_arcs:
        v_mps = config.forward_mps
        requested_omega = config.arc_turn_radps if turn_key == "a" else -config.arc_turn_radps
        omega = arc_omega_for_speed(v_mps, requested_omega, config)
        reason = "manual_terminal_arc_left" if turn_key == "a" else "manual_terminal_arc_right"
        return v_mps, omega, reason
    if turn_key is not None and config.allow_pivot_keys:
        return velocity_for_key(turn_key, config)
    return None


def build_velocity_payload(
    v_mps: float,
    omega_radps: float,
    *,
    reason: str,
    sequence: int,
    config: TeleopConfig,
) -> Dict[str, Any]:
    half_track = 0.5 * max(1e-6, float(config.track_width_m))
    left_mps = float(v_mps) - float(omega_radps) * half_track
    right_mps = float(v_mps) + float(omega_radps) * half_track
    turn_radius_m = None
    if abs(float(v_mps)) > 1e-9 and abs(float(omega_radps)) > 1e-9:
        turn_radius_m = abs(float(v_mps) / float(omega_radps))
    side_reverse = (left_mps < 0.0 < right_mps) or (right_mps < 0.0 < left_mps)
    return {
        "command_type": "velocity",
        "controller": config.controller,
        "mode": "VELOCITY",
        "v_mps": float(v_mps),
        "omega_radps": float(omega_radps),
        "target_left_mps": left_mps,
        "target_right_mps": right_mps,
        "turn_radius_m": turn_radius_m,
        "track_width_m": float(config.track_width_m),
        "arc_side_reverse": bool(side_reverse),
        "reason": reason,
        "manual_sequence": int(sequence),
    }


def build_stop_payload(*, reason: str, sequence: int, config: TeleopConfig) -> Dict[str, Any]:
    return {
        "command_type": "stop",
        "controller": config.controller,
        "mode": "STOP",
        "v_mps": 0.0,
        "omega_radps": 0.0,
        "reason": reason,
        "manual_sequence": int(sequence),
    }


def payload_for_state(state: TeleopState, config: TeleopConfig, now_s: float) -> Dict[str, Any]:
    state.sequence += 1
    if state.active_key and state.last_motion_time_s is not None:
        age = max(0.0, float(now_s) - float(state.last_motion_time_s))
        keys = state.pressed_motion_keys if state.pressed_motion_keys else [state.active_key]
        velocity = velocity_for_pressed_keys(
            keys,
            config,
            terminal_single_key_arcs=(not state.release_events_supported and bool(config.terminal_single_key_arcs)),
        )
        if velocity is not None and state.release_events_supported and age <= config.key_state_stale_timeout_s:
            v_mps, omega_radps, reason = velocity
            return build_velocity_payload(
                v_mps,
                omega_radps,
                reason=reason,
                sequence=state.sequence,
                config=config,
            )
        if velocity is not None and not state.release_events_supported and age <= config.deadman_timeout_s:
            v_mps, omega_radps, reason = velocity
            return build_velocity_payload(
                v_mps,
                omega_radps,
                reason=reason,
                sequence=state.sequence,
                config=config,
            )
    reason = "manual_key_state_stale_stop" if state.release_events_supported else "manual_deadman_stop"
    clear_motion_state(state)
    return build_stop_payload(reason=reason, sequence=state.sequence, config=config)


def payload_json(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True)


def should_publish(previous: Optional[Dict[str, Any]], current: Dict[str, Any], *, always_publish: bool) -> bool:
    if always_publish:
        return True
    if previous is None:
        return True
    comparable_keys = ("command_type", "v_mps", "omega_radps", "reason")
    return any(previous.get(key) != current.get(key) for key in comparable_keys)


def apply_key_action(key: str, config: TeleopConfig) -> KeyAction:
    lower = key.lower()
    if lower == "+" or lower == "=":
        updated = replace(
            config,
            forward_mps=min(config.max_forward_mps, config.forward_mps + config.speed_step_mps),
            reverse_mps=min(config.max_reverse_mps, config.reverse_mps + config.speed_step_mps),
        )
        return KeyAction("config", "speed_up", normalized_config(updated))
    if lower == "-":
        updated = replace(
            config,
            forward_mps=max(0.0, config.forward_mps - config.speed_step_mps),
            reverse_mps=max(0.0, config.reverse_mps - config.speed_step_mps),
        )
        return KeyAction("config", "speed_down", normalized_config(updated))
    if lower == "]":
        if config.adjust_pivot_turn:
            updated = replace(
                config,
                pivot_turn_radps=min(config.max_pivot_turn_radps, config.pivot_turn_radps + config.turn_step_radps),
            )
            return KeyAction("config", "pivot_turn_up", normalized_config(updated))
        updated = replace(
            config,
            arc_turn_radps=min(config.max_arc_turn_radps, config.arc_turn_radps + config.turn_step_radps),
            turn_radps=None,
        )
        return KeyAction("config", "arc_turn_up", normalized_config(updated))
    if lower == "[":
        if config.adjust_pivot_turn:
            updated = replace(config, pivot_turn_radps=max(0.0, config.pivot_turn_radps - config.turn_step_radps))
            return KeyAction("config", "pivot_turn_down", normalized_config(updated))
        updated = replace(config, arc_turn_radps=max(0.0, config.arc_turn_radps - config.turn_step_radps), turn_radps=None)
        return KeyAction("config", "arc_turn_down", normalized_config(updated))
    if lower in QUIT_KEYS:
        return KeyAction("quit", "quit", config)
    if lower in STOP_KEYS:
        return KeyAction("stop", "manual_key_stop", config)
    if lower in MOTION_KEYS:
        return KeyAction("motion", lower, config)
    return KeyAction("ignore", lower, config)


def clear_motion_state(state: TeleopState) -> None:
    state.active_key = None
    state.last_motion_time_s = None
    state.pressed_motion_keys.clear()


def set_motion_key_pressed(state: TeleopState, key: str, now_s: float, *, release_events_supported: bool) -> None:
    state.release_events_supported = bool(release_events_supported)
    if not release_events_supported:
        # Terminal/SSH input reports key presses but not releases, so keeping a
        # held-key set would make old W/S/A/D keys stick and create fake combos.
        state.pressed_motion_keys.clear()
        state.pressed_motion_keys.append(key)
        state.active_key = key
        state.last_motion_time_s = float(now_s)
        return
    if key in state.pressed_motion_keys:
        state.pressed_motion_keys.remove(key)
    state.pressed_motion_keys.append(key)
    state.active_key = key
    state.last_motion_time_s = float(now_s)


def set_motion_key_released(state: TeleopState, key: str, now_s: float) -> None:
    if key in state.pressed_motion_keys:
        state.pressed_motion_keys.remove(key)
    if state.active_key == key:
        state.active_key = state.pressed_motion_keys[-1] if state.pressed_motion_keys else None
    if state.active_key is None:
        state.last_motion_time_s = None
    else:
        state.last_motion_time_s = float(now_s)


class KeyboardInput:
    """Cross-platform nonblocking keyboard reader with Linux evdev support."""

    def __init__(self, *, backend: str = "auto", devices: Optional[List[str]] = None) -> None:
        if backend not in INPUT_BACKENDS:
            raise ValueError(f"unsupported keyboard backend: {backend}")
        self.backend = backend
        self.devices = list(devices or [])
        self._impl: Any = None
        self.description = "uninitialized"
        self.release_events_supported = False

    def __enter__(self) -> "KeyboardInput":
        if self.backend == "auto" and sys.platform.startswith("linux"):
            try:
                self._impl = _LinuxEvdevKeyboardInput(self.devices)
                self._impl.__enter__()
            except OSError:
                if self.devices:
                    raise
                self._impl = _terminal_input_impl()
                self._impl.__enter__()
        else:
            self._impl = self._make_impl()
            self._impl.__enter__()
        self.description = getattr(self._impl, "description", type(self._impl).__name__)
        self.release_events_supported = bool(getattr(self._impl, "release_events_supported", False))
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        if self._impl is not None:
            self._impl.__exit__(exc_type, exc, tb)

    def _make_impl(self) -> Any:
        if self.backend == "evdev":
            return _LinuxEvdevKeyboardInput(self.devices)
        if self.backend == "terminal":
            return _terminal_input_impl()
        return _terminal_input_impl()

    def read_events(self) -> List[KeyEvent]:
        if self._impl is None:
            return []
        return self._impl.read_events()


def _terminal_input_impl() -> Any:
    return _WindowsKeyboardInput() if sys.platform.startswith("win") else _PosixKeyboardInput()


class _WindowsKeyboardInput:
    release_events_supported = False
    description = "terminal"

    def __enter__(self) -> "_WindowsKeyboardInput":
        import msvcrt  # type: ignore

        self._msvcrt = msvcrt
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        return None

    def read_events(self) -> List[KeyEvent]:
        events: List[KeyEvent] = []
        while self._msvcrt.kbhit():
            raw = self._msvcrt.getwch()
            if raw in ("\x00", "\xe0") and self._msvcrt.kbhit():
                self._msvcrt.getwch()
                continue
            events.append(KeyEvent(raw, "press"))
        return events


class _PosixKeyboardInput:
    release_events_supported = False
    description = "terminal"

    def __enter__(self) -> "_PosixKeyboardInput":
        import termios
        import tty

        self._termios = termios
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_settings)

    def read_events(self) -> List[KeyEvent]:
        events: List[KeyEvent] = []
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if not readable:
                break
            events.append(KeyEvent(sys.stdin.read(1), "press"))
        return events


class _LinuxEvdevKeyboardInput:
    release_events_supported = True

    def __init__(self, devices: Optional[List[str]] = None) -> None:
        self.device_paths = [str(path) for path in devices] if devices else sorted(glob.glob("/dev/input/event*"))
        self._fds: List[int] = []
        self._buffers: Dict[int, bytes] = {}
        self._event_struct = struct.Struct("@llHHI")
        self.description = "evdev"

    def __enter__(self) -> "_LinuxEvdevKeyboardInput":
        errors: List[str] = []
        for path in self.device_paths:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as exc:
                errors.append(f"{path}: {exc}")
                continue
            self._fds.append(fd)
            self._buffers[fd] = b""
        if not self._fds:
            detail = "; ".join(errors[:3]) if errors else "no /dev/input/event* devices found"
            raise OSError(f"could not open any evdev keyboard input: {detail}")
        names = [os.path.basename(path) for path in self.device_paths]
        self.description = f"evdev:{','.join(names[:4])}"
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        for fd in self._fds:
            try:
                os.close(fd)
            except OSError:
                pass
        self._fds.clear()
        self._buffers.clear()

    def read_events(self) -> List[KeyEvent]:
        events: List[KeyEvent] = []
        if not self._fds:
            return events
        readable, _, _ = select.select(self._fds, [], [], 0)
        for fd in readable:
            while True:
                try:
                    chunk = os.read(fd, self._event_struct.size * 32)
                except BlockingIOError:
                    break
                except OSError:
                    break
                if not chunk:
                    break
                buffer = self._buffers.get(fd, b"") + chunk
                while len(buffer) >= self._event_struct.size:
                    event_bytes = buffer[: self._event_struct.size]
                    buffer = buffer[self._event_struct.size :]
                    _, _, event_type, code, value = self._event_struct.unpack(event_bytes)
                    if event_type != EV_KEY or code not in EVDEV_KEY_CODES:
                        continue
                    if value == 0:
                        events.append(KeyEvent(EVDEV_KEY_CODES[code], "release"))
                    elif value == 1:
                        events.append(KeyEvent(EVDEV_KEY_CODES[code], "press"))
                    elif value == 2:
                        events.append(KeyEvent(EVDEV_KEY_CODES[code], "repeat"))
                self._buffers[fd] = buffer
        return events


def print_help(config: TeleopConfig) -> None:
    preview = velocity_for_pressed_keys(["a"], config, terminal_single_key_arcs=True)
    preview_line = ""
    if preview is not None:
        preview_v, preview_omega, _ = preview
        half_track = 0.5 * config.track_width_m
        left_mps = preview_v - preview_omega * half_track
        right_mps = preview_v + preview_omega * half_track
        radius_m = abs(preview_v / preview_omega) if abs(preview_omega) > 1e-9 else float("inf")
        preview_line = (
            "  Effective single-key arc preview: "
            f"omega={preview_omega:.3f} rad/s, R={radius_m:.2f}m, "
            f"left={left_mps:.3f} m/s, right={right_mps:.3f} m/s, "
            f"side_reverse={'allowed' if config.allow_arc_side_reverse else 'blocked'}\n"
        )
    terminal_line = (
        "  Terminal/SSH fallback: A / D alone sends forward rolling arc; combos require evdev\n"
        if config.terminal_single_key_arcs
        else "  Terminal/SSH fallback disabled: A / D alone uses pivot fallback; combos require evdev\n"
    )
    print(
        "\nUGV manual teleop controls\n"
        "  W: forward while held\n"
        "  S: reverse while held\n"
        "  W+A / W+D: forward rolling arc left/right\n"
        "  S+A / S+D: reverse rolling arc left/right\n"
        "  A / D: low-speed pivot fallback with real key-release keyboard input\n"
        f"{terminal_line}"
        "  Space or X: stop\n"
        "  +/-: adjust forward/reverse speed\n"
        "  [/]: adjust arc turn rate by default\n"
        "  Q or Esc: quit, sending STOP\n"
        f"\nCurrent: forward={config.forward_mps:.3f} m/s, reverse={config.reverse_mps:.3f} m/s, "
        f"arc={config.arc_turn_radps:.3f} rad/s, pivot={config.pivot_turn_radps:.3f} rad/s\n"
        f"Limits: forward<={config.max_forward_mps:.3f}, reverse<={config.max_reverse_mps:.3f}, "
        f"arc<={config.max_arc_turn_radps:.3f}, pivot<={config.max_pivot_turn_radps:.3f}; "
        f"arc_min_radius={config.arc_min_turn_radius_m:.2f}m; "
        f"arc_inner_min={config.arc_inner_min_mps:.3f}m/s; "
        f"track={config.track_width_m:.3f}m; "
        f"terminal deadman={config.deadman_timeout_s:.2f}s\n",
        preview_line,
        flush=True,
    )


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual WASD teleop publisher for /ugv_nav_cmd.")
    parser.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC)
    parser.add_argument("--forward-mps", type=float, default=DEFAULT_FORWARD_MPS)
    parser.add_argument("--reverse-mps", type=float, default=DEFAULT_REVERSE_MPS)
    parser.add_argument("--arc-turn-radps", "--turn-radps", dest="arc_turn_radps", type=float, default=DEFAULT_ARC_TURN_RADPS)
    parser.add_argument("--pivot-turn-radps", type=float, default=DEFAULT_PIVOT_TURN_RADPS)
    parser.add_argument("--max-forward-mps", type=float, default=DEFAULT_MAX_FORWARD_MPS)
    parser.add_argument("--max-reverse-mps", type=float, default=DEFAULT_MAX_REVERSE_MPS)
    parser.add_argument("--max-arc-turn-radps", "--max-turn-radps", dest="max_arc_turn_radps", type=float, default=DEFAULT_MAX_ARC_TURN_RADPS)
    parser.add_argument("--max-pivot-turn-radps", type=float, default=DEFAULT_MAX_PIVOT_TURN_RADPS)
    parser.add_argument("--publish-hz", type=float, default=DEFAULT_PUBLISH_HZ)
    parser.add_argument("--deadman-timeout-s", type=float, default=DEFAULT_DEADMAN_TIMEOUT_S)
    parser.add_argument("--key-state-stale-timeout-s", type=float, default=DEFAULT_KEY_STATE_STALE_TIMEOUT_S)
    parser.add_argument("--speed-step-mps", type=float, default=DEFAULT_SPEED_STEP_MPS)
    parser.add_argument("--turn-step-radps", type=float, default=DEFAULT_TURN_STEP_RADPS)
    parser.add_argument("--arc-min-turn-radius-m", type=float, default=DEFAULT_ARC_MIN_TURN_RADIUS_M)
    parser.add_argument(
        "--arc-inner-min-mps",
        type=float,
        default=DEFAULT_ARC_INNER_MIN_MPS,
        help=(
            "Minimum forward speed kept on the inside side of a rolling arc. "
            "This prevents heavy-load turns from degenerating into one stationary side."
        ),
    )
    parser.add_argument("--track-width-m", type=float, default=DEFAULT_TRACK_WIDTH_M)
    parser.add_argument(
        "--allow-arc-side-reverse",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Debug only: allow rolling-arc commands to reverse one side when the requested "
            "turn radius is smaller than half the track width. Disabled by default."
        ),
    )
    parser.add_argument(
        "--terminal-single-key-arcs",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_TERMINAL_SINGLE_KEY_ARCS,
        help="For terminal/SSH input without key-release events, make A/D alone command forward arcs. Disabled by default.",
    )
    parser.add_argument("--allow-pivot-keys", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--adjust-pivot-turn",
        action="store_true",
        help="Debug mode: make [/ ] adjust pivot fallback rate instead of arc rate.",
    )
    parser.add_argument(
        "--input-backend",
        choices=sorted(INPUT_BACKENDS),
        default="auto",
        help="Keyboard input backend. 'auto' tries Linux evdev first and falls back to terminal repeat.",
    )
    parser.add_argument(
        "--keyboard-device",
        action="append",
        default=[],
        help="Linux evdev device path, e.g. /dev/input/event3. Can be provided more than once.",
    )
    parser.add_argument(
        "--publish-on-change-only",
        action="store_true",
        help=(
            "Only publish when payload changes. Not recommended for real driving because "
            "the motor bridge command timeout expects repeated velocity commands."
        ),
    )
    parser.add_argument(
        "--allow-command-topic-sharing",
        action="store_true",
        help=(
            "Debug only: allow another node to publish on the same command topic. "
            "By default teleop exits if another /ugv_nav_cmd publisher is detected."
        ),
    )
    return parser.parse_args(list(argv) if argv is not None else None)


def config_from_args(args: argparse.Namespace) -> TeleopConfig:
    return normalized_config(
        TeleopConfig(
            command_topic=str(args.command_topic),
            forward_mps=float(args.forward_mps),
            reverse_mps=float(args.reverse_mps),
            arc_turn_radps=float(args.arc_turn_radps),
            pivot_turn_radps=float(args.pivot_turn_radps),
            max_forward_mps=float(args.max_forward_mps),
            max_reverse_mps=float(args.max_reverse_mps),
            max_arc_turn_radps=float(args.max_arc_turn_radps),
            max_pivot_turn_radps=float(args.max_pivot_turn_radps),
            publish_hz=float(args.publish_hz),
            deadman_timeout_s=float(args.deadman_timeout_s),
            key_state_stale_timeout_s=float(args.key_state_stale_timeout_s),
            speed_step_mps=float(args.speed_step_mps),
            turn_step_radps=float(args.turn_step_radps),
            arc_min_turn_radius_m=float(args.arc_min_turn_radius_m),
            arc_inner_min_mps=float(args.arc_inner_min_mps),
            track_width_m=float(args.track_width_m),
            terminal_single_key_arcs=bool(args.terminal_single_key_arcs),
            allow_arc_side_reverse=bool(args.allow_arc_side_reverse),
            allow_pivot_keys=bool(args.allow_pivot_keys),
            adjust_pivot_turn=bool(args.adjust_pivot_turn),
        )
    )


def run_teleop(args: argparse.Namespace) -> None:
    import rclpy  # type: ignore
    from std_msgs.msg import String  # type: ignore

    config = config_from_args(args)
    rclpy.init()
    node = rclpy.create_node("ugv_manual_teleop")
    publisher = node.create_publisher(String, config.command_topic, 10)
    state = TeleopState()
    last_peer_check_s = 0.0

    def publish(payload: Dict[str, Any], *, force: bool = False) -> None:
        if not rclpy.ok():
            return
        repeated_velocity = payload.get("command_type") == "velocity" and not bool(args.publish_on_change_only)
        if should_publish(state.last_payload, payload, always_publish=repeated_velocity or force):
            try:
                publisher.publish(String(data=payload_json(payload)))
            except Exception as exc:  # pragma: no cover - depends on ROS shutdown timing.
                print(f"\nManual teleop publish skipped during shutdown: {exc}", file=sys.stderr, flush=True)
                return
            state.last_payload = dict(payload)

    def command_topic_peers() -> List[str]:
        peers: List[str] = []
        for info in node.get_publishers_info_by_topic(config.command_topic):
            if info.node_name == node.get_name() and info.node_namespace == node.get_namespace():
                continue
            peers.append(f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/"))
        return peers

    try:
        print_help(config)
        publish(build_stop_payload(reason="manual_startup_stop", sequence=0, config=config), force=True)
        period_s = 1.0 / max(MIN_PUBLISH_HZ, config.publish_hz)
        with KeyboardInput(backend=str(args.input_backend), devices=list(args.keyboard_device or [])) as keyboard:
            state.release_events_supported = keyboard.release_events_supported
            print(f"Keyboard backend: {keyboard.description}", flush=True)
            while rclpy.ok():
                now_s = time.monotonic()
                if not bool(args.allow_command_topic_sharing) and now_s - last_peer_check_s >= 1.0:
                    last_peer_check_s = now_s
                    peers = command_topic_peers()
                    if peers:
                        print(
                            "\nAnother publisher is already using "
                            f"{config.command_topic}: {', '.join(sorted(peers))}\n"
                            "Manual teleop is exiting to avoid command fights. "
                            "Stop the nav controller or rerun with --allow-command-topic-sharing for debug only.",
                            flush=True,
                        )
                        publish(
                            build_stop_payload(
                                reason="manual_command_topic_conflict_stop",
                                sequence=state.sequence + 1,
                                config=config,
                            ),
                            force=True,
                        )
                        return
                for event in keyboard.read_events():
                    action = apply_key_action(event.key, config)
                    config = action.config
                    state.release_events_supported = keyboard.release_events_supported
                    if event.is_release:
                        if action.action_type == "motion":
                            set_motion_key_released(state, action.description, now_s)
                        continue
                    if action.action_type == "quit":
                        print("\nQuitting manual teleop; sending STOP.", flush=True)
                        publish(build_stop_payload(reason="manual_quit_stop", sequence=state.sequence + 1, config=config), force=True)
                        return
                    if action.action_type == "stop":
                        clear_motion_state(state)
                        publish(build_stop_payload(reason=action.description, sequence=state.sequence + 1, config=config), force=True)
                    elif action.action_type == "motion":
                        set_motion_key_pressed(
                            state,
                            action.description,
                            now_s,
                            release_events_supported=keyboard.release_events_supported,
                        )
                    elif action.action_type == "config":
                        print(
                            f"\rforward={config.forward_mps:.3f} m/s reverse={config.reverse_mps:.3f} m/s "
                            f"arc={config.arc_turn_radps:.3f} rad/s pivot={config.pivot_turn_radps:.3f} rad/s "
                            f"(limits {config.max_forward_mps:.2f}/{config.max_reverse_mps:.2f}/"
                            f"{config.max_arc_turn_radps:.2f}/{config.max_pivot_turn_radps:.2f})   ",
                            end="",
                            flush=True,
                        )
                payload = payload_for_state(state, config, time.monotonic())
                publish(payload)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(period_s)
    except KeyboardInterrupt:
        publish(build_stop_payload(reason="manual_keyboard_interrupt_stop", sequence=state.sequence + 1, config=config), force=True)
    finally:
        try:
            publish(build_stop_payload(reason="manual_shutdown_stop", sequence=state.sequence + 1, config=config), force=True)
            time.sleep(0.05)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(argv: Optional[Iterable[str]] = None) -> None:
    run_teleop(parse_args(argv))


if __name__ == "__main__":
    main()
