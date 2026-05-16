#!/usr/bin/env python3
"""Recovery-state scaffold for migrating active-scan and stuck handling."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class RecoveryState(str, Enum):
    NORMAL = "NORMAL"
    SLOW_APPROACH = "SLOW_APPROACH"
    BLOCKED_CONFIRM = "BLOCKED_CONFIRM"
    PANORAMIC_SCAN = "PANORAMIC_SCAN"
    SELECT_ESCAPE_HEADING = "SELECT_ESCAPE_HEADING"
    FORWARD_PROBE = "FORWARD_PROBE"
    REPLAN = "REPLAN"
    WAIT_OR_STOP = "WAIT_OR_STOP"


@dataclass
class RecoveryContext:
    state: RecoveryState = RecoveryState.NORMAL
    front_clearance_m: Optional[float] = None
    stuck_steps: int = 0
    plan_failed_steps: int = 0
    active_scan_remaining: int = 0
    active_scan_cooldown_steps: int = 0
    forward_view_confident: bool = True


@dataclass
class RecoveryDecision:
    state: RecoveryState
    reason: str = ""
    start_active_scan: bool = False
    force_replan: bool = False
    stop: bool = False


def classify_recovery_state(ctx: RecoveryContext) -> RecoveryDecision:
    """Pure classification helper used as the first migration point.

    UGVNavigator still owns the behavior in this slice. The next safe migration
    points are:
    - active-scan start gating in UGVNavigator._can_start_active_scan()
    - no-safe-trajectory handling after VelocityLocalPlanner.choose_command()
    - stuck/blocked patch handling near UGVNavigator.step() stuck_counter logic
    """
    if ctx.active_scan_remaining > 0:
        return RecoveryDecision(RecoveryState.PANORAMIC_SCAN, "active scan running")
    if ctx.active_scan_cooldown_steps > 0:
        return RecoveryDecision(RecoveryState.FORWARD_PROBE, "active scan cooldown/probe")
    if ctx.stuck_steps > 0 or ctx.plan_failed_steps > 0:
        return RecoveryDecision(RecoveryState.BLOCKED_CONFIRM, "confirming blocked path")
    if ctx.front_clearance_m is not None and ctx.front_clearance_m < 0.75:
        return RecoveryDecision(RecoveryState.SLOW_APPROACH, "front clearance constrained")
    return RecoveryDecision(RecoveryState.NORMAL, "normal")
