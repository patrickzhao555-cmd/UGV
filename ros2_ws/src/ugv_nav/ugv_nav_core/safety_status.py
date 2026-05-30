"""Shared safety-status normalization helpers."""

from __future__ import annotations

from typing import Any, Optional


NO_FAULT_TEXT = {"", "none", "0", "false", "null", "ok"}


def normalize_fault_value(value: Any) -> Optional[str]:
    """Return a meaningful fault string, or None for common no-fault sentinels."""
    if value is None:
        return None
    text = str(value).strip()
    if text.lower() in NO_FAULT_TEXT:
        return None
    return text


def motor_fault_reason(status: dict[str, Any]) -> Optional[str]:
    """Normalize the fault fields used by motor-controller status JSON."""
    for key in ("fault_reason", "fault"):
        fault = normalize_fault_value(status.get(key))
        if fault is not None:
            return fault
    return None
