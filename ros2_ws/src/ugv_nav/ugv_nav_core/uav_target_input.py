"""Helpers for UAV/ESP/terminal target-coordinate input.

The formal mission supervisor only consumes ``/ugv/uav_target``. These helpers
normalize human terminal input and ESP line protocol messages into the same
field-frame marker coordinate contract.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from typing import Optional

from .nav2_bridge import target_units_scale


@dataclass(frozen=True)
class ParsedUavTarget:
    accepted: bool
    x_m: Optional[float] = None
    y_m: Optional[float] = None
    seq: Optional[int] = None
    units: str = "meters"
    reason: str = "ok"
    source_format: str = "unknown"
    raw: str = ""


def esp_line_checksum(payload: str) -> int:
    """Return an NMEA-style XOR checksum for the payload before ``*``."""

    checksum = 0
    for char in str(payload):
        checksum ^= ord(char)
    return checksum & 0xFF


def append_esp_checksum(payload: str) -> str:
    return f"{payload}*{esp_line_checksum(payload):02X}"


def _split_optional_checksum(line: str) -> tuple[str, Optional[str]]:
    if "*" not in line:
        return line, None
    payload, checksum_text = line.rsplit("*", 1)
    return payload.strip(), checksum_text.strip()


def _validate_checksum(payload: str, checksum_text: Optional[str], *, require_checksum: bool) -> Optional[str]:
    if checksum_text is None:
        return "checksum_missing" if require_checksum else None
    if not re.fullmatch(r"[0-9a-fA-F]{2}", checksum_text):
        return "checksum_invalid"
    expected = esp_line_checksum(payload)
    received = int(checksum_text, 16)
    if received != expected:
        return "checksum_mismatch"
    return None


def _scale_for_units(units: str) -> Optional[float]:
    return target_units_scale(units)


def _parse_number(value: object) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _looks_numeric(value: object) -> bool:
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def _parse_seq(value: object) -> Optional[int]:
    if value is None or value == "":
        return None
    text = str(value).strip()
    if not re.fullmatch(r"[+-]?\d+", text):
        return None
    try:
        seq_int = int(text)
    except (TypeError, ValueError):
        return None
    return seq_int


def _apply_units(
    *,
    x_value: object,
    y_value: object,
    units: str,
    seq: Optional[int],
    raw: str,
    source_format: str,
) -> ParsedUavTarget:
    scale = _scale_for_units(units)
    if scale is None:
        return ParsedUavTarget(False, seq=seq, units=units, reason="units_invalid", source_format=source_format, raw=raw)
    x = _parse_number(x_value)
    y = _parse_number(y_value)
    if x is None or y is None:
        return ParsedUavTarget(False, seq=seq, units=units, reason="coordinate_invalid", source_format=source_format, raw=raw)
    return ParsedUavTarget(
        True,
        x_m=x * scale,
        y_m=y * scale,
        seq=seq,
        units=units,
        reason="ok",
        source_format=source_format,
        raw=raw,
    )


def _parse_json_payload(payload: str, *, default_units: str, raw: str) -> ParsedUavTarget:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return ParsedUavTarget(False, units=default_units, reason="json_invalid", source_format="json", raw=raw)
    if not isinstance(data, dict):
        return ParsedUavTarget(False, units=default_units, reason="json_not_object", source_format="json", raw=raw)

    units = str(data.get("units") or data.get("target_units") or default_units).strip()
    seq = _parse_seq(data.get("seq", data.get("sequence")))
    x_value = data.get("x_m", data.get("x", data.get("marker_x_m", data.get("marker_x"))))
    y_value = data.get("y_m", data.get("y", data.get("marker_y_m", data.get("marker_y"))))
    return _apply_units(x_value=x_value, y_value=y_value, units=units, seq=seq, raw=raw, source_format="json")


def _parse_csv_payload(payload: str, *, default_units: str, raw: str) -> ParsedUavTarget:
    parts = [part.strip() for part in re.split(r"[\s,]+", payload) if part.strip()]
    if not parts:
        return ParsedUavTarget(False, units=default_units, reason="empty", source_format="text", raw=raw)

    label = ""
    if not _looks_numeric(parts[0]):
        label = parts.pop(0).lower()

    if len(parts) < 2:
        return ParsedUavTarget(False, units=default_units, reason="field_count_invalid", source_format="text", raw=raw)

    seq: Optional[int] = None
    units = default_units
    x_value: object
    y_value: object

    # Preferred ESP/human forms:
    #   TARGET,x,y
    #   TARGET,x,y,meters
    #   TARGET,seq,x,y
    #   TARGET,seq,x,y,meters
    # Plain numeric terminal forms are intentionally only x,y to avoid
    # confusing a valid x coordinate with a sequence number.
    if label and len(parts) == 2:
        x_value, y_value = parts[0], parts[1]
    elif label and len(parts) == 3 and _scale_for_units(parts[2]) is not None:
        x_value, y_value = parts[0], parts[1]
        units = parts[2]
    elif label and len(parts) >= 3:
        seq = _parse_seq(parts[0])
        if seq is None:
            return ParsedUavTarget(False, units=units, reason="sequence_invalid", source_format=label, raw=raw)
        x_value, y_value = parts[1], parts[2]
        if len(parts) >= 4:
            units = parts[3]
    else:
        x_value, y_value = parts[0], parts[1]
        if len(parts) >= 3:
            units = parts[2]

    return _apply_units(x_value=x_value, y_value=y_value, units=units, seq=seq, raw=raw, source_format=label or "text")


def parse_uav_target_line(
    line: str,
    *,
    default_units: str = "meters",
    require_checksum: bool = False,
) -> ParsedUavTarget:
    """Parse one target line into map-frame meters.

    Accepted examples:

    - ``5.0 7.0``
    - ``5.0,7.0``
    - ``TARGET,5.0,7.0``
    - ``TARGET,12,5.0,7.0``
    - ``{"x_m": 5.0, "y_m": 7.0, "seq": 12}``
    - any of the line payloads above with ``*XX`` XOR checksum appended
    """

    raw = str(line).strip()
    if not raw:
        return ParsedUavTarget(False, units=default_units, reason="empty", raw=raw)
    if raw.startswith("#"):
        return ParsedUavTarget(False, units=default_units, reason="comment", raw=raw)

    payload, checksum_text = _split_optional_checksum(raw)
    checksum_error = _validate_checksum(payload, checksum_text, require_checksum=require_checksum)
    if checksum_error is not None:
        return ParsedUavTarget(False, units=default_units, reason=checksum_error, raw=raw)

    if payload.lstrip().startswith("{"):
        return _parse_json_payload(payload, default_units=default_units, raw=raw)
    return _parse_csv_payload(payload, default_units=default_units, raw=raw)
