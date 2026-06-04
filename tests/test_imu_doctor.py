import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ugv_imu_doctor import diagnose_imu_health, rate_hz  # noqa: E402


def test_rate_hz_from_arrival_times():
    assert rate_hz([1.0, 1.1, 1.2, 1.3]) == pytest.approx(10.0)
    assert rate_hz([1.0]) == 0.0


def test_diagnose_imu_health_passes_when_topic_rate_is_good():
    ok, reason = diagnose_imu_health(
        imu_messages=20,
        imu_rate_hz=50.0,
        min_rate_hz=20.0,
        zed_status=None,
    )
    assert ok
    assert reason == "imu_ok"


def test_diagnose_imu_health_points_at_zed_sdk_error():
    ok, reason = diagnose_imu_health(
        imu_messages=0,
        imu_rate_hz=0.0,
        min_rate_hz=20.0,
        zed_status={"imu_count": 0, "last_imu_error": "nonfinite_imu_vector"},
    )
    assert not ok
    assert reason == "zed_imu_not_publishing:nonfinite_imu_vector"


def test_diagnose_imu_health_detects_topic_or_qos_mismatch():
    ok, reason = diagnose_imu_health(
        imu_messages=0,
        imu_rate_hz=0.0,
        min_rate_hz=20.0,
        zed_status={"imu_count": 100, "last_imu_error": None},
    )
    assert not ok
    assert reason == "imu_topic_missing_or_qos_mismatch"
