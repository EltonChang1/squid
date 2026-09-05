import math

import pytest

from tactile_umi.calibration import EncoderCalibration, FiniteDifference


def test_linear_map_and_clamp():
    cal = EncoderCalibration(scale_m_per_rad=0.008, offset_m=0.0)
    assert cal.width(0.0) == 0.0
    assert cal.width(1.0) == pytest.approx(0.008)
    assert cal.width(-1.0) == 0.0  # clamped at closed
    assert cal.width(100.0) == pytest.approx(0.085)  # clamped at max stroke
    unclamped = EncoderCalibration(0.008, clamp=False)
    assert unclamped.width(100.0) == pytest.approx(0.8)


def test_velocity_scaling():
    cal = EncoderCalibration(scale_m_per_rad=0.01)
    assert cal.velocity(2.0) == pytest.approx(0.02)


def test_two_point_fit():
    # Closed at 0.1 rad, open (85 mm) at 10.7 rad.
    cal = EncoderCalibration.from_two_points(0.1, 0.0, 10.7, 0.085)
    assert cal.width(0.1) == pytest.approx(0.0, abs=1e-12)
    assert cal.width(10.7) == pytest.approx(0.085)
    assert cal.width((0.1 + 10.7) / 2) == pytest.approx(0.0425)
    assert cal.scale_m_per_rad == pytest.approx(0.085 / 10.6)


def test_two_point_fit_rejects_degenerate():
    with pytest.raises(ValueError):
        EncoderCalibration.from_two_points(1.0, 0.0, 1.0, 0.085)


def test_finite_difference():
    fd = FiniteDifference(alpha=1.0)  # no smoothing
    assert fd.update(0.0, 0) == 0.0
    assert fd.update(0.001, 1_000_000) == pytest.approx(1.0)  # 1 mm in 1 ms = 1 m/s
    assert fd.update(0.001, 2_000_000) == pytest.approx(0.0)
    # Non-monotonic time is ignored.
    assert fd.update(0.005, 1_500_000) == pytest.approx(0.0)


def test_finite_difference_smoothing():
    fd = FiniteDifference(alpha=0.5)
    fd.update(0.0, 0)
    v1 = fd.update(0.001, 1_000_000)  # raw 1.0 -> 0.5
    assert v1 == pytest.approx(0.5)
    v2 = fd.update(0.002, 2_000_000)  # raw 1.0 -> 0.75
    assert v2 == pytest.approx(0.75)
    assert not math.isnan(v2)
