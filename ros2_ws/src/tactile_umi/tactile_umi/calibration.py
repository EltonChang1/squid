"""Encoder angle -> jaw opening width mapping.

The firmware streams the unwrapped AS5048A angle in radians. For a
rack-and-pinion or lever drive the jaw width is a linear function of angle:

    width_m = scale * angle_rad + offset

``scale`` is metres per radian (pinion pitch radius for a rack drive) and
``offset`` is the width at angle 0 (the boot zero). The blueprint stroke is
0 to 85 mm, so the result is clamped to ``[min_width, max_width]`` by default.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EncoderCalibration:
    scale_m_per_rad: float
    offset_m: float = 0.0
    min_width_m: float = 0.0
    max_width_m: float = 0.085
    clamp: bool = True

    def width(self, angle_rad: float) -> float:
        w = self.scale_m_per_rad * angle_rad + self.offset_m
        if self.clamp:
            w = min(max(w, self.min_width_m), self.max_width_m)
        return w

    def velocity(self, angle_rate_rad_s: float) -> float:
        return self.scale_m_per_rad * angle_rate_rad_s

    @classmethod
    def from_two_points(cls, angle_a: float, width_a: float, angle_b: float, width_b: float,
                        **kw) -> 'EncoderCalibration':
        """Fit from two (angle, width) measurements, e.g. fully closed and fully open."""
        if angle_a == angle_b:
            raise ValueError('calibration angles must differ')
        scale = (width_b - width_a) / (angle_b - angle_a)
        offset = width_a - scale * angle_a
        return cls(scale_m_per_rad=scale, offset_m=offset, **kw)


class FiniteDifference:
    """First-order derivative with exponential smoothing for JointState.velocity."""

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self._last_x = None
        self._last_t_ns = None
        self.value = 0.0

    def update(self, x: float, t_ns: int) -> float:
        if self._last_x is not None and t_ns > self._last_t_ns:
            raw = (x - self._last_x) / ((t_ns - self._last_t_ns) * 1e-9)
            self.value = self.alpha * raw + (1 - self.alpha) * self.value
        self._last_x = x
        self._last_t_ns = t_ns
        return self.value
