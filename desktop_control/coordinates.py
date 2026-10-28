"""Mount coordinates, independent of the v1 protocol's raw microsecond words.

The photographed home is pan 90 / tilt 0 at 1500 us on both axes. Direction
is a mounting preference; numerical angles remain commands, not measurements.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MountAxis:
    neutral: float
    maximum: float
    reversed: bool = False
    minimum: float = 0

    def __post_init__(self):
        if not (all(math.isfinite(value) for value in (self.minimum, self.neutral, self.maximum)) and
                self.minimum < self.maximum and self.minimum <= self.neutral <= self.maximum):
            raise ValueError("轴角度范围无效，范围必须包含回正位置")

    def to_us(self, angle):
        value = float(angle)
        if not math.isfinite(value) or not self.minimum <= value <= self.maximum:
            raise ValueError(f"目标角度必须在 {self.minimum:g}～{self.maximum:g}° 内")
        sign = -1 if self.reversed else 1
        return round(1500 + sign * (value - self.neutral) * 2000 / 180)

    def from_us(self, pulse):
        # Never clip: an old firmware limit or position can be below mount 0°.
        sign = -1 if self.reversed else 1
        return round(self.neutral + sign * (pulse - 1500) * 180 / 2000, 2)

    def pulse_limits(self, minimum, maximum):
        minimum, maximum = float(minimum), float(maximum)
        if not (self.minimum <= minimum < maximum <= self.maximum and
                minimum <= self.neutral <= maximum):
            raise ValueError(f"限位需在 {self.minimum:g}～{self.maximum:g}° 内，最小值小于最大值，且包含回正位置 {self.neutral:g}°")
        return sorted((self.to_us(minimum), self.to_us(maximum)))

    def angle_limits(self, pulse_min, pulse_max):
        return sorted((self.from_us(pulse_min), self.from_us(pulse_max)))
