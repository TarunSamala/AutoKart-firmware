"""Standard AutoKart encoder filters (MicroPython compatible).

Recommended acquisition rate: 200 Hz.

AS5600 steering/brake:
  - unwrap 12-bit circular angle
  - median-of-three spike suppression
  - first-order low-pass filter (10 ms time constant)
  - reject impossible angular-rate jumps
  - calibrated signed angle about a software zero

AS5047P BLDC:
  - unwrap 14-bit motor angle into a cumulative count
  - reject parity/diagnostic failures before calling ``update``
  - reject impossible motor-speed jumps
  - first-order RPM filter (30 ms time constant)
  - estimate axle RPM, vehicle speed, and distance from configured geometry

Filtering never turns an invalid sample into a valid one. Callers must also
enforce sample freshness and react safely when ``update`` returns ``None``.
"""

import time


def clamp(value, low, high):
    return max(low, min(high, value))


def monotonic_us():
    if hasattr(time, "ticks_us"):
        return time.ticks_us()
    return int(time.monotonic() * 1_000_000)


def ticks_difference(current, previous):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(current, previous)
    return current - previous


def median_three(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def wrapped_delta(current, previous, counts_per_revolution):
    half = counts_per_revolution // 2
    return (
        (int(current) - int(previous) + half)
        % counts_per_revolution
    ) - half


def low_pass(previous, sample, elapsed_s, time_constant_s):
    if previous is None or time_constant_s <= 0:
        return float(sample)
    alpha = elapsed_s / (time_constant_s + elapsed_s)
    return previous + alpha * (sample - previous)


class AS5600AngleFilter:
    """Filtered calibrated angle for steering or brake AS5600 sensors."""

    COUNTS_PER_REVOLUTION = 4096

    def __init__(
        self,
        time_constant_s=0.010,
        maximum_rate_dps=1000.0,
        direction=1,
    ):
        self.time_constant_s = float(time_constant_s)
        self.maximum_rate_dps = float(maximum_rate_dps)
        self.direction = 1 if direction >= 0 else -1
        self.last_raw = None
        self.last_timestamp_us = None
        self.unwrapped_counts = None
        self.filtered_counts = None
        self.zero_counts = None
        self.median_window = []
        self.valid_samples = 0
        self.rejected_samples = 0

    def reset(self):
        self.last_raw = None
        self.last_timestamp_us = None
        self.unwrapped_counts = None
        self.filtered_counts = None
        self.median_window = []
        self.valid_samples = 0
        self.rejected_samples = 0

    def calibrate_zero(self):
        if self.filtered_counts is None:
            return False
        self.zero_counts = self.filtered_counts
        return True

    def set_zero_raw(self, raw_count):
        self.zero_counts = int(raw_count) & 0x0FFF

    def update(self, raw_count, timestamp_us=None, magnet_ok=True):
        if not magnet_ok:
            self.rejected_samples += 1
            return None

        raw_count = int(raw_count)
        if raw_count < 0 or raw_count >= self.COUNTS_PER_REVOLUTION:
            self.rejected_samples += 1
            return None
        if timestamp_us is None:
            timestamp_us = monotonic_us()

        if self.last_raw is None:
            self.last_raw = raw_count
            self.last_timestamp_us = timestamp_us
            self.unwrapped_counts = float(raw_count)
            self.filtered_counts = float(raw_count)
            self.median_window = [float(raw_count)]
            self.valid_samples += 1
            return self.value()

        elapsed_us = ticks_difference(timestamp_us, self.last_timestamp_us)
        if elapsed_us <= 0:
            self.rejected_samples += 1
            return None
        elapsed_s = elapsed_us / 1_000_000.0
        delta = wrapped_delta(
            raw_count,
            self.last_raw,
            self.COUNTS_PER_REVOLUTION,
        )
        rate_dps = abs(
            delta * 360.0
            / self.COUNTS_PER_REVOLUTION
            / elapsed_s
        )
        if rate_dps > self.maximum_rate_dps:
            self.rejected_samples += 1
            return None

        self.last_raw = raw_count
        self.last_timestamp_us = timestamp_us
        self.unwrapped_counts += delta
        self.median_window.append(self.unwrapped_counts)
        if len(self.median_window) > 3:
            self.median_window.pop(0)
        median_counts = median_three(self.median_window)
        self.filtered_counts = low_pass(
            self.filtered_counts,
            median_counts,
            elapsed_s,
            self.time_constant_s,
        )
        self.valid_samples += 1
        return self.value()

    def value(self):
        if self.filtered_counts is None:
            return None
        zero = self.zero_counts if self.zero_counts is not None else 0.0
        degrees = (
            (self.filtered_counts - zero)
            * 360.0
            / self.COUNTS_PER_REVOLUTION
            * self.direction
        )
        return {
            "angle_deg": degrees,
            "raw_count": self.last_raw,
            "filtered_count": self.filtered_counts,
            "zero_valid": self.zero_counts is not None,
            "valid_samples": self.valid_samples,
            "rejected_samples": self.rejected_samples,
        }


class AS5047KinematicsFilter:
    """Motor-angle unwrap plus filtered RPM, speed, and distance estimates."""

    COUNTS_PER_REVOLUTION = 16384

    def __init__(
        self,
        motor_revolutions_per_axle_revolution,
        wheel_circumference_m,
        time_constant_s=0.030,
        maximum_motor_rpm=6000.0,
    ):
        self.reduction = float(motor_revolutions_per_axle_revolution)
        self.wheel_circumference_m = float(wheel_circumference_m)
        self.time_constant_s = float(time_constant_s)
        self.maximum_motor_rpm = float(maximum_motor_rpm)
        self.last_raw = None
        self.last_timestamp_us = None
        self.total_motor_counts = 0
        self.filtered_motor_rpm = None
        self.valid_samples = 0
        self.rejected_samples = 0

    def reset(self):
        self.last_raw = None
        self.last_timestamp_us = None
        self.total_motor_counts = 0
        self.filtered_motor_rpm = None
        self.valid_samples = 0
        self.rejected_samples = 0

    def update(self, raw_count, timestamp_us=None, sample_ok=True):
        if not sample_ok or self.reduction <= 0:
            self.rejected_samples += 1
            return None

        raw_count = int(raw_count)
        if raw_count < 0 or raw_count >= self.COUNTS_PER_REVOLUTION:
            self.rejected_samples += 1
            return None
        if timestamp_us is None:
            timestamp_us = monotonic_us()

        if self.last_raw is None:
            self.last_raw = raw_count
            self.last_timestamp_us = timestamp_us
            self.filtered_motor_rpm = 0.0
            self.valid_samples += 1
            return self.value(0.0)

        elapsed_us = ticks_difference(timestamp_us, self.last_timestamp_us)
        if elapsed_us <= 0:
            self.rejected_samples += 1
            return None
        elapsed_s = elapsed_us / 1_000_000.0
        delta = wrapped_delta(
            raw_count,
            self.last_raw,
            self.COUNTS_PER_REVOLUTION,
        )
        raw_motor_rpm = (
            delta
            / self.COUNTS_PER_REVOLUTION
            / elapsed_s
            * 60.0
        )
        if abs(raw_motor_rpm) > self.maximum_motor_rpm:
            self.rejected_samples += 1
            return None

        self.last_raw = raw_count
        self.last_timestamp_us = timestamp_us
        self.total_motor_counts += delta
        self.filtered_motor_rpm = low_pass(
            self.filtered_motor_rpm,
            raw_motor_rpm,
            elapsed_s,
            self.time_constant_s,
        )
        self.valid_samples += 1
        return self.value(raw_motor_rpm)

    def value(self, raw_motor_rpm=0.0):
        motor_rpm = self.filtered_motor_rpm or 0.0
        axle_rpm = motor_rpm / self.reduction
        vehicle_speed_mps = (
            axle_rpm / 60.0 * self.wheel_circumference_m
        )
        distance_m = (
            self.total_motor_counts
            / self.COUNTS_PER_REVOLUTION
            / self.reduction
            * self.wheel_circumference_m
        )
        return {
            "raw_count": self.last_raw,
            "total_motor_counts": self.total_motor_counts,
            "raw_motor_rpm": raw_motor_rpm,
            "motor_rpm": motor_rpm,
            "axle_rpm": axle_rpm,
            "vehicle_speed_mps": vehicle_speed_mps,
            "distance_m": distance_m,
            "valid_samples": self.valid_samples,
            "rejected_samples": self.rejected_samples,
        }
