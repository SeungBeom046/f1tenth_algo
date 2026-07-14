"""Shared VESC conversion helpers for the real F1TENTH car."""

ERPM_GAIN = 4614.0
MIN_DRIVE_ERPM = 1750.0
MIN_DRIVE_SPEED_MS = MIN_DRIVE_ERPM / ERPM_GAIN


def apply_min_drive_speed(speed_ms, deadband=0.0):
    """
    Keep non-braking drive commands above the measured stable motor range.

    A zero speed command is treated as braking/stop and is preserved. If a
    deadband is provided, tiny joystick inputs are also treated as stop.
    """
    if abs(speed_ms) <= deadband:
        return 0.0

    if abs(speed_ms) < MIN_DRIVE_SPEED_MS:
        return MIN_DRIVE_SPEED_MS if speed_ms > 0.0 else -MIN_DRIVE_SPEED_MS

    return speed_ms


def speed_to_erpm(speed_ms):
    """Convert a speed command in m/s to motor ERPM."""
    return speed_ms * ERPM_GAIN
