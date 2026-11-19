"""Held-level baseline: board +X forward, +Y left, +Z up; field in uT.

Heading is clockwise from magnetic north in [0, 360). This does not correct
magnetic declination, compensate tilt, or feed magnetic data into IMU yaw.
"""
from math import acos, atan2, degrees, hypot, isfinite


def level_heading(field, accel, reference_field):
    """Return (heading or None, diagnostic). Inputs must already use board axes."""
    if not all(isfinite(v) for v in (*field, *accel, reference_field)) or reference_field <= 0:
        return None, 'Invalid sensor values'
    strength, gravity = hypot(*field), hypot(*accel)
    if not .8*reference_field <= strength <= 1.2*reference_field:
        return None, 'Field strength outside calibration range'
    if not .9 <= gravity <= 1.1:
        return None, 'Acceleration unreliable; hold still'
    tilt = degrees(acos(max(-1, min(1, accel[2]/gravity))))
    if tilt > 5:
        return None, f'Hold level: tilt {tilt:.1f} deg (limit 5 deg)'
    if hypot(field[0], field[1]) < 5:
        return None, 'Horizontal field too weak'
    return degrees(atan2(field[1], field[0])) % 360, f'Level baseline; tilt {tilt:.1f} deg'
