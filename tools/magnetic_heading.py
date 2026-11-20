"""Magnetic heading: board +X forward, +Y left, +Z up; field in uT.

Clockwise from magnetic north in [0,360); no declination or magnetic fusion.
Tilt mode projects onto the horizontal plane defined by the IMU up vector.
"""
from math import acos, atan2, degrees, hypot, isfinite, radians, sin


def tilt_heading(field, accel, reference_field, up):
    if not all(isfinite(v) for v in (*field, *accel, reference_field)) or reference_field <= 0:
        return None, 'Invalid sensor values'
    strength, gravity = hypot(*field), hypot(*accel)
    if not .8*reference_field <= strength <= 1.2*reference_field:
        return None, 'Field strength outside calibration range'
    if not .9 <= gravity <= 1.1:
        return None, 'Acceleration unreliable; hold still'
    if up is None:
        return None, 'Press Enter; hold level and still for IMU reference'
    length = hypot(*up)
    if not all(isfinite(v) for v in up) or not isfinite(length) or length < 1e-6:
        return None, 'Invalid gravity estimate'
    u = tuple(v/length for v in up)
    if u[2] <= 0:
        return None, 'Keep component side facing up'
    # Forward is board +X projected onto the horizontal plane; left = up x forward.
    forward = (1-u[0]*u[0], -u[0]*u[1], -u[0]*u[2])
    projection = hypot(*forward)
    if projection < sin(radians(10)):
        return None, 'Forward direction nearly vertical'
    f = tuple(v/projection for v in forward)
    left = (u[1]*f[2]-u[2]*f[1], u[2]*f[0]-u[0]*f[2], u[0]*f[1]-u[1]*f[0])
    x, y = sum(field[i]*f[i] for i in range(3)), sum(field[i]*left[i] for i in range(3))
    tilt = degrees(acos(max(-1, min(1, u[2]))))
    if hypot(x, y) < 5:
        return None, 'Horizontal field too weak'
    return degrees(atan2(y, x)) % 360, f'Tilt compensated; tilt {tilt:.1f} deg'
