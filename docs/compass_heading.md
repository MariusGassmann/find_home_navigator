# Magnetic heading

Use the viewer command in [README](../README.md). RESET, hold level/still,
press Enter and wait for the two-second IMU reference. All XYZ field plots are
retained. Sensor and accelerometer calibration files stay unchanged.

Corrected compass sensor `(Y,X,-Z)` maps into board XYZ. This mapping was
inferred from the rotation captures and the user confirmed heading direction
physically. Small mounting errors and absolute heading accuracy remain unmeasured.

The IMU quaternion supplies estimated up in board axes. Project board +X onto
the horizontal plane and normalize it as forward `f`; horizontal left is
`up cross f`. Heading is `atan2(B dot left, B dot f)` in degrees, wrapped to
[0,360), clockwise from magnetic north. The dial fixes POTI/forward at the top
and points its N needle toward north. The 3D slab uses gyro/gravity only; magnetic
readings do not feed back into its orientation. No declination correction yet.

Heading is unavailable without a valid reference, calibration or mounting,
after overflow, with sensor age >0.5 s or timestamp separation >100 ms, field
outside 80-120% of the calibrated reference, acceleration outside 0.9-1.1 g,
horizontal field <5 uT, component side down, or forward within 10 degrees of
vertical. Ordinary tilt warnings recover when levelled; lost IMU tracking needs
Enter again. Linear acceleration and interference can still distort heading.

For a hardware check, keep POTI's horizontal direction fixed while slowly pitching
and rolling 20-30 degrees, pause, then return level. Heading should stay near its
initial value. Turn through a full circle to check signs. Log with a new filename,
e.g. `--log captures/heading-tilt.txt`; no GNSS fix or recalibration is needed.

Frames and coordinate alignment follow [NXP AN4248](https://www.nxp.com/docs/en/application-note/AN4248.pdf),
using our forward/left/up convention explicitly.

The original `mag-validation-v6.txt` was overwritten. Its original source hash
and 4,649-sample / 3.76% RMS validation results remain in `calibration/mag.json`.
The replacement file does not reproduce those results; original report/plots
remain under `captures/mag-calibration-final`. No raw backup was found.
