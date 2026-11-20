# Fused magnetic north demo

This increment runs independently on the STM32. GNSS continues streaming but
does not affect the arrow. Home capture, Z animation, sleep/wake and distance
navigation are not implemented yet. Enter currently means heading reference.

## Hardware check

1. Build/flash Debug using the existing VS Code GUI. Start the viewer with the
   command in README, choosing a new capture filename if it already exists.
2. Press RESET. A red 3x3 X means no heading reference. The shield is actually
   4x3; the X uses three columns and leaves the fourth off.
3. Keep level and still, press board Enter (or viewer Enter), and remain still
   for about 2.3 seconds. Blinking amber centre means gyro calibration or compass
   alignment. Once ready, green points north and red marks the opposite end.
4. Rotate slowly through 360 degrees, pause at quarter turns, then reverse.
   The arrow should keep pointing toward the same physical magnetic north.
   The blue gyro, yellow compass and green fused curves should follow together;
   small compass noise should be smoothed in the fused curve. Their numbers
   are clockwise degrees from magnetic north, not relative yaw.
5. Keep forward's horizontal direction fixed and gently tilt 20–30 degrees,
   pause, and return level. Check that north remains approximately fixed.
6. Leave stationary for two minutes. The independent gyro curve may drift;
   the fused curve should stay near the compass. Close with Escape.

Return the capture filename and note any arrow reversal, persistent X, or large
heading jump. No new calibration or outdoor GNSS fix is required. The module
must retain its calibrated mounting; keep nearby magnetic objects stationary.
The 3D slab still shows relative gyro/gravity orientation for comparison.

## Implementation and limits

`north.c` combines the gravity-corrected IMU quaternion with the existing
tilt-compensated compass calculation. Verified transforms are IMU `(Y,-X,Z)`
and compass `(Y,X,-Z)` into forward/left/up board coordinates. Both saved
calibrations are exported automatically at build time; startup prints their
combined fingerprint. Raw sensor records remain unchanged.

After the two-second stationary gyro bias measurement, require 200 ms of
consistent compass alignment. Anchor the independent gyro and fused headings
to that magnetic heading. Propagate both with negative quaternion-yaw changes;
correct only fused heading toward compass with a five-second time constant,
using the shortest angular difference through 0/360. The gyro bias remains fixed
for this run. LED sectors retain three degrees of hysteresis.

Compass checks retain field strength 80–120%, acceleration 0.9–1.1 g,
horizontal field at least 5 uT, IMU age at most 100 ms, overflow rejection and
components-up/forward-away-from-vertical requirements. Also reject compass
innovations over 30 degrees. After 0.5 s without accepted compass data, use an
amber gyro arrow; after 2 s, show red X. An IMU gap over 100 ms or saturation
requires Enter again. Large sustained compass disagreement also needs a fresh
reference after removing interference. Ordinary invalid tilt realigns when
returned upright. A plausible but distorted magnetic field can still pass these
checks. This is magnetic north; true-north declination is a later navigation step.

Telemetry at 10 Hz:
`FUSE,tick_ms,state,gyro_mdeg,compass_mdeg,fused_mdeg,sector,compass_used`.
Angles use clockwise millidegrees [0,360000); INT32_MIN means unavailable.
States are WAITING, CALIBRATING, ALIGNING, TRACKING, COASTING, UNRELIABLE.
The viewer plots these firmware values directly. Its existing Python compass
calculation remains available for old captures without FUSE records.
