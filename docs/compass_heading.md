# Magnetic heading and simple tilt compensation

No firmware reflash is needed after the combined sensor test. Close other serial
programs, then run from the repository root:

```powershell
conda activate find-home
python tools/imu_viewer.py --port COM3 --compass --accel-cal calibration/imu_accel.json --mag-cal calibration/mag.json --mag-axes=y,x,-z --log captures/heading-check.txt
```

Press RESET after connecting, hold level and still, press Enter and wait for the
two-second IMU reference. The magnetic dial then uses the IMU's estimated up
direction to compensate normal pickup and tilt. Keep motions gentle. No new
sensor calibration is required; all XYZ field plots remain available.

Add `--level-only` to compare with the original held-level baseline (5-degree
limit, no IMU reference needed for the compass). Existing gyro-only, no-bias and
accelerometer calibration options remain; gyro-only allows the estimated tilt
to drift because gravity correction is disabled. Both capture scripts preserve
existing logs unless `--overwrite` is supplied.

## Frames and mounting

Board +X points from buttons toward POTI, +Y is left viewed along +X, +Z points
out of the component side. Magnetometer calibration operates in SENSOR axes
first; `--mag-axes=y,x,-z` then maps corrected sensor (Y,X,-Z) into board (X,Y,Z).
Signed-axis transforms must be proper rotations, never reflections. Omitting
`--mag-axes` retains the field plots without enabling a heading dial.

This transform was inferred for the current zip-tied mounting; the user
subsequently confirmed the held-level heading direction physically. On the v5 capture, it gives 2.05 uT standard
deviation of the field component along measured gravity; the next best of the
24 proper signed-axis transforms gives 13.50 uT. Earlier gyro-motion checks also
favored this mapping. These support the axis choice but do not prove absolute
heading accuracy or remove a small arbitrary mounting misalignment.

## Display and validity

In level-only mode, `heading = atan2(board_By, board_Bx)` in degrees, wrapped to
[0,360). Heading increases clockwise from MAGNETIC north: north 0, east 90,
south 180, west 270. Board +Y points left. Gyro yaw increases counterclockwise
relative to its starting orientation. The dial fixes board forward at the top;
its N needle points toward magnetic north relative to the board.

Default tilt compensation normalizes estimated up `u`, projects board +X into
the horizontal plane, and normalizes this to forward `f`. Horizontal left is
`u cross f`; heading is `atan2(B dot left, B dot f)`. The up vector is world-up
rotated into board axes by the IMU quaternion. It depends on tilt, not absolute
gyro yaw, so gyro yaw drift does not directly rotate the magnetic heading.
There is no declination correction or magnetic feedback into the IMU quaternion.
The 3D slab and the XYZ field plots retain their existing meanings.

The dial hides its needle for missing calibration/mounting, overflow, sensor age
>0.5 s, IMU/magnetic timestamps >100 ms apart, field magnitude outside 80-120% of
the calibrated reference, acceleration outside 0.9-1.1 g, or horizontal field
<5 uT. Tilt mode also requires a valid IMU reference, component side facing up,
and board forward at least 10 degrees from vertical. Loss of the IMU reference
requires Enter again; ordinary tilt warnings recover automatically. Level-only
mode instead enforces top-up tilt <=5 degrees.

This supports ordinary gentle handling, not arbitrary dynamic motion. Linear
acceleration can corrupt the gravity estimate even inside the acceleration gate;
magnetic interference can change direction while preserving magnitude. Summary
JSON records the mode, heading and validity reason.

The coordinate alignment principle is described in
[NXP AN4248](https://www.nxp.com/docs/en/application-note/AN4248.pdf);
our forward/left/up frame is used explicitly rather than its NED convention.

## Physical check to return

1. Start the command above with a new log name, e.g. `captures/heading-tilt.txt`.
   RESET, then Enter while level/still; wait for the reference to finish.
2. Point POTI in a marked direction and note the heading. Slowly raise/lower the
   POTI end 20-30 degrees while keeping its horizontal direction fixed. Pause at
   each tilt, then return level. Repeat by rolling 20-30 degrees left/right.
3. Heading should remain near the initial value. Do not turn left/right during
   these tilts: that changes the actual heading. Note approximate excursions.
4. Return level and turn through a full circle to check signs still agree.
   Forward almost vertical or upside down should hide the needle; level it again
   and check recovery. Close the viewer and return the log plus your observations.
5. `--level-only` permits comparison with the previous implementation. Neither a
   GNSS fix nor repeating magnetic calibration is needed.

## Validation capture provenance

The original `mag-validation-v6.txt` was overwritten on 2026-11-19. No raw backup
was found in the repository or capture directory. The unchanged calibration JSON
retains the original SHA256 `68e2e8ddf512e4a4420455e146a17c70d0b544523089150919a730d4bd8ef95c`
and its original validation statistics (4,649 samples, 3.76% RMS). The current
file is a different, short recording; it must not be used to reproduce those
original results. Saved report/plots remain in `captures/mag-calibration-final`.
