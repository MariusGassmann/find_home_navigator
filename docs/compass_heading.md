# Held-level magnetic heading

No firmware reflash is needed after the combined sensor test. Close other serial
programs, then run from the repository root:

```powershell
conda activate find-home
python tools/imu_viewer.py --port COM3 --compass --accel-cal calibration/imu_accel.json --mag-cal calibration/mag.json --mag-axes=y,x,-z --log captures/heading-check.txt
```

Press RESET after connecting. Enter still collects the two-second gyro reference
for the 3D view; magnetic heading does not use that reference. Keep the board
physically level within 5 degrees. Enter does not redefine gravity or make a
sloping board level. The dial states the measured tilt when it rejects a sample.
Both capture scripts preserve existing logs unless `--overwrite` is supplied.

## Frames and mounting

Board +X points from buttons toward POTI, +Y is left viewed along +X, +Z points
out of the component side. Magnetometer calibration operates in SENSOR axes
first; `--mag-axes=y,x,-z` then maps corrected sensor (Y,X,-Z) into board (X,Y,Z).
Signed-axis transforms must be proper rotations, never reflections. Omitting
`--mag-axes` retains the field plots without enabling a heading dial.

This transform is inferred for the current zip-tied mounting, not confirmed
against an independent compass. On the v5 capture, it gives 2.05 uT standard
deviation of the field component along measured gravity; the next best of the
24 proper signed-axis transforms gives 13.50 uT. Earlier gyro-motion checks also
favored this mapping. These support the axis choice but do not prove absolute
heading accuracy or remove a small arbitrary mounting misalignment.

## Display and validity

`heading = atan2(board_By, board_Bx)` in degrees, wrapped to [0,360). It increases
clockwise from MAGNETIC north: north 0, east 90, south 180, west 270. Board +Y
points left, so this sign differs from formulas using a rightward Y axis.
Gyro yaw still increases counterclockwise relative to its starting orientation.
The dial fixes board forward at the top; its N needle points toward magnetic
north relative to the board. There is no declination correction, magnetic
fusion, or tilt compensation yet. The 3D slab continues to use gyro/gravity only.

The dial hides its needle when any condition fails: missing calibration/mounting,
overflow, sensor age >0.5 s, IMU/magnetic timestamps separated by >100 ms, field
magnitude outside 80-120% of the calibrated reference, acceleration outside
0.9-1.1 g, top-up tilt >5 degrees, or horizontal field <5 uT. This is a held-level
baseline, not a guarantee of reliable heading under all interference. Interference
can change direction while preserving field magnitude, and acceleration can
look like a change of gravity. Summary JSON includes heading and its validity reason.

The coordinate alignment principle is described in
[NXP AN4248](https://www.nxp.com/docs/en/application-note/AN4248.pdf);
our forward/left/up frame is used explicitly rather than its NED convention.

## Physical check to return

1. Use an independent compass reference for approximate magnetic north, keeping
   phones/magnetic objects away from the assembly during readings.
2. Keep level and point POTI approximately north. Expect near 0 degrees.
3. Turn the whole assembly clockwise through east, south and west, pausing about
   five seconds each: expect roughly 90,180,270 degrees and N pointing correctly.
4. Return north. Tilt beyond 5 degrees: the needle should hide and show a level
   warning. Level it again: the heading should recover without a new reference.
5. Close the viewer and return `captures/heading-check.txt`, plus which physical
   directions you used. This checks mounting/signs first; it is not a precision
   heading accuracy measurement. GNSS fix is not required.

## Validation capture provenance

The original `mag-validation-v6.txt` was overwritten on 2026-11-19. No raw backup
was found in the repository or capture directory. The unchanged calibration JSON
retains the original SHA256 `68e2e8ddf512e4a4420455e146a17c70d0b544523089150919a730d4bd8ef95c`
and its original validation statistics (4,649 samples, 3.76% RMS). The current
file is a different, short recording; it must not be used to reproduce those
original results. Saved report/plots remain in `captures/mag-calibration-final`.
