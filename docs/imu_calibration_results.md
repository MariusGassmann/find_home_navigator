# Accelerometer calibration

Measured on this board on 2026-11-17. Source: `captures/six-poses.txt`, preserved
from the original `captures/imu.txt`; 8,999 samples over 148.733 seconds. Capture
hash, selected windows and measurements are in `calibration/imu_accel_report.json`.
Raw captures are local and ignored by Git.

Three-second stable windows were selected from top up/down, POTI/buttons end up,
left edge down and right edge down. These poses confirm the acceleration-axis
signs for `--axes=y,-x,z`. A diagonal ellipsoid fit accommodates imperfectly aligned
holds; it does not estimate cross-axis coupling or gyro calibration.

Correction in **sensor XYZ**, before the board transform:
`corrected_g = (measured_g - offset_g) * scale`.

| Sensor axis | Offset (g) | Scale |
| --- | --- | --- |
| X | +0.010943 | 1.000705 |
| Y | -0.005445 | 0.998155 |
| Z | +0.052379 | 0.993573 |

Fitting uses the first 90 samples of each selected window; the remaining 90 and
an additional top-up hold are reserved for validation. Validation magnitude RMS
error changes from 0.03638 g to 0.00201 g. The extra top-up mean changes from
1.05873 g to 0.99984 g. The apparent 1.06 g is largely a Z offset.

These are within-capture consistency results, not an independent accuracy
specification. Six coefficients are fitted to six means; adjacent validation
samples share similar conditions. Repeatability over temperature and other
orientations remains unmeasured. Use these coefficients only for this board.

```powershell
python tools/imu_viewer.py --port COM3 --accel-cal calibration/imu_accel.json --log captures/calibrated.txt
python tools/imu_calibration.py captures/six-poses.txt --output calibration/imu_accel.json --report calibration/imu_accel_report.json
```

Calibration is opt-in. Logs retain raw packets plus the applied coefficients.
Replay needs the same `--accel-cal` flag/file; no firmware flashing is needed.
