# Compass calibration

The current `calibration/mag.json` was fitted from v5 (3,800 samples) and validated
on the original v6 (4,649 samples), 2026-11-19. Coverage and residual checks pass:
fit RMS 3.99%, validation RMS 3.76%, validation 95th percentile 6.78%, maximum
10.93%. Nominal reference field is 50.53 uT. JSON records coefficients, quality
and source hashes. Keep it for the current fixed mounting.

## Repeat only after a mounting/environment change

Use the dependencies in `tools/requirements.txt`. Keep the module and cable fixed
relative to the board, away from magnets, steel and nearby electrical equipment.
Close the viewer before capture; GNSS fix and Enter reference are unnecessary.

```powershell
python tools/sensor_capture.py --port COM3 --seconds 0 --log captures/mag-fit-new.txt
python tools/sensor_capture.py --port COM3 --seconds 0 --log captures/mag-validation-new.txt
```

Run each command separately. RESET after connecting, wait two seconds, then slowly
rotate for 60-90 seconds before Ctrl+C. Include all six faces: top, underside,
POTI edge, buttons edge, left edge and right edge upward. Turn through a full
circle with each face upward and include tilted transitions. Exact levelling is
unnecessary. Repeat in a different order for validation, with mounting unchanged.
Only one reset is accepted; the pre-reset prefix is discarded. Existing files
are protected unless `--overwrite` is supplied.

```powershell
python tools/mag_calibration.py captures/mag-fit-new.txt --validation captures/mag-validation-new.txt --output calibration/mag-new.json
```

Only passing fit AND validation publish the JSON. Existing calibration files are
preserved. Reports/plots go to `captures/mag-calibration/` (`--report` changes it;
`--no-plots` skips figures). Failures return exit 1 and record the reason.
Apply a passing result with `--mag-cal calibration/mag-new.json` in the viewer;
raw MAG packets and applied coefficients are logged together.

## Method and acceptance

In sensor XYZ: `corrected = matrix @ (raw_uT - offset_uT)`. A normalized ellipsoid
fit estimates hard-iron offset and a symmetric positive-definite soft-iron matrix,
following [NXP AN4246](https://www.nxp.com/docs/en/application-note/AN4246.pdf).
Determinant-one normalization fixes common scale; nominal field magnitude is not
an independent absolute-field measurement. Mounting rotation is separate.

Each set needs >=300 usable samples spanning >=20 s, without overflow. Coverage
requires both signs of every corrected axis beyond +/-0.65 of field magnitude,
and smallest directional second-moment eigenvalue >=0.08. RMS residual <=5%,
95th percentile <=10%, maximum <=20%, relative to the FIT field radius, apply to
both sets. Nonphysical/ill-conditioned fits and extreme gains are rejected.
No outliers are silently removed. These are project checks, not manufacturer
accuracy specifications. Constant norm alone does not prove heading accuracy.

The original v6 raw recording was overwritten; its original hash/statistics remain
in the JSON. See [heading/provenance](compass_heading.md). Accelerometer coefficients
and measurements are documented in [IMU calibration results](imu_calibration_results.md).
