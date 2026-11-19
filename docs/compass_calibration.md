# Compass calibration

QMC5883P, +/-8 G, 37.5 counts/uT; sensor XYZ as streamed by firmware.
No firmware change is required after the combined sensor test.

## Capture two independent rotation sets

Keep the module fixed to the shield and secure its cable. Keep the powerbank and
wiring in their intended relative positions. Use an open area away from steel,
magnets and the laptop as far as the USB cable permits. GNSS fix is not required.
Install the laptop dependencies once:

```powershell
conda activate find-home
python -m pip install -r tools/calibration_requirements.txt
```

Close other serial programs. Record the fit set:

```powershell
python tools/sensor_capture.py --port COM3 --seconds 0 --log captures/mag-fit.txt
```

For 60-90 seconds, slowly rotate the entire mounted assembly through full turns
around each of its three axes, including upside down. Mix these with tilted
turns so intermediate orientations are covered. Rotate near the same location;
keep hands/cable clear enough to avoid pulling the module loose. No level hold,
Enter reference or exactly timed poses are needed. Press Ctrl+C to finish.

Repeat in a second recording, using a different order of rotations and the same
mounting and environment. Do not copy or split the first recording:

```powershell
python tools/sensor_capture.py --port COM3 --seconds 0 --log captures/mag-validation.txt
```

After starting each capture, press board RESET once and wait two seconds before
rotating. The tool discards data preceding that reset; further resets within the
recording are rejected. Each selected session requires >=300 usable samples spanning >=20 seconds, and rejects overflow.
A few missing records are reported; unlike gyro integration, this fit does not
require uninterrupted samples.

## Fit, validate and display

```powershell
python tools/mag_calibration.py captures/mag-fit.txt --validation captures/mag-validation.txt
```

Only a passing fit AND validation create `calibration/mag.json`. Existing output
files are preserved; use `--output calibration/mag-v2.json` for a new attempt.
Diagnostics and raw/corrected plots go to `captures/mag-calibration/`; `--report`
changes that directory and `--no-plots` skips plotting. Failure returns exit 1
and records its reason in `report.json`. Input and output paths must differ.

```powershell
python tools/imu_viewer.py --port COM3 --compass --accel-cal calibration/imu_accel.json --mag-cal calibration/mag.json --log captures/mag-corrected.txt
```

The compass label changes to CAL and its plots use corrected sensor XYZ. Raw
MAG packets and the applied calibration are still logged. IMU yaw remains
relative to its starting direction; no magnetic fusion is applied yet.

## Method and limits

`corrected = matrix @ (raw_uT - offset_uT)`. The fit uses a normalized nine-term
ellipsoid least-squares system and its symmetric positive-definite square root,
following [NXP AN4246](https://www.nxp.com/docs/en/application-note/AN4246.pdf).
No vendor source code is copied. The matrix has determinant one, fixing the
otherwise unknown common scale; fitted field strength is nominal, not an
independently measured absolute field. The transform does not establish the
module-to-shield rotation or magnetic declination.

Both sets must cover both signs of all corrected axes beyond +/-0.65 of field
magnitude. The smallest eigenvalue of the directional second-moment matrix must
be >=0.08. Fit and validation residuals relative to the TRAINING field radius must
have RMS <=5%, 95th percentile <=10%, and maximum <=20%. These are initial project
acceptance limits, not manufacturer accuracy specifications. No outliers are
silently removed. Extreme gains, nonphysical ellipsoids and ill-conditioned
fits are rejected. JSON records source hashes and both sets of metrics.

Constant norm is necessary but does not prove correct heading. Calibration
cannot correct changing interference, determine arbitrary sensor misalignment,
or guarantee performance after the module/nearby metal/wiring moves. Check heading
separately, then repeat calibration after final enclosure assembly if needed.

## Current mounted assembly

`calibration/mag.json` was fitted from `mag-validation-v5.txt` (3,800 samples)
and independently checked on `mag-validation-v6.txt` (4,649 samples), 2026-11-19.
Both pass coverage and residual limits: fit RMS 3.99%, validation RMS 3.76%,
validation 95th percentile 6.78%, maximum 10.93%. Reference field is nominally
50.53 uT. Source hashes and matrix are stored in the JSON. Heading accuracy and
module-to-shield mounting rotation remain to be verified.

The original v6 raw recording was subsequently overwritten. Its original hash
and validation metrics remain in the calibration JSON; see
[heading checks and provenance](compass_heading.md).
