# Find Home navigator

STM32 NUCLEO-L476RG, ETH PBL Shield rev 1.2, LSM6DSV16BX IMU and HGLRC
M100-5883 GNSS/QMC5883P compass. Sensors, calibration, visualization and the
LED north demo work, including onboard gyro/compass fusion. Home capture,
navigation and sleep/wake are next.

## Build and flash

Use the existing STM32 Run & Debug GUI with the `Debug` preset. The course target
name `LAB04_I2C` stays unchanged. Terminal equivalent:

```powershell
cmake --preset Debug
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -d build/Debug/LAB04_I2C.elf -v -rst
```

## Viewer

From the project root, using the existing Conda environment:

```powershell
conda activate find-home
python -m pip install -r tools/requirements.txt
python tools/imu_viewer.py --port COM3 --compass --accel-cal calibration/imu_accel.json --mag-cal calibration/mag.json --mag-axes=y,x,-z --log captures/north-demo.txt
```

Close other serial monitors first. Press board RESET, hold level and still,
press Enter, and wait two seconds for the gyro reference. The 3D board shows
relative gyro/gravity orientation; the dial and LED arrow show the STM32's fused
magnetic heading. A time plot compares anchored gyro, compass and fused heading.
XYZ field plots remain in sensor axes. Board +X points toward POTI.
See [north demo/test steps](docs/north_demo.md), [hardware/frames](docs/hardware.md)
and [tilt compensation](docs/compass_heading.md).

Useful options: `--gyro-only` disables gravity correction; `--no-bias` disables
gyro bias subtraction; omit `--accel-cal` to compare raw acceleration. These
options affect the laptop estimate; onboard fusion always uses saved calibration,
gravity correction and measured gyro bias.
`--demo` replaces the port with simulated IMU motion; `--replay captures/session.txt`
replays a log with the same calibration/axes options. `--headless --seconds 60`
records without a window. Escape closes the viewer. Logs remain raw and include
settings; a summary JSON is written on close. Existing logs require an explicit
`--overwrite`, or choose a new name.

## Capture and calibration

```powershell
python tools/sensor_capture.py --port COM3 --seconds 0 --log captures/sensors.txt
python tools/sensor_analysis.py captures/sensors.txt --output captures/analysis
```

RESET after connecting; Ctrl+C stops. The script logs everything and prints only
startup/status lines. GNSS reception needs open sky; IMU/compass tests need no fix.
[Calibration instructions](docs/compass_calibration.md) and the measured
[accelerometer results](docs/imu_calibration_results.md) describe the saved data.
Keep `calibration/` versioned. Raw captures and build outputs stay local.

## Firmware modules and checks

`Core/Src/imu.c`, `mag.c`, `gnss.c`, `heading.c`, `north.c` and `matrix.c` contain the reusable
drivers/math. `imu_port.c` owns shield SPI2. `sensor_test.c` is the current combined
bench loop. `imu_test.c` retains the standalone onboard IMU/LED pointer test;
select it with `cmake --preset Debug -DFIND_HOME_SENSOR_TEST=OFF`, then rebuild.
Its viewer connection is 115200 baud without `--compass`; T walks all LED pixels.
Restore the combined bench with `-DFIND_HOME_SENSOR_TEST=ON` and rebuild.
The combined build generates calibration constants from the two JSON files;
Python 3 must be available during configuration/build. No extra Python packages
are needed for this export.

```powershell
python -m pip install -r tests/requirements.txt
python -m unittest discover -s tests -q
python tests/run_driver_test.py
```

Python tests cover estimator, calibration, protocol and capture behaviour. ARM
emulation tests the production C drivers, parsers and matrix/heading code; it
requires the ARM GCC tools on PATH. HAL/CMSIS, CubeMX configuration and generated
sources are retained. The inherited linker emits an RWX-segment warning.
The assignment PDF is not present; course requirements still need checking
before the final application/submission.
