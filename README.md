# Find Home navigator

STM32 NUCLEO-L476RG, ETH PBL Shield rev 1.2, LSM6DSV16BX IMU and HGLRC
M100-5883 GNSS/QMC5883P compass. The default firmware captures home, sleeps,
wakes and points home using GNSS and fused gyro/compass heading. Navigation is
tested in ARM simulation; physical sleep/wake and outdoor accuracy need checking.

## Build and flash

Use the existing STM32 Run & Debug GUI with the `Debug` preset. The course target
name `LAB04_I2C` stays unchanged. Terminal equivalent:

```powershell
cmake --preset Debug -DFIND_HOME_MODE=NAVIGATOR
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -d build/Debug/LAB04_I2C.elf -v -rst
```

## Viewer

From the project root, using the existing Conda environment:

```powershell
conda activate find-home
python -m pip install -r tools/requirements.txt
python tools/imu_viewer.py --port COM3 --compass --accel-cal calibration/imu_accel.json --mag-cal calibration/mag.json --mag-axes=y,x,-z --log captures/find-home.txt
```

Close other serial monitors first. Outdoors, RESET shows a red X. Hold level
and still and press Enter: ten good fixes save home, then green Z for three
seconds, a one-second fade, and sleep. Walk away; press **board Enter**, hold
level/still for about 2.3 seconds, then follow the green arrowhead. Enter again
sleeps; RESET erases home. No laptop is required during use.

The viewer shows state, home/position, distance and STM32 IMU calibration progress. Its
dial points home; the existing heading comparison and XYZ field plots remain.
Click **GNSS map** for a blank north-up latitude/longitude plot with live position,
saved home, a 5 m ring and the recorded trail. It needs no internet or map tiles.
Grey marks the last fix after reception stops; trails have gaps through sleep
and fix loss. Amber positions have a receiver fix below navigation quality limits.
The 3D board shows relative gyro/gravity orientation. Board +X points toward
POTI. See [operation, LED meanings and field test](docs/navigation.md),
[hardware/frames](docs/hardware.md) and [tilt compensation](docs/compass_heading.md).

Useful options: `--gyro-only` disables gravity correction; `--no-bias` disables
gyro bias subtraction; omit `--accel-cal` to compare raw acceleration. These
options affect the laptop estimate; onboard fusion always uses saved calibration,
gravity correction and measured gyro bias.
The control bar labels these settings **Laptop reference**; the FUSE comparison,
home dial and LEDs still use the firmware's results. Replaying the same raw log
with different options allows a controlled comparison without reflashing.
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
drivers/math. `navigator.c` owns home/quality/navigation logic; `power.c` owns
sleep/wake. `imu_port.c` owns shield SPI2; `app.c` connects the modules.
Select the [north-pointing diagnostic](docs/north_demo.md) with
`cmake --preset Debug -DFIND_HOME_MODE=NORTH`, then rebuild.
`imu_test.c` retains the standalone IMU/LED pointer test;
select it with `-DFIND_HOME_MODE=IMU`, then rebuild.
Its viewer connection is 115200 baud without `--compass`; T walks all LED pixels.
Restore the product with `-DFIND_HOME_MODE=NAVIGATOR` and rebuild.
The combined build generates calibration constants from the two JSON files;
Python 3 must be available during configuration/build. No extra Python packages
are needed for this export.

```powershell
python -m pip install -r tests/requirements.txt
python -m unittest discover -s tests -q
python tests/run_driver_test.py
python tests/run_driver_test.py captures/gnss_stationary.txt
```

Python tests cover estimator, calibration, protocol and capture behaviour. ARM
emulation tests the production C drivers, parsers, heading and full navigator
workflow; the optional capture argument replays real UART data into home capture. It
requires the ARM GCC tools on PATH. HAL/CMSIS, CubeMX configuration and generated
sources are retained. The inherited linker emits an RWX-segment warning.
The assignment PDF is not present; course requirements still need checking
before the final application/submission.
