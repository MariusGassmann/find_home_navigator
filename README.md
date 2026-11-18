# Find Home navigator

STM32 NUCLEO-L476RG with ETH PBL Shield rev 1.2 and LSM6DSV16BX IMU.
Firmware streams IMU measurements and drives a fixed-direction LED pointer using
a 3D gyro/gravity heading estimate. The laptop provides full 3D diagnostics.
GNSS, magnetic heading, home navigation and sleep are next.

## Build and flash

Use the existing STM32 Run & Debug configuration in VS Code, with CMake configure
and build presets set to `Debug`. The legacy target name `LAB04_I2C` is retained
for compatibility. Command-line equivalent:

```powershell
cmake --preset Debug
cmake --build --preset Debug
STM32_Programmer_CLI -c port=SWD -d build/Debug/LAB04_I2C.elf -v -rst
```

## Laptop viewer

```powershell
conda create -n find-home python=3.12 pip -y
conda activate find-home
python -m pip install -r tools/requirements.txt
python tools/imu_viewer.py --port COM3 --accel-cal calibration/imu_accel.json --log captures/imu.txt
```

Close other serial monitors before opening COM3. Rediscover the port with
`python -m serial.tools.list_ports -v` if needed. Place the board level and still,
press Enter in the viewer or on the shield, and hold still for two seconds.
+X points toward the POTI. Initial yaw is zero, not geographic north.
The board blinks its center LEDs while calibrating, then shows a green head toward
the starting direction and a red tail opposite. Normal pickup and tilt are allowed.
The pointer goes dark when the POTI points almost vertically, then resumes when
lowered. All red means clipping, lost samples or a hardware error; reference again.
Press `T` in the live viewer for a 4.8-second LED pixel/color test.

| Option | Effect |
| --- | --- |
| Default | Gyro integration, initial bias subtraction, gravity correction |
| `--gyro-only` | Disable gravity correction; keep bias subtraction |
| `--no-bias` | Disable gyro bias subtraction; combine with other flags |
| `--accel-cal calibration/imu_accel.json` | Apply this board's accelerometer calibration; otherwise off |
| `--axes=y,-x,z` | Sensor-to-board transform; this is the default |
| `--demo` instead of `--port COM3` | Synthetic three-axis motion; no hardware |
| `--replay captures/imu.txt` instead of `--port COM3` | Replay raw data; supply the original correction flags |
| `--log captures/imu.txt` | Save raw packets, settings and reference events; overwrites this filename |
| `--headless --seconds 60` | Timed capture without a window; shield Enter still works |

Escape closes the viewer. Logging also saves `<name>.summary.json` on close.
See [IMU details](docs/imu_test.md) and [calibration](docs/imu_calibration_results.md).

## Code and checks

- `Core/Src/imu.c`: portable sensor driver.
- `Core/Src/imu_test.c`: SPI2 transport, button and serial diagnostic.
- `Core/Src/heading.c`: level bias calibration, 3D attitude and pointer sector.
- `Core/Src/matrix.c`: eight pointer patterns and shield shift-register encoding.
- `tools/imu_model.py`: coordinate transform and attitude estimation.
- `tools/imu_viewer.py`: serial, simulation/replay, logging and 3D view.
- `tools/imu_calibration.py`: six-pose calibration extraction and fitting.
- `tests/`: synthetic attitude/calibration tests and ARM driver emulation.

```powershell
python -m pip install -r tests/requirements.txt
python -m unittest discover -s tests -v
python tests/run_driver_test.py
```

The ARM test requires `arm-none-eabi-gcc`, `objcopy` and `nm` on PATH. Firmware
builds require CMake, Ninja and the STM32 ARM toolchain. Calibration coefficients
and the calibration report are versioned; local raw captures and build outputs are ignored.
