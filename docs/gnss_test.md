# GNSS / QMC5883P bench test

Default firmware mode: `FIND_HOME_SENSOR_TEST=ON`. Build/flash with the existing
VS Code Debug preset and target `LAB04_I2C`. To return to the IMU/matrix test:

```powershell
cmake --preset Debug -DFIND_HOME_SENSOR_TEST=OFF
cmake --build --preset Debug
```

Use `ON` and rebuild to restore this diagnostic. The LED matrix is unused here.
UART reception, parser and compass driver are separate from the IMU estimator.

## Connections

Disconnect USB before wiring. Use the verified module signals, not cable colours.
The module takes 3.3–5V according to [HGLRC](https://www.hglrc.com/collections/gps-module/products/m100-5883-gps).
Measured on this unit with 5V input: SDA/SCL idle 3.3V, RX 3.3V, TX average 2.5V.
The TX average is consistent with serial activity, not a peak-level measurement.

| Module | Nucleo Morpho connector | MCU / shield label |
| --- | --- | --- |
| 5V | CN7 pin 18 | +5V |
| GND | CN7 pin 20 | GND |
| TX | CN7 pin 2 | PC11 / WiFi TX; USART3 RX |
| RX | Leave disconnected and insulated | Receive-only test; no GPS commands |
| SCL | CN10 pin 3 | PB8 / I2C SCL |
| SDA | CN10 pin 5 | PB9 / I2C SDA |

Shield J1/J2 correspond to Nucleo CN7/CN10. Follow connector pin-1 markings;
the underside view is mirrored. WiFi labels do not enable WiFi. USART2 remains
the USB COM3 diagnostic port, now **460800** in this mode. USART3 receives GPS at
115200 8N1. The IMU viewer/mode still uses USB 115200. Wiring has not changed.

## Capture

Close the IMU viewer/other serial monitors. From the existing Conda environment:

```powershell
conda activate find-home
python -m pip install -r tools/plot_requirements.txt
python tools/sensor_capture.py --port COM3 --seconds 180 --log captures/gnss_stationary.txt
python tools/sensor_analysis.py captures/gnss_stationary.txt --output captures/gnss-analysis
```

Press shield RESET after the capture opens. The time limit starts when the script
opens the port, not when RESET is pressed. Use `--seconds 0` to record until
Ctrl+C; the log footer records duration/stop/error. Log files are overwritten. The GPS
should be outdoors with the ceramic antenna facing up and clear sky; its label
side is not the ceramic antenna. Allow several minutes for the first fix. LEDs
indicate activity/power; use the decoded fix status to establish position validity.

For this first recording, keep the assembly stationary on wood/plastic outdoors.
The capture script saves all data but hides high-volume RX/MAG lines in the terminal.
Return the capture filename; no need to copy thousands of terminal lines.

First check `# MAG init=1` and increasing `mag_samples` in a stationary 10–20 s
capture. If init=0, return that line; no rotations are useful until initialization
works. `op=R/W` means a transport failure; `op=V` means readback mismatch.
Logs include register, expected and actual values,
plus ID/CTRL1/CTRL2 snapshots. An uninitialized compass no longer looks
healthy merely because its read-error counter is zero.

For a second recording, use `--seconds 0 --log captures/mag_rotations.txt` and
stop with Ctrl+C when comfortable. There is no need to finish movements to a timer.
Keep the GPS module fixed relative to the board; note which way its connector
edge points relative to POTI. Stay away from steel, laptop speakers and magnets:

1. Hold level and still for 10 s; +X is toward POTI.
2. Over 20 s, turn the whole assembly clockwise twice while keeping it level.
3. Over 20 s, roll it about +X through a complete turn, slowly.
4. Over 20 s, pitch it about +Y through a complete turn, slowly.
5. Return to the initial level direction and hold still for 20 s.

Keep cables relaxed and use normal hand movements. This checks axis response and
repeatability; it does not apply a compass calibration or establish magnetic north.
Run analysis on this file with `--output captures/mag-analysis`.

`GNSS` status reports cumulative byte/NMEA/GGA-fix counts, bytes per second,
checksum/framing/field errors (`bad`), coordinates in degrees times 10^7,
quality, satellites used, HDOP times 100, and position age in milliseconds.
Age `4294967295` means no accepted fix yet. HDOP is dimensionless, not metres.
Historical coordinates can remain printed after loss; only `FIX` is usable.

- `NO_DATA`: no UART bytes during the last reporting interval.
- `NO_FIX`: no currently valid decoded GGA fix, including a reported fix loss.
- `STALE`: last accepted position is older than two seconds.
- `FIX`: checksum-valid GGA with supported GNSS quality and recent position.

`uart_errors` and ring-buffer `dropped` should remain zero. Their change discards
partial input and invalidates the position until another valid GGA. A 2048-byte
interrupt-fed ring keeps reception independent of USB printing and I2C checks.
`fe`, `ne`, `ore` distinguish UART framing, noise and overrun flags. Raw `RX`
records contain MCU tick, byte offset and hex data, batched at most 128 bytes or
about 10 ms. Errored bytes/discarded backlog are marked by `RX_LOSS`, not silently
treated as a complete stream. Byte offsets detect missing USB log records.
ASCII `i` repeats discovery; `m` retries compass initialization. Repeated I2C scans
can cause buffering pressure. The capture script does not send GPS commands.

The MCU now checks/skips UBX frames and reports separate `ubx`/`ubx_bad` counters;
`bad` describes NMEA failures. Earlier v2 firmware mistook some binary payloads
for malformed NMEA. No UBX position is used yet.

Analysis saves `summary.json`, `gnss.png` (metres east/north of the median, plus
quality/fix-loss timeline), and `magnetic.png` when compass samples exist.
It independently checks raw NMEA checksums/GGA fields and UBX frame checksums.
Binary UBX explains some bad-line counts in earlier firmware; a bad-line count
alone does not prove electrical corruption. Older captures use
the 1 Hz status coordinates and cannot diagnose individual rejected messages.
Each MCU boot is analyzed separately; default is the last, selectable by `--segment`.
Spread/R95 is relative to the observed median, not absolute positioning accuracy.
Do not interpret the compass-motion recording as stationary GNSS accuracy.

## Scope and sources

Position parsing currently supports GGA with any two-character talker ID (GP/GN
etc.), checksum checking, N/S/E/W, bounds and fresh-data checks. Qualities 1/2/4/5
are accepted; estimated/manual/simulated fixes are excluded. This is a reception
test, not the later home-position quality gate. No receiver settings are changed.
The MCU does not decode binary UBX positions. See [u-blox M10 protocol](https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf), sections 2.1/2.5/2.7.5/3.4.

Hardware reported address 0x2C and ID register 0x00 = 0x80 twice, matching QMC5883P.
The driver requires that ID before writing, enters suspend, explicitly clears
CTRL2 (including reset/self-test), then sets axis-sign register 0x29
to 0x06, CTRL2 to 0x08 (+/-8 gauss, set/reset enabled), CTRL1 to 0x05 (normal,
50 Hz), and checks ID/control-register readback. Register 0x29 is prescribed as
a setup write, but its readback is not defined; it is not compared.

The v4 sequence follows [QST QMC5883P Rev E](https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf),
sections 7.1/9, using 50 Hz instead of the example's 200 Hz. QST's normal startup
and [Adafruit begin()](https://github.com/adafruit/Adafruit_QMC5883P/blob/main/Adafruit_QMC5883P.cpp)
do not require a software-reset request. In `compass_check2.txt`, the earlier
reset sequence left CTRL2=0x80 after 100 ms. This differs from the reset-bit
self-clear assumption in [PX4](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/magnetometer/qmc5883p/QMC5883P.cpp).
We now clear the previous reset request explicitly and check the documented
normal-mode configuration. Hardware measurements remain required to establish
successful sampling. Regression tests model a latched reset and warm startup
from a running mode, as well as failures to clear/configure the device.

`MAG,tick_ms,sequence,x,y,z,status` reports signed raw **sensor** XYZ. Nominal
conversion: divide counts by 37.5 for microtesla. Bit 0 means fresh; bit 1 means
overflow, which analysis excludes. `mag_errors`/`mag_overflows` should stay zero.
No board mounting transform, magnetic calibration or compass heading is applied.
New raw-compass hardware behavior remains to be checked. ARM tests cover the
driver's identity/readback/error paths and every signed 16-bit value on each axis.

## Combined IMU and compass view

Build/flash the default sensor firmware, close any capture script, then run:

```powershell
conda activate find-home
python tools/imu_viewer.py --port COM3 --compass --accel-cal calibration/imu_accel.json --log captures/combined.txt
```

Press board RESET after connecting. Place the shield level and still, press Enter
in the viewer or on the board, and wait two seconds for the gyro reference. The
existing accelerometer calibration remains valid. No compass calibration is
applied: the three field projections show sensor XY/XZ/YZ in nominal microtesla,
with the latest sample highlighted and ~30 seconds of history. Their axes are
not yet transformed into the shield frame. A compass calibration dataset needs
broad three-dimensional orientation coverage with the module fixed in its final
mounting; simply restarting the viewer does not require repeating that calibration.

The 3D board still uses gyro/gravity only; magnetic samples do not correct yaw.
GNSS raw records, compass and IMU samples share one log and USB connection at
460800 baud. Existing gyro-only, no-bias and accelerometer calibration options
remain available. The LED pointer/pixel test belongs to the selectable standalone
IMU/matrix firmware (`FIND_HOME_SENSOR_TEST=OFF`, viewer without `--compass`).

For independent compass calibration captures and corrected plots, see
[compass calibration](compass_calibration.md).
