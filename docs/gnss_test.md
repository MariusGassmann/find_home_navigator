# GNSS / compass discovery

Default firmware mode: `FIND_HOME_SENSOR_TEST=ON`. Build/flash with the existing
VS Code Debug preset and target `LAB04_I2C`. To return to the IMU/matrix test:

```powershell
cmake --preset Debug -DFIND_HOME_SENSOR_TEST=OFF
cmake --build --preset Debug
```

Use `ON` and rebuild to restore this diagnostic. The LED matrix is unused here.
UART reception, parser and compass ID checks are separate from the IMU estimator.

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
the USB COM3 diagnostic port at 115200; USART3 receives GPS at 115200 8N1.

## Capture

Close the IMU viewer/other serial monitors. From the existing Conda environment:

```powershell
conda activate find-home
python tools/sensor_capture.py --port COM3 --seconds 180 --log captures/gnss.txt
```

Press shield RESET after the capture opens. Log files are overwritten. The GPS
should be outdoors with the ceramic antenna facing up and clear sky; its label
side is not the ceramic antenna. Allow several minutes for the first fix. LEDs
indicate activity/power; use the decoded fix status to establish position validity.

Return `captures/gnss.txt`, especially startup scan/ID, `RX first32`, and status
lines before/after acquisition. No Enter calibration or compass rotation yet.

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
With a serial terminal, ASCII `v` toggles raw checksum-valid NMEA output; `i`
repeats compass discovery. A repeated full scan/raw output may increase buffering
pressure; inspect the counters. The capture script does not send commands.

## Scope and sources

Position parsing currently supports GGA with any two-character talker ID (GP/GN
etc.), checksum checking, N/S/E/W, bounds and fresh-data checks. Qualities 1/2/4/5
are accepted; estimated/manual/simulated fixes are excluded. This is a reception
test, not the later home-position quality gate. No receiver settings are changed.
If bytes arrive without NMEA, `first32` helps identify a different baud/protocol;
binary UBX is not decoded yet. See [u-blox M10 protocol](https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf), sections 2.1/2.5/2.7.5.

The compass scan prints 7-bit addresses (the shield temperature sensor may also
respond). Read-only probes compare two ID reads against QST documentation:

| Candidate | Address | ID register | Expected ID |
| --- | --- | --- | --- |
| QMC5883L | 0x0D | 0x0D | 0xFF |
| QMC5883P | 0x2C | 0x00 | 0x80 |

Sources: [QST QMC5883L Rev B](https://www.qstcorp.com/upload/pdf/202512/13-52-04%20QMC5883L%20Datasheet%20Rev.%20B.pdf),
[QST QMC5883P Rev E](https://www.qstcorp.com/upload/pdf/202512/2C939E5AA0704285BC3BE71132B8629B.pdf).
An ACK or ID match is evidence for a candidate, not full validation of its register
map. No compass configuration writes, magnetic calibration or heading yet.
Hardware reception/identity remains to be tested; ARM tests cover the production
parser, including 1000 noisy-stream recovery trials. Run `tests/run_driver_test.py`.
