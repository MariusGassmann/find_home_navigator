# Hardware and diagnostic protocol

NUCLEO-L476RG, STM32L476RGT6, 80 MHz. Shield documentation:
`PBLShield_Full_Documentation.PDF`, rev 1.2 sheets 3 and 5. Keep the course PDFs
and `.ioc` as references; custom SPI2/UART3 initialization lives outside CubeMX.

| Connection | MCU / connector |
| --- | --- |
| IMU SPI2 SCK / MISO / MOSI | PB13 / PB14 / PC3, AF5, mode 3, 1.25 MHz |
| IMU CS / matrix latch | PB12 active low / PB11 |
| Shield Enter | PH0 active low, 30 ms debounce |
| ST-LINK USB serial | USART2 PA2/PA3, COM3, 460800 8N1 in combined bench |
| GNSS module TX | PC11 USART3 RX, CN7 pin 2 (shield label WiFi TX) |
| GNSS module RX | Disconnected and insulated; no receiver commands sent |
| Module supply / ground | 5V CN7 pin 18 / GND CN7 pin 20 |
| Compass I2C1 SCL / SDA | PB8 CN10 pin 3 / PB9 CN10 pin 5 |

Disconnect power before rewiring. Follow connector pin-1 markings; underside
views are mirrored. Supply and signal voltages differ: this module accepts
3.3-5V supply; the measured SDA/SCL idle level is 3.3V. The ceramic antenna face
points toward clear sky. WiFi labels are just shield net names.

## Sensors and frames

IMU identity `0x71` matches LSM6DSV16BX. Both sensors run at 60 Hz:
acceleration +/-2 g (0.000061 g/count), gyro +/-500 deg/s (0.0175 deg/s/count).
Acceleration registers are Z,Y,X and are reordered by the driver.
Compass identity is QMC5883P at I2C 0x2C, register 0x00 = 0x80; verified twice
before configuration. It runs at 50 Hz, +/-8 G, 37.5 counts/uT. Driver setup uses
0x29=06, CTRL2=08, CTRL1=05 after suspend/clearing reset. Identity, configuration
readback, fresh-data and overflow checks remain active.

Board axes: +X buttons toward POTI, +Y left viewed along +X, +Z out of the
component side. IMU sensor-to-board is `(Y,-X,Z)`; calibrated compass-to-board is
`(Y,X,-Z)` for the current mounting. Calibrate in sensor axes before transforming.
Gyro yaw is counterclockwise relative to start. Magnetic heading is clockwise
from magnetic north; no declination correction is applied yet.

## Serial records

- `IMU,tick,sequence,reference,ax,ay,az,gx,gy,gz`: raw signed sensor XYZ counts.
- `MAG,tick,sequence,x,y,z,status`: raw signed compass counts; status bit 0 fresh,
  bit 1 overflow. Overflow samples are unusable.
- `RX,tick,offset,hex`: raw GNSS UART bytes, including binary UBX, at most 128 bytes
  per record. Offsets expose lost log data. `RX_LOSS` invalidates partial parsing.
- `GNSS,tick,state,...`: 1 Hz coordinates/quality and transport counters.

Ticks are uint32 milliseconds. Enter or ASCII `r` increments the IMU reference.
Startup prints IMU/compass initialization results; reset after opening a capture
to include them. An unavailable sensor does not imply the other sensors failed.

GNSS receives at 115200 8N1 through a 2048-byte interrupt-fed ring. The parser
accepts checksum-valid GGA qualities 1/2/4/5, coordinate bounds and fixes no older
than two seconds; it validates/skips UBX without decoding UBX positions.
`FIX` means usable reception, `NO_FIX` invalid/no fix, `STALE` old position,
`NO_DATA` no bytes in the interval. Printed old coordinates alone are not valid.
Latitude/longitude are degrees times 1e7; HDOP is dimensionless times 100.
`age_ms=4294967295` means no accepted fix. The navigator adds stricter
[home/position quality checks](navigation.md). UART errors/overruns/dropped bytes
should remain zero. GGA UTC is parsed to distinguish duplicate measurement epochs.

Analysis produces protocol counters, GNSS spread around the median (not absolute
accuracy), and magnetic field plots. `--segment` selects a boot, default last.

## IMU/LED diagnostic

The optional standalone IMU/matrix bench uses USB 115200. Enter collects at least
100 samples over two seconds: level within 8 degrees, acceleration 0.9-1.1 g,
gyro <=3 deg/s, per-axis noise <=0.3 deg/s. Gyro bias is fixed after referencing.
Quaternion integration uses gravity gain 1.5/s within the acceleration gate.
Clipping or a stream gap invalidates tracking; Enter references again. Gravity
corrects roll/pitch; yaw remains relative and drifts. Yaw/min is accumulated yaw
*60/elapsed time, meaningful as drift only while stationary.

Matrix logical row 0 faces POTI. Eight green-head/red-tail pointers select the
starting direction using negative yaw and 3-degree sector hysteresis. Amber
centre blinks during reference; all red means invalid tracking. The pointer
hides when forward is within 10 degrees of vertical. T walks all pixels green,
then red. SPI transfers deselect the IMU and pulse the latch only after a full
24-bit frame. Onboard heading uses raw gravity; laptop calibration flags affect
the laptop estimator only. Neither diagnostic mode sleeps; the default navigator
uses the saved calibration onboard and implements STOP2 as described above.

Sources: [ST IMU datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16bx.pdf),
[QST compass datasheet](https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf),
[HGLRC module](https://www.hglrc.com/products/m100-5883-gps),
[u-blox M10 protocol](https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf).
