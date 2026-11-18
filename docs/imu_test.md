# IMU diagnostic reference

## Hardware

NUCLEO-L476RG, STM32L476RGT6, 80 MHz. PBL Shield rev 1.2 sheets 3 and 5
establish the wiring and identify U7 as LSM6DSV16BX. The connected device reports
WHO_AM_I `0x71`, matching the [ST datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16bx.pdf).

| Signal | Pin / configuration |
| --- | --- |
| SPI2 SCK / MISO / MOSI | PB13 / PB14 / PC3, AF5, mode 3, 1.25 MHz |
| IMU chip select | PB12, active low |
| Shared LED-matrix latch | PB11, pulsed after a complete 24-bit frame |
| Shield Enter | PH0, active low, external pull-up, 30 ms debounce |
| USB ST-LINK virtual COM | USART2, PA2 TX / PA3 RX, 115200 8N1 |
| I2C1, reserved for external sensors | PB8 SCL / PB9 SDA, original timing retained |

The driver resets the IMU, checks readback and enables both sensors at 60 Hz.
Accelerometer: +/-2 g, 0.061 mg/count. Gyro: +/-500 degrees/s, 17.5 mdps/count.
Expected banner: `ID=71 CTRL1=05 CTRL2=05 CTRL3=44 CTRL6=02 CTRL8=00`.
Acceleration registers are Z,Y,X; the driver reorders them into sensor XYZ.

SPI2 initialization is owned by `imu_test.c`. CubeMX owns the clock, GPIO and
USART2 setup. Preserve the custom source list/SPI HAL definition in the root
CMake file when regenerating. The original I2C1 setup is retained for external
sensors; the temperature test is removed. ST SPI HAL files come from STM32Cube
FW_L4 V1.18.1.

## Frames, units and protocol

Board +X points from buttons toward POTI; +Z points out of the component side;
+Y points left when looking along +X with the component side up. The frame is
right-handed. Board XYZ = (sensor Y, -sensor X, sensor Z), applied to both sensors.
Positive yaw is counterclockwise viewed from above. The quaternion maps board to
reference vectors; displayed Euler angles use ZYX yaw/pitch/roll in degrees.

```text
IMU,tick_ms,sequence,reference,ax,ay,az,gx,gy,gz
```

Measurements are signed raw counts in sensor XYZ. Timestamp is the MCU tick at
read completion, with 1 ms resolution. Sequence and reference are uint32 counters.
A debounced shield press or received ASCII `r` increments reference and starts
onboard heading calibration. Keyboard Enter also sends `r` in live mode. Samples are
read only when acceleration and gyro are both ready. Transport failure or no
fresh data for 500 ms enters the firmware error handler.

## Estimation and validity

Enter collects at least 100 samples over two seconds. Calibration requires
0.9-1.1 g, top-up tilt <=8 degrees, gyro magnitude <=3 degrees/s and each gyro
axis standard deviation <=0.3 degrees/s. Invalid motion restarts the window.
The gyro mean becomes a fixed bias; orientation and elapsed time reset to zero.

Gyro rates are integrated with quaternions. Default gravity correction uses a
1.5/s proportional gain when acceleration magnitude is 0.9-1.1 g. Optional
accelerometer calibration is applied in the sensor frame before mounting.
Raw sensor clipping checks remain active regardless of calibration.

- Gravity constrains roll/pitch, not yaw. Linear acceleration can corrupt tilt.
- Gyro-only permits drift on all axes. Slow yaw cannot be distinguished from bias
  during the initial stationary calibration.
- Tracking stops near +/-500 degrees/s or +/-2 g, on missing/reordered packets,
  restart, or a sample interval above 100 ms. Enter references again.
- The viewer marks data older than 0.5 s stale. Counter wrap is supported.
- Displayed gyro is before bias subtraction; displayed acceleration is after
  optional calibration. `raw norm` is always uncalibrated.
- Yaw wraps at +/-180 degrees; accumulated yaw tracks signed turns. Average
  yaw/min is accumulated yaw * 60 / elapsed seconds, meaningful as drift only
  when stationary. Euler yaw is ill-conditioned near +/-90-degree pitch.

## Validation and remaining work

Driver identity/readback and approximately 60.5 Hz streaming were verified on
hardware. Operator tests confirmed axis signs, low short-term stationary yaw
change with bias subtraction, and tilt recovery with gravity correction.
Six-pose calibration and 28 Python tests cover the current diagnostic baseline;
ARM emulation checks all 65,536 signed patterns on every channel.

## LED fixed-direction test

The 4x3 logical display has row 0 toward POTI and columns left-to-right looking
forward. A green head and opposite red tail select one of eight directions.
Positive sectors run forward, forward-left, left, back-left, back, back-right,
right, forward-right. Pointer angle is negative onboard yaw, with 3-degree
hysteresis at the 45-degree sector boundaries. Direction is therefore coarse.

Enter requires two seconds and >=100 consecutive level/still samples, using the
reference checks above. All three gyro biases are subtracted before quaternion
integration in the board frame. Gravity correction uses gain 1.5/s when raw
acceleration is 0.9-1.1 g; outside that range, gyro tracking continues alone.
There is no tracking tilt limit. Pointer yaw comes from the horizontal projection
of board +X; within 10 degrees of vertical, the pointer goes dark while orientation
continues tracking, then resumes automatically. This is relative heading without
a magnetic reference. Linear acceleration inside the gravity gate can still
disturb tilt. Onboard gravity uses raw acceleration; the optional six-pose
calibration and viewer flags currently affect laptop estimation only.

| Matrix state | Meaning |
| --- | --- |
| Off | Waiting for Enter, or forward is almost vertical during tracking |
| Blinking red+green center pair | Collecting level/still reference |
| Green head / red tail | Tracking the starting direction |
| All red | Clipping, >100 ms sample gap, or hardware failure; reference/reset needed |

`T` in the live viewer sends ASCII `t`: walk all 12 pixels green, then red,
200 ms each, row-by-row starting toward POTI. Raw sensor streaming continues.
The test ends off; press Enter to reference afterward. Enter cancels a running
pixel test. `# HEADING` serial diagnostics report state, yaw in millidegrees,
sensor gyro Z bias in millidegrees/s, sector and horizontal-direction validity
once per second while tracking.

All matrix transfers deselect the IMU, write U3/U2/U1 bytes MSB first, then pulse
the latch. IMU transactions can shift the internal registers but never latch them,
so the visible frame persists. ARM tests cover all 24 color outputs, frame byte
order, pointer direction, hysteresis, bias, wrap, rejection and re-reference.

This is sufficient to develop navigation logic. Temperature dependence, precise
motion accuracy, magnetic heading and power consumption remain uncharacterized.
The bench stays active; no sleep mode is implemented. The inherited linker
script emits an RWX-segment warning. The assignment is still missing from `docs`;
its requirements must be checked before finalizing the application.
