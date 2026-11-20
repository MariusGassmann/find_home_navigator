# Find Home operation

RESET clears home and shows a steady red 3x3 X. The fourth matrix column is
used by arrows/progress; X, Z and arrival O use three columns. With the antenna
facing open sky, stand still, hold the board approximately level and press Enter.
Startup allows up to 30 degrees of tilt. Gyro bias still needs two continuous
seconds of rest (rate <=3 degrees/s, per-axis deviation <=0.3 degrees/s);
resting the assembly on a nonmetallic surface is easiest. Arrival O depends on
position only and does not confirm heading readiness: check STM32 TRACKING in
the viewer before walking. Its scrollable diagnostics remain visible at arrival,
including compass rejection reasons and IMU calibration/restart messages.
Amber progress means collecting home. Enter again cancels capture. Once saved,
green Z stays for 3 seconds, fades for 1 second, then the matrix goes dark and
the MCU enters STOP2. Walk away, press board Enter and hold level/still for about
2.3 seconds to measure a new gyro bias and align heading. The green arrowhead
points home; red marks the opposite end. Enter sleeps again. Home survives sleep
in RAM, but not RESET, loss of power or a firmware reflash.

## LED readout

| Display | Meaning |
| --- | --- |
| Steady red X | No home; press Enter to capture |
| Blinking amber X / amber progress | Waiting for good fixes / collecting home |
| Green Z, then fade and dark | Home saved or navigation paused; entering sleep |
| Blinking red X after wake | Position unavailable or poor quality |
| Amber X after wake | Heading not ready or unreliable; hold level/still |
| Green arrowhead, red tail | Navigate home |
| Green O | Arrival confirmed within the provisional 5 m radius |

Arrows show direction only: green tip, red tail, including brief gyro fallback.
Distance and COASTING status remain in laptop telemetry. The viewer shows the
onboard IMU bias progress as a compact inline elapsed/2 s readout;
old captures without progress telemetry use a labelled laptop estimate.

If an amber heading X persists, Enter to sleep, then Enter to wake and reference
again away from interference. The compass disagreement safeguard remains active.

## Position and angles

Home requires ten checksum-valid GGA fixes of quality 1, 2, 4 or 5, at least six
satellites, HDOP <=2.5, and age <=2 seconds. Repeated UTC epochs do not refresh
position age. Samples have distinct UTC epochs,
are at least 600 ms apart, and must lie within 5 m of the first sample and the
final mean. A position outlier restarts the window; poor/stale fixes clear it.
This normally takes about 6–10 seconds once reception is good. After 120 seconds
without success, return to red X; press Enter to retry. These checks measure
consistency, not absolute accuracy. Remain stationary during capture.

Navigation uses the same quality limits and a spherical great-circle distance
and initial true bearing (Earth radius 6,371,008.8 m). Arrival requires three new
fixes inside 5 m, at least 600 ms apart. The O clears beyond 8 m or on fix loss.
This hysteresis reduces flicker; GNSS errors can still exceed the arrival radius.
The device indicates direction, not a safe route or obstacles.

Frames remain +X toward POTI, +Y left, +Z up. Heading/bearing are clockwise from
north; the target angle is `magnetic_heading + declination - true_bearing`,
wrapped to [-180,180] degrees counterclockwise from board forward. Eight arrow
sectors retain 3 degrees of hysteresis. Tilt-compensated heading and magnetic
checks are described in [north_demo.md](north_demo.md).

**Declination defaults to 0 degrees (correction off).** Set the local east-positive
value from [swisstopo's calculator](https://geodesy.geo.admin.ch/declination/index.html?lang=en)
or [NOAA](https://www.ncei.noaa.gov/products/world-magnetic-model) for the location/date:
`cmake --preset Debug -DFIND_HOME_DECLINATION_DEG=VALUE`, then rebuild/flash.
The configured correction appears in startup/NAV telemetry. With zero, the
uncompensated declination remains a direction error; sensor calibration does not
remove it. No geographic or date assumption is built into the firmware.

## Power and wake

During active operation, polling idles with WFI between interrupts. In sleep,
the IMU ODRs are set to zero, compass enters suspend, display and shield status
LEDs are off, serial reception and peripheral clocks stop, and SysTick pauses.
STOP2 retains RAM. PH0 falling-edge EXTI wakes the MCU, followed by a 30 ms press
check, 80 MHz clock restoration, sensor reinitialization, fresh GNSS reception
and a new heading reference. A held button must be released before sleeping;
one press is consumed on wake. Serial Enter cannot wake the MCU. The Z fade uses
250 Hz software PWM of complete SPI frames while sensor acquisition is stopped.

The GNSS remains powered/tracking because its RX wire is disconnected and no
supply switch is fitted. The Nucleo ST-LINK and other shield circuits also remain
powered. This reduces MCU/sensor power; whole-board savings need measurement.
HAL ticks exclude time asleep. Debug sessions can lose connection in STOP2;
test normal operation after flashing with the debugger detached. A powerbank
must continue supplying power or RAM home will be lost.

Implementation follows ST's [NUCLEO-L476RG STOP2 example](https://github.com/STMicroelectronics/STM32CubeL4/tree/master/Projects/NUCLEO-L476RG/Examples/PWR/PWR_STOP2),
the supplied RM0351 and [IMU datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16bx.pdf).

## First hardware test

1. Configure NAVIGATOR, build/flash, start the README viewer with a new log name,
   then RESET. Outdoors, wait for reception and check red X before Enter.
2. Press Enter while still/level. Verify progress, 3-second Z, smooth fade, then
   dark. The viewer should report SLEEPING; serial silence during sleep is normal.
3. Remain at home, press board Enter, hold level/still for 2.3 seconds. Confirm
   fresh GNSS/heading and green O after three fixes. Do not RESET after saving home.
4. Walk 20–40 m away. Check green head points back, and rotate the board to check
   the arrow counter-rotates. Return slowly; test the 5 m indication empirically.
5. Enter to sleep and wake again; verify home coordinates stay unchanged. Repeat
   on powerbank power without the laptop. Changing power sources may reset home:
   save a new home after the powerbank is connected.

Return the capture filename, whether sleep/wake worked, any persistent X, and
the approximate physical distance when O first appeared. `NAV` records contain
state, home/current coordinates (e7 degrees), distance (cm; -1 unavailable), true
bearing and relative angle (millidegrees; INT32_MIN unavailable), sample count,
satellites/HDOP, declination and red/green LED bitmasks. FUSE/IMU/MAG/RX/GNSS logs
and plots remain available. Calibration and previous captures are preserved.
