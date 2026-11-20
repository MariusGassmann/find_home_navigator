"""Execute the production IMU, matrix and heading code on an emulated Cortex-M4."""
from pathlib import Path
import argparse
import shutil
import struct
import subprocess
import sys

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import (UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0,
                              UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('captures', nargs='*', type=Path, help='Optionally replay RX hex captures through production gnss.c')
args = parser.parse_args()

root = Path(__file__).resolve().parents[1]
output = root / 'build' / 'driver-test'
output.mkdir(parents=True, exist_ok=True)
gcc = shutil.which('arm-none-eabi-gcc')
if not gcc:
    sys.exit('Put arm-none-eabi-gcc on PATH first')
objcopy = shutil.which('arm-none-eabi-objcopy')
nm = shutil.which('arm-none-eabi-nm')
elf, binary = output/'test.elf', output/'test.bin'
script = output/'test.ld'
script.write_text('ENTRY(test_main)\nSECTIONS { . = 0x10000; .text : { *(.text*) *(.rodata*) } '
                  '.data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } end = .; }\n')
subprocess.run([sys.executable, str(root/'tools/export_calibration.py'),
                str(root/'calibration/imu_accel.json'), str(root/'calibration/mag.json'),
                str(output/'north_calibration.h')], check=True)
subprocess.run([gcc, '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-O1',
                '-ffreestanding', '-fno-builtin', '-nostdlib', '--specs=nano.specs',
                '-Wall', '-Wextra', '-Werror',
                '-I', str(output), str(root/'Core/Src/north.c'), str(root/'tests/north_test.c'),
                str(root/'Core/Src/navigator.c'), str(root/'tests/navigator_test.c'),
                '-I', str(root/'Core/Inc'), str(root/'Core/Src/imu.c'),
                str(root/'Core/Src/matrix.c'), str(root/'Core/Src/heading.c'),
                str(root/'Core/Src/gnss.c'), str(root/'tests/gnss_test.c'),
                str(root/'Core/Src/mag.c'), str(root/'tests/mag_test.c'),
                str(root/'tests/imu_driver_test.c'), str(root/'tests/matrix_heading_test.c'),
                '-T', str(script), '-o', str(elf), '-lm', '-lc', '-lnosys', '-lgcc'], check=True)
subprocess.run([objcopy, '-O', 'binary', str(elf), str(binary)], check=True)
symbols = subprocess.check_output([nm, str(elf)], text=True)
entry = next(int(line.split()[0], 16) for line in symbols.splitlines() if line.endswith(' test_main'))
cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0x10000, 0x10000)
cpu.mem_write(0x10000, binary.read_bytes())
cpu.mem_map(0x20000000, 0x10000)
cpu.reg_write(UC_ARM_REG_SP, 0x20010000)
cpu.reg_write(UC_ARM_REG_LR, 0x1f001)
cpu.emu_start(entry | 1, 0x1f000, timeout=60000000, count=300000000)
if cpu.reg_read(UC_ARM_REG_PC) != 0x1f000:
    sys.exit('ARM driver test timed out')
result = cpu.reg_read(UC_ARM_REG_R0)
if result:
    if result >= 5000:
        sys.exit(f'ARM navigator CHECK failed at tests/navigator_test.c:{result-5000}')
    if result >= 4000:
        sys.exit(f'ARM north fusion CHECK failed at tests/north_test.c:{result-4000}')
    if result >= 3000:
        sys.exit(f'ARM magnetometer CHECK failed at tests/mag_test.c:{result-3000}')
    if result >= 2000:
        sys.exit(f'ARM GNSS CHECK failed at tests/gnss_test.c:{result-2000}')
    if result >= 1000:
        sys.exit(f'ARM matrix/heading CHECK failed at tests/matrix_heading_test.c:{result-1000}')
    sys.exit(f'ARM driver CHECK failed at tests/imu_driver_test.c:{result}')
print('PASS: ARM driver identity/reset/readback/error/ready/axis-order tests; '
      'all 65,536 signed patterns on all six channels')
print('PASS: matrix wiring/colors/directions; heading bias, counter-rotation, hysteresis, '
      'wrap, tilted turns, pickup acceleration, vertical recovery, gravity correction, '
      'reference noise/tilt rejection, clipping/gap rejection and re-reference')
print('PASS: GGA checksums, coordinates, hemispheres, limits, fix loss, freshness/wrap, '
      'malformed fields, long/partial/binary frames and 1,000 noisy-stream recoveries')
print('PASS: QMC5883P identity, reset, configuration readback, fresh/overflow/error paths; '
      'all 65,536 signed patterns on each axis')
print('PASS: calibrated north fusion: bias drift, tilted turns, north crossing, '
      'spike rejection, compass fallback/recovery, LED states and clock wrap')
print('PASS: navigator GGA-to-home workflow, geometry/declination, duplicate/poor/stale '
      'fix rejection, averaging/outliers, arrival hysteresis, LED states, sleep/wake and clock wrap')

if args.captures:
    replay = next(int(line.split()[0], 16) for line in symbols.splitlines() if line.endswith(' gnss_replay'))
    nav_replay = next(int(line.split()[0], 16) for line in symbols.splitlines() if line.endswith(' navigator_replay'))

    def feed(data=b'', tick=0, reset=0, hook=replay):
        if data: cpu.mem_write(0x20000000, data)
        for register, value in ((UC_ARM_REG_R0, 0x20000000), (UC_ARM_REG_R1, len(data)),
                                (UC_ARM_REG_R2, tick), (UC_ARM_REG_R3, reset),
                                (UC_ARM_REG_LR, 0x1f001), (UC_ARM_REG_SP, 0x20010000)):
            cpu.reg_write(register, value)
        cpu.emu_start(hook | 1, 0x1f000, count=1000000)
        if cpu.reg_read(UC_ARM_REG_PC) != 0x1f000: sys.exit('ARM replay timed out')
        return struct.unpack('<5I', cpu.mem_read(cpu.reg_read(UC_ARM_REG_R0), 20))

    for capture in args.captures:
        counters = feed(reset=1)
        home = feed(reset=1,hook=nav_replay)
        offset, chunks, boot = 0, 0, 0
        for line in capture.read_text(errors='replace').splitlines():
            if '# SENSOR ' in line or '# GNSS discovery:' in line:
                if chunks: print(f'REPLAY {capture.name} boot={boot}: NMEA/bad/UBX/UBX_bad/fixes={counters}', flush=True)
                boot += 1; offset = chunks = 0; counters = feed(reset=1)
                home = feed(reset=1,hook=nav_replay)
            elif line.startswith('# RX_LOSS'):
                counters = feed(reset=2)
                home = feed(reset=2,hook=nav_replay)
            elif line.startswith('RX,'):
                try:
                    _, tick, start, hex_data = line.split(',')
                    data = bytes.fromhex(hex_data)
                    if len(data) > 128: raise ValueError('oversize RX')
                    if int(start) != offset:
                        feed(reset=2); feed(reset=2,hook=nav_replay)
                    counters = feed(data, int(tick))
                    home = feed(data,int(tick),hook=nav_replay)
                    offset = int(start)+len(data); chunks += 1
                except ValueError:
                    feed(reset=2)
                    feed(reset=2,hook=nav_replay)
        print(f'REPLAY {capture.name} boot={boot}: NMEA/bad/UBX/UBX_bad/fixes={counters}', flush=True)
        lat, lon = (v if v < 0x80000000 else v-0x100000000 for v in home[2:4])
        print(f'HOME REPLAY {capture.name}: saved={bool(home[1])}, samples={home[4]}, lat_e7={lat}, lon_e7={lon}',flush=True)
