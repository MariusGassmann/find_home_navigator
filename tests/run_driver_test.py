"""Compile the production C driver for ARM, then run fake-bus tests on Unicorn."""
from pathlib import Path
import shutil
import subprocess
import sys

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_SP

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
                  '.data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } }\n')
subprocess.run([gcc, '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-O1',
                '-ffreestanding', '-fno-builtin', '-nostdlib', '-Wall', '-Wextra', '-Werror',
                '-I', str(root/'Core/Inc'), str(root/'Core/Src/imu.c'),
                str(root/'tests/imu_driver_test.c'), '-T', str(script), '-o', str(elf)], check=True)
subprocess.run([objcopy, '-O', 'binary', str(elf), str(binary)], check=True)
symbols = subprocess.check_output([nm, str(elf)], text=True)
entry = next(int(line.split()[0], 16) for line in symbols.splitlines() if line.endswith(' test_main'))
cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0x10000, 0x10000)
cpu.mem_write(0x10000, binary.read_bytes())
cpu.mem_map(0x20000000, 0x10000)
cpu.reg_write(UC_ARM_REG_SP, 0x20010000)
cpu.reg_write(UC_ARM_REG_LR, 0x1f001)
cpu.emu_start(entry | 1, 0x1f000, timeout=30000000, count=30000000)
if cpu.reg_read(UC_ARM_REG_PC) != 0x1f000:
    sys.exit('ARM driver test timed out')
result = cpu.reg_read(UC_ARM_REG_R0)
if result:
    sys.exit(f'ARM driver CHECK failed at tests/imu_driver_test.c:{result}')
print('PASS: ARM driver identity/reset/readback/error/ready/axis-order tests; '
      'all 65,536 signed patterns on all six channels')
