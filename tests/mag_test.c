#include "mag.h"
#include <string.h>
#define CHECK(c) do { if (!(c)) return 3000 + __LINE__; } while (0)

static uint8_t registers[64];
static unsigned calls, fail_at, writes, waited;
static bool corrupt;
static bool reset_busy, ignore_clear;

static bool read_reg(void *context, uint8_t reg, uint8_t *data, uint16_t size) {
  (void)context;
  if (++calls == fail_at || reg+size > sizeof registers) return false;
  memcpy(data, registers+reg, size);
  if (reset_busy && reg == 0x0b) data[0] |= 0x80;
  if (reg == 9) registers[9] = 0; /* Status flags clear on read. */
  if (reg == 0x29) data[0] = 0xff; /* Undocumented readback must not reject hardware. */
  if (corrupt && reg == 0x0b) data[0] ^= 1;
  return true;
}

static bool write_reg(void *context, uint8_t reg, uint8_t value) {
  (void)context;
  if (++calls == fail_at || reg >= sizeof registers) return false;
  ++writes;
  if (reg == 0x0b && value & 0x80) return false; /* Normal setup must not assert reset. */
  if (reg == 0x0b && value == 0) {
    if (ignore_clear) return true;
    reset_busy = false;
  }
  if (reset_busy) return true; /* A held-reset device may ACK but ignore setup. */
  if (reg == 0x0a && (registers[reg] & 3) && (value & 3) &&
      (registers[reg] & 3) != (value & 3)) return false; /* Transition via suspend. */
  registers[reg] = value;
  return true;
}

static void delay(uint32_t ms) { waited += ms; }
static void reset(void) {
  memset(registers, 0, sizeof registers); registers[0] = 0x80;
  calls = fail_at = writes = waited = 0; corrupt = false;
  reset_busy = ignore_clear = false;
}

int mag_test(void) {
  mag_bus_t bus = {0, read_reg, write_reg, delay};
  mag_check_t check;
  reset(); CHECK(mag_init(&bus, &check) && check.operation == 0);
  CHECK(writes == 5 && waited == 10);
  CHECK(registers[0x29] == 6 && registers[0x0b] == 8 && registers[0x0a] == 5);
  unsigned operations = calls;
  for (unsigned i = 1; i <= operations; ++i) {
    reset(); fail_at = i; CHECK(!mag_init(&bus, &check));
    CHECK(check.operation == 'R' || check.operation == 'W');
  }
  reset(); registers[0] = 0xff;
  CHECK(!mag_init(&bus, &check) && writes == 0 && check.operation == 'V');
  CHECK(check.reg == 0 && check.expected == 0x80 && check.actual == 0xff);
  reset(); corrupt = true; CHECK(!mag_init(&bus, &check));
  CHECK(check.reg == 0x0b && check.operation == 'V');
  reset(); reset_busy = true; /* Reproduce hardware: reset does not self-clear. */
  CHECK(mag_init(&bus, &check));
  CHECK(!reset_busy && registers[0x0b] == 8 && registers[0x0a] == 5);
  reset(); registers[0x0a] = 3; /* Re-reference from an already running device. */
  CHECK(mag_init(&bus, &check) && registers[0x0a] == 5);
  reset(); reset_busy = ignore_clear = true;
  CHECK(!mag_init(&bus, &check) && check.operation == 'V' && check.reg == 0x0b);
  reset(); mag_sample_t sample;
  CHECK(mag_read(&bus, &sample) == 0 && calls == 1);
  fail_at = calls+1; CHECK(mag_read(&bus, &sample) == -1);
  fail_at = calls+2; registers[9] = 1;
  CHECK(mag_read(&bus, &sample) == -1);
  fail_at = 0;
  registers[9] = 2; CHECK(mag_read(&bus, &sample) == 2 && sample.status == 2);
  registers[9] = 3; CHECK(mag_read(&bus, &sample) == 2 && sample.status == 3);
  registers[1] = 0xff; registers[2] = 0x7f;
  registers[3] = 0; registers[4] = 0x80;
  registers[5] = registers[6] = 0xff;
  registers[9] = 1; CHECK(mag_read(&bus, &sample) == 1);
  CHECK(sample.xyz[0] == 32767 && sample.xyz[1] == -32768 && sample.xyz[2] == -1);
  CHECK(mag_read(&bus, &sample) == 0); /* Never return stale data as fresh. */
  for (unsigned word = 0; word < 65536; ++word) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      registers[axis*2+1] = (uint8_t)word;
      registers[axis*2+2] = (uint8_t)(word >> 8);
    }
    registers[9] = 1; CHECK(mag_read(&bus, &sample) == 1);
    int expected = word < 32768 ? (int)word : (int)word-65536;
    for (unsigned axis = 0; axis < 3; ++axis) CHECK(sample.xyz[axis] == expected);
  }
  return 0;
}
