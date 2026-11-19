#include "mag.h"

static bool expect(const mag_bus_t *b, uint8_t reg, uint8_t value, mag_check_t *check) {
  *check = (mag_check_t){'R', reg, value, 0};
  if (!b->read(b->context, reg, &check->actual, 1)) return false;
  check->operation = 'V'; /* V = register value mismatch; R/W = transport error. */
  return check->actual == value;
}

static bool write_reg(const mag_bus_t *b, uint8_t reg, uint8_t value, mag_check_t *check) {
  *check = (mag_check_t){'W', reg, value, 0};
  return b->write(b->context, reg, value);
}

bool mag_init(const mag_bus_t *b, mag_check_t *check) {
  if (!expect(b, 0, 0x80, check) || !expect(b, 0, 0x80, check)) return false;
  /* QST section 7.1 normal-mode setup does not require a software reset.
     Enter suspend and clear reset/self-test left by an earlier firmware run. */
  if (!write_reg(b, 0x0a, 0, check) || !write_reg(b, 0x0b, 0, check)) return false;
  b->delay_ms(10);
  if (!expect(b, 0x0a, 0, check) || !expect(b, 0x0b, 0, check)) return false;
  /* The datasheet prescribes this write but does not define 0x29 readback. */
  if (!write_reg(b, 0x29, 0x06, check)) return false;
  const uint8_t registers[] = {0x0b, 0x0a}, values[] = {0x08, 0x05};
  for (unsigned i = 0; i < sizeof registers; ++i) {
    if (!write_reg(b, registers[i], values[i], check)) return false;
  }
  for (unsigned i = 0; i < sizeof registers; ++i)
    if (!expect(b, registers[i], values[i], check)) return false;
  *check = (mag_check_t){0};
  return true;
}

int mag_read(const mag_bus_t *b, mag_sample_t *s) {
  *s = (mag_sample_t){0};
  if (!b->read(b->context, 0x09, &s->status, 1)) return -1;
  if (!(s->status & 3)) return 0;
  if (s->status & 1) {
    uint8_t raw[6];
    if (!b->read(b->context, 1, raw, sizeof raw)) return -1;
    for (unsigned i = 0; i < 3; ++i) {
      unsigned word = raw[2*i] | ((unsigned)raw[2*i+1] << 8);
      s->xyz[i] = (int16_t)(word < 32768 ? (int)word : (int)word-65536);
    }
  }
  return s->status & 2 ? 2 : 1;
}
