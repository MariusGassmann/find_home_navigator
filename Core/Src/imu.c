#include "imu.h"

static int16_t signed_le(const uint8_t *p) {
  uint16_t value = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  return (int16_t)(value < 32768U ? (int32_t)value : (int32_t)value - 65536);
}

bool imu_init(const imu_bus_t *b, imu_config_t *c) {
  *c = (imu_config_t){0};
  if (!b->read(b->context, 0x0f, &c->id, 1) || c->id != 0x71)
    return false;
  if (!b->write(b->context, 0x12, 0x01)) return false; /* SW_RESET */
  uint8_t reset = 1;
  for (unsigned i = 0; i < 100 && (reset & 1); ++i) {
    b->delay_ms(1);
    if (!b->read(b->context, 0x12, &reset, 1)) return false;
  }
  if (reset & 1) return false;
  /* BDU + address increment; scales first, then enable both sensors. */
  const uint8_t reg[] = {0x12, 0x15, 0x17, 0x10, 0x11};
  const uint8_t val[] = {0x44, 0x02, 0x00, 0x05, 0x05};
  for (unsigned i = 0; i < sizeof reg; ++i) {
    uint8_t actual;
    if (!b->write(b->context, reg[i], val[i]) ||
        !b->read(b->context, reg[i], &actual, 1) || actual != val[i])
      return false;
  }
  c->ctrl1 = c->ctrl2 = 0x05;
  c->ctrl3 = 0x44; c->ctrl6 = 0x02; c->ctrl8 = 0;
  b->delay_ms(100); /* Allow startup/filter settling before diagnostics. */
  return true;
}

bool imu_suspend(const imu_bus_t *b) {
  for (uint8_t reg = 0x10; reg <= 0x11; ++reg) {
    uint8_t value;
    if (!b->write(b->context, reg, 0) || !b->read(b->context, reg, &value, 1) || value != 0)
      return false;
  }
  return true;
}

int imu_read(const imu_bus_t *b, imu_sample_t *s) {
  uint8_t status, raw[12];
  if (!b->read(b->context, 0x1e, &status, 1)) return -1;
  if ((status & 3) != 3) return 0;
  if (!b->read(b->context, 0x22, raw, sizeof raw)) return -1;
  for (unsigned i = 0; i < 3; ++i) {
    s->gyro[i] = signed_le(raw + 2 * i);
    /* Unlike many other LSM6 parts, acceleration is stored Z, Y, X. */
    s->accel[i] = signed_le(raw + 10 - 2 * i);
  }
  return 1;
}
