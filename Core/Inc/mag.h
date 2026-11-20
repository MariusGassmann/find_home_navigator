#ifndef MAG_H
#define MAG_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  void *context;
  bool (*read)(void *, uint8_t, uint8_t *, uint16_t);
  bool (*write)(void *, uint8_t, uint8_t);
  void (*delay_ms)(uint32_t);
} mag_bus_t;
typedef struct { int16_t xyz[3]; uint8_t status; } mag_sample_t;
typedef struct { char operation; uint8_t reg, expected, actual; } mag_check_t;
/* QMC5883P at 0x2C only: sensor XYZ, axis-sign register 0x06, 50 Hz,
   +/-8 gauss, 3750 counts/gauss = 37.5 counts/microtesla. Not calibrated. */
/* On failure, check identifies the read/write/mismatch and register. */
bool mag_init(const mag_bus_t *bus, mag_check_t *check);
bool mag_suspend(const mag_bus_t *bus);
/* -1: transport error, 0: no new sample, 1: fresh, 2: overflow (unusable). */
int mag_read(const mag_bus_t *bus, mag_sample_t *sample);
#endif
