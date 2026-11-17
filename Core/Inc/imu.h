#ifndef IMU_H
#define IMU_H
#include <stdbool.h>
#include <stdint.h>

/* Transport returns false on error. Addresses are unmodified register numbers. */
typedef struct {
  void *context;
  bool (*read)(void *, uint8_t, uint8_t *, uint16_t);
  bool (*write)(void *, uint8_t, uint8_t);
  void (*delay_ms)(uint32_t);
} imu_bus_t;
typedef struct { int16_t accel[3], gyro[3]; } imu_sample_t;
typedef struct { uint8_t id, ctrl1, ctrl2, ctrl3, ctrl6, ctrl8; } imu_config_t;
/* LSM6DSV16BX: 60 Hz, +/-2 g (0.061 mg/LSB), +/-500 dps (17.5 mdps/LSB). */
bool imu_init(const imu_bus_t *bus, imu_config_t *config);
/* Returns -1 on transport error, 0 if not ready, 1 for a fresh sample. */
int imu_read(const imu_bus_t *bus, imu_sample_t *sample);
#endif
