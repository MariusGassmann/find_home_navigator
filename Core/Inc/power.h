#ifndef POWER_H
#define POWER_H
#include "imu.h"
#include "mag.h"

/* Caller has stopped GNSS reception. Fade Z, suspend sensors, STOP2 until a
   debounced PH0 press. Caller reinitializes sensors/GNSS and heading afterwards. */
void power_sleep(const imu_bus_t *imu, const mag_bus_t *mag);
#endif
