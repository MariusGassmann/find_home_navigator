#ifndef IMU_PORT_H
#define IMU_PORT_H
#include "imu.h"

/* Shield SPI2 transport shared by both diagnostic modes. */
imu_bus_t imu_port_init(void);
void imu_port_matrix(uint8_t bytes[3]);
#endif
