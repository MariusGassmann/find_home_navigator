#ifndef IMU_PORT_H
#define IMU_PORT_H
#include "imu.h"
#include "matrix.h"

/* Shield SPI2 transport shared by both diagnostic modes. */
imu_bus_t imu_port_init(void);
void imu_port_display(matrix_frame_t frame);
#endif
