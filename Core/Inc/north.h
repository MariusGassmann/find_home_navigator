#ifndef NORTH_H
#define NORTH_H
#include "heading.h"
#include "mag.h"
#include "matrix.h"

typedef struct {
  float accel_offset[3], accel_scale[3], mag_offset[3], mag_matrix[3][3], field;
} north_calibration_t;

typedef enum { NORTH_WAITING, NORTH_CALIBRATING, NORTH_ALIGNING,
               NORTH_TRACKING, NORTH_COASTING, NORTH_UNRELIABLE } north_state_t;
typedef struct {
  heading_t imu;
  const north_calibration_t *cal;
  float accel[3], gyro, compass, fused, last_yaw, candidate;
  uint32_t accepted, mag_tick, align_start, align_count;
  unsigned sector;
  bool aligned, mag_used, compass_valid;
} north_t;

void north_init(north_t *n, const north_calibration_t *cal);
void north_reference(north_t *n);
void north_imu(north_t *n, const imu_sample_t *sample, uint32_t tick);
void north_mag(north_t *n, const mag_sample_t *sample, uint32_t tick);
north_state_t north_state(const north_t *n, uint32_t tick);
const char *north_state_name(north_state_t state);
matrix_frame_t north_display(north_t *n, uint32_t tick);
#endif
