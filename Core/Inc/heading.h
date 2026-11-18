#ifndef HEADING_H
#define HEADING_H
#include "imu.h"

typedef enum { HEADING_WAITING, HEADING_CALIBRATING, HEADING_TRACKING, HEADING_INVALID } heading_state_t;
typedef struct {
  heading_state_t state;
  uint32_t tick, start, count;
  int32_t sum[3];
  uint32_t squares[3];
  float bias_raw[3], q[4], yaw_deg;
  unsigned sector;
  bool timed, direction_valid;
} heading_t;

void heading_reference(heading_t *heading);
/* Level reference, then 3D gyro integration with gravity correction. */
void heading_update(heading_t *heading, const imu_sample_t *sample, uint32_t tick);
#endif
