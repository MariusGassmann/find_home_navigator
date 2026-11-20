#include "north.h"
#include <math.h>

static float wrap(float a) {
  while (a >= 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}
static float positive(float a) { a = wrap(a); return a < 0 ? a+360 : a; }

void north_init(north_t *n, const north_calibration_t *cal) {
  *n = (north_t){.cal = cal};
}
void north_reference(north_t *n) {
  north_init(n, n->cal);
  heading_reference(&n->imu);
}

void north_imu(north_t *n, const imu_sample_t *s, uint32_t tick) {
  float a[3];
  for (unsigned i = 0; i < 3; ++i)
    a[i] = (s->accel[i]*.000061f-n->cal->accel_offset[i])*n->cal->accel_scale[i];
  heading_update_calibrated(&n->imu, s, tick, a);
  n->accel[0] = a[1]; n->accel[1] = -a[0]; n->accel[2] = a[2];
  if (n->imu.state != HEADING_TRACKING) return;
  float *q = n->imu.q;
  if (!n->imu.direction_valid || 1-2*(q[1]*q[1]+q[2]*q[2]) <= 0) {
    n->aligned = false; n->align_count = 0; return;
  }
  /* Quaternion yaw is CCW; compass headings are clockwise from magnetic north. */
  float delta = -wrap(n->imu.yaw_deg-n->last_yaw);
  n->last_yaw = n->imu.yaw_deg;
  if (n->aligned) {
    n->gyro = positive(n->gyro+delta);
    n->fused = positive(n->fused+delta);
  }
}

void north_mag(north_t *n, const mag_sample_t *s, uint32_t tick) {
  uint32_t dt = tick-n->mag_tick;
  n->mag_tick = tick;
  n->mag_used = n->compass_valid = false;
  float a2 = 0;
  for (unsigned i = 0; i < 3; ++i) a2 += n->accel[i]*n->accel[i];
  if ((s->status & 3) != 1 || n->imu.state != HEADING_TRACKING ||
      (uint32_t)(tick-n->imu.tick) > 100 || !n->imu.direction_valid ||
      a2 < .81f || a2 > 1.21f) goto rejected;
  float corrected[3] = {0}, raw[3];
  for (unsigned i = 0; i < 3; ++i) raw[i] = s->xyz[i]/37.5f-n->cal->mag_offset[i];
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j) corrected[i] += n->cal->mag_matrix[i][j]*raw[j];
  /* Verified compass -> board mounting: (Y, X, -Z). */
  float b[3] = {corrected[1], corrected[0], -corrected[2]};
  float b2 = b[0]*b[0]+b[1]*b[1]+b[2]*b[2], field2 = n->cal->field*n->cal->field;
  if (b2 < .64f*field2 || b2 > 1.44f*field2) goto rejected;
  float *q = n->imu.q;
  float u[3] = {2*(q[1]*q[3]-q[0]*q[2]), 2*(q[2]*q[3]+q[0]*q[1]),
                1-2*(q[1]*q[1]+q[2]*q[2])};
  if (u[2] <= 0) goto rejected;
  float f[3] = {1-u[0]*u[0], -u[0]*u[1], -u[0]*u[2]};
  float norm = sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
  if (norm < .173648f) goto rejected;
  for (unsigned i = 0; i < 3; ++i) f[i] /= norm;
  float left[3] = {u[1]*f[2]-u[2]*f[1], u[2]*f[0]-u[0]*f[2], u[0]*f[1]-u[1]*f[0]};
  float bx = 0, by = 0;
  for (unsigned i = 0; i < 3; ++i) { bx += b[i]*f[i]; by += b[i]*left[i]; }
  if (bx*bx+by*by < 25) goto rejected;
  n->compass = positive(atan2f(by, bx)*57.29578f);
  n->compass_valid = true;
  if (!n->aligned) {
    float candidate = wrap(n->compass+n->imu.yaw_deg);
    if (!n->align_count || dt > 100 || fabsf(wrap(candidate-n->candidate)) > 10) {
      n->align_count = 0; n->align_start = tick;
    }
    n->candidate = candidate;
    ++n->align_count;
    if ((uint32_t)(tick-n->align_start) < 200 || n->align_count < 8) return;
    n->gyro = n->fused = n->compass;
    n->last_yaw = n->imu.yaw_deg;
    n->sector = (unsigned)((n->fused+22.5f)/45) % 8;
    n->aligned = true;
  } else {
    float error = wrap(n->compass-n->fused);
    if (fabsf(error) > 30) goto rejected;
    /* Five-second time constant; never integrate a long missing interval. */
    if (dt > 100) dt = 20;
    n->fused = positive(n->fused + (1-expf(-(float)dt*.0002f))*error);
  }
  n->accepted = tick;
  n->mag_used = true;
  return;
rejected:
  n->align_count = 0;
}

north_state_t north_state(const north_t *n, uint32_t tick) {
  if (n->imu.state == HEADING_WAITING) return NORTH_WAITING;
  if (n->imu.state == HEADING_INVALID || !n->imu.timed ||
      (uint32_t)(tick-n->imu.tick) > 100) return NORTH_UNRELIABLE;
  if (n->imu.state == HEADING_CALIBRATING) return NORTH_CALIBRATING;
  if (!n->aligned) return NORTH_ALIGNING;
  uint32_t age = tick-n->accepted;
  return age > 2000 ? NORTH_UNRELIABLE : age > 500 ? NORTH_COASTING : NORTH_TRACKING;
}
const char *north_state_name(north_state_t state) {
  static const char *names[] = {"WAITING", "CALIBRATING", "ALIGNING", "TRACKING", "COASTING", "UNRELIABLE"};
  return names[state];
}
matrix_frame_t north_display(north_t *n, uint32_t tick) {
  north_state_t state = north_state(n, tick);
  if (state == NORTH_CALIBRATING || state == NORTH_ALIGNING) {
    uint16_t dot = tick/250 % 2 ? 0x20 : 0;
    return (matrix_frame_t){dot, dot};
  }
  if (state != NORTH_TRACKING && state != NORTH_COASTING)
    return (matrix_frame_t){.red = 0x525}; /* 3x3 X; fourth column off. */
  float delta = wrap(n->fused-n->sector*45.0f);
  if (fabsf(delta) > 25.5f) n->sector = (unsigned)((n->fused+22.5f)/45) % 8;
  matrix_frame_t frame = matrix_pointer(n->sector);
  if (state == NORTH_COASTING) frame.red |= frame.green;
  return frame;
}
