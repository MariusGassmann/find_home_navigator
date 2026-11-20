#include "heading.h"
#include "matrix.h"
#include <math.h>
#include <stddef.h>

static void restart_window(heading_t *h) {
  h->count = 0;
  for (unsigned i = 0; i < 3; ++i) { h->sum[i] = 0; h->squares[i] = 0; }
}

void heading_reference(heading_t *h) {
  *h = (heading_t){.state = HEADING_CALIBRATING};
}

void heading_update(heading_t *h, const imu_sample_t *s, uint32_t tick) {
  heading_update_calibrated(h, s, tick, NULL);
}

void heading_update_calibrated(heading_t *h, const imu_sample_t *s, uint32_t tick,
                               const float accel[3]) {
  uint32_t dt = tick - h->tick;
  bool gap = h->timed && (dt == 0 || dt > 100);
  h->tick = tick;
  h->timed = true;
  if (h->state != HEADING_CALIBRATING && h->state != HEADING_TRACKING) return;
  float a2 = 0, g2 = 0;
  bool clipped = false;
  for (unsigned i = 0; i < 3; ++i) {
    float a = s->accel[i] * 0.000061f, g = s->gyro[i] * 0.0175f;
    float corrected = accel ? accel[i] : a;
    a2 += corrected*corrected; g2 += g*g;
    if (a > 1.99f || a < -1.99f || g > 490 || g < -490) clipped = true;
  }
  float z = accel ? accel[2] : s->accel[2] * 0.000061f;
  /* Bias needs rest, not perfect level; permit up to 30 degrees of tilt. */
  bool level = a2 >= .81f && a2 <= 1.21f && z > 0 && z*z >= .75f*a2;
  if (h->state == HEADING_CALIBRATING) {
    if (gap || clipped || !level || g2 > 9) { restart_window(h); return; }
    if (!h->count) h->start = tick;
    ++h->count;
    for (unsigned i = 0; i < 3; ++i) {
      int32_t raw = s->gyro[i];
      h->sum[i] += raw;
      h->squares[i] += (uint32_t)(raw*raw);
    }
    if ((uint32_t)(tick - h->start) < 2000 || h->count < 100) return;
    for (unsigned i = 0; i < 3; ++i) {
      float mean = (float)h->sum[i] / h->count;
      /* Variance limit corresponds to 0.3 degrees/s standard deviation. */
      if ((float)h->squares[i] / h->count - mean*mean > 293.88f) {
        restart_window(h); return;
      }
    }
    for (unsigned i = 0; i < 3; ++i) h->bias_raw[i] = (float)h->sum[i] / h->count;
    h->q[0] = 1;
    if (accel) {
      float norm = sqrtf(a2);
      h->q[0] = sqrtf((1 + z/norm)*.5f);
      h->q[1] = -accel[0]/(2*norm*h->q[0]);
      h->q[2] = -accel[1]/(2*norm*h->q[0]);
    }
    h->yaw_deg = 0; h->sector = 0; h->direction_valid = true;
    h->state = HEADING_TRACKING;
    return;
  }
  if (gap || clipped) { h->state = HEADING_INVALID; return; }
  /* Sensor -> board: (Y, -X, Z). Quaternion maps board to reference. */
  float rate[3] = {s->gyro[1] - h->bias_raw[1],
                  h->bias_raw[0] - s->gyro[0], s->gyro[2] - h->bias_raw[2]};
  for (unsigned i = 0; i < 3; ++i) rate[i] *= .00030543262f; /* rad/s/count */
  float w = h->q[0], x = h->q[1], y = h->q[2], v = h->q[3];
  if (a2 >= .81f && a2 <= 1.21f) {
    float norm = sqrtf(a2);
    float a[3] = {(accel ? accel[1] : s->accel[1]*.000061f)/norm,
                 -(accel ? accel[0] : s->accel[0]*.000061f)/norm, z/norm};
    float up[3] = {2*(x*v-w*y), 2*(y*v+w*x), 1-2*(x*x+y*y)};
    /* Measured up cross predicted up; skip correction during large acceleration. */
    for (unsigned i = 0; i < 3; ++i)
      rate[i] += 1.5f*(a[(i+1)%3]*up[(i+2)%3] - a[(i+2)%3]*up[(i+1)%3]);
  }
  float speed = sqrtf(rate[0]*rate[0] + rate[1]*rate[1] + rate[2]*rate[2]);
  float half = speed*dt*.0005f;
  float scale = speed > 1e-8f ? sinf(half)/speed : dt*.0005f;
  float c = cosf(half), dx = rate[0]*scale, dy = rate[1]*scale, dz = rate[2]*scale;
  h->q[0] = w*c - x*dx - y*dy - v*dz;
  h->q[1] = w*dx + x*c + y*dz - v*dy;
  h->q[2] = w*dy - x*dz + y*c + v*dx;
  h->q[3] = w*dz + x*dy - y*dx + v*c;
  float norm = 0;
  for (unsigned i = 0; i < 4; ++i) norm += h->q[i]*h->q[i];
  norm = sqrtf(norm);
  for (unsigned i = 0; i < 4; ++i) h->q[i] /= norm;
  w = h->q[0]; x = h->q[1]; y = h->q[2]; v = h->q[3];
  float forward_x = 1-2*(y*y+v*v), forward_y = 2*(x*y+w*v);
  /* Near vertical, forward has no useful horizontal bearing. Keep integrating. */
  h->direction_valid = forward_x*forward_x + forward_y*forward_y >= .030154f;
  if (!h->direction_valid) return;
  h->yaw_deg = atan2f(forward_y, forward_x)*57.29578f;
  h->sector = matrix_sector(-h->yaw_deg,h->sector);
}
