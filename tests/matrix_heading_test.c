#include "heading.h"
#include "matrix.h"
#include <math.h>
#define CHECK(condition) do { if (!(condition)) return 1000 + __LINE__; } while (0)

static imu_sample_t still(void) {
  return (imu_sample_t){.accel = {0, 0, 16393}, .gyro = {100, -14, 28}};
}

static heading_t calibrated(uint32_t start) {
  heading_t h;
  imu_sample_t s = still();
  heading_reference(&h);
  for (unsigned i = 0; i <= 100; ++i) heading_update(&h, &s, start + i*20);
  return h;
}

static bool close_to(float a, float b, float tolerance) {
  return a-b <= tolerance && b-a <= tolerance;
}

static void set_yaw(heading_t *h, float degrees) {
  float half = degrees * .00872664626f;
  h->q[0] = cosf(half); h->q[1] = h->q[2] = 0; h->q[3] = sinf(half);
}

/* Analytic Rz(yaw)*Rx(roll): at fixed roll, a world-Z turn has body Y/Z rates.
   This catches the old gyro-Z-only approximation (78 rather than 90 degrees). */
static imu_sample_t rolled(float roll, float roll_rate, float yaw_rate) {
  float angle = roll * .01745329252f;
  imu_sample_t s = still();
  s.accel[0] = (int16_t)lroundf(-sinf(angle)/.000061f);
  s.accel[2] = (int16_t)lroundf(cosf(angle)/.000061f);
  s.gyro[0] -= (int16_t)lroundf(yaw_rate*sinf(angle)/.0175f);
  s.gyro[1] += (int16_t)lroundf(roll_rate/.0175f);
  s.gyro[2] += (int16_t)lroundf(yaw_rate*cosf(angle)/.0175f);
  return s;
}

static imu_sample_t pitched(float pitch, float pitch_rate, float yaw_rate) {
  float angle = pitch * .01745329252f;
  imu_sample_t s = still();
  s.accel[1] = (int16_t)lroundf(-sinf(angle)/.000061f);
  s.accel[2] = (int16_t)lroundf(cosf(angle)/.000061f);
  s.gyro[0] -= (int16_t)lroundf(pitch_rate/.0175f);
  s.gyro[1] -= (int16_t)lroundf(yaw_rate*sinf(angle)/.0175f);
  s.gyro[2] += (int16_t)lroundf(yaw_rate*cosf(angle)/.0175f);
  return s;
}

int matrix_heading_test(void) {
  uint8_t bytes[3];
  const unsigned wire[12] = {8, 9, 10, 11, 4, 5, 6, 7, 0, 1, 2, 3};
  for (unsigned pixel = 0; pixel < 12; ++pixel) {
    for (unsigned color = 0; color < 2; ++color) {
      matrix_frame_t frame = {0};
      if (color) frame.red = (uint16_t)(1U << pixel);
      else frame.green = (uint16_t)(1U << pixel);
      matrix_encode(frame, bytes);
      uint32_t actual = ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[2];
      CHECK(actual == (1UL << (wire[pixel] + 12*color)));
    }
  }
  matrix_encode((matrix_frame_t){0}, bytes);
  CHECK(bytes[0] == 0 && bytes[1] == 0 && bytes[2] == 0);
  matrix_encode((matrix_frame_t){0xfff, 0xfff}, bytes);
  CHECK(bytes[0] == 255 && bytes[1] == 255 && bytes[2] == 255);
  matrix_encode(matrix_pointer(0), bytes);
  CHECK(bytes[0] == 0 && bytes[1] == 0x66 && bytes[2] == 0);
  for (unsigned sector = 0; sector < 32; ++sector) {
    matrix_frame_t p = matrix_pointer(sector), opposite = matrix_pointer(sector+4);
    CHECK((p.red & p.green) == 0 && p.red && p.green);
    CHECK(p.red == opposite.green && p.green == opposite.red);
  }

  imu_sample_t s = still();
  heading_t h = {0};
  heading_update(&h, &s, 100);
  CHECK(h.state == HEADING_WAITING);
  h = calibrated(0);
  CHECK(h.state == HEADING_TRACKING && h.sector == 0 && h.yaw_deg == 0);
  CHECK(close_to(h.bias_raw[0], 100, .0001f));
  CHECK(close_to(h.bias_raw[1], -14, .0001f));
  CHECK(close_to(h.bias_raw[2], 28, .0001f));
  for (unsigned i = 1; i <= 3000; ++i) heading_update(&h, &s, 2000+i*20);
  CHECK(h.state == HEADING_TRACKING && h.yaw_deg == 0);
  /* A CCW board turn must move the pointer right; a CW turn moves it left. */
  for (int sign = -1; sign <= 1; sign += 2) {
    h = calibrated(0);
    s = still(); s.gyro[2] += (int16_t)(sign*1714); /* Approximately +/-30 dps. */
    for (unsigned i = 1; i <= 150; ++i) heading_update(&h, &s, 2000+i*20);
    CHECK(h.state == HEADING_TRACKING);
    CHECK(close_to(h.yaw_deg, sign*90.0f, .02f));
    CHECK(h.sector == (sign > 0 ? 6U : 2U));
    s.gyro[2] = 28;
    heading_update(&h, &s, 5020);
    CHECK(close_to(h.yaw_deg, sign*90.0f, .02f));
  }
  s = still(); h = calibrated(0);
  set_yaw(&h, -24); heading_update(&h, &s, 2020); CHECK(h.sector == 0);
  set_yaw(&h, -26); heading_update(&h, &s, 2040); CHECK(h.sector == 1);
  set_yaw(&h, -23); heading_update(&h, &s, 2060); CHECK(h.sector == 1);
  set_yaw(&h, -19); heading_update(&h, &s, 2080); CHECK(h.sector == 0);
  set_yaw(&h, -179); heading_update(&h, &s, 2100); CHECK(h.sector == 4);
  set_yaw(&h, 179); heading_update(&h, &s, 2120); CHECK(h.sector == 4);
  heading_reference(&h); CHECK(h.state == HEADING_CALIBRATING && h.count == 0);

  h = calibrated(0); heading_update(&h, &s, 2101); CHECK(h.state == HEADING_INVALID);
  h = calibrated(0); heading_update(&h, &s, 2000); CHECK(h.state == HEADING_INVALID);
  h = calibrated(0xfffffc00U);
  CHECK(h.state == HEADING_TRACKING);
  heading_update(&h, &s, 0xfffffc00U + 2020U);
  CHECK(h.state == HEADING_TRACKING);
  h = calibrated(0); s.gyro[0] = 30000;
  heading_update(&h, &s, 2020); CHECK(h.state == HEADING_INVALID);
  h = calibrated(0); s = still(); s.accel[2] = 32767;
  heading_update(&h, &s, 2020); CHECK(h.state == HEADING_INVALID);
  h = calibrated(0); s = still(); s.accel[0] = 6000; s.accel[2] = 15200;
  heading_update(&h, &s, 2020); CHECK(h.state == HEADING_TRACKING);
  heading_reference(&h);
  for (unsigned i = 0; i < 200; ++i) heading_update(&h, &s, i*20);
  CHECK(h.state == HEADING_TRACKING); /* Roughly 22-degree startup tilt is OK. */
  s = rolled(40, 0, 0); heading_reference(&h);
  for (unsigned i = 0; i < 200; ++i) heading_update(&h, &s, i*20);
  CHECK(h.state == HEADING_CALIBRATING && h.count == 0);
  s = still(); heading_reference(&h);
  for (unsigned i = 0; i <= 100; ++i) {
    s.gyro[2] = (int16_t)(28 + (i%2 ? 40 : -40));
    heading_update(&h, &s, i*20);
  }
  CHECK(h.state == HEADING_CALIBRATING && h.count == 0);
  s = still();
  for (unsigned i = 101; i <= 201; ++i) heading_update(&h, &s, i*20);
  CHECK(h.state == HEADING_TRACKING);

  for (unsigned axis = 0; axis < 2; ++axis) {
    imu_sample_t (*pose)(float, float, float) = axis ? pitched : rolled;
    for (int sign = -1; sign <= 1; sign += 2) {
      h = calibrated(0);
      for (unsigned i = 1; i <= 50; ++i) {
        s = pose(sign*i*.6f, sign*30, 0);
        heading_update(&h, &s, 2000+i*20);
        CHECK(h.state == HEADING_TRACKING && h.direction_valid);
      }
      for (unsigned i = 1; i <= 150; ++i) {
        s = pose(sign*30, 0, sign*30);
        heading_update(&h, &s, 3000+i*20);
      }
      CHECK(h.state == HEADING_TRACKING && h.direction_valid);
      CHECK(close_to(h.yaw_deg, sign*90, .4f));
      CHECK(h.sector == (sign > 0 ? 6U : 2U));
      /* Return level without resetting, preserving the starting direction. */
      for (unsigned i = 1; i <= 50; ++i) {
        s = pose(sign*(30-i*.6f), -sign*30, 0);
        heading_update(&h, &s, 6000+i*20);
      }
      CHECK(close_to(h.yaw_deg, sign*90, .4f));
      CHECK(close_to(h.q[1], 0, .01f) && close_to(h.q[2], 0, .01f));
    }
  }
  /* A pickup acceleration/free-fall skips gravity feedback, not gyro updates. */
  h = calibrated(0); s = still(); s.gyro[2] += 1714;
  for (unsigned i = 1; i <= 150; ++i) {
    s.accel[0] = 10000; s.accel[1] = 5000;
    s.accel[2] = i%2 ? 20000 : 0; /* Norm above 1.1 g or below 0.9 g. */
    heading_update(&h, &s, 2000+i*20);
  }
  CHECK(h.state == HEADING_TRACKING && close_to(h.yaw_deg, 90, .02f));
  /* Vertical forward temporarily hides bearing; lowering recovers automatically. */
  h = calibrated(0); s = still(); s.accel[0] = s.accel[1] = s.accel[2] = 0;
  s.gyro[0] -= 1714; /* Board +Y, approximately 30 dps. */
  for (unsigned i = 1; i <= 150; ++i) heading_update(&h, &s, 2000+i*20);
  CHECK(h.state == HEADING_TRACKING && !h.direction_valid);
  s.gyro[0] += 3428;
  for (unsigned i = 1; i <= 150; ++i) heading_update(&h, &s, 5000+i*20);
  CHECK(h.state == HEADING_TRACKING && h.direction_valid && h.sector == 0);
  /* Gravity feedback removes tilt error even without gyro motion. */
  h = calibrated(0); s = rolled(30, 0, 0);
  for (unsigned i = 1; i <= 250; ++i) heading_update(&h, &s, 2000+i*20);
  CHECK(h.state == HEADING_TRACKING && close_to(h.q[1], sinf(.261799388f), .002f));
  s = still();
  for (unsigned i = 1; i <= 250; ++i) heading_update(&h, &s, 7000+i*20);
  CHECK(close_to(h.q[1], 0, .002f));
  return 0;
}
