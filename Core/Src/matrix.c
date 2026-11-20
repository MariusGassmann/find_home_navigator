#include "matrix.h"
#include <math.h>

unsigned matrix_sector(float angle, unsigned previous) {
  float delta = remainderf(angle-previous*45.0f, 360);
  if (fabsf(delta) <= 25.5f) return previous;
  angle = fmodf(angle, 360);
  if (angle < 0) angle += 360;
  return (unsigned)((angle+22.5f)/45) % 8;
}

matrix_frame_t matrix_pointer(unsigned sector) {
  /* Forward, forward-left, left, back-left, back, back-right, right, forward-right. */
  static const uint16_t head[8] = {0x006, 0x001, 0x010, 0x100, 0x600, 0x800, 0x080, 0x008};
  return (matrix_frame_t){head[(sector + 4) % 8], head[sector % 8]};
}

void matrix_encode(matrix_frame_t frame, uint8_t bytes[3]) {
  uint32_t bits = 0;
  for (unsigned pixel = 0; pixel < 12; ++pixel) {
    /* Sheet 2: green occupies bits 0..11, red 12..23. Sheet 13: y=2 faces POTI. */
    unsigned wire = (2 - pixel / 4) * 4 + pixel % 4;
    if (frame.green & (1U << pixel)) bits |= 1UL << wire;
    if (frame.red & (1U << pixel)) bits |= 1UL << (wire + 12);
  }
  bytes[0] = (uint8_t)(bits >> 16); /* U3, furthest from serial input */
  bytes[1] = (uint8_t)(bits >> 8);  /* U2 */
  bytes[2] = (uint8_t)bits;         /* U1 */
}
