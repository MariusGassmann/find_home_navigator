#ifndef MATRIX_H
#define MATRIX_H
#include <stdint.h>

/* Logical 4x3 display: row 0 toward POTI; columns left to right looking forward.
   Bit (row*4+column) controls one LED. Positive sectors rotate CCW from forward. */
typedef struct { uint16_t red, green; } matrix_frame_t;
matrix_frame_t matrix_pointer(unsigned sector);
/* Degrees CCW from forward, with 3-degree hysteresis around 45-degree sectors. */
unsigned matrix_sector(float angle, unsigned previous);
void matrix_encode(matrix_frame_t frame, uint8_t bytes[3]);
#endif
