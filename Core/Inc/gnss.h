#ifndef GNSS_H
#define GNSS_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  char line[160];
  unsigned used;
  uint32_t good, bad, fixes, updated;
  uint32_t ubx_good, ubx_bad;
  uint16_t ubx_left;
  uint8_t ubx_phase, ck_a, ck_b, received_a;
  int32_t lat_e7, lon_e7;
  unsigned quality, satellites, hdop_x100;
  bool collecting, valid;
} gnss_t;

/* Feed UART bytes; true means line contains a checksum-valid NMEA sentence.
   Position currently comes from GGA only. No magnetic heading or home capture. */
bool gnss_feed(gnss_t *g, uint8_t byte, uint32_t now);
bool gnss_fresh(const gnss_t *g, uint32_t now);
#endif
