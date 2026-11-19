#include "gnss.h"
#include <stdio.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) return 2000 + __LINE__; } while (0)

static bool sentence(gnss_t *g, const char *body, uint32_t now) {
  unsigned checksum = 0;
  for (const char *p = body; *p; ++p) checksum ^= (unsigned char)*p;
  char line[240];
  snprintf(line, sizeof line, "$%s*%02X\r\n", body, checksum);
  bool result = false;
  for (const char *p = line; *p; ++p) result = gnss_feed(g, (uint8_t)*p, now);
  return result;
}

static const char *fix = "GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";

/* ARM replay hook: 1 resets a boot, 2 discards partial input after known loss. */
uint32_t *gnss_replay(const uint8_t *data, unsigned size, uint32_t tick, unsigned reset) {
  static gnss_t g;
  static uint32_t counters[5];
  if (reset == 1) g = (gnss_t){0};
  if (reset == 2) { g.collecting = g.valid = false; g.ubx_phase = 0; }
  for (unsigned i = 0; i < size; ++i) gnss_feed(&g, data[i], tick);
  counters[0] = g.good; counters[1] = g.bad;
  counters[2] = g.ubx_good; counters[3] = g.ubx_bad; counters[4] = g.fixes;
  return counters;
}

int gnss_test(void) {
  gnss_t g = {0};
  CHECK(!gnss_fresh(&g, 0));
  CHECK(sentence(&g, fix, 100));
  CHECK(g.good == 1 && g.bad == 0 && g.fixes == 1 && g.valid);
  CHECK(g.lat_e7 == 481173000 && g.lon_e7 == 115166667);
  CHECK(g.quality == 1 && g.satellites == 8 && g.hdop_x100 == 90);
  CHECK(gnss_fresh(&g, 2100) && !gnss_fresh(&g, 2101));
  CHECK(!strcmp(g.line, "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59"));
  CHECK(sentence(&g, "GPGGA,000000,9000.000,S,18000.000,W,2,12,1.23,0,M,0,M,,", 0xfffffff0U));
  CHECK(g.lat_e7 == -900000000 && g.lon_e7 == -1800000000 && g.hdop_x100 == 123);
  CHECK(gnss_fresh(&g, 10) && !gnss_fresh(&g, 3000));
  CHECK(sentence(&g, "GPGGA,000000,0000,N,00000,E,4,08,1,0,M,0,M,,", 100));
  CHECK(g.valid && g.lat_e7 == 0 && g.lon_e7 == 0);
  CHECK(sentence(&g, "GNGGA,000000,,,,,0,00,,,,,,,", 200));
  CHECK(!g.valid && g.quality == 0 && g.satellites == 0);
  const char *invalid[] = {
    "GNGGA,000000,4860,N,01100,E,1,08,1,0,M,,,,",
    "GNGGA,000000,9000.01,N,01100,E,1,08,1,0,M,,,,",
    "GNGGA,000000,4800,N,18000.01,E,1,08,1,0,M,,,,",
    "GNGGA,000000,4800,X,01100,E,1,08,1,0,M,,,,",
    "GNGGA,000000,4800,NN,01100,E,1,08,1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,N,1,08,1,0,M,,,,",
    "GNGGA,000000,4800.,N,01100,E,1,08,1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,1,08,-1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,1,08,0,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,1,08,1x,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,1,08,99999999999,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,1.2,08,1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,1,100,1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,3,08,1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,6,08,1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,7,08,1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,8,08,1,0,M,,,,",
    "GNGGA,000000,4800,N,01100,E,1",
  };
  for (unsigned i = 0; i < sizeof invalid/sizeof invalid[0]; ++i) {
    CHECK(sentence(&g, fix, 300));
    CHECK(sentence(&g, invalid[i], 400));
    CHECK(!g.valid && !gnss_fresh(&g, 400));
  }
  g = (gnss_t){0};
  const char *bad[] = {"$GPGGA*00\n", "$GPGGA*XX\n", "$GPGGA*123\n", "$GPGGA\n", "$GPGGA*\n"};
  for (unsigned i = 0; i < sizeof bad/sizeof bad[0]; ++i)
    for (const char *p = bad[i]; *p; ++p) CHECK(!gnss_feed(&g, (uint8_t)*p, 0));
  CHECK(g.good == 0 && g.bad == 5);
  gnss_feed(&g, '$', 0);
  for (unsigned i = 0; i < 300; ++i) gnss_feed(&g, 'A', 0);
  CHECK(g.bad == 6 && !g.collecting);
  gnss_feed(&g, '$', 0); gnss_feed(&g, 0, 0);
  CHECK(g.bad == 7);
  CHECK(sentence(&g, fix, 500));
  CHECK(sentence(&g, "GNRMC,123519,A,4807.038,N,01131.000,E,0,0,230394,,,A", 600));
  CHECK(g.updated == 500 && g.fixes == 1); /* Other sentences never refresh GGA. */
  /* UBX payload may contain arbitrary NMEA-looking text. Never interpret it. */
  g = (gnss_t){0};
  uint8_t ubx[] = {0xb5, 0x62, 1, 7, 4, 0, '$', '\n', 0, 0xff, 0, 0};
  uint8_t a = 0, b = 0;
  for (unsigned i = 2; i < sizeof ubx-2; ++i) { a += ubx[i]; b += a; }
  ubx[sizeof ubx-2] = a; ubx[sizeof ubx-1] = b;
  for (unsigned i = 0; i < sizeof ubx; ++i) CHECK(!gnss_feed(&g, ubx[i], 0));
  CHECK(g.ubx_good == 1 && g.ubx_bad == 0 && g.good == 0 && g.bad == 0);
  ubx[sizeof ubx-1] ^= 1;
  for (unsigned i = 0; i < sizeof ubx; ++i) gnss_feed(&g, ubx[i], 0);
  CHECK(g.ubx_good == 1 && g.ubx_bad == 1 && g.bad == 0);
  CHECK(sentence(&g, fix, 700) && g.valid);
  const uint8_t empty[] = {0xb5, 0x62, 1, 7, 0, 0, 8, 25};
  for (unsigned i = 0; i < sizeof empty; ++i) gnss_feed(&g, empty[i], 0);
  CHECK(g.ubx_good == 2);
  const uint8_t huge[] = {0xb5, 0x62, 1, 7, 0xff, 0xff};
  for (unsigned i = 0; i < sizeof huge; ++i) gnss_feed(&g, huge[i], 0);
  CHECK(g.ubx_bad == 2 && g.ubx_phase == 0);
  CHECK(sentence(&g, fix, 800));
  /* Interrupted frames and arbitrary binary/noise must resynchronize at '$'. */
  uint32_t random = 1234567;
  for (unsigned trial = 0; trial < 1000; ++trial) {
    for (unsigned i = 0; i < 50; ++i) {
      random = random*1664525U + 1013904223U;
      gnss_feed(&g, (uint8_t)(random >> 24), trial);
      CHECK(g.used < sizeof g.line);
    }
    CHECK(sentence(&g, fix, trial));
    CHECK(g.valid && g.lat_e7 == 481173000 && g.lon_e7 == 115166667);
  }
  return 0;
}
