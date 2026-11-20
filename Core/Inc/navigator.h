#ifndef NAVIGATOR_H
#define NAVIGATOR_H
#include "gnss.h"
#include "matrix.h"

#define NAV_HOME_SAMPLES 10
#define NAV_SAMPLE_INTERVAL_MS 600U
#define NAV_CAPTURE_TIMEOUT_MS 120000U
#define NAV_MIN_SATELLITES 6U
#define NAV_MAX_HDOP_X100 250U
#define NAV_HOME_SPREAD_M 5.0f
#define NAV_ARRIVAL_M 5.0f
#define NAV_DEPARTURE_M 8.0f
#define NAV_NOTICE_MS 3000U

typedef enum { NAV_IDLE, NAV_CAPTURE, NAV_SLEEP_NOTICE, NAV_SLEEP, NAV_ACTIVE } nav_state_t;
typedef struct {
  nav_state_t state;
  uint32_t since, seen_fix, seen_utc, sample_tick;
  int32_t lat[NAV_HOME_SAMPLES], lon[NAV_HOME_SAMPLES];
  int32_t home_lat, home_lon;
  float distance, bearing, relative, declination;
  unsigned count, arrival_hits, sector;
  bool home_valid, position_valid, heading_valid, arrived, coasting, sector_valid;
} navigator_t;

void navigator_init(navigator_t *n, float declination_deg);
void navigator_press(navigator_t *n, const gnss_t *g, uint32_t tick);
/* heading is CW magnetic degrees; NAN means unavailable. */
void navigator_update(navigator_t *n, const gnss_t *g, float heading, bool coasting, uint32_t tick);
const char *navigator_status(const navigator_t *n);
matrix_frame_t navigator_display(const navigator_t *n, uint32_t tick);
/* Great-circle distance and initial true bearing, coordinates in degrees * 1e7. */
bool navigator_geometry(int32_t lat, int32_t lon, int32_t dest_lat, int32_t dest_lon,
                        float *distance_m, float *bearing_deg);
#endif
