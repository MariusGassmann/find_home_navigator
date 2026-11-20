#include "navigator.h"
#include <math.h>

static float wrap(float a) { return remainderf(a, 360); }

bool navigator_geometry(int32_t lat, int32_t lon, int32_t dest_lat, int32_t dest_lon,
                        float *distance, float *bearing) {
  if (lat < -900000000 || lat > 900000000 || dest_lat < -900000000 || dest_lat > 900000000 ||
      lon < -1800000000 || lon > 1800000000 || dest_lon < -1800000000 || dest_lon > 1800000000)
    return false;
  const double rad = 1.7453292519943296e-9; /* radians per e7 degree */
  double a = lat*rad, b = dest_lat*rad, dl = ((int64_t)dest_lon-lon)*rad;
  double s = sin((b-a)*.5), t = sin(dl*.5);
  double h = s*s+cos(a)*cos(b)*t*t;
  if (h > 1) h = 1;
  if (h < 0) h = 0;
  *distance = (float)(12742017.6*atan2(sqrt(h),sqrt(1-h)));
  double x = cos(a)*sin(b)-sin(a)*cos(b)*cos(dl), y = sin(dl)*cos(b);
  *bearing = (float)(atan2(y,x)*57.29577951308232);
  if (*bearing < 0) *bearing += 360;
  if (*bearing >= 360) *bearing = 0;
  return true;
}

static bool good_fix(const gnss_t *g, uint32_t tick) {
  return gnss_fresh(g,tick) && g->satellites >= NAV_MIN_SATELLITES &&
         g->hdop_x100 > 0 && g->hdop_x100 <= NAV_MAX_HDOP_X100;
}

void navigator_init(navigator_t *n, float declination) {
  *n = (navigator_t){.declination = declination};
}

void navigator_press(navigator_t *n, const gnss_t *g, uint32_t tick) {
  switch (n->state) {
    case NAV_IDLE:
      n->state = NAV_CAPTURE; n->since = tick; n->count = 0;
      n->seen_fix = g->fixes; n->seen_utc = g->utc_ms;
      break;
    case NAV_CAPTURE: n->state = NAV_IDLE; n->count = 0; break;
    case NAV_ACTIVE: n->state = NAV_SLEEP_NOTICE; n->since = tick; break;
    case NAV_SLEEP:
      n->state = NAV_ACTIVE; n->position_valid = n->heading_valid = n->arrived = false;
      n->arrival_hits = 0; n->sector_valid = false;
      n->seen_fix = g->fixes; n->seen_utc = g->utc_ms;
      n->sample_tick = tick;
      break;
    case NAV_SLEEP_NOTICE: break;
  }
}

static void capture(navigator_t *n, const gnss_t *g, uint32_t tick) {
  if ((uint32_t)(tick-n->since) >= NAV_CAPTURE_TIMEOUT_MS) { n->state = NAV_IDLE; return; }
  if (!good_fix(g,tick)) { n->count = 0; return; }
  if (g->fixes == n->seen_fix) return;
  n->seen_fix = g->fixes;
  if (g->utc_ms == n->seen_utc) return;
  n->seen_utc = g->utc_ms;
  if (n->count && (uint32_t)(g->updated-n->sample_tick) < NAV_SAMPLE_INTERVAL_MS) return;
  float distance, bearing;
  if (!navigator_geometry(g->lat_e7,g->lon_e7,g->lat_e7,g->lon_e7,&distance,&bearing)) {
    n->count = 0; return;
  }
  if (n->count) {
    navigator_geometry(n->lat[0],n->lon[0],g->lat_e7,g->lon_e7,&distance,&bearing);
    if (distance > NAV_HOME_SPREAD_M) n->count = 0;
  }
  n->lat[n->count] = g->lat_e7; n->lon[n->count++] = g->lon_e7;
  n->sample_tick = g->updated;
  if (n->count < NAV_HOME_SAMPLES) return;
  int64_t lat_sum = 0, lon_sum = 0;
  for (unsigned i=0; i<n->count; ++i) {
    lat_sum += n->lat[i];
    int64_t delta = (int64_t)n->lon[i]-n->lon[0];
    if (delta > 1800000000LL) delta -= 3600000000LL;
    if (delta < -1800000000LL) delta += 3600000000LL;
    lon_sum += delta;
  }
  int32_t lat = (int32_t)(lat_sum/n->count);
  int64_t lon = n->lon[0]+lon_sum/n->count;
  if (lon > 1800000000LL) lon -= 3600000000LL;
  if (lon < -1800000000LL) lon += 3600000000LL;
  for (unsigned i=0; i<n->count; ++i) {
    navigator_geometry(lat,(int32_t)lon,n->lat[i],n->lon[i],&distance,&bearing);
    if (distance > NAV_HOME_SPREAD_M) { n->count = 0; return; }
  }
  n->home_lat = lat; n->home_lon = (int32_t)lon; n->home_valid = true;
  n->state = NAV_SLEEP_NOTICE; n->since = tick;
}

void navigator_update(navigator_t *n, const gnss_t *g, float heading, bool coasting, uint32_t tick) {
  if (n->state == NAV_CAPTURE) { capture(n,g,tick); return; }
  if (n->state == NAV_SLEEP_NOTICE && (uint32_t)(tick-n->since) >= NAV_NOTICE_MS)
    n->state = NAV_SLEEP;
  if (n->state != NAV_ACTIVE) return;
  bool usable = n->home_valid && good_fix(g,tick);
  /* Geography changes only with a new fix, not on every UART-byte interrupt. */
  if (usable && (g->fixes != n->seen_fix || !n->position_valid))
    usable = navigator_geometry(g->lat_e7,g->lon_e7,n->home_lat,n->home_lon,&n->distance,&n->bearing);
  n->position_valid = usable;
  n->heading_valid = isfinite(heading);
  n->coasting = coasting;
  if (!n->position_valid) { n->arrival_hits = 0; n->arrived = false; return; }
  if (g->fixes != n->seen_fix) {
    n->seen_fix = g->fixes;
    if (g->utc_ms != n->seen_utc && (uint32_t)(g->updated-n->sample_tick) >= NAV_SAMPLE_INTERVAL_MS) {
      n->seen_utc = g->utc_ms; n->sample_tick = g->updated;
      if (n->distance <= NAV_ARRIVAL_M) {
        if (n->arrival_hits < 3) ++n->arrival_hits;
        if (n->arrival_hits == 3) n->arrived = true;
      } else n->arrival_hits = 0;
      if (n->distance > NAV_DEPARTURE_M) n->arrived = false;
    }
  }
  if (n->heading_valid) {
    /* Heading CW - bearing CW = target CCW relative to board forward. */
    n->relative = wrap(heading+n->declination-n->bearing);
    if (!n->sector_valid) {
      float angle = n->relative < 0 ? n->relative+360 : n->relative;
      n->sector = (unsigned)((angle+22.5f)/45) % 8; n->sector_valid = true;
    } else n->sector = matrix_sector(n->relative,n->sector);
  }
}

const char *navigator_status(const navigator_t *n) {
  switch (n->state) {
    case NAV_IDLE: return "NO_HOME";
    case NAV_CAPTURE: return "CAPTURING";
    case NAV_SLEEP_NOTICE: return "SLEEP_NOTICE";
    case NAV_SLEEP: return "SLEEPING";
    case NAV_ACTIVE:
      if (!n->position_valid) return "NO_FIX";
      if (n->arrived) return "ARRIVED";
      if (!n->heading_valid) return "NO_HEADING";
      return n->coasting ? "COASTING" : "GO_HOME";
  }
  return "NO_HOME";
}

matrix_frame_t navigator_display(const navigator_t *n, uint32_t tick) {
  if (n->state == NAV_SLEEP) return (matrix_frame_t){0};
  if (n->state == NAV_SLEEP_NOTICE) return (matrix_frame_t){.green=0x727};
  if (n->state == NAV_CAPTURE) {
    uint16_t progress = n->count ? (uint16_t)((1U<<n->count)-1) : tick/400%2 ? 0x525 : 0;
    return (matrix_frame_t){progress,progress};
  }
  if (n->state == NAV_IDLE) return (matrix_frame_t){.red=0x525};
  if (!n->position_valid) return (matrix_frame_t){.red=tick/500%2 ? 0x525 : 0};
  if (n->arrived) return (matrix_frame_t){.green=0x757};
  if (!n->heading_valid) return (matrix_frame_t){0x525,0x525};
  return matrix_pointer(n->sector);
}
