#include "navigator.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) return 5000+__LINE__; } while (0)

static void fix(gnss_t *g, int32_t lat, int32_t lon, uint32_t tick) {
  g->valid = true; g->quality = 1; g->satellites = 8; g->hdop_x100 = 120;
  g->updated = tick; ++g->fixes; g->utc_ms = tick % 86400000;
  g->lat_e7 = lat; g->lon_e7 = lon;
}
static void sentence(gnss_t *g, const char *body, uint32_t tick) {
  unsigned sum = 0;
  for (const char *p=body; *p; ++p) sum ^= (unsigned char)*p;
  char line[180]; snprintf(line,sizeof line,"$%s*%02X\r\n",body,sum);
  for (const char *p=line; *p; ++p) gnss_feed(g,(uint8_t)*p,tick);
}
static uint32_t save(navigator_t *n, gnss_t *g, int32_t lat, int32_t lon, uint32_t tick) {
  navigator_press(n,g,tick);
  for (unsigned i=0; i<NAV_HOME_SAMPLES; ++i) {
    tick += 1000; fix(g,lat,lon,tick); navigator_update(n,g,NAN,false,tick);
  }
  return tick;
}
static bool mask(matrix_frame_t a, matrix_frame_t b) { return a.red==b.red && a.green==b.green; }

/* Replay actual recorded UART bytes through the production parser and capture. */
uint32_t *navigator_replay(const uint8_t *data, unsigned size, uint32_t tick, unsigned reset) {
  static gnss_t g;
  static navigator_t n;
  static bool started;
  static uint32_t result[5];
  if (reset==1) { g=(gnss_t){0}; navigator_init(&n,0); started=false; }
  if (reset==2) { g.collecting=g.valid=false; g.ubx_phase=0; n.count=0; }
  if (size && !started) { navigator_press(&n,&g,tick); started=true; }
  for (unsigned i=0; i<size; ++i) gnss_feed(&g,data[i],tick);
  navigator_update(&n,&g,NAN,false,tick);
  result[0]=n.state; result[1]=n.home_valid; result[2]=(uint32_t)n.home_lat;
  result[3]=(uint32_t)n.home_lon; result[4]=n.count;
  return result;
}

int navigator_test(void) {
  navigator_t n; gnss_t g = {0}; float d, b;
  /* Geographic reference cases and metre-scale precision near Zurich. */
  CHECK(navigator_geometry(0,0,10000000,0,&d,&b));
  CHECK(fabsf(d-111195.08f)<.1f && fabsf(b)<.001f);
  CHECK(navigator_geometry(0,0,0,10000000,&d,&b));
  CHECK(fabsf(d-111195.08f)<.1f && fabsf(b-90)<.001f);
  CHECK(navigator_geometry(473769000,85417000,473769450,85417000,&d,&b));
  CHECK(fabsf(d-5.00378f)<.001f && fabsf(b)<.001f);
  CHECK(navigator_geometry(0,1799999900,0,-1799999900,&d,&b));
  CHECK(fabsf(d-2.2239f)<.001f && fabsf(b-90)<.001f);
  CHECK(!navigator_geometry(900000001,0,0,0,&d,&b));
  CHECK(!navigator_geometry(0,0,0,1800000001,&d,&b));
  CHECK(navigator_geometry(0,0,0,1800000000,&d,&b) && isfinite(d));

  navigator_init(&n,0);
  CHECK(!strcmp(navigator_status(&n),"NO_HOME"));
  CHECK(mask(navigator_display(&n,0),(matrix_frame_t){.red=0x525}));
  uint32_t tick=save(&n,&g,473769000,85417000,0);
  CHECK(n.home_valid && n.home_lat==473769000 && n.home_lon==85417000);
  CHECK(n.count==10 && n.state==NAV_SLEEP_NOTICE);
  CHECK(mask(navigator_display(&n,tick),(matrix_frame_t){.green=0x727}));
  navigator_update(&n,&g,0,false,tick+2999);
  CHECK(n.state==NAV_SLEEP_NOTICE);
  navigator_update(&n,&g,0,false,tick+3000);
  CHECK(n.state==NAV_SLEEP && mask(navigator_display(&n,tick+3000),(matrix_frame_t){0}));
  int32_t home_lat=n.home_lat, home_lon=n.home_lon;
  /* App clears transport and position on wake, retains RAM home only. */
  g=(gnss_t){0}; tick+=4000; navigator_press(&n,&g,tick);
  navigator_update(&n,&g,0,false,tick);
  CHECK(n.state==NAV_ACTIVE && !strcmp(navigator_status(&n),"NO_FIX"));
  CHECK(n.home_lat==home_lat && n.home_lon==home_lon);
  fix(&g,home_lat-9000,home_lon,tick+=1000); /* ~100 m south, home true north. */
  navigator_update(&n,&g,90,false,tick);
  CHECK(!strcmp(navigator_status(&n),"GO_HOME"));
  CHECK(fabsf(n.distance-100.0756f)<.01f && fabsf(n.bearing)<.01f);
  CHECK(n.sector==2); /* Board points east, home lies to its left. */
  CHECK(navigator_display(&n,tick).green==matrix_pointer(2).green);
  CHECK(mask(navigator_display(&n,tick),matrix_pointer(2)));
  navigator_update(&n,&g,0,true,tick);
  CHECK(!strcmp(navigator_status(&n),"COASTING"));
  CHECK(mask(navigator_display(&n,tick),matrix_pointer(n.sector)));
  navigator_update(&n,&g,NAN,false,tick);
  CHECK(!strcmp(navigator_status(&n),"NO_HEADING"));
  CHECK(mask(navigator_display(&n,tick),(matrix_frame_t){0x525,0x525}));
  navigator_update(&n,&g,0,false,tick+2001);
  CHECK(!strcmp(navigator_status(&n),"NO_FIX"));
  CHECK(n.home_valid);
  /* Three genuinely new epochs inside 5 m; no repeated-loop or duplicate fix arrival. */
  for (unsigned i=0; i<2; ++i) {
    fix(&g,home_lat-360,home_lon,tick+=1000);
    navigator_update(&n,&g,0,false,tick); CHECK(!n.arrived);
    for (unsigned j=0; j<100; ++j) navigator_update(&n,&g,0,false,tick);
    CHECK(n.arrival_hits==i+1);
  }
  ++g.fixes; navigator_update(&n,&g,0,false,tick+700);
  CHECK(!n.arrived && n.arrival_hits==2); /* Same UTC is not independent. */
  fix(&g,home_lat-360,home_lon,tick+=1000); navigator_update(&n,&g,NAN,false,tick);
  CHECK(n.arrived && !strcmp(navigator_status(&n),"ARRIVED"));
  CHECK(mask(navigator_display(&n,tick),(matrix_frame_t){.green=0x757}));
  fix(&g,home_lat-630,home_lon,tick+=1000); navigator_update(&n,&g,0,false,tick);
  CHECK(n.arrived); /* 7 m: hysteresis. */
  fix(&g,home_lat-810,home_lon,tick+=1000); navigator_update(&n,&g,0,false,tick);
  CHECK(!n.arrived);
  /* Repeated sleep/wake never overwrites home or accepts the old fix. */
  for (unsigned i=0; i<20; ++i) {
    navigator_press(&n,&g,tick); CHECK(n.state==NAV_SLEEP_NOTICE);
    navigator_update(&n,&g,0,false,tick+=3000); CHECK(n.state==NAV_SLEEP);
    g=(gnss_t){0}; navigator_press(&n,&g,++tick);
    navigator_update(&n,&g,NAN,false,tick);
    CHECK(n.home_lat==home_lat && n.home_lon==home_lon && !n.position_valid);
  }
  /* Bad quality, stale data, outliers and duplicate epochs cannot set home. */
  navigator_init(&n,0); g=(gnss_t){0}; tick=0; navigator_press(&n,&g,tick);
  for (unsigned i=0; i<5; ++i) {
    fix(&g,home_lat+i,home_lon,tick+=1000); navigator_update(&n,&g,0,false,tick);
  }
  CHECK(n.count==5);
  g.satellites=5; navigator_update(&n,&g,0,false,tick); CHECK(n.count==0);
  g.satellites=8; g.hdop_x100=251; navigator_update(&n,&g,0,false,tick); CHECK(n.count==0);
  g.hdop_x100=120; navigator_update(&n,&g,0,false,tick+2001); CHECK(n.count==0);
  for (unsigned i=0; i<5; ++i) {
    fix(&g,home_lat,home_lon,tick+=1000); navigator_update(&n,&g,0,false,tick);
  }
  CHECK(n.count==5);
  fix(&g,home_lat+1000,home_lon,tick+=1000); navigator_update(&n,&g,0,false,tick);
  CHECK(n.count==1); /* 11 m away restarts averaging. */
  uint32_t utc=g.utc_ms;
  for (unsigned i=0; i<15; ++i) {
    fix(&g,home_lat+1000,home_lon,tick+=1000); g.utc_ms=utc;
    navigator_update(&n,&g,0,false,tick);
  }
  CHECK(!n.home_valid && n.count==1);
  navigator_press(&n,&g,tick); CHECK(n.state==NAV_IDLE);
  navigator_press(&n,&g,tick);
  navigator_update(&n,&g,0,false,tick+NAV_CAPTURE_TIMEOUT_MS);
  CHECK(n.state==NAV_IDLE && !n.home_valid);
  /* Mean and longitude unwrapping across the date line; clock wrap too. */
  navigator_init(&n,4); g=(gnss_t){0}; tick=UINT32_MAX-8000;
  navigator_press(&n,&g,tick);
  for (unsigned i=0; i<10; ++i) {
    fix(&g,0,i%2 ? 1799999900 : -1799999900,tick+=1000);
    navigator_update(&n,&g,0,false,tick);
  }
  CHECK(n.home_valid && n.home_lat==0 && (n.home_lon==1800000000 || n.home_lon==-1800000000));
  /* End-to-end: real checksummed GGA -> capture -> sleep -> wake -> home arrow.
     Injected declination verifies true/magnetic conversion independently. */
  navigator_init(&n,4); g=(gnss_t){0}; tick=0; navigator_press(&n,&g,tick);
  for (unsigned i=0; i<10; ++i) {
    char body[130];
    snprintf(body,sizeof body,"GNGGA,1200%02u.000,4722.6140,N,00832.5020,E,1,08,1.2,0,M,0,M,,",i);
    sentence(&g,body,tick+=1000); navigator_update(&n,&g,0,false,tick);
  }
  CHECK(n.home_valid && n.home_lat==473769000 && n.home_lon==85417000);
  navigator_update(&n,&g,0,false,tick+=3000); g=(gnss_t){0}; navigator_press(&n,&g,tick);
  sentence(&g,"GNGGA,120100.000,4722.5600,N,00832.5020,E,1,08,1.2,0,M,0,M,,",tick+=1000);
  navigator_update(&n,&g,86,false,tick);
  CHECK(n.position_valid && n.sector==2 && fabsf(n.relative-90)<.01f);
  /* Invalid UTC must never refresh an accepted position. */
  const char *bad_utc[]={"","240000","126000","120060","120000.","12x000","120000x"};
  for (unsigned i=0; i<sizeof bad_utc/sizeof bad_utc[0]; ++i) {
    char body[130]; snprintf(body,sizeof body,"GNGGA,%s,4722.614,N,00832.502,E,1,08,1.2,0,M,0,M,,",bad_utc[i]);
    sentence(&g,body,++tick); CHECK(!g.valid);
  }
  return 0;
}
