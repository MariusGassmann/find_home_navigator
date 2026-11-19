#include "gnss.h"
#include <string.h>

static int hex(char c) {
  if (c >= '0' && c <= '9') return c-'0';
  if (c >= 'a' && c <= 'f') return c-'a'+10;
  if (c >= 'A' && c <= 'F') return c-'A'+10;
  return -1;
}

/* Unsigned decimal, scaled by 100; reject missing, malformed or huge fields. */
static bool decimal(const char *s, unsigned *out) {
  unsigned value = 0, fraction = 0, digits = 0;
  if (*s < '0' || *s > '9') return false;
  while (*s >= '0' && *s <= '9') {
    value = value*10 + (unsigned)(*s++-'0');
    if (value > 100000) return false;
  }
  if (*s == '.') {
    ++s;
    if (*s < '0' || *s > '9') return false;
    while (*s >= '0' && *s <= '9') {
      if (digits < 2) fraction = fraction*10 + (unsigned)(*s-'0');
      ++digits; ++s;
    }
  }
  if (*s) return false;
  if (digits == 1) fraction *= 10;
  *out = value*100 + fraction;
  return true;
}

static bool coordinate(const char *s, const char *hemisphere, bool latitude, int32_t *out) {
  unsigned degrees = 0, minutes = 0, fraction = 0, digits = 0;
  unsigned width = latitude ? 2 : 3;
  for (unsigned i = 0; i < width+2; ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    if (i < width) degrees = degrees*10 + (unsigned)(s[i]-'0');
    else minutes = minutes*10 + (unsigned)(s[i]-'0');
  }
  s += width+2;
  if (*s == '.') {
    ++s;
    if (*s < '0' || *s > '9') return false;
    while (*s >= '0' && *s <= '9') {
      if (digits < 7) fraction = fraction*10 + (unsigned)(*s-'0');
      ++digits; ++s;
    }
  }
  if (*s || minutes >= 60 || degrees > (latitude ? 90U : 180U)) return false;
  while (digits < 7) { fraction *= 10; ++digits; }
  if (degrees == (latitude ? 90U : 180U) && (minutes || fraction)) return false;
  char positive = latitude ? 'N' : 'E', negative = latitude ? 'S' : 'W';
  if (hemisphere[0] != positive && hemisphere[0] != negative) return false;
  if (hemisphere[1]) return false;
  *out = (int32_t)(degrees*10000000U + (minutes*10000000U + fraction + 30)/60);
  if (hemisphere[0] == negative) *out = -*out;
  return true;
}

static void position(gnss_t *g, uint32_t now) {
  char copy[sizeof g->line];
  strcpy(copy, g->line);
  char *fields[20] = {copy}, *p = copy;
  unsigned count = 1;
  while (*p && *p != '*') {
    if (*p == ',') {
      *p = 0;
      if (count == 20) return;
      fields[count++] = p+1;
    }
    ++p;
  }
  *p = 0;
  if (strlen(fields[0]) != 6 || strcmp(fields[0]+3, "GGA")) return;
  g->valid = false;
  g->quality = g->satellites = 0; g->hdop_x100 = 0;
  if (count < 10) { ++g->bad; return; }
  unsigned quality, satellites, hdop;
  if (!decimal(fields[6], &quality) || quality%100 || quality > 800 ||
      !decimal(fields[7], &satellites) || satellites%100 || satellites > 9900) {
    ++g->bad; return;
  }
  g->quality = quality/100; g->satellites = satellites/100;
  if (!quality) return; /* Empty position/HDOP fields are normal before a fix. */
  int32_t lat, lon;
  if (!decimal(fields[8], &hdop) || !hdop || hdop > 99999 ||
      !coordinate(fields[2], fields[3], true, &lat) ||
      !coordinate(fields[4], fields[5], false, &lon)) { ++g->bad; return; }
  g->hdop_x100 = hdop;
  if (quality != 100 && quality != 200 && quality != 400 && quality != 500) return;
  g->lat_e7 = lat; g->lon_e7 = lon;
  g->updated = now; ++g->fixes; g->valid = true;
}

bool gnss_feed(gnss_t *g, uint8_t byte, uint32_t now) {
  if (byte == '$') { g->used = 0; g->collecting = true; }
  if (!g->collecting) return false;
  if (byte == '\r') return false;
  if (byte != '\n') {
    if (byte < 32 || byte > 126 || g->used >= sizeof g->line-1) {
      ++g->bad; g->collecting = false; return false;
    }
    g->line[g->used++] = (char)byte; return false;
  }
  g->collecting = false; g->line[g->used] = 0;
  char *star = strchr(g->line, '*');
  unsigned checksum = 0;
  if (!star || strlen(star) != 3 || hex(star[1]) < 0 || hex(star[2]) < 0) {
    ++g->bad; return false;
  }
  for (const char *p = g->line+1; p < star; ++p) checksum ^= (unsigned char)*p;
  if (checksum != (unsigned)(hex(star[1])*16 + hex(star[2]))) { ++g->bad; return false; }
  ++g->good; position(g, now); return true;
}

bool gnss_fresh(const gnss_t *g, uint32_t now) {
  return g->valid && (uint32_t)(now-g->updated) <= 2000;
}
