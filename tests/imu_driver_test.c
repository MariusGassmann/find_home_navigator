/* Execute the real driver on an emulated Cortex-M4 with a fake register bus. */
#include "imu.h"
int matrix_heading_test(void);
#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
static uint8_t registers[256];
static int read_failure, write_failure, stuck_reset, corrupt, delayed;
static bool read_bus(void *ctx, uint8_t reg, uint8_t *out, uint16_t n) {
  (void)ctx;
  if (read_failure) return false;
  for (unsigned i = 0; i < n; ++i) out[i] = registers[reg+i];
  if (corrupt && reg == 0x15) out[0] ^= 1;
  return true;
}
static bool write_bus(void *ctx, uint8_t reg, uint8_t value) {
  (void)ctx;
  if (write_failure) return false;
  registers[reg] = (reg == 0x12 && value == 1 && !stuck_reset) ? 0 : value;
  return true;
}
static void delay(uint32_t ms) { delayed += (int)ms; }
static void word(unsigned address, uint16_t value) {
  registers[address] = value & 255;
  registers[address+1] = value >> 8;
}

int test_main(void) {
  imu_bus_t bus = {0, read_bus, write_bus, delay};
  imu_config_t config;
  imu_sample_t sample;
  registers[0x0f] = 0x71;
  CHECK(imu_init(&bus, &config));
  CHECK(config.id == 0x71 && config.ctrl1 == 5 && config.ctrl2 == 5);
  CHECK(config.ctrl3 == 0x44 && config.ctrl6 == 2 && config.ctrl8 == 0);
  CHECK(delayed == 101);
  CHECK(imu_suspend(&bus) && registers[0x10] == 0 && registers[0x11] == 0);
  read_failure = 1; CHECK(!imu_suspend(&bus)); read_failure = 0;
  write_failure = 1; CHECK(!imu_suspend(&bus)); write_failure = 0;
  CHECK(imu_init(&bus, &config));
  for (unsigned identity = 0; identity < 256; ++identity) {
    registers[0x0f] = (uint8_t)identity;
    CHECK(imu_init(&bus, &config) == (identity == 0x71));
    CHECK(config.id == identity);
  }
  registers[0x0f] = 0x71;
  read_failure = 1; CHECK(!imu_init(&bus, &config));
  CHECK(imu_read(&bus, &sample) == -1); read_failure = 0;
  write_failure = 1; CHECK(!imu_init(&bus, &config)); write_failure = 0;
  stuck_reset = 1; delayed = 0;
  CHECK(!imu_init(&bus, &config)); CHECK(delayed == 100); stuck_reset = 0;
  corrupt = 1; CHECK(!imu_init(&bus, &config)); corrupt = 0;
  CHECK(imu_init(&bus, &config));
  for (unsigned status = 0; status < 256; ++status) {
    registers[0x1e] = (uint8_t)status;
    CHECK(imu_read(&bus, &sample) == ((status & 3) == 3 ? 1 : 0));
  }
  registers[0x1e] = 3;
  word(0x22, 0x7fff); word(0x24, 0x8000); word(0x26, 0xffff);
  word(0x28, 0x1234); word(0x2a, 0xfedc); word(0x2c, 0xabcd);
  CHECK(imu_read(&bus, &sample) == 1);
  CHECK(sample.gyro[0] == 32767 && sample.gyro[1] == -32768 && sample.gyro[2] == -1);
  CHECK(sample.accel[0] == -21555 && sample.accel[1] == -292 && sample.accel[2] == 4660);
  /* All signed 16-bit patterns, including both endpoints, in each output axis. */
  for (unsigned v = 0; v < 65536; ++v) {
    for (unsigned reg = 0x22; reg <= 0x2c; reg += 2) word(reg, (uint16_t)v);
    CHECK(imu_read(&bus, &sample) == 1);
    int expected = v < 32768 ? (int)v : (int)v - 65536;
    for (unsigned axis = 0; axis < 3; ++axis) {
      CHECK(sample.accel[axis] == expected);
      CHECK(sample.gyro[axis] == expected);
    }
  }
  int result = matrix_heading_test();
  extern int gnss_test(void);
  extern int mag_test(void);
  extern int north_test(void);
  extern int navigator_test(void);
  if (!result) result = gnss_test();
  if (!result) result = mag_test();
  if (!result) result = north_test();
  return result ? result : navigator_test();
}
