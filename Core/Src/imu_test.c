#include "imu_test.h"
#include "imu_port.h"
#include "heading.h"
#include "matrix.h"
#include "main.h"
#include <inttypes.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

static void fatal(void) {
  imu_port_display((matrix_frame_t){.red = 0xfff});
  Error_Handler();
}


void imu_test_run(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  imu_bus_t bus = imu_port_init();
  imu_port_display((matrix_frame_t){0});
  imu_config_t config;
  HAL_Delay(20);
  if (!imu_init(&bus, &config)) {
    printf("# ERROR IMU init: WHO_AM_I=0x%02x expected=0x71; check shield/SPI\r\n", config.id);
    fatal();
  }
  printf("# IMU v1 ID=%02x CTRL1=%02x CTRL2=%02x CTRL3=%02x CTRL6=%02x CTRL8=%02x\r\n",
         config.id, config.ctrl1, config.ctrl2, config.ctrl3, config.ctrl6, config.ctrl8);
  printf("# IMU,tick_ms,sequence,reference,ax,ay,az,gx,gy,gz (raw sensor XYZ)\r\n");
  printf("# MATRIX Enter/r: level reference; t: pixel test (green, then red)\r\n");
  heading_t heading = {0};
  heading_state_t reported = HEADING_WAITING;
  bool testing = false;
  uint32_t test_start = 0, last_report = 0;
  uint32_t sequence = 0, reference = 0, last_sample = HAL_GetTick();
  uint32_t changed = HAL_GetTick();
  GPIO_PinState previous = GPIO_PIN_SET, stable = GPIO_PIN_SET;
  while (1) {
    uint32_t now = HAL_GetTick();
    GPIO_PinState pressed = HAL_GPIO_ReadPin(BTN_ENTER_GPIO_Port, BTN_ENTER_Pin);
    if (pressed != previous) { previous = pressed; changed = now; }
    if (pressed != stable && (uint32_t)(now - changed) >= 30) {
      stable = pressed;
      if (stable == GPIO_PIN_RESET) { ++reference; heading_reference(&heading); testing = false; }
    }
    uint8_t command;
    if (HAL_UART_Receive(&huart2, &command, 1, 0) == HAL_OK) {
      if (command == 'r') { ++reference; heading_reference(&heading); testing = false; }
      if (command == 't') {
        testing = true; test_start = now; heading = (heading_t){0};
        printf("# MATRIX test: row 0 faces POTI; walk rows left-to-right, green then red\r\n");
      }
    }
    imu_sample_t s;
    int result = imu_read(&bus, &s);
    if (result < 0) { printf("# ERROR SPI read\r\n"); fatal(); }
    if (result == 1) {
      last_sample = HAL_GetTick();
      heading_update(&heading, &s, last_sample);
      printf("IMU,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%d,%d,%d,%d,%d,%d\r\n",
             last_sample, sequence++, reference, s.accel[0], s.accel[1], s.accel[2],
             s.gyro[0], s.gyro[1], s.gyro[2]);
    }
    now = HAL_GetTick();
    if ((uint32_t)(now - last_sample) > 100 && heading.state == HEADING_TRACKING)
      heading.state = HEADING_INVALID;
    matrix_frame_t frame = {0};
    if (testing) {
      unsigned step = (uint32_t)(now - test_start) / 200;
      if (step < 12) frame.green = (uint16_t)(1U << step);
      else if (step < 24) frame.red = (uint16_t)(1U << (step - 12));
      else testing = false;
    } else if (heading.state == HEADING_TRACKING && heading.direction_valid)
      frame = matrix_pointer(heading.sector);
    else if (heading.state == HEADING_INVALID) frame.red = 0xfff;
    else if (heading.state == HEADING_CALIBRATING && (now / 250) % 2)
      frame.red = frame.green = 0x060; /* Amber center pair while collecting bias. */
    imu_port_display(frame);
    if (heading.state != reported ||
        (heading.state == HEADING_TRACKING && (uint32_t)(now - last_report) >= 1000)) {
      static const char *const states[] = {"WAITING", "CALIBRATING", "TRACKING", "INVALID"};
      printf("# HEADING %s yaw_mdeg=%ld bias_mdps=%ld sector=%u direction_valid=%u\r\n", states[heading.state],
             (long)(heading.yaw_deg * 1000), (long)(heading.bias_raw[2] * 17.5f), heading.sector,
             (unsigned)heading.direction_valid);
      reported = heading.state; last_report = now;
    }
    if ((uint32_t)(now - last_sample) > 500) {
      printf("# ERROR no fresh IMU data for 500 ms\r\n"); fatal();
    }
    HAL_Delay(1);
  }
}
