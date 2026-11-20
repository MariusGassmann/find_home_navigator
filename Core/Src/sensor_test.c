#include "sensor_test.h"
#include "gnss.h"
#include "mag.h"
#include "imu_port.h"
#include "north_calibration.h"
#include "main.h"
#include <inttypes.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;
static UART_HandleTypeDef gps;
static volatile uint8_t ring[2048];
static volatile unsigned head, tail;
static volatile uint32_t received, errors, dropped;
static volatile uint32_t framing, noise, overrun;

/* One producer (ISR), one consumer (main); never print or use I2C in the ISR. */
void USART3_IRQHandler(void) {
  uint32_t status = USART3->ISR;
  bool faulty = (status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0;
  if (faulty) {
    if (status & USART_ISR_FE) ++framing;
    if (status & USART_ISR_NE) ++noise;
    if (status & USART_ISR_ORE) ++overrun;
    USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_PECF;
    ++errors;
  }
  if (status & USART_ISR_RXNE) {
    uint8_t byte = (uint8_t)USART3->RDR;
    ++received;
    if (faulty) return;
    unsigned next = (head+1) % sizeof ring;
    if (next == tail) ++dropped;
    else { ring[head] = byte; head = next; }
  }
}

static void uart_init(void) {
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_11; /* PC11: USART3_RX; PC10/TX deliberately unused. */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Alternate = GPIO_AF7_USART3;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &gpio);
  gps.Instance = USART3;
  gps.Init.BaudRate = 115200;
  gps.Init.WordLength = UART_WORDLENGTH_8B;
  gps.Init.StopBits = UART_STOPBITS_1;
  gps.Init.Parity = UART_PARITY_NONE;
  gps.Init.Mode = UART_MODE_RX;
  gps.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  gps.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&gps) != HAL_OK) Error_Handler();
  HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  SET_BIT(USART3->CR3, USART_CR3_EIE);
  SET_BIT(USART3->CR1, USART_CR1_RXNEIE | USART_CR1_PEIE);
}

static bool mag_read_reg(void *context, uint8_t reg, uint8_t *data, uint16_t size) {
  return HAL_I2C_Mem_Read(context, 0x2c << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, 5) == HAL_OK;
}

static bool mag_write_reg(void *context, uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(context, 0x2c << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 5) == HAL_OK;
}

static bool start_compass(const mag_bus_t *bus) {
  mag_check_t check;
  bool ready = mag_init(bus, &check);
  if (ready) printf("# MAG init=1 address=2c CTRL1=05 CTRL2=08 AXIS=06; raw XYZ active\r\n");
  else printf("# MAG init=0 op=%c reg=%02x expected=%02x actual=%02x; no compass samples\r\n",
              check.operation, check.reg, check.expected, check.actual);
  return ready;
}

/* Hex preserves malformed NMEA and binary data. USB 460800 provides room for
   115200-baud GPS input plus hex expansion, compass samples and diagnostics. */
static void raw_record(const uint8_t *data, unsigned size, uint32_t tick, uint32_t offset) {
  static const char hex[] = "0123456789abcdef";
  char line[320];
  unsigned used = (unsigned)snprintf(line, sizeof line, "RX,%" PRIu32 ",%" PRIu32 ",", tick, offset);
  for (unsigned i = 0; i < size; ++i) {
    line[used++] = hex[data[i] >> 4]; line[used++] = hex[data[i] & 15];
  }
  line[used++] = '\r'; line[used++] = '\n';
  if (HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)used, 30) != HAL_OK) Error_Handler();
}

void sensor_test_run(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  huart2.Init.BaudRate = 460800;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
  printf("# SENSOR v6: GPS 115200, USB 460800; RX + MAG + IMU + FUSE\r\n");
  printf("# NORTH magnetic demo; Enter/r: level reference; calibration=" NORTH_CALIBRATION_ID "\r\n");
  mag_bus_t bus = {&hi2c1, mag_read_reg, mag_write_reg, HAL_Delay};
  bool mag_ready = start_compass(&bus);
  imu_bus_t imu_bus = imu_port_init();
  north_t north;
  north_init(&north, &north_calibration);
  imu_port_display(north_display(&north, HAL_GetTick()));
  imu_config_t config;
  bool imu_ready = imu_init(&imu_bus, &config);
  printf("# IMU init=%u ID=%02x expected=71\r\n", (unsigned)imu_ready, config.id);
  uart_init();
  uint32_t imu_sequence = 0, reference = 0, changed = HAL_GetTick();
  GPIO_PinState previous = GPIO_PIN_SET, stable = GPIO_PIN_SET;
  gnss_t g = {0};
  uint32_t last = HAL_GetTick(), prior_rx = 0, prior_faults = 0;
  uint32_t offset = 0, mag_tick = last, mag_sequence = 0, mag_errors = 0, mag_overflows = 0;
  uint32_t raw_tick = last, fusion_tick = last, prior_reference = 0;
  uint8_t chunk[128];
  unsigned size = 0;
  while (1) {
    uint32_t faults = errors + dropped;
    if (faults != prior_faults) {
      printf("# RX_LOSS errors=%" PRIu32 " dropped=%" PRIu32 "\r\n", errors, dropped);
      g.collecting = g.valid = false; g.ubx_phase = 0;
      tail = head; size = 0; prior_faults = faults;
    }
    while (size < sizeof chunk && tail != head) {
      uint8_t byte = ring[tail]; tail = (tail+1) % sizeof ring;
      chunk[size++] = byte;
      gnss_feed(&g, byte, HAL_GetTick());
    }
    if (size && (size == sizeof chunk || (uint32_t)(HAL_GetTick()-raw_tick) >= 10)) {
      raw_record(chunk, size, HAL_GetTick(), offset);
      offset += size; size = 0; raw_tick = HAL_GetTick();
    }
    uint8_t command;
    if (HAL_UART_Receive(&huart2, &command, 1, 0) == HAL_OK) {
      if (command == 'r') ++reference;
    }
    uint32_t now = HAL_GetTick();
    GPIO_PinState pressed = HAL_GPIO_ReadPin(BTN_ENTER_GPIO_Port, BTN_ENTER_Pin);
    if (pressed != previous) { previous = pressed; changed = now; }
    if (pressed != stable && (uint32_t)(now-changed) >= 30) {
      stable = pressed;
      if (stable == GPIO_PIN_RESET) ++reference;
    }
    if (reference != prior_reference) {
      north_reference(&north);
      prior_reference = reference;
    }
    if (imu_ready) {
      imu_sample_t sample;
      int result = imu_read(&imu_bus, &sample);
      if (result < 0) {
        printf("# ERROR IMU SPI read; reset board\r\n");
        imu_ready = false;
      }
      if (result > 0) {
        north_imu(&north, &sample, HAL_GetTick());
        printf("IMU,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%d,%d,%d,%d,%d,%d\r\n",
               HAL_GetTick(), imu_sequence++, reference, sample.accel[0], sample.accel[1],
               sample.accel[2], sample.gyro[0], sample.gyro[1], sample.gyro[2]);
      }
    }
    if (mag_ready && (uint32_t)(now-mag_tick) >= 10) {
      mag_tick = now;
      mag_sample_t sample;
      int result = mag_read(&bus, &sample);
      if (result < 0) ++mag_errors;
      if (result == 2) ++mag_overflows;
      if (result > 0) {
        north_mag(&north, &sample, HAL_GetTick());
        printf("MAG,%" PRIu32 ",%" PRIu32 ",%d,%d,%d,%u\r\n", now, mag_sequence++,
               sample.xyz[0], sample.xyz[1], sample.xyz[2], sample.status);
      }
    }
    now = HAL_GetTick();
    imu_port_display(north_display(&north, now));
    if ((uint32_t)(now-fusion_tick) >= 100) {
      fusion_tick = now;
      printf("FUSE,%" PRIu32 ",%s,%" PRId32 ",%" PRId32 ",%" PRId32 ",%u,%u\r\n",
             now, north_state_name(north_state(&north, now)),
             north.aligned ? (int32_t)(north.gyro*1000) : INT32_MIN,
             north.compass_valid && (uint32_t)(now-north.mag_tick) <= 100 ?
                 (int32_t)(north.compass*1000) : INT32_MIN,
             north.aligned ? (int32_t)(north.fused*1000) : INT32_MIN,
             north.sector, (unsigned)(north.mag_used && (uint32_t)(now-north.mag_tick) <= 100));
    }
    if ((uint32_t)(now-last) >= 1000) {
      const char *state = received == prior_rx ? "NO_DATA" :
                          gnss_fresh(&g, now) ? "FIX" :
                          g.valid ? "STALE" : "NO_FIX";
      printf("GNSS,%" PRIu32 ",%s,bytes=%" PRIu32 ",bytes_s=%" PRIu32
             ",nmea=%" PRIu32 ",bad=%" PRIu32 ",ubx=%" PRIu32 ",ubx_bad=%" PRIu32 ",fixes=%" PRIu32
             ",lat_e7=%" PRId32 ",lon_e7=%" PRId32 ",quality=%u,sat=%u,hdop_x100=%u"
             ",age_ms=%" PRIu32 ",uart_errors=%" PRIu32 ",dropped=%" PRIu32
             ",fe=%" PRIu32 ",ne=%" PRIu32 ",ore=%" PRIu32
             ",mag_ready=%u,mag_samples=%" PRIu32
             ",mag_errors=%" PRIu32 ",mag_overflows=%" PRIu32 "\r\n",
             now, state, received, (received-prior_rx)*1000/(uint32_t)(now-last),
             g.good, g.bad, g.ubx_good, g.ubx_bad, g.fixes, g.lat_e7, g.lon_e7, g.quality, g.satellites,
             g.hdop_x100, g.fixes ? (uint32_t)(now-g.updated) : UINT32_MAX, errors, dropped,
             framing, noise, overrun, (unsigned)mag_ready, mag_sequence, mag_errors, mag_overflows);
      prior_rx = received; last = now;
    }
  }
}
