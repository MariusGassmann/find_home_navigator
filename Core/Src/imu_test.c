#include "imu_test.h"
#include "imu.h"
#include "main.h"
#include <inttypes.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;
static SPI_HandleTypeDef spi;

static bool read_reg(void *context, uint8_t reg, uint8_t *data, uint16_t len) {
  SPI_HandleTypeDef *port = context;
  uint8_t tx[13] = {0}, rx[13];
  if (len > 12) return false;
  tx[0] = reg | 0x80;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(port, tx, rx, len + 1, 20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  if (status != HAL_OK) return false;
  for (unsigned i = 0; i < len; ++i) data[i] = rx[i + 1];
  return true;
}

static bool write_reg(void *context, uint8_t reg, uint8_t value) {
  uint8_t tx[] = {reg, value};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_Transmit(context, tx, sizeof tx, 20);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  return status == HAL_OK;
}

static void init_spi(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();
  /* Shield rev 1.2, sheet 3: PB13 SCK, PB14 MISO, PC3 MOSI, PB12 CS.
     PB11 is the shared LED matrix latch: keep it low while clocking SPI. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOC, &gpio);
  spi.Instance = SPI2;
  spi.Init.Mode = SPI_MODE_MASTER;
  spi.Init.Direction = SPI_DIRECTION_2LINES;
  spi.Init.DataSize = SPI_DATASIZE_8BIT;
  spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  spi.Init.NSS = SPI_NSS_SOFT;
  spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; /* 1.25 MHz */
  spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi.Init.TIMode = SPI_TIMODE_DISABLE;
  spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi.Init.CRCPolynomial = 7;
  spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&spi) != HAL_OK) Error_Handler();
}

void imu_test_run(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  init_spi();
  imu_bus_t bus = {&spi, read_reg, write_reg, HAL_Delay};
  imu_config_t config;
  HAL_Delay(20);
  if (!imu_init(&bus, &config)) {
    printf("# ERROR IMU init: WHO_AM_I=0x%02x expected=0x71; check shield/SPI\r\n", config.id);
    Error_Handler();
  }
  printf("# IMU v1 ID=%02x CTRL1=%02x CTRL2=%02x CTRL3=%02x CTRL6=%02x CTRL8=%02x\r\n",
         config.id, config.ctrl1, config.ctrl2, config.ctrl3, config.ctrl6, config.ctrl8);
  printf("# IMU,tick_ms,sequence,reference,ax,ay,az,gx,gy,gz (raw sensor XYZ)\r\n");
  uint32_t sequence = 0, reference = 0, last_sample = HAL_GetTick();
  uint32_t changed = HAL_GetTick();
  GPIO_PinState previous = GPIO_PIN_SET, stable = GPIO_PIN_SET;
  while (1) {
    uint32_t now = HAL_GetTick();
    GPIO_PinState pressed = HAL_GPIO_ReadPin(BTN_ENTER_GPIO_Port, BTN_ENTER_Pin);
    if (pressed != previous) { previous = pressed; changed = now; }
    if (pressed != stable && (uint32_t)(now - changed) >= 30) {
      stable = pressed;
      if (stable == GPIO_PIN_RESET) ++reference;
    }
    uint8_t command;
    if (HAL_UART_Receive(&huart2, &command, 1, 0) == HAL_OK && command == 'r')
      ++reference;
    imu_sample_t s;
    int result = imu_read(&bus, &s);
    if (result < 0) { printf("# ERROR SPI read\r\n"); Error_Handler(); }
    if (result == 1) {
      last_sample = HAL_GetTick();
      printf("IMU,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%d,%d,%d,%d,%d,%d\r\n",
             last_sample, sequence++, reference, s.accel[0], s.accel[1], s.accel[2],
             s.gyro[0], s.gyro[1], s.gyro[2]);
    }
    if ((uint32_t)(HAL_GetTick() - last_sample) > 500) {
      printf("# ERROR no fresh IMU data for 500 ms\r\n"); Error_Handler();
    }
    HAL_Delay(1);
  }
}
