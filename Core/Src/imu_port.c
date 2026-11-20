#include "imu_port.h"
#include "main.h"
#include <stdio.h>

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

imu_bus_t imu_port_init(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();
  /* Shield rev 1.2, sheet 3: PB13 SCK, PB14 MISO, PC3 MOSI, PB12 CS.
     PB11 is the matrix latch: pulse only after a complete matrix frame. */
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
  return (imu_bus_t){&spi, read_reg, write_reg, HAL_Delay};
}

void imu_port_display(matrix_frame_t frame) {
  static matrix_frame_t previous = {UINT16_MAX, UINT16_MAX};
  if (frame.red == previous.red && frame.green == previous.green) return;
  uint8_t bytes[3];
  matrix_encode(frame, bytes);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&spi, bytes, 3, 20) != HAL_OK) {
    printf("# ERROR matrix SPI write\r\n"); Error_Handler();
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  previous = frame;
}
