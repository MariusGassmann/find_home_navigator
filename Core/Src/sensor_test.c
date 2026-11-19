#include "sensor_test.h"
#include "gnss.h"
#include "main.h"
#include <inttypes.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;
static UART_HandleTypeDef gps;
static volatile uint8_t ring[2048];
static volatile unsigned head, tail;
static volatile uint32_t received, errors, dropped;

/* One producer (ISR), one consumer (main); never print or use I2C in the ISR. */
void USART3_IRQHandler(void) {
  uint32_t status = USART3->ISR;
  bool faulty = (status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0;
  if (faulty) {
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

static void identify_compass(void) {
  printf("# I2C scan (7-bit addresses):");
  for (unsigned address = 8; address < 120; ++address)
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 1, 3) == HAL_OK)
      printf(" %02x", address);
  printf("\r\n");
  const struct { uint8_t address, reg, expected; const char *name; } probes[] = {
    {0x0d, 0x0d, 0xff, "QMC5883L"}, {0x2c, 0x00, 0x80, "QMC5883P"}
  };
  for (unsigned i = 0; i < sizeof probes/sizeof probes[0]; ++i) {
    uint8_t id[2];
    bool ok = true;
    for (unsigned n = 0; n < 2; ++n)
      if (HAL_I2C_Mem_Read(&hi2c1, probes[i].address << 1, probes[i].reg,
                          I2C_MEMADD_SIZE_8BIT, &id[n], 1, 10) != HAL_OK) ok = false;
    if (ok) printf("# MAG %s candidate address=%02x reg=%02x id=%02x/%02x match=%u; read-only test\r\n",
                   probes[i].name, probes[i].address, probes[i].reg, id[0], id[1],
                   (unsigned)(id[0] == probes[i].expected && id[1] == probes[i].expected));
  }
  printf("# MAG no configuration writes; send i to repeat scan/ID test\r\n");
}

void sensor_test_run(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("# GNSS discovery: USART3 PC11 RX, 115200 8N1; USB 115200; GGA parser\r\n");
  printf("# Commands: v toggles checksum-valid raw NMEA; i repeats compass ID\r\n");
  identify_compass(); /* Initial scan before UART reception starts. */
  uart_init();
  gnss_t g = {0};
  bool verbose = false;
  uint32_t last = HAL_GetTick(), prior_rx = 0, prior_faults = 0;
  uint8_t first[32];
  unsigned preview = 0;
  bool preview_reported = false;
  while (1) {
    uint32_t faults = errors + dropped;
    if (faults != prior_faults) {
      g.collecting = g.valid = false; tail = head; prior_faults = faults;
    }
    for (unsigned budget = 0; budget < 256 && tail != head; ++budget) {
      uint8_t byte = ring[tail]; tail = (tail+1) % sizeof ring;
      if (preview < sizeof first) first[preview++] = byte;
      if (gnss_feed(&g, byte, HAL_GetTick()) && verbose) printf("NMEA,%s\r\n", g.line);
    }
    uint8_t command;
    if (HAL_UART_Receive(&huart2, &command, 1, 0) == HAL_OK) {
      if (command == 'v') { verbose = !verbose; printf("# raw NMEA=%u\r\n", (unsigned)verbose); }
      if (command == 'i') identify_compass();
    }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now-last) >= 1000) {
      if (preview && !preview_reported) {
        printf("# RX first%u hex:", preview);
        for (unsigned i = 0; i < preview; ++i) printf(" %02x", first[i]);
        printf("\r\n"); preview_reported = true;
      }
      const char *state = received == prior_rx ? "NO_DATA" :
                          gnss_fresh(&g, now) ? "FIX" :
                          g.valid ? "STALE" : "NO_FIX";
      printf("GNSS,%" PRIu32 ",%s,bytes=%" PRIu32 ",bytes_s=%" PRIu32
             ",nmea=%" PRIu32 ",bad=%" PRIu32 ",fixes=%" PRIu32
             ",lat_e7=%" PRId32 ",lon_e7=%" PRId32 ",quality=%u,sat=%u,hdop_x100=%u"
             ",age_ms=%" PRIu32 ",uart_errors=%" PRIu32 ",dropped=%" PRIu32 "\r\n",
             now, state, received, (received-prior_rx)*1000/(uint32_t)(now-last),
             g.good, g.bad, g.fixes, g.lat_e7, g.lon_e7, g.quality, g.satellites,
             g.hdop_x100, g.fixes ? (uint32_t)(now-g.updated) : UINT32_MAX, errors, dropped);
      prior_rx = received; last = now;
    }
  }
}
