#include "power.h"
#include "imu_port.h"
#include "main.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;
extern void SystemClock_Config(void);

void EXTI0_IRQHandler(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(BTN_ENTER_Pin);
}

static bool pressed(void) {
  return HAL_GPIO_ReadPin(BTN_ENTER_GPIO_Port,BTN_ENTER_Pin) == GPIO_PIN_RESET;
}

static void fade_z(void) {
  /* No output-enable pin on this shield: PWM complete latched SPI frames.
     GPS reception is off, so this one-second animation cannot overrun its UART. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t start = HAL_GetTick(), period = SystemCoreClock/250;
  uint32_t elapsed;
  while ((elapsed = (uint32_t)(HAL_GetTick()-start)) < 1000) {
    uint32_t remaining = 1000-elapsed;
    uint32_t on = (uint32_t)((uint64_t)period*remaining/1000);
    imu_port_display((matrix_frame_t){.green=DWT->CYCCNT%period < on ? 0x727 : 0});
  }
  imu_port_display((matrix_frame_t){0});
}

void power_sleep(const imu_bus_t *imu, const mag_bus_t *mag) {
  if (!imu_suspend(imu)) printf("# WARN IMU power-down not confirmed\r\n");
  if (!mag_suspend(mag)) printf("# WARN compass suspend not confirmed\r\n");
  fade_z();
  /* A held Enter must be released before it can be used to wake the board. */
  while (pressed()) HAL_Delay(10);
  HAL_Delay(30);
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
  CLEAR_BIT(huart2.Instance->CR1,USART_CR1_UE);
  __HAL_RCC_SPI2_CLK_DISABLE();
  __HAL_RCC_I2C1_CLK_DISABLE();
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_USART3_CLK_DISABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
  HAL_DBGMCU_DisableDBGStopMode();

  GPIO_InitTypeDef gpio = {.Pin=BTN_ENTER_Pin, .Mode=GPIO_MODE_IT_FALLING, .Pull=GPIO_NOPULL};
  __disable_irq();
  HAL_GPIO_Init(BTN_ENTER_GPIO_Port,&gpio);
  __HAL_GPIO_EXTI_CLEAR_IT(BTN_ENTER_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI0_IRQn,2,0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  __enable_irq();
  for (;;) {
    /* Masking interrupts closes the check-to-WFI race: a pending edge still
       wakes WFI, and its ISR runs immediately when interrupts are restored. */
    __disable_irq();
    HAL_SuspendTick();
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    if (!pressed() && !__HAL_GPIO_EXTI_GET_IT(BTN_ENTER_Pin)) {
      __DSB();
      HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    }
    HAL_ResumeTick();
    __enable_irq();
    SystemClock_Config();
    if (pressed()) { HAL_Delay(30); if (pressed()) break; }
  }
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  CLEAR_BIT(EXTI->IMR1,BTN_ENTER_Pin);
  __HAL_GPIO_EXTI_CLEAR_IT(BTN_ENTER_Pin);
  gpio.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(BTN_ENTER_GPIO_Port,&gpio);
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_UART_CLEAR_FLAG(&huart2,UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
  __HAL_UART_SEND_REQ(&huart2,UART_RXDATA_FLUSH_REQUEST);
  SET_BIT(huart2.Instance->CR1,USART_CR1_UE);
}
