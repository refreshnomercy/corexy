/* PA0 LED，低电平有效。 */
#include "LED.h"

/* 初始化GPIO并默认关闭；On/Off使用头文件定义的有效电平。 */
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin   = LED_GPIO_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

  LED_Off();
}

void LED_On(void)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, LED_ACTIVE_LEVEL);
}

void LED_Off(void)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, LED_INACTIVE_LEVEL);
}
