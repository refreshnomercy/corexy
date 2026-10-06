/* PA15蜂鸣器，低电平有效。 */
#include "Buzzer.h"

/* 初始化GPIO并默认关闭；On/Off使用头文件定义的有效电平。 */
void Buzzer_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin   = BUZZER_GPIO_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

  Buzzer_Off();
}

void Buzzer_On(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, BUZZER_ACTIVE_LEVEL);
}

void Buzzer_Off(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, BUZZER_INACTIVE_LEVEL);
}
