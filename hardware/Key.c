/* PA4/PA5双按键独立消抖，稳定按下生成一次事件。 */
#include "Key.h"

typedef struct
{
  GPIO_TypeDef  *port;
  uint16_t       pin;
  GPIO_PinState  lastRaw;
  GPIO_PinState  stable;
  uint32_t       changeTick;
} KEY_Dev_t;

static KEY_Dev_t s_keyDev[2];

/* 配置上拉输入，消抖初始状态为松开。 */
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t i;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin  = KEY1_GPIO_PIN | KEY2_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  s_keyDev[0].port = KEY1_GPIO_PORT;
  s_keyDev[0].pin  = KEY1_GPIO_PIN;
  s_keyDev[1].port = KEY2_GPIO_PORT;
  s_keyDev[1].pin  = KEY2_GPIO_PIN;

  for (i = 0; i < 2; i++)
  {
    s_keyDev[i].lastRaw    = KEY_INACTIVE_LEVEL;
    s_keyDev[i].stable     = KEY_INACTIVE_LEVEL;
    s_keyDev[i].changeTick = 0u;
  }
}

/* 电平持续稳定KEY_DEBOUNCE_MS才更新状态，只报告按下沿。 */
static KEY_Event_t KEY_ScanOne(KEY_Dev_t *pKey, KEY_Event_t evt)
{
  GPIO_PinState raw = HAL_GPIO_ReadPin(pKey->port, pKey->pin);

  if (raw != pKey->lastRaw)
  {
    pKey->lastRaw    = raw;
    pKey->changeTick = HAL_GetTick();
    return KEY_NONE;
  }

  if ((HAL_GetTick() - pKey->changeTick) < KEY_DEBOUNCE_MS)
  {
    return KEY_NONE;
  }

  if (raw != pKey->stable)
  {
    pKey->stable = raw;
    if (raw == KEY_ACTIVE_LEVEL)
    {
      return evt;
    }
  }

  return KEY_NONE;
}

/* 主循环周期扫描，返回单个按下事件；不阻塞等待。 */
KEY_Event_t KEY_Scan(void)
{
  KEY_Event_t ev;

  ev = KEY_ScanOne(&s_keyDev[0], KEY1_PRESS);
  if (ev != KEY_NONE)
  {
    return ev;
  }

  ev = KEY_ScanOne(&s_keyDev[1], KEY2_PRESS);
  if (ev != KEY_NONE)
  {
    return ev;
  }

  return KEY_NONE;
}
