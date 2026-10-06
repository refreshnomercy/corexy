/* 双按键GPIO配置与按下事件接口。 */
#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define KEY1_GPIO_PORT        B1_GPIO_Port
#define KEY1_GPIO_PIN         B1_Pin
#define KEY2_GPIO_PORT        B2_GPIO_Port
#define KEY2_GPIO_PIN         B2_Pin

#define KEY_ACTIVE_LEVEL      GPIO_PIN_RESET
#define KEY_INACTIVE_LEVEL    GPIO_PIN_SET

#define KEY_DEBOUNCE_MS       20u

typedef enum
{
  KEY_NONE    = 0,
  KEY1_PRESS  = 1,
  KEY2_PRESS  = 2,
} KEY_Event_t;

void        KEY_Init(void);
KEY_Event_t KEY_Scan(void);

#ifdef __cplusplus
}
#endif

#endif
