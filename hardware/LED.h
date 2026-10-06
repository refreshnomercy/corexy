#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LED_GPIO_PORT         LED_GPIO_Port
#define LED_GPIO_PIN          LED_Pin

#define LED_ACTIVE_LEVEL      GPIO_PIN_RESET
#define LED_INACTIVE_LEVEL    GPIO_PIN_SET

void LED_Init(void);
void LED_On(void);
void LED_Off(void);

#ifdef __cplusplus
}
#endif

#endif
