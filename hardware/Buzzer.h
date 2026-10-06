#ifndef __BUZZER_H
#define __BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BUZZER_GPIO_PORT       BUZZER_GPIO_Port
#define BUZZER_GPIO_PIN        BUZZER_Pin

#define BUZZER_ACTIVE_LEVEL    GPIO_PIN_RESET
#define BUZZER_INACTIVE_LEVEL  GPIO_PIN_SET

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

#ifdef __cplusplus
}
#endif

#endif
