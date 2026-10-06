#ifndef HMI_HOST_MAIN_H
#define HMI_HOST_MAIN_H
#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
#define USART2 ((void *)2)
#define USART2_IRQn 38
#define USART3 ((void *)3)
#define USART3_IRQn 39
#define UART_FLAG_TC 1
#define RESET 0
#define HAL_OK 0
#define __HAL_UART_GET_FLAG(u, f) 1
static uint32_t host_irq_mask;
static uint32_t __get_PRIMASK(void) { return host_irq_mask; }
static void __disable_irq(void) { host_irq_mask = 1; }
static void __set_PRIMASK(uint32_t mask) { host_irq_mask = mask; }
void HAL_NVIC_SetPriority(int irq, int preempt, int sub);
void HAL_NVIC_EnableIRQ(int irq);
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
uint32_t HAL_GetTick(void);
#endif
