#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
typedef struct { void *Instance; struct { uint32_t BaudRate; } Init; } UART_HandleTypeDef;
#define USART1 ((void *)1)
#define USART1_IRQn 37
#define USART2 ((void *)2)
#define USART2_IRQn 38
#define UART_FLAG_TC 1
#define RESET 0
#define __HAL_UART_GET_FLAG(u, f) 1
void HAL_NVIC_SetPriority(int irq, int priority, int sub);
void HAL_NVIC_EnableIRQ(int irq);
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
int HAL_UART_Init(UART_HandleTypeDef *uart);
int HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len, uint32_t timeout);
#define HAL_OK 0
void Error_Handler(void);
#define __IO volatile
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
static uint32_t __get_PRIMASK(void) { return 0u; }
static void __disable_irq(void) { }
static void __set_PRIMASK(uint32_t v) { (void)v; }
#endif
