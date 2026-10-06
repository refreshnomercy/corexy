/* TJC屏PA2发送→屏RX，PA3接收←屏TX，USART2 115200 8N1。 */
#ifndef __TJC_USART_HMI_H
#define __TJC_USART_HMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define TJC_UART          huart2
#define TJC_UART_INS      USART2
#define TJC_UART_IRQn     USART2_IRQn
extern UART_HandleTypeDef huart2;

#define RINGBUFFER_LEN    500u

void tjc_init(void);

typedef enum
{
    TJC_EVENT_NONE = 0,
    TJC_EVENT_OK,
    TJC_EVENT_TOUCH,
    TJC_EVENT_POSITION_REQUEST
} TJC_EventType_t;

typedef struct
{
    TJC_EventType_t type;
    uint16_t x;
    uint16_t y;
} TJC_Event_t;

uint8_t tjc_take_event(TJC_Event_t *event);

/* 整包复制到模块缓冲后中断发送。1=已接收，0=忙/过长；调用者可稍后重试。 */
uint8_t tjc_send_string(char *str);
uint8_t tjc_send_txt(char *objname, char *attribute, char *txt);
uint8_t tjc_send_val(char *objname, char *attribute, int val);
uint8_t tjc_send_nstring(char *str, unsigned char str_length);
uint8_t tjc_send_position(uint16_t px, uint16_t py, uint8_t valid);

void     initRingBuffer(void);
void     write1ByteToRingBuffer(uint8_t data);
void     deleteRingBuffer(uint16_t size);
uint16_t getRingBufferLength(void);
uint8_t  read1ByteFromRingBuffer(uint16_t position);

extern uint8_t RxBuffer[1];

#define usize      getRingBufferLength()
#define code_c()   initRingBuffer()
#define udelete(x) deleteRingBuffer(x)
#define u(x)       read1ByteFromRingBuffer(x)

#ifdef __cplusplus
}
#endif

#endif
