/* USART1蓝牙接口，固定115200，命令以CR/LF结束。 */
#ifndef __BT_H
#define __BT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BT_BAUD             115200u
#define BT_ECHO             1u
#define BT_STATUS_PERIOD_MS 500u

void BT_Init(void);
void BT_Poll(void);
void BT_DiscardInput(void);
void BT_UartRxCallback(UART_HandleTypeDef *uart);
void BT_UartErrorCallback(UART_HandleTypeDef *uart);

void BT_Puts(const char *s);
void BT_PutInt(int32_t v);
void BT_PutCm(float mm);
void BT_PutMm(float mm);
void BT_Tag(const char *tag); /* 单独输出一行，会清空当前拼接缓冲 */
void BT_NewLine(void);

void BT_Status(void);

void BT_Dimensions(void);

void BT_ReportMove(const char *what, int32_t a0, int32_t b0, int32_t ea0, int32_t eb0);

#ifdef __cplusplus
}
#endif

#endif
