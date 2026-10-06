/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

/* CAN 发送通道自检与恢复（实现见 can.c 末尾） -------------------------------*/
void     CAN_Health_Init(void);      /* 清零发送失败计数（可在重新初始化后调用） */
uint32_t CAN_TxErrorCount(void);     /* 累计发送失败帧数（无应答/仲裁丢失/被中止） */
uint32_t CAN_RecoveryCount(void);    /* 累计就地恢复次数（一直涨 = 帧在反复失败） */
void     CAN_GetDiag(uint8_t *lec, uint8_t *boff, uint8_t *tec, uint8_t *rec);
                                     /* 总线健康指标：错误类型/总线关闭/发送·接收错误计数 */
uint8_t  CAN_IsBusOff(void);         /* 当前是否处于总线关闭 */
void     CAN_Unwedge(void);          /* 只清软件发送队列（轻量解卡，不动外设） */
uint8_t  CAN_HealthRecover(void);    /* 总线关闭时重启外设；返回 1 = 已恢复 */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

