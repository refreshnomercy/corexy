/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "corexy.h"

void CAN_Filter_Init(void);

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  /* ---------------- CAN 速率（必须与两个 Emm_V5 驱动器菜单里的 CAN 速率一致）-------
     APB1 = 36MHz；HAL 把 Prescaler 写成 BRP = Prescaler-1，所以实际分频就是 Prescaler：
         位时间 = 12 tq × Prescaler / 36MHz   （BS1=9TQ, BS2=2TQ -> 1+9+2 = 12 tq）
             Prescaler = 3   -> 1.000 µs -> 1 Mbps    （当前）
             Prescaler = 6   -> 2.000 µs -> 500 kbps  ← 只改这一个数即可降到 500K
             Prescaler = 12  -> 4.000 µs -> 250 kbps
     ★1 Mbps 对终端电阻和分支线长很敏感★：如果状态行里出现
       REC(接收错误计数) 明显不为 0、LEC=1/2/6、TEC 持续增长，基本就是物理层在出错
       （缺 120Ω 终端、线太长/分支太长、屏蔽没接），把两边一起降到 500K 往往立刻就好。
       LEC=3(应答错) 则是"没有人应答"：速率不一致、驱动器没上电、CANH/CANL 接反。 */
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  /* 自动退出总线关闭：置 ENABLE 后硬件在检测到 128 次连续 11 位隐性位后自行
     恢复通信。原来为 DISABLE，一旦进入 bus-off 就再也发不出任何帧（总线上
     一条报文都没有 = CANH/CANL 灯不亮），只能靠断电重启。 */
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_Filter_Init();

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  CAN 接收过滤器配置并启动
  * @note   掩码全 0 = 接收总线上所有帧（标准帧/扩展帧均可），
  *         FIFO0 接收 + 中断通知
  */
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank           = 0;
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh         = 0x0000;
  sFilterConfig.FilterIdLow          = 0x0000;
  sFilterConfig.FilterMaskIdHigh     = 0x0000;
  sFilterConfig.FilterMaskIdLow      = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation     = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
}

/**
  * @brief  CAN RX FIFO0 消息挂起回调
  * @note   由 HAL_CAN_IRQHandler 在 RX0 中断中调用；
  *         电机地址在扩展帧 ID 高 8 位，逐个帧分发给运动学模块解析
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
  {
    corexy_rx_parse((uint8_t)(rxHeader.ExtId >> 8), rxData, rxHeader.DLC);
  }
}

/* 发送错误计数；失败邮箱也必须释放发送队列，否则后续读位置请求会卡住。 */
static uint32_t s_canTxErrors = 0u;   /* 累计发送失败帧数 */
static uint32_t s_canRecover  = 0u;   /* 累计就地恢复次数（一直涨 = 帧在反复失败） */

/**
  * @brief  重新初始化后清零发送失败计数
  */
void CAN_Health_Init(void)
{
  s_canTxErrors = 0u;
  s_canRecover  = 0u;
}

/* 丢弃软件待发命令；已入硬件邮箱的一帧结束后不再发旧命令后续分包。
 * 不重启CAN；会丢弃排队命令，因此不能在正常运动中周期调用。 */
void CAN_Unwedge(void)
{
  CAN_Send_Reset();
}

/**
  * @brief  累计发送失败帧数（无应答/仲裁丢失/被中止）
  */
uint32_t CAN_TxErrorCount(void)
{
  return s_canTxErrors;
}

/**
  * @brief  累计就地恢复次数
  */
uint32_t CAN_RecoveryCount(void)
{
  return s_canRecover;
}

/**
  * @brief  读总线健康指标（直接读 ESR 寄存器）
  * @param  lec   最后一次错误类型：0=无错 1=位填充 2=格式 3=应答(ACK) 4=位隐性
  *               5=位显性 6=CRC
  * @param  boff  1 = 处于总线关闭
  * @param  tec   发送错误计数（每失败一帧 +8，>255 进总线关闭）
  * @param  rec   接收错误计数
  * @note   ★LEC=3 且 TEC 持续增长 = 总线上没有任何节点应答我们的帧★
  *        （速率不一致 / 驱动器没上电 / CANH-CANL 接错 / 缺终端电阻都是这个结果）
  */
void CAN_GetDiag(uint8_t *lec, uint8_t *boff, uint8_t *tec, uint8_t *rec)
{
  uint32_t esr = hcan.Instance->ESR;

  *lec  = (uint8_t)((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
  *boff = ((esr & CAN_ESR_BOFF) != 0u) ? 1u : 0u;
  *tec  = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
  *rec  = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
}

/**
  * @brief  当前是否处于总线关闭
  * @note   直接读 ESR 的 BOFF 位，不依赖是否使能了 bus-off 中断
  */
uint8_t CAN_IsBusOff(void)
{
  return ((hcan.Instance->ESR & CAN_ESR_BOFF) != 0u) ? 1u : 0u;
}

/**
  * @brief  发送错误回调：只计数、只清标志，不在中断里做重配置
  * @note   恢复动作交给主循环的 CAN_HealthRecover()，避免在 CAN 中断上下文里
  *         停/启外设。
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if ((hcan->ErrorCode & (HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2 |
                          HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_ALST2)) != 0u)
  {
    s_canTxErrors++;
    CAN_Send_Error(hcan->ErrorCode);
  }

  hcan->ErrorCode = HAL_CAN_ERROR_NONE;   /* HAL 内部只做 |=，不清零，这里手动清 */
}

/* 仅BOFF总线关闭时重启CAN并清队列；不能因普通累计错误周期重启，避免撤销挂起目标。
 * 返回1表示执行过恢复；本函数不会配置或使能电机。 */
uint8_t CAN_HealthRecover(void)
{
  if (CAN_IsBusOff() == 0u)
  {
    return 0u;
  }

  CAN_Recover();
  s_canTxErrors = 0u;
  s_canRecover++;

  return 1u;
}

/* USER CODE END 1 */

