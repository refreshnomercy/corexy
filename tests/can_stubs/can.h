#ifndef HOST_CAN_TRANSPORT_H
#define HOST_CAN_TRANSPORT_H
#include <stdint.h>
#define __IO volatile
typedef struct { uint32_t ErrorCode; } CAN_HandleTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t FilterBank, FilterMode, FilterScale, FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow, FilterFIFOAssignment, FilterActivation; } CAN_FilterTypeDef;
extern CAN_HandleTypeDef hcan;
#define CAN_ID_EXT 4u
#define CAN_RTR_DATA 0u
#define CAN_FILTERMODE_IDMASK 0u
#define CAN_FILTERSCALE_32BIT 1u
#define CAN_RX_FIFO0 0u
#define ENABLE 1u
#define HAL_OK 0
#define CAN_IT_RX_FIFO0_MSG_PENDING 2u
#define CAN_IT_TX_MAILBOX_EMPTY 1u
#define CAN_TX_MAILBOX0 1u
#define CAN_TX_MAILBOX1 2u
#define CAN_TX_MAILBOX2 4u
#define HAL_CAN_ERROR_NONE 0u
#define HAL_CAN_ERROR_TX_ALST0 1u
#define HAL_CAN_ERROR_TX_ALST1 2u
#define HAL_CAN_ERROR_TX_ALST2 4u
#define HAL_CAN_ERROR_TX_TERR0 8u
#define HAL_CAN_ERROR_TX_TERR1 16u
#define HAL_CAN_ERROR_TX_TERR2 32u
extern uint32_t host_irq_mask;
static uint32_t __get_PRIMASK(void) { return host_irq_mask; }
static void __disable_irq(void) { host_irq_mask=1u; }
static void __set_PRIMASK(uint32_t mask) { host_irq_mask=mask; }
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, CAN_FilterTypeDef *);
int HAL_CAN_Start(CAN_HandleTypeDef *);
int HAL_CAN_Stop(CAN_HandleTypeDef *);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
void Error_Handler(void);
#endif
