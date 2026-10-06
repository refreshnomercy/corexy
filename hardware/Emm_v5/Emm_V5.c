/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/
#include "Emm_V5.h"
#include <string.h>
#include <stdbool.h>

/* 恢复路径配置CAN过滤器并开启RX/TX中断通知；正常初始化在Core/Src/can.c。 */
void CAN1_Filter_Bank(void)
{

	__IO uint8_t id_o, im_o; __IO uint16_t id_l, id_h, im_l, im_h;
	id_o = (0x00);
	id_h = (uint16_t)((uint16_t)id_o >> 5);
	id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_ID_EXT;
	im_o = (0x00);
	im_h = (uint16_t)((uint16_t)im_o >> 5);
	im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_ID_EXT;

    CAN_FilterTypeDef FilterConfig;

    FilterConfig.FilterBank = 1;
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterIdHigh = id_h;
    FilterConfig.FilterIdLow = id_l;
    FilterConfig.FilterMaskIdHigh = im_h;
    FilterConfig.FilterMaskIdLow = im_l;
    FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    FilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
}

#define MAX_CMD_QUEUE 5
#define MAX_PACKETS 10
#define MAX_CMD_LEN 64

typedef struct {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
} CAN_Packet_t;
typedef struct { uint8_t data[MAX_CMD_LEN], len; } CAN_Cmd_t;
typedef struct {
    CAN_Cmd_t cmds[MAX_CMD_QUEUE];
    uint8_t head, tail, count;
} CAN_CmdQueue_t;
typedef struct {
    CAN_Packet_t packets[MAX_PACKETS];
    uint8_t totalPackets, currentPacket;
    bool sending, cancelled;
    uint32_t mailbox;
} CAN_SendBuffer_t;

static CAN_CmdQueue_t canCmdQueue = {0};
static CAN_SendBuffer_t canSendBuffer = {0};
static void can_SendNextCmd(void);

/* Caller holds PRIMASK: foreground enqueue and TX IRQ share these structures. */
static void can_DropHead(void)
{
    if(canCmdQueue.count) {
        canCmdQueue.head = (canCmdQueue.head + 1u) % MAX_CMD_QUEUE;
        --canCmdQueue.count;
    }
}

/* Discard queued commands but retain ownership of an already submitted frame.
 * Its late completion must never advance the replacement command/fragment. */
void CAN_Send_Reset(void)
{
    uint32_t lock = __get_PRIMASK();
    __disable_irq();
    memset(&canCmdQueue, 0, sizeof(canCmdQueue));
    if(canSendBuffer.sending) canSendBuffer.cancelled = true;
    else memset(&canSendBuffer, 0, sizeof(canSendBuffer));
    __set_PRIMASK(lock);
}

void CAN_Recover(void)
{
    HAL_CAN_Stop(&hcan);
    CAN_Send_Reset();
    CAN1_Filter_Bank();
}

/* Copy and publish atomically; restore the previous interrupt mask. */
bool can_EnqueueCmd(__IO uint8_t *cmd, uint8_t len)
{
    uint32_t lock = __get_PRIMASK();
    uint8_t idx;
    __disable_irq();
    if(canCmdQueue.count >= MAX_CMD_QUEUE || len < 3u || len > MAX_CMD_LEN) {
        __set_PRIMASK(lock);
        return false;
    }
    idx = canCmdQueue.tail;
    memcpy(canCmdQueue.cmds[idx].data, (const void *)cmd, len);
    canCmdQueue.cmds[idx].len = len;
    canCmdQueue.tail = (canCmdQueue.tail + 1u) % MAX_CMD_QUEUE;
    ++canCmdQueue.count;
    if(!canSendBuffer.sending) can_SendNextCmd();
    __set_PRIMASK(lock);
    return true;
}
void can_SendCmd(__IO uint8_t *cmd, uint8_t len) { (void)can_EnqueueCmd(cmd,len); }

/* Only one owned frame in flight. Do not mark sending until HAL accepts it. */
static void can_SendNextCmd(void)
{
    uint8_t *cmd, remaining, dataOffset=0, packetNum=0, payloadLen, i;
    uint32_t mailbox;
    CAN_Packet_t *pkt;
    if(canSendBuffer.sending || !canCmdQueue.count || !HAL_CAN_GetTxMailboxesFreeLevel(&hcan)) return;
    cmd = canCmdQueue.cmds[canCmdQueue.head].data;
    remaining = canCmdQueue.cmds[canCmdQueue.head].len - 2u;
    canSendBuffer.totalPackets = canSendBuffer.currentPacket = 0u;
    canSendBuffer.cancelled = false;
    while(remaining && canSendBuffer.totalPackets < MAX_PACKETS) {
        pkt = &canSendBuffer.packets[canSendBuffer.totalPackets];
        memset(&pkt->TxHeader,0,sizeof(pkt->TxHeader));
        pkt->TxHeader.ExtId = ((uint32_t)cmd[0] << 8) | packetNum;
        pkt->TxHeader.IDE = CAN_ID_EXT;
        pkt->TxHeader.RTR = CAN_RTR_DATA;
        payloadLen = remaining > 7u ? 7u : remaining;
        pkt->TxHeader.DLC = payloadLen + 1u;
        pkt->TxData[0] = cmd[1];
        for(i=0;i<payloadLen;++i) pkt->TxData[i+1u] = cmd[dataOffset+2u+i];
        remaining -= payloadLen; dataOffset += payloadLen; ++packetNum;
        ++canSendBuffer.totalPackets;
    }
    if(HAL_CAN_AddTxMessage(&hcan,&canSendBuffer.packets[0].TxHeader,
                           canSendBuffer.packets[0].TxData,&mailbox)==HAL_OK) {
        canSendBuffer.mailbox = mailbox;
        canSendBuffer.sending = true;
    }
}

/* Retry a queued first frame when HAL was busy; never replay partial commands. */
void CAN_Send_Poll(void)
{
    uint32_t lock = __get_PRIMASK();
    __disable_irq();
    can_SendNextCmd();
    __set_PRIMASK(lock);
}

/* A failed/aborted frame has no TX-success callback. Drop its whole command,
 * then release the sender so later position/Stop requests are not wedged. */
static void can_TxFailed(uint32_t mailbox)
{
    if(!canSendBuffer.sending || canSendBuffer.mailbox != mailbox) return;
    if(!canSendBuffer.cancelled) can_DropHead();
    canSendBuffer.sending = canSendBuffer.cancelled = false;
    can_SendNextCmd();
}
void CAN_Send_Error(uint32_t code)
{
    uint32_t owned, lock=__get_PRIMASK();
    __disable_irq();
    owned=canSendBuffer.mailbox;
    if((owned==CAN_TX_MAILBOX0 && (code&(HAL_CAN_ERROR_TX_ALST0|HAL_CAN_ERROR_TX_TERR0))) ||
       (owned==CAN_TX_MAILBOX1 && (code&(HAL_CAN_ERROR_TX_ALST1|HAL_CAN_ERROR_TX_TERR1))) ||
       (owned==CAN_TX_MAILBOX2 && (code&(HAL_CAN_ERROR_TX_ALST2|HAL_CAN_ERROR_TX_TERR2))))
        can_TxFailed(owned);
    __set_PRIMASK(lock);
}
static void can_TxCompleteHandler(CAN_HandleTypeDef *can,uint32_t mailbox)
{
    uint32_t nextMailbox;
    if(!canSendBuffer.sending || canSendBuffer.mailbox!=mailbox) return;
    if(canSendBuffer.cancelled) {
        canSendBuffer.sending = canSendBuffer.cancelled = false;
        can_SendNextCmd();
        return;
    }
    ++canSendBuffer.currentPacket;
    if(canSendBuffer.currentPacket < canSendBuffer.totalPackets) {
        CAN_Packet_t *pkt=&canSendBuffer.packets[canSendBuffer.currentPacket];
        if(HAL_CAN_AddTxMessage(can,&pkt->TxHeader,pkt->TxData,&nextMailbox)==HAL_OK)
            canSendBuffer.mailbox=nextMailbox;
        else can_TxFailed(mailbox);
    } else {
        can_DropHead(); canSendBuffer.sending=false;
        can_SendNextCmd();
    }
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *can) { can_TxCompleteHandler(can,CAN_TX_MAILBOX0); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *can) { can_TxCompleteHandler(can,CAN_TX_MAILBOX1); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *can) { can_TxCompleteHandler(can,CAN_TX_MAILBOX2); }
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *can) { (void)can;can_TxFailed(CAN_TX_MAILBOX0); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *can) { (void)can;can_TxFailed(CAN_TX_MAILBOX1); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *can) { (void)can;can_TxFailed(CAN_TX_MAILBOX2); }

/* 发送当前位置清零命令；软件坐标和编码器基准由corexy_set_zero同步复位。 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0A;
  cmd[2] =  0x6D;
  cmd[3] =  0x6B;

  can_SendCmd(cmd, 4);
}

/* 发送解除堵转保护命令；恢复使能前需调用，不代表已经重新使能。 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0E;
  cmd[2] =  0x52;
  cmd[3] =  0x6B;

  can_SendCmd(cmd, 4);
}

/* 参数枚举转协议功能码；CPHA=27，CPOS=36，FLAG=3A，尾字节6B。 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  switch(s)
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;

  can_SendCmd(cmd, i);
}

/* 命令[addr,46,69,svF,mode,6B]；mode=2闭环，svF=false不保存配置。 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x46;
  cmd[2] =  0x69;
  cmd[3] =  svF;
  cmd[4] =  ctrl_mode;
  cmd[5] =  0x6B;

  can_SendCmd(cmd, 6);
}

/* 命令[addr,F3,AB,state,snF,6B]；state=1使能、0卸力。
 * snF=false立即执行，true挂起待广播；本项目使能/卸力均使用立即执行。
 * 地址进入CAN扩展ID，发送数据为F3 AB state snF 6B；本函数不等待ACK。 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF3;
  cmd[2] =  0xAB;
  cmd[3] =  (uint8_t)state;
  cmd[4] =  snF;
  cmd[5] =  0x6B;

  can_SendCmd(cmd, 6);
}

/* FD位置命令：dir方向，vel整数RPM，acc加速度档位，clk命令脉冲。
 * raF=false为相对位移，snF=true挂起待同步广播；本项目XY使用这两个参数。 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xFD;
  cmd[2]  =  dir;
  cmd[3]  =  (uint8_t)(vel >> 8);
  cmd[4]  =  (uint8_t)(vel >> 0);
  cmd[5]  =  acc;
  cmd[6]  =  (uint8_t)(clk >> 24);
  cmd[7]  =  (uint8_t)(clk >> 16);
  cmd[8]  =  (uint8_t)(clk >> 8);
  cmd[9]  =  (uint8_t)(clk >> 0);
  cmd[10] =  raF;
  cmd[11] =  snF;
  cmd[12] =  0x6B;

  can_SendCmd(cmd, 13);
}

/* 命令[addr,FE,98,snF,6B]；本项目snF=false立即停止，仍保持使能。 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFE;
  cmd[2] =  0x98;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  can_SendCmd(cmd, 5);
}

/* 命令[addr,FF,66,6B]；addr=0广播，让两个已挂起的目标同时启动。 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFF;
  cmd[2] =  0x66;
  cmd[3] =  0x6B;

  can_SendCmd(cmd, 4);
}
