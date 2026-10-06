/* Real Emm sender and the actual extracted HAL error callback; only HAL is simulated. */
#include <string.h>
static void *host_copy(void *dst,const void *src,unsigned long long n)
{ unsigned char *d=dst; const unsigned char *s=src; while(n--)*d++=*s++; return dst; }
static void *host_fill(void *dst,int value,unsigned long long n)
{ unsigned char *d=dst; while(n--)*d++=(unsigned char)value; return dst; }
#define memcpy host_copy
#define memset host_fill
#include "../hardware/Emm_v5/Emm_V5.c"
static uint32_t s_canTxErrors;
#include "can_error_callback.inc"
CAN_HandleTypeDef hcan;
uint32_t host_irq_mask;
static uint32_t host_mailbox, host_free, host_sent;
static uint8_t host_add_fail, host_func[64];
static uint8_t host_payload[64][8],host_dlc[64];
static uint32_t host_id[64];
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can,CAN_FilterTypeDef *f) { (void)can;(void)f;return HAL_OK; }
int HAL_CAN_Start(CAN_HandleTypeDef *can) { (void)can;return HAL_OK; }
int HAL_CAN_Stop(CAN_HandleTypeDef *can) { (void)can;return HAL_OK; }
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can,uint32_t n) { (void)can;(void)n;return HAL_OK; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can) { (void)can;return host_free; }
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can,CAN_TxHeaderTypeDef *h,uint8_t *d,uint32_t *m)
{
  (void)can;
  if(host_add_fail || !host_free)return 1;
  *m=host_mailbox; host_free=0;
  if(host_sent<64){ host_func[host_sent]=d[0];host_id[host_sent]=h->ExtId;
    host_copy(host_payload[host_sent],d,h->DLC);host_dlc[host_sent]=(uint8_t)h->DLC; }
  ++host_sent;return HAL_OK;
}
void Error_Handler(void) { }
static void complete(uint32_t mailbox)
{
  host_free=1;
  if(mailbox==1)HAL_CAN_TxMailbox0CompleteCallback(&hcan);
  if(mailbox==2)HAL_CAN_TxMailbox1CompleteCallback(&hcan);
  if(mailbox==4)HAL_CAN_TxMailbox2CompleteCallback(&hcan);
}
static void fail(uint32_t code)
{ host_free=1;hcan.ErrorCode=code;HAL_CAN_ErrorCallback(&hcan); }
static void reset_transport(void)
{
  host_fill(&canCmdQueue,0,sizeof(canCmdQueue));host_fill(&canSendBuffer,0,sizeof(canSendBuffer));
  host_sent=0;host_free=1;host_mailbox=1;host_add_fail=0;host_irq_mask=0;s_canTxErrors=0;
}
__declspec(dllexport) int transport_case(int which)
{
  unsigned int i;reset_transport();
  if(which<6)
  {
    host_mailbox=1u<<(which%3);
    Emm_V5_Stop_Now(1,0);
    Emm_V5_Read_Sys_Params(1,S_CPOS);Emm_V5_Read_Sys_Params(2,S_CPOS);
    fail(which<3 ? (HAL_CAN_ERROR_TX_ALST0<<(which%3)) : (HAL_CAN_ERROR_TX_TERR0<<(which%3)));
    /* A failed mailbox has no success callback; positions must start from the error path. */
    if(host_sent!=2 || host_func[1]!=0x36)return 0;
    for(i=0;i<8 && canSendBuffer.sending;++i)complete(host_mailbox);
    return host_sent==3 && canCmdQueue.count==0 && !canSendBuffer.sending && s_canTxErrors==1;
  }
  if(which==6) /* Reset while first FD fragment is in hardware; old IRQ must not skip new FD. */
  {
    Emm_V5_Pos_Control(1,1,45,50,6400,0,1);
    CAN_Send_Reset();
    Emm_V5_Pos_Control(2,0,45,50,6400,0,1);
    if(host_sent!=1)return 0;
    complete(1);
    if(host_sent!=2 || host_id[1]!=0x200)return 0;
    complete(1);
    if(host_sent!=3 || host_id[2]!=0x201)return 0;
    complete(1);
    return !canSendBuffer.sending && !canCmdQueue.count;
  }
  if(which==7) /* Error from a discarded command must not discard replacement Stop. */
  {
    Emm_V5_Pos_Control(1,1,45,50,6400,0,1);CAN_Send_Reset();
    Emm_V5_Stop_Now(1,0);Emm_V5_Stop_Now(2,0);
    fail(HAL_CAN_ERROR_TX_TERR0);
    for(i=0;i<4 && canSendBuffer.sending;++i)complete(1);
    return host_sent==3 && host_func[1]==0xFE && host_func[2]==0xFE && !canCmdQueue.count;
  }
  if(which==8) /* A failed FD first fragment must not send its second fragment. */
  {
    Emm_V5_Pos_Control(1,1,45,50,6400,0,1);Emm_V5_Read_Sys_Params(2,S_CPOS);
    fail(HAL_CAN_ERROR_TX_TERR0);complete(1);
    return host_sent==2 && host_func[1]==0x36 && host_id[1]==0x200 && !canCmdQueue.count;
  }
  if(which==9) /* No free mailbox: foreground pump starts queued command later. */
  {
    host_free=0;Emm_V5_Read_Sys_Params(1,S_CPOS);
    if(host_sent || canSendBuffer.sending)return 0;
    host_free=1;CAN_Send_Poll();complete(1);
    return host_sent==1 && !canCmdQueue.count;
  }
  if(which==10) /* First HAL Add failure must remain retryable without a second enqueue. */
  {
    host_add_fail=1;Emm_V5_Read_Sys_Params(1,S_CPOS);
    if(host_sent || canSendBuffer.sending)return 0;
    host_add_fail=0;CAN_Send_Poll();complete(1);
    return host_sent==1 && !canCmdQueue.count;
  }
  if(which>=11 && which<=13) /* Aborted frame has no success callback. */
  {
    host_mailbox=1u<<(which-11);
    Emm_V5_Stop_Now(1,0);Emm_V5_Read_Sys_Params(2,S_CPOS);host_free=1;
    if(which==11)HAL_CAN_TxMailbox0AbortCallback(&hcan);
    if(which==12)HAL_CAN_TxMailbox1AbortCallback(&hcan);
    if(which==13)HAL_CAN_TxMailbox2AbortCallback(&hcan);
    if(host_sent!=2)return 0;
    complete(host_mailbox);return !canCmdQueue.count;
  }
  if(which==14) /* Unrelated mailbox success/error cannot advance the owned fragment. */
  {
    Emm_V5_Pos_Control(1,1,45,50,6400,0,1);
    HAL_CAN_TxMailbox1CompleteCallback(&hcan);fail(HAL_CAN_ERROR_TX_TERR1);
    if(host_sent!=1 || canSendBuffer.currentPacket!=0)return 0;
    complete(1);complete(1);return host_sent==2 && !canCmdQueue.count;
  }
  if(which==15) /* Preserve caller's disabled IRQ mask across enqueue/reset/pump/error. */
  {
    host_irq_mask=1;Emm_V5_Stop_Now(1,0);
    if(host_irq_mask!=1)return 0;
    CAN_Send_Reset();CAN_Send_Poll();fail(HAL_CAN_ERROR_TX_TERR0);
    return host_irq_mask==1;
  }
  if(which==16) /* HAL rejects second fragment: discard it; do not restart partial FD. */
  {
    Emm_V5_Pos_Control(1,1,45,50,6400,0,1);Emm_V5_Read_Sys_Params(2,S_CPOS);
    host_add_fail=1;complete(1);host_add_fail=0;CAN_Send_Poll();complete(1);
    return host_sent==2 && host_func[1]==0x36 && !canCmdQueue.count;
  }
  if(which==17) /* Invalid tiny and oversized commands cannot underflow fragment length. */
  {
    uint8_t cmd[65]={1,0x36,0x6B};
    return !can_EnqueueCmd(cmd,2) && !can_EnqueueCmd(cmd,65) && !host_sent && !canCmdQueue.count;
  }
  if(which==18) /* Real wire encoding of absolute, hung, finite stream target. */
  {
    Emm_V5_Pos_Control(1,1,42,255,0x12345678u,1,1);complete(1);complete(1);
    return host_sent==2 && host_id[0]==0x100 && host_id[1]==0x101 &&
      host_dlc[0]==8 && host_dlc[1]==5 && host_payload[0][0]==0xFD &&
      host_payload[0][1]==1 && host_payload[0][2]==0 && host_payload[0][3]==42 &&
      host_payload[0][4]==255 && host_payload[0][5]==0x12 &&
      host_payload[0][6]==0x34 && host_payload[0][7]==0x56 &&
      host_payload[1][0]==0xFD && host_payload[1][1]==0x78 &&
      host_payload[1][2]==1 && host_payload[1][3]==1 && host_payload[1][4]==0x6B;
  }
  return 0;
}
