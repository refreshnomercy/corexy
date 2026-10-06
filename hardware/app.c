/* 应用调度：按键、自检提示、串口屏、CAN健康与蓝牙。 */
#include "app.h"
#include "main.h"
#include "can.h"
#include "LED.h"
#include "Key.h"
#include "Buzzer.h"
#include "corexy.h"
#include "bt.h"
#include "tjc_usart_hmi.h"
#include "hmi.h"
#include "motion.h"

#define JOG_STEP_MM          10.0f
#define JOG_FEED             20.0f
#define AUTO_CALIBRATE       0u
#define CAN_CHECK_PERIOD_MS  200u

static uint32_t s_can_tick;
static uint32_t s_can_error_seen, s_recovery_error_seen;

static void App_Beep(uint32_t ms)
{
  Buzzer_On();
  HAL_Delay(ms);
  Buzzer_Off();
}

/* 仅KEY1自检前恢复；累计错误用已处理基准比较，旧错误不反复撤销标定。 */
static uint8_t App_RecoverIfNeeded(void)
{
  uint32_t tx_error = CAN_TxErrorCount();
  uint8_t recovered = CAN_HealthRecover();

  if (tx_error != s_recovery_error_seen || corexy_watch_stalled() != 0u) recovered = 1u;
  if (recovered != 0u) corexy_rearm_drivers();
  s_recovery_error_seen = s_can_error_seen = CAN_TxErrorCount();
  return recovered;
}

/* KEY1启动阻塞自检；仅返回DONE时点亮LED。 */
static void App_RunCalibration(void)
{
  LED_Off();
  Motion_Stop();
  HMI_DiscardInput(); BT_DiscardInput();
  if (corexy_home_and_calibrate() == COREXY_CAL_DONE) LED_On();
  HMI_DiscardInput(); BT_DiscardInput();
}

/* KEY1必须空闲时启动自检；KEY2提交+Y点动，不再阻塞普通主循环。 */
static void App_HandleKey(KEY_Event_t event)
{
  if (event != KEY1_PRESS && event != KEY2_PRESS) return;

  if (event == KEY1_PRESS)
  {
    if (Motion_IsBusy()) { BT_Tag("CAL REJECT: moving, send S first"); return; }
    App_Beep(30u);
    if (App_RecoverIfNeeded() != 0u) BT_Tag("-- recovered (CAN re-armed) --");
    App_RunCalibration();
  }
  else (void)Motion_SubmitRelative(0.0f, JOG_STEP_MM, JOG_FEED, "KEY2/Y");
}

/* BOFF恢复后立即停止卸力并撤销标定；运动中新发送错误同样锁存故障。
 * 普通失联由位置监视300ms门限处理，不因历史错误反复重配驱动器。 */
static void App_CheckCan(void)
{
  if (HAL_GetTick() - s_can_tick < CAN_CHECK_PERIOD_MS) return;
  s_can_tick = HAL_GetTick();
  if (CAN_IsBusOff())
  {
    (void)CAN_HealthRecover();
    corexy_fault_stop(7u);
    HMI_DiscardInput();
  }
  else if (CAN_TxErrorCount() != s_can_error_seen && Motion_IsBusy())
    corexy_fault_stop(7u);
  s_can_error_seen = CAN_TxErrorCount();
  if (!Motion_IsBusy() && !corexy_feedback_ok()) CAN_Unwedge();
}

/* 初始化提示设备/串口/任务层→配置电机→CAN诊断；上电仍须KEY1自检。 */
void App_Init(void)
{
  s_can_tick = 0u;
  s_can_error_seen = s_recovery_error_seen = 0u;
  LED_Init();
  KEY_Init();
  Buzzer_Init();
  tjc_init();
  BT_Init();
  Motion_Init();
  BT_Tag("=== COREXY BOOT ===");
  HMI_Init();
  BT_Tag("CMDS: X<mm> Y<mm> A<mm> B<mm>  S=stop Z=zero R=status H=help");
  App_Beep(100u);
  corexy_init();
  CAN_Health_Init();
  BT_Status();

#if AUTO_CALIBRATE
  App_RunCalibration();
#endif
}

/* 已收到的S先于轨迹更新；ACK等待期间新收到的S也先于待执行任务启动。
 * 普通定位持续轮询，自检仍为阻塞调用，前后丢弃缓存输入。 */
void App_Poll(void)
{
  BT_Poll();
  corexy_poll_status();
  HMI_Poll();
  App_HandleKey(KEY_Scan());
  if (corexy_take_boundary_hit() != 0u)
  {
    BT_Tag("-- BOUNDARY REACHED: current > threshold, stopped and retreated --");
    BT_Status();
    App_Beep(150u);
  }
  App_CheckCan();
  BT_Poll();
  Motion_Poll();
  if (!corexy_is_calibrated()) LED_Off();
  HMI_PublishPosition();
}
