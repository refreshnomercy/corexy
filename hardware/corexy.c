/* CoreXY脉冲坐标、双电机同步位置控制、边界回退与四段标定。 */
#include "corexy.h"
#include "bt.h"
#include "Buzzer.h"
#include "line_plan.h"
#include <math.h>

#define FEEDBACK_TIMEOUT_MS    1000u

static struct
{
  int32_t a_pulse; /* 已下发目标的累计命令脉冲 */
  int32_t b_pulse;
  float   x;
  float   y;

  volatile int32_t a_read; /* 相对基准的累计编码器计数，由CAN中断更新 */
  volatile int32_t b_read;
  volatile uint8_t a_flag;
  volatile uint8_t b_flag;
  volatile uint8_t a_org;
  volatile uint8_t b_org;

  uint32_t last_tick;
} s_corexy;

static CoreXY_CalState_t s_cal_state = COREXY_CAL_IDLE;
static float s_x_max = 0.0f;
static float s_y_max = 0.0f;

static uint8_t s_enabled = 1u; /* 软件运动门禁 */

static uint8_t s_motion_fault = 0u; /* 当前边界/回退故障锁存 */

static volatile int32_t s_cont_a = 0, s_cont_b = 0;
static volatile uint8_t s_cont_a_v = 0u, s_cont_b_v = 0u;
static volatile int32_t s_pos_origin[2] = {0, 0};
static volatile uint32_t s_pos_tick[2] = {0u, 0u};
static volatile uint32_t s_pos_count[2] = {0u, 0u};
static volatile uint32_t s_cur_tick[2] = {0u, 0u};
static volatile uint32_t s_pos_ack[2] = {0u, 0u}; /* FD/02目标接受ACK计数 */
static volatile uint32_t s_stop_ack[2] = {0u, 0u}; /* FE/02停止接受ACK计数 */
static volatile uint32_t s_pos_reply[2] = {0u, 0u};
static volatile uint8_t s_pos_status[2] = {0u, 0u};

static volatile uint16_t s_cpha_a = 0u, s_cpha_b = 0u;
static volatile uint8_t  s_cpha_len_a = 0u, s_cpha_len_b = 0u;
static volatile uint32_t s_cpha_cnt_a = 0u, s_cpha_cnt_b = 0u;

static volatile uint32_t s_rx_a_tick = 0u;
static volatile uint32_t s_rx_b_tick = 0u;

static int8_t  s_cmd_dx = 0, s_cmd_dy = 0;

static float   s_move_x0 = 0.0f, s_move_y0 = 0.0f;
static int32_t s_move_pulse0[2] = {0, 0};

static uint8_t s_drv_en = 1u; /* 软件记录的硬件使能状态 */

static int32_t abs_i32(int32_t v)
{
  return (v < 0) ? (-v) : v;
}

static int32_t cmd_to_enc(int32_t cmd_pulses)
{
  return (int32_t)((float)cmd_pulses * ENC_COUNTS_PER_MM / PULSE_PER_MM);
}

#define BOUND_CUR_MA       COREXY_BOUND_CURRENT_MA
#define BOUND_GRACE_MS     300u /* 起步屏蔽 */
#define BOUND_CONFIRM_N    3u /* 每轴连续不同电流包数 */
#define BOUND_POLL_MS      20u /* 探测/回退轮询节拍 */
#define BOUND_FRESH_PKTS   2u /* 仅使用本段开始后收到的新电流 */
#define BOUND_COOL_MS      400u /* 回退后的边界冷却 */
#define BOUND_RETREAT_MM   5.0f
#define BOUND_RETREAT_FEED 15.0f /* mm/s，短回退需另外计入加减速等待 */
#define RETREAT_STABLE_MS  60u /* 到位后确认停稳 */

#define BOUND_RATE_MS      800u /* 加速后才评估位置窗口速度 */
#define BOUND_RATE_FRAC    0.25f /* 低于命令预期速度的比例 */
#define BOUND_WINDOW_MS    250u
#define BOUND_QUIET_N      2u /* 连续低速窗口数 */
#define BOUND_SAMPLE_MS    300u /* 相应类型的有效位置/电流最大年龄 */

#define BOUND_BY_CURRENT   1u
#define BOUND_BY_STALL     2u
#define BOUND_BAD_FEEDBACK 3u
#define BOUND_NOT_STRAIGHT 4u
#define BOUND_NO_MOTION_ACK 5u
#define BOUND_DRIVE_NOT_READY 6u

static struct
{
  uint8_t  armed;
  uint8_t  cnt[2];
  uint8_t  quiet;
  uint8_t  skew;
  uint8_t  low_rate;
  uint8_t  fault; /* 轨迹/反馈故障，禁止当作边界继续标定 */
  uint8_t  hit; /* 读后清零的边界触发事件 */
  uint32_t t0;
  uint32_t cool_end;
  int32_t  enc_a0; /* 本段编码器起点；回退用Stop后的新样本覆盖 */
  int32_t  enc_b0;
  uint32_t cur_a0;
  uint32_t cur_b0;
  uint32_t cur_seen[2]; /* 防止同一电流包重复累计 */
  uint32_t pos_seen[2];
  uint32_t window_tick[2];
  int32_t  window_enc[2];
  int32_t  need[2]; /* 目标位移幅值，编码器计数 */
  int32_t  start_pulse[2]; /* 发目标之前的命令模型起点 */
  uint32_t move_ms;
  float    feed_mmps;
} s_bound;

static volatile uint32_t s_bound_ms     = 0u;
static volatile uint16_t s_bound_ma     = 0u;
static volatile uint8_t  s_bound_reason = 0u;

static uint32_t s_move_ms   = 0u;
static float    s_last_feed = 0.0f;
static uint8_t  s_retreat_active = 0u;

/* 普通运动异步验收；自检仍沿用原来的探测/回退时序。 */
static struct
{
  CoreXY_MoveState_t state;
  uint8_t launch, motor_test, stable, cur_n[2], line_mode;
  float x, y, feed;
  uint32_t t0, deadline, ramp_ms, base[2], seen[2], cur_seen[2];
  uint32_t ack_base[2], reply_base[2], stable_t0, window_t0;
  int32_t start[2], target[2], delta[2], stable_pos[2], window_pos[2];
} s_run;
static LinePlan_t s_line_plan;
static uint32_t s_line_last_send;
static uint8_t s_line_mask;

static void run_poll(void);

/* 0x36是带符号累计位置；减首次有效样本，保留丢包后的多圈位移。 */
static void enc_update(uint8_t addr, int32_t angle)
{
  uint8_t i = (addr == EMM_ADDR_A) ? 0u : 1u;

  if (addr == EMM_ADDR_A)
  {
    if (s_cont_a_v == 0u) { s_pos_origin[i] = angle; s_cont_a_v = 1u; }
    s_cont_a = angle - s_pos_origin[i];
  }
  else if (addr == EMM_ADDR_B)
  {
    if (s_cont_b_v == 0u) { s_pos_origin[i] = angle; s_cont_b_v = 1u; }
    s_cont_b = angle - s_pos_origin[i];
  }
  else return;
  s_pos_tick[i] = HAL_GetTick();
  s_pos_count[i]++;
}

/* 清发送队列→双轴Stop→分别断使能卸力；s_drv_en=0使下一次运动重新准备驱动器。 */
static void drive_cut_off(void)
{
  CAN_Unwedge();
  HAL_Delay(3);
  Emm_V5_Stop_Now(EMM_ADDR_A, 0u);
  Emm_V5_Stop_Now(EMM_ADDR_B, 0u);
  HAL_Delay(10);
  Emm_V5_En_Control(EMM_ADDR_A, false, false);
  HAL_Delay(10);
  Emm_V5_En_Control(EMM_ADDR_B, false, false);
  HAL_Delay(10);

  s_drv_en = 0u;
}

/* 用本段编码器位移幅值修正接触点，方向来自命令。纯X/Y取双轴较小位移，
 * 从脉冲起点回推，避免编码器极性和浮点尾差混入另一坐标。 */
static void correct_model_after_trip(void)
{
  int32_t da  = abs_i32(s_corexy.a_read - s_bound.enc_a0);
  int32_t db  = abs_i32(s_corexy.b_read - s_bound.enc_b0);
  float   ach = (float)((da < db) ? da : db) / ENC_COUNTS_PER_MM;

  if ((s_cmd_dx == 0) != (s_cmd_dy == 0))
  {
    int32_t dp = (int32_t)(ach * PULSE_PER_MM + 0.5f);
    int32_t sign = (s_cmd_dx != 0) ? s_cmd_dx * COREXY_X_SIGN : s_cmd_dy * COREXY_Y_SIGN;
    s_corexy.a_pulse = s_bound.start_pulse[0] + sign * dp;
#if COREXY_SWAP_XY
    s_corexy.b_pulse = s_bound.start_pulse[1] + ((s_cmd_dx != 0) ? -sign : sign) * dp;
#else
    s_corexy.b_pulse = s_bound.start_pulse[1] + ((s_cmd_dx != 0) ? sign : -sign) * dp;
#endif
    corexy_get_pos(&s_corexy.x, &s_corexy.y);
    return;
  }

  if (s_cmd_dx != 0)
  {
    s_corexy.x = s_move_x0 + (float)s_cmd_dx * ach;
  }
  if (s_cmd_dy != 0)
  {
    s_corexy.y = s_move_y0 + (float)s_cmd_dy * ach;
  }

  s_corexy.a_pulse = (int32_t)((s_corexy.x * COREXY_X_SIGN + s_corexy.y * COREXY_Y_SIGN) * PULSE_PER_MM);
#if COREXY_SWAP_XY
  s_corexy.b_pulse = (int32_t)((s_corexy.y * COREXY_Y_SIGN - s_corexy.x * COREXY_X_SIGN) * PULSE_PER_MM);
#else
  s_corexy.b_pulse = (int32_t)((s_corexy.x * COREXY_X_SIGN - s_corexy.y * COREXY_Y_SIGN) * PULSE_PER_MM);
#endif
}

/* 建立本段位置/电流基准并清故障锁存；冷却期仍记录起点，但不判断碰墙。 */
static void bound_arm(int32_t need_a, int32_t need_b)
{
  s_bound.armed = ((s_bound.cool_end != 0u) &&
                   ((int32_t)(HAL_GetTick() - s_bound.cool_end) < 0)) ? 0u : 1u;
  s_bound.cnt[0]    = s_bound.cnt[1] = 0u;
  s_bound.quiet    = s_bound.skew = s_bound.low_rate = 0u;
  s_bound.fault    = 0u;
  s_motion_fault = 0u;
  s_bound.t0        = HAL_GetTick();
  s_bound.enc_a0    = s_corexy.a_read;
  s_bound.enc_b0    = s_corexy.b_read;
  s_bound.cur_a0    = s_cpha_cnt_a;
  s_bound.cur_b0    = s_cpha_cnt_b;
  s_bound.cur_seen[0] = s_cpha_cnt_a;
  s_bound.cur_seen[1] = s_cpha_cnt_b;
  s_bound.pos_seen[0] = s_pos_count[0];
  s_bound.pos_seen[1] = s_pos_count[1];
  s_bound.window_tick[0] = s_bound.window_tick[1] = s_bound.t0;
  s_bound.window_enc[0] = s_bound.enc_a0;
  s_bound.window_enc[1] = s_bound.enc_b0;
  s_bound.need[0] = need_a;
  s_bound.need[1] = need_b;
  s_bound.start_pulse[0] = s_move_pulse0[0];
  s_bound.start_pulse[1] = s_move_pulse0[1];
  s_bound.move_ms   = s_move_ms;
  s_bound.feed_mmps = s_last_feed;
}

/* 记录原因→停止卸力→修正接触点→反向回退5mm；回退期间关闭边界判断。
 * 自检通过bound_wait_retreat验收到位后才进入下一段，hit仅表示触发回退。 */
static void bound_trigger(uint8_t reason)
{
  float ux, uy, len, tx, ty;

  s_bound_ms     = HAL_GetTick() - s_bound.t0;
  s_bound_ma     = (s_cpha_a > s_cpha_b) ? s_cpha_a : s_cpha_b;
  s_bound_reason = reason;

  s_bound.armed = 0u;
  drive_cut_off();
  BT_Puts("BOUND HIT BR="); BT_PutInt((int32_t)reason);
  BT_Puts(" I="); BT_PutInt((int32_t)s_bound_ma);
  BT_Puts(" TH="); BT_PutInt((int32_t)BOUND_CUR_MA);
  BT_Puts(" elapsed="); BT_PutInt((int32_t)s_bound_ms);
  BT_Puts("ms");
  BT_Puts(" -> reverse retreat 5mm"); BT_NewLine();

  correct_model_after_trip();

  ux  = (float)s_cmd_dx;
  uy  = (float)s_cmd_dy;
  len = sqrtf(ux * ux + uy * uy);
  if (len < 0.001f)
  {
    ux  = 1.0f;
    uy  = 0.0f;
    len = 1.0f;
  }

  tx = s_corexy.x - (ux / len) * BOUND_RETREAT_MM;
  ty = s_corexy.y - (uy / len) * BOUND_RETREAT_MM;

  s_bound.armed     = 0u;
  s_bound.hit       = 1u;
  s_bound.cool_end  = HAL_GetTick() + BOUND_COOL_MS;

  s_retreat_active = 1u;
  corexy_move_to(tx, ty, BOUND_RETREAT_FEED);
  s_bound.armed = 0u;
}

/* 反馈、轨迹或ACK故障：锁存原因、停止卸力；不能当作有效边界继续标定。 */
static void bound_abort(uint8_t reason)
{
  if (corexy_move_busy() != 0u)
  {
    corexy_fault_stop(reason);
    return;
  }
  s_bound.armed = 0u;
  s_bound.fault = reason;
  s_bound.hit = 0u;
  s_bound_ms = HAL_GetTick() - s_bound.t0;
  s_bound_reason = reason;

  s_motion_fault = 1u;
  s_retreat_active = 0u;
  drive_cut_off();
  BT_Puts("CAL ABORT BR="); BT_PutInt((int32_t)reason);
  BT_Puts(" FD_A="); BT_PutInt((int32_t)s_pos_status[0]);
  BT_Puts(" FD_B="); BT_PutInt((int32_t)s_pos_status[1]);
  BT_Puts(" (226=E2, disabled/clogged)"); BT_NewLine();
}

/* s_drv_en=0时清双轴堵转，每轴Enable后立即Stop，尽量缩短旧目标恢复窗口。
 * Enable/Stop紧邻入队，不等待Enable ACK；双轴各收到新0xFE/0x02后置s_drv_en=1。
 * 150ms超时由调用方停止卸力；0xFD必须在使能后发送，否则可能被E2拒绝。 */
static uint8_t drive_prepare_motion(void)
{
  uint32_t sa, sb, t0;
  if (s_drv_en != 0u) return 1u;
  Emm_V5_Reset_Clog_Pro(EMM_ADDR_A); HAL_Delay(5);
  Emm_V5_Reset_Clog_Pro(EMM_ADDR_B); HAL_Delay(5);
  sa = s_stop_ack[0]; sb = s_stop_ack[1];
  Emm_V5_En_Control(EMM_ADDR_A, true, false);
  Emm_V5_Stop_Now(EMM_ADDR_A, false);
  HAL_Delay(5);
  Emm_V5_En_Control(EMM_ADDR_B, true, false);
  Emm_V5_Stop_Now(EMM_ADDR_B, false);
  HAL_Delay(5);
  t0 = HAL_GetTick();
  while ((s_stop_ack[0] == sa) || (s_stop_ack[1] == sb))
  {
    if ((HAL_GetTick() - t0) >= 150u) return 0u;
    HAL_Delay(1);
  }
  s_drv_en = 1u;
  BT_Tag("DRIVE READY: enabled, old target stopped; load new motion");
  return 1u;
}

/* 双轴Stop确认之后、回退目标下发之前，各取得两个新位置包作为回退起点。
 * 不能把碰墙卸力造成的读数变化算入5mm回退。 */
static uint8_t retreat_read_start(int32_t pos[2], uint32_t count[2])
{
  uint32_t base[2], t0 = HAL_GetTick(), lock;
  base[0] = s_pos_count[0]; base[1] = s_pos_count[1];
  for (;;)
  {
    Emm_V5_Read_Sys_Params(EMM_ADDR_A, S_CPOS); HAL_Delay(3u);
    Emm_V5_Read_Sys_Params(EMM_ADDR_B, S_CPOS); HAL_Delay(3u);
    lock = __get_PRIMASK(); __disable_irq();
    pos[0] = s_corexy.a_read; pos[1] = s_corexy.b_read;
    count[0] = s_pos_count[0]; count[1] = s_pos_count[1];
    __set_PRIMASK(lock);
    if (count[0] - base[0] >= 2u && count[1] - base[1] >= 2u) return 1u;
    if (HAL_GetTick() - t0 >= BOUND_SAMPLE_MS)
    {
      BT_Tag("RETREAT BASE FAIL: no fresh stopped positions");
      return 0u;
    }
    HAL_Delay(5u);
  }
}

/* 起步屏蔽后优先判断新电流包：任一轴连续3包>200mA触发回退。
 * 位置窗口提供低速兜底；反馈过期或自检双轴持续不等速按故障停止。
 * 中断数据成组快照，同一包不重复累计；正常终点不再判墙。 */
static void bound_poll(void)
{
  uint32_t now = HAL_GetTick(), el, lock;
  uint32_t pc[2], pt[2], cc[2], ct[2], dt[2];
  int32_t pos[2], delta[2];
  uint16_t cur[2];
  uint8_t i, low[2], moving = 1u, any_current = 0u;
  float expected[2];

  if (s_bound.armed == 0u) return;
  el = now - s_bound.t0;
  if (el < BOUND_GRACE_MS) return;

  if ((s_bound.move_ms == 0u) || (el + 100u >= s_bound.move_ms))
  {
    s_bound.armed = 0u;
    return;
  }

  lock = __get_PRIMASK();
  __disable_irq();
  pos[0] = s_corexy.a_read; pos[1] = s_corexy.b_read;
  cur[0] = s_cpha_a; cur[1] = s_cpha_b;
  cc[0] = s_cpha_cnt_a; cc[1] = s_cpha_cnt_b;
  for (i = 0u; i < 2u; ++i)
  {
    pc[i] = s_pos_count[i]; pt[i] = s_pos_tick[i]; ct[i] = s_cur_tick[i];
  }
  __set_PRIMASK(lock);

  for (i = 0u; i < 2u; ++i)
  {
    uint32_t base_count = (i == 0u) ? s_bound.cur_a0 : s_bound.cur_b0;
    if (((cc[i] - base_count) < BOUND_FRESH_PKTS) ||
        ((now - ct[i]) > BOUND_SAMPLE_MS))
    {
      s_bound.cnt[i] = 0u;
      continue;
    }
    if (cc[i] == s_bound.cur_seen[i]) continue;
    s_bound.cur_seen[i] = cc[i];
    if (cur[i] > BOUND_CUR_MA)
    {
      if (++s_bound.cnt[i] >= BOUND_CONFIRM_N) any_current = 1u;
    }
    else s_bound.cnt[i] = 0u;
  }
  if (any_current != 0u)
  {
    bound_trigger(BOUND_BY_CURRENT);
    return;
  }

  for (i = 0u; i < 2u; ++i)
  {
    if ((s_bound.need[i] != 0) &&
        ((pc[i] == 0u) || ((now - pt[i]) > BOUND_SAMPLE_MS)))
    {
      bound_abort(BOUND_BAD_FEEDBACK);
      return;
    }
  }

  if (el < BOUND_RATE_MS)
  {
    for (i = 0u; i < 2u; ++i)
    {
      s_bound.window_enc[i] = pos[i]; s_bound.window_tick[i] = pt[i];
      s_bound.pos_seen[i] = pc[i];
    }
    return;
  }

  for (i = 0u; i < 2u; ++i)
  {
    dt[i] = pt[i] - s_bound.window_tick[i];
    if ((s_bound.need[i] != 0) &&
        ((pc[i] == s_bound.pos_seen[i]) || (dt[i] < BOUND_WINDOW_MS)))
      moving = 0u;
  }
  if (moving != 0u)
  {
    for (i = 0u; i < 2u; ++i)
    {
      delta[i] = abs_i32(pos[i] - s_bound.window_enc[i]);
      expected[i] = (float)s_bound.need[i] * (float)dt[i] / (float)s_bound.move_ms;
      low[i] = ((s_bound.need[i] != 0) &&
                ((float)delta[i] < expected[i] * BOUND_RATE_FRAC)) ? 1u : 0u;
      s_bound.window_enc[i] = pos[i]; s_bound.window_tick[i] = pt[i];
      s_bound.pos_seen[i] = pc[i];
    }
    if ((s_cal_state == COREXY_CAL_RUNNING) &&
        (s_bound.need[0] != 0) && (s_bound.need[0] == s_bound.need[1]))
    {
      float va = (float)delta[0] / (float)dt[0];
      float vb = (float)delta[1] / (float)dt[1];
      float vmax = (va > vb) ? va : vb;

      if (fabsf(va - vb) > vmax * 0.25f + ENC_COUNTS_PER_MM * 0.5f / BOUND_WINDOW_MS)
      {
        if (++s_bound.skew >= BOUND_QUIET_N)
        {
          bound_abort(BOUND_NOT_STRAIGHT);
          return;
        }
      }
      else s_bound.skew = 0u;
    }
    s_bound.low_rate = (((s_bound.need[0] == 0) || low[0]) &&
                        ((s_bound.need[1] == 0) || low[1])) ? 1u : 0u;
    if (s_bound.low_rate != 0u) ++s_bound.quiet;
    else
    {
      s_bound.quiet = 0u;
    }
  }

  if (s_bound.quiet >= BOUND_QUIET_N) bound_trigger(BOUND_BY_STALL);
}

/* 消费一次边界触发事件，读后清零；不表示回退已通过位移验收。 */
uint8_t corexy_take_boundary_hit(void)
{
  uint8_t h = s_bound.hit;

  s_bound.hit = 0u;
  return h;
}

uint32_t corexy_get_bound_ms(void) { return s_bound_ms; }
uint16_t corexy_get_bound_ma(void) { return s_bound_ma; }
uint8_t  corexy_get_bound_reason(void) { return s_bound_reason; }
uint32_t corexy_get_a_pos_age_ms(void) { return HAL_GetTick() - s_pos_tick[0]; }
uint32_t corexy_get_b_pos_age_ms(void) { return HAL_GetTick() - s_pos_tick[1]; }

/* 撤销旧标定，防止新的摸边目标被旧软限位截断。 */
void corexy_clear_area(void)
{
  s_cal_state = COREXY_CAL_IDLE;
  s_x_max     = 0.0f;
  s_y_max     = 0.0f;
}

/* 保存可移动范围并置DONE，后续目标按此范围钳制。 */
void corexy_set_area(float x_max, float y_max)
{
  if (x_max < 1.0f) x_max = 1.0f;
  if (y_max < 1.0f) y_max = 1.0f;

  s_x_max      = x_max;
  s_y_max      = y_max;
  s_cal_state  = COREXY_CAL_DONE;
}

/* 优先轮询双轴位置，验证方向、目标位移、轴间差和60ms停稳窗口。
 * 等待上限含加减速/反馈余量；位置失效报BR=3，回退不合格报BR=4。 */
static uint8_t bound_wait_retreat(void)
{
  uint32_t t0 = HAL_GetTick(), lock, now, pc[2], pt[2], base[2], seen[2];
  uint32_t stable_t0 = 0u, ramp_ms, deadline;
  int32_t pos[2], previous[2], delta[2], start[2], target[2], stable_pos[2];
  int32_t tol = (int32_t)ENC_COUNTS_PER_MM;
  int32_t quiet = (int32_t)(ENC_COUNTS_PER_MM * 0.1f);
  uint8_t stable = 0u, fresh = 0u, i;

  start[0] = previous[0] = s_bound.enc_a0;
  start[1] = previous[1] = s_bound.enc_b0;
  for (i = 0u; i < 2u; ++i) base[i] = seen[i] = s_bound.pos_seen[i];
  target[0] = cmd_to_enc(s_corexy.a_pulse - s_bound.start_pulse[0]);
  target[1] = cmd_to_enc(s_corexy.b_pulse - s_bound.start_pulse[1]);
  ramp_ms = (uint32_t)(BOUND_RETREAT_FEED * 60.0f / PULLEY_CIRC_MM *
                       (256u - COREXY_ACC) * 50.0f / 1000.0f);
  deadline = corexy_get_move_ms() + 2u * ramp_ms + 2u * BOUND_SAMPLE_MS;

  for (;;)
  {
    corexy_poll_status();
    if (s_bound.fault != 0u) { s_retreat_active = 0u; return 0u; }
    lock = __get_PRIMASK(); __disable_irq();
    pos[0] = s_corexy.a_read; pos[1] = s_corexy.b_read;
    for (i = 0u; i < 2u; ++i) { pc[i] = s_pos_count[i]; pt[i] = s_pos_tick[i]; }
    __set_PRIMASK(lock);
    now = HAL_GetTick();
    fresh = ((pc[0] - base[0] >= 2u) && (pc[1] - base[1] >= 2u) &&
             (now - pt[0] <= BOUND_SAMPLE_MS) &&
             (now - pt[1] <= BOUND_SAMPLE_MS)) ? 1u : 0u;
    delta[0] = pos[0] - start[0]; delta[1] = pos[1] - start[1];
    if (!fresh) stable = 0u;
    if (fresh && pc[0] != seen[0] && pc[1] != seen[1])
    {
      if ((abs_i32(delta[0] - target[0]) <= tol) &&
          (abs_i32(delta[1] - target[1]) <= tol) &&
          (abs_i32(abs_i32(delta[0]) - abs_i32(delta[1])) <= tol) &&
          (abs_i32(pos[0] - previous[0]) <= quiet) &&
          (abs_i32(pos[1] - previous[1]) <= quiet))
      {
        if (!stable)
        {
          stable = 1u; stable_t0 = now;
          stable_pos[0] = pos[0]; stable_pos[1] = pos[1];
        }
        else if (abs_i32(pos[0] - stable_pos[0]) > quiet ||
                 abs_i32(pos[1] - stable_pos[1]) > quiet)
        {
          stable_t0 = now;
          stable_pos[0] = pos[0]; stable_pos[1] = pos[1];
        }
        else if (now - stable_t0 >= RETREAT_STABLE_MS)
        {
          s_retreat_active = 0u;
          BT_Puts("RETREAT OK A=");
          BT_PutInt((int32_t)((float)abs_i32(delta[0]) * 1000.0f / ENC_COUNTS_PER_MM));
          BT_Puts("um B=");
          BT_PutInt((int32_t)((float)abs_i32(delta[1]) * 1000.0f / ENC_COUNTS_PER_MM));
          BT_Puts("um wait="); BT_PutInt((int32_t)(now - t0)); BT_Puts("ms"); BT_NewLine();
          return 1u;
        }
      }
      else stable = 0u;
      for (i = 0u; i < 2u; ++i) { seen[i] = pc[i]; previous[i] = pos[i]; }
    }
    if (now - t0 >= deadline)
    {
      uint8_t error = 0u;
      if (abs_i32(delta[0] - target[0]) > tol) error |= 1u;
      if (abs_i32(delta[1] - target[1]) > tol) error |= 2u;
      if (abs_i32(abs_i32(delta[0]) - abs_i32(delta[1])) > tol) error |= 4u;
      if (!stable || now - stable_t0 < RETREAT_STABLE_MS) error |= 8u;
      if (!fresh) error |= 16u;
      BT_Puts("RETREAT TIMEOUT PA="); BT_PutInt((int32_t)(now - pt[0]));
      BT_Puts(" PB="); BT_PutInt((int32_t)(now - pt[1]));
      BT_Puts(" NEW_A="); BT_PutInt((int32_t)(pc[0] - base[0]));
      BT_Puts(" NEW_B="); BT_PutInt((int32_t)(pc[1] - base[1])); BT_NewLine();
      BT_Puts("RETREAT DIST A=");
      BT_PutInt((int32_t)((float)delta[0] * 1000.0f / ENC_COUNTS_PER_MM));
      BT_Puts("um B=");
      BT_PutInt((int32_t)((float)delta[1] * 1000.0f / ENC_COUNTS_PER_MM));
      BT_Puts("um TA=");
      BT_PutInt((int32_t)((float)target[0] * 1000.0f / ENC_COUNTS_PER_MM));
      BT_Puts("um TB=");
      BT_PutInt((int32_t)((float)target[1] * 1000.0f / ENC_COUNTS_PER_MM));
      BT_Puts("um"); BT_NewLine();
      BT_Puts("RETREAT CHECK ERR="); BT_PutInt((int32_t)error);
      BT_Puts(" wait="); BT_PutInt((int32_t)(now - t0));
      BT_Puts("ms limit="); BT_PutInt((int32_t)deadline); BT_Puts("ms"); BT_NewLine();
      bound_abort(fresh ? BOUND_NOT_STRAIGHT : BOUND_BAD_FEEDBACK);
      return 0u;
    }
    HAL_Delay(5u);
  }
}

/* 仅允许纯X/Y；双轴位置预检→探测边界→验收回退，返回1才可进入下一腿。 */
uint8_t corexy_probe_to_boundary(int8_t dir_x, int8_t dir_y, float max_mm, float feed)
{
  uint32_t t0, lim, ca, cb, diag_tick, tx0;

  if (!(((dir_x == 1 || dir_x == -1) && dir_y == 0) ||
        ((dir_y == 1 || dir_y == -1) && dir_x == 0))) return 0u;
  if ((max_mm <= BOUND_RETREAT_MM) || (feed <= 0.0f) || !s_enabled) return 0u;
  ca = s_pos_count[0]; cb = s_pos_count[1];
  tx0 = CAN_TxErrorCount();
  t0 = HAL_GetTick();
  while (((s_pos_count[0] - ca) < 2u) || ((s_pos_count[1] - cb) < 2u))
  {
    corexy_poll_status();
    if ((HAL_GetTick() - t0) >= 600u)
    {
      BT_Puts("CAL PREFLIGHT POS_N_A="); BT_PutInt((int32_t)(s_pos_count[0] - ca));
      BT_Puts(" POS_N_B="); BT_PutInt((int32_t)(s_pos_count[1] - cb));
      BT_Puts(" PA="); BT_PutInt((int32_t)(HAL_GetTick() - s_pos_tick[0]));
      BT_Puts(" PB="); BT_PutInt((int32_t)(HAL_GetTick() - s_pos_tick[1]));
      BT_Puts(" TX_NEW="); BT_PutInt((int32_t)(CAN_TxErrorCount() - tx0));
      BT_Puts(" TX_TOTAL="); BT_PutInt((int32_t)CAN_TxErrorCount());
      BT_NewLine();
      bound_abort(BOUND_BAD_FEEDBACK);
      return 0u;
    }
    HAL_Delay(5u);
  }

  s_bound.hit      = 0u;
  s_bound.cool_end = 0u;

  corexy_move_rel((float)dir_x * max_mm, (float)dir_y * max_mm, feed);
  if (s_bound.armed == 0u) return 0u;

  lim = corexy_get_move_ms() + 3000u;
  t0  = HAL_GetTick();
  diag_tick = t0;

  while ((HAL_GetTick() - t0) < lim)
  {
    corexy_poll_status();
    if (s_bound.fault != 0u) return 0u;

    if (s_bound.hit != 0u)
    {
      s_bound.hit = 0u;
      return bound_wait_retreat();
    }

    if ((HAL_GetTick() - diag_tick) >= 500u)
    {
      diag_tick = HAL_GetTick();
      BT_Status();
    }
    HAL_Delay(5u);
  }

  s_bound.armed = 0u;
  corexy_stop();
  return 0u;
}

/* 皮带速度(mm/s)转整数RPM；纯X/Y两电机复用同一结果。 */
static uint16_t speed_to_rpm(float speed)
{
  float rpm = speed * 60.0f / PULLEY_CIRC_MM;
  if (rpm < 1.0f)    return 1u;
  if (rpm > 5000.0f) return 5000u;
  return (uint16_t)rpm;
}

uint32_t corexy_get_move_ms(void)
{
  return s_move_ms;
}

static uint32_t move_nominal_ms(float dist_mm, float feed_mmps)
{
  if (feed_mmps < 0.1f) feed_mmps = 0.1f;
  return (uint32_t)(dist_mm / feed_mmps * 1000.0f);
}

/* 上电打开软件门禁→A/B设闭环模式2→分别发立即使能→清驱动计数与软件坐标。
 * 命令间隔20ms；未等待Enable ACK，s_drv_en是下发记录，非实际使能反馈。 */
void corexy_init(void)
{
  s_enabled = 1u;

  Emm_V5_Modify_Ctrl_Mode(EMM_ADDR_A, false, 2);
  HAL_Delay(20);
  Emm_V5_Modify_Ctrl_Mode(EMM_ADDR_B, false, 2);
  HAL_Delay(20);
  Emm_V5_En_Control(EMM_ADDR_A, true, false);
  HAL_Delay(20);
  Emm_V5_En_Control(EMM_ADDR_B, true, false);
  HAL_Delay(20);

  corexy_set_zero();
  HAL_Delay(20);

  s_drv_en = 1u;
}

/* 清故障并撤销标定→停止卸力→清堵转→重设闭环→请求位置。
 * 保留模型不清零，保持硬件断使能；下一次运动由drive_prepare_motion恢复使能。 */
void corexy_rearm_drivers(void)
{
  corexy_watch_clear();

  s_cal_state    = COREXY_CAL_IDLE;
  s_enabled      = 1u;

  drive_cut_off();

  Emm_V5_Reset_Clog_Pro(EMM_ADDR_A);
  HAL_Delay(20);
  Emm_V5_Reset_Clog_Pro(EMM_ADDR_B);
  HAL_Delay(20);

  Emm_V5_Modify_Ctrl_Mode(EMM_ADDR_A, false, 2);
  HAL_Delay(20);
  Emm_V5_Modify_Ctrl_Mode(EMM_ADDR_B, false, 2);
  HAL_Delay(20);

  Emm_V5_Read_Sys_Params(EMM_ADDR_A, S_CPOS);
  HAL_Delay(5);
  Emm_V5_Read_Sys_Params(EMM_ADDR_B, S_CPOS);
  HAL_Delay(60);
}

/* 同步清驱动计数与软件模型；作废编码器基准及旧标定，须重新建立范围。 */
void corexy_set_zero(void)
{
  if (corexy_move_busy()) corexy_stop();
  if (s_motion_fault) return;
  CAN_Unwedge();
  Emm_V5_Reset_CurPos_To_Zero(EMM_ADDR_A);
  HAL_Delay(5u);
  Emm_V5_Reset_CurPos_To_Zero(EMM_ADDR_B);
  HAL_Delay(5u);

  s_cal_state = COREXY_CAL_IDLE;
  s_x_max     = 0.0f;
  s_y_max     = 0.0f;

  s_corexy.a_pulse = 0;
  s_corexy.b_pulse = 0;
  s_corexy.x       = 0.0f;
  s_corexy.y       = 0.0f;

  s_cont_a = 0; s_cont_b = 0;
  /* 驱动器已执行计数清零；第一包也可能已有位移，不能再把第一包当零。 */
  s_pos_origin[0] = s_pos_origin[1] = 0;
  s_cont_a_v = 1u; s_cont_b_v = 1u;
  s_corexy.a_read = 0; s_corexy.b_read = 0;
  s_cpha_a = 0u; s_cpha_b = 0u;
}

/* 门禁/限位→逆运动学和配速→驱动准备→目标下发→同步启动→更新命令模型。
 * 纯X/Y双电机同RPM、位移幅值、加速度；自检拒绝斜线目标。
 * 双轴0xFD挂起(snF=1)，自检确认双方接受ACK后广播同步；回退另取Stop后的起点。
 * 模型坐标先更新为目标，不能表示滑块已经实际到位。 */
void corexy_move_to(float x, float y, float feed)
{
  float   dx, dy, dist;
  int32_t da, db, abs_a, abs_b;
  uint16_t rpm_a, rpm_b;
  uint32_t ack_a, ack_b, ack_t0, reply_a, reply_b;
  int32_t retreat_start[2];
  uint32_t retreat_count[2];
  uint8_t retreat = s_retreat_active;

  if (s_enabled == 0u) return;
  if (s_cal_state == COREXY_CAL_FAILED) return;
  if (corexy_move_busy() != 0u && s_run.launch == 0u) return;

  if (s_cal_state == COREXY_CAL_DONE)
  {
    if (x < 0.0f)    x = 0.0f;
    if (y < 0.0f)    y = 0.0f;
    if (x > s_x_max) x = s_x_max;
    if (y > s_y_max) y = s_y_max;
  }

  dx = x - s_corexy.x;
  dy = y - s_corexy.y;

  if (fabsf(dx) < 0.5f / PULSE_PER_MM) { dx = 0.0f; x = s_corexy.x; }
  if (fabsf(dy) < 0.5f / PULSE_PER_MM) { dy = 0.0f; y = s_corexy.y; }
  if ((s_cal_state == COREXY_CAL_RUNNING) && (dx != 0.0f) && (dy != 0.0f))
  {
    bound_abort(BOUND_NOT_STRAIGHT);
    return;
  }

  s_cmd_dx = (dx > 0.0f) ? (int8_t)1 : ((dx < 0.0f) ? (int8_t)-1 : (int8_t)0);
  s_cmd_dy = (dy > 0.0f) ? (int8_t)1 : ((dy < 0.0f) ? (int8_t)-1 : (int8_t)0);
  s_move_x0 = s_corexy.x;
  s_move_y0 = s_corexy.y;
  s_move_pulse0[0] = s_corexy.a_pulse;
  s_move_pulse0[1] = s_corexy.b_pulse;

  da = (int32_t)((dx * COREXY_X_SIGN + dy * COREXY_Y_SIGN) * PULSE_PER_MM);
#if COREXY_SWAP_XY
  db = (int32_t)((dy * COREXY_Y_SIGN - dx * COREXY_X_SIGN) * PULSE_PER_MM);
#else
  db = (int32_t)((dx * COREXY_X_SIGN - dy * COREXY_Y_SIGN) * PULSE_PER_MM);
#endif
  abs_a = abs_i32(da);
  abs_b = abs_i32(db);

  if (abs_a == 0 && abs_b == 0) return;

  dist = sqrtf(dx * dx + dy * dy);
  if (dist < 0.001f) dist = 0.001f;
  rpm_a = speed_to_rpm((float)abs_a / PULSE_PER_MM / dist * feed);
  rpm_b = speed_to_rpm((float)abs_b / PULSE_PER_MM / dist * feed);
  if ((dx == 0.0f) || (dy == 0.0f))
    rpm_a = rpm_b = speed_to_rpm(feed);

  if (s_cal_state == COREXY_CAL_RUNNING)
  {
    BT_Puts("CAL MOVE A_DIR="); BT_PutInt((da >= 0) ? 0 : 1);
    BT_Puts(" B_DIR="); BT_PutInt((db >= 0) ? 0 : 1);
    BT_Puts(" RPM="); BT_PutInt((int32_t)rpm_a);
    BT_Puts(" TH="); BT_PutInt((int32_t)BOUND_CUR_MA);
    BT_Puts("mA"); BT_NewLine();
  }

  s_move_ms = move_nominal_ms(dist, feed);
  s_last_feed = feed;

  CAN_Unwedge(); /* 清旧读请求，腾出运动命令的队列容量。 */
  HAL_Delay(3);

  if (drive_prepare_motion() == 0u)
  {
    bound_abort(BOUND_DRIVE_NOT_READY);
    return;
  }

  if (retreat && !retreat_read_start(retreat_start, retreat_count))
  {
    bound_abort(BOUND_BAD_FEEDBACK);
    return;
  }

  if (abs_a == 0)
  {
    Emm_V5_Pos_Control(EMM_ADDR_B, (db >= 0) ? 0u : 1u,
                       rpm_b,
                       COREXY_ACC, (uint32_t)abs_b, 0u, s_run.launch);
  }
  else if (abs_b == 0)
  {
    Emm_V5_Pos_Control(EMM_ADDR_A, (da >= 0) ? 0u : 1u,
                       rpm_a,
                       COREXY_ACC, (uint32_t)abs_a, 0u, s_run.launch);
  }
  else
  {
    ack_a = s_pos_ack[0]; ack_b = s_pos_ack[1];
    reply_a = s_pos_reply[0]; reply_b = s_pos_reply[1];
    Emm_V5_Pos_Control(EMM_ADDR_A, (da >= 0) ? 0u : 1u,
                       rpm_a,
                       COREXY_ACC, (uint32_t)abs_a, 0u, 1u);
    HAL_Delay(5);
    Emm_V5_Pos_Control(EMM_ADDR_B, (db >= 0) ? 0u : 1u,
                       rpm_b,
                       COREXY_ACC, (uint32_t)abs_b, 0u, 1u);
    HAL_Delay(5);

    if (s_cal_state == COREXY_CAL_RUNNING || s_run.launch != 0u)
    {
      ack_t0 = HAL_GetTick();
      while ((s_pos_ack[0] == ack_a) || (s_pos_ack[1] == ack_b))
      {
        if (((s_pos_reply[0] != reply_a) && (s_pos_status[0] != 0x02u)) ||
            ((s_pos_reply[1] != reply_b) && (s_pos_status[1] != 0x02u)))
        {
          bound_abort(BOUND_NO_MOTION_ACK);
          return;
        }
        if ((HAL_GetTick() - ack_t0) >= 150u)
        {
          bound_abort(BOUND_NO_MOTION_ACK);
          return;
        }
        HAL_Delay(1u);
      }
    }

    Emm_V5_Synchronous_motion(EMM_ADDR_BROADCAST);
  }

  /* 单电机诊断也挂起目标、确认接受后才启动，静止轴无需FD ACK。 */
  if (s_run.launch != 0u && (abs_a == 0 || abs_b == 0))
  {
    uint8_t axis = (abs_a == 0) ? 1u : 0u;
    ack_t0 = HAL_GetTick();
    while (s_pos_ack[axis] == s_run.ack_base[axis])
    {
      if ((s_pos_reply[axis] != s_run.reply_base[axis] && s_pos_status[axis] != 0x02u) ||
          HAL_GetTick() - ack_t0 >= 150u)
      {
        bound_abort(BOUND_NO_MOTION_ACK);
        return;
      }
      HAL_Delay(1u);
    }
    Emm_V5_Synchronous_motion(EMM_ADDR_BROADCAST);
  }

  s_corexy.a_pulse += da;
  s_corexy.b_pulse += db;
  /* 脉冲量化后统一命令坐标基准，避免下一段把亚脉冲尾差误判为斜线。
   * 这里只更新已下发的命令模型，实际到位仍由新鲜编码器反馈验收。 */
  corexy_get_pos(&s_corexy.x, &s_corexy.y);

  bound_arm(cmd_to_enc(abs_a), cmd_to_enc(abs_b));
  if (s_run.launch != 0u) s_bound.armed = 0u;
  if (retreat)
  {
    s_bound.enc_a0 = retreat_start[0];
    s_bound.enc_b0 = retreat_start[1];
    s_bound.pos_seen[0] = retreat_count[0];
    s_bound.pos_seen[1] = retreat_count[1];
  }
}

/* 以当前命令模型坐标为起点调用绝对接口，继承同一套限位和同步时序。 */
void corexy_move_rel(float dx, float dy, float feed)
{
  corexy_move_to(s_corexy.x + dx, s_corexy.y + dy, feed);
}

/* 双轴Stop并撤销边界/回退监视；保持使能和保持力矩，不等于卸力。 */
void corexy_stop(void)
{
  uint32_t sa = s_stop_ack[0], sb = s_stop_ack[1], t0;
  uint8_t tracked = corexy_move_busy();
  if (tracked != 0u)
  {
    /* 中途停止时模型仍是未到达的目标，撤销标定后才可重新建立可信坐标。 */
    CAN_Unwedge();
    s_run.state = COREXY_MOVE_STOPPED;
    s_run.launch = 0u;
    corexy_clear_area();
  }
  Emm_V5_Stop_Now(EMM_ADDR_A, 0u);
  if (tracked) HAL_Delay(3u);
  Emm_V5_Stop_Now(EMM_ADDR_B, 0u);

  if (tracked)
  {
    t0 = HAL_GetTick();
    while (s_stop_ack[0] == sa || s_stop_ack[1] == sb)
    {
      if (HAL_GetTick() - t0 >= 150u)
      { corexy_fault_stop(BOUND_DRIVE_NOT_READY); break; }
      HAL_Delay(1u);
    }
  }

  s_bound.armed = 0u;
  s_retreat_active = 0u;
}

/* 从命令脉冲模型导出XY(mm)，不是实时编码器测量坐标。 */
void corexy_get_pos(float *x, float *y)
{
  float a = (float)s_corexy.a_pulse;
  float b = (float)s_corexy.b_pulse;

#if COREXY_SWAP_XY
  *x = COREXY_X_SIGN * (a - b) * 0.5f / PULSE_PER_MM;
  *y = COREXY_Y_SIGN * (a + b) * 0.5f / PULSE_PER_MM;
#else
  *x = COREXY_X_SIGN * (a + b) * 0.5f / PULSE_PER_MM;
  *y = COREXY_Y_SIGN * (a - b) * 0.5f / PULSE_PER_MM;
#endif
}

CoreXY_MoveState_t corexy_get_move_state(void) { return s_run.state; }

/* 同一快照转换双编码器的实际XY，避免把已下发的目标当成实时位置。 */
void corexy_get_actual_pos(float *x, float *y)
{
  int32_t a, b;
  uint32_t lock = __get_PRIMASK();
  __disable_irq();
  a = s_corexy.a_read; b = s_corexy.b_read;
  __set_PRIMASK(lock);
#if COREXY_SWAP_XY
  *x = COREXY_X_SIGN * ((float)a - (float)b) * 0.5f / ENC_COUNTS_PER_MM;
  *y = COREXY_Y_SIGN * ((float)a + (float)b) * 0.5f / ENC_COUNTS_PER_MM;
#else
  *x = COREXY_X_SIGN * ((float)a + (float)b) * 0.5f / ENC_COUNTS_PER_MM;
  *y = COREXY_Y_SIGN * ((float)a - (float)b) * 0.5f / ENC_COUNTS_PER_MM;
#endif
}

/* 双轴位置与年龄在同一临界区取样；未收到过位置包也视为无效。 */
uint8_t corexy_get_actual_pos_fresh(float *x, float *y, uint32_t max_age_ms)
{
  int32_t a, b;
  uint8_t fresh;
  uint32_t now, lock;
  if (x == 0 || y == 0) return 0u;
  lock = __get_PRIMASK();
  __disable_irq();
  now = HAL_GetTick();
  fresh = s_pos_count[0] != 0u && s_pos_count[1] != 0u &&
          now - s_pos_tick[0] <= max_age_ms && now - s_pos_tick[1] <= max_age_ms;
  a = s_corexy.a_read; b = s_corexy.b_read;
  __set_PRIMASK(lock);
  if (!fresh) return 0u;
#if COREXY_SWAP_XY
  *x = COREXY_X_SIGN * ((float)a - (float)b) * 0.5f / ENC_COUNTS_PER_MM;
  *y = COREXY_Y_SIGN * ((float)a + (float)b) * 0.5f / ENC_COUNTS_PER_MM;
#else
  *x = COREXY_X_SIGN * ((float)a + (float)b) * 0.5f / ENC_COUNTS_PER_MM;
  *y = COREXY_Y_SIGN * ((float)a - (float)b) * 0.5f / ENC_COUNTS_PER_MM;
#endif
  return 1u;
}

/* 普通定位故障锁存：没有自动回退/续跑，KEY1重新自检才建立可信坐标。 */
void corexy_fault_stop(uint8_t reason)
{
  CoreXY_MoveState_t failed_state = s_run.state;
  s_run.state = COREXY_MOVE_FAULT;
  s_run.launch = 0u;
  s_motion_fault = 1u;
  s_bound_reason = reason;
  s_bound.armed = s_bound.hit = 0u;
  s_retreat_active = 0u;
  corexy_clear_area();
  drive_cut_off();
  BT_Puts("POS FAULT BR="); BT_PutInt(reason);
  BT_Puts(" ST="); BT_PutInt(failed_state);
  BT_Puts(" -> stopped, calibration required"); BT_NewLine();
  BT_Puts("POS CHECK ERR_A="); BT_PutInt(s_corexy.a_read - s_run.target[0]);
  BT_Puts(" ERR_B="); BT_PutInt(s_corexy.b_read - s_run.target[1]);
  BT_Puts(" PA="); BT_PutInt((int32_t)corexy_get_a_pos_age_ms());
  BT_Puts(" PB="); BT_PutInt((int32_t)corexy_get_b_pos_age_ms());
  BT_Puts(" FD_A="); BT_PutInt(s_pos_status[0]);
  BT_Puts(" FD_B="); BT_PutInt(s_pos_status[1]); BT_NewLine();
  if(s_run.line_mode && failed_state==COREXY_MOVE_MOVING) {
    int32_t actual[2];actual[0]=s_corexy.a_read;actual[1]=s_corexy.b_read;
    BT_Puts("POS LINE CHECK CROSS=");BT_PutMm(LinePlan_PathError(&s_line_plan,actual));
    BT_Puts("mm T=");BT_PutInt((int32_t)(HAL_GetTick()-s_run.t0));BT_Puts("ms");BT_NewLine();
  }
}

static uint8_t run_request(float x, float y, float feed, uint8_t motor_test)
{
  uint8_t i;
  if (!s_enabled || s_motion_fault || corexy_move_busy() ||
      s_cal_state == COREXY_CAL_RUNNING || s_cal_state == COREXY_CAL_FAILED ||
      !(feed > 0.0f && feed <= 100.0f)) return 0u;
  s_run.x = x; s_run.y = y; s_run.feed = feed;
  s_run.motor_test = motor_test;
  s_run.line_mode = 0u;
  s_run.launch = s_run.stable = 0u;
  s_run.t0 = HAL_GetTick();
  s_run.target[0] = cmd_to_enc(s_corexy.a_pulse);
  s_run.target[1] = cmd_to_enc(s_corexy.b_pulse);
  for (i = 0u; i < 2u; ++i) s_run.base[i] = s_pos_count[i];
  s_run.state = COREXY_MOVE_PREPARING;
  return 1u;
}

/* 接受一段纯X/Y请求，准备/运行/到位均由主循环推进，拒绝覆盖活动目标。 */
uint8_t corexy_request_axis_move(float x, float y, float feed)
{
  float dx = x - s_corexy.x, dy = y - s_corexy.y;
  if (s_cal_state != COREXY_CAL_DONE ||
      !(x >= 0.0f && x <= s_x_max && y >= 0.0f && y <= s_y_max)) return 0u;
  if (fabsf(dx) >= 0.5f / PULSE_PER_MM && fabsf(dy) >= 0.5f / PULSE_PER_MM)
    return 0u;
  return run_request(x, y, feed, 0u);
}

/* 普通XY允许任意方向；所有保护和到位验收仍由同一个运行状态机执行。 */
uint8_t corexy_request_line_move(float x,float y,float feed)
{
  if(s_cal_state!=COREXY_CAL_DONE ||
     !(x>=0.0f && x<=s_x_max && y>=0.0f && y<=s_y_max))return 0u;
  if(!run_request(x,y,feed,0u))return 0u;
  s_run.line_mode=1u;
  return 1u;
}

/* A/B诊断保留单电机操作，但纳入相同反馈验收，并作废原来的XY标定。 */
uint8_t corexy_request_motor_move(uint8_t motor, int32_t pulse, uint16_t rpm)
{
  float delta = (float)pulse * 0.5f / PULSE_PER_MM;
  float x, y;
  if ((motor != EMM_ADDR_A && motor != EMM_ADDR_B) || pulse == 0 ||
      pulse > 384000 || pulse < -384000 || rpm == 0 || rpm > 150u) return 0u;
  x = s_corexy.x + COREXY_X_SIGN * delta;
  y = s_corexy.y + COREXY_Y_SIGN * delta;
  if (motor == EMM_ADDR_B)
  {
#if COREXY_SWAP_XY
    x = s_corexy.x - COREXY_X_SIGN * delta;
#else
    y = s_corexy.y - COREXY_Y_SIGN * delta;
#endif
  }
  if (!run_request(x, y, (float)rpm * PULLEY_CIRC_MM / 60.0f / 1.41421356f, 1u))
    return 0u;
  corexy_clear_area();
  return 1u;
}

/* 到位后清残留目标，双轴Stop ACK确认完成，再保留标定交下一段。 */
static uint8_t run_finish(void)
{
  uint32_t sa = s_stop_ack[0], sb = s_stop_ack[1], t0;
  CAN_Unwedge();
  Emm_V5_Stop_Now(EMM_ADDR_A, 0u);
  HAL_Delay(3u);
  Emm_V5_Stop_Now(EMM_ADDR_B, 0u);
  t0 = HAL_GetTick();
  while (s_stop_ack[0] == sa || s_stop_ack[1] == sb)
  {
    if (HAL_GetTick() - t0 >= 150u)
    {
      corexy_fault_stop(BOUND_DRIVE_NOT_READY);
      return 0u;
    }
    HAL_Delay(1u);
  }
  s_run.state = COREXY_MOVE_DONE;
  return 1u;
}

/* 挂起有限的绝对位置目标→确认各运动轴新FD ACK→广播同步。
 * 加减速由共同时间曲线规划，驱动档255快速跟随各次RPM更新；自检仍用ACC50。
 * 静止轴不发FD，避免45度单电机运动被另一轴的无关ACK阻塞。 */
static uint8_t line_send(const int32_t pos[2],uint32_t now)
{
  int32_t target[2];
  uint16_t rpm[2];
  uint32_t ack[2],reply[2],t0;
  uint8_t i;
  LinePlan_Sample(&s_line_plan,now-s_run.t0,pos,target,rpm);
  for(i=0u;i<2u;++i){ack[i]=s_pos_ack[i];reply[i]=s_pos_reply[i];}
  for(i=0u;i<2u;++i) {
    if(!(s_line_mask&(1u<<i)))continue;
    Emm_V5_Pos_Control(i==0u ? EMM_ADDR_A : EMM_ADDR_B,target[i]<0 ? 1u : 0u,
                       rpm[i],255u,(uint32_t)abs_i32(target[i]),1u,1u);
    HAL_Delay(2u);
  }
  t0=HAL_GetTick();
  for(;;) {
    uint8_t waiting=0u;
    for(i=0u;i<2u;++i) {
      if(!(s_line_mask&(1u<<i)))continue;
      if(s_pos_reply[i]!=reply[i] && s_pos_status[i]!=0x02u) {
        corexy_fault_stop(BOUND_NO_MOTION_ACK);return 0u;
      }
      if(s_pos_ack[i]==ack[i])waiting=1u;
    }
    if(!waiting)break;
    if(HAL_GetTick()-t0>=150u){corexy_fault_stop(BOUND_NO_MOTION_ACK);return 0u;}
    HAL_Delay(1u);
  }
  if(s_line_mask)Emm_V5_Synchronous_motion(EMM_ADDR_BROADCAST);
  s_line_last_send=now;
  return 1u;
}

static uint8_t line_start(const int32_t pos[2])
{
  int32_t target[2];
  float a,b;
  uint8_t i;
  a=(s_run.x*COREXY_X_SIGN+s_run.y*COREXY_Y_SIGN)*PULSE_PER_MM;
#if COREXY_SWAP_XY
  b=(s_run.y*COREXY_Y_SIGN-s_run.x*COREXY_X_SIGN)*PULSE_PER_MM;
#else
  b=(s_run.x*COREXY_X_SIGN-s_run.y*COREXY_Y_SIGN)*PULSE_PER_MM;
#endif
  target[0]=(int32_t)(a>=0.0f ? a+0.5f : a-0.5f);
  target[1]=(int32_t)(b>=0.0f ? b+0.5f : b-0.5f);
  LinePlan_Init(&s_line_plan,pos,target,s_run.feed);
  s_line_mask=0u;
  for(i=0u;i<2u;++i) {
    s_run.target[i]=cmd_to_enc(target[i]);
    s_run.delta[i]=s_run.target[i]-pos[i];
    if(abs_i32(s_run.delta[i])>=2)s_line_mask|=1u<<i;
  }
  CAN_Unwedge();HAL_Delay(3u);
  if(!drive_prepare_motion()){corexy_fault_stop(BOUND_DRIVE_NOT_READY);return 0u;}
  s_run.t0=s_run.window_t0=HAL_GetTick();
  s_run.ramp_ms=(uint32_t)(s_line_plan.ramp*1000.0f);
  s_move_ms=LinePlan_DurationMs(&s_line_plan);
  s_run.deadline=s_move_ms+2000u;
  if(!line_send(pos,s_run.t0))return 0u;
  s_corexy.a_pulse=target[0];s_corexy.b_pulse=target[1];
  corexy_get_pos(&s_corexy.x,&s_corexy.y);
  s_bound.armed=0u;
  s_run.state=COREXY_MOVE_MOVING;
  BT_Puts("POS LINE L=");BT_PutMm(s_line_plan.length);BT_Puts("mm T=");
  BT_PutInt((int32_t)s_move_ms);BT_Puts("ms MASK=");BT_PutInt(s_line_mask);BT_NewLine();
  return 1u;
}

/* 双轴各取两个新起点包；运行全程检查反馈、方向、同步、电流和到位。
 * 标称时长只用于超时上限，不能作为完成条件；停稳窗使用累计变化量。 */
static void run_poll(void)
{
  uint32_t now = HAL_GetTick(), lock, pc[2], pt[2], cc[2], ct[2];
  int32_t pos[2], error[2], travelled[2];
  int32_t tol = (int32_t)(COREXY_POSITION_TOL_MM * ENC_COUNTS_PER_MM);
  int32_t quiet = (int32_t)(0.05f * ENC_COUNTS_PER_MM);
  uint16_t cur[2];
  uint8_t i, pair_new;

  if (!corexy_move_busy() || s_run.launch) return;
  if (!s_run.motor_test && s_cal_state != COREXY_CAL_DONE)
  { corexy_fault_stop(BOUND_NOT_STRAIGHT); return; }
  lock = __get_PRIMASK(); __disable_irq();
  pos[0] = s_corexy.a_read; pos[1] = s_corexy.b_read;
  cur[0] = s_cpha_a; cur[1] = s_cpha_b;
  cc[0] = s_cpha_cnt_a; cc[1] = s_cpha_cnt_b;
  for (i = 0u; i < 2u; ++i)
  { pc[i] = s_pos_count[i]; pt[i] = s_pos_tick[i]; ct[i] = s_cur_tick[i]; }
  __set_PRIMASK(lock);
  now=HAL_GetTick(); /* 快照期间新包时间可能前进，年龄用快照后的时钟计算。 */

  if (s_run.state == COREXY_MOVE_PREPARING)
  {
    if (pc[0] - s_run.base[0] < 2u || pc[1] - s_run.base[1] < 2u ||
        now - pt[0] > BOUND_SAMPLE_MS || now - pt[1] > BOUND_SAMPLE_MS)
    {
      if (now - s_run.t0 >= 600u) corexy_fault_stop(BOUND_BAD_FEEDBACK);
      return;
    }
    if (abs_i32(pos[0] - cmd_to_enc(s_corexy.a_pulse)) > (int32_t)ENC_COUNTS_PER_MM ||
        abs_i32(pos[1] - cmd_to_enc(s_corexy.b_pulse)) > (int32_t)ENC_COUNTS_PER_MM)
    { corexy_fault_stop(BOUND_NOT_STRAIGHT); return; }
    for (i = 0u; i < 2u; ++i)
    {
      s_run.start[i] = s_run.window_pos[i] = pos[i];
      s_run.base[i] = s_run.seen[i] = pc[i];
      s_run.cur_seen[i] = cc[i]; s_run.cur_n[i] = 0u;
      s_run.ack_base[i] = s_pos_ack[i]; s_run.reply_base[i] = s_pos_reply[i];
    }
    if(s_run.line_mode){(void)line_start(pos);return;}
    s_run.launch = 1u;
    s_move_ms = 0u;
    corexy_move_to(s_run.x, s_run.y, s_run.feed);
    s_run.launch = 0u;
    if (s_run.state == COREXY_MOVE_FAULT) return;
    s_run.target[0] = cmd_to_enc(s_corexy.a_pulse);
    s_run.target[1] = cmd_to_enc(s_corexy.b_pulse);
    for (i = 0u; i < 2u; ++i) s_run.delta[i] = s_run.target[i] - s_run.start[i];
    s_run.t0 = s_run.window_t0 = HAL_GetTick();
    s_run.ramp_ms = (uint32_t)(s_run.feed * 1.41421356f * 60.0f / PULLEY_CIRC_MM *
                            (256u - COREXY_ACC) * 50.0f / 1000.0f);
    s_run.deadline = s_move_ms + 2u * s_run.ramp_ms + 1000u;
    s_run.state = COREXY_MOVE_MOVING;
    return;
  }

  for (i = 0u; i < 2u; ++i)
  {
    if (pc[i] == 0u || now - pt[i] > BOUND_SAMPLE_MS)
    { corexy_fault_stop(BOUND_BAD_FEEDBACK); return; }
    error[i] = abs_i32(pos[i] - s_run.target[i]);
    travelled[i] = pos[i] - s_run.start[i];
    if (now - s_run.t0 >= BOUND_GRACE_MS)
    {
      if (now - ct[i] > BOUND_SAMPLE_MS) s_run.cur_n[i] = 0u;
      else if (cc[i] != s_run.cur_seen[i])
      {
        s_run.cur_seen[i] = cc[i];
        if (cur[i] > BOUND_CUR_MA)
        {
          if (++s_run.cur_n[i] >= BOUND_CONFIRM_N)
          { corexy_fault_stop(BOUND_BY_CURRENT); return; }
        }
        else s_run.cur_n[i] = 0u;
      }
    }
    if ((s_run.delta[i] > tol && travelled[i] < -(int32_t)ENC_COUNTS_PER_MM) ||
        (s_run.delta[i] < -tol && travelled[i] > (int32_t)ENC_COUNTS_PER_MM) ||
        (abs_i32(s_run.delta[i]) <= tol && abs_i32(travelled[i]) > (int32_t)ENC_COUNTS_PER_MM))
    { corexy_fault_stop(BOUND_NOT_STRAIGHT); return; }
  }
  if(s_run.line_mode && LinePlan_PathError(&s_line_plan,pos)>LINE_PLAN_PATH_TOL_MM)
  { corexy_fault_stop(BOUND_NOT_STRAIGHT);return; }
  if (!s_run.line_mode && !s_run.motor_test && now - s_run.t0 > s_run.ramp_ms + 300u &&
      abs_i32(abs_i32(travelled[0]) - abs_i32(travelled[1])) > (int32_t)ENC_COUNTS_PER_MM)
  { corexy_fault_stop(BOUND_NOT_STRAIGHT); return; }

  pair_new = pc[0] != s_run.seen[0] && pc[1] != s_run.seen[1];
  if (pair_new)
  {
    if (error[0] <= tol && error[1] <= tol &&
        pc[0] - s_run.base[0] >= 2u && pc[1] - s_run.base[1] >= 2u &&
        (!s_run.line_mode || now-s_run.t0>=s_move_ms))
    {
      if (!s_run.stable || abs_i32(pos[0] - s_run.stable_pos[0]) > quiet ||
          abs_i32(pos[1] - s_run.stable_pos[1]) > quiet)
      {
        s_run.stable = 1u; s_run.stable_t0 = now;
        s_run.stable_pos[0] = pos[0]; s_run.stable_pos[1] = pos[1];
      }
      else if (now - s_run.stable_t0 >= COREXY_SETTLE_MS)
      { (void)run_finish(); return; }
    }
    else s_run.stable = 0u;
    if (now - s_run.window_t0 >= 500u)
    {
      if (now - s_run.t0 > s_run.ramp_ms + 500u &&
          (error[0] > tol || error[1] > tol) &&
          abs_i32(pos[0] - s_run.window_pos[0]) < (int32_t)(0.1f * ENC_COUNTS_PER_MM) &&
          abs_i32(pos[1] - s_run.window_pos[1]) < (int32_t)(0.1f * ENC_COUNTS_PER_MM))
      { corexy_fault_stop(BOUND_BY_STALL); return; }
      s_run.window_t0 = now;
      s_run.window_pos[0] = pos[0]; s_run.window_pos[1] = pos[1];
    }
    s_run.seen[0] = pc[0]; s_run.seen[1] = pc[1];
  }
  if (now - s_run.t0 >= s_run.deadline) corexy_fault_stop(8u);
  else if(s_run.line_mode && now-s_line_last_send>=LINE_PLAN_UPDATE_MS)
    (void)line_send(pos,now);
}

/* 探测20ms/空闲50ms节拍检查边界并请求反馈；回退优先读位置。
 * 错开双地址请求，避免挤满5条深发送队列；本轮新包到达后再次判断。 */
void corexy_poll_status(void)
{
  static uint8_t poll_step = 0;
  uint32_t now = HAL_GetTick();
  uint32_t period = (s_bound.armed || corexy_move_busy() || s_cal_state == COREXY_CAL_RUNNING) ?
                     BOUND_POLL_MS : COREXY_POLL_MS / 2u;

  CAN_Send_Poll();
  if ((now - s_corexy.last_tick) < period) return;
  s_corexy.last_tick = now;

  bound_poll();

  run_poll();

  if (s_retreat_active)
  {
    Emm_V5_Read_Sys_Params(EMM_ADDR_A, S_CPOS); HAL_Delay(3u);
    Emm_V5_Read_Sys_Params(EMM_ADDR_B, S_CPOS); HAL_Delay(3u);
    return;
  }

  Emm_V5_Read_Sys_Params(EMM_ADDR_A, S_CPHA);
  HAL_Delay(2);
  Emm_V5_Read_Sys_Params(EMM_ADDR_B, S_CPHA);
  HAL_Delay(2);
  Emm_V5_Read_Sys_Params(EMM_ADDR_A, S_CPOS);
  HAL_Delay(2);
  Emm_V5_Read_Sys_Params(EMM_ADDR_B, S_CPOS);

  poll_step++;
  if (poll_step >= 3u)
  {
    poll_step = 0u;
    HAL_Delay(2);
    Emm_V5_Read_Sys_Params(EMM_ADDR_A, S_FLAG);
    Emm_V5_Read_Sys_Params(EMM_ADDR_B, S_FLAG);
  }

  bound_poll();
  run_poll();
}

/* CAN中断分发位置、电流、状态和ACK；校验有效格式后更新样本。
 * FD/02表示目标接受，FE/02表示停止接受，均不表示运动实际到位。 */
void corexy_rx_parse(uint8_t addr, uint8_t *data, uint8_t len)
{
  if (len < 1u) return;

  if (addr == EMM_ADDR_A)      s_rx_a_tick = HAL_GetTick();
  else if (addr == EMM_ADDR_B) s_rx_b_tick = HAL_GetTick();

  switch (data[0])
  {
    case 0xFD:
      if ((len == 3u) && (data[2] == 0x6Bu) &&
          ((data[1] == 0x02u) || (data[1] == 0xE2u)))
      {
        uint8_t i;
        if (addr != EMM_ADDR_A && addr != EMM_ADDR_B) break;
        i = (addr == EMM_ADDR_A) ? 0u : 1u;
        s_pos_status[i] = data[1];
        ++s_pos_reply[i];
        if (data[1] == 0x02u) ++s_pos_ack[i];
      }
      break;
    case 0xFE:
      if ((len == 3u) && (data[1] == 0x02u) && (data[2] == 0x6Bu))
      {
        if (addr == EMM_ADDR_A) ++s_stop_ack[0];
        else if (addr == EMM_ADDR_B) ++s_stop_ack[1];
      }
      break;
    case 0x36:
      if ((len == 7u) && (data[1] <= 1u) && (data[6] == 0x6Bu))
      {
        int32_t ang = (int32_t)(((uint32_t)data[2] << 24) |
                                ((uint32_t)data[3] << 16) |
                                ((uint32_t)data[4] << 8)  |
                                ((uint32_t)data[5]));
        if (data[1] == 1u) ang = -ang;

        enc_update(addr, ang);

        if (addr == EMM_ADDR_A)      s_corexy.a_read = s_cont_a;
        else if (addr == EMM_ADDR_B) s_corexy.b_read = s_cont_b;
      }
      break;

    case 0x27:
      {
        uint16_t ma = 0u;

        if ((len == 4u) && (data[3] == 0x6Bu))
        {
          ma = (uint16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[2]);
          if (addr == EMM_ADDR_A)      { s_cpha_a = ma; s_cpha_len_a = len; s_cur_tick[0] = HAL_GetTick(); s_cpha_cnt_a++; }
          else if (addr == EMM_ADDR_B) { s_cpha_b = ma; s_cpha_len_b = len; s_cur_tick[1] = HAL_GetTick(); s_cpha_cnt_b++; }
        }
      }
      break;

    case 0x3A:
      if (len >= 2u)
      {
        if (addr == EMM_ADDR_A)      s_corexy.a_flag = data[1];
        else if (addr == EMM_ADDR_B) s_corexy.b_flag = data[1];
      }
      break;

    case 0x3B:
      if (len >= 2u)
      {
        if (addr == EMM_ADDR_A)      s_corexy.a_org = data[1];
        else if (addr == EMM_ADDR_B) s_corexy.b_org = data[1];
      }
      break;

    default:
      break;
  }
}

#define CAL_PROBE_MAX_MM   300.0f /* 探测目标大于机械行程，接触后由边界检测停止 */
#define CAL_PROBE_FEED     30.0f /* 当前用户摸边速度，mm/s */
#define CAL_MIN_SPAN_MM    30.0f /* 防止半路误触发建立异常小的范围 */
#define CAL_STEP_BEEP_MS   120u
#define CAL_STEP_GAP_MS    500u

/* 单段验收后播报、提示音和停顿；自检为阻塞调用，屏幕/蓝牙处理会延后。 */
static void cal_step_done(uint8_t step, const char *what)
{
  BT_Puts("   step ");
  BT_PutInt((int32_t)step);
  BT_Puts("/4 done: ");
  BT_Puts(what);
  BT_NewLine();

  Buzzer_On();
  HAL_Delay(CAL_STEP_BEEP_MS);
  Buzzer_Off();

  HAL_Delay(CAL_STEP_GAP_MS);
}

/* 统一四方向探测与回退验收，失败输出BR并交自检终止。 */
static uint8_t cal_probe(int8_t dx, int8_t dy)
{
  uint8_t ok = corexy_probe_to_boundary(dx, dy, CAL_PROBE_MAX_MM, CAL_PROBE_FEED);

  if (ok != 0u)
  {
    BT_Puts("   boundary by=");
    BT_PutInt((int32_t)corexy_get_bound_reason());
    BT_Puts(" current=");
    BT_PutInt((int32_t)corexy_get_bound_ma());
    BT_Puts("mA");
    BT_Puts(" elapsed=");
    BT_PutInt((int32_t)corexy_get_bound_ms());
    BT_Puts("ms");
    BT_NewLine();
  }
  else
  {
    BT_Tag("   !! probe failed: BR=3 position, 4 retreat/motion, 5 position ACK, 6 enable/stop !!");
    BT_Status();
  }

  return ok;
}

/* 阻塞自检：+X→+Y→-X→-Y，各段碰墙后反向回退5mm并验收。
 * 停点差加两侧回退量计算行程；左下角内缩5mm处清零，远端另留3mm余量。
 * 全部成功才置DONE；失败停止卸力并置FAILED，拒绝普通运动。 */
CoreXY_CalState_t corexy_home_and_calibrate(void)
{
  float x1, y1, x2, y2, x3, y3, x4, y4;
  float span_x, span_y, x_max, y_max;

  corexy_stop();
  s_run.state = COREXY_MOVE_IDLE;
  s_run.launch = 0u;
  s_run.line_mode = 0u;
  s_motion_fault = 0u;
  s_bound.hit = 0u;
  s_bound.fault = 0u;
  s_bound_reason = 0u;
  s_bound_ms = 0u;
  s_bound_ma = 0u;
  corexy_clear_area();
  s_cal_state = COREXY_CAL_RUNNING;

  BT_Tag("== CAL REV LINE-CANQ-CUR200: +X retreat, +Y retreat, -X retreat, -Y retreat ==");
  BT_Puts("CAL CFG PROBE="); BT_PutInt((int32_t)CAL_PROBE_FEED);
  BT_Puts("mm/s RETREAT="); BT_PutInt((int32_t)BOUND_RETREAT_FEED);
  BT_Puts("mm/s ACC="); BT_PutInt((int32_t)COREXY_ACC);
  BT_Puts(" CUR_N="); BT_PutInt((int32_t)BOUND_CONFIRM_N); BT_NewLine();
  Buzzer_On();
  HAL_Delay(80);
  Buzzer_Off();

  if (corexy_feedback_ok() == 0u)
  {
    BT_Tag("== CAL ABORT: no CAN feedback from drivers ==");
    BT_Status();
    goto cal_fail;
  }

  BT_Tag("== step 1/4: probe +X until boundary, then retreat ==");
  if (cal_probe(+1, 0) == 0u) goto cal_fail;
  corexy_get_pos(&x1, &y1);
  cal_step_done(1u, "+X wall touched, retreated");

  BT_Tag("== step 2/4: probe +Y until boundary, then retreat ==");
  if (cal_probe(0, +1) == 0u) goto cal_fail;
  corexy_get_pos(&x2, &y2);
  cal_step_done(2u, "+Y wall touched, retreated");

  BT_Tag("== step 3/4: probe -X until boundary, then retreat ==");
  if (cal_probe(-1, 0) == 0u) goto cal_fail;
  corexy_get_pos(&x3, &y3);
  cal_step_done(3u, "-X wall touched, retreated");

  BT_Tag("== step 4/4: probe -Y until boundary, then retreat ==");
  if (cal_probe(0, -1) == 0u) goto cal_fail;
  corexy_get_pos(&x4, &y4);
  cal_step_done(4u, "-Y wall touched -> at origin corner");

  span_x = (x1 + BOUND_RETREAT_MM) - (x3 - BOUND_RETREAT_MM);
  span_y = (y2 + BOUND_RETREAT_MM) - (y4 - BOUND_RETREAT_MM);

  BT_Puts("   measured span: X=");
  BT_PutCm(span_x);
  BT_Puts("cm  Y=");
  BT_PutCm(span_y);
  BT_Puts("cm");
  BT_NewLine();

  if ((span_x < CAL_MIN_SPAN_MM) || (span_y < CAL_MIN_SPAN_MM))
  {
    BT_Tag("== CAL FAILED: measured span too small (judge fired early) ==");
    BT_Tag("   check BOUND_CUR_MA / probe feed on serial, then retry (KEY1)");
    goto cal_fail;
  }

  corexy_set_zero();

  x_max = span_x - BOUND_RETREAT_MM - CAL_SAFE_MARGIN_MM;
  y_max = span_y - BOUND_RETREAT_MM - CAL_SAFE_MARGIN_MM;
  corexy_set_area(x_max, y_max);

  BT_Tag("== CAL DONE: origin = bottom-left, soft limits ACTIVE ==");
  BT_Dimensions();
  Buzzer_On();
  HAL_Delay(500);
  Buzzer_Off();
  return s_cal_state;

cal_fail:
  corexy_stop();
  drive_cut_off();
  Buzzer_On();
  HAL_Delay(800);
  Buzzer_Off();
  s_cal_state = COREXY_CAL_FAILED;
  return s_cal_state;
}

void corexy_get_area(float *x_max, float *y_max)
{
  *x_max = s_x_max;
  *y_max = s_y_max;
}

uint8_t corexy_is_calibrated(void)
{
  return (s_cal_state == COREXY_CAL_DONE) ? 1u : 0u;
}

CoreXY_CalState_t corexy_get_cal_state(void)
{
  return s_cal_state;
}

/* 保留既有调用名：读取当前边界/回退故障锁存，不再使用旧看门狗。 */
uint8_t corexy_watch_stalled(void)
{
  return s_motion_fault;
}

/* 只清运动故障锁存，不使能硬件，也不恢复标定。 */
void corexy_watch_clear(void)
{
  s_motion_fault = 0u;
}

/* 只有新鲜位置反馈到位并停稳后，普通运动才从busy退出。 */
uint8_t corexy_move_busy(void)
{
  return s_run.state == COREXY_MOVE_PREPARING || s_run.state == COREXY_MOVE_MOVING;
}

int32_t corexy_get_a_pulse(void) { return s_corexy.a_pulse; }
int32_t corexy_get_b_pulse(void) { return s_corexy.b_pulse; }
int32_t corexy_get_a_read(void)  { return s_corexy.a_read; }
int32_t corexy_get_b_read(void)  { return s_corexy.b_read; }
uint8_t corexy_get_a_flag(void)  { return s_corexy.a_flag; }
uint8_t corexy_get_b_flag(void)  { return s_corexy.b_flag; }

uint16_t corexy_get_a_current(void) { return s_cpha_a; }
uint16_t corexy_get_b_current(void) { return s_cpha_b; }
uint8_t  corexy_get_a_cur_len(void) { return s_cpha_len_a; }
uint8_t  corexy_get_b_cur_len(void) { return s_cpha_len_b; }
uint32_t corexy_get_a_cur_count(void) { return s_cpha_cnt_a; }
uint32_t corexy_get_b_cur_count(void) { return s_cpha_cnt_b; }

uint32_t corexy_get_a_silent_ms(void) { return HAL_GetTick() - s_rx_a_tick; }
uint32_t corexy_get_b_silent_ms(void) { return HAL_GetTick() - s_rx_b_tick; }

/* 双轴最近CAN回包均须小于1s；位置新鲜度由独立时间戳判断。 */
uint8_t corexy_feedback_ok(void)
{
  return ((corexy_get_a_silent_ms() < FEEDBACK_TIMEOUT_MS) &&
          (corexy_get_b_silent_ms() < FEEDBACK_TIMEOUT_MS)) ? 1u : 0u;
}

uint8_t corexy_get_drv_en(void)     { return s_drv_en; }
/* 软件门禁：0禁止运动并双轴Stop，1只放行，不发送硬件Enable。 */
void corexy_set_enabled(uint8_t enable)
{
  s_enabled = (enable != 0u) ? 1u : 0u;
  if (s_enabled == 0u) corexy_stop();
}

uint8_t corexy_is_enabled(void)
{
  return s_enabled;
}

