#include "motion.h"
#include "corexy.h"
#include "bt.h"

typedef struct
{
  float x, y, feed;
  const char *source;
  uint32_t id;
} Motion_Target_t;

static Motion_Target_t s_target, s_pending;
static uint8_t s_active, s_has_pending, s_phase, s_motor_test;
static uint32_t s_sequence;
static int32_t s_a0, s_b0, s_ea0, s_eb0;

static void Motion_ReportTarget(const char *event, const Motion_Target_t *target)
{
  BT_Puts("POS "); BT_Puts(event);
  BT_Puts(" ID="); BT_PutInt((int32_t)target->id);
  BT_Puts(" SRC="); BT_Puts(target->source);
  BT_Puts(" X="); BT_PutMm(target->x);
  BT_Puts("mm Y="); BT_PutMm(target->y); BT_Puts("mm"); BT_NewLine();
}

void Motion_Init(void)
{
  s_active = s_has_pending = s_phase = s_motor_test = 0u;
  s_sequence = 0u;
}

uint8_t Motion_IsBusy(void) { return s_active || corexy_move_busy(); }

/* 已开始的任务执行到终点；新点击只更新一个待执行槽，不覆盖驱动目标。 */
uint8_t Motion_SubmitXY(float x, float y, float feed, const char *source)
{
  float xm, ym;
  Motion_Target_t target;
  if (!corexy_is_calibrated() || !corexy_is_enabled() || corexy_watch_stalled())
  { BT_Tag("POS REJECT: calibration required or motion disabled"); return 0u; }
  corexy_get_area(&xm, &ym);
  if (!(x >= 0.0f && x <= xm + 0.001f && y >= 0.0f && y <= ym + 0.001f &&
        feed > 0.0f && feed <= 50.0f) || source == 0)
  { BT_Tag("POS REJECT: invalid target/feed"); return 0u; }
  if (x > xm) x = xm;
  if (y > ym) y = ym;
  target.x = x; target.y = y; target.feed = feed;
  target.source = source; target.id = ++s_sequence;
  if (s_active)
  {
    s_pending = target; s_has_pending = 1u;
    Motion_ReportTarget("QUEUED LAST", &target);
  }
  else
  {
    if (corexy_move_busy())
    { BT_Tag("POS REJECT: core busy"); return 0u; }
    s_target = target; s_active = 1u; s_phase = s_motor_test = 0u;
    Motion_ReportTarget("ACCEPT", &target);
  }
  return 1u;
}

/* 连续相对命令以最近待执行终点为起点；软限位钳制与原点动行为一致。 */
uint8_t Motion_SubmitRelative(float dx, float dy, float feed, const char *source)
{
  float x, y, xm, ym;
  if (s_has_pending) { x = s_pending.x; y = s_pending.y; }
  else if (s_active && !s_motor_test) { x = s_target.x; y = s_target.y; }
  else corexy_get_pos(&x, &y);
  corexy_get_area(&xm, &ym);
  x += dx; y += dy;
  if (x < 0.0f) x = 0.0f;
  if (y < 0.0f) y = 0.0f;
  if (x > xm) x = xm;
  if (y > ym) y = ym;
  return Motion_SubmitXY(x, y, feed, source);
}

uint8_t Motion_JogMotor(uint8_t motor, int32_t pulse, uint16_t rpm)
{
  if (Motion_IsBusy() || !corexy_request_motor_move(motor, pulse, rpm))
  { BT_Tag("POS REJECT: motor diagnostic busy/fault/invalid"); return 0u; }
  s_active = s_motor_test = 1u; s_phase = 2u;
  s_target.source = motor == EMM_ADDR_A ? "BT/A" : "BT/B";
  s_target.id = ++s_sequence;
  s_a0 = corexy_get_a_pulse(); s_b0 = corexy_get_b_pulse();
  s_ea0 = corexy_get_a_read(); s_eb0 = corexy_get_b_read();
  BT_Tag("POS MOTOR TEST: calibration revoked");
  return 1u;
}

void Motion_Stop(void)
{
  s_active = s_has_pending = s_phase = s_motor_test = 0u;
  corexy_stop();
}

static uint8_t Motion_StartLine(void)
{
  if (!corexy_request_line_move(s_target.x, s_target.y, s_target.feed)) return 0u;
  s_phase = 2u;
  BT_Puts("POS START ID="); BT_PutInt((int32_t)s_target.id);
  BT_Puts(" LEG=LINE"); BT_NewLine();
  return 1u;
}

/* 只在整条连线实际到位且停稳后取待执行目标；任何失败都清空待执行槽。 */
void Motion_Poll(void)
{
  CoreXY_MoveState_t state;
  float x, y;
  if (!s_active) return;
  state = corexy_get_move_state();
  if (state == COREXY_MOVE_FAULT || state == COREXY_MOVE_STOPPED ||
      corexy_watch_stalled() || (!s_motor_test && !corexy_is_calibrated()))
  {
    s_active = s_has_pending = s_phase = s_motor_test = 0u;
    if (corexy_move_busy()) corexy_fault_stop(4u);
    BT_Tag("POS CANCEL: pending cleared; run KEY1 calibration");
    return;
  }
  if (s_phase == 0u)
  {
    s_a0 = corexy_get_a_pulse(); s_b0 = corexy_get_b_pulse();
    s_ea0 = corexy_get_a_read(); s_eb0 = corexy_get_b_read();
    if (!Motion_StartLine()) goto rejected;
    return;
  }
  if (state != COREXY_MOVE_DONE) return;
  corexy_get_actual_pos(&x, &y);
  BT_Puts("POS DONE ID="); BT_PutInt((int32_t)s_target.id);
  BT_Puts(" ACTUAL X="); BT_PutMm(x);
  BT_Puts("mm Y="); BT_PutMm(y); BT_Puts("mm"); BT_NewLine();
  BT_ReportMove(s_target.source, s_a0, s_b0, s_ea0, s_eb0);
  s_active = s_motor_test = s_phase = 0u;
  if (s_has_pending)
  {
    s_target = s_pending; s_has_pending = 0u; s_active = 1u;
    Motion_ReportTarget("ACCEPT", &s_target);
  }
  return;

rejected:
  s_active = s_has_pending = s_phase = s_motor_test = 0u;
  BT_Tag("POS CANCEL: line rejected, pending cleared");
}
