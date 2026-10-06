#include "line_plan.h"
#include "corexy.h"
#include <math.h>

static int32_t line_round(float value)
{ return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f); }

/* 从静止的实际编码器起点到量化后的目标规划；短线自动变为三角速度曲线。 */
void LinePlan_Init(LinePlan_t *p, const int32_t start_enc[2],
                   const int32_t target_pulse[2], float feed)
{
  uint8_t i;
  float motor_mm[2];
  for(i=0u;i<2u;++i) {
    p->start[i]=(float)start_enc[i]*PULSE_PER_MM/ENC_COUNTS_PER_MM;
    p->target[i]=target_pulse[i];
    p->delta[i]=(float)p->target[i]-p->start[i];
    motor_mm[i]=p->delta[i]/PULSE_PER_MM;
  }
  p->length=sqrtf((motor_mm[0]*motor_mm[0]+motor_mm[1]*motor_mm[1])*0.5f);
  p->feed=feed;
  p->ramp=feed/LINE_PLAN_ACC_MMPS2;
  if(p->ramp*p->ramp*LINE_PLAN_ACC_MMPS2>p->length)
    p->ramp=sqrtf(p->length/LINE_PLAN_ACC_MMPS2);
  p->coast=p->ramp>0.0f ?
    (p->length-p->ramp*p->ramp*LINE_PLAN_ACC_MMPS2)/(p->ramp*LINE_PLAN_ACC_MMPS2) : 0.0f;
  if(p->coast<0.0f)p->coast=0.0f;
  p->total=2.0f*p->ramp+p->coast;
}

static float line_distance(const LinePlan_t *p,float t,float *velocity)
{
  float peak=LINE_PLAN_ACC_MMPS2*p->ramp, rest;
  if(t<=0.0f){*velocity=0.0f;return 0.0f;}
  if(t>=p->total){*velocity=0.0f;return p->length;}
  if(t<p->ramp){*velocity=LINE_PLAN_ACC_MMPS2*t;return 0.5f*LINE_PLAN_ACC_MMPS2*t*t;}
  if(t<p->ramp+p->coast){*velocity=peak;return 0.5f*peak*p->ramp+peak*(t-p->ramp);}
  rest=p->total-t;*velocity=LINE_PLAN_ACC_MMPS2*rest;
  return p->length-0.5f*LINE_PLAN_ACC_MMPS2*rest*rest;
}

/* 前视位置仍在同一条连线上且不超过终点；不用无限速度模式。
 * 反馈修正抵消整数RPM舍入的累计误差；80ms前视避免每包位置命令都停稳再走。 */
void LinePlan_Sample(const LinePlan_t *p,uint32_t elapsed,const int32_t actual_enc[2],
                     int32_t target[2],uint16_t rpm[2])
{
  float t=(float)elapsed/1000.0f, v, unused, current, ahead, rate, correction, cap, motor;
  uint8_t i;
  current=line_distance(p,t,&v);
  ahead=line_distance(p,t+(float)LINE_PLAN_LOOKAHEAD_MS/1000.0f,&unused);
  for(i=0u;i<2u;++i) {
    motor=(float)actual_enc[i]*PULSE_PER_MM/ENC_COUNTS_PER_MM;
    if(p->length<0.5f/PULSE_PER_MM){target[i]=p->target[i];rpm[i]=1u;continue;}
    target[i]=line_round(p->start[i]+p->delta[i]*(ahead/p->length));
    correction=(p->start[i]+p->delta[i]*(current/p->length)-motor)/0.1f;
    rate=fabsf((p->delta[i]*(v/p->length)+correction)/PULSE_PER_MM)*60.0f/PULLEY_CIRC_MM;
    cap=fabsf(p->delta[i]/PULSE_PER_MM)*p->feed/p->length*60.0f/PULLEY_CIRC_MM+2.0f;
    if(rate>cap)rate=cap;
    if(rate<1.0f)rate=1.0f;
    rpm[i]=(uint16_t)(rate+0.5f);
  }
}

/* 电机空间垂距换算成滑块空间垂距；双电机位移不再要求相等。 */
float LinePlan_PathError(const LinePlan_t *p,const int32_t actual_enc[2])
{
  float a=(float)actual_enc[0]*PULSE_PER_MM/ENC_COUNTS_PER_MM-p->start[0];
  float b=(float)actual_enc[1]*PULSE_PER_MM/ENC_COUNTS_PER_MM-p->start[1];
  if(p->length<0.5f/PULSE_PER_MM)return 0.0f;
  return fabsf(a*p->delta[1]-b*p->delta[0])/(2.0f*p->length*PULSE_PER_MM*PULSE_PER_MM);
}
uint32_t LinePlan_DurationMs(const LinePlan_t *p) { return (uint32_t)(p->total*1000.0f+1.0f); }
