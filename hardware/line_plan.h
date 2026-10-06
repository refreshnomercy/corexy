/* 两电机共用一个梯形时间进度；输出绝对脉冲目标与反馈修正后的整数RPM。 */
#ifndef LINE_PLAN_H
#define LINE_PLAN_H
#include "main.h"
#define LINE_PLAN_ACC_MMPS2 100.0f
#define LINE_PLAN_LOOKAHEAD_MS 80u
#define LINE_PLAN_UPDATE_MS 20u
#define LINE_PLAN_PATH_TOL_MM 1.0f
typedef struct {
  float start[2], delta[2], length, feed, ramp, coast, total;
  int32_t target[2];
} LinePlan_t;
void LinePlan_Init(LinePlan_t *plan, const int32_t start_enc[2],
                   const int32_t target_pulse[2], float feed);
void LinePlan_Sample(const LinePlan_t *plan, uint32_t elapsed_ms,
                     const int32_t actual_enc[2], int32_t target[2], uint16_t rpm[2]);
float LinePlan_PathError(const LinePlan_t *plan, const int32_t actual_enc[2]);
uint32_t LinePlan_DurationMs(const LinePlan_t *plan);
#endif
