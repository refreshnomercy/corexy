/* 普通定位任务：两点连线；运动中仅保留最后一个待执行目标。 */
#ifndef __MOTION_H
#define __MOTION_H
#include "main.h"

void Motion_Init(void);
void Motion_Poll(void);
uint8_t Motion_IsBusy(void);
uint8_t Motion_SubmitXY(float x, float y, float feed, const char *source);
uint8_t Motion_SubmitRelative(float dx, float dy, float feed, const char *source);
uint8_t Motion_JogMotor(uint8_t motor, int32_t pulse, uint16_t rpm);
void Motion_Stop(void); /* 清任务和待执行点击，中途停止会撤销标定 */
#endif
