/* 屏幕触摸坐标→标定后的滑块目标→异步定位任务。 */
#ifndef __HMI_H
#define __HMI_H

#include "main.h"

/* 必须与屏幕m0的x/y/w/h一致；像素范围含两个端点。 */
#define HMI_TOUCH_X       20u
#define HMI_TOUCH_Y       20u
#define HMI_TOUCH_WIDTH   470u
#define HMI_TOUCH_HEIGHT  420u
#define HMI_MOVE_FEED     50.0f /* mm/s，普通点击定位速度 */
#define HMI_POSITION_PERIOD_MS 100u
#define HMI_POSITION_MAX_AGE_MS 300u

void HMI_Init(void);
void HMI_Poll(void);
void HMI_DiscardInput(void);
void HMI_PublishPosition(void);
uint8_t HMI_MapTouch(uint16_t px, uint16_t py, float x_max, float y_max,
                     float *x_mm, float *y_mm);
uint8_t HMI_MapActualPosition(float x_mm, float y_mm, float x_max, float y_max,
                              uint16_t *px, uint16_t *py);

#endif
