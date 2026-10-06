#include "hmi.h"
#include "tjc_usart_hmi.h"
#include "corexy.h"
#include "bt.h"
#include "motion.h"
#include <float.h>

#if HMI_TOUCH_WIDTH < 2u || HMI_TOUCH_HEIGHT < 2u
#error Touch region must contain at least two pixels per axis
#endif

static uint8_t s_position_requested, s_position_view_seen;
static uint32_t s_position_tick;

static uint8_t HMI_Inside(uint16_t px, uint16_t py)
{
    return px >= HMI_TOUCH_X && px < HMI_TOUCH_X + HMI_TOUCH_WIDTH &&
           py >= HMI_TOUCH_Y && py < HMI_TOUCH_Y + HMI_TOUCH_HEIGHT;
}

/* 屏幕左上角为零，滑块左下角为零：扣除热区偏移并翻转Y。
 * 上限使用本次自检的软限位，不使用屏幕像素或历史行程代替。 */
uint8_t HMI_MapTouch(uint16_t px, uint16_t py, float x_max, float y_max,
                     float *x_mm, float *y_mm)
{
    if (x_mm == 0 || y_mm == 0 || HMI_Inside(px, py) == 0u ||
        !(x_max > 0.0f) || !(y_max > 0.0f)) return 0u;

    *x_mm = (float)(px - HMI_TOUCH_X) * x_max / (float)(HMI_TOUCH_WIDTH - 1u);
    *y_mm = (float)(HMI_TOUCH_Y + HMI_TOUCH_HEIGHT - 1u - py) * y_max /
            (float)(HMI_TOUCH_HEIGHT - 1u);
    return 1u;
}

/* 将编码器实际mm反向映射到同一热区；半毫米边缘误差只钳制显示。
 * 明显超出标定范围、NaN/无穷大不显示，不改变定位任务或目标。 */
uint8_t HMI_MapActualPosition(float x_mm, float y_mm, float x_max, float y_max,
                              uint16_t *px, uint16_t *py)
{
    if (px == 0 || py == 0 ||
        !(x_max > 0.0f && x_max < FLT_MAX && y_max > 0.0f && y_max < FLT_MAX) ||
        !(x_mm >= -COREXY_POSITION_TOL_MM && x_mm <= x_max + COREXY_POSITION_TOL_MM &&
          y_mm >= -COREXY_POSITION_TOL_MM && y_mm <= y_max + COREXY_POSITION_TOL_MM))
        return 0u;
    if (x_mm < 0.0f) x_mm = 0.0f;
    if (y_mm < 0.0f) y_mm = 0.0f;
    if (x_mm > x_max) x_mm = x_max;
    if (y_mm > y_max) y_mm = y_max;
    *px = (uint16_t)(HMI_TOUCH_X +
                    (uint16_t)(x_mm / x_max * (HMI_TOUCH_WIDTH - 1u) + 0.5f));
    *py = (uint16_t)(HMI_TOUCH_Y + HMI_TOUCH_HEIGHT - 1u -
                    (uint16_t)(y_mm / y_max * (HMI_TOUCH_HEIGHT - 1u) + 0.5f));
    return 1u;
}

/* 只响应新UI的取样请求。双轴新鲜反馈、有效标定才显示绿点。
 * UART忙保留一个请求，稍后重读最新位置；不排队旧位置，不等待发送。 */
void HMI_PublishPosition(void)
{
    float x, y, xm, ym;
    uint16_t px = 0u, py = 0u;
    uint8_t valid = 0u;
    uint32_t now = HAL_GetTick();
    if (!s_position_requested || now - s_position_tick < HMI_POSITION_PERIOD_MS) return;
    if (corexy_is_calibrated() && !corexy_watch_stalled() &&
        corexy_get_actual_pos_fresh(&x, &y, HMI_POSITION_MAX_AGE_MS))
    {
        corexy_get_area(&xm, &ym);
        valid = HMI_MapActualPosition(x, y, xm, ym, &px, &py);
    }
    if (tjc_send_position(px, py, valid))
    {
        s_position_requested = 0u;
        s_position_tick = now;
    }
}

/* 收到有效帧先报告原始像素；仅CAL_DONE且位于热区内才计算目标。
 * 有效目标交给普通定位任务，运行中再次点击由任务模块保留最后一次。 */
static void HMI_ReportTouch(uint16_t px, uint16_t py)
{
    float x_max, y_max, x_mm, y_mm;

    BT_Puts("HMI TOUCH PX="); BT_PutInt(px);
    BT_Puts(" PY="); BT_PutInt(py);
    if (HMI_Inside(px, py) == 0u)
        BT_Puts(" -> REJECT: outside touch region");
    else if (corexy_is_calibrated() == 0u)
        BT_Puts(" -> TARGET UNAVAILABLE: run KEY1 calibration");
    else
    {
        corexy_get_area(&x_max, &y_max);
        if (HMI_MapTouch(px, py, x_max, y_max, &x_mm, &y_mm) == 0u)
            BT_Puts(" -> REJECT: invalid calibrated range");
        else
        {
            BT_Puts(" -> TARGET X="); BT_PutMm(x_mm);
            BT_Puts("mm Y="); BT_PutMm(y_mm);
            BT_Puts("mm"); BT_NewLine();
            (void)Motion_SubmitXY(x_mm, y_mm, HMI_MOVE_FEED, "HMI");
            return;
        }
    }
    BT_NewLine();
}

void HMI_Init(void)
{
    s_position_requested = s_position_view_seen = 0u;
    s_position_tick = HAL_GetTick() - HMI_POSITION_PERIOD_MS;
    BT_Puts("HMI POSITION: direct line, "); BT_PutMm(HMI_MOVE_FEED);
    BT_Puts("mm/s; latest pending click wins"); BT_NewLine();
    BT_Puts("HMI REGION X="); BT_PutInt(HMI_TOUCH_X);
    BT_Puts(" Y="); BT_PutInt(HMI_TOUCH_Y);
    BT_Puts(" W="); BT_PutInt(HMI_TOUCH_WIDTH);
    BT_Puts(" H="); BT_PutInt(HMI_TOUCH_HEIGHT);
    BT_NewLine();
    BT_Tag("HMI ACTUAL MARKER: request 55 50 01, encoder feedback, 100ms");
}

/* 自检前后或停止时丢弃旧点击，防止动作结束后突然执行缓存目标。 */
void HMI_DiscardInput(void)
{
    initRingBuffer();
    s_position_requested = 0u;
    if (s_position_view_seen) (void)tjc_send_position(0u, 0u, 0u);
}

/* 在App主循环消费事件；UART中断仅收字节，不做换算或蓝牙发送。 */
void HMI_Poll(void)
{
    TJC_Event_t event;

    while (tjc_take_event(&event) != 0u)
    {
        if (event.type == TJC_EVENT_OK) BT_Tag("ok");
        else if (event.type == TJC_EVENT_TOUCH) HMI_ReportTouch(event.x, event.y);
        else if (event.type == TJC_EVENT_POSITION_REQUEST)
        {
            s_position_requested = 1u;
            s_position_view_seen = 1u;
        }
    }
}
