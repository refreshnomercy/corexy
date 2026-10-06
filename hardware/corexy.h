/* CoreXY公共接口：长度mm、进给mm/s；XY位置为命令模型坐标。 */
#ifndef __COREXY_H
#define __COREXY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "Emm_V5.h"

#define EMM_ADDR_A          1u
#define EMM_ADDR_B          2u
#define EMM_ADDR_BROADCAST  0u

#define COREXY_SWAP_XY    0u /* 本机实测轴系，不用交换XY修正方向 */
#define COREXY_X_SIGN    (-1) /* +X：A/B方向均为1 */
#define COREXY_Y_SIGN    (-1) /* +Y：A=1、B=0 */
#define COREXY_BOUND_CURRENT_MA 200u /* 相电流mA，严格大于阈值才累计 */

#define PULLEY_CIRC_MM    40.0f /* GT2 20T带轮 */
#define PULSE_PER_MM      1280.0f /* 200整步×256细分 / 带轮周长 */
#define COREXY_ACC        50u /* 驱动器加速度参数档位 */
#define COREXY_POLL_MS    100u /* 空闲请求周期取此值的一半 */

#define COREXY_X_MAX_MM   170.0f /* 标称参考，不用于限制标定行程 */
#define COREXY_Y_MAX_MM   150.0f

#define CAL_SAFE_MARGIN_MM 3.0f /* 标定后远端墙面的额外余量 */

#define ENC_COUNTS_PER_MM  (65536.0f / PULLEY_CIRC_MM) /* 编码器计数，与命令脉冲分开 */

typedef enum
{
  COREXY_CAL_IDLE    = 0,
  COREXY_CAL_RUNNING = 1,
  COREXY_CAL_DONE    = 2,
  COREXY_CAL_FAILED  = 3,
} CoreXY_CalState_t;

typedef enum
{
  COREXY_MOVE_IDLE = 0,
  COREXY_MOVE_PREPARING,
  COREXY_MOVE_MOVING,
  COREXY_MOVE_DONE,
  COREXY_MOVE_STOPPED,
  COREXY_MOVE_FAULT
} CoreXY_MoveState_t;

#define COREXY_POSITION_TOL_MM 0.5f /* 到位验收容差，不是整机精度承诺 */
#define COREXY_SETTLE_MS       120u

uint8_t corexy_request_axis_move(float x, float y, float feed);
uint8_t corexy_request_line_move(float x, float y, float feed);
uint8_t corexy_request_motor_move(uint8_t motor, int32_t pulse, uint16_t rpm);
CoreXY_MoveState_t corexy_get_move_state(void);
void corexy_get_actual_pos(float *x, float *y);
uint8_t corexy_get_actual_pos_fresh(float *x, float *y, uint32_t max_age_ms);
void corexy_fault_stop(uint8_t reason); /* 锁存故障，停止卸力并撤销标定 */

void    corexy_rx_parse(uint8_t addr, uint8_t *data, uint8_t len);

void    corexy_init(void); /* 上电发闭环/使能/清零命令，未等Enable ACK */
void    corexy_rearm_drivers(void); /* 重配并保持断使能；保留模型、撤销标定 */
void    corexy_set_zero(void); /* 清硬件计数；第一包位置仍保留真实位移 */
void    corexy_move_to(float x, float y, float feed); /* 绝对XY(mm)，feed(mm/s) */
void    corexy_move_rel(float dx, float dy, float feed);
void    corexy_stop(void); /* 双轴Stop保持力矩，不等于断使能 */
void    corexy_get_pos(float *x, float *y); /* 命令模型坐标(mm)，不表示实际到位 */
void    corexy_poll_status(void);

uint8_t corexy_watch_stalled(void); /* 当前运动故障锁存，保留既有名称 */
void    corexy_watch_clear(void);
uint8_t corexy_move_busy(void); /* 准备或实际运动/停稳验收期间为1 */
uint32_t corexy_get_move_ms(void); /* 普通直线为含加减速的规划时长，自检为巡航标称时间 */

CoreXY_CalState_t corexy_home_and_calibrate(void); /* 阻塞自检，成功建立原点及软限位 */
CoreXY_CalState_t corexy_get_cal_state(void);
void    corexy_get_area(float *x_max, float *y_max);
uint8_t corexy_is_calibrated(void);

int32_t corexy_get_a_pulse(void);
int32_t corexy_get_b_pulse(void);
int32_t corexy_get_a_read(void);
int32_t corexy_get_b_read(void);
uint8_t corexy_get_a_flag(void);
uint8_t corexy_get_b_flag(void);

uint16_t corexy_get_a_current(void);
uint16_t corexy_get_b_current(void);
uint32_t corexy_get_a_cur_count(void);
uint32_t corexy_get_b_cur_count(void);
uint8_t  corexy_get_a_cur_len(void);
uint8_t  corexy_get_b_cur_len(void);
uint32_t corexy_get_a_silent_ms(void);
uint32_t corexy_get_b_silent_ms(void);
uint8_t corexy_get_drv_en(void); /* 软件下发记录，非硬件使能回读 */
uint8_t corexy_feedback_ok(void);
uint8_t corexy_take_boundary_hit(void); /* 读后清零的边界触发事件，非回退验收结果 */
uint32_t corexy_get_bound_ms(void);
uint16_t corexy_get_bound_ma(void);
uint8_t  corexy_get_bound_reason(void); /* 1电流/2低速；3位置/4轨迹回退/5目标ACK/6驱动准备故障 */
uint32_t corexy_get_a_pos_age_ms(void);
uint32_t corexy_get_b_pos_age_ms(void);
void     corexy_clear_area(void);

void    corexy_set_area(float x_max, float y_max);
uint8_t corexy_probe_to_boundary(int8_t dir_x, int8_t dir_y,
                                 float max_mm, float feed);

void    corexy_set_enabled(uint8_t enable); /* 软件门禁；启用不发硬件Enable */
uint8_t corexy_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif
