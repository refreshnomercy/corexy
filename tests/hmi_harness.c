#include "main.h"
#define API __declspec(dllexport)
UART_HandleTypeDef huart2 = { USART2 };
static uint32_t rx_rearms;
static int enabled_irq;
static uint32_t rx_port;
static uint32_t host_tick, tx_attempts, tx_starts;
static int tx_result;
static uint8_t *tx_pointer;
static uint16_t tx_length;
uint32_t HAL_GetTick(void) { return host_tick; }
void HAL_NVIC_SetPriority(int irq, int preempt, int sub)
{ (void)irq; (void)preempt; (void)sub; }
void HAL_NVIC_EnableIRQ(int irq) { enabled_irq = irq; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{ rx_port = (uint32_t)(uintptr_t)uart->Instance; (void)data; (void)len; rx_rearms++; return 0; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    (void)uart; ++tx_attempts;
    if (tx_result != HAL_OK) return tx_result;
    tx_pointer = data; tx_length = len; ++tx_starts;
    return HAL_OK;
}
#include "../hardware/tjc_usart_hmi.c"

/* 协议/映射层测试仅替换任务提交；实际定位另在boundary_harness中测试。 */
#define __COREXY_H
#define COREXY_POSITION_TOL_MM 0.5f
int _fltused = 0;
static uint8_t calibrated;
static uint8_t position_seen, position_fault;
static uint32_t position_age_a, position_age_b;
static float actual_x, actual_y;
static float area_x, area_y;
static char output[8192];
static uint16_t output_length;
static uint32_t targets;
static float target_x, target_y;
uint8_t Motion_SubmitXY(float x, float y, float feed, const char *source)
{ (void)feed; (void)source; target_x=x; target_y=y; ++targets; return 1; }
void BT_UartRxCallback(UART_HandleTypeDef *uart) { (void)uart; }
void BT_UartErrorCallback(UART_HandleTypeDef *uart) { (void)uart; }
uint8_t corexy_is_calibrated(void) { return calibrated; }
void corexy_get_area(float *x, float *y) { *x = area_x; *y = area_y; }
uint8_t corexy_watch_stalled(void) { return position_fault; }
uint8_t corexy_get_actual_pos_fresh(float *x, float *y, uint32_t max_age)
{
    if (!position_seen || position_age_a > max_age || position_age_b > max_age) return 0u;
    *x = actual_x; *y = actual_y; return 1u;
}
static void output_char(char c)
{
    if (output_length < sizeof(output) - 1u)
    {
        output[output_length++] = c;
        output[output_length] = 0;
    }
}
void BT_Puts(const char *s) { while (*s) output_char(*s++); }
void BT_PutInt(int32_t value)
{
    char digits[12];
    uint8_t n = 0u;
    uint32_t magnitude;
    if (value < 0) { output_char('-'); magnitude = (uint32_t)(-value); }
    else magnitude = (uint32_t)value;
    do { digits[n++] = (char)('0' + magnitude % 10u); magnitude /= 10u; }
    while (magnitude != 0u);
    while (n != 0u) output_char(digits[--n]);
}
void BT_NewLine(void) { BT_Puts("\r\n"); }
void BT_Tag(const char *s) { BT_Puts(s); BT_NewLine(); }
void BT_PutMm(float mm)
{
    int32_t value=(int32_t)(mm*100.0f+0.5f), fraction=value%100;
    BT_PutInt(value/100); BT_Puts(".");
    if(fraction<10) BT_Puts("0");
    BT_PutInt(fraction);
}
#include "../hardware/hmi.c"

API void host_init(void)
{
    rx_rearms = 0; host_irq_mask = 0;
    calibrated = 0; area_x = 0.0f; area_y = 0.0f;
    output_length = 0; output[0] = 0;
    targets=0; target_x=target_y=0;
    host_tick=tx_attempts=tx_starts=0; tx_result=0; tx_pointer=0; tx_length=0;
    position_seen=position_fault=0; position_age_a=position_age_b=0;
    actual_x=actual_y=0;
    tjc_init();
    HMI_Init(); output_length=0; output[0]=0;
}
API void host_receive(uint8_t byte, uint8_t port)
{
    UART_HandleTypeDef uart;
    uart.Instance = (void *)(uintptr_t)port;
    RxBuffer[0] = byte;
    HAL_UART_RxCpltCallback(&uart);
}
API void host_error(uint8_t port)
{
    UART_HandleTypeDef uart;
    uart.Instance = (void *)(uintptr_t)port;
    HAL_UART_ErrorCallback(&uart);
}
API uint8_t host_take_ok(void)
{
    TJC_Event_t event;
    return tjc_take_event(&event) != 0u && event.type == TJC_EVENT_OK;
}
API uint8_t host_take_event(uint16_t *x, uint16_t *y)
{
    TJC_Event_t event;
    if (tjc_take_event(&event) == 0u) return 0u;
    *x = event.x; *y = event.y;
    return (uint8_t)event.type;
}
API void host_set_area(float x, float y, uint8_t cal)
{ area_x = x; area_y = y; calibrated = cal; }
API void host_poll(void) { HMI_Poll(); }
API void host_hmi_init(void) { HMI_Init(); }
API const char *host_output(void) { return output; }
API void host_clear_output(void) { output_length = 0; output[0] = 0; }
API uint32_t host_targets(void) { return targets; }
API float host_target_x(void) { return target_x; }
API float host_target_y(void) { return target_y; }
API uint8_t host_map_touch(uint16_t px, uint16_t py, float xm, float ym,
                          float *x, float *y)
{ return HMI_MapTouch(px, py, xm, ym, x, y); }
API uint16_t host_length(void) { return getRingBufferLength(); }
API uint32_t host_rearms(void) { return rx_rearms; }
API int host_enabled_irq(void) { return enabled_irq; }
API uint32_t host_rx_port(void) { return rx_port; }
API void host_set_mask(uint32_t value) { host_irq_mask = value; }
API uint32_t host_get_mask(void) { return host_irq_mask; }
API void host_set_tick(uint32_t value) { host_tick=value; }
API void host_set_actual(float x, float y, uint32_t age_a, uint32_t age_b,
                         uint8_t seen, uint8_t fault)
{
    actual_x=x; actual_y=y; position_age_a=age_a; position_age_b=age_b;
    position_seen=seen; position_fault=fault;
}
API void host_publish(void) { HMI_PublishPosition(); }
API void host_discard(void) { HMI_DiscardInput(); }
API const uint8_t *host_tx_data(void) { return tx_pointer; }
API uint16_t host_tx_length(void) { return tx_length; }
API uint32_t host_tx_attempts(void) { return tx_attempts; }
API uint32_t host_tx_starts(void) { return tx_starts; }
API void host_tx_result(int result) { tx_result=result; }
API void host_tx_complete(uint8_t port)
{
    UART_HandleTypeDef uart; uart.Instance=(void *)(uintptr_t)port;
    HAL_UART_TxCpltCallback(&uart);
}
API void host_tx_abort(void) { HAL_UART_AbortTransmitCpltCallback(&huart2); }
API uint8_t host_send_string(char *text) { return tjc_send_string(text); }
API uint8_t host_send_value(int value) { return tjc_send_val("n0", "val", value); }
API uint8_t host_send_text(char *text) { return tjc_send_txt("t0", "txt", text); }
API uint8_t host_send_position(uint16_t x, uint16_t y, uint8_t valid)
{ return tjc_send_position(x,y,valid); }
API uint8_t host_map_actual(float x, float y, float xm, float ym,
                           uint16_t *px, uint16_t *py)
{ return HMI_MapActualPosition(x,y,xm,ym,px,py); }
