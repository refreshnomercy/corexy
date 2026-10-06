/* TJC屏：USART2单字节接收中断→环形缓冲→主循环解析。 */
#include "main.h"
#include "tjc_usart_hmi.h"
#include "bt.h"

typedef struct
{
    volatile uint16_t Head;
    volatile uint16_t Tail;
    volatile uint16_t Length;
    uint8_t  Ring_data[RINGBUFFER_LEN];
} RingBuffer_t;

static RingBuffer_t ringBuffer;
uint8_t RxBuffer[1];

#define TJC_TX_CAPACITY 192u
static uint8_t s_tx_data[TJC_TX_CAPACITY];
static volatile uint8_t s_tx_busy;
static uint16_t s_tx_length;

/* 前台整包构建，HAL发送期间保持缓冲不变，不等待TC或中断。 */
static uint8_t tx_begin(void)
{
    if (s_tx_busy) return 0u;
    s_tx_length = 0u;
    return 1u;
}

static uint8_t tx_byte(uint8_t value)
{
    if (s_tx_length >= TJC_TX_CAPACITY) return 0u;
    s_tx_data[s_tx_length++] = value;
    return 1u;
}

static uint8_t tx_text(const char *text)
{
    if (text == 0) return 0u;
    while (*text) if (!tx_byte((uint8_t)*text++)) return 0u;
    return 1u;
}

static uint8_t tx_integer(int value)
{
    char digits[10];
    uint8_t n = 0u;
    uint32_t magnitude;
    if (value < 0)
    {
        if (!tx_byte('-')) return 0u;
        magnitude = (uint32_t)(-(value + 1)) + 1u;
    }
    else magnitude = (uint32_t)value;
    do { digits[n++] = (char)('0' + magnitude % 10u); magnitude /= 10u; }
    while (magnitude);
    while (n) if (!tx_byte((uint8_t)digits[--n])) return 0u;
    return 1u;
}

static uint8_t tx_end_command(void)
{
    return tx_byte(0xffu) && tx_byte(0xffu) && tx_byte(0xffu);
}

static uint8_t tx_start(void)
{
    s_tx_busy = 1u;
    if (HAL_UART_Transmit_IT(&TJC_UART, s_tx_data, s_tx_length) != HAL_OK)
    { s_tx_busy = 0u; return 0u; }
    return 1u;
}

uint8_t tjc_send_string(char *str)
{
    if (!tx_begin() || !tx_text(str) || !tx_end_command()) return 0u;
    return tx_start();
}

uint8_t tjc_send_txt(char *objname, char *attribute, char *txt)
{
    if (!tx_begin() || !tx_text(objname) || !tx_byte('.') ||
        !tx_text(attribute) || !tx_text("=\"") || !tx_text(txt) ||
        !tx_byte('"') || !tx_end_command()) return 0u;
    return tx_start();
}

uint8_t tjc_send_val(char *objname, char *attribute, int val)
{
    if (!tx_begin() || !tx_text(objname) || !tx_byte('.') ||
        !tx_text(attribute) || !tx_byte('=') || !tx_integer(val) ||
        !tx_end_command()) return 0u;
    return tx_start();
}

uint8_t tjc_send_nstring(char *str, unsigned char str_length)
{
    unsigned int i;
    if (!tx_begin() || str == 0) return 0u;
    for (i = 0u; i < str_length; ++i) if (!tx_byte((uint8_t)str[i])) return 0u;
    if (!tx_end_command()) return 0u;
    return tx_start();
}

static uint8_t tx_position_value(const char *object, int value)
{
    return tx_text("page0.") && tx_text(object) && tx_text(".val=") &&
           tx_integer(value) && tx_end_command();
}

/* 最后gnew=1提交快照；四个绿点变量须设为全局。
 * 不缓存旧坐标。跨页面的在途包只改page0变量，不直接绘图。 */
uint8_t tjc_send_position(uint16_t px, uint16_t py, uint8_t valid)
{
    if (!tx_begin() || !tx_position_value("gnew", 0) ||
        !tx_position_value("gpx", px) || !tx_position_value("gpy", py) ||
        !tx_position_value("gok", valid ? 1 : 0) ||
        !tx_position_value("gnew", 1)) return 0u;
    return tx_start();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == TJC_UART_INS) s_tx_busy = 0u;
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == TJC_UART_INS) s_tx_busy = 0u;
}
/* 清缓冲，启用USART2单字节接收中断，优先级低于CAN。 */
void tjc_init(void)
{
    initRingBuffer();
    s_tx_busy = 0u;

    HAL_NVIC_SetPriority(TJC_UART_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TJC_UART_IRQn);

    HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);
}

/* 共用HAL串口回调：USART2入屏幕缓冲，其他串口交蓝牙入口检查地址。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == TJC_UART_INS)
    {
        write1ByteToRingBuffer(RxBuffer[0]);
        HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);
    }
    else BT_UartRxCallback(huart);
}

/* 串口错误后重挂接收；屏幕丢弃半帧，蓝牙丢字节由主循环停止任务。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == TJC_UART_INS)
    {
        initRingBuffer();
        (void)HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);
    }
    else BT_UartErrorCallback(huart);
}

/* 同一入口消费OK/坐标帧，半帧保留到下次；坐标为小端uint16。
 * 固定长度检查帧尾，避免坐标字节0xFF被误当结束符。
 * 每次窥读/消费使用短临界区，防止UART错误回调中途复位缓冲。 */
uint8_t tjc_take_event(TJC_Event_t *event)
{
    uint32_t lock;
    uint16_t length;

    if (event == 0) return 0u;
    event->type = TJC_EVENT_NONE;
    event->x = 0u;
    event->y = 0u;

    for (;;)
    {
        lock = __get_PRIMASK();
        __disable_irq();
        if (getRingBufferLength() == 0u)
        {
            __set_PRIMASK(lock);
            return 0u;
        }
        if (u(0) != 0x55u)
        {
            deleteRingBuffer(1u);
            __set_PRIMASK(lock);
            continue;
        }
        if (getRingBufferLength() < 2u)
        {
            __set_PRIMASK(lock);
            return 0u;
        }
        if (u(1) == 0x54u) length = 9u;
        else if (u(1) == 0x6fu || u(1) == 0x50u) length = 6u;
        else
        {
            deleteRingBuffer(1u);
            __set_PRIMASK(lock);
            continue;
        }
        if (getRingBufferLength() < length)
        {
            __set_PRIMASK(lock);
            return 0u;
        }
        if (u(length - 3u) != 0xffu || u(length - 2u) != 0xffu ||
            u(length - 1u) != 0xffu ||
            (u(1) == 0x6fu && u(2) != 0x6bu) ||
            (u(1) == 0x50u && u(2) != 0x01u))
        {
            deleteRingBuffer(1u);
            __set_PRIMASK(lock);
            continue;
        }
        if (length == 9u)
        {
            event->type = TJC_EVENT_TOUCH;
            event->x = (uint16_t)((uint16_t)u(2) | ((uint16_t)u(3) << 8));
            event->y = (uint16_t)((uint16_t)u(4) | ((uint16_t)u(5) << 8));
        }
        else if (u(1) == 0x50u) event->type = TJC_EVENT_POSITION_REQUEST;
        else event->type = TJC_EVENT_OK;
        deleteRingBuffer(length);
        __set_PRIMASK(lock);
        return 1u;
    }
}

/* 复位共享缓冲时保存/恢复原中断屏蔽状态，不能无条件开中断。 */
void initRingBuffer(void)
{
    uint32_t lock = __get_PRIMASK();
    __disable_irq();
    ringBuffer.Head = 0;
    ringBuffer.Tail = 0;
    ringBuffer.Length = 0;
    __set_PRIMASK(lock);
}

void write1ByteToRingBuffer(uint8_t data)
{
    if (ringBuffer.Length >= RINGBUFFER_LEN)
    {
        return;
    }
    ringBuffer.Ring_data[ringBuffer.Tail] = data;
    ringBuffer.Tail = (ringBuffer.Tail + 1) % RINGBUFFER_LEN;
    ringBuffer.Length++;
}

/* 主循环消费字节，修改共享索引时使用短临界区。 */
void deleteRingBuffer(uint16_t size)
{
    uint32_t lock = __get_PRIMASK();
    __disable_irq();
    if (size >= ringBuffer.Length)
    {
        ringBuffer.Head = ringBuffer.Tail;
        ringBuffer.Length = 0;
    }
    else
    {
        ringBuffer.Head = (ringBuffer.Head + size) % RINGBUFFER_LEN;
        ringBuffer.Length -= size;
    }
    __set_PRIMASK(lock);
}

/* 按Head相对偏移窥读、不消费，帧解析宏u(x)间接调用。 */
uint8_t read1ByteFromRingBuffer(uint16_t position)
{
    uint16_t realPosition = (ringBuffer.Head + position) % RINGBUFFER_LEN;
    return ringBuffer.Ring_data[realPosition];
}

uint16_t getRingBufferLength(void)
{
    return ringBuffer.Length;
}
