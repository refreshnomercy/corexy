/* USART1蓝牙控制台：ASCII命令行与状态/运动报告。 */
#include "bt.h"
#include "usart.h"
#include "corexy.h"
#include "can.h"
#include "Buzzer.h"
#include "motion.h"
#include "hmi.h"

static char    s_buf[144];
static uint8_t s_len = 0u;

static char    s_line[24];
static uint8_t s_lineLen = 0u;
static uint8_t s_line_drop;
#define BT_RX_CAPACITY 128u
static uint8_t s_rx_byte, s_rx[BT_RX_CAPACITY];
static volatile uint16_t s_rx_head, s_rx_tail, s_rx_len;
static volatile uint8_t s_rx_lost;

static void bt_ch(char c)
{
  if (s_len < (uint8_t)(sizeof(s_buf) - 2u))
  {
    s_buf[s_len++] = c;
  }
}

static void bt_flush(void)
{
  if (s_len > 0u)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)s_buf, (uint16_t)s_len, 100u);
  }
  s_len = 0u;
}

void BT_Puts(const char *s)
{
  while (*s != '\0') bt_ch(*s++);
}

void BT_PutInt(int32_t v)
{
  char     tmp[12];
  uint8_t  n = 0u;
  uint32_t u;

  if (v < 0) { bt_ch('-'); u = (uint32_t)(-v); }
  else       { u = (uint32_t)v; }

  do
  {
    tmp[n++] = (char)('0' + (u % 10u));
    u /= 10u;
  } while ((u != 0u) && (n < 11u));

  while (n > 0u) bt_ch(tmp[--n]);
}

void BT_Tag(const char *tag)
{
  s_len = 0u;
  BT_Puts(tag);
  bt_ch('\r');
  bt_ch('\n');
  bt_flush();
}

void BT_NewLine(void)
{
  bt_ch('\r');
  bt_ch('\n');
  bt_flush();
}

void BT_PutCm(float mm)
{
  int32_t v = (int32_t)(mm + 0.5f);

  if (v < 0)
  {
    bt_ch('-');
    v = -v;
  }

  BT_PutInt(v / 10);
  bt_ch('.');
  BT_PutInt(v % 10);
}

/* 统一位置报告的毫米格式，兼容实际编码器坐标的轻微负值。 */
void BT_PutMm(float mm)
{
  int32_t value, fraction;
  if (mm < 0.0f) { bt_ch('-'); mm = -mm; }
  value = (int32_t)(mm * 100.0f + 0.5f);
  fraction = value % 100;
  BT_PutInt(value / 100); bt_ch('.');
  if (fraction < 10) bt_ch('0');
  BT_PutInt(fraction);
}

/* 输出电流、标定范围、回包年龄与边界/故障原因BR。 */
void BT_Status(void)
{
  float xm, ym;

  corexy_get_area(&xm, &ym);

  s_len = 0u;
  BT_Puts("CA=");   BT_PutInt((int32_t)corexy_get_a_current());
  BT_Puts(" CB=");  BT_PutInt((int32_t)corexy_get_b_current());
  BT_Puts(" CAL="); BT_PutInt((int32_t)corexy_is_calibrated());
  BT_Puts(" X=");   BT_PutCm(xm);
  BT_Puts("cm Y="); BT_PutCm(ym);

  BT_Puts("cm SA="); BT_PutInt((int32_t)corexy_get_a_silent_ms());
  BT_Puts(" SB=");   BT_PutInt((int32_t)corexy_get_b_silent_ms());
  BT_Puts(" BR=");   BT_PutInt((int32_t)corexy_get_bound_reason());
  BT_Puts(" PA=");   BT_PutInt((int32_t)corexy_get_a_pos_age_ms());
  BT_Puts(" PB=");   BT_PutInt((int32_t)corexy_get_b_pos_age_ms());
  BT_Puts(" TH=");   BT_PutInt((int32_t)COREXY_BOUND_CURRENT_MA);
  bt_ch('\r');
  bt_ch('\n');
  bt_flush();
}

void BT_Dimensions(void)
{
  float xm, ym;

  corexy_get_area(&xm, &ym);

  s_len = 0u;
  BT_Puts("DIM ");

  if (corexy_is_calibrated() != 0u)
  {
    BT_Puts("X=");  BT_PutCm(xm);  BT_Puts("cm  Y=");  BT_PutCm(ym);  BT_Puts("cm");
  }
  else
  {
    BT_Puts("not calibrated");
  }

  bt_ch('\r');
  bt_ch('\n');
  bt_flush();
}

/* 报告命令/编码器增量、模型坐标和BR；命令接受不表示实际到位。 */
void BT_ReportMove(const char *what, int32_t a0, int32_t b0, int32_t ea0, int32_t eb0)
{
  float x, y;

  corexy_get_pos(&x, &y);

  s_len = 0u;
  BT_Puts("MOVE ");
  BT_Puts(what);
  BT_Puts(" cmdA="); BT_PutInt(corexy_get_a_pulse() - a0);
  BT_Puts(" cmdB="); BT_PutInt(corexy_get_b_pulse() - b0);
  BT_Puts(" encA="); BT_PutInt(corexy_get_a_read() - ea0);
  BT_Puts(" encB="); BT_PutInt(corexy_get_b_read() - eb0);
  BT_Puts(" -> x="); BT_PutInt((int32_t)(x * 100.0f));
  BT_Puts("/100 y="); BT_PutInt((int32_t)(y * 100.0f));
  BT_Puts("/100 BR="); BT_PutInt((int32_t)corexy_get_bound_reason());
  bt_ch('\r');
  bt_ch('\n');
  bt_flush();
}

/* 普通运动提交异步任务；状态/停止命令不恢复使能，也不解除故障。 */
static void bt_exec(void)
{
  char     cmd = s_line[0];
  int32_t  v   = 0;
  uint8_t  i   = 1u;
  uint8_t  neg = 0u;

  while (s_line[i] == ' ') i++;
  if (s_line[i] == '-')      { neg = 1u; i++; }
  else if (s_line[i] == '+') { i++; }

  while ((s_line[i] >= '0') && (s_line[i] <= '9'))
  {
    if (v > 300 || v * 10 + (int32_t)(s_line[i] - '0') > 300)
    { BT_Tag("CMD REJECT: distance exceeds 300mm"); return; }
    v = (v * 10) + (int32_t)(s_line[i] - '0');
    i++;
  }
  if (neg != 0u) v = -v;

  while (s_line[i] == ' ') ++i;
  if (s_line[i] != '\0')
  { BT_Tag("CMD REJECT: integer distance required"); return; }

  switch (cmd)
  {
    case 'X': case 'x':
      (void)Motion_SubmitRelative((float)v, 0.0f, 20.0f, "BT/X");
      break;

    case 'Y': case 'y':
      (void)Motion_SubmitRelative(0.0f, (float)v, 20.0f, "BT/Y");
      break;

    case 'A': case 'a':
      (void)Motion_JogMotor(EMM_ADDR_A, (int32_t)((float)v * PULSE_PER_MM), 100u);
      break;

    case 'B': case 'b':
      (void)Motion_JogMotor(EMM_ADDR_B, (int32_t)((float)v * PULSE_PER_MM), 100u);
      break;

    case 'S': case 's':
      Motion_Stop();
      HMI_DiscardInput();
      BT_DiscardInput();
      BT_Tag("STOP: pending cleared");
      break;

    case 'Z': case 'z':
      Motion_Stop();
      HMI_DiscardInput();
      BT_DiscardInput();
      if (corexy_watch_stalled()) { BT_Tag("ZERO REJECT: stop/fault not cleared"); break; }
      corexy_set_zero();
      BT_Tag("ZERO");
      break;

    case 'D': case 'd':
      BT_Dimensions();
      break;

    case 'R': case 'r':
      BT_Status();
      break;

    case 'H': case 'h':
    default:
      BT_Tag("CMDS: X<mm> Y<mm> A<mm> B<mm>  S=stop Z=zero D=dimensions R=status H=help");
      break;
  }
}

/* USART1中断只入队并重挂接收；主循环解析，避免ACK短等待丢掉停止命令。 */
void BT_UartRxCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance != huart1.Instance) return;
  if (s_rx_len < BT_RX_CAPACITY)
  {
    s_rx[s_rx_tail] = s_rx_byte;
    s_rx_tail = (s_rx_tail + 1u) % BT_RX_CAPACITY;
    ++s_rx_len;
  }
  else s_rx_lost = 1u;
  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
}

void BT_UartErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance != huart1.Instance) return;
  s_rx_lost = 1u;
  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
}

void BT_DiscardInput(void)
{
  uint32_t lock = __get_PRIMASK(); __disable_irq();
  s_rx_head = s_rx_tail = s_rx_len = 0u;
  s_rx_lost = s_lineLen = s_line_drop = 0u;
  __set_PRIMASK(lock);
}

/* 丢字节/超长行不能拼成另一条运动命令；清队列并取消活动任务。 */
static void bt_rx(void)
{
  uint8_t ch;
  uint32_t lock;
  for (;;)
  {
    lock = __get_PRIMASK(); __disable_irq();
    if (s_rx_lost)
    {
      s_rx_head = s_rx_tail = s_rx_len = 0u;
      s_rx_lost = 0u; s_lineLen = 0u; s_line_drop = 1u;
      __set_PRIMASK(lock);
      Motion_Stop(); HMI_DiscardInput();
      BT_Tag("BT RX LOST: stopped; send a new line");
      return;
    }
    if (s_rx_len == 0u) { __set_PRIMASK(lock); return; }
    ch = s_rx[s_rx_head]; s_rx_head = (s_rx_head + 1u) % BT_RX_CAPACITY;
    --s_rx_len;
    __set_PRIMASK(lock);

#if BT_ECHO

    if (((ch >= 32u) && (ch < 127u)) || (ch == (uint8_t)'\r') || (ch == (uint8_t)'\n'))
    {
      (void)HAL_UART_Transmit(&huart1, &ch, 1u, 20u);
    }
#endif

    if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n'))
    {
      if (!s_line_drop && s_lineLen > 0u)
      {
        s_line[s_lineLen] = '\0';
        bt_exec();
        s_lineLen = 0u;
      }
      s_lineLen = s_line_drop = 0u;
    }
    else if (!s_line_drop && s_lineLen < (uint8_t)(sizeof(s_line) - 1u))
    {
      s_line[s_lineLen++] = (char)ch;
    }
    else
    {
      s_lineLen = 0u; s_line_drop = 1u;
    }
  }
}

/* USART1固定115200，等待蓝牙上电稳定，不再扫描波特率。 */
void BT_Init(void)
{
  huart1.Init.BaudRate = BT_BAUD;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(600);
  s_rx_head = s_rx_tail = s_rx_len = 0u;
  s_rx_lost = s_lineLen = s_line_drop = 0u;
  HAL_NVIC_SetPriority(USART1_IRQn, 5u, 0u);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
}

/* 接收命令并每500ms输出状态；阻塞自检期间不会持续执行。 */
void BT_Poll(void)
{
  static uint32_t statusTick = 0u;

  bt_rx();

  if ((HAL_GetTick() - statusTick) >= BT_STATUS_PERIOD_MS)
  {
    statusTick = HAL_GetTick();
    BT_Status();
  }
}
