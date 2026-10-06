/* Actual production C, with only hardware/console interfaces replaced.
   No Windows/CRT dependency: compile as a DLL and invoke from Python ctypes. */
#include <math.h>
#include "../hardware/corexy.h"
#define sqrtf host_sqrtf
#define fabsf host_fabsf
int _fltused = 0;
static float host_fabsf(float v) { return v < 0 ? -v : v; }
static float host_sqrtf(float v)
{
  float x = v > 1.0f ? v : 1.0f;
  int i;
  for (i = 0; i < 24; ++i) x = (x + v / x) * 0.5f;
  return x;
}
#include "../hardware/line_plan.c"
#include "../hardware/corexy.c"
#include "../hardware/motion.c"
#include "../hardware/tjc_usart_hmi.c"
#include "../hardware/hmi.c"
#include "../hardware/bt.c"
#include "../hardware/app.c"
UART_HandleTypeDef huart2={USART2};
UART_HandleTypeDef huart1={USART1};
static uint8_t *host_uart_rx[2];
static uint32_t host_uart_rearms[2];
static uint8_t host_uart_irq[2];
static char host_console[65536];
static uint32_t host_console_len, host_tx_errors;
static uint8_t host_bus_off;
static uint8_t host_stop_inject;
static uint32_t host_stop_inject_tick;
static void bluetooth_receive(const char *text);
static KEY_Event_t host_key;
static uint8_t host_screen_tx[192];
static uint16_t host_screen_tx_len;
static uint32_t host_screen_tx_count;
void HAL_NVIC_SetPriority(int irq,int priority,int sub) { (void)irq; (void)priority; (void)sub; }
void HAL_NVIC_EnableIRQ(int irq) { if(irq==37)host_uart_irq[0]=1; if(irq==38)host_uart_irq[1]=1; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart,uint8_t *data,uint16_t len)
{
  uint8_t i=uart->Instance==USART1?0:1; (void)len;
  host_uart_rx[i]=data; ++host_uart_rearms[i]; return 0;
}
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,uint8_t *data,uint16_t len)
{
  uint16_t i;
  if(uart->Instance==USART2)
  {
    for(i=0;i<len && i<sizeof(host_screen_tx);++i)host_screen_tx[i]=data[i];
    host_screen_tx_len=len; ++host_screen_tx_count;
    HAL_UART_TxCpltCallback(uart);
  }
  return HAL_OK;
}
int HAL_UART_Init(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
int HAL_UART_Transmit(UART_HandleTypeDef *uart,uint8_t *data,uint16_t len,uint32_t timeout)
{
  uint16_t i; (void)uart; (void)timeout;
  for(i=0;i<len && host_console_len<sizeof(host_console)-1;++i)
    host_console[host_console_len++]=(char)data[i];
  host_console[host_console_len]=0; return HAL_OK;
}
void Error_Handler(void) { }
void KEY_Init(void) { }
KEY_Event_t KEY_Scan(void) { KEY_Event_t event=host_key; host_key=KEY_NONE; return event; }
void LED_Init(void) { }
void LED_On(void) { }
void LED_Off(void) { }
void Buzzer_Init(void) { }
void CAN_Health_Init(void) { host_tx_errors=0; }
uint32_t CAN_TxErrorCount(void) { return host_tx_errors; }
uint8_t CAN_IsBusOff(void) { return host_bus_off; }
uint8_t CAN_HealthRecover(void)
{ if(!host_bus_off)return 0; host_bus_off=0; host_tx_errors=0; return 1; }

static uint32_t tick, synchronizations;
static uint8_t automatic, feedback_mask, current_mask, retreat_blocked, skew_motor;
static uint8_t ack_mask;
static uint8_t stop_ack_mask;
static uint32_t disabled_position_rejections;
static uint8_t running[2], enabled[2], direction[2], pending[2];
static uint8_t pending_dir[2],pending_acc[2],pending_absolute[2];
static uint16_t pending_rpm[2];
static uint32_t pending_pulses[2];
static float pending_target[2];
static uint32_t pulses[2];
static uint16_t rpm[2];
static uint8_t acceleration[2], sync_flag[2];
static float physical[2], origin[2], remaining[2];
static uint16_t amps[2];
static uint32_t starts, unequal_commands;
static uint8_t wall_current, trace[16][2];
static uint8_t ramp_motion, delayed_retreat_feedback;
static uint8_t host_wrong_mask;
static float host_path_error,host_path_length;
static uint8_t retreat_feedback_mask, wrong_retreat_direction;
static uint8_t wall_encoder_preload, no_stopped_feedback;
static float motor_rpm[2];
static uint32_t retreat_start, contact_tick, contact_to_stop;

static void zero_bytes(void *ptr, unsigned int n)
{
  unsigned char *p = ptr;
  while (n--) *p++ = 0;
}
static void position_packet(uint8_t addr, int32_t value)
{
  uint32_t mag = (uint32_t)(value < 0 ? -value : value);
  uint8_t d[7] = {0x36, 0, 0, 0, 0, 0, 0x6B};
  d[1] = value < 0;
  d[2] = (uint8_t)(mag >> 24); d[3] = (uint8_t)(mag >> 16);
  d[4] = (uint8_t)(mag >> 8); d[5] = (uint8_t)mag;
  corexy_rx_parse(addr, d, 7);
}
static void current_packet(uint8_t addr, uint16_t value)
{
  uint8_t d[4] = {0x27, 0, 0, 0x6B};
  d[1] = (uint8_t)(value >> 8); d[2] = (uint8_t)value;
  corexy_rx_parse(addr, d, 4);
}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms)
{
  while (ms--)
  {
    float step[2], na, nb, x, y;
    uint8_t i;
    ++tick;
    if(host_stop_inject_tick && tick==host_stop_inject_tick)
    {
      uint8_t reply[3]={0xFE,0x02,0x6B};
      bluetooth_receive("S\r");
      corexy_rx_parse(1,reply,3); corexy_rx_parse(2,reply,3);
      host_stop_inject_tick=0;
    }
    if (!automatic) continue;
    for (i = 0; i < 2; ++i)
    {
      float speed = (float)rpm[i];
      if (ramp_motion && running[i] && enabled[i])
      {
        /* Vendor ramp: one RPM every (256-acc)*50us, including deceleration. */
        float rate = 20.0f / (256.0f - (float)acceleration[i]);
        float enc_per_rpm_ms = PULLEY_CIRC_MM / 60000.0f * ENC_COUNTS_PER_MM;
        float braking = motor_rpm[i] * motor_rpm[i] * enc_per_rpm_ms / (2.0f * rate);
        motor_rpm[i] += remaining[i] <= braking ? -rate : rate;
        if (motor_rpm[i] < rate) motor_rpm[i] = rate;
        if (motor_rpm[i] > speed) motor_rpm[i] = speed;
        speed = motor_rpm[i];
      }
      step[i] = (speed * PULLEY_CIRC_MM / 60.0f / 1000.0f) * ENC_COUNTS_PER_MM;
      if (step[i] > remaining[i]) step[i] = remaining[i];
      if (!running[i] || !enabled[i] || skew_motor==3 || (skew_motor == i + 1) ||
          (retreat_blocked && pulses[i] == 6400u)) step[i] = 0;
      if (direction[i]) step[i] = -step[i];
      if(host_wrong_mask&(1u<<i))step[i]=-step[i];
    }
    na = physical[0] + step[0]; nb = physical[1] + step[1];
    /* Physical mapping confirmed by the user: 0/0 is -X, 0/1 is -Y.
       Keep independent of production sign macros to catch mapping regressions. */
    x = -(na + nb) * 0.5f / ENC_COUNTS_PER_MM;
    y = -(na - nb) * 0.5f / ENC_COUNTS_PER_MM;
    /* Full 170 x 150 mm rectangle. Both motors stop at wall contact. */
    if ((x >= 0 && x <= 170) && (y >= 0 && y <= 150))
    {
      if(s_run.line_mode && s_run.state==COREXY_MOVE_MOVING) {
        int32_t actual[2];float cross;
        host_path_length+=host_sqrtf((step[0]*step[0]+step[1]*step[1])*0.5f)/ENC_COUNTS_PER_MM;
        actual[0]=(int32_t)(na-origin[0]);actual[1]=(int32_t)(nb-origin[1]);
        cross=LinePlan_PathError(&s_line_plan,actual);
        if(cross>host_path_error)host_path_error=cross;
      }
      physical[0] = na; physical[1] = nb;
      for (i = 0; i < 2; ++i) remaining[i] -= host_fabsf(step[i]);
      if (wall_current) amps[0] = amps[1] = 120;
    }
    else if (wall_current)
    {
      amps[0] = amps[1] = 900;
      if (!contact_tick) contact_tick=tick;
    }
  }
}
void CAN_Unwedge(void) { }
void CAN_Send_Poll(void) { }
void Emm_V5_Stop_Now(uint8_t addr, bool sync)
{
  uint8_t i=addr-1;
  uint8_t reply[3]={0xFE,0x02,0x6B};
  if (contact_tick && !contact_to_stop) contact_to_stop=tick-contact_tick;
  (void)sync;
  if(!enabled[i]) reply[1]=0xE2;
  else { running[i]=0; pending[i]=0; remaining[i]=0; motor_rpm[i]=0; }
  if(stop_ack_mask & (1u<<i)) corexy_rx_parse(addr,reply,3);
  if(host_stop_inject && addr==2 && s_run.state==COREXY_MOVE_MOVING)
  { host_stop_inject_tick=tick+20; host_stop_inject=0; }
}
void Emm_V5_En_Control(uint8_t addr, bool en, bool sync) { (void)sync; enabled[addr-1] = en; }
void Emm_V5_Reset_Clog_Pro(uint8_t addr) { (void)addr; }
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool save, uint8_t mode) { (void)addr; (void)save; (void)mode; }
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr) { origin[addr-1] = physical[addr-1]; }
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool absolute, bool sync)
{
  uint8_t i = addr-1;
  (void)absolute;
  /* Manufacturer protocol: FD is rejected with E2 while motor is disabled. */
  if(!enabled[i])
  {
    uint8_t reject[3]={0xFD,0xE2,0x6B};
    ++disabled_position_rejections;
    corexy_rx_parse(addr,reject,3);
    return;
  }
  pending_dir[i]=dir;pending_rpm[i]=vel;pending_acc[i]=acc;pending_pulses[i]=clk;
  pending_absolute[i]=absolute;
  pending_target[i]=origin[i]+(dir ? -1.0f : 1.0f)*(float)clk/PULSE_PER_MM*ENC_COUNTS_PER_MM;
  sync_flag[i] = sync; pending[i] = 1;
  if (!sync) {
    direction[i]=dir;rpm[i]=vel;acceleration[i]=acc;pulses[i]=clk;
    remaining[i]=(float)clk/PULSE_PER_MM*ENC_COUNTS_PER_MM;running[i]=1;
  }
  if (ack_mask & (1u << i))
  {
    uint8_t ack[3]={0xFD,0x02,0x6B};
    corexy_rx_parse(addr,ack,3);
  }
}
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t i;
  (void)addr;
  ++synchronizations; ++starts;
  if (!s_run.line_mode && starts % 2 == 0) { retreat_start=tick; motor_rpm[0]=motor_rpm[1]=0; }
  for(i=0;i<2;++i)if(pending[i]) {
    uint8_t newdir=pending_absolute[i] ? pending_target[i]<physical[i] : pending_dir[i];
    if(direction[i]!=newdir)motor_rpm[i]=0;
    direction[i]=newdir;rpm[i]=pending_rpm[i];acceleration[i]=pending_acc[i];pulses[i]=pending_pulses[i];
    remaining[i]=pending_absolute[i] ? host_fabsf(pending_target[i]-physical[i]) :
      (float)pulses[i]/PULSE_PER_MM*ENC_COUNTS_PER_MM;
  }
  if (starts % 2 == 0 && wrong_retreat_direction)
    { direction[0]^=1; direction[1]^=1; }
  if(starts<=16) { trace[starts-1][0]=direction[0]; trace[starts-1][1]=direction[1]; }
  if (!s_run.line_mode && ((rpm[0] != rpm[1]) || (pulses[0] != pulses[1]) ||
      (acceleration[0] != acceleration[1]) || !sync_flag[0] || !sync_flag[1])) ++unequal_commands;
  if (pending[0]) running[0] = 1;
  if (pending[1]) running[1] = 1;
  pending[0] = pending[1] = 0;
}
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t param)
{
  uint8_t i = addr-1;
  if (!automatic)
  {
    if (param == S_CPOS && (feedback_mask & (1u << i)))
      position_packet(addr, s_pos_origin[i] + (i == 0 ? s_corexy.a_read : s_corexy.b_read));
    return;
  }
  if (param == S_CPOS && no_stopped_feedback && starts == 1 && !running[0] && !running[1]) return;
  /* Recovering drivers may temporarily delay position replies after a retreat starts. */
  if (param == S_CPOS && delayed_retreat_feedback && starts && starts % 2 == 0 &&
      tick-retreat_start < 750u) return;
  if (param == S_CPOS && starts && starts % 2 == 0 &&
      !(retreat_feedback_mask & (1u << i))) return;
  if (param == S_CPOS && (feedback_mask & (1u << i)))
  {
    float read = physical[i] - origin[i];
    /* Loaded rotor/belt position differs from the stopped, unloaded start position. */
    if (wall_encoder_preload && running[i] && amps[i] > 200)
      read += (direction[i] ? -1.5f : 1.5f) * ENC_COUNTS_PER_MM;
    position_packet(addr, (int32_t)read);
  }
  if (param == S_CPHA && (current_mask & (1u << i))) current_packet(addr, amps[i]);
}
void Buzzer_On(void) { }
void Buzzer_Off(void) { }

static void reset(void)
{
  zero_bytes(&s_corexy, sizeof(s_corexy)); zero_bytes(&s_bound, sizeof(s_bound));
  zero_bytes(&s_run,sizeof(s_run)); Motion_Init(); tjc_init();
  HMI_Init(); host_screen_tx_len=0; host_screen_tx_count=0;
  BT_DiscardInput(); s_len=0;
  HAL_UART_Receive_IT(&huart1,&s_rx_byte,1);
  host_console_len=0; host_console[0]=0;
  host_bus_off=host_tx_errors=0; host_key=KEY_NONE;
  host_stop_inject=0; host_stop_inject_tick=0;
  s_can_tick=s_can_error_seen=s_recovery_error_seen=0;
  s_motion_fault = 0;
  s_cont_a = s_cont_b = 0; s_cont_a_v = s_cont_b_v = 0;
  s_pos_count[0] = s_pos_count[1] = s_pos_tick[0] = s_pos_tick[1] = 0;
  s_cur_tick[0] = s_cur_tick[1] = s_cpha_cnt_a = s_cpha_cnt_b = 0;
  s_cpha_a = s_cpha_b = 0; s_bound_reason = 0;
  s_retreat_active=0;
  s_enabled = s_drv_en = 1; s_cal_state = COREXY_CAL_RUNNING;
  tick = 1000; automatic = 0; feedback_mask = current_mask = 3;
  ack_mask=3; s_pos_ack[0]=s_pos_ack[1]=0;
  stop_ack_mask=3; disabled_position_rejections=0;
  s_stop_ack[0]=s_stop_ack[1]=0;
  s_pos_reply[0]=s_pos_reply[1]=0;
  s_pos_status[0]=s_pos_status[1]=0;
  retreat_blocked = skew_motor = 0;
  running[0] = running[1] = pending[0] = pending[1] = 0;
  enabled[0] = enabled[1] = 1; physical[0] = physical[1] = origin[0] = origin[1] = 0;
  starts = unequal_commands = synchronizations = 0;
  amps[0] = amps[1] = 120; wall_current=0;
  ramp_motion=delayed_retreat_feedback=0; retreat_start=0;
  retreat_feedback_mask=3; wrong_retreat_direction=0;
  wall_encoder_preload=no_stopped_feedback=0;
  contact_tick=contact_to_stop=0;
  motor_rpm[0]=motor_rpm[1]=0;
  host_wrong_mask=0;host_path_error=host_path_length=0;
  position_packet(1, 0); position_packet(2, 0);
}
static void arm(void)
{
  s_move_ms = 20000; s_last_feed = 15;
  s_move_x0 = s_move_y0 = 0; s_move_pulse0[0] = s_move_pulse0[1] = 0;
  s_cmd_dx = 1; s_cmd_dy = 0;
  s_corexy.a_pulse = s_corexy.b_pulse = -384000;
  s_corexy.x = 300; s_corexy.y = 0;
  bound_arm(491520, 491520);
}
static void sample(uint32_t elapsed, int32_t a, int32_t b, uint16_t ca, uint16_t cb)
{
  tick = 1000 + elapsed;
  position_packet(1, a); position_packet(2, b);
  current_packet(1, ca); current_packet(2, cb);
  bound_poll();
}

__declspec(dllexport) int run_case(int which)
{
  int i;
  uint32_t trip;
  reset();
  if (which == 0) /* Low current at late contact after ~120mm travel. */
  {
    arm();
    for (i=50; i<=8000; i+=50) sample(i, -i*24, -i*24, 120, 120);
    for (i=8050; i<=8800 && s_bound_reason==0; i+=50) sample(i,-192000,-192000,120,120);
    return s_bound.hit && s_bound_reason==2 && s_bound_ms<=8800 && !unequal_commands;
  }
  if (which == 1) /* Current >200 is sufficient even while positions still report motion. */
  {
    arm();
    for (i=50; i<=3000 && !s_bound.hit; i+=50) sample(i,-i*24,-i*24,i<2500?120:260,120);
    return s_bound.hit && s_bound_reason==1 && s_bound_ms>=2500 && s_bound_ms<=2650;
  }
  if (which == 2) /* Other CAN packets cannot keep stale positions alive. */
  {
    uint8_t flags[3]={0x3A,3,0x6B};
    arm();
    for (i=50; i<=1000 && !s_bound.fault; i+=50)
    {
      tick=1000+i; corexy_rx_parse(1,flags,3); corexy_rx_parse(2,flags,3);
      current_packet(1,120); current_packet(2,120); bound_poll();
    }
    return s_bound.fault==3 && !s_bound.hit && !s_drv_en;
  }
  if (which == 3) /* One stopped motor is an abnormal trajectory. */
  {
    arm();
    for (i=50; i<=1800 && !s_bound.fault; i+=50) sample(i,-i*24,0,120,120);
    return s_bound.fault==4 && !s_bound.hit && !s_drv_en;
  }
  if (which == 4) /* Starting against wall: current trips without any cruise baseline. */
  {
    arm();
    for (i=50; i<=1600 && !s_bound.hit; i+=50) sample(i,0,0,1800,1800);
    return s_bound.hit && s_bound_reason==1 && s_bound_ms<=450;
  }
  if (which == 5) /* A single reused high-current packet cannot count three times. */
  {
    arm();
    for (i=50; i<=1500; i+=50) sample(i,i*24,i*24,120,120);
    sample(1550,36000,36000,1800,1800);
    for (i=1600; i<=1900; i+=50)
    {
      tick=1000+i; position_packet(1,36000); position_packet(2,36000); bound_poll();
    }
    return s_bound_reason!=1 && s_bound.cnt[0]<=1 && s_bound.cnt[1]<=1;
  }
  if (which == 6) /* Full signed multi-turn position, including >half turn gap. */
  {
    position_packet(1,196608); position_packet(2,-196608);
    return corexy_get_a_read()==196608 && corexy_get_b_read()==-196608;
  }
  if (which == 7) /* Tiny coordinate discrepancy cannot turn axis probe into unequal RPM. */
  {
    s_corexy.x=12.34567f; s_corexy.y=9.12345f;
    s_corexy.a_pulse=(int32_t)((-s_corexy.x-s_corexy.y)*PULSE_PER_MM);
    s_corexy.b_pulse=(int32_t)((-s_corexy.x+s_corexy.y)*PULSE_PER_MM);
    corexy_move_to(s_corexy.x+300,s_corexy.y+0.0001f,15);
    return rpm[0]==rpm[1] && pulses[0]==pulses[1] && direction[0]==1 && direction[1]==1 && !unequal_commands;
  }
  if (which == 8) /* Stop must disarm old movement. */
  {
    arm(); corexy_stop(); sample(1800,0,0,1800,1800);
    return !s_bound.armed && !s_bound.hit && !synchronizations;
  }
  if (which == 9) /* Actual four-wall sweep with no current from B. */
  {
    automatic=1; current_mask=1;
    physical[0]=(-60.0f-50.0f)*ENC_COUNTS_PER_MM;
    physical[1]=(-60.0f+50.0f)*ENC_COUNTS_PER_MM;
    corexy_poll_status(); HAL_Delay(60); corexy_poll_status();
    if (corexy_home_and_calibrate()!=COREXY_CAL_DONE) return 0;
    return !unequal_commands && starts==8 && s_x_max>160 && s_x_max<165 &&
      s_y_max>140 && s_y_max<145;
  }
  if (which == 10) /* Retreat command received but mechanically not executed. */
  {
    automatic=1; retreat_blocked=1;
    physical[0]=-110*ENC_COUNTS_PER_MM; physical[1]=-10*ENC_COUNTS_PER_MM;
    return !corexy_probe_to_boundary(1,0,300,15) && s_bound.fault==4;
  }
  if (which == 11) /* Full sweep from near a corner: preserve straight retreat. */
  {
    automatic=1; physical[0]=-315*ENC_COUNTS_PER_MM; physical[1]=-23*ENC_COUNTS_PER_MM;
    corexy_poll_status(); HAL_Delay(60); corexy_poll_status();
    return corexy_home_and_calibrate()==COREXY_CAL_DONE && !unequal_commands && starts==8;
  }
  if (which == 12) /* Reject invalid/truncated current data. */
  {
    uint8_t short_packet[2]={0x27,0xFF}, corrupt[4]={0x27,0xFF,0xFF,0};
    corexy_rx_parse(1,short_packet,2); corexy_rx_parse(2,corrupt,4);
    return s_cpha_cnt_a==0 && s_cpha_cnt_b==0;
  }
  if (which == 13) /* Real current rise plus slow motion gets its own reason. */
  {
    arm();
    for(i=50;i<=3000;i+=50) sample(i,i*24,i*24,120,120);
    for(i=3050;i<=3900 && !s_bound.hit;i+=50) sample(i,72000,72000,1500,1500);
    trip=s_bound_ms;
    return s_bound.hit && s_bound_reason==1 && trip<3900;
  }
  if (which == 14) /* No position packets: preflight must not send any motion. */
  {
    automatic=1; feedback_mask=0;
    return !corexy_probe_to_boundary(1,0,300,15) && s_bound.fault==3 && !starts;
  }
  if (which == 15) /* Temporary feedback delays within budget must not invent a wall. */
  {
    arm();
    for(i=50;i<=5000;i+=50)
    {
      tick=1000+i;
      if(i%200==0) { position_packet(1,i*24); position_packet(2,i*24); }
      current_packet(1,120); current_packet(2,120); bound_poll();
    }
    return s_bound.armed && !s_bound.hit && !s_bound.fault;
  }
  if (which == 16) /* An isolated 250ms pause should resume without a wall event. */
  {
    arm();
    for(i=50;i<=2000;i+=50) sample(i,i*24,i*24,120,120);
    for(i=2050;i<=2250;i+=50) sample(i,48000,48000,120,120);
    for(i=2300;i<=3500;i+=50) sample(i,(i-250)*24,(i-250)*24,120,120);
    return s_bound.armed && !s_bound.hit && !s_bound.fault;
  }
  if (which == 17) /* Diagonal probe must be refused before either motor is started. */
    return !corexy_probe_to_boundary(1,1,300,15) && !starts;
  if (which == 18) /* A normal short move ending must not become a boundary later. */
  {
    s_cal_state=COREXY_CAL_IDLE;
    corexy_move_rel(5,0,15); tick+=600;
    bound_poll();
    return !s_bound.armed && !s_bound.hit;
  }
  if (which == 19) /* Missing B command ACK must prevent broadcast synchronization. */
  {
    ack_mask=1;
    corexy_move_rel(300,0,15);
    return !starts && s_bound.fault==5 && !s_drv_en && !running[0] && !running[1];
  }
  if (which == 20) /* Position-reached notification is not a successful command ACK. */
  {
    uint8_t reached[3]={0xFD,0x9F,0x6B};
    corexy_rx_parse(1,reached,3); corexy_rx_parse(2,reached,3);
    return !s_pos_ack[0] && !s_pos_ack[1];
  }
  if (which == 21) /* All eight motions: +X/back, +Y/back, -X/back, -Y/back. */
  {
    static const uint8_t expected_dirs[8][2]={{1,1},{0,0},{1,0},{0,1},{0,0},{1,1},{0,1},{1,0}};
    automatic=1; wall_current=1; current_mask=1;
    physical[0]=-110*ENC_COUNTS_PER_MM; physical[1]=-10*ENC_COUNTS_PER_MM;
    corexy_poll_status(); HAL_Delay(60); corexy_poll_status();
    if(corexy_home_and_calibrate()!=COREXY_CAL_DONE || starts!=8 || unequal_commands) return 0;
    for(i=0;i<8;++i) if(trace[i][0]!=expected_dirs[i][0] || trace[i][1]!=expected_dirs[i][1]) return 0;
    return s_bound_reason==1 && host_fabsf(-(physical[0]+physical[1])*0.5f/ENC_COUNTS_PER_MM-5)<1 &&
      host_fabsf(-(physical[0]-physical[1])*0.5f/ENC_COUNTS_PER_MM-5)<1;
  }
  if(which == 22) /* 200mA exactly is not above threshold; 201mA on B alone is. */
  {
    arm();
    for(i=50;i<=2000;i+=50) sample(i,i*24,i*24,200,200);
    if(s_bound.hit) return 0;
    for(i=2050;i<=2200 && !s_bound.hit;i+=50) sample(i,i*24,i*24,120,201);
    return s_bound.hit && s_bound_reason==1 && s_bound_ms<=2200;
  }
  if(which == 23) /* Low current resets confirmation: isolated spikes cannot retreat. */
  {
    arm();
    for(i=50;i<=3000;i+=50) sample(i,i*24,i*24,i%150==0?120:300,120);
    return !s_bound.hit && !s_bound.fault;
  }
  if(which == 24) /* Reproduce old sequence: unloading then FD rejects with E2. */
  {
    drive_cut_off();
    Emm_V5_Pos_Control(1,0,37,50,6400,0,1);
    Emm_V5_Pos_Control(2,0,37,50,6400,0,1);
    return disabled_position_rejections==2 && !s_pos_ack[0] && !s_pos_ack[1] &&
      s_pos_status[0]==0xE2 && s_pos_status[1]==0xE2;
  }
  if(which == 25) /* Real protocol constraint: disabled -> prepare -> FD -> sync. */
  {
    drive_cut_off();
    corexy_move_rel(-5,0,25);
    return starts==1 && enabled[0] && enabled[1] && s_stop_ack[0] && s_stop_ack[1] &&
      !disabled_position_rejections && pulses[0]==6400 && pulses[1]==6400;
  }
  if(which == 26) /* No motor-B stop ACK: no synchronized move may start. */
  {
    drive_cut_off(); stop_ack_mask=1;
    corexy_move_rel(-5,0,25);
    return !starts && s_bound.fault==6 && !s_drv_en && !running[0] && !running[1];
  }
  if(which == 27) /* A driver-side disabled condition yields explicit E2, no broadcast. */
  {
    enabled[1]=0; /* Software thought both enabled, simulate a driver dropping enable. */
    corexy_move_rel(300,0,15);
    return !starts && s_bound.fault==5 && s_pos_status[1]==0xE2;
  }
  if(which == 28 || which == 29) /* Fresh feedback recovery and realistic acceleration. */
  {
    automatic=wall_current=1;
    delayed_retreat_feedback=(which==28);
    ramp_motion=1;
    physical[0]=-110*ENC_COUNTS_PER_MM; physical[1]=-10*ENC_COUNTS_PER_MM;
    corexy_poll_status(); HAL_Delay(60); corexy_poll_status();
    return corexy_home_and_calibrate()==COREXY_CAL_DONE && starts==8 && !unequal_commands &&
      host_fabsf(-(physical[0]+physical[1])*0.5f/ENC_COUNTS_PER_MM-5)<1 &&
      host_fabsf(-(physical[0]-physical[1])*0.5f/ENC_COUNTS_PER_MM-5)<1;
  }
  if(which >= 30 && which <= 33)
  {
    automatic=wall_current=ramp_motion=1;
    physical[0]=-110*ENC_COUNTS_PER_MM; physical[1]=-10*ENC_COUNTS_PER_MM;
    if(which==30) retreat_feedback_mask=1;
    if(which==31) retreat_feedback_mask=0;
    if(which==32) wrong_retreat_direction=1;
    if(which==33)
      return corexy_probe_to_boundary(1,0,300,15) && contact_to_stop>0 && contact_to_stop<=80;
    return !corexy_probe_to_boundary(1,0,300,15) && starts==2 &&
      s_bound.fault==(which==32?4:3) && !s_drv_en && !s_retreat_active;
  }
  if (which == 34 || which == 35)
  {
    automatic=wall_current=ramp_motion=1;
    wall_encoder_preload=1;
    no_stopped_feedback=(which==35);
    physical[0]=-110*ENC_COUNTS_PER_MM; physical[1]=-10*ENC_COUNTS_PER_MM;
    if (which==35)
      return !corexy_probe_to_boundary(1,0,300,20) && starts==1 && s_bound.fault==3 && !s_drv_en;
    return corexy_probe_to_boundary(1,0,300,20) && starts==2 && !unequal_commands &&
      host_fabsf(-(physical[0]+physical[1])*0.5f/ENC_COUNTS_PER_MM-165)<1;
  }
  return 0;
}

__declspec(dllexport) uint32_t get_simulated_contact_to_stop_ms(void) { return contact_to_stop; }
__declspec(dllexport) const char *get_host_console(void) { return host_console; }
#include "motion_cases.c"
#include "line_cases.c"
#include "marker_cases.c"
