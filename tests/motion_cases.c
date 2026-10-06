/* Included by boundary_harness.c: production core/HMI/task code with a physical motor model. */
static void motion_reset(void)
{
  reset(); automatic=ramp_motion=1; s_cal_state=COREXY_CAL_DONE;
  s_x_max=161; s_y_max=144;
  /* Calibrated (0,0) lies 5mm from both physical walls. */
  physical[0]=origin[0]=-10*ENC_COUNTS_PER_MM;
  physical[1]=origin[1]=0;
  s_pos_origin[0]=s_pos_origin[1]=0;
  s_cont_a_v=s_cont_b_v=1;
  position_packet(1,0); position_packet(2,0);
}
static void motion_tick(void)
{ corexy_poll_status(); Motion_Poll(); HAL_Delay(10); }
static void motion_run(uint32_t duration)
{
  uint32_t t0=tick;
  while(tick-t0<duration && Motion_IsBusy()) motion_tick();
}
static int actual_near(float x,float y)
{
  float ax,ay; corexy_get_actual_pos(&ax,&ay);
  return host_fabsf(ax-x)<=0.5f && host_fabsf(ay-y)<=0.5f;
}
static int motion_fault(uint8_t reason)
{
  return corexy_get_move_state()==COREXY_MOVE_FAULT && s_bound_reason==reason &&
    !Motion_IsBusy() && !s_has_pending && !corexy_is_calibrated() &&
    !enabled[0] && !enabled[1];
}
static int launch_target(float x,float y)
{
  uint32_t t0=tick;
  if(!Motion_SubmitXY(x,y,20,"TEST")) return 0;
  while(tick-t0<700 && !starts && Motion_IsBusy()) motion_tick();
  return starts==1 && corexy_move_busy();
}

__declspec(dllexport) int run_motion_case(int which)
{
  int i;
  uint32_t t0,starts0;
  float x,y;
  motion_reset();
  if(which==0)
  {
    if(!launch_target(139.31f,142.48f)) return 0;
    motion_run(25000);
    return !Motion_IsBusy() && corexy_is_calibrated() && starts>1 &&
      host_path_error<0.5f && host_path_length<200.5f && actual_near(139.31f,142.48f);
  }
  if(which==1)
  {
    if(!launch_target(100,40)) return 0;
    corexy_get_pos(&x,&y);
    return x>99 && y>39 && !actual_near(100,40) && Motion_IsBusy() &&
      corexy_get_move_state()==COREXY_MOVE_MOVING;
  }
  if(which==2)
  {
    if(!launch_target(40,20)) return 0;
    Motion_SubmitXY(70,30,20,"TEST"); Motion_SubmitXY(10,60,20,"TEST");
    motion_run(20000);
    return !Motion_IsBusy() && !s_has_pending && s_sequence==3 &&
      host_path_error<0.5f && actual_near(10,60);
  }
  if(which==3)
  {
    if(!launch_target(100,80)) return 0;
    Motion_SubmitXY(20,20,20,"TEST");
    for(i=0;i<50;++i) motion_tick();
    starts0=starts;Motion_Stop(); HMI_DiscardInput();
    for(i=0;i<100;++i) motion_tick();
    return starts==starts0 && !running[0] && !running[1] && !s_has_pending &&
      !Motion_IsBusy() && !corexy_is_calibrated() &&
      corexy_get_move_state()==COREXY_MOVE_STOPPED &&
      !Motion_SubmitXY(30,30,20,"TEST");
  }
  if(which==4)
  {
    if(!launch_target(100,80)) return 0;
    Motion_SubmitXY(20,20,20,"TEST"); feedback_mask=1;
    t0=tick; motion_run(1000);
    return motion_fault(3) && tick-t0<400 && s_cpha_cnt_b>1;
  }
  if(which==5)
  {
    if(!launch_target(2,2)) return 0;
    amps[0]=201; motion_run(2000);
    return motion_fault(1);
  }
  if(which==6 || which==7)
  {
    if(which==6) ack_mask=1;
    else enabled[1]=0;
    Motion_SubmitXY(40,20,20,"TEST"); motion_run(2000);
    return motion_fault(5) && starts==0;
  }
  if(which==8)
  {
    if(!launch_target(100,80)) return 0;
    host_wrong_mask=3; motion_run(2000);
    return motion_fault(4);
  }
  if(which==9)
  {
    if(!launch_target(100,80)) return 0;
    skew_motor=2; Motion_SubmitXY(20,20,20,"TEST"); motion_run(3000);
    return motion_fault(4);
  }
  if(which==10)
  {
    skew_motor=3;
    Motion_SubmitXY(5,20,20,"TEST"); motion_run(3000);
    return motion_fault(2);
  }
  if(which==11)
  {
    Motion_SubmitXY(0,0,20,"TEST"); Motion_Poll();
    if(!Motion_IsBusy()) return 0;
    motion_run(2000);
    return !Motion_IsBusy() && starts==0 && corexy_is_calibrated() && actual_near(0,0);
  }
  if(which==12)
  {
    if(!launch_target(10,0)) return 0;
    automatic=0; t0=tick;
    for(i=0;i<16;++i)
    {
      tick=t0+i*20;
      position_packet(1,(int32_t)(-(9.6f+i*0.02f)*ENC_COUNTS_PER_MM));
      position_packet(2,(int32_t)(-(9.6f+i*0.02f)*ENC_COUNTS_PER_MM));
      run_poll();
      if(!corexy_move_busy()) return 0;
    }
    for(i=0;i<45;++i)
    {
      tick=t0+320+i*20;
      position_packet(1,-16384); position_packet(2,-16384); run_poll();
    }
    return corexy_get_move_state()==COREXY_MOVE_DONE;
  }
  if(which==13)
  {
    if(!launch_target(20,40)) return 0;
    stop_ack_mask=1; motion_run(5000);
    return motion_fault(6);
  }
  if(which==14)
  {
    feedback_mask=0; Motion_SubmitXY(20,20,20,"TEST"); motion_run(2000);
    return motion_fault(3) && starts==0;
  }
  if(which==15)
  {
    physical[0]-=3*ENC_COUNTS_PER_MM;
    physical[1]-=3*ENC_COUNTS_PER_MM;
    Motion_SubmitXY(20,20,20,"TEST"); motion_run(2000);
    return motion_fault(4) && starts==0;
  }
  if(which==16)
  {
    if(!launch_target(100,80)) return 0;
    Motion_SubmitXY(20,20,20,"TEST"); corexy_clear_area(); motion_run(1000);
    return motion_fault(4) && starts==1;
  }
  if(which==17)
  {
    if(!launch_target(20,10)) return 0;
    amps[0]=amps[1]=200; current_mask=1; motion_run(10000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(20,10);
  }
  if(which==18)
  {
    if(!launch_target(20,10)) return 0;
    corexy_move_rel(200,0,30); corexy_get_pos(&x,&y);
    return starts==1 && host_fabsf(x-20)<0.01f && host_fabsf(y-10)<0.01f && corexy_move_busy();
  }
  if(which==19)
  {
    if(!Motion_JogMotor(1,2560,100)) return 0;
    if(corexy_is_calibrated() || Motion_SubmitXY(20,20,20,"TEST")) return 0;
    motion_run(4000);
    return !Motion_IsBusy() && starts==1 && actual_near(-1,-1) && !corexy_is_calibrated();
  }
  if(which==20)
  {
    if(!launch_target(20,20)) return 0;
    feedback_mask=0; t0=tick;
    /* Read counters must advance: a cached target-looking position cannot finish. */
    s_corexy.a_read=s_run.target[0]; s_corexy.b_read=s_run.target[1];
    while(tick-t0<200) { run_poll(); HAL_Delay(10); }
    return corexy_move_busy() && !s_run.stable;
  }
  if(which==21)
  {
    if(!launch_target(100,80)) return 0;
    corexy_fault_stop(7); Motion_Poll();
    if(!motion_fault(7)) return 0;
    corexy_rearm_drivers();
    return corexy_home_and_calibrate()==COREXY_CAL_DONE && corexy_get_move_state()==COREXY_MOVE_IDLE;
  }
  if(which==22)
  {
    tick=0xffffff00u;
    Motion_SubmitXY(5,6,20,"TEST"); motion_run(8000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(5,6);
  }
  if(which==23)
  {
    uint8_t frame[9]={0x55,0x54,0xAA,0x01,0x16,0,0xFF,0xFF,0xFF};
    for(i=0;i<9;++i) write1ByteToRingBuffer(frame[i]);
    HMI_Poll(); motion_run(25000);
    return !Motion_IsBusy() && host_path_error<0.5f && actual_near(139.373f,143.312f);
  }
  if(which==24)
  {
    corexy_clear_area();
    write1ByteToRingBuffer(0x55); write1ByteToRingBuffer(0x54);
    write1ByteToRingBuffer(20); write1ByteToRingBuffer(0);
    write1ByteToRingBuffer(20); write1ByteToRingBuffer(0);
    for(i=0;i<3;++i) write1ByteToRingBuffer(0xFF);
    HMI_Poll(); motion_run(2000);
    return !Motion_IsBusy() && !starts;
  }
  if(which==25)
  {
    corexy_set_zero();
    position_packet(1,1234); position_packet(2,-4321);
    return corexy_get_a_read()==1234 && corexy_get_b_read()==-4321;
  }
  if(which==26)
  {
    if(!launch_target(100,80)) return 0;
    Motion_SubmitXY(20,20,20,"TEST");
    corexy_fault_stop(7); Motion_Poll();
    for(i=0;i<100;++i) motion_tick();
    return motion_fault(7) && starts==1 && !Motion_SubmitXY(30,30,20,"TEST");
  }
  if(which==27)
  {
    /* 56.62mm is 72473.6 pulses: old float/model mismatch rejects the next X leg. */
    if(!Motion_SubmitXY(123.35f,56.62f,20,"TEST"))return 0;
    motion_run(20000);
    if(Motion_IsBusy() || !corexy_is_calibrated() || !actual_near(123.35f,56.62f))return 0;
    if(!Motion_SubmitXY(100,40,20,"TEST"))return 0;
    motion_run(20000);
    return !Motion_IsBusy() && corexy_is_calibrated() && s_sequence==2 && actual_near(100,40);
  }
  if(which==28)
  {
    /* A fractional X pulse must not turn the first Y leg into an apparent diagonal. */
    if(!Motion_SubmitXY(123.351f,56.62f,20,"TEST"))return 0;
    motion_run(20000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(123.351f,56.62f);
  }
  return 0;
}

static void bluetooth_receive(const char *text)
{
  while(*text)
  {
    *host_uart_rx[0]=(uint8_t)*text++;
    HAL_UART_RxCpltCallback(&huart1);
  }
}
static int console_contains(const char *text)
{
  uint32_t i,j;
  for(i=0;i<host_console_len;++i)
  {
    for(j=0;text[j] && i+j<host_console_len && host_console[i+j]==text[j];++j) { }
    if(!text[j]) return 1;
  }
  return 0;
}
static void app_run(uint32_t duration)
{
  uint32_t t0=tick;
  while(tick-t0<duration && Motion_IsBusy()) { App_Poll(); HAL_Delay(10); }
}
__declspec(dllexport) int run_app_case(int which)
{
  int i;
  uint32_t starts0;
  motion_reset();
  if(which==0)
  {
    BT_Init();
    return huart1.Init.BaudRate==115200 && host_uart_irq[0] && host_uart_rx[0]==&s_rx_byte;
  }
  if(which==1)
  {
    bluetooth_receive("X20\r"); BT_Poll();
    if(!Motion_IsBusy() || starts || console_contains("POS DONE")) return 0;
    app_run(6000);
    return !Motion_IsBusy() && actual_near(20,0) && console_contains("POS DONE");
  }
  if(which==2)
  {
    if(!launch_target(100,80)) return 0;
    Motion_SubmitXY(20,20,20,"TEST");
    starts0=starts;
    bluetooth_receive("S\rX10\r"); App_Poll();
    for(i=0;i<50;++i) { App_Poll(); HAL_Delay(10); }
    return starts==starts0 && !Motion_IsBusy() && !s_has_pending &&
      !running[0] && !running[1] && !corexy_is_calibrated() && console_contains("STOP: pending cleared");
  }
  if(which==3)
  {
    if(!launch_target(20,40)) return 0;
    /* Stop received at final arrival must clear pending before it can start. */
    Motion_SubmitXY(80,70,20,"TEST");
    while(corexy_get_move_state()!=COREXY_MOVE_DONE && tick<10000)
    { corexy_poll_status(); HAL_Delay(10); }
    starts0=starts;bluetooth_receive("S\r"); App_Poll();
    return !Motion_IsBusy() && !s_has_pending && starts==starts0 && actual_near(20,40);
  }
  if(which==4)
  {
    for(i=0;i<30;++i) bluetooth_receive("A");
    bluetooth_receive("X10\r"); BT_Poll(); Motion_Poll();
    return !Motion_IsBusy() && !starts;
  }
  if(which==5)
  {
    if(!launch_target(100,80)) return 0;
    for(i=0;i<150;++i) bluetooth_receive("X");
    BT_Poll();
    return !Motion_IsBusy() && !corexy_is_calibrated() &&
      !running[0] && !running[1] && console_contains("BT RX LOST");
  }
  if(which==6)
  {
    if(!launch_target(100,80)) return 0;
    HAL_UART_ErrorCallback(&huart1); App_Poll();
    return !Motion_IsBusy() && !running[0] && !running[1] && !corexy_is_calibrated();
  }
  if(which==7)
  {
    host_tx_errors=10; App_CheckCan();
    corexy_set_area(161,144);
    if(!launch_target(20,10)) return 0;
    app_run(6000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(20,10);
  }
  if(which==8)
  {
    if(!launch_target(100,80)) return 0;
    Motion_SubmitXY(20,20,20,"TEST"); host_tx_errors=1;
    App_CheckCan(); Motion_Poll();
    return motion_fault(7) && starts==1;
  }
  if(which==9)
  {
    if(!launch_target(100,80)) return 0;
    host_bus_off=1; App_CheckCan(); Motion_Poll();
    return !host_bus_off && motion_fault(7);
  }
  if(which==10)
  {
    host_tx_errors=1;
    if(!App_RecoverIfNeeded()) return 0;
    corexy_set_area(161,144);
    return !App_RecoverIfNeeded() && corexy_is_calibrated();
  }
  if(which==11)
  {
    if(!launch_target(100,80)) return 0;
    App_HandleKey(KEY1_PRESS);
    return corexy_move_busy() && starts==1 && corexy_is_calibrated() && console_contains("CAL REJECT");
  }
  if(which==12)
  {
    App_HandleKey(KEY2_PRESS);
    if(starts || !Motion_IsBusy()) return 0;
    app_run(6000);
    return !Motion_IsBusy() && actual_near(0,10) && console_contains("POS DONE");
  }
  if(which==13)
  {
    if(!launch_target(20,10)) return 0;
    bluetooth_receive("R\rD\rH\r"); BT_Poll();
    return corexy_is_calibrated() && corexy_move_busy() && starts==1;
  }
  if(which==14)
  {
    bluetooth_receive("X999999999999999\rX1.5\rX20junk\r"); BT_Poll();
    return !Motion_IsBusy() && !starts && console_contains("CMD REJECT");
  }
  if(which==15)
  {
    if(!launch_target(100,80)) return 0;
    Motion_SubmitXY(20,20,20,"TEST");
    starts0=starts;bluetooth_receive("Z\r"); App_Poll();
    return !Motion_IsBusy() && !s_has_pending && !corexy_is_calibrated() && starts==starts0;
  }
  if(which==16)
  {
    bluetooth_receive("X20\r"); BT_Poll();
    app_run(100);
    return Motion_IsBusy() && !console_contains("POS DONE") && !console_contains("MOVE BT/X");
  }
  if(which==17)
  {
    if(!launch_target(20,40)) return 0;
    host_stop_inject=1; stop_ack_mask=0;
    app_run(5000);
    return !Motion_IsBusy() && actual_near(20,40) &&
      console_contains("STOP: pending cleared");
  }
  if(which==18)
  {
    uint8_t frame[9]={0x55,0x54,0xFF,0,0xE6,0,0xFF,0xFF,0xFF};
    for(i=0;i<9;++i) { *host_uart_rx[1]=frame[i]; HAL_UART_RxCpltCallback(&huart2); }
    App_Poll(); app_run(15000);
    return !Motion_IsBusy() && actual_near(80.672f,71.828f) && host_path_error<0.5f &&
      console_contains("HMI TOUCH PX=255 PY=230") && console_contains("POS DONE");
  }
  if(which==19)
  {
    if(!launch_target(20,40)) return 0;
    stop_ack_mask=1; bluetooth_receive("S\r"); App_Poll();
    return motion_fault(6);
  }
  if(which==20)
  {
    if(!launch_target(20,40)) return 0;
    stop_ack_mask=1; bluetooth_receive("Z\r"); App_Poll();
    return motion_fault(6) && s_corexy.a_pulse!=0 && console_contains("ZERO REJECT");
  }
  if(which==21 || which==22)
  {
    /* Separate screen presses: fractional calibration and corner targets included. */
    uint16_t px[12]={255,160,440,110,489,20,489,20,321,148,487,21};
    uint16_t py[12]={230,170,400,270,20,439,439,20,92,348,21,438};
    uint8_t frame[9]={0x55,0x54,0,0,0,0,0xFF,0xFF,0xFF};
    float xm=which==21 ? 161.0f : 161.1234f;
    float ym=which==21 ? 144.0f : 143.5678f;
    int press, count=which==21 ? 4 : 12;
    corexy_set_area(xm,ym);
    for(press=0;press<count;++press)
    {
      frame[2]=(uint8_t)px[press]; frame[3]=(uint8_t)(px[press]>>8);
      frame[4]=(uint8_t)py[press]; frame[5]=(uint8_t)(py[press]>>8);
      for(i=0;i<9;++i) { *host_uart_rx[1]=frame[i]; HAL_UART_RxCpltCallback(&huart2); }
      App_Poll(); app_run(20000);
      if(Motion_IsBusy() || !corexy_is_calibrated() ||
         !actual_near((float)(px[press]-20)*xm/469,(float)(439-py[press])*ym/419)) return 0;
      for(i=0;i<30;++i) { App_Poll(); HAL_Delay(10); }
    }
    return host_path_error<0.5f && s_sequence==(uint32_t)count &&
      console_contains(which==21 ? "POS DONE ID=4" : "POS DONE ID=12");
  }
  return 0;
}
