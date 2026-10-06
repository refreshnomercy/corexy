/* Actual production planner/core/task with signed absolute CAN targets and motor ramp simulation. */
static void line_at(float x,float y)
{
  motion_reset();
  s_corexy.a_pulse=(int32_t)(-(x+y)*PULSE_PER_MM);
  s_corexy.b_pulse=(int32_t)(-(x-y)*PULSE_PER_MM);
  corexy_get_pos(&s_corexy.x,&s_corexy.y);
  physical[0]=origin[0]-(x+y)*ENC_COUNTS_PER_MM;
  physical[1]=origin[1]-(x-y)*ENC_COUNTS_PER_MM;
  position_packet(1,(int32_t)(physical[0]-origin[0]));
  position_packet(2,(int32_t)(physical[1]-origin[1]));
}
__declspec(dllexport) int run_line_case(int which)
{
  float dx[8]={40,40,-40,-40,15,15,-15,-15};
  float dy[8]={15,-15,15,-15,40,-40,40,-40};
  float x,y,expected;
  uint32_t count0,t0;
  int i;
  line_at(80,70);
  if(which<8) {
    x=80+dx[which];y=70+dy[which];
    expected=host_sqrtf(dx[which]*dx[which]+dy[which]*dy[which]);
    if(!Motion_SubmitXY(x,y,20,"LINE TEST"))return 0;
    motion_run(8000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(x,y) &&
      host_path_error<0.5f && host_path_length<expected+0.4f &&
      console_contains("LEG=LINE") && !console_contains("LEG=X") &&
      !console_contains("LEG=Y") && console_contains("POS DONE");
  }
  if(which>=8 && which<=11) {
    dx[0]=which<10 ? 20 : -20;dy[0]=(which==8 || which==10) ? 20 : -20;
    x=80+dx[0];y=70+dy[0];
    /* Unused motor needs no FD ACK, but must keep reporting positions. */
    ack_mask=dx[0]==dy[0] ? 1 : 2;
    Motion_SubmitXY(x,y,20,"LINE TEST");motion_run(8000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(x,y) &&
      s_line_mask==ack_mask && host_path_error<0.1f;
  }
  if(which==12 || which==13) {
    line_at(0,0);
    x=140;y=which==12 ? 139.99f : 140.01f;
    Motion_SubmitXY(x,y,20,"LINE TEST");motion_run(18000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(x,y) && host_path_error<0.2f;
  }
  if(which==14) {
    line_at(0,0);Motion_SubmitXY(0.001f,0.001f,20,"LINE TEST");motion_run(2000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(0.001f,0.001f);
  }
  if(which==15) {
    line_at(0,0);Motion_SubmitXY(2,1,20,"LINE TEST");motion_run(4000);
    return !Motion_IsBusy() && corexy_is_calibrated() && actual_near(2,1) && host_path_error<0.2f;
  }
  if(which==16) {
    Motion_SubmitXY(150,110,20,"LINE TEST");Motion_Poll();
    while(!starts && tick<2000)motion_tick();
    for(i=0;i<50;++i)motion_tick();
    count0=starts;ack_mask=1;motion_run(1000);
    return motion_fault(5) && starts==count0 && !console_contains("POS DONE");
  }
  if(which==17) {
    Motion_SubmitXY(150,110,20,"LINE TEST");Motion_Poll();
    while(!starts && tick<2000)motion_tick();
    Motion_SubmitXY(20,20,20,"LINE TEST");
    /* A 3mm transverse physical displacement must fault instead of an unequal-RPM check. */
    physical[1]-=3*ENC_COUNTS_PER_MM;motion_run(2000);
    return motion_fault(4) && host_path_error>1.0f &&
      console_contains("POS LINE CHECK CROSS=") && !console_contains("POS DONE");
  }
  if(which==18) {
    line_at(0,0);Motion_SubmitXY(40,40,20,"LINE TEST");Motion_Poll();
    while(!starts && tick<2000)motion_tick();
    feedback_mask=1;motion_run(1000);
    return motion_fault(3) && !console_contains("POS DONE");
  }
  if(which==19) {
    Motion_SubmitXY(150,110,20,"LINE TEST");Motion_Poll();
    while(!starts && tick<2000)motion_tick();
    /* Already received S must win even when the next streaming update is due. */
    HAL_Delay(30);count0=starts;bluetooth_receive("S\r");App_Poll();
    return starts==count0 && !Motion_IsBusy() && !running[0] && !running[1] && !corexy_is_calibrated();
  }
  if(which==20) {
    /* CPU stalls past the planner deadline: do not send a late catch-up target. */
    Motion_SubmitXY(20,40,20,"LINE TEST");Motion_Poll();
    while(!starts && tick<2000)motion_tick();
    count0=starts;automatic=0;tick+=s_run.deadline+1;
    position_packet(1,s_corexy.a_read+1638);position_packet(2,s_corexy.b_read+546);run_poll();Motion_Poll();
    return motion_fault(8) && starts==count0;
  }
  if(which==21) {
    t0=tick;Motion_SubmitXY(150,110,20,"LINE TEST");Motion_Poll();
    while(!starts && tick-t0<1000)motion_tick();
    /* Every hardware target is finite, collinear and no farther than the final point. */
    for(i=0;i<120 && Motion_IsBusy();++i) {
      motion_tick();
      if(pending_pulses[0]>(uint32_t)abs_i32(s_line_plan.target[0]) ||
         pending_pulses[1]>(uint32_t)abs_i32(s_line_plan.target[1]))return 0;
      if(pending_acc[0]!=255 || pending_acc[1]!=255 || !pending_absolute[0] || !pending_absolute[1])return 0;
    }
    return Motion_IsBusy() && host_path_error<0.5f;
  }
  return 0;
}
